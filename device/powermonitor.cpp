// Power Monitor Firmware
// Dual-core operation with real I2C sampling

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <cmath>
#include <cstdio>
#include <cstring>

#include "command_handler.hpp"
#include "core/raw_sample.hpp"
#include "hardware_tests.hpp"
#include "pico/multicore.h"
#include "protocol/parser.hpp"
#include "sampler.hpp"
#include "state_machine.hpp"

// I2C Configuration
#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3
#define INA228_ADDR 0x40
#define INA228_SHUNT_OHMS 0.015f

#include "INA228.hpp"
#include "hardware/clocks.h"
#include "pio_i2c.h"

// Global state
static device::DeviceContext g_ctx;
static protocol::Parser g_parser;
static device::CommandHandler *g_handler = nullptr;

// Static member definition for CommandHandler::tx_buf_ (keeps it off the stack)
uint8_t device::CommandHandler::tx_buf_[device::CommandHandler::kMaxFrameBytes];

// USB stress mode sample template (fixed values)
static constexpr uint32_t kStressVbusRaw = 0x0F000; // ~12V equivalent
static constexpr int32_t kStressVshuntRaw = 0x00100;
static constexpr int32_t kStressCurrentRaw = 0x01000; // Fixed current raw value
static constexpr int16_t kStressDietempRaw = 4480;    // ~35C equivalent
static constexpr uint8_t kStressBurstFramesPerLoop =
    1; // Max DATA_SAMPLE sent per main-loop tick

// Shared context for inter-core communication
static core::SharedContext g_shared_ctx;
static bool g_boot_text_sent = false;
static constexpr char kBootTextReport[] = "device online";

// Device-driven time sync: send EVT_TIME_SYNC_REQUEST every 2 minutes when
// streaming
static constexpr uint64_t kTimeSyncRequestPeriodUs =
    120ULL * 1000ULL * 1000ULL;                           // 2 minutes
static constexpr uint64_t kSyncWaitTimeoutUs = 200000ULL; // 200ms
static uint64_t g_last_time_sync_request_us = 0;

// T2 timing: capture time when frame is received via USB
namespace device {
uint64_t g_last_frame_recv_time_us = 0;
}

// USB CDC write function with optional flush, backoff and timeout.
// Returns true if all |len| bytes were written, false on timeout/disconnect.
template <bool Flush = true>
inline static bool usb_cdc_write(const uint8_t *data, size_t len,
                                 uint32_t timeout_us = 200000,  // 200 ms default
                                 uint32_t backoff_us = 100) {
  if (!data || len == 0 || !tud_cdc_connected())
    return false;

  const uint64_t start = time_us_64();

  size_t remaining = len;
  const uint8_t *ptr = data;

  while (remaining > 0) {
    if (!tud_cdc_connected()) {
      return false;
    }

    if (time_us_64() - start > timeout_us) {
      return false;
    }

    const uint32_t avail = tud_cdc_write_available();
    if (avail == 0) {
      tud_task();
      if (Flush) {
        tud_cdc_write_flush();
      }
      sleep_us(backoff_us);
      continue;
    }

    const uint32_t to_write = static_cast<uint32_t>(
        (remaining < static_cast<size_t>(avail)) ? remaining
                                                 : static_cast<size_t>(avail));
    const uint32_t written = tud_cdc_write(ptr, to_write);
    if (written == 0) {
      // Avoid tight spinning when backend temporarily cannot accept bytes.
      tud_task();
      if (Flush) {
        tud_cdc_write_flush();
      }
      sleep_us(backoff_us);
      continue;
    }

    ptr += written;
    remaining -= written;

    // Flushing once at end reduces USB transaction overhead in chunked writes.
    if (Flush && remaining == 0) {
      tud_cdc_write_flush();
    }
  }
  return true;
}

static bool usb_cdc_write_flush(const uint8_t *data, size_t len) {
  // Control/EVT paths: allow longer blocking timeout.
  return usb_cdc_write<true>(data, len, 500000);
}

static bool usb_cdc_write_no_flush(const uint8_t *data, size_t len) {
  // Data path: shorter timeout, no flush at end.
  return usb_cdc_write<false>(data, len, 100000);
}

// Parser callback - called when a complete frame is received.
// T2 is captured here, immediately after CRC verification and before
// handle_frame(), so it reflects precisely when this specific frame completed.
static void on_frame_received(const protocol::FixedFrame &frame, void *user_data) {
  (void)user_data;
  device::g_last_frame_recv_time_us = time_us_64();
  if (g_handler) {
    g_handler->handle_frame(frame);
  }
}

// USB stress mode: emit fixed-value DATA_SAMPLE as fast as main loop allows.
static void process_streaming_usb_stress() {
  if (!g_ctx.is_streaming() || !g_ctx.usb_stress_mode) {
    return;
  }

  for (uint8_t i = 0; i < kStressBurstFramesPerLoop; ++i) {
    core::RawSample sample{};
    sample.timestamp_us = time_us_64() - g_ctx.stream_start_us;
    sample.vbus_raw = kStressVbusRaw;
    sample.vshunt_raw = kStressVshuntRaw;
    sample.current_raw = kStressCurrentRaw;
    sample.dietemp_raw = kStressDietempRaw;
    sample.power_raw = 0;
    sample.energy_raw = 0;
    sample.charge_raw = 0;
    sample.flags = core::SampleFlags::kCnvrf | core::SampleFlags::kCalValid;
    sample._pad = 0;
    g_handler->send_data_sample_noflush(sample);
  }
}

// Pop samples from queue and send
// Core 1 produces samples, Core 0 consumes and sends
static void process_streaming() {
  if (!g_ctx.is_streaming())
    return;
  // Pop and send multiple samples per call to keep up with 1kHz rate
  // Process up to 32 samples per iteration
  for (int i = 0; i < 32; i++) {
    core::RawSample sample;
    if (!g_shared_ctx.sample_queue.pop(sample)) {
      break; // Queue empty, done
    }
    g_handler->send_data_sample_noflush(sample);
  }
}

// Streaming processor
static void process_streaming_loop() {
  if (g_ctx.usb_stress_mode) {
    process_streaming_usb_stress();
    return;
  }

  process_streaming();
}

int main() {
  // Initialize stdio (for debug output)
  stdio_init_all();

  // Wait until the host actually opens CDC (more robust, no fixed 2s delay
  // needed)
  absolute_time_t deadline = make_timeout_time_ms(3000);
  while (!tud_cdc_connected() && !time_reached(deadline)) {
    sleep_ms(10);
  }

  // Initialize I2C
  i2c_init(I2C_PORT, 1000 * 1000); // 1 MHz
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);

  // Initialize INA228
  INA228 ina228(I2C_PORT, INA228_ADDR, INA228_SHUNT_OHMS);
  // Software reset before configure() — recovers the chip from any
  // degenerate state left by a previous bad SET_CFG sequence, since
  // INA228 has its own VDD and is not power-cycled by a Pico reflash.
  (void)ina228.reset_all();
  sleep_ms(20);
  ina228.configure();
  g_ctx.ina228 = &ina228;
  g_ctx.adcrange = ina228.get_adc_range();
  // INA228 current_lsb = max_current / 2^19; convert A/LSB to nA/LSB for
  // CFG_REPORT.
  g_ctx.current_lsb_nA =
      static_cast<uint32_t>(std::llround((3.5 / 524288.0) * 1e9));
  g_ctx.cal_valid = g_ctx.current_lsb_nA > 0;

  // Initialize shared context and link to device context
  g_shared_ctx.init();
  g_ctx.shared_ctx = &g_shared_ctx;

  // Initialize sampler with shared context and INA228 pointer
  device::sampler_init(&g_shared_ctx, &ina228, INA228_ADDR);

#ifndef POWERMONITOR_TEST_MODE
  // Transition from HW I2C to PIO I2C for high-speed sampling
  i2c_deinit(I2C_PORT);
  gpio_init(I2C_SDA);
  gpio_init(I2C_SCL);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);

  PIO pio = pio0;
  uint sm = pio_claim_unused_sm(pio, true);
  uint offset = pio_add_program(pio, &i2c_program);
  i2c_program_init(pio, sm, offset, I2C_SDA, I2C_SCL);

  // Set PIO clock to 1 MHz
  float div = (float)clock_get_hz(clk_sys) / (32 * 1000000);
  pio_sm_set_clkdiv(pio, sm, div);

  // Initialize DMA channels and buffers for the sampler
  device::sampler_init_dma(pio, sm, INA228_ADDR);

  // Launch Core 1 for sampling
  printf("Starting high-speed PIO DMA sampling on Core 1...\n");
  device::sampler_launch_core1();
#endif

  // Initialize command handler
  device::CommandHandler handler(g_ctx, usb_cdc_write_flush,
                                 usb_cdc_write_no_flush);
  g_handler = &handler;

  // Initialize parser with callback
  g_parser.set_callback(on_frame_received, nullptr);

  printf("Power Monitor Dual-Core Mode\n");
  printf("Waiting for USB connection...\n");

  // Main loop (Core 0)
  while (true) {
    // TinyUSB housekeeping
    tud_task();

#ifdef POWERMONITOR_TEST_MODE
    if (tud_cdc_connected()) {
      device::run_hardware_tests(g_ctx, g_shared_ctx);
    }
    continue;
#endif
    // Send boot text: retry each iteration until successful (USB may not be
    // ready on first attempt)
    if (!g_boot_text_sent && tud_cdc_connected()) {
      const uint8_t *text = reinterpret_cast<const uint8_t *>(kBootTextReport);
      const size_t text_len = sizeof(kBootTextReport) - 1;
      // Keep retrying each loop iteration until send succeeds
      if (g_handler && g_handler->send_text_report(text, text_len)) {
        g_boot_text_sent = true;
      }
    }
    // Process incoming USB data
    if (tud_cdc_available()) {
      uint8_t buf[64];
      auto count = tud_cdc_read(buf, sizeof(buf));
      if (count > 0) {
        g_parser.feed(buf, count);
      }
    }

    if (g_ctx.sync_waiting) {
      if (time_us_64() - g_ctx.sync_wait_start_us > kSyncWaitTimeoutUs) {
        g_ctx.sync_waiting = false;
      }
      continue; // Tight loop: no streaming, no stats
    }

    if (!g_ctx.is_streaming()) { // if not streaming, there is no need to process streaming loop
        continue;
    }

    // Normal path: check 5s timer and maybe send time sync request
    const uint64_t now_us = time_us_64();
    if (g_handler &&
        (g_last_time_sync_request_us == 0 ||
         now_us - g_last_time_sync_request_us >= kTimeSyncRequestPeriodUs)) {
      if (g_handler->send_time_sync_request()) {
        g_last_time_sync_request_us = now_us;
        g_ctx.sync_waiting = true;
        g_ctx.sync_wait_start_us = now_us;
      }
    }

    if (g_handler) {
      g_handler->maybe_emit_stats_report(now_us);
    }
    tud_task();
    process_streaming_loop();
  }

  return 0;
}
