#ifndef DEVICE_SAMPLER_HPP
#define DEVICE_SAMPLER_HPP

#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <cstdio>
#include "pio_i2c.h"
#include <cstdio>
#include <pico/stdio.h>

#ifdef POWERMONITOR_DEBUG_DMA
#define DEBUG_DMA_PRINT(...)                                                   \
  do {                                                                         \
    printf(__VA_ARGS__);                                                       \
    stdio_flush();                                                             \
  } while (0)
#else
#define DEBUG_DMA_PRINT(...) ((void)0)
#endif
#include <cstdint>

#include "INA228.hpp"
#include "core/raw_sample.hpp"
#include "core/shared_context.hpp"

namespace device {

enum class SamplerMode { kPioDma, kPioNonDma };

// Sampler context for Core 1
// Holds pointers to shared resources and local state
struct SamplerContext {
  core::SharedContext *shared; // Pointer to shared context
  INA228 *ina228;              // Pointer to INA228 driver
  repeating_timer_t timer;     // Timer handle
  bool timer_active;           // Is timer currently running
  SamplerMode mode;            // Current sampler mode
  uint8_t i2c_addr;            // Target I2C address

  // ISR -> Worker communication
  volatile uint32_t isr_seq; // Incremented by ISR each tick
  uint32_t worker_seq;       // Last seq processed by worker

  // Duplicate suppression state (Core 1 local)
  bool has_last_sent;
  uint32_t last_vbus_raw;
  int32_t last_vshunt_raw;
  int32_t last_current_raw;
  uint64_t last_sent_time_us;

  // DMA channels
  int dma_tx_chan;
  int dma_rx_chan;

  // PIO details
  PIO pio;
  uint sm;

  // DMA Buffers
  // TX buffer holds instruction codes for PIO I2C plus the 8-bit data
  // payloads. It needs to be large enough for all address+register+read
  // sequences. We'll allocate 256 words (154 needed for 8 registers).
  uint16_t dma_tx_buf[256];
  uint32_t dma_tx_len;

  // RX buffer holds halfwords from PIO RX FIFO. We'll allocate 64.
  uint32_t dma_rx_buf[64];
};

// Global sampler context (Core 1 only)
static SamplerContext g_sampler_ctx;

// Timer ISR - extremely short, only increment counter and wake worker
static bool sampler_timer_isr(repeating_timer_t *rt) {
  SamplerContext *ctx = static_cast<SamplerContext *>(rt->user_data);

  // Increment sequence number (atomic on ARM for aligned uint32_t)
  ctx->isr_seq++;

  // Wake up worker via SEV (Send Event)
  __sev();

  return true; // Keep timer repeating
}

// Worker function - does actual I2C reads with DMA (called from Core 1 main
// loop)
static void sampler_do_work_dma(SamplerContext *ctx) {
  constexpr uint16_t kDiagCnvrfBit = (1u << 1);

  // Read current ISR sequence
  uint32_t current_seq = ctx->isr_seq;

  // Nothing new?
  if (current_seq == ctx->worker_seq) {
    return;
  }

  // Calculate how many ticks we missed
  uint32_t missed = current_seq - ctx->worker_seq - 1;
  if (missed > 0) {
    ctx->shared->samples_dropped += missed;
    ctx->shared->dropped_worker_missed_tick += missed;
  }

  // Update worker seq to current (we'll produce one sample for this seq)
  ctx->worker_seq = current_seq;

  core::SharedContext *shared = ctx->shared;
  core::RawSample sample;

  // Calculate timestamp relative to stream start (64-bit, no wraparound)
  uint64_t now64 = time_us_64();
  sample.timestamp_us = now64 - shared->stream_start_us;
  // Calculate absolute Unix timestamp at sampling time (not send time)
  sample.timestamp_unix_us = now64 +
                             static_cast<uint64_t>(shared->epoch_offset_us);

  // Gate reads by conversion-ready (CNVRF) to avoid sampling mid-conversion
  // values from VBUS/VSHUNT/CURRENT registers.
  // However, with DMA, we read DIAG_ALRT first as part of the DMA chain.
  // If CNVRF is not set in the read value, the rest of the read data is
  // technically "old", but we still got it. We should discard and not push
  // to queue if CNVRF isn't set, to avoid duplicate sequence data.

  DEBUG_DMA_PRINT("[DMA] reset addrs\n");

  // 1. Reset DMA write/read addresses
  pio_sm_clear_fifos(ctx->pio, ctx->sm);
  dma_channel_set_write_addr(ctx->dma_rx_chan, ctx->dma_rx_buf, false);
  dma_channel_set_read_addr(ctx->dma_tx_chan, ctx->dma_tx_buf, false);

  // Also reset transfer counts
  dma_channel_set_trans_count(ctx->dma_tx_chan, ctx->dma_tx_len, false);
  // Total RX bytes = 26 data bytes + 24 dummy command bytes (ADDR+W, REG,
  // ADDR+R) = 50 bytes
  dma_channel_set_trans_count(ctx->dma_rx_chan, 50, false);

  DEBUG_DMA_PRINT("[DMA] starting RX+TX (tx_len=%d)\n", (int)ctx->dma_tx_len);

  // 2. Start RX then TX
  dma_channel_start(ctx->dma_rx_chan);
  dma_channel_start(ctx->dma_tx_chan);

  DEBUG_DMA_PRINT("[DMA] waiting for RX...\n");

  // 3. Wait for RX to complete, with a timeout. This must NOT block
  // unboundedly: on an unexpected NACK the PIO program parks itself at
  // 'irq wait' (i2c.pio do_nack) and stops autopushing, so the RX DMA
  // never reaches its 50-word count. An unbounded wait here deadlocks
  // Core 1, and once the 8-entry multicore FIFO fills with unserviced
  // start/stop commands, multicore_fifo_push_blocking wedges Core 0 too.
  // Same bounded-poll + abort + resume pattern as __read_one_dma below.
  uint32_t wait_start = time_us_32();
  bool timed_out = false;
  while (dma_channel_is_busy(ctx->dma_rx_chan)) {
    if (time_us_32() - wait_start > 10000) { // 10ms timeout
      timed_out = true;
      break;
    }
  }

#ifdef POWERMONITOR_DEBUG_DMA
  DEBUG_DMA_PRINT("[DMA] RX done\n");
  DEBUG_DMA_PRINT("[DMA] rx_buf: ");
  for (int i = 0; i < 50; ++i) {
    // 32-bit words, but data is in lowest 8 bits
    DEBUG_DMA_PRINT("%02X ", ctx->dma_rx_buf[i] & 0xFF);
  }
  DEBUG_DMA_PRINT("\n");
#endif
  // Check if any errors occurred on the PIO side (NACK, timeout)
  bool ok = !timed_out && !pio_i2c_check_error(ctx->pio, ctx->sm);

  if (timed_out) {
    // Stop the DMA channels first, then hard-reset the SM: resume clears
    // the PIO FIFOs, which must not happen while DMA is still armed.
    dma_channel_abort(ctx->dma_rx_chan);
    dma_channel_abort(ctx->dma_tx_chan);
  }
  if (!ok) {
    DEBUG_DMA_PRINT("[DMA] PIO error detected, resuming\n");
    pio_i2c_resume_after_error(ctx->pio, ctx->sm);
  }
  if (timed_out) {
    // rx_buf is incomplete — do not parse or push a sample. The SM has
    // been reset above, so the next tick starts a clean transaction.
    shared->samples_dropped++;
    shared->i2c_error = true;
    return;
  }

  // 4. Parse the RX buffer
  // With Auto-Push enabled, every byte transmitted (ADDR+W, REG, ADDR+R) is
  // also pushed into the RX FIFO. We have 8 register reads, resulting in 3
  // dummy pushed bytes per register (24 total). Total: 50 bytes per DMA reading
  // sequence.
  // Read 0: 0x0B (2 bytes) -> [0]=ADDR+W, [1]=REG, [2]=ADDR+R, [3..4]=DIAG_ALRT
  // Read 1: 0x05 (3 bytes) -> [5]=ADDR+W, [6]=REG, [7]=ADDR+R, [8..10]=VBUS
  // Read 2: 0x04 (3 bytes) -> [11]=ADDR+W, [12]=REG, [13]=ADDR+R, [14..16]=VSHUNT
  // Read 3: 0x07 (3 bytes) -> [17]=ADDR+W, [18]=REG, [19]=ADDR+R, [20..22]=CURRENT
  // Read 4: 0x06 (2 bytes) -> [23]=ADDR+W, [24]=REG, [25]=ADDR+R, [26..27]=DIETEMP
  // Read 5: 0x08 (3 bytes) -> [28]=ADDR+W, [29]=REG, [30]=ADDR+R, [31..33]=POWER
  // Read 6: 0x09 (5 bytes) -> [34]=ADDR+W, [35]=REG, [36]=ADDR+R, [37..41]=ENERGY
  // Read 7: 0x0A (5 bytes) -> [42]=ADDR+W, [43]=REG, [44]=ADDR+R, [45..49]=CHARGE

  uint16_t diag_flags = (ctx->dma_rx_buf[3] << 8) | ctx->dma_rx_buf[4];

  // If CNVRF is not set, we polled too early.
  // Return early without pushing to SpscQueue to avoid duplicates.
#ifndef POWERMONITOR_TEST_MODE
  if (!(diag_flags & kDiagCnvrfBit)) {
    shared->dropped_cnvrf_not_ready++;
    return;
  }
#endif

  if (diag_flags == 0xFFFF) { // Basic I2C error pseudo-check (stuck bus)
    ok = false;
  }

  uint32_t vbus_reg24 = (ctx->dma_rx_buf[8] << 16) | (ctx->dma_rx_buf[9] << 8) |
                        ctx->dma_rx_buf[10];
  uint32_t vshunt_reg24 = (ctx->dma_rx_buf[14] << 16) |
                          (ctx->dma_rx_buf[15] << 8) | ctx->dma_rx_buf[16];
  uint32_t current_reg24 = (ctx->dma_rx_buf[20] << 16) |
                           (ctx->dma_rx_buf[21] << 8) | ctx->dma_rx_buf[22];
  int16_t temp_raw = (ctx->dma_rx_buf[26] << 8) | ctx->dma_rx_buf[27];

  uint32_t power_reg24 = (ctx->dma_rx_buf[31] << 16) | (ctx->dma_rx_buf[32] << 8) | ctx->dma_rx_buf[33];
  uint64_t energy_reg40 = ((uint64_t)ctx->dma_rx_buf[37] << 32) | ((uint64_t)ctx->dma_rx_buf[38] << 24) |
                          ((uint64_t)ctx->dma_rx_buf[39] << 16) | ((uint64_t)ctx->dma_rx_buf[40] << 8) | ctx->dma_rx_buf[41];
  uint64_t charge_reg40 = ((uint64_t)ctx->dma_rx_buf[45] << 32) | ((uint64_t)ctx->dma_rx_buf[46] << 24) |
                          ((uint64_t)ctx->dma_rx_buf[47] << 16) | ((uint64_t)ctx->dma_rx_buf[48] << 8) | ctx->dma_rx_buf[49];

  uint32_t vbus_raw = (vbus_reg24 >> 4) & 0x0FFFFF;

  int32_t vshunt_raw = (vshunt_reg24 >> 4) & 0x0FFFFF;
  if (vshunt_raw & 0x080000)
    vshunt_raw |= 0xFFF00000;

  int32_t current_raw = (current_reg24 >> 4) & 0x0FFFFF;
  if (current_raw & 0x080000)
    current_raw |= 0xFFF00000;

  int64_t charge_raw = charge_reg40;
  if (charge_raw & 0x8000000000ULL)
    charge_raw |= 0xFFFFFF0000000000ULL;

  sample.vbus_raw = vbus_raw;
  sample.vshunt_raw = vshunt_raw;
  sample.current_raw = current_raw;
  sample.dietemp_raw = temp_raw;
  sample.power_raw = power_reg24;
  sample.energy_raw = energy_reg40;
  sample.charge_raw = charge_raw;

  // Build flags from DIAG_ALRT register
  // CNVRF is bit 1, MATHOF is bit 9 in DIAG_ALRT
  sample.flags = 0;
  if (diag_flags & kDiagCnvrfBit)
    sample.flags |= core::SampleFlags::kCnvrf;
  if (diag_flags & (1 << 9))
    sample.flags |= core::SampleFlags::kMathOvf;

  // CAL_VALID: always set since we configure INA228 on startup
  sample.flags |= core::SampleFlags::kCalValid;

  if (!ok) {
    sample.flags |= core::SampleFlags::kI2cError;
    shared->i2c_error = true;
  }

  sample._pad = 0;

  // Push to queue
  if (!shared->sample_queue.push(sample)) {
    // Queue full, sample dropped
    shared->samples_dropped++;
    shared->dropped_queue_full++;
  } else {
    shared->samples_produced++;
    if (ok) {
      ctx->has_last_sent = true;
      ctx->last_vbus_raw = sample.vbus_raw;
      ctx->last_vshunt_raw = sample.vshunt_raw;
      ctx->last_current_raw = sample.current_raw;
      ctx->last_sent_time_us = now64;
    }
  }

  // Removed pio_i2c_wait_idle: The RX DMA finishes when the 28th byte is
  // pushed. The PIO immediately processes the STOP condition (takes ~1.2us).
  // The blocking wait was hitting a 10ms internal timeout because of autopull
  // interactions.
}

static void sampler_do_work_nodma(SamplerContext *ctx) {
  constexpr uint16_t kDiagCnvrfBit = (1u << 1);
  uint32_t current_seq = ctx->isr_seq;
  if (current_seq == ctx->worker_seq) {
    return;
  }
  uint32_t missed = current_seq - ctx->worker_seq - 1;
  if (missed > 0) {
    ctx->shared->samples_dropped += missed;
    ctx->shared->dropped_worker_missed_tick += missed;
  }
  ctx->worker_seq = current_seq;

  core::SharedContext *shared = ctx->shared;
  core::RawSample sample;


  bool ok = true;
  uint8_t buf[3] = {0};

  // 1. DIAG_ALRT
  if (!pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x0B, buf, 2))
    ok = false;
  uint16_t diag_flags = (buf[0] << 8) | buf[1];

#ifndef POWERMONITOR_TEST_MODE
  if (!(diag_flags & kDiagCnvrfBit)) {
    shared->dropped_cnvrf_not_ready++;
    return;
  }
#endif

  if (diag_flags == 0xFFFF)
    ok = false;

  // 2. VBUS
  if (ok && !pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x05, buf, 3))
    ok = false;
  uint32_t vbus_reg24 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
  uint32_t vbus_raw = (vbus_reg24 >> 4) & 0x0FFFFF;

  // 3. VSHUNT
  if (ok && !pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x04, buf, 3))
    ok = false;
  uint32_t vshunt_reg24 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
  int32_t vshunt_raw = (vshunt_reg24 >> 4) & 0x0FFFFF;
  if (vshunt_raw & 0x080000)
    vshunt_raw |= 0xFFF00000;

  // 4. CURRENT
  if (ok && !pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x07, buf, 3))
    ok = false;
  uint32_t current_reg24 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
  int32_t current_raw = (current_reg24 >> 4) & 0x0FFFFF;
  if (current_raw & 0x080000)
    current_raw |= 0xFFF00000;

  // 5. DIETEMP
  if (ok && !pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x06, buf, 2))
    ok = false;
  int16_t temp_raw = (buf[0] << 8) | buf[1];

  // 6. POWER
  if (ok && !pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x08, buf, 3))
    ok = false;
  uint32_t power_reg24 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

  // 7. ENERGY
  uint8_t buf5[5] = {0};
  if (ok && !pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x09, buf5, 5))
    ok = false;
  uint64_t energy_reg40 = ((uint64_t)buf5[0] << 32) | ((uint64_t)buf5[1] << 24) |
                          ((uint64_t)buf5[2] << 16) | ((uint64_t)buf5[3] << 8) | buf5[4];

  // 8. CHARGE
  if (ok && !pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x0A, buf5, 5))
    ok = false;
  uint64_t charge_reg40 = ((uint64_t)buf5[0] << 32) | ((uint64_t)buf5[1] << 24) |
                          ((uint64_t)buf5[2] << 16) | ((uint64_t)buf5[3] << 8) | buf5[4];

    uint64_t now64 = time_us_64();
    sample.timestamp_us = now64 - shared->stream_start_us;
    sample.timestamp_unix_us = now64 +
                        static_cast<uint64_t>(shared->epoch_offset_us);
  sample.vbus_raw = vbus_raw;
  sample.vshunt_raw = vshunt_raw;
  sample.current_raw = current_raw;
  sample.dietemp_raw = temp_raw;
  sample.power_raw = power_reg24;
  sample.energy_raw = energy_reg40;

  int64_t charge_raw = charge_reg40;
  if (charge_raw & 0x8000000000ULL)
    charge_raw |= 0xFFFFFF0000000000ULL;
  sample.charge_raw = charge_raw;

  sample.flags = 0;
  if (diag_flags & kDiagCnvrfBit)
    sample.flags |= core::SampleFlags::kCnvrf;
  if (diag_flags & (1 << 9))
    sample.flags |= core::SampleFlags::kMathOvf;
  sample.flags |= core::SampleFlags::kCalValid;

  if (!ok) {
    sample.flags |= core::SampleFlags::kI2cError;
    shared->i2c_error = true;
  }
  sample._pad = 0;

  if (!shared->sample_queue.push(sample)) {
    shared->samples_dropped++;
    shared->dropped_queue_full++;
  } else {
    shared->samples_produced++;
    if (ok) {
      ctx->has_last_sent = true;
      ctx->last_vbus_raw = sample.vbus_raw;
      ctx->last_vshunt_raw = sample.vshunt_raw;
      ctx->last_current_raw = sample.current_raw;
      ctx->last_sent_time_us = now64;
    }
  }
}

static void sampler_do_work(SamplerContext *ctx) {
  if (ctx->mode == SamplerMode::kPioDma) {
    sampler_do_work_dma(ctx);
  } else {
    sampler_do_work_nodma(ctx);
  }
}

// Core 1 entry point
// Two execution paths:
//   1. Timer ISR: ultra-short, only seq++ and __sev()
//   2. Worker loop: wait for event, do I2C, write to queue
static void core1_entry() {
  // Initialize sampler context
  g_sampler_ctx.timer_active = false;
  g_sampler_ctx.isr_seq = 0;
  g_sampler_ctx.worker_seq = 0;
  g_sampler_ctx.has_last_sent = false;
  g_sampler_ctx.last_vbus_raw = 0;
  g_sampler_ctx.last_vshunt_raw = 0;
  g_sampler_ctx.last_current_raw = 0;
  g_sampler_ctx.last_sent_time_us = 0;

  while (true) {
    // Check for command from Core 0 (non-blocking)
    if (multicore_fifo_rvalid()) {
      uint32_t cmd_raw = multicore_fifo_pop_blocking();

      // Memory barrier to ensure subsequent reads from shared_ctx
      // see the values written by Core 0 before the FIFO push.
      __dmb();

      core::FifoCmd cmd = static_cast<core::FifoCmd>(cmd_raw);

      switch (cmd) {
      case core::FifoCmd::kStartStream:
        if (!g_sampler_ctx.timer_active) {
          // Reset sequence counters
          g_sampler_ctx.isr_seq = 0;
          g_sampler_ctx.worker_seq = 0;
          g_sampler_ctx.has_last_sent = false;
          g_sampler_ctx.last_vbus_raw = 0;
          g_sampler_ctx.last_vshunt_raw = 0;
          g_sampler_ctx.last_current_raw = 0;
          g_sampler_ctx.last_sent_time_us = 0;

          // Reset statistics
          g_sampler_ctx.shared->reset_stats();

          // Read period from shared context
          int32_t period_us =
              -static_cast<int32_t>(g_sampler_ctx.shared->stream_period_us);

          // Start repeating timer (negative period = repeat)
          // ISR only does seq++ and __sev()
          add_repeating_timer_us(period_us, sampler_timer_isr, &g_sampler_ctx,
                                 &g_sampler_ctx.timer);
          g_sampler_ctx.timer_active = true;
        }
        break;

      case core::FifoCmd::kStopStream:
        if (g_sampler_ctx.timer_active) {
          cancel_repeating_timer(&g_sampler_ctx.timer);
          g_sampler_ctx.timer_active = false;
        }
        break;

      default:
        break;
      }
    }

    // If streaming, do work (I2C reads) when ISR signals new tick
    if (g_sampler_ctx.timer_active) {
      sampler_do_work(&g_sampler_ctx);
    }

    // Wait for event (from ISR's __sev() or FIFO)
    // This is low-power and wakes on any event
    __wfe();
  }
}

// Initialize sampler (call from Core 0 before launching Core 1)
inline void sampler_init(core::SharedContext *shared, INA228 *ina228,
                         uint8_t i2c_addr) {
  g_sampler_ctx.shared = shared;
  g_sampler_ctx.ina228 = ina228;
  g_sampler_ctx.i2c_addr = i2c_addr;
  g_sampler_ctx.mode = SamplerMode::kPioDma;
  g_sampler_ctx.timer_active = false;
  g_sampler_ctx.has_last_sent = false;
  g_sampler_ctx.last_vbus_raw = 0;
  g_sampler_ctx.last_vshunt_raw = 0;
  g_sampler_ctx.last_current_raw = 0;
  g_sampler_ctx.last_sent_time_us = 0;
}

// Initialize DMA channels and prebuild the TX buffer
// Call this from Core 0 before launching Core 1
inline void sampler_init_dma(PIO pio, uint sm, uint8_t i2c_addr) {
  g_sampler_ctx.pio = pio;
  g_sampler_ctx.sm = sm;
  g_sampler_ctx.dma_tx_chan = dma_claim_unused_channel(true);
  g_sampler_ctx.dma_rx_chan = dma_claim_unused_channel(true);

  // Build the static command sequence to read 8 registers:
  // 0x0B (DIAG_ALRT: 2 bytes), 0x05 (VBUS: 3 bytes), 0x04 (VSHUNT: 3 bytes)
  // 0x07 (CURRENT: 3 bytes), 0x06 (DIETEMP: 2 bytes), 0x08 (POWER: 3), 0x09 (ENERGY: 5), 0x0A (CHARGE: 5)
  // Total RX bytes = 26 (so 26 bytes data + 24 command dummy = 50 words in RX FIFO)
  size_t len = 0;
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x0B, 2);
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x05, 3);
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x04, 3);
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x07, 3);
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x06, 2);
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x08, 3);
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x09, 5);
  len += pio_i2c_build_command_sequence(&g_sampler_ctx.dma_tx_buf[len],
                                        i2c_addr, 0x0A, 5);
  g_sampler_ctx.dma_tx_len = len;

  // Configure TX DMA (Memory to PIO TX FIFO)
  dma_channel_config c_tx =
      dma_channel_get_default_config(g_sampler_ctx.dma_tx_chan);
  channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_16);
  channel_config_set_read_increment(&c_tx, true);
  channel_config_set_write_increment(&c_tx, false);
  channel_config_set_dreq(&c_tx, pio_get_dreq(pio, sm, true));

  dma_channel_configure(g_sampler_ctx.dma_tx_chan, &c_tx, &pio->txf[sm],
                        g_sampler_ctx.dma_tx_buf, // Source
                        g_sampler_ctx.dma_tx_len, // Length in 16-bit words
                        false);                   // Don't start yet

  // Configure RX DMA (PIO RX FIFO to Memory)
  dma_channel_config c_rx =
      dma_channel_get_default_config(g_sampler_ctx.dma_rx_chan);
  // PIO RX FIFO is read as 32-bit (words) where the byte is at the bottom.
  channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_32);
  channel_config_set_read_increment(&c_rx, false);
  channel_config_set_write_increment(&c_rx, true);
  channel_config_set_dreq(&c_rx, pio_get_dreq(pio, sm, false));

  // Total RX bytes = 26 (data) + 24 (ADDR+W, REG, ADDR+R echoes) = 50 words
  dma_channel_configure(g_sampler_ctx.dma_rx_chan, &c_rx,
                        g_sampler_ctx.dma_rx_buf, // Dest
                        &pio->rxf[sm],
                        50,     // 50 bytes to read
                        false); // Don't start yet

  // Enable RX FIFO auto-push for the I2C PIO SM so reads enter the FIFO
  pio_i2c_rx_enable(pio, sm, true);
}

// Launch Core 1 (call from Core 0 after sampler_init)
inline void sampler_launch_core1() { multicore_launch_core1(core1_entry); }

// Send start command to Core 1 (call from Core 0)
inline void sampler_start() {
  multicore_fifo_push_blocking(
      static_cast<uint32_t>(core::FifoCmd::kStartStream));
}

// Send stop command to Core 1 (call from Core 0)
inline void sampler_stop() {
  multicore_fifo_push_blocking(
      static_cast<uint32_t>(core::FifoCmd::kStopStream));
}

// ====================================================================
// Slow Registers Reading (Side-Channel for Diagnostics / Benchmarks)
// Reads POWER (0x08), ENERGY (0x09), CHARGE (0x0A)
// ====================================================================

// Non-DMA polling version
static inline bool sampler_read_slow_registers_nodma(SamplerContext *ctx, uint32_t *power, uint64_t *energy, int64_t *charge) {
  uint8_t buf[5];
  bool ok = true;

  if (!pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x08, buf, 3)) ok = false;
  else if (power) *power = (buf[0] << 16) | (buf[1] << 8) | buf[2];
  busy_wait_us_32(5);

  if (!pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x09, buf, 5)) ok = false;
  else if (energy) {
    *energy = ((uint64_t)buf[0] << 32) | ((uint64_t)buf[1] << 24) |
              ((uint64_t)buf[2] << 16) | ((uint64_t)buf[3] << 8) | buf[4];
  }
  busy_wait_us_32(5);

  if (!pio_i2c_read_reg(ctx->pio, ctx->sm, ctx->i2c_addr, 0x0A, buf, 5)) ok = false;
  else if (charge) {
    uint64_t c = ((uint64_t)buf[0] << 32) | ((uint64_t)buf[1] << 24) |
                 ((uint64_t)buf[2] << 16) | ((uint64_t)buf[3] << 8) | buf[4];
    if (c & 0x8000000000ULL) c |= 0xFFFFFF0000000000ULL; // 40-bit sign extend
    *charge = (int64_t)c;
  }
  return ok;
}

static inline bool __read_one_dma(SamplerContext *ctx, uint8_t reg, size_t size, uint32_t *rx_buf) {
  uint16_t tx_buf[32];
  int tx_len = pio_i2c_build_command_sequence(tx_buf, ctx->i2c_addr, reg, size);
  int rx_len = 3 + size; // 1 (addrW) + 1 (reg) + 1 (addrR) + size (data)

  pio_i2c_wait_idle(ctx->pio, ctx->sm);
  pio_sm_clear_fifos(ctx->pio, ctx->sm);

  dma_channel_set_read_addr(ctx->dma_tx_chan, tx_buf, false);
  dma_channel_set_trans_count(ctx->dma_tx_chan, tx_len, false);
  dma_channel_set_write_addr(ctx->dma_rx_chan, rx_buf, false);
  dma_channel_set_trans_count(ctx->dma_rx_chan, rx_len, false);

  pio_i2c_rx_enable(ctx->pio, ctx->sm, true); // Ensure autopush is armed
  dma_channel_start(ctx->dma_rx_chan);
  dma_channel_start(ctx->dma_tx_chan);

  uint32_t start = time_us_32();
  bool timed_out = false;
  while (dma_channel_is_busy(ctx->dma_rx_chan)) {
    if (time_us_32() - start > 10000) { // 10ms timeout
      timed_out = true;
      break;
    }
  }

  bool ok = !timed_out && !pio_i2c_check_error(ctx->pio, ctx->sm);

  if (timed_out) {
    uint32_t remain = dma_channel_hw_addr(ctx->dma_rx_chan)->transfer_count;
    printf("[DMA SlowRead] TIMEOUT on reg 0x%02X! Expected %d bytes, missing %lu bytes.\n", reg, rx_len, remain);
    dma_channel_abort(ctx->dma_rx_chan);
    dma_channel_abort(ctx->dma_tx_chan);
  }

  if (!ok) pio_i2c_resume_after_error(ctx->pio, ctx->sm);
  return ok;
}

// DMA version (dynamically repoints DMA channels, then restores)
static inline bool sampler_read_slow_registers_dma(SamplerContext *ctx, uint32_t *power, uint64_t *energy, int64_t *charge) {
  uint32_t rx_buf[10];
  bool ok = true;

  if (ok && power) {
    if (__read_one_dma(ctx, 0x08, 3, rx_buf)) {
      *power = (rx_buf[3] << 16) | (rx_buf[4] << 8) | rx_buf[5];
    } else ok = false;
    busy_wait_us_32(5); // t_BUF bus free time
  }

  if (ok && energy) {
    if (__read_one_dma(ctx, 0x09, 5, rx_buf)) {
      *energy = ((uint64_t)rx_buf[3] << 32) | ((uint64_t)rx_buf[4] << 24) |
                ((uint64_t)rx_buf[5] << 16) | ((uint64_t)rx_buf[6] << 8) | rx_buf[7];
    } else ok = false;
    busy_wait_us_32(5); // t_BUF bus free time
  }

  if (ok && charge) {
    if (__read_one_dma(ctx, 0x0A, 5, rx_buf)) {
      uint64_t c = ((uint64_t)rx_buf[3] << 32) | ((uint64_t)rx_buf[4] << 24) |
                   ((uint64_t)rx_buf[5] << 16) | ((uint64_t)rx_buf[6] << 8) | rx_buf[7];
      if (c & 0x8000000000ULL) c |= 0xFFFFFF0000000000ULL;
      *charge = (int64_t)c;
    } else ok = false;
  }

  // Restore is not needed because sampler_do_work_dma
  // explicitly resets all read/write addresses on every cycle.

  return ok;
}

} // namespace device

#endif // DEVICE_SAMPLER_HPP
