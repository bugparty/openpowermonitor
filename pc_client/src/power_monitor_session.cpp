#include "power_monitor_session.h"
#include "thread_affinity.h"
#include "onboard_sampler.h"
#include "onboard_sample_queue.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <CLI/CLI.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/loop.hpp>
#include <ftxui/component/mouse.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <serial/serial.h>

#include "protocol/frame_builder.h"
#include "protocol/unpack.h"
#include "protocol/serialization.h"
#include "protocol_helpers.h"
#include "read_thread.h"
#include "response_queue.h"
#include "sample_queue.h"
#include "session.h"


namespace powermonitor {
namespace client {

using namespace ftxui;

namespace {
std::atomic<bool> g_signal_interrupted{false};

struct PicoEngineeringValues {
    double vbus_v = 0.0;
    double current_a = 0.0;
    double temp_c = 0.0;
    double power_w = 0.0;
    double energy_j = 0.0;
};

PicoEngineeringValues compute_pico_engineering_values(const Session::Config& config, const Session::Sample& sample) {
    const double vbus_lsb = 195.3125e-6;
    const double current_lsb = config.current_lsb_nA * 1e-9;
    const double temp_lsb = 7.8125e-3;
    const double power_lsb = current_lsb * 3.2;
    const double energy_lsb = current_lsb * 3.2 * 16.0;

    PicoEngineeringValues values;
    values.vbus_v = sample.vbus_raw * vbus_lsb;
    values.current_a = sample.current_raw * current_lsb;
    values.temp_c = sample.temp_raw * temp_lsb;
    values.power_w = sample.power_raw * power_lsb;
    values.energy_j = sample.energy_raw * energy_lsb;
    return values;
}

constexpr uint16_t kStreamMaskUsbStressMode = 0x8000;

// Time sync: rounds and outlier filter (see docs/device/time_sync.md)
constexpr int kInitialSyncRounds = 10;
constexpr int kPeriodicSyncRounds = 3;
constexpr int64_t kMaxOffsetAfterInitUs = 1000;   // Reject |offset| > 1000 us after init
constexpr uint64_t kInitPeriodUs = 30000000ULL;    // 30 seconds from sync start before applying filter
constexpr int64_t kAsymmetryThresholdUs = 2000;    // Asymmetry threshold — tune per link

uint64_t now_steady_us() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

// Get current Unix time in microseconds (real absolute time)
uint64_t now_unix_us() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

int64_t signed_diff_u64(uint64_t a, uint64_t b) {
    if (a >= b) {
        return static_cast<int64_t>(a - b);
    }
    return -static_cast<int64_t>(b - a);
}

int64_t choose_offset(int64_t T1, int64_t T2, int64_t T3, int64_t T4) {
    int64_t forward = T1 - T2;
    int64_t reverse = T3 - T4;
    int64_t offset_ntp = ((T1 - T2) + (T4 - T3)) / 2;
    int64_t asym = llabs(forward - reverse);

    // Asymmetry threshold — tune per link; 2000us is a good starting point
    if (asym > kAsymmetryThresholdUs) {
        return forward;    // Asymmetric link: fall back to forward-path-only offset
    } else {
        return offset_ntp; // Symmetric link: use full NTP offset
    }
}

// Scrollable log area: focusable component with scroll_y in [0,1]. Uses
// focusPositionRelative(0, scroll_y) so yframe shows the correct region.
class ScrollableLogArea : public ComponentBase {
public:
    using LogGetter = std::function<std::deque<std::string>()>;
    explicit ScrollableLogArea(LogGetter get_logs)
        : ComponentBase(Components{}), get_logs_(std::move(get_logs)) {}

    Element OnRender() override {
        std::deque<std::string> logs = get_logs_();
        Elements log_lines;
        if (logs.empty()) {
            log_lines.push_back(text("(no logs yet)"));
        } else {
            for (auto it = logs.rbegin(); it != logs.rend(); ++it) {
                log_lines.push_back(text(*it));
            }
        }
        return vbox(std::move(log_lines))
             | focusPositionRelative(0.f, scroll_y_)
             | vscroll_indicator
             | yframe
             | size(HEIGHT, LESS_THAN, 15);
    }

    bool OnEvent(Event event) override {
        constexpr float step = 0.1f;
        if (event == Event::PageUp) {
            scroll_y_ = std::max(0.f, scroll_y_ - step);
            return true;
        }
        if (event == Event::PageDown) {
            scroll_y_ = std::min(1.f, scroll_y_ + step);
            return true;
        }
        if (event.is_mouse()) {
            if (event.mouse().button == Mouse::WheelUp) {
                scroll_y_ = std::max(0.f, scroll_y_ - step);
                return true;
            }
            if (event.mouse().button == Mouse::WheelDown) {
                scroll_y_ = std::min(1.f, scroll_y_ + step);
                return true;
            }
        }
        return false;
    }

    bool Focusable() const override { return true; }

private:
    LogGetter get_logs_;
    float scroll_y_ = 1.f;  // 0 = top, 1 = bottom (newest)
};

#ifdef _WIN32
BOOL WINAPI console_ctrl_handler(DWORD ctrl_type) {
    switch (ctrl_type) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
        g_signal_interrupted.store(true);
        // Return TRUE immediately to block system default handling
        return TRUE;
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        g_signal_interrupted.store(true);
        // Give the main thread a moment to respond
        Sleep(1000);
        return TRUE;
    default:
        return FALSE;
    }
}
#else
void signal_handler(int) {
    g_signal_interrupted.store(true);
}
#endif
}  // namespace

PowerMonitorSession::PowerMonitorSession(const Options& options)
    : options_(options) {
    sample_queue_ = std::make_unique<SampleQueue>();
    response_queue_ = std::make_unique<ResponseQueue>();
    session_ = std::make_unique<Session>();
    stats_ = std::make_unique<ThreadStats>();

}

PowerMonitorSession::~PowerMonitorSession() {
    if (read_thread_ && read_thread_->is_running()) {
        stop();
    }
}

int PowerMonitorSession::run() {
    // Register signal handlers
#ifdef _WIN32
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#else
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
#endif

    session_start_unix_us_ = now_unix_us();

    append_log("Connecting to device...");

    if (!connect_device()) {
        std::cerr << "Failed to connect to device" << std::endl;
        append_log("Failed to connect to device");
        return 1;
    }
    {
        std::lock_guard<std::mutex> lock(ui_state_.mutex);
        ui_state_.connected = true;
    }

    append_log("Initializing device...");

    if (!initialize_device()) {
        std::cerr << "Failed to initialize device" << std::endl;
        append_log("Failed to initialize device");
        return 1;
    }
    {
        std::lock_guard<std::mutex> lock(ui_state_.mutex);
        ui_state_.initialized = true;
    }

    append_log("Streaming started.");

    // Start onboard sampler if enabled
    if (options_.onboard_enabled) {
        append_log("Starting onboard hwmon sampler...");
        onboard_queue_ = std::make_shared<OnboardSampleQueue>();

        OnboardSampler::Config onboard_cfg;
        onboard_cfg.hwmon_path = options_.onboard_hwmon_path;
        onboard_cfg.period_us = options_.onboard_period_us;
        onboard_cfg.telemetry_period_us = options_.onboard_telemetry_period_us;
        onboard_cfg.cpu_core = options_.onboard_cpu_core;
        onboard_cfg.rt_prio = options_.onboard_rt_prio;
        onboard_cfg.jetson_freq_path = options_.onboard_jetson_freq_path;

        onboard_sampler_ = std::make_unique<OnboardSampler>(onboard_cfg, onboard_queue_);

        if (!onboard_sampler_->start()) {
            std::cerr << "Error: Failed to start onboard sampler: "
                      << onboard_sampler_->get_last_error() << std::endl;
            append_log("Error: Failed to start onboard sampler: " + onboard_sampler_->get_last_error());
            return 1;
        } else {
            // Start onboard processing thread
            onboard_proc_thread_ = std::thread([this] {
                process_onboard_loop();
            });
            append_log("Onboard sampler started on " + options_.onboard_hwmon_path);

            // Set onboard meta in session
            Session::OnboardMeta onboard_meta;
            onboard_meta.hwmon_path = options_.onboard_hwmon_path;
            onboard_meta.source = "onboard_cpp";
            session_->set_onboard_meta(onboard_meta);
        }
    }

    // Start sample processing loop
    std::thread processor([this] {
#ifndef _WIN32
        if (options_.proc_thread_core >= 0) {
            ThreadAffinity::SetCpuAffinity(options_.proc_thread_core);
        }
#endif
        process_samples_loop();
    });

    // Start periodic flush thread
    if (!options_.output_file.empty()) {
        std::string flush_dir = options_.output_file + ".chunks";
        std::filesystem::create_directories(flush_dir);
        session_->set_flush_dir(flush_dir);
        flush_thread_ = std::thread([this] { flush_loop(); });
        append_log("Periodic flush enabled, interval=" + std::to_string(options_.flush_interval_s) + "s");
    }

    int tui_rc = 0;
    if (options_.interactive) {
        tui_rc = run_tui_loop();
    } else {
        auto next_stats_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point deadline;
        bool has_deadline = false;
        if (options_.duration_us > 0 || options_.duration_s > 0) {
            uint64_t run_us = options_.duration_us;
            if (run_us == 0 && options_.duration_s > 0) {
                run_us = static_cast<uint64_t>(options_.duration_s) * 1000000ULL;
            }
            deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(run_us);
            has_deadline = true;
        }

        while (!g_signal_interrupted.load() && !stop_requested_.load()) {
            if (has_deadline && std::chrono::steady_clock::now() >= deadline) {
                append_log("Capture duration reached, stopping session");
                break;
            }
            protocol::DynamicFrame async_frame;
            if (response_queue_->pop_wait(async_frame, 50)) {
                if (async_frame.type == protocol::FrameType::kEvt &&
                    async_frame.msgid == static_cast<uint8_t>(protocol::MsgId::kTimeSyncRequest) && streaming_.load()) {
                    on_time_sync_request();
                } else {
                    process_async_control_frame(async_frame);
                }
            }
            if (std::chrono::steady_clock::now() >= next_stats_time) {
                print_statistics(true);
                next_stats_time += std::chrono::milliseconds(100);
            }
        }
        std::cout << "\r" << std::string(last_stats_width_, ' ') << "\r" << std::flush;
    }

    append_log("Stopping session...");
    stop_requested_ = true;
    sample_queue_->stop();

    // Stop onboard sampler
    if (onboard_sampler_) {
        onboard_sampler_->stop();
        if (onboard_queue_) {
            onboard_queue_->stop();
        }
    }

    if (streaming_.load()) {
        stop_streaming();
    }


    if (processor.joinable()) {
        processor.join();
    }

    // Join onboard processing thread
    if (onboard_proc_thread_.joinable()) {
        onboard_proc_thread_.join();
    }

    // Join onboard sampler
    if (onboard_sampler_) {
        onboard_sampler_->join();
    }

    session_end_unix_us_ = now_unix_us();

    // Join flush thread before saving
    if (flush_thread_.joinable()) {
        flush_thread_.join();
    }

    save_and_exit();

    std::cout << "Session complete. Samples collected: "
              << sample_counter_.load(std::memory_order_relaxed) << std::endl;
    append_log("Session complete.");

    return tui_rc;
}

void PowerMonitorSession::flush_loop() {
    while (!stop_requested_.load()) {
        for (uint32_t i = 0; i < options_.flush_interval_s && !stop_requested_.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (stop_requested_.load()) break;

        if (session_->flush_to_chunks()) {
            append_log("Flushed samples to chunk files (total pico: " +
                std::to_string(session_->total_pico_count()) + ")");
        } else {
            append_log("WARNING: Flush failed");
        }
    }
}

void PowerMonitorSession::stop() {
    stop_requested_ = true;
    sample_queue_->stop();
    if (read_thread_) {
        read_thread_->join();
    }
}

bool PowerMonitorSession::connect_device() {
    try {
        serial_ = std::make_unique<serial::Serial>(
            options_.port,
            options_.baud_rate,
            serial::Timeout::simpleTimeout(2000),
            serial::bytesize_t::eightbits,
            serial::parity_t::parity_none,
            serial::stopbits_t::stopbits_one,
            serial::flowcontrol_t::flowcontrol_none,
            serial::dtrcontrol_t::dtr_enable,
            serial::rtscontrol_t::rts_enable
        );

        if (!serial_->isOpen()) {
            std::cerr << "Failed to open serial port: " << options_.port << std::endl;
            return false;
        }



        return true;
    } catch (const serial::SerialException& e) {
        std::cerr << "Serial error: " << e.what() << std::endl;
        return false;
    }
}

bool PowerMonitorSession::initialize_device() {
    // Start read thread (must be started before sending any commands)
    read_thread_ = std::make_unique<ReadThread>(
        serial_.get(), sample_queue_.get(), response_queue_.get(),
        &stop_requested_, stats_.get(),
        options_.read_thread_core, options_.rt_prio
    );
    read_thread_->start();

    // Allow time for thread startup and USB CDC init
    //std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    // Send PING
    if (!send_ping()) {
        std::cerr << "PING failed" << std::endl;
        return false;
    }

    append_log("Device responded to PING");

    // Get device config
    if (!get_device_config()) {
        std::cerr << "Failed to get device configuration" << std::endl;
        return false;
    }

    append_log("Device configuration obtained");

    // Ensure initial calibration runs before any data collection.
    // Device may still be streaming from a previous session, so try to stop it first.
    send_command_with_retry(protocol::MsgId::kStreamStop, {});

    // Drop any samples that might have arrived before calibration completed.
    SampleQueue::Sample dropped_sample;
    while (sample_queue_->pop(dropped_sample)) {
    }

    // Set initial epoch with TIME_SET (Unix time), then fine-tune with TIME_SYNC rounds (Unix time on both sides).
    {
        std::vector<uint8_t> time_set_payload;
        // Use system_clock for real Unix time (steady_clock is relative to process start)
        auto now = std::chrono::system_clock::now().time_since_epoch();
        uint64_t unix_time_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count());
        protocol::append_u64(time_set_payload, unix_time_us);
        if (send_command_with_retry(protocol::MsgId::kTimeSet, time_set_payload, nullptr, true)) {
            // Log with actual time value for debugging
            std::ostringstream oss;
            oss << "TIME_SET succeeded: unix_time_us=" << unix_time_us;
            append_log(oss.str());
            emit_time_sync_debug(oss.str());
            // Record sync start time for offset filtering
            sync_start_time_us_ = now_unix_us();
            if (!run_time_sync_rounds(kInitialSyncRounds)) {
                std::cerr << "Initial TIME_SYNC failed, continuing with degraded timestamp accuracy" << std::endl;
                append_log("WARNING: Initial TIME_SYNC failed, device running with uncalibrated timestamps");
                time_sync_succeeded_ = false;
            } else {
                append_log("Initial time sync completed (" + std::to_string(kInitialSyncRounds) + " rounds)");
                time_sync_succeeded_ = true;
            }
        } else {
            append_log("TIME_SET failed, device time not synchronized!");
            time_sync_succeeded_ = false;
        }
    }

    // Start stream
    if (!start_streaming()) {
        std::cerr << "Failed to start streaming" << std::endl;
        return false;
    }

    return true;
}

bool PowerMonitorSession::send_ping() {
    return send_command_with_retry(protocol::MsgId::kPing, {});
}

bool PowerMonitorSession::get_device_config() {
    std::vector<uint8_t> rsp_data;
    if (!send_command_with_retry(protocol::MsgId::kGetCfg, {}, &rsp_data)) {
        return false;
    }

    // Wait for CFG_REPORT (0x91)
    protocol::DynamicFrame frame;
    if (!wait_for_message_by_id(protocol::MsgId::kCfgReport, &frame, 2000)) {
        std::cerr << "Timeout waiting for CFG_REPORT" << std::endl;
        return false;
    }

    if (frame.data.size() < 16) {
        std::cerr << "Invalid CFG_REPORT" << std::endl;
        return false;
    }

    Session::Config config;
    config.current_lsb_nA = unpack_u32(frame.data.data() + 2);
    config.adcrange = (frame.data[1] >> 2) & 0x01;
    config.shunt_cal = unpack_u16(frame.data.data() + 6);
    config.config_reg = unpack_u16(frame.data.data() + 8);
    config.adc_config_reg = unpack_u16(frame.data.data() + 10);
    config.stream_period_us = unpack_u16(frame.data.data() + 12);
    config.stream_mask = unpack_u16(frame.data.data() + 14);

    session_->set_config(config);

    return true;
}

bool PowerMonitorSession::start_streaming() {
    std::vector<uint8_t> payload(4);
    pack_u16(payload.data(), options_.stream_period_us);
    uint16_t stream_mask = options_.stream_mask;
    if (options_.usb_stress_mode) {
        stream_mask = static_cast<uint16_t>(stream_mask | kStreamMaskUsbStressMode);
        append_log("USB stress mode enabled (STREAM_START mask bit15)");
    }
    pack_u16(payload.data() + 2, stream_mask);

    const bool ok = send_command_with_retry(protocol::MsgId::kStreamStart, payload);
    streaming_.store(ok);
    {
        std::lock_guard<std::mutex> lock(ui_state_.mutex);
        ui_state_.streaming = ok;
    }
    if (ok) {
        append_log("STREAM_START acknowledged");
    }
    return ok;
}

bool PowerMonitorSession::stop_streaming() {
    const bool ok = send_command_with_retry(protocol::MsgId::kStreamStop, {});
    streaming_.store(false);
    {
        std::lock_guard<std::mutex> lock(ui_state_.mutex);
        ui_state_.streaming = false;
    }
    append_log(ok ? "STREAM_STOP acknowledged" : "STREAM_STOP failed");
    return ok;
}

bool PowerMonitorSession::perform_time_sync_once(std::string* detail, int64_t* out_offset, bool send_adjust) {
    if (detail) {
        detail->clear();
    }
    if (out_offset) {
        *out_offset = 0;
    }

    // Use system_clock (Unix time) for both T1 and T4; device returns T2/T3 in Unix time after TIME_SET
    std::vector<uint8_t> sync_payload;
    protocol::append_u64(sync_payload, now_unix_us());

    std::vector<uint8_t> sync_rsp;
    if (!send_command_with_retry(protocol::MsgId::kTimeSync, sync_payload, &sync_rsp, false)) {
        if (detail) {
            *detail = "TIME_SYNC cmd failed";
        }
        return false;
    }
    if (sync_rsp.size() < 26) {
        if (detail) {
            *detail = "TIME_SYNC rsp too short";
        }
        return false;
    }

    const uint64_t T1 = protocol::read_u64(sync_rsp, 2);
    const uint64_t T2 = protocol::read_u64(sync_rsp, 10);
    const uint64_t T3 = protocol::read_u64(sync_rsp, 18);
    const uint64_t T4 = now_unix_us();

    const int64_t delay = signed_diff_u64(T4, T1) - signed_diff_u64(T3, T2);
    const int64_t offset = choose_offset(T1, T2, T3, T4);
    // Outlier filter: after kInitPeriodUs from sync start, reject |offset| > kMaxOffsetAfterInitUs
    if (sync_start_time_us_ > 0 && (T4 - sync_start_time_us_) > kInitPeriodUs) {
        if (offset > kMaxOffsetAfterInitUs || offset < -kMaxOffsetAfterInitUs) {
            if (detail) {
                *detail = "offset rejected: " + std::to_string(offset) + "us (max " +
                          std::to_string(static_cast<int>(kMaxOffsetAfterInitUs)) + "us after 30s)";
            }
            return false;
        }
    }

    if (out_offset) {
        *out_offset = offset;
    }

    if (send_adjust) {
        std::vector<uint8_t> adjust_payload;
        protocol::append_i64(adjust_payload, -offset);
        if (!send_command_with_retry(protocol::MsgId::kTimeAdjust, adjust_payload, nullptr, false)) {
            if (detail) {
                std::ostringstream oss;
                oss << "TIME_ADJUST failed offset=" << offset << "us"
                    << " delay=" << delay << "us";
                *detail = oss.str();
            }
            return false;
        }
    }

    if (detail) {
        std::ostringstream oss;
        oss << "offset=" << offset << "us"
            << " delay=" << delay << "us"
            << " T1=" << T1 << " T2=" << T2 << " T3=" << T3 << " T4=" << T4;
        *detail = oss.str();
    }
    return true;
}

bool PowerMonitorSession::run_time_sync_rounds(int rounds) {
    if (rounds <= 0) {
        return true;
    }
    std::vector<int64_t> offsets;
    for (int i = 0; i < rounds; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        std::string detail;
        int64_t offset = 0;
        // Pass false for send_adjust so we can calculate min offset and send once at the end
        if (!perform_time_sync_once(&detail, &offset, false)) {
            append_log("Time sync round " + std::to_string(i + 1) + "/" + std::to_string(rounds) + " failed: " + detail);
            emit_time_sync_debug("Time sync round " + std::to_string(i + 1) + "/" + std::to_string(rounds) + " failed: " + detail);
            return false;
        }
        offsets.push_back(offset);
        append_log("Time sync round " + std::to_string(i + 1) + "/" + std::to_string(rounds) + " ok: offset=" + std::to_string(offset) + "us");
        append_log(detail);
        emit_time_sync_debug("Time sync round " + std::to_string(i + 1) + "/" + std::to_string(rounds) + " ok: offset=" + std::to_string(offset) + "us");
        emit_time_sync_debug(detail);
    }

    // Apply minimum offset (best result) unless disabled by --no-apply-time-offset
    if (!options_.no_apply_time_offset && !offsets.empty()) {
        int64_t min_offset = *std::min_element(offsets.begin(), offsets.end());
        std::vector<uint8_t> adjust_payload;
        protocol::append_i64(adjust_payload, -min_offset);
        if (send_command_with_retry(protocol::MsgId::kTimeAdjust, adjust_payload, nullptr, false)) {
            append_log("Applied min offset: " + std::to_string(min_offset) + "us");
            emit_time_sync_debug("Applied min offset: " + std::to_string(min_offset) + "us");
        }
    } else if (options_.no_apply_time_offset && !offsets.empty()) {
        int64_t min_offset = *std::min_element(offsets.begin(), offsets.end());
        append_log("Time sync offset (not applied): min=" + std::to_string(min_offset) + "us (--no-apply-time-offset)");
        emit_time_sync_debug("Time sync offset (not applied): min=" + std::to_string(min_offset) + "us (--no-apply-time-offset)");
    }
    if (options_.debug_time_sync) {
        debug_time_sync_samples_remaining_.store(8, std::memory_order_relaxed);
    }
    return true;
}

void PowerMonitorSession::on_time_sync_request() {
    if (!streaming_.load()) {
        return;
    }
    if (run_time_sync_rounds(kPeriodicSyncRounds)) {
        append_log("Periodic time sync ok (" + std::to_string(kPeriodicSyncRounds) + " rounds)");
    } else {
        append_log("Periodic time sync failed");
    }
}

bool PowerMonitorSession::send_command_with_retry(protocol::MsgId msgid,
                                                   const std::vector<uint8_t>& payload,
                                                   std::vector<uint8_t>* rsp_data,
                                                   bool try_lock_command) {
    const uint8_t msgid_u8 = static_cast<uint8_t>(msgid);
    std::unique_lock<std::mutex> lock(command_mutex_, std::defer_lock);
    if (try_lock_command) {
        if (!lock.try_lock()) {
            return false;
        }
    } else {
        lock.lock();
    }
    const int kMaxRetries = 3;
    const int kTimeoutMs = 500;

    for (int retry = 0; retry < kMaxRetries; ++retry) {
        if (stop_requested_.load()) {
            return false;
        }
        try {
            // Build frame
            std::vector<uint8_t> frame_data = protocol::build_frame(
                protocol::FrameType::kCmd, 0, cmd_seq_++, msgid_u8, payload
            );
            const uint8_t sent_seq = cmd_seq_ - 1;

            // Send
            serial_->write(frame_data);
            stats_->tx_counts[msgid_u8].fetch_add(1, std::memory_order_relaxed);

            // Wait for response
            protocol::DynamicFrame rsp;
            if (wait_for_response(sent_seq, msgid, &rsp, kTimeoutMs)) {
                // RSP DATA = [orig_msgid, status, extra...]
                // data[0] = orig_msgid, data[1] = status
                if (rsp.data.size() >= 2 && rsp.data[1] == 0x00) {  // OK
                    if (rsp_data) {
                        *rsp_data = rsp.data;
                    }
                    return true;
                }
                // Non-OK status, fail immediately (no retry)
                if (rsp.data.size() >= 2) {
                    std::cerr << "Command failed with status: 0x"
                              << std::hex << static_cast<int>(rsp.data[1]) << std::dec << std::endl;
                }
                return false;
            }

            // Timeout, retry
            std::cerr << "Command timeout, retry " << (retry + 1) << "/" << kMaxRetries << std::endl;
            stats_->timeouts.fetch_add(1, std::memory_order_relaxed);
            stats_->retries.fetch_add(1, std::memory_order_relaxed);

        } catch (const serial::SerialException& e) {
            std::cerr << "Serial error: " << e.what() << std::endl;
            return false;
        }
    }

    return false;
}

bool PowerMonitorSession::wait_for_response(uint8_t expected_seq,
                                             protocol::MsgId expected_msgid,
                                             protocol::DynamicFrame* frame,
                                             int timeout_ms) {
    const uint8_t expected_msgid_u8 = static_cast<uint8_t>(expected_msgid);
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (!stop_requested_.load() && std::chrono::steady_clock::now() < deadline) {
        int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()
            ).count()
        );

        remaining_ms = std::max(remaining_ms, 0);

        if (response_queue_->pop_wait(*frame, remaining_ms)) {
            // Only RSP frames can satisfy command/response matching.
            if (frame->type != protocol::FrameType::kRsp) {
                process_async_control_frame(*frame);
                continue;
            }

            if (frame->seq == expected_seq) {
                if (frame->data.size() < 2 || frame->data[0] != expected_msgid_u8) {
                    process_async_control_frame(*frame);
                    continue;
                }
                return true;
            }

            // SEQ mismatch, keep waiting (may be other response)
            process_async_control_frame(*frame);
        }
    }

    return false;  // Timeout
}

bool PowerMonitorSession::wait_for_message_by_id(protocol::MsgId expected_msgid,
                                                  protocol::DynamicFrame* frame,
                                                  int timeout_ms) {
    const uint8_t expected_msgid_u8 = static_cast<uint8_t>(expected_msgid);
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()
            ).count()
        );

        remaining_ms = std::max(remaining_ms, 0);

        if (response_queue_->pop_wait(*frame, remaining_ms)) {
            if (frame->msgid == expected_msgid_u8) {
                return true;
            }
            // msgid mismatch, keep waiting (may be other message)
            process_async_control_frame(*frame);
        }
    }

    return false;  // Timeout
}

void PowerMonitorSession::process_async_control_frame(const protocol::DynamicFrame& frame) {
    if (frame.type == protocol::FrameType::kEvt && frame.msgid == static_cast<uint8_t>(protocol::MsgId::kTextReport)) {
        const std::string text(frame.data.begin(), frame.data.end());
        append_log("TEXT_REPORT len=" + std::to_string(frame.data.size()) + " text=\"" + text + "\"");
        return;
    }

    if (frame.type == protocol::FrameType::kEvt && frame.msgid == static_cast<uint8_t>(protocol::MsgId::kTimeSyncRequest)) {
        append_log("EVT_TIME_SYNC_REQUEST (sync triggered by caller)");
        return;
    }

    if (frame.msgid != static_cast<uint8_t>(protocol::MsgId::kStatsReport)) {
        return;
    }
    if (frame.data.size() < 31) {
        append_log("STATS_REPORT ignored: payload too short");
        return;
    }

    const uint16_t report_seq = unpack_u16(frame.data.data());
    const uint32_t produced = unpack_u32(frame.data.data() + 2);
    const uint32_t dropped = unpack_u32(frame.data.data() + 6);
    const uint32_t dropped_cnvrf_not_ready = unpack_u32(frame.data.data() + 10);
    const uint32_t dropped_duplicate_suppressed = unpack_u32(frame.data.data() + 14);
    const uint32_t dropped_worker_missed_tick = unpack_u32(frame.data.data() + 18);
    const uint32_t dropped_queue_full = unpack_u32(frame.data.data() + 22);
    const uint16_t queue_depth = unpack_u16(frame.data.data() + 26);
    const uint8_t reason_bits = frame.data[28];
    const uint16_t window_ms = unpack_u16(frame.data.data() + 29);

    {
        std::lock_guard<std::mutex> lock(ui_state_.mutex);
        ui_state_.has_stats_report = true;
        ui_state_.stats_report_seq = report_seq;
        ui_state_.stats_samples_produced = produced;
        ui_state_.stats_samples_dropped = dropped;
        ui_state_.stats_dropped_cnvrf_not_ready = dropped_cnvrf_not_ready;
        ui_state_.stats_dropped_duplicate_suppressed = dropped_duplicate_suppressed;
        ui_state_.stats_dropped_worker_missed_tick = dropped_worker_missed_tick;
        ui_state_.stats_dropped_queue_full = dropped_queue_full;
        ui_state_.stats_queue_depth = queue_depth;
        ui_state_.stats_reason_bits = reason_bits;
        ui_state_.stats_window_ms = window_ms;
    }

    std::ostringstream oss;
    oss << "STATS_REPORT seq=" << report_seq
        << " produced=" << produced
        << " dropped=" << dropped
        << " q_depth=" << queue_depth
        << " reason=0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(reason_bits)
        << std::dec << std::setfill(' ')
        << " window_ms=" << window_ms;
    append_log(oss.str());
}

void PowerMonitorSession::process_samples_loop() {
    SampleQueue::Sample sample;

    while (!stop_requested_.load()) {
        if (sample_queue_->pop_wait(sample)) {
            if (sample.raw_data.size() < 41) {
                continue;
            }

            Session::Sample session_sample;
            session_sample.seq = sample.seq;
            session_sample.host_timestamp_us = sample.host_timestamp_us;
            session_sample.device_timestamp_us = sample.device_timestamp_us;
            session_sample.device_timestamp_unix_us = sample.device_timestamp_unix_us;

            const size_t flags_offset = 16;
            const size_t vbus_offset = 17;
            const size_t vshunt_offset = 20;
            const size_t current_offset = 23;
            const size_t power_offset = 26;
            const size_t temp_offset = 29;
            const size_t energy_offset = 31;
            const size_t charge_offset = 36;

            session_sample.flags = sample.raw_data[flags_offset];
            session_sample.vbus_raw = protocol::unpack_u20(sample.raw_data.data() + vbus_offset);
            session_sample.vshunt_raw = protocol::unpack_s20(sample.raw_data.data() + vshunt_offset);
            session_sample.current_raw = protocol::unpack_s20(sample.raw_data.data() + current_offset);
            session_sample.power_raw = protocol::unpack_u24(sample.raw_data.data() + power_offset);
            session_sample.temp_raw = protocol::unpack_s16(sample.raw_data.data() + temp_offset);
            session_sample.energy_raw = protocol::unpack_u40(sample.raw_data.data() + energy_offset);
            session_sample.charge_raw = protocol::unpack_s40(sample.raw_data.data() + charge_offset);

            session_->add_sample(session_sample);
            export_pico_power_sample(session_sample);
            maybe_debug_time_sync_sample(sample);
            sample_counter_.fetch_add(1, std::memory_order_relaxed);
            if ((session_sample.flags & 0x10U) == 0) {
                std::lock_guard<std::mutex> lock(ui_state_.mutex);
                ui_state_.latest_sample = session_sample;
                ui_state_.has_latest_sample = true;
            }
        }
    }

    while (sample_queue_->pop(sample)) {
        if (sample.raw_data.size() < 41) {
            continue;
        }

        Session::Sample session_sample;
        session_sample.seq = sample.seq;
        session_sample.host_timestamp_us = sample.host_timestamp_us;
        session_sample.device_timestamp_us = sample.device_timestamp_us;
        session_sample.device_timestamp_unix_us = sample.device_timestamp_unix_us;

        const size_t flags_offset = 16;
        const size_t vbus_offset = 17;
        const size_t vshunt_offset = 20;
        const size_t current_offset = 23;
        const size_t power_offset = 26;
        const size_t temp_offset = 29;
        const size_t energy_offset = 31;
        const size_t charge_offset = 36;

        session_sample.flags = sample.raw_data[flags_offset];
        session_sample.vbus_raw = protocol::unpack_u20(sample.raw_data.data() + vbus_offset);
        session_sample.vshunt_raw = protocol::unpack_s20(sample.raw_data.data() + vshunt_offset);
        session_sample.current_raw = protocol::unpack_s20(sample.raw_data.data() + current_offset);
        session_sample.power_raw = protocol::unpack_u24(sample.raw_data.data() + power_offset);
        session_sample.temp_raw = protocol::unpack_s16(sample.raw_data.data() + temp_offset);
        session_sample.energy_raw = protocol::unpack_u40(sample.raw_data.data() + energy_offset);
        session_sample.charge_raw = protocol::unpack_s40(sample.raw_data.data() + charge_offset);

        session_->add_sample(session_sample);
        export_pico_power_sample(session_sample);
        maybe_debug_time_sync_sample(sample);
        sample_counter_.fetch_add(1, std::memory_order_relaxed);
        if ((session_sample.flags & 0x10U) == 0) {
            std::lock_guard<std::mutex> lock(ui_state_.mutex);
            ui_state_.latest_sample = session_sample;
            ui_state_.has_latest_sample = true;
        }
    }
}

void PowerMonitorSession::process_onboard_loop() {
    if (!onboard_queue_) {
        return;
    }

    OnboardSample sample;
    while (!stop_requested_.load()) {
        if (onboard_queue_->pop_wait(sample)) {
            session_->add_onboard_sample(sample);
            export_onboard_power_sample(sample);
        }
    }

    while (onboard_queue_->pop(sample)) {
        session_->add_onboard_sample(sample);
        export_onboard_power_sample(sample);
    }
}

void PowerMonitorSession::export_pico_power_sample(const Session::Sample& sample) {
    if (!pico_ring_buffer_.ok()) {
        return;
    }

    const PicoEngineeringValues values = compute_pico_engineering_values(session_->get_config(), sample);

    PicoSample power_sample{};
    power_sample.sequence_num = realtime_pico_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    power_sample.flags = sample.flags;
    power_sample.host_timestamp_us = sample.host_timestamp_us;
    power_sample.unix_timestamp_us = sample.device_timestamp_unix_us;
    power_sample.device_timestamp_us = sample.device_timestamp_us;
    power_sample.power_w = values.power_w;
    power_sample.voltage_v = values.vbus_v;
    power_sample.current_a = values.current_a;
    power_sample.ina228_temp_c = values.temp_c;
    power_sample.energy_j = values.energy_j;

    pico_ring_buffer_.push(power_sample);
}

void PowerMonitorSession::export_onboard_power_sample(const OnboardSample& sample) {
    if (!onboard_ring_buffer_.ok()) {
        return;
    }

    OnboardShmSample power_sample{};
    power_sample.sequence_num = realtime_onboard_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    power_sample.host_timestamp_us = sample.mono_ns > 0 ? static_cast<uint64_t>(sample.mono_ns / 1000) : 0;
    power_sample.unix_timestamp_us = sample.unix_ns > 0 ? static_cast<uint64_t>(sample.unix_ns / 1000) : 0;
    power_sample.power_w = sample.total_mw / 1000.0;
    power_sample.gpu_freq_hz = sample.gpu_freq_hz > 0 ? static_cast<uint64_t>(sample.gpu_freq_hz) : 0;
    power_sample.cpu_cluster0_freq_hz = sample.cpu_cluster0_freq_hz > 0 ? static_cast<uint64_t>(sample.cpu_cluster0_freq_hz) : 0;
    power_sample.cpu_cluster1_freq_hz = sample.cpu_cluster1_freq_hz > 0 ? static_cast<uint64_t>(sample.cpu_cluster1_freq_hz) : 0;
    power_sample.emc_freq_hz = sample.emc_freq_hz > 0 ? static_cast<uint64_t>(sample.emc_freq_hz) : 0;
    power_sample.cpu_temp_c = sample.temp_cpu_mc > 0 ? (sample.temp_cpu_mc / 1000.0) : 0.0;
    power_sample.gpu_temp_c = sample.temp_gpu_mc > 0 ? (sample.temp_gpu_mc / 1000.0) : 0.0;

    onboard_ring_buffer_.push(power_sample);
}

int PowerMonitorSession::run_tui_loop() {
#ifndef _WIN32
    if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
        append_log("Interactive mode requires a TTY");
        std::cerr << "Interactive mode requires a TTY. Run without -i for headless capture." << std::endl;
        return 1;
    }
#endif

    auto sum_counts = [](const std::atomic<uint64_t> counts[256]) {
        uint64_t total = 0;
        for (int i = 0; i < 256; ++i) {
            total += counts[i].load(std::memory_order_relaxed);
        }
        return total;
    };

    ScreenInteractive screen = ScreenInteractive::TerminalOutput();
    screen.TrackMouse(false);

    auto get_logs = [this]() {
        std::lock_guard<std::mutex> lock(ui_state_.mutex);
        return ui_state_.logs;
    };
    Component log_component = Make<ScrollableLogArea>(get_logs);

    Component header = Renderer([&] {
        bool connected = false;
        bool initialized = false;
        bool streaming = false;
        bool has_latest_sample = false;
        bool has_stats_report = false;
        Session::Sample latest_sample{};
        uint16_t stats_report_seq = 0;
        uint32_t stats_samples_produced = 0;
        uint32_t stats_samples_dropped = 0;
        uint32_t stats_dropped_cnvrf_not_ready = 0;
        uint32_t stats_dropped_duplicate_suppressed = 0;
        uint32_t stats_dropped_worker_missed_tick = 0;
        uint32_t stats_dropped_queue_full = 0;
        uint16_t stats_queue_depth = 0;
        uint8_t stats_reason_bits = 0;
        uint16_t stats_window_ms = 0;
        {
            std::lock_guard<std::mutex> lock(ui_state_.mutex);
            connected = ui_state_.connected;
            initialized = ui_state_.initialized;
            streaming = ui_state_.streaming;
            has_latest_sample = ui_state_.has_latest_sample;
            has_stats_report = ui_state_.has_stats_report;
            latest_sample = ui_state_.latest_sample;
            stats_report_seq = ui_state_.stats_report_seq;
            stats_samples_produced = ui_state_.stats_samples_produced;
            stats_samples_dropped = ui_state_.stats_samples_dropped;
            stats_dropped_cnvrf_not_ready = ui_state_.stats_dropped_cnvrf_not_ready;
            stats_dropped_duplicate_suppressed = ui_state_.stats_dropped_duplicate_suppressed;
            stats_dropped_worker_missed_tick = ui_state_.stats_dropped_worker_missed_tick;
            stats_dropped_queue_full = ui_state_.stats_dropped_queue_full;
            stats_queue_depth = ui_state_.stats_queue_depth;
            stats_reason_bits = ui_state_.stats_reason_bits;
            stats_window_ms = ui_state_.stats_window_ms;
        }

        const uint64_t samples = sample_counter_.load(std::memory_order_relaxed);
        const uint64_t crc_fail = stats_->crc_fail.load(std::memory_order_relaxed);
        const uint64_t queue_overflow = stats_->queue_overflow.load(std::memory_order_relaxed);
        const uint64_t timeouts = stats_->timeouts.load(std::memory_order_relaxed);
        const uint64_t io_errors = stats_->io_errors.load(std::memory_order_relaxed);
        const uint64_t tx_total = sum_counts(stats_->tx_counts);
        const uint64_t rx_total = sum_counts(stats_->rx_counts);
        const uint64_t ema_interval_us = stats_->ema_interval_us.load(std::memory_order_relaxed);
        const uint64_t min_interval_us = stats_->min_interval_us.load(std::memory_order_relaxed);
        const uint64_t max_interval_us = stats_->max_interval_us.load(std::memory_order_relaxed);

        auto format_us_as_ms = [](uint64_t us) -> std::string {
            if (us == 0 || us == UINT64_MAX) return "—";
            return std::to_string(us / 1000) + "." +
                   std::to_string(((us % 1000) / 10) / 10) +
                   std::to_string(((us % 1000) / 10) % 10);
        };

        std::string latest = "No sample yet";
        if (has_latest_sample) {
            const auto cfg = session_->get_config();
            const double lsb_a = cfg.current_lsb_nA * 1e-9;
            const PicoEngineeringValues values = compute_pico_engineering_values(cfg, latest_sample);
            const double charge_c = latest_sample.charge_raw * lsb_a;
            std::ostringstream oss;
            oss << "SEQ=" << std::right << std::setw(3) << std::setfill('0')
                << static_cast<unsigned>(latest_sample.seq) << std::setfill(' ')
                << " V=" << std::fixed << std::setprecision(3) << values.vbus_v
                << " I=" << std::setprecision(4) << values.current_a
                << " P=" << std::internal << std::setfill('0') << std::setw(7) << std::setprecision(4) << values.power_w << " W" << std::setfill(' ')
                << " E=" << std::internal << std::setfill('0') << std::setw(10) << std::setprecision(4) << values.energy_j << " J" << std::setfill(' ')
                << " C= " << std::internal << std::setfill('0') << std::setw(9) << std::setprecision(4) << charge_c << std::setfill(' ')
                << " T=" << std::left << std::setprecision(1) << values.temp_c;
            latest = oss.str();
        }

        const std::string state_line =
            std::string("connected=") + (connected ? "yes" : "no") +
            " initialized=" + (initialized ? "yes" : "no") +
            " streaming=" + (streaming ? "yes" : "no");

        std::string stats_line = "stats_report=(none)";
        if (has_stats_report) {
            std::ostringstream oss;
            oss << "stats_report: seq=" << stats_report_seq
                << " produced=" << stats_samples_produced
                << " dropped=" << stats_samples_dropped
                << " cnvrf=" << stats_dropped_cnvrf_not_ready
                << " dup=" << stats_dropped_duplicate_suppressed
                << " missed=" << stats_dropped_worker_missed_tick
                << " qfull=" << stats_dropped_queue_full
                << " q_depth=" << stats_queue_depth
                << " reason=0x" << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned>(stats_reason_bits)
                << std::dec << std::setfill(' ')
                << " window_ms=" << stats_window_ms;
            stats_line = oss.str();
        }

        return vbox({
                   text("Host PC Client - FTXUI") | bold,
                   separator(),
                   text(state_line),
                   text(stats_line),
                   text("samples=" + std::to_string(samples) + " rx=" + std::to_string(rx_total) +
                        " tx=" + std::to_string(tx_total) + " crc_fail=" + std::to_string(crc_fail) +
                        " timeout=" + std::to_string(timeouts) + " io_err=" + std::to_string(io_errors) +
                        " q_overflow=" + std::to_string(queue_overflow)),
                   text("USB latency  avg: " + (ema_interval_us > 0 ? format_us_as_ms(ema_interval_us) : "—") + " ms  "
                        "min: " + (min_interval_us != UINT64_MAX ? std::to_string(min_interval_us) : "—") + " us  "
                        "max: " + format_us_as_ms(max_interval_us) + " ms"),
                   text([&] {
                       if (!has_latest_sample) {
                           return std::string("Last sample age (now - device_ts):      — ms");
                       }
                       const uint64_t now_us = now_unix_us();
                       const uint64_t dev_us = latest_sample.device_timestamp_unix_us;
                       const int64_t delta_us = static_cast<int64_t>(now_us - dev_us);
                       const double delta_ms = static_cast<double>(delta_us) / 1000.0;
                       std::ostringstream oss;
                       oss << "Last sample age (now - device_ts): "
                           << std::fixed << std::setprecision(3) << std::setw(10) << std::right
                           << delta_ms << " ms";
                       return oss.str();
                   }()),
                   text(latest),
                   separator(),
                   text("Keys: [t]=toggle stream  [s]=save snapshot  [q]=quit  Tab=focus logs, PgUp/PgDn=scroll") | dim,
                   separator(),
                   text("Logs (newest first):") | bold,
               });
    });

    Component body = Container::Vertical({header, log_component});
    Component renderer = Renderer(body, [body] { return body->Render() | border; });

    renderer = CatchEvent(renderer, [&](Event event) {
        if (event == Event::Character('q') || event == Event::Character('Q') || event == Event::CtrlC) {
            stop_requested_.store(true);
            return true;
        }
        if (event == Event::Character('s') || event == Event::Character('S')) {
            save_snapshot(options_.output_file);
            return true;
        }
        if (event == Event::Character('t') || event == Event::Character('T')) {
            if (streaming_.load()) {
                stop_streaming();
            } else {
                start_streaming();
            }
            return true;
        }
        return false;
    });

    Loop loop(&screen, renderer);
    while (!stop_requested_.load() && !loop.HasQuitted()) {
        if (g_signal_interrupted.load()) {
            stop_requested_.store(true);
            break;
        }

        protocol::DynamicFrame async_frame;
        while (response_queue_->pop_by_msgid(async_frame, static_cast<uint8_t>(protocol::MsgId::kStatsReport))) {
            process_async_control_frame(async_frame);
        }

        constexpr int kMaxControlFramesPerTick = 16;
        for (int i = 0; i < kMaxControlFramesPerTick && response_queue_->pop_wait(async_frame, 0); ++i) {
            if (async_frame.type == protocol::FrameType::kEvt &&
                async_frame.msgid == static_cast<uint8_t>(protocol::MsgId::kTimeSyncRequest) && streaming_.load()) {
                on_time_sync_request();
            } else {
                process_async_control_frame(async_frame);
            }
        }

        screen.RequestAnimationFrame();
        loop.RunOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

bool PowerMonitorSession::save_snapshot(const std::string& path) {
    if (path.empty()) {
        append_log("Save skipped: output path is empty");
        return false;
    }
    try {
        session_->save_merged(path);
        append_log("Saved snapshot to " + path);
        return true;
    } catch (const std::exception& e) {
        append_log(std::string("Save failed: ") + e.what());
        return false;
    }
}

void PowerMonitorSession::append_log(const std::string& message) {
    std::lock_guard<std::mutex> lock(ui_state_.mutex);
    ui_state_.logs.push_back(message);
    constexpr size_t kMaxLogs = 500;
    while (ui_state_.logs.size() > kMaxLogs) {
        ui_state_.logs.pop_front();
    }
}

void PowerMonitorSession::emit_time_sync_debug(const std::string& message) {
    if (!options_.debug_time_sync) {
        return;
    }
    append_log("[TIME_DEBUG] " + message);
    if (!options_.interactive) {
        std::cerr << "[TIME_DEBUG] " << message << std::endl;
    }
}

void PowerMonitorSession::maybe_debug_time_sync_sample(const SampleQueue::Sample& sample) {
    if (!options_.debug_time_sync) {
        return;
    }
    int remaining = debug_time_sync_samples_remaining_.load(std::memory_order_relaxed);
    while (remaining > 0) {
        if (debug_time_sync_samples_remaining_.compare_exchange_weak(
                remaining, remaining - 1, std::memory_order_relaxed, std::memory_order_relaxed)) {
            const int64_t delta_unix_us = signed_diff_u64(sample.device_timestamp_unix_us, sample.host_timestamp_unix_us);
            std::ostringstream oss;
            oss << "sample seq=" << sample.seq
                << " host_unix_us=" << sample.host_timestamp_unix_us
                << " device_unix_us=" << sample.device_timestamp_unix_us
                << " delta_us(device-host)=" << delta_unix_us
                << " host_steady_us=" << sample.host_timestamp_us
                << " device_rel_us=" << sample.device_timestamp_us;
            emit_time_sync_debug(oss.str());
            break;
        }
    }
}

void PowerMonitorSession::save_and_exit() {
    Session::RuntimeMeta runtime_meta;
    runtime_meta.vid_hex = options_.vid_hex;
    runtime_meta.pid_hex = options_.pid_hex;
    runtime_meta.port = options_.port;
    runtime_meta.baud = static_cast<uint32_t>(options_.baud_rate);
    runtime_meta.run_label = options_.run_label;
    runtime_meta.tags = options_.run_tags;
    session_->set_runtime_meta(runtime_meta);

    Session::Stats session_stats;
    for (size_t i = 0; i < session_stats.rx_counts.size(); ++i) {
        session_stats.rx_counts[i] = stats_->rx_counts[i].load(std::memory_order_relaxed);
        session_stats.tx_counts[i] = stats_->tx_counts[i].load(std::memory_order_relaxed);
    }
    session_stats.crc_fail = stats_->crc_fail.load(std::memory_order_relaxed);
    session_stats.queue_overflow = stats_->queue_overflow.load(std::memory_order_relaxed);
    session_stats.timeouts = stats_->timeouts.load(std::memory_order_relaxed);
    session_stats.retries = stats_->retries.load(std::memory_order_relaxed);
    session_stats.io_errors = stats_->io_errors.load(std::memory_order_relaxed);
    session_->set_stats(session_stats);

    session_->set_session_timing(session_start_unix_us_, session_end_unix_us_);

    if (!options_.output_file.empty()) {
        std::cout << "Saving data to " << options_.output_file << std::endl;
        try {
            if (options_.onboard_enabled && onboard_sampler_) {
                session_->save_bundle_merged(options_.output_file);
            } else {
                session_->save_merged(options_.output_file);
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to save data: " << e.what() << std::endl;
        }
    }

    if (options_.verbose) {
        print_statistics();
    }
}

void PowerMonitorSession::print_statistics(bool inline_mode) const {
    auto sum_counts = [](const std::atomic<uint64_t> counts[256]) {
        uint64_t total = 0;
        for (int i = 0; i < 256; ++i) {
            total += counts[i].load(std::memory_order_relaxed);
        }
        return total;
    };

    const uint64_t samples = sample_counter_.load(std::memory_order_relaxed);
    const uint64_t crc_fail = stats_->crc_fail.load(std::memory_order_relaxed);
    const uint64_t queue_overflow = stats_->queue_overflow.load(std::memory_order_relaxed);
    const uint64_t timeouts = stats_->timeouts.load(std::memory_order_relaxed);
    const uint64_t io_errors = stats_->io_errors.load(std::memory_order_relaxed);
    const uint64_t tx_total = sum_counts(stats_->tx_counts);
    const uint64_t rx_total = sum_counts(stats_->rx_counts);

    if (inline_mode) {
        const uint64_t ema_us = stats_->ema_interval_us.load(std::memory_order_relaxed);
        const uint64_t min_us = stats_->min_interval_us.load(std::memory_order_relaxed);
        const uint64_t max_us = stats_->max_interval_us.load(std::memory_order_relaxed);
        std::ostringstream oss;
        oss << "Samples:" << std::setw(10) << samples
            << "  CRC_FAIL:" << std::setw(6) << crc_fail
            << "  RX:" << std::setw(8) << rx_total
            << "  TX:" << std::setw(8) << tx_total
            << "  Timeouts:" << std::setw(5) << timeouts
            << "  IO_Errors:" << std::setw(4) << io_errors
            << "  QueueOverflow:" << std::setw(4) << queue_overflow;
        if (ema_us > 0 || min_us != UINT64_MAX || max_us > 0) {
            oss << "  USB_ms:";
            if (ema_us > 0) oss << " avg=" << std::fixed << std::setprecision(2) << (static_cast<double>(ema_us) / 1000.0);
            if (min_us != UINT64_MAX) oss << " min=" << min_us << "us";
            if (max_us > 0) oss << " max=" << std::fixed << std::setprecision(2) << (static_cast<double>(max_us) / 1000.0);
        }
        std::string line = oss.str();
        if (line.size() < last_stats_width_) {
            line.append(last_stats_width_ - line.size(), ' ');
        } else {
            last_stats_width_ = line.size();
        }
        std::cout << "\r" << line << std::flush;
        return;
    }

    std::cout << "\n=== Session Statistics ===" << std::endl;
    std::cout << "Samples collected: " << samples << std::endl;
    std::cout << "CRC failures: " << crc_fail << std::endl;
    std::cout << "Queue overflows: " << queue_overflow << std::endl;
    std::cout << "Timeouts: " << timeouts << std::endl;
    std::cout << "IO errors: " << io_errors << std::endl;

    std::cout << "\nTX counts:" << std::endl;
    for (int i = 0; i < 256; ++i) {
        uint64_t count = stats_->tx_counts[i].load(std::memory_order_relaxed);
        if (count > 0) {
            std::cout << "  MSG 0x" << std::hex << std::setw(2) << std::setfill('0') << i
                      << std::dec << ": " << count << std::endl;
        }
    }

    std::cout << "\nRX counts:" << std::endl;
    for (int i = 0; i < 256; ++i) {
        uint64_t count = stats_->rx_counts[i].load(std::memory_order_relaxed);
        if (count > 0) {
            std::cout << "  MSG 0x" << std::hex << std::setw(2) << std::setfill('0') << i
                      << std::dec << ": " << count << std::endl;
        }
    }
}


}  // namespace client
}  // namespace powermonitor
