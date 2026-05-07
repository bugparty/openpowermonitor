#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "protocol/frame_builder.h"
#include "sample_queue.h"
#include "session.h"
#include "shm_power_ring_buffer.h"

namespace serial {
class Serial;
}

namespace powermonitor {
namespace client {

class ResponseQueue;
class ReadThread;
class Session;
class OnboardSampler;
class OnboardSampleQueue;
struct ThreadStats;

class PowerMonitorSession {
public:
    struct Options {
        std::string port;
        int baud_rate = 115200;
        std::string output_file;
        uint16_t stream_period_us = 1000;
        uint16_t stream_mask = 0x000F;
        bool usb_stress_mode = false;
        bool verbose = false;
        bool interactive = false;
        bool debug_time_sync = false;
        bool no_apply_time_offset = false;  // If true, do not send TIME_ADJUST after sync
        uint32_t duration_s = 0;
        uint64_t duration_us = 0;
        std::string run_label;
        std::vector<std::string> run_tags;
        std::string vid_hex = "0x0000";
        std::string pid_hex = "0x0000";
        int read_thread_core = -1;
        int proc_thread_core = -1;
        int rt_prio = -1;

        // Onboard sampler options
        bool onboard_enabled = false;
        std::string onboard_hwmon_path = "/sys/class/hwmon/hwmon1";
        uint64_t onboard_period_us = 1000;
        int onboard_cpu_core = -1;
        int onboard_rt_prio = -1;

        // Path to jetson_freq_reader kernel module procfs output
        std::string onboard_jetson_freq_path = "/proc/jetson_freqs";

        // Flush interval for periodic data dump to chunks
        uint32_t flush_interval_s = 5;
    };

    explicit PowerMonitorSession(const Options& options);
    ~PowerMonitorSession();

    PowerMonitorSession(const PowerMonitorSession&) = delete;
    PowerMonitorSession& operator=(const PowerMonitorSession&) = delete;

    int run();
    void stop();

private:
    bool connect_device();
    bool initialize_device();
    bool send_ping();
    bool get_device_config();
    bool perform_time_sync_once(std::string* detail = nullptr, int64_t* out_offset = nullptr,
                                bool send_adjust = true);
    bool run_time_sync_rounds(int rounds);
    bool start_streaming();
    bool stop_streaming();

    bool wait_for_response(uint8_t expected_seq, protocol::MsgId expected_msgid,
                           protocol::DynamicFrame* frame, int timeout_ms);
    bool wait_for_message_by_id(protocol::MsgId expected_msgid, protocol::DynamicFrame* frame, int timeout_ms);
    void process_async_control_frame(const protocol::DynamicFrame& frame);
    void on_time_sync_request();  // Single entry for EVT_TIME_SYNC_REQUEST: run periodic sync when streaming
    bool send_command_with_retry(protocol::MsgId msgid, const std::vector<uint8_t>& payload,
                                 std::vector<uint8_t>* rsp_data = nullptr,
                                 bool try_lock_command = false);

    void process_samples_loop();
    void process_onboard_loop();
    void flush_loop();
    int run_tui_loop();
    bool save_snapshot(const std::string& path);
    void append_log(const std::string& message);
    void print_statistics(bool inline_mode = false) const;
    void save_and_exit();
    void emit_time_sync_debug(const std::string& message);
    void maybe_debug_time_sync_sample(const SampleQueue::Sample& sample);
    void export_pico_power_sample(const Session::Sample& sample);
    void export_onboard_power_sample(const OnboardSample& sample);

    struct UiState {
        mutable std::mutex mutex;
        std::deque<std::string> logs;
        Session::Sample latest_sample{};
        bool has_latest_sample = false;
        bool has_stats_report = false;
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
        bool connected = false;
        bool initialized = false;
        bool streaming = false;
    };

     Options options_;
     std::unique_ptr<serial::Serial> serial_;
     std::unique_ptr<SampleQueue> sample_queue_;
     std::unique_ptr<ResponseQueue> response_queue_;
     std::unique_ptr<ReadThread> read_thread_;
     std::unique_ptr<Session> session_;
     std::unique_ptr<ThreadStats> stats_;

     // Onboard sampler
     std::shared_ptr<OnboardSampleQueue> onboard_queue_;
     std::unique_ptr<OnboardSampler> onboard_sampler_;
     std::thread onboard_proc_thread_;

     std::thread flush_thread_;

     std::atomic<bool> stop_requested_{false};
     std::atomic<bool> interrupted_{false};
     std::atomic<bool> streaming_{false};
     std::atomic<bool> save_requested_{false};
     std::atomic<uint64_t> sample_counter_{0};
     std::atomic<uint64_t> realtime_pico_sequence_{0};
     std::atomic<uint64_t> realtime_onboard_sequence_{0};
     uint64_t session_start_unix_us_ = 0;
     uint64_t session_end_unix_us_ = 0;
     uint8_t cmd_seq_ = 0;
     uint64_t sync_start_time_us_ = 0;  // Set when first TIME_SET succeeds
     bool time_sync_succeeded_ = false;  // Tracks if TIME_SYNC completed successfully
     std::atomic<int> debug_time_sync_samples_remaining_{0};
     std::mutex command_mutex_;
     mutable size_t last_stats_width_ = 0;
     UiState ui_state_;
     ShmPicoRingBuffer pico_ring_buffer_;
     ShmOnboardRingBuffer onboard_ring_buffer_;
  };


}  // namespace client
}  // namespace powermonitor
