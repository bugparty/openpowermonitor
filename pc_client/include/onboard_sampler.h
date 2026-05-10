#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "session.h"  // For OnboardSample

// Forward declaration (OnboardSampleQueue defined in onboard_sample_queue.h)
class OnboardSampleQueue;

namespace powermonitor {
namespace client {

// Forward declaration
class OnboardSampleQueue;

/**
 * @brief Hardware monitor sampler for Jetson Nano onboard power measurement
 *
 * Architecture: two threads share one OnboardSample via a mutex.
 *
 * Thread 1 - INA thread (high-rate):
 *   Reads INA3221 in1-3 (V×I = power) at the configured period (default 1 kHz).
 *   Acquires shared lock, fills power fields, releases, pushes to queue.
 *
 * Thread 2 - Telemetry thread (configurable, default 100 Hz):
 *   Reads GPU/CPU/EMC freq, thermal zones, fan RPM.
 *   Acquires shared lock, fills telemetry fields (INA fields stay stale).
 *
 * Thread model:
 *   - Uses a std::mutex to protect the shared OnboardSample between the two threads.
 *   - Uses absolute timing (clock_nanosleep) to maintain precise sampling rate.
 *
 * Configuration:
 *   - hwmon_path: Path to hwmon directory (default: /sys/class/hwmon/hwmon1)
 *   - period_us: INA thread sampling period in microseconds (default: 1000 = 1 kHz)
 *   - telemetry_period_us: Telemetry thread period in microseconds (default: 10000 = 100 Hz)
 *   - cpu_core: CPU core affinity for INA thread (-1 = no affinity)
 *   - rt_prio: Real-time priority for INA thread (-1 = disabled)
 *   - jetson_freq_path: Path to /proc/jetson_freqs (requires kernel module)
 */
class OnboardSampler {
public:
    struct Config {
        std::string hwmon_path = "/sys/class/hwmon/hwmon1";
        uint64_t period_us = 1000;          // INA thread period in microseconds (default: 1 kHz)
        uint64_t telemetry_period_us = 10000; // Telemetry thread period in microseconds (default: 100 Hz)
        int cpu_core = -1;          // CPU core affinity for INA thread (-1 = disabled)
        int rt_prio = -1;           // RT priority for INA thread (-1 = disabled)

        // Path to jetson_freq_reader kernel module procfs output
        std::string jetson_freq_path = "/proc/jetson_freqs";
    };

    /**
     * @brief Construct onboard sampler
     * @param cfg Configuration parameters
     * @param queue Shared pointer to sample queue (must not be null)
     */
    OnboardSampler(const Config& cfg, std::shared_ptr<OnboardSampleQueue> queue);

    /**
     * @brief Destructor - stops and joins both threads
     */
    ~OnboardSampler();

    // Non-copyable
    OnboardSampler(const OnboardSampler&) = delete;
    OnboardSampler& operator=(const OnboardSampler&) = delete;

    /**
     * @brief Start both sampler threads
     * @return true if threads started successfully, false on error
     */
    bool start();

    /**
     * @brief Signal both threads to stop
     */
    void stop();

    /**
     * @brief Wait for both threads to exit
     */
    void join();

    /**
     * @brief Check if INA thread is running
     */
    bool is_running() const;

    /**
     * @brief Get last error message
     */
    const std::string& get_last_error() const { return last_error_; }

private:
    /**
     * @brief INA thread: reads INA3221 power rails at 1 kHz
     */
    void ina_loop();

    /**
     * @brief Telemetry thread: reads GPU/CPU/EMC freq + temps + fan at configurable rate (default 100 Hz)
     */
    void telemetry_loop();

    /**
     * @brief Apply CPU affinity and RT scheduling to the calling thread
     */
    bool apply_thread_affinity();

    Config config_;
    std::shared_ptr<OnboardSampleQueue> queue_;

    // Shared state between the two threads
    OnboardSample shared_sample_{};
    std::mutex shared_mutex_;

    std::unique_ptr<std::thread> ina_thread_;
    std::unique_ptr<std::thread> telemetry_thread_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::string last_error_;
};

}  // namespace client
}  // namespace powermonitor
