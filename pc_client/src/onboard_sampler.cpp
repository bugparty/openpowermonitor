#include "onboard_sampler.h"
#include "onboard_sample_queue.h"
#include "session.h"  // For OnboardSample definition

#ifndef _WIN32

#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <optional>

namespace powermonitor {
namespace client {

namespace {

// Helper: Get current time in nanoseconds
int64_t now_ns(clockid_t cid) {
    struct timespec ts {};
    clock_gettime(cid, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// Helper: Convert nanoseconds to timespec (absolute)
timespec ns_to_ts(int64_t ns) {
    timespec ts {};
    ts.tv_sec  = ns / 1000000000LL;
    ts.tv_nsec = ns % 1000000000LL;
    if (ts.tv_nsec < 0) {
        ts.tv_nsec += 1000000000LL;
        ts.tv_sec  -= 1;
    }
    return ts;
}

}  // anonymous namespace (now_ns, ns_to_ts)

// ── Kernel module frequency reader ─────────────────────────────────────────
// Reads /proc/jetson_freqs: "cpu0_hz cpu4_hz gpu_hz emc_hz read_time_ns"
struct JetsonFreqs {
    int64_t cpu0_hz = -1;
    int64_t cpu4_hz = -1;
    int64_t gpu_hz  = -1;
    int64_t emc_hz  = -1;
};

std::optional<JetsonFreqs> read_jetson_freqs(const std::string& path) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return std::nullopt;

    char buf[128] = {};
    ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
    ::close(fd);
    if (n <= 0) return std::nullopt;

    JetsonFreqs f;
    int64_t read_time_ns = 0;
    if (sscanf(buf, "%ld %ld %ld %ld %ld",
               &f.cpu0_hz, &f.cpu4_hz, &f.gpu_hz, &f.emc_hz, &read_time_ns) != 5) {
        return std::nullopt;
    }
    return f;
}

// Read integer from sysfs via plain open/read (no sudo subprocess).
// Thread-safe; used for temperature and fan readings.
std::optional<int64_t> sysfs_read(const std::string& path) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return std::nullopt;
    char buf[64] = {};
    ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
    ::close(fd);
    if (n <= 0) return std::nullopt;
    buf[n] = '\0';
    try {
        return std::stoll(std::string(buf));
    } catch (...) {
        return std::nullopt;
    }
}

OnboardSampler::OnboardSampler(const Config& cfg,
                               std::shared_ptr<OnboardSampleQueue> queue)
    : config_(cfg), queue_(queue) {
    if (!queue_) {
        last_error_ = "OnboardSampleQueue pointer is null";
    }
}

OnboardSampler::~OnboardSampler() {
    stop();
    join();
}

bool OnboardSampler::start() {
    if (running_.load()) {
        last_error_ = "Sampler already running";
        return false;
    }
    if (!queue_) {
        last_error_ = "OnboardSampleQueue pointer is null";
        return false;
    }
    if (config_.period_us == 0) {
        last_error_ = "Invalid period_us: must be > 0";
        return false;
    }
    if (config_.telemetry_period_us == 0) {
        last_error_ = "Invalid telemetry_period_us: must be > 0";
        return false;
    }

    stop_requested_ = false;

    // Initialize shared_sample_ with first telemetry read to avoid -1 stale data
    {
        auto freqs = read_jetson_freqs(config_.jetson_freq_path);
        if (!freqs) {
            last_error_ = "Cannot read " + config_.jetson_freq_path +
                          " — is jetson_freq_reader kernel module loaded? (sudo insmod jetson_freq_reader.ko)";
            return false;
        }

        std::lock_guard<std::mutex> lock(shared_mutex_);
        shared_sample_.gpu_freq_hz         = freqs->gpu_hz;
        shared_sample_.cpu_cluster0_freq_hz = freqs->cpu0_hz;
        shared_sample_.cpu_cluster1_freq_hz = freqs->cpu4_hz;
        shared_sample_.emc_freq_hz          = freqs->emc_hz;
        shared_sample_.temp_cpu_mc = sysfs_read("/sys/class/thermal/thermal_zone0/temp").value_or(-1);
        shared_sample_.temp_gpu_mc = sysfs_read("/sys/class/thermal/thermal_zone1/temp").value_or(-1);
        shared_sample_.temp_soc0_mc = sysfs_read("/sys/class/thermal/thermal_zone5/temp").value_or(-1);
        shared_sample_.temp_soc1_mc = sysfs_read("/sys/class/thermal/thermal_zone6/temp").value_or(-1);
        shared_sample_.temp_soc2_mc = sysfs_read("/sys/class/thermal/thermal_zone7/temp").value_or(-1);
        shared_sample_.temp_tj_mc = sysfs_read("/sys/class/thermal/thermal_zone8/temp").value_or(-1);
        shared_sample_.fan_rpm = sysfs_read("/sys/class/hwmon/hwmon2/rpm").value_or(-1);
    }

    try {
        ina_thread_ = std::make_unique<std::thread>(&OnboardSampler::ina_loop, this);
        telemetry_thread_ = std::make_unique<std::thread>(&OnboardSampler::telemetry_loop, this);
        running_ = true;
        return true;
    } catch (const std::exception& e) {
        // Clean up any started threads
        stop_requested_ = true;
        if (ina_thread_ && ina_thread_->joinable()) ina_thread_->join();
        if (telemetry_thread_ && telemetry_thread_->joinable()) telemetry_thread_->join();
        ina_thread_.reset();
        telemetry_thread_.reset();
        last_error_ = std::string("Failed to start thread: ") + e.what();
        return false;
    }
}

void OnboardSampler::stop() {
    stop_requested_ = true;
}

void OnboardSampler::join() {
    if (ina_thread_ && ina_thread_->joinable())      ina_thread_->join();
    if (telemetry_thread_ && telemetry_thread_->joinable()) telemetry_thread_->join();
    running_ = false;
}

bool OnboardSampler::is_running() const {
    return running_.load();
}

// ─────────────────────────────────────────────────────────────────────────────
// INA thread: reads INA3221 in1-3 (V×I = power) at 1 kHz.
// Holds shared_mutex_ only while updating shared_sample_; the actual I/O is
// done unlocked so it can run in parallel with the telemetry thread.
// ─────────────────────────────────────────────────────────────────────────────
void OnboardSampler::ina_loop() {
    apply_thread_affinity();  // RT priority + CPU affinity on this thread

    const std::string hw = config_.hwmon_path;
    // Core INA3221 power rails (VDD_IN, VDD_CPU_GPU_CV, VDD_SOC)
    const std::string in1 = hw + "/in1_input";
    const std::string i1  = hw + "/curr1_input";
    const std::string in2 = hw + "/in2_input";
    const std::string i2  = hw + "/curr2_input";
    const std::string in3 = hw + "/in3_input";
    const std::string i3  = hw + "/curr3_input";

    const int64_t period_ns = static_cast<int64_t>(config_.period_us) * 1000LL;
    int64_t next_tick = now_ns(CLOCK_MONOTONIC);

    while (!stop_requested_.load()) {
        // ── Read INA3221 power rails (outside the lock — ~3.6 ms) ────────
        const auto v1 = sysfs_read(in1);
        const auto c1 = sysfs_read(i1);
        const auto v2 = sysfs_read(in2);
        const auto c2 = sysfs_read(i2);
        const auto v3 = sysfs_read(in3);
        const auto c3 = sysfs_read(i3);

        // ── Build partial sample and snapshot shared telemetry ─────────────
        OnboardSample s;
        s.mono_ns = now_ns(CLOCK_MONOTONIC);
        s.unix_ns = now_ns(CLOCK_REALTIME);

        if (v1 && c1) s.vdd_in_mw          = (*v1) * (*c1) / 1000LL;
        if (v2 && c2) s.vdd_cpu_gpu_cv_mw  = (*v2) * (*c2) / 1000LL;
        if (v3 && c3) s.vdd_soc_mw         = (*v3) * (*c3) / 1000LL;
        s.total_mw = s.vdd_in_mw + s.vdd_cpu_gpu_cv_mw + s.vdd_soc_mw;

        // Snapshot telemetry (written by telemetry thread at configurable rate, default 100 Hz)
        {
            std::lock_guard<std::mutex> lock(shared_mutex_);
            s.gpu_freq_hz        = shared_sample_.gpu_freq_hz;
            s.cpu_cluster0_freq_hz = shared_sample_.cpu_cluster0_freq_hz;
            s.cpu_cluster1_freq_hz = shared_sample_.cpu_cluster1_freq_hz;
            s.emc_freq_hz         = shared_sample_.emc_freq_hz;
            s.temp_cpu_mc         = shared_sample_.temp_cpu_mc;
            s.temp_gpu_mc         = shared_sample_.temp_gpu_mc;
            s.temp_soc0_mc        = shared_sample_.temp_soc0_mc;
            s.temp_soc1_mc        = shared_sample_.temp_soc1_mc;
            s.temp_soc2_mc        = shared_sample_.temp_soc2_mc;
            s.temp_tj_mc          = shared_sample_.temp_tj_mc;
            s.fan_rpm             = shared_sample_.fan_rpm;
        }

        // Push to queue (INA data at 1 kHz, telemetry at configurable rate)
        queue_->push(s);

        // Precise sleep until next tick
        next_tick += period_ns;
        const timespec ts = ns_to_ts(next_tick);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry thread: reads GPU/CPU/EMC freq + thermal zones + fan (default 100 Hz).
// No RT priority: telemetry is less latency-critical than the INA thread (1 kHz).
// Holds shared_mutex_ only while writing shared_sample_; the I/O (~0.9 ms)
// is done unlocked so it can overlap with the INA thread's idle/sleep time.
// ─────────────────────────────────────────────────────────────────────────────
void OnboardSampler::telemetry_loop() {
    const int64_t period_ns = static_cast<int64_t>(config_.telemetry_period_us) * 1000LL;
    int64_t next_tick = now_ns(CLOCK_MONOTONIC);

    while (!stop_requested_.load()) {
        // ── Frequency read via kernel module (~1.2 µs) ────────────────────
        auto freqs = read_jetson_freqs(config_.jetson_freq_path);

        // ── Thermal + fan still via sysfs ────────────────────────────────
        int64_t t_cpu  = sysfs_read("/sys/class/thermal/thermal_zone0/temp").value_or(-1);
        int64_t t_gpu  = sysfs_read("/sys/class/thermal/thermal_zone1/temp").value_or(-1);
        int64_t t_soc0 = sysfs_read("/sys/class/thermal/thermal_zone5/temp").value_or(-1);
        int64_t t_soc1 = sysfs_read("/sys/class/thermal/thermal_zone6/temp").value_or(-1);
        int64_t t_soc2 = sysfs_read("/sys/class/thermal/thermal_zone7/temp").value_or(-1);
        int64_t t_tj   = sysfs_read("/sys/class/thermal/thermal_zone8/temp").value_or(-1);
        int64_t fan    = sysfs_read("/sys/class/hwmon/hwmon2/rpm").value_or(-1);

        // ── Write into shared sample under lock (very brief) ─────────────
        {
            std::lock_guard<std::mutex> lock(shared_mutex_);
            if (freqs) {
                shared_sample_.gpu_freq_hz         = freqs->gpu_hz;
                shared_sample_.cpu_cluster0_freq_hz = freqs->cpu0_hz;
                shared_sample_.cpu_cluster1_freq_hz = freqs->cpu4_hz;
                shared_sample_.emc_freq_hz          = freqs->emc_hz;
            }
            shared_sample_.temp_cpu_mc          = t_cpu;
            shared_sample_.temp_gpu_mc          = t_gpu;
            shared_sample_.temp_soc0_mc         = t_soc0;
            shared_sample_.temp_soc1_mc         = t_soc1;
            shared_sample_.temp_soc2_mc         = t_soc2;
            shared_sample_.temp_tj_mc           = t_tj;
            shared_sample_.fan_rpm              = fan;
        }

        // ── Sleep until next telemetry tick ────────────────────────────────
        next_tick += period_ns;
        const timespec ts = ns_to_ts(next_tick);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
bool OnboardSampler::apply_thread_affinity() {
    if (config_.cpu_core >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(static_cast<unsigned>(config_.cpu_core), &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) {
            last_error_ = std::string("sched_setaffinity failed: ") + strerror(errno);
        }
    }
    if (config_.rt_prio >= 0) {
        struct sched_param sp {};
        sp.sched_priority = config_.rt_prio;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            last_error_ = std::string("sched_setscheduler(SCHED_FIFO) failed: ") + strerror(errno);
        }
    }
    return true;
}

}  // namespace client
}  // namespace powermonitor

#else  // _WIN32 — stub implementations (OnboardSampler is Jetson Linux only)

namespace powermonitor {
namespace client {

OnboardSampler::OnboardSampler(const Config& cfg,
                               std::shared_ptr<OnboardSampleQueue> queue)
    : config_(cfg), queue_(queue) {
    last_error_ = "OnboardSampler is not supported on Windows";
}

OnboardSampler::~OnboardSampler() {}

bool OnboardSampler::start() {
    last_error_ = "OnboardSampler is not supported on Windows";
    return false;
}

void OnboardSampler::stop() {}
void OnboardSampler::join() {}
bool OnboardSampler::is_running() const { return false; }
void OnboardSampler::ina_loop() {}
void OnboardSampler::telemetry_loop() {}
bool OnboardSampler::apply_thread_affinity() { return false; }

}  // namespace client
}  // namespace powermonitor

#endif  // _WIN32
