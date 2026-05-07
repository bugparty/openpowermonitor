#pragma once

#include <cstddef>
#include <cstdint>

namespace powermonitor {
namespace client {

// Pico INA228 external current sensor SHM
constexpr const char* PICO_METRICS_SHM_NAME = "/powermonitor_pico_metrics";
constexpr uint32_t kPicoMetricsVersion = 4;
constexpr size_t kPicoMetricsRingCapacity = 4096;

// Jetson hwmon onboard sampler SHM
constexpr const char* ONBOARD_METRICS_SHM_NAME = "/powermonitor_onboard_metrics";
constexpr uint32_t kOnboardMetricsVersion = 4;
constexpr size_t kOnboardMetricsRingCapacity = 256;

struct PicoSample {
    uint64_t sequence_num;
    uint64_t host_timestamp_us;    // Host monotonic clock (μs)
    uint64_t unix_timestamp_us;    // Unix time (μs)
    uint64_t device_timestamp_us;  // Pico device clock (μs)
    uint32_t flags;
    uint32_t _pad;
    double   power_w;              // Instantaneous power (W)
    double   voltage_v;            // Bus voltage (V)
    double   current_a;            // Current (A)
    double   ina228_temp_c;        // INA228 chip temperature (°C)
    double   energy_j;             // Cumulative energy (J)
};

struct OnboardShmSample {
    uint64_t sequence_num;
    uint64_t host_timestamp_us;         // Host monotonic clock (μs)
    uint64_t unix_timestamp_us;         // Unix time (μs)
    double   power_w;                   // Total power from INA3221 rails (W)
    double   cpu_temp_c;                // CPU temperature (°C)
    double   gpu_temp_c;                // GPU temperature (°C)
    uint64_t gpu_freq_hz;
    uint64_t cpu_cluster0_freq_hz;
    uint64_t cpu_cluster1_freq_hz;
    uint64_t emc_freq_hz;
};

}  // namespace client
}  // namespace powermonitor
