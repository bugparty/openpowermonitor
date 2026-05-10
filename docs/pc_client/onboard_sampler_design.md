# Onboard Sampler Design

This document describes the architecture, performance characteristics, and configuration of the onboard hardware telemetry sampler for Jetson Orin Nano.

## Overview

The `OnboardSampler` class collects hardware telemetry from Jetson Orin Nano's onboard sensors:

- **INA3221**: 3-channel power monitor (VDD_IN, VDD_CPU_GPU_CV, VDD_SOC)
- **GPU frequency**: via devfreq sysfs
- **CPU cluster frequencies**: via cpufreq sysfs
- **EMC (memory) frequency**: via clk debugfs
- **Thermal zones**: CPU, GPU, SoC, Tj temperatures
- **Fan RPM**: via hwmon sysfs

## Architecture

### Dual-Thread Design

The sampler uses two independent threads to maximize throughput:

```
┌─────────────────────────────────┐    ┌─────────────────────────────────┐
│     INA Thread (high-rate)       │    │   Telemetry Thread (100 Hz)     │
│                                  │    │                                 │
│  ┌───────────────────────────┐  │    │  ┌───────────────────────────┐  │
│  │ read in1,c1,in2,c2,in3,c3 │  │    │  │ read GPU/CPU/EMC freq    │  │
│  │ ~3.5 ms (I2C hardware)    │  │    │  │ read 6 thermal zones     │  │
│  │ (no lock held)            │  │    │  │ read fan RPM             │  │
│  └───────────────────────────┘  │    │  │ ~0.8 ms total            │  │
│              │                   │    │  │ (no lock held)           │  │
│              ▼                   │    │  └───────────────────────────┘  │
│  ┌───────────────────────────┐  │    │              │                  │
│  │ lock mutex                │  │    │              ▼                  │
│  │ snapshot telemetry data   │  │    │  ┌───────────────────────────┐  │
│  │ unlock mutex              │  │    │  │ lock mutex               │  │
│  │ push sample to queue      │  │    │  │ write to shared_sample_  │  │
│  │ ~5 µs total               │  │    │  │ unlock mutex             │  │
│  └───────────────────────────┘  │    │  │ ~1 µs total              │  │
│              │                   │    │  └───────────────────────────┘  │
│              ▼                   │    │              │                  │
│  sleep until next tick          │    │  sleep 10 ms                    │
│  (TIMER_ABSTIME)                │    │  (TIMER_ABSTIME)                │
└─────────────────────────────────┘    └─────────────────────────────────┘
                  │                                    │
                  ▼                                    ▼
           ~270 Hz effective                    100 Hz (configurable)
           (limited by I2C)                     (default --onboard-telemetry-period-us 10000)
```

### Why Two Threads?

The INA3221 power sensor has an inherent I2C transaction latency of ~590 µs per file read. Reading 6 files (3 voltage + 3 current) takes ~3.5 ms, which already exceeds the 1 ms budget for 1 kHz sampling.

By separating the telemetry reads (GPU/CPU/EMC/temps/fan) into a parallel thread:

- INA samples at ~270 Hz (limited by I2C hardware, not software)
- Telemetry updates at 100 Hz (configurable, sufficient for frequency/temperature tracking)
- No blocking between the two paths

### Shared State

Both threads share a single `OnboardSample` instance protected by `std::mutex`:

```cpp
OnboardSample shared_sample_;  // Written by telemetry thread, read by INA thread
std::mutex shared_mutex_;       // Very brief lock times (~1-5 µs)
```

The INA thread snapshots telemetry data while holding the lock, then releases immediately. The telemetry thread writes new values under the lock, then releases.

## Performance Characteristics

### Measured Overhead (Jetson Orin Nano, sudo root)

| Source | Files | Per-File Latency | Total per Sample |
|--------|-------|------------------|------------------|
| INA3221 power rails | 6 | ~590 µs | ~3.5 ms |
| GPU freq | 1 | ~13 µs | - |
| CPU cluster0 freq | 1 | ~78 µs | - |
| CPU cluster1 freq | 1 | ~76 µs | - |
| EMC freq | 1 | ~12 µs | - |
| Thermal zones | 6 | ~117 µs | ~0.7 ms |
| Fan RPM | 1 | ~23 µs | - |
| **Telemetry total** | 11 | - | ~0.9 ms |

### Effective Sample Rates

| Thread | Target | Actual | Limiting Factor |
|--------|--------|--------|-----------------|
| INA | 1 kHz | ~270 Hz | I2C hardware (~590 µs/channel) |
| Telemetry | 100 Hz | 100 Hz | Software-configurable via `--onboard-telemetry-period-us` |

### Why Not 1 kHz for INA?

The INA3221's I2C transactions include ADC conversion time. Each `open()` + `read()` + `close()` on a sysfs hwmon file triggers a full I2C transaction. This is a hardware limitation, not a software issue.

Options to achieve higher INA sample rates:
1. Modify kernel `ina3221` driver to batch reads (requires kernel module rebuild)
2. Replace INA3221 with a sensor supporting I2C block reads
3. Use a different current sensing approach (e.g., shunt + ADC with parallel sampling)

## Data Fields

### Output JSON Structure

```json
{
  "sources": {
    "onboard_cpp": {
      "format": "onboard_csv/v1",
      "meta": {
        "columns": "mono_ns,unix_ns,vdd_in_mw,vdd_cpu_gpu_cv_mw,vdd_soc_mw,total_mw,gpu_freq_hz,cpu_cluster0_freq_hz,cpu_cluster1_freq_hz,emc_freq_hz,temp_cpu_mc,temp_gpu_mc,temp_soc0_mc,temp_soc1_mc,temp_soc2_mc,temp_tj_mc,fan_rpm"
      },
      "samples": [
        {
          "mono_ns": 5654444135397,
          "unix_ns": 1774248484856625115,
          "power_w": 7.322,
          "rails": {
            "vdd_in_w": 5.254,
            "vdd_cpu_gpu_cv_w": 0.596,
            "vdd_soc_w": 1.472
          },
          "freqs": {
            "gpu_hz": 306000000,
            "cpu_cluster0_hz": 1190400,
            "cpu_cluster1_hz": 729600,
            "emc_hz": 204000000
          },
          "temps": {
            "cpu_mc": 55718,
            "gpu_mc": 56062,
            "soc0_mc": 54750,
            "soc1_mc": 55062,
            "soc2_mc": 54187,
            "tj_mc": 56062
          },
          "fan_rpm": 2163
        }
      ]
    }
  }
}
```

### Field Descriptions

| Field | Unit | Source | Description |
|-------|------|--------|-------------|
| `vdd_in_mw` | mW | INA3221 ch1 | Total input power |
| `vdd_cpu_gpu_cv_mw` | mW | INA3221 ch2 | CPU+GPU+CV rail power |
| `vdd_soc_mw` | mW | INA3221 ch3 | SoC rail power |
| `gpu_hz` | Hz | devfreq | GPU core frequency |
| `cpu_cluster0_hz` | Hz | cpufreq | CPU cluster 0 (little cores) |
| `cpu_cluster1_hz` | Hz | cpufreq | CPU cluster 1 (big cores) |
| `emc_hz` | Hz | clk debugfs | External Memory Controller frequency |
| `temp_cpu_mc` | m°C | thermal_zone0 | CPU temperature |
| `temp_gpu_mc` | m°C | thermal_zone1 | GPU temperature |
| `temp_soc0_mc` | m°C | thermal_zone5 | SoC zone 0 temperature |
| `temp_soc1_mc` | m°C | thermal_zone6 | SoC zone 1 temperature |
| `temp_soc2_mc` | m°C | thermal_zone7 | SoC zone 2 temperature |
| `temp_tj_mc` | m°C | thermal_zone8 | Junction temperature |
| `fan_rpm` | RPM | hwmon2 | Fan speed |

## Configuration

### CLI Options

```bash
sudo ./host_pc_client --onboard \
    --onboard-path /sys/class/hwmon/hwmon1 \
    --onboard-period-us 1000 \
    --onboard-telemetry-period-us 10000 \
    --jetson-freq-path /proc/jetson_freqs \
    --duration-s 60
```

| Option | Default | Description |
|--------|---------|-------------|
| `--onboard` | (off) | Enable onboard sampling |
| `--onboard-path` | `/sys/class/hwmon/hwmon1` | INA3221 hwmon path |
| `--onboard-period-us` | `1000` | INA thread sample period (µs) |
| `--onboard-telemetry-period-us` | `10000` | Telemetry thread sample period (µs) |
| `--jetson-freq-path` | `/proc/jetson_freqs` | Path to jetson_freq_reader kernel module output |

### Running as Root

CPU and EMC frequency files require root access:

```bash
# These files are root-owned:
/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq  # -r--r----- root root
/sys/kernel/debug/clk/emc/clk_rate                        # -r-------- root root
```

**Solution**: Run the binary as root:

```bash
sudo ./host_pc_client --onboard ...
```

This allows plain `open()/read()` syscalls without sudo subprocess overhead (~15 ms per call with popen).

## Implementation Details

### Thread Startup Sequence

1. `start()` validates configuration
2. Pre-initializes `shared_sample_` with one telemetry read (avoids -1 stale values)
3. Launches both threads
4. If any thread fails to start, cleans up the other

### Telemetry Data Freshness

- Telemetry fields are updated at 100 Hz (configurable via `--onboard-telemetry-period-us`) by the telemetry thread
- INA thread snapshots the current values on each sample
- Maximum staleness: 10 ms (default) for frequency/temperature data
- Power data is always fresh (read directly in INA thread)

### Error Handling

- `sysfs_read()` returns `std::nullopt` on any error
- Missing fields are set to `-1` in the output
- Telemetry thread continues even if individual reads fail
- INA thread continues even if telemetry thread is slow

### Real-Time Scheduling

Only the INA thread applies RT scheduling (`SCHED_FIFO`) and CPU affinity:

```cpp
// INA thread
apply_thread_affinity();  // Sets RT priority if configured

// Telemetry thread
// No RT priority: telemetry is less latency-critical than the INA thread
```

## Troubleshooting

### Low Sample Rate

If INA sample rate is lower than expected:

1. Check I2C bus contention: `i2cdetect -y 1`
2. Verify no other processes are reading INA3221
3. Confirm expected per-file latency: ~590 µs

### Missing Telemetry Data

If frequency/temperature fields show `-1`:

1. Verify running as root: `sudo ./host_pc_client ...`
2. Check file permissions: `ls -la /sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq`
3. Confirm paths exist: `cat /sys/kernel/debug/clk/emc/clk_rate`

### Permission Denied

CPU/EMC frequency files require root. Without `sudo`:

```
cpu_cluster0_freq_hz: -1
emc_hz: -1
```

Solution: Always run as `sudo ./host_pc_client --onboard ...`

## Future Improvements

1. **Batch INA reads**: Modify kernel driver to read all channels in one I2C transaction
2. **Voltage rails**: Add in4-in7 if monitoring additional power rails becomes necessary
3. **Alternative sensors**: Support non-INA3221 power monitors with faster sampling
