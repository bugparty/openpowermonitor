# powermonitor/CLAUDE.md

This file provides guidance for working with the powermonitor system.

## Overview

The `powermonitor/` directory contains a power monitoring system with four major components:
- **Pico firmware** — RP2040-based INA228 sensor reader (`device/`)
- **PC client** — Host-side serial client that collects samples and writes SHM + JSON (`pc_client/`)
- **Web viewer** — React timeline visualization for power + latency data (`web_viewer/`)
- **Kernel module** — Jetson frequency control + readback via /proc (`jetson_freq_reader/`)

## Building

### PC Client (C++17, CMake)

```bash
cd powermonitor
cmake -B build && cmake --build build -j$(nproc)
```

### Pico Firmware (Pico SDK)

```bash
cd powermonitor/device
cmake -B build && cmake --build build -j$(nproc)
```

### Web Viewer (React + Vite)

```bash
cd powermonitor/web_viewer
npm install
npm run dev          # Dev server
npm run build:web    # Production build
```

### Kernel Module (jetson_freq_reader)

```bash
cd powermonitor/jetson_freq_reader && make
sudo insmod jetson_freq_reader.ko
```

See `scheduler/CLAUDE.md` for the full kernel module API (`/proc/jetson_freqs`, `/proc/jetson_freqs_set`).

## PC Client

The pc_client connects to a Pico via USB serial, collects power samples, and can:
- Write JSON output files (for offline analysis)
- Write to Power SHM ring buffer (for realtime DVFS control)

Key source files:
- `pc_client/src/main.cpp` — CLI entry point (CLI11, YAML config)
- `pc_client/src/power_monitor_session.cpp` — Session management
- `pc_client/src/onboard_sampler.cpp` — Jetson Nano onboard telemetry (INA3221, freq, thermal)
- `pc_client/include/realtime_power_metrics.h` — SHM header layout
- `pc_client/include/shm_power_ring_buffer.h` — SHM ring buffer helper

When running with `--shm`, pc_client writes to `/powermonitor_power_metrics` SHM for the scheduler to read. When running without, it writes JSON to disk.

## Pico Firmware

RP2040 firmware that reads INA228 power sensors via I2C and streams samples over USB serial.

Key source files:
- `device/powermonitor.cpp` — Main firmware (dual-core)
- `device/INA228.cpp/hpp` — INA228 sensor driver
- `device/sampler.hpp` — Sampling logic
- `device/command_handler.hpp` — Command processing
- `device/pio_i2c.c/h` — PIO-based I2C bitbang

Protocol: UART framing with CRC16. See `docs/uart_protocol.md` for details.

## Web Viewer

Browser-based timeline/Gantt visualization for power + latency JSON data.

Key source files:
- `web_viewer/src/App.tsx` — Main React component
- `web_viewer/src/components/GanttPanel.tsx` — Gantt/timeline visualization
- `web_viewer/src/components/TimelineChart.tsx` — Chart component
- `web_viewer/src/domain/parsePayload.ts` — JSON parser

## UART Protocol

See `docs/uart_protocol.md` for the full protocol specification including frame format, CRC, and command set.

## Web Viewer Data Samples

- Agents generating JSON data for the web viewer must follow the reference format in `web_viewer/src/dataset/agent_format_sample.json`.
- When designing or reviewing web viewer UI, load `web_viewer/src/dataset/full_capability_sample.json` as the reference dataset.
