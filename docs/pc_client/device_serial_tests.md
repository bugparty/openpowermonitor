# PC Client Device Serial Functional Tests

A design for gtest-based on-device functional tests that reuse `pc_client` facilities to exercise the device via serial/UART while minimizing disruption to device state.

## Related code
- `pc_client/include/read_thread.h`
- `pc_client/include/response_queue.h`
- `pc_client/include/sample_queue.h`
- `pc_client/include/protocol_helpers.h`
- `pc_client/src/serial/serial.cpp`
- `protocol/frame_builder.cpp`
- `protocol/parser.cpp`

## Goals
- Verify device protocol behavior over real serial links.
- Order tests from lowest side-effect to highest; abort if a risky step fails.
- Reuse existing pc_client framing, CRC, logging, and auto port discovery.
- Provide configurable timeouts and soak duration (default 30s).

## Assumptions
- Device responds within ~1s for commands; default command/event timeout is 1s.
- All registers are safe to read; config register 0x0 is safe to write (software reset is `RST` bit 15, keep it 0 for tests).
- pc_client can auto-discover/connect to the device; optional flags can override port/baud.

## Test Ordering (low -> high risk)
1. **PingSmoke**: Send `PING`, expect `RSP OK`.
2. **GetCfgReport**: `GET_CFG` -> `RSP OK` + `CFG_REPORT` with sane fields.
3. **SetCfgMinimal**: Safe `SET_CFG` (no streaming), expect `RSP OK` + `CFG_REPORT`.
4. **RegReadConfig**: Read config register `0x0`; verify fields (RST=0, ADCRANGE matches report, etc.).
5. **BadFrameCRC**: Inject one frame with bad CRC; next `PING` still works.
6. **BadFrameLenTooLarge**: Send frame with LEN above max; parser resync; next `PING` works.
7. **IdleCmd**: While idle, send benign CMD (e.g., `GET_CFG`) to confirm stable idle handling.
8. **StreamStartStopBasic**: Start stream with small mask/period; collect a few `DATA_SAMPLE`; stop; verify stop.
9. **StreamMidCommand**: While streaming, send `PING`/`GET_CFG`; expect responses and continued streaming; then stop.
10. **StreamStopIdempotent**: Stop twice; second stop is benign.
11. **StreamBadFrameDuringStream**: Inject one bad frame during streaming; streaming continues; then stop.
12. **StreamLongRunSoak**: 30s soak, track SEQ continuity/drop count; stop cleanly (run last).

## Failure and Abort Policy
- Maintain a global abort flag: any stream-control failure (start/stop/midstream) or serious protocol error sets it; remaining tests are skipped.
- On any failure, attempt a best-effort `STREAM_STOP`; if that fails, abort all.
- Keep tests deterministic; avoid randomness without fixed seeds.

## Harness Design
- Location: `pc_client/tests` with a gtest binary `pc_client_tests`.
- Shared fixture `DeviceSerialTest`:
  - Opens serial via pc_client auto-discovery; allows overrides for port/baud/timeouts.
  - Drains input before/after each test.
  - Tracks streaming state; ensures stop in teardown.
- Helpers:
  - `SendCommand(msgid, payload, timeout_ms=1000)`.
  - `WaitForEvent(msgid, timeout_ms=1000)`.
  - `StartStream(mask, period_us)` / `StopStream()`; sets/clears streaming flag.
  - `InjectRawFrame(bytes)` for malformed tests.
  - `DrainInput()` to clear RX noise before/after disruptive tests.

## Configuration Flags
- `--port` (optional) serial device; default: auto-discover.
- `--baud` (optional) baud rate if not default.
- `--cmd_timeout_ms` (default 1000).
- `--event_timeout_ms` (default 1000).
- `--soak_duration_s` (default 30) for the final soak test.
- `--abort_on_fail` (default true).

## Logging
- Keep concise; avoid flooding during streaming.
- Count CRC failures, drops, retries; log summary per test.

## TDD Workflow
- Add each test first, then implement helper/support code.
- Run targeted gtest filters during development, then full `pc_client_tests`, then `pwsh workflow.ps1` before commit.

## Safety Notes for Config Register 0x0
- Bit 15 `RST`: software reset (do not set in tests).
- Bit 14 `RSTACC`: clears ENERGY/CHARGE accumulators (avoid unless a test requires it).
- Bits 13–6 `CONVDLY`: conversion delay.
- Bit 5 `TEMPCOMP`: enable shunt tempco compensation.
- Bit 4 `ADCRANGE`: ±163.84 mV / ±40.96 mV full-scale selection.

## Deliverables
- New CMake target `pc_client_tests` built under `pc_client/tests`.
- Tests implement the ordered list above with abort-on-risk behavior.
- Documentation here kept in sync with test code per `docs/naming_convention.md`.

## Stress Test: STREAM_START/STOP Wedge Hunt

`DISABLED_Stress_StreamStartStopWedgeHunt` automates the hunt for a field-reported wedge (2026-07-01: rapid TUI stream toggling killed the sample flow until a full power cycle; not reproduced since). It is `DISABLED_` and excluded from the ordered suite — run explicitly:

```bash
pc_client_tests --port /dev/ttyACMx --gtest_also_run_disabled_tests --gtest_filter='*WedgeHunt*'
```

Structure (~2 minutes, deterministic sweeps, no randomness):
- **Phase A** — 100 rapid start/stop cycles (dwell 40–160 ms, gap 0–49 ms), sample-flow check per cycle, PING health check every 10 cycles. Three consecutive zero-sample cycles with live commands fails with a "sampler/INA228 dead" diagnosis.
- **Phase B** — streams until `EVT_TIME_SYNC_REQUEST` arrives (fires on wall time: immediately on the first stream after boot, then every 120 s), never answers `TIME_ADJUST`, and hammers a 12× STOP/START burst inside the device's 200 ms `sync_waiting` window — the race a TUI user hits pressing `t` right after starting a stream that followed a >2 min idle gap. This phase can wait up to ~2 min for the EVT.
- **Final** — PING + 100 samples in 2 s + stop-silence check, distinguishing "Core 0 wedged" from "sampler dead" in every failure message.

If it fails: do **not** power cycle (that also resets the separately-powered INA228 and destroys the evidence). Capture PING state and `STATS_REPORT` drop counters first.

## Hardware-Free Session Shutdown E2E Test

Separate from the on-device tests above, `pc_client/tests/session_shutdown_tests.cpp`
(CMake target `session_shutdown_test`, POSIX-only) runs a real
`PowerMonitorSession` end to end against a fake device on a PTY pair — no
hardware required. The fake device speaks the real protocol
(`protocol::Parser` + `protocol::build_frame`), answers the full init
handshake (PING, GET_CFG + CFG_REPORT, STREAM_STOP, TIME_SET, TIME_SYNC
rounds, STREAM_START), and records the order of received commands.

### `StreamStopIsSentOnDurationShutdown`
Runs a session with a short `duration_us` and asserts a `STREAM_STOP` is
received *after* the last `STREAM_START` (the init-time stop does not
count). Regression test for the shutdown-ordering bug where
`stop_requested_` was raised before `stop_streaming()`, so the stop command
was short-circuited and the device kept streaming after every normal exit.
