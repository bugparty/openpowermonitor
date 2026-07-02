# Powermonitor Bug Audit Report

## Scope

Five parallel reviewers covered `device/`, `protocol/+node/`, `pc_client/`, `jetson_freq_reader/+onboard_power_sampler/`, and `web_viewer/+sim/`. The known `SET_CFG` PIO/hardware-I2C contention issue was excluded because it is already documented.

I re-verified the items marked `[verified]`. The protocol reviewer additionally reproduced its findings with compiled test cases.

## High Severity

1. Time-sync correction has the wrong sign. `[verified]`
   - `pc_client/src/power_monitor_session.cpp:107` computes `offset_ntp = ((T1−T2)+(T4−T3))/2`, which is host−device, while the spec and firmware use device−host.
   - The payload is negated again at `pc_client/src/power_monitor_session.cpp:746`.
   - Result: each sync pushes the device clock farther away from the host until later samples are rejected as outliers.

2. `STREAM_STOP` is never actually sent at shutdown. `[verified]`
   - `power_monitor_session.cpp:345` sets `stop_requested_ = true` before `stop_streaming()`.
   - `send_command_with_retry()` returns false immediately once that flag is set.
   - Result: normal exit logs `STREAM_STOP failed` without sending a byte, and the Pico keeps streaming.

3. Core-1 sampler can deadlock the whole device on a single I2C NACK. `[verified structurally]`
   - `device/sampler.hpp:151` waits on DMA before checking the error path at line 166.
   - On a NACK the PIO program halts and RX DMA never completes, so the error path is unreachable.
   - Detailed mechanism:
     - The PIO I2C engine halts by design on an unexpected NACK, raising its IRQ and parking the state machine until the flag is cleared.
     - The RX DMA is armed for an exact word count, so a mid-transaction halt means the count never reaches zero.
     - The hot path calls `dma_channel_wait_for_finish_blocking()` before `pio_i2c_check_error()`, so the recovery code is placed after an unbounded wait and never runs when the transaction is actually broken.
     - The codebase already has the right recovery pattern in the slow DMA diagnostic path and the CPU path; only the 1 kHz DMA hot path lacks the timeout.
     - Once Core 1 wedges, it stops draining the multicore FIFO. Core 0 then eventually blocks on `multicore_fifo_push_blocking()`, which takes the whole device down.
   - This can be triggered by a transient bus glitch, a marginal cable or pull-up, INA228 brownout or reset, or the documented dual-master contention case.
   - The fix shape is to replace the blocking wait with a bounded poll, abort both DMA channels on timeout, call `pio_i2c_resume_after_error()`, increment a drop counter, and return without pushing a sample.

4. Unaligned 64-bit store in the `TIME_SYNC` responder. `[verified]`
   - `device/command_handler.hpp:244` writes `t3` through a misaligned `uint64_t*`.
   - On Cortex-M0+ this is undefined behavior and may fault depending on codegen.

5. `REG_READ` / `REG_WRITE` use hardware I2C after boot deinitializes it. `[verified]`
   - `device/powermonitor.cpp:233` deinitializes the hardware I2C peripheral in non-test builds.
   - The register handlers still call the blocking INA228 hardware-I2C path.
   - Result: a debug command can hang Core 0 until power cycle.

6. Web viewer crashes on hover for latency-only datasets. `[verified]`
   - `web_viewer/src/state/useTimelineState.ts:110` calls `point.power.toFixed(4)` without a null guard.
   - `parsePayload.ts:186` intentionally sets `power: null` for latency-only samples.

## Medium Severity

### Protocol / Parser

- `protocol/parser.cpp:99` erases frame bytes before the CRC check, so `resync()` cannot rescan them.
- `parser.cpp:141-151` never treats a trailing lone `0xAA` as a possible SOF start.
- The parser has no header/payload timeout even though the spec state machine requires one.
- `kDefaultMaxLen = 1024` is smaller than the firmware/spec limit for `TEXT_REPORT`.
- `TIME_SYNC` responses are CRC-exempt by design, but `pc_node.cpp` has no offset sanity limit.

### Firmware

- `command_handler.hpp:447` leaks the Core-1 sampling timer during stress-to-normal transitions.
- `sampler.hpp:117-120` writes 64-bit offsets as two 32-bit halves, so Core 1 can observe torn values.
- The sample queue is not drained on `STREAM_STOP`, so the next stream can emit stale samples.
- `STREAM_START` while streaming updates the reported config but not the actual timer.
- `command_handler.hpp:360` byte-swaps the 16-bit `REG_READ` path before `memcpy`.
- `pio_i2c.c:27-41` does not restart the PIO state machine during recovery.

### PC Client

- `push_overwrite` in the vendored SPSC buffer publishes slot data before the sequence number.
- `choose_offset` uses an algebraically incorrect asymmetry test.
- `onboard_sampler.cpp:191-236` can become a 100% `SCHED_FIFO` spinner under `--onboard-rt-prio`.
- `shm_power_ring_buffer.h` does not verify compatibility on attach.
- `serial.cpp:30` uses a very small async read buffer, adding callback overhead.
- `pc_node.cpp:224` advances the drop detector only on DATA frames even though CFG/TEXT/DATA share one sequence counter.

### Sim / Kernel Module

- `sim/virtual_link.cpp:30` seeds RNG from wall clock, so packet-drop tests are not reproducible.
- The sim time-sync loop is open, so corrections do not converge properly.
- `jetson_freq_reader.c:84-94` discards sysfs read errors and returns 0 Hz as a successful reading.
- The checked-in module does not expose the `/proc/jetson_freqs_set` path referenced in repo docs.

## Low Severity

- `varint2float` breaks on negative values and has UB shifts for 40-bit fields.
- `samples_dropped` is incremented racily from both cores.
- `get_current_raw` writes into the pointer variable instead of the buffer.
- The Core-1 sampling timer fires on Core 0's default alarm pool.
- No boot `CFG_REPORT` despite the protocol spec.
- `stop()` on all three PC client queues can lose wakeups.
- `read_thread` accepts DATA_SAMPLE frames that the processor later drops.
- Min-signed-offset selection is biased.
- Chunk filenames use one-second resolution and can overwrite after a clock step-back.
- `onboard_power_sampler` does not catch malformed CLI exceptions.
- `--period-us 0` busy-spins.
- Web viewer zoom state resets when the moving-average window changes.
- Gantt bars can paint over the label gutter.
- `preventDefault()` on wheel zoom is ineffective under React 19 passive listeners.
- `NaN end_us` can blank charts.
- Unvalidated lane values yield undefined span colors.
- `min_chunk == 0` hangs `VirtualLink::send`.
- The event-loop FIFO tie-break id is dead code.
- The INA228 model temperature depends on call rate, not time.

## Suggested Priorities

1. Fix the time-sync sign inversion first, since it silently corrupts all device timestamps and alignment.
2. Fix the SHM publish ordering next, because it can feed torn samples into DVFS control.
3. Fix `STREAM_STOP` and the firmware deadlock path together, because they explain the device-wedged symptoms.
4. Fix the web-viewer hover crash separately if latency-only datasets are in active use.

## Notes

- The highest-risk data corruption issues are the time-sync sign inversion and torn SHM samples.
- The shutdown and I2C deadlock issues are the most plausible causes of a device that stays wedged until power cycle.
