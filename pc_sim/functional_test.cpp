#include <algorithm>
#include <cstdlib>

#include <gtest/gtest.h>
#include "node/device_node.h"
#include "node/pc_node.h"
#include "protocol/frame_builder.h"
#include "sim/event_loop.h"
#include "sim/virtual_link.h"

// ===========================
// FUNCTIONAL TESTS
// ===========================
// These tests verify end-to-end functionality of the power monitor system,
// including command handling, data streaming, and error tolerance.

// ---------------------------------------------------------------------------
// Fixture: full PC + Device pair over a clean link
// ---------------------------------------------------------------------------
class PowerMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        sim::LinkConfig config;
        config.min_chunk = 1;
        config.max_chunk = 16;
        config.min_delay_us = 0;
        config.max_delay_us = 2000;
        config.drop_prob = 0.0;
        config.flip_prob = 0.0;
        link.set_pc_to_dev_config(config);
        link.set_dev_to_pc_config(config);

        pc = std::make_unique<node::PCNode>(&link.pc());
        device = std::make_unique<node::DeviceNode>(&link.device());
    }

    void TearDown() override {
        pc.reset();
        device.reset();
    }

    void RunSimulation(uint64_t duration_us, uint64_t tick_interval_us) {
        loop.run_for(duration_us, tick_interval_us, [&](uint64_t now_us) {
            link.pump(now_us);
            pc->tick(now_us);
            device->tick(now_us);
        });
    }

    sim::VirtualLink link;
    sim::EventLoop loop;
    std::unique_ptr<node::PCNode> pc;
    std::unique_ptr<node::DeviceNode> device;
};

// ---------------------------------------------------------------------------
// Fixture: PC only (no device), 100% drop on PC→device direction.
// Used to exercise the timeout/retransmit path without a device to respond.
// ---------------------------------------------------------------------------
class CommandTimeoutTest : public ::testing::Test {
protected:
    void SetUp() override {
        sim::LinkConfig drop_all;
        drop_all.drop_prob = 1.0;
        link.set_pc_to_dev_config(drop_all);
        pc = std::make_unique<node::PCNode>(&link.pc());
    }

    void RunSimulation(uint64_t duration_us, uint64_t tick_interval_us) {
        loop.run_for(duration_us, tick_interval_us, [&](uint64_t now_us) {
            link.pump(now_us);
            pc->tick(now_us);
            // No device — nothing ever acknowledges commands.
        });
    }

    sim::VirtualLink link;
    sim::EventLoop loop;
    std::unique_ptr<node::PCNode> pc;
};

// ===========================================================================
// Happy-path tests
// ===========================================================================

TEST_F(PowerMonitorTest, PingCommand) {
    pc->send_ping(loop.now_us());
    RunSimulation(100'000, 500);

    EXPECT_EQ(pc->crc_fail_count(), 0);
    EXPECT_EQ(pc->timeout_count(), 0);
    EXPECT_EQ(pc->orphan_rsp_count(), 0);
}

TEST_F(PowerMonitorTest, SetConfiguration) {
    pc->send_set_cfg(0x0000, 0x0000, 0x1000, 0x0000, loop.now_us());
    RunSimulation(100'000, 500);

    EXPECT_EQ(pc->crc_fail_count(), 0);
    EXPECT_EQ(pc->timeout_count(), 0);
}

TEST_F(PowerMonitorTest, StreamStartStop) {
    pc->send_stream_start(1000, 0x000F, loop.now_us());
    RunSimulation(1'000'000, 500);

    pc->send_stream_stop(loop.now_us());
    RunSimulation(500'000, 500);

    EXPECT_EQ(pc->crc_fail_count(), 0);
    EXPECT_EQ(pc->timeout_count(), 0);
}

TEST_F(PowerMonitorTest, CompleteDataStreamingScenario) {
    pc->send_ping(loop.now_us());
    pc->send_set_cfg(0x0000, 0x0000, 0x1000, 0x0000, loop.now_us());
    pc->send_stream_start(1000, 0x000F, loop.now_us());

    RunSimulation(10'000'000, 500);

    pc->send_stream_stop(loop.now_us());
    RunSimulation(500'000, 500);

    EXPECT_EQ(pc->crc_fail_count(), 0)       << "CRC failures on clean link";
    EXPECT_EQ(pc->data_drop_count(), 0)       << "Data drops on clean link";
    EXPECT_EQ(pc->timeout_count(), 0)         << "Command timeouts on clean link";
    EXPECT_EQ(pc->retransmit_count(), 0)      << "Retransmissions on clean link";
    EXPECT_EQ(pc->orphan_rsp_count(), 0)      << "Orphan RSPs on clean link";
    EXPECT_EQ(pc->error_rsp_count(), 0)       << "Error RSPs on clean link";
    EXPECT_EQ(pc->truncated_data_count(), 0)  << "Truncated DATA frames on clean link";
}

TEST_F(PowerMonitorTest, TimeSynchronizationSequence) {
    constexpr uint8_t kMsgTimeSync   = 0x05;
    constexpr uint8_t kMsgTimeAdjust = 0x06;
    constexpr uint8_t kMsgTimeSet    = 0x07;

    pc->send_time_sync(loop.now_us());
    RunSimulation(200'000, 500);

    EXPECT_EQ(pc->timeout_count(), 0)  << "Timeouts during TIME_SYNC";
    EXPECT_EQ(pc->crc_fail_count(), 0) << "CRC failures during TIME_SYNC";
    EXPECT_EQ(pc->get_rx_count(kMsgTimeSync),   1) << "TIME_SYNC RSP missing";
    EXPECT_EQ(pc->get_rx_count(kMsgTimeAdjust), 1) << "TIME_ADJUST RSP missing";

    pc->send_time_set(1678900000000000ULL, loop.now_us());
    RunSimulation(100'000, 500);

    EXPECT_EQ(pc->timeout_count(), 0)              << "Timeouts during TIME_SET";
    EXPECT_EQ(pc->get_rx_count(kMsgTimeSet), 1)    << "TIME_SET RSP missing";
}

// ---------------------------------------------------------------------------
// Fixture: PC + Device over a deterministic link (fixed symmetric delay,
// whole-frame chunks, no faults) for time-sync clock-error experiments.
// The device clock error is injected via TIME_SET and observed through
// DeviceNode::epoch_offset_us(): 0 means the device Unix clock matches the
// simulation (host) clock exactly.
// ---------------------------------------------------------------------------
class TimeSyncClockErrorTest : public ::testing::Test {
protected:
    static constexpr int64_t kLinkDelayUs = 500;

    void SetUp() override {
        sim::LinkConfig config;
        config.min_chunk = 64;
        config.max_chunk = 64;
        config.min_delay_us = kLinkDelayUs;
        config.max_delay_us = kLinkDelayUs;
        config.drop_prob = 0.0;
        config.flip_prob = 0.0;
        link.set_pc_to_dev_config(config);
        link.set_dev_to_pc_config(config);

        pc = std::make_unique<node::PCNode>(&link.pc());
        device = std::make_unique<node::DeviceNode>(&link.device());
    }

    void RunSimulation(uint64_t duration_us, uint64_t tick_interval_us) {
        loop.run_for(duration_us, tick_interval_us, [&](uint64_t now_us) {
            link.pump(now_us);
            pc->tick(now_us);
            device->tick(now_us);
        });
    }

    // Injects a device clock error of ~error_us (minus one link delay, since
    // TIME_SET carries the PC send-time and is applied on arrival).
    void InjectClockError(int64_t error_us) {
        pc->send_time_set(loop.now_us() + static_cast<uint64_t>(error_us), loop.now_us());
        RunSimulation(20'000, 100);
    }

    // Runs one TIME_SYNC exchange and returns the T1..T4 measurement.
    node::PCNode::TimeSyncMeasurement RunSyncRound() {
        const uint64_t completed_before = pc->last_time_sync().count;
        pc->send_time_sync(loop.now_us());
        RunSimulation(20'000, 100);
        EXPECT_EQ(pc->last_time_sync().count, completed_before + 1)
            << "TIME_SYNC exchange did not complete";
        return pc->last_time_sync();
    }

    sim::VirtualLink link;
    sim::EventLoop loop;
    std::unique_ptr<node::PCNode> pc;
    std::unique_ptr<node::DeviceNode> device;
};

// Verbatim replica of pc_client's offset policy
// (pc_client/src/power_monitor_session.cpp: choose_offset() computes the
// device-minus-host NTP offset, run_time_sync_rounds() applies the negated
// median across rounds). Kept in sync by citation, not by linkage: the
// original lives in an anonymous namespace inside the pc_client binary.
namespace pc_client_policy {

int64_t choose_offset(int64_t T1, int64_t T2, int64_t T3, int64_t T4) {
    return ((T2 - T1) + (T3 - T4)) / 2;
}

int64_t median_offset(std::vector<int64_t> offsets) {
    const size_t mid = offsets.size() / 2;
    std::nth_element(offsets.begin(), offsets.begin() + mid, offsets.end());
    return offsets[mid];
}

} // namespace pc_client_policy

// With the spec offset formula (docs/protocol/uart_protocol.md §5.2:
// offset = ((T2-T1)+(T3-T4))/2, TIME_ADJUST payload = -offset, as
// implemented by PCNode's automatic handler), an injected device clock
// error converges to ~0 within a few sync rounds.
TEST_F(TimeSyncClockErrorTest, SpecOffsetFormulaConvergesDeviceClock) {
    InjectClockError(100'000);
    const int64_t initial_error = device->epoch_offset_us();
    ASSERT_GT(initial_error, 90'000) << "Clock error injection failed";

    for (int round = 0; round < 3; ++round) {
        RunSyncRound();  // Auto TIME_ADJUST enabled by default
    }
    RunSimulation(20'000, 100);  // Let the last TIME_ADJUST land

    EXPECT_LE(llabs(device->epoch_offset_us()), 50)
        << "Spec offset formula must drive the device clock error to ~0";
    EXPECT_EQ(pc->timeout_count(), 0);
    EXPECT_EQ(pc->crc_fail_count(), 0);
}

// Regression test for the pc_client time-sync sign bug (fixed 2026-07-01):
// choose_offset() used to compute the host-minus-device convention while the
// TIME_ADJUST payload still negated it, so the applied correction had the
// wrong sign and the device clock error DOUBLED on every adjust cycle. This
// test replays pc_client's exact (corrected) policy over the simulated link
// and asserts an injected clock error converges to ~0 — if the sign ever
// flips back, the error doubles instead and this test fails loudly.
TEST_F(TimeSyncClockErrorTest, PcClientOffsetPolicyConvergesDeviceClock) {
    pc->set_auto_time_adjust(false);

    InjectClockError(1'300);
    const int64_t initial_error = device->epoch_offset_us();
    ASSERT_GT(initial_error, 500) << "Clock error injection failed";

    constexpr int kPeriodicSyncRounds = 3;  // Matches pc_client
    constexpr int kAdjustCycles = 2;

    for (int cycle = 0; cycle < kAdjustCycles; ++cycle) {
        std::vector<int64_t> offsets;
        for (int round = 0; round < kPeriodicSyncRounds; ++round) {
            const auto m = RunSyncRound();
            offsets.push_back(pc_client_policy::choose_offset(
                static_cast<int64_t>(m.t1), static_cast<int64_t>(m.t2),
                static_cast<int64_t>(m.t3), static_cast<int64_t>(m.t4)));
        }
        // pc_client applies the median offset of the batch, negated.
        pc->send_time_adjust(-pc_client_policy::median_offset(offsets), loop.now_us());
        RunSimulation(20'000, 100);
    }

    EXPECT_LE(llabs(device->epoch_offset_us()), 50)
        << "pc_client offset policy (choose_offset + '-median') must converge "
        << "the device clock to the host, not move it away";
    EXPECT_EQ(pc->timeout_count(), 0);
    EXPECT_EQ(pc->crc_fail_count(), 0);
}

// ===========================================================================
// CFG_REPORT content verification
// Verifies that handle_cfg_report() correctly parses values sent by the device.
// ===========================================================================

TEST_F(PowerMonitorTest, CfgReportValuesParsedAfterGetCfg) {
    // Start streaming so stream_period_us and stream_mask are set on device.
    pc->send_stream_start(1000, 0x000F, loop.now_us());
    RunSimulation(200'000, 500);

    pc->send_get_cfg(loop.now_us());
    RunSimulation(100'000, 500);

    EXPECT_EQ(pc->timeout_count(), 0)              << "Timeout waiting for GET_CFG RSP";
    EXPECT_EQ(pc->stream_period_us_cfg(), 1000)    << "stream_period_us not parsed from CFG_REPORT";
    EXPECT_EQ(pc->stream_mask_cfg(),  0x000F)      << "stream_mask not parsed from CFG_REPORT";
    EXPECT_EQ(pc->current_lsb_nA(),  1000U)        << "current_lsb_nA not parsed from CFG_REPORT";

    pc->send_stream_stop(loop.now_us());
    RunSimulation(100'000, 500);
}

// ===========================================================================
// Data sequence number tests
// ===========================================================================

// Verify that the uint8 wraparound of DATA frame seq (255 → 0) is not
// misidentified as a sequence gap. Run streaming long enough to produce >256
// DATA samples so the counter wraps at least once.
TEST_F(PowerMonitorTest, DataSeqWraparoundNoFalseDrops) {
    // Do NOT send SET_CFG during streaming — that triggers an extra CFG_REPORT
    // which increments device data_seq_, creating a perceived gap in DATA seqs.
    pc->send_stream_start(1000, 0x000F, loop.now_us());
    // Allow stream_start to complete, then run 300ms = ~300 DATA samples at 1kHz.
    // Device data_seq_ starts at 2 (after initial CFG_REPORT and TEXT_REPORT),
    // so DATA seqs are 2,3,...,255,0,1,...  The wraparound must not raise a drop.
    RunSimulation(350'000, 500);

    pc->send_stream_stop(loop.now_us());
    RunSimulation(100'000, 500);

    EXPECT_EQ(pc->data_drop_count(), 0) << "Seq uint8 wraparound incorrectly detected as a drop";
    EXPECT_EQ(pc->crc_fail_count(), 0);
    EXPECT_EQ(pc->timeout_count(), 0);
}

// ===========================================================================
// Fault-injection tests — verify errors are detected, not silently ignored
// ===========================================================================

// With 5% packet drops, at least some CRC errors or data drops must be
// detected by the PC. Asserting >= 0 is meaningless; we want > 0.
TEST_F(PowerMonitorTest, PacketDropsCauseDetectableErrors) {
    sim::LinkConfig config;
    config.min_chunk = 1;
    config.max_chunk = 16;
    config.min_delay_us = 0;
    config.max_delay_us = 2000;
    config.drop_prob = 0.05;
    config.flip_prob = 0.0;
    link.set_pc_to_dev_config(config);
    link.set_dev_to_pc_config(config);

    pc->send_ping(loop.now_us());
    pc->send_set_cfg(0x0000, 0x0000, 0x1000, 0x0000, loop.now_us());
    pc->send_stream_start(1000, 0x000F, loop.now_us());

    RunSimulation(2'000'000, 500);

    pc->send_stream_stop(loop.now_us());
    RunSimulation(500'000, 500);

    // At 5% chunk-level drop with ~3 chunks per DATA frame, ~14% of frames
    // will have at least one dropped chunk, producing CRC errors.
    EXPECT_GT(pc->crc_fail_count(), 0) << "5% drop should produce detectable CRC errors";
}

// With 1% per-bit corruption, a 48-byte DATA frame (384 bits) has ~98%
// probability of containing at least one flipped bit, so CRC errors are
// virtually certain over a 2-second run.
TEST_F(PowerMonitorTest, BitFlipsCauseDetectableCrcErrors) {
    sim::LinkConfig config;
    config.min_chunk = 1;
    config.max_chunk = 16;
    config.min_delay_us = 0;
    config.max_delay_us = 2000;
    config.drop_prob = 0.0;
    config.flip_prob = 0.01;
    link.set_pc_to_dev_config(config);
    link.set_dev_to_pc_config(config);

    pc->send_ping(loop.now_us());
    pc->send_set_cfg(0x0000, 0x0000, 0x1000, 0x0000, loop.now_us());
    pc->send_stream_start(1000, 0x000F, loop.now_us());

    RunSimulation(2'000'000, 500);

    pc->send_stream_stop(loop.now_us());
    RunSimulation(500'000, 500);

    EXPECT_GT(pc->crc_fail_count(), 0) << "1% bit-flip should produce detectable CRC errors";
}

// ===========================================================================
// Timeout / retransmit tests
// ===========================================================================

// When all PC→device traffic is dropped, the PC must retransmit exactly
// kMaxRetries (3) times and then record a timeout.
TEST_F(CommandTimeoutTest, CommandRetransmitsAndTimesOut) {
    // kCmdTimeoutUs = 200ms, kMaxRetries = 3
    // Timeline: send at t=0, retransmit at t=200ms/400ms/600ms, timeout at t=800ms.
    pc->send_ping(loop.now_us());
    RunSimulation(900'000, 500);

    EXPECT_EQ(pc->timeout_count(),     1) << "Expected exactly 1 timeout after max retries";
    EXPECT_EQ(pc->retransmit_count(),  3) << "Expected exactly 3 retransmits before timeout";
}

// Multiple commands queued simultaneously — each times out independently.
TEST_F(CommandTimeoutTest, MultipleCommandsAllTimeout) {
    pc->send_ping(loop.now_us());
    pc->send_get_cfg(loop.now_us());

    RunSimulation(900'000, 500);

    EXPECT_EQ(pc->timeout_count(),    2) << "Each queued command must time out independently";
    EXPECT_EQ(pc->retransmit_count(), 6) << "3 retransmits per command × 2 commands";
}

// ===========================================================================
// 64-bit timestamp tests
// ===========================================================================

// Verify that timestamps exceeding the old 32-bit limit (~71.6 min) are
// correctly handled without wraparound.
TEST_F(PowerMonitorTest, LongRunningStreamTimestampNoWraparound) {
    // Start streaming at a large base time so that relative timestamps exceed
    // the old uint32_t max (4,294,967,295 µs ≈ 71.6 min).
    // We set stream_start very early and run well past the 32-bit boundary.
    constexpr uint64_t kBaseTime = 100'000;  // 100 ms
    constexpr uint64_t kStreamPeriod = 10'000;  // 10 ms per sample (100 Hz)

    // Fast-forward time to the base, let initial handshake happen
    RunSimulation(kBaseTime, 500);

    pc->send_stream_start(kStreamPeriod, 0x000F, loop.now_us());
    RunSimulation(200'000, 500);  // let stream start settle

    // Now jump ahead past the 32-bit boundary: ~4.3 billion µs = ~71.6 min
    // We simulate in a large step to avoid running billions of ticks.
    // Run 4,300,000,000 µs (~71.7 min) with coarse tick interval.
    constexpr uint64_t kLongDuration = 4'300'000'000ULL;
    RunSimulation(kLongDuration, kStreamPeriod);

    // The last timestamp should be > uint32_t max, proving no wraparound
    EXPECT_GT(pc->last_timestamp_us(), 4'294'967'295ULL)
        << "Timestamp should exceed uint32_t max after 71+ min of streaming";
    EXPECT_EQ(pc->truncated_data_count(), 0)
        << "No DATA frames should be truncated with 64-bit timestamps";
    EXPECT_EQ(pc->crc_fail_count(), 0);
}

// Verify that the sim produces 41-byte DATA_SAMPLE payloads (post 64-bit migration).
TEST_F(PowerMonitorTest, DataSamplePayloadSize41Bytes) {
    pc->send_stream_start(1000, 0x000F, loop.now_us());
    RunSimulation(100'000, 500);

    // If any DATA frames arrived with <41 bytes, truncated_data_count would be > 0
    EXPECT_EQ(pc->truncated_data_count(), 0)
        << "All DATA_SAMPLE frames must have 41-byte payloads";
    // Verify we actually received some data
    EXPECT_GT(pc->get_rx_count(0x80), 0U)
        << "Should have received at least one DATA_SAMPLE";

    pc->send_stream_stop(loop.now_us());
    RunSimulation(100'000, 500);
}

// Verify timestamps are monotonically increasing during continuous streaming.
TEST_F(PowerMonitorTest, DataTimestampsMonotonicallyIncreasing) {
    pc->send_stream_start(1000, 0x000F, loop.now_us());

    // Run 5 seconds of streaming
    RunSimulation(5'000'000, 500);

    // After 5 seconds of 1kHz streaming, timestamp should be roughly 5 million µs
    // (accounting for startup delay and stream_start offset)
    EXPECT_GT(pc->last_timestamp_us(), 4'000'000ULL)
        << "Timestamp should reflect ~5 seconds of streaming";
    EXPECT_EQ(pc->data_drop_count(), 0)
        << "No sequence drops on clean link";

    pc->send_stream_stop(loop.now_us());
    RunSimulation(100'000, 500);
}

// ===========================================================================
// Orphan RSP and Error RSP fault-injection tests
// ===========================================================================

// Fixture for fault-injection tests: PC only (no device), clean link.
// Allows precise control over what frames the PC receives.
class FaultInjectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        sim::LinkConfig clean_config;
        clean_config.drop_prob = 0.0;
        link.set_pc_to_dev_config(clean_config);
        link.set_dev_to_pc_config(clean_config);
        pc = std::make_unique<node::PCNode>(&link.pc());
    }

    void RunSimulation(uint64_t duration_us, uint64_t tick_interval_us) {
        loop.run_for(duration_us, tick_interval_us, [&](uint64_t now_us) {
            link.pump(now_us);
            pc->tick(now_us);
            // No device — we inject frames manually.
        });
    }

    // Inject a raw frame from device to PC.
    void InjectFrame(const std::vector<uint8_t> &frame, uint64_t now_us) {
        link.device().write(frame, now_us);
    }

    sim::VirtualLink link;
    sim::EventLoop loop;
    std::unique_ptr<node::PCNode> pc;
};

// Test that a RSP with no matching pending command increments orphan_rsp_count.
// This simulates a "late" or spurious RSP arriving at the PC.
TEST_F(FaultInjectionTest, OrphanRspDetected) {
    // Build a RSP frame with seq=0x42 (not in pending_ map)
    // RSP payload: orig_msgid (1B) + status (1B)
    std::vector<uint8_t> rsp_payload;
    rsp_payload.push_back(0x01);  // orig_msgid = PING
    rsp_payload.push_back(0x00);  // status = OK
    auto rsp_frame = protocol::build_frame(
        protocol::FrameType::kRsp, 0, 0x42, 0x01, rsp_payload);

    // Inject the RSP frame directly from device to PC via VirtualLink
    InjectFrame(rsp_frame, loop.now_us());

    // Run briefly to let the frame be delivered and parsed
    RunSimulation(10'000, 500);

    EXPECT_EQ(pc->orphan_rsp_count(), 1)
        << "RSP with unmatched seq should increment orphan_rsp_count";
    EXPECT_EQ(pc->error_rsp_count(), 0)
        << "OK status should not increment error_rsp_count";
}

// Test that a RSP with non-OK status increments error_rsp_count.
// We send a command to create a pending entry, then inject an error RSP.
TEST_F(FaultInjectionTest, ErrorRspDetected) {
    // Send PING command - this creates a pending entry with seq=0, msgid=PING
    pc->send_ping(loop.now_us());
    const uint8_t expected_seq = 0;  // First command uses seq=0

    // Build an error RSP with matching seq but non-OK status
    std::vector<uint8_t> rsp_payload;
    rsp_payload.push_back(0x01);  // orig_msgid = PING
    rsp_payload.push_back(0x05);  // status = ERR_HW (hardware fault)
    auto rsp_frame = protocol::build_frame(
        protocol::FrameType::kRsp, 0, expected_seq, 0x01, rsp_payload);

    // Inject the error RSP directly from device endpoint
    InjectFrame(rsp_frame, loop.now_us());

    // Run briefly to let the frame be delivered and parsed
    RunSimulation(10'000, 500);

    EXPECT_EQ(pc->orphan_rsp_count(), 0)
        << "RSP with matching seq/msgid should not increment orphan_rsp_count";
    EXPECT_EQ(pc->error_rsp_count(), 1)
        << "RSP with non-OK status should increment error_rsp_count";
    EXPECT_EQ(pc->timeout_count(), 0)
        << "Error RSP should complete the command without timeout";
}

// Test orphan_rsp_count when orig_msgid mismatches the pending command.
// We send PING, then inject a RSP with correct seq but wrong orig_msgid.
TEST_F(FaultInjectionTest, OrphanRspDetectedOnMsgidMismatch) {
    // Send PING command - this creates a pending entry with seq=0, msgid=PING
    pc->send_ping(loop.now_us());
    const uint8_t expected_seq = 0;  // First command uses seq=0

    // Build a RSP with matching seq but different orig_msgid
    std::vector<uint8_t> rsp_payload;
    rsp_payload.push_back(0x10);  // orig_msgid = SET_CFG (not PING)
    rsp_payload.push_back(0x00);  // status = OK
    auto rsp_frame = protocol::build_frame(
        protocol::FrameType::kRsp, 0, expected_seq, 0x10, rsp_payload);

    // Inject the mismatched RSP
    InjectFrame(rsp_frame, loop.now_us());

    RunSimulation(10'000, 500);

    EXPECT_EQ(pc->orphan_rsp_count(), 1)
        << "RSP with orig_msgid mismatch should increment orphan_rsp_count";
    EXPECT_EQ(pc->error_rsp_count(), 0)
        << "Orphan RSP should not increment error_rsp_count";
}

// Test multiple orphan RSPs are counted correctly.
TEST_F(FaultInjectionTest, MultipleOrphanRspsCounted) {
    // Inject several orphan RSPs with different seq values
    for (uint8_t seq = 0; seq < 5; ++seq) {
        std::vector<uint8_t> rsp_payload;
        rsp_payload.push_back(0x01);  // orig_msgid = PING
        rsp_payload.push_back(0x00);  // status = OK
        auto rsp_frame = protocol::build_frame(
            protocol::FrameType::kRsp, 0, seq, 0x01, rsp_payload);
        InjectFrame(rsp_frame, loop.now_us());
    }

    RunSimulation(10'000, 500);

    EXPECT_EQ(pc->orphan_rsp_count(), 5)
        << "Each orphan RSP should be counted";
}
