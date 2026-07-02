#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "protocol/frame_builder.h"
#include "protocol_helpers.h"
#include "read_thread.h"
#include "response_queue.h"
#include "sample_queue.h"
#include "serial/serial.h"

namespace {

constexpr uint8_t kMsgPing = 0x01;
constexpr uint8_t kMsgSetCfg = 0x10;
constexpr uint8_t kMsgGetCfg = 0x11;
constexpr uint8_t kMsgRegRead = 0x20;
constexpr uint8_t kMsgStreamStart = 0x30;
constexpr uint8_t kMsgStreamStop = 0x31;
constexpr uint8_t kMsgDataSample = 0x80;
constexpr uint8_t kMsgCfgReport = 0x91;
constexpr uint8_t kMsgTextReport = 0x93;
constexpr uint8_t kMsgTimeSyncRequest = 0x94;

struct TestOptions {
    std::string port;
    uint32_t baud = 115200;
    uint16_t vid = 0x2E8A;
    uint16_t pid = 0x000A;
    int cmd_timeout_ms = 1000;
    int event_timeout_ms = 1000;
    int soak_duration_s = 5;
    bool abort_on_fail = true;
};

TestOptions g_options;
std::atomic<bool> g_abort_requested{false};

bool parse_bool(const std::string &value, bool *out) {
    if (value == "1" || value == "true" || value == "TRUE" || value == "True") {
        *out = true;
        return true;
    }
    if (value == "0" || value == "false" || value == "FALSE" || value == "False") {
        *out = false;
        return true;
    }
    return false;
}

bool consume_string_arg(int &i, int argc, char **argv, const std::string &name, std::string *out) {
    const std::string arg(argv[i]);
    if (arg == name) {
        if (i + 1 >= argc) {
            return false;
        }
        *out = argv[++i];
        return true;
    }
    const std::string prefix = name + "=";
    if (arg.rfind(prefix, 0) == 0) {
        *out = arg.substr(prefix.size());
        return true;
    }
    return false;
}

bool consume_int_arg(int &i, int argc, char **argv, const std::string &name, int *out) {
    std::string value;
    if (!consume_string_arg(i, argc, argv, name, &value)) {
        return false;
    }
    *out = std::stoi(value);
    return true;
}

bool consume_uint32_arg(int &i, int argc, char **argv, const std::string &name, uint32_t *out) {
    std::string value;
    if (!consume_string_arg(i, argc, argv, name, &value)) {
        return false;
    }
    *out = static_cast<uint32_t>(std::stoul(value));
    return true;
}

bool consume_bool_arg(int &i, int argc, char **argv, const std::string &name, bool *out) {
    const std::string arg(argv[i]);
    if (arg == name) {
        *out = true;
        return true;
    }
    const std::string prefix = name + "=";
    if (arg.rfind(prefix, 0) == 0) {
        return parse_bool(arg.substr(prefix.size()), out);
    }
    return false;
}

std::string to_upper(std::string input) {
    for (char &c : input) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return input;
}

bool parse_hex_u16(const std::string &text, uint16_t *value) {
    try {
        size_t idx = 0;
        int base = 16;
        if (text.rfind("0x", 0) == 0 || text.rfind("0X", 0) == 0) {
            base = 0;
        }
        unsigned long raw = std::stoul(text, &idx, base);
        if (idx != text.size() || raw > 0xFFFF) {
            return false;
        }
        *value = static_cast<uint16_t>(raw);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_vid_pid(const std::string &hardware_id, uint16_t *vid, uint16_t *pid) {
    const std::string upper = to_upper(hardware_id);
    const std::string win_key = "USB\\VID_";
    const std::string linux_key = "VID:PID=";
    if (upper.rfind(win_key, 0) == 0) {
        if (upper.size() < win_key.size() + 13) {
            return false;
        }
        const std::string vid_str = upper.substr(win_key.size(), 4);
        const std::string pid_str = upper.substr(win_key.size() + 9, 4);
        return parse_hex_u16(vid_str, vid) && parse_hex_u16(pid_str, pid);
    }
    const auto pos = upper.find(linux_key);
    if (pos != std::string::npos && pos + linux_key.size() + 9 <= upper.size()) {
        const std::string vid_str = upper.substr(pos + linux_key.size(), 4);
        const std::string pid_str = upper.substr(pos + linux_key.size() + 5, 4);
        return parse_hex_u16(vid_str, vid) && parse_hex_u16(pid_str, pid);
    }
    return false;
}

std::optional<serial::PortInfo> find_port() {
    std::vector<serial::PortInfo> ports = serial::list_ports();
    if (!g_options.port.empty()) {
        for (const auto &port : ports) {
            if (port.port == g_options.port) {
                return port;
            }
        }
        return std::nullopt;
    }

    for (const auto &port : ports) {
        if (port.hardware_id.empty()) {
            continue;
        }
        uint16_t vid = 0;
        uint16_t pid = 0;
        if (!parse_vid_pid(port.hardware_id, &vid, &pid)) {
            continue;
        }
        if (vid == g_options.vid && pid == g_options.pid) {
            return port;
        }
    }
    return std::nullopt;
}

class DeviceSerialTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        auto port = find_port();
        if (!port.has_value()) {
            skip_all_ = true;
            return;
        }

        try {
            serial_ = std::make_unique<serial::Serial>(
                port->port,
                g_options.baud,
                serial::Timeout::simpleTimeout(2000),
                serial::bytesize_t::eightbits,
                serial::parity_t::parity_none,
                serial::stopbits_t::stopbits_one,
                serial::flowcontrol_t::flowcontrol_none,
                serial::dtrcontrol_t::dtr_enable,
                serial::rtscontrol_t::rts_enable);
        } catch (const serial::SerialException &ex) {
            std::cerr << "Serial open failed: " << ex.what() << "\n";
            skip_all_ = true;
            return;
        }

        if (!serial_ || !serial_->isOpen()) {
            skip_all_ = true;
            return;
        }

        sample_queue_ = std::make_unique<powermonitor::client::SampleQueue>();
        response_queue_ = std::make_unique<powermonitor::client::ResponseQueue>();
        stats_ = std::make_unique<powermonitor::client::ThreadStats>();
        read_thread_ = std::make_unique<powermonitor::client::ReadThread>(
            serial_.get(), sample_queue_.get(), response_queue_.get(), &stop_flag_, stats_.get());
        read_thread_->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    static void TearDownTestSuite() {
        if (read_thread_) {
            read_thread_->join();
        }
        if (response_queue_) {
            response_queue_->stop();
        }
        if (sample_queue_) {
            sample_queue_->stop();
        }
        if (serial_ && serial_->isOpen()) {
            serial_->close();
        }
    }

    void SetUp() override {
        if (skip_all_) {
            GTEST_SKIP() << "Device not available";
        }
        if (g_abort_requested.load()) {
            GTEST_SKIP() << "Aborted after earlier failure";
        }
        risky_test_ = false;
        streaming_active_ = false;
        DrainInput();
    }

    void TearDown() override {
        if (streaming_active_) {
            StopStreamBestEffort();
        }
        DrainInput();
        if (g_options.abort_on_fail && risky_test_ && HasFailure()) {
            g_abort_requested.store(true);
        }
    }

    void SetRisky(bool risky) { risky_test_ = risky; }

    void DrainInput() {
        protocol::DynamicFrame frame;
        while (response_queue_ && response_queue_->pop_wait(frame, 0)) {
        }
        if (sample_queue_) {
            powermonitor::client::SampleQueue::Sample sample;
            while (sample_queue_->pop(sample)) {
            }
        }
    }

    bool SendCommand(uint8_t msgid, const std::vector<uint8_t> &payload,
                     protocol::DynamicFrame *rsp = nullptr) {
        constexpr int kMaxRetries = 3;
        for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
            const uint8_t seq = cmd_seq_++;
            const std::vector<uint8_t> frame =
                protocol::build_frame(protocol::FrameType::kCmd, 0, seq, msgid, payload);
            try {
                serial_->write(frame);
            } catch (const serial::SerialException &ex) {
                std::cerr << "Serial write failed: " << ex.what() << "\n";
                return false;
            }

            protocol::DynamicFrame response;
            if (WaitForRsp(seq, &response, g_options.cmd_timeout_ms)) {
                // Per docs/protocol/uart_protocol.md:
                // RSP payload starts with { orig_msgid, status, ... }.
                if (response.data.size() < 2) {
                    return false;
                }
                if (response.data[0] != msgid) {
                    return false;
                }
                if (response.data[1] != 0x00) {
                    return false;
                }
                if (rsp) {
                    *rsp = response;
                }
                return true;
            }
        }
        return false;
    }

    bool WaitForRsp(uint8_t seq, protocol::DynamicFrame *out, int timeout_ms) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            const int remaining = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now())
                    .count());
            protocol::DynamicFrame frame;
            if (!response_queue_->pop_wait(frame, std::max(remaining, 0))) {
                continue;
            }
            if (frame.type != protocol::FrameType::kRsp) {
                continue;
            }
            if (frame.seq != seq) {
                continue;
            }
            if (out) {
                *out = frame;
            }
            return true;
        }
        return false;
    }

    bool WaitForEvent(uint8_t msgid, protocol::DynamicFrame *out, int timeout_ms) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            const int remaining = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now())
                    .count());
            protocol::DynamicFrame frame;
            if (!response_queue_->pop_wait(frame, std::max(remaining, 0))) {
                continue;
            }
            if (frame.type != protocol::FrameType::kEvt || frame.msgid != msgid) {
                continue;
            }
            if (out) {
                *out = frame;
            }
            return true;
        }
        return false;
    }

    bool StartStream(uint16_t period_us = 1000, uint16_t mask = 0x000F) {
        std::vector<uint8_t> payload(4);
        powermonitor::client::pack_u16(payload.data(), period_us);
        powermonitor::client::pack_u16(payload.data() + 2, mask);
        if (!SendCommand(kMsgStreamStart, payload)) {
            return false;
        }
        streaming_active_ = true;
        return true;
    }

    bool StopStream() {
        if (!SendCommand(kMsgStreamStop, {})) {
            return false;
        }
        streaming_active_ = false;
        return true;
    }

    bool WaitForNoSamples(std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            powermonitor::client::SampleQueue::Sample sample;
            if (sample_queue_ && sample_queue_->pop(sample)) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return true;
    }

    void StopStreamBestEffort() {
        SendCommand(kMsgStreamStop, {});
        streaming_active_ = false;
    }

    bool WaitForSamples(size_t target, std::chrono::milliseconds timeout, size_t *received = nullptr) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        size_t count = 0;
        while (std::chrono::steady_clock::now() < deadline && count < target) {
            powermonitor::client::SampleQueue::Sample sample;
            if (sample_queue_->pop(sample)) {
                ++count;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (received) {
            *received = count;
        }
        return count >= target;
    }

    void InjectRawFrame(const std::vector<uint8_t> &frame) {
        if (!serial_) {
            return;
        }
        serial_->write(frame);
    }

    static std::unique_ptr<serial::Serial> serial_;
    static std::unique_ptr<powermonitor::client::SampleQueue> sample_queue_;
    static std::unique_ptr<powermonitor::client::ResponseQueue> response_queue_;
    static std::unique_ptr<powermonitor::client::ThreadStats> stats_;
    static std::unique_ptr<powermonitor::client::ReadThread> read_thread_;
    static std::atomic<bool> stop_flag_;
    static std::atomic<uint8_t> cmd_seq_;
    static bool skip_all_;

    bool streaming_active_ = false;
    bool risky_test_ = false;
};

std::unique_ptr<serial::Serial> DeviceSerialTest::serial_;
std::unique_ptr<powermonitor::client::SampleQueue> DeviceSerialTest::sample_queue_;
std::unique_ptr<powermonitor::client::ResponseQueue> DeviceSerialTest::response_queue_;
std::unique_ptr<powermonitor::client::ThreadStats> DeviceSerialTest::stats_;
std::unique_ptr<powermonitor::client::ReadThread> DeviceSerialTest::read_thread_;
std::atomic<bool> DeviceSerialTest::stop_flag_{false};
std::atomic<uint8_t> DeviceSerialTest::cmd_seq_{0};
bool DeviceSerialTest::skip_all_ = false;

TEST_F(DeviceSerialTest, Order01_PingSmoke) {
    ASSERT_TRUE(SendCommand(kMsgPing, {}));
}

TEST_F(DeviceSerialTest, Order02_GetCfgReport) {
    ASSERT_TRUE(SendCommand(kMsgGetCfg, {}));
    protocol::DynamicFrame cfg_report;
    ASSERT_TRUE(WaitForEvent(kMsgCfgReport, &cfg_report, g_options.event_timeout_ms));
    EXPECT_GE(cfg_report.data.size(), 16u);
}

TEST_F(DeviceSerialTest, Order02b_TextReport) {
    protocol::DynamicFrame text_report;
    ASSERT_TRUE(WaitForEvent(kMsgTextReport, &text_report, g_options.event_timeout_ms));
    EXPECT_GE(text_report.data.size(), 1u);
    EXPECT_LE(text_report.data.size(), 4096u);
}

TEST_F(DeviceSerialTest, Order03_SetCfgMinimal) {
    std::vector<uint8_t> payload(8, 0);
    powermonitor::client::pack_u16(payload.data(), 0x0000);
    powermonitor::client::pack_u16(payload.data() + 2, 0x1000);
    powermonitor::client::pack_u16(payload.data() + 4, 0x1000);
    powermonitor::client::pack_u16(payload.data() + 6, 0x0000);
    ASSERT_TRUE(SendCommand(kMsgSetCfg, payload));
    protocol::DynamicFrame cfg_report;
    ASSERT_TRUE(WaitForEvent(kMsgCfgReport, &cfg_report, g_options.event_timeout_ms));
    EXPECT_GE(cfg_report.data.size(), 16u);
}

TEST_F(DeviceSerialTest, Order04_RegReadConfig) {
    std::vector<uint8_t> payload;
    payload.push_back(0x40);
    payload.push_back(0x00);
    payload.push_back(0x00);
    protocol::DynamicFrame rsp;
    ASSERT_TRUE(SendCommand(kMsgRegRead, payload, &rsp));
    ASSERT_GE(rsp.data.size(), 5u);
    EXPECT_EQ(rsp.data[0], kMsgRegRead);
    EXPECT_EQ(rsp.data[1], 0x00);
    EXPECT_EQ(rsp.data[2], 0x00);
}

TEST_F(DeviceSerialTest, Order05_BadFrameCRC) {
    std::vector<uint8_t> frame =
        protocol::build_frame(protocol::FrameType::kCmd, 0, cmd_seq_++, kMsgPing, {});
    frame.back() ^= 0xFF;
    InjectRawFrame(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(SendCommand(kMsgPing, {}));
}

TEST_F(DeviceSerialTest, Order06_BadFrameLenTooLarge) {
    std::vector<uint8_t> payload(1024, 0x00);
    std::vector<uint8_t> frame =
        protocol::build_frame(protocol::FrameType::kCmd, 0, cmd_seq_++, kMsgPing, payload);
    InjectRawFrame(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(SendCommand(kMsgPing, {}));
}

TEST_F(DeviceSerialTest, Order07_IdleCmd) {
    ASSERT_TRUE(SendCommand(kMsgGetCfg, {}));
}

TEST_F(DeviceSerialTest, Order08_StreamStartStopBasic) {
    SetRisky(true);
    ASSERT_TRUE(StartStream());
    ASSERT_TRUE(WaitForSamples(3, std::chrono::milliseconds(2000)));
    ASSERT_TRUE(StopStream());

    // After STOP, device should not continue streaming. Drain buffered samples then
    // verify that no new samples arrive for 1 second.
    DrainInput();
    ASSERT_TRUE(WaitForNoSamples(std::chrono::milliseconds(1000)));
}

TEST_F(DeviceSerialTest, Order09_StreamMidCommand) {
    SetRisky(true);
    ASSERT_TRUE(StartStream());
    ASSERT_TRUE(WaitForSamples(3, std::chrono::milliseconds(1500)));
    ASSERT_TRUE(SendCommand(kMsgGetCfg, {}));
    protocol::DynamicFrame cfg_report;
    ASSERT_TRUE(WaitForEvent(kMsgCfgReport, &cfg_report, g_options.event_timeout_ms));
    ASSERT_TRUE(WaitForSamples(3, std::chrono::milliseconds(1500)));
    ASSERT_TRUE(StopStream());
}

TEST_F(DeviceSerialTest, Order10_StreamStopIdempotent) {
    SetRisky(true);
    ASSERT_TRUE(StartStream());
    ASSERT_TRUE(StopStream());
    ASSERT_TRUE(SendCommand(kMsgStreamStop, {}));
}

TEST_F(DeviceSerialTest, Order11_StreamBadFrameDuringStream) {
    SetRisky(true);
    ASSERT_TRUE(StartStream());
    ASSERT_TRUE(WaitForSamples(3, std::chrono::milliseconds(1500)));
    std::vector<uint8_t> frame =
        protocol::build_frame(protocol::FrameType::kCmd, 0, cmd_seq_++, kMsgPing, {});
    frame.back() ^= 0x5A;
    InjectRawFrame(frame);
    ASSERT_TRUE(WaitForSamples(3, std::chrono::milliseconds(1500)));
    ASSERT_TRUE(StopStream());
}

TEST_F(DeviceSerialTest, Order12_StreamLongRunSoak) {
    SetRisky(true);
    ASSERT_TRUE(StartStream());
    const auto duration = std::chrono::seconds(g_options.soak_duration_s);
    const auto deadline = std::chrono::steady_clock::now() + duration;
    size_t samples = 0;
    size_t drops = 0;
    bool have_seq = false;
    uint8_t last_seq = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        powermonitor::client::SampleQueue::Sample sample;
        if (!sample_queue_->pop(sample)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (have_seq) {
            const uint8_t expected = static_cast<uint8_t>(last_seq + 1);
            if (sample.seq != expected) {
                drops += static_cast<uint8_t>(sample.seq - expected);
            }
        } else {
            have_seq = true;
        }
        last_seq = sample.seq;
        ++samples;
    }

    RecordProperty("samples", static_cast<int>(samples));
    RecordProperty("drops", static_cast<int>(drops));
    ASSERT_GT(samples, 1000u);
    ASSERT_TRUE(StopStream());
}

// ---------------------------------------------------------------------------
// Stress: STREAM_START/STOP wedge hunt (DISABLED_ — run explicitly)
// ---------------------------------------------------------------------------
// Field report (2026-07-01): repeatedly toggling stream start/stop in the
// TUI once killed the sample flow permanently ("Flush failed" on the host);
// only a full power cycle — which also power-cycles the separately-supplied
// INA228 — recovered it, and it has not reproduced since. This test
// automates the hunt with two attack phases and pinpointed health checks so
// a recurrence yields a diagnosis (command path dead vs sampler dead)
// instead of a dead board.
//
// Deliberately destructive in intent, so it is DISABLED_ and excluded from
// the ordered suite. Run it explicitly:
//   pc_client_tests --port /dev/ttyACMx \
//     --gtest_also_run_disabled_tests --gtest_filter='*WedgeHunt*'
//
// Determinism: dwell/gap sweeps use fixed multiplicative sequences, not
// randomness, per the repo rule on deterministic test seeds.
TEST_F(DeviceSerialTest, DISABLED_Stress_StreamStartStopWedgeHunt) {
    SetRisky(true);

    constexpr int kRapidCycles = 100;
    constexpr int kSyncRaceCycles = 12;
    constexpr int kHealthCheckEvery = 10;

    // Distinguishes "whole device dead" (PING fails -> Core 0 wedged, e.g.
    // multicore FIFO saturation) from "sampler dead" (PING ok but no
    // samples -> Core 1 / PIO / INA228 path, e.g. stuck bus).
    auto require_alive = [&](const char *phase, int cycle) {
        ASSERT_TRUE(SendCommand(kMsgPing, {}))
            << phase << " cycle " << cycle
            << ": PING dead — Core 0 wedged (command path unresponsive)";
    };

    // ---- Phase A: rapid toggling ----
    // Dwell 40..160 ms, gap 0..49 ms, both swept deterministically.
    int zero_sample_streak = 0;
    for (int i = 0; i < kRapidCycles; ++i) {
        ASSERT_TRUE(StartStream())
            << "Phase A cycle " << i << ": STREAM_START not acknowledged";
        const int dwell_ms = 40 + (i * 13) % 120;
        size_t received = 0;
        WaitForSamples(1, std::chrono::milliseconds(dwell_ms), &received);
        if (received == 0 && dwell_ms >= 100) {
            ++zero_sample_streak;
            ASSERT_LT(zero_sample_streak, 3)
                << "Phase A cycle " << i << ": 3 consecutive cycles with zero "
                << "samples while commands still work — sampler/INA228 path "
                << "dead (stuck-bus signature). Do NOT power cycle; capture "
                << "STATS_REPORT drop counters first.";
        } else if (received > 0) {
            zero_sample_streak = 0;
        }
        ASSERT_TRUE(StopStream())
            << "Phase A cycle " << i << ": STREAM_STOP not acknowledged";
        std::this_thread::sleep_for(std::chrono::milliseconds((i * 7) % 50));
        if (i % kHealthCheckEvery == kHealthCheckEvery - 1) {
            require_alive("Phase A", i);
            DrainInput();
        }
    }
    DrainInput();

    // ---- Phase B: toggle burst inside the sync_waiting window ----
    // EVT_TIME_SYNC_REQUEST fires on WALL time (time_us_64), evaluated only
    // while streaming: immediately on the first streaming iteration after
    // boot, then every kTimeSyncRequestPeriodUs = 120 s. After it fires the
    // device sits in a tight sync_waiting loop for up to 200 ms waiting for
    // TIME_ADJUST. This harness never answers, streams until the EVT
    // arrives (<= one full period), then hammers STOP/START inside the
    // window — the race a TUI user hits pressing 't' right after starting
    // a stream that followed a >2 min idle gap.
    ASSERT_TRUE(StartStream()) << "Phase B: STREAM_START not acknowledged";
    protocol::DynamicFrame evt;
    const bool got_evt = WaitForEvent(kMsgTimeSyncRequest, &evt, 125000);
    RecordProperty("sync_race_evt_hit", got_evt ? 1 : 0);
    EXPECT_TRUE(got_evt)
        << "Phase B never observed EVT_TIME_SYNC_REQUEST within a full "
        << "period — check kTimeSyncRequestPeriodUs/streaming state";
    for (int k = 0; k < kSyncRaceCycles; ++k) {
        ASSERT_TRUE(StopStream())
            << "Phase B burst " << k << ": STREAM_STOP not acknowledged"
            << (got_evt && k == 0 ? " (inside sync_waiting window)" : "");
        std::this_thread::sleep_for(std::chrono::milliseconds((k * 23) % 60));
        ASSERT_TRUE(StartStream())
            << "Phase B burst " << k << ": STREAM_START not acknowledged";
    }
    ASSERT_TRUE(StopStream()) << "Phase B: final STREAM_STOP not acknowledged";
    require_alive("Phase B", kSyncRaceCycles);
    DrainInput();

    // ---- Final: full functional check ----
    // Commands alive AND the sampler actually produces data at rate.
    require_alive("Final", 0);
    ASSERT_TRUE(StartStream());
    ASSERT_TRUE(WaitForSamples(100, std::chrono::milliseconds(2000)))
        << "Final check: sampler did not deliver 100 samples in 2 s after "
        << "the stress — sample path degraded even though commands work";
    ASSERT_TRUE(StopStream());
    DrainInput();
    ASSERT_TRUE(WaitForNoSamples(std::chrono::milliseconds(1000)))
        << "Final check: device kept streaming after STREAM_STOP";
}

}  // namespace

int main(int argc, char **argv) {
    std::vector<char *> gtest_args;
    gtest_args.push_back(argv[0]);

    for (int i = 1; i < argc; ++i) {
        if (consume_string_arg(i, argc, argv, "--port", &g_options.port)) {
            continue;
        }
        if (consume_uint32_arg(i, argc, argv, "--baud", &g_options.baud)) {
            continue;
        }
        if (consume_int_arg(i, argc, argv, "--cmd_timeout_ms", &g_options.cmd_timeout_ms)) {
            continue;
        }
        if (consume_int_arg(i, argc, argv, "--event_timeout_ms", &g_options.event_timeout_ms)) {
            continue;
        }
        if (consume_int_arg(i, argc, argv, "--soak_duration_s", &g_options.soak_duration_s)) {
            continue;
        }
        if (consume_bool_arg(i, argc, argv, "--abort_on_fail", &g_options.abort_on_fail)) {
            continue;
        }
        gtest_args.push_back(argv[i]);
    }

    int gtest_argc = static_cast<int>(gtest_args.size());
    ::testing::InitGoogleTest(&gtest_argc, gtest_args.data());
    return RUN_ALL_TESTS();
}
