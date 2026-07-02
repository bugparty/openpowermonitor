// E2E shutdown tests for PowerMonitorSession over a PTY pair (no hardware).
//
// A fake device thread owns the PTY master side and speaks the real UART
// protocol (protocol::Parser + protocol::build_frame), recording every CMD
// it receives; a real PowerMonitorSession runs against the PTY slave path
// exactly as it would against /dev/ttyACM0.
//
// Regression target: the session's shutdown path must actually transmit
// STREAM_STOP. Historically, run() set stop_requested_ before calling
// stop_streaming(), and send_command_with_retry()/wait_for_response()/
// ReadThread all bail out once that flag is set — so the device kept
// streaming after every normal exit.

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "power_monitor_session.h"
#include "protocol/frame_builder.h"
#include "protocol/parser.h"
#include "protocol/protocol_constants.h"
#include "protocol/serialization.h"

namespace {

using protocol::FrameType;
using protocol::MsgId;

// Fake power monitor device on the master end of a PTY pair. Responds to
// every command with RSP(OK) (TIME_SYNC gets a well-formed T1/T2/T3 payload,
// GET_CFG is followed by a CFG_REPORT) and records the order of received
// command MSGIDs for later assertions.
class FakeDevice {
public:
    FakeDevice()
        : parser_([this](const protocol::DynamicFrame &frame, uint64_t) { on_frame(frame); }) {
        master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
            return;
        }
        char path[128] = {};
        if (::ptsname_r(master_fd_, path, sizeof(path)) != 0) {
            return;
        }
        slave_path_ = path;

        // Raw mode so the PTY line discipline does not mangle binary frames.
        termios tio{};
        if (::tcgetattr(master_fd_, &tio) == 0) {
            ::cfmakeraw(&tio);
            ::tcsetattr(master_fd_, TCSANOW, &tio);
        }

        running_ = true;
        reader_ = std::thread([this] { reader_loop(); });
    }

    ~FakeDevice() { stop(); }

    bool ok() const { return master_fd_ >= 0 && !slave_path_.empty(); }
    const std::string &slave_path() const { return slave_path_; }

    void stop() {
        running_ = false;
        if (reader_.joinable()) {
            reader_.join();
        }
        if (master_fd_ >= 0) {
            ::close(master_fd_);
            master_fd_ = -1;
        }
    }

    std::vector<uint8_t> received_cmds() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return received_cmds_;
    }

    size_t count(MsgId msgid) const {
        const uint8_t id = static_cast<uint8_t>(msgid);
        size_t n = 0;
        for (uint8_t cmd : received_cmds()) {
            if (cmd == id) {
                ++n;
            }
        }
        return n;
    }

    // True if a STREAM_STOP was received after the last STREAM_START —
    // i.e. the session's shutdown stop, as opposed to the init-time stop
    // that precedes streaming.
    bool stream_stop_after_last_start() const {
        const auto cmds = received_cmds();
        size_t last_start = cmds.size();
        for (size_t i = 0; i < cmds.size(); ++i) {
            if (cmds[i] == static_cast<uint8_t>(MsgId::kStreamStart)) {
                last_start = i;
            }
        }
        if (last_start == cmds.size()) {
            return false;  // Never started streaming
        }
        for (size_t i = last_start + 1; i < cmds.size(); ++i) {
            if (cmds[i] == static_cast<uint8_t>(MsgId::kStreamStop)) {
                return true;
            }
        }
        return false;
    }

private:
    void reader_loop() {
        while (running_) {
            pollfd pfd{master_fd_, POLLIN, 0};
            const int rc = ::poll(&pfd, 1, 20);
            if (rc <= 0) {
                continue;
            }
            uint8_t buf[512];
            const ssize_t n = ::read(master_fd_, buf, sizeof(buf));
            if (n > 0) {
                parser_.feed(buf, static_cast<size_t>(n));
            } else if (n < 0 && errno == EIO) {
                // Slave end not open (yet, or anymore) — keep waiting.
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    void on_frame(const protocol::DynamicFrame &frame) {
        if (frame.type != FrameType::kCmd) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            received_cmds_.push_back(frame.msgid);
        }

        // RSP payload = [orig_msgid, status, extra...]
        std::vector<uint8_t> rsp;
        rsp.push_back(frame.msgid);
        rsp.push_back(0x00);  // kOk

        if (frame.msgid == static_cast<uint8_t>(MsgId::kTimeSync) && frame.data.size() >= 8) {
            // Echo T1 as T2/T3: a zero-offset device clock.
            const uint64_t t1 = protocol::read_u64(frame.data, 0);
            protocol::append_u64(rsp, t1);
            protocol::append_u64(rsp, t1);
            protocol::append_u64(rsp, t1);
        }

        write_frame(protocol::build_frame(FrameType::kRsp, 0, frame.seq, frame.msgid, rsp));

        if (frame.msgid == static_cast<uint8_t>(MsgId::kGetCfg)) {
            send_cfg_report();
        }
    }

    void send_cfg_report() {
        std::vector<uint8_t> payload;
        payload.push_back(protocol::kProtoVersion);       // proto_ver
        payload.push_back(0x00);                          // flags (ADCRANGE=0)
        protocol::append_u32(payload, 1000);              // current_lsb_nA
        protocol::append_u16(payload, 0x1000);            // shunt_cal
        protocol::append_u16(payload, 0x0000);            // config_reg
        protocol::append_u16(payload, 0xFB68);            // adc_config_reg
        protocol::append_u16(payload, 1000);              // stream_period_us
        protocol::append_u16(payload, 0x000F);            // stream_mask
        write_frame(protocol::build_frame(
            FrameType::kEvt, 0, evt_seq_++, static_cast<uint8_t>(MsgId::kCfgReport), payload));
    }

    void write_frame(const std::vector<uint8_t> &bytes) {
        size_t written = 0;
        while (written < bytes.size()) {
            const ssize_t n = ::write(master_fd_, bytes.data() + written, bytes.size() - written);
            if (n <= 0) {
                return;
            }
            written += static_cast<size_t>(n);
        }
    }

    int master_fd_ = -1;
    std::string slave_path_;
    std::atomic<bool> running_{false};
    std::thread reader_;
    protocol::Parser parser_;
    uint8_t evt_seq_ = 0;
    mutable std::mutex mutex_;
    std::vector<uint8_t> received_cmds_;
};

}  // namespace

// A full session run (connect -> init -> stream -> duration deadline ->
// shutdown) must transmit STREAM_STOP to the device during shutdown.
// The init-time STREAM_STOP does not count: only a stop received AFTER
// STREAM_START proves the device was told to stop streaming.
TEST(SessionShutdownTest, StreamStopIsSentOnDurationShutdown) {
    FakeDevice device;
    ASSERT_TRUE(device.ok()) << "Failed to create PTY pair";

    powermonitor::client::PowerMonitorSession::Options options;
    options.port = device.slave_path();
    options.duration_us = 250'000;  // Auto-stop shortly after streaming starts
    options.interactive = false;

    powermonitor::client::PowerMonitorSession session(options);
    const int rc = session.run();

    device.stop();

    EXPECT_EQ(rc, 0) << "Session run failed — fake device handshake incomplete";
    ASSERT_EQ(device.count(MsgId::kStreamStart), 1u)
        << "Session never started streaming against the fake device";
    EXPECT_GE(device.count(MsgId::kStreamStop), 1u)
        << "No STREAM_STOP at all (not even the init-time one)";
    EXPECT_TRUE(device.stream_stop_after_last_start())
        << "STREAM_STOP was never transmitted during shutdown: the device is "
        << "left streaming after the session exits (stop_requested_ short-"
        << "circuits send_command_with_retry before stop_streaming runs)";
}
