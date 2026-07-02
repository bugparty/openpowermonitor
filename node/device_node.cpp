#include "node/device_node.h"

#include <cstring>
#include <iostream>

#include "protocol/serialization.h"

namespace node {

namespace {

constexpr uint8_t kMsgPing = 0x01;
constexpr uint8_t kMsgTimeSync = 0x05;
constexpr uint8_t kMsgTimeAdjust = 0x06;
constexpr uint8_t kMsgTimeSet = 0x07;
constexpr uint8_t kMsgSetCfg = 0x10;
constexpr uint8_t kMsgGetCfg = 0x11;
constexpr uint8_t kMsgStreamStart = 0x30;
constexpr uint8_t kMsgStreamStop = 0x31;
constexpr uint8_t kMsgDataSample = 0x80;
constexpr uint8_t kMsgCfgReport = 0x91;
constexpr uint8_t kMsgTextReport = 0x93;
constexpr uint8_t kStatusOk = 0x00;


} // namespace

DeviceNode::DeviceNode(sim::VirtualLinkEndpoint *endpoint)
    : endpoint_(endpoint),
      parser_([this](const protocol::DynamicFrame &frame, uint64_t receive_time_us) {
          on_frame(frame, receive_time_us);
      }) {}

void DeviceNode::tick(uint64_t now_us) {
    current_now_us_ = now_us;
    if (!initial_cfg_sent_) {
        send_cfg_report(now_us);
        initial_cfg_sent_ = true;
    }
    if (!text_report_sent_) {
        static constexpr char kText[] = "device online";
        send_text_report(kText, sizeof(kText) - 1, now_us);
        text_report_sent_ = true;
    }
    if (endpoint_ && endpoint_->available() > 0) {
        // Set receive time before feeding data to parser
        // Use last_receive_time() which is the actual delivery time from VirtualLink
        const uint64_t receive_time = endpoint_->last_receive_time();
        parser_.set_receive_time(receive_time > 0 ? receive_time : now_us);
        auto bytes = endpoint_->read(endpoint_->available());
        parser_.feed(bytes);
    }

    if (streaming_on_ && now_us >= next_sample_due_us_) {
        send_data_sample(now_us);
        next_sample_due_us_ += stream_period_us_;
    }
}

void DeviceNode::on_frame(const protocol::DynamicFrame &frame, uint64_t receive_time_us) {
    ++rx_counts_[frame.msgid];
    if (frame.type == protocol::FrameType::kCmd) {
        // Use receive_time_us for accurate T2 capture in time sync
        handle_cmd(frame, receive_time_us);
    }
}

void DeviceNode::handle_cmd(const protocol::DynamicFrame &frame, uint64_t now_us) {
    switch (frame.msgid) {
    case kMsgPing: {
        send_rsp(frame.seq, frame.msgid, kStatusOk, {}, now_us);
        break;
    }
    case kMsgTimeSync: {
        // TIME_SYNC: Extract T1, capture T2 (immediately after parse), capture T3 (before send)
        if (frame.data.size() >= 8) {
            const uint64_t T1 = protocol::read_u64(frame.data, 0);
            // T2 captured immediately after frame parsing completes.
            // Like the firmware (device/command_handler.hpp handle_time_sync),
            // T2/T3 are reported in the device's Unix domain: mono + epoch_offset.
            // now_us is the simulated monotonic clock; epoch_offset_us_ carries
            // the device clock error, so corrections via TIME_ADJUST are
            // observable in subsequent exchanges.
            const uint64_t T2 = now_us + static_cast<uint64_t>(epoch_offset_us_);

            // Build RSP payload: orig_msgid(1) + status(1) + T1(8) + T2(8) + T3(8)
            // Prepare all data except T3 first
            std::vector<uint8_t> rsp_data;
            protocol::append_u64(rsp_data, T1);
            protocol::append_u64(rsp_data, T2);

            // All processing is done, capture T3 immediately before sending
            // In real implementation, this should be captured just before the serial write
            const uint64_t T3 = now_us + static_cast<uint64_t>(epoch_offset_us_);
            protocol::append_u64(rsp_data, T3);

            send_rsp(frame.seq, frame.msgid, kStatusOk, rsp_data, now_us);
        } else {
            send_rsp(frame.seq, frame.msgid, 0x02, {}, now_us);  // ERR_LEN
        }
        break;
    }
    case kMsgTimeAdjust: {
        // TIME_ADJUST: Apply clock offset correction
        if (frame.data.size() >= 8) {
            const int64_t offset_us = protocol::read_i64(frame.data, 0);
            epoch_offset_us_ += offset_us;
            send_rsp(frame.seq, frame.msgid, kStatusOk, {}, now_us);
        } else {
            send_rsp(frame.seq, frame.msgid, 0x02, {}, now_us);  // ERR_LEN
        }
        break;
    }
    case kMsgTimeSet: {
        // TIME_SET: Set absolute Unix time
        if (frame.data.size() >= 8) {
            const uint64_t unix_time_us = protocol::read_u64(frame.data, 0);
            // Set epoch_offset such that: unix_time = monotonic + epoch_offset
            // In simulation, we use now_us as the monotonic time
            epoch_offset_us_ = static_cast<int64_t>(unix_time_us) - static_cast<int64_t>(now_us);
            send_rsp(frame.seq, frame.msgid, kStatusOk, {}, now_us);
        } else {
            send_rsp(frame.seq, frame.msgid, 0x02, {}, now_us);  // ERR_LEN
        }
        break;
    }
    case kMsgSetCfg: {
        if (frame.data.size() >= 8) {
            config_reg_ = protocol::read_u16(frame.data, 0);
            adc_config_reg_ = protocol::read_u16(frame.data, 2);
            shunt_cal_reg_ = protocol::read_u16(frame.data, 4);
            shunt_tempco_ = protocol::read_u16(frame.data, 6);
            current_lsb_nA_ = shunt_cal_reg_ == 0 ? 1000 : current_lsb_nA_;
        }
        send_rsp(frame.seq, frame.msgid, kStatusOk, {}, now_us);
        send_cfg_report(now_us);
        break;
    }
    case kMsgGetCfg: {
        send_rsp(frame.seq, frame.msgid, kStatusOk, {}, now_us);
        send_cfg_report(now_us);
        break;
    }
    case kMsgStreamStart: {
        if (frame.data.size() >= 4) {
            stream_period_us_ = protocol::read_u16(frame.data, 0);
            stream_mask_ = protocol::read_u16(frame.data, 2);
            streaming_on_ = true;
            stream_start_us_ = now_us;
            next_sample_due_us_ = now_us + stream_period_us_;
        }
        send_rsp(frame.seq, frame.msgid, kStatusOk, {}, now_us);
        break;
    }
    case kMsgStreamStop: {
        streaming_on_ = false;
        send_rsp(frame.seq, frame.msgid, kStatusOk, {}, now_us);
        break;
    }
    default:
        send_rsp(frame.seq, frame.msgid, 0x03, {}, now_us);
        break;
    }
}

void DeviceNode::send_rsp(uint8_t seq, uint8_t orig_msgid, uint8_t status,
                          const std::vector<uint8_t> &data, uint64_t now_us) {
    std::vector<uint8_t> payload;
    payload.push_back(orig_msgid);
    payload.push_back(status);
    payload.insert(payload.end(), data.begin(), data.end());
    auto bytes = protocol::build_frame(protocol::FrameType::kRsp, 0, seq, orig_msgid, payload);
    endpoint_->write(bytes, now_us);
}

void DeviceNode::send_cfg_report(uint64_t now_us) {
    std::vector<uint8_t> payload;
    payload.push_back(protocol::kProtoVersion);
    uint8_t flags = 0;
    if (streaming_on_) {
        flags |= 0x01;
    }
    if (shunt_cal_reg_ != 0) {
        flags |= 0x02;
    }
    if (adcrange_) {
        flags |= 0x04;
    }
    payload.push_back(flags);
    protocol::append_u32(payload, current_lsb_nA_);
    protocol::append_u16(payload, shunt_cal_reg_);
    protocol::append_u16(payload, config_reg_);
    protocol::append_u16(payload, adc_config_reg_);
    protocol::append_u16(payload, stream_period_us_);
    protocol::append_u16(payload, stream_mask_);

    const uint8_t seq = data_seq_++;
    auto bytes = protocol::build_frame(protocol::FrameType::kEvt, 0, seq, kMsgCfgReport, payload);
    endpoint_->write(bytes, now_us);
    std::cout << "DEV CFG_REPORT sent\n";
}

void DeviceNode::send_text_report(const char *text, size_t text_len, uint64_t now_us) {
    if (text == nullptr || text_len == 0 || text_len > 4096) {
        return;
    }
    std::vector<uint8_t> payload(text, text + text_len);
    const uint8_t seq = data_seq_++;
    auto bytes = protocol::build_frame(protocol::FrameType::kEvt, 0, seq, kMsgTextReport, payload);
    endpoint_->write(bytes, now_us);
}

void DeviceNode::send_data_sample(uint64_t now_us) {
    const auto raw = model_.sample(now_us, current_lsb_nA_, adcrange_);
    std::vector<uint8_t> payload;
    // Wire order matches DataSamplePayload: timestamp_unix_us first, then timestamp_us
    const uint64_t timestamp_unix = now_us + epoch_offset_us_;
    protocol::append_u64(payload, timestamp_unix);              // offset 0-7: timestamp_unix_us
    const uint64_t timestamp = now_us - stream_start_us_;
    protocol::append_u64(payload, timestamp);                   // offset 8-15: timestamp_us (64-bit)
    uint8_t flags = 0;
    flags |= 0x01;
    if (shunt_cal_reg_ != 0) {
        flags |= 0x04;
    }
    payload.push_back(flags);                                   // offset 16: flags

    uint8_t buf[3] = {};
    protocol::pack_u20(raw.vbus_raw, buf);
    payload.insert(payload.end(), buf, buf + 3);                // offset 17-19: vbus20
    protocol::pack_s20(raw.vshunt_raw, buf);
    payload.insert(payload.end(), buf, buf + 3);                // offset 20-22: vshunt20
    protocol::pack_s20(raw.current_raw, buf);
    payload.insert(payload.end(), buf, buf + 3);                // offset 23-25: current20
    payload.insert(payload.end(), 3, 0);                        // offset 26-28: power24 (not modelled, zeros)
    protocol::append_u16(payload, static_cast<uint16_t>(raw.temp_raw)); // offset 29-30: temp16
    payload.insert(payload.end(), 5, 0);                        // offset 31-35: energy40 (not modelled, zeros)
    payload.insert(payload.end(), 5, 0);                        // offset 36-40: charge40 (not modelled, zeros)

    const uint8_t seq = data_seq_++;
    auto bytes = protocol::build_frame(protocol::FrameType::kData, 0, seq, kMsgDataSample, payload);
    endpoint_->write(bytes, now_us);
}

} // namespace node
