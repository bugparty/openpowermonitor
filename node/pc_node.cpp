#include "node/pc_node.h"

#include <cstring>
#include <iostream>
#include <string>

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
constexpr uint64_t kCmdTimeoutUs = 200000;
constexpr uint8_t kMaxRetries = 3;

int64_t signed_diff_u64(uint64_t lhs, uint64_t rhs) {
    return static_cast<int64_t>(lhs) - static_cast<int64_t>(rhs);
}

} // namespace

PCNode::PCNode(sim::VirtualLinkEndpoint *endpoint)
    : endpoint_(endpoint),
      parser_([this](const protocol::DynamicFrame &frame, uint64_t receive_time_us) {
          on_frame(frame, receive_time_us);
      }) {}

void PCNode::tick(uint64_t now_us) {
    current_time_us_ = now_us;
    if (endpoint_ && endpoint_->available() > 0) {
        // Set receive time before feeding data to parser
        // Use last_receive_time() which is the actual delivery time from VirtualLink
        const uint64_t receive_time = endpoint_->last_receive_time();
        parser_.set_receive_time(receive_time > 0 ? receive_time : now_us);
        auto bytes = endpoint_->read(endpoint_->available());
        parser_.feed(bytes);
    }

    for (auto it = pending_.begin(); it != pending_.end();) {
        if (now_us >= it->second.deadline_us) {
            if (it->second.retries >= kMaxRetries) {
                ++timeout_count_;
                std::cout << "CMD timeout msgid=0x" << std::hex
                          << static_cast<int>(it->second.msgid) << std::dec << "\n";
                it = pending_.erase(it);
                continue;
            }
            ++retransmit_count_;
            it->second.retries++;
            it->second.deadline_us = now_us + kCmdTimeoutUs;
            endpoint_->write(it->second.bytes, now_us);
            std::cout << "CMD retry msgid=0x" << std::hex << static_cast<int>(it->second.msgid)
                      << " seq=" << static_cast<int>(it->second.seq) << std::dec << "\n";
        }
        ++it;
    }
}

void PCNode::send_ping(uint64_t now_us) { send_cmd(kMsgPing, {}, now_us); }

void PCNode::send_time_sync(uint64_t now_us) {
    std::vector<uint8_t> payload;
    protocol::append_u64(payload, now_us);  // T1 = send time
    time_sync_T1_ = now_us;
    send_cmd(kMsgTimeSync, payload, now_us);
}

void PCNode::send_time_adjust(int64_t offset_us, uint64_t now_us) {
    std::vector<uint8_t> payload;
    protocol::append_i64(payload, offset_us);
    send_cmd(kMsgTimeAdjust, payload, now_us);
}

void PCNode::send_time_set(uint64_t unix_time_us, uint64_t now_us) {
    std::vector<uint8_t> payload;
    protocol::append_u64(payload, unix_time_us);
    send_cmd(kMsgTimeSet, payload, now_us);
}

void PCNode::send_set_cfg(uint16_t config_reg, uint16_t adc_config_reg, uint16_t shunt_cal,
                          uint16_t shunt_tempco, uint64_t now_us) {
    std::vector<uint8_t> payload;
    protocol::append_u16(payload, config_reg);
    protocol::append_u16(payload, adc_config_reg);
    protocol::append_u16(payload, shunt_cal);
    protocol::append_u16(payload, shunt_tempco);
    send_cmd(kMsgSetCfg, payload, now_us);
}

void PCNode::send_get_cfg(uint64_t now_us) { send_cmd(kMsgGetCfg, {}, now_us); }

void PCNode::send_stream_start(uint16_t period_us, uint16_t mask, uint64_t now_us) {
    std::vector<uint8_t> payload;
    protocol::append_u16(payload, period_us);
    protocol::append_u16(payload, mask);
    send_cmd(kMsgStreamStart, payload, now_us);
}

void PCNode::send_stream_stop(uint64_t now_us) { send_cmd(kMsgStreamStop, {}, now_us); }

void PCNode::send_cmd(uint8_t msgid, const std::vector<uint8_t> &payload, uint64_t now_us) {
    const uint8_t seq = cmd_seq_++;
    auto bytes = protocol::build_frame(protocol::FrameType::kCmd, 0, seq, msgid, payload);
    endpoint_->write(bytes, now_us);
    PendingCmd pending;
    pending.msgid = msgid;
    pending.seq = seq;
    pending.deadline_us = now_us + kCmdTimeoutUs;
    pending.bytes = bytes;
    pending_[seq] = pending;
    std::cout << "CMD send msgid=0x" << std::hex << static_cast<int>(msgid)
              << " seq=" << static_cast<int>(seq) << std::dec << "\n";
}

void PCNode::on_frame(const protocol::DynamicFrame &frame, uint64_t receive_time_us) {
    ++rx_counts_[frame.msgid];
    if (frame.type == protocol::FrameType::kRsp) {
        handle_rsp(frame, receive_time_us);
        return;
    }
    if (frame.type == protocol::FrameType::kEvt && frame.msgid == kMsgCfgReport) {
        handle_cfg_report(frame);
        return;
    }
    if (frame.type == protocol::FrameType::kEvt && frame.msgid == kMsgTextReport) {
        handle_text_report(frame);
        return;
    }
    if (frame.type == protocol::FrameType::kData && frame.msgid == kMsgDataSample) {
        handle_data_sample(frame);
        return;
    }
}

void PCNode::handle_rsp(const protocol::DynamicFrame &frame, uint64_t receive_time_us) {
    if (frame.data.size() < 2) {
        return;
    }
    const uint8_t orig_msgid = frame.data[0];
    const uint8_t status = frame.data[1];
    auto it = pending_.find(frame.seq);
    if (it == pending_.end()) {
        ++orphan_rsp_count_;  // Late or duplicate RSP — no matching pending command
        return;
    }
    if (it->second.msgid != orig_msgid) {
        ++orphan_rsp_count_;  // orig_msgid mismatch — seq collision or protocol error
        return;
    }
    pending_.erase(it);
    std::cout << "RSP msgid=0x" << std::hex << static_cast<int>(orig_msgid)
              << " status=0x" << static_cast<int>(status) << std::dec << "\n";
    if (status != kStatusOk) {
        ++error_rsp_count_;
        return;
    }

    // Handle TIME_SYNC response: extract T1, T2, T3, calculate offset, send TIME_ADJUST
    if (orig_msgid == kMsgTimeSync && frame.data.size() >= 26) {
        const uint64_t T1 = protocol::read_u64(frame.data, 2);   // Echoed back T1
        const uint64_t T2 = protocol::read_u64(frame.data, 10);  // Device receive time
        const uint64_t T3 = protocol::read_u64(frame.data, 18);  // Device send time

        // T4 is the receive time from VirtualLink (accurate delivery time)
        const uint64_t T4 = receive_time_us;

        // Calculate delay and offset
        const int64_t delay = signed_diff_u64(T4, T1) - signed_diff_u64(T3, T2);
        const int64_t offset = (signed_diff_u64(T2, T1) + signed_diff_u64(T3, T4)) / 2;

        std::cout << "TIME_SYNC: T1=" << T1 << " T2=" << T2 << " T3=" << T3 << " T4=" << T4
                  << " delay=" << delay << " offset=" << offset << "\n";

        last_time_sync_.t1 = T1;
        last_time_sync_.t2 = T2;
        last_time_sync_.t3 = T3;
        last_time_sync_.t4 = T4;
        ++last_time_sync_.count;

        // Send TIME_ADJUST with negative offset
        if (auto_time_adjust_) {
            send_time_adjust(-offset, T4);
        }
    }
}

void PCNode::handle_cfg_report(const protocol::DynamicFrame &frame) {
    if (frame.data.size() < 16) {
        return;
    }
    const uint8_t proto_ver = frame.data[0];
    const uint8_t flags = frame.data[1];
    current_lsb_nA_ = protocol::read_u32(frame.data, 2);
    const uint16_t shunt_cal = protocol::read_u16(frame.data, 6);
    const uint16_t config_reg = protocol::read_u16(frame.data, 8);
    const uint16_t adc_config_reg = protocol::read_u16(frame.data, 10);
    stream_period_us_ = protocol::read_u16(frame.data, 12);
    stream_mask_ = protocol::read_u16(frame.data, 14);
    adcrange_ = (flags & 0x04) != 0;
    std::cout << "CFG_REPORT ver=" << static_cast<int>(proto_ver)
              << " lsb_nA=" << current_lsb_nA_ << " shunt_cal=" << shunt_cal
              << " config=0x" << std::hex << config_reg << " adc=0x" << adc_config_reg
              << " period_us=" << std::dec << stream_period_us_ << " mask=0x" << std::hex
              << stream_mask_ << std::dec << "\n";
}

void PCNode::handle_text_report(const protocol::DynamicFrame &frame) {
    if (frame.data.empty()) {
        return;
    }
    const std::string text(frame.data.begin(), frame.data.end());
    std::cout << "TEXT_REPORT len=" << frame.data.size() << " text=\"" << text << "\"\n";
}

void PCNode::handle_data_sample(const protocol::DynamicFrame &frame) {
    if (frame.data.size() < 41) {
        ++truncated_data_count_;
        return;
    }
    if (!has_data_seq_) {
        has_data_seq_ = true;
        last_data_seq_ = frame.seq;
    } else {
        const uint8_t expected = static_cast<uint8_t>(last_data_seq_ + 1);
        if (frame.seq != expected) {
            ++data_drop_count_;
        }
        last_data_seq_ = frame.seq;
    }

    // Wire layout (41 bytes): timestamp_unix_us(8) + timestamp_us(8) + flags(1) + ...
    const uint64_t timestamp = protocol::read_u64(frame.data, 8);
    const uint8_t flags = frame.data[16];
    const uint8_t *vbus20 = &frame.data[17];
    const uint8_t *vshunt20 = &frame.data[20];
    const uint8_t *current20 = &frame.data[23];
    const uint8_t *power24 = &frame.data[26];
    const int16_t temp_raw = static_cast<int16_t>(protocol::read_u16(frame.data, 29));
    const uint8_t *energy40 = &frame.data[31];
    const uint8_t *charge40 = &frame.data[36];

    const uint32_t vbus_raw = protocol::unpack_u20(vbus20);
    const int32_t vshunt_raw = protocol::unpack_s20(vshunt20);
    const int32_t current_raw = protocol::unpack_s20(current20);
    const uint32_t power_raw = protocol::unpack_u24(power24);
    const uint64_t energy_raw = protocol::unpack_u40(energy40);
    const int64_t charge_raw = protocol::unpack_s40(charge40);

    auto sample = protocol::to_engineering(vbus_raw, vshunt_raw, current_raw, temp_raw,
                                           power_raw, energy_raw, charge_raw,
                                           current_lsb_nA_, adcrange_);

    last_timestamp_us_ = timestamp;

    if ((timestamp % 1000000ULL) < 1000ULL) {
        std::cout << "DATA ts_us=" << timestamp << " flags=0x" << std::hex
                  << static_cast<int>(flags) << std::dec << " vbus=" << sample.vbus_v
                  << " current=" << sample.current_a << " temp=" << sample.temp_c
                  << " power=" << sample.power_w << "\n";
    }
}

} // namespace node
