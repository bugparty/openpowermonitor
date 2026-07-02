#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "protocol/frame_builder.h"
#include "protocol/parser.h"
#include "protocol/unpack.h"
#include "sim/virtual_link.h"

namespace node {

class PCNode {
public:
    // Raw timestamps from the last completed TIME_SYNC exchange, exposed so
    // tests can evaluate alternative offset-selection policies against the
    // same measurement the built-in handler saw.
    struct TimeSyncMeasurement {
        uint64_t t1 = 0;     // PC send time (echoed by device)
        uint64_t t2 = 0;     // Device receive time (device Unix domain)
        uint64_t t3 = 0;     // Device send time (device Unix domain)
        uint64_t t4 = 0;     // PC receive time
        uint64_t count = 0;  // Number of completed exchanges so far
    };

    explicit PCNode(sim::VirtualLinkEndpoint *endpoint);

    void tick(uint64_t now_us);

    void send_ping(uint64_t now_us);
    void send_time_sync(uint64_t now_us);
    void send_time_adjust(int64_t offset_us, uint64_t now_us);
    void send_time_set(uint64_t unix_time_us, uint64_t now_us);
    void send_set_cfg(uint16_t config_reg, uint16_t adc_config_reg, uint16_t shunt_cal,
                      uint16_t shunt_tempco, uint64_t now_us);
    void send_get_cfg(uint64_t now_us);
    void send_stream_start(uint16_t period_us, uint16_t mask, uint64_t now_us);
    void send_stream_stop(uint64_t now_us);

    uint64_t crc_fail_count() const { return parser_.crc_fail_count(); }
    uint64_t data_drop_count() const { return data_drop_count_; }
    uint64_t timeout_count() const { return timeout_count_; }
    uint64_t retransmit_count() const { return retransmit_count_; }
    uint64_t get_rx_count(uint8_t msgid) const { return rx_counts_[msgid]; }

    // Counters for silent-drop paths — useful for fault-injection tests.
    uint64_t orphan_rsp_count() const { return orphan_rsp_count_; }     // RSP with no matching pending cmd
    uint64_t error_rsp_count() const { return error_rsp_count_; }       // RSP with non-OK status
    uint64_t truncated_data_count() const { return truncated_data_count_; } // DATA_SAMPLE too short

    // Last values parsed from a CFG_REPORT.
    uint32_t current_lsb_nA() const { return current_lsb_nA_; }
    uint16_t stream_period_us_cfg() const { return stream_period_us_; }
    uint16_t stream_mask_cfg() const { return stream_mask_; }

    // When disabled, TIME_SYNC responses are recorded in last_time_sync() but
    // no TIME_ADJUST is sent, letting tests apply their own offset policy.
    void set_auto_time_adjust(bool enabled) { auto_time_adjust_ = enabled; }
    const TimeSyncMeasurement &last_time_sync() const { return last_time_sync_; }

private:
    // Tracks an outstanding command awaiting RSP, used for timeout/retransmit logic.
    struct PendingCmd {
        uint8_t msgid = 0;              // Original command message ID
        uint8_t seq = 0;                // Sequence number for RSP matching
        uint8_t retries = 0;            // Retransmit count (max 3)
        uint64_t deadline_us = 0;       // Timeout deadline in microseconds
        std::vector<uint8_t> bytes;     // Raw frame bytes for retransmission
    };

    void on_frame(const protocol::DynamicFrame &frame, uint64_t receive_time_us);
    void send_cmd(uint8_t msgid, const std::vector<uint8_t> &payload, uint64_t now_us);
    void handle_rsp(const protocol::DynamicFrame &frame, uint64_t receive_time_us);
    void handle_cfg_report(const protocol::DynamicFrame &frame);
    void handle_text_report(const protocol::DynamicFrame &frame);
    void handle_data_sample(const protocol::DynamicFrame &frame);

    sim::VirtualLinkEndpoint *endpoint_ = nullptr;
    protocol::Parser parser_;
    uint8_t cmd_seq_ = 0;
    std::map<uint8_t, PendingCmd> pending_;
    uint32_t current_lsb_nA_ = 1000;
    bool adcrange_ = false;
    uint16_t stream_period_us_ = 0;
    uint16_t stream_mask_ = 0;
    bool has_data_seq_ = false;
    uint8_t last_data_seq_ = 0;
    uint64_t data_drop_count_ = 0;
    uint64_t timeout_count_ = 0;
    uint64_t retransmit_count_ = 0;
    uint64_t orphan_rsp_count_ = 0;
    uint64_t error_rsp_count_ = 0;
    uint64_t truncated_data_count_ = 0;
    uint64_t rx_counts_[256] = {};
    uint64_t time_sync_T1_ = 0;  // T1 timestamp for TIME_SYNC calculation
    bool auto_time_adjust_ = true;       // Send TIME_ADJUST automatically on TIME_SYNC RSP
    TimeSyncMeasurement last_time_sync_; // Timestamps of the last completed exchange
    uint64_t current_time_us_ = 0;  // Current time updated in tick()
    uint64_t last_timestamp_us_ = 0;  // Last parsed DATA_SAMPLE timestamp_us

public:
    uint64_t last_timestamp_us() const { return last_timestamp_us_; }
};

} // namespace node
