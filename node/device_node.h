#pragma once

#include <cstdint>
#include <vector>

#include "node/ina228_model.h"
#include "protocol/frame_builder.h"
#include "protocol/parser.h"
#include "sim/virtual_link.h"

namespace node {

class DeviceNode {
public:
    explicit DeviceNode(sim::VirtualLinkEndpoint *endpoint);

    void tick(uint64_t now_us);

    // Device clock error relative to the simulation clock (the host reference).
    // After a perfect sync, this converges to 0. Exposed for time-sync tests.
    int64_t epoch_offset_us() const { return epoch_offset_us_; }

private:
    void on_frame(const protocol::DynamicFrame &frame, uint64_t receive_time_us);
    void handle_cmd(const protocol::DynamicFrame &frame, uint64_t now_us);

    void send_rsp(uint8_t seq, uint8_t orig_msgid, uint8_t status, const std::vector<uint8_t> &data,
                  uint64_t now_us);
    void send_cfg_report(uint64_t now_us);
    void send_text_report(const char *text, size_t text_len, uint64_t now_us);
    void send_data_sample(uint64_t now_us);

    sim::VirtualLinkEndpoint *endpoint_ = nullptr;
    protocol::Parser parser_;
    INA228Model model_;

    uint16_t config_reg_ = 0;
    uint16_t adc_config_reg_ = 0;
    uint16_t shunt_cal_reg_ = 0;
    uint16_t shunt_tempco_ = 0;
    uint32_t current_lsb_nA_ = 1000;
    bool streaming_on_ = false;
    bool adcrange_ = false;
    uint16_t stream_period_us_ = 0;
    uint16_t stream_mask_ = 0x000F;
    uint64_t stream_start_us_ = 0;
    uint64_t next_sample_due_us_ = 0;
    uint8_t data_seq_ = 0;
    uint64_t rx_counts_[256] = {};
    uint64_t current_now_us_ = 0;
    bool initial_cfg_sent_ = false;
    bool text_report_sent_ = false;
    int64_t epoch_offset_us_ = 0;  // Clock offset for Unix time conversion
};

} // namespace node
