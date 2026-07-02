#ifndef DEVICE_COMMAND_HANDLER_HPP
#define DEVICE_COMMAND_HANDLER_HPP

#include <cstdint>
#include <cstring>
#include "pico/stdlib.h"

#include "protocol/frame_defs.hpp"
#include "protocol/frame_builder.hpp"
#include "state_machine.hpp"
#include "core/raw_sample.hpp"
#include "sampler.hpp"

namespace device {

// External global variable for T2 timing (defined in powermonitor.cpp)
extern uint64_t g_last_frame_recv_time_us;

constexpr uint16_t kStreamMaskUsbStressMode = 0x8000;

// USB CDC write function type. Returns true on success (all bytes written),
// false on failure/timeout.
using UsbWriteFn = bool (*)(const uint8_t* data, size_t len);

// Command handler - processes incoming frames and sends responses
class CommandHandler {
public:
    explicit CommandHandler(DeviceContext& ctx, UsbWriteFn write_fn)
        : ctx_(ctx), write_flush_fn_(write_fn), write_noflush_fn_(write_fn) {}

    CommandHandler(DeviceContext& ctx, UsbWriteFn write_fn, UsbWriteFn data_write_fn)
        : ctx_(ctx), write_flush_fn_(write_fn), write_noflush_fn_(data_write_fn) {}

    void maybe_emit_stats_report(uint64_t now_us) {
        if (!ctx_.is_streaming()) {
            return;
        }
        constexpr uint16_t kStatsReportPeriodMs = 1000;
        if (last_stats_report_us_ == 0) {
            last_stats_report_us_ = now_us;
            return;
        }
        const uint64_t elapsed_us = now_us - last_stats_report_us_;
        if (elapsed_us < static_cast<uint64_t>(kStatsReportPeriodMs) * 1000ULL) {
            return;
        }
        const uint16_t window_ms = static_cast<uint16_t>(elapsed_us / 1000ULL);
        if (send_stats_report(window_ms)) {
            last_stats_report_us_ = now_us;
        }
    }

    // Handle a received frame (called from parser callback)
    void handle_frame(const protocol::FixedFrame& frame) {
        // Only process CMD frames
        if (frame.type != protocol::FrameType::kCmd) {
            return;
        }

        ctx_.cmd_received++;

        switch (static_cast<protocol::MsgId>(frame.msgid)) {
        case protocol::MsgId::kPing:
            handle_ping(frame);
            break;
        case protocol::MsgId::kTimeSync:
            handle_time_sync(frame);
            break;
        case protocol::MsgId::kTimeAdjust:
            handle_time_adjust(frame);
            break;
        case protocol::MsgId::kTimeSet:
            handle_time_set(frame);
            break;
        case protocol::MsgId::kSetCfg:
            handle_set_cfg(frame);
            break;
        case protocol::MsgId::kGetCfg:
            handle_get_cfg(frame);
            break;
        case protocol::MsgId::kRegRead:
            handle_reg_read(frame);
            break;
        case protocol::MsgId::kRegWrite:
            handle_reg_write(frame);
            break;
        case protocol::MsgId::kStreamStart:
            handle_stream_start(frame);
            break;
        case protocol::MsgId::kStreamStop:
            handle_stream_stop(frame);
            break;
        default:
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrUnkCmd);
            break;
        }
    }

    // Build and send a DATA_SAMPLE frame from a RawSample
    // Use template parameter to choose flush behavior at compile time
    template<bool FlushAfterWrite = true>
    size_t send_data_sample_impl(const core::RawSample& sample) {
        // Build DATA_SAMPLE payload (37 bytes)
        protocol::DataSamplePayload payload;
        payload.timestamp_us = sample.timestamp_us;
        payload.timestamp_unix_us = sample.timestamp_unix_us;  // Use value from sampling time
        payload.flags = sample.flags;

        // Pack values
        core::pack_u20(payload.vbus20, sample.vbus_raw);
        core::pack_s20(payload.vshunt20, sample.vshunt_raw);
        core::pack_s20(payload.current20, sample.current_raw);
        core::pack_u24(payload.power24, sample.power_raw);
        payload.dietemp16 = sample.dietemp_raw;
        core::pack_u40(payload.energy40, sample.energy_raw);
        core::pack_s40(payload.charge40, sample.charge_raw);

        // Build and send frame
        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kData,
            0,  // flags
            ctx_.next_data_seq(),
            static_cast<uint8_t>(protocol::MsgId::kDataSample),
            reinterpret_cast<const uint8_t*>(&payload),
            sizeof(payload)
        );

        if (len > 0) {
            bool ok = false;
            if constexpr (FlushAfterWrite) {
                if (write_flush_fn_) {
                    ok = write_flush_fn_(tx_buf_, len);
                }
            } else {
                if (write_noflush_fn_) {
                    ok = write_noflush_fn_(tx_buf_, len);
                }
            }
            if (ok) {
                ctx_.samples_sent++;
            } else if (ctx_.shared_ctx && !ctx_.usb_stress_mode) {
                // USB write failed: count as dropped sample in shared stats.
                ctx_.shared_ctx->samples_dropped++;
            }
        }
        return len;
    }

    // Default: flush after write (for real USB device)
    inline size_t send_data_sample(const core::RawSample& sample) {
        return send_data_sample_impl<true>(sample);
    }

    // For simulation: no flush needed
    inline size_t send_data_sample_noflush(const core::RawSample& sample) {
        return send_data_sample_impl<false>(sample);
    }

    // Send asynchronous text event to PC.
    // Payload is raw text bytes with length range [1, 4096].
    bool send_text_report(const uint8_t* text, size_t text_len) {
        if (text == nullptr || text_len == 0 || text_len > 4096) {
            return false;
        }

        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kEvt,
            0,
            ctx_.next_data_seq(),
            static_cast<uint8_t>(protocol::MsgId::kTextReport),
            text,
            text_len
        );

        if (len > 0 && write_flush_fn_) {
            return write_flush_fn_(tx_buf_, len);
        }
        return false;
    }

    // Send EVT_TIME_SYNC_REQUEST to notify PC to initiate time sync (no payload).
    bool send_time_sync_request() {
        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kEvt,
            0,
            ctx_.next_data_seq(),
            static_cast<uint8_t>(protocol::MsgId::kTimeSyncRequest),
            nullptr,
            0
        );

        if (len > 0 && write_flush_fn_) {
            return write_flush_fn_(tx_buf_, len);
        }
        return false;
    }

private:
    void handle_ping(const protocol::FixedFrame& frame) {
        send_rsp(frame.seq, frame.msgid, protocol::Status::kOk);
    }

    void handle_time_sync(const protocol::FixedFrame& frame) {
        // T2: use global variable captured at frame receive time
        uint64_t t2_mono = g_last_frame_recv_time_us;
        if (t2_mono == 0) {
            t2_mono = time_us_64();  // fallback
        }

        if (frame.data_len < sizeof(protocol::TimeSyncPayload)) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrLen);
            return;
        }

        const auto* cmd = reinterpret_cast<const protocol::TimeSyncPayload*>(frame.data);

        // Build response payload, T3 placeholder (0) first
        // This allows CRC to be calculated without T3
        protocol::TimeSyncResponsePayload rsp;
        rsp.orig_msgid = frame.msgid;
        rsp.status = static_cast<uint8_t>(protocol::Status::kOk);
        rsp.t1 = cmd->t1;
        rsp.t2 = t2_mono + static_cast<uint64_t>(ctx_.epoch_offset_us);
        //rsp.t3 = 0;  //  Wont waste time setting 0, will be patched before send

        // Build frame with T3=0 (CRC calculated over placeholder)
        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kRsp, 0, frame.seq, frame.msgid,
            reinterpret_cast<const uint8_t*>(&rsp), sizeof(rsp)
        );

        if (len > 0 && write_flush_fn_) {
            // Capture T3 right before USB write
            const uint64_t t3_mono = time_us_64();
            const uint64_t t3 = t3_mono + static_cast<uint64_t>(ctx_.epoch_offset_us);

            // Patch T3 into frame after CRC is calculated
            // Frame layout: SOF(2) + Header(6) + MSGID(1) + orig_msgid(1) + status(1) + t1(8) + t2(8) + t3(8)
            // T3 offset = 2 + 6 + 1 + 1 + 1 + 8 + 8 = 27
            uint64_t* t3_ptr = reinterpret_cast<uint64_t*>(tx_buf_ + 27);
            *t3_ptr = t3;

            (void)write_flush_fn_(tx_buf_, len);
        }
    }

    void handle_time_adjust(const protocol::FixedFrame& frame) {
        if (frame.data_len < sizeof(protocol::TimeAdjustPayload)) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrLen);
            return;
        }
        const auto* cmd = reinterpret_cast<const protocol::TimeAdjustPayload*>(frame.data);
        ctx_.epoch_offset_us += cmd->offset_us;
        // Sync epoch_offset to shared context for Core 1 to use
        if (ctx_.shared_ctx) {
            ctx_.shared_ctx->epoch_offset_us = ctx_.epoch_offset_us;
        }
        send_rsp(frame.seq, frame.msgid, protocol::Status::kOk);
        ctx_.sync_waiting = false;  // Sync complete, resume normal loop
    }

    void handle_time_set(const protocol::FixedFrame& frame) {
        if (frame.data_len < sizeof(protocol::TimeSetPayload)) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrLen);
            return;
        }
        const auto* cmd = reinterpret_cast<const protocol::TimeSetPayload*>(frame.data);
        uint64_t now_us = time_us_64();
        //timestamp_unix_us = static_cast<uint64_t>(now) + shared->epoch_offset_us
        // epoch_offset = unix_time - monotonic_time
        ctx_.epoch_offset_us = static_cast<int64_t>(cmd->unix_time_us) - static_cast<int64_t>(now_us);
        // Sync epoch_offset to shared context for Core 1 to use
        if (ctx_.shared_ctx) {
            ctx_.shared_ctx->epoch_offset_us = ctx_.epoch_offset_us;
        }
        send_rsp(frame.seq, frame.msgid, protocol::Status::kOk);
    }

    void handle_set_cfg(const protocol::FixedFrame& frame) {
        if (frame.data_len < sizeof(protocol::SetCfgPayload)) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrLen);
            return;
        }

        const auto* cfg = reinterpret_cast<const protocol::SetCfgPayload*>(frame.data);

        // Update software context
        ctx_.config_reg = cfg->config_reg;
        ctx_.adc_config_reg = cfg->adc_config_reg;
        ctx_.shunt_cal = cfg->shunt_cal;
        ctx_.shunt_tempco = cfg->shunt_tempco;

        // Update hardware registers. NOTE(2026-05-27): the four
        // INA228::write_register16 calls below use Pico SDK hardware
        // I2C, while the sampler running on Core 1 drives the same
        // SDA/SCL pins via PIO bitbang (see sampler.hpp:10 and
        // INA228.cpp:307). When this handler is invoked while the
        // sampler timer is active, the two I2C masters collide and the
        // hardware-I2C call hangs (observed empirically: pc_client
        // SET_CFG times out, then PING also times out, and the chip
        // recovers only after a Pico re-flash + power cycle).
        // Pc_client currently does not invoke SET_CFG because of this;
        // the proper fix is to rewrite these writes through pio_i2c so
        // both masters share the same bus driver. Until then this
        // handler is kept in its original form for protocol coverage.
        if (ctx_.ina228) {
            bool ok = true;
            ok &= ctx_.ina228->write_register16(INA228::INA228_Register::CONFIG, cfg->config_reg);
            ok &= ctx_.ina228->write_register16(INA228::INA228_Register::ADC_CONFIG, cfg->adc_config_reg);
            ok &= ctx_.ina228->write_register16(INA228::INA228_Register::SHUNT_CAL, cfg->shunt_cal);
            ok &= ctx_.ina228->write_register16(INA228::INA228_Register::SHUNT_TEMPCO, cfg->shunt_tempco);
            if (!ok) {
                send_rsp(frame.seq, frame.msgid, protocol::Status::kErrHw);
                return;
            }
        }

        // Calculate current_lsb_nA from shunt_cal (simplified)
        // In real implementation, this would involve the actual INA228 formula
        ctx_.cal_valid = (cfg->shunt_cal != 0);
        ctx_.adcrange = (cfg->config_reg >> 4) & 0x01;

        // Send RSP(OK) then CFG_REPORT
        send_rsp(frame.seq, frame.msgid, protocol::Status::kOk);
        send_cfg_report();
    }

    void handle_get_cfg(const protocol::FixedFrame& frame) {
        // Send RSP(OK) then CFG_REPORT
        send_rsp(frame.seq, frame.msgid, protocol::Status::kOk);
        send_cfg_report();
    }

    void handle_reg_read(const protocol::FixedFrame& frame) {
        if (frame.data_len < sizeof(protocol::RegReadCmdPayload)) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrLen);
            return;
        }
        const auto* cmd = reinterpret_cast<const protocol::RegReadCmdPayload*>(frame.data);
        auto reg_addr = static_cast<INA228::INA228_Register>(cmd->reg_addr);

        size_t value_len = 0;
        if (cmd->reg_type == 0) {
            value_len = 2;
        } else if (cmd->reg_type == 1) {
            value_len = 3;
        } else if (cmd->reg_type == 2) {
            value_len = 5;
        } else {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrParam);
            return;
        }

        // While streaming, Core 1 owns the I2C bus (PIO DMA sampling); a
        // second master here would corrupt its transactions. timer_active
        // additionally covers the brief window after STREAM_STOP while the
        // async stop command is still in flight to Core 1 (best-effort
        // cross-core read; benign race).
        if (ctx_.is_streaming() || g_sampler_ctx.timer_active) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrHw);
            return;
        }

        uint8_t raw[5] = {0}; // Register bytes, MSB first (INA228 wire order)
        bool ok = false;
        if (g_sampler_ctx.pio != nullptr) {
            // Normal build: boot handed the bus from hardware I2C to PIO
            // (powermonitor.cpp i2c_deinit), so the INA228 driver's blocking
            // hardware-I2C calls would spin forever. Use the PIO CPU path,
            // which has internal timeouts and self-recovers after errors.
            ok = pio_i2c_read_reg(g_sampler_ctx.pio, g_sampler_ctx.sm,
                                  g_sampler_ctx.i2c_addr, cmd->reg_addr,
                                  raw, value_len);
        } else if (ctx_.ina228) {
            // TEST_MODE build: hardware I2C is still initialized.
            if (cmd->reg_type == 0) {
                uint16_t val = 0;
                ok = ctx_.ina228->read_register16(reg_addr, val);
                raw[0] = static_cast<uint8_t>(val >> 8);
                raw[1] = static_cast<uint8_t>(val);
            } else if (cmd->reg_type == 1) {
                uint32_t val = 0;
                ok = ctx_.ina228->read_register24(reg_addr, val);
                raw[0] = static_cast<uint8_t>(val >> 16);
                raw[1] = static_cast<uint8_t>(val >> 8);
                raw[2] = static_cast<uint8_t>(val);
            } else {
                uint64_t val = 0;
                ok = ctx_.ina228->read_register40(reg_addr, val);
                raw[0] = static_cast<uint8_t>(val >> 32);
                raw[1] = static_cast<uint8_t>(val >> 24);
                raw[2] = static_cast<uint8_t>(val >> 16);
                raw[3] = static_cast<uint8_t>(val >> 8);
                raw[4] = static_cast<uint8_t>(val);
            }
        }

        if (!ok) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrHw);
            return;
        }

        // Build and send response frame
        uint8_t rsp_buf[10]; // Space for header + 40-bit value
        rsp_buf[0] = frame.msgid;
        rsp_buf[1] = static_cast<uint8_t>(protocol::Status::kOk);
        rsp_buf[2] = cmd->reg_addr;
        // Spec: variable-length register data is little-endian on the wire.
        for (size_t i = 0; i < value_len; ++i) {
            rsp_buf[3 + i] = raw[value_len - 1 - i];
        }

        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kRsp, 0, frame.seq, frame.msgid,
            rsp_buf, 3 + value_len
        );

        if (len > 0 && write_flush_fn_) {
            (void)write_flush_fn_(tx_buf_, len);
        }
    }

    void handle_reg_write(const protocol::FixedFrame& frame) {
        if (frame.data_len < sizeof(protocol::RegWriteCmdPayload)) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrLen);
            return;
        }
        const auto* cmd = reinterpret_cast<const protocol::RegWriteCmdPayload*>(frame.data);

        // Same bus-ownership rule as handle_reg_read.
        if (ctx_.is_streaming() || g_sampler_ctx.timer_active) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrHw);
            return;
        }

        bool ok = false;
        if (g_sampler_ctx.pio != nullptr) {
            // Normal build: hardware I2C is deinitialized at boot; write via
            // the PIO CPU path (INA228 register write = reg, MSB, LSB).
            uint8_t tx[3] = {cmd->reg_addr,
                             static_cast<uint8_t>(cmd->reg_value >> 8),
                             static_cast<uint8_t>(cmd->reg_value)};
            ok = pio_i2c_write_blocking(g_sampler_ctx.pio, g_sampler_ctx.sm,
                                        g_sampler_ctx.i2c_addr, tx, 3,
                                        /*nostop=*/false) == 0;
            // pio_i2c_write_blocking disables RX autopush and does not
            // restore it, but the DMA sampler relies on it staying armed
            // (enabled once in sampler_init_dma, not per tick). Re-arm it
            // or the next STREAM_START would produce no samples.
            pio_i2c_rx_enable(g_sampler_ctx.pio, g_sampler_ctx.sm, true);
        } else if (ctx_.ina228) {
            // TEST_MODE build: hardware I2C is still initialized.
            auto reg_addr = static_cast<INA228::INA228_Register>(cmd->reg_addr);
            ok = ctx_.ina228->write_register16(reg_addr, cmd->reg_value);
        }

        send_rsp(frame.seq, frame.msgid, ok ? protocol::Status::kOk : protocol::Status::kErrHw);
    }

    void handle_stream_start(const protocol::FixedFrame& frame) {
        if (frame.data_len < sizeof(protocol::StreamStartPayload)) {
            send_rsp(frame.seq, frame.msgid, protocol::Status::kErrLen);
            return;
        }

        const auto* cmd = reinterpret_cast<const protocol::StreamStartPayload*>(frame.data);
        const bool usb_stress_mode = (cmd->channel_mask & kStreamMaskUsbStressMode) != 0;
        const uint16_t channel_mask = static_cast<uint16_t>(
            cmd->channel_mask & static_cast<uint16_t>(~kStreamMaskUsbStressMode));

        // Start streaming with given parameters
        ctx_.start_streaming(cmd->period_us, channel_mask, time_us_64());
        ctx_.usb_stress_mode = usb_stress_mode;
        last_stats_report_us_ = time_us_64();

        if (!ctx_.usb_stress_mode) {
            // Signal Core 1 to start sampling
            sampler_start();
        }

        send_rsp(frame.seq, frame.msgid, protocol::Status::kOk);
    }

    void handle_stream_stop(const protocol::FixedFrame& frame) {
        const uint64_t now_us = time_us_64();
        if (!ctx_.usb_stress_mode) {
            // Signal Core 1 to stop sampling
            sampler_stop();
        }

        ctx_.stop_streaming();
        send_rsp(frame.seq, frame.msgid, protocol::Status::kOk);
        uint16_t window_ms = 0;
        if (last_stats_report_us_ != 0) {
            window_ms = static_cast<uint16_t>((now_us - last_stats_report_us_) / 1000ULL);
        }
        send_stats_report(window_ms);
        last_stats_report_us_ = 0;
    }

    void send_rsp(uint8_t seq, uint8_t orig_msgid, protocol::Status status) {
        uint8_t payload[2];
        payload[0] = orig_msgid;
        payload[1] = static_cast<uint8_t>(status);

        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kRsp,
            0,
            seq,
            orig_msgid,
            payload,
            sizeof(payload)
        );

        if (len > 0 && write_flush_fn_) {
            (void)write_flush_fn_(tx_buf_, len);
        }
    }

    void send_cfg_report() {
        protocol::CfgReportPayload payload;
        payload.proto_ver = protocol::kProtoVersion;
        payload.flags = 0;
        if (ctx_.is_streaming()) payload.flags |= 0x01;
        if (ctx_.cal_valid) payload.flags |= 0x02;
        if (ctx_.adcrange) payload.flags |= 0x04;

        payload.current_lsb_nA = ctx_.current_lsb_nA;
        payload.shunt_cal_reg = ctx_.shunt_cal;
        payload.config_reg = ctx_.config_reg;
        payload.adc_config_reg = ctx_.adc_config_reg;
        payload.stream_period_us = ctx_.stream_period_us;
        payload.stream_mask = ctx_.stream_mask;

        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kEvt,
            0,
            ctx_.next_data_seq(),
            static_cast<uint8_t>(protocol::MsgId::kCfgReport),
            reinterpret_cast<const uint8_t*>(&payload),
            sizeof(payload)
        );

        if (len > 0 && write_flush_fn_) {
            write_flush_fn_(tx_buf_, len);
        }
    }

    bool send_stats_report(uint16_t window_ms) {
        protocol::StatsReportPayload payload{};
        payload.report_seq = stats_report_seq_++;

        payload.samples_produced = ctx_.samples_sent;

        if (ctx_.shared_ctx && !ctx_.usb_stress_mode) {
            payload.samples_produced = ctx_.shared_ctx->samples_produced;
            payload.samples_dropped = ctx_.shared_ctx->samples_dropped;
            payload.dropped_cnvrf_not_ready = ctx_.shared_ctx->dropped_cnvrf_not_ready;
            payload.dropped_duplicate_suppressed =
                ctx_.shared_ctx->dropped_duplicate_suppressed;
            payload.dropped_worker_missed_tick =
                ctx_.shared_ctx->dropped_worker_missed_tick;
            payload.dropped_queue_full = ctx_.shared_ctx->dropped_queue_full;
            payload.queue_depth = static_cast<uint16_t>(ctx_.shared_ctx->sample_queue.size());
        }

        payload.reason_bits = 0;
        payload.window_ms = window_ms;

        size_t len = protocol::build_frame(
            tx_buf_, sizeof(tx_buf_),
            protocol::FrameType::kEvt,
            0,
            ctx_.next_data_seq(),
            static_cast<uint8_t>(protocol::MsgId::kStatsReport),
            reinterpret_cast<const uint8_t*>(&payload),
            sizeof(payload)
        );

        // Use noflush to avoid blocking the main loop
        if (len > 0 && write_noflush_fn_) {
            return write_noflush_fn_(tx_buf_, len);
        }
        return false;
    }

    DeviceContext& ctx_;
    UsbWriteFn write_flush_fn_;
    UsbWriteFn write_noflush_fn_;
    // Use kMaxTxPayloadLen for the send buffer (needs to hold TEXT_REPORT up to 4096 bytes).
    // Static to keep it off the stack and avoid RP2040 stack overflow.
    static constexpr size_t kMaxFrameBytes = 2 + protocol::kHeaderSize + protocol::kMaxTxPayloadLen + 2;
    static uint8_t tx_buf_[kMaxFrameBytes];
    uint16_t stats_report_seq_ = 0;
    uint64_t last_stats_report_us_ = 0;
};

} // namespace device

#endif // DEVICE_COMMAND_HANDLER_HPP
