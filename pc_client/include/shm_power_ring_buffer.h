#pragma once

#include <mutex>

#include <spsc_ring_buffer.hpp>

#include "realtime_power_metrics.h"

namespace powermonitor {
namespace client {

class ShmPicoRingBuffer {
public:
    ShmPicoRingBuffer()
        : buffer_(PICO_METRICS_SHM_NAME,
                  kPicoMetricsVersion,
                  buffers::ShmOpenMode::create_or_open) {}

    explicit ShmPicoRingBuffer(const char* shm_name)
        : buffer_(shm_name,
                  kPicoMetricsVersion,
                  buffers::ShmOpenMode::create_or_open) {}

    void push(const PicoSample& sample) {
        std::lock_guard<std::mutex> lock(producer_mutex_);
        buffer_.push_overwrite(sample);
    }

    uint64_t overflow_count() const { return buffer_.overflow_count(); }
    bool valid() const { return buffer_.valid(); }
    bool ok() const { return valid(); }
    bool is_creator() const { return buffer_.is_creator(); }

private:
    using PicoSpscBuffer = buffers::spsc_ring_buffer<
        PicoSample,
        kPicoMetricsRingCapacity,
        buffers::ShmStorage
    >;

    PicoSpscBuffer buffer_;
    std::mutex producer_mutex_;
};

class ShmOnboardRingBuffer {
public:
    ShmOnboardRingBuffer()
        : buffer_(ONBOARD_METRICS_SHM_NAME,
                  kOnboardMetricsVersion,
                  buffers::ShmOpenMode::create_or_open) {}

    explicit ShmOnboardRingBuffer(const char* shm_name)
        : buffer_(shm_name,
                  kOnboardMetricsVersion,
                  buffers::ShmOpenMode::create_or_open) {}

    void push(const OnboardShmSample& sample) {
        std::lock_guard<std::mutex> lock(producer_mutex_);
        buffer_.push_overwrite(sample);
    }

    uint64_t overflow_count() const { return buffer_.overflow_count(); }
    bool valid() const { return buffer_.valid(); }
    bool ok() const { return valid(); }
    bool is_creator() const { return buffer_.is_creator(); }

private:
    using OnboardSpscBuffer = buffers::spsc_ring_buffer<
        OnboardShmSample,
        kOnboardMetricsRingCapacity,
        buffers::ShmStorage
    >;

    OnboardSpscBuffer buffer_;
    std::mutex producer_mutex_;
};

}  // namespace client
}  // namespace powermonitor
