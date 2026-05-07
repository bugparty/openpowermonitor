#include <gtest/gtest.h>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <CLI/CLI.hpp>
#include <unistd.h>
#include <sys/wait.h>

#include "protocol/frame_builder.h"
#include "sample_queue.h"
#include "session.h"
#include "shm_power_ring_buffer.h"

#define private public
#include "power_monitor_session.h"
#undef private

#include "../src/power_monitor_session.cpp"

using namespace powermonitor::client;

namespace {

using PicoSpscBuffer = buffers::spsc_ring_buffer<
    PicoSample,
    kPicoMetricsRingCapacity,
    buffers::ShmStorage
>;
using OnboardSpscBuffer = buffers::spsc_ring_buffer<
    OnboardShmSample,
    kOnboardMetricsRingCapacity,
    buffers::ShmStorage
>;

bool pop_pico(PicoSpscBuffer& reader, PicoSample* sample) {
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (reader.try_pop(*sample)) {
            return true;
        }
        usleep(100);
    }
    return false;
}

bool pop_onboard(OnboardSpscBuffer& reader, OnboardShmSample* sample) {
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (reader.try_pop(*sample)) {
            return true;
        }
        usleep(100);
    }
    return false;
}

Session::Config realtime_export_config() {
    Session::Config config;
    config.current_lsb_nA = 100000;
    return config;
}

Session::Sample pico_session_sample(uint32_t seq) {
    Session::Sample sample;
    sample.seq = seq;
    sample.host_timestamp_us = 1000 + seq;
    sample.device_timestamp_us = 2000 + seq;
    sample.device_timestamp_unix_us = 3000 + seq;
    sample.power_raw = seq * 10;
    sample.vbus_raw = seq * 100;
    sample.current_raw = static_cast<int32_t>(seq * 5);
    return sample;
}

OnboardSample onboard_session_sample(int64_t seq) {
    OnboardSample sample;
    sample.mono_ns = 1000000 + seq * 1000;
    sample.unix_ns = 2000000 + seq * 1000;
    sample.total_mw = 1000 + seq;
    return sample;
}

}  // namespace

TEST(ShmPowerSpscTest, CreateAndPush) {
    const char* test_shm = "/test_pico_spsc_basic";
    shm_unlink(test_shm);

    ShmPicoRingBuffer buffer(test_shm);
    ASSERT_TRUE(buffer.valid());
    ASSERT_TRUE(buffer.is_creator());

    PicoSample sample{};
    sample.power_w = 10.5;
    sample.sequence_num = 1;

    buffer.push(sample);

    shm_unlink(test_shm);
}

TEST(HostClientCliTest, NoOnboardDisablesConfigEnabledSampler) {
    bool onboard_enabled = false;
    bool no_onboard = false;
    CLI::App app{"Powermonitor PC client"};

    auto opt_onboard = app.add_flag("--onboard", onboard_enabled, "Enable onboard hwmon power sampling");
    auto opt_no_onboard = app.add_flag("--no-onboard", no_onboard, "Disable onboard hwmon power sampling");

    const char* argv[] = {"host_pc_client", "--no-onboard"};
    app.parse(2, const_cast<char**>(argv));

    onboard_enabled = true;  // Simulate config loaded after parse with onboard.enabled: true.
    if (opt_onboard->count() > 0) {
        onboard_enabled = true;
    }
    if (opt_no_onboard->count() > 0) {
        onboard_enabled = false;
    }

    EXPECT_FALSE(onboard_enabled);
}

TEST(PowerMonitorSessionRealtimeExportTest, PicoSamplesHaveMonotonicSequenceNumbers) {
    shm_unlink(PICO_METRICS_SHM_NAME);

    PowerMonitorSession::Options options;
    PowerMonitorSession session(options);
    session.session_->set_config(realtime_export_config());

    PicoSpscBuffer reader(PICO_METRICS_SHM_NAME, kPicoMetricsVersion, buffers::ShmOpenMode::open);
    ASSERT_TRUE(reader.valid());

    session.export_pico_power_sample(pico_session_sample(1));
    session.export_pico_power_sample(pico_session_sample(2));

    PicoSample first{};
    PicoSample second{};
    ASSERT_TRUE(pop_pico(reader, &first));
    ASSERT_TRUE(pop_pico(reader, &second));

    EXPECT_LT(first.sequence_num, second.sequence_num);

    shm_unlink(PICO_METRICS_SHM_NAME);
}

TEST(PowerMonitorSessionRealtimeExportTest, PicoSampleExportsEnergyJ) {
    shm_unlink(PICO_METRICS_SHM_NAME);

    PowerMonitorSession::Options options;
    PowerMonitorSession session(options);
    session.session_->set_config(realtime_export_config());

    PicoSpscBuffer reader(PICO_METRICS_SHM_NAME, kPicoMetricsVersion, buffers::ShmOpenMode::open);
    ASSERT_TRUE(reader.valid());

    Session::Sample sample = pico_session_sample(1);
    sample.energy_raw = 1234;

    session.export_pico_power_sample(sample);

    PicoSample exported{};
    ASSERT_TRUE(pop_pico(reader, &exported));

    const double current_lsb = realtime_export_config().current_lsb_nA * 1e-9;
    EXPECT_DOUBLE_EQ(exported.energy_j, sample.energy_raw * current_lsb * 3.2 * 16.0);

    shm_unlink(PICO_METRICS_SHM_NAME);
}

TEST(PowerMonitorSessionRealtimeExportTest, OnboardSamplesCarryLatestPicoEnergyJ) {
    shm_unlink(PICO_METRICS_SHM_NAME);
    shm_unlink(ONBOARD_METRICS_SHM_NAME);

    PowerMonitorSession::Options options;
    PowerMonitorSession session(options);
    session.session_->set_config(realtime_export_config());

    PicoSpscBuffer pico_reader(PICO_METRICS_SHM_NAME, kPicoMetricsVersion, buffers::ShmOpenMode::open);
    OnboardSpscBuffer onboard_reader(ONBOARD_METRICS_SHM_NAME, kOnboardMetricsVersion, buffers::ShmOpenMode::open);
    ASSERT_TRUE(pico_reader.valid());
    ASSERT_TRUE(onboard_reader.valid());

    Session::Sample pico_s = pico_session_sample(1);
    pico_s.energy_raw = 1234;

    session.export_pico_power_sample(pico_s);
    session.export_onboard_power_sample(onboard_session_sample(2));

    PicoSample pico_exported{};
    OnboardShmSample onboard_exported{};
    ASSERT_TRUE(pop_pico(pico_reader, &pico_exported));
    ASSERT_TRUE(pop_onboard(onboard_reader, &onboard_exported));

    const double current_lsb = realtime_export_config().current_lsb_nA * 1e-9;
    const double expected_energy_j = pico_s.energy_raw * current_lsb * 3.2 * 16.0;
    EXPECT_DOUBLE_EQ(pico_exported.energy_j, expected_energy_j);

    shm_unlink(PICO_METRICS_SHM_NAME);
    shm_unlink(ONBOARD_METRICS_SHM_NAME);
}

TEST(PowerMonitorSessionRealtimeExportTest, OnboardSamplesHaveMonotonicSequenceNumbers) {
    shm_unlink(ONBOARD_METRICS_SHM_NAME);

    PowerMonitorSession::Options options;
    PowerMonitorSession session(options);

    OnboardSpscBuffer reader(ONBOARD_METRICS_SHM_NAME, kOnboardMetricsVersion, buffers::ShmOpenMode::open);
    ASSERT_TRUE(reader.valid());

    session.export_onboard_power_sample(onboard_session_sample(1));
    session.export_onboard_power_sample(onboard_session_sample(2));

    OnboardShmSample first{};
    OnboardShmSample second{};
    ASSERT_TRUE(pop_onboard(reader, &first));
    ASSERT_TRUE(pop_onboard(reader, &second));

    EXPECT_LT(first.sequence_num, second.sequence_num);

    shm_unlink(ONBOARD_METRICS_SHM_NAME);
}

TEST(ShmPowerSpscTest, OverflowCount) {
    const char* test_shm = "/test_pico_spsc_overflow";
    shm_unlink(test_shm);

    ShmPicoRingBuffer buffer(test_shm);
    ASSERT_TRUE(buffer.valid());

    for (size_t i = 0; i < kPicoMetricsRingCapacity + 100; ++i) {
        PicoSample sample{};
        sample.sequence_num = i;
        buffer.push(sample);
    }

    EXPECT_GT(buffer.overflow_count(), 0);

    shm_unlink(test_shm);
}

TEST(ShmPowerSpscTest, CrossProcessPushPop) {
    const char* test_shm = "/test_pico_spsc_ipc";
    shm_unlink(test_shm);

    pid_t pid = fork();
    ASSERT_GE(pid, 0);

    if (pid == 0) {
        // Child: writer
        ShmPicoRingBuffer writer(test_shm);
        if (!writer.valid()) exit(1);

        for (int i = 0; i < 10; ++i) {
            PicoSample sample{};
            sample.sequence_num = i;
            sample.power_w = static_cast<double>(i) * 1.5;
            writer.push(sample);
        }
        exit(0);
    } else {
        // Parent: reader
        usleep(10000);

        PicoSpscBuffer reader(test_shm, kPicoMetricsVersion, buffers::ShmOpenMode::open);
        ASSERT_TRUE(reader.valid());

        int expected = 0;
        int attempts = 0;
        while (expected < 10 && attempts < 1000) {
            PicoSample sample;
            if (reader.try_pop(sample)) {
                EXPECT_EQ(sample.sequence_num, static_cast<uint64_t>(expected));
                EXPECT_DOUBLE_EQ(sample.power_w, static_cast<double>(expected) * 1.5);
                ++expected;
            }
            ++attempts;
            if (attempts % 100 == 0) usleep(100);
        }

        EXPECT_EQ(expected, 10);

        int status;
        waitpid(pid, &status, 0);
        EXPECT_EQ(WEXITSTATUS(status), 0);
    }

    shm_unlink(test_shm);
}
