#include <CLI/CLI.hpp>
#include <serial/serial.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "power_monitor_session.h"

namespace {

std::atomic<bool> g_stop_requested{false};

void handle_signal(int) { g_stop_requested = true; }

std::string format_filename_time(std::chrono::system_clock::time_point tp) {
    const std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string hex_u16(uint16_t value) {
    std::ostringstream oss;
    oss << "0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
        << value;
    return oss.str();
}

bool parse_hex_u16(const std::string &text, uint16_t &value) {
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
        value = static_cast<uint16_t>(raw);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_yaml_u32(const YAML::Node &node, uint32_t &out) {
    if (!node || !node.IsScalar()) {
        return false;
    }
    try {
        out = static_cast<uint32_t>(std::stoul(node.Scalar(), nullptr, 0));
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_yaml_u16(const YAML::Node &node, uint16_t &out) {
    if (!node || !node.IsScalar()) {
        return false;
    }
    try {
        out = static_cast<uint16_t>(std::stoul(node.Scalar(), nullptr, 0));
        return true;
    } catch (...) {
        return false;
    }
}

struct ConfigOptions {
    std::string port;
    uint32_t baud = 115200;
    uint16_t vid = 0x2E8A;
    uint16_t pid = 0x000A;
    uint16_t stream_period_us = 1000;
    uint16_t stream_mask = 0x000F;
    bool usb_stress_mode = false;
    uint16_t config_reg = 0x0000;
    uint16_t adc_config_reg = 0x1000;
    uint16_t shunt_cal = 0x1000;
    uint16_t shunt_tempco = 0x0000;
    bool config_overridden = false;
    bool verbose = false;
    bool interactive = false;
    bool debug_time_sync = false;
    bool no_apply_time_offset = false;
    uint32_t duration_s = 0;
    uint64_t duration_us = 0;
    std::string run_label;
    std::vector<std::string> run_tags;
    std::string output_path;
    std::string config_path;
    int read_thread_core = -1;
    int proc_thread_core = -1;
    int rt_prio = -1;

    // Onboard sampler options
    bool onboard_enabled = false;
    std::string onboard_hwmon_path = "/sys/class/hwmon/hwmon1";
    uint64_t onboard_period_us = 1000;
    uint64_t onboard_telemetry_period_us = 10000;
    int onboard_cpu_core = -1;
    int onboard_rt_prio = -1;
    std::string onboard_jetson_freq_path = "/proc/jetson_freqs";
};

bool parse_vid_pid(const std::string &hardware_id, uint16_t &vid, uint16_t &pid) {
    std::string upper = hardware_id;
    for (char &c : upper) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }

    const std::string windows_key = "USB\\VID_";
    const auto windows_pos = upper.find(windows_key);
    if (windows_pos != std::string::npos) {
        const auto vid_pos = windows_pos + windows_key.size();
        const auto pid_anchor = upper.find("&PID_", vid_pos);
        if (pid_anchor != std::string::npos && vid_pos + 4 <= upper.size() && pid_anchor + 9 <= upper.size()) {
            const std::string vid_str = upper.substr(vid_pos, 4);
            const std::string pid_str = upper.substr(pid_anchor + 5, 4);
            if (parse_hex_u16(vid_str, vid) && parse_hex_u16(pid_str, pid)) {
                return true;
            }
        }
    }

    const std::string cross_key = "VID:PID=";
    const auto cross_pos = upper.find(cross_key);
    if (cross_pos != std::string::npos) {
        const auto start = cross_pos + cross_key.size();
        if (start + 9 <= upper.size() && upper[start + 4] == ':') {
            const std::string vid_str = upper.substr(start, 4);
            const std::string pid_str = upper.substr(start + 5, 4);
            if (parse_hex_u16(vid_str, vid) && parse_hex_u16(pid_str, pid)) {
                return true;
            }
        }
    }

    // Handle simple "xxxx:xxxx" format (e.g., "2e8a:000a")
    const auto colon_pos = upper.find(':');
    if (colon_pos != std::string::npos && colon_pos + 5 <= upper.size()) {
        const std::string vid_str = upper.substr(0, colon_pos);
        const std::string pid_str = upper.substr(colon_pos + 1, 4);
        if (parse_hex_u16(vid_str, vid) && parse_hex_u16(pid_str, pid)) {
            return true;
        }
    }

    return false;
}

bool load_yaml_config(const std::string &path, ConfigOptions &options) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception &ex) {
        std::cerr << "Failed to load YAML config: " << ex.what() << "\n";
        return false;
    }

    if (const auto stream = root["stream"]) {
        uint16_t period = options.stream_period_us;
        uint16_t mask = options.stream_mask;
        if (parse_yaml_u16(stream["period_us"], period)) {
            options.stream_period_us = period;
        }
        if (parse_yaml_u16(stream["mask"], mask)) {
            options.stream_mask = mask;
        }
        if (stream["usb_stress_mode"] && stream["usb_stress_mode"].IsScalar()) {
            try {
                options.usb_stress_mode = stream["usb_stress_mode"].as<bool>();
            } catch (...) {
            }
        }
    }

    if (const auto ina = root["ina228"]) {
        uint16_t value = 0;
        if (parse_yaml_u16(ina["config_reg"], value)) {
            options.config_reg = value;
            options.config_overridden = true;
        }
        if (parse_yaml_u16(ina["adc_config_reg"], value)) {
            options.adc_config_reg = value;
            options.config_overridden = true;
        }
        if (parse_yaml_u16(ina["shunt_cal"], value)) {
            options.shunt_cal = value;
            options.config_overridden = true;
        }
        if (parse_yaml_u16(ina["shunt_tempco"], value)) {
            options.shunt_tempco = value;
            options.config_overridden = true;
        }
    }

    if (const auto affinity = root["affinity"]) {
        if (affinity["read_thread_core"] && affinity["read_thread_core"].IsScalar()) {
            options.read_thread_core = affinity["read_thread_core"].as<int>();
        }
        if (affinity["proc_thread_core"] && affinity["proc_thread_core"].IsScalar()) {
            options.proc_thread_core = affinity["proc_thread_core"].as<int>();
        }
        if (affinity["rt_prio"] && affinity["rt_prio"].IsScalar()) {
            options.rt_prio = affinity["rt_prio"].as<int>();
        }
    }

    // Parse onboard sampler configuration
    if (const auto onboard = root["onboard"]) {
        if (onboard["enabled"] && onboard["enabled"].IsScalar()) {
            options.onboard_enabled = onboard["enabled"].as<bool>();
        }
        if (onboard["hwmon_path"] && onboard["hwmon_path"].IsScalar()) {
            options.onboard_hwmon_path = onboard["hwmon_path"].as<std::string>();
        }
        if (onboard["period_us"] && onboard["period_us"].IsScalar()) {
            options.onboard_period_us = onboard["period_us"].as<uint64_t>();
        }
        if (onboard["telemetry_period_us"] && onboard["telemetry_period_us"].IsScalar()) {
            options.onboard_telemetry_period_us = onboard["telemetry_period_us"].as<uint64_t>();
        }
        if (onboard["cpu_core"] && onboard["cpu_core"].IsScalar()) {
            options.onboard_cpu_core = onboard["cpu_core"].as<int>();
        }
        if (onboard["rt_prio"] && onboard["rt_prio"].IsScalar()) {
            options.onboard_rt_prio = onboard["rt_prio"].as<int>();
        }
        if (onboard["jetson_freq_path"] && onboard["jetson_freq_path"].IsScalar()) {
            options.onboard_jetson_freq_path = onboard["jetson_freq_path"].as<std::string>();
        }
    }

    return true;
}

std::string default_output_path() {
    const auto now = std::chrono::system_clock::now();
    const std::string stamp = format_filename_time(now);
    return "./output/powermonitor_" + stamp + ".json";
}

std::optional<serial::PortInfo> select_port(const ConfigOptions &options) {
    std::vector<serial::PortInfo> ports = serial::list_ports();
    std::vector<serial::PortInfo> matches;
    bool any_parsed = false;
    for (const auto &port : ports) {
        if (port.hardware_id.empty()) {
            continue;
        }
        uint16_t vid = 0;
        uint16_t pid = 0;
        if (!parse_vid_pid(port.hardware_id, vid, pid)) {
            continue;
        }
        any_parsed = true;
        if (vid == options.vid && pid == options.pid) {
            matches.push_back(port);
        }
    }

    if (!options.port.empty()) {
        for (const auto &candidate : matches) {
            if (candidate.port == options.port) {
                return candidate;
            }
        }
        return std::nullopt;
    }

    if (!matches.empty()) {
        return matches.front();
    }

    if (!any_parsed && ports.size() == 1) {
        return ports.front();
    }

    return std::nullopt;
}

bool ensure_output_dir(const std::string &path) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
        if (ec) {
            std::cerr << "Failed to create directory '" << p.parent_path().string()
                      << "': " << ec.message() << "\n";
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char **argv) {
    std::signal(SIGINT, handle_signal);

    ConfigOptions options;
    options.output_path = default_output_path();

    CLI::App app{"Powermonitor PC client"};
    std::string config_path;
    std::string output_path;
    std::string port_override;
    std::string vid_hex;
    std::string pid_hex;
    uint32_t baud = options.baud;
    uint16_t period = options.stream_period_us;
    uint16_t mask = options.stream_mask;
    bool usb_stress_mode = options.usb_stress_mode;
    uint16_t config_reg = options.config_reg;
    uint16_t adc_config_reg = options.adc_config_reg;
    uint16_t shunt_cal = options.shunt_cal;
    uint16_t shunt_tempco = options.shunt_tempco;
    uint32_t duration_s = options.duration_s;
    uint64_t duration_us = options.duration_us;
    std::string run_label = options.run_label;
    std::vector<std::string> run_tags = options.run_tags;
    bool list_devices = false;
    bool no_apply_time_offset = false;
    bool debug_time_sync = false;
    bool no_onboard = false;

    auto opt_output = app.add_option("-o,--output", output_path, "Output JSON file path");
    auto opt_config = app.add_option("-c,--config", config_path, "YAML configuration file");
    auto opt_port = app.add_option("-p,--port", port_override, "Serial port");
    auto opt_baud = app.add_option("-b,--baud", baud, "Baud rate");
    auto opt_period = app.add_option("--period", period, "Sampling period in microseconds");
    auto opt_mask = app.add_option("--mask", mask, "Channel mask");
    app.add_flag("--usb-stress", usb_stress_mode,
                 "Enable device USB throughput stress mode (sets STREAM_START mask bit15)");
    auto opt_vid = app.add_option("--vid", vid_hex, "USB VID (hex)");
    auto opt_pid = app.add_option("--pid", pid_hex, "USB PID (hex)");
    auto opt_config_reg = app.add_option("--config-reg", config_reg, "INA228 config register");
    auto opt_adc_reg = app.add_option("--adc-config-reg", adc_config_reg, "INA228 ADC config");
    auto opt_shunt_cal = app.add_option("--shunt-cal", shunt_cal, "INA228 shunt calibration");
    auto opt_shunt_tempco =
        app.add_option("--shunt-tempco", shunt_tempco, "INA228 shunt tempco");
    auto opt_duration_s =
        app.add_option("--duration-s", duration_s, "Auto-stop capture after N seconds (non-interactive)");
    auto opt_duration_us =
        app.add_option("--duration-us", duration_us, "Auto-stop capture after N microseconds (non-interactive)");
    auto opt_run_label = app.add_option("--run-label", run_label, "Run label for metadata");
    auto opt_run_tag = app.add_option("--run-tag", run_tags, "Run tag (repeatable)");
    app.add_flag("-l,--list-devices", list_devices, "List available Pico USB serial devices");
    app.add_flag("-v,--verbose", options.verbose, "Verbose logging");
    app.add_flag("-i,--interactive", options.interactive, "Interactive mode");
    app.add_flag("--debug-time-sync", debug_time_sync,
                 "Print time sync diagnostics and post-sync sample timestamp deltas");
    app.add_flag("--no-apply-time-offset", no_apply_time_offset,
                 "Do not send TIME_ADJUST after time sync (measure offset only, do not correct device clock)");

    auto opt_read_core = app.add_option("--read-thread-core", options.read_thread_core, "CPU core for serial read thread");
    auto opt_proc_core = app.add_option("--proc-thread-core", options.proc_thread_core, "CPU core for data processing thread");
    auto opt_rt_prio = app.add_option("--rt-prio", options.rt_prio, "Real-time priority (SCHED_FIFO)");

    // Onboard sampler options
    auto opt_onboard = app.add_flag("--onboard", options.onboard_enabled, "Enable onboard hwmon power sampling");
    // --no-onboard is the explicit negation of --onboard (onboard_enabled defaults to false)
    auto opt_no_onboard = app.add_flag("--no-onboard", no_onboard, "Disable onboard hwmon power sampling");
    app.add_option("--onboard-path", options.onboard_hwmon_path, "Hwmon path (default: /sys/class/hwmon/hwmon1)");
    app.add_option("--onboard-period-us", options.onboard_period_us, "INA thread sampling period in microseconds (default: 1000 = 1 kHz)");
    app.add_option("--onboard-telemetry-period-us", options.onboard_telemetry_period_us, "Telemetry thread sampling period in microseconds (default: 10000 = 100 Hz)");
    app.add_option("--onboard-core", options.onboard_cpu_core, "CPU core for onboard sampler thread");
    app.add_option("--onboard-rt-prio", options.onboard_rt_prio, "RT priority for onboard sampler");
    app.add_option("--jetson-freq-path", options.onboard_jetson_freq_path,
                   "Path to /proc/jetson_freqs (default: /proc/jetson_freqs)");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }

    if (opt_config->count() > 0) {
        if (!load_yaml_config(config_path, options)) {
            return 1;
        }
    }
    if (opt_onboard->count() > 0) {
        options.onboard_enabled = true;
    }
    if (opt_no_onboard->count() > 0) {
        options.onboard_enabled = false;
    }

    if (opt_output->count() > 0) {
        options.output_path = output_path;
    }
    if (options.output_path.empty()) {
        options.output_path = default_output_path();
    }

    if (opt_port->count() > 0) {
        options.port = port_override;
    }


    if (opt_baud->count() > 0) {
        options.baud = baud;
    }

    if (opt_period->count() > 0) {
        options.stream_period_us = period;
    }

    if (opt_mask->count() > 0) {
        options.stream_mask = mask;
    }

    options.usb_stress_mode = usb_stress_mode;

    if (opt_vid->count() > 0) {
        if (!parse_hex_u16(vid_hex, options.vid)) {
            std::cerr << "Invalid VID value\n";
            return 1;
        }
    }

    if (opt_pid->count() > 0) {
        if (!parse_hex_u16(pid_hex, options.pid)) {
            std::cerr << "Invalid PID value\n";
            return 1;
        }
    }

    if (opt_config_reg->count() > 0) {
        options.config_reg = config_reg;
        options.config_overridden = true;
    }
    if (opt_adc_reg->count() > 0) {
        options.adc_config_reg = adc_config_reg;
        options.config_overridden = true;
    }
    if (opt_shunt_cal->count() > 0) {
        options.shunt_cal = shunt_cal;
        options.config_overridden = true;
    }
    if (opt_shunt_tempco->count() > 0) {
        options.shunt_tempco = shunt_tempco;
        options.config_overridden = true;
    }

    if (opt_duration_us->count() > 0) {
        options.duration_us = duration_us;
    }
    if (opt_duration_s->count() > 0) {
        options.duration_s = duration_s;
    }
    if (opt_run_label->count() > 0) {
        options.run_label = run_label;
    }
    if (opt_run_tag->count() > 0) {
        options.run_tags = run_tags;
    }

    options.no_apply_time_offset = no_apply_time_offset;
    options.debug_time_sync = debug_time_sync;

    if (!ensure_output_dir(options.output_path)) {
        return 1;
    }

    // Handle --list-devices option
    if (list_devices) {
        std::cout << "Searching for Pico USB devices (VID=" << hex_u16(options.vid)
                  << ", PID=" << hex_u16(options.pid) << ")...\n\n";

        std::vector<serial::PortInfo> all_ports = serial::list_ports();
        std::vector<serial::PortInfo> pico_devices;
        uint16_t default_vid = options.vid;
        uint16_t default_pid = options.pid;

        for (const auto &port : all_ports) {
            if (port.hardware_id.empty()) {
                continue;
            }
            uint16_t vid = 0;
            uint16_t pid = 0;
            if (parse_vid_pid(port.hardware_id, vid, pid)) {
                if (vid == default_vid && pid == default_pid) {
                    pico_devices.push_back(port);
                }
            }
        }

        if (pico_devices.empty()) {
            std::cout << "No Pico devices found.\n";
            std::cout << "\nAll available serial ports:\n";
            for (const auto &port : all_ports) {
                std::cout << "  " << port.port;
                if (!port.description.empty()) {
                    std::cout << " - " << port.description;
                }
                if (!port.hardware_id.empty()) {
                    std::cout << " [" << port.hardware_id << "]";
                }
                std::cout << "\n";
            }
            return 0;
        }

        std::cout << "Found " << pico_devices.size() << " Pico device(s):\n\n";
        for (size_t i = 0; i < pico_devices.size(); ++i) {
            const auto &port = pico_devices[i];
            std::cout << "  [" << i << "] " << port.port << "\n";
            if (!port.description.empty()) {
                std::cout << "      Description: " << port.description << "\n";
            }
            if (!port.manufacturer.empty()) {
                std::cout << "      Manufacturer: " << port.manufacturer << "\n";
            }
            if (!port.hardware_id.empty()) {
                std::cout << "      Hardware ID: " << port.hardware_id << "\n";
            }
            std::cout << "\n";
        }
        std::cout << "Use with: --port <device> or -p <device>\n";
        return 0;
    }

    std::optional<serial::PortInfo> selected_port;
    int wait_count = 0;
    while (!g_stop_requested) {
        selected_port = select_port(options);
        if (selected_port.has_value() && !selected_port->port.empty()) {
            if (wait_count > 0) {
                std::cout << "\n";
            }
            break;
        }
        if (!options.port.empty()) {
            std::cerr << "Specified port not found\n";
            return 1;
        }
        const int dots = (wait_count % 3) + 1;
        std::cout << "\rWaiting for device (VID=" << hex_u16(options.vid)
                  << ", PID=" << hex_u16(options.pid) << ")";
        for (int i = 0; i < dots; ++i) {
            std::cout << ".";
        }
        for (int i = dots; i < 3; ++i) {
            std::cout << " ";
        }
        std::cout << std::flush;
        ++wait_count;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!selected_port.has_value()) {
        std::cerr << "No matching serial port found\n";
        return 1;
    }

    options.port = selected_port->port;

    powermonitor::client::PowerMonitorSession::Options session_options;
    session_options.port = options.port;
    session_options.baud_rate = static_cast<int>(options.baud);
    session_options.output_file = options.output_path;
    session_options.stream_period_us = options.stream_period_us;
    session_options.stream_mask = options.stream_mask;
    session_options.usb_stress_mode = options.usb_stress_mode;
    session_options.verbose = options.verbose;
    session_options.interactive = options.interactive;
    session_options.debug_time_sync = options.debug_time_sync;
    session_options.duration_s = options.duration_s;
    session_options.duration_us = options.duration_us;
    session_options.run_label = options.run_label;
    session_options.run_tags = options.run_tags;
    session_options.no_apply_time_offset = options.no_apply_time_offset;
    session_options.vid_hex = hex_u16(options.vid);
    session_options.pid_hex = hex_u16(options.pid);
    session_options.read_thread_core = options.read_thread_core;
    session_options.proc_thread_core = options.proc_thread_core;
    session_options.rt_prio = options.rt_prio;
    session_options.onboard_enabled = options.onboard_enabled;
    session_options.onboard_hwmon_path = options.onboard_hwmon_path;
    session_options.onboard_period_us = options.onboard_period_us;
    session_options.onboard_telemetry_period_us = options.onboard_telemetry_period_us;
    session_options.onboard_cpu_core = options.onboard_cpu_core;
    session_options.onboard_rt_prio = options.onboard_rt_prio;
    session_options.onboard_jetson_freq_path = options.onboard_jetson_freq_path;
    // Carry INA228 register overrides into the session so SET_CFG actually
    // reaches the device. Previously these fields existed on
    // ConfigOptions but were dropped at this transfer, leaving
    // --adc-config-reg as dead code on the wire side.
    session_options.config_reg = options.config_reg;
    session_options.adc_config_reg = options.adc_config_reg;
    session_options.shunt_cal = options.shunt_cal;
    session_options.shunt_tempco = options.shunt_tempco;
    session_options.config_overridden = options.config_overridden;

    powermonitor::client::PowerMonitorSession session(session_options);
    try {
        return session.run();
    } catch (const serial::SerialException& e) {
        std::cerr << "\n[ERROR] Serial Exception: " << e.what() << std::endl;
        return 1;
    } catch (const serial::PortNotOpenedException& e) {
        std::cerr << "\n[ERROR] Port Exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] Unexpected Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "\n[ERROR] Unknown exception occurred" << std::endl;
        return 1;
    }
}
