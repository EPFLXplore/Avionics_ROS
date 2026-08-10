/**
 * @file NexusMain.cpp
 * @brief Entry point: always brings up all four masters (ids 0..3).
 *
 * We construct one Nexus per possible master id regardless of what is plugged in.
 * A node whose /dev/ttyNova<id> is absent simply sits in its reconnect loop
 * (periodic "link DOWN" warning) and attaches the instant that master appears,
 * so hotplug needs no relaunch and no udev monitor. Present masters work exactly
 * as before.
 *
 * All nodes share ONE SingleThreadedExecutor: every subscription callback across
 * every node is serialised onto one thread, which is what makes _proto.send()
 * safe without a mutex (each node's RX thread is the only other user of its wire).
 */

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Nexus.h"

namespace {

/**
 * Warn if two masters are strapped to the same board id.
 *
 * The strap picks both the board profile and the USB serial ("NOVA<id>"), and
 * 99-nova.rules turns that serial into /dev/ttyNova<id>. Two boards answering to
 * one id therefore both claim the SAME symlink: udev points it at one of them,
 * the other becomes invisible to ROS, and a replug can silently move it to the
 * other physical board - same topic, same device ids, different hardware.
 *
 * Nothing else can catch this. noDuplicateIds() in the firmware checks the
 * profile TABLE, and two boards on the same strap are both legitimately row 0.
 * Nexus cannot see the second board at all, because it has no port to open. So
 * the only place with the whole picture is here, before any node exists: sysfs
 * lists every enumerated device, symlink collision or not.
 *
 * Read straight from sysfs rather than via libudev - no dependency, and the
 * three files we need are plain text.
 */
void warnOnDuplicateBoardIds(const rclcpp::Logger& log) {
    constexpr const char* VID = "0483";   // STMicroelectronics
    constexpr const char* PID = "5740";   // STM32 Virtual ComPort
    const std::filesystem::path root{"/sys/bus/usb/devices"};

    std::error_code ec;
    if (!std::filesystem::exists(root, ec)) return;   // not Linux, or no sysfs

    auto readTrimmed = [](const std::filesystem::path& p) -> std::string {
        std::ifstream f(p);
        std::string v;
        std::getline(f, v);
        return v;
    };

    std::map<std::string, std::vector<std::string>> byserial;  // serial -> device dirs
    for (const auto& e : std::filesystem::directory_iterator(root, ec)) {
        if (readTrimmed(e.path() / "idVendor")  != VID) continue;
        if (readTrimmed(e.path() / "idProduct") != PID) continue;
        const std::string serial = readTrimmed(e.path() / "serial");
        if (!serial.empty()) byserial[serial].push_back(e.path().filename().string());
    }

    for (const auto& [serial, devices] : byserial) {
        if (devices.size() < 2) continue;
        std::string where;
        for (const auto& d : devices) where += (where.empty() ? "" : ", ") + d;
        RCLCPP_ERROR(log,
            "TWO MASTERS ARE STRAPPED TO THE SAME BOARD ID: %zu devices report serial '%s' "
            "(usb %s). They both claim /dev/tty%s, so only one is reachable and a replug can "
            "move commands to the other board. Fix the PB4/PB5 straps.",
            devices.size(), serial.c_str(), where.c_str(), serial.c_str());
    }
}

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto log = rclcpp::get_logger("avionics_nexus");
    RCLCPP_INFO(log, "avionics_nexus starting: bringing up masters 0..3 (absent ones will warn + retry)");

    warnOnDuplicateBoardIds(log);

    rclcpp::executors::SingleThreadedExecutor exec;
    std::vector<std::shared_ptr<Nexus>> nodes;
    for (int id = 0; id < 4; ++id) {
        auto node = std::make_shared<Nexus>(id);
        exec.add_node(node);
        nodes.push_back(node);
    }

    RCLCPP_INFO(log, "spinning %zu nodes", nodes.size());
    exec.spin();   // returns on SIGINT (Ctrl-C)

    /* Ctrl-C stops us, not the boards. Tell every master to clear its avionics
     * LED before we let go of the wire, so the strip stops claiming this process
     * is running. spin() has returned, so the executor thread is idle and we are
     * ON it - the same thread that owns _proto.send() during normal operation,
     * which is what makes this safe without a lock. The ports are still open and
     * the RX threads still running: nothing has been torn down yet.
     *
     * SENDS ONLY. An earlier version also did nodes.clear() here, to make the
     * teardown explicit - which is a use-after-free: the nodes are still added to
     * exec, and destroying one frees the notify guard condition that exec still
     * holds a weak reference to. ~Executor then walks freed memory at the end of
     * main. Node lifetime belongs to the end of this scope, in the order it
     * already had; this only adds a message before it. */
    RCLCPP_INFO(log, "avionics_nexus stopping: clearing the avionics LED on every master");
    for (auto& node : nodes) node->announceShutdown();

    RCLCPP_INFO(log, "avionics_nexus stopped");
    rclcpp::shutdown();
    return 0;   // exec, then nodes: destroyed here, exactly as before this change
}
