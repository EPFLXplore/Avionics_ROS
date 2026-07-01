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
 * every node is serialised onto one thread, which is what makes proto_.send()
 * safe without a mutex (each node's RX thread is the only other user of its wire).
 */

#include <memory>
#include <vector>

#include "Nexus.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto log = rclcpp::get_logger("avionics_nexus");
    RCLCPP_INFO(log, "avionics_nexus starting: bringing up masters 0..3 (absent ones will warn + retry)");

    rclcpp::executors::SingleThreadedExecutor exec;
    std::vector<std::shared_ptr<Nexus>> nodes;
    for (int id = 0; id < 4; ++id) {
        auto node = std::make_shared<Nexus>(id);
        exec.add_node(node);
        nodes.push_back(node);
    }

    RCLCPP_INFO(log, "spinning %zu nodes", nodes.size());
    exec.spin();

    RCLCPP_INFO(log, "avionics_nexus stopped");
    rclcpp::shutdown();
    return 0;
}
