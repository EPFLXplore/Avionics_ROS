/*
 * roco_interface.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"
#include "BRoCoManager.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("roco_interface");
    RCLCPP_INFO(node->get_logger(), "Creating node...");

    rclcpp::spin(std::make_shared<BRoCoManager>());
    rclcpp::shutdown();
    return 0;
}