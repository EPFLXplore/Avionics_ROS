/*
 * avionics_mux.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"
#include "MuxManager.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("avionics_mux");
  RCLCPP_INFO(node->get_logger(), "Creating node...");

  rclcpp::spin(std::make_shared<MuxManager>());
  rclcpp::shutdown();
  return 0;
}