/*
 * avionics_mux.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "MuxManager.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  // Initialize the ROS2 client library with arguments of main. Sets up the
  // necessary communication infrastructure.
  rclcpp::init(argc, argv);

  // Creates a new ROS2 node named avionics_mux
  auto node = rclcpp::Node::make_shared("avionics_mux");

  // logs an message to indicate that the node creation has begun
  RCLCPP_INFO(node->get_logger(), "Creating node...");

  // Creates a shared instance of MuxManager and gives control to ROS2 event
  // loop AKA spinner Spinner will keep program running processing callbacks -->
  // i.e. subscriptions, timers, etc. Continues until shutdown signal received
  // Node runs inside the ::spin() in a loop, blocks the ::shutdown() signal
  // until interrupt signal in ::spin() makes it exit and then the ::shutdown()
  // command is executed
  // rclcpp::spin(std::make_shared<MuxManager>());

  // Cleans up and shuts down ROS2 client library
  // rclcpp::shutdown();

  // C++ vibes be like
  return 0;
}