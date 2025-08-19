/**
 * @file NexusPublisher.h
 * @author Eliot Abramo, Matas Jones
 *
 * @brief NexusPublisher is the cost effective communication module between ROS
 * and the sensors It recieves data from the sensors via serial and sends this
 * data via ROS on ethernet
 *
 * @date 2025-03-18
 */

#ifndef NEXUSPUBLISHER_H
#define NEXUSPUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include <thread>
/*
* atomic elements means that all operations on them are indivisible and guarenteed to complete
* without being interrupted by other threads. Ensures thread safe access to shared data.
*/ 
#include <atomic> 
#include "Nexus.hpp"

// Include the custom messages
#include "custom_msg/msg/dust_data.hpp"
#include "custom_msg/msg/mass_packet.hpp"
#include "custom_msg/msg/heartbeat.hpp"

// Create the NexusPublisher class, it is used to setup the topics and the handles
class NexusPublisher : public rclcpp::Node {
public:
  /**
   * @brief Read the data coming from the serial and call the appropriate handle
   * depending on the serial message id
   *
   */
  NexusPublisher();
  ~NexusPublisher();

private:
  // Create a shared pointer which will be used to reference the publisher in the cpp
  rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_sensor_;
  rclcpp::Publisher<custom_msg::msg::MassPacket>::SharedPtr mass_packet_;
  rclcpp::Publisher<custom_msg::msg::Heartbeat>::SharedPtr heartbeat_;

  // Create a timer to read the data from the serial
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread serial_thread_;
  std::atomic<bool> running_;

  // Declare the different handle functions
  void mass_packet_handle(int *data);
  void dust_sensor_handle(int *data);
};

#endif /* NEXUSPUBLISHER_H */