/**
 * @file CostcoPublisher.h
 * @author Eliot Abramo, Matas Jones
 *
 * @brief CostcoPublisher is the cost effective communication module between ROS
 * and the sensors It recieves data from the sensors via serial and sends this
 * data via ROS on ethernet
 *
 * @date 2025-03-18
 */

#ifndef COSTCOPUBLISHER_H
#define COSTCOPUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <atomic>
#include "Cosco.hpp"

// Include the custom messages
#include "custom_msg/msg/servo_response.hpp"
#include "custom_msg/msg/dust_data.hpp"
#include "custom_msg/msg/mass_packet.hpp"
#include "custom_msg/msg/heartbeat.hpp"

// Create the CostcoPublisher class, it is used to setup the topics and the
// handles
class CostcoPublisher : public rclcpp::Node {
public:
  /**
   * @brief Read the data coming from the serial and call the appropriate handle
   * depending on the serial message id
   *
   */
  CostcoPublisher();
  ~CostcoPublisher();

private:
  // Create a shared pointer which will be used to reference the publisher in
  // the cpp
  rclcpp::Publisher<custom_msg::msg::ServoResponse>::SharedPtr servo_response_;
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

#endif /* COSTCOPUBLISHER_H */