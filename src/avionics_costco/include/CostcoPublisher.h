/**
 * @file CostcoPublisher.h
 * @author Matas Jones
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

// Include the custom messages
#include "custom_msg/msg/dust_data.hpp"
#include "custom_msg/msg/four_in_one.hpp"
#include "custom_msg/msg/mass_array.hpp"

// Define the case used to call the indivdual cases depending on the message id
#define CASE_MASS_ARRAY 0
#define CASE_FOUR_IN_ONE 1
#define CASE_DUST_SENSOR 2

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
  rclcpp::Publisher<custom_msg::msg::MassArray>::SharedPtr mass_array_;

  rclcpp::Publisher<custom_msg::msg::FourInOne>::SharedPtr four_in_one_;

  rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_sensor_;

  // Create a timer to read the data from the serial
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread serial_thread_;
  std::atomic<bool> running_;

  // Declare the different handle functions
  /**
   * @brief General handle layout: parse data depending of message type, prepare
   * fill message with data, publish message
   *
   * @param data is the data from the serial
   */
  void mass_array_handle(int *data);
  void four_in_one_handle(int *data);
  void dust_sensor_handle(int *data);

  // Is temporary and will be replaced with the serial handler
  void timer_callback();
};

#endif /* COSTCOPUBLISHER_H */