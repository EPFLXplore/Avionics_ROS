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

#include "SerialHandler.hpp"
#include "rclcpp/rclcpp.hpp"

// Create the CostcoPublisher class, it is used to setup the topics
// (subscriptions and publishers)
class CostcoPublisher : public rclcpp::Node {
public:
  CostcoPublisher();
  ~CostcoPublisher();

private:
  // Create a shared pointer which will be used to reference the publisher in
  // the cpp
  rclcpp::Publisher<custom_msg::msg::MassArray> *mass_array_;

  // Declare the different functions used by CostcoPublisher

  /**
   * @brief publish_to_ros: publishes data from serial to ros
   *
   * @param data: is the data recieved by serial to be sent via ros
   */
  CostcoPublisher::publish_to_ros(custom_msg::msg::MassArray data);
};

#endif /* COSTCOPUBLISHER_H */