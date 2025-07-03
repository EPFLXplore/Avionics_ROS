/**
 * @file CostcoSubscriber.h
 * @author Eliot Abramo, Matas Jones
 *
 */

#ifndef COSTCO_SUBSCRIBER_H
#define COSTCO_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include <thread>

// Include the custom messages
#include "custom_msg/msg/led_message.hpp"
#include "custom_msg/msg/servo_request.hpp"
#include "custom_msg/msg/mass_request_hd.hpp"
#include "custom_msg/msg/mass_request_drill.hpp"

// Create the CostcoPublisher class, it is used to setup the topics
// (subscriptions and publishers)
class CostcoSubscriber : public rclcpp::Node {
public:
  /**
   * @brief Read the data coming from the serial and call the appropriate handle
   * depending on the serial message id
   *
   */
  CostcoSubscriber();
  ~CostcoSubscriber();

private:
  /**
   * @brief Create a shared pointer which will be used to reference the publisher in the cpp
   * 
   */
  // rclcpp::Subscription<custom_msg::msg::LEDMessage>::SharedPtr led_message_;
  rclcpp::Subscription<custom_msg::msg::ServoRequest>::SharedPtr servo_request_;
  rclcpp::Subscription<custom_msg::msg::MassRequestHD>::SharedPtr mass_request_hd_;
  rclcpp::Subscription<custom_msg::msg::MassRequestDrill>::SharedPtr mass_request_drill_;

  /**
   * @brief General layout for handles: reveive ros message, wrap it into a
   * serial msg, send it to serial
   *
   * @param msg The message received from the ros topic
   */
  void ServoRequestHandler(const custom_msg::msg::ServoRequest::SharedPtr msg);
  void MassRequestHDHandler(const custom_msg::msg::MassRequestHD::SharedPtr msg);
  void MassRequestDrillHandler(const custom_msg::msg::MassRequestDrill::SharedPtr msg);

};

#endif