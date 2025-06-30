/**
 * @file CostcoSubscriber.h
 * @author Eliot Abramo, Matas Jones
 *
 */
#include "CostcoSubscriber.h"
#include "Cosco.hpp"

Cosco cosco_sub;

/**
 * @brief Constructor for the costco publisher, declare the subscribers on the
 * topics and read data from ros to be sent on serial
 */
CostcoSubscriber::CostcoSubscriber() : Node("costco_subscriber") {

  RCLCPP_INFO(this->get_logger(), "Creating CostcoSubscriber");

  this->declare_parameter<std::string>("port_name", "");
  std::string port_name = this->get_parameter("port_name").as_string();
  RCLCPP_INFO(this->get_logger(), "port_name_2: %s", port_name.c_str());
  cosco_sub = Cosco(port_name);

  // Instantiate the subscribers
  // !!! topic name != message name (for LEDMessage, why? ask Gio)
  // These subscribers are binded with a handle such that when a message is
  // received, the handle is called

  this->led_message_ = this->create_subscription<custom_msg::msg::LEDMessage>(
      ("/EL/LedCommands"), 10,
        std::bind(&CostcoSubscriber::LEDHandler, this, std::placeholders::_1));

  this->servo_request_ =
      this->create_subscription<custom_msg::msg::ServoRequest>(
          ("/EL/servo_req"), 10,
          std::bind(&CostcoSubscriber::ServoRequestHandler, this,
                    std::placeholders::_1));
}

/**
 * @brief Destroy the costco subscriber, delete all the pointers and set address
 * to nullptr
 */
CostcoSubscriber::~CostcoSubscriber() {
  RCLCPP_INFO(this->get_logger(), "Deleting CostcoSubscriber");
}

// Handle for the LED message
void CostcoSubscriber::LEDHandler(const custom_msg::msg::LEDMessage::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "LED msg received");
}

// Handle for the Servo message
void CostcoSubscriber::ServoRequestHandler(const custom_msg::msg::ServoRequest::SharedPtr msg) {
  ServoRequest servoRequesteMsg;
  servoRequesteMsg.id = msg->id;
  servoRequesteMsg.increment = msg->increment;
  servoRequesteMsg.zero_in = msg->zero_in;
  RCLCPP_INFO(this->get_logger(), "Servo received");
  
  cosco_sub.sendServoRequestPacket(&servoRequesteMsg);
  RCLCPP_INFO(this->get_logger(), "Servo sent to ESP32");
}
