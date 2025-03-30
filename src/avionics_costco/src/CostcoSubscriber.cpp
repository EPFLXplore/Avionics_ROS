#include "CostcoSubscriber.h"
#include "Cosco.hpp"

/**
 * @brief Constructor for the costco publisher, declare the subscribers on the
 * topics and read data from ros to be sent on serial
 */
CostcoSubscriber::CostcoSubscriber() : Node("costco_subscriber") {

  RCLCPP_INFO(this->get_logger(), "Creating CostcoSubscriber");

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

  // delete this->mass_array_;
  // this->mass_array_ = nullptr;
}

// Handle for the LED message
void CostcoSubscriber::LEDHandler(const custom_msg::msg::LEDMessage::SharedPtr msg) {
  uint8_t low = msg->low;
  uint8_t high = msg->high;
  uint8_t system = msg->system;
  uint8_t mode = msg->mode;
  RCLCPP_INFO(this->get_logger(), "LED msg received");

  // TODO: Send to serial
}

// Handle for the Servo message
void CostcoSubscriber::ServoRequestHandler(const custom_msg::msg::ServoRequest::SharedPtr msg) {
  ServoRequest servoRequesteMsg;
  servoRequesteMsg.id = msg->id;
  servoRequesteMsg.increment = msg->increment;
  servoRequesteMsg.zero_in = msg->zero_in;
  RCLCPP_INFO(this->get_logger(), "Servo received");

  Cosco coscoSend;
  coscoSend.sendServoRequestPacket(&servoRequesteMsg);
  RCLCPP_INFO(this->get_logger(), "Servo sent to ESP32");
}
