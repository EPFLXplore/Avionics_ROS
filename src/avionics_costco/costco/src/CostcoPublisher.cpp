#include "CostcoPublisher.h"

/**
 * @brief Constructor for the costco publisher, declare the publishers on the
 * topics and read data from serial
 */

CostcoPublisher::CostcoPublisher() : Node("costco_publisher") {

  this->mass_array_ =
      this->publisher<custom_msg::msg::MassArray>(("CostcoPublisher"), 10);

  while (true) {
    SerialHandler::receive(*data);
  }
}

/**
 * @brief Destroy the costco publisher, delete all the pointers and set address
 * to nullptr
 */
CostcoPublisher::~CostcoPublisher() {
  RCLCPP_INFO(this->get_logger(), "Deleting CostcoPublisher");
  delete this->mass_array;
  this->mass_array = nullptr;
}

CostcoPublisher::publish_to_ros(custom_msg::msg::MassArray data) {
  mass_array_->publish(data);
}