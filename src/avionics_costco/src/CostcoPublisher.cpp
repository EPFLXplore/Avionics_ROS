#include "CostcoPublisher.h"
#include "Cosco.hpp"
#include "serial_protocol.hpp"

#include <custom_msg/msg/mass_array.hpp>
#include <custom_msg/msg/four_in_one.hpp>
#include <custom_msg/msg/dust_data.hpp>


#include <rclcpp/rclcpp.hpp>

Cosco cosco_;

/**
 * @brief Constructor for the costco publisher, declare the publishers on the
 * topics and read data from serial
 */

CostcoPublisher::CostcoPublisher() : Node("costco_publisher") {

  RCLCPP_INFO(this->get_logger(), "Creating CostcoPublisher");

  // Instantiate the publishers
  this->mass_array_ = this->create_publisher<custom_msg::msg::MassArray>("/EL/MassArray", 10);
  this->four_in_one_ = this->create_publisher<custom_msg::msg::FourInOne>("/EL/four_in_one", 10);
  this->dust_sensor_ = this->create_publisher<custom_msg::msg::DustData>("/EL/dust_sensor", 10);

  mass_pub = this->mass_array_;
  fourinone_pub = this->four_in_one_;
  dust_pub = this->dust_sensor_;

  register_cosco_callbacks();

  /**
   * @brief right now the publisher is handled by a timer
   *
   * @brief Later on when intergrating with the serial, the timer should be
   * replaced with a when Serial.available() == true
   *
   */

  /////// REPLACE THIS WITH SERIAL HANDLER /////////
  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&CostcoPublisher::timer_callback, this));

  /////// REPLACE THIS WITH SERIAL HANDLER /////////
}

/**
 * @brief Destroy the costco publisher, delete all the pointers and set address
 * to nullptr
 */
CostcoPublisher::~CostcoPublisher() {
  RCLCPP_INFO(this->get_logger(), "Deleting CostcoPublisher");
  // delete this->mass_array_;
  // this->mass_array_ = nullptr;
}

/////// REPLACE THIS WITH SERIAL HANDLER /////////
/**
 * @brief The idea is to read the frist byte of the message which will call the
 * appropriate handle, to parse the data and send the message
 */
void CostcoPublisher::timer_callback() {
  cosco_.receive();
}

// Handle for the mass array
void CostcoPublisher::mass_array_handle(int *data) {
  RCLCPP_INFO(this->get_logger(), "In mass_array_handle");
}

// Handle for the four in one sensor
void CostcoPublisher::four_in_one_handle(int *data) {
  RCLCPP_INFO(this->get_logger(), "In four_in_one_handle");
}

// Handle for the dust sensor
void CostcoPublisher::dust_sensor_handle(int *data) {
  RCLCPP_INFO(this->get_logger(), "In dust_sensor_handle");
}