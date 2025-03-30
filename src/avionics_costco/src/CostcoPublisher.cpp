#include "CostcoPublisher.h"

/**
 * @brief Constructor for the costco publisher, declare the publishers on the
 * topics and read data from serial
 */

CostcoPublisher::CostcoPublisher() : Node("costco_publisher") {

  RCLCPP_INFO(this->get_logger(), "Creating CostcoPublisher");

  // Instantiate the publishers
  this->mass_array_ = this->create_publisher<custom_msg::msg::MassArray>(("/EL/MassArray"), 10);

  this->four_in_one_ = this->create_publisher<custom_msg::msg::FourInOne>(("/EL/four_in_one"), 10);

  this->dust_sensor_ = this->create_publisher<custom_msg::msg::DustData>(("/EL/dust_sensor"), 10);

  // this->
  /**
   * @brief right now the publisher is handled by a timer
   *
   * @brief Later on when intergrating with the serial, the timer should be
   * replaced with a when Serial.available() == true
   *
   */

  /////// REPLACE THIS WITH SERIAL HANDLER /////////
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
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
  // Read data coming from the serial
  int data = 9999;
  // TODO     //SerialHandler::receive(*data);

  // Extract the id byte of the message

  // TODO
  uint8_t id = 0x0;

  // Select handle based on id
  switch (id) {
  case CASE_MASS_ARRAY:
    // Call handle
    CostcoPublisher::mass_array_handle(&data);
    break;

  case CASE_FOUR_IN_ONE:
    // Call handle
    CostcoPublisher::four_in_one_handle(&data);
    break;

  case CASE_DUST_SENSOR:
    // Call handle
    CostcoPublisher::dust_sensor_handle(&data);
    break;

  default:
    // The message could not be parsed
    break;
  }
}

// Handle for the mass array
void CostcoPublisher::mass_array_handle(int *data) {
  RCLCPP_INFO(this->get_logger(), "In mass_array_handle");
  // Initialize the custom_message
  auto msg = custom_msg::msg::MassArray();

  // Parse the data based on the message struct

  // TODO

  // Add the data to the custom_message
  // msg.id = 10001;
  // msg.mass[0] = 1.1;
  // msg.mass[1] = 2.2;
  // msg.mass[2] = 3.3;
  // msg.mass[3] = 4.4;

  // // Publish the custom message
  // mass_array_->publish(msg);
}

// Handle for the four in one sensor
void CostcoPublisher::four_in_one_handle(int *data) {
  RCLCPP_INFO(this->get_logger(), "In four_in_one_handle");
  // Initialize the custom_message
  auto msg = custom_msg::msg::FourInOne();

  // Parse the data based on the message struct

  // TODO

  // Add the data to the custom_message
  msg.id = 10110;
  msg.temperature = 1.3;
  msg.moisture = 3.14;
  msg.conductivity = 5.21;
  msg.ph = 6.9; // Noice

  // Publish the custom message
  four_in_one_->publish(msg);
}

// Handle for the dust sensor
void CostcoPublisher::dust_sensor_handle(int *data) {
  RCLCPP_INFO(this->get_logger(), "In dust_sensor_handle");
  // Initialize the custom_message
  auto msg = custom_msg::msg::DustData();

  // Parse the data based on the message struct

  // TODO

  // Add the data to the custom_message
  // msg.dust_msg = 1234;

  // Publish the custom message
  // dust_sensor_->publish(msg);
}