/**
 * @file CoscoPublisher.cpp
 * @author Ilyas Asmouki
*/

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

    /*
        this below ensures the serial thread is safely called when
        serial data is available.
        it replaces the old timer_callback() that was bound to the 
        publisher
    */
    running_ = true;
    serial_thread_ = std::thread([this]() {
        int fd = cosco_.get_fd();
        if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid serial FD");
            return;
        }

        while (running_) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(fd, &read_fds);

            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 20000;  // 20ms, similar to old timer

            int ret = select(fd + 1, &read_fds, NULL, NULL, &timeout);
            if (ret > 0 && FD_ISSET(fd, &read_fds)) {
                cosco_.receive();
            }
        }
    });
}

/**
 * @brief Destroy the costco publisher, delete all the pointers and set address
 * to nullptr
 */
CostcoPublisher::~CostcoPublisher() {
  RCLCPP_INFO(this->get_logger(), "Deleting CostcoPublisher");
  running_ = false;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
}

/**
 * @brief The idea is to read the frist byte of the message which will call the
 * appropriate handle, to parse the data and send the message
 */

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