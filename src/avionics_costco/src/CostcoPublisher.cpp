/**
 * @file CoscoPublisher.cpp
 * @author Ilyas Asmouki
*/

#include "CostcoPublisher.h"
#include "Cosco.hpp"
#include "serial_protocol.hpp"

#include <custom_msg/msg/servo_response.hpp>
#include <custom_msg/msg/dust_data.hpp>
#include <custom_msg/msg/mass_packet.hpp>


#include <rclcpp/rclcpp.hpp>

Cosco cosco_;

/**
 * @brief Constructor for the costco publisher, declare the publishers on the
 * topics and read data from serial
 */

CostcoPublisher::CostcoPublisher() : Node("costco_publisher") {

    RCLCPP_INFO(this->get_logger(), "Creating CostcoPublisher");

    // Instantiate the publishers
    this->servo_response_ = this->create_publisher<custom_msg::msg::ServoResponse>("/EL/servo_response", 10);
    this->dust_sensor_ = this->create_publisher<custom_msg::msg::DustData>("/EL/dust_sensor", 10);
    this->mass_packet_ = this->create_publisher<custom_msg::msg::MassPacket>("/EL/mass_packet", 10);

    servo_response_pub = this->servo_response_;
    dust_pub = this->dust_sensor_;
    mass_pub = this->mass_packet_;

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
