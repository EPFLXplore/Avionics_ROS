/**
 * @file CostcoPublisher.cpp
 * @author Eliot Abramo, Ilyas Asmouki
 * @brief Publisher Implementation of Cosco node
 * @date 2025-07-03
 */

 #include "CostcoPublisher.h"
#include "Cosco.hpp"

#include <custom_msg/msg/servo_response.hpp>
#include <custom_msg/msg/dust_data.hpp>
#include <custom_msg/msg/mass_packet.hpp>


#include <rclcpp/rclcpp.hpp>

std::unique_ptr<Cosco> cosco_;

/**
 * @brief Constructor for the costco publisher, declare the publishers on the
 * topics and read data from serial
 */
CostcoPublisher::CostcoPublisher() : Node("costco_publisher") {

    RCLCPP_INFO(this->get_logger(), "Creating CostcoPublisher");

    this->declare_parameter<std::string>("port_name", "");
    std::string port_name = this->get_parameter("port_name").as_string();
    RCLCPP_INFO(this->get_logger(), "port_name: %s", port_name.c_str());
    cosco_ = std::make_unique<Cosco>(port_name, 115200);

    // Instantiate the publishers
    this->servo_response_ = this->create_publisher<custom_msg::msg::ServoResponse>("/EL/servo_response", 10);
    this->dust_sensor_ = this->create_publisher<custom_msg::msg::DustData>("/EL/dust_sensor", 10);
    this->mass_packet_ = this->create_publisher<custom_msg::msg::MassPacket>("/EL/mass_packet", 10);
    this->heartbeat_ = this->create_publisher<custom_msg::msg::Heartbeat>("/EL/heartbeat_packet", 10);

    servo_response_pub = this->servo_response_;
    dust_pub = this->dust_sensor_;
    mass_pub = this->mass_packet_;
    heartbeat_pub = this->heartbeat_;

    running_ = true;
    serial_thread_ = std::thread([this]() {
        while (running_) {
            try {
                /* Blocks until one complete, CRC-valid frame arrives.
                On success Cosco immediately calls the corresponding
                callback. */
                cosco_->readOne();
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                            "Serial error in readOne(): %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
