/**
 * @file NexusPublisher.cpp
 * @author Eliot Abramo, Ilyas Asmouki
 * @brief Publisher Implementation of Nexus node
 * @date 2025-07-03
 */

 #include "NexusPublisher.h"
#include "Nexus.hpp"

#include <custom_msg/msg/dust_data.hpp>
#include <custom_msg/msg/mass_packet.hpp>

#include <rclcpp/rclcpp.hpp>

std::unique_ptr<Nexus> nexus_;

/**
 * @brief Constructor for the nexus publisher, declare the publishers on the
 * topics and read data from serial
 */
NexusPublisher::NexusPublisher() : Node("nexus_publisher") {

    RCLCPP_INFO(this->get_logger(), "Creating NexusPublisher");

    // take port name from launch file, allows us to not hardcode port name in case we want to change dev rule.
    // look at launch.py
    this->declare_parameter<std::string>("port_name", "");
    std::string port_name = this->get_parameter("port_name").as_string();

    RCLCPP_INFO(this->get_logger(), "port_name: %s", port_name.c_str());

    // create nexus instance
    nexus_ = std::make_unique<Nexus>(port_name, 115200); 

    // Instantiate the publishers
    this->dust_sensor_ = this->create_publisher<custom_msg::msg::DustData>("/EL/dust_sensor", 10);
    this->mass_packet_ = this->create_publisher<custom_msg::msg::MassPacket>("/EL/mass_packet", 10);
    this->heartbeat_ = this->create_publisher<custom_msg::msg::Heartbeat>("/EL/heartbeat_packet", 10);

    dust_pub = this->dust_sensor_;
    mass_pub = this->mass_packet_;
    heartbeat_pub = this->heartbeat_;

    running_ = true;
    serial_thread_ = std::thread([this]() {
        while (running_) {
            try {
                /* Blocks until one complete, CRC-valid frame arrives.
                On success Nexus immediately calls the corresponding
                callback. */
                nexus_->readOne();
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
 * @brief Destroy the nexus publisher, delete all the pointers and set address
 * to nullptr
 */
NexusPublisher::~NexusPublisher() {
  RCLCPP_INFO(this->get_logger(), "Deleting NexusPublisher");
  running_ = false;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
}
