/*
 * BRoCoSubscriber.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoSubscriber.h"
#include "BRoCoManager.h"
#include "BRoCo/CanSocketDriver.h"

#include "Utils.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

BRoCoSubscriber::BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent) : bus(bus), parent(parent) {
    this->clk = parent->get_clock();
    RCLCPP_INFO(parent->get_logger(), "Creating subscribers");
    this->mass_config_req_sub = parent->create_subscription<custom_msg::msg::MassConfigRequestJetson>
        (get_prefix() + get_param<std::string>("MASS_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::massConfigReqCallback, this, _1));
    this->servo_config_req_sub = parent->create_subscription<custom_msg::msg::ServoConfigRequestJetson>
        (get_prefix() + get_param<std::string>("SERVO_CONFIG_REQ_JETSON_TOPIC"), 10, std::bind(&BRoCoSubscriber::servoConfigReqCallback, this, _1));
    
    this->mass_drill_calib_offset_sub = parent->create_subscription<custom_msg::msg::MassCalibOffset>
        (get_prefix() + get_param<std::string>("DRILL_MASS_CALIB_OFFSET_TOPIC"), 10, std::bind(&BRoCoSubscriber::massDrillCalibOffsetCallback, this, _1));
    this->mass_container_calib_offset_sub = parent->create_subscription<custom_msg::msg::MassCalibOffset>
        (get_prefix() + get_param<std::string>("CONTAINER_MASS_CALIB_OFFSET_TOPIC"), 10, std::bind(&BRoCoSubscriber::massContainerCalibOffsetCallback, this, _1));
    this->mass_drill_calib_scale_sub = parent->create_subscription<custom_msg::msg::MassCalibScale>
        (get_prefix() + get_param<std::string>("DRILL_MASS_CALIB_SCALE_TOPIC"), 10, std::bind(&BRoCoSubscriber::massDrillCalibScaleCallback, this, _1));
    this->mass_container_calib_scale_sub = parent->create_subscription<custom_msg::msg::MassCalibScale>
        (get_prefix() + get_param<std::string>("CONTAINER_MASS_CALIB_SCALE_TOPIC"), 10, std::bind(&BRoCoSubscriber::massContainerCalibScaleCallback, this, _1));

    RCLCPP_INFO(parent->get_logger(), "Subscribers created");
}

void BRoCoSubscriber::servoReqCallback(const custom_msg::msg::ServoRequest::SharedPtr msg) {
    // uint16_t id = 0;
    // if (msg->destination_id != 0)
    //     id = msg->destination_id;
    // else
    //     id = get_node_id("HD_NODE_ID");
    // RCLCPP_INFO(parent->get_logger(), "Sending servo request to node ID %s...", std::to_string(id).c_str());
    // static ServoPacket packet;
    // packet.channel = msg->channel;
    // packet.angle = msg->angle;
    // MAKE_IDENTIFIABLE(packet);
    // MAKE_RELIABLE(packet);
    // set_destination_id(id);
    // bus->send(&packet);
}

void BRoCoSubscriber::massConfigReqCallback(const custom_msg::msg::MassConfigRequestJetson::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("MASS_DRILL_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Mass config to node ID %s...", std::to_string(id).c_str());
    static MassConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_offset = msg->set_offset;
    packet.set_alpha = msg->set_alpha;
    packet.set_channels_status = msg->set_channels_status;

    for (uint8_t i = 0; i < 4; ++i) {
        packet.offset[i] = msg->offset[i];
        packet.scale[i] = msg->scale[i];
        if (msg->scale[i] = 0) {
            RCLCPP_INFO(parent->get_logger(), "Scale for channel %d is zero, not sending scale configuration", i+1);
            packet.set_scale = false;
        }
        packet.enabled_channels[i] = msg->enabled_channels[i];
    }

    packet.alpha = msg->alpha;

    MAKE_IDENTIFIABLE(packet);
    MAKE_RELIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::servoConfigReqCallback(const custom_msg::msg::ServoConfigRequestJetson::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("HD_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending Servo config to node ID %s...", std::to_string(id).c_str());
    static ServoConfigPacket packet;
    packet.remote_command = msg->remote_command;
    packet.set_min_duty = msg->set_min_duty;
    packet.set_max_duty = msg->set_max_duty;
    packet.set_min_angles = msg->set_min_angles;
    packet.set_max_angles = msg->set_max_angles;

    float min_duty[4];
    float max_duty[4];
    float min_angles[4];
    float max_angles[4];

    for (uint8_t i = 0; i < 4; ++i) {
        min_duty[i] = msg->min_duty[i];
        max_duty[i] = msg->max_duty[i];
        min_angles[i] = msg->min_angles[i];
        max_angles[i] = msg->max_angles[i];
    }

    packet.min_duty_max_val = get_max_val(min_duty, 4);
    packet.max_duty_max_val = get_max_val(max_duty, 4);
    packet.min_angles_max_val = get_max_val(min_angles, 4);
    packet.max_angles_max_val = get_max_val(max_angles, 4);

    for (uint8_t i = 0; i < 4; ++i) {
        packet.min_duty[i] = floatToScaledUInt16(min_duty[i], packet.min_duty_max_val);
        packet.max_duty[i] = floatToScaledUInt16(max_duty[i], packet.max_duty_max_val);
        packet.min_angles[i] = floatToScaledUInt16(min_angles[i], packet.min_angles_max_val);
        packet.max_angles[i] = floatToScaledUInt16(max_angles[i], packet.max_angles_max_val);
    }
    
    MAKE_IDENTIFIABLE(packet);
    MAKE_RELIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}


void BRoCoSubscriber::massDrillCalibOffsetCallback(const custom_msg::msg::MassCalibOffset::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_DRILL_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending offset calibration request to node ID %s...", std::to_string(id).c_str());
    static MassCalibPacket packet;
    packet.calib_offset = true;
    packet.calib_scale = false;
    packet.channel = msg->channel;
    packet.expected_weight = 1;

    MAKE_IDENTIFIABLE(packet);
    MAKE_RELIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::massContainerCalibOffsetCallback(const custom_msg::msg::MassCalibOffset::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_CONTAINER_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending offset calibration request to node ID %s...", std::to_string(id).c_str());
    static MassCalibPacket packet;
    packet.calib_offset = true;
    packet.calib_scale = false;
    packet.channel = msg->channel;
    packet.expected_weight = 1;

    MAKE_IDENTIFIABLE(packet);
    MAKE_RELIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::massDrillCalibScaleCallback(const custom_msg::msg::MassCalibScale::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_DRILL_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending scale calibration request to node ID %s...", std::to_string(id).c_str());
    static MassCalibPacket packet;
    packet.calib_offset = false;
    packet.calib_scale = true;
    packet.channel = msg->channel;
    packet.expected_weight = msg->expected_weight;

    MAKE_IDENTIFIABLE(packet);
    MAKE_RELIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

void BRoCoSubscriber::massContainerCalibScaleCallback(const custom_msg::msg::MassCalibScale::SharedPtr msg) {
    uint16_t id = 0;
    if (msg->destination_id != 0)
        id = msg->destination_id;
    else
        id = get_node_id("SC_CONTAINER_NODE_ID");

    RCLCPP_INFO(parent->get_logger(), "Sending scale calibration request to node ID %s...", std::to_string(id).c_str());
    static MassCalibPacket packet;
    packet.calib_offset = false;
    packet.calib_scale = true;
    packet.channel = msg->channel;
    packet.expected_weight = msg->expected_weight;

    MAKE_IDENTIFIABLE(packet);
    MAKE_RELIABLE(packet);
    set_destination_id(id);
    bus->send(&packet);
}

uint32_t BRoCoSubscriber::get_node_id(std::string node_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
}

void BRoCoSubscriber::set_destination_id(std::string node_name) {
    uint16_t id = dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

void BRoCoSubscriber::set_destination_id(uint16_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

std::string BRoCoSubscriber::get_prefix() {
  return dynamic_cast<BRoCoManager*>(parent)->get_prefix();
}

std::string BRoCoSubscriber::get_bus() {
  return dynamic_cast<BRoCoManager*>(parent)->get_bus();
}

template <typename T>
void BRoCoSubscriber::set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value) {
    dynamic_cast<BRoCoManager*>(parent)->set_param_calib(sensor, parameter_name, value);
}

template <typename T>
T BRoCoSubscriber::get_param(const std::string& parameter_name) {
  return dynamic_cast<BRoCoManager*>(parent)->get_param<T>(parameter_name);
}