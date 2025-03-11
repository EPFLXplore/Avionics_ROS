/*
 * BRoCoPublisher.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "BRoCoPublisher.h"
#include "BRoCoManager.h"

#include "Utils.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

BRoCoPublisher::BRoCoPublisher(CANBus* bus, rclcpp::Node* parent) : bus(bus), parent(parent) {
    this->clk = parent->get_clock();
    RCLCPP_INFO(parent->get_logger(), "Creating publishers");
    this->timer = parent->create_wall_timer(std::chrono::milliseconds(get_param<uint32_t>("NODE_PING_INTERVAL")), std::bind(&BRoCoPublisher::timerPingCallback, this));
    this->node_state_pub_timer = parent->create_wall_timer(std::chrono::milliseconds(get_param<uint32_t>("NODE_STATE_PUBLISH_INTERVAL")), std::bind(&BRoCoPublisher::nodeStateCallback, this));
    this->voltage_pub = parent->create_publisher<custom_msg::msg::Voltage>(get_prefix() + get_param<std::string>("VOLTAGE_TOPIC"), 10);
    this->drill_mass_pub = parent->create_publisher<custom_msg::msg::MassArray>(get_prefix() + get_param<std::string>("DRILL_MASS_TOPIC"), 10);
    this->container_mass_pub = parent->create_publisher<custom_msg::msg::MassArray>(get_prefix() + get_param<std::string>("CONTAINER_MASS_TOPIC"), 10);
    this->servo_response_pub = parent->create_publisher<custom_msg::msg::ServoResponse>(get_prefix() + get_param<std::string>("SERVO_TOPIC"), 10);

    this->mass_config_req_pub = parent->create_publisher<custom_msg::msg::MassConfigRequestMCU>(get_prefix() + get_param<std::string>("MASS_CONFIG_REQ_MCU_TOPIC"), 10);
    this->mass_config_response_pub = parent->create_publisher<custom_msg::msg::MassConfigResponse>(get_prefix() + get_param<std::string>("MASS_CONFIG_TOPIC"), 10);

    this->servo_config_req_pub = parent->create_publisher<custom_msg::msg::ServoConfigRequestMCU>(get_prefix() + get_param<std::string>("SERVO_CONFIG_REQ_MCU_TOPIC"), 10);
    this->servo_config_response_pub = parent->create_publisher<custom_msg::msg::ServoConfigResponse>(get_prefix() + get_param<std::string>("SERVO_CONFIG_TOPIC"), 10);

    RCLCPP_INFO(parent->get_logger(), "Publishers created");

    RCLCPP_INFO(parent->get_logger(), "Adding handles...");
    bus->handle<MassPacket>(std::bind(&BRoCoPublisher::handleMassPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<ServoResponsePacket>(std::bind(&BRoCoPublisher::handleServoPacket, this, std::placeholders::_1, std::placeholders::_2));
    
    bus->handle<MassConfigRequestPacket>(std::bind(&BRoCoPublisher::handleMassConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<MassConfigResponsePacket>(std::bind(&BRoCoPublisher::handleMassConfigPacket, this, std::placeholders::_1, std::placeholders::_2));

    bus->handle<ServoConfigRequestPacket>(std::bind(&BRoCoPublisher::handleServoConfigReqPacket, this, std::placeholders::_1, std::placeholders::_2));
    bus->handle<ServoConfigResponsePacket>(std::bind(&BRoCoPublisher::handleServoConfigPacket, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(parent->get_logger(), "Handles created");

    node_state.resize(get_param<uint32_t>("MAX_NUMBER_NODES"), false);
    watchdog_timers.resize(node_state.size());
    
    for (size_t i = 0; i < watchdog_timers.size(); ++i) {
        watchdog_timers[i] = parent->create_wall_timer(
            std::chrono::milliseconds(get_param<uint32_t>("NODE_STATE_WATCHDOG_TIMEOUT")),
            [this, i]() {
                this->watchdogCallback(i);
            }
        );
    }
}

void BRoCoPublisher::timerPingCallback() {
    static PingPacket packet;
    MAKE_IDENTIFIABLE(packet);
    MAKE_RELIABLE(packet);
    set_destination_id("GENERAL_NODE_ID");
    bus->send(&packet);
}

void BRoCoPublisher::nodeStateCallback() {
    auto msg = custom_msg::msg::NodeStateArray();
    for (int i = 0; i < node_state.size(); ++i)
    msg.node_state.push_back(node_state[i]);
    node_state_pub->publish(msg);
}

void BRoCoPublisher::watchdogCallback(size_t nodeID) {
    // This callback is triggered when the timer for nodeID expires
    node_state[nodeID] = false;
}

void BRoCoPublisher::handleMassPacket(uint8_t senderID, MassPacket* packet) {
    if(!(IS_RELIABLE(*packet))) {
		RCLCPP_ERROR(parent->get_logger(), "Unreliable mass packet");
		return;
	}
    auto msg = custom_msg::msg::MassArray();

    msg.id = packet->id;

    for (uint8_t i = 0; i < msg.mass.size(); ++i)
        msg.mass[i] = packet->mass[i];
    if (packet->id == get_node_id("SC_DRILL_NODE_ID"))
        drill_mass_pub->publish(msg);
    else if (packet->id == get_node_id("SC_CONTAINER_NODE_ID"))
        container_mass_pub->publish(msg);
    else
        RCLCPP_INFO(parent->get_logger(), "Mass packet received but ID is not valid");
}

void BRoCoPublisher::handleServoPacket(uint8_t senderID, ServoResponsePacket* packet) {
    if(!(IS_RELIABLE(*packet))) {
		RCLCPP_ERROR(parent->get_logger(), "Unreliable servo packet");
		return;
	}
    auto msg = custom_msg::msg::ServoResponse();

    msg.id = packet->id;

    msg.channel = packet->channel;
    msg.angle = packet->angle;
    msg.success = packet->success;

    servo_response_pub->publish(msg);
}

void BRoCoPublisher::handleMassConfigReqPacket(uint8_t senderID, MassConfigRequestPacket* packet) {
    if(!(IS_RELIABLE(*packet))) {
		RCLCPP_ERROR(parent->get_logger(), "Unreliable mass config request packet");
		return;
	}
    auto msg = custom_msg::msg::MassConfigRequestMCU();

    msg.id = packet->id;

    msg.req_offset = packet->req_offset;
    msg.req_scale = packet->req_scale;
    msg.req_alpha = packet->req_alpha;
    msg.req_channels_status = packet->req_channels_status;

    mass_config_req_pub->publish(msg);
}

void BRoCoPublisher::handleMassConfigPacket(uint8_t senderID, MassConfigResponsePacket* packet) {
    if(!(IS_RELIABLE(*packet))) {
		RCLCPP_ERROR(parent->get_logger(), "Unreliable mass config response packet");
		return;
	}
    auto msg = custom_msg::msg::MassConfigResponse();

    msg.id = packet->id;

    msg.set_offset = packet->set_offset;
    msg.set_scale = packet->set_scale;
    msg.set_alpha = packet->set_alpha;
    msg.set_channels_status = packet->set_channels_status;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.offset[i] = packet->offset[i];
        msg.scale[i] = packet->scale[i];
        msg.enabled_channels[i] = packet->enabled_channels[i];
    }
    msg.alpha = packet->alpha;

    // Set parameters inside parameter server
    std::string sensor;
    bool valid_id = false;
    if (packet->id == get_node_id("SC_DRILL_NODE_ID")) {
        sensor = "mass_drill";
        valid_id = true;
    } else if (packet->id == get_node_id("SC_CONTAINER_ID")) {
        sensor = "mass_container";
        valid_id = true;
    } else {
        RCLCPP_INFO(parent->get_logger(), "Mass config packet received but ID is not valid. " 
            "Not saving parameters to parameter server.");
        valid_id = false;
    }
    if (valid_id) {
        if (packet->set_offset) {
            std::vector<double> offset_vector(packet->offset, packet->offset 
                + sizeof(packet->offset) / sizeof(packet->offset[0]));
            set_param_calib(sensor, "offset", offset_vector);
        }
        if (packet->set_scale) {
            std::vector<double> scale_vector(packet->scale, packet->scale 
                + sizeof(packet->scale) / sizeof(packet->scale[0]));
            set_param_calib(sensor, "scale", scale_vector);
        }
        if (packet->set_channels_status) {
            std::vector<bool> enabled_channels_vector(packet->enabled_channels, packet->enabled_channels 
                + sizeof(packet->enabled_channels) / sizeof(packet->enabled_channels[0]));
            set_param_calib(sensor, "enabled_channels", enabled_channels_vector);

        }
        if (packet->set_alpha)
            set_param_calib(sensor, "alpha", packet->alpha);
    }

    mass_config_response_pub->publish(msg);
}

void BRoCoPublisher::handleServoConfigReqPacket(uint8_t senderID, ServoConfigRequestPacket* packet) {
    if(!(IS_RELIABLE(*packet))) {
		RCLCPP_ERROR(parent->get_logger(), "Unreliable servo config request packet");
		return;
	}
    auto msg = custom_msg::msg::ServoConfigRequestMCU();

    msg.id = packet->id;

    msg.req_min_duty = packet->req_min_duty;
    msg.req_max_duty = packet->req_max_duty;
    msg.req_min_angles = packet->req_min_angles;
    msg.req_max_angles = packet->req_max_angles;

    servo_config_req_pub->publish(msg);
}

void BRoCoPublisher::handleServoConfigPacket(uint8_t senderID, ServoConfigResponsePacket* packet) {
    if(!(IS_RELIABLE(*packet))) {
		RCLCPP_ERROR(parent->get_logger(), "Unreliable servo config response packet");
		return;
	}
    auto msg = custom_msg::msg::ServoConfigResponse();

    msg.id = packet->id;

    msg.set_min_duty = packet->set_min_duty;
    msg.set_max_duty = packet->set_max_duty;
    msg.set_min_angles = packet->set_min_angles;
    msg.set_max_angles = packet->set_max_angles;
    msg.success = packet->success;

    for (uint8_t i = 0; i < 4; ++i) {
        msg.min_duty[i] = scaledUInt16ToFloat(packet->min_duty[i], packet->min_duty_max_val);
        msg.max_duty[i] = scaledUInt16ToFloat(packet->max_duty[i], packet->max_duty_max_val);
        msg.min_angles[i] = scaledUInt16ToFloat(packet->min_angles[i], packet->min_angles_max_val);
        msg.max_angles[i] = scaledUInt16ToFloat(packet->max_angles[i], packet->max_angles_max_val);
    }

    std::string sensor = "servo";

    std::vector<double> min_duty_vector;
    std::vector<double> max_duty_vector;
    std::vector<double> min_angles_vector;
    std::vector<double> max_angles_vector;
    for (int i = 0; i < 4; ++i) {
        min_duty_vector.push_back(static_cast<double>(msg.min_duty[i]));
        max_duty_vector.push_back(static_cast<double>(msg.max_duty[i]));
        min_angles_vector.push_back(static_cast<double>(msg.min_angles[i]));
        max_angles_vector.push_back(static_cast<double>(msg.max_angles[i]));
    }

    if (packet->set_min_duty) 
        set_param_calib(sensor, "min_duty", min_duty_vector);
    
    if (packet->set_max_duty) 
        set_param_calib(sensor, "max_duty", max_duty_vector);
    
    if (packet->set_min_angles) 
        set_param_calib(sensor, "min_angles", min_angles_vector);
    
    if (packet->set_max_angles) 
        set_param_calib(sensor, "max_angles", max_angles_vector);
    

    servo_config_response_pub->publish(msg);
}

uint32_t BRoCoPublisher::get_node_id(std::string node_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
}

void BRoCoPublisher::set_destination_id(std::string node_name) {
    uint16_t id = dynamic_cast<BRoCoManager*>(parent)->get_param<uint32_t>(node_name);
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig(id);
}

void BRoCoPublisher::set_destination_id(uint16_t id) {
    dynamic_cast<CanSocketDriver*>(bus->get_driver())->TxFrameConfig((uint32_t)id);
}

std::string BRoCoPublisher::get_prefix() {
    return dynamic_cast<BRoCoManager*>(parent)->get_prefix();
}

std::string BRoCoPublisher::get_bus() {
    return dynamic_cast<BRoCoManager*>(parent)->get_bus();
}

template <typename T>
T BRoCoPublisher::get_param(const std::string& parameter_name) {
    return dynamic_cast<BRoCoManager*>(parent)->get_param<T>(parameter_name);
}

template <typename T>
void BRoCoPublisher::set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value) {
    dynamic_cast<BRoCoManager*>(parent)->set_param_calib(sensor, parameter_name, value);
}