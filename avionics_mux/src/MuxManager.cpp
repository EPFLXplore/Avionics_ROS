/*
 * MuxManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "MuxManager.h"


MuxManager::MuxManager() : Node("mux_manager") {
    
    this->declare_parameter("bus0", "default_bus0");

    // Node IDs
    this->declare_parameter("JETSON_NODE_ID", 4);
    this->declare_parameter("SC_CONTAINER_NODE_ID", 9);
    this->declare_parameter("SC_DRILL_NODE_ID", 3);
    this->declare_parameter("NAV_NODE_ID", 8);
    this->declare_parameter("HD_NODE_ID", 2);
    this->declare_parameter("GENERAL_NODE_ID", 7);
    this->declare_parameter("MAX_NUMBER_NODES", 1);

    // Topic names =================================
    // Publishers
    this->declare_parameter("LED_TOPIC", "s");

    // Subscribers
    this->declare_parameter("LED_REQ_TOPIC", "jj");

    if (!this->get_parameter("bus0", bus0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get the 'bus0' parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Selected bus 0: %s", bus0.c_str());

    this->declare_parameter("bus1", "default_bus1");

    if (!this->get_parameter("bus1", bus1)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get the 'bus1' parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Selected bus 1: %s", bus1.c_str());

    bus0 = "/" + bus0;
    bus1 = "/" + bus1;

    max_number_nodes = get_param<uint32_t>("MAX_NUMBER_NODES");

    bus0_state.resize(max_number_nodes, false);
    bus1_state.resize(max_number_nodes, false);

    bus0_node_state_sub = this->create_subscription<custom_msg::msg::NodeStateArray>(
        bus0 + get_param<std::string>("NODE_STATE_TOPIC"), 10, std::bind(&MuxManager::bus0StateCallback, this, std::placeholders::_1));

    bus1_node_state_sub = this->create_subscription<custom_msg::msg::NodeStateArray>(
        bus1 + get_param<std::string>("NODE_STATE_TOPIC"), 10, std::bind(&MuxManager::bus1StateCallback, this, std::placeholders::_1));

    led_response_mux = new MuxPublisher<custom_msg::msg::LEDResponse>(this, get_param<std::string>("LED_TOPIC"));

    led_req_mux = new MuxSubscriber<custom_msg::msg::LedsCommand>(this, get_param<std::string>("LED_REQ_TOPIC"), get_node_id("NAV_NODE_ID"));
}

MuxManager::~MuxManager() {
    RCLCPP_INFO(this->get_logger(), "Deleting Mux Manager");

    delete this->led_response_mux;

    delete this->led_req_mux;
    
}

void MuxManager::bus0StateCallback(const custom_msg::msg::NodeStateArray::SharedPtr msg) {
    for (size_t i = 0; i < msg->node_state.size(); ++i)
    {
        bus0_state[i] = msg->node_state[i];
    }
}

void MuxManager::bus1StateCallback(const custom_msg::msg::NodeStateArray::SharedPtr msg) {
    for (size_t i = 0; i < msg->node_state.size(); ++i)
    {
        bus1_state[i] = msg->node_state[i];
    }
}

std::vector<bool> MuxManager::get_bus0_state() const {
    return bus0_state;
}

std::vector<bool> MuxManager::get_bus1_state() const {
    return bus1_state;
}

uint32_t MuxManager::get_max_number_nodes() const {
    return max_number_nodes;
}

uint16_t MuxManager::get_node_id(std::string node_name) {
    return get_param<uint32_t>(node_name);
}