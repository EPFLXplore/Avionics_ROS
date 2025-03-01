/*
 * MuxSubscriber.cpp
 *
 *      Author: Vincent Nguyen
 */
#include "rclcpp/rclcpp.hpp"
#include "MuxSubscriber.h"
#include "MuxManager.h"

template<typename MessageT>
MuxSubscriber<MessageT>::MuxSubscriber(
    rclcpp::Node* parent,
    const std::string& topic_name, uint16_t default_id
) : parent(parent), topic_name(topic_name),
    bus0(dynamic_cast<MuxManager*>(parent)->get_param<std::string>("bus0")),
    bus1(dynamic_cast<MuxManager*>(parent)->get_param<std::string>("bus1")),
    default_id(default_id)
{
    pub0 = parent->create_publisher<MessageT>("/" + bus0 + topic_name, 10);
    pub1 = parent->create_publisher<MessageT>("/" + bus1 + topic_name, 10);
    // Initialize callbacks
    initCallback();
}

template<typename MessageT>
void MuxSubscriber<MessageT>::initCallback() {
    sub = parent->create_subscription<MessageT>(
        topic_name, 10,
        [this](const typename MessageT::SharedPtr msg) {
            callback(msg);
        });
}

template<typename MessageT>
void MuxSubscriber<MessageT>::callback(const typename MessageT::SharedPtr msg) {
    if (msg->destination_id < dynamic_cast<MuxManager*>(parent)->get_max_number_nodes()) {
        bool bus0_state;
        bool bus1_state;
        if (msg->destination_id != 0) {
            bus0_state = dynamic_cast<MuxManager*>(parent)->get_bus0_state()[msg->destination_id];
            bus1_state = dynamic_cast<MuxManager*>(parent)->get_bus1_state()[msg->destination_id];
        } else {
            bus0_state = dynamic_cast<MuxManager*>(parent)->get_bus0_state()[default_id];
            bus1_state = dynamic_cast<MuxManager*>(parent)->get_bus1_state()[default_id];
        }
        switch(selected_bus(bus0_state, bus1_state)) {
            case 0:
                pub0->publish(*msg);
                break;
            case 1:
                pub1->publish(*msg);
                break;
            case NOBUS:
                pub0->publish(*msg); // Still try to publish on bus 0
        }
    } else {
        // Invalid node ID, still attempt publishing to bus 0. Target all nodes.
        RCLCPP_ERROR(parent->get_logger(), "Invalid destination node ID for %s: %d. Publishing to all nodes...", topic_name.c_str(), msg->destination_id);

        msg->destination_id = dynamic_cast<MuxManager*>(parent)->get_param<uint32_t>("GENERAL_NODE_ID");
        pub0->publish(*msg);
    }
}

template<typename MessageT>
uint8_t MuxSubscriber<MessageT>::selected_bus(bool node_state_bus0, bool node_state_bus1) {
    // We choose to prioritize bus0

    // Truth table:
    // node_state_bus0      node_state_bus1     selected_bus
    // 0                    0                   2 (no bus available)
    // 0                    1                   1
    // 1                    0                   0
    // 1                    1                   0
    if (node_state_bus0)
        return 0;
    else if (node_state_bus1)
        return 1;
    else
        return NOBUS;
}

template class MuxSubscriber<custom_msg::msg::LaserRequest>;
template class MuxSubscriber<custom_msg::msg::LedsCommand>;
template class MuxSubscriber<custom_msg::msg::ServoRequest>;
template class MuxSubscriber<custom_msg::msg::SpectroRequest>;

template class MuxSubscriber<custom_msg::msg::MassConfigRequestJetson>;
template class MuxSubscriber<custom_msg::msg::PotConfigRequestJetson>;
template class MuxSubscriber<custom_msg::msg::ServoConfigRequestJetson>;
template class MuxSubscriber<custom_msg::msg::AccelConfigRequestJetson>;
template class MuxSubscriber<custom_msg::msg::GyroConfigRequestJetson>;
template class MuxSubscriber<custom_msg::msg::MagConfigRequestJetson>;

template class MuxSubscriber<custom_msg::msg::MassCalibOffset>;
template class MuxSubscriber<custom_msg::msg::MassCalibScale>;
template class MuxSubscriber<custom_msg::msg::ImuCalib>;