// /*
//  * MuxPublisher.cpp
//  *
//  *      Author: Vincent Nguyen
//  */
// #include "rclcpp/rclcpp.hpp"
// #include "MuxPublisher.h"
// #include "MuxManager.h"

// template<typename MessageT>
// MuxPublisher<MessageT>::MuxPublisher(
//     rclcpp::Node* parent,
//     const std::string& topic_name
// ) : parent(parent), topic_name(topic_name),
//     bus0(dynamic_cast<MuxManager*>(parent)->get_param<std::string>("bus0")),
//     bus1(dynamic_cast<MuxManager*>(parent)->get_param<std::string>("bus1"))
// {
//     // Create a publisher based on the provided publish topic name
//     pub = parent->create_publisher<MessageT>(topic_name, 10);

//     // Initialize callbacks
//     initCallbacks();
// }

// template<typename MessageT>
// void MuxPublisher<MessageT>::initCallbacks() {
//     sub0 = parent->create_subscription<MessageT>(
//         "/" + bus0 + topic_name, 10,
//         [this](const typename MessageT::SharedPtr msg) {
//             callback0(msg);
//         });

//     sub1 = parent->create_subscription<MessageT>(
//         "/" + bus1 + topic_name, 10,
//         [this](const typename MessageT::SharedPtr msg) {
//             callback1(msg);
//         });
// }

// template<typename MessageT>
// void MuxPublisher<MessageT>::callback0(const typename MessageT::SharedPtr msg) {
//     if (msg->id < dynamic_cast<MuxManager*>(parent)->get_max_number_nodes()) {
//         bool bus0_state = dynamic_cast<MuxManager*>(parent)->get_bus0_state()[msg->id];
//         bool bus1_state = dynamic_cast<MuxManager*>(parent)->get_bus1_state()[msg->id];
//         // Normally NOBUS should never happen as we cannot receive data if there is no bus connected
//         if (selected_bus(bus0_state, bus1_state) == 0 or selected_bus(bus0_state, bus1_state) == NOBUS)
//             pub->publish(*msg);
//     } else {
//         RCLCPP_ERROR(parent->get_logger(), ("Invalid node ID for " + bus0 + topic_name + ": " + std::to_string(msg->id) + ". Not publishing...").c_str()); //!!!!!
//          pub->publish(*msg);
//     }
// }

// template<typename MessageT>
// void MuxPublisher<MessageT>::callback1(const typename MessageT::SharedPtr msg) {
//     if (msg->id < dynamic_cast<MuxManager*>(parent)->get_max_number_nodes()) {
//         bool bus0_state = dynamic_cast<MuxManager*>(parent)->get_bus0_state()[msg->id];
//         bool bus1_state = dynamic_cast<MuxManager*>(parent)->get_bus1_state()[msg->id];
//         // Normally NOBUS should never happen as we cannot receive data if there is no bus connected
//         if (selected_bus(bus0_state, bus1_state) == 1 or selected_bus(bus0_state, bus1_state) == NOBUS)
//             pub->publish(*msg);
//     } else {
//         RCLCPP_ERROR(parent->get_logger(), ("Invalid node ID for " + bus1 + topic_name + ": " + std::to_string(msg->id) + ". Not publishing...").c_str());
//         pub->publish(*msg);
//     }
// }

// template<typename MessageT>
// uint8_t MuxPublisher<MessageT>::selected_bus(bool node_state_bus0, bool node_state_bus1) {
//     // We choose to prioritize bus0

//     // Truth table:
//     // node_state_bus0      node_state_bus1     selected_bus
//     // 0                    0                   2 (no bus available)
//     // 0                    1                   1
//     // 1                    0                   0
//     // 1                    1                   0
//     if (node_state_bus0)
//         return 0;
//     else if (node_state_bus1)
//         return 1;
//     else
//         return NOBUS;
// }

// template class MuxPublisher<custom_msg::msg::LEDResponse>;