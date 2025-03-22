// /*
//  * MuxSubscriber.cpp
//  *
//  *      Author: Matas Jones, Eliot Abramo
//  */

//  #include <chrono>
// #include <memory>
// #include <functional>
// #include "rclcpp/rclcpp.hpp"

// #include "MuxSubscriber.h"
// #include "MuxManager.h"

// #include "BRoCo/CanSocketDriver.h"

// using namespace std::chrono_literals;
// using std::placeholders::_1;

// // Create subscribers based on the pointers in h file and attach callbacks
// MuxSubscriber::MuxSubscriber(CANBus* bus, rclcpp::Node* parent) : bus(bus),
// parent(parent) {

//     RCLCPP_INFO(parent->get_logger(), "Creating subscribers");

//     // // This block creates a subscriber on a specified topic and binds it
//     to a callback
//     // this->mass_config_req_sub =
//     parent->create_subscription<custom_msg::msg::<MassArray>
//     //     (get_prefix() + get_param<std::string>("MASS_REQ_ARRAY"), 10,
//     std::bind(&MuxSubscriber::massDrillCalibOffsetCallback, this, _1));

//     // RCLCPP_INFO(parent->get_logger(), "Subscribers created");
// }

// // void MuxSubscriber::ledReqCallback(const
// custom_msg::msg::LedsCommand::SharedPtr msg) {
// //     uint16_t id = 0;
// //     if (msg->destination_id != 0)
// //         id = msg->destination_id;
// //     else
// //         id = get_node_id("NAV_NODE_ID");
// //     RCLCPP_INFO(parent->get_logger(), "Sending LED request to node ID
// %s...", std::to_string(id).c_str());
// //     static LEDPacket packet;
// //     packet.state = msg->state;
// //     MAKE_IDENTIFIABLE(packet);
// //     MAKE_RELIABLE(packet);
// //     set_destination_id(id);
// //     bus->send(&packet);
// // }

// // void MuxSubscriber::massDrillCalibOffsetCallback(const
// custom_msg::msg::MassCalibOffset::SharedPtr msg) {
// //     uint16_t id = 0;
// //     if (msg->destination_id != 0)
// //         id = msg->destination_id;
// //     else
// //         id = get_node_id("SC_DRILL_NODE_ID");

// //     RCLCPP_INFO(parent->get_logger(), "Sending offset calibration request
// to node ID %s...", std::to_string(id).c_str());
// //     static MassCalibPacket packet;
// //     packet.calib_offset = true;
// //     packet.calib_scale = false;
// //     packet.channel = msg->channel;
// //     packet.expected_weight = 1;

// //     MAKE_IDENTIFIABLE(packet);
// //     MAKE_RELIABLE(packet);
// //     set_destination_id(id);
// //     bus->send(&packet);
// // }
