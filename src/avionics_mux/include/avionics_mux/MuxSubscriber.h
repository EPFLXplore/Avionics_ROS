/*
 * MuxSubscriber.h
 *
 *      Author: Matas Jones
 */

#ifndef MUX_SUBSCRIBER_H
#define MUX_SUBSCRIBER_H

#include <string>
#include "rclcpp/rclcpp.hpp"

#include "BRoCo/CANBus.h"
#include "Protocol/Protocol.h"

#include "custom_msg/msg/mass_array.hpp"
// #include "custom_msg/msg/mass_calib_offset.hpp"

class MuxSubscriber {
public:
    MuxSubscriber(CANBus* bus, rclcpp::Node* parent);

private:

    rclcpp::Node* parent;
    CANBus* bus;
    
    // void MuxSubscriber::massDrillCalibOffsetCallback(const custom_msg::msg::MassCalibOffset::SharedPtr msg);

    // // Declare a pointer which will be used in the cpp file to create a sub instance
    // rclcpp::Subscription<custom_msg::msg::MassCalibOffset>::SharedPtr mass_config_req_sub;


};

#endif /* MUX_SUBSCRIBER_H */