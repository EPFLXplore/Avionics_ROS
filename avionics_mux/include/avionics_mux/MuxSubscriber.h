/*
 * MuxSubscriber.h
 *
 *      Author: Matas Jones
 */

#ifndef MUX_SUBSCRIBER_H
#define MUX_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "BRoCo/CANBus.h"

class MuxSubscriber {
public:
    MuxSubscriber(
        rclcpp::Node* parent,
        const std::string& topic_name,
        );

private:

    rclcpp::Node* parent;
    CANBus* bus;

    // Declare a pointer which will be used in the cpp file to create a sub instance
    rclcpp::Subscription<custom_msg::msg::LedsCommand>::SharedPtr led_req_sub;


};

#endif /* MUX_SUBSCRIBER_H */