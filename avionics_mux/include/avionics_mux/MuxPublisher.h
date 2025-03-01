/*
 * MuxPublisher.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef MUX_PUBLISHER_H
#define MUX_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include <string>

template<typename MessageT>
class MuxPublisher {
public:
    MuxPublisher(
        rclcpp::Node* parent,
        const std::string& topic_name);

    void callback0(const typename MessageT::SharedPtr msg);
    void callback1(const typename MessageT::SharedPtr msg);

private:
    rclcpp::Node* parent;
    std::string topic_name;
    std::string bus0 = "";
    std::string bus1 = "";
    typename rclcpp::Publisher<MessageT>::SharedPtr pub;
    typename rclcpp::Subscription<MessageT>::SharedPtr sub0;
    typename rclcpp::Subscription<MessageT>::SharedPtr sub1;

    void initCallbacks();
    uint8_t selected_bus(bool node_state_bus0, bool node_state_bus1);
};

#endif /* MUX_PUBLISHER_H */