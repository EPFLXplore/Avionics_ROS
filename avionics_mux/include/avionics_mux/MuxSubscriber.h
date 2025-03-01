/*
 * MuxSubscriber.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef MUX_SUBSCRIBER_H
#define MUX_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include <string>

template<typename MessageT>
class MuxSubscriber {
public:
    MuxSubscriber(
        rclcpp::Node* parent,
        const std::string& topic_name,
        uint16_t default_id);

    void callback(const typename MessageT::SharedPtr msg);

private:
    rclcpp::Node* parent;
    std::string topic_name;
    std::string bus0 = "";
    std::string bus1 = "";
    uint16_t default_id = 0;
    typename rclcpp::Publisher<MessageT>::SharedPtr pub0;
    typename rclcpp::Publisher<MessageT>::SharedPtr pub1;
    typename rclcpp::Subscription<MessageT>::SharedPtr sub;

    void initCallback();
    uint8_t selected_bus(bool node_state_bus0, bool node_state_bus1);
};

#endif /* MUX_SUBSCRIBER_H */