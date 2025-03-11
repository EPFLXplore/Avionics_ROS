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
    void initCallbacks();
};

#endif /* MUX_PUBLISHER_H */