/*
 * MuxManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef MUX_MANAGER_H
#define MUX_MANAGER_H

#define NOBUS (2)


#include "custom_msg/msg/led_response.hpp"

#include "MuxPublisher.h"
#include "MuxSubscriber.h"

#include "BRoCo/CanSocketDriver.h"
#include "BRoCo/CANBus.h"

class MuxManager : public rclcpp::Node {
public:
    MuxManager();
    ~MuxManager();

    template <typename T>
    T get_param(const std::string& parameter_name) {
        T value;
        if (this->get_parameter(parameter_name, value)) {
            return value;
        } else {
            RCLCPP_WARN(this->get_logger(), "Parameter [%s] not found, using default value.", parameter_name.c_str());
            return T();
        }
    }

private:
    void createPubSub(int bus_id);
    void verifyConnection(int bus_id);

    CanSocketDriver* driver[2];
    CANBus* bus[2];
    std::string bus_name[2] = {'can0', 'can1'};

    uint32_t retry_count[2] = {0};
    rclcpp::TimerBase::SharedPtr retry_timer[2];

    BRoCoPublisher* pub;
    BRoCoSubscriber* sub;

    MuxPublisher<custom_msg::msg::LEDResponse>* led_response_mux;
};

#endif /* MUX_MANAGER_H */