/*
 * MuxManager.h
 *
 *      Author: Matas Jones
 */

#ifndef MUX_MANAGER_H
#define MUX_MANAGER_H

// #define NOBUS (2)

#include "MuxPublisher.h"
#include "MuxSubscriber.h"

#include "BRoCo/CanSocketDriver.h"
#include "BRoCo/CANBus.h"

class MuxManager : public rclcpp::Node {
public:
    MuxManager();
    ~MuxManager();

    // allows function to return any type 'T', so like int, str, etc.
    template <typename T>
    T get_param(const std::string& parameter_name) {
        T value;
        //try and obtain parameter_name value, returns true if found and stores value in value.
        //function comes from rclcpp::node inheritance.
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
    const char* bus_name[2] = {"can0", "can1"};

    uint32_t retry_count[2] = {0, 0};
    rclcpp::TimerBase::SharedPtr retry_timer[2];

    MuxPublisher* pub;
    MuxSubscriber* sub;

};

#endif /* MUX_MANAGER_H */