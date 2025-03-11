/*
 * BRoCoManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef BROCO_MANAGER_H
#define BROCO_MANAGER_H

#include "BRoCoPublisher.h"
#include "BRoCoSubscriber.h"

#include "BRoCo/CanSocketDriver.h"

#include <iostream>
#include <fstream>

class BRoCoManager : public rclcpp::Node {
public:
    BRoCoManager();
    ~BRoCoManager();

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

    // only set in the parameter server
    template <typename T>
    bool set_param_calib(const std::string& sensor, const std::string& parameter_name, const T& value)
    {
        rclcpp::Parameter parameter(sensor + "." + parameter_name, value);
        auto result = this->set_parameter(parameter);
        if (result.successful)
            return true;
        else
            return false;
    }
    std::string get_prefix() const;
    std::string get_bus() const;

private:

    void retryConnection();
    void createPubSub();

    uint32_t retry_count = 0;
    std::string bus_name = "";
    std::string prefix = "";

    CanSocketDriver* driver;
    CANBus* bus;
    BRoCoPublisher* pub;
    BRoCoSubscriber* sub;
    rclcpp::TimerBase::SharedPtr retry_timer;
    
};

#endif /* BROCO_MANAGER_H */