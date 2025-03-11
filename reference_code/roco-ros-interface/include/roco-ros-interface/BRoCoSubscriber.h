/*
 * BRoCoSubscriber.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef BROCO_SUBSCRIBER_H
#define BROCO_SUBSCRIBER_H

#include <chrono>
#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "BRoCo/CANBus.h"
#include "Protocol/Protocol.h"

#include "custom_msg/msg/spectro_request.hpp"
#include "custom_msg/msg/servo_request.hpp"
#include "custom_msg/msg/laser_request.hpp"
#include "custom_msg/msg/led_request.hpp"
#include "custom_msg/msg/leds_command.hpp"

#include "custom_msg/msg/mass_config_request_jetson.hpp"
#include "custom_msg/msg/pot_config_request_jetson.hpp"
#include "custom_msg/msg/servo_config_request_jetson.hpp"
#include "custom_msg/msg/accel_config_request_jetson.hpp"
#include "custom_msg/msg/gyro_config_request_jetson.hpp"
#include "custom_msg/msg/mag_config_request_jetson.hpp"

#include "custom_msg/msg/mass_calib_offset.hpp"
#include "custom_msg/msg/mass_calib_scale.hpp"
#include "custom_msg/msg/imu_calib.hpp"

#define MAKE_IDENTIFIABLE(PACKET) (PACKET).id = 0x000;

class BRoCoSubscriber {
public:
    BRoCoSubscriber(CANBus* bus, rclcpp::Node* parent);

    void performServoRequest();

private:
    uint32_t get_node_id(std::string node_name);
    void set_destination_id(uint16_t id);
    void set_destination_id(std::string node_name);

    std::string get_prefix();
    std::string get_bus();

    // template <typename T>
    // void set_param_calib(concrc.toString(16)); // Commented cus do not know what concrc is!
    template <typename T>
    T get_param(const std::string &parameter_name);

    rclcpp::Subscription<custom_msg::msg::SpectroRequest>::SharedPtr spectro_req_sub;
    rclcpp::Subscription<custom_msg::msg::ServoRequest>::SharedPtr servo_req_sub;
    rclcpp::Subscription<custom_msg::msg::LaserRequest>::SharedPtr laser_req_sub;
    rclcpp::Subscription<custom_msg::msg::LedsCommand>::SharedPtr led_req_sub;

    rclcpp::Subscription<custom_msg::msg::MassConfigRequestJetson>::SharedPtr mass_config_req_sub;
    rclcpp::Subscription<custom_msg::msg::PotConfigRequestJetson>::SharedPtr pot_config_req_sub;
    rclcpp::Subscription<custom_msg::msg::ServoConfigRequestJetson>::SharedPtr servo_config_req_sub;
    rclcpp::Subscription<custom_msg::msg::AccelConfigRequestJetson>::SharedPtr accel_config_req_sub;
    rclcpp::Subscription<custom_msg::msg::GyroConfigRequestJetson>::SharedPtr gyro_config_req_sub;
    rclcpp::Subscription<custom_msg::msg::MagConfigRequestJetson>::SharedPtr mag_config_req_sub;

    rclcpp::Subscription<custom_msg::msg::MassCalibOffset>::SharedPtr mass_drill_calib_offset_sub;
    rclcpp::Subscription<custom_msg::msg::MassCalibOffset>::SharedPtr mass_container_calib_offset_sub;
    rclcpp::Subscription<custom_msg::msg::MassCalibScale>::SharedPtr mass_drill_calib_scale_sub;
    rclcpp::Subscription<custom_msg::msg::MassCalibScale>::SharedPtr mass_container_calib_scale_sub;
    rclcpp::Subscription<custom_msg::msg::ImuCalib>::SharedPtr imu_calib_sub;

    void spectroReqCallback(const custom_msg::msg::SpectroRequest::SharedPtr msg);
    void servoReqCallback(const custom_msg::msg::ServoRequest::SharedPtr msg);
    void laserReqCallback(const custom_msg::msg::LaserRequest::SharedPtr msg);
    void ledReqCallback(const custom_msg::msg::LedsCommand::SharedPtr msg);
    
    void massConfigReqCallback(const custom_msg::msg::MassConfigRequestJetson::SharedPtr msg);
    void potConfigReqCallback(const custom_msg::msg::PotConfigRequestJetson::SharedPtr msg);
    void servoConfigReqCallback(const custom_msg::msg::ServoConfigRequestJetson::SharedPtr msg);
    void accelConfigReqCallback(const custom_msg::msg::AccelConfigRequestJetson::SharedPtr msg);
    void gyroConfigReqCallback(const custom_msg::msg::GyroConfigRequestJetson::SharedPtr msg);
    void magConfigReqCallback(const custom_msg::msg::MagConfigRequestJetson::SharedPtr msg);

    void massDrillCalibOffsetCallback(const custom_msg::msg::MassCalibOffset::SharedPtr msg);
    void massContainerCalibOffsetCallback(const custom_msg::msg::MassCalibOffset::SharedPtr msg);
    void massDrillCalibScaleCallback(const custom_msg::msg::MassCalibScale::SharedPtr msg);
    void massContainerCalibScaleCallback(const custom_msg::msg::MassCalibScale::SharedPtr msg);
    void imuCalibCallback(const custom_msg::msg::ImuCalib::SharedPtr msg);

    template <typename T>
    void set_param_calib(const std::string &sensor, const std::string &parameter_name, const T &value);

    rclcpp::Clock::SharedPtr clk;
    rclcpp::TimerBase::SharedPtr timer;
    CANBus* bus;
    rclcpp::Node* parent;
};

#endif /* BROCO_SUBSCRIBER_H */