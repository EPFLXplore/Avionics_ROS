/*
 * MuxManager.h
 *
 *      Author: Vincent Nguyen
 */

#ifndef MUX_MANAGER_H
#define MUX_MANAGER_H

#define NOBUS (2)

#include "custom_msg/msg/four_in_one.hpp"
#include "custom_msg/msg/npk.hpp"
#include "custom_msg/msg/voltage.hpp"
#include "custom_msg/msg/mass_array.hpp"
#include "custom_msg/msg/imu.hpp"
#include "custom_msg/msg/mag.hpp"
#include "custom_msg/msg/angle_array.hpp"
#include "custom_msg/msg/spectro_response.hpp"
#include "custom_msg/msg/laser_response.hpp"
#include "custom_msg/msg/servo_response.hpp"
#include "custom_msg/msg/led_response.hpp"
#include "custom_msg/msg/node_state_array.hpp"

#include "custom_msg/msg/laser_request.hpp"
#include "custom_msg/msg/leds_command.hpp"
#include "custom_msg/msg/servo_request.hpp"
#include "custom_msg/msg/spectro_request.hpp"

#include "custom_msg/msg/mass_config_request_mcu.hpp"
#include "custom_msg/msg/mass_config_response.hpp"

#include "custom_msg/msg/pot_config_request_mcu.hpp"
#include "custom_msg/msg/pot_config_response.hpp"

#include "custom_msg/msg/servo_config_request_mcu.hpp"
#include "custom_msg/msg/servo_config_response.hpp"

#include "custom_msg/msg/accel_config_request_mcu.hpp"
#include "custom_msg/msg/accel_config_response.hpp"

#include "custom_msg/msg/gyro_config_request_mcu.hpp"
#include "custom_msg/msg/gyro_config_response.hpp"

#include "custom_msg/msg/mag_config_request_mcu.hpp"
#include "custom_msg/msg/mag_config_response.hpp"

#include "custom_msg/msg/mass_config_request_jetson.hpp"
#include "custom_msg/msg/pot_config_request_jetson.hpp"
#include "custom_msg/msg/servo_config_request_jetson.hpp"
#include "custom_msg/msg/accel_config_request_jetson.hpp"
#include "custom_msg/msg/gyro_config_request_jetson.hpp"
#include "custom_msg/msg/mag_config_request_jetson.hpp"

#include "custom_msg/msg/mass_calib_offset.hpp"
#include "custom_msg/msg/mass_calib_scale.hpp"
#include "custom_msg/msg/imu_calib.hpp"

#include "MuxPublisher.h"
#include "MuxSubscriber.h"

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

    std::vector<bool> get_bus0_state() const;
    std::vector<bool> get_bus1_state() const;
    uint32_t get_max_number_nodes() const;
    uint16_t get_node_id(std::string node_name);

private:
    rclcpp::Subscription<custom_msg::msg::NodeStateArray>::SharedPtr bus0_node_state_sub;
    rclcpp::Subscription<custom_msg::msg::NodeStateArray>::SharedPtr bus1_node_state_sub;

    void bus0StateCallback(const custom_msg::msg::NodeStateArray::SharedPtr msg);
    void bus1StateCallback(const custom_msg::msg::NodeStateArray::SharedPtr msg);

    std::string bus0 = "";
    std::string bus1 = "";

    std::vector<bool> bus0_state;
    std::vector<bool> bus1_state;

    uint32_t max_number_nodes = 16;

    MuxPublisher<custom_msg::msg::FourInOne>* four_in_one_mux;
    MuxPublisher<custom_msg::msg::NPK>* npk_mux;
    MuxPublisher<custom_msg::msg::Voltage>* voltage_mux;
    MuxPublisher<custom_msg::msg::MassArray>* drill_mass_mux;
    MuxPublisher<custom_msg::msg::MassArray>* container_mass_mux;
    MuxPublisher<custom_msg::msg::Imu>* imu_mux;
    MuxPublisher<custom_msg::msg::Mag>* mag_mux;
    MuxPublisher<custom_msg::msg::AngleArray>* potentiometer_mux;
    MuxPublisher<custom_msg::msg::SpectroResponse>* spectro_response_mux;
    MuxPublisher<custom_msg::msg::LaserResponse>* laser_response_mux;
    MuxPublisher<custom_msg::msg::ServoResponse>* servo_response_mux;
    MuxPublisher<custom_msg::msg::LEDResponse>* led_response_mux;

    MuxPublisher<custom_msg::msg::MassConfigRequestMCU>* mass_config_req_mcu_mux;
    MuxPublisher<custom_msg::msg::MassConfigResponse>* mass_config_response_mux;

    MuxPublisher<custom_msg::msg::PotConfigRequestMCU>* pot_config_req_mcu_mux;
    MuxPublisher<custom_msg::msg::PotConfigResponse>* pot_config_response_mux;

    MuxPublisher<custom_msg::msg::ServoConfigRequestMCU>* servo_config_req_mcu_mux;
    MuxPublisher<custom_msg::msg::ServoConfigResponse>* servo_config_response_mux;

    MuxPublisher<custom_msg::msg::AccelConfigRequestMCU>* accel_config_req_mcu_mux;
    MuxPublisher<custom_msg::msg::AccelConfigResponse>* accel_config_response_mux;

    MuxPublisher<custom_msg::msg::GyroConfigRequestMCU>* gyro_config_req_mcu_mux;
    MuxPublisher<custom_msg::msg::GyroConfigResponse>* gyro_config_response_mux;

    MuxPublisher<custom_msg::msg::MagConfigRequestMCU>* mag_config_req_mcu_mux;
    MuxPublisher<custom_msg::msg::MagConfigResponse>* mag_config_response_mux;

    MuxSubscriber<custom_msg::msg::LaserRequest>* laser_req_mux;
    MuxSubscriber<custom_msg::msg::LedsCommand>* led_req_mux;
    MuxSubscriber<custom_msg::msg::ServoRequest>* servo_req_mux;
    MuxSubscriber<custom_msg::msg::SpectroRequest>* spectro_req_mux;

    MuxSubscriber<custom_msg::msg::MassConfigRequestJetson>* mass_config_req_jetson_mux;
    MuxSubscriber<custom_msg::msg::PotConfigRequestJetson>* pot_config_req_jetson_mux;
    MuxSubscriber<custom_msg::msg::ServoConfigRequestJetson>* servo_config_req_jetson_mux;
    MuxSubscriber<custom_msg::msg::AccelConfigRequestJetson>* accel_config_req_jetson_mux;
    MuxSubscriber<custom_msg::msg::GyroConfigRequestJetson>* gyro_config_req_jetson_mux;
    MuxSubscriber<custom_msg::msg::MagConfigRequestJetson>* mag_config_req_jetson_mux;

    MuxSubscriber<custom_msg::msg::MassCalibOffset>* mass_container_calib_offset_mux;
    MuxSubscriber<custom_msg::msg::MassCalibOffset>* mass_drill_calib_offset_mux;
    MuxSubscriber<custom_msg::msg::MassCalibScale>* mass_container_calib_scale_mux;
    MuxSubscriber<custom_msg::msg::MassCalibScale>* mass_drill_calib_scale_mux;
    MuxSubscriber<custom_msg::msg::ImuCalib>* imu_calib_mux;
};

#endif /* MUX_MANAGER_H */