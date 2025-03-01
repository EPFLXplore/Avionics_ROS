/*
 * MuxManager.cpp
 *
 *      Author: Vincent Nguyen
 */

#include "rclcpp/rclcpp.hpp"

#include "MuxManager.h"


MuxManager::MuxManager() : Node("mux_manager") {
    
    this->declare_parameter("bus0", "default_bus0");

    // Node IDs
    this->declare_parameter("JETSON_NODE_ID", 4);
    this->declare_parameter("SC_CONTAINER_NODE_ID", 9);
    this->declare_parameter("SC_DRILL_NODE_ID", 3);
    this->declare_parameter("NAV_NODE_ID", 8);
    this->declare_parameter("HD_NODE_ID", 2);
    this->declare_parameter("GENERAL_NODE_ID", 7);
    this->declare_parameter("MAX_NUMBER_NODES", 1);

    // Topic names =================================
    // Publishers
    this->declare_parameter("FOUR_IN_ONE_TOPIC", "h");
    this->declare_parameter("NPK_TOPIC", "i");
    this->declare_parameter("VOLTAGE_TOPIC", "j");
    this->declare_parameter("DRILL_MASS_TOPIC", "k");
    this->declare_parameter("CONTAINER_MASS_TOPIC", "l");
    this->declare_parameter("IMU_TOPIC", "m");
    this->declare_parameter("MAG_TOPIC", "n");
    this->declare_parameter("POTENTIOMETER_TOPIC", "o");
    this->declare_parameter("SPECTRO_TOPIC", "p");
    this->declare_parameter("LASER_TOPIC", "q");
    this->declare_parameter("SERVO_TOPIC", "r");
    this->declare_parameter("LED_TOPIC", "s");

    this->declare_parameter("MASS_CONFIG_REQ_MCU_TOPIC", "t");
    this->declare_parameter("MASS_CONFIG_TOPIC", "u");
    this->declare_parameter("POT_CONFIG_REQ_MCU_TOPIC", "v");
    this->declare_parameter("POT_CONFIG_TOPIC", "w");
    this->declare_parameter("SERVO_CONFIG_REQ_MCU_TOPIC", "x");
    this->declare_parameter("SERVO_CONFIG_TOPIC", "y");
    this->declare_parameter("ACCEL_CONFIG_REQ_MCU_TOPIC", "z");
    this->declare_parameter("ACCEL_CONFIG_TOPIC", "aa");
    this->declare_parameter("GYRO_CONFIG_REQ_MCU_TOPIC", "bb");
    this->declare_parameter("GYRO_CONFIG_TOPIC", "cc");
    this->declare_parameter("MAG_CONFIG_REQ_MCU_TOPIC", "dd");
    this->declare_parameter("MAG_CONFIG_TOPIC", "ee");

    this->declare_parameter("NODE_STATE_TOPIC", "ff");

    // Subscribers
    this->declare_parameter("SPECTRO_REQ_TOPIC", "gg");
    this->declare_parameter("SERVO_REQ_TOPIC", "hh");
    this->declare_parameter("LASER_REQ_TOPIC", "ii");
    this->declare_parameter("LED_REQ_TOPIC", "jj");

    this->declare_parameter("MASS_CONFIG_REQ_JETSON_TOPIC", "kk");
    this->declare_parameter("POT_CONFIG_REQ_JETSON_TOPIC", "ll");
    this->declare_parameter("SERVO_CONFIG_REQ_JETSON_TOPIC", "mm");
    this->declare_parameter("ACCEL_CONFIG_REQ_JETSON_TOPIC", "nn");
    this->declare_parameter("GYRO_CONFIG_REQ_JETSON_TOPIC", "oo");
    this->declare_parameter("MAG_CONFIG_REQ_JETSON_TOPIC", "pp");

    this->declare_parameter("CONTAINER_MASS_CALIB_OFFSET_TOPIC", "qq");
    this->declare_parameter("DRILL_MASS_CALIB_OFFSET_TOPIC", "rr");
    this->declare_parameter("CONTAINER_MASS_CALIB_SCALE_TOPIC", "ss");
    this->declare_parameter("DRILL_MASS_CALIB_SCALE_TOPIC", "tt");
    this->declare_parameter("IMU_CALIB_TOPIC", "uu");

    if (!this->get_parameter("bus0", bus0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get the 'bus0' parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Selected bus 0: %s", bus0.c_str());

    this->declare_parameter("bus1", "default_bus1");

    if (!this->get_parameter("bus1", bus1)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get the 'bus1' parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Selected bus 1: %s", bus1.c_str());

    bus0 = "/" + bus0;
    bus1 = "/" + bus1;

    max_number_nodes = get_param<uint32_t>("MAX_NUMBER_NODES");

    bus0_state.resize(max_number_nodes, false);
    bus1_state.resize(max_number_nodes, false);

    bus0_node_state_sub = this->create_subscription<custom_msg::msg::NodeStateArray>(
        bus0 + get_param<std::string>("NODE_STATE_TOPIC"), 10, std::bind(&MuxManager::bus0StateCallback, this, std::placeholders::_1));

    bus1_node_state_sub = this->create_subscription<custom_msg::msg::NodeStateArray>(
        bus1 + get_param<std::string>("NODE_STATE_TOPIC"), 10, std::bind(&MuxManager::bus1StateCallback, this, std::placeholders::_1));

    four_in_one_mux = new MuxPublisher<custom_msg::msg::FourInOne>(this, get_param<std::string>("FOUR_IN_ONE_TOPIC"));
    npk_mux = new MuxPublisher<custom_msg::msg::NPK>(this, get_param<std::string>("NPK_TOPIC"));
    voltage_mux = new MuxPublisher<custom_msg::msg::Voltage>(this, get_param<std::string>("VOLTAGE_TOPIC"));
    drill_mass_mux = new MuxPublisher<custom_msg::msg::MassArray>(this, get_param<std::string>("DRILL_MASS_TOPIC"));
    container_mass_mux = new MuxPublisher<custom_msg::msg::MassArray>(this, get_param<std::string>("CONTAINER_MASS_TOPIC"));
    imu_mux = new MuxPublisher<custom_msg::msg::Imu>(this, get_param<std::string>("IMU_TOPIC"));
    mag_mux = new MuxPublisher<custom_msg::msg::Mag>(this, get_param<std::string>("MAG_TOPIC"));
    potentiometer_mux = new MuxPublisher<custom_msg::msg::AngleArray>(this, get_param<std::string>("POTENTIOMETER_TOPIC"));
    spectro_response_mux = new MuxPublisher<custom_msg::msg::SpectroResponse>(this, get_param<std::string>("SPECTRO_TOPIC"));
    laser_response_mux = new MuxPublisher<custom_msg::msg::LaserResponse>(this, get_param<std::string>("LASER_TOPIC"));
    servo_response_mux = new MuxPublisher<custom_msg::msg::ServoResponse>(this, get_param<std::string>("SERVO_TOPIC"));
    led_response_mux = new MuxPublisher<custom_msg::msg::LEDResponse>(this, get_param<std::string>("LED_TOPIC"));

    mass_config_req_mcu_mux = new MuxPublisher<custom_msg::msg::MassConfigRequestMCU>(this, get_param<std::string>("MASS_CONFIG_REQ_MCU_TOPIC"));
    mass_config_response_mux = new MuxPublisher<custom_msg::msg::MassConfigResponse>(this, get_param<std::string>("MASS_CONFIG_TOPIC"));

    pot_config_req_mcu_mux = new MuxPublisher<custom_msg::msg::PotConfigRequestMCU>(this, get_param<std::string>("POT_CONFIG_REQ_MCU_TOPIC"));
    pot_config_response_mux = new MuxPublisher<custom_msg::msg::PotConfigResponse>(this, get_param<std::string>("POT_CONFIG_TOPIC"));

    servo_config_req_mcu_mux = new MuxPublisher<custom_msg::msg::ServoConfigRequestMCU>(this, get_param<std::string>("SERVO_CONFIG_REQ_MCU_TOPIC"));
    servo_config_response_mux = new MuxPublisher<custom_msg::msg::ServoConfigResponse>(this, get_param<std::string>("SERVO_CONFIG_TOPIC"));

    accel_config_req_mcu_mux = new MuxPublisher<custom_msg::msg::AccelConfigRequestMCU>(this, get_param<std::string>("ACCEL_CONFIG_REQ_MCU_TOPIC"));
    accel_config_response_mux = new MuxPublisher<custom_msg::msg::AccelConfigResponse>(this, get_param<std::string>("ACCEL_CONFIG_TOPIC"));

    gyro_config_req_mcu_mux = new MuxPublisher<custom_msg::msg::GyroConfigRequestMCU>(this, get_param<std::string>("GYRO_CONFIG_REQ_MCU_TOPIC"));
    gyro_config_response_mux = new MuxPublisher<custom_msg::msg::GyroConfigResponse>(this, get_param<std::string>("GYRO_CONFIG_TOPIC"));

    mag_config_req_mcu_mux = new MuxPublisher<custom_msg::msg::MagConfigRequestMCU>(this, get_param<std::string>("MAG_CONFIG_REQ_MCU_TOPIC"));
    mag_config_response_mux = new MuxPublisher<custom_msg::msg::MagConfigResponse>(this, get_param<std::string>("MAG_CONFIG_TOPIC"));

    laser_req_mux = new MuxSubscriber<custom_msg::msg::LaserRequest>(this, get_param<std::string>("LASER_REQ_TOPIC"), get_node_id("HD_NODE_ID"));
    led_req_mux = new MuxSubscriber<custom_msg::msg::LedsCommand>(this, get_param<std::string>("LED_REQ_TOPIC"), get_node_id("NAV_NODE_ID"));
    servo_req_mux = new MuxSubscriber<custom_msg::msg::ServoRequest>(this, get_param<std::string>("SERVO_REQ_TOPIC"), get_node_id("HD_NODE_ID"));
    spectro_req_mux = new MuxSubscriber<custom_msg::msg::SpectroRequest>(this, get_param<std::string>("SPECTRO_REQ_TOPIC"), get_node_id("SC_DRILL_NODE_ID"));

    mass_config_req_jetson_mux = new MuxSubscriber<custom_msg::msg::MassConfigRequestJetson>(this, get_param<std::string>("MASS_CONFIG_REQ_JETSON_TOPIC"), get_node_id("GENERAL_NODE_ID"));
    pot_config_req_jetson_mux = new MuxSubscriber<custom_msg::msg::PotConfigRequestJetson>(this, get_param<std::string>("POT_CONFIG_REQ_JETSON_TOPIC"), get_node_id("NAV_NODE_ID"));
    servo_config_req_jetson_mux = new MuxSubscriber<custom_msg::msg::ServoConfigRequestJetson>(this, get_param<std::string>("SERVO_CONFIG_REQ_JETSON_TOPIC"), get_node_id("HD_NODE_ID"));
    accel_config_req_jetson_mux = new MuxSubscriber<custom_msg::msg::AccelConfigRequestJetson>(this, get_param<std::string>("ACCEL_CONFIG_REQ_JETSON_TOPIC"), get_node_id("NAV_NODE_ID"));
    gyro_config_req_jetson_mux = new MuxSubscriber<custom_msg::msg::GyroConfigRequestJetson>(this, get_param<std::string>("GYRO_CONFIG_REQ_JETSON_TOPIC"), get_node_id("NAV_NODE_ID"));
    mag_config_req_jetson_mux = new MuxSubscriber<custom_msg::msg::MagConfigRequestJetson>(this, get_param<std::string>("MAG_CONFIG_REQ_JETSON_TOPIC"), get_node_id("NAV_NODE_ID"));

    mass_container_calib_offset_mux = new MuxSubscriber<custom_msg::msg::MassCalibOffset>(this, get_param<std::string>("CONTAINER_MASS_CALIB_OFFSET_TOPIC"), get_node_id("SC_CONTAINER_NODE_ID"));
    mass_drill_calib_offset_mux = new MuxSubscriber<custom_msg::msg::MassCalibOffset>(this, get_param<std::string>("DRILL_MASS_CALIB_OFFSET_TOPIC"), get_node_id("SC_DRILL_NODE_ID"));
    mass_container_calib_scale_mux = new MuxSubscriber<custom_msg::msg::MassCalibScale>(this, get_param<std::string>("CONTAINER_MASS_CALIB_SCALE_TOPIC"), get_node_id("SC_CONTAINER_NODE_ID"));
    mass_drill_calib_scale_mux = new MuxSubscriber<custom_msg::msg::MassCalibScale>(this, get_param<std::string>("DRILL_MASS_CALIB_SCALE_TOPIC"), get_node_id("SC_DRILL_NODE_ID"));
    imu_calib_mux = new MuxSubscriber<custom_msg::msg::ImuCalib>(this, get_param<std::string>("IMU_CALIB_TOPIC"), get_node_id("NAV_NODE_ID"));
}

MuxManager::~MuxManager() {
    RCLCPP_INFO(this->get_logger(), "Deleting Mux Manager");
    delete this->four_in_one_mux;
    delete this->npk_mux;
    delete this->voltage_mux;
    delete this->drill_mass_mux;
    delete this->container_mass_mux;
    delete this->imu_mux;
    delete this->mag_mux;
    delete this->potentiometer_mux;
    delete this->spectro_response_mux;
    delete this->laser_response_mux;
    delete this->servo_response_mux;
    delete this->led_response_mux;

    delete this->mass_config_req_mcu_mux;
    delete this->mass_config_response_mux;

    delete this->pot_config_req_mcu_mux;
    delete this->pot_config_response_mux;

    delete this->servo_config_req_mcu_mux;
    delete this->servo_config_response_mux;

    delete this->accel_config_req_mcu_mux;
    delete this->accel_config_response_mux;

    delete this->gyro_config_req_mcu_mux;
    delete this->gyro_config_response_mux;

    delete this->mag_config_req_mcu_mux;
    delete this->mag_config_response_mux;

    delete this->laser_req_mux;
    delete this->led_req_mux;
    delete this->servo_req_mux;
    delete this->spectro_req_mux;

    delete this->mass_config_req_jetson_mux;
    delete this->pot_config_req_jetson_mux;
    delete this->servo_config_req_jetson_mux;
    delete this->accel_config_req_jetson_mux;
    delete this->gyro_config_req_jetson_mux;
    delete this->mag_config_req_jetson_mux;

    delete this->mass_container_calib_offset_mux;
    delete this->mass_drill_calib_offset_mux;
    delete this->mass_container_calib_scale_mux;
    delete this->mass_drill_calib_scale_mux;
    delete this->imu_calib_mux;
    
}

void MuxManager::bus0StateCallback(const custom_msg::msg::NodeStateArray::SharedPtr msg) {
    for (size_t i = 0; i < msg->node_state.size(); ++i)
    {
        bus0_state[i] = msg->node_state[i];
    }
}

void MuxManager::bus1StateCallback(const custom_msg::msg::NodeStateArray::SharedPtr msg) {
    for (size_t i = 0; i < msg->node_state.size(); ++i)
    {
        bus1_state[i] = msg->node_state[i];
    }
}

std::vector<bool> MuxManager::get_bus0_state() const {
    return bus0_state;
}

std::vector<bool> MuxManager::get_bus1_state() const {
    return bus1_state;
}

uint32_t MuxManager::get_max_number_nodes() const {
    return max_number_nodes;
}

uint16_t MuxManager::get_node_id(std::string node_name) {
    return get_param<uint32_t>(node_name);
}