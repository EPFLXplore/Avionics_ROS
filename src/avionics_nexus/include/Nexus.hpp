/**
 * @file Nexus.hpp
 * @author Eliot Abramo
 * @brief 
 * @date 2025-07-03
 */

#ifndef NEXUS_HPP
#define NEXUS_HPP

#include <vector>
#include "packet_definition.hpp"
#include "packet_id.hpp"
#include "SerialProtocol.hpp"
#include "SerialDriver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <custom_msg/msg/heartbeat.hpp>
#include <custom_msg/msg/dust_data.hpp>
#include <custom_msg/msg/mass_packet.hpp>

extern rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_pub;
extern rclcpp::Publisher<custom_msg::msg::MassPacket>::SharedPtr mass_pub;
extern rclcpp::Publisher<custom_msg::msg::Heartbeat>::SharedPtr heartbeat_pub;

class Nexus {
public:
    Nexus(const std::string &port, int baud=115200);
    ~Nexus(){};

    // Send to ESP32
    void sendMassRequestHD(const MassRequestHD* data);
    void sendMassRequestDrill(const MassRequestDrill* data);
    void sendDust(const DustData* data);
    void sendServo(const ServoRequest* data, uint8_t ID);

    void readOne();

    template<typename T>
    bool as(const uint8_t* pl, uint16_t len, T& out){
        if (len != sizeof(T)) return false;
        std::memcpy(&out, pl, len);
        return true;
    }

private:
    PosixSerial serial_;
    SerialProtocol<128> proto_;

    // Handle ROS
    void mass_packet_handle(MassPacket* mp);
    void dust_handle(DustData* d);
    void heartbeat_handle(Heartbeat* data);    
    void send_ROS(const typename SerialProtocol<128>::Frame &f);
};

#endif /* NEXUS_HPP */
