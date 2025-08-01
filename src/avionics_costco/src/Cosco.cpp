/**
 * @file Cosco.cpp
 * @author Eliot Abramo
 * @brief Implementation of Serial Protocol AKA Cosco
 * @date 2025-07-03
 * 
 */

#include "Cosco.hpp"

// bound at runtime in CoscoPublisher
rclcpp::Publisher<custom_msg::msg::ServoResponse>::SharedPtr servo_response_pub;
rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_pub;
rclcpp::Publisher<custom_msg::msg::MassPacket>::SharedPtr mass_pub;
rclcpp::Publisher<custom_msg::msg::Heartbeat>::SharedPtr heartbeat_pub;

Cosco::Cosco(const std::string &port, int baud) : serial_(port, baud), proto_(serial_) {
    if(!serial_.ok()) throw std::runtime_error("Serial open failed");
}

void Cosco::sendMassRequestHD(const MassRequestHD* data){
    proto_.send(MassHD_Request_ID,data,sizeof(MassRequestHD)); 
}

void Cosco::sendMassRequestDrill(const MassRequestDrill* data){
    proto_.send(MassDrill_Request_ID, data,sizeof(MassRequestDrill)); 
}

void Cosco::sendDust(const DustData* data){
    proto_.send(DustData_ID,data,sizeof(DustData));
}

void Cosco::sendServo(const ServoRequest* data, uint8_t ID){
    if(ID == 1){
        proto_.send(ServoCam_ID,data,sizeof(ServoRequest));
    }else if(ID == 2){
        proto_.send(ServoDrill_ID,data,sizeof(ServoRequest));
    }
}

void Cosco::readOne() {
    while(true){
        int b = serial_.read(); if(b<0) continue;
        if(!proto_.processByte(uint8_t(b))) continue;
        send_ROS(proto_.frame()); return;
    }
}

void Cosco::mass_packet_handle(MassPacket* data) {
    custom_msg::msg::MassPacket ros_msg;
    ros_msg.id = data->id;
    ros_msg.mass = data->mass;

    if (mass_pub) {
        mass_pub->publish(ros_msg);
    }

    std::cout<< "[MassData] Send mass pub to /EL/mass_packet" << std::endl;
}

void Cosco::dust_handle(DustData* data) {
    custom_msg::msg::DustData ros_msg;
    ros_msg.pm1_0_std = data->pm1_0_std;
    ros_msg.pm2_5_std = data->pm2_5_std;
    ros_msg.pm10_std = data->pm10_std;
    ros_msg.pm1_0_atm = data->pm1_0_atm;
    ros_msg.pm2_5_atm = data->pm2_5_atm;
    ros_msg.pm10_atm = data->pm10_atm;
    ros_msg.num_particles_0_3 = data->num_particles_0_3;
    ros_msg.num_particles_0_5 = data->num_particles_0_5;
    ros_msg.num_particles_1_0 = data->num_particles_1_0;
    ros_msg.num_particles_2_5 = data->num_particles_2_5;
    ros_msg.num_particles_5_0 = data->num_particles_5_0;
    ros_msg.num_particles_10 = data->num_particles_10;

    if (dust_pub) {
        dust_pub->publish(ros_msg);
    }
    std::cout << "[DustData] Published to /EL/dust_sensor" << std::endl;
}

void Cosco::servo_request_handle(ServoRequest* data) {
    // std::cout << "[ServoRequest] id=" << data->id
    //           << " increment=" << static_cast<unsigned>(data->increment)
    //           << " zero_in=" << std::boolalpha << data->zero_in << std::endl;
}

void Cosco::servo_response_handle(ServoResponse* data) {
    custom_msg::msg::ServoResponse ros_msg;
    ros_msg.id = data->id;
    ros_msg.angle = data->angle;
    ros_msg.success = data->success;
    if (servo_response_pub) {
        servo_response_pub->publish(ros_msg);
    }
}

void Cosco::heartbeat_handle(Heartbeat* data) {
    custom_msg::msg::Heartbeat ros_msg;
    ros_msg.dummy = data->dummy;
    if (heartbeat_pub) {
        heartbeat_pub->publish(ros_msg);
    }
}

void Cosco::send_ROS(const typename SerialProtocol<128>::Frame &f){
    switch(f.id){
        case MassDrill_ID:
        case MassHD_ID:{
            MassPacket mass;
            if (as(f.payload.data(), f.length, mass))
                mass_packet_handle(&mass); 
            break;
        }
        case DustData_ID:
        {
            DustData dust; 
            if (as(f.payload.data(), f.length, dust))
                dust_handle(&dust);
            break;
        }
        case ServoCam_Response_ID:
        case ServoDrill_Response_ID:
        {
            ServoResponse servoReq;
            if (as(f.payload.data(), f.length, servoReq))
              servo_response_handle(&servoReq);
            break;
        }
        
        case ServoCam_ID:
        case ServoDrill_ID:
        {
            ServoResponse servoRep; 
            if (as(f.payload.data(), f.length, servoRep))
                servo_response_handle(&servoRep);
            break;
        }

        case Heartbeat_ID:
        {
            Heartbeat heart;
            if (as(f.payload.data(), f.length, heart))
                heartbeat_handle(&heart);
        }
        default:
            break;
    }
}