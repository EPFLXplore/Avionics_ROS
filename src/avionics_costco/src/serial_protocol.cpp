/**
 * @file serial_protocol.cpp
 * @author Ilyas Asmouki
*/

#include "serial_protocol.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <unordered_map>

// bound at runtime in CoscoPublisher
rclcpp::Publisher<custom_msg::msg::ServoResponse>::SharedPtr servo_response_pub;
rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_pub;
rclcpp::Publisher<custom_msg::msg::MassPacket>::SharedPtr mass_pub;
rclcpp::Publisher<custom_msg::msg::FourInOne>::SharedPtr fourinone_pub;

// internal packet registry
std::unordered_map<uint8_t, std::pair<size_t, callback_t>> handlers;

/*
    the function below applies the raw serial settings.
    please do not touch unless something goes wrong/you know what you're doing.
*/
void setup_serial(int fd) {
    termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    options.c_lflag &= ~(ICANON | ECHO | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
}

/*
    pre-packet callbacks
*/

void mass_packet_cb(const void* ptr) {
    const MassPacket* data = reinterpret_cast<const MassPacket*>(ptr);
    custom_msg::msg::MassPacket ros_msg;

    ros_msg.id = data->id;
    ros_msg.mass = data->mass;

    if (mass_pub) {
        mass_pub->publish(ros_msg);
    }

    std::cout<< "[MassData] Send mass pub to /EL/mass_packet" << std::endl;
}

void fourinone_cb(const void* ptr) {
    auto ros_msg = custom_msg::msg::FourInOne();
    const auto* data = reinterpret_cast<const FourInOne*>(ptr);

    ros_msg.id = data->id;
    ros_msg.temperature = data->temperature;
    ros_msg.humidity = data->humidity;
    ros_msg.conductivity = data->conductivity;
    ros_msg.ph = data->ph;
    fourinone_pub->publish(ros_msg);
}

void dust_cb(const void* ptr) {
    const DustData* data = reinterpret_cast<const DustData*>(ptr);

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

void servo_request_cb(const void* ptr) {
    const ServoRequest* data = reinterpret_cast<const ServoRequest*>(ptr);
    std::cout << "[ServoRequest] id=" << data->id
              << " increment=" << static_cast<unsigned>(data->increment)
              << " zero_in=" << std::boolalpha << data->zero_in << std::endl;
}

void servo_response_cb(const void* ptr) {
    const ServoResponse* data = reinterpret_cast<const ServoResponse*>(ptr);
    // std::cout << "[ServoResponse] id=" << data->id
    //           << " angle=" << static_cast<int32_t>(data->angle)
    //           << " success=" << std::boolalpha << data->success << std::endl;
    custom_msg::msg::ServoResponse ros_msg;
    ros_msg.id = data->id;
    ros_msg.angle = data->angle;
    ros_msg.success = data->success;
    if (servo_response_pub) {
        servo_response_pub->publish(ros_msg);
    }
    std::cout << "[ServoResponse] Published to /EL/servo_response" << std::endl;
}

// registration + processing
void register_cosco_callbacks() {
    register_packet(MassDrill_ID, sizeof(MassPacket), mass_packet_cb);
    register_packet(MassHD_ID, sizeof(MassPacket), mass_packet_cb);
    register_packet(FourInOne_ID, sizeof(FourInOne), fourinone_cb);
    register_packet(DustData_ID, sizeof(DustData), dust_cb);
    register_packet(ServoCam_ID, sizeof(ServoRequest), servo_request_cb);
    register_packet(ServoDrill_ID, sizeof(ServoRequest), servo_request_cb);
    register_packet(ServoCam_ID, sizeof(ServoResponse), servo_response_cb);
    register_packet(ServoDrill_ID, sizeof(ServoResponse), servo_response_cb);
}

void process_packets(int fd) {
    uint8_t packet_id;
    if (read(fd, &packet_id, 1) == 1) {
        auto it = handlers.find(packet_id);
        if (it != handlers.end()) {
            size_t size = it->second.first;
            callback_t& cb = it->second.second;
            std::vector<uint8_t> buffer(size);
            if (read_fully(fd, buffer.data(), size)) {
                cb(buffer.data());
            }
        } else {
            std::cerr << "[HOST] Unknown packet ID: 0x" << std::hex << (int)packet_id << " — skipping\n" << std::dec;
        }
    }
}
