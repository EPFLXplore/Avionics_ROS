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
rclcpp::Publisher<custom_msg::msg::MassArray>::SharedPtr mass_pub;
rclcpp::Publisher<custom_msg::msg::FourInOne>::SharedPtr fourinone_pub;
rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_pub;

// most recent config data 
MassConfigRequestPacket latest_mass_config_request;
MassConfigResponsePacket latest_mass_config_response;

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

bool try_read_packet(int fd) {
    uint8_t packet_id;
    ssize_t read_bytes = read(fd, &packet_id, 1);
    if (read_bytes != 1)
        return false;

    auto it = handlers.find(packet_id);
    if (it == handlers.end()) {
        std::cerr << "[HOST] Unknown packet ID: 0x" << std::hex << (int)packet_id << " — skipping\n" << std::dec;
        return false;
    }

    size_t size = it->second.first;
    callback_t& cb = it->second.second;
    std::vector<uint8_t> buffer(size);

    size_t total_read = 0;
    while (total_read < size) {
        ssize_t chunk = read(fd, buffer.data() + total_read, size - total_read);
        if (chunk <= 0) {
            usleep(1000);
            continue;
        }
        total_read += chunk;
    }

    cb(buffer.data());
    return true;
}

/*
    pre-packet callbacks
*/
void mass_config_request_callback(const void* ptr) {
    const MassConfigRequestPacket* data = reinterpret_cast<const MassConfigRequestPacket*>(ptr);
    latest_mass_config_request = *data;

    std::cout << "[MassConfigRequest] id=" << data->id
              << " req_config=" << std::boolalpha << data->req_config << "\n";
}

void mass_config_response_callback(const void* ptr) {
    const MassConfigResponsePacket* data = reinterpret_cast<const MassConfigResponsePacket*>(ptr);
    latest_mass_config_response = *data;

    std::cout << "[MassConfigResponse] id=" << data->id
              << " offset=" << data->offset
              << " scale=" << data->scale
              << " offset_set=" << std::boolalpha << data->offset_set
              << " scale_set=" << data->scale_set << "\n";
}


void mass_array_cb(const void* ptr) {
    const MassArray* data = reinterpret_cast<const MassArray*>(ptr);
    custom_msg::msg::MassArray ros_msg;

    ros_msg.id = data->id;
    ros_msg.mass = data->mass;

    if (mass_pub) {
        mass_pub->publish(ros_msg);
    }
}

void fourinone_cb(const void* ptr) {
    auto ros_msg = custom_msg::msg::FourInOne();
    const auto* data = reinterpret_cast<const FourInOne*>(ptr);

    ros_msg.id = data->id;
    ros_msg.temperature = data->temperature;
    ros_msg.moisture = data->moisture;
    ros_msg.conductivity = data->conductivity;
    ros_msg.ph = data->ph;
    fourinone_pub->publish(ros_msg);
}

void dust_cb(const void* ptr) {
    // const DustData* data = reinterpret_cast<const DustData*>(ptr);

    // std::cout << "[DustData] pm1_0_std=" << data->pm1_0_std
    //           << " pm2_5_std=" << data->pm2_5_std
    //           << " pm10__std=" << data->pm10_std
    //           << " pm1_0_atm=" << data->pm1_0_atm
    //           << " pm2_5_atm=" << data->pm2_5_atm
    //           << " pm10__atm=" << data->pm10_atm
    //           << " num_particles_0_3=" << data->num_particles_0_3
    //           << " num_particles_0_5=" << data->num_particles_0_5
    //           << " num_particles_1_0=" << data->num_particles_1_0
    //           << " num_particles_2_5=" << data->num_particles_2_5
    //           << " num_particles_5_0=" << data->num_particles_5_0
    //           << " num_particles_10_=" << data->num_particles_10
    //           << std::endl;

    auto ros_msg = custom_msg::msg::DustData();
    const auto* data = reinterpret_cast<const DustData*>(ptr);

    memcpy(&ros_msg.pm1_0_std, data, sizeof(DustData));
    dust_pub->publish(ros_msg);
}

void servo_request_cb(const void* ptr) {
    const ServoRequest* data = reinterpret_cast<const ServoRequest*>(ptr);
    std::cout << "[ServoRequest] id=" << data->id
              << " increment=" << static_cast<unsigned>(data->increment)
              << " zero_in=" << std::boolalpha << data->zero_in << std::endl;
}

void servo_response_cb(const void* ptr) {
    const ServoResponse* data = reinterpret_cast<const ServoResponse*>(ptr);
    std::cout << "[ServoResponse] id=" << data->id
              << " angle=" << static_cast<int32_t>(data->angle)
              << " success=" << std::boolalpha << data->success << std::endl;
}

// registration + processing
void register_cosco_callbacks() {
    register_packet(MassData_ID, sizeof(MassArray), mass_array_cb);
    register_packet(FourInOne_ID, sizeof(FourInOne), fourinone_cb);
    register_packet(DustData_ID, sizeof(DustData), dust_cb);
    register_packet(ServoConfigRequest_ID, sizeof(ServoRequest), servo_request_cb);
    register_packet(ServoConfigResponse_ID, sizeof(ServoResponse), servo_response_cb);
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
