#include "serial_protocol.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <unordered_map>

MassConfigRequestPacket latest_mass_config_request;
MassConfigResponsePacket latest_mass_config_response;

static std::unordered_map<uint8_t, std::pair<size_t, callback_t>> handlers;

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

void register_packet_handler(uint8_t id, size_t size, callback_t cb) {
    handlers[id] = { size, cb };
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
