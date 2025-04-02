#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <unistd.h>
#include "packet_definition.hpp"
#include "packet_id.hpp"

#include <rclcpp/rclcpp.hpp>
#include <custom_msg/msg/mass_array.hpp>
#include <custom_msg/msg/four_in_one.hpp>
#include <custom_msg/msg/dust_data.hpp>

extern rclcpp::Publisher<custom_msg::msg::MassArray>::SharedPtr mass_pub;
extern rclcpp::Publisher<custom_msg::msg::FourInOne>::SharedPtr fourinone_pub;
extern rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_pub;


void setup_serial(int fd);

using callback_t = std::function<void(const void*)>;
extern std::unordered_map<uint8_t, std::pair<size_t, callback_t>> handlers;

inline void register_packet(uint8_t id, size_t size, callback_t cb) {
    handlers[id] = std::make_pair(size, cb);
}
inline void register_packet_handler(uint8_t id, size_t size, callback_t cb);

inline bool read_fully(int fd, void* buffer, size_t size) {
    uint8_t* ptr = static_cast<uint8_t*>(buffer);
    size_t total_read = 0;
    while (total_read < size) {
        ssize_t bytes = read(fd, ptr + total_read, size - total_read);
        if (bytes > 0) {
            total_read += bytes;
        } else {
            usleep(1000);  // sleep 1ms before trying again
        }
    }
    return true;
}
bool try_read_packet(int fd);

template<typename T>
bool send_packet(int fd, uint8_t id, const T& packet) {
    uint8_t buffer[sizeof(T) + 1];
    buffer[0] = id;
    std::memcpy(buffer + 1, &packet, sizeof(T));
    return write(fd, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

void register_cosco_callbacks();
void process_packets(int fd);

#endif // SERIAL_PROTOCOL_HPP
