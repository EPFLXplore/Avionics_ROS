/**
 * @file serial_protocol.hpp
 * @author Ilyas Asmouki
*/

#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <custom_msg/msg/servo_response.hpp>
#include <custom_msg/msg/dust_data.hpp>
#include <custom_msg/msg/mass_packet.hpp>
#include <custom_msg/msg/four_in_one.hpp>

#include "packet_definition.hpp"
#include "packet_id.hpp"

// global publishers used in the callback handlers
extern rclcpp::Publisher<custom_msg::msg::ServoResponse>::SharedPtr servo_response_pub;
extern rclcpp::Publisher<custom_msg::msg::DustData>::SharedPtr dust_pub;
extern rclcpp::Publisher<custom_msg::msg::MassPacket>::SharedPtr mass_pub;
extern rclcpp::Publisher<custom_msg::msg::FourInOne>::SharedPtr fourinone_pub;

// setup serial port with non-blocking, raw config at 115200 baud and no flow control
void setup_serial(int fd);

// map packet ID -> (size, callback)
using callback_t = std::function<void(const void*)>;
extern std::unordered_map<uint8_t, std::pair<size_t, callback_t>> handlers;

// register a packet handler (inline version)
inline void register_packet(uint8_t id, size_t size, callback_t cb) {
    handlers[id] = std::make_pair(size, cb);
}

// blocking read to get "size" bytes from the file descriptor into buffer
// returns true on success
inline bool read_fully(int fd, void* buffer, size_t size) {
    uint8_t* ptr = static_cast<uint8_t*>(buffer);
    size_t total_read = 0;
    while (total_read < size) {
        ssize_t bytes = read(fd, ptr + total_read, size - total_read);
        if (bytes > 0) {
            total_read += bytes;
        } else {
            usleep(1000);  // back off a bit (seems to help)
        }
    }
    return true;
}

// serialize and send a packet with its ID prefix
template<typename T>
bool send_packet(int fd, uint8_t id, const T& packet) {
    uint8_t buffer[sizeof(T) + 1];
    buffer[0] = id;
    std::memcpy(buffer + 1, &packet, sizeof(T));
    return write(fd, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

// register all known packet types and handlers (TO BE DONE BEFOREHAND)
void register_cosco_callbacks();

// called periodically to poll for and process packets
void process_packets(int fd);

#endif // SERIAL_PROTOCOL_HPP
