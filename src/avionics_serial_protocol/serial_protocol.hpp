#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <unistd.h>

#pragma pack(push, 1)
struct MassConfigRequestPacket {
    uint16_t id;
    bool req_config;
};

struct MassConfigResponsePacket {
    uint16_t id;
    float offset;
    float scale;
    bool offset_set;
    bool scale_set;
};
#pragma pack(pop)

#define MassConfigRequest_ID 2
#define MassConfigResponse_ID 5

extern MassConfigRequestPacket latest_mass_config_request;
extern MassConfigResponsePacket latest_mass_config_response;

void mass_config_request_callback(const void* ptr);
void mass_config_response_callback(const void* ptr);
void setup_serial(int fd);

using callback_t = std::function<void(const void*)>;
void register_packet_handler(uint8_t id, size_t size, callback_t cb);
bool try_read_packet(int fd);

template<typename T>
bool send_packet(int fd, uint8_t id, const T& packet) {
    uint8_t buffer[sizeof(T) + 1];
    buffer[0] = id;
    std::memcpy(buffer + 1, &packet, sizeof(T));
    return write(fd, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

#endif // SERIAL_PROTOCOL_HPP
