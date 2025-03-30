#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <functional>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

// below is the unique USB ID of the ESP32 when it is connected to port 0 of raspberry pi
const char* portname = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";

/*
    if the esp32 is the only device connected, this portname should work just fine:
    const char* portname = "/dev/ttyUSB0";
    or for macOS:
    const char* portname = "/dev/tty.usbserial-0001"
*/

/*
    the section below is critical for packet synchronization
    #pragma pack(push, 1) ensures no padding between packets, which is critical
    in a serial bus. This padding setting is "pushed" at compile time in an internal
    stack and popped at the end at the #pragma pack(pop) directive. Please add all
    packets inside this section, or very nasty out of sync packet bugs might happen
*/

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

MassConfigRequestPacket latest_mass_config_request;
MassConfigResponsePacket latest_mass_config_response;

/*
    the type below maps packet ID -> (packet size, callback function)
    allows dynamically dispatching to the correct handler
*/
using callback_t = std::function<void(const void*)>;
std::unordered_map<uint8_t, std::pair<size_t, callback_t>> packet_handlers;

bool read_fully(int fd, void* buffer, size_t size) {
    uint8_t* ptr = static_cast<uint8_t*>(buffer);
    size_t total_read = 0;
    const int timeout_ms = 100;

    auto start = std::chrono::steady_clock::now();

    while (total_read < size) {
        ssize_t bytes = read(fd, ptr + total_read, size - total_read);
        if (bytes > 0) {
            total_read += bytes;
        } else {
            usleep(1000);
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
            return false; // Timed out
        }
    }
    return true;
}

void mass_config_request_callback(const void* ptr) {
    const auto* data = reinterpret_cast<const MassConfigRequestPacket*>(ptr);
    latest_mass_config_request = *data;
    std::cout << "[MassConfigRequest] id=" << data->id
              << " req_config=" << std::boolalpha << data->req_config << "\n";
}

void mass_config_response_callback(const void* ptr) {
    const auto* data = reinterpret_cast<const MassConfigResponsePacket*>(ptr);
    latest_mass_config_response = *data;
    std::cout << "[MassConfigResponse] id=" << data->id
              << " offset=" << data->offset
              << " scale=" << data->scale
              << " offset_set=" << std::boolalpha << data->offset_set
              << " scale_set=" << data->scale_set << "\n";
}

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
    function that sends a raw packet by prepending the ID byte
*/
template<typename T>
bool send_packet(int fd, uint8_t id, const T& packet) {
    uint8_t buffer[sizeof(T) + 1];
    buffer[0] = id;
    std::memcpy(buffer + 1, &packet, sizeof(T));
    return write(fd, buffer, sizeof(buffer)) == sizeof(buffer);
}

int main() {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }
    fcntl(fd, F_SETFL, 0); // switch to blocking mode
    setup_serial(fd);

    // Register packet handlers
    packet_handlers[MassConfigRequest_ID]  = { sizeof(MassConfigRequestPacket),  mass_config_request_callback };
    packet_handlers[MassConfigResponse_ID] = { sizeof(MassConfigResponsePacket), mass_config_response_callback };

    std::cout << "[HOST] Sending and receiving packets...\n";

    uint16_t counter = 0;

    while (true) {
        // === SENDING ===
        MassConfigRequestPacket req = {
            .id = counter++,
            .req_config = true
        };
        send_packet(fd, MassConfigRequest_ID, req);
        std::cout << "[HOST] Sent MassConfigRequest\n";

        MassConfigResponsePacket resp = {
            .id = counter++,
            .offset = 1.11f,
            .scale = 2.22f,
            .offset_set = true,
            .scale_set = true
        };
        send_packet(fd, MassConfigResponse_ID, resp);
        std::cout << "[HOST] Sent MassConfigResponse\n";

        // === RECEIVING ===
        while (true) {
            uint8_t packet_id;
            ssize_t bytes_read = read(fd, &packet_id, 1);
            if (bytes_read != 1) break; // nothing left to read

            auto it = packet_handlers.find(packet_id);
            if (it == packet_handlers.end()) {
                std::cerr << "[HOST] Unknown packet ID: 0x"
                          << std::hex << (int)packet_id << std::dec << " — skipping\n";
                continue;
            }

            size_t size = it->second.first;
            std::vector<uint8_t> buffer(size);

            if (!read_fully(fd, buffer.data(), size)) {
                std::cerr << "[HOST] Incomplete packet for ID: " << (int)packet_id << "\n";
                continue;
            }

            it->second.second(buffer.data());
            break; // process only one packet per cycle
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    close(fd);
    return 0;
}
