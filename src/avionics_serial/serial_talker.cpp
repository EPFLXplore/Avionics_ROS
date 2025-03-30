#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>

// below is the unique USB ID of the ESP32 when it is connected to port 0 of raspberry pi
const char* portname = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";

/*
    if the esp32 is the only device connected, this portname should work just fine:
    const char* portname = "/dev/ttyUSB0";
    or for macOS:
    const char* portname = "/dev/tty.usbserial-0001"
*/

// same #pragma pack trick to avoid padding in struct serialization
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

// Sends raw struct with its ID prepended
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

    std::cout << "[HOST] Sending packets to ESP32\n";

    while (true) {
        // example: send a MassConfigRequest every 2 seconds
        MassConfigRequestPacket request = {
            .id = 69,
            .req_config = true
        };

        if (!send_packet(fd, MassConfigRequest_ID, request)) {
            std::cerr << "Failed to send MassConfigRequest\n";
        } else {
            std::cout << "[Sent] MassConfigRequest (id=" << request.id << ")\n";
        }

        // example: send a MassConfigResponse as well
        MassConfigResponsePacket response = {
            .id = 42,
            .offset = -1.23f,
            .scale = 3.141f,
            .offset_set = true,
            .scale_set = true
        };

        if (!send_packet(fd, MassConfigResponse_ID, response)) {
            std::cerr << "Failed to send MassConfigResponse\n";
        } else {
            std::cout << "[Sent] MassConfigResponse (id=" << response.id << ")\n";
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    close(fd);
    return 0;
}
