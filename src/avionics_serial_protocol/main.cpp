#include "serial_protocol.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

/*
    below is the unique USB ID of the ESP32 when it is connected to port 0 of raspberry pi
    if the esp32 is the only device connected, this portname should work just fine:
    const char* portname = "/dev/ttyUSB0";
    or for macOS:
    const char* portname = "/dev/tty.usbserial-0001"
*/
const char* portname = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";

int main() {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }

    fcntl(fd, F_SETFL, 0); // switch to blocking mode
    setup_serial(fd);

    // Register callbacks
    register_packet_handler(MassConfigRequest_ID, sizeof(MassConfigRequestPacket), mass_config_request_callback);
    register_packet_handler(MassConfigResponse_ID, sizeof(MassConfigResponsePacket), mass_config_response_callback);

    std::cout << "[HOST] Sending and receiving packets\n";

    uint16_t counter = 0;

    while (true) {
        // below is an example use of the sending and receiving functions
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
        auto start = std::chrono::steady_clock::now();
        int received_count = 0;
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(200)) {
            if (try_read_packet(fd)) {
                received_count++;
            } else {
                // optionally sleep a bit to avoid tight spinning
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        std::cout << "[HOST] Received " << received_count << " packets this cycle\n";
    }

    close(fd);
    return 0;
}
