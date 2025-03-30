#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <functional>
#include <unordered_map>

const char* portname = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";

#pragma pack(push, 1)
struct DustData {
    uint16_t pm1_0_std, pm2_5_std, pm10__std;
    uint16_t pm1_0_atm, pm2_5_atm, pm10__atm;
    uint16_t num_particles_0_3, num_particles_0_5;
    uint16_t num_particles_1_0, num_particles_2_5;
    uint16_t num_particles_5_0, num_particles_10_;
};

struct MassData {
    float mass[4];
};
#pragma pack(pop)

const uint8_t MassData_ID = 1;
const uint8_t DustData_ID = 15;

DustData latestDustData;
MassData latestMassData;

using Callback = std::function<void(const void*)>;
std::unordered_map<uint8_t, std::pair<size_t, Callback>> packetHandlers;

bool read_fully(int fd, void* buffer, size_t size) {
    uint8_t* ptr = static_cast<uint8_t*>(buffer);
    size_t total_read = 0;
    while (total_read < size) {
        ssize_t bytes = read(fd, ptr + total_read, size - total_read);
        if (bytes > 0) {
            total_read += bytes;
        } else {
            usleep(1000);
        }
    }
    return true;
}

void dust_callback(const void* ptr) {
    const DustData* data = reinterpret_cast<const DustData*>(ptr);
    latestDustData = *data;
    std::cout << "[DustData] ";
    for (int i = 0; i < 12; ++i)
        std::cout << reinterpret_cast<const uint16_t*>(data)[i] << " ";
    std::cout << "\n";
}

void mass_callback(const void* ptr) {
    const MassData* data = reinterpret_cast<const MassData*>(ptr);
    latestMassData = *data;
    std::cout << "[MassData] "
              << "CH1: " << data->mass[0] << "\t"
              << "CH2: " << data->mass[1] << "\t"
              << "CH3: " << data->mass[2] << "\t"
              << "CH4: " << data->mass[3] << "\n";
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

int main() {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }
    fcntl(fd, F_SETFL, 0); // switch to blocking mode
    setup_serial(fd);

    // Register packet handlers
    packetHandlers[DustData_ID] = { sizeof(DustData), dust_callback };
    packetHandlers[MassData_ID] = { sizeof(MassData), mass_callback };

    std::cout << "[HOST] Listening for packets\n";

    while (true) {
        uint8_t packetID;
        if (read(fd, &packetID, 1) != 1) {
            usleep(1000);
            continue;
        }

        auto it = packetHandlers.find(packetID);
        if (it != packetHandlers.end()) {
            size_t size = it->second.first;
            Callback& cb = it->second.second;
            std::vector<uint8_t> buffer(size);

            if (!read_fully(fd, buffer.data(), size)) {
                std::cerr << "Incomplete packet for ID " << (int)packetID << "\n";
                continue;
            }
            cb(buffer.data());
        } else {
            std::cerr << "[HOST] Unknown packet ID: 0x"
                      << std::hex << (int)packetID << std::dec << "\n";
        }
    }

    close(fd);
    return 0;
}
