#include "BMS_control.hpp"
#include <iostream>
#include <chrono>
using namespace mn::CppLinuxSerial;

// Example usage
int main() {
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    BMSControl TinyBMS(serialPort);
    TinyBMS.init_BMS_comm();

    int attempts = 5;
    // while(attempts){
    //     std::vector<uint8_t> packet = {ADDR, 0x03, 0x01, 0xF4, 0x00, 0x04, 0x1D, 0xDC};
    //     serialPort.WriteBinary(packet);
    // attempts--;
    // }

    // std::vector<uint8_t> packet = {ADDR, RESET, 0x01, 0xF4, 0x00, 0x04, 0x1D, 0xDC};
    // serialPort.WriteBinary(packet);

    // // reset BMS
    // std::cout << "Test" << std::endl;
    // attempts = 5;
    // while(attempts){
    //     bool success = TinyBMS.reset_BMS(BMS);
    //     if(success)
    //         std::cout << "BMS reset successfully" << std::endl;
    //     else
    //         std::cout << "BMS reset failed" << std::endl;
    //     attempts--;
    //     // Add a small delay using busy-wait loop
    //     auto start = std::chrono::high_resolution_clock::now();
    //     while (std::chrono::high_resolution_clock::now() - start < std::chrono::milliseconds(1000)) {
    //         // Busy-wait loop
    //     }
    // }

    while(attempts){
        // get total voltage
        std::vector<uint8_t> pack_voltage = TinyBMS.send_read_command(PACK_VOLTAGE);
        if (pack_voltage.empty())
            std::cout << "PACK_VOLTAGE: ERROR" << std::endl;
        else
            std::cout << "PACK_VOLTAGE: " << pack_voltage[0] << std::endl;
        // Add a small delay using busy-wait loop
        auto start = std::chrono::high_resolution_clock::now();
        while (std::chrono::high_resolution_clock::now() - start < std::chrono::milliseconds(1000)) {
            // Busy-wait loop
        }
        attempts--;
    }

    // get temperature readings
    std::vector<uint8_t> temps = TinyBMS.send_read_command(TEMPERATURE);
    if(temps.empty())
        std::cout << "TEMPERATURE: ERROR" << std::endl;
    else
        for(uint8_t val : temps)
            std::cout << (int)val << std::endl;

    return 0;
}