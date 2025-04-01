/**
 * @file Cosco.cpp
 * @author Eliot Abramo
*/
#include "Cosco.hpp"
#include "serial_protocol.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

#include <packet_id.hpp>
#include <packet_definition.hpp>
// #include <libserialport.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// struct sp_port *port;

// // Macro de Ilyas
// #define HANDLE_PACKET(packet_type) do {                                         \
//     if (len == sizeof(packet_type)) {                                           \
//         memcpy(packet, buffer + 1, sizeof(packet_type));                        \
//         /**println(String(#packet_type) + " packet copied successfully");**/   \
//     } else {                                                                    \
//        /** Serial.println("Received data too "                                     \
//                         + String(len > sizeof(packet_type) ? "long" : "short")  \
//                         + " for " + String(#packet_type));                   */    \
//     }                                                                           \
//     break;                                                                      \
// } while (0)

// struct sp_port *port;

Cosco::Cosco()
{
    // sp_return result;
    // result = sp_get_port_by_name("/dev/ttyUSB0", &port);
    // if (result != SP_OK) {
    //     printf("Error opening serial port!\n");
    //     return;
    // }

    // result = sp_open(port, SP_MODE_READ_WRITE);
    // if (result != SP_OK) {
    //     printf("Error opening the port for read/write!\n");
    //     return;
    // }

    // sp_set_baudrate(port, 115200);
    // sp_set_bits(port, 8);
    // sp_set_parity(port, SP_PARITY_NONE);
    // sp_set_stopbits(port, 1);
    // sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);

    // printf("Serial launched\n");
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "[Cosco] Failed to open serial port\n";
        return;
    }
    fcntl(fd, F_SETFL, 0);
    setup_serial(fd);
    std::cout << "[Cosco] Serial port opened\n";
}

Cosco::~Cosco()
{
    // // Close the serial port
    // if (port != nullptr) {
    //     sp_close(port);
    // }
    if (fd != -1) {
        close(fd);
    }
}

void Cosco::sendMassConfigPacket(MassConfigPacket *configPacket)
{
    // // Serialize and send MassConfigPacket
    // uint8_t packetBuffer[sizeof(MassConfigPacket) + 1];
    // packetBuffer[0] = MassConfig_ID; 
    // memcpy(packetBuffer + 1, configPacket, sizeof(MassConfigPacket));

    // sp_nonblocking_write(port, packetBuffer, sizeof(MassConfigPacket) + 1);
    send_packet(fd, MassConfig_ID, *configPacket);
}

void Cosco::sendMassConfigRequestPacket(MassConfigRequestPacket *requestPacket)
{
    // // Serialize and send MassConfigRequestPacket
    // uint8_t packetBuffer[sizeof(MassConfigRequestPacket) + 1];
    // packetBuffer[0] = MassConfigRequest_ID; 
    // memcpy(packetBuffer + 1, requestPacket, sizeof(MassConfigRequestPacket));

    // sp_nonblocking_write(port, packetBuffer, sizeof(MassConfigRequestPacket) + 1);
    send_packet(fd, MassConfigRequest_ID, *requestPacket);
}

void Cosco::sendMassConfigResponsePacket(MassConfigResponsePacket *responsePacket)
{
    // // Serialize and send MassConfigResponsePacket
    // uint8_t packetBuffer[sizeof(MassConfigResponsePacket) + 1];
    // packetBuffer[0] = MassConfigResponse_ID;
    // memcpy(packetBuffer + 1, responsePacket, sizeof(MassConfigResponsePacket));

    // sp_nonblocking_write(port, packetBuffer, sizeof(MassConfigResponsePacket) + 1);
    send_packet(fd, MassConfigResponse_ID, *responsePacket);
}

void Cosco::sendMassDataPacket(MassArray *massPacket)
{
    // // Serialize and send sendMassDataPacket
    // uint8_t packetBuffer[sizeof(MassArray) + 1];
    // packetBuffer[0] = MassData_ID;
    // memcpy(packetBuffer + 1, massPacket, sizeof(MassArray));

    // sp_nonblocking_write(port, packetBuffer, sizeof(MassArray) + 1);
    send_packet(fd, MassData_ID, *massPacket);
}

void Cosco::sendServoRequestPacket(ServoRequest* requestPacket)
{
    // // Serialize and send sendServoRequestPacket
    // uint8_t packetBuffer[sizeof(ServoRequest) + 1];
    // packetBuffer[0] = ServoConfigRequest_ID;
    // memcpy(packetBuffer + 1, requestPacket, sizeof(ServoRequest));

    // sp_nonblocking_write(port, packetBuffer, sizeof(ServoRequest) + 1);
    send_packet(fd, ServoConfigRequest_ID, *requestPacket);
}

void Cosco::sendServoResponsePacket(ServoResponse* responsePacket)
{
    // // Serialize and send sendServoResponsePacket
    // uint8_t packetBuffer[sizeof(ServoResponse) + 1];
    // packetBuffer[0] = ServoResponse_ID;
    // memcpy(packetBuffer + 1, responsePacket, sizeof(ServoResponse));

    // sp_nonblocking_write(port, packetBuffer, sizeof(ServoResponse) + 1);
    send_packet(fd, ServoResponse_ID, *responsePacket);
}


void Cosco::receive()
{
    // uint8_t packet_id = 0; // Variable to hold the packet ID
    //     char buffer[128] = {0}; // Buffer to hold incoming data
    // int len = 0;
    process_packets(fd);

//     // Check if data is available (non-blocking read)
//     len = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
//     if (len > 0) {
//         buffer[len] = '\0'; // Null-terminate the string

//         // Read and store the first byte (the "first bit")
//         packet_id = static_cast<uint8_t>(buffer[0]);
        
//         switch (packet_id) {
//             case MassData_ID: HANDLE_PACKET(MassPacket);
//             case MassConfigRequest_ID: HANDLE_PACKET(MassConfigRequestPacket);
//             // case MassCalib_ID: HANDLE_PACKET();
//             // case MassConfig_ID: HANDLE_PACKET();
//             case MassConfigResponse_ID: HANDLE_PACKET(MassConfigResponsePacket);

//             default: {
//                 printf("Unknown packet ID\n");
//                 break;
//             }
//         }
//     }
}




// /**
//  * @file Cosco.cpp
//  * @author Eliot Abramo
// */
// #include "Cosco.hpp"
// // #include <Arduino.h>
// #include <packet_id.hpp>
// #include <packet_definition.hpp>
// #include <libserialport.h>
// #include <stdio.h>
// #include <stdint.h>
// #include <string.h> // memcpy

// // Macro de Ilyas
// #define HANDLE_PACKET(packet_type) do {                                         \
//     if (len == sizeof(packet_type)) {                                           \
//         memcpy(packet, buffer + 1, sizeof(packet_type));                        \
//         Serial.println(String(#packet_type) + " packet copied successfully");   \
//     } else {                                                                    \
//         Serial.println("Received data too "                                     \
//                         + String(len > sizeof(packet_type) ? "long" : "short")  \
//                         + " for " + String(#packet_type));                  Serial    \
//     }                                                                           \
//     break;                                                                      \
// } while (0)

// Cosco::Cosco()
// {
//     // Serial.begin(115200);
//     // Open the serial port
//     sp_return result;
//     result = sp_get_port_by_name("/dev/ttyUSB0", &port);
//     if (result != SP_OK) {
//         printf("Error opening serial port!\n");
//         return;
//     }

//     result = sp_open(port, SP_MODE_READ_WRITE);
//     if (result != SP_OK) {
//         printf("Error opening the port for read/write!\n");
//         return;
//     }

//     // Set serial port options (baud rate, bits, stop bits, etc.)
//     sp_set_baudrate(port, 115200);
//     sp_set_bits(port, 8);
//     sp_set_parity(port, SP_PARITY_NONE);
//     sp_set_stopbits(port, 1);
//     sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);

//     printf("Serial launched\n");
//     Serial.println("Serial launched");
// }

// Cosco::~Cosco()
// {
//     // Close the serial port
//     if (port != nullptr) {
//         sp_close(port);
//     }
// }

// void Cosco::sendMassConfigPacket(MassConfigPacket *configPacket)
// {
//     // Serialize and send MassConfigPacket
//     uint8_t packetBuffer[sizeof(MassConfigPacket) + 1];
//     packetBuffer[0] = MassConfig_ID; 
//     memcpy(packetBuffer + 1, configPacket, sizeof(MassConfigPacket));

//     sp_nonblocking_write(port, packetBuffer, sizeof(MassConfigPacket) + 1);
// }

// void Cosco::sendMassConfigRequestPacket(MassConfigRequestPacket *requestPacket)
// {
//     // Serialize and send MassConfigRequestPacket
//     uint8_t packetBuffer[sizeof(MassConfigRequestPacket) + 1];
//     packetBuffer[0] = MassConfigRequest_ID; 
//     memcpy(packetBuffer + 1, requestPacket, sizeof(MassConfigRequestPacket));

//     sp_nonblocking_write(port, packetBuffer, sizeof(MassConfigRequestPacket) + 1);
// }

// void Cosco::sendMassConfigResponsePacket(MassConfigResponsePacket *responsePacket)
// {
//     // Serialize and send MassConfigResponsePacket
//     uint8_t packetBuffer[sizeof(MassConfigResponsePacket) + 1];
//     packetBuffer[0] = MassConfigResponse_ID;
//     memcpy(packetBuffer + 1, responsePacket, sizeof(MassConfigResponsePacket));

//     sp_nonblocking_write(port, packetBuffer, sizeof(MassConfigResponsePacket) + 1);
// }

// void Cosco::sendMassDataPacket(MassArray *massPacket)
// {
//     // Serialize and send sendMassDataPacket
//     uint8_t packetBuffer[sizeof(MassArray) + 1];
//     packetBuffer[0] = MassData_ID;
//     memcpy(packetBuffer + 1, massPacket, sizeof(MassArray));

//     sp_nonblocking_write(port, packetBuffer, sizeof(MassArray) + 1);
// }

// void sendServoRequestPacket(ServoRequest* requestPacket)
// {
//     // Serialize and send sendServoRequestPacket
//     uint8_t packetBuffer[sizeof(ServoRequest) + 1];
//     packetBuffer[0] = ServoConfigRequest_ID;
//     memcpy(packetBuffer + 1, requestPacket, sizeof(ServoRequest));

//     sp_nonblocking_write(port, packetBuffer, sizeof(ServoRequest) + 1);
// }

// void sendServoResponsePacket(ServoResponse* responsePacket)
// {
//     // Serialize and send sendServoResponsePacket
//     uint8_t packetBuffer[sizeof(ServoResponse) + 1];
//     packetBuffer[0] = ServoResponse_ID;
//     memcpy(packetBuffer + 1, responsePacket, sizeof(ServoResponse));

//     sp_nonblocking_write(port, packetBuffer, sizeof(ServoResponse) + 1);
// }



// void Cosco::receive(void* packet)
// {
//     uint8_t packet_id = 0; // Variable to hold the packet ID
//         char buffer[128] = {0}; // Buffer to hold incoming data
//     int len = 0;

//     // Check if data is available (non-blocking read)
//     len = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
//     if (len > 0) {
//         buffer[len] = '\0'; // Null-terminate the string

//         // Read and store the first byte (the "first bit")
//         packet_id = static_cast<uint8_t>(buffer[0]);
        
//         switch (packet_id) {
//             case MassData_ID: HANDLE_PACKET(MassPacket);
//             case MassConfigRequest_ID: HANDLE_PACKET(MassConfigRequestPacket);
//             // case MassCalib_ID: HANDLE_PACKET();
//             // case MassConfig_ID: HANDLE_PACKET();
//             case MassConfigResponse_ID: HANDLE_PACKET(MassConfigResponsePacket);

//             default: {
//                 printf("Unknown packet ID\n");
//                 break;
//             }
//         }
//     }
// }

