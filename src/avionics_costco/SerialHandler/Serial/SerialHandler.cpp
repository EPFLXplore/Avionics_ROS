/*
* SerialHandler.cpp
* @author Eliot Abramo
*/

#include "SerialHandler.hpp"
#include <Wire.h>
#include <Arduino.h>

SerialHandler::SerialHandler()
{
    Serial.begin(115200);
    Serial.println("Serial launched");
}

void SerialHandler::sendMassConfigPacket(MassConfigPacket *configPacket)
{
    // Serialize and send MassConfigPacket
    uint8_t packetBuffer[sizeof(MassConfigPacket)];
    memcpy(packetBuffer, configPacket, sizeof(MassConfigPacket));
    Serial.write(packetBuffer, sizeof(MassConfigPacket));
}

void SerialHandler::sendMassConfigRequestPacket(MassConfigRequestPacket *requestPacket)
{
    // Serialize and send MassConfigRequestPacket
    uint8_t packetBuffer[sizeof(MassConfigRequestPacket)];
    memcpy(packetBuffer, requestPacket, sizeof(MassConfigRequestPacket));
    Serial.write(packetBuffer, sizeof(MassConfigRequestPacket));
}

void SerialHandler::sendMassConfigResponsePacket(MassConfigResponsePacket *responsePacket)
{
    // Serialize and send MassConfigResponsePacket
    uint8_t packetBuffer[sizeof(MassConfigResponsePacket)];
    memcpy(packetBuffer, responsePacket, sizeof(MassConfigResponsePacket));
    Serial.write(packetBuffer, sizeof(MassConfigResponsePacket));
}

void SerialHandler::sendMassDataPacket(MassData *responsePacket)
{
    // Serialize and send sendMassDataPacket
    uint8_t packetBuffer[sizeof(MassData)];
    memcpy(packetBuffer, responsePacket, sizeof(MassData));
    Serial.write(packetBuffer, sizeof(MassData));
}


void SerialHandler::receive(MassConfigPacket *configPacket, MassConfigRequestPacket *requestPacket, MassConfigResponsePacket *responsePacket)
{
    // Check if data is available
    if (Serial.available() > 0)
    {
        // Read the incoming data into a buffer
        char buffer[64];
        int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0'; // Null-terminate the string

        // Parse the command from the buffer
        int packetType;
        int parsed = sscanf(buffer, "%d", &packetType);

        // Check if the parsing was successful
        if (parsed == 1)
        {
            // Determine the packet type and send the corresponding packet
            if (packetType == 1)
            {
                sendMassConfigPacket(configPacket);
            }
            else if (packetType == 2)
            {
                sendMassConfigRequestPacket(requestPacket);
            }
            else if (packetType == 3)
            {
                sendMassConfigResponsePacket(responsePacket);
            }
            else
            {
                Serial.println("Unknown packet type");
            }
        }
        else
        {
            Serial.println("Failed to parse packet type");
        }
    }
}

// #include "Protocol23.hpp"
// #include <Wire.h>

// void SerialHandler(MassConfigPacket* configPacket, MassConfigRequestPacket* requestPacket, MassConfigResponsePacket* responsePacket) {
//     // Check if data is available
//     if (Serial.available() > 0) {
//         // Read the incoming data into a buffer
//         char buffer[64];
//         int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
//         buffer[len] = '\0'; // Null-terminate the string

//         // Parse the command from the buffer
//         int packetType;
//         int parsed = sscanf(buffer, "%d", &packetType);

//         // Check if the parsing was successful
//         if (parsed == 1) {
//             // Determine the packet type and send the corresponding packet
//             if (packetType == 1) {
//                 // Serialize and send MassConfigPacket
//                 uint8_t packetBuffer[sizeof(MassConfigPacket)];
//                 memcpy(packetBuffer, configPacket, sizeof(MassConfigPacket));
//                 Serial.write(packetBuffer, sizeof(MassConfigPacket));
//             } else if (packetType == 2) {
//                 // Serialize and send MassConfigRequestPacket
//                 uint8_t packetBuffer[sizeof(MassConfigRequestPacket)];
//                 memcpy(packetBuffer, requestPacket, sizeof(MassConfigRequestPacket));
//                 Serial.write(packetBuffer, sizeof(MassConfigRequestPacket));
//             } else if (packetType == 3) {
//                 // Serialize and send MassConfigResponsePacket
//                 uint8_t packetBuffer[sizeof(MassConfigResponsePacket)];
//                 memcpy(packetBuffer, responsePacket, sizeof(MassConfigResponsePacket));
//                 Serial.write(packetBuffer, sizeof(MassConfigResponsePacket));
//             } else {
//                 Serial.println("Unknown packet type");
//             }
//         } else {
//             Serial.println("Failed to parse packet type");
//         }
//     }
// }
