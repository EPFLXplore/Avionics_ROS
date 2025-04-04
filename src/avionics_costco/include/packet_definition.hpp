/** 
 * @file packet_definition.hpp 
 * @author Eliot Abramo 
*/ 

#ifndef PACKET_DEFINITION_H
#define PACKET_DEFINITION_H

#include <iostream>
#include <packet_id.hpp>

#pragma pack(push, 1)

struct LEDMessage {
    uint8_t low;
    uint8_t high;
    uint8_t system;
    uint8_t mode;
};

struct DustData {
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    uint16_t pm1_0_atm;
    uint16_t pm2_5_atm;
    uint16_t pm10_atm;
    uint16_t num_particles_0_3;
    uint16_t num_particles_0_5;
    uint16_t num_particles_1_0;
    uint16_t num_particles_2_5;
    uint16_t num_particles_5_0;
    uint16_t num_particles_10;
};

struct MassCalibPacket {
    uint16_t id;
    float expected_weight;
    uint32_t num_samples;
    bool calibrate_offset;
    bool calibrate_scale;
};

struct FourInOne {
    uint16_t id;
    float temperature;
    float moisture;
    float conductivity;
    float ph;
};

struct ServoResponse {
    uint16_t id;
    int32_t angle;
    bool success;
};

struct MassConfigPacket {
    uint16_t id;
    float offset;
    float scale;
    bool set_offset;
    bool set_scale;
};

struct MassConfigResponsePacket {
    uint16_t id;
    float offset;
    float scale;
    bool offset_set;
    bool scale_set;
};

struct MassArray {
    uint16_t id;
    float mass;
};

struct NPK {
    uint16_t id;
    uint16_t nitrogen;
    uint16_t phosphorus;
    uint16_t potassium;
};

struct BMS {
    std::string status;
    float v_bat;
    float current;
    uint32_t voltages[4];
};

struct MassPacket {
    uint8_t id;
    float mass;
};

struct ServoRequest {
    uint16_t id;
    int32_t increment;
    bool zero_in;
};

struct LEDResponse {
    bool success;
};

struct MassConfigRequestPacket {
    uint16_t id;
    bool req_config;
};

#pragma pack(pop)

#endif /* PACKET_DEFINITION_H */