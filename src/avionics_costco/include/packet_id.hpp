/**
 * @file packet_id.hpp
 * @author Eliot Abramo
*/
#include <iostream>
#include "packet_definition.hpp"

// Mass packets
#define MassData_ID 1
#define MassConfigRequest_ID 2
#define MassCalib_ID 3
#define MassConfig_ID 4
#define MassConfigResponse_ID 5

// Servo packets
#define Servo_ID 6
#define ServoResponse_ID 7
#define ServoConfigRequest_ID 8
#define ServoConfig_ID 9
#define ServoConfigResponse_ID 10

// LED packets
#define LED_ID 11
#define LEDResponse_ID 12

// Dust packets
#define DustData_ID (15)

// FourInOne and NPK packets
#define FourInOne_ID 13
#define NPK_ID 14
