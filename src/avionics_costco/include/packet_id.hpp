/**
 * @file packet_id.hpp
 * @author Eliot Abramo
*/

#ifndef PACKET_ID_HPP
#define PACKET_ID_HPP

#include <iostream>
#include "packet_definition.hpp"

// Servo packets
#define ServoDrill_ID 1
#define ServoDrill_Response_ID 2

#define ServoCam_ID 3
#define ServoCam_Response_ID 4

// Mass packets
#define MassDrill_ID 5
#define MassDrill_Request_ID 6

#define MassHD_ID 7
#define MassHD_Request_ID 8

// LED packets
#define LED0_ID 11
#define LED1_ID 12

// FourInOne and NPK packets
#define FourInOne_ID 13
#define NPK_ID 14

// Dust packets
#define DustData_ID 15

#define Heartbeat_ID 20

#endif /*PACKET_ID_HPP*/