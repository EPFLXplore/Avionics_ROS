/**
 * @file packet_id.hpp
 * @author Eliot Abramo
*/

/**
 * CHANGE THIS PLEASE
 * This address list allows both the ROS code and communication protocol to understand what
 * we are trying to send them. It's essentially a big lookup table.
 * 
 * The code is okay, what's bad about this is that you have to manually ensure that ids are synchronized
 * between both codes. i.e. that the packet_id.hpp file is always the same. 
 * 
 * You can just make a submodule of this with the custom_msg and so you have:
 *      - custom_msg
 *      - generate_structs.cpp
 *      - packet_definition.hpp
 *
 * All in the same module. Talk with the future Software SE to see if we can add these three files somewhere in ERC_SE_CustomMessages
 * 
 */

#ifndef PACKET_ID_HPP
#define PACKET_ID_HPP

#include <iostream>
#include "packet_definition.hpp"

// Servo packets
#define ServoDrill_ID 1
#define ServoDrill_Response_ID 2 // Not used.

#define ServoCam_ID 3
#define ServoCam_Response_ID 4 // Not used.

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
#define NPK_ID 14 // Not used

// Dust packets
#define DustData_ID 15

#define Heartbeat_ID 20

#endif /*PACKET_ID_HPP*/