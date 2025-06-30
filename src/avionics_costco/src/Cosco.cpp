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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

const char* portname = "/dev/ttyESP32_Avionics";

Cosco::Cosco() {
    fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        perror("[Cosco] Failed to open serial port");
        return;
    }
    fcntl(fd, F_SETFL, 0);
    setup_serial(fd);
    std::cout << "[Cosco] Serial port opened" << std::endl;
}

Cosco::~Cosco() {
    if (fd != -1) {
        close(fd);
    }
}

void Cosco::sendMassPacket(MassPacket *massPacket) {
    send_packet(fd, MassDrill_ID, *massPacket);
}

void Cosco::sendServoRequestPacket(ServoRequest* requestPacket) {
    if(requestPacket->id == ServoCam_ID){
        send_packet(fd, 1, *requestPacket);
    } else if(requestPacket->id == ServoDrill_ID){
        send_packet(fd, 2, *requestPacket);
    }
}

int Cosco::get_fd() const {
    return this->fd;
}

void Cosco::receive() {
    process_packets(fd);
}
