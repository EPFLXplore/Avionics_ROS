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

const char* portname = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";

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

void Cosco::sendMassConfigPacket(MassConfigPacket *configPacket) {
    send_packet(fd, MassConfig_ID, *configPacket);
}

void Cosco::sendMassConfigRequestPacket(MassConfigRequestPacket *requestPacket) {
    send_packet(fd, MassConfigRequest_ID, *requestPacket);
}

void Cosco::sendMassConfigResponsePacket(MassConfigResponsePacket *responsePacket) {
    send_packet(fd, MassConfigResponse_ID, *responsePacket);
}

void Cosco::sendMassDataPacket(MassArray *massPacket) {
    send_packet(fd, MassData_ID, *massPacket);
}

void Cosco::sendServoRequestPacket(ServoRequest* requestPacket) {
    send_packet(fd, ServoConfigRequest_ID, *requestPacket);
}

void Cosco::sendServoResponsePacket(ServoResponse* responsePacket) {
    send_packet(fd, ServoResponse_ID, *responsePacket);
}

int Cosco::get_fd() const {
    return this->fd;
}
  

void Cosco::receive() {
    process_packets(fd);
}
