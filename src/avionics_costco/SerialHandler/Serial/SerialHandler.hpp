/*
* SerialHandler.hpp
* @author Eliot Abramo
*/

#ifndef SERIALHANDLER_HPP
#define SERIALHANDLER_HPP

#include "Protocol23.hpp"

class SerialHandler {
public:
    SerialHandler();
    ~SerialHandler(){}

    void sendMassConfigPacket(MassConfigPacket* configPacket);
    void sendMassConfigRequestPacket(MassConfigRequestPacket* requestPacket);
    void sendMassConfigResponsePacket(MassConfigResponsePacket* responsePacket);
    void sendMassDataPacket(MassData *responsePacket);
    
    void receive(MassConfigPacket* configPacket, MassConfigRequestPacket* requestPacket, MassConfigResponsePacket* responsePacket);
};

#endif // SERIALHANDLER_HPP