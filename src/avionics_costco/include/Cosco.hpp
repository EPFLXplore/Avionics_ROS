/**
 * @file Cosco.hpp
 * @author Eliot Abramo
*/
#ifndef COSCO_HPP
#define COSCO_HPP

#include "packet_definition.hpp"
#include "packet_id.hpp"
#include "serial_protocol.hpp"
#include <string>


class Cosco {
public:
    /**
     * @brief Create a new Cosco Object
     */
    Cosco(std::string port_name);

    Cosco() {}
    
    /**
     * @brief Destroys a Cosco Object. Should unalocate any pointers and memory used up in class
     */    
    ~Cosco();

    /**
     * @brief Send mass data  packet
     * 
     * @param configPacket: pointer to packet to be sent. Defined in Packets->->packet_definition.hpp
     * @return null 
     */
    void sendMassPacket(MassPacket *responsePacket);

    /**
     * @brief Send mass configuration packet
     * 
     * @param requestPacket: pointer to packet to be sent. Defined in Packets->->packet_definition.hpp
     * @return null 
     */
    void sendServoRequestPacket(ServoRequest* requestPacket);

    /**
     * @brief Send mass configuration response packet
     * 
     * @param responsePacket: pointer to packet to be sent. Defined in Packets->->packet_definition.hpp
     * @return null 
     */
    void sendServoCamResponse(ServoResponse* pkt);
    void sendServoDrillResponse(ServoResponse* pkt);

    /**
     * @brief Send sensor data packet
     * 
     * @param dataPacket: pointer to packet to be sent. Defined in Packets->->packet_definition.hpp
     * @return null 
     */
    void sendDustDataPacket(DustData *dataPacket);

    int get_fd() const;

    /**
     * @brief functions that receive commands  
     * 
     * @param configPacket 
     * @param requestPacket 
     * @param responsePacket 
     * @return null
     */    
    void receive();

private:
    int fd;
    const char* _port_name;
};

#endif /* COSCO_HPP */