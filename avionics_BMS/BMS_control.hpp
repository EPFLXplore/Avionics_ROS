#ifndef BMS_CONTROL_H
#define BMS_CONTROL_H

#include "CppLinuxSerial/SerialPort.hpp"

/**** BMS ADDRESS 8 bits ****/
#define ADDR 0xAA

/**** BMS COMMANDS: 8 bits ****/
#define RESET       0x12
#define NEW_EVENTS  0x11 // get all new events: packet length (bytes) in 3rd byte of response
#define ALL_EVENTS  0x12 // get all events: packet length (bytes) in 3rd byte of response

#define PACK_VOLTAGE 0x14 //sum of voltage of all cells
#define PACK_CURRENT 0x15 //sum of current of all cells

#define MAX_CELL_VOLT 0x16 //max cell voltage configuration
#define MIN_CELL_VOLT 0x17 //min cell voltage configuration

#define STATUS              0x18 //get status of BMS: see status ids below
#define LIFETIME_COUNTER    0x19 //get lifetime counter of BMS
#define TEMPERATURE         0x1B //get temperatures of BMS: 16 bits; internal_temp, ext_temp1, ext_temp2

#define CELLS_VOLTAGE 0x1C //get voltage of all cells: 16 bits per cell, length in bytes defined in 3rd byte

#define VERSION 0x1E //get version of BMS: 16 bits; length in bytes defined in 3rd byte of response

#define READ_INDIVIDUAL_REGISTERS   0x09 //get individual registers: 16 bits per register, length in bytes defined in 3rd byte of request
#define WRITE_INDIVIDUAL_REGISTERS  0x0D //set individual registers: 16 bits per register, length in bytes defined in 3rd byte of request

/**** BMS RESET OPTIONS: 8 bits ****/
#define EVENTS  0x01
#define Stats   0x02
#define BMS     0x05

/**** BMS RESPONSES: 8 bits ****/
#define NACK_UART_ERROR 0x00
#define ACK_UART        0x01

/*** SPECIAL_ERROR types ***/
#define CMD_ERROR 0x00
#define CRC_ERROR 0x01

/**** STATUS IDS: 16 bits ****/
#define CHARGING        0x91
#define CHARGED         0x92
#define DISCHARGING     0x93
#define REGEN           0x96
#define IDLE            0x97
#define FAULT           0x9B

/**** Data positions ****/
#define DATA_ADDR   0
#define DATA_ACK    1
#define DATA_PL     2
#define DATA_CMD    2

//check if command has a dynamic length response
#define IS_DYNAMIC_LEN(command) (command == NEW_EVENTS || command == ALL_EVENTS || command == CELLS_VOLTAGE || command == VERSION)

class BMSControl {
public:
    // Constructor
    BMSControl(mn::CppLinuxSerial::SerialPort& serialPort) : serialPort(serialPort) {}

    // Destructor
    ~BMSControl() {this->serialPort.Close();} // automatically close serial port

    // API
    void init_BMS_comm();
    bool reset_BMS(uint8_t option);
    bool send_command(uint8_t command);
    std::vector<uint8_t> send_read_command(uint8_t command);
    bool write_individual_registers(std::vector<uint16_t> addresses, std::vector<uint16_t> data);
    std::vector<uint8_t> read_individual_registers(std::vector<uint16_t> addresses);

    // WRITE FUNCTIONS
    uint16_t write_1_byte(uint8_t data);
    uint16_t write_n_bytes(uint8_t command, std::vector<uint16_t> data);

    // READ FUNCTION
    std::vector<uint8_t> read_n_bytes(uint8_t command, uint16_t crc);

    // UTILS
    bool check_packet_error(std::vector<uint8_t> response, uint8_t command, uint16_t crc);
    uint16_t append_crc(std::vector<uint8_t>& packet);
    inline uint16_t CRC16(const uint8_t* data, uint16_t length);

    struct report {
        uint8_t event;
        uint8_t length;
        uint8_t* data;
    };

private:
    mn::CppLinuxSerial::SerialPort& serialPort;
};

#endif // BMS_CONTROL_H