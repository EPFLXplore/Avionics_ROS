/**
 * @file SerialDriver.cpp
 * @author Eliot Abramo
 * @brief 
 * @date 2025-07-03
 */

#include "SerialDriver.hpp"

PosixSerial::PosixSerial(const std::string &dev, int baud) { 
    open(dev, baud); 
}
    

bool PosixSerial::ok() const {
    return fd_>=0; 
}

void PosixSerial::open(const std::string& dev, int baud) {
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        // Print a message describing the meaning of the value of errno.
        perror(dev.c_str()); return;
    }

    // Struct def by termios-struct.h. Needed by linux to interface with USB
    termios tty{}; 
    // Put the state of FD into *tty.
    tcgetattr(fd_, &tty); 
    // Set *TERMIOS_P to indicate raw mode so that can send raw byte by byte with FSM. 
    cfmakeraw(&tty);
    
    speed_t sp = B115200; // baud==115200

    cfsetispeed(&tty, sp); // set baud rate input
    cfsetospeed(&tty, sp); // set baud rate output
    
    tty.c_cflag |= (CLOCAL|CREAD); //set flags for what we want to do

    // Set the state of FD to tty. More flags to set. 
    tcsetattr(fd_, TCSANOW, &tty);
}
