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
        perror(dev.c_str()); return;
    }
    
    termios tty{}; 
    tcgetattr(fd_, &tty); 
    cfmakeraw(&tty);
    
    speed_t sp = B115200; // baud==115200

    cfsetispeed(&tty, sp); 
    cfsetospeed(&tty, sp);
    
    tty.c_cflag |= (CLOCAL|CREAD);
    tcsetattr(fd_, TCSANOW, &tty);
}
