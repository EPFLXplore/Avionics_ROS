/**
 * @file SerialDriver.hpp
 * @author Eliot Abramo
 * @brief 
 * @date 2025-07-03
 */

#ifndef SERIAL_DRIVR_HPP
#define SERIAL_DRIVR_HPP

#include "StreamLike.hpp"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/*
* Inherits from custom StreamLike to try and keep as many similarities with the
* normal version as possible. Gives more flexibility to develop other drivers
* if need be and reduces differences with HW code.
*
* This code is just a very small hardware driver, code is a bit hard to explain
* but just follows the serial norms, bit allignements, procedures, etc. 
*/
class PosixSerial : public StreamLike {
public:
    PosixSerial(const std::string &dev, int baud); 

    ~PosixSerial() override { 
        if (fd_>=0) ::close(fd_); // close port when you kill object.
    }

    bool ok() const;

    size_t write(const uint8_t *buf, size_t len) override { 
        return ::write(fd_, buf, len); 
    }
    
    size_t write(uint8_t b) override {
        return ::write(fd_, &b, 1); 
    }
    
    void flush() override { 
        /* Wait for pending output to be written on FD.
        This function is a cancellation point and therefore not marked with
        a throw. */
        tcdrain(fd_);
    }
    
    int read() override { 
        uint8_t b; return (::read(fd_, &b, 1)==1)? b : -1; 
    }
    
    int available() override { 
        int n=0; 
        /* Perform the I/O control operation specified by REQUEST (FIONREAD) on FD.
        One argument may follow; its presence and type depend on REQUEST.
        Return value depends on REQUEST.  Usually -1 indicates error.  */
        ioctl(fd_, FIONREAD, &n); return n; 
    }

private:
    int fd_{-1};
    void open(const std::string& dev, int baud);
};

#endif // SERIAL_DRIVR_HPP
