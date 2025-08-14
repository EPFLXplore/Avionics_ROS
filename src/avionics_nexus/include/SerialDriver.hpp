

#ifndef SERIAL_DRIVR_HPP
#define SERIAL_DRIVR_HPP

#include "StreamLike.hpp"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class PosixSerial : public StreamLike {
public:
    PosixSerial(const std::string &dev, int baud); 

    ~PosixSerial() override { 
        if (fd_>=0) ::close(fd_); 
    }

    bool ok() const;

    size_t write(const uint8_t *buf, size_t len) override { 
        return ::write(fd_, buf, len); 
    }
    
    size_t write(uint8_t b) override {
        return ::write(fd_, &b, 1); 
    }
    
    void flush() override { 
        tcdrain(fd_);
    }
    
    int read() override { 
        uint8_t b; return (::read(fd_, &b, 1)==1)? b : -1; 
    }
    
    int available() override { 
        int n=0; ioctl(fd_, FIONREAD, &n); return n; 
    }

private:
    int fd_{-1};
    void open(const std::string& dev, int baud);
};

#endif // SERIAL_DRIVR_HPP
