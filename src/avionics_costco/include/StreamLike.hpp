/**
 * @file StreamLike.hpp
 * @author Eliot Abramo
 * @brief 
 * @date 2025-07-03
 */

#ifndef STREAM_HPP
#define STREAM_HPP

#include <array>
#include <cstdint>
#include <iostream>

class StreamLike {
public:
    virtual ~StreamLike() = default;
    virtual size_t write(const uint8_t *buf, size_t len)=0;
    virtual size_t write(uint8_t b)=0;
    virtual void flush()=0;
    virtual int read()=0;          // blocking single‑byte read, -1 on error
    virtual int available()=0;     // bytes in driver RX queue
};

#endif // STREAM_HPP
