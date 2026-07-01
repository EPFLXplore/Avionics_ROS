/**
 * @file Transport.hpp
 * @brief PosixTransport: the RPi side of the SerialProtocol Transport concept.
 *
 * Mirror of the MCU's CdcTransport: a thin POSIX serial driver exposing exactly
 * the two buffer-oriented methods SerialProtocol<MaxPayload, Transport> needs:
 *
 *      bool     write(const uint8_t* d, uint16_t n);  // whole frame
 *      uint16_t read (uint8_t* dst, uint16_t max);    // bytes available now
 *
 * It replaces the old StreamLike vtable: the protocol is templated on this type,
 * so there is no virtual dispatch. Framing/CRC live entirely in the protocol;
 * this class only knows the file descriptor.
 */

#ifndef POSIX_TRANSPORT_HPP
#define POSIX_TRANSPORT_HPP

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

class PosixTransport {
  public:
    PosixTransport() = default;
    ~PosixTransport() { if (fd_ >= 0) ::close(fd_); }

    /** Open the serial device in raw mode. Throws std::runtime_error on failure
     *  (e.g. not plugged in yet); the RX loop catches and retries.
     *
     *  This is a USB-FS CDC virtual COM port (~12 Mbit/s on the wire), so the
     *  termios line speed is ignored by the device: we just pin it to the max. */
    void open(const std::string& port) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("open " + port + ": " + std::strerror(errno));

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) { const int e = errno; close(); throw std::runtime_error(std::string("tcgetattr: ") + std::strerror(e)); }

        cfmakeraw(&tty);
        cfsetispeed(&tty, B4000000);
        cfsetospeed(&tty, B4000000);
        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem lines, enable receiver
        tty.c_cc[VMIN]  = 0;             // read() returns as soon as bytes arrive...
        tty.c_cc[VTIME] = 1;             // ...or after 100 ms, so the RX loop never busy-spins

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { const int e = errno; close(); throw std::runtime_error(std::string("tcsetattr: ") + std::strerror(e)); }
        tcflush(fd_, TCIOFLUSH);
    }

    bool ok() const { return fd_ >= 0; }
    void close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

    /** Push a whole frame; true if all bytes were accepted. */
    bool write(const uint8_t* d, uint16_t n) {
        if (fd_ < 0) return false;
        return ::write(fd_, d, n) == static_cast<ssize_t>(n);
    }

    /** Read up to max bytes available now. Returns 0 when nothing is available
     *  yet (EAGAIN, the normal idle case with O_NONBLOCK); throws
     *  std::runtime_error when the device went away, so the RX loop reconnects.
     *
     *  On unplug the tty hangs up: ::read() then returns 0 (EOF) or -1 with
     *  EIO/ENODEV/ENXIO. Only EAGAIN/EWOULDBLOCK means "still connected, just no
     *  data"; everything else (including r == 0) is a disconnect. */
    uint16_t read(uint8_t* dst, uint16_t max) {
        ssize_t r = ::read(fd_, dst, max);
        if (r > 0) return static_cast<uint16_t>(r);
        if (r == 0) throw std::runtime_error("serial read: EOF (device unplugged?)");
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // connected, no data now
        throw std::runtime_error(std::string("serial read: ") + std::strerror(errno));
    }

  private:
    int fd_ = -1;
};

#endif // POSIX_TRANSPORT_HPP
