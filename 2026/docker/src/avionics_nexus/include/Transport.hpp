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
#include <poll.h>
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

    /** Read up to max bytes. Blocks in poll() for at most POLL_TIMEOUT_MS, so an
     *  idle link parks the thread instead of spinning; returns 0 on timeout.
     *  Throws std::runtime_error when the device went away, so the RX loop
     *  reconnects.
     *
     *  poll() rather than a blocking read(): the fd stays O_NONBLOCK (needed so
     *  open() fails fast on a dead port), which would otherwise make read()
     *  return EAGAIN instantly and burn a whole core. Clearing O_NONBLOCK and
     *  leaning on VMIN=0/VTIME=1 is not an option either - a blocking read()
     *  then returns 0 on *timeout*, which is indistinguishable from EOF.
     *
     *  On unplug the tty hangs up: poll() reports POLLHUP, or ::read() returns
     *  0 (EOF) or -1 with EIO/ENODEV/ENXIO. Only EAGAIN/EWOULDBLOCK means
     *  "still connected, just no data". */
    uint16_t read(uint8_t* dst, uint16_t max) {
        if (fd_ < 0) throw std::runtime_error("serial read: port not open");

        pollfd pfd{};
        pfd.fd     = fd_;
        pfd.events = POLLIN;

        const int pr = ::poll(&pfd, 1, POLL_TIMEOUT_MS);
        if (pr < 0) {
            if (errno == EINTR) return 0;   // signal, not an error
            throw std::runtime_error(std::string("serial poll: ") + std::strerror(errno));
        }
        if (pr == 0) return 0;              // idle: nothing arrived within the timeout
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))
            throw std::runtime_error("serial poll: device hung up (unplugged?)");

        ssize_t r = ::read(fd_, dst, max);
        if (r > 0) return static_cast<uint16_t>(r);
        if (r == 0) throw std::runtime_error("serial read: EOF (device unplugged?)");
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // raced with poll, fine
        throw std::runtime_error(std::string("serial read: ") + std::strerror(errno));
    }

  private:
    /** How long read() parks waiting for bytes. Long enough that an idle link
     *  costs ~nothing, short enough that shutdown and stall detection stay
     *  responsive. */
    static constexpr int POLL_TIMEOUT_MS = 100;

    int fd_ = -1;
};

#endif // POSIX_TRANSPORT_HPP
