/**
 * @file Transport.h
 * @brief PosixTransport: the RPi side of the SerialProtocol Transport concept.
 *
 * Mirror of the MCU's CdcTransport: a thin POSIX serial driver exposing exactly
 * the two buffer-oriented methods SerialProtocol<MaxPayload, Transport> needs:
 *
 *      bool     write(const uint8_t* data, uint16_t len);  // whole frame
 *      uint16_t read (uint8_t* dest, uint16_t maxLen);     // bytes available now
 *
 * It replaces the old StreamLike vtable: the protocol is templated on this type,
 * so there is no virtual dispatch. Framing/CRC live entirely in the protocol;
 * this class only knows the file descriptor.
 */

#pragma once

#include <chrono>
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
    ~PosixTransport() { if (_fd >= 0) ::close(_fd); }

    /** Open the serial device in raw mode. Throws std::runtime_error on failure
     *  (e.g. not plugged in yet); the RX loop catches and retries.
     *
     *  This is a USB-FS CDC virtual COM port (~12 Mbit/s on the wire), so the
     *  termios line speed is ignored by the device: we just pin it to the max. */
    void open(const std::string& port) {
        _fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (_fd < 0) throw std::runtime_error("open " + port + ": " + std::strerror(errno));

        termios tty{};
        if (tcgetattr(_fd, &tty) != 0) { const int e = errno; close(); throw std::runtime_error(std::string("tcgetattr: ") + std::strerror(e)); }

        cfmakeraw(&tty);
        cfsetispeed(&tty, B4000000);
        cfsetospeed(&tty, B4000000);
        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem lines, enable receiver
        tty.c_cc[VMIN]  = 0;             // read() returns as soon as bytes arrive...
        tty.c_cc[VTIME] = 1;             // ...or after 100 ms, so the RX loop never busy-spins

        if (tcsetattr(_fd, TCSANOW, &tty) != 0) { const int e = errno; close(); throw std::runtime_error(std::string("tcsetattr: ") + std::strerror(e)); }
        tcflush(_fd, TCIOFLUSH);

        _lastRx    = std::chrono::steady_clock::now();
        _linkReset = true;   // epoch: the protocol resyncs its parser off this

        /* FLUSH PREAMBLE. Reopening this fd fires no USB event on the MCU, so a
         * parser left parked mid-frame over there has no way to learn the link
         * changed - and it would splice our first real frame onto the tail of a
         * dead one. MAX_FRAME zero bytes run any parked parser out to its
         * declared length and fail it on CRC, which is a clean resync. This is
         * the one loss case the MCU cannot observe for itself. */
        static const uint8_t preamble[135] = {};
        (void)::write(_fd, preamble, sizeof preamble);
    }

    bool ok() const { return _fd >= 0; }

    /** Housekeeping, given a clock. On THIS side that means stall detection:
     *  the fd stays valid and read() never errors when the MCU goes quiet - a
     *  hung TX ring, a USB suspend, a re-enumeration onto a different ttyACM
     *  minor while we hold the old one. None of those make read() throw, so
     *  without this the link stays dead forever and even a replug does not
     *  recover it. Throwing hands over to the RX loop's reconnect path.
     *
     *  (The MCU's CdcTransport::serviceLink does something quite different -
     *  it releases a TX whose completion never came. Same concept, "you have a
     *  clock, do your housekeeping"; deliberately different work.)
     *
     *  nowMs is unused here: this side already has steady_clock, and read() is
     *  what knows when bytes last arrived. */
    void serviceLink(uint32_t /*nowMs*/) {
        if (_fd < 0) return;
        if (std::chrono::steady_clock::now() - _lastRx <= STALL_TIMEOUT) return;
        _lastRx = std::chrono::steady_clock::now();   // do not re-throw every call
        throw std::runtime_error("no bytes for " +
                                 std::to_string(STALL_TIMEOUT.count()) + "s (MCU silent)");
    }

    /** True ONCE per successful open(). */
    bool linkReset() { const bool was = _linkReset; _linkReset = false; return was; }

    /** Bytes taken off the wire since construction (diagnostics). */
    uint64_t rxBytes() const { return _rxBytes; }
    void close() { if (_fd >= 0) { ::close(_fd); _fd = -1; } }

    /** Push a whole frame; true if all bytes were accepted. */
    bool write(const uint8_t* data, uint16_t len) {
        if (_fd < 0) return false;
        return ::write(_fd, data, len) == static_cast<ssize_t>(len);
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
    uint16_t read(uint8_t* dest, uint16_t maxLen) {
        if (_fd < 0) throw std::runtime_error("serial read: port not open");

        pollfd pfd{};
        pfd.fd     = _fd;
        pfd.events = POLLIN;

        const int ready = ::poll(&pfd, 1, POLL_TIMEOUT_MS);
        if (ready < 0) {
            if (errno == EINTR) return 0;   // signal, not an error
            throw std::runtime_error(std::string("serial poll: ") + std::strerror(errno));
        }
        if (ready == 0) return 0;           // idle: nothing arrived within the timeout
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))
            throw std::runtime_error("serial poll: device hung up (unplugged?)");

        ssize_t got = ::read(_fd, dest, maxLen);
        if (got > 0) { _rxBytes += static_cast<uint64_t>(got);
                       _lastRx = std::chrono::steady_clock::now();
                       return static_cast<uint16_t>(got); }
        if (got == 0) throw std::runtime_error("serial read: EOF (device unplugged?)");
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // raced with poll, fine
        throw std::runtime_error(std::string("serial read: ") + std::strerror(errno));
    }

  private:
    /** How long read() parks waiting for bytes. Long enough that an idle link
     *  costs ~nothing, short enough that shutdown and stall detection stay
     *  responsive. */
    static constexpr int POLL_TIMEOUT_MS = 100;

    /** How long the MCU may stay silent before the link is presumed dead. */
    static constexpr std::chrono::seconds STALL_TIMEOUT{3};

    std::chrono::steady_clock::time_point _lastRx{};
    bool     _linkReset = false;
    uint64_t _rxBytes   = 0;

    int _fd = -1;
};

