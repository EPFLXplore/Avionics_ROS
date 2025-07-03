/**
 * @file SerialProtocol.hpp
 * @author Eliot Abramo
 * 
 * @brief Implements a simple serial protocol for framing, sending, and receiving packets with CRC16 error checking.
 * This template class provides methods to send and receive framed data packets over a generic stream interface.
 * Each packet includes a start-of-frame marker, length, packet ID, payload, and CRC16 checksum for integrity.
 * The protocol supports incremental byte-wise parsing and validates frames before making them available.
 *
 * @date 2025-07-03
 */

#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include "StreamLike.hpp"

/**
 * 
 * Robust, self‑synchronising framed serial protocol.
 *
 *  Frame format (little‑endian):
 *  +------------+------------+----------------+----------+----------------+---------+
 *  | 0xA5 (STX) | 0x5A       | uint16 len     | uint8 id | payload[len-1] | CRC16   |
 *  +------------+------------+----------------+----------+----------------+---------+
 *  - len counts *id + payload* (so min len=1). CRC is X25/Modbus (poly 0xA001).
 *
 *  Typical use on the transmitting side:
 *      SerialProtocol<128> proto(Serial);
 *      MyPacket pkt{ ... };
 *      proto.send(kMyPacketId, &pkt, sizeof(pkt));
 *
 *  On the receiving side (inside loop()):
 *      while (Serial.available()) {
 *          if (proto.processByte(Serial.read())) {
 *              handleFrame(proto.frame());
 *          }
 *      }
 */

template <std::size_t MaxPayload>
class SerialProtocol {
  public:
    struct Frame {
        uint8_t id; // Packet ID
        uint16_t length; // Length, HAS TO BE less than MaxPayload (def above)
        std::array<uint8_t, MaxPayload> payload; // Raw bytes
    };

    SerialProtocol(){}
    explicit SerialProtocol(StreamLike &stream) : s_(stream) {}

    void send(uint8_t id, const void *payload, uint16_t len) {
        if (len > MaxPayload || len == 0) return; // only allow correctly sized packets, otherwise errors
        write8(kStx1); // Write start-of-frame marker (first sync byte)
        write8(kStx2); // Write second sync byte to mark frame start
        write16(static_cast<uint16_t>(len + 1));   // +1 for id byte
        write8(id);
        const uint8_t *p = static_cast<const uint8_t *>(payload);
        s_.write(p, len);
        write16(crc16(id, p, len));
        s_.flush(); // ensure bytes hit the wire
    }

    /** Feed one byte. Returns true when a full valid frame has been assembled. */
    bool processByte(uint8_t b) {
        switch (state_) {
            case State::Stx1:
                if (b == kStx1) state_ = State::Stx2;
                break;
            case State::Stx2:
                state_ = (b == kStx2) ? State::LenLo : State::Stx1;
                break;
            case State::LenLo:
                len_ = b;
                state_ = State::LenHi;
                break;
            case State::LenHi:
                len_ |= static_cast<uint16_t>(b) << 8;
                if (len_ == 0 || len_ > MaxPayload + 1) { reset(); break; }
                bytes_ = 0;
                state_ = State::Id;
                break;
            case State::Id:
                frame_.id = b;
                bytes_ = 0;
                state_ = (len_ == 1) ? State::CrcLo : State::Payload;
                break;
            case State::Payload:
                frame_.payload[bytes_++] = b;
                if (bytes_ == len_ - 1) state_ = State::CrcLo;
                break;
            case State::CrcLo:
                crcRead_ = b;
                state_ = State::CrcHi;
                break;
            case State::CrcHi:
                crcRead_ |= static_cast<uint16_t>(b) << 8;
                if (crcRead_ == crc16(frame_.id, frame_.payload.data(), len_ - 1)) {
                    frame_.length = len_ - 1;
                    reset();
                    return true;
                }
                reset();
                break;
        }
        return false;
    }

    /** Last valid frame, only safe to access after processByte() returned true */
    const Frame &frame() const { return frame_; }

  private:
    enum class State : uint8_t { Stx1, Stx2, LenLo, LenHi, Id, Payload, CrcLo, CrcHi };

    static constexpr uint8_t kStx1 = 0xA5;
    static constexpr uint8_t kStx2 = 0x5A;

    StreamLike &s_;
    Frame frame_{};
    State state_ = State::Stx1;
    uint16_t len_ = 0;
    uint16_t bytes_ = 0;
    uint16_t crcRead_ = 0;

    void reset() {
        state_ = State::Stx1;
        len_ = bytes_ = crcRead_ = 0;
    }

    void write8(uint8_t v) { s_.write(v); }
    void write16(uint16_t v) {
        s_.write(static_cast<uint8_t>(v & 0xFF));
        s_.write(static_cast<uint8_t>((v >> 8) & 0xFF));
    }

    static uint16_t updateCrc(uint16_t crc, uint8_t b) {
        crc ^= b;
        for (uint8_t i = 0; i < 8; ++i) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        return crc;
    }
    static uint16_t crc16(uint8_t id, const uint8_t *data, uint16_t len) {
        uint16_t crc = 0xFFFF;
        crc = updateCrc(crc, id);
        for (uint16_t i = 0; i < len; ++i) crc = updateCrc(crc, data[i]);
        return crc;
    }
};

#endif // SERIAL_PROTOCOL_HPP
