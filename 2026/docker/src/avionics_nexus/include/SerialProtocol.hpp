/**
 * @file SerialProtocol.hpp
 * @author Eliot Abramo (original) · reworked for the USB-FS native transport
 *
 * @brief Self-synchronising, CRC16-framed serial protocol.
 *
 * Difference with the original Avionics_stack / avionics_nexus version:
 *   - The transport is a COMPILE-TIME POLICY (template parameter) instead of a
 *     runtime StreamLike& vtable. The concrete transport is known at compile
 *     time, so every call inlines and the abstraction costs nothing.
 *   - send() assembles the WHOLE frame in one stack buffer and hands it to the
 *     transport in a single write() - one USB transfer per frame, no per-byte
 *     poking and no flush(). The assembly memcpy is the deliberate price of
 *     "one transfer per frame"; do not "optimise" it back into byte streaming.
 *   - parse() drains a whole USB packet through the RX state machine in one go.
 *
 * The framing and CRC are byte-for-byte identical to the original, so this end
 * talks to any peer still running the StreamLike version.
 *
 *  Frame format (little-endian):
 *  +------------+------------+----------------+----------+----------------+---------+
 *  | 0xA5 (STX) | 0x5A       | uint16 len     | uint8 id | payload[len-1] | CRC16   |
 *  +------------+------------+----------------+----------+----------------+---------+
 *  - len counts *id + payload* (so min len=1). CRC is X25/Modbus (poly 0xA001).
 *
 * The Transport is a CONCEPT, not a base class. Any type providing these two
 * buffer-oriented methods satisfies it (see CdcTransport / PosixTransport):
 *
 *      bool     write(const uint8_t* d, uint16_t n);  // whole frame; false if not queued
 *      uint16_t read (uint8_t* dst, uint16_t max);    // up to max bytes available now
 *
 *  Typical use on the transmitting side:
 *      SerialProtocol<128, MyTransport> proto(io);
 *      MyPacket pkt{ ... };
 *      proto.send(kMyPacketId, &pkt, sizeof(pkt));
 *
 *  On the receiving side (inside the comms loop):
 *      uint8_t chunk[64];
 *      uint16_t n = io.read(chunk, sizeof chunk);
 *      proto.parse(chunk, n, [&](const Frame& f){ dispatch(f); });
 */

#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

template <std::size_t MaxPayload, class Transport> // allows us to set max payload however we want
class SerialProtocol {
  public:
    struct Frame {
        uint8_t id;                              // Packet ID
        uint16_t length;                         // payload length, < MaxPayload (def above)
        std::array<uint8_t, MaxPayload> payload; // raw bytes
    };

    explicit SerialProtocol(Transport& io) : io_(io) {}

    /****************************** Send ******************************
     * @brief Assemble one complete frame and hand it to the transport whole.
     * @param id: application-level packet identification (see packets.h)
     * @param payload: raw bytes to send
     * @param len: length of payload (1 <= len <= MaxPayload)
     *
     * Safety guards:
     *   - oversized or zero-length payloads are silently dropped.
     *   - the frame is built in a stack buffer and pushed with ONE io_.write();
     *     "make sure the bytes leave" is now the transport's job, not flush()'s.
     */
    bool send(uint8_t id, const void* payload, uint16_t len) {
        if (len == 0 || len > MaxPayload) return false; // drop oversized/empty packets

        uint8_t buf[MaxFrame]; // compile-time bound, lives on the stack
        uint16_t n = 0;

        /* Start-of-frame markers (see comment of processByte) */
        buf[n++] = kStx1;
        buf[n++] = kStx2;

        /* Length = id + payload = 1 + payload */
        const uint16_t L = static_cast<uint16_t>(len + 1);
        buf[n++] = static_cast<uint8_t>(L & 0xFF);
        buf[n++] = static_cast<uint8_t>((L >> 8) & 0xFF);

        /* ID byte */
        buf[n++] = id;

        /* Raw payload */
        const uint8_t* p = static_cast<const uint8_t*>(payload);
        std::memcpy(buf + n, p, len);
        n += len;

        /* CRC (calculated over ID + payload) */
        const uint16_t crc = crc16(id, p, len);
        buf[n++] = static_cast<uint8_t>(crc & 0xFF);
        buf[n++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

        return io_.write(buf, n); // ONE transfer: TxState busy lives in the transport
    }

    /****************************** Parse *****************************
     * @brief Drain a chunk of received bytes through the state machine.
     * @param data: bytes just read from the transport
     * @param len: how many
     * @param onFrame: callable invoked with the Frame for every complete frame
     *
     * Lets a whole USB packet flow through the self-syncing FSM in one pass
     * instead of pulling one byte at a time. Packet boundaries are invisible to
     * the parser: the frame's own len + CRC delimit it, so a frame split across
     * packets, or several frames coalesced into one read(), both reassemble.
     */
    template <class OnFrame>
    void parse(const uint8_t* data, uint16_t len, OnFrame&& onFrame) {
        for (uint16_t i = 0; i < len; ++i) {
            if (processByte(data[i])) onFrame(frame_); // emit each complete frame
        }
    }

    /** This feeds a single byte into the protocol state machine. It returns true
     * if a complete, valid frame has been received. This is an aggressive filter
     * for any noise on the serial line: packets have to be perfect or they are
     * rejected, which means a stray printf can never crash the pipeline.
     *
     *   +------------+------------+----------------+----------+----------------+---------+
     *  | 0xA5 (STX) | 0x5A       | uint16 len     | uint8 id | payload[len-1] | CRC16   |
     *  +------------+------------+----------------+----------+----------------+---------+
     *  - len counts *id + payload* (so min len=1). CRC is X25/Modbus (poly 0xA001).
     *
     * Start bytes 0xA5, 0x5A were chosen because they're rare and bit-inverse of
     * each other, so a complete bit-flip on the bus still won't fake a frame.
     */
    bool processByte(uint8_t b) {
        switch (state_) {
            /**** 0xA5 hunt ****/
            case State::Stx1:
                if (b == kStx1) state_ = State::Stx2;
                break;

            /**** 0x5A confirmation ****/
            case State::Stx2:
                state_ = (b == kStx2) ? State::LenLo : State::Stx1;
                break;

            /**** length low byte ****/
            case State::LenLo:
                len_ = b;
                state_ = State::LenHi;
                break;

            /**** length high byte ****/
            case State::LenHi:
                len_ |= static_cast<uint16_t>(b) << 8;
                // sanity check
                if (len_ == 0 || len_ > MaxPayload + 1) { ++badLen_; reset(); break; }
                bytes_ = 0; // new payload counter
                state_ = State::Id;
                break;

            /**** packet ID ****/
            case State::Id:
                frame_.id = b;
                bytes_ = 0;
                state_ = (len_ == 1) ? State::CrcLo : State::Payload;
                break;

            /**** payload stream ****/
            case State::Payload:
                frame_.payload[bytes_++] = b;
                if (bytes_ == len_ - 1) state_ = State::CrcLo;
                break;

            /**** CRC16 LSB ****/
            case State::CrcLo:
                crcRead_ = b;
                state_ = State::CrcHi;
                break;

            /**** CRC16 MSB & verdict ****/
            case State::CrcHi:
                crcRead_ |= static_cast<uint16_t>(b) << 8;
                if (crcRead_ == crc16(frame_.id, frame_.payload.data(), len_ - 1)) {
                    frame_.length = len_ - 1; // strip ID
                    reset();                  // ready for next frame
                    return true;              // success!
                }
                /* CRC mismatch -> drop frame and resync */
                ++crcErrors_;
                reset();
                break;
        }
        return false; // frame not yet finished
    }

    /** Grab the last good frame. Only call RIGHT AFTER processByte() returned
     * true (or use it inside the parse() callback). */
    const Frame& frame() const { return frame_; }

    /* Diagnostics: cumulative dropped-frame counts since construction. crcErrors
     * framed correctly but failed CRC; badLen had an impossible length field.
     * Both mean line noise / desync that the self-syncing FSM recovered from. */
    uint32_t crcErrors() const { return crcErrors_; }
    uint32_t badLen()    const { return badLen_; }

  private:
    enum class State : uint8_t { Stx1, Stx2, LenLo, LenHi, Id, Payload, CrcLo, CrcHi };

    static constexpr uint8_t kStx1 = 0xA5;                       // start token 1
    static constexpr uint8_t kStx2 = 0x5A;                       // start token 2
    static constexpr uint16_t MaxFrame = MaxPayload + 7;         // STX2 + len2 + id1 + payload + crc2

    Transport& io_;            // wire abstraction (compile-time policy)
    Frame frame_{};            // rolling RX buffer
    State state_ = State::Stx1; // current parser state
    uint16_t len_ = 0;         // expected (id+payload) length
    uint16_t bytes_ = 0;       // payload bytes read so far
    uint16_t crcRead_ = 0;     // CRC from wire
    uint32_t crcErrors_ = 0;   // frames dropped on CRC mismatch (diagnostics)
    uint32_t badLen_ = 0;      // frames dropped on impossible length (diagnostics)

    /* Reset parser to STX hunt */
    void reset() {
        state_ = State::Stx1;
        len_ = bytes_ = crcRead_ = 0;
    }

    static uint16_t updateCrc(uint16_t crc, uint8_t b) {
        crc ^= b;
        for (uint8_t i = 0; i < 8; ++i) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        return crc;
    }

    static uint16_t crc16(uint8_t id, const uint8_t* data, uint16_t len) {
        uint16_t crc = 0xFFFF;
        crc = updateCrc(crc, id);
        for (uint16_t i = 0; i < len; ++i) crc = updateCrc(crc, data[i]);
        return crc;
    }
};

#endif // SERIAL_PROTOCOL_HPP
