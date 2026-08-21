/**
 * @file SerialProtocol.h
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
 *      proto.send(MY_PACKET_ID, &pkt, sizeof(pkt));
 *
 *  On the receiving side (inside the comms loop):
 *      uint8_t chunk[64];
 *      uint16_t n = io.read(chunk, sizeof chunk);
 *      proto.parse(chunk, n, [&](const Frame& f){ dispatch(f); });
 */

#pragma once
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

    explicit SerialProtocol(Transport& io) : _io(io) {}

    /****************************** Send ******************************
     * @brief Assemble one complete frame and hand it to the transport whole.
     * @param id: application-level packet identification (see packets.h)
     * @param payload: raw bytes to send
     * @param len: length of payload (1 <= len <= MaxPayload)
     *
     * Safety guards:
     *   - oversized or zero-length payloads are silently dropped.
     *   - the frame is built in a stack buffer and pushed with ONE _io.write();
     *     "make sure the bytes leave" is now the transport's job, not flush()'s.
     */
    bool send(uint8_t id, const void* payload, uint16_t len) {
        if (len == 0 || len > MaxPayload) return false; // drop oversized/empty packets

        uint8_t buf[MaxFrame]; // compile-time bound, lives on the stack
        uint16_t n = 0;

        /* Start-of-frame markers (see comment of processByte) */
        buf[n++] = STX1;
        buf[n++] = STX2;

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

        return _io.write(buf, n); // ONE transfer: TxState busy lives in the transport
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
        if (len) _idlePolls = 0; // bytes are flowing: the frame is still arriving
        for (uint16_t i = 0; i < len; ++i) {
            if (processByte(data[i])) onFrame(_frame); // emit each complete frame
        }
    }

    /****************************** Idle ******************************
     * @brief Tell the parser a poll produced NO bytes. Call it every time.
     *
     * This is the reset the state machine was missing. Every other exit from a
     * half-built frame is driven by a byte that arrives - a bad length, a failed
     * CRC. Nothing handled the frame whose remaining bytes never arrive at all,
     * which is exactly what a full RX ring produces: the tail of the frame is
     * dropped by the ISR, the FSM is left parked in Payload waiting for bytes
     * that no longer exist, and it then eats the NEXT frame's header as payload.
     * That frame dies too, on CRC - so ONE ring overflow silently costs at least
     * two commands, and the second one looks for all the world like a command
     * that was never sent.
     *
     * The line going quiet mid-frame is unambiguous evidence of that: on USB-FS
     * a command frame (<= 18 bytes) always fits in ONE 64-byte packet, so its
     * bytes are never legitimately separated by an idle poll. IDLE_ABORT_POLLS
     * of margin covers a host that splits one anyway.
     *
     * Counted in truncated(), NOT in badLen()/crcErrors(): those mean the bytes
     * arrived corrupted, this means they never arrived.
     */
    void idle() {
        if (_state == StateType::Stx1) { _idlePolls = 0; return; } // not mid-frame
        if (++_idlePolls < IDLE_ABORT_POLLS) return;
        ++_truncated;
        reset();
        _idlePolls = 0;
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
        switch (_state) {
            /**** 0xA5 hunt ****/
            case StateType::Stx1:
                if (b == STX1) _state = StateType::Stx2;
                break;

            /**** 0x5A confirmation ****/
            case StateType::Stx2:
                if (b == STX2)      _state = StateType::LenLo;
                else if (b == STX1) _state = StateType::Stx2; // stay: THIS byte is
                    // the marker now. Dropping to Stx1 here lost a real frame
                    // whenever the byte before its header happened to be 0xA5
                    // (or a truncated frame left one behind): A5 A5 5A used to
                    // resync on the FIRST A5, reject the second, and then hunt
                    // past the 5A - one whole frame missed per stray A5.
                else                _state = StateType::Stx1;
                break;

            /**** length low byte ****/
            case StateType::LenLo:
                _len = b;
                _state = StateType::LenHi;
                break;

            /**** length high byte ****/
            case StateType::LenHi:
                _len |= static_cast<uint16_t>(b) << 8;
                // sanity check
                if (_len == 0 || _len > MaxPayload + 1) { ++_badLen; reset(); break; }
                _bytes = 0; // new payload counter
                _state = StateType::Id;
                break;

            /**** packet ID ****/
            case StateType::Id:
                _frame.id = b;
                _bytes = 0;
                _state = (_len == 1) ? StateType::CrcLo : StateType::Payload;
                break;

            /**** payload stream ****/
            case StateType::Payload:
                _frame.payload[_bytes++] = b;
                if (_bytes == _len - 1) _state = StateType::CrcLo;
                break;

            /**** CRC16 LSB ****/
            case StateType::CrcLo:
                _crcRead = b;
                _state = StateType::CrcHi;
                break;

            /**** CRC16 MSB & verdict ****/
            case StateType::CrcHi:
                _crcRead |= static_cast<uint16_t>(b) << 8;
                if (_crcRead == crc16(_frame.id, _frame.payload.data(), _len - 1)) {
                    _frame.length = _len - 1; // strip ID
                    reset();                  // ready for next frame
                    return true;              // success!
                }
                /* CRC mismatch -> drop frame and resync */
                ++_crcErrors;
                reset();
                break;
        }
        return false; // frame not yet finished
    }

    /** Grab the last good frame. Only call RIGHT AFTER processByte() returned
     * true (or use it inside the parse() callback). */
    const Frame& frame() const { return _frame; }

    /* Diagnostics: cumulative dropped-frame counts since construction. crcErrors
     * framed correctly but failed CRC; badLen had an impossible length field.
     * Both mean line noise / desync that the self-syncing FSM recovered from. */
    uint32_t crcErrors() const { return _crcErrors; }
    uint32_t badLen()    const { return _badLen; }
    /** Frames abandoned half-received because the line went quiet - see idle().
     *  A nonzero value here means bytes are being LOST upstream of the parser
     *  (RX ring overflow), not corrupted on the wire. */
    uint32_t truncated() const { return _truncated; }

  private:
    enum class StateType : uint8_t { Stx1, Stx2, LenLo, LenHi, Id, Payload, CrcLo, CrcHi };

    static constexpr uint8_t STX1 = 0xA5;                       // start token 1
    static constexpr uint8_t STX2 = 0x5A;                       // start token 2
    static constexpr uint16_t MaxFrame = MaxPayload + 7;         // STX2 + len2 + id1 + payload + crc2
    /* Consecutive empty polls before a half-received frame is abandoned. In
     * POLLS, not milliseconds - the parser has no clock, and its two callers
     * poll at very different rates: the MCU's SerialThread every 1 ms (so this
     * is ~5 ms), Nexus's rxLoop up to 100 ms per poll() (so ~500 ms). Both are
     * far longer than any gap inside a single-packet frame and far shorter than
     * the 3 s stall detector that tears the link down. */
    static constexpr uint8_t IDLE_ABORT_POLLS = 5;

    Transport& _io;            // wire abstraction (compile-time policy)
    Frame _frame{};            // rolling RX buffer
    StateType _state = StateType::Stx1; // current parser state
    uint16_t _len = 0;         // expected (id+payload) length
    uint16_t _bytes = 0;       // payload bytes read so far
    uint16_t _crcRead = 0;     // CRC from wire
    uint32_t _crcErrors = 0;   // frames dropped on CRC mismatch (diagnostics)
    uint32_t _badLen = 0;      // frames dropped on impossible length (diagnostics)
    uint32_t _truncated = 0;   // frames abandoned half-received (see idle())
    uint8_t  _idlePolls = 0;   // consecutive empty polls while mid-frame

    /* Reset parser to STX hunt */
    void reset() {
        _state = StateType::Stx1;
        _len = _bytes = _crcRead = 0;
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

