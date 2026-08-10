/**
 * @file Nexus.h
 * @brief The RP wire-owner: mirror of the MCU's SerialThread.
 *
 * Collapses the old NexusMain + NexusPublisher + NexusSubscriber (+ the global
 * publisher pointers and the raw std::thread) into a single rclcpp::Node that
 * owns the transport and the protocol. One class, one switch(id) on RX, one
 * subscription callback per command on TX. Only the far side differs from the
 * MCU: ROS topics here, FreeRTOS queues there.
 *
 * Thread-safety: the RX thread is the only caller of _proto.parse()/_io.read();
 * the executor (single-threaded, see NexusMain) is the only caller of
 * _proto.send()/_io.write(). One owner per direction, no shared mutable state
 * between them, so no locks are needed.
 */

#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "custom_msg/msg/heartbeat.hpp"
#include "custom_msg/msg/led_request.hpp"
#include "custom_msg/msg/mass_packet.hpp"
#include "custom_msg/msg/mass_request.hpp"
#include "custom_msg/msg/ph_packet.hpp"
#include "custom_msg/msg/ph_request.hpp"
#include "custom_msg/msg/servo_request.hpp"

#include "SerialProtocol.h"
#include "Transport.h"
#include "device_ids.h"
#include "packets.h"

/* Every load cell in the FLEET, with the mass_cal.yaml key that carries its
 * slope - not the cells behind any one master. Nexus deliberately does not know
 * which master serves which device: commands go out to every port and the
 * firmware drops the ids its board profile does not carry (deviceFor() ->
 * nullptr).
 *
 * The parameter is named after the DEVICE, not its id number, so a slope is
 * readable in the yaml and survives any renumbering of MassIdType. The names live
 * here rather than in device_ids.h because they are a ROS-config concern - the
 * firmware has no use for strings - but the name<->id pairing is exactly what
 * must not drift, so it is checked below rather than trusted.
 *
 * At namespace scope, not inside Nexus: a constexpr member cannot be called by a
 * static_assert in its own class body (the class is still incomplete there). */
struct MassParam {
    uint8_t     id;    // global MassIdType from device_ids.h
    const char* param; // ros parameter key in mass_cal.yaml
};

constexpr MassParam MASS_PARAMS[] = {
    { idOf(MassIdType::SandRocks), "mass_slope_sand_rocks" },
    { idOf(MassIdType::Drill),     "mass_slope_drill"      },
};
constexpr std::size_t MASS_ID_COUNT = sizeof(MASS_PARAMS) / sizeof(MASS_PARAMS[0]);

/* Every declared load cell has a slope parameter. Add a MassIdType to device_ids.h
 * without adding its key here and the build fails, instead of that cell silently
 * running the firmware fallback forever. */
static_assert(MASS_ID_COUNT == sizeof(ALL_MASS_IDS) / sizeof(ALL_MASS_IDS[0]),
              "MASS_PARAMS and MassIdType disagree: every load cell needs a slope parameter");

/* ...and in the same order, so MASS_PARAMS[i].id == ALL_MASS_IDS[i] and
 * _slopes[i] means the same cell whichever table you walk. This is the check
 * that stops a slope being replayed against the wrong scale: swap two lines in
 * MASS_PARAMS and it will not compile. */
constexpr bool massParamsMatchIds() {
    for (std::size_t i = 0; i < MASS_ID_COUNT; ++i)
        if (MASS_PARAMS[i].id != ALL_MASS_IDS[i]) return false;
    return true;
}
static_assert(massParamsMatchIds(),
              "MASS_PARAMS is not in MassIdType order: a slope would be sent to the wrong cell");

/* Every servo in the FLEET, with the servo_cal.yaml key holding its home angle.
 *
 * Same contract as MASS_PARAMS: named after the device, bound to ServoIdType at
 * compile time, checked below so a home angle cannot land on the wrong servo.
 * One number each - the angle in degrees that go_to_zero drives to - because the
 * MCU turns it into microseconds with the same constexpr map it uses for a
 * commanded angle, so a configured home and a commanded one cannot disagree. */
struct ServoParam {
    uint8_t     id;
    const char* param;
};

constexpr ServoParam SERVO_PARAMS[] = {
    { idOf(ServoIdType::FrontCam),           "servo_zero_front_cam"            },
    { idOf(ServoIdType::Drill),              "servo_zero_drill"                },
    { idOf(ServoIdType::LeftServiceModule),  "servo_zero_left_service_module"  },
    { idOf(ServoIdType::RightServiceModule), "servo_zero_right_service_module" },
};
constexpr std::size_t SERVO_ID_COUNT = sizeof(SERVO_PARAMS) / sizeof(SERVO_PARAMS[0]);

static_assert(SERVO_ID_COUNT == sizeof(ALL_SERVO_IDS) / sizeof(ALL_SERVO_IDS[0]),
              "SERVO_PARAMS and ServoIdType disagree: every servo needs a home-angle parameter");

constexpr bool servoParamsMatchIds() {
    for (std::size_t i = 0; i < SERVO_ID_COUNT; ++i)
        if (SERVO_PARAMS[i].id != ALL_SERVO_IDS[i]) return false;
    return true;
}
static_assert(servoParamsMatchIds(),
              "SERVO_PARAMS is not in ServoIdType order: a home angle would go to the wrong servo");

/* The pH probe's calibration keys in ph_cal.yaml.
 *
 * No id and no table: PhPacket carries no id because there is one probe in the
 * rover and only ever will be, so there is nothing to index. The MCU converts
 * volts to pH itself and publishes the finished value; these two numbers are the
 * only thing the RPi ever tells it about the probe. */
constexpr const char* PH_SLOPE_PARAM  = "ph_slope_solution";
constexpr const char* PH_OFFSET_PARAM = "ph_offset_solution";

class Nexus : public rclcpp::Node {
  public:
    /** One node per master id (0..3). Binds to /dev/ttyNova<id>; if that port is
     *  not present the node just keeps warning + retrying (rxLoop reconnect), so
     *  a master plugged in later is picked up with no relaunch. */
    explicit Nexus(int id);
    ~Nexus() override;

  private:
    using Proto = SerialProtocol<128, PosixTransport>;
    using Frame = Proto::Frame;

    /* RX: serial -> ROS (own thread) */
    void rxLoop();
    void onFrame(const Frame& f);

    /* TX: ROS -> serial (executor callbacks) */
    void onServoReq(const custom_msg::msg::ServoRequest::SharedPtr msg);
    void onMassReq(const custom_msg::msg::MassRequest::SharedPtr msg);
    void onLedReq(const custom_msg::msg::LEDRequest::SharedPtr msg);
    void onPhReq(const custom_msg::msg::PhRequest::SharedPtr msg);

    /* TX gate: commands are broadcast to every Nexus, but only the ports with a
     * live link forward them. Down link -> count + drop silently (DEBUG), so a
     * command aimed at master 0 doesn't make masters 1..3 warn-spam. */
    bool txReady(const char* what);

    /* Calibration replay: push the configured slopes to a freshly connected MCU.
     * The MCU keeps its slope in RAM, so every reset (power cycle, watchdog,
     * reflash) drops it back to the firmware fallback in MassThread.h. Both of
     * those re-enumerate USB, which the RX loop sees as a reconnect - so
     * replaying on every link-up is what makes mass_cal.yaml the source of
     * truth without any flash writes on the MCU. */
    /* Everything that has to be (re)asserted on a freshly connected MCU. Gated on
     * proof the far side is alive, then runs once per link-up. */
    void onLinkReady();
    void replayCalibration();
    void replayPhCalibration();
    void sendMassScale(uint8_t id, float scale);
    void sendPhCal(float slope, float offset);
    void replayServoZeros();
    void sendServoZero(uint8_t id, int32_t angle);

    /* Announce that avionics is alive on the LED strip. The strip lives on one
     * master, but this goes out on every port for the same reason commands do:
     * Nexus has no board profile, and a master with no strip just drops it
     * (LedsThread is never started, so nothing consumes the queue). Re-sent on
     * every link-up because an MCU reset takes the strip back to its boot state. */
    void sendAvionicsLedOn();

    /* LedSystemType / LedModeType come from device_ids.h in the shared messages
     * submodule, so this and LEDStrip::handleMode() on the firmware compile
     * against the same enum rather than two hand-kept copies of the numbers. */

    /* size-checked reinterpret of a frame payload as a wire struct */
    template <class T>
    static T as(const Frame& f) {
        T out{};
        if (f.length == sizeof(T)) std::memcpy(&out, f.payload.data(), sizeof(T));
        return out;
    }

    /* Runtime guard: a frame's payload length must equal the struct we cast it
     * to, else the wire contract drifted (or it's noise). Warn and skip. */
    template <class T>
    bool lengthOk(const Frame& f, const char* what) {
        if (f.length == sizeof(T)) return true;
        RCLCPP_WARN(get_logger(), "[%s] %s: payload length %u != struct size %zu (dropped)",
                    _port.c_str(), what, static_cast<unsigned>(f.length), sizeof(T));
        return false;
    }

    /* Silence budget before the RX loop tears the port down and reopens it.
     * The MCU heartbeats at ~10 Hz, so 3 s of nothing is unambiguously dead,
     * while still being far longer than any legitimate gap. */
    static constexpr std::chrono::seconds STALL_TIMEOUT{3};

    /* Is this a device the fleet actually declares? Used only to WARN - the
     * request is still forwarded either way, because broadcast IS the routing
     * model: every master sees every command and drops what its board profile
     * does not carry. But an id that no board can ever match is a publisher bug,
     * and it would otherwise vanish without a trace on all four ports. The old
     * SERVICE_MODULE_BOTH = 10 is exactly that case. */
    template <std::size_t N>
    static bool knownId(const uint8_t (&ids)[N], uint8_t id) {
        for (uint8_t known : ids)
            if (known == id) return true;
        return false;
    }

    PosixTransport _io;
    Proto _proto{_io};

    std::atomic<bool> _running{true};
    std::atomic<bool> _linkUp{false};   // set by rxLoop: true once the port is open, false on disconnect
    std::atomic<unsigned> _reconnects{0}; // number of link-down events (each triggers a reconnect)
    std::thread _rxThread;

    rclcpp::Publisher<custom_msg::msg::MassPacket>::SharedPtr _massPub;
    rclcpp::Publisher<custom_msg::msg::PhPacket>::SharedPtr _phPub;
    rclcpp::Publisher<custom_msg::msg::Heartbeat>::SharedPtr _hbPub;

    rclcpp::Subscription<custom_msg::msg::ServoRequest>::SharedPtr _servoSub;
    rclcpp::Subscription<custom_msg::msg::MassRequest>::SharedPtr _massSub;
    rclcpp::Subscription<custom_msg::msg::LEDRequest>::SharedPtr _ledSub;
    rclcpp::Subscription<custom_msg::msg::PhRequest>::SharedPtr _phSub;

    /* link-health logging (no data spam: lifecycle, first-of-kind, periodic counts) */
    std::string _port;
    rclcpp::TimerBase::SharedPtr _statusTimer;
    std::atomic<unsigned> _rxBytes{0};
    std::atomic<unsigned> _rxFrames{0};
    std::atomic<unsigned> _txFrames{0};
    std::atomic<unsigned> _txDropped{0}; // commands dropped while the link was down
    unsigned _lastRxBytes{0};           // snapshot from the previous status tick (executor-thread only)

    /* calibration replay state. Indexed by SLOT in ALL_MASS_IDS, not by device
     * id: ids are a sparse, fleet-wide namespace, so _slopes[i] belongs to
     * ALL_MASS_IDS[i] and the two are only ever walked together. */
    double _slopes[MASS_ID_COUNT];           // from mass_cal.yaml; NaN = unconfigured, leave the firmware fallback
    double _phSlope{0.0};                   // from ph_cal.yaml; NaN = unconfigured
    double _phOffset{0.0};                  // both must be set for a replay to happen
    double _servoZeros[SERVO_ID_COUNT];     // from servo_cal.yaml; NaN = leave the firmware default
    rclcpp::TimerBase::SharedPtr _calTimer;
    std::atomic<bool> _linkReadyPending{false}; // set by rxLoop on link-up, cleared once the executor has sent
    std::atomic<unsigned> _framesAtOpen{0}; // _rxFrames snapshot at link-up: readiness baseline
};

