/**
 * @file Nexus.hpp
 * @brief The RP wire-owner: mirror of the MCU's SerialThread.
 *
 * Collapses the old NexusMain + NexusPublisher + NexusSubscriber (+ the global
 * publisher pointers and the raw std::thread) into a single rclcpp::Node that
 * owns the transport and the protocol. One class, one switch(id) on RX, one
 * subscription callback per command on TX. Only the far side differs from the
 * MCU: ROS topics here, FreeRTOS queues there.
 *
 * Thread-safety: the RX thread is the only caller of proto_.parse()/io_.read();
 * the executor (single-threaded, see NexusMain) is the only caller of
 * proto_.send()/io_.write(). One owner per direction, no shared mutable state
 * between them, so no locks are needed.
 */

#ifndef NEXUS_HPP
#define NEXUS_HPP

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
#include "custom_msg/msg/servo_request.hpp"

#include "SerialProtocol.hpp"
#include "Transport.hpp"
#include "device_ids.h"
#include "packets.h"

/* Every load cell in the FLEET, with the mass_cal.yaml key that carries its
 * slope - not the cells behind any one master. Nexus deliberately does not know
 * which master serves which device: commands go out to every port and the
 * firmware drops the ids its board profile does not carry (deviceFor() ->
 * nullptr).
 *
 * The parameter is named after the DEVICE, not its id number, so a slope is
 * readable in the yaml and survives any renumbering of MassId. The names live
 * here rather than in device_ids.h because they are a ROS-config concern - the
 * firmware has no use for strings - but the name<->id pairing is exactly what
 * must not drift, so it is checked below rather than trusted.
 *
 * At namespace scope, not inside Nexus: a constexpr member cannot be called by a
 * static_assert in its own class body (the class is still incomplete there). */
struct MassParam {
    uint8_t     id;    // global MassId from device_ids.h
    const char* param; // ros parameter key in mass_cal.yaml
};

constexpr MassParam MASS_PARAMS[] = {
    { id_of(MassId::SandRocks), "mass_slope_sand_rocks" },
    { id_of(MassId::Drill),     "mass_slope_drill"      },
};
constexpr std::size_t MASS_ID_COUNT = sizeof(MASS_PARAMS) / sizeof(MASS_PARAMS[0]);

/* Every declared load cell has a slope parameter. Add a MassId to device_ids.h
 * without adding its key here and the build fails, instead of that cell silently
 * running the firmware fallback forever. */
static_assert(MASS_ID_COUNT == sizeof(ALL_MASS_IDS) / sizeof(ALL_MASS_IDS[0]),
              "MASS_PARAMS and MassId disagree: every load cell needs a slope parameter");

/* ...and in the same order, so MASS_PARAMS[i].id == ALL_MASS_IDS[i] and
 * slopes_[i] means the same cell whichever table you walk. This is the check
 * that stops a slope being replayed against the wrong scale: swap two lines in
 * MASS_PARAMS and it will not compile. */
constexpr bool massParamsMatchIds() {
    for (std::size_t i = 0; i < MASS_ID_COUNT; ++i)
        if (MASS_PARAMS[i].id != ALL_MASS_IDS[i]) return false;
    return true;
}
static_assert(massParamsMatchIds(),
              "MASS_PARAMS is not in MassId order: a slope would be sent to the wrong cell");

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
    void sendMassScale(uint8_t id, float scale);

    /* Announce that avionics is alive on the LED strip. The strip lives on one
     * master, but this goes out on every port for the same reason commands do:
     * Nexus has no board profile, and a master with no strip just drops it
     * (LedsThread is never started, so nothing consumes the queue). Re-sent on
     * every link-up because an MCU reset takes the strip back to its boot state. */
    void sendAvionicsLedOn();

    /* From LEDRequest.msg: systems NAV=0, HD=1, DRILL=2, AVIONICS=3;
     * patterns OFF=0, ON=1, BLINK=2, FAULT=3. Mirrored by the switch in
     * LEDStrip::handleMode() on the firmware side. */
    static constexpr uint8_t LED_SYSTEM_AVIONICS = 3;
    static constexpr uint8_t LED_MODE_ON         = 1;

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
                    port_.c_str(), what, static_cast<unsigned>(f.length), sizeof(T));
        return false;
    }

    /* Silence budget before the RX loop tears the port down and reopens it.
     * The MCU heartbeats at ~10 Hz, so 3 s of nothing is unambiguously dead,
     * while still being far longer than any legitimate gap. */
    static constexpr std::chrono::seconds STALL_TIMEOUT{3};

    /* The id contract this node was written against. The two repos pin the shared
     * submodule independently, so they can sit at different commits - and a
     * drifted enum is the same size as a correct one, so it passes SerialProtocol's
     * CRC and lengthOk() alike and surfaces only as a command reaching the wrong
     * device. Bumping DEVICE_CONTRACT_VERSION must therefore break this build,
     * so that mass_cal.yaml's keys and anything else keyed by id get re-checked. */
    static_assert(DEVICE_CONTRACT_VERSION == 1,
                  "device id contract changed: re-check mass_cal.yaml keys and any id-keyed config");

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

    PosixTransport io_;
    Proto proto_{io_};

    std::atomic<bool> running_{true};
    std::atomic<bool> link_up_{false};   // set by rxLoop: true once the port is open, false on disconnect
    std::atomic<unsigned> reconnects_{0}; // number of link-down events (each triggers a reconnect)
    std::thread rx_thread_;

    rclcpp::Publisher<custom_msg::msg::MassPacket>::SharedPtr mass_pub_;
    rclcpp::Publisher<custom_msg::msg::Heartbeat>::SharedPtr hb_pub_;

    rclcpp::Subscription<custom_msg::msg::ServoRequest>::SharedPtr servo_sub_;
    rclcpp::Subscription<custom_msg::msg::MassRequest>::SharedPtr mass_sub_;
    rclcpp::Subscription<custom_msg::msg::LEDRequest>::SharedPtr led_sub_;

    /* link-health logging (no data spam: lifecycle, first-of-kind, periodic counts) */
    std::string port_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::atomic<unsigned> rx_bytes_{0};
    std::atomic<unsigned> rx_frames_{0};
    std::atomic<unsigned> tx_frames_{0};
    std::atomic<unsigned> tx_dropped_{0}; // commands dropped while the link was down
    unsigned last_rx_bytes_{0};           // snapshot from the previous status tick (executor-thread only)

    /* calibration replay state. Indexed by SLOT in ALL_MASS_IDS, not by device
     * id: ids are a sparse, fleet-wide namespace, so slopes_[i] belongs to
     * ALL_MASS_IDS[i] and the two are only ever walked together. */
    double slopes_[MASS_ID_COUNT];           // from mass_cal.yaml; NaN = unconfigured, leave the firmware fallback
    rclcpp::TimerBase::SharedPtr cal_timer_;
    std::atomic<bool> link_ready_pending_{false}; // set by rxLoop on link-up, cleared once the executor has sent
    std::atomic<unsigned> frames_at_open_{0}; // rx_frames_ snapshot at link-up: readiness baseline
};

#endif // NEXUS_HPP
