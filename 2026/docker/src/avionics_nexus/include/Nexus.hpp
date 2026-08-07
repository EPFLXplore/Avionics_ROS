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
#include "packets.h"

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
    static constexpr std::chrono::seconds kStallTimeout{3};

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
};

#endif // NEXUS_HPP
