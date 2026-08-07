/**
 * @file Nexus.cpp
 * @brief Implementation of the RP wire-owner (see Nexus.hpp).
 *
 * Logging policy: report everything that proves the link is alive (port open,
 * pubs/subs ready, the FIRST frame of each kind, the FIRST command forwarded,
 * write failures) plus a periodic count pulse, but never log per-packet data.
 * First-of-kind lines use *_ONCE; the link-down warning fires once per
 * disconnect (down-transition), and the 15s timer reports ongoing state.
 */

#include "Nexus.hpp"

#include <chrono>
#include <stdexcept>

using std::placeholders::_1;
using namespace std::chrono_literals;

Nexus::Nexus(int id) : rclcpp::Node("nexus", "ttyNova" + std::to_string(id)) {
    // Fixed by role: udev (99-nova.rules) always names master <id>'s port
    // /dev/ttyNova<id>, so there is nothing to parameterise.
    port_ = "/dev/ttyNova" + std::to_string(id);

    RCLCPP_INFO(get_logger(), "Nexus[%d] starting on %s (connects + auto-reconnects)", id, port_.c_str());

    /* Telemetry: MCU -> RP */
    mass_pub_ = create_publisher<custom_msg::msg::MassPacket>("/EL/mass_packet", 10);
    hb_pub_ = create_publisher<custom_msg::msg::Heartbeat>("/EL/heartbeat", 10);
    RCLCPP_INFO(get_logger(), "publishers ready: /EL/mass_packet, /EL/heartbeat");

    /* Commands: RP -> MCU */
    servo_sub_ = create_subscription<custom_msg::msg::ServoRequest>(
        "/EL/servo_req", 10, std::bind(&Nexus::onServoReq, this, _1));
    mass_sub_ = create_subscription<custom_msg::msg::MassRequest>(
        "/EL/mass_req", 10, std::bind(&Nexus::onMassReq, this, _1));
    led_sub_ = create_subscription<custom_msg::msg::LEDRequest>(
        "/EL/led_req", 10, std::bind(&Nexus::onLedReq, this, _1));
    RCLCPP_INFO(get_logger(), "subscriptions ready: /EL/servo_req, /EL/mass_req, /EL/led_req");

    /* Periodic link-health pulse (proves comms are up without spamming data). */
    status_timer_ = create_wall_timer(15s, [this]() {
        const unsigned rb = rx_bytes_.load();
        const unsigned rf = rx_frames_.load();
        const unsigned tf = tx_frames_.load();
        const unsigned delta = rb - last_rx_bytes_; // NEW bytes since the last tick, not the running total
        last_rx_bytes_ = rb;

        if (!link_up_.load())
            RCLCPP_WARN(get_logger(), "[%s] link DOWN: port not open, waiting for (re)connect... (%u commands dropped)",
                        port_.c_str(), tx_dropped_.load());
        else if (delta == 0)
            RCLCPP_WARN(get_logger(), "[%s] link stalled: port open but no new bytes in 15s (MCU unplugged/hung?)",
                        port_.c_str());
        else if (rf == 0)
            RCLCPP_WARN(get_logger(), "[%s] receiving bytes (+%u) but no valid frames yet, check framing/CRC",
                        port_.c_str(), delta);
        else
            RCLCPP_INFO(get_logger(),
                        "[%s] link up: +%u bytes/15s (rx %u frames, tx %u, crc_err %u, bad_len %u, reconnects %u)",
                        port_.c_str(), delta, rf, tf,
                        proto_.crcErrors(), proto_.badLen(), reconnects_.load());
    });

    rx_thread_ = std::thread(&Nexus::rxLoop, this);
    RCLCPP_INFO(get_logger(), "RX thread started, bridge up on %s", port_.c_str());
}

Nexus::~Nexus() {
    RCLCPP_INFO(get_logger(), "Nexus shutting down (%s)", port_.c_str());
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
}

/* ----------------------------- RX: serial -> ROS ------------------------- */

void Nexus::rxLoop() {
    uint8_t chunk[128];
    while (running_) {
        try {
            // (Re)connect: the port vanishes on unplug and reappears (same udev
            // symlink) on replug. open() throws until it's back.
            if (!io_.ok()) {
                io_.open(port_);
                link_up_ = true;
                RCLCPP_INFO(get_logger(), "[%s] serial port opened (USB-FS CDC)", port_.c_str());
            }
            uint16_t n = io_.read(chunk, sizeof chunk); // returns 0 when idle; throws on unplug
            if (n) {
                rx_bytes_ += n;
                RCLCPP_INFO_ONCE(get_logger(), "[%s] first bytes received from the MCU", port_.c_str());
                proto_.parse(chunk, n, [this](const Frame& f) { onFrame(f); });
            }
        } catch (const std::exception& e) {
            // Warn once on the down-transition; the 15s status timer covers the
            // ongoing DOWN state, so we don't repeat it every 500ms retry.
            if (link_up_.exchange(false)) {
                ++reconnects_;
                RCLCPP_WARN(get_logger(), "[%s] link down (%s), waiting for (re)connect...",
                            port_.c_str(), e.what());
            }
            io_.close();
            std::this_thread::sleep_for(500ms);
        }
    }
}

void Nexus::onFrame(const Frame& f) {
    ++rx_frames_;
    switch (f.id) {
        case MassPacket_ID: {
            if (!lengthOk<::MassPacket>(f, "MassPacket")) break;
            ::MassPacket packet = as<::MassPacket>(f);
            custom_msg::msg::MassPacket msg;
            msg.id = packet.id;
            msg.mass = packet.mass;
            mass_pub_->publish(msg);
            RCLCPP_INFO_ONCE(get_logger(), "[%s] first MassPacket (cell %u) received and published to /EL/mass_packet",
                             port_.c_str(), packet.id);
            break;
        }
        case Heartbeat_ID: {
            if (!lengthOk<::Heartbeat>(f, "Heartbeat")) break;
            ::Heartbeat packet = as<::Heartbeat>(f);
            custom_msg::msg::Heartbeat msg;
            msg.board_id = packet.board_id;
            hb_pub_->publish(msg);
            RCLCPP_INFO_ONCE(get_logger(), "[%s] first Heartbeat received and published to /EL/heartbeat",
                             port_.c_str());
            break;
        }
        default:
            RCLCPP_WARN(get_logger(), "[%s] unknown frame id %u (dropped)", port_.c_str(), f.id);
            break;
    }
}

/* ----------------------------- TX: ROS -> serial ------------------------- */

bool Nexus::txReady(const char* what) {
    if (link_up_.load()) return true;
    ++tx_dropped_;
    RCLCPP_DEBUG(get_logger(), "[%s] link down: %s dropped", port_.c_str(), what);
    return false;
}

void Nexus::onServoReq(const custom_msg::msg::ServoRequest::SharedPtr msg) {
    if (!txReady("ServoRequest")) return;
    ::ServoRequest packet{};
    packet.id = msg->id;
    packet.angle = msg->angle;
    packet.go_to_zero = msg->go_to_zero ? 1 : 0;
    if (proto_.send(ServoRequest_ID, &packet, sizeof packet)) {
        ++tx_frames_;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first ServoRequest forwarded to the MCU (id %u)",
                         port_.c_str(), packet.id);
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (ServoRequest)", port_.c_str());
    }
}

void Nexus::onMassReq(const custom_msg::msg::MassRequest::SharedPtr msg) {
    if (!txReady("MassRequest")) return;
    ::MassRequest packet{};
    packet.id = msg->id;
    packet.tare = msg->tare ? 1 : 0;
    packet.change_scale = msg->change_scale ? 1 : 0;
    packet.scale = msg->scale;
    if (proto_.send(MassRequest_ID, &packet, sizeof packet)) {
        ++tx_frames_;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first MassRequest forwarded to the MCU (cell %u)",
                         port_.c_str(), packet.id);
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (MassRequest)", port_.c_str());
    }
}

void Nexus::onLedReq(const custom_msg::msg::LEDRequest::SharedPtr msg) {
    if (!txReady("LEDRequest")) return;
    ::LEDRequest packet{};
    packet.system = msg->system;
    packet.mode = msg->mode;
    if (proto_.send(LEDRequest_ID, &packet, sizeof packet)) {
        ++tx_frames_;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first LEDRequest forwarded to the MCU", port_.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (LEDRequest)", port_.c_str());
    }
}
