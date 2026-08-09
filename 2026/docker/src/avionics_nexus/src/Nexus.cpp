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
#include <limits>
#include <stdexcept>
#include <string>


using std::placeholders::_1;
using namespace std::chrono_literals;

Nexus::Nexus(int id) : rclcpp::Node("nexus", "ttyNova" + std::to_string(id)) {
    // Fixed by role: udev (99-nova.rules) always names master <id>'s port
    // /dev/ttyNova<id>, so there is nothing to parameterise.
    port_ = "/dev/ttyNova" + std::to_string(id);

    RCLCPP_INFO(get_logger(), "Nexus[%d] starting on %s (connects + auto-reconnects)", id, port_.c_str());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    /* Telemetry: MCU -> RP */
    mass_pub_ = create_publisher<custom_msg::msg::MassPacket>("/EL/mass_packet", qos);
    hb_pub_ = create_publisher<custom_msg::msg::Heartbeat>("/EL/heartbeat", qos);
    RCLCPP_INFO(get_logger(), "publishers ready: /EL/mass_packet, /EL/heartbeat");


    /* Commands: RP -> MCU */
    servo_sub_ = create_subscription<custom_msg::msg::ServoRequest>(
        "/EL/servo_req", qos, std::bind(&Nexus::onServoReq, this, _1));
    mass_sub_ = create_subscription<custom_msg::msg::MassRequest>(
        "/EL/mass_req", qos, std::bind(&Nexus::onMassReq, this, _1));
    led_sub_ = create_subscription<custom_msg::msg::LEDRequest>(
        "/EL/led_req", qos, std::bind(&Nexus::onLedReq, this, _1));
    RCLCPP_INFO(get_logger(), "subscriptions ready: /EL/servo_req, /EL/mass_req, /EL/led_req");

    /* Calibration: one slope per load cell DEVICE, replayed to the MCU on every
     * link-up. Keyed by global MassId from device_ids.h, not by connector index:
     * a slope belongs to a physical scale, so it follows that scale when it moves
     * to another connector or another master. Unset (NaN) means "no calibration
     * configured": we then send nothing and the MCU keeps the fallback compiled
     * into MassThread.h, which is strictly better than pushing a made-up number.
     *
     * Every node declares every id - see MASS_ID_COUNT in Nexus.hpp. The board
     * that does not carry a given cell simply drops the frame. */
    const double unset = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < MASS_ID_COUNT; ++i) {
        const MassParam& p = MASS_PARAMS[i];
        slopes_[i] = declare_parameter(p.param, unset);
        if (std::isfinite(slopes_[i]))
            RCLCPP_INFO(get_logger(), "%s (mass id %u) calibration slope %.10f", p.param, p.id, slopes_[i]);
        else
            RCLCPP_WARN(get_logger(), "%s (mass id %u) has no configured slope: the MCU will keep its "
                                      "firmware fallback", p.param, p.id);
    }

    /* Executor-side half of the replay. rxLoop only raises link_ready_pending_; the send
     * itself has to happen here because proto_.send() belongs to the executor
     * thread (see the thread-safety note in Nexus.hpp). */
    cal_timer_ = create_wall_timer(500ms, [this]() { onLinkReady(); });

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
    auto last_rx = std::chrono::steady_clock::now();
    while (running_) {
        try {
            // (Re)connect: the port vanishes on unplug and reappears (same udev
            // symlink) on replug. open() throws until it's back.
            if (!io_.ok()) {
                io_.open(port_);
                link_up_ = true;
                last_rx = std::chrono::steady_clock::now(); // don't inherit the old link's silence
                // A new link means a possibly-reset MCU, i.e. slopes back at the
                // firmware fallback. Arm the replay; the executor does the send.
                frames_at_open_ = rx_frames_.load();
                link_ready_pending_ = true;
                RCLCPP_INFO(get_logger(), "[%s] serial port opened (USB-FS CDC)", port_.c_str());
            }
            uint16_t n = io_.read(chunk, sizeof chunk); // parks up to 100ms; throws on unplug
            if (n) {
                rx_bytes_ += n;
                last_rx = std::chrono::steady_clock::now();
                RCLCPP_INFO_ONCE(get_logger(), "[%s] first bytes received from the MCU", port_.c_str());
                proto_.parse(chunk, n, [this](const Frame& f) { onFrame(f); });
            } else if (std::chrono::steady_clock::now() - last_rx > STALL_TIMEOUT) {
                // Self-heal. The fd is still valid and read() never errored, but
                // the MCU stopped talking: a hung TX ring on the firmware side,
                // a USB suspend, or a re-enumeration onto a different ttyACM
                // minor while we held the old one. None of those ever make
                // read() throw, so without this the link stays dead forever and
                // even a replug does not recover it. Throwing hands over to the
                // catch below, which is the same proven close/reopen path used
                // for a real unplug.
                throw std::runtime_error("no bytes for " +
                                         std::to_string(STALL_TIMEOUT.count()) + "s (MCU silent)");
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
            RCLCPP_INFO_ONCE(get_logger(), "[%s] first MassPacket (mass id %u) received and published to /EL/mass_packet",
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
    if (!knownId(ALL_SERVO_IDS, msg->id))
        RCLCPP_WARN_ONCE(get_logger(),
                         "[%s] ServoRequest id %u is not a ServoId in device_ids.h: forwarded, but no "
                         "board can match it, so every master will drop it",
                         port_.c_str(), msg->id);
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
    if (!knownId(ALL_MASS_IDS, msg->id))
        RCLCPP_WARN_ONCE(get_logger(),
                         "[%s] MassRequest id %u is not a MassId in device_ids.h: forwarded, but no "
                         "board can match it, so every master will drop it",
                         port_.c_str(), msg->id);
    ::MassRequest packet{};
    packet.id = msg->id;
    packet.tare = msg->tare ? 1 : 0;
    packet.change_scale = msg->change_scale ? 1 : 0;
    packet.scale = msg->scale;
    if (proto_.send(MassRequest_ID, &packet, sizeof packet)) {
        ++tx_frames_;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first MassRequest forwarded to the MCU (mass id %u)",
                         port_.c_str(), packet.id);
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (MassRequest)", port_.c_str());
    }
}

/* --------------------------- calibration replay -------------------------- */

void Nexus::onLinkReady() {
    if (!link_ready_pending_.load() || !link_up_.load()) return;

    // Wait for proof the MCU is actually running, not just that the port opened:
    // USB enumerates while the firmware is still early in boot, and a MassRequest
    // sent then races MassThread::init(), whose tare + 600ms settle come after.
    // One received frame is the cheapest evidence the far side is alive.
    if (rx_frames_.load() == frames_at_open_.load()) return;

    link_ready_pending_ = false;
    replayCalibration();
    sendAvionicsLedOn();
}

void Nexus::replayCalibration() {
    // Broadcast every configured slope to this master. We do not filter by board:
    // Nexus has no board profile, and the MCU already drops ids it does not carry.
    for (std::size_t i = 0; i < MASS_ID_COUNT; ++i) {
        if (!std::isfinite(slopes_[i])) continue; // unconfigured: firmware fallback stands
        sendMassScale(MASS_PARAMS[i].id, static_cast<float>(slopes_[i]));
    }
}

void Nexus::sendAvionicsLedOn() {
    ::LEDRequest packet{};
    packet.system = LED_SYSTEM_AVIONICS;
    packet.mode   = LED_MODE_ON;
    if (proto_.send(LEDRequest_ID, &packet, sizeof packet)) {
        ++tx_frames_;
        RCLCPP_INFO(get_logger(), "[%s] avionics LED set ON (system %u, mode %u)",
                    port_.c_str(), packet.system, packet.mode);
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (avionics LED on)", port_.c_str());
    }
}

void Nexus::sendMassScale(uint8_t id, float scale) {
    /* Last line of defence before a slope hits the wire. The static_asserts in
     * Nexus.hpp already pin name<->id, so reaching this means someone called us
     * with an id that came from somewhere else. A calibration sent under a wrong
     * id would be applied by whichever cell does answer to it, silently
     * corrupting a good scale - so refuse rather than forward. */
    if (!knownId(ALL_MASS_IDS, id)) {
        RCLCPP_ERROR(get_logger(), "[%s] refusing to replay calibration under id %u: not a MassId in "
                                   "device_ids.h", port_.c_str(), id);
        return;
    }
    ::MassRequest packet{};
    packet.id = id;
    packet.tare = 0;
    packet.change_scale = 1;
    packet.scale = scale;
    if (proto_.send(MassRequest_ID, &packet, sizeof packet)) {
        ++tx_frames_;
        RCLCPP_INFO(get_logger(), "[%s] calibration replayed to the MCU: mass id %u slope %.10f",
                    port_.c_str(), id, static_cast<double>(scale));
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (calibration replay, mass id %u): "
                                  "the MCU is running its firmware fallback slope",
                    port_.c_str(), id);
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

