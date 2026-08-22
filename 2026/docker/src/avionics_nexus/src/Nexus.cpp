/**
 * @file Nexus.cpp
 * @brief Implementation of the RP wire-owner (see Nexus.h).
 *
 * Logging policy: report everything that proves the link is alive (port open,
 * pubs/subs ready, the FIRST frame of each kind, the FIRST command forwarded,
 * write failures) plus a periodic count pulse, but never log per-packet data.
 * First-of-kind lines use *_ONCE; the link-down warning fires once per
 * disconnect (down-transition), and the 15s timer reports ongoing state.
 */

#include "Nexus.h"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>


using std::placeholders::_1;
using namespace std::chrono_literals;

Nexus::Nexus(int id) : rclcpp::Node("nexus", "ttyNova" + std::to_string(id)) {
    // Fixed by role: udev (99-nova.rules) always names master <id>'s port
    // /dev/ttyNova<id>, so there is nothing to parameterise.
    _port = "/dev/ttyNova" + std::to_string(id);

    RCLCPP_INFO(get_logger(), "Nexus[%d] starting on %s (connects + auto-reconnects)", id, _port.c_str());

    /* One QoS for every topic on this node.
     *
     * DEPTH 10 is what 2025 had - NexusSubscriber.cpp used the integer form,
     * `create_subscription<T>("/EL/servo_req", 10, cb)`, which rclcpp expands to
     * KeepLast(10). The 2026 port took it to KeepLast(1). All four Nexus nodes
     * share ONE SingleThreadedExecutor, so at depth 1 a second message arriving
     * before the executor drains the first is discarded - and the calibration
     * replay sends several commands back to back on every link-up, straight into
     * that window. History is not an RxO policy, so raising it is purely local
     * and cannot affect matching with anyone.
     *
     * RELIABILITY stays best_effort, and NOT because it is right for commands.
     * Telemetry is sampled state and best_effort suits it; a command is a
     * one-shot that nothing re-sends, and over WiFi to a base station a dropped
     * datagram with no retransmit looks exactly like "the command was never
     * sent". 2025 was RELIABLE here.
     *
     * It cannot be fixed from this side alone. Reliability IS an RxO policy,
     * offered >= requested, so a best_effort PUBLISHER will not match a reliable
     * SUBSCRIPTION - the topic goes dead rather than flaky. Both ends have to
     * move together. Do not "fix" this line on its own; check the publisher with
     * `ros2 topic info <topic> --verbose` first.
     *
     * DURABILITY stays volatile deliberately. transient_local would fix a
     * `ros2 topic pub --once` (lost because the publisher exits before discovery
     * completes - pass `-w 1` instead), but a transient_local SUBSCRIPTION also
     * refuses to match a volatile publisher, and a reconnecting Nexus would then
     * be handed the last command it never saw - which for a servo means moving
     * on subscribe. */
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    /* Telemetry: MCU -> RP */
    _massPub = create_publisher<custom_msg::msg::MassPacket>("/EL/mass_packet", qos);
    _phPub = create_publisher<custom_msg::msg::PhPacket>("/EL/ph_packet", qos);
    _hbPub = create_publisher<custom_msg::msg::Heartbeat>("/EL/heartbeat", qos);
    RCLCPP_INFO(get_logger(), "publishers ready: /EL/mass_packet, /EL/ph_packet, /EL/heartbeat");


    /* Commands: RP -> MCU
     *
     * Optionally sourced from the qos_bridge instead of the raw topics. The
     * bridge republishes each command on <topic>_latched with TRANSIENT_LOCAL,
     * so a Nexus that matches late - a restart, a redeploy, a reconnect - is
     * handed the last command instead of never seeing it.
     *
     *   ros2 run avionics_nexus avionics_nexus --ros-args -p command_suffix:=_latched
     *
     * A PARAMETER rather than a hard switch, for two reasons. Subscribing to both
     * the raw and the latched topic would deliver every command TWICE, and a
     * doubled tare is worse than a lost one. And hard-wiring the latched topic
     * would make the bridge a single point of failure for all command traffic -
     * if it is not running, nothing reaches the MCU at all. Default is empty, so
     * out of the box this behaves exactly as before and the bridge is opt-in.
     *
     * The QoS has to move with the topic: transient_local is RxO, so a
     * transient_local subscription only matches a transient_local publisher (the
     * bridge), and would refuse the volatile GUI. */
    const std::string cmdSuffix = declare_parameter<std::string>("command_suffix", "");
    const auto cmdQos = cmdSuffix.empty()
        ? qos
        : rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    if (!cmdSuffix.empty())
        RCLCPP_WARN(get_logger(),
                    "[%s] commands sourced from '*%s' (qos_bridge). A latched SERVO command "
                    "moves the servo on subscribe - check LATCH_SERVO in QosBridge.cpp",
                    _port.c_str(), cmdSuffix.c_str());

    _servoSub = create_subscription<custom_msg::msg::ServoRequest>(
        "/EL/servo_req" + cmdSuffix, cmdQos, std::bind(&Nexus::onServoReq, this, _1));
    _massSub = create_subscription<custom_msg::msg::MassRequest>(
        "/EL/mass_req" + cmdSuffix, cmdQos, std::bind(&Nexus::onMassReq, this, _1));
    _ledSub = create_subscription<custom_msg::msg::LEDRequest>(
        "/EL/led_req" + cmdSuffix, cmdQos, std::bind(&Nexus::onLedReq, this, _1));
    _phSub = create_subscription<custom_msg::msg::PhRequest>(
        "/EL/ph_req" + cmdSuffix, cmdQos, std::bind(&Nexus::onPhReq, this, _1));
    RCLCPP_INFO(get_logger(), "subscriptions ready: /EL/servo_req, /EL/mass_req, /EL/led_req, /EL/ph_req");

    /* Calibration: one slope per load cell DEVICE, replayed to the MCU on every
     * link-up. Keyed by global MassId from device_ids.h, not by connector index:
     * a slope belongs to a physical scale, so it follows that scale when it moves
     * to another connector or another master. Unset (NaN) means "no calibration
     * configured": we then send nothing and the MCU keeps the fallback compiled
     * into MassThread.h, which is strictly better than pushing a made-up number.
     *
     * Every node declares every id - see MASS_ID_COUNT in Nexus.h. The board
     * that does not carry a given cell simply drops the frame. */
    const double unset = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < MASS_ID_COUNT; ++i) {
        const MassParam& p = MASS_PARAMS[i];
        _slopes[i] = declare_parameter(p.param, unset);
        if (std::isfinite(_slopes[i]))
            RCLCPP_INFO(get_logger(), "%s (mass id %u) calibration slope %.10f", p.param, p.id, _slopes[i]);
        else
            RCLCPP_WARN(get_logger(), "%s (mass id %u) has no configured slope: the MCU will keep its "
                                      "firmware fallback", p.param, p.id);
    }

    /* pH calibration: two numbers for the one probe (ph = slope * volts + offset,
     * applied on the MCU). BOTH must be configured before we replay anything -
     * half a calibration is worse than none, because the firmware fallback at
     * least has the right shape. */
    _phSlope  = declare_parameter(PH_SLOPE_PARAM, unset);
    _phOffset = declare_parameter(PH_OFFSET_PARAM, unset);
    if (std::isfinite(_phSlope) && std::isfinite(_phOffset))
        RCLCPP_INFO(get_logger(), "pH calibration slope %.6f offset %.6f", _phSlope, _phOffset);
    else
        RCLCPP_WARN(get_logger(), "pH has no complete calibration (%s / %s): the MCU will keep its "
                                  "firmware fallback", PH_SLOPE_PARAM, PH_OFFSET_PARAM);

    /* Servo home positions: one angle per servo, replayed on link-up. Unset
     * (NaN) leaves SERVO_ZERO_PULSE_US compiled into the firmware, which is the
     * right default - a made-up home would drive real hardware somewhere. */
    for (std::size_t i = 0; i < SERVO_ID_COUNT; ++i) {
        const ServoParam& p = SERVO_PARAMS[i];
        _servoZeros[i] = declare_parameter(p.param, unset);
        if (std::isfinite(_servoZeros[i]))
            RCLCPP_INFO(get_logger(), "%s (servo id %u) home angle %.1f deg", p.param, p.id, _servoZeros[i]);
        else
            RCLCPP_WARN(get_logger(), "%s (servo id %u) has no configured home: the MCU will keep its "
                                      "firmware default", p.param, p.id);
    }

    /* Executor-side half of the replay. rxLoop only raises _linkReadyPending; the send
     * itself has to happen here because _proto.send() belongs to the executor
     * thread (see the thread-safety note in Nexus.h). */
    _calTimer = create_wall_timer(500ms, [this]() { onLinkReady(); });

    /* Periodic link-health pulse (proves comms are up without spamming data). */
    _statusTimer = create_wall_timer(15s, [this]() {
        const unsigned rb = _rxBytes.load();
        const unsigned rf = _rxFrames.load();
        const unsigned tf = _txFrames.load();
        const unsigned delta = rb - _lastRxBytes; // NEW bytes since the last tick, not the running total
        _lastRxBytes = rb;

        if (!_linkUp.load())
            RCLCPP_WARN(get_logger(), "[%s] link DOWN: port not open, waiting for (re)connect... (%u commands dropped)",
                        _port.c_str(), _txDropped.load());
        else if (delta == 0)
            RCLCPP_WARN(get_logger(), "[%s] link stalled: port open but no new bytes in 15s (MCU unplugged/hung?)",
                        _port.c_str());
        else if (rf == 0)
            RCLCPP_WARN(get_logger(), "[%s] receiving bytes (+%u) but no valid frames yet, check framing/CRC",
                        _port.c_str(), delta);
        else
            RCLCPP_INFO(get_logger(),
                        "[%s] link up: +%u bytes/15s (rx %u frames, tx %u, crc_err %u, bad_len %u, "
                        "truncated %u, reconnects %u)",
                        _port.c_str(), delta, rf, tf,
                        _proto.crcErrors(), _proto.badLen(), _proto.truncated(), _reconnects.load());
    });

    _rxThread = std::thread(&Nexus::rxLoop, this);
    RCLCPP_INFO(get_logger(), "RX thread started, bridge up on %s", _port.c_str());
}

Nexus::~Nexus() {
    RCLCPP_INFO(get_logger(), "Nexus shutting down (%s)", _port.c_str());
    _running = false;
    if (_rxThread.joinable()) _rxThread.join();
}

/* ----------------------------- RX: serial -> ROS ------------------------- */

void Nexus::rxLoop() {
    uint8_t chunk[128];
    auto last_rx = std::chrono::steady_clock::now();
    while (_running) {
        try {
            // (Re)connect: the port vanishes on unplug and reappears (same udev
            // symlink) on replug. open() throws until it's back.
            if (!_io.ok()) {
                _io.open(_port);
                _linkUp = true;
                last_rx = std::chrono::steady_clock::now(); // don't inherit the old link's silence
                // A new link means a possibly-reset MCU, i.e. slopes back at the
                // firmware fallback. Arm the replay; the executor does the send.
                _framesAtOpen = _rxFrames.load();
                _linkReadyPending = true;
                RCLCPP_INFO(get_logger(), "[%s] serial port opened (USB-FS CDC)", _port.c_str());
            }
            uint16_t n = _io.read(chunk, sizeof chunk); // parks up to 100ms; throws on unplug
            if (n) {
                _rxBytes += n;
                last_rx = std::chrono::steady_clock::now();
                RCLCPP_INFO_ONCE(get_logger(), "[%s] first bytes received from the MCU", _port.c_str());
                _proto.parse(chunk, n, [this](const Frame& f) { onFrame(f); });
            } else if (_proto.idle(), std::chrono::steady_clock::now() - last_rx > STALL_TIMEOUT) {
                // idle() first, unconditionally: a poll that returned nothing is
                // how the parser learns a half-received frame is never going to
                // finish. Without it the FSM stays parked mid-payload and eats
                // the next frame's header when the line comes back. Comma
                // operator so the stall test below is untouched.
                //
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
            if (_linkUp.exchange(false)) {
                ++_reconnects;
                RCLCPP_WARN(get_logger(), "[%s] link down (%s), waiting for (re)connect...",
                            _port.c_str(), e.what());
            }
            _io.close();
            std::this_thread::sleep_for(500ms);
        }
    }
}

void Nexus::onFrame(const Frame& f) {
    ++_rxFrames;
    switch (f.id) {
        case MassPacket_ID: {
            if (!lengthOk<::MassPacket>(f, "MassPacket")) break;
            ::MassPacket packet = as<::MassPacket>(f);
            custom_msg::msg::MassPacket msg;
            msg.id = packet.id;
            msg.mass = packet.mass;
            _massPub->publish(msg);
            RCLCPP_INFO_ONCE(get_logger(), "[%s] first MassPacket (mass id %u) received and published to /EL/mass_packet",
                             _port.c_str(), packet.id);
            break;
        }
        case PhPacket_ID: {
            if (!lengthOk<::PhPacket>(f, "PhPacket")) break;
            ::PhPacket packet = as<::PhPacket>(f);
            custom_msg::msg::PhPacket msg;
            msg.ph = packet.ph;
            _phPub->publish(msg);
            RCLCPP_INFO_ONCE(get_logger(), "[%s] first PhPacket received and published to /EL/ph_packet",
                             _port.c_str());
            break;
        }

        case Heartbeat_ID: {
            if (!lengthOk<::Heartbeat>(f, "Heartbeat")) break;
            ::Heartbeat packet = as<::Heartbeat>(f);
            custom_msg::msg::Heartbeat msg;
            msg.board_id = packet.board_id;
            _hbPub->publish(msg);
            RCLCPP_INFO_ONCE(get_logger(), "[%s] first Heartbeat received and published to /EL/heartbeat",
                             _port.c_str());
            break;
        }
        default:
            RCLCPP_WARN(get_logger(), "[%s] unknown frame id %u (dropped)", _port.c_str(), f.id);
            break;
    }
}

/* ----------------------------- TX: ROS -> serial ------------------------- */

bool Nexus::txReady(const char* what) {
    if (_linkUp.load()) return true;
    ++_txDropped;
    RCLCPP_DEBUG(get_logger(), "[%s] link down: %s dropped", _port.c_str(), what);
    return false;
}

void Nexus::onServoReq(const custom_msg::msg::ServoRequest::SharedPtr msg) {
    if (!txReady("ServoRequest")) return;
    if (!knownId(ALL_SERVO_IDS, msg->id))
        RCLCPP_WARN_ONCE(get_logger(),
                         "[%s] ServoRequest id %u is not a ServoId in device_ids.h: forwarded, but no "
                         "board can match it, so every master will drop it",
                         _port.c_str(), msg->id);
    ::ServoRequest packet{};
    packet.id = msg->id;
    packet.angle = msg->angle;
    packet.go_to_zero  = msg->go_to_zero ? 1 : 0;
    packet.change_zero = msg->change_zero ? 1 : 0;
    packet.zero        = msg->zero;
    if (_proto.send(ServoRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first ServoRequest forwarded to the MCU (id %u)",
                         _port.c_str(), packet.id);
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (ServoRequest)", _port.c_str());
    }
}

void Nexus::onMassReq(const custom_msg::msg::MassRequest::SharedPtr msg) {
    if (!txReady("MassRequest")) return;
    if (!knownId(ALL_MASS_IDS, msg->id))
        RCLCPP_WARN_ONCE(get_logger(),
                         "[%s] MassRequest id %u is not a MassId in device_ids.h: forwarded, but no "
                         "board can match it, so every master will drop it",
                         _port.c_str(), msg->id);
    ::MassRequest packet{};
    packet.id = msg->id;
    packet.tare = msg->tare ? 1 : 0;
    packet.change_scale = msg->change_scale ? 1 : 0;
    packet.scale = msg->scale;
    if (_proto.send(MassRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first MassRequest forwarded to the MCU (mass id %u)",
                         _port.c_str(), packet.id);
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (MassRequest)", _port.c_str());
    }
}

void Nexus::onPhReq(const custom_msg::msg::PhRequest::SharedPtr msg) {
    if (!txReady("PhRequest")) return;
    ::PhRequest packet{};
    packet.change_cal = msg->change_cal ? 1 : 0;
    packet.slope      = msg->slope;
    packet.offset     = msg->offset;
    if (_proto.send(PhRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first PhRequest forwarded to the MCU", _port.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (PhRequest)", _port.c_str());
    }
}

/* --------------------------- calibration replay -------------------------- */

void Nexus::onLinkReady() {
    if (!_linkReadyPending.load() || !_linkUp.load()) return;

    // Wait for proof the MCU is actually running, not just that the port opened:
    // USB enumerates while the firmware is still early in boot, and a MassRequest
    // sent then races MassThread::init(), whose tare + 600ms settle come after.
    // One received frame is the cheapest evidence the far side is alive.
    if (_rxFrames.load() == _framesAtOpen.load()) return;

    _linkReadyPending = false;
    // A reconnect means the MCU rebooted and dropped its LED state: forget what
    // we last sent so the next request is forwarded even if it repeats it.
    _lastLedValid = false;
    replayCalibration();
    replayPhCalibration();
    replayServoZeros();
    sendAvionicsLedOn();
}

void Nexus::replayCalibration() {
    // Broadcast every configured slope to this master. We do not filter by board:
    // Nexus has no board profile, and the MCU already drops ids it does not carry.
    for (std::size_t i = 0; i < MASS_ID_COUNT; ++i) {
        if (!std::isfinite(_slopes[i])) continue; // unconfigured: firmware fallback stands
        sendMassScale(MASS_PARAMS[i].id, static_cast<float>(_slopes[i]));
    }
}

void Nexus::replayPhCalibration() {
    // Same broadcast rule as the mass slopes: send it to every master and let the
    // board profile decide what sticks. A board with no probe drops it.
    if (!std::isfinite(_phSlope) || !std::isfinite(_phOffset)) return;
    sendPhCal(static_cast<float>(_phSlope), static_cast<float>(_phOffset));
}

void Nexus::sendPhCal(float slope, float offset) {
    ::PhRequest packet{};
    packet.change_cal = 1;
    packet.slope      = slope;
    packet.offset     = offset;
    if (_proto.send(PhRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        RCLCPP_INFO(get_logger(), "[%s] pH calibration replayed to the MCU: slope %.6f offset %.6f",
                    _port.c_str(), static_cast<double>(slope), static_cast<double>(offset));
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (pH calibration replay)", _port.c_str());
    }
}

void Nexus::replayServoZeros() {
    // Same broadcast rule as every other calibration: send each configured home
    // to every master and let the board profile decide what sticks.
    for (std::size_t i = 0; i < SERVO_ID_COUNT; ++i) {
        if (!std::isfinite(_servoZeros[i])) continue;
        sendServoZero(SERVO_PARAMS[i].id, static_cast<int32_t>(_servoZeros[i]));
    }
}

void Nexus::sendServoZero(uint8_t id, int32_t angle) {
    if (!knownId(ALL_SERVO_IDS, id)) {
        RCLCPP_ERROR(get_logger(), "[%s] refusing to set a home angle under id %u: not a ServoIdType "
                                   "in device_ids.h", _port.c_str(), id);
        return;
    }
    ::ServoRequest packet{};
    packet.id          = id;
    packet.change_zero = 1;
    packet.zero        = angle;
    if (_proto.send(ServoRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        RCLCPP_INFO(get_logger(), "[%s] home angle replayed to the MCU: servo id %u -> %ld deg",
                    _port.c_str(), id, static_cast<long>(angle));
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (servo home, id %u)", _port.c_str(), id);
    }
}

void Nexus::sendAvionicsLedOn() {
    ::LEDRequest packet{};
    packet.system = idOf(LedSystemType::Avionics);
    packet.mode   = idOf(LedModeType::On);
    if (_proto.send(LEDRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        _lastLedValid  = true;
        _lastLedSystem = packet.system;
        _lastLedMode   = packet.mode;
        RCLCPP_INFO(get_logger(), "[%s] avionics LED set ON (system %u, mode %u)",
                    _port.c_str(), packet.system, packet.mode);
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (avionics LED on)", _port.c_str());
    }
}

void Nexus::sendMassScale(uint8_t id, float scale) {
    /* Last line of defence before a slope hits the wire. The static_asserts in
     * Nexus.h already pin name<->id, so reaching this means someone called us
     * with an id that came from somewhere else. A calibration sent under a wrong
     * id would be applied by whichever cell does answer to it, silently
     * corrupting a good scale - so refuse rather than forward. */
    if (!knownId(ALL_MASS_IDS, id)) {
        RCLCPP_ERROR(get_logger(), "[%s] refusing to replay calibration under id %u: not a MassId in "
                                   "device_ids.h", _port.c_str(), id);
        return;
    }
    ::MassRequest packet{};
    packet.id = id;
    packet.tare = 0;
    packet.change_scale = 1;
    packet.scale = scale;
    if (_proto.send(MassRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        RCLCPP_INFO(get_logger(), "[%s] calibration replayed to the MCU: mass id %u slope %.10f",
                    _port.c_str(), id, static_cast<double>(scale));
    } else {
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (calibration replay, mass id %u): "
                                  "the MCU is running its firmware fallback slope",
                    _port.c_str(), id);
    }
}

void Nexus::onLedReq(const custom_msg::msg::LEDRequest::SharedPtr msg) {
    if (!txReady("LEDRequest")) return;

    /* Edge-triggered: the LED state lives in the MCU, so re-sending what it
     * already holds only burns link bandwidth that the sensor stream needs. */
    if (_lastLedValid && msg->system == _lastLedSystem && msg->mode == _lastLedMode) {
        RCLCPP_DEBUG(get_logger(), "[%s] LEDRequest unchanged (system %u, mode %u): not forwarded",
                     _port.c_str(), msg->system, msg->mode);
        return;
    }

    ::LEDRequest packet{};
    packet.system = msg->system;
    packet.mode = msg->mode;
    if (_proto.send(LEDRequest_ID, &packet, sizeof packet)) {
        ++_txFrames;
        _lastLedValid  = true;
        _lastLedSystem = packet.system;
        _lastLedMode   = packet.mode;
        RCLCPP_INFO_ONCE(get_logger(), "[%s] first LEDRequest forwarded to the MCU", _port.c_str());
    } else {
        /* Cache stays as-is on a failed write: nothing reached the MCU, so the
         * next identical request must still be allowed through. */
        RCLCPP_WARN(get_logger(), "[%s] serial write failed (LEDRequest)", _port.c_str());
    }
}

