/**
 * @file QosBridge.cpp
 * @brief Latches the last command on each topic so a late-joining Nexus gets it.
 *
 * WHAT THIS FIXES
 *
 * Nexus's command subscriptions are VOLATILE: a message published while Nexus
 * was not matched is gone. Nexus is matched-late more often than it looks - it
 * restarts, it is redeployed, and its RX thread tears down and reopens the port
 * on every unplug. Anything sent in those windows is silently lost, and a
 * command is a one-shot that nothing re-sends.
 *
 * This node subscribes with QoS that matches ANY publisher, and republishes on
 * <topic>_latched with TRANSIENT_LOCAL, which retains the last sample and hands
 * it to a subscriber that matches later. Nexus subscribes to the _latched topic.
 *
 * WHAT THIS DOES NOT FIX
 *
 * `ros2 topic pub --once`. That message is lost because the PUBLISHER is created,
 * publishes, and exits before discovery completes - at the moment it publishes it
 * has zero matched subscribers. This node is a subscriber with exactly the same
 * exposure: if Nexus misses the message, so does this, and it never has anything
 * to latch. Pass `-w 1` to make the publisher wait for a match before sending.
 * No amount of subscriber-side QoS can recover a sample that was never sent to
 * anyone.
 *
 * THE HAZARD, read before deploying
 *
 * Transient-local means a NEW subscriber is handed the last command ON MATCH.
 * ALL FOUR command topics are bridged, servo included, so restarting Nexus - or
 * any reconnect that re-matches it - drives every servo to its last commanded
 * angle with no operator asking. THE ROVER CAN MOVE ON A NODE RESTART.
 *
 * That is not a side effect, it is the mechanism: the same replay that rescues a
 * lost tare rescues a stale servo angle. It is a deliberate choice - a command
 * silently lost is the failure this exists to stop - but it means a node restart
 * is no longer a safe thing to do next to the hardware. Treat it like a power-on.
 *
 * To take servo back out, drop the servo line in the constructor; the other three
 * are idempotent config or state and replay harmlessly.
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "custom_msg/msg/led_request.hpp"
#include "custom_msg/msg/mass_request.hpp"
#include "custom_msg/msg/ph_request.hpp"
#include "custom_msg/msg/servo_request.hpp"

namespace {

/* Input QoS: deliberately the WEAKEST settings, so this matches every publisher.
 * Reliability is RxO (offered >= requested), so a best_effort publisher would
 * refuse a reliable subscription - requesting best_effort here matches both.
 * Depth 10 because all callbacks share one executor and commands arrive in
 * bursts (the calibration replay sends several back to back). */
rclcpp::QoS inQos() { return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); }

/* Output QoS: transient_local is the whole reason this node exists. Reliable
 * because durability without it buys little - and both ends here are ours, on
 * one machine, so there is no publisher we might fail to match. */
rclcpp::QoS outQos() { return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(); }

} // namespace

class QosBridge : public rclcpp::Node {
  public:
    QosBridge() : rclcpp::Node("qos_bridge") {
        bridge<custom_msg::msg::ServoRequest>("/EL/servo_req", _servoPub, _servoSub);
        bridge<custom_msg::msg::MassRequest>("/EL/mass_req", _massPub, _massSub);
        bridge<custom_msg::msg::LEDRequest>("/EL/led_req", _ledPub, _ledSub);
        bridge<custom_msg::msg::PhRequest>("/EL/ph_req", _phPub, _phSub);

        RCLCPP_INFO(get_logger(), "qos_bridge up: <topic> -> <topic>_latched (transient_local)");
        RCLCPP_WARN(get_logger(),
                    "servo commands are latched: a subscriber (re)joining is handed the last "
                    "commanded angle and the servo WILL move. Restarting a node is not a "
                    "no-op next to the hardware.");
    }

  private:
    /* One subscription and one publisher per topic. The callback is a plain
     * forward: no filtering, no rewriting. Anything this node decided for itself
     * would be a second place to look when a command goes missing. */
    template <class MsgT, class PubT, class SubT>
    void bridge(const std::string& topic, PubT& pub, SubT& sub) {
        const std::string out = topic + "_latched";
        pub = create_publisher<MsgT>(out, outQos());
        sub = create_subscription<MsgT>(
            topic, inQos(),
            [this, pub_ = pub, topic, out](typename MsgT::SharedPtr msg) {
                pub_->publish(*msg);
                RCLCPP_DEBUG(get_logger(), "%s -> %s", topic.c_str(), out.c_str());
            });
    }

    rclcpp::Publisher<custom_msg::msg::ServoRequest>::SharedPtr _servoPub;
    rclcpp::Publisher<custom_msg::msg::MassRequest>::SharedPtr  _massPub;
    rclcpp::Publisher<custom_msg::msg::LEDRequest>::SharedPtr   _ledPub;
    rclcpp::Publisher<custom_msg::msg::PhRequest>::SharedPtr    _phPub;

    rclcpp::Subscription<custom_msg::msg::ServoRequest>::SharedPtr _servoSub;
    rclcpp::Subscription<custom_msg::msg::MassRequest>::SharedPtr  _massSub;
    rclcpp::Subscription<custom_msg::msg::LEDRequest>::SharedPtr   _ledSub;
    rclcpp::Subscription<custom_msg::msg::PhRequest>::SharedPtr    _phSub;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QosBridge>());
    rclcpp::shutdown();
    return 0;
}
