#ifndef MUX_PUBLISHER_H
#define MUX_PUBLISHER_H

// #include "rclcpp/rclcpp.hpp"
// #include <string>

// #include "custom_msg/msg/mass_array.hpp"

// #include "BRoCo/CANBus.h"
// #include "Protocol/Protocol.h"

// class MuxPublisher {
// public:
//     //Topic name follow this template:
//     // topic_name = get_prefix() + get_param<std::string>("**INSERT TOPIC NAME**")
//     MuxPublisher(CANBus* _bus, rclcpp::Node* _parent);
//         // const std::string& topic_name);
//     ~MuxPublisher();
//     // void callback0(const typename MessageT::SharedPtr msg);
//     // void callback1(const typename MessageT::SharedPtr msg);

// private:
//     rclcpp::Node* parent;
//     CANBus* bus;

//     void initCallbacks();

//     rclcpp::Publisher<custom_msg::msg::MassArray>::SharedPtr mass_pub;    
//     void handleMassPacket(uint8_t senderID, MassPacket* packet);
// };

#endif /* MUX_PUBLISHER_H */