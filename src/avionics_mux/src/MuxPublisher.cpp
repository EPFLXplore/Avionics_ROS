#include "MuxPublisher.h"
#include "MuxManager.h"

#include "custom_msg/msg/mass_array.hpp"

#include "rclcpp/rclcpp.hpp"

MuxPublisher::MuxPublisher(CANBus* _bus, rclcpp::Node* _parent) : bus(_bus), parent(_parent) {    
    // Create a publisher based on the provided publish topic name
    this->mass_pub = parent->create_publisher<custom_msg::msg::MassArray>(("MASS_ARRAY"), 10);
    
    RCLCPP_INFO(parent->get_logger(), "Publisher created");

    bus->handle<MassPacket>(std::bind(&MuxPublisher::handleMassPacket, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(parent->get_logger(), "Handles created");

}

MuxPublisher::~MuxPublisher(){}

/*
template<typename MessageT>
void MuxPublisher<MessageT>::callback0(const typename MessageT::SharedPtr msg) {
    if (msg->id < dynamic_cast<MuxManager*>(parent)->get_max_number_nodes()) {
        bool bus0_state = dynamic_cast<MuxManager*>(parent)->get_bus0_state()[msg->id];
        bool bus1_state = dynamic_cast<MuxManager*>(parent)->get_bus1_state()[msg->id];
        // Normally NOBUS should never happen as we cannot receive data if there is no bus connected
        if (selected_bus(bus0_state, bus1_state) == 0 or selected_bus(bus0_state, bus1_state) == NOBUS)
            pub->publish(*msg);
    } else {
        RCLCPP_ERROR(parent->get_logger(), ("Invalid node ID for " + bus0 + topic_name + ": " + std::to_string(msg->id) + ". Not publishing...").c_str()); //!!!!!
         pub->publish(*msg);
    }
}

template<typename MessageT>
void MuxPublisher<MessageT>::callback1(const typename MessageT::SharedPtr msg) {
    if (msg->id < dynamic_cast<MuxManager*>(parent)->get_max_number_nodes()) {
        bool bus0_state = dynamic_cast<MuxManager*>(parent)->get_bus0_state()[msg->id];
        bool bus1_state = dynamic_cast<MuxManager*>(parent)->get_bus1_state()[msg->id];
        // Normally NOBUS should never happen as we cannot receive data if there is no bus connected
        if (selected_bus(bus0_state, bus1_state) == 1 or selected_bus(bus0_state, bus1_state) == NOBUS)
            pub->publish(*msg);
    } else {
        RCLCPP_ERROR(parent->get_logger(), ("Invalid node ID for " + bus1 + topic_name + ": " + std::to_string(msg->id) + ". Not publishing...").c_str());
        pub->publish(*msg);
    }
}

template<typename MessageT>
uint8_t MuxPublisher<MessageT>::selected_bus(bool node_state_bus0, bool node_state_bus1) {
    // We choose to prioritize bus0

    // Truth table:
    // node_state_bus0      node_state_bus1     selected_bus
    // 0                    0                   2 (no bus available)
    // 0                    1                   1
    // 1                    0                   0
    // 1                    1                   0
    if (node_state_bus0)
        return 0;
    else if (node_state_bus1)
        return 1;
    else
        return NOBUS;
}
*/

void MuxPublisher::handleMassPacket(uint8_t senderID, MassPacket* packet) {
    if(!(IS_RELIABLE(*packet))) {
		RCLCPP_ERROR(parent->get_logger(), "Unreliable mass packet");
		return;
	}
  
    auto msg = custom_msg::msg::MassArray();

    msg.id = packet->id;

    for (uint8_t i = 0; i < msg.mass.size(); ++i)
        msg.mass[i] = packet->mass[i];

    mass_pub->publish(msg);

    // if (packet->id == get_node_id("SC_DRILL_NODE_ID"))
    //     drill_mass_pub->publish(msg);
    // else if (packet->id == get_node_id("SC_CONTAINER_NODE_ID"))
    //     container_mass_pub->publish(msg);
    // else
    //     RCLCPP_INFO(parent->get_logger(), "Mass packet received but ID is not valid");
}


