// /*
//  * MuxManager.cpp
//  *
//  *      Author: Matas Jones
//  */

// #include "rclcpp/rclcpp.hpp"

// #include "MuxManager.h"

// #define NB_BUS 2

// MuxManager::MuxManager() : Node("avionics_mux") {
    
//     this->declare_parameter("ATTEMPT_RETRY", true);
//     this->declare_parameter("CONNECTION_RETRY_INTERVAL", 1);

//     // For Can0 and Can1 create the driver, connect to the Can, create a pub and sub instance
//     for (int bus_id = 0; bus_id < NB_BUS; bus_id++){

//         // Attempt to create a driver
//         this->driver[bus_id] = new CanSocketDriver(bus_name[bus_id]);

//         // Check if succesfull, this will create a callback on timer, pubsub() in here
//         if(get_param<bool>("ATTEMPT_RETRY") == true){
//                 this->retry_timer[bus_id] = this->create_wall_timer(
//                 std::chrono::milliseconds(get_param<uint32_t>("CONNECTION_RETRY_INTERVAL")),
//                 [this, bus_id]() { 
//                     this->verifyConnection(bus_id); 
//                 } // Capture bus_id by value
//             );

//         }
//     }
// }

// MuxManager::~MuxManager() {
//     RCLCPP_INFO(this->get_logger(), "Deleting Mux Manager");
//     // Unalocate POINTERS !!!!!!!!!!!!!!!!!!!!!!!!!!
// }

// void MuxManager::verifyConnection(int bus_id) {
//     // Check if maximum retry attempts have been reached
//     if (retry_count[bus_id] >= get_param<uint32_t>("CONNECTION_RETRY_NUM")) {
//         RCLCPP_ERROR(this->get_logger(), "Maximum retry attempts reached. Giving up.");
//         retry_timer[bus_id]->cancel();  // Stop the retry attempts
//         rclcpp::shutdown();
//         return;
//     }

//     // This is not ideal, but it works -> improve when we have time
//     if (driver[bus_id]->isConnected()) {
//         RCLCPP_INFO(this->get_logger(), "CAN driver connected on %s", this->bus_name[bus_id]);
//         retry_timer[bus_id]->cancel();  // Stop the retry attempts
//         createPubSub(bus_id);
//     } else{
//         ++retry_count[bus_id];
//         RCLCPP_WARN(this->get_logger(), "CAN driver not connected on '%s', retrying... (Attempt %d/%d)", bus_name[bus_id], 
//             retry_count[bus_id], get_param<uint32_t>("CONNECTION_RETRY_NUM"));
    
//         delete driver[bus_id];  // Clean up the previous instance
//         driver[bus_id] = nullptr;
//     }    
// }

// // Attach to Can0 and Can1
// void MuxManager::createPubSub(int bus_id) {
//     this->bus[bus_id] = new CANBus(this->driver[bus_id]);
//     this->pub[bus_id] = new MuxPublisher(this->bus[bus_id], this);
//     this->sub[bus_id] = new MuxSubscriber(this->bus[bus_id], this);
//     this->driver[bus_id]->start_reception();
// }