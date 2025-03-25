#include "CostcoPublisher.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("CostcoPublisher");
  RCLCPP_INFO(node->get_logger(), "Creating node...");

  rclcpp::spin(std::make_shared<CostcoPublisher>());
  rclcpp::shutdown();
  return 0;
}