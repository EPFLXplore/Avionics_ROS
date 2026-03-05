cmake_minimum_required(VERSION 3.8)
project(micro_ros_custom_msgs)

# 1. Find the necessary ROS 2 build tools
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate the message interfaces
# This command looks for the .msg file in the /msg folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BeatPacket.msg"
)

# 3. Export the dependencies
ament_export_dependencies(rosidl_default_runtime)

# 4. Standard ROS 2 package boilerplate
ament_package()