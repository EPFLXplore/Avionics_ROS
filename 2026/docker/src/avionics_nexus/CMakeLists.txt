cmake_minimum_required(VERSION 3.8)
project(avionics_nexus)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg REQUIRED)

add_executable(avionics_nexus
  src/NexusMain.cpp
  src/Nexus.cpp)

# packets.h lives in the shared messages submodule (generated from the .msg),
# so both firmware and RPi include the exact same file. custom_msg is a sibling
# package in this workspace.
target_include_directories(avionics_nexus PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../custom_msg/include)

ament_target_dependencies(avionics_nexus
  rclcpp
  custom_msg)

install(TARGETS avionics_nexus
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Wire-contract test: packed struct sizes + distinct ids (see test_packets.cpp).
  ament_add_gtest(test_packets test/test_packets.cpp)
  if(TARGET test_packets)
    target_include_directories(test_packets PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../custom_msg/include)
  endif()
endif()

ament_package()
