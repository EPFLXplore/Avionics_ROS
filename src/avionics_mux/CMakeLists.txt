cmake_minimum_required(VERSION 3.5)
project(avionics_mux)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # Assume BRoCo is located in a sibling directory; adjust the path if necessary.
# set(BRoCo_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../BRoCo)
# if(EXISTS ${BRoCo_DIR}/CMakeLists.txt)
#   add_subdirectory(${BRoCo_DIR} ${CMAKE_BINARY_DIR}/BRoCo)
# else()
#   message(FATAL_ERROR "BRoCo source not found in ${BRoCo_DIR}")
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg REQUIRED)

include_directories(
  include/avionics_mux
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(avionics_mux
  src/avionics_mux.cpp
  src/MuxManager.cpp
  src/MuxPublisher.cpp
  src/MuxSubscriber.cpp
)

ament_target_dependencies(avionics_mux rclcpp custom_msg)

install(TARGETS
  avionics_mux
  DESTINATION lib/${PROJECT_NAME})

ament_package()
