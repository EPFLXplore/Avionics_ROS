cmake_minimum_required(VERSION 3.8)
project(avionics_costco)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_msg REQUIRED)
find_package(rclcpp REQUIRED)

# Find the serial port library
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIALPORT REQUIRED libserialport)


include_directories(
  include
  ${SERIALPORT_INCLUDE_DIRS}
)

link_directories(${SERIALPORT_LIBRARY_DIRS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Specify Arduino libraries you want to link to
link_directories(
  ${ARDUINO_CORE_PATH}/lib
)

# Create the executable
add_executable(avionics_costco
  src/CostcoPublisher.cpp
  src/CostcoMain.cpp
  src/CostcoSubscriber.cpp
  src/Cosco.cpp
  src/SerialDriver.cpp
)

target_link_libraries(avionics_costco
  ${SERIALPORT_LIBRARIES}
)


ament_target_dependencies(
  avionics_costco
  rclcpp
  custom_msg
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the target
install(TARGETS
  avionics_costco
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
