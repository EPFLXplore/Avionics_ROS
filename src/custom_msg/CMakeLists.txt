cmake_minimum_required(VERSION 3.5)
project(custom_msg)

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

find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

# install config directory
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


set(message_files
  "msg/not_hd/AngleArray.msg"
  "msg/not_hd/FourInOne.msg"
  "msg/not_hd/Imu.msg"
  "msg/not_hd/LaserResponse.msg"
  "msg/not_hd/LEDResponse.msg"
  "msg/not_hd/Mag.msg"
  "msg/not_hd/MassArray.msg"
  "msg/not_hd/NPK.msg"
  "msg/not_hd/ServoResponse.msg"
  "msg/not_hd/SpectroResponse.msg"
  "msg/not_hd/Voltage.msg"
  "msg/not_hd/LedsCommand.msg"
  "msg/not_hd/Led.msg"

  "msg/not_hd/NodeStateArray.msg"

  # HD
  "msg/hd/JointSpaceCmd.msg"
  "msg/hd/MotorCommand.msg"
  "msg/hd/Object.msg"
  "msg/hd/PoseGoal.msg"
  "msg/hd/ArucoObject.msg"
  "msg/hd/Task.msg"
  "msg/hd/CompressedRGBD.msg"
  "msg/hd/MotorCommands.msg"
  "msg/hd/OldMotorStatus.msg"
  "msg/hd/MotorStatushd.msg"
  "msg/hd/HDGoal.msg"
  "msg/hd/Rock.msg"
  "msg/hd/RockArray.msg"
  "msg/hd/Brick.msg"
  "msg/hd/BrickArray.msg"
  "msg/hd/Probe.msg"
  "msg/hd/ProbeArray.msg"
  "msg/hd/Ethernet.msg"
  "msg/hd/EthernetArray.msg"
  "msg/hd/BoundingBox.msg"
  "msg/hd/Mask.msg"
  "msg/hd/SegmentationData.msg"
  "msg/hd/Model.msg"
)

set(message_request_files
  "msg/not_hd/LaserRequest.msg"
  "msg/not_hd/LEDRequest.msg"
  "msg/not_hd/ServoRequest.msg"
  "msg/not_hd/SpectroRequest.msg"
)

set(message_config_files
  "msg/not_hd/MassConfigRequestJetson.msg"
  "msg/not_hd/MassConfigRequestMCU.msg"
  "msg/not_hd/MassConfigResponse.msg"

  "msg/not_hd/PotConfigRequestJetson.msg"
  "msg/not_hd/PotConfigRequestMCU.msg"
  "msg/not_hd/PotConfigResponse.msg"

  "msg/not_hd/ServoConfigRequestJetson.msg"
  "msg/not_hd/ServoConfigRequestMCU.msg"
  "msg/not_hd/ServoConfigResponse.msg"

  "msg/not_hd/AccelConfigRequestJetson.msg"
  "msg/not_hd/AccelConfigRequestMCU.msg"
  "msg/not_hd/AccelConfigResponse.msg"

  "msg/not_hd/GyroConfigRequestJetson.msg"
  "msg/not_hd/GyroConfigRequestMCU.msg"
  "msg/not_hd/GyroConfigResponse.msg"

  "msg/not_hd/MagConfigRequestJetson.msg"
  "msg/not_hd/MagConfigRequestMCU.msg"
  "msg/not_hd/MagConfigResponse.msg"
)

set(message_calib_files
  "msg/not_hd/MassCalibOffset.msg"
  "msg/not_hd/MassCalibScale.msg"

  "msg/not_hd/ImuCalib.msg"
)

set(message_nav
  # "msg/ArucoMarkers.msg"
  "msg/not_hd/Motorcmds.msg"
  "msg/not_hd/NavFeedback.msg"
  "msg/not_hd/Wheelstatus.msg"
  "msg/not_hd/Statussteering.msg"
  "msg/not_hd/MotorStatus.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DroneGoal.action"
  "action/HDManipulation.action"
  "action/DrillCmd.action"
  "action/DrillMain.action"
  "action/NAVReachGoal.action"
  "action/NewHDGoal.action"
  "srv/ChangeModeCamera.srv"
  "srv/ChangeModeSystem.srv"
  "srv/DroneState.srv"
  "srv/DrillMode.srv"
  "srv/RockSelection.srv"

  "srv/hd/HDMode.srv"
  "srv/hd/ButtonPressControlPanel.srv"
  "srv/CameraParams.srv"
  "srv/hd/GetCameraDistortionCoefficients.srv"
  "srv/hd/GetCameraIntrinsics.srv"
  "srv/hd/SelectPerceptionModules.srv"
  "srv/hd/SetPerceptionPipeline.srv"
  "srv/hd/Detect.srv"
  "srv/hd/InitializeModel.srv"
  "srv/hd/RequestHDGoal.srv"
  "srv/hd/SetCameraRGBMode.srv"

  "msg/not_hd/ScFSMStatusDrill.msg"
  "msg/not_hd/ScMotorCmds.msg"
  "msg/not_hd/ScMotorStatus.msg"
  "msg/not_hd/ScDrillCmds.msg"
  ${message_files}
  ${message_request_files}
  ${message_config_files}
  ${message_calib_files}
  ${message_nav}
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs geometry_msgs nav_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

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

ament_package()