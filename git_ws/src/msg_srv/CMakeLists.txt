################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(msg_srv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/AudioMute.msg"
  "msg/BottomSensor.msg"
  "msg/BatteryLevel.msg"
  "msg/BreedingStatus.msg"
  "msg/BumperSensor.msg"
  "msg/ChargeConnection.msg"
  "msg/ChargePower.msg"
  "msg/Current.msg"
  "msg/DockingSensor.msg"
  "msg/DrivingCommand.msg"
  "msg/Encoder.msg"
  "msg/ErrorCode.msg"
  "msg/HeartBeat.msg"
  "msg/LEDBright.msg"
  "msg/LineLaser.msg"
  "msg/LockSensor.msg"
  "msg/MotorBrake.msg"
  "msg/NaviGuider.msg"
  "msg/PowerStatus.msg"
  "msg/PowerThreshold.msg"
  "msg/RadarSensor.msg"
  "msg/RelaySwitch.msg"
  "msg/SeroConfig.msg"
  "msg/ShuttingDown.msg"
  "msg/SleepMode.msg"
  "msg/StationID.msg"
  "msg/Temperature.msg"
  "msg/Time.msg"
  "msg/TopSensor.msg"
  "msg/TouchSensor.msg"
  "msg/UltrasonicSensor.msg"
)

set(srv_files
  "srv/SoundPlay.srv"
)

set(action_files

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()
