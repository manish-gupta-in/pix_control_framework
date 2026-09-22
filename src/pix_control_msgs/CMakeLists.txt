cmake_minimum_required(VERSION 3.8)
project(pix_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GearReport.msg"
  "msg/TurnIndicatorsCommand.msg"
  "msg/TurnIndicatorsReport.msg"
  "msg/HazardLightsCommand.msg"
  "msg/HazardLightsReport.msg"
  "msg/ControlModeReport.msg"
  "msg/SteeringReport.msg"
  "msg/VelocityReport.msg"
  "srv/ControlModeCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
