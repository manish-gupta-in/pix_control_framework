cmake_minimum_required(VERSION 3.8)
project(pix_vehicle_interface_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(pix_control_msgs REQUIRED)
find_package(pix_vehicle_msgs REQUIRED)
find_package(pix_can_codec REQUIRED)

# ─── Executable ───────────────────────────────────────────────────────────────
add_executable(pix_vehicle_interface
  src/pix_vehicle_interface.cpp
)

target_include_directories(pix_vehicle_interface PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(pix_vehicle_interface
  rclcpp
  can_msgs
  pix_control_msgs
  pix_vehicle_msgs
  pix_can_codec
)

target_link_libraries(pix_vehicle_interface
  pix_can_codec::pix_can_codec
)

# ─── Install ──────────────────────────────────────────────────────────────────
install(TARGETS pix_vehicle_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
