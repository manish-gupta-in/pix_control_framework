cmake_minimum_required(VERSION 3.8)
project(pix_dbw_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pix_msgs REQUIRED)
find_package(pix_vehicle_interface REQUIRED)

add_library(${PROJECT_NAME} src/dbw_manager.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} rclcpp pix_msgs pix_vehicle_interface)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp pix_msgs pix_vehicle_interface)
ament_package()
