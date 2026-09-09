cmake_minimum_required(VERSION 3.8)
project(pix_can_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

# ─── Executable ───────────────────────────────────────────────────────────────
add_executable(pix_can_driver
  src/pix_can_driver.cpp
)

target_include_directories(pix_can_driver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(pix_can_driver
  rclcpp
  can_msgs
)

# ─── Install ──────────────────────────────────────────────────────────────────
install(TARGETS pix_can_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
