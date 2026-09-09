cmake_minimum_required(VERSION 3.8)
project(pix_can_codec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

# ─── Library ──────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/encoder.cpp
  src/decoder.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ─── Install ──────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ─── Testing ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_can_codec test/test_can_codec.cpp)
  target_link_libraries(test_can_codec ${PROJECT_NAME})
  target_include_directories(test_can_codec PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

# ─── Export ───────────────────────────────────────────────────────────────────
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
