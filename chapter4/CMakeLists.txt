cmake_minimum_required(VERSION 3.10)
project(so_arm101_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(
  so_arm101_hardware
  SHARED
  src/so_arm101_hardware.cpp
)

target_include_directories(
  so_arm101_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  so_arm101_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

target_link_libraries(
  so_arm101_hardware
  Threads::Threads
)

pluginlib_export_plugin_description_file(
  hardware_interface
  so_arm101_driver.xml
)

install(
  TARGETS so_arm101_hardware
  EXPORT export_so_arm101_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  export_so_arm101_hardware
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()