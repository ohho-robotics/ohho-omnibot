cmake_minimum_required(VERSION 3.14)
project(mecanum_drive_ros2)

# Require C++17 for the header-only library
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Install header-only C++ library ─────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

# Export include directories for downstream packages
ament_export_include_directories(include)

# ── Install Python module ────────────────────────────────────────────────────
ament_python_install_package(mecanum_drive_ros2)

# ── Install config ───────────────────────────────────────────────────────────
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mecanum_kinematics test/test_mecanum_kinematics.cpp)
  target_include_directories(test_mecanum_kinematics PRIVATE include)
endif()

ament_package()
