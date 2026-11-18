cmake_minimum_required(VERSION 3.8)
project(omnibot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PlanarWheelMock is a Gazebo system plugin. gz_ros2_control loads the
# gz_sim_vendor shim, and that shim's find_package(gz-sim8) only succeeds
# when the versioned vendor prefixes are on CMAKE_PREFIX_PATH. setup.bash
# adds them, but colcon's -DCMAKE_PREFIX_PATH replaces that list with the
# ament prefixes and the shim then cannot find gz-sim8Config.cmake. Search
# every prefix under /opt/ros/<distro>/opt before the find. Sim is the only
# caller, so a missing gz-sim8 skips the plugin instead of failing configure.
if(DEFINED ENV{CMAKE_PREFIX_PATH})
  string(REPLACE ":" ";" _omnibot_env_prefixes "$ENV{CMAKE_PREFIX_PATH}")
  list(APPEND CMAKE_PREFIX_PATH ${_omnibot_env_prefixes})
endif()
find_package(gz_sim_vendor QUIET)
if(gz_sim_vendor_DIR)
  get_filename_component(
    _omnibot_gz_opt
    "${gz_sim_vendor_DIR}/../../../opt"
    ABSOLUTE
  )
  if(IS_DIRECTORY "${_omnibot_gz_opt}")
    file(GLOB _omnibot_gz_prefixes LIST_DIRECTORIES true "${_omnibot_gz_opt}/*")
    list(PREPEND CMAKE_PREFIX_PATH ${_omnibot_gz_prefixes})
  endif()
endif()
find_package(gz-sim8 QUIET CONFIG)
set(_omnibot_gz_sim8_found "${gz-sim8_FOUND}")
if(_omnibot_gz_sim8_found)
  find_package(gz_ros2_control REQUIRED)
  find_package(gz-sim REQUIRED)

  # GZResourceManager only loads GazeboSimSystemInterface. The planar wheel
  # block uses this plugin so the mecanum controller can be active. Non-zero
  # wheel velocities are not written into Gazebo. A zero command brakes the
  # joint so the chassis cannot free-roll. A non-zero command clears that
  # brake with a zero joint force, because a Harmonic velocity command
  # latches a DART servo.
  add_library(omnibot_planar_wheel_mock SHARED
    src/planar_wheel_mock.cpp
  )
  target_link_libraries(omnibot_planar_wheel_mock gz-sim::gz-sim)
  ament_target_dependencies(omnibot_planar_wheel_mock
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    gz_ros2_control
  )
  pluginlib_export_plugin_description_file(gz_ros2_control omnibot_planar_wheel_mock.xml)

  install(TARGETS omnibot_planar_wheel_mock
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(STATUS
    "gz-sim8 was not found. Skipping omnibot_planar_wheel_mock. "
    "Gazebo sim launch needs ros-jazzy-gz-ros2-control and ros-jazzy-gz-sim-vendor."
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # No Gazebo. ros2_control_node plus GenericSystem, same controllers.yaml.
  add_launch_test(
    test/test_mock_controllers.py
    TIMEOUT 180
  )
  ament_add_pytest_test(test_teleop_default
    test/test_teleop_default.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_teleop_safety
    test/test_teleop_safety.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_sim_vr_profile
    test/test_sim_vr_profile.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_robot_vr_profile
    test/test_robot_vr_profile.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_pick_place_spawn
    test/test_pick_place_spawn.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_sim_cameras
    test/test_sim_cameras.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_sim_vr_launch
    test/test_sim_vr_launch.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_record_launch
    test/test_record_launch.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_dds_profile
    test/test_dds_profile.py
    TIMEOUT 60
  )

  # Headless Gazebo launch test. The sim-smoke CI job exports
  # OMNIBOT_SIM_SMOKE=1 before configure. The default colcon test job
  # does not, so it does not start Gazebo.
  if("$ENV{OMNIBOT_SIM_SMOKE}" STREQUAL "1")
    add_launch_test(
      test/test_sim_launch.py
      TIMEOUT 420
    )
    add_launch_test(
      test/test_teleop_safety_sim.py
      TIMEOUT 240
    )
    add_launch_test(
      test/test_sim_vr_contract.py
      TIMEOUT 420
    )
  endif()
endif()

install(PROGRAMS
  scripts/descriptor_node.py
  scripts/ros2_control_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/ros2_control_adapter_logic.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/descriptor_assemble.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (including master/ subfolder)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
