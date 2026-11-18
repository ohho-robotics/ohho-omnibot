cmake_minimum_required(VERSION 3.8)
project(omnibot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and config
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts so they are discoverable as ROS 2 executables
install(PROGRAMS
  scripts/waypoint_navigator.py
  scripts/test_autonomous.py
  scripts/go_to_waypoint.py
  scripts/sim_map_sweep.py
  scripts/generate_sim_map.sh
  scripts/run_nav_scenarios.sh
  scripts/run_nav_regression_suite.sh
  scripts/spawn_regression_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
)
# Importable helpers that the scripts above load from their own directory.
install(FILES
  scripts/waypoint_loader.py
  scripts/regression_scenario_loader.py
  scripts/summarize_nav_results.py
  scripts/aggregate_nav_results.py
  scripts/write_nav_regression_report.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks — launch files / configs have no
  # copyright headers (consistent with omnibot_bringup / omnibot_description).
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ROS-free checks: waypoints, Nav2 holonomic params, saved map consistency.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_waypoints test/test_waypoints.py TIMEOUT 60)
  ament_add_pytest_test(test_nav_assets test/test_nav_assets.py TIMEOUT 60)
  ament_add_pytest_test(test_nav_results test/test_nav_results.py TIMEOUT 60)
  ament_add_pytest_test(test_regression_scenarios test/test_regression_scenarios.py TIMEOUT 60)
  ament_add_pytest_test(test_aggregate_nav_results test/test_aggregate_nav_results.py TIMEOUT 60)

  # Headless Gazebo scenarios. The nav-sim CI job exports OMNIBOT_NAV_SIM=1
  # before configure; the default colcon test job does not, so it does not
  # start Gazebo. One test per scenario so each starts from the spawn pose.
  if("$ENV{OMNIBOT_NAV_SIM}" STREQUAL "1")
    find_package(launch_testing_ament_cmake REQUIRED)
    foreach(scenario kitchen desk door)
      add_launch_test(
        test/test_nav_sim_scenarios.py
        TARGET nav_sim_${scenario}
        TIMEOUT 600
        ENV OMNIBOT_NAV_SCENARIO=${scenario}
      )
    endforeach()
  endif()
endif()

ament_package()
