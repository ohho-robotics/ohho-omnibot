cmake_minimum_required(VERSION 3.8)
project(omnibot_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint. Same as the other ament packages in this repo.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_moveit_config_files
    test/test_moveit_config_files.py
    TIMEOUT 60
  )
  # Headless mock hardware. The ros CI job runs this. sim-smoke does not.
  add_launch_test(
    test/test_moveit_plan_execute.py
    TIMEOUT 420
  )
endif()

install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
