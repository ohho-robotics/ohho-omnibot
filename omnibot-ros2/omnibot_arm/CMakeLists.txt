cmake_minimum_required(VERSION 3.8)
project(omnibot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    scripts/arm_driver_node.py
    scripts/arm_math.py
    scripts/arm_safety.py
    scripts/arm_sim_logic.py
    scripts/arm_sim_driver_node.py
    scripts/arm_sim_scripted_move.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_arm_driver
    test/test_arm_driver.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_arm_safety
    test/test_arm_safety.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_arm_sim
    test/test_arm_sim.py
    TIMEOUT 60
  )
endif()

ament_package()
