# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Keep plain `pytest omnibot-ros2/omnibot_navigation/test` ROS-free.

test_nav_sim_scenarios.py starts Gazebo and needs ROS 2. It runs through
scripts/run_nav_scenarios.sh (launch_test) or the colcon nav_sim_* tests, not
through a bare pytest collection, unless OMNIBOT_NAV_SIM=1 is set.
"""

import os

_GAZEBO_TESTS = ["test_nav_sim_scenarios.py", "test_nav_regression_sim.py"]
collect_ignore = [] if os.environ.get("OMNIBOT_NAV_SIM") == "1" else _GAZEBO_TESTS
