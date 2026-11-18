#!/bin/bash
# Runs once after the DevContainer is created (postCreateCommand).
# Installs workspace dependencies and builds the ROS 2 workspace.
set -e

cd /workspaces/OmniBot

echo "=== Installing rosdep dependencies ==="
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
  --dependency-types=build \
  --dependency-types=buildtool \
  --dependency-types=build_export \
  --dependency-types=buildtool_export \
  --dependency-types=exec \
  --skip-keys "rtabmap_ros octomap_server pcl_ros octomap_msgs"

echo "=== Installing standalone Python packages that live in this repo ==="
pip3 install --break-system-packages \
  -e mecanum-kinematics \
  -e yahboom-python-driver \
  -e omnibot-ai-engines/learning_engine \
  -e omnibot-ai-engines/agent_engine

echo "=== Building ROS 2 workspace ==="
colcon build --symlink-install

echo "=== Done! Source the workspace: ==="
echo "  source /workspaces/OmniBot/install/setup.bash"
