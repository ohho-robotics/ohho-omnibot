## Cursor Cloud specific instructions

This repository is a meta-workspace (ROS 2 robot, AI engines, Android, Unity Quest). On a headless Linux VM without a robot, GPU, Unity, or the Android SDK, develop the pure-Python cores below. Standard install and test commands are in each package README (`mecanum-kinematics/README.md`, `omnibot-ai-engines/learning_engine/README.md`, `omnibot-ai-engines/agent_engine/ARCHITECTURE.md`, `yahboom-python-driver/README.md`, `ros2-bev-stitcher/README.md`).

- Use the repo-local virtualenv at `.venv` (`source .venv/bin/activate`). It is not committed. Editable installs: `mecanum-kinematics`, `omnibot-ai-engines/learning_engine`, `omnibot-ai-engines/agent_engine`, `yahboom-python-driver`. Dev extras in that env: `pytest`, `numpy`, `pyserial`, `pyyaml`.
- Tests that pass with only that env:
  - `pytest mecanum-kinematics/tests`
  - `pytest yahboom-python-driver/tests`
  - `pytest ros2-bev-stitcher/tests` (the test file puts the package on `sys.path`; numpy is enough)
  - `python3 -m unittest discover -s omnibot-ai-engines/learning_engine/tests -t omnibot-ai-engines`
  - `python3 -m unittest discover -s omnibot-ai-engines/agent_engine/tests -t omnibot-ai-engines`
  - `PYTHONPATH=omnibot-ai-engines pytest omnibot-ai-engines/data_engine/tests` (the synchronizer test is numpy-only; do not `pip install -e` `data_engine`, its `setup.py` pulls torch)
- There is no project linter config (no ruff, flake8, or pre-commit). `python3 -m compileall` on the Python trees is the syntax check.
- `omnibot-ros2/omnibot_arm/test` imports `rclpy` at module load, and `omnibot_hybrid` / `omnibot_driver` tests import ROS message packages. ROS 2 Jazzy is not installed. The root README's `colcon` / `ros2 launch` flow needs Ubuntu 24.04 + ROS 2 Jazzy and, for simulation, Gazebo. `omnibot-digital-twin/docker/docker-compose.yml` still points at a missing `digital_twin/docker/Dockerfile.sim` and `robot_ws/install/setup.bash`.
- `omnibot-android` needs the Android SDK and JDK 17 (`./gradlew`). `omnibot-vr` is a Unity project (`ProjectSettings/ProjectVersion.txt`). `vla_engine` is packaged via `omnibot-ai-engines/vla_engine/pyproject.toml`; CI installs CPU torch and runs `pytest omnibot-ai-engines/vla_engine/tests`. Full OpenVLA inference still needs a GPU stack (`torch`, `transformers`). `lerobot_engine` and `rl_engine` need GPU stacks (torch / Isaac) and are not installed by default.
