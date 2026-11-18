# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Root wrapper for omnibot_bringup record.launch.py.

Usage:
    ros2 launch record.launch.py
    ros2 launch omnibot_bringup record.launch.py
"""

import importlib.util
from pathlib import Path

_LAUNCH_PATH = (
    Path(__file__).resolve().parent
    / "omnibot-ros2"
    / "omnibot_bringup"
    / "launch"
    / "record.launch.py"
)

spec = importlib.util.spec_from_file_location("record_launch_module", str(_LAUNCH_PATH))
_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(_module)

DEFAULT_TOPICS = _module.DEFAULT_TOPICS
generate_launch_description = _module.generate_launch_description
_build_record_process = _module._build_record_process
