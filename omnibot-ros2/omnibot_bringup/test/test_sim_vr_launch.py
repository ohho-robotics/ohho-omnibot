# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""CPU check for the VR container launch. No ROS and no Gazebo.

sim_vr.launch.py is the profile docker-compose.gpu.yml runs. It must
force the Quest cameras and headless rendering, and it must not change
the sim.launch.py defaults used by sim-smoke.
"""

from pathlib import Path


BRINGUP = Path(__file__).resolve().parents[1]
SIM_VR = BRINGUP / "launch" / "sim_vr.launch.py"
SIM = BRINGUP / "launch" / "sim.launch.py"


def test_sim_vr_launch_pins_the_quest_profile():
    text = SIM_VR.read_text(encoding="utf-8")
    code = text.split('"""', 2)[-1]
    assert "sim.launch.py" in code
    assert '"vr": "true"' in code
    assert '"gui": "false"' in code
    assert '"arm": "true"' in code
    assert '"rosbridge": "true"' in code
    assert 'default_value="apartment"' in code
    assert 'default_value="true"' in code
    assert 'default_value="0.0.0.0"' in code
    assert "sim_cameras.launch.py" not in code
    assert "teleop_safety" not in code
    assert "cmd_vel_mux" not in code


def test_sim_launch_defaults_stay_headless_and_loopback():
    text = SIM.read_text(encoding="utf-8")
    assert 'gz_args = f"-r -s --headless-rendering {world_path}"' in text
    address = text.split('"rosbridge_address",', 1)[1].split('"vr",', 1)[0]
    assert 'default_value="127.0.0.1"' in address
    vr = text.split('"vr",', 1)[1].split('"video",', 1)[0]
    assert 'default_value="false"' in vr
