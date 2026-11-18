"""Home-tick loader, lerobot JSON conversion, and arm.launch.py wiring.

No ROS and no arm. The JSON and yaml fixtures are synthetic.
"""

import ast
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
FIXTURES = PACKAGE / "test" / "fixtures"
FIXTURE_JSON = FIXTURES / "lerobot_calibrate_ci.json"
FIXTURE_YAML = FIXTURES / "so101_ci_fixture.yaml"
ARM_PARAMS = PACKAGE / "config" / "arm_params.yaml"
LAUNCH = PACKAGE / "launch" / "arm.launch.py"
REPO = PACKAGE.parents[1]

sys.path.insert(0, str(SCRIPTS))

import servo_offsets  # noqa: E402

FIXTURE_TICKS = [100, 200, 300, 400, 500, 600]
FACTORY = [2048, 2048, 2048, 2048, 2048, 2048]


@pytest.fixture(autouse=True)
def _no_calibration_env(monkeypatch):
    monkeypatch.delenv("OMNIBOT_ARM_CALIBRATION", raising=False)
    monkeypatch.delenv("OMNIBOT_ARM_ROBOT_ID", raising=False)
    monkeypatch.delenv("OMNIBOT_ARM_CALIBRATION_DIR", raising=False)


def _resolve(tmp_path, **kwargs):
    kwargs.setdefault("environ", {})
    kwargs.setdefault("config_dir", str(tmp_path))
    return servo_offsets.resolve_home_ticks(**kwargs)


def _dump(path, ticks, robot_id=None, drive_modes=None):
    payload = {"home_ticks": list(ticks)}
    if robot_id is not None:
        payload["robot_id"] = robot_id
    if drive_modes is not None:
        payload["joints"] = []
        for index, mode in enumerate(drive_modes):
            payload["joints"].append(
                {
                    "lerobot_name": servo_offsets.LEROBOT_NAMES[index],
                    "joint_name": servo_offsets.JOINT_NAMES[index],
                    "id": index + 1,
                    "drive_mode": mode,
                    "home_tick": ticks[index],
                }
            )
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")


class TestValidation:
    def test_factory_constant_is_six_midpoints(self):
        assert list(servo_offsets.FACTORY_HOME_TICKS) == FACTORY
        assert servo_offsets.validate_home_ticks(FACTORY) == FACTORY

    def test_rejects_wrong_length(self):
        with pytest.raises(servo_offsets.CalibrationError, match="6"):
            servo_offsets.validate_home_ticks([0, 0, 0, 0, 0])
        with pytest.raises(servo_offsets.CalibrationError, match="6"):
            servo_offsets.validate_home_ticks([0, 0, 0, 0, 0, 0, 0])

    def test_rejects_out_of_range_and_bool_and_float(self):
        with pytest.raises(servo_offsets.CalibrationError, match="4096"):
            servo_offsets.validate_home_ticks([4096, 0, 0, 0, 0, 0])
        with pytest.raises(servo_offsets.CalibrationError, match="-1"):
            servo_offsets.validate_home_ticks([-1, 0, 0, 0, 0, 0])
        with pytest.raises(servo_offsets.CalibrationError, match="integer"):
            servo_offsets.validate_home_ticks([0, True, 0, 0, 0, 0])
        with pytest.raises(servo_offsets.CalibrationError, match="integer"):
            servo_offsets.validate_home_ticks([0.0, 0, 0, 0, 0, 0])
        with pytest.raises(servo_offsets.CalibrationError, match="integer"):
            servo_offsets.validate_home_ticks(["0", 0, 0, 0, 0, 0])

    def test_endpoints_are_inclusive(self):
        assert servo_offsets.validate_home_ticks([0, 4095, 0, 4095, 0, 4095]) == [
            0,
            4095,
            0,
            4095,
            0,
            4095,
        ]

    def test_robot_id_rejects_path_separators(self):
        for robot_id in ("../secret", "a/b", "a\\b", "", "has space", ".hidden"):
            with pytest.raises(servo_offsets.CalibrationError):
                servo_offsets.validate_robot_id(robot_id)


class TestLerobotJson:
    def test_fixture_midpoints_ignore_homing_offset(self):
        payload = json.loads(FIXTURE_JSON.read_text(encoding="utf-8"))
        motors = servo_offsets.parse_lerobot_json(payload)
        assert [motor.home_tick for motor in motors] == FIXTURE_TICKS
        wrist = motors[4]
        assert wrist.lerobot_name == "wrist_roll"
        assert wrist.homing_offset == 2048
        assert wrist.home_tick == 500
        assert wrist.home_tick != wrist.homing_offset

    def test_full_range_midpoint_is_2047(self):
        payload = {
            name: {"id": index + 1, "drive_mode": 0, "range_min": 0, "range_max": 4095}
            for index, name in enumerate(servo_offsets.LEROBOT_NAMES)
        }
        motors = servo_offsets.parse_lerobot_json(payload)
        assert [motor.home_tick for motor in motors] == [2047] * 6

    def test_equal_or_inverted_range_is_rejected(self):
        payload = json.loads(FIXTURE_JSON.read_text(encoding="utf-8"))
        payload["gripper"]["range_min"] = 10
        payload["gripper"]["range_max"] = 10
        with pytest.raises(servo_offsets.CalibrationError, match="range_min"):
            servo_offsets.parse_lerobot_json(payload)

    def test_missing_or_unknown_joint_is_rejected(self):
        payload = json.loads(FIXTURE_JSON.read_text(encoding="utf-8"))
        del payload["gripper"]
        with pytest.raises(servo_offsets.CalibrationError, match="missing"):
            servo_offsets.parse_lerobot_json(payload)
        payload = json.loads(FIXTURE_JSON.read_text(encoding="utf-8"))
        payload["shoulderpan"] = payload["shoulder_pan"]
        with pytest.raises(servo_offsets.CalibrationError, match="unknown"):
            servo_offsets.parse_lerobot_json(payload)

    def test_legacy_start_end_arrays(self):
        payload = {
            "homing_offset": [9, 9, 9, 9, 9, 9],
            "drive_mode": [0, 0, 0, 0, 0, 0],
            "start_pos": [0, 100, 200, 300, 400, 500],
            "end_pos": [200, 300, 400, 500, 600, 700],
            "note": "synthetic legacy file",
        }
        motors = servo_offsets.parse_lerobot_json(payload)
        assert [motor.home_tick for motor in motors] == FIXTURE_TICKS
        assert motors[0].homing_offset == 9

    def test_not_calibration_json(self):
        with pytest.raises(servo_offsets.CalibrationError, match="lerobot-calibrate"):
            servo_offsets.parse_lerobot_json({"nope": 1})
        with pytest.raises(servo_offsets.CalibrationError):
            servo_offsets.parse_lerobot_json([1, 2, 3, 4, 5, 6])


class TestConverterCli:
    def test_fake_json_writes_yaml_the_loader_accepts(self, tmp_path):
        output = tmp_path / "so101_ci_fixture.yaml"
        script = SCRIPTS / "lerobot_calibration_to_yaml.py"
        completed = subprocess.run(
            [
                sys.executable,
                str(script),
                str(FIXTURE_JSON),
                "--robot-id",
                "ci_fixture",
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        assert "Wrote" in completed.stderr
        loaded = _resolve(tmp_path, calibration_file=str(output))
        assert loaded.home_ticks == FIXTURE_TICKS
        assert loaded.used_factory_default is False
        assert loaded.warning is None
        on_disk = yaml.safe_load(FIXTURE_YAML.read_text(encoding="utf-8"))
        assert on_disk["home_ticks"] == FIXTURE_TICKS
        assert "Not a physical" in FIXTURE_YAML.read_text(encoding="utf-8")

    def test_bad_json_does_not_write_yaml(self, tmp_path):
        source = tmp_path / "bad.json"
        source.write_text("{", encoding="utf-8")
        output = tmp_path / "out.yaml"
        completed = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS / "lerobot_calibration_to_yaml.py"),
                str(source),
                "--robot-id",
                "ci_fixture",
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode == 2
        assert not output.exists()

    def test_drive_mode_warns_and_still_writes(self, tmp_path):
        payload = json.loads(FIXTURE_JSON.read_text(encoding="utf-8"))
        payload["gripper"]["drive_mode"] = 1
        source = tmp_path / "signed.json"
        source.write_text(json.dumps(payload), encoding="utf-8")
        output = tmp_path / "so101_ci_fixture.yaml"
        completed = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS / "lerobot_calibration_to_yaml.py"),
                str(source),
                "--robot-id",
                "ci_fixture",
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        assert "drive_mode" in completed.stderr
        loaded = _resolve(tmp_path, calibration_file=str(output))
        assert loaded.home_ticks == FIXTURE_TICKS
        assert "drive_mode" in loaded.warning
        assert "NOT this robot" not in loaded.warning


class TestLoader:
    def test_launch_arg_calibration_file(self, tmp_path):
        loaded = _resolve(tmp_path, calibration_file=str(FIXTURE_YAML))
        assert loaded.home_ticks == FIXTURE_TICKS
        assert loaded.source == "calibration_file"
        assert loaded.warning is None

    def test_empty_launch_args_fall_back_to_factory(self, tmp_path):
        loaded = _resolve(tmp_path, calibration_file="", robot_id="")
        assert loaded.home_ticks == FACTORY
        assert loaded.used_factory_default is True
        assert "NOT this robot" in loaded.warning
        assert "2048" in loaded.warning

    def test_custom_parameter_fallback_does_not_warn(self, tmp_path):
        custom = [10, 20, 30, 40, 50, 60]
        loaded = _resolve(tmp_path, calibration_file="  ", fallback=custom)
        assert loaded.home_ticks == custom
        assert loaded.warning is None
        assert loaded.used_factory_default is False
        assert loaded.source == "parameter"

    def test_launch_arg_beats_env_and_robot_id(self, tmp_path, monkeypatch):
        via_arg = tmp_path / "from_arg.yaml"
        via_env = tmp_path / "from_env.yaml"
        _dump(via_arg, FIXTURE_TICKS, robot_id="ci_fixture")
        _dump(via_env, [11, 22, 33, 44, 55, 66], robot_id="other")
        robot_dir = tmp_path / "calibration"
        robot_dir.mkdir()
        _dump(robot_dir / "so101_other.yaml", [9, 9, 9, 9, 9, 9], robot_id="other")
        loaded = _resolve(
            tmp_path,
            calibration_file=str(via_arg),
            robot_id="ci_fixture",
            environ={
                "OMNIBOT_ARM_CALIBRATION": str(via_env),
                "OMNIBOT_ARM_ROBOT_ID": "other",
            },
        )
        assert loaded.home_ticks == FIXTURE_TICKS
        assert loaded.source == "calibration_file"

    def test_env_path_beats_robot_id(self, tmp_path):
        via_env = tmp_path / "from_env.yaml"
        _dump(via_env, [11, 22, 33, 44, 55, 66])
        robot_dir = tmp_path / "calibration"
        robot_dir.mkdir()
        _dump(
            robot_dir / "so101_ci_fixture.yaml",
            FIXTURE_TICKS,
            robot_id="ci_fixture",
        )
        loaded = _resolve(
            tmp_path,
            robot_id="ci_fixture",
            environ={"OMNIBOT_ARM_CALIBRATION": str(via_env)},
        )
        assert loaded.home_ticks == [11, 22, 33, 44, 55, 66]
        assert loaded.source == "OMNIBOT_ARM_CALIBRATION"

    def test_robot_id_loads_so101_yaml(self, tmp_path):
        directory = tmp_path / "calibration"
        directory.mkdir()
        _dump(
            directory / "so101_ci_fixture.yaml",
            FIXTURE_TICKS,
            robot_id="ci_fixture",
        )
        loaded = _resolve(tmp_path, robot_id="ci_fixture")
        assert loaded.home_ticks == FIXTURE_TICKS
        assert loaded.source == "robot_id"
        assert loaded.path.endswith(os.path.join("calibration", "so101_ci_fixture.yaml"))

    def test_calibration_dir_env_replaces_package_dir(self, tmp_path):
        package_dir = tmp_path / "calibration"
        package_dir.mkdir()
        _dump(
            package_dir / "so101_ci_fixture.yaml",
            FIXTURE_TICKS,
            robot_id="ci_fixture",
        )
        local_dir = tmp_path / "local"
        local_dir.mkdir()
        _dump(local_dir / "so101_ci_fixture.yaml", [7, 7, 7, 7, 7, 7], robot_id="ci_fixture")
        loaded = _resolve(
            tmp_path,
            robot_id="ci_fixture",
            environ={"OMNIBOT_ARM_CALIBRATION_DIR": str(local_dir)},
        )
        assert loaded.home_ticks == [7, 7, 7, 7, 7, 7]
        empty = tmp_path / "empty"
        empty.mkdir()
        with pytest.raises(servo_offsets.CalibrationError, match="not found"):
            _resolve(
                tmp_path,
                robot_id="ci_fixture",
                environ={"OMNIBOT_ARM_CALIBRATION_DIR": str(empty)},
            )

    def test_missing_requested_file_does_not_fall_back(self, tmp_path):
        with pytest.raises(servo_offsets.CalibrationError, match="not found"):
            _resolve(tmp_path, calibration_file=str(tmp_path / "missing.yaml"))
        with pytest.raises(servo_offsets.CalibrationError, match="not found"):
            _resolve(tmp_path, robot_id="ci_fixture")

    def test_robot_id_mismatch_and_bad_yaml(self, tmp_path):
        path = tmp_path / "so101_ci_fixture.yaml"
        _dump(path, FIXTURE_TICKS, robot_id="ci_fixture")
        with pytest.raises(servo_offsets.CalibrationError, match="expected"):
            _resolve(tmp_path, calibration_file=str(path), robot_id="other_robot")
        path.write_text("home_ticks: [1, 2, 3]\n", encoding="utf-8")
        with pytest.raises(servo_offsets.CalibrationError, match="6"):
            _resolve(tmp_path, calibration_file=str(path))
        path.write_text("home_ticks: [1, true, 3, 4, 5, 6]\n", encoding="utf-8")
        with pytest.raises(servo_offsets.CalibrationError, match="integer"):
            _resolve(tmp_path, calibration_file=str(path))

    def test_joints_must_agree_and_sign_warns(self, tmp_path):
        path = tmp_path / "signed.yaml"
        _dump(path, [10, 20, 30, 40, 50, 60], drive_modes=[0, 0, 0, 0, 0, 1])
        loaded = _resolve(tmp_path, calibration_file=str(path))
        assert loaded.home_ticks == [10, 20, 30, 40, 50, 60]
        assert "arm_gripper" in loaded.warning
        path.write_text(
            yaml.safe_dump(
                {
                    "home_ticks": [10, 20, 30, 40, 50, 60],
                    "joints": [
                        {
                            "lerobot_name": servo_offsets.LEROBOT_NAMES[index],
                            "joint_name": servo_offsets.JOINT_NAMES[index],
                            "id": index + 1,
                            "drive_mode": 0,
                            "home_tick": 0,
                        }
                        for index in range(6)
                    ],
                }
            ),
            encoding="utf-8",
        )
        with pytest.raises(servo_offsets.CalibrationError, match="mixed offsets"):
            _resolve(tmp_path, calibration_file=str(path))

    def test_static_fixture_and_arm_params(self):
        loaded = servo_offsets.resolve_home_ticks(
            calibration_file=str(FIXTURE_YAML),
            config_dir=str(PACKAGE / "config"),
            environ={},
        )
        assert loaded.home_ticks == FIXTURE_TICKS
        assert servo_offsets.home_ticks_from_arm_params(str(ARM_PARAMS)) == FACTORY
        params = yaml.safe_load(ARM_PARAMS.read_text(encoding="utf-8"))
        names = params["arm_driver_node"]["ros__parameters"]["joint_names"]
        assert names == list(servo_offsets.JOINT_NAMES)
        committed = list((PACKAGE / "config" / "calibration").glob("so101_*.yaml"))
        assert committed == []

    def test_launch_overrides_keep_resolved_flag(self, tmp_path):
        resolution = _resolve(tmp_path, calibration_file=str(FIXTURE_YAML), robot_id="")
        overrides = servo_offsets.launch_parameter_overrides(resolution, "")
        assert overrides["home_ticks"] == FIXTURE_TICKS
        assert overrides["home_ticks_resolved"] is True
        assert overrides["calibration_file"] == resolution.path
        assert overrides["robot_id"] == ""

    def test_node_helper_trusts_launch_and_skips_a_bad_env(self, tmp_path):
        trusted = servo_offsets.home_ticks_for_node(
            home_ticks_param=FIXTURE_TICKS,
            calibration_file="",
            robot_id="",
            resolved_by_launch=True,
            config_dir=str(tmp_path),
            environ={"OMNIBOT_ARM_CALIBRATION": str(tmp_path / "missing.yaml")},
        )
        assert trusted.home_ticks == FIXTURE_TICKS
        assert trusted.source == "launch"
        assert trusted.warning is None
        with pytest.raises(servo_offsets.CalibrationError):
            servo_offsets.home_ticks_for_node(
                home_ticks_param=[1, 2, 3],
                calibration_file="",
                robot_id="",
                resolved_by_launch="true",
                config_dir=str(tmp_path),
                environ={},
            )


class TestLaunchWiring:
    def test_arm_launch_declares_args_and_resolves(self):
        text = LAUNCH.read_text(encoding="utf-8")
        assert 'DeclareLaunchArgument(\n                "calibration_file"' in text
        assert 'DeclareLaunchArgument(\n                "robot_id"' in text
        assert "home_ticks_from_arm_params(" in text
        assert "resolve_home_ticks(" in text
        assert "launch_parameter_overrides(" in text
        assert "if resolution.warning:" in text
        assert "print(resolution.warning, file=sys.stderr" in text
        ast.parse(text)

    def test_bringup_launches_forward_the_same_args(self):
        mobile = (
            REPO / "omnibot-ros2" / "omnibot_bringup" / "launch" / "mobile_manipulation.launch.py"
        ).read_text(encoding="utf-8")
        pi = (
            REPO
            / "omnibot-ros2"
            / "omnibot_bringup"
            / "launch"
            / "master"
            / "omnibot_pi.launch.py"
        ).read_text(encoding="utf-8")
        for text in (mobile, pi):
            assert '"calibration_file": LaunchConfiguration("calibration_file")' in text
            assert '"robot_id": LaunchConfiguration("robot_id")' in text
            assert 'DeclareLaunchArgument(\n                "calibration_file"' in text or (
                'DeclareLaunchArgument(\n        "calibration_file"' in text
            )
            ast.parse(text)

    def test_loader_sources_do_not_import_a_motor_bus(self):
        for path in (SCRIPTS / "servo_offsets.py", SCRIPTS / "lerobot_calibration_to_yaml.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            modules = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    modules.extend(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    modules.append(node.module.split(".")[0])
            assert "serial" not in modules
            assert "lerobot" not in modules

    def test_driver_resolves_before_opening_the_bus(self):
        text = (SCRIPTS / "arm_driver_node.py").read_text(encoding="utf-8")
        assert text.index("home_ticks_for_node(") < text.index("self.connect_follower()")
