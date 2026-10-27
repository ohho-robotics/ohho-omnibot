"""Load SO-101 home ticks from a calibration file. No ROS and no hardware.

arm.launch.py and arm_driver_node call this module. It reads YAML or converts
a lerobot-calibrate JSON file. It does not open a port or import a motor bus.

home_tick for each joint is (range_min + range_max) // 2. That is LeRobot's
degree-mode zero, in the same Present_Position counts the driver reads after
lerobot-calibrate has written Homing_Offset into the servo. homing_offset is
not added again.
"""

import json
import os
import re
from collections.abc import Mapping
from collections.abc import Sequence

try:
    import yaml
except ImportError as exc:  # pragma: no cover - CI installs python3-yaml
    raise ImportError(
        "PyYAML is required to load SO-101 calibration files. "
        "Install the rosdep package python3-yaml."
    ) from exc


TICK_MIN = 0
TICK_MAX = 4095
JOINT_COUNT = 6

# Same order as arm_params.yaml joint_names and LeRobot SO-101 motor ids 1..6.
JOINT_NAMES = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
LEROBOT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

# STS3215 half-turn written in arm_params.yaml. Not a measured calibration.
FACTORY_HOME_TICKS = (2048, 2048, 2048, 2048, 2048, 2048)

ENV_CALIBRATION = "OMNIBOT_ARM_CALIBRATION"
ENV_ROBOT_ID = "OMNIBOT_ARM_ROBOT_ID"
ENV_CALIBRATION_DIR = "OMNIBOT_ARM_CALIBRATION_DIR"

_ROBOT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


class CalibrationError(ValueError):
    """The calibration file is missing, unreadable, or not safe to apply."""


class HomeTicksResolution:
    """Ticks the driver should use, and whether they are the factory fallback."""

    def __init__(
        self, home_ticks, source, path, warning, used_factory_default, detail
    ):
        self.home_ticks = list(home_ticks)
        self.source = source
        self.path = path
        self.warning = warning
        self.used_factory_default = used_factory_default
        self.detail = detail


class MotorRecord:
    """One joint from a lerobot-calibrate JSON file."""

    def __init__(
        self,
        lerobot_name,
        joint_name,
        motor_id,
        drive_mode,
        homing_offset,
        range_min,
        range_max,
    ):
        self.lerobot_name = lerobot_name
        self.joint_name = joint_name
        self.motor_id = motor_id
        self.drive_mode = drive_mode
        self.homing_offset = homing_offset
        self.range_min = range_min
        self.range_max = range_max

    @property
    def home_tick(self):
        return (self.range_min + self.range_max) // 2


def factory_fallback_warning():
    """Banner printed when no calibration file was requested."""
    ticks = ", ".join(str(tick) for tick in FACTORY_HOME_TICKS)
    return (
        "\n"
        "****************************************************************\n"
        "WARNING: SO-101 home_ticks are the factory defaults\n"
        f"[{ticks}]. These are NOT this robot.\n"
        "Joint angle zero will be wrong until a calibration file is loaded.\n"
        "ros2 launch omnibot_arm arm.launch.py calibration_file:=<yaml>\n"
        "ros2 launch omnibot_arm arm.launch.py robot_id:=<id>\n"
        f"or set {ENV_CALIBRATION} to that yaml path.\n"
        "Procedure: omnibot-ros2/omnibot_arm/CALIBRATION.md\n"
        "****************************************************************\n"
    )


def validate_robot_id(robot_id):
    """Reject ids that could escape the calibration directory."""
    if not isinstance(robot_id, str) or _ROBOT_ID.fullmatch(robot_id) is None:
        raise CalibrationError(
            "robot_id must match [A-Za-z0-9][A-Za-z0-9_-]{0,63} "
            f"(got {robot_id!r})"
        )
    return robot_id


def validate_home_ticks(value, what="home_ticks"):
    """Require six integers, each in 0..4095. Bool is not an integer here."""
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise CalibrationError(
            f"{what} must be a list of {JOINT_COUNT} integers, "
            f"got {type(value).__name__}"
        )
    if len(value) != JOINT_COUNT:
        raise CalibrationError(
            f"{what} must contain {JOINT_COUNT} values, got {len(value)}"
        )
    ticks = []
    for index, item in enumerate(value):
        ticks.append(_require_tick(item, f"{what}[{index}]"))
    return ticks


def package_config_dir():
    """Installed share/config, or the source config directory in unit tests."""
    share = _share_directory()
    if share:
        return os.path.join(share, "config")
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))


def home_ticks_from_arm_params(path):
    """Read the factory (or hand-edited) home_ticks list out of arm_params.yaml."""
    data = _read_yaml(path)
    try:
        raw = data["arm_driver_node"]["ros__parameters"]["home_ticks"]
    except (TypeError, KeyError, AttributeError) as exc:
        raise CalibrationError(
            f"{path} has no arm_driver_node.ros__parameters.home_ticks"
        ) from exc
    return validate_home_ticks(raw, what=f"{path} home_ticks")


def parse_lerobot_json(payload):
    """Return six MotorRecords from a lerobot-calibrate JSON object.

    Current files are one object per joint (shoulder_pan ... gripper) with
    range_min and range_max. Older files use start_pos and end_pos arrays.
    """
    if not isinstance(payload, dict):
        raise CalibrationError(_schema_message())
    if _looks_named(payload):
        return _parse_named(payload)
    if "start_pos" in payload or "end_pos" in payload:
        return _parse_legacy(payload)
    raise CalibrationError(_schema_message())


def format_calibration_yaml(robot_id, motors):
    """YAML text for config/calibration/so101_<id>.yaml. Does not touch a robot."""
    robot_id = validate_robot_id(robot_id)
    if len(motors) != JOINT_COUNT:
        raise CalibrationError(f"expected {JOINT_COUNT} motors, got {len(motors)}")
    ticks = validate_home_ticks([motor.home_tick for motor in motors])
    lines = [
        "# OmniBot SO-101 home ticks converted from lerobot-calibrate JSON.",
        "# home_tick = (range_min + range_max) // 2.",
        "# That is LeRobot degree-mode zero in Present_Position counts.",
        "# homing_offset was written to the servo by lerobot-calibrate and is not added here.",
        "# This file is not a hardware check. Confirm the arm before driving it.",
        "robot_id: " + json.dumps(robot_id),
        "home_ticks: [" + ", ".join(str(tick) for tick in ticks) + "]",
        "joints:",
    ]
    for motor in motors:
        lines.append(f"  - lerobot_name: {motor.lerobot_name}")
        lines.append(f"    joint_name: {motor.joint_name}")
        lines.append(f"    id: {motor.motor_id}")
        lines.append(f"    drive_mode: {motor.drive_mode}")
        if motor.homing_offset is not None:
            lines.append(f"    homing_offset: {motor.homing_offset}")
        lines.append(f"    range_min: {motor.range_min}")
        lines.append(f"    range_max: {motor.range_max}")
        lines.append(f"    home_tick: {motor.home_tick}")
    return "\n".join(lines) + "\n"


def resolve_home_ticks(
    calibration_file=None,
    robot_id=None,
    config_dir=None,
    fallback=None,
    environ=None,
):
    """Pick home ticks from a launch arg, an env override, or a robot id.

    Precedence:
      1. calibration_file argument (arm.launch.py calibration_file:=)
      2. OMNIBOT_ARM_CALIBRATION (absolute or ~ path to a yaml file)
      3. robot_id argument, else OMNIBOT_ARM_ROBOT_ID, loading
         so101_<id>.yaml from OMNIBOT_ARM_CALIBRATION_DIR or
         <config_dir>/calibration
      4. fallback (arm_params.yaml). Warn when that fallback is the
         factory [2048] x 6 list.

    A requested file that is missing or invalid raises CalibrationError.
    It does not silently switch back to 2048.
    """
    env = os.environ if environ is None else environ
    if fallback is None:
        fallback_ticks = list(FACTORY_HOME_TICKS)
    else:
        fallback_ticks = validate_home_ticks(fallback, what="fallback home_ticks")

    explicit = _clean_text(calibration_file)
    if explicit:
        return _from_file(
            explicit,
            source="calibration_file",
            expected_robot_id=_clean_text(robot_id),
        )

    env_path = _clean_text(env.get(ENV_CALIBRATION))
    if env_path:
        expected = _clean_text(robot_id) or _clean_text(env.get(ENV_ROBOT_ID))
        return _from_file(
            env_path,
            source=ENV_CALIBRATION,
            expected_robot_id=expected,
        )

    requested_id = _clean_text(robot_id) or _clean_text(env.get(ENV_ROBOT_ID))
    if requested_id:
        directory = _clean_text(env.get(ENV_CALIBRATION_DIR))
        if directory is None:
            if not config_dir:
                raise CalibrationError(
                    "robot_id is set but config_dir is missing and "
                    f"{ENV_CALIBRATION_DIR} is unset"
                )
            directory = os.path.join(config_dir, "calibration")
        path = os.path.join(directory, f"so101_{validate_robot_id(requested_id)}.yaml")
        return _from_file(path, source="robot_id", expected_robot_id=requested_id)

    warning = None
    detail = (
        f"Using home_ticks {fallback_ticks} from the home_ticks parameter. "
        "No calibration file was requested."
    )
    used_factory = tuple(fallback_ticks) == FACTORY_HOME_TICKS
    if used_factory:
        warning = factory_fallback_warning()
        detail = ""
    return HomeTicksResolution(
        fallback_ticks,
        source="parameter",
        path=None,
        warning=warning,
        used_factory_default=used_factory,
        detail=detail,
    )


def home_ticks_for_node(
    home_ticks_param,
    calibration_file,
    robot_id,
    resolved_by_launch,
    config_dir,
    environ=None,
):
    """Node entry. Launch-resolved ticks are validated and not loaded again."""
    if _as_bool(resolved_by_launch):
        ticks = validate_home_ticks(home_ticks_param)
        return HomeTicksResolution(
            ticks,
            source="launch",
            path=_clean_text(calibration_file),
            warning=None,
            used_factory_default=False,
            detail="",
        )
    return resolve_home_ticks(
        calibration_file=calibration_file,
        robot_id=robot_id,
        config_dir=config_dir,
        fallback=home_ticks_param,
        environ=environ,
    )


def launch_parameter_overrides(resolution, robot_id):
    """Parameter dict arm.launch.py merges over arm_params.yaml."""
    requested = _clean_text(robot_id) or ""
    return {
        "home_ticks": list(resolution.home_ticks),
        "home_ticks_resolved": True,
        "calibration_file": resolution.path or "",
        "robot_id": requested,
    }


def _from_file(path, source, expected_robot_id):
    expanded = os.path.expandvars(os.path.expanduser(path))
    if not os.path.isfile(expanded):
        raise CalibrationError(
            f"Calibration file not found: {expanded}. Refusing to fall back "
            "to factory home_ticks because a file was requested."
        )
    data = _read_yaml(expanded)
    if not isinstance(data, dict):
        raise CalibrationError(f"{expanded} must be a YAML mapping")
    if "home_ticks" not in data:
        raise CalibrationError(f"{expanded} is missing home_ticks")
    ticks = validate_home_ticks(data["home_ticks"], what=f"{expanded} home_ticks")
    file_id = data.get("robot_id")
    if file_id is not None:
        file_id = validate_robot_id(file_id) if isinstance(file_id, str) else None
        if file_id is None:
            raise CalibrationError(f"{expanded} robot_id must be a string")
        if expected_robot_id and file_id != expected_robot_id:
            raise CalibrationError(
                f"{expanded} robot_id is {file_id!r}, expected {expected_robot_id!r}"
            )
    sign_names = _check_joints(data.get("joints"), ticks, expanded)
    warning = None
    if sign_names:
        warning = (
            "WARNING: drive_mode is not 0 for "
            + ", ".join(sign_names)
            + ". OmniBot does not flip joint sign. Confirm direction on the "
            "arm before moving it."
        )
    detail = f"Loaded SO-101 home_ticks {ticks} from {expanded} ({source})."
    return HomeTicksResolution(
        ticks,
        source=source,
        path=expanded,
        warning=warning,
        used_factory_default=False,
        detail=detail,
    )


def _check_joints(joints, ticks, path):
    """When joints are present, they must agree with home_ticks. Return sign flips."""
    if joints is None:
        return []
    if not isinstance(joints, list) or len(joints) != JOINT_COUNT:
        raise CalibrationError(
            f"{path} joints must be a list of {JOINT_COUNT} entries"
        )
    sign_names = []
    derived = []
    for index, joint in enumerate(joints):
        if not isinstance(joint, dict):
            raise CalibrationError(f"{path} joints[{index}] must be a mapping")
        if "home_tick" not in joint:
            raise CalibrationError(f"{path} joints[{index}] is missing home_tick")
        derived.append(_require_tick(joint["home_tick"], f"{path} joints[{index}].home_tick"))
        _check_optional_name(joint, "lerobot_name", LEROBOT_NAMES[index], path, index)
        _check_optional_name(joint, "joint_name", JOINT_NAMES[index], path, index)
        if "id" in joint:
            motor_id = _require_int(joint["id"], f"{path} joints[{index}].id")
            if motor_id != index + 1:
                raise CalibrationError(
                    f"{path} joints[{index}].id is {motor_id}, expected {index + 1}"
                )
        if "range_min" in joint or "range_max" in joint:
            lo = _require_endpoint(joint.get("range_min"), f"{path} joints[{index}].range_min")
            hi = _require_endpoint(joint.get("range_max"), f"{path} joints[{index}].range_max")
            if lo >= hi:
                raise CalibrationError(
                    f"{path} joints[{index}] range_min must be < range_max"
                )
            if (lo + hi) // 2 != derived[-1]:
                raise CalibrationError(
                    f"{path} joints[{index}] home_tick does not match "
                    "(range_min + range_max) // 2"
                )
        mode = joint.get("drive_mode", 0)
        mode = _require_int(mode, f"{path} joints[{index}].drive_mode")
        if mode not in (0, 1):
            raise CalibrationError(
                f"{path} joints[{index}].drive_mode must be 0 or 1"
            )
        if mode != 0:
            sign_names.append(JOINT_NAMES[index])
        if "homing_offset" in joint:
            _require_int(joint["homing_offset"], f"{path} joints[{index}].homing_offset")
    if derived != ticks:
        raise CalibrationError(
            f"{path} home_ticks does not match joints[].home_tick. "
            "Refusing to load mixed offsets."
        )
    return sign_names


def _check_optional_name(joint, key, expected, path, index):
    if key not in joint:
        return
    if joint[key] != expected:
        raise CalibrationError(
            f"{path} joints[{index}].{key} is {joint[key]!r}, expected {expected!r}"
        )


def _parse_named(payload):
    missing = [name for name in LEROBOT_NAMES if name not in payload]
    unknown = sorted(key for key in payload if key not in LEROBOT_NAMES)
    if missing or unknown:
        raise CalibrationError(
            "lerobot-calibrate JSON joint names must be exactly "
            + ", ".join(LEROBOT_NAMES)
            + f". missing={missing or '[]'} unknown={unknown or '[]'}"
        )
    motors = []
    for index, name in enumerate(LEROBOT_NAMES):
        entry = payload[name]
        if not isinstance(entry, Mapping):
            raise CalibrationError(f"{name} must be a JSON object")
        lo = _require_endpoint(entry.get("range_min"), f"{name}.range_min")
        hi = _require_endpoint(entry.get("range_max"), f"{name}.range_max")
        if lo >= hi:
            raise CalibrationError(f"{name} range_min must be < range_max")
        mode = _require_int(entry.get("drive_mode", 0), f"{name}.drive_mode")
        if mode not in (0, 1):
            raise CalibrationError(f"{name}.drive_mode must be 0 or 1")
        offset = entry.get("homing_offset")
        if offset is not None:
            offset = _require_int(offset, f"{name}.homing_offset")
        motor_id = _require_int(entry.get("id", index + 1), f"{name}.id")
        if motor_id != index + 1:
            raise CalibrationError(
                f"{name}.id is {motor_id}, expected {index + 1}"
            )
        motors.append(
            MotorRecord(
                lerobot_name=name,
                joint_name=JOINT_NAMES[index],
                motor_id=motor_id,
                drive_mode=mode,
                homing_offset=offset,
                range_min=lo,
                range_max=hi,
            )
        )
    return motors


def _parse_legacy(payload):
    if "start_pos" not in payload or "end_pos" not in payload:
        raise CalibrationError(
            "legacy lerobot calibration JSON needs both start_pos and end_pos"
        )
    start = _six_endpoints(payload["start_pos"], "start_pos")
    end = _six_endpoints(payload["end_pos"], "end_pos")
    offsets = payload.get("homing_offset")
    if offsets is None:
        offsets = [None] * JOINT_COUNT
    else:
        offsets = _six_ints(offsets, "homing_offset")
    modes = payload.get("drive_mode")
    if modes is None:
        modes = [0] * JOINT_COUNT
    else:
        modes = _six_ints(modes, "drive_mode")
    motors = []
    for index, name in enumerate(LEROBOT_NAMES):
        if start[index] >= end[index]:
            raise CalibrationError(f"{name} start_pos must be < end_pos")
        if modes[index] not in (0, 1):
            raise CalibrationError(f"drive_mode[{index}] must be 0 or 1")
        motors.append(
            MotorRecord(
                lerobot_name=name,
                joint_name=JOINT_NAMES[index],
                motor_id=index + 1,
                drive_mode=modes[index],
                homing_offset=offsets[index],
                range_min=start[index],
                range_max=end[index],
            )
        )
    return motors


def _six_endpoints(values, what):
    numbers = _six_ints(values, what)
    for index, number in enumerate(numbers):
        if number < TICK_MIN or number > TICK_MAX:
            raise CalibrationError(
                f"{what}[{index}]={number} is outside {TICK_MIN}..{TICK_MAX}"
            )
    return numbers


def _six_ints(values, what):
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise CalibrationError(f"{what} must be a list of {JOINT_COUNT} integers")
    if len(values) != JOINT_COUNT:
        raise CalibrationError(
            f"{what} must contain {JOINT_COUNT} values, got {len(values)}"
        )
    return [_require_int(item, f"{what}[{index}]") for index, item in enumerate(values)]


def _require_tick(value, what):
    number = _require_int(value, what)
    if number < TICK_MIN or number > TICK_MAX:
        raise CalibrationError(f"{what}={number} is outside {TICK_MIN}..{TICK_MAX}")
    return number


def _require_endpoint(value, what):
    if value is None:
        raise CalibrationError(f"{what} is missing")
    return _require_tick(value, what)


def _require_int(value, what):
    if isinstance(value, bool) or not isinstance(value, int):
        raise CalibrationError(f"{what} must be an integer, got {value!r}")
    return value


def _looks_named(payload):
    return any(name in payload for name in LEROBOT_NAMES)


def _schema_message():
    return (
        "JSON is not a lerobot-calibrate file. Expected one object per joint "
        "(" + ", ".join(LEROBOT_NAMES) + ") with range_min and range_max, "
        "or legacy start_pos and end_pos arrays of 6 integers."
    )


def _read_yaml(path):
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError as exc:
        raise CalibrationError(f"Cannot read {path}: {exc}") from exc
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CalibrationError(f"Invalid YAML in {path}: {exc}") from exc
    return data


def _clean_text(value):
    if value is None:
        return None
    if isinstance(value, os.PathLike):
        value = os.fspath(value)
    if not isinstance(value, str):
        raise CalibrationError(
            f"calibration path and robot_id must be strings, got {value!r}"
        )
    text = value.strip()
    return text or None


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes"):
            return True
        if text in ("0", "false", "no", ""):
            return False
    raise CalibrationError(f"home_ticks_resolved must be a boolean, got {value!r}")


def _share_directory():
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        return get_package_share_directory("omnibot_arm")
    except Exception:
        return None
