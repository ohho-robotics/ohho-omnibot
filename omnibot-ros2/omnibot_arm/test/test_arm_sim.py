"""Gazebo arm sim: safety reuse, yaml limits, and the scripted command path.

No ROS graph and no Gazebo. ArmSimCommander is pure Python. A second group
stubs rclpy only long enough to import arm_sim_driver_node and check that
the node writes position targets, echoes joint names, and freezes on e-stop.
"""

import os
import re
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, _SCRIPTS)

import arm_safety  # noqa: E402 — scripts path must come first
import arm_sim_logic  # noqa: E402

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_ARM_PARAMS = os.path.join(_SCRIPTS, "..", "config", "arm_params.yaml")
_XACRO = os.path.join(
    _REPO, "omnibot-ros2", "omnibot_description", "urdf", "omnibot.urdf.xacro"
)
_BRIDGE = os.path.join(
    _REPO, "omnibot-ros2", "omnibot_bringup", "config", "sim_bridge.yaml"
)
_LAUNCH = os.path.join(
    _REPO, "omnibot-ros2", "omnibot_bringup", "launch", "sim.launch.py"
)
_APARTMENT = os.path.join(
    _REPO, "omnibot-ros2", "omnibot_bringup", "worlds", "apartment.sdf"
)
_SIM_TEST = os.path.join(
    _REPO, "omnibot-ros2", "omnibot_bringup", "test", "test_sim_launch.py"
)

# SO-101 calibration stops already in the URDF. Software limits in
# arm_params.yaml are wider on some joints. These must not be raised.
_CALIB_LIMITS = {
    "arm_shoulder_pan": (-1.91986, 1.91986),
    "arm_shoulder_lift": (-1.74533, 1.74533),
    "arm_elbow_flex": (-1.69, 1.69),
    "arm_wrist_flex": (-1.65806, 1.65806),
    "arm_wrist_roll": (-2.74385, 2.84121),
    "arm_gripper": (-0.174533, 1.74533),
}


def _params():
    return arm_sim_logic.load_arm_driver_params(_ARM_PARAMS)


def _commander():
    return arm_sim_logic.ArmSimCommander.from_driver_params(_params())


def _urdf_revolute_limits(text):
    limits = {}
    pattern = re.compile(
        r'<joint name="([^"]+)" type="revolute">(.*?)</joint>',
        re.S,
    )
    for match in pattern.finditer(text):
        name, body = match.group(1), match.group(2)
        limit = re.search(r'lower="([^"]+)" upper="([^"]+)"', body)
        if limit:
            limits[name] = (float(limit.group(1)), float(limit.group(2)))
    return limits


class TestLimitsMatchArmParams:
    def test_loader_reads_the_yaml_block(self):
        with open(_ARM_PARAMS, encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)["arm_driver_node"]["ros__parameters"]
        params = _params()
        assert list(params["joint_names"]) == list(raw["joint_names"])
        assert list(params["joint_min"]) == pytest.approx(list(raw["joint_min"]))
        assert list(params["joint_max"]) == pytest.approx(list(raw["joint_max"]))
        assert float(params["max_joint_delta_rad"]) == float(raw["max_joint_delta_rad"])
        assert float(params["command_timeout_sec"]) == float(raw["command_timeout_sec"])

    def test_fallback_matches_yaml(self):
        params = _params()
        fallback = arm_sim_logic.fallback_arm_params()
        assert list(params["joint_names"]) == fallback["joint_names"]
        assert list(params["joint_min"]) == pytest.approx(fallback["joint_min"])
        assert list(params["joint_max"]) == pytest.approx(fallback["joint_max"])
        assert float(params["max_joint_delta_rad"]) == fallback["max_joint_delta_rad"]
        assert float(params["command_timeout_sec"]) == fallback["command_timeout_sec"]

    def test_commander_uses_yaml_and_safety_constants(self):
        params = _params()
        commander = _commander()
        assert commander.joint_names == list(params["joint_names"])
        assert commander.safety.joint_min == pytest.approx(
            [float(v) for v in params["joint_min"]]
        )
        assert commander.safety.joint_max == pytest.approx(
            [float(v) for v in params["joint_max"]]
        )
        assert commander.safety.max_joint_delta_rad == arm_safety.MAX_JOINT_DELTA_RAD
        assert commander.safety.command_timeout_sec == arm_safety.COMMAND_TIMEOUT_SEC
        assert commander.safety.max_joint_delta_rad == 0.15
        assert commander.safety.command_timeout_sec == 0.2

    def test_gripper_software_limit_stays_tighter_than_urdf(self):
        text = open(_XACRO, encoding="utf-8").read()
        urdf = _urdf_revolute_limits(text)
        commander = _commander()
        gripper = commander.joint_names.index("arm_gripper")
        assert commander.safety.joint_max[gripper] == pytest.approx(0.8)
        assert commander.safety.joint_min[gripper] == pytest.approx(-0.1)
        assert commander.safety.joint_max[gripper] < urdf["arm_gripper"][1]


class TestUrdfStopsNotWidened:
    def test_calib_limits_unchanged(self):
        text = open(_XACRO, encoding="utf-8").read()
        urdf = _urdf_revolute_limits(text)
        for name, (lower, upper) in _CALIB_LIMITS.items():
            assert urdf[name][0] == pytest.approx(lower, abs=1e-5)
            assert urdf[name][1] == pytest.approx(upper, abs=1e-5)

    def test_shoulder_pan_urdf_stays_tighter_than_yaml(self):
        text = open(_XACRO, encoding="utf-8").read()
        urdf = _urdf_revolute_limits(text)
        params = _params()
        pan = list(params["joint_names"]).index("arm_shoulder_pan")
        assert urdf["arm_shoulder_pan"][1] < float(params["joint_max"][pan])
        assert urdf["arm_shoulder_pan"][0] > float(params["joint_min"][pan])


class TestScriptedCommand:
    def test_each_joint_reaches_target_under_ideal_tracking(self):
        commander = _commander()
        reached, measured, target = arm_sim_logic.run_scripted_sequence(commander)
        assert reached
        assert arm_sim_logic.positions_within(
            measured, target, arm_sim_logic.POSITION_TOLERANCE_RAD
        )
        assert arm_sim_logic.positions_within(
            measured, arm_sim_logic.SCRIPTED_TARGETS, arm_sim_logic.POSITION_TOLERANCE_RAD
        )
        names = commander.joint_names
        state_names, state_pos = names, commander.measured
        assert state_names == list(_params()["joint_names"])
        assert state_pos == pytest.approx(measured)

    def test_scripted_targets_fit_yaml_and_urdf(self):
        params = _params()
        text = open(_XACRO, encoding="utf-8").read()
        urdf = _urdf_revolute_limits(text)
        names = list(params["joint_names"])
        assert len(names) == len(arm_sim_logic.SCRIPTED_TARGETS)
        for name, goal, lower, upper in zip(
            names,
            arm_sim_logic.SCRIPTED_TARGETS,
            params["joint_min"],
            params["joint_max"],
        ):
            assert float(lower) <= goal <= float(upper)
            ulo, uhi = urdf[name]
            assert ulo <= goal <= uhi

    def test_step_never_exceeds_delta(self):
        commander = _commander()
        prev = [0.0] * len(commander.joint_names)
        now = 0.0
        target = list(arm_sim_logic.SCRIPTED_TARGETS)
        for _ in range(30):
            written = commander.on_command(commander.joint_names, target, now)
            assert written is not None
            for actual, before in zip(written, prev):
                assert abs(actual - before) <= arm_safety.MAX_JOINT_DELTA_RAD + 1e-9
            prev = list(written)
            commander.on_feedback(commander.joint_names, written)
            now += 0.05

    def test_over_limit_clamps_to_yaml_max(self):
        commander = _commander()
        target = [99.0] * len(commander.joint_names)
        now = 0.0
        written = None
        for _ in range(40):
            written = commander.on_command(commander.joint_names, target, now)
            assert written is not None
            commander.on_feedback(commander.joint_names, written)
            now += 0.05
        assert written == pytest.approx(commander.safety.joint_max)

    def test_estop_holds_and_rejects_until_reenabled(self):
        commander = _commander()
        names = commander.joint_names
        written = commander.on_command(names, list(arm_sim_logic.SCRIPTED_TARGETS), 0.0)
        assert written is not None
        commander.on_feedback(names, written)
        held = list(commander.measured)

        action, hold = commander.on_emergency_stop(True)
        assert action.value == 0
        assert hold == pytest.approx(held)
        assert commander.on_command(names, [0.0] * len(names), 1.0) is None
        assert commander.measured == pytest.approx(held)

        enable_action, enable_written = commander.on_arm_enable(True, 1.1)
        assert enable_written is None
        assert enable_action.value == 0
        assert commander.safety.torque_enabled is False

        clear_action, clear_hold = commander.on_emergency_stop(False)
        assert clear_hold is None
        assert clear_action.value == 0
        assert commander.safety.torque_enabled is False
        assert commander.on_command(names, list(arm_sim_logic.SCRIPTED_TARGETS), 2.0) is None

        reenable_action, reenable_written = commander.on_arm_enable(True, 2.1)
        assert reenable_action.value == 1
        assert reenable_written == pytest.approx(held)
        assert commander.safety.torque_enabled is True

    def test_timeout_hold_rewrites_once_and_not_during_estop(self):
        commander = _commander()
        names = commander.joint_names
        written = commander.on_command(names, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 10.0)
        assert written is not None
        assert commander.hold_if_stale(10.05).rewrite is False
        holding = commander.hold_if_stale(10.25)
        assert holding.rewrite is True
        assert holding.positions == pytest.approx(written)
        assert "holding last command" in holding.message
        assert commander.hold_if_stale(10.5).rewrite is False

        commander.on_emergency_stop(True)
        stale = commander.hold_if_stale(20.0)
        assert stale.rewrite is False
        assert stale.positions is None

    def test_missing_name_defaults_to_zero(self):
        commander = _commander()
        written = commander.on_command(["arm_shoulder_pan"], [0.1], 0.0)
        assert written[0] == pytest.approx(0.1)
        assert written[1:] == pytest.approx([0.0] * 5)


class TestGazeboWiring:
    def test_bridge_and_urdf_topics(self):
        bridge = yaml.safe_load(open(_BRIDGE, encoding="utf-8").read())
        by_topic = {entry["ros_topic_name"]: entry for entry in bridge}
        xacro = open(_XACRO, encoding="utf-8").read()
        assert "/arm/sim/${joint}/cmd_pos" in xacro
        assert "gz-sim-joint-position-controller-system" in xacro
        assert "<use_velocity_commands>true</use_velocity_commands>" in xacro
        assert "<cmd_max>1.5</cmd_max>" in xacro
        assert "<initial_position>0</initial_position>" in xacro
        for name in _params()["joint_names"]:
            topic = arm_sim_logic.command_topic(name)
            entry = by_topic[topic]
            assert entry["gz_topic_name"] == topic
            assert entry["ros_type_name"] == "std_msgs/msg/Float64"
            assert entry["gz_type_name"] == "gz.msgs.Double"
            assert entry["direction"] == "ROS_TO_GZ"
            assert f'joint="{name}"' in xacro
            assert f"<joint_name>{name}</joint_name>" in xacro
        feedback = by_topic[arm_sim_logic.FEEDBACK_TOPIC]
        assert feedback["direction"] == "GZ_TO_ROS"
        assert feedback["ros_type_name"] == "sensor_msgs/msg/JointState"
        assert feedback["gz_type_name"] == "gz.msgs.Model"
        assert arm_sim_logic.FEEDBACK_TOPIC in xacro

    def test_launch_starts_sim_driver_with_arm_params(self):
        text = open(_LAUNCH, encoding="utf-8").read()
        assert 'executable="arm_sim_driver_node.py"' in text
        assert "arm_params.yaml" in text
        assert "arm_params_file" in text
        sim_test = open(_SIM_TEST, encoding="utf-8").read()
        assert "arm_sim_scripted_move.py" in sim_test

    def test_apartment_has_a_dynamic_grasp_cube(self):
        root = ET.parse(_APARTMENT).getroot()
        cube = None
        for model in root.findall("world/model"):
            if model.get("name") == "grasp_cube":
                cube = model
                break
        assert cube is not None
        static = cube.find("static")
        assert static is None or static.text.strip().lower() == "false"
        mass = float(cube.find("link/inertial/mass").text)
        assert 0.01 <= mass <= 0.2
        assert cube.find("link/collision") is not None
        assert cube.find("link/visual") is not None
        pose = [float(part) for part in cube.find("pose").text.split()]
        # Desk top is z=0.76, footprint x in [1.95, 3.15], y in [0.10, 0.70].
        assert 1.95 < pose[0] < 3.15
        assert 0.10 < pose[1] < 0.70
        assert 0.76 < pose[2] < 0.90


# ---------------------------------------------------------------------------
# Node wiring (stubbed rclpy)
# ---------------------------------------------------------------------------


def _install_ros_stubs():
    import types
    from unittest.mock import MagicMock

    saved = {}

    def _module(name):
        saved[name] = sys.modules.get(name)
        module = types.ModuleType(name)
        sys.modules[name] = module
        return module

    class _Param:
        def __init__(self, value):
            self.value = value

    class _Logger:
        def __init__(self):
            self.warnings = []
            self.infos = []
            self.errors = []

        def warn(self, message, **_kwargs):
            self.warnings.append(message)

        def info(self, message, **_kwargs):
            self.infos.append(message)

        def error(self, message, **_kwargs):
            self.errors.append(message)

    class _Node:
        def __init__(self, name):
            self._name = name
            self._params = {}
            self._logger = _Logger()
            self._subs = []
            self._pubs = []
            self._timers = []

        def declare_parameter(self, name, default=None):
            self._params[name] = default

        def get_parameter(self, name):
            return _Param(self._params[name])

        def get_logger(self):
            return self._logger

        def get_clock(self):
            clock = MagicMock()
            clock.now.return_value.to_msg.return_value = object()
            return clock

        def create_publisher(self, _msg_type, topic, _qos):
            pub = MagicMock()
            self._pubs.append((topic, pub))
            return pub

        def create_subscription(self, _msg_type, topic, callback, _qos):
            self._subs.append((topic, callback))
            return MagicMock()

        def create_timer(self, period, callback):
            self._timers.append((period, callback))
            return MagicMock()

        def destroy_node(self):
            return None

    class JointState:
        def __init__(self):
            self.name = []
            self.position = []
            self.header = types.SimpleNamespace(stamp=None)

    class Bool:
        def __init__(self):
            self.data = False

    class Float64:
        def __init__(self):
            self.data = 0.0

    rclpy = _module("rclpy")
    rclpy_node = _module("rclpy.node")
    rclpy.node = rclpy_node
    rclpy_node.Node = _Node
    rclpy.init = lambda *_a, **_k: None
    rclpy.shutdown = lambda *_a, **_k: None
    rclpy.spin = lambda *_a, **_k: None

    sensor_msgs = _module("sensor_msgs")
    sensor_msgs_msg = _module("sensor_msgs.msg")
    sensor_msgs.msg = sensor_msgs_msg
    sensor_msgs_msg.JointState = JointState

    std_msgs = _module("std_msgs")
    std_msgs_msg = _module("std_msgs.msg")
    std_msgs.msg = std_msgs_msg
    std_msgs_msg.Bool = Bool
    std_msgs_msg.Float64 = Float64

    try:
        import arm_sim_driver_node  # noqa: E402
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
    return arm_sim_driver_node


arm_sim_driver_node = _install_ros_stubs()


def _node():
    return arm_sim_driver_node.ArmSimDriverNode()


def _bool(value):
    msg = arm_sim_driver_node.Bool()
    msg.data = value
    return msg


def _joints(names, positions):
    msg = arm_sim_driver_node.JointState()
    msg.name = list(names)
    msg.position = list(positions)
    return msg


class TestSimDriverNode:
    def test_loads_yaml_limits_and_topics(self):
        node = _node()
        params = _params()
        assert node.params_path.endswith("arm_params.yaml")
        assert node.joint_names == list(params["joint_names"])
        assert node.joint_max == pytest.approx([float(v) for v in params["joint_max"]])
        assert node._commander.safety.max_joint_delta_rad == 0.15
        topics = [topic for topic, _callback in node._subs]
        assert "/arm/joint_commands/out" in topics
        assert "/emergency_stop" in topics
        assert "/arm/enable" in topics
        assert arm_sim_logic.FEEDBACK_TOPIC in topics
        published = [topic for topic, _pub in node._pubs]
        assert "/arm/joint_states" in published
        for name in node.joint_names:
            assert arm_sim_logic.command_topic(name) in published

    def test_command_publishes_delta_limited_targets(self):
        node = _node()
        node.joint_command_cb(
            _joints(node.joint_names, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        )
        published = []
        for _topic, pub in node._pubs:
            if pub.publish.called and hasattr(pub.publish.call_args[0][0], "data"):
                published.append(pub.publish.call_args[0][0].data)
        assert published[0] == pytest.approx(0.15)
        assert published[1:] == pytest.approx([0.0] * 5)

    def test_feedback_echoes_real_joint_names(self):
        node = _node()
        positions = [0.2, -0.1, 0.1, 0.0, -0.2, 0.3]
        node.feedback_cb(_joints(node.joint_names, positions))
        state_pub = node._pubs[0][1]
        msg = state_pub.publish.call_args[0][0]
        assert list(msg.name) == node.joint_names
        assert list(msg.position) == pytest.approx(positions)

    def test_estop_freezes_and_rejects(self):
        node = _node()
        node.feedback_cb(_joints(node.joint_names, [0.2, 0.0, 0.0, 0.0, 0.0, 0.2]))
        for _topic, pub in node._pubs[1:]:
            pub.publish.reset_mock()
        node.emergency_stop_cb(_bool(True))
        frozen = []
        for _topic, pub in node._pubs[1:]:
            assert pub.publish.called
            frozen.append(pub.publish.call_args[0][0].data)
            pub.publish.reset_mock()
        assert frozen == pytest.approx([0.2, 0.0, 0.0, 0.0, 0.0, 0.2])
        node.joint_command_cb(_joints(node.joint_names, [1.0] * 6))
        for _topic, pub in node._pubs[1:]:
            pub.publish.assert_not_called()

    def test_timeout_republishes_last_target(self, monkeypatch):
        clock = {"now": 10.0}
        monkeypatch.setattr(arm_sim_driver_node.time, "monotonic", lambda: clock["now"])
        node = _node()
        node.joint_command_cb(_joints(node.joint_names, [0.0] * 6))
        for _topic, pub in node._pubs[1:]:
            pub.publish.reset_mock()
        node.get_logger().warnings.clear()
        clock["now"] = 10.25
        node._on_timer()
        assert any("holding last command" in message for message in node.get_logger().warnings)
        for _topic, pub in node._pubs[1:]:
            assert pub.publish.called
            assert pub.publish.call_args[0][0].data == pytest.approx(0.0)
