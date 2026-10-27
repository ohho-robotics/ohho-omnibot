# omnibot_hybrid

Hybrid Nav2 + VLA control: `cmd_vel_mux`, `teleop_safety`, `mission_planner`,
`rosbag_recorder`, and `telemetry_aggregator`.

`teleop_safety` and the muxes are unchanged by the telemetry node. The
aggregator only reads topics and publishes a snapshot.

## Telemetry (OHH-208)

`telemetry_aggregator` publishes one `ohho.telemetry.v1` JSON sample on
`/ohho/telemetry` (`std_msgs/String`) at 20 Hz. QoS is reliable and
transient-local (depth 1) so a HUD that joins late still gets the last sample.

It also copies ohho.telemetry.v1 heartbeats from `/ohho/heartbeat`
(`std_msgs/String`) onto `/ohho/heartbeat_echo`, adding the server time `t`.
The client measures RTT from that echo. `seq` and `t_client` are copied
unchanged.

### How OHH-198 and OHH-220 include it

Do not paste this node into `sim.launch.py`, the `vr:=true` profile, or
`teleop_safety.launch.py`. Include this launch file from those profiles:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

telemetry = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("omnibot_hybrid"),
            "launch",
            "telemetry.launch.py",
        ])
    ),
    # OHH-198 sim VR profile:
    launch_arguments={"sim": "true"}.items(),
    # OHH-220 on the robot: omit sim, or pass sim:=false (the default).
)
```

```bash
ros2 launch omnibot_hybrid telemetry.launch.py sim:=true
ros2 launch omnibot_hybrid telemetry.launch.py sim:=false robot_id:=omnibot-1
ros2 topic hz /ohho/telemetry
```

`sim:=true` sets `kind` to `sim` and `battery.simulated` to true. With no pack
sample it reports 12.6 V at 100 percent. `sim:=false` omits `battery` until
`/battery` (`sensor_msgs/BatteryState`) or `/voltage` (`std_msgs/Float32`,
Yahboom pack voltage) arrives, and then sets `simulated` false. `use_sim_time`
stays false so the 20 Hz tick and `age_ms` stay on wall time.

Joint limits are read from `omnibot_arm` `config/arm_params.yaml` when that
package is installed. Otherwise the node uses the same default min/max pairs.

### Snapshot

The pure assembler is `omnibot_hybrid.telemetry_snapshot` (no rclpy). Optional
blocks are omitted until every required field is present. A zero that arrived
on the wire is kept. A missing velocity or effort array is not filled with
zero, so a position-only `/arm/joint_states` does not produce `joints` yet.
`arm_gripper` stays in `joints` (radian limits from `arm_params.yaml`). The
linear `gripper` block is only emitted when a metre opening is supplied.

`age_ms` is milliseconds since each cached field was received, so the HUD can
grey stale values without dropping them. Suggested grey-out hints are
`SUGGESTED_GREY_MS` in the assembler (250 ms for base, joints, and IMU; longer
for 1 Hz mode, safety, and battery). They are not watchdogs and they do not
stop the robot.

`age_ms` is not part of ohho.telemetry.v1 (`additionalProperties` is false).
`schema_body()` strips it before jsonschema. Generated SDK models will reject
the raw string until the schema grows an optional `age_ms` object. Strip it
before a strict parse, and keep it for the HUD.

`link.hz` is the JSON heartbeat rate over the last 2 s (0 when the stream is
quiet). `link.rtt_ms` stays 0 on the robot: the echo is what the client uses
to compute RTT, and the heartbeat schema has no field to report that number
back. `deadman` and `estop` are copied from `/ohho/safety/state`. When that
topic has not arrived, `deadman` is false. This node does not arm, latch, or
clear the e-stop.

Stock Nav2 `NavigateToPose` feedback has an ETA and a current pose, not the
goal pose. `nav` is omitted until a goal `{x, y, yaw}` is present together
with a status and an ETA. Recorder status `idle` and `recording:<name>`
without a frame count are omitted (`rec.frames` is required and is not
invented). `recording:<episode>:<frames>` and a JSON
`{active, episode, frames}` object are published.

### Heartbeat type

`teleop_safety` still subscribes to `std_msgs/Empty` on `/ohho/heartbeat` for
the deadman. This node subscribes to `std_msgs/String` JSON
`{v, seq, t_client}` on the same name for the RTT echo. ROS 2 will not
deliver one type to the other subscription. Publishing JSON does not feed the
deadman, and publishing Empty does not produce an echo.

### Tests

Schema fixtures are copied from ohho-sdk `ohh-201-telemetry-schema` at
`ae1909439092bceac667553ea7f5cfeaf506c4f6`
(`test/fixtures/ohho.telemetry.v1/SOURCE.txt`).

```bash
pytest omnibot-ros2/omnibot_hybrid/tests/test_telemetry_snapshot.py omnibot-ros2/omnibot_hybrid/tests/test_telemetry_schema.py -q
```

The 20 Hz test skips without ROS 2. With Jazzy sourced and this package on
`PYTHONPATH`:

```bash
pytest omnibot-ros2/omnibot_hybrid/tests/test_telemetry_hz.py -q
ros2 launch omnibot_hybrid telemetry.launch.py sim:=true
ros2 topic hz /ohho/telemetry
```
