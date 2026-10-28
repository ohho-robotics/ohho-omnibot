# omnibot_hybrid

Hybrid Nav2 + VLA control: `cmd_vel_mux`, `teleop_safety`, `mission_planner`,
`rosbag_recorder`, `telemetry_aggregator`, and `vr_session_recorder`.

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

## VR session MCAP (OHH-239)

`vr_session_recorder` records one VR session to MCAP. It only subscribes. It does
not publish `/cmd_vel` or joint commands, and it does not change `teleop_safety`.

Topics, and nothing else:

- `/cmd_vel/teleop*`
- `/arm/joint_commands/teleop*`
- `/ohho/heartbeat`
- `/ohho/telemetry`
- `/ohho/safety/state`
- `/control_mode*`

`/ohho/vr_session/start` (`std_msgs/String`, empty id uses the time) starts a
session. `/ohho/vr_session/stop` (`std_msgs/Empty`) ends it. With
`auto_session:=true` (the default) the first `std_msgs/Empty` on `/ohho/heartbeat` starts a session
and `session_idle_sec` of silence (default 2 s) ends it. That idle timer is not
the 300 ms deadman.

Parts rotate into a new directory at `max_bag_size_bytes` (default 64 MiB).
Oldest part directories under the output dir are deleted once their bytes exceed
`max_total_bytes` (default 512 MiB). The part currently being written is kept.
`max_bag_size_bytes` cannot be larger than `max_total_bytes`.

```bash
ros2 launch omnibot_hybrid vr_session_recorder.launch.py
ros2 launch omnibot_bringup sim.launch.py vr:=true record_vr_session:=true
```

`record_vr_session` defaults to false, including on the `vr:=true` profile, so
sim-smoke does not record. OHH-198 `sim_vr.launch.py` includes `sim.launch.py`.
Forward the flag there and do not include `vr_session_recorder.launch.py` as
well:

```python
DeclareLaunchArgument("record_vr_session", default_value="false")
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "sim.launch.py")),
    launch_arguments={
        "vr": "true",
        "record_vr_session": LaunchConfiguration("record_vr_session"),
    }.items(),
)
```

Metrics for the Grafana dashboard are read-only, on `127.0.0.1:9188/metrics`:
`ohho_vr_rtt_ms`, `ohho_vr_heartbeat_gap_seconds`, `ohho_vr_heartbeat_gaps_total`,
`ohho_vr_safety_trips_total`. `link.rtt_ms` stays 0 until a telemetry sample
carries the client RTT. The dashboard is
`infra/observability/grafana/dashboards/vr-session.json`. OHH-64's Prometheus
and Grafana stack is not on this branch; put that file beside `robot-health.json`
when it is, and scrape `127.0.0.1:9188`. Foxglove layout:
`foxglove/vr_session.json`.

OHH-34's `record.launch.py` (not merged here) records a demo run (`/tf`, `/odom`,
`/scan`, cameras, `/agent/trace`) for the whole launch. This recorder is a
separate per-session bag. The MCAP storage flag and the Foxglove Studio layout
shape follow that pattern and were reimplemented for these topics.

A local topic check, when ROS 2 Jazzy is installed, publishes zero commands on
localhost domain 42 and does not start Gazebo:

```bash
python3 scripts/ohh239_sim_mcap.py
```

```bash
pytest omnibot-ros2/omnibot_hybrid/tests/test_vr_session_record.py -q
```
