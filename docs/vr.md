# VR topic contract

Shared by the real robot (`robot_vr.launch.py`, `mobile_manipulation.launch.py`, `omnibot_pi.launch.py`, `hybrid_robot.launch.py`) and the Gazebo VR profile. The machine-readable list is [`vr_topic_contract.yaml`](../omnibot-ros2/omnibot_bringup/config/vr_topic_contract.yaml). `sim_vr.launch.py` and `robot_vr.launch.py` both load it. Caps, the 300 ms heartbeat, and the e-stop latch live in [safety.md](safety.md). This page is the topic map those launches agree on.

```bash
ros2 launch omnibot_bringup sim_vr.launch.py
ros2 launch omnibot_bringup sim_vr.launch.py nav:=true
ros2 launch omnibot_bringup sim_teleop.launch.py world:=flat vr:=true
```

`sim_vr.launch.py` includes [`sim.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py) with `world:=apartment` and `vr:=true`. That is the Quest camera profile (wrist, BEV, chase, MJPEG). Inside the GPU container its rosbridge bind is `0.0.0.0` so Docker can publish `127.0.0.1`. It does not start the safety gate.

[`sim_teleop.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim_teleop.launch.py) is the teleop profile. It passes `bridge_config:=sim_bridge_vr.yaml`, so Gazebo's `/cmd_vel` is fed from `/cmd_vel/out`. A Twist on ROS `/cmd_vel` is the Nav2 source into the mux. It does not drive the simulated base while `/control_mode` is `teleop`.

On the robot, `yahboom_driver` reads `/cmd_vel/out` the same way. `teleop_safety`, `cmd_vel_mux`, and `arm_cmd_mux` are the same nodes, started once.

## Who publishes what

| Topic | Type | Publisher | Consumer | Forwarded when |
|---|---|---|---|---|
| `/cmd_vel/teleop_raw` | `geometry_msgs/Twist` | Quest, Android, browser | `teleop_safety` | gated, then republished |
| `/arm/joint_commands/teleop_raw` | `sensor_msgs/JointState` | Quest, Android, browser | `teleop_safety` | gated, then republished |
| `/ohho/heartbeat` | `std_msgs/Empty` | client, 10 Hz | `teleop_safety` | arms the 300 ms watchdog |
| `/emergency_stop` | `std_msgs/Bool` | client | `teleop_safety`, both muxes, base driver, arm driver | `true` zeros `/cmd_vel/out` and holds every arm source |
| `/ohho/estop_reset` | `std_srvs/Trigger` | client | `teleop_safety` | clears the latch only while the stop is false |
| `/ohho/safety/state` | `std_msgs/String` JSON | `teleop_safety` | clients | latched state |
| `/cmd_vel/teleop` | `geometry_msgs/Twist` | `teleop_safety` | `cmd_vel_mux` | `/control_mode` is `teleop` and e-stop is false |
| `/arm/joint_commands/teleop` | `sensor_msgs/JointState` | `teleop_safety` | `arm_cmd_mux` | `/control_mode` is `teleop` and e-stop is false |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 `velocity_smoother` | `cmd_vel_mux` | mode `nav2` |
| `/cmd_vel/vla` | `geometry_msgs/Twist` | VLA node | `cmd_vel_mux` | mode `vla` |
| `/cmd_vel/rl` | `geometry_msgs/Twist` | RL nav | `cmd_vel_mux` | mode `rl_nav` |
| `/cmd_vel/out` | `geometry_msgs/Twist` | `cmd_vel_mux` | robot driver; Gazebo via the VR bridge | the selected source |
| `/control_mode` | `std_msgs/String` | client or mission planner | both muxes | `teleop`, `nav2`, `vla`, `rl_nav` |
| `/control_mode/active` | `std_msgs/String` | `cmd_vel_mux` | clients | the mode in effect |
| `/arm/joint_commands/policy` | `sensor_msgs/JointState` | `policy_node` | `arm_cmd_mux` | `vla` or `nav2`, arm source `policy`, e-stop clear |
| `/arm/joint_commands` | `sensor_msgs/JointState` | deprecated alias of the policy input | `arm_cmd_mux` | same gate as the policy topic |
| `/arm/joint_commands/rl` | `sensor_msgs/JointState` | RL arm | `arm_cmd_mux` | `rl_nav`, arm source `rl_arm`, e-stop clear |
| `/arm/joint_commands/out` | `sensor_msgs/JointState` | `arm_cmd_mux` | arm driver | the selected arm source |
| `/arm/cmd_mode` | `std_msgs/String` | client | `arm_cmd_mux` | `policy` or `rl_arm` after leaving teleop |
| `/arm/cmd_mode/active` | `std_msgs/String` | `arm_cmd_mux` | clients | arm source preference |
| `/odom` | `nav_msgs/Odometry` | Gazebo bridge or the base | Nav2, clients | |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo bridge or the depth scan | Nav2, clients | |
| `/imu` | `sensor_msgs/Imu` | Gazebo bridge or the base | clients | |
| `/camera/front/image_raw` | `sensor_msgs/Image` | Gazebo bridge or the camera | MJPEG, clients | |
| `/camera/front/camera_info` | `sensor_msgs/CameraInfo` | Gazebo bridge or the camera | clients | |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo bridge or robot_state_publisher | Nav2 | `odom` to `base_footprint` in sim |
| `/imu/data` | `sensor_msgs/Imu` | base driver | EKF, telemetry | robot IMU |
| `/camera/wrist/image_raw` | `sensor_msgs/Image` | wrist camera | MJPEG, clients | |
| `/camera/wrist/camera_info` | `sensor_msgs/CameraInfo` | wrist camera | clients | |
| `/camera/rear/image_raw` | `sensor_msgs/Image` | rear camera | BEV stitcher | |
| `/camera/left/image_raw` | `sensor_msgs/Image` | left camera | BEV stitcher | |
| `/camera/right/image_raw` | `sensor_msgs/Image` | right camera | BEV stitcher | |
| `/camera/base/bev/image_raw` | `sensor_msgs/Image` | BEV stitcher | MJPEG, clients | |
| `/ohho/telemetry` | `std_msgs/String` | `telemetry_aggregator` | VR HUD | 20 Hz, `sim:=false` on the robot |
| `/ohho/descriptor` | `std_msgs/String` | `ohho_descriptor` | clients | latched JSON when OHH-214 is installed |

Startup mode is `teleop`. Switching `/control_mode` publishes one zero Twist on `/cmd_vel/out` in that callback. A stale active source is zeroed by the mux watchdog (0.3 s, wall clock) described in [safety.md](safety.md).

Publishing `/cmd_vel/teleop` moves the simulated base only in teleop. Publishing `/emergency_stop` true stops it. The headless check is [`test_sim_vr_contract.py`](../omnibot-ros2/omnibot_bringup/test/test_sim_vr_contract.py). It launches `sim_teleop.launch.py` with `vr:=true` and `world:=flat` so the drive is clear of the apartment furniture. The `sim-smoke` job runs it. [`test_sim_vr_launch.py`](../omnibot-ros2/omnibot_bringup/test/test_sim_vr_launch.py) is the CPU check that the GPU profile still forces the Quest cameras and does not start the gate.

## rosbridge and video

| Port | Process | Bind |
|---|---|---|
| 9090 | `rosbridge_websocket` | `rosbridge_address`. `sim.launch.py` and `sim_teleop.launch.py` default to `127.0.0.1`. `sim_vr.launch.py` defaults to `0.0.0.0` for the GPU container; the compose file publishes that port on `127.0.0.1` only. |
| 8080 | `web_video_server` | the same address |

MJPEG for the Quest view:

```text
http://127.0.0.1:8080/stream?topic=/camera/front/image_raw
```

`gateway:=false` is the default. `rosbridge_address:=0.0.0.0` is an explicit opt-in and is unauthenticated. `gateway:=true` keeps rosbridge and web_video_server on `127.0.0.1` and does not bind `0.0.0.0`. When `omnibot_bringup/launch/ohho_gateway.launch.py` is installed, that flag includes it and passes `rosbridge:=false` into `sim.launch.py` so port 9090 is claimed once. Until that file is in this tree, `gateway:=true` logs the missing snippet and still binds the upstreams to loopback.

## Optional pieces

| Arg | Default | What it adds |
|---|---|---|
| `nav:=true` | `false` | map server, AMCL, and Nav2 on the apartment map. No second Gazebo. Needs `omnibot_navigation`. |
| `arm:=true` | `false` | include `omnibot_arm/launch/sim_arm.launch.py`, which mirrors `/arm/joint_commands/out` onto `/arm/joint_states`. That include does not start `arm_sim_driver`. `sim.launch.py` is still passed `arm:=true`, so `arm_sim_driver` starts there. |
| `cameras:=extra` | `front` | include point `sim_cameras.launch.py` (OHH-200). `front` is the camera `sim.launch.py` already bridges. |
| `telemetry:=true` | `false` | include point `telemetry_aggregator.launch.py`. The aggregator is not in this tree. |

Asking for a missing include point logs the path and the rest of the profile still starts.

## CI contract

No Quest is in CI. `tests/vr_contract/virtual_headset.py` connects to rosbridge and does what the headset does: set `/control_mode` to `teleop`, publish `/ohho/heartbeat`, call `/ohho/estop_reset` to clear the startup latch, drive `/cmd_vel/teleop_raw` one metre forward, strafe, move three arm joints on `/arm/joint_commands/teleop_raw`, latch `/emergency_stop`, reset through `/ohho/estop_reset`, then stop the heartbeat. It asserts `/odom`, `/arm/joint_states`, and `/ohho/safety/state`.

The `vr-contract` workflow runs that client against `sim_teleop.launch.py world:=flat arm:=true` (the safety gate and `sim_bridge_vr.yaml`) and starts `arm_joint_state_mirror` so `/arm/joint_states` follows `/arm/joint_commands/out`. It uploads the launch log plus an MCAP. Gazebo's `/cmd_vel` is fed from `/cmd_vel/out`. Pointing the bridge at `/cmd_vel` leaves the base still, so the job fails. `sim_vr.launch.py` stays the camera profile and does not start the gate.

## Real robot

One command on the Pi. It does not start SLAM or Nav2. Depth stays off unless `depth_camera:=true`. Wheels stay on blocks until Varun has run the contract check.

```bash
ros2 launch omnibot_bringup robot_vr.launch.py
```

`teleop_safety` is included with no arguments and starts before `cmd_vel_mux` and `arm_cmd_mux`. Both muxes start in `teleop` with `use_sim_time` false. The base driver remaps `cmd_vel` to `/cmd_vel/out`. The arm driver remaps `joint_commands` to `/arm/joint_commands/out`. Quest, Android, and the browser publish `/cmd_vel/teleop_raw` and `/arm/joint_commands/teleop_raw`. This profile does not start `joy_node` or `teleop_twist_joy`.

`gateway:=true` is the default. When `omnibot_bringup/launch/ohho_gateway.launch.py` is installed, that snippet owns rosbridge, foxglove, and `web_video_server`, and this file does not start a second copy. Until that file is in the tree, the same three processes bind `127.0.0.1`. A `rosbridge_address` other than loopback is clamped to `127.0.0.1`. This profile does not bind rosbridge to `0.0.0.0`.

MJPEG, all three contract streams, from the loopback server:

```text
http://127.0.0.1:8080/stream?topic=/camera/front/image_raw
http://127.0.0.1:8080/stream?topic=/camera/wrist/image_raw
http://127.0.0.1:8080/stream?topic=/camera/base/bev/image_raw
```

`video_backend:=webrtc` includes `webrtc_streamer.launch.py` when that file is installed (OHH-210). Otherwise `web_video_server` still serves the three topics. `telemetry:=true` includes `omnibot_hybrid/launch/telemetry.launch.py` with `sim:=false`. The sim profile's include point remains `telemetry_aggregator.launch.py`, which is still not in this tree. `descriptor:=true` starts `descriptor_node.py` only when that executable and `robot_descriptor.yaml` are installed.

`virtual_headset.py` is not in this repository. Varun runs it against the robot on blocks.

## systemd

[`omnibot-robot-vr.service`](../omnibot-ros2/omnibot_bringup/config/systemd/omnibot-robot-vr.service) is the headless boot unit. Installing the ROS package does not copy it into `/etc/systemd/system` and does not enable it. Nothing in this repository runs `systemctl`.

On the Pi, after the on-blocks check, edit `User` and the `setup.bash` paths, then:

```bash
sudo cp omnibot-ros2/omnibot_bringup/config/systemd/omnibot-robot-vr.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now omnibot-robot-vr.service
```

Keep `OHHO_GATEWAY_TOKEN` in `/etc/omnibot/robot-vr.env`. That file is not in git.

## Pi CPU budget

Capture this on the Raspberry Pi while `robot_vr.launch.py` is running and a client is reading front, wrist, and BEV. Use `top` and `htop`. Leave the cells blank. Do not invent measurements. This table has no results yet.

```bash
top -b -n 30 -d 2
htop
```

Confirm the streams while the capture is running:

```bash
ros2 topic hz /camera/front/image_raw
ros2 topic hz /camera/wrist/image_raw
ros2 topic hz /camera/base/bev/image_raw
ros2 topic hz /ohho/telemetry
```

| Stream | Process | %CPU | %MEM |
|---|---|---|---|
| front | | | |
| wrist | | | |
| BEV | | | |
| rosbridge_websocket | | | |
| web_video_server | | | |
| telemetry_aggregator | | | |
| teleop_safety | | | |
| whole profile | | | |
