# OmniBot reference

Detailed notes moved from the top-level README. Back to [README](../README.md).

## Needs hardware / needs GPU

| Feature | Needs | Evidence |
|---|---|---|
| `ohho sim --robot omnibot` | Neither | [ohho-os 1.1.2](https://pypi.org/project/ohho-os/1.1.2/) and the docs-test job in [ci.yml](../.github/workflows/ci.yml) |
| Colcon build and test of `src/` | Neither | [src/](../src/) and the `ros` job in [ci.yml](../.github/workflows/ci.yml). That job needs ROS 2 Jazzy. |
| Kinematics, Yahboom codec, BEV homography, arm tick math, SO-101 home-tick file loading, arm command safety (joint limits, 0.15 rad/cycle, 200 ms hold, e-stop torque latch), policy inference guards (black/missing frame refusal, latency drop, 0.15 rad/cycle arm clamp), learning engine, agent engine | Neither | The `cpu` job in [ci.yml](../.github/workflows/ci.yml) runs the CPU pytest/unittest suites and reports line coverage ([test-coverage.md](test-coverage.md)). Those tests do not start ROS or the arm. |
| Xbox teleop, cameras, arm, LeRobot recording | Robot | [robot_with_joy.launch.py](../omnibot-ros2/omnibot_bringup/launch/robot_with_joy.launch.py), [perception.launch.py](../omnibot-ros2/omnibot_bringup/launch/perception.launch.py), [arm.launch.py](../omnibot-ros2/omnibot_arm/launch/arm.launch.py), [lerobot_record.launch.py](../omnibot-ai-ros2/omnibot_lerobot/launch/lerobot_record.launch.py) (gamepad + wrist/BEV + optional leader arm), [teleop_record.launch.py](../omnibot-ai-ros2/omnibot_lerobot/launch/teleop_record.launch.py). Collection steps and the blank Hub card: [demo-collection-checklist.md](demo-collection-checklist.md), [omnibot-pick-place dataset card](datasets/omnibot-pick-place/README.md). |
| Nav2 and SLAM on the real robot | Robot | [omnibot_navigation](../omnibot-ros2/omnibot_navigation/). Not run on hardware for this change. |
| Nav2 + `slam_toolbox` in the Gazebo apartment world | Neither | [Nav2 + slam_toolbox in Gazebo](../README.md#nav2--slam_toolbox-in-gazebo-ohh-30), the [`nav-sim`](../.github/workflows/nav-sim.yml) workflow, and the committed [run log](../omnibot-ros2/omnibot_navigation/docs/nav_sim_runs_local_log.txt). Needs ROS 2 Jazzy and Gazebo Harmonic. |
| Gazebo compose files | Neither | [docker-compose.yml](../omnibot-digital-twin/docker/docker-compose.yml) runs headless Gazebo. [docker-compose.gpu.yml](../omnibot-digital-twin/docker/docker-compose.gpu.yml) is an optional NVIDIA overlay. The docs-test job does not start compose. |
| Gazebo Harmonic `sim.launch.py` | Neither | [sim.launch.py](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py) and the `sim-smoke` job in [ci.yml](../.github/workflows/ci.yml). That job needs ROS 2 Jazzy and Gazebo Harmonic. The launch does not exit, so docs-test skips it. |
| OmniVLA engine (`vla_engine`) / OpenVLA / SmolVLA weights | GPU for real inference | [vla_engine/README.md](../omnibot-ai-engines/vla_engine/README.md), editable `pip install -e "omnibot-ai-engines/vla_engine[serve,gpu]"`, and the `vla-engine` job in [ci.yml](../.github/workflows/ci.yml) (CPU torch, mocked server tests) |
| Isaac Lab RL | GPU | [rl_engine/README.md](../omnibot-ai-engines/rl_engine/README.md) and [requirements.txt](../omnibot-ai-engines/rl_engine/requirements.txt) (`isaaclab`, `onnxruntime-gpu`) |
| Android controller | Robot | [omnibot-android/](../omnibot-android/). Building it needs the Android SDK and JDK 17 ([`build.gradle.kts`](../omnibot-android/build.gradle.kts)), which is separate from a desktop GPU. |
| Quest teleop | Robot | [omnibot-vr/](../omnibot-vr/). Unity **6000.5.2f1** is pinned in [`ProjectSettings/ProjectVersion.txt`](../omnibot-vr/ProjectSettings/ProjectVersion.txt). |
| ACT GPU fine-tune and 20-trial robot eval (OHH-52) | GPU and robot | [gpu-train-runbook.md](gpu-train-runbook.md) and [results.md](results.md). From the repo root: `python scripts/ohh52_act_train_dry_run.py`. Needs Varun: budget + robot. |
| Optional perception, metrics, and OTA nodes | Separate packages, not in this repo | [Optional packages](#optional-packages-not-in-this-repository). A public `colcon` build skips each missing node with one warning. |


## Optional packages not in this repository

These nodes are referenced by the real-robot launches and are **off by default**. The package is not built from this tree. Launch checks the ament index, logs one warning that names the missing package, and does not start the node. Passing the flag without installing the package does not raise `PackageNotFoundError`.

| Node | Package | Launch argument (default `false`) | What it needs |
|---|---|---|---|
| `object_perception_node` | `omnibot_perception` | `ai_perception` on [`perception.launch.py`](../omnibot-ros2/omnibot_bringup/launch/perception.launch.py) and [`master/omnibot_pi.launch.py`](../omnibot-ros2/omnibot_bringup/launch/master/omnibot_pi.launch.py). The Pi file documents `ai_perception:=false`. | Install `omnibot_perception`, then `ai_perception:=true`. The node uses the depth camera for object distance and pose. |
| `metrics_bridge` (node name `ros2_prometheus_bridge`) | `omnibot_metrics` | `use_metrics` on [`robot.launch.py`](../omnibot-ros2/omnibot_bringup/launch/robot.launch.py). `metrics_port` defaults to `8888`. | Install `omnibot_metrics`, then `use_metrics:=true`. Prometheus job `omnibot_ros2_bridge` scrapes that port and stays down until the node is running. Nothing else is substituted for it. |
| `ota_agent` (node name `ota_agent_node`) | `omnibot_ota` | `use_ota` on [`robot.launch.py`](../omnibot-ros2/omnibot_bringup/launch/robot.launch.py) and [`hybrid_robot.launch.py`](../omnibot-ros2/omnibot_hybrid/launch/hybrid_robot.launch.py). `ota_manifest_url` is the manifest location. | Install `omnibot_ota`, set `ota_manifest_url`, then `use_ota:=true`. |

`astra_camera` (Orbbec Astra Pro) is also outside apt/rosdep. [`perception.launch.py`](../omnibot-ros2/omnibot_bringup/launch/perception.launch.py) and [`omnibot_pi.launch.py`](../omnibot-ros2/omnibot_bringup/launch/master/omnibot_pi.launch.py) keep `depth_camera:=true` and skip the driver with one warning when that package is missing. [`rtabmap.launch.py`](../omnibot-ros2/omnibot_navigation/launch/rtabmap.launch.py) documents the source build.


## Gazebo Harmonic simulation

One command starts Gazebo Harmonic, spawns `omnibot_description`, and bridges the base and the SO-101 arm. The default is headless (`gui:=false`), which is what the `sim-smoke` job runs. The launch does not exit, so docs-test skips it.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omnibot_bringup sim.launch.py world:=flat
```

`world:=apartment` loads a single room with a kitchen (counter, stove, fridge), a desk, and a door. A dynamic `grasp_cube` sits on the desk for dataset tasks. Launch file: [`omnibot-ros2/omnibot_bringup/launch/sim.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py). The mecanum chassis is driven in the plane (`drive:=planar` by default; `drive:=mecanum` selects the MecanumDrive plugin). With `vr:=false` (the default, including sim-smoke) the sensors are a 2D lidar, a small front RGB camera, and the IMU.

`arm:=true` (the default) starts [`arm_sim_driver_node`](../omnibot-ros2/omnibot_arm/scripts/arm_sim_driver_node.py). It reads [`arm_params.yaml`](../omnibot-ros2/omnibot_arm/config/arm_params.yaml) and applies `arm_safety.py` — the same joint limits, 0.15 rad per-command cap, 0.2 s command-timeout hold, and e-stop latch as the Feetech driver. [`ros2_control_adapter.py`](../omnibot-ros2/omnibot_bringup/scripts/ros2_control_adapter.py) turns those `/arm/sim/<joint>/cmd_pos` samples into one arm trajectory and a gripper command, and republishes the arm slice of `/joint_states` on `/arm/sim/joint_states`. Measured joints are republished on `/arm/joint_states` with those URDF names. This launch does not start `arm_cmd_mux` or `arm_driver_node`, so command the arm on `/arm/joint_commands/out`. `/emergency_stop` true freezes the sim joints at the measured pose and rejects later commands until `/arm/enable` true while the stop is clear.

`drive:=planar` keeps gz-sim `VelocityControl` and `OdometryPublisher`, because the wheel collision is a cylinder and a wheel-velocity mecanum model already missed Nav2's map-frame timeout. In that mode the wheel command interfaces are `omnibot_bringup/PlanarWheelMock`. The gz resource manager only loads `GazeboSimSystemInterface`, so `mock_components/GenericSystem` never came up there. The mock does not write a non-zero wheel velocity, so the mecanum controller can be active without locking the kinematic base. A claimed zero brakes the wheel joint. A non-zero command clears that brake with a zero joint force: Harmonic keeps a DART velocity servo after the velocity component is removed, and that servo was holding teleop at zero. Arm links set `turnGravityOff` (sdformat ignores a plain `gravity` tag) and joint damping, so the arm cannot fall and shove the chassis while `gz_ros2_control` is still waiting on `/robot_description`. `drive:=mecanum` with `use_sim:=true` drops the gz `MecanumDrive` plugin and commands the wheel joints. The URDF revolute stops stay the SO-101 calibration values and are not widened to the software limits. `simulation.launch.py` does not pass `use_sim`, so it keeps the desktop gz drive plugins. The per-joint `JointPositionController` plugins are gone in every mode.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omnibot_bringup sim.launch.py world:=flat
ros2 run omnibot_arm arm_sim_scripted_move.py
```

The VR profile is [`sim_vr.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim_vr.launch.py). It includes this launch with `world:=apartment` and `vr:=true`, so Gazebo follows `/cmd_vel/out`. rosbridge and `web_video_server` bind `127.0.0.1` unless `rosbridge_address` is set with the gateway off. The real-robot command is [`robot_vr.launch.py`](../omnibot-ros2/omnibot_bringup/launch/robot_vr.launch.py). Both load [`vr_topic_contract.yaml`](../omnibot-ros2/omnibot_bringup/config/vr_topic_contract.yaml). The Pi CPU table and the systemd unit are in [vr.md](vr.md). The unit is not enabled by this repo.

`ros_gz_bridge` topics. `rosbridge:=true` (the default) also starts `rosbridge_websocket` on port 9090. The sim-smoke test passes `rosbridge:=false` and does not start rosbridge:

| Topic | Type |
|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/imu` | `sensor_msgs/msg/Imu` |
| `/camera/front/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/tf` (`odom` -> `base_footprint`, added for Nav2 and `slam_toolbox`) | `tf2_msgs/msg/TFMessage` |
| `/arm/sim/joint_states` | `sensor_msgs/msg/JointState` (bridge entry kept; in `use_sim` the adapter publishes this from `/joint_states`) |
| `/arm/sim/<joint>/cmd_pos` | `std_msgs/msg/Float64` (one topic per arm joint; the adapter subscribes, the gz side is idle) |

ROS topics that are not bridged: `/arm/joint_commands/out` and `/arm/enable` and `/emergency_stop` go to `arm_sim_driver`, which publishes `/arm/joint_states`.

Bridge config: [`omnibot-ros2/omnibot_bringup/config/sim_bridge.yaml`](../omnibot-ros2/omnibot_bringup/config/sim_bridge.yaml).

## ros2_control

Sim and the robot load the same [`controllers.yaml`](../omnibot-ros2/omnibot_bringup/config/controllers.yaml). `sim.launch.py` passes `use_sim:=true` and starts the manager inside `gz_ros2_control`. `robot.launch.py` (`ros2_control:=true`, the default) starts `controller_manager`'s `ros2_control_node` and does not start `yahboom_controller_node`, so the base serial port is opened once. `ros2_control:=false` is the previous Yahboom node. `robot_vr.launch.py` and `omnibot_pi.launch.py` still use the Python Yahboom and Feetech drivers.

| Controller | Plugin | Command |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | publishes `/joint_states` |
| `mecanum_drive_controller` | `mecanum_drive_controller/MecanumDriveController` | `TwistStamped` on `/mecanum_drive_controller/reference` |
| `arm_joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | `FollowJointTrajectory` for the five arm joints |
| `gripper_controller` | `parallel_gripper_action_controller/GripperActionController` | `ParallelGripperCommand` on `/gripper_controller/gripper_cmd` |

Humble's gripper plugin name was `position_controllers/GripperActionController`. This yaml uses the Jazzy name. The manager update rate is 50 Hz. `reference_timeout` is 0.5 s. `tf_frame_prefix_enable` is false and `base_frame_id` is `base_footprint`. `enable_odom_tf` is false because planar Gazebo and the robot EKF already publish `odom` → `base_footprint`. Wheel geometry is radius 0.04 m and `lx + ly = 0.19` (`wheel_separation_x` 0.165 and `wheel_separation_y` 0.215, each halved).

The adapter keeps the old topics. It listens to `/cmd_vel` or `/cmd_vel/out` (the latter when the bridge file is `sim_bridge_mux.yaml` or `sim_bridge_vr.yaml`) and republishes a stamped twist. A twist older than 0.25 s, or `/emergency_stop` true, becomes zeros. On the robot it loads `arm_safety.py` and does not forward arm commands if that import fails.

### Hardware interfaces

| Joints | Command | State |
|---|---|---|
| `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint` | velocity | position, velocity |
| `arm_shoulder_pan`, `arm_shoulder_lift`, `arm_elbow_flex`, `arm_wrist_flex`, `arm_wrist_roll`, `arm_gripper` | position, limited to the URDF stops | position, velocity |

`use_mock:=true` (the bringup launch test, no Gazebo) puts every joint on `mock_components/GenericSystem`. `use_sim:=true` with `drive:=planar` puts the arm on `gz_ros2_control/GazeboSimSystem` and the wheels on `omnibot_bringup/PlanarWheelMock`. Any other `use_sim` drive puts every joint on `GazeboSimSystem`. Jazzy `controller_manager` loads that URDF from the `/robot_description` topic published by `robot_state_publisher`, not from a node parameter. The gz plugin gain is `position_proportional_gain` 0.15 with `hold_joints` true.

### `omnibot_hardware/OmniBotSystem`

Plugin `omnibot_hardware/OmniBotSystem` (`hardware_interface::SystemInterface`). The base speaks the Yahboom frames in `yahboom-python-driver`: five `SET_CAR_TYPE` writes, then `FUNC_MOTION` body velocity. Wheel rad/s from the controller are converted with the same mecanum inverse kinematics the controller uses. There is no per-wheel encoder opcode. `TYPE_VELOCITY` is integrated into encoder-equivalent ticks (`encoder_ticks_per_revolution`, default 1440) and reported as wheel position. The arm speaks Feetech STS3215 protocol 0: `Torque_Enable` at address 40, `Goal_Position` at 42, `Present_Position` at 56, `Present_Speed` at 58. Tick conversion matches `arm_math.py`. `on_configure` opens the ports. `on_activate` reads the present positions and writes those goals before torque is enabled, so the first command is a hold. A failed read does not command tick 0. `on_deactivate` and `on_error` send a zero `FUNC_MOTION` and `Torque_Enable` 0. `/emergency_stop` true stops the base and drops arm torque. Clearing the stop does not re-enable torque. `/arm/enable` true re-enables it only while the stop is clear.

| Parameter | Default |
|---|---|
| `base_port` | `/dev/ttyUSB0` |
| `base_baud` | `115200` |
| `wheel_radius` | `0.04` |
| `wheel_separation_length` | `0.165` (full front-back; `lx` is half) |
| `wheel_separation_width` | `0.215` (full left-right; `ly` is half) |
| `encoder_ticks_per_revolution` | `1440` |
| `arm_port` | `/dev/ttyACM0` |
| `arm_baud` | `1000000` |
| `servo_ids` | `1,2,3,4,5,6` |
| `home_ticks` | `2048,2048,2048,2048,2048,2048` (factory half-turn, not this robot) |
| `ticks_per_rev` | `4096` |
| `calibration_file` | empty. An OHH-94 yaml replaces `home_ticks`. A `joints` list must agree with `home_ticks`. |

`robot.launch.py` forwards `calibration_file:=`. The gtest covers framing, tick conversion, calibration mismatch, and a fake serial port. It does not open `/dev`.

### VR cameras (`vr:=true`)

`vr:=true` is the Quest camera profile. It keeps `minimal_sensors` (the depth camera and the four fisheyes stay off) and adds three sensors at 960×540, 30 Hz: a wrist camera on the SO-101 gripper link, a synthetic top-down camera publishing `/camera/base/bev/image_raw`, and a chase camera fixed behind `base_link` publishing `/sim/chase_cam/image_raw`. The front camera is raised to the same size. `video:=true` (the default) starts `web_video_server` on port 8080, bound to `rosbridge_address` (127.0.0.1 unless you set it).

```bash
ros2 launch omnibot_bringup sim.launch.py world:=apartment vr:=true
```

Chase view, once the server is up: `http://<host>:8080/stream?topic=/sim/chase_cam/image_raw`. The same server also serves `/camera/front/image_raw`, `/camera/wrist/image_raw`, and `/camera/base/bev/image_raw`. Extra bridge: [`sim_bridge_cameras.yaml`](../omnibot-ros2/omnibot_bringup/config/sim_bridge_cameras.yaml).

[`sim_vr.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim_vr.launch.py) is that profile for the GPU container: `world:=apartment`, `vr:=true`, `gui:=false` (so `--headless-rendering` stays on), and `rosbridge_address:=0.0.0.0` so Docker can publish the sockets. `sim.launch.py` still defaults that address to 127.0.0.1. From the repo root, `docker compose -f docker-compose.gpu.yml up sim-vr` publishes 9090 and 8080 on 127.0.0.1 and points EGL at the NVIDIA driver. The CPU image and the CPU compose file stay on llvmpipe.

`gui:=false` (the default) passes `-s --headless-rendering` so those cameras render without a Gazebo window. `gui:=true` drops that flag and opens the client. The sensors are `always_on`, so the GPU cost is paid even when no browser is connected. `web_video_server` encodes a topic only while a client holds the stream. The GPU delta was not measured here (no Gazebo install). This flag does not start `teleop_safety`.

The headless launch test [`omnibot-ros2/omnibot_bringup/test/test_sim_launch.py`](../omnibot-ros2/omnibot_bringup/test/test_sim_launch.py) checks that `/odom`, `/scan`, `/imu`, and `/camera/front/image_raw` publish within 30 s and that `/cmd_vel` moves the robot at least 0.2 m. The same launch runs [`arm_sim_scripted_move.py`](../omnibot-ros2/omnibot_arm/scripts/arm_sim_scripted_move.py), which moves each arm joint to a target within 0.08 rad and checks that `/emergency_stop` holds that pose. The `sim-smoke` job in [`.github/workflows/ci.yml`](../.github/workflows/ci.yml) runs it on `ros:jazzy`.

Eight frames from `/camera/front/image_raw` saved by that test (`OMNIBOT_SIM_FRAME_DIR`) while it published `/cmd_vel` in `world:=flat`. The red box is the `marker` model in [`flat.sdf`](../omnibot-ros2/omnibot_bringup/worlds/flat.sdf):

<p align="center">
  <img src="../assets/omnibot_gazebo_flat.gif" width="320" alt="Front camera frames from the flat Gazebo world while the robot drives"/>
</p>

## Teleop from the website

The OhhO website's Connect page can drive the sim over rosbridge. Start the flat Gazebo world, then open Connect and set the ROSBridge URL to `ws://localhost:9090`. rosbridge listens on localhost only by default; `rosbridge_address:=0.0.0.0` exposes it on the network without authentication. The launch does not exit, so the docs-test job skips this fence.

```bash
ros2 launch omnibot_bringup sim.launch.py world:=flat
```

`/cmd_vel` moves the base, `/odom` publishes, and `/camera/front/image_raw` publishes when the camera sensor starts. `world:=apartment` stays the furnished world.

The requirement files below pull GPU stacks. Docs-test skips them.

```bash
pip install -e "omnibot-ai-engines/vla_engine[serve,gpu]"
pip install -r omnibot-ai-engines/rl_engine/requirements.txt
```

## Quest to a WSL2 sim on Windows (OHH-205)

A Quest on Wi-Fi reaches rosbridge and `web_video_server` on the Windows LAN address. Docker inside WSL2 is behind NAT until mirrored networking or a TCP portproxy is in place. The choice is a Varun TODO. The script and the troubleshooting notes are [vr-sim-network-windows.md](vr-sim-network-windows.md) and [`scripts/windows/vr-sim-network.ps1`](../scripts/windows/vr-sim-network.ps1). `-WhatIf` prints the LAN IP, pairing code, and QR code and leaves the firewall, portproxy, and `.wslconfig` alone.

`sim.launch.py` binds rosbridge to `127.0.0.1` and does not start `web_video_server`. The Quest browser URL is `http://<pc-ip>:8080`. The VR app uses `ws://<pc-ip>:9090`.

## Bring-up control mode

`mobile_manipulation.launch.py` starts `teleop_safety`, `cmd_vel_mux`, and `arm_cmd_mux` in `teleop`. Clients publish `/cmd_vel/teleop_raw`; `teleop_safety` publishes `/cmd_vel/teleop`, which the mux forwards. Clients must also publish `/ohho/heartbeat` (`std_msgs/Empty`) at 10 Hz. The gate starts latched, so a heartbeat alone does not arm it; call `/ohho/estop_reset` while `/emergency_stop` is false. Without a fresh heartbeat, `teleop_safety` remains disarmed and zeros the base. A true `/emergency_stop` remains latched until `/ohho/estop_reset` is called after the stop input is cleared. See [safety.md](safety.md). Nav2, VLA, and RL base commands are not forwarded until `/control_mode` is `vla`, `nav2`, or `rl_nav`. `/arm/joint_commands/teleop` (Quest VR hand IK and Android joint sliders) is forwarded in `teleop` while `/emergency_stop` is false, and held in `vla`, `nav2`, and `rl_nav`. `/arm/joint_commands/policy` (`policy_node`) and `/arm/joint_commands/rl` stay held in `teleop`. `/arm/joint_commands` is a deprecated alias of the policy input for one release and is held in teleop the same way. Switching `/control_mode` between any two different modes (teleop→autonomous, autonomous→teleop, or autonomous→autonomous) publishes a zero Twist to stop the base on that callback so the previous source's latched velocity does not run away. Setting `/control_mode` back to `teleop` stops the policy and RL arm streams on that callback, forwards `/cmd_vel/teleop`, and forwards `/arm/joint_commands/teleop`. Publishing `/emergency_stop` true stops base and arm outputs, including teleop, on that callback. `sim_teleop.launch.py` is the Gazebo profile for the same gate. It includes `sim.launch.py` with `bridge_config:=sim_bridge_vr.yaml` so Gazebo follows `/cmd_vel/out`, and it starts `teleop_safety`, `cmd_vel_mux`, and `arm_cmd_mux`. `vr:=true` on `sim.launch.py` stays the camera profile and does not start this gate. `sim_vr.launch.py` does not start it either.

| Arm input | Who publishes | Forwarded when |
|---|---|---|
| `/arm/joint_commands/teleop` | `teleop_safety` (clients publish `/arm/joint_commands/teleop_raw`) | `/control_mode` is `teleop` and `/emergency_stop` is false |
| `/arm/joint_commands/policy` | `policy_node` | `vla` or `nav2`, arm source `policy`, e-stop clear |
| `/arm/joint_commands` | deprecated alias of the policy input (one release) | same gate as `/arm/joint_commands/policy` |
| `/arm/joint_commands/rl` | `rl_arm_node` | `rl_nav`, arm source `rl_arm`, e-stop clear |
| `/arm/joint_commands/out` | `arm_cmd_mux` | the selected source above; `arm_driver_node` subscribes |

`hybrid_robot.launch.py` and `omnibot_pi.launch.py` still pass `default_mode:=nav2` for the base mux. This gate was unit-tested here without ROS 2. It was not run with `ros2 launch`, Gazebo, or a robot.

## Flagship stack launch architecture

The flagship single launch file (`hybrid_robot.launch.py`), physical bringup (`mobile_manipulation.launch.py`), topic routing, and Pi 5 vs GPU workstation node placement are documented in [docs/frame-launch.md](frame-launch.md). It also clarifies that the 20-trial evaluation (OHH-101) has not run yet.

## VR session MCAP

`vr:=true record_vr_session:=true` on [`sim.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py) includes [`vr_session_recorder.launch.py`](../omnibot-ros2/omnibot_hybrid/launch/vr_session_recorder.launch.py). The flag defaults to false. The recorder writes `/cmd_vel/teleop*`, `/arm/joint_commands/teleop*`, `/ohho/heartbeat`, `/ohho/telemetry`, `/ohho/safety/state`, and `/control_mode*` to rotating MCAP parts under `~/.ohho/vr_sessions`, with a 64 MiB part size and a 512 MiB cap. It does not change `teleop_safety`. Replay a part in Foxglove with [`foxglove/vr_session.json`](../foxglove/vr_session.json). The Grafana panels for RTT, heartbeat gaps, and safety trips are [`infra/observability/grafana/dashboards/vr-session.json`](../infra/observability/grafana/dashboards/vr-session.json). OHH-198's `sim_vr.launch.py` should forward `record_vr_session` into `sim.launch.py` and should not start a second recorder. A real-robot bag and a Foxglove screenshot still need a person.
