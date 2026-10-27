# OmniBot reference

Detailed notes moved from the top-level README. Back to [README](../README.md).

## Needs hardware / needs GPU

| Feature | Needs | Evidence |
|---|---|---|
| `ohho sim --robot omnibot` | Neither | [ohho-os 1.1.2](https://pypi.org/project/ohho-os/1.1.2/) and the docs-test job in [ci.yml](../.github/workflows/ci.yml) |
| Colcon build and test of `src/` | Neither | [src/](../src/) and the `ros` job in [ci.yml](../.github/workflows/ci.yml). That job needs ROS 2 Jazzy. |
| Kinematics, Yahboom codec, BEV homography, arm tick math, arm command safety (joint limits, 0.15 rad/cycle, 200 ms hold, e-stop torque latch), policy inference guards (black/missing frame refusal, latency drop, 0.15 rad/cycle arm clamp), learning engine, agent engine | Neither | The `cpu` job in [ci.yml](../.github/workflows/ci.yml) runs `pytest omnibot-ros2/omnibot_arm/test omnibot-ai-ros2/omnibot_lerobot/test`. Those tests do not start ROS or the arm. |
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


## Gazebo Harmonic simulation

One command starts Gazebo Harmonic, spawns `omnibot_description`, and bridges the base and the SO-101 arm. The default is headless (`gui:=false`), which is what the `sim-smoke` job runs. The launch does not exit, so docs-test skips it.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omnibot_bringup sim.launch.py world:=flat
```

`world:=apartment` loads a single room with a kitchen (counter, stove, fridge), a desk, and a door. A dynamic `grasp_cube` sits on the desk for dataset tasks. Launch file: [`omnibot-ros2/omnibot_bringup/launch/sim.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py). The mecanum chassis is driven in the plane (`drive:=planar` by default; `drive:=mecanum` selects the MecanumDrive plugin). With `vr:=false` (the default, including sim-smoke) the sensors are a 2D lidar, a small front RGB camera, and the IMU.

`arm:=true` (the default) starts [`arm_sim_driver_node`](../omnibot-ros2/omnibot_arm/scripts/arm_sim_driver_node.py). It reads [`arm_params.yaml`](../omnibot-ros2/omnibot_arm/config/arm_params.yaml) and applies `arm_safety.py` — the same joint limits, 0.15 rad per-command cap, 0.2 s command-timeout hold, and e-stop latch as the Feetech driver. Each arm joint, including `arm_gripper`, has a gz-sim `JointPositionController`. Measured joints are republished on `/arm/joint_states` with those URDF names. This launch does not start `arm_cmd_mux` or `arm_driver_node`, so command the arm on `/arm/joint_commands/out`. `/emergency_stop` true freezes the sim joints at the measured pose and rejects later commands until `/arm/enable` true while the stop is clear.

`gz_ros2_control` is not used. The sim image already runs the gz-sim systems that ship with `ros_gz_sim`, including `JointPositionController` and `JointStatePublisher`. The URDF revolute stops stay the SO-101 calibration values and are not widened to the software limits.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omnibot_bringup sim.launch.py world:=flat
ros2 run omnibot_arm arm_sim_scripted_move.py
```

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
| `/arm/sim/joint_states` | `sensor_msgs/msg/JointState` (Gazebo arm joints) |
| `/arm/sim/<joint>/cmd_pos` | `std_msgs/msg/Float64` (one topic per arm joint, ROS to Gazebo) |

ROS topics that are not bridged: `/arm/joint_commands/out` and `/arm/enable` and `/emergency_stop` go to `arm_sim_driver`, which publishes `/arm/joint_states`.

Bridge config: [`omnibot-ros2/omnibot_bringup/config/sim_bridge.yaml`](../omnibot-ros2/omnibot_bringup/config/sim_bridge.yaml).

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

`mobile_manipulation.launch.py` starts `cmd_vel_mux` and `arm_cmd_mux` in `teleop`. `/cmd_vel/teleop` is forwarded. Nav2, VLA, and RL base commands, and both arm command sources, are not forwarded until `/control_mode` is `vla`, `nav2`, or `rl_nav`. Switching `/control_mode` between any two different modes (teleop→autonomous, autonomous→teleop, or autonomous→autonomous) publishes a zero Twist to stop the base on that callback so the previous source's latched velocity does not run away. Setting `/control_mode` back to `teleop` also stops the autonomous arm stream on that callback and forwards `/cmd_vel/teleop`. Publishing `/emergency_stop` true stops base and arm outputs, including teleop, on that callback. `/arm/joint_commands` is shared by the policy and the Android app, so Android joint commands wait for an autonomous mode as well.

`hybrid_robot.launch.py` and `omnibot_pi.launch.py` still pass `default_mode:=nav2` for the base mux. This gate was unit-tested here without ROS 2. It was not run with `ros2 launch`, Gazebo, or a robot.

## Flagship stack launch architecture

The flagship single launch file (`hybrid_robot.launch.py`), physical bringup (`mobile_manipulation.launch.py`), topic routing, and Pi 5 vs GPU workstation node placement are documented in [docs/frame-launch.md](frame-launch.md). It also clarifies that the 20-trial evaluation (OHH-101) has not run yet.
