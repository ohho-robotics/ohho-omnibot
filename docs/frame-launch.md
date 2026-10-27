# OhhO Frame: Flagship Stack Launch Architecture (OHH-104)

This document specifies the launch architecture for the OmniBot flagship stack under OhhO Frame, detailing the primary launch files, topic routing, compute node placement between the onboard Raspberry Pi 5 and offboard GPU Workstation, and the verification baseline for upcoming evaluations.

---

## 1. Flagship Single Launch File

In this repository, the primary unified launch file representing the complete flagship mobile-manipulation stack is:

* **[`omnibot-ros2/omnibot_hybrid/launch/hybrid_robot.launch.py`](../omnibot-ros2/omnibot_hybrid/launch/hybrid_robot.launch.py)** (Package: `omnibot_hybrid`)

### Why `hybrid_robot.launch.py` is the Current Best Single Launch

`hybrid_robot.launch.py` unites the low-level chassis control, arm safety gating, 2D SLAM, autonomous navigation, vision-language-action (VLA) inference, and operator connectivity into a single launch tree:

1. **End-to-End Control Routing**: Integrates the base driver (`yahboom_controller_node.py`), arm command multiplexer (`arm_cmd_mux`), and base velocity multiplexer (`cmd_vel_mux`).
2. **Localization & Mapping**: Starts URDF robot state publishing (`robot_state_publisher`), EKF sensor fusion (`ekf_node` from `robot_localization`), and SLAM Toolbox (`slam_toolbox.launch.py`).
3. **Autonomous Navigation**: Launches the full Nav2 stack (`autonomous_navigation.launch.py` via `nav2_params.yaml`), allowing waypoint and costmap-driven navigation.
4. **VLA Policy Inference**: Mounts OpenVLA 7B (`vla_node` from `omnibot_vla`) or visuomotor policies like SmolVLA (`policy_inference.launch.py` from `omnibot_lerobot`).
5. **Perception Fusing**: Starts the Bird's-Eye View stitcher (`bev_stitcher_node`), warping 4 base cameras into a unified top-down view.
6. **Mission & Remote Connectivity**: Starts `mission_planner`, ROSBridge WebSocket on port 9090 (`rosbridge_websocket`), and Foxglove bridge on port 8765 (`foxglove_bridge`).

### Physical Bringup Launch

On physical hardware, hardware bringup with safe teleop defaults (OHH-96) is launched via:

* **[`omnibot-ros2/omnibot_bringup/launch/mobile_manipulation.launch.py`](../omnibot-ros2/omnibot_bringup/launch/mobile_manipulation.launch.py)** (Package: `omnibot_bringup`)

`mobile_manipulation.launch.py` brings up all 5 physical USB cameras (front, rear, left, right, wrist), the Orbbec Astra Pro depth camera, the SO-101 6-DOF arm driver (`arm_driver_node.py`), and the Yahboom mecanum base driver, starting `cmd_vel_mux` and `arm_cmd_mux` in `teleop` mode to prevent uncommanded autonomous motion at power-on.

---

## 2. Topics Started and Routed by the Flagship Stack

The flagship stack coordinates motion, perception, and state through a multiplexed topic bus:

### Actuation & Velocity Topics

| Topic | Type | Source / Publisher | Target / Subscriber | Purpose |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `Nav2` (`controller_server`) | `cmd_vel_mux` | Autonomous navigation velocity commands |
| `/cmd_vel/vla` | `geometry_msgs/msg/Twist` | `vla_node` / `policy_node` | `cmd_vel_mux` | Semantic VLA vision-guided velocity commands |
| `/cmd_vel/teleop` | `geometry_msgs/msg/Twist` | Joystick / Web / Quest | `cmd_vel_mux` | Manual operator teleoperation commands |
| `/cmd_vel/rl` | `geometry_msgs/msg/Twist` | `rl_nav_node` | `cmd_vel_mux` | Reinforcement learning policy velocity commands |
| `/cmd_vel/out` | `geometry_msgs/msg/Twist` | `cmd_vel_mux` | `yahboom_driver` | Gated output velocity passed directly to base motors |

### Arm Manipulation Topics

| Topic | Type | Source / Publisher | Target / Subscriber | Purpose |
|---|---|---|---|---|
| `/arm/joint_commands` | `sensor_msgs/msg/JointState` | `smolvla_node` / Android | `arm_cmd_mux` | Autonomous or manual joint target positions |
| `/arm/joint_commands/rl` | `sensor_msgs/msg/JointState` | `rl_arm_node` | `arm_cmd_mux` | RL arm policy joint target commands |
| `/arm/joint_commands/out` | `sensor_msgs/msg/JointState` | `arm_cmd_mux` | `arm_driver_node` | Gated joint commands routed to Feetech STS3215 servos |
| `/arm/joint_states` | `sensor_msgs/msg/JointState` | `arm_driver_node` | Policy nodes, EKF, RViz | Measured positions, velocities, and currents |

### State Estimation & Mapping Topics

| Topic | Type | Source / Publisher | Target / Subscriber | Purpose |
|---|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `yahboom_driver` | `ekf_node`, Nav2, policy | Raw wheel odometry from encoders |
| `/imu/data` | `sensor_msgs/msg/Imu` | `yahboom_driver` | `ekf_node` | Onboard IMU linear acceleration & angular velocity |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `ekf_node` | Nav2, SLAM | EKF filtered state estimation (`odom` $\to$ `base_link` TF) |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `rsp`, `ekf_node`, `slam` | Entire stack | Coordinate transform tree |
| `/scan` | `sensor_msgs/msg/LaserScan` | Lidar / depthimage converter | `slam_toolbox`, Nav2 | 2D laser scan for mapping and obstacle avoidance |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` / `map_server` | Nav2, Foxglove | 2D grid map of the environment (`map` $\to$ `odom` TF) |

### Perception Streams

| Topic | Type | Source / Publisher | Target / Subscriber | Purpose |
|---|---|---|---|---|
| `/camera/front/image_raw` | `sensor_msgs/msg/Image` | Front `usb_cam` | `vla_node`, `bev_stitcher` | Front RGB camera stream |
| `/camera/rear/image_raw` | `sensor_msgs/msg/Image` | Rear `usb_cam` | `bev_stitcher` | Rear RGB camera stream |
| `/camera/left/image_raw` | `sensor_msgs/msg/Image` | Left `usb_cam` | `bev_stitcher` | Left RGB camera stream |
| `/camera/right/image_raw` | `sensor_msgs/msg/Image` | Right `usb_cam` | `bev_stitcher` | Right RGB camera stream |
| `/camera/wrist/image_raw` | `sensor_msgs/msg/Image` | Wrist `usb_cam` | `smolvla_node`, LeRobot | End-effector camera stream for manipulation |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Orbbec Astra Pro | `depthimage_to_laserscan` | Depth image stream |
| `/camera/base/bev/image_raw` | `sensor_msgs/msg/Image` | `bev_stitcher_node` | `smolvla_node`, Foxglove | Fused 800x800 bird's-eye view surround image |

### Control Mode, Mission & Safety Topics

| Topic | Type | Source / Publisher | Target / Subscriber | Purpose |
|---|---|---|---|---|
| `/control_mode` | `std_msgs/msg/String` | `mission_planner` / User | `cmd_vel_mux`, `arm_cmd_mux` | Mode selector (`teleop`, `nav2`, `vla`, `rl_nav`) |
| `/control_mode/active` | `std_msgs/msg/String` | `cmd_vel_mux` | Mission / Telemetry | Confirmed active multiplexer mode |
| `/emergency_stop` | `std_msgs/msg/Bool` | User / UI / Monitor | Muxes, `arm_driver_node` | Latches zero-velocity stop and arm torque drop |
| `/mission/command` | `std_msgs/msg/String` | Agent / Orchestrator | `mission_planner` | Compound missions (e.g. `navigate:kitchen,vla:find cup`) |
| `/mission/status` | `std_msgs/msg/String` | `mission_planner` | User / Telemetry | Execution progress of composite mission |
| `/mission/cancel` | `std_msgs/msg/String` | User / UI | `mission_planner` | Abort signal for active missions |

---

## 3. Compute Node Placement: Raspberry Pi 5 vs. GPU Workstation

For distributed physical deployment, compute is partitioned across two nodes:

* **Raspberry Pi 5 Master Launch**: [`omnibot-ros2/omnibot_bringup/launch/master/omnibot_pi.launch.py`](../omnibot-ros2/omnibot_bringup/launch/master/omnibot_pi.launch.py)
* **GPU Workstation Master Launch**: [`omnibot-ros2/omnibot_bringup/launch/master/omnibot_workstation.launch.py`](../omnibot-ros2/omnibot_bringup/launch/master/omnibot_workstation.launch.py)

Both machines communicate over a shared ROS 2 domain (`ROS_DOMAIN_ID=30`) with static peers configured via `ROS_STATIC_PEERS`.

### Allocation Matrix

| Node | Executable / Package | Target Host | Resource Footprint | Rationale |
|---|---|---|---|---|
| `yahboom_driver` | `omnibot_driver/yahboom_controller_node.py` | **Raspberry Pi 5** | Light CPU, USB-Serial | Direct physical USB connection (`/dev/ttyUSB0`) to Yahboom STM32 motor board |
| `arm_driver_node` | `omnibot_arm/arm_driver_node.py` | **Raspberry Pi 5** | Light CPU, USB-Serial | Direct physical serial bus to Feetech STS3215 arm servos; hardware e-stop safety |
| `usb_cam` (x5) | `usb_cam/usb_cam_node_exe` | **Raspberry Pi 5** | Moderate CPU, USB V4L2 | Direct USB video devices (`/dev/video1` through `/dev/video11`) |
| `astra_camera` | `orbbec_camera/orbbec_camera_node` | **Raspberry Pi 5** | Moderate CPU, USB 2.0/3.0 | Direct physical connection to Orbbec Astra Pro depth sensor |
| `bev_stitcher_node` | `omnibot_lerobot/bev_stitcher_node` | **Raspberry Pi 5** | Moderate CPU / OpenCV | Stitching 4 base streams locally minimizes raw camera network bandwidth |
| `ekf_node` | `robot_localization/ekf_node` | **Raspberry Pi 5** | Light CPU | Low-latency state estimation loop (fuses `/odom` + `/imu/data`) |
| `slam_toolbox` | `slam_toolbox/async_slam_toolbox_node` | **Raspberry Pi 5** | Moderate CPU | Local 2D LiDAR SLAM, maintains `map` $\to$ `odom` transform on-robot |
| `nav2_*` servers | `nav2_*` (controller, planner, BT) | **Raspberry Pi 5** | Moderate CPU | Local real-time path planning and obstacle avoidance loop |
| `cmd_vel_mux` | `omnibot_hybrid/cmd_vel_mux.py` | **Raspberry Pi 5** | Very Light CPU | Critical safety gate directly ahead of base driver |
| `arm_cmd_mux` | `omnibot_rl/arm_cmd_mux.py` | **Raspberry Pi 5** | Very Light CPU | Critical safety gate directly ahead of arm driver |
| `mission_planner` | `omnibot_hybrid/mission_planner.py` | **Raspberry Pi 5** | Light CPU | Coordinates Nav2 goals and VLA prompts on the local bus |
| `rosbridge_websocket` | `rosbridge_server/rosbridge_websocket` | **Raspberry Pi 5** | Light CPU, Network | Local websocket server (port 9090) for Android app / browser connection |
| `foxglove_bridge` | `foxglove_bridge/foxglove_bridge` | **Raspberry Pi 5** | Light CPU, Network | Local websocket bridge (port 8765) for live telemetry streaming |
| `vla_node` | `omnibot_vla/vla_node.py` | **GPU Workstation** | Heavy GPU ($\ge$16 GB VRAM) | OpenVLA 7B foundation model requiring desktop NVIDIA CUDA compute |
| `smolvla_node` / `policy_node` | `omnibot_lerobot/policy_node.py` | **GPU Workstation** | Heavy GPU ($\ge$8 GB VRAM) | SmolVLA 9-DOF unified policy inference at 10 Hz |
| `rl_nav_node` / `rl_arm_node` | `omnibot_rl/rl_*_node.py` | **GPU Workstation** | Moderate GPU / ONNX | High-rate (20 Hz) reinforcement learning policy inference |
| `langchain_agent_node` | `omnibot_orchestration/langchain_agent_node.py` | **GPU Workstation** | Network / LLM API | High-level agent reasoning (Claude/LangGraph) connecting to cloud APIs |

---

## 4. Status of the 20-Trial Evaluation (OHH-101)

> [!IMPORTANT]
> **The 20-trial evaluation (tracked under Linear issue OHH-101) has NOT run yet.**

* No empirical 20-trial benchmark results, success rate percentages, intervention counts, or hardware evaluation logs exist in this repository.
* Any documentation or public claims regarding OhhO Frame "Works today" (such as on the `ohho-robotics.com` website) **must not claim that the 20-trial evaluation has been completed**.
* Frame's currently verified capabilities consist of:
  1. Gazebo Harmonic simulation with Nav2 and `slam_toolbox` in the apartment environment, verified by 15 automated test runs across 3 scenarios (OHH-30).
  2. Gated control multiplexing with safe teleop startup defaults, mode-switch stops, and arm e-stop latches (OHH-96, OHH-93, OHH-100).
  3. Agent goal verification and loop tracing connecting high-level goals to Nav2 (OHH-31).
  4. Camera schema standardization (wrist and BEV) and episode quality filtering (OHH-95, OHH-99).
  5. Pure-Python core test suites for kinematics, Yahboom serial protocol, and learning engines passing in CPU CI.

Running the full 20-trial physical benchmark remains blocked on physical hardware setup and execution under OHH-101.

---

## 5. Relationship to OHH-34 (MCAP Recording & Foxglove Replay)

Related work in branch `origin/ohh-34-mcap-foxglove` (issue OHH-34) introduces tooling for deterministic logging and visualization:

* **Recording Launch**: [`omnibot-ros2/omnibot_bringup/launch/record.launch.py`](../omnibot-ros2/omnibot_bringup/launch/record.launch.py) records `/tf`, `/odom`, `/scan`, camera feeds, and `/agent/loop_trace` into compressed `.mcap` files using `rosbag2_storage_mcap`.
* **Foxglove Layout**: [`foxglove/omnibot_layout.json`](../foxglove/omnibot_layout.json) enables timeline scrubbing and 3D visual playback in Foxglove Studio.

### Scope Distinction for Frame "Works today"

* **MCAP & Foxglove**: While OHH-34 adds MCAP recording tooling to the repository, it represents dedicated recording tooling and must not be conflated with the base Frame operational stack.
* **Isaac Sim & USD**: Isaac Sim and USD digital twin workflows are **not built** (`isaac.launch.py` is a stub that prints `not built` and exits with code 2). Frame "Works today" must NOT claim Isaac Sim, USD assets, or full MCAP replay as functional live features.
