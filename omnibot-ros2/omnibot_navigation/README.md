# OmniBot Autonomous Navigation

Autonomous navigation for the OmniBot mecanum-wheel robot, built on the ROS 2
Nav2 stack with `slam_toolbox` 2-D SLAM and an optional RTAB-Map + OctoMap
3-D RGB-D pipeline.

## Features

- **2-D SLAM** with `slam_toolbox`
- **3-D RGB-D SLAM** (optional) with RTAB-Map + OctoMap from the Orbbec Astra
  depth camera
- **Nav2 path planning + obstacle avoidance** (omnidirectional, mecanum)
- **EKF state estimation** via `robot_localization` (fuses wheel odom + IMU)
- **Waypoint following** (`waypoint_navigator`)

## Package Structure

```
omnibot_navigation/
??? config/
?   ??? nav2_params.yaml          # Nav2 parameters (DWB, holonomic; velocity caps match the driver)
?   ??? robot_localization.yaml   # EKF state estimation
?   ??? slam_toolbox_params.yaml  # 2-D SLAM
?   ??? rtabmap_params.yaml       # RTAB-Map 3-D RGB-D SLAM
?   ??? octomap_params.yaml       # OctoMap server (cloud ? 2-D projected map)
?   ??? sensor_integration.yaml   # Sensor frame/topic templates
?   ??? waypoints.yaml            # Named waypoints (kitchen, desk, door) in the sim map frame
??? docs/
?   ??? SENSOR_INTEGRATION.md     # Sensor integration guide
?   ??? nav_sim_runs_local.jsonl  # Time-to-goal results of the local Gazebo runs
?   ??? nav_sim_runs_local_log.txt
??? launch/
?   ??? autonomous_robot.launch.py        # Top-level: base + description + EKF + SLAM + Nav2 (+ RViz)
?   ??? autonomous_navigation.launch.py   # Nav2 stack only
?   ??? autonomous_with_waypoints.launch.py
?   ??? slam_toolbox.launch.py            # 2-D SLAM (lifecycle node, auto-activated)
?   ??? sim_mapping.launch.py             # Gazebo apartment + slam_toolbox (+ scripted sweep)
?   ??? sim_navigation.launch.py          # Gazebo apartment + map_server + AMCL + Nav2
?   ??? rtabmap.launch.py                 # Astra driver + RTAB-Map + OctoMap (3-D)
??? maps/                         # Saved apartment sim map (yaml, pgm, posegraph); see maps/README.md
??? scripts/
?   ??? waypoint_navigator.py     # Waypoint navigation node (`route` param picks names)
?   ??? waypoint_loader.py        # Waypoint YAML loader/validator (no ROS)
?   ??? go_to_waypoint.py         # CLI: Nav2 to a named waypoint, reports time-to-goal
?   ??? sim_map_sweep.py          # Scripted mapping sweep for the sim
?   ??? generate_sim_map.sh       # Regenerate maps/apartment.*
?   ??? run_nav_scenarios.sh      # Gazebo scenarios, fresh sim per run
?   ??? summarize_nav_results.py  # Markdown table from the scenario results
?   ??? render_map_overlay.py     # PNG of the map with the waypoints
?   ??? test_autonomous.py        # Autonomous test sequence
??? test/                         # ROS-free checks + the Gazebo scenario test
```

This is an `ament_cmake` package (`CMakeLists.txt` + `package.xml`); the Python
scripts are installed as ROS 2 executables.

## Build

```bash
cd ~/OmniBotPro/robot_ws
colcon build --packages-select omnibot_navigation
source install/setup.bash
```

## Launch

### Full autonomous robot (base + description + EKF + SLAM + Nav2)

```bash
ros2 launch omnibot_navigation autonomous_robot.launch.py
```

`autonomous_robot.launch.py` includes the Yahboom base control
(`omnibot_driver/yahboom_base_control.launch.py`), the robot description, the
shared EKF state estimation (`omnibot_bringup/state_estimation.launch.py`),
`slam_toolbox`, the Nav2 stack, and RViz.

Key launch arguments (defaults):

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use the Gazebo clock |
| `params_file` | `config/nav2_params.yaml` | Nav2 parameters |
| `autostart` | `true` | Autostart the Nav2 lifecycle |
| `use_slam` | `true` | Run `slam_toolbox` 2-D SLAM |
| `use_3d_mapping` | `false` | Also launch RTAB-Map + OctoMap (requires depth camera) |
| `use_rviz` | `true` | Start RViz |
| `rviz_config_file` | `omnibot_description/config/omnibot_navigation.rviz` | RViz config |

Set navigation goals from RViz with the **2D Nav Goal** tool, or send a
`navigate_to_pose` action goal.

### Waypoint following

```bash
ros2 launch omnibot_navigation autonomous_with_waypoints.launch.py
```

The `waypoint_navigator` node loads the named waypoints in `config/waypoints.yaml`
(map frame) and drives the robot through them with one Nav2
`navigate_through_poses` goal. The shipped poses are for the simulated apartment
map, not for a real room.

| `waypoint_navigator` param | Default |
|---|---|
| `waypoint_file` | `waypoints.yaml` |
| `route` | `""` (all waypoints, file order) |
| `loop_waypoints` | `false` |
| `waypoint_timeout` | `30.0` s |

### Gazebo simulation (apartment world)

Simulation only; needs ROS 2 Jazzy and Gazebo Harmonic (see the root
[README](../../README.md#nav2--slam_toolbox-in-gazebo-ohh-30) for the evidence and the time-to-goal table).

```bash
# 2-D SLAM in the apartment world; drive by hand or add sweep:=true for the scripted route
ros2 launch omnibot_navigation sim_mapping.launch.py
# Regenerate the checked-in map (maps/apartment.*), see maps/README.md
scripts/generate_sim_map.sh
# Nav2 + AMCL on the saved map, then a named goal from config/waypoints.yaml
ros2 launch omnibot_navigation sim_navigation.launch.py
ros2 run omnibot_navigation go_to_waypoint.py kitchen desk door
# The scenario test: fresh sim per run, fails if a waypoint is not reached in 60 s
scripts/run_nav_scenarios.sh --repeat 5 kitchen desk door
# Proof v0 scenario regression suite (10 start/goal/obstacle scenarios, OHH-62)
scripts/run_nav_regression_suite.sh
```

Nightly CI runs the full **scenario regression suite** in
[`.github/workflows/nav-regression-nightly.yml`](../../.github/workflows/nav-regression-nightly.yml)
(headless Gazebo in the navdev image stack). It uploads
`nav-regression-results.json`, a Markdown report, and a workflow badge in the
job summary. Scenario definitions live in
[`config/nav_regression_scenarios.yaml`](config/nav_regression_scenarios.yaml).

`autonomous_with_waypoints.launch.py` (and its `route:=kitchen,desk,door` argument)
starts `autonomous_robot.launch.py`, which brings up the real Yahboom base driver.
Do not use it in simulation; use `sim_navigation.launch.py` plus `go_to_waypoint.py`.

Goal tolerance: Nav2's goal checker uses 0.15 m and 0.20 rad
(`general_goal_checker` in `config/nav2_params.yaml`). The scenario test also
checks the final ground-truth pose against `/odom` within 0.30 m and 0.50 rad.

### 2-D SLAM only

```bash
ros2 launch omnibot_navigation slam_toolbox.launch.py
# Drive around, then save the map:
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 3-D RGB-D SLAM (RTAB-Map + OctoMap)

```bash
ros2 launch omnibot_navigation rtabmap.launch.py                 # physical robot
ros2 launch omnibot_navigation rtabmap.launch.py use_sim_time:=true   # Gazebo
```

On the physical robot this starts the Orbbec Astra driver, RTAB-Map RGB-D SLAM,
and an OctoMap server. In simulation the Astra driver is skipped (Gazebo's RGBD
sensor + `ros_gz_bridge` provide the same topics). See the file header of
`launch/rtabmap.launch.py` for the topic layout and Orbbec driver install steps.

## Configuration

- **`nav2_params.yaml`** — velocity caps (0.20 m/s, 1.0 m/s² accel) and
  `min_vel_x: -0.20` match the Yahboom driver's clamp/ramp limiter. AMCL uses
  the `OmnidirectionalMotionModel`.
- **`robot_localization.yaml`** — EKF fuses wheel odom as velocities only and
  the IMU as differential orientation + angular velocity (linear accel
  disabled). It is the single `odom→base_link` TF broadcaster, so the driver
  must run with `publish_tf: False`.
- **`rtabmap_params.yaml` / `octomap_params.yaml`** — consumed by
  `rtabmap.launch.py`.

## Hardware

| Component | Notes |
|---|---|
| Compute | Raspberry Pi 5 (8 GB) |
| Motor board | Yahboom ROS Robot Expansion Board (USB serial) |
| Depth camera | Orbbec Astra Pro (`/camera/color/*`, `/camera/depth/*`) |
| IMU | provided by the Yahboom board → `/imu/data` |
| Odometry | wheel encoders via the Yahboom board → `/odom` |

The Orbbec Astra driver (`ros2_astra_camera`) is built from source, not apt —
see `launch/rtabmap.launch.py` and `docs/SENSOR_INTEGRATION.md`.

## Troubleshooting

```bash
ros2 node list | grep nav2          # Nav2 nodes up?
ros2 topic echo /map                # SLAM producing a map?
ros2 topic echo /odom               # odometry flowing?
ros2 topic echo /imu/data           # IMU flowing?
ros2 topic echo /navigate_to_pose/_action/status   # navigation status
```
