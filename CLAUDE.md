# OmniBot agent reference (ROS names & topics)

Canonical names for code, launch files, and docs. **Do not** use legacy `smolvla_node` or arm mux mode `smolvla`.

## ROS nodes (executable = node name unless noted)

| Role | Package | Executable | Node name | Notes |
|------|---------|------------|-----------|--------|
| LeRobot visuomotor inference | `omnibot_lerobot` | `policy_node` | `policy_node` | Multi-backend via `model_type` |
| OpenVLA base policy | `omnibot_vla` | `vla_node` | `vla_node` | Publishes `/cmd_vel/vla` |
| Arm command mux | `omnibot_rl` | `arm_cmd_mux` | `arm_cmd_mux` | Modes below |
| Base command mux | `omnibot_hybrid` | `cmd_vel_mux` | `cmd_vel_mux` | `/control_mode` gating |
| BEV stitcher | `omnibot_lerobot` or `ros2_bev_stitcher` | `bev_stitcher_node` | `bev_stitcher_node` | Required for `policy_node` |

Launch flags: `use_policy` includes `policy_inference.launch.py` (see `hybrid_robot.launch.py`).

## `model_type` vs mux mode

- **`model_type`** (parameter on `policy_node`): LeRobot registry name — `smolvla` (default), `act`, `diffusion`, `openvla`.
- **`/arm/cmd_mode`** (arm mux): `policy` | `rl_arm` — selects `/arm/joint_commands` vs `/arm/joint_commands/rl`.
- **`/control_mode`** (base + arm gate): `teleop` | `nav2` | `vla` | `rl_nav`.

## Policy control topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/policy/task` | `std_msgs/String` | Task instruction for `policy_node` |
| `/policy/enable` | `std_msgs/Bool` | Enable inference |

## Topic map (learning / logging)

Mirrored in `omnibot-ai-engines/learning_engine/ros2/topics.py`.

| Topic | Notes |
|-------|--------|
| `/odom` | Base state |
| `/arm/joint_states` | Arm feedback |
| `/camera/front/image_raw` | Front RGB |
| `/camera/wrist/image_raw` | Wrist RGB |
| `/camera/base/bev/image_raw` | Bird's-eye (policy + recorder) |
| `/cmd_vel/out` | Post-mux base command |
| `/arm/joint_commands/out` | Post-mux arm command |
| `/emergency_stop` | `true` latches e-stop |

## Arm joint names

SO-101 joints use the `arm_` prefix in commands (see `omnibot_arm` params). Default `arm_driver_node` joint names in some configs omit this prefix — see `omnibot-ai-ros2/omnibot_rl/config/rl_arm_params.yaml`.

## Mechanical constants

Mecanum wheel spacing and limits: `mecanum-kinematics/config/mecanum_params.yaml`, `omnibot_arm` safety (`MAX_JOINT_DELTA_RAD` = 0.15 rad/cycle).

## Test coverage

CPU package line coverage is measured in CI (`scripts/run_cpu_coverage.py`). See [docs/test-coverage.md](docs/test-coverage.md).
