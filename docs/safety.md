# Arm command and policy safety

Moved from the top-level README. Back to [README](../README.md).

`arm_driver_node` drops follower torque when `/emergency_stop` is true (`Torque_Enable` 0) and leaves it off when the stop clears, until a later `/arm/enable` true arrives while the stop is false. An enable message received during the stop does not re-enable torque. If `/arm/joint_commands/out` is silent for `command_timeout_sec` (0.2 s) the driver holds the last accepted joint command and logs that once. Each accepted target is clamped to `joint_min` / `joint_max` and to `max_joint_delta_rad` per cycle. The shared default is `MAX_JOINT_DELTA_RAD` (0.15 rad) in [`omnibot-ros2/omnibot_arm/scripts/arm_safety.py`](../omnibot-ros2/omnibot_arm/scripts/arm_safety.py). `policy_node` reuses that constant. The Yahboom base controller already zeros velocity on `/emergency_stop`; this change does not modify that node.

In `policy_node`, inference guards enforce that missing or black camera frames (wrist or BEV) refuse publication with a throttled warning, actions from `select_action` taking longer than the policy period (1 / `policy_hz`) are dropped as stale, and arm command deltas are clamped to the same `MAX_JOINT_DELTA_RAD` (0.15 rad/cycle) limit as the driver.

The decisions above are unit-tested without ROS and without the arm (`pytest omnibot-ros2/omnibot_arm/test omnibot-ai-ros2/omnibot_lerobot/test`). Feetech bus writes, `ros2 launch`, and real model inference on hardware were not run for this behavior. Position servos only; there is no force control.

`arm_sim_driver_node` is the Gazebo path. It calls the same `ArmSafety` object rather than a second copy of the limits, the 0.15 rad cap, the 0.2 s timeout, or the e-stop latch. On `/emergency_stop` true it rejects new `/arm/joint_commands/out` targets and publishes the measured joint pose once so the Gazebo position controllers hold that pose. Clearing the stop leaves commands rejected until `/arm/enable` true while the stop is clear. The URDF revolute stops stay the SO-101 calibration values; they are not widened to the software limits in `arm_params.yaml`.
