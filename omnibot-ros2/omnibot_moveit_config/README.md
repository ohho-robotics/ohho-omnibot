# omnibot_moveit_config

MoveIt 2 configuration for the SO-101 arm and gripper on the OmniBot. The mecanum base is a fixed virtual joint (`world` → `base_footprint`). Plans move the arm and the gripper through the ros2_control controllers in `omnibot_bringup`.

The arm has 5 DOF. `config/kinematics.yaml` sets `position_only_ik` on the `arm` group. That is the Cartesian goal mode: the solver matches the tool position. Pilz `LIN` still takes a pose. The orientation tolerance stays loose, and the tool position is the constraint that is solved.

## Demo

Mock hardware, `move_group`, and RViz:

```bash
ros2 launch omnibot_moveit_config demo.launch.py
```

`rviz:=false` is the headless path. `gz:=true` starts Gazebo Harmonic with `gz_ros2_control` instead of the mock `ros2_control_node`. `gui:=true` opens the Gazebo window.

## Robot

Start the bringup manager first. It already runs the controller manager. Then:

```bash
ros2 launch omnibot_bringup robot.launch.py
ros2 launch omnibot_moveit_config move_group.launch.py
```

`move_group.launch.py` uses the hardware URDF (`use_mock:=false`, `use_sim:=false`). `rsp:=true` also starts `robot_state_publisher` when that node is not already running.

## Planners

The default pipeline is `ompl` and the default planner is `RRTConnect`. The other pipeline is `pilz_industrial_motion_planner` with planners `PTP`, `LIN`, and `CIRC`. The RViz MotionPlanning panel uses those names. On `/move_action` they are `pipeline_id` and `planner_id`.

| Controller | Action |
|---|---|
| `arm_joint_trajectory_controller` | `/arm_joint_trajectory_controller/follow_joint_trajectory` |
| `gripper_controller` | `/gripper_controller/gripper_cmd` (`ParallelGripperCommand`) |

## Checks

```bash
python -m pytest omnibot-ros2/omnibot_moveit_config/test/test_moveit_config_files.py
```

That parses the SRDF and the YAML. The plan-and-execute launch test runs in the colcon `ros` job (`demo.launch.py rviz:=false gz:=false`).
