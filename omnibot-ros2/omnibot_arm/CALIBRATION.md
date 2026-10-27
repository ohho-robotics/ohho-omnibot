# SO-101 home ticks

The arm driver converts servo counts to radians with

`radians = (ticks - home_ticks) / ticks_per_rad`.

`omnibot_arm/config/arm_params.yaml` still has the factory list

`[2048, 2048, 2048, 2048, 2048, 2048]`.

That is the STS3215 half-turn, not a measurement of this robot. Joint zero will be wrong until a calibration file is loaded. This repository does not contain measured ticks. Do not paste guessed numbers into `arm_params.yaml`.

Nothing here opens a serial port. E-stop, torque latch, `max_joint_delta_rad` (0.15), and `command_timeout_sec` (0.2) are unchanged.

## What file the launch loads

`ros2 launch omnibot_arm arm.launch.py` resolves home ticks, then passes them to the driver. A requested file that is missing or invalid aborts the launch. It does not silently return to 2048.

Precedence:

1. Launch arg `calibration_file:=` (full path to a yaml file).
2. Env `OMNIBOT_ARM_CALIBRATION` (full path, including `~`). This is the local override so measured ticks can live outside the git tree.
3. Launch arg `robot_id:=`, or else env `OMNIBOT_ARM_ROBOT_ID`. The file is `so101_<id>.yaml`.
   - If `OMNIBOT_ARM_CALIBRATION_DIR` is set, that directory is the only place searched.
   - Otherwise the file is `omnibot_arm/config/calibration/so101_<id>.yaml` (the installed share path at runtime).
4. The `home_ticks` list already in `arm_params.yaml`. If that list is still the factory 2048s, the launch prints a banner that says `NOT this robot`.

If both a path and a robot id are set, the path chooses the file, and the file's `robot_id` field must match the id you passed.

`robot_id` is a single token: letters, digits, `_`, and `-`. It cannot contain a path separator.

These launches accept the same two arguments. The driver also reads the env vars when the launch did not already resolve ticks (`mobile_manipulation` and the Pi launch pass the args through; the driver loads the file):

```text
ros2 launch omnibot_arm arm.launch.py calibration_file:=/path/to/so101_<id>.yaml
ros2 launch omnibot_arm arm.launch.py robot_id:=<id>
ros2 launch omnibot_bringup mobile_manipulation.launch.py robot_id:=<id>
ros2 launch omnibot_bringup master/omnibot_pi.launch.py robot_id:=<id>
```

Gazebo `simulation.launch.py` and `isaac_sim.launch.py` do not declare those args. Export `OMNIBOT_ARM_CALIBRATION` or `OMNIBOT_ARM_ROBOT_ID` before starting them. With neither set, the sim keeps the factory ticks and the driver logs the same banner.

## Yaml the driver accepts

```yaml
robot_id: "your_id"
home_ticks: []  # exactly six integers, each from 0 through 4095
```

Joint order is the `joint_names` order in `arm_params.yaml`:

1. `arm_shoulder_pan`
2. `arm_shoulder_lift`
3. `arm_elbow_flex`
4. `arm_wrist_flex`
5. `arm_wrist_roll`
6. `arm_gripper`

The converter also writes a `joints:` list (LeRobot name, motor id, `drive_mode`, `homing_offset`, `range_min`, `range_max`, `home_tick`). If that list is present it must agree with `home_ticks`. A mismatch aborts the load.

`config/calibration/` is where per-robot files go. It ships with no `so101_*.yaml`. The synthetic file under `test/fixtures/` is for CI only. Do not point a robot at it.

## Convert lerobot-calibrate JSON

The tool only reads JSON and writes YAML.

```text
python omnibot-ros2/omnibot_arm/scripts/lerobot_calibration_to_yaml.py INPUT.json --robot-id YOUR_ID --output omnibot-ros2/omnibot_arm/config/calibration/so101_YOUR_ID.yaml
```

After the package is installed, the same script is `ros2 run omnibot_arm lerobot_calibration_to_yaml.py`.

For each joint:

`home_tick = (range_min + range_max) // 2`

That is LeRobot's degree-mode zero (`mid = (range_min + range_max) / 2` in the motors bus, truncated to an integer count). It is in the same Present_Position frame the driver reads **after** `lerobot-calibrate` has written `Homing_Offset` into the servo. The `homing_offset` field is copied into the yaml and is not added into `home_tick`.

A full 0..4095 range (current LeRobot does this for `wrist_roll`) becomes home tick **2047**, not 2048. `(0 + 4095) // 2 = 2047`.

The JSON must be either:

- one object per joint named `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`, each with integer `range_min` and `range_max` (`range_min < range_max`, both in 0..4095), optional `id` (must be 1..6 in that order), `drive_mode` 0 or 1, and `homing_offset`; or
- the older arrays `start_pos` and `end_pos` (six integers each), with the same midpoint rule. Optional `drive_mode` and `homing_offset` arrays.

Extra top-level keys on the per-joint file are rejected so a typo'd joint name is not dropped.

`drive_mode` other than 0 is written through and the tool prints a warning. The driver does not flip joint sign.

## Run calibration on the robot

This section is for the physical SO-101. CI does not run it. Confirm the serial ports on that machine before plugging in; `arm_params.yaml` currently names follower `/dev/ttyACM0` and leader `/dev/ttyACM1`.

Use the flags from the installed `lerobot-calibrate --help` if they differ. The 2026 LeRobot SO-101 flow is:

```text
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=YOUR_ID
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=YOUR_ID
```

An older install used:

```text
lerobot-calibrate --robot-type=so101 --robot-port=/dev/ttyACM0
lerobot-calibrate --robot-type=so101 --robot-port=/dev/ttyACM1
```

The command prints the JSON path when it finishes. Current LeRobot stores files under:

```text
~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<id>.json
~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/<id>.json
```

Convert the **follower** JSON. The driver has one `home_ticks` vector and uses it for both buses. Load that follower yaml with `calibration_file:=` or `robot_id:=`.

Put the yaml outside the repo if you do not want to commit it, and export:

```text
OMNIBOT_ARM_CALIBRATION=/absolute/path/so101_YOUR_ID.yaml
```

On launch you should see `Loaded SO-101 home_ticks ...` and no factory banner. `ros2 param get /arm_driver_node home_ticks` should show those six numbers when you started from `arm.launch.py` (that launch writes the resolved list onto the node).

## Checks that still need the arm

- The six converted ticks match a Present_Position read at the pose you want to call zero radians, with the calibration homing offset still stored in the servos. Resetting servo homing and keeping this yaml will not match.
- `wrist_roll` home tick 2047 (full-range record) is the zero you want, or record a different pose and edit the yaml from a real read. Do not invent the number.
- Gripper: LeRobot's own 0–100 scale puts 0 at `range_min`. This loader uses the range midpoint as 0 radians for the gripper too. Confirm open and closed against the URDF.
- Any `drive_mode` other than 0. The warning means joint direction was not applied.
- Leader and follower JSON midpoints differ. The driver cannot load two vectors. Decide which pose is zero before teleop.
- A motion after load stays inside the joint limits and the 0.15 rad per-cycle cap, and `/emergency_stop` still drops torque.
