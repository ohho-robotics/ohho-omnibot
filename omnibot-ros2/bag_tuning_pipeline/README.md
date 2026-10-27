# Bag tuning pipeline (OHH-36)

Offline analysis for **EKF (`robot_localization`)**, **odom vs IMU drift**, and **Nav2** tuning from rosbag2 or MCAP recordings. This package ships **synthetic fixtures only**; nothing here claims real-robot validation.

## Quick start (dev PC)

```bash
cd omnibot-ros2/bag_tuning_pipeline
python -m pip install -r requirements.txt
python scripts/build_fixtures.py   # regenerates committed rosbag2 + mcap if needed

# Plot trajectory vs goal
PYTHONPATH=. python -m bag_tuning.plot_trajectory fixtures/topics/nav_tune_mini.json -o /tmp/traj.png
PYTHONPATH=. python -m bag_tuning.plot_trajectory fixtures/rosbag2/nav_tune_mini -o /tmp/traj_bag.png

# Drift + EKF proposal (YAML + markdown)
PYTHONPATH=. python -m bag_tuning.analyze_bag fixtures/rosbag2/nav_tune_mini --out-dir /tmp/bag_tune_out

pytest tests
```

## Recording bags on the robot

1. On the Pi (hybrid or bringup stack running), use the Android **REC** button or:
   ```bash
   ros2 service call /rosbag_recorder/start std_msgs/msg/String "{data: 'nav_tune_YYYYMMDD'}"
   # ... drive mission / manual teleop ...
   ros2 service call /rosbag_recorder/stop std_msgs/msg/Empty
   ```
2. Default topics include `/odom`, `/odometry/filtered`, `/imu/data`, `/cmd_vel/out` (see `omnibot_hybrid/rosbag_recorder.py`).
3. For Nav2 goal comparison, also record a goal topic (e.g. echo goals to `/goal_pose`) or note the goal pose from the mission log.
4. Copy the bag directory (or MCAP) to your dev machine.

## Running the pipeline on a real bag

```bash
cd omnibot-ros2/bag_tuning_pipeline
PYTHONPATH=. python -m bag_tuning.plot_trajectory ~/rosbags/nav_tune_YYYYMMDD -o out/traj.png
PYTHONPATH=. python -m bag_tuning.analyze_bag ~/rosbags/nav_tune_YYYYMMDD --out-dir out/
```

Review `out/ekf_proposal.md` and compare against `omnibot_navigation/config/robot_localization.yaml`. Deploy **only after** on-robot checks.

## Nav2 draft params

`config/nav2_params_real_robot_unvalidated.yaml` — velocity/accel limits aligned with existing `nav2_params.yaml` and driver manifest (`yahboom_params.yaml`). **UNVALIDATED ON HARDWARE.**

## Human TODO (required before closing OHH-36 on hardware)

- [ ] Record straight-line and in-place rotation bags on the real robot.
- [ ] Re-run `analyze_bag.py` and confirm yaw RMS trend is acceptable.
- [ ] Merge EKF proposal into `robot_localization.yaml` only after `/odometry/filtered` looks stable for 60 s idle + 2 m drive.
- [ ] Validate `nav2_params_real_robot_unvalidated.yaml` on the physical base (figure-eight, goal tolerance).
- [ ] Do **not** treat fixture-derived plots or covariance suggestions as proof of navigation success.

## Fixtures

| Path | Purpose |
|------|---------|
| `fixtures/topics/nav_tune_mini.json` | Fast unit tests, no binary parser |
| `fixtures/rosbag2/nav_tune_mini/` | Tiny committed rosbag2 |
| `fixtures/mcap/nav_tune_mini/` | Tiny committed rosbag2 (MCAP storage) |

Regenerate with `python scripts/build_fixtures.py`.
