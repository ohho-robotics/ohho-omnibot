# Saved simulation map: apartment

`apartment.*` is a 2-D occupancy map of the Gazebo `apartment` world
([`omnibot_bringup/worlds/apartment.sdf`](../../omnibot_bringup/worlds/apartment.sdf)),
built by `slam_toolbox` from the simulated 2-D lidar. It is simulation data. It
has nothing to do with a real room or the real robot.

| File | What it is |
|---|---|
| `apartment.yaml` | `nav2_map_server` metadata: image, 0.05 m resolution, origin, thresholds |
| `apartment.pgm` | Occupancy grid, 138 x 105 cells (6.9 m x 5.25 m) |
| `apartment.posegraph`, `apartment.data` | `slam_toolbox` serialized pose graph (`serialize_map`). Needed only to continue mapping or to run `slam_toolbox` in localization mode on this map. Nav2 uses the yaml and pgm. |

## Frame convention

The robot spawns at the Gazebo world origin facing +X (`sim.launch.py`
`-x 0 -y 0`), and `slam_toolbox` starts its `map` frame at the robot's first
pose. So the `map` frame and the Gazebo world frame coincide. That is why the
named waypoints in [`../config/waypoints.yaml`](../config/waypoints.yaml) can
be written in map coordinates and still be checked against `apartment.sdf`.
`test/test_nav_assets.py` checks this: walls and furniture in the pgm sit
within 0.15 m of where `apartment.sdf` puts them.

## Regenerate

Needs ROS 2 Jazzy and Gazebo Harmonic (Ubuntu 24.04, or the `ros:jazzy-ros-base-noble` image as
in [`.github/workflows/nav-sim.yml`](../../../.github/workflows/nav-sim.yml)). Simulation only.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src --packages-select omnibot_description omnibot_arm omnibot_hybrid omnibot_rl omnibot_bringup omnibot_navigation
source install/setup.bash
omnibot-ros2/omnibot_navigation/scripts/generate_sim_map.sh
```

[`generate_sim_map.sh`](../scripts/generate_sim_map.sh):

1. runs `ros2 launch omnibot_navigation sim_mapping.launch.py sweep:=true`
   (headless Gazebo + `slam_toolbox`, [`config/slam_toolbox_params.yaml`](../config/slam_toolbox_params.yaml)),
2. waits for [`sim_map_sweep.py`](../scripts/sim_map_sweep.py) to finish its fixed route
   (a full turn, a lap of the room at 0.25 m/s staying 0.5 m or more from furniture, a full turn, back at the start pose),
3. saves the grid with `nav2_map_server map_saver_cli` and the pose graph with the
   `/slam_toolbox/serialize_map` service into `maps/` (or the directory you pass).

It took about three minutes in the run that produced the checked-in map. Regeneration is
not byte-for-byte reproducible: a second run produced a map one pixel column wider
(139 x 105 versus 138 x 105), so do not diff the files; re-run `test_nav_assets.py`
instead. To map by hand instead, run
`ros2 launch omnibot_navigation sim_mapping.launch.py`, drive with
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`, then run the two save
commands from the script.

After regenerating, run `pytest omnibot-ros2/omnibot_navigation/test`. It checks
the map still matches `apartment.sdf` and that `kitchen`, `desk`, and `door` are
in free space and reachable. If the world or the waypoints change, regenerate the map and
re-run the Gazebo scenarios (`scripts/run_nav_scenarios.sh kitchen desk door`).
