# omnibot-sim-vr-pick

Scripted simulation of a rigid-cube pick into a bin, recorded through the VR teleop contract. The collector is `scripted-sim` and `human_vr` is false. This directory is a dataset card and the OHH-99 report from one headless smoke. It does not contain episode parquet, video, or npy files.

The Hugging Face dataset `ohho-robotics/omnibot-sim-vr-pick` has not been created or uploaded.

## Scene

Gazebo Harmonic apartment world (`omnibot_bringup` `apartment.sdf`, world name `apartment`). That SDF is unchanged. `sim.launch.py` still defaults to `pick_place:=false` and `cmd_vel_from_mux:=false`.

`pick_place:=true` spawns three extra models after Gazebo is up:

- `pick_place_table`, static, 0.90 m by 0.55 m by 0.72 m, centre `(0.20, 1.15, 0.37)`
- `pick_place_cube`, dynamic 4 cm cube
- `pick_place_bin`, static open bin, 0.18 m by 0.16 m by 0.08 m

Cube and bin poses come from `random.Random(pick_place_seed)` in the order cube x, cube y, cube yaw, bin x, bin y, bin yaw. The same seed repeats the same poses. The smoke used seed 237.

## How the smoke was collected

`scripts/ohh237_scripted_vr_pick_smoke.sh` inside `omnibot-nav-dev:latest` on 14 October 2026.

- `ROS_LOCALHOST_ONLY=1`
- `ROS_DOMAIN_ID=88`. FastDDS cannot compute a multicast port for domain 237 (the port overflows above 232), so 88 was used. Domain 30 is the robot domain and is refused.
- Software rendering: `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe`, `QT_QPA_PLATFORM=offscreen`, `MESA_GL_VERSION_OVERRIDE=3.3`
- `ros2 launch omnibot_lerobot scripted_vr_pick.launch.py world:=apartment pick_place:=true pick_place_seed:=237 episodes:=3`
- The headset publishes only `/arm/joint_commands/teleop_raw`, `/cmd_vel/teleop_raw`, `/ohho/heartbeat` at 10 Hz, and `/control_mode` `teleop`.
- `teleop_safety` gates those onto `/arm/joint_commands/teleop` and `/cmd_vel/teleop`. `arm_cmd_mux` and `cmd_vel_mux` forward them in teleop. Gazebo's base command is bridged from `/cmd_vel/out` (`cmd_vel_from_mux:=true`). The arm sim driver still subscribes to `/arm/joint_commands/out`.
- The recorder uses `action_source:=vr`, `operator:=scripted-sim`, `repo_id:=local/omnibot-sim-vr-pick`.

Gazebo reported entity creation successful for the robot, `pick_place_table`, `pick_place_cube`, and `pick_place_bin`. The headset printed:

```
SCRIPTED_VR_PICK episodes=3 operator=scripted-sim human_vr=false seed=237
```

Each saved episode is about 13.1 s and 132 frames at 10 Hz. The OHH-99 joint-step metric on `observation.state` peaked at 0.107 rad, 0.0804 rad, and 0.0803 rad. Whether the cube ended in the bin was not measured.

Episode files stayed in the container at `/tmp/omnibot-sim-vr-pick` and were not copied into git.

## OHH-99 filter

`data_engine.quality.episode_filter` defaults: frozen video 0.5 s, camera skew 50 ms, joint step 0.5 rad, minimum duration 2.0 s. The run printed `OHH-99 total=3 kept=0 rejected=3`.

| episode | duration | frames | max joint step | result |
| --- | ---: | ---: | ---: | --- |
| 0 | 13.1002 s | 132 | 0.107 rad | reject |
| 1 | 13.1001 s | 132 | 0.0804 rad | reject |
| 2 | 13.1000 s | 132 | 0.0803 rad | reject |

Every episode was rejected because `observation.images.wrist` and `observation.images.bev` were identical for the whole episode (frozen duration equal to the episode duration). Duration and joint step were inside the filter limits. The filter summary marks camera skew as unchecked because the dataset has no per-camera timestamps.

This launch bridges `/camera/front/image_raw` only (`minimal_sensors:=true`). The recorder subscribes to `/camera/wrist/image_raw` and `/camera/base/bev/image_raw`, and stores a 240 by 320 black frame when those topics are silent. No camera images were synthesised to change the filter result.

Full text: [reject_report.md](reject_report.md) and [reject_report.json](reject_report.json). Sidecar from the headset: [collection.json](collection.json).

## TODO for Varun

- Record at least 50 human headset episodes of the October pick-into-bin task.
- Provide a Hugging Face token and push the validated set to `ohho-robotics/omnibot-sim-vr-pick`.
- Headset time on the Quest. This smoke did not use a headset or the real robot.

The acceptance bar for OHH-237 is a public Hugging Face dataset with at least 50 validated episodes. This smoke does not meet that bar.
