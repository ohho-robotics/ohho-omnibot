# 10-minute live demo (OHH-72)

Presenter script for a laptop talk. Simulation is the demo. Rehearsal and the one recording are Varun's. The recording stays off the git remote; see [offline-fallback.md](offline-fallback.md).

| Clock | Block |
|---|---|
| 0:00–2:00 | Context |
| 2:00–7:00 | Live, simulation first |
| 7:00–10:00 | Code and CI |
| 10:00 | Stop |

A phone timer stays visible. When a beat runs long, cut it. At 10:00, stop even if a slide is still open.

## Before you walk on

Prepare these the day before. Nothing in this list is installed or built on stage.

- A terminal at the repo root, with the CPU quickstart already installed (`ohho doctor` succeeds).
- [README](../../README.md) scrolled to the quickstart and the architecture diagram.
- The [CI workflow](../../.github/workflows/ci.yml) on `main`, and [nav-sim](../../.github/workflows/nav-sim.yml).
- [docs/reference.md](../reference.md) Gazebo section, the flat-world gif [`assets/omnibot_gazebo_flat.gif`](../../assets/omnibot_gazebo_flat.gif), and the apartment waypoint figure in the README.
- The local backup recording named in [offline-fallback.md](offline-fallback.md), on this machine only.

If ROS 2 Jazzy and a `colcon` install are already on the laptop, source them once before the talk and leave `sim.launch.py` **not** running. Starting Gazebo during the talk is optional and only for a machine you have already rehearsed.

## 0:00–2:00 — Context

Say this, in order:

1. OmniBot is OhhO's open-source ROS 2 Jazzy mobile manipulator: a mecanum base and an SO-101 arm, in one repository.
2. The laptop path needs no robot and no GPU. The three commands in the README quickstart are what the docs job runs. Hardware and GPU stacks are labelled in [docs/reference.md](../reference.md).
3. This talk shows that sim, then where the tests live.

Put the README architecture diagram on screen for the last 30 seconds. Teleop, Android and Quest, Nav2, VLA, and RL meet `cmd_vel_mux` and `arm_cmd_mux`. `/control_mode` (`teleop`, `nav2`, `vla`, `rl_nav`) and `/emergency_stop` gate both outputs. The bring-up default is teleop. Point at [Bring-up control mode](../reference.md#bring-up-control-mode) and [docs/safety.md](../safety.md). Leave the arm unlaunched.

## 2:00–7:00 — Live (simulation)

One Gazebo process only. [`sim_navigation.launch.py`](../../omnibot-ros2/omnibot_navigation/launch/sim_navigation.launch.py) already includes [`sim.launch.py`](../../omnibot-ros2/omnibot_bringup/launch/sim.launch.py). Do not start both.

### 2:00–3:30 — CPU sim (always this)

Same commands the docs job runs, from a venv prepared before the talk:

```bash
ohho doctor
ohho sim --robot omnibot --seconds 2
```

`ohho doctor` is the environment check. `ohho sim` runs the omnibot model for 2 seconds. No robot and no GPU.

If either command fails, leave the live block and open [offline-fallback.md](offline-fallback.md). Diagnose after the talk.

### 3:30–6:15 — Gazebo, or the committed evidence

**Rehearsed laptop** (Jazzy sourced, `install/setup.bash` present, this launch already timed in rehearsal):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omnibot_bringup sim.launch.py world:=flat gui:=false
```

The process stays up. That is expected. The [`sim-smoke`](../../.github/workflows/ci.yml) job runs the same launch headless and checks that `/odom`, `/scan`, `/imu`, and `/camera/front/image_raw` publish. Bridged topics are listed in [docs/reference.md](../reference.md#gazebo-harmonic-simulation). On stage, point at the terminal log. Describe motion only if `/odom` is actually changing in the window you have.

**Any other laptop:** do not start Gazebo cold. Show [`assets/omnibot_gazebo_flat.gif`](../../assets/omnibot_gazebo_flat.gif) and the README time-to-goal table (kitchen, desk, door). Those times are from local `ros:jazzy` runs checked into the repo, not from the machine in front of the audience. [`nav-sim`](../../.github/workflows/nav-sim.yml) reruns the same three scenarios.

A live Nav2 goal is a rehearsal variant, not a second process beside the flat world. Use it only when rehearsal 3 chose this as the single Gazebo launch:

```bash
ros2 launch omnibot_navigation sim_navigation.launch.py
ros2 run omnibot_navigation go_to_waypoint.py kitchen
```

Named waypoints are `kitchen`, `desk`, and `door`. If the goal is not accepted before 6:15, cut to the map figure and move on.

### 6:15–7:00 — Physical robot, only if it is already in the room

Use this beat only when a human already has the robot powered and a teleop session running before you start the timer. Talk about what is already on screen.

This script does not start [`robot_with_joy.launch.py`](../../omnibot-ros2/omnibot_bringup/launch/robot_with_joy.launch.py), perception, the arm, or a recorder, and it does not claim a real-robot demo works. When the robot is absent or idle, say the live path is simulation and start the code tour early.

## 7:00–10:00 — Code and CI

About half a minute each. Skip down the list when the clock is short. Safety is the one to keep if you can open only one file.

1. README quickstart. The `docs` job runs [`scripts/docs_test_readme.py`](../../scripts/docs_test_readme.py). Fences marked `cpu` run in a fresh venv. Fences marked `ros`, `hardware`, or `gpu` are printed and skipped.
2. [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml): `cpu` (pytest, no ROS), `docs`, `ros` (colcon in `ros:jazzy-ros-base-noble`), `sim-smoke` (headless Gazebo via [`test_sim_launch.py`](../../omnibot-ros2/omnibot_bringup/test/test_sim_launch.py)).
3. [`.github/workflows/nav-sim.yml`](../../.github/workflows/nav-sim.yml): kitchen, desk, and door. The nightly run repeats each scenario five times. Simulation only.
4. [docs/safety.md](../safety.md): joint limits, 0.15 rad per cycle, a 200 ms command hold, and an e-stop torque latch. The tests there do not start ROS or the arm.
5. README layout table. [`src/`](../../src/) is the colcon workspace (symlinks into the package trees).

## When the live sim fails

Switch to [offline-fallback.md](offline-fallback.md). Stay off the robot. Build or run containers locally; do not push images. Leave the recording on the presenter machine.

## Five rehearsals

Varun runs these. Each box stays open until that pass is done. Each pass uses a visible timer and ends by 10:00. Record after rehearsal 5, one take, under 10 minutes. The file is local; this change does not publish it.

- [ ] Rehearsal 1 — read the script aloud with the timer, no commands
- [ ] Rehearsal 2 — CPU sim only (`ohho doctor`, `ohho sim --robot omnibot --seconds 2`), and it fits in 2:00–3:30
- [ ] Rehearsal 3 — one Gazebo path: flat `sim.launch.py`, or the committed gif and the Nav2 table when Gazebo is not already warm
- [ ] Rehearsal 4 — stop the live sim at 4:00 and switch to the offline fallback without leaving the clock
- [ ] Rehearsal 5 — dress rehearsal, full 10 minutes, simulation first, robot beat only if the robot was already running

## Out of scope

Hardware bring-up, a claim that the real-robot demo works, publishing the recording, and pushing container images.
