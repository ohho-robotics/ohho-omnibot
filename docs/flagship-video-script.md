# DRAFT for Varun — OmniBot flagship video (OHH-39)

**Status: DRAFT for Varun. Not approved for recording. Not approved for publish.**

| | |
|---|---|
| Ticket | OHH-39 |
| Deliverable | Script and shot list only |
| Films | 90-second flagship, then a 5-minute walkthrough |
| Recorded in this change | No |
| Merge | Do not merge this branch to `main` from this ticket |

Read the voiceover aloud once before any shoot. Pace is about 140 words a minute. Every number below is copied from a file in this repo. If a later run changes a table, change the line before anyone records it.

On-screen labels used throughout:

| Label | Meaning |
|---|---|
| `SIM` | Gazebo / `ohho sim` / a checked-in sim log |
| `REAL STILL` | A photograph or existing desk loop of the physical robot |
| `CODE` | Source, a unit test, or a trace file |
| `TABLE` | A results table already in the repo |

---

## 90-second flagship — voiceover

Teleprompter copy. About 190 words, about 81 seconds of speech, with picture holds taking it to 0:90.

**0:00–0:08**
This is OmniBot. Four mecanum wheels, an SO-101 arm, and the open ROS 2 stack behind them.

**0:08–0:18**
You can run it on a laptop before you own the robot. Three commands. No GPU.

**0:18–0:28**
pip install ohho-os. ohho doctor. ohho sim, robot omnibot, two seconds. That smoke test is what continuous integration runs.

**0:28–0:42**
The same repo has the wheel kinematics, the Yahboom base driver, arm limits and an e-stop, Gazebo, and Nav2 on a saved apartment map.

**0:42–0:58**
In simulation, fifteen runs. Kitchen, desk, and door. All fifteen reached the goal. Median time: seventeen seconds, twenty seconds, twelve point eight seconds. Read the table. It is simulation time, on a planar drive, not the physical robot.

**0:58–1:10**
When you drive, teleop wins. Xbox, the Android app, a Quest, or the browser on rosbridge. Autonomy does not talk over you.

**1:10–1:22**
Say go to the kitchen. The checked-in agent is scripted: it plans navigate-to, a verifier approves it, and a goal outside the map is blocked.

**1:22–1:30**
Apache two-point-oh. Star the repo. Run the sim. Then build the robot. OmniBot.

---

## 90-second flagship — shot list

| ID | In | Out | Picture | Voiceover beat | Super | Asset | Label |
|---|---|---|---|---|---|---|---|
| F1 | 0:00 | 0:08 | Desk hero. Hold the May 5 side view, then the three-quarter view. Wheels and the white arm fill the frame. No screen content behind the robot if it shows another product UI. | “This is OmniBot…” | `OmniBot` / `mecanum base + SO-101` / `ROS 2 Jazzy` | [`assets/PXL_20260505_121303728.jpg`](../assets/PXL_20260505_121303728.jpg), [`assets/PXL_20260505_121328008.jpg`](../assets/PXL_20260505_121328008.jpg) | `REAL STILL` |
| F2 | 0:08 | 0:18 | Cut to a laptop terminal, empty prompt, then the three commands typing on. Keep the physical robot small in the corner so the laptop path stays distinct from the hardware. | “You can run it on a laptop…” | `No robot. No GPU.` | Commands in [README quickstart](../README.md#quickstart-no-robot-no-gpu) | `CODE` |
| F3 | 0:18 | 0:28 | Full-screen terminal. Exact lines, in order. Hold `ohho sim` through the two-second run. | “pip install ohho-os…” | `docs-test job · ci.yml` | [`.github/workflows/ci.yml`](../.github/workflows/ci.yml), [`scripts/docs_test_readme.py`](../scripts/docs_test_readme.py) | `CODE` |
| F4 | 0:28 | 0:42 | Five quick cuts, one per noun in the line: kinematics snippet, Yahboom driver, arm e-stop note, flat Gazebo camera, apartment map. | “The same repo has…” | One label per cut: `kinematics` · `Yahboom driver` · `arm e-stop` · `Gazebo` · `Nav2 map` | [mecanum README](../mecanum-kinematics/README.md), [safety](safety.md), [`assets/omnibot_gazebo_flat.gif`](../assets/omnibot_gazebo_flat.gif), [map PNG](../omnibot-ros2/omnibot_navigation/docs/apartment_map_waypoints.png) | `CODE` then `SIM` |
| F5 | 0:42 | 0:58 | Apartment map full frame. Flash yellow, red, blue, green to match the sentence order start → kitchen → desk → door. Then the time-to-goal table. Leave the table up for the whole median sentence. | “In simulation, fifteen runs…” | `SIM · planar holonomic drive` then `15 / 15 reached` | [Time to goal](../README.md#time-to-goal), [map PNG](../omnibot-ros2/omnibot_navigation/docs/apartment_map_waypoints.png) | `SIM` `TABLE` |
| F6 | 0:58 | 1:10 | Four-icon board: Xbox, Android, Quest, browser. Under it, the mux test name. Use a real device recording only if that recording exists; otherwise stay on the icon board. | “When you drive, teleop wins…” | `teleop preempts Nav2 · unit test` | [`test_cmd_vel_mux.py`](../omnibot-ros2/omnibot_hybrid/test/test_cmd_vel_mux.py) `TestTeleopPreemptsAutonomy` | `CODE` |
| F7 | 1:10 | 1:22 | Split screen. Left: kitchen trace, `navigate_to` highlighted. Right: verifier trace, the blocked `(99, 99)` line. | “Say go to the kitchen…” | `ScriptedBrain · not a filmed policy` | [kitchen trace](evidence/ohh-31/kitchen_goal_trace.jsonl), [reject trace](evidence/ohh-31/verifier_reject_outside_map.jsonl) | `CODE` |
| F8 | 1:22 | 1:30 | End card on black. Repo URL, licence, the three-command install, YouTube. | “Apache two-point-oh…” | `Apache-2.0` · `pip install ohho-os` · `github.com/ohho-robotics/OmniBot` | [LICENSE](../LICENSE) | end card |

Terminal text for F3, exact:

```bash
pip install ohho-os
ohho doctor
ohho sim --robot omnibot --seconds 2
```

Map colours for F5, from the README caption: yellow start, red `kitchen`, blue `desk`, green `door`. Grey is unknown space.

`assets/Omnibot_demo1.gif` is a physical indoor loop and it carries a `makeagit.com` watermark. Crop that bug out, or leave the gif out of the public 90-second cut and stay on the May 5 stills. `assets/Omnibot_demo2.gif` is the same class of asset: use it only for motion that is actually in the frames. Do not caption either gif with a task name.

Gazebo picture for F4: [`assets/omnibot_gazebo_flat.gif`](../assets/omnibot_gazebo_flat.gif) is the front camera in `world:=flat` while `/cmd_vel` is published. The red object is the `marker` model in `flat.sdf`. It is not the apartment `door` waypoint.

---

## 5-minute walkthrough — voiceover

About 700 words. At 140 words a minute the read is five minutes. The table beat (W6) may run long. Let the graphic hold. Keep the simulation label on screen until the beat ends.

**0:00–0:25 · W1**
This is the five-minute walkthrough of OmniBot, the open-source mobile manipulator from OhhO Robotics. A mecanum base, an SO-101 arm, ROS 2 Jazzy. I am going to show you what already runs, and I am going to read the numbers from the tables in the repo. If a result is simulation, the lower third says simulation.

**0:25–0:55 · W2**
These are the desk photographs from the repository, taken on the fifth of May. You can see the four mecanum wheels, the white SO-101 arm, a Raspberry Pi in the chassis, and the cameras on the deck. The loops in the assets folder are the physical robot indoors. They are short. They are a look at the machine. The public cut crops the watermark on the first demo loop.

**0:55–1:25 · W3**
The laptop path does not need the robot and does not need a GPU. The commands on screen are the ones continuous integration runs: pip install ohho-os, ohho doctor, and ohho sim with robot omnibot for two seconds. Two seconds is a smoke test. It proves the package starts. The apartment navigation demo is a different launch, later in this walkthrough.

**1:25–1:55 · W4**
The Gazebo path needs Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic. Launch the flat world and the front camera publishes this view: the robot driving toward the red marker. The bridged topics are command velocity, odometry, the laser scan, the IMU, and the front camera. Rosbridge can sit on port 9090 so a browser can drive the same simulation. That launch does not exit, so the docs test skips it. The sim-smoke job in CI is what checks the topics.

**1:55–2:35 · W5**
The apartment world is one room: a kitchen counter, a desk, and a door. Slam toolbox built this map in simulation, and the map is checked in. Yellow is the start pose. Red is kitchen. Blue is desk. Green is door. Grey is unknown space, inside the furniture and outside the walls. Nav2 on this map uses a planar holonomic drive, with the same speed caps as the Yahboom driver: zero point two metres a second, and one radian a second. The published table is that planar drive. Wheel-physics mecanum with Nav2 is a separate attempt, and the map frame did not come up before the startup timeout.

**2:35–3:15 · W6**
Here is the time-to-goal table. Each run is a fresh headless Gazebo, robot at the origin, facing plus X. The clock is simulation time from goal acceptance to Nav2 success, with a sixty-second limit. A pass also needs the ground-truth pose within zero point three metres and zero point five radians of the waypoint. Kitchen: five of five, fifteen point seven, seventeen point oh, seventeen point five. Desk: five of five, eighteen point eight, twenty point oh, twenty-one point six. Door: five of five, eight point two, twelve point eight, thirteen point four. Fifteen of fifteen. These fifteen runs are local, in a Jazzy container, software rendering, sixteen CPUs, real-time factor one. They are a local log. The nightly workflow writes its own copy of this table, and those times can differ.

**3:15–3:40 · W7**
Safety is in the same repo, and it was unit-tested without the physical arm. An emergency stop drops follower torque and leaves it off until a later enable, and only if the stop is already clear. If arm commands go quiet for two hundred milliseconds, the driver holds the last accepted target. Each cycle clamps the joint step to zero point one five radians. Switching control mode publishes a zero twist so a latched velocity does not run on.

**3:40–4:10 · W8**
Teleop is the mode the mux starts in. Xbox, the Android app, the Quest project, and the browser all meet the robot through that gate. While the mux is in teleop, Nav2 twists are ignored. That behaviour is a unit test in the hybrid package. The Quest client is Unity, and the video path documented today is MJPEG. WebRTC is still on the roadmap. Headset latency stays off this walkthrough until a measured number is in the repo.

**4:10–4:40 · W9**
The agent evidence is a trace. The checked-in brain is scripted. The goal text is go to the kitchen. Perceive, reason, verify, act, then reflect. Reason calls navigate_to. Verify approves one call. The mission command is navigate, kitchen. A second trace sends a goal at ninety-nine, ninety-nine, and verify blocks it: outside the map envelope. You can regenerate the files with the evidence script. No ROS required for that regeneration.

**4:40–5:00 · W10**
What you can run today is the CPU package, the Gazebo smoke, and this navigation table. Real-robot Nav2, a priced bill of materials, and GPU policy scores are still open work. The licence is Apache two-point-oh. The install is pip install ohho-os. The repository is ohho-robotics slash OmniBot.

---

## 5-minute walkthrough — shot list

| ID | In | Out | Picture | Super | Asset | Label |
|---|---|---|---|---|---|---|
| W1 | 0:00 | 0:25 | Title over the May 5 three-quarter still. Fade the lower third in before the first sentence ends. | `DRAFT walkthrough` · `labels match the repo` | [`PXL_20260505_121328008.jpg`](../assets/PXL_20260505_121328008.jpg) | `REAL STILL` |
| W2 | 0:25 | 0:55 | Slow push across both stills. Optional two-second crop of a demo gif with the watermark outside the frame. | `Desk photos · 5 May 2026` | [`PXL_20260505_121303728.jpg`](../assets/PXL_20260505_121303728.jpg), [`Omnibot_demo1.gif`](../assets/Omnibot_demo1.gif), [`Omnibot_demo2.gif`](../assets/Omnibot_demo2.gif) | `REAL STILL` |
| W3 | 0:55 | 1:25 | Terminal. Same three commands as F3. Circle `--seconds 2`. | `CPU smoke · ci.yml docs-test` | [README quickstart](../README.md#quickstart-no-robot-no-gpu) | `CODE` |
| W4 | 1:25 | 1:55 | Play `omnibot_gazebo_flat.gif`. Cut to the bridged-topic table in the reference doc. End on `rosbridge` port `9090`. | `SIM · world:=flat · front camera` | [Gazebo section](reference.md#gazebo-harmonic-simulation), [`omnibot_gazebo_flat.gif`](../assets/omnibot_gazebo_flat.gif) | `SIM` |
| W5 | 1:55 | 2:35 | Map PNG. Pointer hits yellow, red, blue, green as the line names them. Then a one-line callout: planar drive, `0.20 m/s`, `1.0 rad/s`. | `SIM map · kitchen / desk / door` | [map PNG](../omnibot-ros2/omnibot_navigation/docs/apartment_map_waypoints.png), [`waypoints.yaml`](../omnibot-ros2/omnibot_navigation/config/waypoints.yaml) | `SIM` |
| W6 | 2:35 | 3:15 | Full-screen results table from the section below. Highlight the row as it is read. Keep `LOCAL SIM · not the nightly artifact` in the corner for the whole beat. | `15 / 15 · sim time · RTF 1.0` | [Time to goal](../README.md#time-to-goal), [jsonl](../omnibot-ros2/omnibot_navigation/docs/nav_sim_runs_local.jsonl) | `TABLE` |
| W7 | 3:15 | 3:40 | Three cards: torque latch, `0.2 s` hold, `0.15 rad` clamp. Source file path in the corner. | `Unit-tested · no arm attached` | [docs/safety.md](safety.md), [`arm_safety.py`](../omnibot-ros2/omnibot_arm/scripts/arm_safety.py) | `CODE` |
| W8 | 3:40 | 4:10 | Icon board from F6, then the Unity version pin `6000.5.2f1`, then the roadmap line that WebRTC is still open (MJPEG today). | `Mux starts in teleop` | [VR README](../omnibot-vr/README.md), [Android README](../omnibot-android/README.md), [root roadmap](../README.md#roadmap) | `CODE` |
| W9 | 4:10 | 4:40 | Scroll the kitchen JSONL. Hold on `scripted: go to kitchen` and `tools: navigate_to`. Cut to the reject file on the outside-map sentence. | `ScriptedBrain · /mission/command navigate:kitchen` | [WALKTHROUGH](evidence/ohh-31/WALKTHROUGH.md), [summary.json](evidence/ohh-31/summary.json) | `CODE` |
| W10 | 4:40 | 5:00 | Three open items as quiet cards (real-robot Nav2, priced BOM, GPU policy scores), then the same end card as F8. | `Apache-2.0` | [reference needs table](reference.md#needs-hardware--needs-gpu), [BOM](../README.md#hardware-bill-of-materials) | end card |

Commands the walkthrough may show on screen. Speak them only if the terminal is actually showing that launch. W3 is the smoke test. W4 and W5 are the ROS launches.

```bash
pip install ohho-os
ohho doctor
ohho sim --robot omnibot --seconds 2
```

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omnibot_bringup sim.launch.py world:=flat
```

```bash
ros2 launch omnibot_navigation sim_navigation.launch.py
ros2 run omnibot_navigation go_to_waypoint.py kitchen
```

Agent lines, from the OHH-31 walkthrough. The checked-in trace was produced by `python scripts/ohh31_record_evidence.py` without ROS. A live `ros2 topic pub` is a later capture, and the lower third still has to say `ScriptedBrain` unless a different reasoner is the one running.

```bash
ros2 topic pub --once /agent/goal std_msgs/msg/String "{data: 'go to the kitchen'}"
```

---

## Sim vs real

Use this as the claim lock for both films. The “Say” column is safe. The “Leave out” column stays off the audio and off the supers.

| Beat | Picture you have | Say | Leave out | Evidence |
|---|---|---|---|---|
| Physical machine | May 5 desk stills; short indoor gif loops | A mecanum chassis and an SO-101 arm exist and are photographed | Any timed task, pick success, charge time, or top speed measured on the carpet | [`assets/`](../assets/) |
| CPU install | Terminal of the three commands | `ohho-os` installs, `ohho doctor` runs, `ohho sim --robot omnibot --seconds 2` is the CI smoke. No robot, no GPU | That the two-second sim is the apartment Nav2 demo | [README quickstart](../README.md#quickstart-no-robot-no-gpu), [reference](reference.md#needs-hardware--needs-gpu) |
| Flat Gazebo | Front-camera gif, red marker | Headless or GUI Gazebo Harmonic, `world:=flat`, topics `/cmd_vel`, `/odom`, `/scan`, `/imu`, `/camera/front/image_raw`. sim-smoke checks motion of at least 0.2 m | That this world contains kitchen, desk, and door | [reference Gazebo section](reference.md#gazebo-harmonic-simulation) |
| Apartment Nav2 | Map PNG + results table | 15/15 local sim runs, planar holonomic drive, DWB, velocity caps 0.20 m/s and 1.0 rad/s matching the driver. Medians 17.0 s, 20.0 s, 12.8 s | Real-robot Nav2 or SLAM. Wheel-physics `drive:=mecanum` as the source of the table. “CI proved these exact times” — the committed numbers are the local Docker log; nightly CI writes a fresh table | [Time to goal](../README.md#time-to-goal) |
| Arm safety | Cards, not a servo close-up unless you film one later | E-stop torque latch, 0.2 s command hold, 0.15 rad/cycle clamp. Position servos | Force control. A filmed e-stop on the real arm, until you film it | [docs/safety.md](safety.md) |
| Teleop mux | Icon board or a real session you record later | Mux boots in `teleop`. Nav2 twists are dropped in teleop. Mode changes publish a zero Twist | A four-device montage you do not have. Quest WebRTC. A latency number | [reference bring-up](reference.md#bring-up-control-mode), [OHH-31 walkthrough](evidence/ohh-31/WALKTHROUGH.md) |
| Agent | JSONL on screen | Scripted reasoner, `navigate:kitchen`, verifier blocks `(99, 99)` as outside the map | “The language model drove the robot.” Cloud Claude is an adapter in the architecture doc; this evidence file is the scripted path | [kitchen trace](evidence/ohh-31/kitchen_goal_trace.jsonl), [summary](evidence/ohh-31/summary.json) |
| Policies | Omit from both cuts | — | OpenVLA, SmolVLA, or Isaac Lab success rates. There is no results table for them in this repo | [vla_engine README](../omnibot-ai-engines/vla_engine/README.md), [reference GPU rows](reference.md#needs-hardware--needs-gpu) |
| BOM | Omit prices | Parts list exists; prices are TBD | Any dollar figure | [Hardware BOM](../README.md#hardware-bill-of-materials) |
| Learning loop | Omit from the 90-second cut. Optional chapter title only in a later film | The learning engine has a numpy core and a benchmark command | A Pi-vs-Jetson latency table. None is checked in | [learning_engine README](../omnibot-ai-engines/learning_engine/README.md) |

Pass rule, so the table graphic matches the README: Nav2 must report success inside 60 s, and the final `/odom` pose must lie within 0.30 m and 0.50 rad of the waypoint. Nav2’s own goal checker is tighter (0.15 m and 0.20 rad). Quote the published min / median / max. Do not invent a per-run average on camera.

---

## Results tables

These are the only numeric tables either film may show. There is no real-robot results table in the repo. Do not design one for the video.

### Time to goal (simulation)

Source of truth, with the method note above the numbers:

- [README · Time to goal](../README.md#time-to-goal)
- Per-run log: [`omnibot-ros2/omnibot_navigation/docs/nav_sim_runs_local.jsonl`](../omnibot-ros2/omnibot_navigation/docs/nav_sim_runs_local.jsonl)
- Text log: [`omnibot-ros2/omnibot_navigation/docs/nav_sim_runs_local_log.txt`](../omnibot-ros2/omnibot_navigation/docs/nav_sim_runs_local_log.txt)
- Map the graphic is drawn from: [`apartment_map_waypoints.png`](../omnibot-ros2/omnibot_navigation/docs/apartment_map_waypoints.png)
- Waypoint coordinates: [`waypoints.yaml`](../omnibot-ros2/omnibot_navigation/config/waypoints.yaml)
- How to rebuild the markdown table: [`summarize_nav_results.py`](../omnibot-ros2/omnibot_navigation/scripts/summarize_nav_results.py)
- Nightly / manual CI that reprints the table into the job summary: [`.github/workflows/nav-sim.yml`](../.github/workflows/nav-sim.yml)

Figures as published (local runs, `ros:jazzy-ros-base-noble`, Mesa, 16 CPUs, real-time factor 1.0). Min / median / max seconds:

| Scenario | Goal (map frame) | Runs | Passed | Time to goal, min / median / max (s) |
|---|---|---|---|---|
| `kitchen` | x −1.90, y 0.70, yaw 180° | 5 | 5 | 15.7 / 17.0 / 17.5 |
| `desk` | x 2.55, y −0.45, yaw 90° | 5 | 5 | 18.8 / 20.0 / 21.6 |
| `door` | x −0.30, y −1.45, yaw −90° | 5 | 5 | 8.2 / 12.8 / 13.4 |

Spoken medians, both films: kitchen seventeen, desk twenty, door twelve point eight. The graphic shows `12.8`, so the voice says twelve point eight.

### What needs hardware or a GPU

[docs/reference.md · Needs hardware / needs GPU](reference.md#needs-hardware--needs-gpu)

That table is the other one worth a frame in W10. Rows the walkthrough should point at:

| On camera | Needs | Where the reference sends you |
|---|---|---|
| `ohho sim --robot omnibot` | Neither | `ohho-os` 1.1.2 and the docs-test job |
| Nav2 + slam_toolbox in the Gazebo apartment | Neither (needs Jazzy + Gazebo Harmonic) | README time-to-goal section and the local run log |
| Xbox, cameras, arm, LeRobot recording | Robot | bring-up launch files |
| Nav2 and SLAM on the real robot | Robot | `omnibot_navigation` — not run on hardware for that write-up |
| OpenVLA / SmolVLA, Isaac Lab RL | GPU | engine READMEs and their `requirements.txt` |
| Android app, Quest teleop | Robot, plus SDK / Unity to build | `omnibot-android/`, `omnibot-vr/` |

### Agent traces (not a timing table)

[docs/evidence/ohh-31/WALKTHROUGH.md](evidence/ohh-31/WALKTHROUGH.md)

| File | What to put on screen |
|---|---|
| [`summary.json`](evidence/ohh-31/summary.json) | `kitchen_mission_command`: `navigate:kitchen` |
| [`kitchen_goal_trace.jsonl`](evidence/ohh-31/kitchen_goal_trace.jsonl) | `scripted: go to kitchen` → `navigate_to` → `plan verified` → `success: true` |
| [`verifier_reject_outside_map.jsonl`](evidence/ohh-31/verifier_reject_outside_map.jsonl) | blocked `navigate_to`, goal `(99.00, 99.00)` outside the map envelope |

Regenerate with `python scripts/ohh31_record_evidence.py`.

### Tables that do not exist yet

Leave these off both films until a file in the repo holds the numbers:

- Real-robot time to goal, slam quality, or battery life
- Policy success rate, latency, or demo count for OpenVLA / SmolVLA / Isaac
- LeRobot episode counts after the quality filter (OHH-49). The filter script is documented; a kept-episode total is not checked in
- Quest or Android round-trip latency
- BOM prices (the price column is `TBD`)

---

## Lower thirds (copy-paste)

- `SIM · Gazebo Harmonic · planar drive`
- `LOCAL LOG · 15/15 · RTF 1.0 · not the nightly artifact`
- `REAL STILL · 5 May 2026`
- `CPU · no robot · no GPU`
- `ScriptedBrain · verifier on`
- `Unit test · arm not attached`
- `teleop preempts Nav2`

---

## Sign-off

Varun reviews this draft before anyone records. This ticket stops at the script.

- [ ] Numbers still match [Time to goal](../README.md#time-to-goal) and the jsonl
- [ ] Door median spoken as 12.8 wherever the table is legible
- [ ] `SIM` stays up for the whole of F5 and W6
- [ ] Demo gif watermark handled
- [ ] No policy score, no BOM price, no real-robot Nav2 claim
- [ ] Approved to record (separate ticket)
