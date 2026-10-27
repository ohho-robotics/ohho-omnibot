# Results

## Sim / real parity results (OHH-37)

Same launch, same typed goal, 10 trials per goal by default. This file is written by `scripts/run_parity_trials.py`. It does not invent a pass, a time, an intervention count, or a bag path.

## Protocol

Each trial calls `demo_drive.drive` (the `scripts/demo_drive.py` path). The typed string is what `/agent/goal` receives. ScriptedBrain turns it into the mission command shown below (`go to the kitchen` → `navigate:kitchen`).

- Launch (sim and real): `ros2 launch omnibot_orchestration agent_nav2.launch.py`
- Sim stack that must already be up before a sim trial: `ros2 launch omnibot_navigation sim_navigation.launch.py`
- Sim clock: `use_sim_time:=true`. Real clock: `use_sim_time:=false`.
- Real mapping (`slam_toolbox` on the robot) and the real trials are not in this file.
- Serial transport and `--mode real` require an explicit `--real`. This render used transport `sim` and `real=false`, and did not open a serial port.
- A completed trial records a bag at `~/rosbags/parity_<sim|real>_<location>_trial<NN>`. No trial completed, so those paths were not created.

Typed goals:

- `go to the kitchen`
- `go to the desk`
- `go to the door`

## Why the sim column is a placeholder

Sim was not run (ros2 is not on PATH (ROS 2 Jazzy is required); gz is not on PATH (Gazebo Harmonic is required)). Sim results are placeholders. No bag was recorded.

Status `not_run` means the goal was dispatched on the in-memory sim transport inside this process. The robot did not move. Time, interventions, failure, and bag are blank because they were not measured.

## Why the real column is empty

Mapping with slam_toolbox and the real trials are Varun's. Real cells are blank on purpose. They are not zeros and they are not a pass.

## Summary

| Goal | Trials planned | Sim completed | Sim passed | Sim failed | Sim time min / median / max (s) | Real completed |
|---|---|---|---|---|---|---|
| go to the kitchen | 10 | 0 |  |  |  |  |
| go to the desk | 10 | 0 |  |  |  |  |
| go to the door | 10 | 0 |  |  |  |  |

## Trials

| Goal | Trial | Mission command | Sim status | Sim time (s) | Sim interventions | Sim failure | Sim bag | Real status | Real time (s) | Real interventions | Real failure | Real bag |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| go to the kitchen | 1 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 2 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 3 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 4 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 5 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 6 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 7 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 8 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 9 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the kitchen | 10 | navigate:kitchen | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 1 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 2 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 3 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 4 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 5 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 6 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 7 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 8 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 9 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the desk | 10 | navigate:desk | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 1 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 2 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 3 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 4 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 5 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 6 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 7 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 8 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 9 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |
| go to the door | 10 | navigate:door | not_run |  |  |  |  |  |  |  |  |  |

## Failures

No completed sim or real trial. No robot failure was observed, and none was invented. A measured failure uses status `fail` and puts the reason in the Sim failure or Real failure cell (timeout, pose tolerance, e-stop, or an abort after an intervention).

## Bags

No raw bags were recorded. Sim bag and Real bag are empty. A completed sim kitchen trial 1 would be `~/rosbags/parity_sim_kitchen_trial01`.

## OHH-52 — 20-trial real-robot eval

**Needs Varun: budget + robot.**

This section is a blank template. Fill it during the robot session described in
[gpu-train-runbook.md](gpu-train-runbook.md), after Varun has agreed a GPU
budget and a `best` ACT checkpoint exists. Leave the cells empty until then.

| Field | Value |
|---|---|
| Date | |
| Operator | |
| Task | |
| Checkpoint path | |
| Policy | ACT (`model_type:=act`) |
| `policy_hz` used | |

Mark each cell `yes` or `no`.

- **Success** and **Failure** are exclusive. One of them is `yes` on a finished trial.
- **Intervention** is independent. Mark `yes` when a person used `/emergency_stop`, set `/policy/enable` false, or moved the robot during that trial.

| Trial | Success | Failure | Intervention | Notes |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |
| 5 | | | | |
| 6 | | | | |
| 7 | | | | |
| 8 | | | | |
| 9 | | | | |
| 10 | | | | |
| 11 | | | | |
| 12 | | | | |
| 13 | | | | |
| 14 | | | | |
| 15 | | | | |
| 16 | | | | |
| 17 | | | | |
| 18 | | | | |
| 19 | | | | |
| 20 | | | | |

| Total | Success | Failure | Intervention |
|---|---|---|---|
| 20 | | | |

**Needs Varun: budget + robot.**
