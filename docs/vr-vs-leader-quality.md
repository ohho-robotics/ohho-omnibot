# VR vs leader-arm dataset quality

OHH-244 compares a real VR teleop set with a leader-arm set of the same task.
Both inputs are LeRobot v2 datasets on disk. This file is the note template.
The real-data table is empty because no Quest session and no leader-arm
session were scored in this change. Fifty real VR episodes are still to be
recorded. Publishing the Hugging Face dataset is a separate step. This tool
does not record and does not upload to Hugging Face.

The scorer is `omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py`.
It reads the layout the OHH-226 recorder writes on branch
`origin/ohh-226-recorder-vr-source` (`meta/info.json`, `meta/episodes.jsonl`,
`data/chunk-*/episode_*.parquet`, optional wrist and BEV timestamp columns).
That branch is not modified here. Source names are `vr` and `leader`, matching
`action_source` on that recorder. A directory is accepted when it has
`meta/info.json` or parquet under `data/`, the same check as
`looks_like_lerobot_v2` on `origin/ohh-243-act-vr-loop`. Train and serve code
on that branch is not modified.

## What each number is

Jerk and smoothness use the `action` column (9-D arm plus base, the OHH-226
vector). The L2 mixes radians and base velocities, so compare datasets that
use that same layout.

- **Time per episode**: last timestamp minus first, using the OHH-99 duration
  rule (nanosecond stamps above 1e12 are converted).
- **Action jerk**: mean and max, over episodes, of the L2 norm of the third
  forward difference of action divided by `dt**3`. Needs at least four frames.
- **Mean action step (L2)**: mean of `||a[t] - a[t-1]||`. Per frame, so both
  sets should be recorded at the same rate. The recorder default is 30 Hz.
- **Action smoothness**: `1 / (1 + mean action step)`. 1 is a constant action.
  Higher is smoother.
- **Wrist-BEV timestamp skew**: mean and max absolute difference between
  `observation.images.wrist.timestamp` and
  `observation.images.bev.timestamp`. The table is in milliseconds. Missing
  per-camera timestamps are unchecked, not a zero.
- **Frame drops**: gaps longer than 1.5 frame periods. Dropped frames are
  `floor(gap * fps) - 1` summed over those gaps. The rate is dropped frames
  divided by (frames present + dropped frames).
- **Quality filter**: `data_engine.quality.episode_filter.score_episode`.
  Reject reasons are frozen video (identical frames longer than 0.5 s),
  wrist/BEV skew over 50 ms, joint step over 0.5 rad, and duration under 2 s.
  Reject counts are episodes, not repeated reason strings. An episode with
  two frozen cameras counts once as `frozen_video`.

`--vr` and `--leader` label every episode in that directory. `--dataset`
reads `source` or `action_source` on the episode row, then those keys on
`info.json`, then `operator` only when it is exactly `vr` or `leader`.
OHH-226 stores `operator: vr` when `action_source` is vr and operator is
empty. The leader path stores an empty operator and does not write
`action_source` on the episode row. Pass two directories.

## Real-data table

Fill this table by running the compare command below and copying the
generated comparison. Leave these cells blank until that run uses the real
VR set and the leader-arm set. Do not paste numbers from the unit-test
fixtures.

<!-- real-data-table -->
| Metric | VR (real) | Leader-arm (real) | VR - leader |
| --- | --- | --- | --- |
| Episodes |  |  |  |
| Kept by quality filter |  |  |  |
| Rejected by quality filter |  |  |  |
| Reject reasons |  |  |  |
| Mean time per episode (s) |  |  |  |
| Median time per episode (s) |  |  |  |
| Min time per episode (s) |  |  |  |
| Max time per episode (s) |  |  |  |
| Mean action jerk |  |  |  |
| Max action jerk |  |  |  |
| Mean action step (L2) |  |  |  |
| Action smoothness (0-1, higher is smoother) |  |  |  |
| Mean wrist-BEV timestamp skew (ms) |  |  |  |
| Max wrist-BEV timestamp skew (ms) |  |  |  |
| Frame-drop gaps |  |  |  |
| Dropped frames |  |  |  |
| Frame-drop rate |  |  |  |

## Commands after recording

Record on the robot with the OHH-226 recorder (`action_source:=vr` is not on
this branch). The task string is the OHH-237 / OHH-243 sim pick task
(`pick cube into bin`). Record about 50 VR episodes. The tool does not check
that count.

`teleop_record.launch.py` on that branch does not declare an `output_dir`
launch argument. The yaml default is `~/datasets/mobile_manipulation`
(`omnibot_lerobot/config/policy_params.yaml`). Record one source, move the
directory aside, then record the other so the two roots stay separate.

```bash
ros2 launch omnibot_lerobot teleop_record.launch.py \
  action_source:=leader \
  robot_kind:=real \
  task_name:="pick cube into bin"

mv ~/datasets/mobile_manipulation ~/datasets/mobile_manipulation_leader

ros2 launch omnibot_lerobot teleop_record.launch.py \
  action_source:=vr \
  robot_kind:=real \
  task_name:="pick cube into bin"

mv ~/datasets/mobile_manipulation ~/datasets/mobile_manipulation_vr
```

Score both roots on a workstation. This does not need the robot, does not
record, and does not upload.

```bash
python omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py \
  --vr ~/datasets/mobile_manipulation_vr \
  --leader ~/datasets/mobile_manipulation_leader \
  --json-out docs/evidence/ohh-244/vr-vs-leader-quality.json \
  --markdown-out docs/evidence/ohh-244/vr-vs-leader-quality.md
```

If a single v2 root already has `action_source` set to `vr` or `leader` on
each `meta/episodes.jsonl` row:

```bash
python omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py \
  --dataset ~/datasets/mobile_manipulation \
  --json-out docs/evidence/ohh-244/vr-vs-leader-quality.json \
  --markdown-out docs/evidence/ohh-244/vr-vs-leader-quality.md
```

Threshold flags match `data_engine.scripts.filter_episodes` (`--min-duration-s`,
`--max-frozen-s`, `--max-camera-skew-s`, `--max-joint-step-rad`,
`--frame-diff-threshold`, `--missing-camera-timestamps`). `--drop-factor`
defaults to 1.5.

The numpy fallback (`episode_*/data.npz`, written when LeRobot is not
installed on the leader path) is refused. Point `--leader` at a v2 root.
A chunk file such as `data/chunk-000/file-000.parquet` is scored when it has
`episode_index`. Per-episode `episode_NNNNNN.parquet` wins if both exist.

Copy the generated comparison into the table above only after the paths are
the real recordings. Keep this template's cells blank in git until then.

## Not done here

- No Quest session, no leader-arm session, and no count of 50 real episodes.
- No Hugging Face upload (`ohho-robotics/omnibot-sim-vr-pick` or a real-VR repo).
- No edit to the OHH-226 recorder branch or to OHH-243 train/serve code.
- No change to deadman, e-stop, or speed caps.
