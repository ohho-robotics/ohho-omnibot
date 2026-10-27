# OHH-49 demo collection checklist

Collect **50–100 kept demonstrations** of one manipulation task for the Hugging Face dataset [`varunvaidhiya/omnibot-pick-place`](https://huggingface.co/datasets/varunvaidhiya/omnibot-pick-place). This file is the procedure. It contains no episodes. The demonstration count is the length of `kept` after the [OHH-99 quality filter](../omnibot-ai-engines/data_engine/README.md#quality-filter-ohh-99). Fill the blanks in the [dataset card template](datasets/omnibot-pick-place/README.md) from the filter report after both sessions. Leave them empty until then.

Record with the existing teleop recorder and LeRobot v2 writer (OHH-48). This checklist adds no recorder, no converter, and no dataset files.

## Pipeline

OHH-48 (`ohh-48-lerobot-recorder`) is the live LeRobot v2 path. Use it. The pieces already on `main` are linked below.

| Step | What to run | Where it lives |
|---|---|---|
| Bring up gamepad, wrist, BEV, optional leader arm, and the recorder | `ros2 launch omnibot_lerobot lerobot_record.launch.py` | [lerobot_record.launch.py](https://github.com/ohho-robotics/OmniBot/blob/ohh-48-lerobot-recorder/omnibot-ai-ros2/omnibot_lerobot/launch/lerobot_record.launch.py) on `ohh-48-lerobot-recorder`. It includes [`teleop_record.launch.py`](../omnibot-ai-ros2/omnibot_lerobot/launch/teleop_record.launch.py). |
| Record | [`teleop_recorder_node`](../omnibot-ai-ros2/omnibot_lerobot/omnibot_lerobot/teleop_recorder_node.py) | OHH-48 writes through [`ohho_teleop_recorder.py`](https://github.com/ohho-robotics/OmniBot/blob/ohh-48-lerobot-recorder/omnibot-ai-ros2/omnibot_lerobot/omnibot_lerobot/ohho_teleop_recorder.py) (`OhhoTeleopRecorder`, ohho-os `Recorder`) into [`append_live_episode`](https://github.com/ohho-robotics/OmniBot/blob/ohh-48-lerobot-recorder/omnibot-ai-engines/data_engine/recording/live_episode.py) (LeRobot v2.0 Parquet + MP4). |
| Parameters | [`policy_params.yaml`](../omnibot-ai-ros2/omnibot_lerobot/config/policy_params.yaml) `teleop_recorder_node` | `output_dir`, `repo_id`, `record_hz` (30), `episode_timeout_s` (60), `joy_record_button` (7), `joy_discard_button` (6). The launch file does not declare overrides for these. |
| Bags already on disk | `python -m data_engine.ingestion.bag_to_omnibot` | [`bag_to_omnibot.py`](../omnibot-ai-engines/data_engine/ingestion/bag_to_omnibot.py) (`codebase_version` `v2.0`). Batch: `python -m data_engine.scripts.ingest_dataset`. |
| Schema check | `python -m data_engine.scripts.validate_dataset --dataset <output_dir>` | [`validate_dataset.py`](../omnibot-ai-engines/data_engine/scripts/validate_dataset.py). OHH-48 also adds `python -m data_engine.scripts.validate_lerobot_load`. |
| Quality filter | `python -m data_engine.scripts.filter_episodes` | [`filter_episodes.py`](../omnibot-ai-engines/data_engine/scripts/filter_episodes.py). |
| Sync numbers for the card | `python -m data_engine.scripts.measure_sync_jitter --dataset <output_dir>` | [OHH-48 script](https://github.com/ohho-robotics/OmniBot/blob/ohh-48-lerobot-recorder/omnibot-ai-engines/data_engine/scripts/measure_sync_jitter.py). Paste the JSON into the card. Leave the cells blank if you have not run it. |

`lerobot_record.launch.py` needs `ohho-os` and `PYTHONPATH` including `omnibot-ai-engines`. If that import fails, the node logs a numpy fallback and writes `episode_#####/data.npz`. Those files are outside the LeRobot v2 layout. They are excluded from `kept` and from the Hub repo.

Layout, camera keys, and the 9-D state/action spec: [data engine README](../omnibot-ai-engines/data_engine/README.md), [`camera_keys.py`](../omnibot-ai-engines/data_engine/schema/camera_keys.py) (`observation.images.wrist`, `observation.images.bev`), [`constants.py`](../omnibot-ai-engines/data_engine/schema/constants.py). End-to-end context: [TRAINING_GUIDE.md](../omnibot-ai-engines/data_engine/TRAINING_GUIDE.md) sections 3 and 4. Pre-record topic checks are in TRAINING_GUIDE §1.3.

Before session 1, set:

- `repo_id` to `varunvaidhiya/omnibot-pick-place` in `teleop_recorder_node` (the checked-in default is `local/mobile_manipulation`).
- `output_dir` to the directory you will filter and later upload.
- `TeleopRecorderNode.task_description`. The node assigns the literal `mobile manipulation task` in code. `task_description` in `policy_params.yaml` is the policy node, not the recorder. Put the pick-and-place sentence you will train on into the recorder before either session, and use that same sentence for every episode.

Upload the dataset only after both sessions, the filter, and a filled card. This repository does not push to the Hub.

## Controls

From [`teleop_recorder_node`](../omnibot-ai-ros2/omnibot_lerobot/omnibot_lerobot/teleop_recorder_node.py) and [`policy_params.yaml`](../omnibot-ai-ros2/omnibot_lerobot/config/policy_params.yaml):

| Control | Parameter | Behavior |
|---|---|---|
| Record toggle | `joy_record_button` 7 (node log: RB) | Start, then press again to save the episode. |
| Discard | `joy_discard_button` 6 (node log: LB) | Drop the open episode. The index does not advance. |
| Timeout | `episode_timeout_s` 60 | Saves the open episode. A timeout save still has to pass the filter. |
| Leader arm | `/arm/leader_states` | Arm action. Base comes from `/joy` (left stick vx/vy, right stick X vz), clamped in the node to 0.2 m/s and 1.0 rad/s. |

Save a take only when the grasp and place succeed. Discard a miss with the discard button before the record toggle. A saved failure still occupies an episode index until the filter rejects it, and it still must stay out of the count.

## Two sessions, varied positions

One task string. Two sessions. The 50–100 target is the combined `kept` list, not a per-session quota. Spread object starts across the workspace. [TRAINING_GUIDE §3.4](../omnibot-ai-engines/data_engine/TRAINING_GUIDE.md) uses about ±15 cm in position and about ±45° in orientation, and a successful take of about 5–20 s (the filter floor is 2.0 s).

### Session 1

- [ ] Wrist (`/camera/wrist/image_raw`), BEV (`/camera/base/bev/image_raw`), leader, follower, `/odom`, and `/joy` are publishing. BEV calibration for this mount is already done (`bev_calibrate` in the training guide).
- [ ] `repo_id`, `output_dir`, and `task_description` match the card you will fill.
- [ ] Object start poses are on the operator's left and in the center of the reachable table, with the approach yaw you use most often.
- [ ] Each take moves the object to a different start. Re-record a miss from a new start after discarding.
- [ ] Hands and the object stay in frame. The arm keeps moving through the grasp and the place.
- [ ] Write the session log row below before you shut the robot down.

### Session 2

A second sitting (another day, or a real break with the robot powered down and brought back). Same task string. Same `output_dir` so episodes append.

- [ ] Lighting or table background differs from session 1 (room lights, time of day, or a shifted backdrop).
- [ ] Object start poses are on the operator's right and farther from the base than session 1. Approach yaw differs by about 45° from the session 1 yaw. Do not reuse session 1 start poses.
- [ ] Same success, discard, and in-frame rules.
- [ ] Write the session 2 log row.

### Session log

Leave these cells empty in git. Fill them on the robot, then copy the numbers into the dataset card.

| | Session 1 | Session 2 |
|---|---|---|
| Date | | |
| Operator | | |
| Task string written into `meta/tasks.jsonl` | | |
| Object starts (describe the poses; no invented indices) | | |
| Episodes discarded before save | | |
| Episodes saved this session | | |

## Quality filter reminders (OHH-99)

Run this on the LeRobot v2 directory after session 2. Substitute your `output_dir`. The sample path is the one in the data engine README.

```bash
python -m data_engine.scripts.filter_episodes \
    --dataset /data/lerobot/pick_place \
    --output /data/lerobot/pick_place/meta/filter_results.json
```

The demonstration count is `summary.kept_episodes`, which is the length of `kept`. Copy `kept`, `rejected`, and `summary` into the dataset card. Episodes in `rejected` stay out of the count and out of the training set.

While recording, these are the default rejects (`FilterThresholds` in [`episode_filter.py`](../omnibot-ai-engines/data_engine/quality/episode_filter.py)):

1. **Frozen video / long pause.** Consecutive frames whose mean absolute pixel difference is ≤ `--frame-diff-threshold` 2.0 (0–255 scale) for longer than `--max-frozen-s` 0.5 s. Lossy MP4 noise sits under that tolerance; a stuck camera or a long hold does not. Keep the wrist and the BEV view changing through the take.
2. **Camera timestamp skew.** Wrist versus bird's-eye timestamp delta above `--max-camera-skew-s` 0.05 (50 ms). The OHH-48 writer stores `observation.images.wrist.timestamp` and `observation.images.bev.timestamp`, which this filter reads. If those columns are absent, the default `--missing-camera-timestamps unchecked` keeps the episode and sets `camera_skew` to `camera_skew: unchecked (no per-camera timestamps)`. An unchecked skew is not a measured pass. Record `summary["camera_skew: unchecked (no per-camera timestamps)"]` on the card. Use `--missing-camera-timestamps reject` when you require a measured skew.
3. **Joint jump.** A joint step above `--max-joint-step-rad` 0.5 between frames. `wrist_roll` uses the shortest wrap across ±π (`--continuous-joints`, default `wrist_roll`). Other joints use the raw difference. Move the leader smoothly.
4. **Short episode.** Duration under `--min-duration-s` 2.0 s.

`status == "keep"` is the only episode that counts. Record more successful takes and re-run the filter until `len(kept)` is between 50 and 100. Stop at or before 100 kept episodes.

Then fill [the dataset card](datasets/omnibot-pick-place/README.md). Sync figures come from `measure_sync_jitter` on the OHH-48 branch, or stay blank.
