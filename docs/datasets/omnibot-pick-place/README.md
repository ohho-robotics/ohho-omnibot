---
license: apache-2.0
task_categories:
  - robotics
tags:
  - LeRobot
  - robotics
  - omnibot
pretty_name: OmniBot pick and place
configs:
  - config_name: default
    data_files: data/*/*.parquet
---

# varunvaidhiya/omnibot-pick-place

**Template.** Copy this file to the Hugging Face dataset repo [`varunvaidhiya/omnibot-pick-place`](https://huggingface.co/datasets/varunvaidhiya/omnibot-pick-place) only after a real collection. Every blank below stays empty in git. This repository has no episodes and does not upload this dataset.

Procedure: [demo collection checklist](../../demo-collection-checklist.md). Format and the OHH-99 count rule: [data engine README](../../../omnibot-ai-engines/data_engine/README.md). Recording path: OHH-48 `teleop_recorder_node` → ohho-os `Recorder` → LeRobot v2.0 (`codebase_version` `v2.0`).

The demonstration count is `summary.kept_episodes` from `meta/filter_results.json` (`status == "keep"`, the `kept` list). Rejected episodes (frozen camera, timestamp skew, long pauses, joint jumps, short takes) stay out of that count.

## Hardware

Documented platform (confirm this unit). Blanks are for the robot you actually recorded.

| Field | Value |
|---|---|
| Robot | OmniBot: mecanum base + SO-101 (Feetech STS3215), Xbox teleop. Unit id: |
| Base driver | Yahboom serial. Port on this unit: |
| Leader arm port | |
| Follower arm port | |
| Wrist camera | `observation.images.wrist` from `/camera/wrist/image_raw`. Device: |
| Bird's-eye view | `observation.images.bev` from `/camera/base/bev/image_raw` (four base cameras stitched). Calibration file date: |
| Depth camera | Orbbec Astra Pro is on the robot for Nav2. It is not a dataset feature. |
| Record rate | 30 Hz unless `record_hz` was changed. Actual fps in `meta/info.json`: |
| State / action | 9-D: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`, `base_vx`, `base_vy`, `base_vz` ([`MOBILE_MANIP_STATE_SPEC`](../../../omnibot-ai-engines/data_engine/schema/constants.py)) |

## Task

One task for the whole dataset. Paste the string stored in `meta/tasks.jsonl` (the recorder writes `TeleopRecorderNode.task_description`).

| Field | Value |
|---|---|
| `task_index` | |
| Task string | |
| Intended behavior (one sentence) | |

## Episodes

Fill from `meta/filter_results.json` and `meta/info.json` after both sessions in the checklist. Leave zeros out: an empty cell means "not collected yet."

| Field | Value |
|---|---|
| Session 1 date | |
| Session 2 date | |
| Session 1 object positions | |
| Session 2 object positions | |
| Episodes in `meta/info.json` `total_episodes` | |
| `summary.total_episodes` | |
| `summary.kept_episodes` (`len(kept)`, the OHH-49 count; target 50–100) | |
| `summary.rejected_episodes` | |
| `summary.keep_ratio` | |
| `kept` episode indices | |
| `rejected` episode indices | |
| `total_frames` (info.json, all saved frames) | |
| Frames in kept episodes only | |

## Known issues

Paste measured values. A blank means the measurement was not run. Sync JSON is the stdout of `python -m data_engine.scripts.measure_sync_jitter` (OHH-48). The OHH-48 [dataset_card_template.md](https://github.com/ohho-robotics/OmniBot/blob/ohh-48-lerobot-recorder/omnibot-ai-engines/data_engine/dataset_card_template.md) is the same paste target for jitter.

| Issue | This dataset |
|---|---|
| Wrist↔BEV max skew (ms) | |
| Wrist↔BEV p95 skew (ms) | |
| State↔wrist max skew (ms) | |
| Episodes with `camera_skew: unchecked (no per-camera timestamps)` | |
| Rejected: frozen video / long pause (>0.5 s at frame-diff threshold 2.0) | |
| Rejected: camera skew >50 ms | |
| Rejected: joint step >0.5 rad | |
| Rejected: duration <2.0 s | |
| Numpy fallback episodes excluded from this repo | |
| Other (calibration, dropped cameras, task-string mismatch) | |

`measure_sync_jitter` JSON:

```json
```
