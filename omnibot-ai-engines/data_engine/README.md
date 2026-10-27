# OmniBot Data Engine

Tooling for collecting, converting, validating, and loading robot demonstrations
in **LeRobot v2.0 format** (Parquet + MP4 video) for SmolVLA training.

## Format

All datasets use the LeRobot v2.0 layout:

```
dataset/
  meta/
    info.json          ← fps, total_episodes, total_frames, feature shapes
    tasks.jsonl        ← task index → task string
    episodes.jsonl     ← episode index → task list, length
    stats.json         ← per-feature mean/std/min/max for normalisation
  data/
    chunk-000/
      episode_000000.parquet   ← state, action, timestamps (one row per frame)
  videos/
    chunk-000/
      observation.images.wrist/episode_000000.mp4
      observation.images.bev/episode_000000.mp4
```

State and action are the unified **9-D** mobile-manipulation spec (6 arm joints
+ 3 base velocities), defined in `schema/constants.py`
(`MOBILE_MANIP_STATE_SPEC` / `MOBILE_MANIP_ACTION_SPEC`). Camera streams written
to the dataset are `observation.images.wrist` and `observation.images.bev`
(wrist close-up and the stitched bird's-eye view). Those two keys are defined
once in `schema/camera_keys.py`. The physical front camera feeds the BEV
stitcher and is not a dataset feature.

## Setup

```bash
pip install -e .          # installs the `vla_data_engine` package
# or: pip install -r requirements.txt
```

## Usage

### ROS 2 bag → dataset (single episode)
```bash
python -m data_engine.ingestion.bag_to_omnibot \
    --bag /data/bags/ep000 \
    --dataset /data/lerobot/pick_place \
    --task "pick up the red cube" \
    --fps 30
```

### Batch convert
```bash
python -m data_engine.scripts.ingest_dataset \
    --input-dir /data/bags \
    --output-dir /data/lerobot/pick_place \
    --task "pick up the red cube"
```

### Validate
```bash
python -m data_engine.scripts.validate_dataset --dataset /data/lerobot/pick_place
python -m data_engine.scripts.validate_lerobot_load --dataset /data/lerobot/pick_place
```

### Sync jitter (dataset card)
```bash
python -m data_engine.scripts.measure_sync_jitter --dataset /data/lerobot/pick_place
```

See `dataset_card_template.md` for fields to publish after a real robot capture.

### Quality filter (OHH-99)

Filter out corrupted demonstrations before policy training and output a keep/reject report:

```bash
python -m data_engine.scripts.filter_episodes \
    --dataset /data/lerobot/pick_place \
    --output /data/lerobot/pick_place/meta/filter_results.json
```

#### Rejection Rules & CLI Thresholds

An episode is rejected if any of the following conditions occur:
1. **Frozen video**: Consecutive frames with mean absolute pixel difference $\le$ threshold (`--frame-diff-threshold 2.0` on a 0–255 scale) for >0.5 s (`--max-frozen-s 0.5`).
   - Production episodes are lossy `.mp4` video files where compression/decompression introduces slight byte-level quantization noise (typically $\pm$1–2 intensity levels, MAD ~0.5–1.8) on static scenes. Exact pixel equality fails to catch stuck cameras in lossy video; a default tolerance of 2.0 treats this codec noise as identical frames while preserving genuine motion (MAD $\gg 2.0$). Pass `--frame-diff-threshold 0.0` for exact byte-for-byte pixel matching.
2. **Camera timestamp skew**: Wrist vs bird's-eye view camera timestamp delta >50 ms (`--max-camera-skew-s 0.05` or `--max-camera-skew-ms 50.0`).
   - `--missing-camera-timestamps {unchecked,reject}` (default: `unchecked`): When separate per-camera timestamps are missing, the skew is not measured (`max_camera_skew_s` is reported as `null`). In `unchecked` mode, the episode is kept and the report notes `camera_skew: unchecked (no per-camera timestamps)` with a count in the summary. In `reject` mode, episodes without per-camera timestamps are rejected with that reason.
   - *Recorder Gap*: Ingested LeRobot v2.0 datasets standardly write a single shared `timestamp` column. Measuring camera skew requires per-camera timestamp columns (e.g., `observation.images.wrist.timestamp`, `observation.images.bev.timestamp`) or sidecar timestamp arrays (`videos/.../{episode}_timestamps.npy`). Safe recorder node changes to expose hardware frame timestamps require ROS 2 Jazzy and physical/simulated cameras to verify.
3. **Joint step**: Joint position change >0.5 rad between consecutive frames (`--max-joint-step-rad 0.5`).
   - Evaluates shortest angular distance across the $\pm\pi$ wrap for continuous joints (configured via `--continuous-joints`, default: `wrist_roll`). Limited joints such as `shoulder_pan` (limited to $[-1.92, 1.92]$ rad per the robot URDF in `omnibot.urdf.xacro`) retain raw differences so large steps are rejected.
4. **Short episode**: Episode duration shorter than 2.0 s (`--min-duration-s 2.0`).

The filter outputs a JSON report with summary statistics, lists of `kept` and `rejected` episode indices, and per-episode metrics and failure reasons. In synthetic tests or environments without video codecs, `FrameSource` abstracts frame access and loads `.npy` arrays directly.

> **Demonstration Count Rule (OHH-49)**:
> OHH-49 ("Collect 50–100 demonstrations of one manipulation task") demonstration counts must **only** include episodes that this quality filter keeps (`status == "keep"`, listed in `kept`). Flawed episodes (frozen camera, timestamp skew, long pauses, or joint jumps) must be discarded and excluded from demonstration counts and policy training sets.

The collection procedure (two sessions, varied object positions, this filter) is [docs/demo-collection-checklist.md](../../docs/demo-collection-checklist.md). The Hugging Face card for `varunvaidhiya/omnibot-pick-place` is the blank template [docs/datasets/omnibot-pick-place/README.md](../../docs/datasets/omnibot-pick-place/README.md). Recording stays on the OHH-48 teleop recorder / LeRobot v2 writer; this tree does not add a second recorder.

### Visualise an episode
```bash
python -m data_engine.visualization.visualize_episode \
    --dataset /data/lerobot/pick_place \
    --episode 0
```

See `TRAINING_GUIDE.md` for the full end-to-end workflow.
