# OmniBot mobile manipulation dataset card

## Schema

- **State / action**: 9-D (6 arm joints + base vx, vy, vz)
- **Cameras**: `observation.images.wrist`, `observation.images.bev` only (no `observation.images.front`)

## Recording

```bash
ros2 launch omnibot_lerobot lerobot_record.launch.py
```

Requires `ohho-os` and `PYTHONPATH` including `omnibot-ai-engines` on the recording machine.

## Sync jitter (camera vs state)

Run after recording:

```bash
python -m data_engine.scripts.measure_sync_jitter --dataset /path/to/dataset
```

### Published metrics

| Metric | Value | Notes |
|--------|-------|-------|
| wrist↔BEV max skew (ms) | **TODO (Varun — real robot)** | Fixture CI reference: see `tests/test_live_episode.py` |
| wrist↔BEV p95 skew (ms) | **TODO** | |
| state↔wrist max skew (ms) | **TODO** | |

Paste the JSON output from `measure_sync_jitter` here after a hardware capture session.
