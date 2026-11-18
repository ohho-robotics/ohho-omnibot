# OHH-99 reject report

Dataset: `/tmp/omnibot-sim-vr-pick`

Filter: `data_engine.quality.episode_filter` (OHH-99). Thresholds are that filter's defaults (frozen video 0.5 s, camera skew 50 ms, joint step 0.5 rad, minimum duration 2.0 s).

## Summary

- total episodes: 3
- kept episodes: 0
- rejected episodes: 3
- keep ratio: 0.0

## Episodes

- 000000 REJECT: frozen_video: camera 'observation.images.wrist' has identical frames for 0.50s (> 0.50s) from frame 0 (t=1791995384.87s) to frame 5 (t=1791995385.37s); frozen_video: camera 'observation.images.bev' has identical frames for 0.50s (> 0.50s) from frame 0 (t=1791995384.87s) to frame 5 (t=1791995385.37s)
- 000001 REJECT: frozen_video: camera 'observation.images.wrist' has identical frames for 0.50s (> 0.50s) from frame 0 (t=1791995398.27s) to frame 5 (t=1791995398.77s); frozen_video: camera 'observation.images.bev' has identical frames for 0.50s (> 0.50s) from frame 0 (t=1791995398.27s) to frame 5 (t=1791995398.77s)
- 000002 REJECT: frozen_video: camera 'observation.images.wrist' has identical frames for 0.50s (> 0.50s) from frame 0 (t=1791995411.57s) to frame 5 (t=1791995412.07s); frozen_video: camera 'observation.images.bev' has identical frames for 0.50s (> 0.50s) from frame 0 (t=1791995411.57s) to frame 5 (t=1791995412.07s)

JSON: `/work/datasets/omnibot-sim-vr-pick/reject_report.json`
