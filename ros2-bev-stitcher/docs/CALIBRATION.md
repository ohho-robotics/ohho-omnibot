# BEV calibration on the real robot

This guide covers bringing up **OhhO View** (`ros2_bev_stitcher`) on the
physical OmniBot with four base-mounted USB cameras. It complements the
package README with Pi-specific steps and evidence collection for OHH-56.

## Prerequisites

On the robot (Raspberry Pi), with ROS 2 Jazzy sourced:

1. **Camera drivers** — `omnibot_bringup` `perception.launch.py` starts
   `usb_cam` for `front`, `rear`, `left`, `right` (and wrist / depth). You can
   run cameras without the legacy lerobot stitcher:

   ```bash
   ros2 launch omnibot_bringup perception.launch.py bev:=false
   ```

2. **TF** — `robot_state_publisher` must be running so each
   `{name}_camera_optical_frame` is published relative to `base_link`.

3. **Topics** — confirm all four feeds exist:

   ```bash
   ros2 topic hz /camera/front/image_raw
   ros2 topic hz /camera/rear/image_raw
   ros2 topic hz /camera/left/image_raw
   ros2 topic hz /camera/right/image_raw
   ```

## Step 1 — Generate an initial `.npz` (workstation or Pi)

Geometric IPM needs no checkerboard. On any machine with this package installed:

```bash
# Editable install from the repo root:
# pip install -e ros2-bev-stitcher

python -m ros2_bev_stitcher.calibrate_bev_ipm \
    --output ~/bev_calibration.npz \
    --summary-json
```

Copy `~/bev_calibration.npz` to the Pi if you generated it elsewhere.

Override poses when URDF defaults do not match your build:

```bash
python -m ros2_bev_stitcher.calibrate_bev_ipm \
    --positions front:0.24:0:0.12 rear:-0.24:0:0.12 \
    --orientations front:0:0.35:0 rear:3.14:0.35:0 \
    --output ~/bev_calibration.npz
```

## Step 2 — Fine-tune on live video (recommended)

With all four cameras streaming:

```bash
ros2 run ros2_bev_stitcher bev_cal_ui \
    --camera-names front rear left right \
    --output ~/bev_calibration.npz
```

Use trackbars per camera, press **s** to save, **q** to quit.

## Step 3 — Run the real-camera launch file

With cameras + TF already running:

```bash
ros2 launch ros2_bev_stitcher bev_real_cameras.launch.py
```

Published topic: `/camera/base/bev/image_raw` (800×800 `rgb8`).

View in a browser (if `web_video_server` is up from perception):

`http://<pi-ip>:8080/stream?topic=/camera/base/bev/image_raw`

## Step 4 — Record the README demo GIF (human step)

**Varun:** record ~20 seconds of live BEV while driving slowly on a flat
surface. Suggested workflow:

```bash
# Terminal A — cameras (if not already up)
ros2 launch omnibot_bringup perception.launch.py bev:=false

# Terminal B — stitcher
ros2 launch ros2_bev_stitcher bev_real_cameras.launch.py

# Terminal C — record (requires ros-humble-rosbag2 or Jazzy equivalent + ffmpeg)
ros2 bag record /camera/base/bev/image_raw -o /tmp/bev_demo
# … drive ~20 s, Ctrl-C, then convert to GIF with your preferred tool and add:
#   ros2-bev-stitcher/docs/assets/bev_real_robot.gif
```

Update the README placeholder once the GIF is in the repo.

## Checkerboard alternative

If geometric IPM is not accurate enough, use a flat checkerboard visible to
all four cameras:

```bash
ros2 run ros2_bev_stitcher bev_calibrate \
    --camera-names front rear left right \
    --square-size 0.025 \
    --output ~/bev_calibration.npz
```

Then launch with `calibration_mode:=checkerboard` or keep `auto` (loads the
`.npz` first).

## Troubleshooting

| Symptom | Check |
|--------|--------|
| Tiled 2×2 fallback view | No valid `.npz` and IPM params missing; run `calibrate_bev_ipm` or `bev_cal_ui`. |
| Misaligned seams | Re-run `bev_cal_ui`; verify `use_tf` and camera_info topics. |
| No output topic | All four `image_raw` streams must publish; stitcher waits for frames. |

## Synthetic / CI validation

Without hardware, validate the calibration script and config from the repo:

```bash
pytest ros2-bev-stitcher/tests/test_calibrate_bev_ipm.py \
       ros2-bev-stitcher/tests/test_bev_real_cameras_launch.py -q
```
