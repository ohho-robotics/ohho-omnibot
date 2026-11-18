# Offline fallback (OHH-72)

Use this when the 5-minute live block in [live-demo-10min.md](live-demo-10min.md) will not start. Order is fixed: local recording, then the Docker sim path, then the committed sim evidence. The physical robot stays out of this plan. The recording stays on the presenter machine. Container images stay local.

The five rehearsal checkboxes live on the live script. Rehearsal 4 is the pass that practises this page.

## 1. Recorded video

Varun records one take after the five rehearsals, under 10 minutes, with the same 2 / 5 / 3 split as the live script.

- Keep the file on the presenter machine. A local name that is not a repo path: `omnibot-ohh-72-live-demo.mp4`.
- Do not commit it, attach it to the pull request, or upload it.
- On stage, play that file. Talk over it only to mark the context, sim, and code blocks.
- With no file yet, go to section 2.

## 2. Docker sim path on this branch

These files are in the tree:

| File | Role |
|---|---|
| [`omnibot-digital-twin/docker/Dockerfile.sim`](../../omnibot-digital-twin/docker/Dockerfile.sim) | Local image. ROS 2 Jazzy on Ubuntu 24.04, Gazebo Harmonic, Nav2, foxglove, rosbridge. Build context is the repo root. The file's own comment tags the image `omnibot-sim` on this machine. |
| [`omnibot-digital-twin/docker/docker-compose.yml`](../../omnibot-digital-twin/docker/docker-compose.yml) | `sim` (`omnibot_sim`) and `foxglove` (`omnibot_foxglove`). |
| [`omnibot-digital-twin/docker/docker-compose.gpu.yml`](../../omnibot-digital-twin/docker/docker-compose.gpu.yml) | Optional NVIDIA overlay. The offline plan does not need it. |

From the repo root, after the image has been built once before the talk:

```bash
docker compose -f omnibot-digital-twin/docker/docker-compose.yml up
```

`sim` runs `ros2 launch omnibot_bringup simulation.launch.py rviz:=false foxglove:=false`. Foxglove, when that service starts, listens on `ws://localhost:8765`. Port 9090 is published for rosbridge. Details and the Foxglove layout path are in the [digital twin README](../../omnibot-digital-twin/README.md).

The compose command sources `install/setup.bash` when that file is present. A colcon install of the workspace is still what makes the OmniBot packages visible inside the container. The image build pulls Gazebo and Nav2 and is not the GitHub Actions CPU job. Build it before the talk. This document does not claim the stack was started here.

[`simulation.launch.py`](../../omnibot-ros2/omnibot_bringup/launch/simulation.launch.py) is the compose entrypoint. The CI `sim-smoke` job tests [`sim.launch.py`](../../omnibot-ros2/omnibot_bringup/launch/sim.launch.py) instead. When compose is still pulling or building at show time, play the recording or the committed gif. Do not switch to hardware.

Leave the image on the machine that built it. Do not `docker push`, and do not log in to a registry for this fallback.

## 3. OHH-33 container, when that branch is present

OHH-33 is not in this tree. It is branch `ohh-33-docker-sim-ghcr`: a CPU-only sim image and devcontainer, published to GHCR on version tags. When that branch is merged, the offline path also has:

- Root `Dockerfile.sim` and root `docker-compose.yml`
- Image `ghcr.io/ohho-robotics/omnibot-sim:latest`
- Workflow `.github/workflows/docker-sim.yml`, which pushes to GHCR only on `v*` tags

Until those files are on the branch you are presenting, use section 2. After they land, `docker compose up sim` from the repo root is the OHH-33 path: the compose file there runs `ros2 launch omnibot_navigation sim_navigation.launch.py autonavigate:=true route:=kitchen`. Pull the image only when that tag exists in GHCR. Pushing images is still out of scope, on this branch and after OHH-33 lands.

## 4. Committed evidence, if video and Docker are both unavailable

Show these and continue the code and CI tour from the live script:

- [`assets/omnibot_gazebo_flat.gif`](../../assets/omnibot_gazebo_flat.gif), frames from the flat world while `/cmd_vel` was published
- The README time-to-goal table for `kitchen`, `desk`, and `door`
- The [CI](../../.github/workflows/ci.yml) and [nav-sim](../../.github/workflows/nav-sim.yml) workflows

The robot is not a fallback.
