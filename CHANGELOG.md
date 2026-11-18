# Changelog

All notable changes to the OmniBot VR profile are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

**Status:** not yet tagged. This is not an OmniBot `v1.0.0` release of the whole repository, and it is not a GitHub Release.

## [vr-profile 1.0.0] — not released yet

Intended git tag: `vr-profile-v1.0.0`.

Baseline of this paperwork branch: `origin/main` at `d296c9be6cc898e11e8f671c2457292b30cec905` (merge of PR #29). That commit has no `sim_vr.launch.py`, no `robot_vr.launch.py`, and no `docker-compose.gpu.yml`. No git tags exist in this repository.

The image name pinned by the coordinated notes is a placeholder: `ghcr.io/ohho-robotics/omnibot-sim:vr-1.0.0`. It is not published. Commands a human would run later are in [docs/releases/vr-profile-tag-commands.md](docs/releases/vr-profile-tag-commands.md).

### Added

Prepared on branch tips. Not merged to `origin/main`. Not tagged.

- **Sim VR profile** — `omnibot_bringup` `sim_vr.launch.py` includes `sim.launch.py` with `world:=apartment` and `vr:=true`. Branch `ohh-198-sim-vr-profile`, tip `fd1dcdcc227d08589f28c61bfc2cbc58a6ce8d32`. `ohh-220-real-vr-profile` contains this file.
- **Real-robot VR profile** — `robot_vr.launch.py` and shared `vr_topic_contract.yaml`, loaded by both the sim and real launches. Branch `ohh-220-real-vr-profile`, tip `b010b84c2bab7ec4f11f928af1b20be1fc31cc95`. That tip contains `sim_vr.launch.py` and `robot_vr.launch.py`. It does not contain `docker-compose.gpu.yml`. The systemd unit in that branch is not enabled by the repo.
- **GPU compose service** — `docker-compose.gpu.yml` service `sim-vr` runs `sim_vr.launch.py` (`world:=apartment`, `vr:=true`, `gui:=false`) and sets `image: ghcr.io/ohho-robotics/omnibot-sim:latest`. The comment in that file says CI does not push the image, and names `ghcr.io/ohho-robotics/omnibot-sim:vr-<sha>` as the manual form. Branch `ohh-204-sim-gpu-container`, tip `ddb4beefd3e81f9c6ae0d21f75a9b0820023de05`. That tip has `sim_vr.launch.py` and the compose file. It does not have `robot_vr.launch.py`. It is not a descendant of the `ohh-220` tip.
- **Descriptor publisher** — branch `ohh-214-robot-descriptor`, tip `764e10a49c43d8c3a290a6f70a8c35d1c3179216`, publishes a `RobotDescriptor`. That tip is not the VR profile tag by itself.

### Not in this tag's tree

- The duplicate Quest Unity project still listed as `omnibot-vr/` in the `origin/main` README is retired on branch `ohh-192-retire-omnibot-vr` (`2071fa9`). The headset client for the coordinated set is OhhO-VR, which is also not yet tagged.
- WebRTC on the robot (`ohh-210-webrtc-streamer`, tip `6056516`) is not the sim profile tag.
- No commit today has `sim_vr.launch.py`, `robot_vr.launch.py`, and `docker-compose.gpu.yml` together. Do not tag `origin/main`.

### Known limits

- `vr-1.0.0` is a placeholder image tag. The compose file that exists on `ohh-204` still says `:latest`.
- This changelog does not claim a hardware run, a Quest sideload, or a GHCR publish.
