# OmniBot VR profile tag commands (do not run)

**Do not run these commands.** No VR profile tag exists. `origin/main` (`d296c9be6cc898e11e8f671c2457292b30cec905`) does not contain the launch files.

`scripts/print-release-commands.sh` prints this file and does not create a tag or a GitHub Release.

No commit contains all of `sim_vr.launch.py`, `robot_vr.launch.py`, and `docker-compose.gpu.yml`.

- Profiles without the GPU compose file: `b010b84c2bab7ec4f11f928af1b20be1fc31cc95` (`ohh-220-real-vr-profile`).
- GPU compose without `robot_vr.launch.py`: `ddb4beefd3e81f9c6ae0d21f75a9b0820023de05` (`ohh-204-sim-gpu-container`).

When one `<COMMIT>` has all three files:

```bash
git fetch origin
git checkout <COMMIT>
test -f omnibot-ros2/omnibot_bringup/launch/sim_vr.launch.py
test -f omnibot-ros2/omnibot_bringup/launch/robot_vr.launch.py
test -f docker-compose.gpu.yml
git tag -a vr-profile-v1.0.0 <COMMIT> -m "OmniBot VR profile v1.0.0"
git push origin vr-profile-v1.0.0
gh release create vr-profile-v1.0.0 \
  --repo ohho-robotics/OmniBot \
  --target <COMMIT> \
  --title "OmniBot VR profile v1.0.0" \
  --notes-file CHANGELOG.md
```

Do not use tag name `v1.0.0` for this profile. Do not push `main`.

Placeholder image, not published: `ghcr.io/ohho-robotics/omnibot-sim:vr-1.0.0`.

Do not `docker push` from this sheet. The `ohh-204` workflow publishes `:latest` and `:<sha>` only on a push to `main`, and that workflow is not on `origin/main`.
