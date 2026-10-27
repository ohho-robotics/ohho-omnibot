# Test coverage (CPU packages)

Line coverage is **not** checked into this file. It is produced on every CI run on `main` and pull requests.

## Where to read numbers

1. Open the latest [CI workflow run](https://github.com/ohho-robotics/OmniBot/actions/workflows/ci.yml).
2. Open the **CPU tests and ruff** job.
3. Expand the **CPU line coverage** step — the total line coverage percentage and per-package breakdown are printed there.
4. Optional: download the `cpu-coverage` artifact (`coverage-summary.txt` and `cpu-coverage.json`).

Do not cite a coverage percentage in docs or badges unless it comes from that CI step or from a local run of the script below.

## Scope

The CPU coverage job measures Python packages exercised by the `cpu` job in [`.github/workflows/ci.yml`](../.github/workflows/ci.yml):

- `mecanum_kinematics`, `yahboom_ros2`, `ros2_bev_stitcher`
- `omnibot_arm` scripts (safety / tick math tests)
- `omnibot_lerobot` policy guard tests (import path only where ROS is not started)
- `omnibot_rl` stream gates (via bringup unit tests)
- `learning_engine`, `agent_engine`, `data_engine` (numpy-only paths)

ROS 2 colcon tests, Gazebo sim-smoke, GPU training trees, Android, and Unity are **out of scope**.

## Reproduce locally

From the repo root with the [`.venv`](../AGENTS.md) dev environment:

```bash
python scripts/run_cpu_coverage.py
```

This mirrors CI and writes `coverage/cpu-coverage.json` and prints a summary to the terminal.
