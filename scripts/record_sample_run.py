#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Record a sample 30 s MCAP run in Gazebo simulation for Twin replay.

Runs headless inside the navdev Docker image (no robot hardware or display needed).
Writes topics /tf, /odom, /scan, /camera/front/image_raw, /agent/trace to an MCAP bag.

Usage:
    python3 scripts/record_sample_run.py [--duration 30] [--output /tmp/omnibot_sample_run]
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def run_command(cmd: list[str], check: bool = True, timeout: float | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, check=check, capture_output=True, text=True, timeout=timeout)


def ensure_workspace() -> str:
    ws_dir = Path("/tmp/omnibot_sim_ws")
    src_dir = ws_dir / "src"
    src_dir.mkdir(parents=True, exist_ok=True)

    # Symlink or copy packages into workspace
    for pkg in ("omnibot_bringup", "omnibot_description"):
        dst = src_dir / pkg
        src = REPO_ROOT / "omnibot-ros2" / pkg
        if not dst.exists():
            if hasattr(os, "symlink"):
                try:
                    os.symlink(str(src), str(dst))
                except OSError:
                    subprocess.run(["cp", "-a", str(src), str(dst)], check=True)
            else:
                subprocess.run(["cp", "-a", str(src), str(dst)], check=True)

    install_setup = ws_dir / "install" / "setup.bash"
    if not install_setup.exists():
        print("[record_sample_run] Building omnibot packages in container...", flush=True)
        build_cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            f"cd {ws_dir} && "
            "colcon build --packages-select omnibot_description omnibot_bringup"
        )
        subprocess.run(["bash", "-c", build_cmd], check=True)

    return str(install_setup)


def main() -> int:
    parser = argparse.ArgumentParser(description="Record a sample MCAP run in simulation")
    parser.add_argument("--duration", type=float, default=30.0, help="Recording duration in seconds")
    parser.add_argument("--output", type=str, default="/tmp/omnibot_sample_run", help="Output MCAP bag directory")
    args = parser.parse_args()

    setup_bash = ensure_workspace()
    record_launch = str(REPO_ROOT / "omnibot-ros2" / "omnibot_bringup" / "launch" / "record.launch.py")
    sim_launch = str(REPO_ROOT / "omnibot-ros2" / "omnibot_bringup" / "launch" / "sim.launch.py")

    env_prefix = f"source /opt/ros/jazzy/setup.bash && source {setup_bash} && "

    # Clean existing output
    out_path = Path(args.output)
    if out_path.exists():
        subprocess.run(["rm", "-rf", str(out_path)], check=True)

    print(f"[record_sample_run] Starting Gazebo simulation (flat world, headless)...", flush=True)
    sim_cmd = f"{env_prefix} exec ros2 launch {sim_launch} world:=flat gui:=false rosbridge:=false"
    sim_proc = subprocess.Popen(["bash", "-c", sim_cmd], preexec_fn=os.setsid)

    try:
        # Wait for simulation to bridge topics
        print("[record_sample_run] Waiting for simulation bridge...", flush=True)
        ready = False
        for _ in range(30):
            time.sleep(1.0)
            res = subprocess.run(
                ["bash", "-c", f"{env_prefix} ros2 topic list"],
                capture_output=True,
                text=True,
            )
            if "/clock" in res.stdout and "/camera/front/image_raw" in res.stdout:
                ready = True
                break
        if not ready:
            print("[record_sample_run] ERROR: Simulation failed to start or publish /clock", file=sys.stderr)
            return 1

        print(f"[record_sample_run] Starting record.launch.py -> {args.output}...", flush=True)
        rec_cmd = (
            f"{env_prefix} exec ros2 launch {record_launch} "
            f"bag_path:={args.output} use_sim_time:=false"
        )
        rec_proc = subprocess.Popen(["bash", "-c", rec_cmd], preexec_fn=os.setsid)

        try:
            time.sleep(2.0)
            start_time = time.time()
            step = 0
            print(f"[record_sample_run] Recording for {args.duration:.1f} s...", flush=True)

            while time.time() - start_time < args.duration:
                step += 1
                # Drive the robot slightly in simulation
                twist_msg = "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {z: 0.1}}"
                subprocess.run(
                    ["bash", "-c", f"{env_prefix} ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{twist_msg}'"],
                    capture_output=True,
                )
                # Emit agent trace
                subprocess.run(
                    ["bash", "-c", f"{env_prefix} ros2 topic pub --once /agent/trace std_msgs/msg/String \"data: 'step_{step}: nav_waypoint'\""],
                    capture_output=True,
                )
                time.sleep(1.5)

            print("[record_sample_run] Stopping recorder...", flush=True)
            os.killpg(os.getpgid(rec_proc.pid), signal.SIGINT)
            try:
                rec_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(rec_proc.pid), signal.SIGKILL)

        finally:
            if rec_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(rec_proc.pid), signal.SIGKILL)
                except OSError:
                    pass

    finally:
        print("[record_sample_run] Stopping simulation...", flush=True)
        if sim_proc.poll() is None:
            try:
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
                sim_proc.wait(timeout=10)
            except (subprocess.TimeoutExpired, OSError):
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)

    # Inspect recorded MCAP
    print("[record_sample_run] Inspecting recorded bag...", flush=True)
    info_res = subprocess.run(
        ["bash", "-c", f"{env_prefix} ros2 bag info {args.output}"],
        capture_output=True,
        text=True,
    )
    print(info_res.stdout)

    # Find .mcap file and calculate size
    mcap_files = list(out_path.glob("*.mcap"))
    if not mcap_files:
        print("[record_sample_run] ERROR: No .mcap file found!", file=sys.stderr)
        return 1

    total_bytes = sum(f.stat().st_size for f in mcap_files)
    total_mb = total_bytes / (1024 * 1024)
    print(f"[record_sample_run] Total MCAP size: {total_bytes} bytes ({total_mb:.2f} MB)")
    if total_mb >= 50.0:
        print(f"[record_sample_run] ERROR: MCAP size {total_mb:.2f} MB exceeds 50 MB limit!", file=sys.stderr)
        return 1

    print("[record_sample_run] SUCCESS: Sample MCAP recorded under 50 MB threshold.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
