#!/usr/bin/env python3
"""Record a short local MCAP of the VR session topics and check ros2 bag info.

Needs ROS 2 Jazzy on PATH. Does not start Gazebo. Child processes are forced
to ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID=42 so this does not join a robot
network. Command payloads are zero Twists and empty joint arrays.

Exit 0 when every proof topic is in the bag and the excluded echo topic is not.
Exit 2 when ros2 is missing or this is not POSIX. Exit 1 when the bag is wrong.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "omnibot-ros2" / "omnibot_hybrid"))

from omnibot_hybrid.vr_session_record import (  # noqa: E402
    EXCLUDED_PROOF_TOPIC,
    PROOF_TOPICS,
    ZERO_TWIST,
    record_argv,
    topic_selected,
    topics_in_bag_info,
)


def _child_env() -> dict[str, str]:
    env = os.environ.copy()
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_DOMAIN_ID"] = "42"
    return env


def _publish(
    env: dict[str, str],
    topic: str,
    msg_type: str,
    payload: str,
    *,
    wait_for_subscriber: bool,
) -> None:
    if topic.startswith("/cmd_vel") and payload != ZERO_TWIST:
        raise SystemExit(f"refusing non-zero command on {topic}")
    cmd = [
        "ros2",
        "topic",
        "pub",
        "--wait-matching-subscriptions",
        "1" if wait_for_subscriber else "0",
    ]
    if wait_for_subscriber:
        cmd.append("--once")
    else:
        # Publish even when the recorder correctly ignored this topic.
        cmd.extend(["--times", "3", "--rate", "5"])
    cmd.extend([topic, msg_type, payload])
    completed = subprocess.run(
        cmd,
        env=env,
        capture_output=True,
        text=True,
        timeout=12,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f"publish {topic} failed ({completed.returncode})\n"
            f"{completed.stdout}\n{completed.stderr}"
        )


def main() -> int:
    if os.name != "posix":
        print("ohh239_sim_mcap: needs POSIX for ros2 bag SIGINT", flush=True)
        return 2
    if shutil.which("ros2") is None:
        print("ohh239_sim_mcap: ros2 is not on PATH", flush=True)
        return 2
    if "x: 0.0" not in ZERO_TWIST:
        print("ohh239_sim_mcap: refusing to publish a non-zero twist", flush=True)
        return 1
    for topic, _msg_type, _payload in PROOF_TOPICS:
        if not topic_selected(topic):
            print(f"ohh239_sim_mcap: {topic} is not a session topic", flush=True)
            return 1
    if topic_selected(EXCLUDED_PROOF_TOPIC[0]):
        print("ohh239_sim_mcap: excluded topic matches the recorder regex", flush=True)
        return 1

    env = _child_env()
    if env["ROS_LOCALHOST_ONLY"] != "1" or env["ROS_DOMAIN_ID"] != "42":
        print("ohh239_sim_mcap: isolated ROS env was not applied", flush=True)
        return 1

    parent = Path(tempfile.mkdtemp(prefix="ohh239-vr-session-"))
    part = parent / "part-0000"
    err_path = parent / "record.err"
    cmd = record_argv(str(part), use_sim_time=False)
    print("recording", " ".join(cmd), flush=True)
    with open(err_path, "wb") as err_handle:
        proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=err_handle,
            start_new_session=True,
        )
        try:
            time.sleep(2.0)
            if proc.poll() is not None:
                print(err_path.read_text(encoding="utf-8", errors="replace"), flush=True)
                print("ohh239_sim_mcap: ros2 bag record exited early", flush=True)
                return 1
            for topic, msg_type, payload in PROOF_TOPICS:
                _publish(env, topic, msg_type, payload, wait_for_subscriber=True)
            excluded_topic, excluded_type, excluded_payload = EXCLUDED_PROOF_TOPIC
            _publish(
                env,
                excluded_topic,
                excluded_type,
                excluded_payload,
                wait_for_subscriber=False,
            )
            time.sleep(1.0)
        finally:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5)
    info = subprocess.run(
        ["ros2", "bag", "info", str(part)],
        env=env,
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )
    print(info.stdout, flush=True)
    if info.returncode != 0:
        print(info.stderr, flush=True)
        print(err_path.read_text(encoding="utf-8", errors="replace"), flush=True)
        return 1
    found = topics_in_bag_info(info.stdout)
    missing = [topic for topic, _msg_type, _payload in PROOF_TOPICS if topic not in found]
    if missing:
        print("ohh239_sim_mcap: missing topics " + ", ".join(missing), flush=True)
        return 1
    if EXCLUDED_PROOF_TOPIC[0] in found:
        print("ohh239_sim_mcap: excluded topic was recorded", flush=True)
        return 1
    print(f"ohh239_sim_mcap: ok {part}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.TimeoutExpired as exc:
        print(f"ohh239_sim_mcap: timed out: {exc}", flush=True)
        raise SystemExit(1)
