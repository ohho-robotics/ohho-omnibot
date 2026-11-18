# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Verification for docs/frame-launch.md (OHH-104).

Asserts that docs/frame-launch.md exists, cites real launch files in the
repository, documents topic routing and Pi vs GPU compute partitioning,
clarifies that the 20-trial eval (OHH-101) has not run yet, and references
OHH-34 MCAP recording as related tooling without claiming Isaac Sim or MCAP
as Frame Works-today.
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
DOC_PATH = REPO_ROOT / "docs" / "frame-launch.md"

CITED_LAUNCH_FILES = [
    "omnibot-ros2/omnibot_hybrid/launch/hybrid_robot.launch.py",
    "omnibot-ros2/omnibot_bringup/launch/mobile_manipulation.launch.py",
    "omnibot-ros2/omnibot_bringup/launch/master/omnibot_pi.launch.py",
    "omnibot-ros2/omnibot_bringup/launch/master/omnibot_workstation.launch.py",
]

KEY_TOPICS = [
    "/cmd_vel",
    "/cmd_vel/out",
    "/cmd_vel/teleop",
    "/cmd_vel/vla",
    "/arm/joint_commands",
    "/arm/joint_commands/out",
    "/control_mode",
    "/emergency_stop",
    "/mission/command",
    "/camera/base/bev/image_raw",
    "/camera/wrist/image_raw",
]


def test_frame_launch_doc_exists_and_non_empty():
    assert DOC_PATH.exists(), f"Missing documentation file: {DOC_PATH}"
    content = DOC_PATH.read_text(encoding="utf-8")
    assert len(content.strip()) > 500, "docs/frame-launch.md is unexpectedly short"


def test_cited_launch_files_exist_on_disk():
    content = DOC_PATH.read_text(encoding="utf-8")
    for rel_path in CITED_LAUNCH_FILES:
        assert rel_path in content, (
            f"Expected {rel_path} to be cited in {DOC_PATH.name}"
        )
        target_file = REPO_ROOT / rel_path
        assert target_file.exists(), (
            f"Cited launch file does not exist: {target_file}"
        )


def test_ohh_101_eval_not_done_is_clearly_stated():
    content = DOC_PATH.read_text(encoding="utf-8")
    assert "OHH-101" in content
    # Ensure there is an explicit statement that the 20-trial eval has NOT run
    lower = content.lower()
    assert (
        "not run yet" in lower
        or "has not run" in lower
        or "not been run" in lower
    ), "Doc must state that the OHH-101 20-trial eval has not run yet"
    assert "works today" in lower, (
        "Doc must mention Frame 'Works today' scope boundaries"
    )


def test_ohh_34_mcap_linked_and_isaac_not_claimed():
    content = DOC_PATH.read_text(encoding="utf-8")
    assert "OHH-34" in content
    assert "MCAP" in content or "mcap" in content
    lower = content.lower()
    assert "isaac sim" in lower
    assert (
        "not built" in lower
        or "must not claim" in lower
        or "not claimed" in lower
    ), "Doc must clarify that Isaac Sim / USD / MCAP are not claimed as Frame Works-today"


def test_key_topics_documented():
    content = DOC_PATH.read_text(encoding="utf-8")
    for topic in KEY_TOPICS:
        assert topic in content, f"Expected topic {topic} in docs/frame-launch.md"


def test_compute_split_matrix_documented():
    content = DOC_PATH.read_text(encoding="utf-8")
    assert "Raspberry Pi 5" in content
    assert "GPU Workstation" in content
    assert "vla_node" in content
    assert "yahboom_driver" in content
    assert "bev_stitcher_node" in content
