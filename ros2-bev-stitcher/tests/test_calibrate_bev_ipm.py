"""Headless IPM calibration script (synthetic / no ROS)."""

import json
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ros2_bev_stitcher.bev_ipm import (
    compute_ipm_homographies_for_cameras,
    save_homography_npz,
)
from ros2_bev_stitcher.calibrate_bev_ipm import main as calibrate_main


class TestComputeIpmHomographiesForCameras:
    def test_default_four_cameras(self):
        names = ["front", "rear", "left", "right"]
        Hs = compute_ipm_homographies_for_cameras(names)
        assert set(Hs.keys()) == set(names)
        for name in names:
            assert Hs[name].shape == (3, 3)
            assert np.isfinite(Hs[name]).all()

    def test_save_and_load_npz(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "cal.npz")
            Hs = compute_ipm_homographies_for_cameras(["front", "rear"])
            out = save_homography_npz(path, Hs)
            assert os.path.isfile(out)
            with np.load(out) as loaded:
                assert set(loaded.files) == {"front", "rear"}


class TestCalibrateCli:
    def test_main_writes_npz(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "bev.npz")
            code = calibrate_main(
                ["--output", out, "--camera-names", "front", "rear"]
            )
            assert code == 0
            with np.load(out) as data:
                assert set(data.files) == {"front", "rear"}

    def test_summary_json(self):
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = {**os.environ, "PYTHONPATH": pkg_root}
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "bev.npz")
            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "ros2_bev_stitcher.calibrate_bev_ipm",
                    "--output",
                    out,
                    "--summary-json",
                ],
                capture_output=True,
                text=True,
                check=False,
                cwd=pkg_root,
                env=env,
            )
            assert completed.returncode == 0
            summary = json.loads(completed.stdout.strip())
            assert summary["output"] == os.path.abspath(out)
            assert "front" in summary["cameras"]
