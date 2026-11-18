#!/usr/bin/env python3
"""Measure mock OhhO Serve /predict latency on CPU (OHH-53 evidence, mock only)."""

from __future__ import annotations

import os
import sys
import time

import numpy as np

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(_ROOT, "omnibot-ai-engines"))

from lerobot_engine.serve.mock_server import MockServeServer  # noqa: E402
from lerobot_engine.serve.client import ServeClient  # noqa: E402


def main() -> int:
    n = int(os.environ.get("SERVE_BENCH_N", "50"))
    server = MockServeServer()
    server.start()
    try:
        client = ServeClient(server.base_url, timeout_sec=5.0)
        client.load_model("hf://mock/act-bench")
        img = np.full((240, 320, 3), 48, dtype=np.uint8)
        samples_ms = []
        for _ in range(n):
            t0 = time.perf_counter()
            client.predict("latency bench", img)
            samples_ms.append((time.perf_counter() - t0) * 1000.0)
        arr = np.asarray(samples_ms, dtype=np.float64)
        print("OhhO Serve mock latency (CPU, stdlib stub)")
        print(f"  samples : {n}")
        print(f"  p50_ms  : {np.percentile(arr, 50):.3f}")
        print(f"  p95_ms  : {np.percentile(arr, 95):.3f}")
        print(f"  max_ms  : {arr.max():.3f}")
        print(f"  metrics : {client.metrics_text().strip().splitlines()[-1]}")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
