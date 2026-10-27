# Serve policy latency (OHH-53)

Policy inference for OmniBot can run in-process (`policy_node` + `lerobot_engine`) or remotely via **OhhO Serve** (`ohho serve --checkpoint hf://…` in [ohho-os](https://github.com/ohho-robotics/ohho-os)). The ROS path always publishes to `/arm/joint_commands`, which **arm_cmd_mux** forwards only when `/control_mode` is autonomous (`vla`, `nav2`, or `rl_nav`) and `/arm/cmd_mode` is `policy`. **PolicyGuard** clamps joint deltas before commands reach the mux.

## Endpoints (OhhO Serve)

| Route | Purpose |
|-------|---------|
| `GET /health` | Liveness and `model_loaded` |
| `POST /load_model` | Load ACT (or other) checkpoint |
| `POST /predict` | Visuomotor action vector |
| `GET /metrics` | Prometheus-style counters (mock + production Serve) |

## Latency table

| Backend | Device | p50 (ms) | p95 (ms) | Notes |
|---------|--------|----------|----------|-------|
| Mock Serve (stdlib stub in `lerobot_engine/serve/mock_server.py`) | CPU | **2.05** | **2.40** | `python scripts/benchmark_serve_mock_latency.py` (50 samples, Oct 2026) |
| OhhO Serve + ACT checkpoint | CPU | **TODO (Varun)** | **TODO (Varun)** | Requires `ohho-os[serve]` + checkpoint |
| OhhO Serve + ACT checkpoint | GPU (laptop) | **TODO (Varun)** | **TODO (Varun)** | Real trained weights |

Do not treat mock numbers as robot or GPU performance. Re-run the benchmark script after changing the mock server or client.

## Real robot video

**TODO (Varun):** Record the arm following a trained policy through Serve with mux in `policy` mode and safety clamps active.
