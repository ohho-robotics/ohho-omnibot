# Observability (1 Robot)

This directory contains the 1-robot observability stack for OmniBot (Linear issue **OHH-64: Fleet v0: 1-robot observability (Prometheus + Grafana)**).

It provides monitoring, alert evaluation, log aggregation, and Grafana dashboards for a single robot (Raspberry Pi onboard compute + host/desktop GPU).

---

## Architecture & Services

The stack runs via Docker Compose in [`docker-compose.yml`](docker-compose.yml):

| Service | Container | Image | Port | Description |
|---|---|---|---|---|
| **Prometheus** | `omnibot_prometheus` | `prom/prometheus:v2.54.1` | `:9090` | Time-series metrics collection and rule evaluation (5 s scrape interval) |
| **Grafana** | `omnibot_grafana` | `grafana/grafana:11.2.0` | `:3000` | Pre-provisioned dashboards and datasources (default login `admin`/`admin`) |
| **Loki** | `omnibot_loki` | `grafana/loki:3.1.1` | `:3100` | Log aggregation engine (queried by Android controller `LokiApi`) |
| **Node Exporter** | `omnibot_node_exporter` | `prom/node-exporter:v1.8.2` | `:9100` | Host / Pi hardware metrics (CPU, RAM, disk, network) |

---

## Scraped Endpoints & Metrics Inventory

In accordance with OHH-64 guidance (*"find the existing metrics node / Serve /metrics; if one does not exist, document that and scrape only what exists; do not invent metrics"*), the scrape targets in [`prometheus/prometheus.yml`](prometheus/prometheus.yml) are:

1. **`omnibot_benchmarks` (`host.docker.internal:8890`)**:
   - **Status**: Implemented and active.
   - **Source**: [`omnibot-ai-engines/learning_engine/benchmarks/reporters.py`](../../omnibot-ai-engines/learning_engine/benchmarks/reporters.py) (`PrometheusReporter`).
   - **Metrics exposed**: `omnibot_bench_step_time_ms`, `omnibot_bench_iterations_per_sec`, `omnibot_bench_loss`, `omnibot_bench_reward`.

2. **`node_exporter` (`node_exporter:9100`)**:
   - **Status**: Implemented and active.
   - **Source**: Standard Prometheus node exporter container with label `machine="raspberry_pi"`.
   - **Metrics exposed**: `node_cpu_seconds_total`, `node_memory_MemAvailable_bytes`, `node_memory_MemTotal_bytes`, `node_filesystem_avail_bytes`, etc.

3. **`omnibot_ros2_bridge` (`host.docker.internal:8888`)**:
   - **Status**: Target configured; planned in ROS 2 bringup.
   - **Source**: Configured in [`omnibot-ros2/omnibot_bringup/launch/robot.launch.py`](../../omnibot-ros2/omnibot_bringup/launch/robot.launch.py) via launch arguments `use_metrics:=true` and `metrics_port:=8888`.
   - **Expected metrics**: `omnibot_estop_active`, `omnibot_control_mode`, `omnibot_node_cycle_*_ms`, `omnibot_robot_vx_ms`, `omnibot_robot_vy_ms`, `omnibot_robot_omega_rads`, `omnibot_missions_done_total`.

4. **Note on `vla_serve` (`host.docker.internal:8000`)**:
   - **Status**: Documented only; `/metrics` is **not** currently implemented on `vla_serve`.
   - **Source**: [`omnibot-ai-engines/vla_engine/inference/server.py`](../../omnibot-ai-engines/vla_engine/inference/server.py) implements `/health`, `/load_model`, and `/predict`.
   - Per OHH-64 rules, no metrics endpoint was invented. Scrape configuration for `vla_serve` remains commented out in `prometheus.yml` until Prometheus instrumentation is added to the server.

---

## Grafana Dashboards

Grafana automatically provisions 3 dashboards from [`grafana/dashboards/`](grafana/dashboards/):

1. **`robot-health.json` (UID: `omnibot-health`) — Robot System & Hardware Health**:
   - Raspberry Pi CPU utilization (`%`), RAM utilization (`%`), and root disk space (`%`).
   - Host/Desktop NVIDIA GPU utilization (`%`), VRAM usage (`%`), and temperature (°C).
   - Prometheus scrape target status (`up`) and table of active firing alerts.

2. **`robot-control.json` (UID: `omnibot-control`) — Robot Control & Kinematics**:
   - Emergency Stop status (`omnibot_estop_active`) with visual alert badges.
   - Active control mode (`omnibot_control_mode`).
   - Mission success rate (`rate(omnibot_missions_done_total{result="success"}[5m])`).
   - Base linear velocities ($v_x, v_y$) and angular yaw rate ($\omega$).
   - Yahboom motor controller cycle latency percentiles ($p50, p95, \max$ ms).

3. **`robot-ai-benchmarks.json` (UID: `omnibot-ai-benchmarks`) — AI Inference & Hardware Benchmarks**:
   - OpenVLA policy inference latency (`omnibot_vla_inference_ms`).
   - Reinforcement learning policy latency for navigation and arm (`omnibot_rl_inference_ms`).
   - Learning engine loop benchmark step duration (`omnibot_bench_step_time_ms`) and throughput (`omnibot_bench_iterations_per_sec`).
   - Training/eval loss and reward history.

---

## Alert Rules

Defined in [`prometheus/alerts.yml`](prometheus/alerts.yml):

| Alert | Severity | Threshold / Condition | Description |
|---|---|---|---|
| `EmergencyStopActive` | `critical` | `omnibot_estop_active > 0` for 5s | E-stop active; motors and arm halted |
| `CriticalDriverCycleLatency` | `critical` | `omnibot_node_cycle_max_ms > 100` for 10s | Driver cycle latency exceeded safety deadline |
| `HighDriverCycleLatency` | `warning` | `omnibot_node_cycle_p95_ms > 50` for 30s | Driver cycle p95 latency elevated |
| `HighPiCpuUsage` | `warning` | Pi CPU $> 90\%$ for 2m | High CPU consumption on Pi |
| `HighPiRamUsage` | `warning` | Pi RAM $> 90\%$ for 2m | High memory consumption on Pi |
| `DiskSpaceRunningLow` | `warning` | Pi Disk $> 85\%$ for 5m | Low disk storage on root partition |
| `TargetDown` | `critical` | `up == 0` for 1m | Scrape target unreachable |

---

## Quick Start (Local or on Robot)

To start the observability stack:

```bash
docker compose -f infra/observability/docker-compose.yml up -d
```

Access web interfaces:
- **Grafana**: [http://localhost:3000](http://localhost:3000) (User: `admin`, Password: `admin`)
- **Prometheus**: [http://localhost:9090](http://localhost:9090)
- **Loki**: [http://localhost:3100/ready](http://localhost:3100/ready)

To tear down:

```bash
docker compose -f infra/observability/docker-compose.yml down
```

---

## Headless Validation

All configurations can be validated without hardware or running services:

1. **Docker Compose schema validation**:
   ```bash
   docker compose -f infra/observability/docker-compose.yml config
   ```

2. **Prometheus configuration validation (`promtool`)**:
   ```bash
   docker run --rm -v "${PWD}/infra/observability/prometheus:/etc/prometheus:ro" \
     --entrypoint promtool prom/prometheus:v2.54.1 check config /etc/prometheus/prometheus.yml
   ```

3. **Prometheus alert rules validation (`promtool`)**:
   ```bash
   docker run --rm -v "${PWD}/infra/observability/prometheus:/etc/prometheus:ro" \
     --entrypoint promtool prom/prometheus:v2.54.1 check rules /etc/prometheus/alerts.yml
   ```

4. **Automated test suite**:
   ```bash
   pytest infra/observability/tests/test_observability.py -v
   ```

---

## Status & Human Handoff (OHH-64)

- [x] **Docker Compose stack**: Configured with Prometheus, Grafana, Loki, and Node Exporter.
- [x] **Datasources & Dashboards**: Prometheus and Loki provisioned; 3 dashboards (`robot-health`, `robot-control`, `robot-ai-benchmarks`) checked in and validated.
- [x] **Alert rules**: Validated with `promtool check rules`.
- [x] **Headless validation**: Automated via `pytest infra/observability/tests/test_observability.py` and included in CI.
- [x] **Signed OTA update**: Out of scope (deferred to Vision backlog per 29 Sep 2026 re-scope).
- [ ] **Real robot verification & dated Grafana screenshot**: **TODO for Varun** on physical robot hardware.
