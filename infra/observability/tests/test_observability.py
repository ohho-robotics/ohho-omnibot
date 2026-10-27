# SPDX-License-Identifier: Apache-2.0
"""Tests for OmniBot observability stack configurations.

Validates:
1. Docker Compose file syntax and structure.
2. Prometheus scrape configuration and alert rules.
3. Grafana datasource and dashboard provisioning.
4. Grafana dashboard JSON schema validity, panel queries, and metadata.
5. Loki configuration syntax.
6. Headless verification via Docker (promtool check config/rules and docker compose config)
   when Docker daemon is available.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

OBS_DIR = Path(__file__).resolve().parents[1]
PROMETHEUS_DIR = OBS_DIR / "prometheus"
GRAFANA_DIR = OBS_DIR / "grafana"
LOKI_DIR = OBS_DIR / "loki"


def _is_docker_available() -> bool:
    """Check if docker CLI is present and the daemon is reachable."""
    if not shutil.which("docker"):
        return False
    try:
        res = subprocess.run(
            ["docker", "info"],
            capture_output=True,
            timeout=5,
            check=False,
        )
        return res.returncode == 0
    except Exception:
        return False


def test_docker_compose_syntax():
    compose_file = OBS_DIR / "docker-compose.yml"
    assert compose_file.exists(), f"Missing {compose_file}"

    with open(compose_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    assert isinstance(data, dict), "docker-compose.yml must be a valid YAML dictionary"
    assert "services" in data, "docker-compose.yml must define 'services'"
    services = data["services"]

    for expected_svc in ["prometheus", "grafana", "loki", "node_exporter"]:
        assert expected_svc in services, f"Missing service '{expected_svc}' in docker-compose.yml"

    # Verify ports
    assert any("9090:9090" in p for p in services["prometheus"].get("ports", []))
    assert any("3000:3000" in p for p in services["grafana"].get("ports", []))
    assert any("3100:3100" in p for p in services["loki"].get("ports", []))
    assert any("9100:9100" in p for p in services["node_exporter"].get("ports", []))


def test_prometheus_config():
    prom_file = PROMETHEUS_DIR / "prometheus.yml"
    assert prom_file.exists(), f"Missing {prom_file}"

    with open(prom_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    assert "global" in data
    assert "scrape_configs" in data
    assert "rule_files" in data
    assert "alerts.yml" in data["rule_files"]

    job_names = [j["job_name"] for j in data["scrape_configs"]]
    assert "prometheus" in job_names
    assert "node_exporter" in job_names
    assert "omnibot_benchmarks" in job_names
    assert "omnibot_ros2_bridge" in job_names


def test_prometheus_alert_rules():
    alerts_file = PROMETHEUS_DIR / "alerts.yml"
    assert alerts_file.exists(), f"Missing {alerts_file}"

    with open(alerts_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    assert "groups" in data
    groups = data["groups"]
    assert len(groups) > 0

    all_alerts = []
    for grp in groups:
        assert "name" in grp
        assert "rules" in grp
        for rule in grp["rules"]:
            assert "alert" in rule
            assert "expr" in rule
            assert "labels" in rule
            assert rule["labels"].get("severity") in ["warning", "critical"]
            assert "annotations" in rule
            assert "summary" in rule["annotations"]
            all_alerts.append(rule["alert"])

    # Core required alerts
    for expected_alert in [
        "EmergencyStopActive",
        "HighDriverCycleLatency",
        "CriticalDriverCycleLatency",
        "HighPiCpuUsage",
        "HighPiRamUsage",
        "DiskSpaceRunningLow",
        "TargetDown",
    ]:
        assert expected_alert in all_alerts, f"Missing alert rule '{expected_alert}'"


def test_grafana_provisioning():
    ds_file = GRAFANA_DIR / "provisioning" / "datasources" / "datasources.yml"
    dash_prov_file = GRAFANA_DIR / "provisioning" / "dashboards" / "dashboards.yml"

    assert ds_file.exists(), f"Missing {ds_file}"
    assert dash_prov_file.exists(), f"Missing {dash_prov_file}"

    with open(ds_file, "r", encoding="utf-8") as f:
        ds_data = yaml.safe_load(f)
    assert "datasources" in ds_data
    ds_names = [d["name"] for d in ds_data["datasources"]]
    assert "Prometheus" in ds_names
    assert "Loki" in ds_names

    with open(dash_prov_file, "r", encoding="utf-8") as f:
        dash_data = yaml.safe_load(f)
    assert "providers" in dash_data
    assert len(dash_data["providers"]) > 0


def test_grafana_dashboards():
    dashboards_dir = GRAFANA_DIR / "dashboards"
    assert dashboards_dir.exists(), f"Missing {dashboards_dir}"

    expected_dashboards = {
        "robot-health.json": "omnibot-health",
        "robot-control.json": "omnibot-control",
        "robot-ai-benchmarks.json": "omnibot-ai-benchmarks",
    }

    for filename, expected_uid in expected_dashboards.items():
        dash_file = dashboards_dir / filename
        assert dash_file.exists(), f"Missing dashboard file {filename}"

        with open(dash_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        assert data.get("uid") == expected_uid, f"Dashboard {filename} UID mismatch"
        assert "title" in data, f"Dashboard {filename} missing title"
        assert "panels" in data, f"Dashboard {filename} missing panels"
        assert len(data["panels"]) >= 3, f"Dashboard {filename} has too few panels"
        assert data.get("schemaVersion", 0) >= 30, f"Dashboard {filename} invalid schemaVersion"

        # Check that non-row panels define Prometheus targets with valid PromQL expressions
        query_count = 0
        for p in data["panels"]:
            if p.get("type") == "row":
                continue
            targets = p.get("targets", [])
            for t in targets:
                if "expr" in t:
                    assert len(t["expr"].strip()) > 0
                    query_count += 1
        assert query_count > 0, f"Dashboard {filename} has no target PromQL expressions"


def test_loki_config():
    loki_file = LOKI_DIR / "loki-config.yml"
    assert loki_file.exists(), f"Missing {loki_file}"

    with open(loki_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    assert "server" in data
    assert data["server"].get("http_listen_port") == 3100
    assert "schema_config" in data


def test_docker_headless_validation():
    """Runs promtool and docker compose config if Docker daemon is available."""
    if not _is_docker_available():
        pytest.skip("Docker daemon is not available; skipping headless container checks.")

    # 1. docker compose config
    compose_file = OBS_DIR / "docker-compose.yml"
    compose_res = subprocess.run(
        ["docker", "compose", "-f", str(compose_file), "config"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert compose_res.returncode == 0, f"docker compose config failed:\n{compose_res.stderr}"

    # 2. promtool check config
    prom_res = subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "-v",
            f"{PROMETHEUS_DIR.resolve()}:/etc/prometheus:ro",
            "--entrypoint",
            "promtool",
            "prom/prometheus:v2.54.1",
            "check",
            "config",
            "/etc/prometheus/prometheus.yml",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert prom_res.returncode == 0, f"promtool check config failed:\n{prom_res.stdout}\n{prom_res.stderr}"

    # 3. promtool check rules
    rules_res = subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "-v",
            f"{PROMETHEUS_DIR.resolve()}:/etc/prometheus:ro",
            "--entrypoint",
            "promtool",
            "prom/prometheus:v2.54.1",
            "check",
            "rules",
            "/etc/prometheus/alerts.yml",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert rules_res.returncode == 0, f"promtool check rules failed:\n{rules_res.stdout}\n{rules_res.stderr}"
