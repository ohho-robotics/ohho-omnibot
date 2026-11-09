# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""The cell DDS profile has no private addresses and one RMW (OHH-397)."""

from __future__ import annotations

import importlib.util
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
NS = {"c": "https://cdds.io/config"}
# RFC 1918. 192.0.2.0/24 (TEST-NET-1) is intentionally not in this set.
RFC1918 = re.compile(
    r"\b(?:10\.\d{1,3}\.\d{1,3}\.\d{1,3}"
    r"|192\.168\.\d{1,3}\.\d{1,3}"
    r"|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})\b"
)
IPV4 = re.compile(r"\b\d{1,3}(?:\.\d{1,3}){3}\b")

PROFILE = REPO / "omnibot-ros2" / "omnibot_bringup" / "config" / "dds" / "cyclonedds.xml"
ENV_FILE = REPO / "omnibot-ros2" / "omnibot_bringup" / "config" / "ros_domain.env"
DDS_ENV = (
    REPO / "omnibot-ros2" / "omnibot_bringup" / "launch" / "master" / "dds_env.py"
)


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _load_dds_env():
    spec = importlib.util.spec_from_file_location("omnibot_dds_env", DDS_ENV)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {DDS_ENV}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _clear_dds_env(monkeypatch: pytest.MonkeyPatch) -> None:
    for name in (
        "RMW_IMPLEMENTATION",
        "ROS_DOMAIN_ID",
        "CYCLONEDDS_URI",
        "OMNIBOT_PI_IP",
        "OMNIBOT_WS_IP",
        "OMNIBOT_WORKSTATION_IP",
        "ROS_STATIC_PEERS",
    ):
        monkeypatch.delenv(name, raising=False)


def test_profile_parses_and_limits_multicast():
    root = ET.parse(PROFILE).getroot()
    assert root.tag == f"{{{NS['c']}}}CycloneDDS"
    domain = root.find("c:Domain", NS)
    assert domain is not None
    assert domain.get("id") == "any"

    allow = root.find(".//c:AllowMulticast", NS)
    assert allow is not None
    assert (allow.text or "").strip() == "false"

    nics = root.findall(".//c:NetworkInterface", NS)
    assert nics, "profile must select an interface"
    assert all(nic.get("multicast") == "false" for nic in nics)
    assert any(nic.get("autodetermine") == "true" for nic in nics)

    peers = [peer.get("address") or "" for peer in root.findall(".//c:Peer", NS)]
    assert "localhost" in peers
    assert "${OMNIBOT_PI_IP}" in peers
    assert "${OMNIBOT_WS_IP}" in peers

    buffer = root.find(".//c:SocketReceiveBufferSize", NS)
    assert buffer is not None
    # A hard min above the kernel rmem cap aborts node creation.
    assert buffer.get("min") is None
    assert buffer.get("max") == "16MB"

    xml = _text(PROFILE)
    assert IPV4.search(xml) is None
    assert RFC1918.search(xml) is None


def test_env_file_exports_one_rmw_and_no_private_ips():
    text = _text(ENV_FILE)
    assert "export ROS_DOMAIN_ID=30" in text
    assert "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in text
    assert "CYCLONEDDS_URI" in text
    assert "cyclonedds.xml" in text
    assert "OMNIBOT_PI_IP" in text
    assert "OMNIBOT_WS_IP" in text
    assert RFC1918.search(text) is None
    assert "rmw_fastrtps_cpp" not in text.split("Do not mix", 1)[0]


def test_dds_environment_defaults(monkeypatch: pytest.MonkeyPatch):
    module = _load_dds_env()
    _clear_dds_env(monkeypatch)
    env = module.dds_environment()
    assert env["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert env["ROS_DOMAIN_ID"] == "30"
    assert env["OMNIBOT_PI_IP"] == "192.0.2.10"
    assert env["OMNIBOT_WS_IP"] == "192.0.2.20"
    assert env["ROS_STATIC_PEERS"] == "192.0.2.10;192.0.2.20"
    assert env["CYCLONEDDS_URI"].startswith("file://")
    assert env["CYCLONEDDS_URI"].endswith("/cyclonedds.xml")
    assert RFC1918.search(env["CYCLONEDDS_URI"]) is None


def test_dds_environment_keeps_explicit_overrides(monkeypatch: pytest.MonkeyPatch):
    module = _load_dds_env()
    _clear_dds_env(monkeypatch)
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    monkeypatch.setenv("OMNIBOT_PI_IP", "192.0.2.50")
    monkeypatch.setenv("OMNIBOT_WORKSTATION_IP", "192.0.2.60")
    monkeypatch.setenv("CYCLONEDDS_URI", "file:///tmp/omnibot-cyclonedds.xml")
    env = module.dds_environment()
    assert env["ROS_DOMAIN_ID"] == "7"
    assert env["OMNIBOT_PI_IP"] == "192.0.2.50"
    assert env["OMNIBOT_WS_IP"] == "192.0.2.60"
    assert env["CYCLONEDDS_URI"] == "file:///tmp/omnibot-cyclonedds.xml"


def test_master_launches_apply_the_dds_environment():
    for name in ("omnibot_pi.launch.py", "omnibot_workstation.launch.py"):
        text = _text(
            REPO
            / "omnibot-ros2"
            / "omnibot_bringup"
            / "launch"
            / "master"
            / name
        )
        assert "dds_env.py" in text
        assert "environment_actions" in text
        assert "SetEnvironmentVariable" in text
        assert "rmw_cyclonedds_cpp" in text


def test_package_xml_rosdep_rmw():
    text = _text(REPO / "omnibot-ros2" / "omnibot_bringup" / "package.xml")
    assert "<exec_depend>rmw_cyclonedds_cpp</exec_depend>" in text


def test_sim_containers_use_the_same_rmw():
    expected = "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    for path in (
        REPO / "Dockerfile.sim",
        REPO / "omnibot-digital-twin" / "docker" / "Dockerfile.sim",
        REPO / "docker-compose.yml",
        REPO / "omnibot-digital-twin" / "docker" / "docker-compose.yml",
    ):
        text = _text(path)
        assert expected in text, path
        assert "cyclonedds.xml" in text, path
    for path in (
        REPO / "docker-compose.gpu.yml",
        REPO / "omnibot-digital-twin" / "docker" / "docker-compose.gpu.yml",
    ):
        text = _text(path)
        assert "RMW_IMPLEMENTATION: rmw_cyclonedds_cpp" in text, path
        assert "cyclonedds.xml" in text, path
    dev = _text(REPO / ".devcontainer" / "devcontainer.json")
    assert '"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"' in dev
    assert "cyclonedds.xml" in dev


def test_readme_and_network_doc():
    readme = _text(REPO / "README.md")
    assert "docs/network.md" in readme
    doc = _text(REPO / "docs" / "network.md")
    assert "rmw_cyclonedds_cpp" in doc
    assert "rmw_fastrtps_cpp" in doc
    assert "ros2 doctor --report" in doc
    assert "ros2 topic hz" in doc
    assert "ros2 node list" in doc
    assert "vr-sim-network-windows.md" in doc
    assert "14900" in doc
    assert RFC1918.search(doc) is None


def test_ci_job_has_an_explicit_negative_case():
    workflow = _text(REPO / ".github" / "workflows" / "ci.yml")
    assert "scripts/ci_dds_static_peers.sh" in workflow
    assert "ros:jazzy" in workflow
    script = _text(REPO / "scripts" / "ci_dds_static_peers.sh")
    assert "AllowMulticast>false" in script
    assert "cyclonedds.xml" in script
    assert "without the static-peer profile" in script
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in script
    assert "ros:jazzy-ros-base-noble" in script
    net = _text(REPO / "scripts" / "net_env.sh")
    assert "ros_domain.env" in net
    assert RFC1918.search(script) is None
    assert RFC1918.search(net) is None


def _bash_script_arg(bash: str, script: Path) -> str:
    """WSL's bash.exe strips backslashes from a Windows path."""
    text = str(script.resolve())
    if "system32" in bash.lower() and len(text) >= 3 and text[1] == ":":
        return f"/mnt/{text[0].lower()}/{text[3:].replace(chr(92), '/')}"
    return text


def test_shell_scripts_pass_bash_n():
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash is not installed")
    for rel in ("scripts/net_env.sh", "scripts/ci_dds_static_peers.sh"):
        proc = subprocess.run(
            [bash, "-n", _bash_script_arg(bash, REPO / rel)],
            check=False,
            capture_output=True,
            text=True,
        )
        assert proc.returncode == 0, proc.stderr


def test_net_env_sh_prints_the_peer_list():
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash is not installed")
    proc = subprocess.run(
        [bash, _bash_script_arg(bash, REPO / "scripts" / "net_env.sh")],
        check=False,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in proc.stdout
    assert "ROS_DOMAIN_ID=30" in proc.stdout
    assert "OMNIBOT_PI_IP=192.0.2.10" in proc.stdout
    assert "OMNIBOT_WS_IP=192.0.2.20" in proc.stdout
    assert "cyclonedds.xml" in proc.stdout
    assert RFC1918.search(proc.stdout) is None
