# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Test Docker sim, devcontainer, docker-compose, and workflow configurations (OHH-33).

Validates Dockerfile.sim, docker-compose.yml, .devcontainer/devcontainer.json,
and .github/workflows/docker-sim.yml without requiring Docker daemon or ROS runtime.
"""

import json
from pathlib import Path
import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]


def test_dockerfile_sim_exists_and_configured():
    dockerfiles = [
        REPO / "Dockerfile.sim",
        REPO / "omnibot-digital-twin" / "docker" / "Dockerfile.sim",
    ]
    for df in dockerfiles:
        assert df.is_file(), f"Missing {df}"
        content = df.read_text(encoding="utf-8")
        assert "FROM ros:jazzy-ros-base-noble" in content
        assert "ros-jazzy-desktop" in content
        assert "ros-jazzy-ros-gz" in content
        assert "ros-jazzy-nav2-bringup" in content
        assert "ros-jazzy-foxglove-bridge" in content
        assert "LIBGL_ALWAYS_SOFTWARE=1" in content
        assert "GALLIUM_DRIVER=llvmpipe" in content
        assert "libglvnd0" in content
        assert "ros-jazzy-web-video-server" in content
        assert "colcon build" in content
        assert "/opt/omnibot/install" in content
        assert "ENTRYPOINT [\"/omnibot_entrypoint.sh\"]" in content


def test_docker_compose_files_valid_and_configured():
    compose_paths = [
        REPO / "docker-compose.yml",
        REPO / "omnibot-digital-twin" / "docker" / "docker-compose.yml",
    ]
    for cp in compose_paths:
        assert cp.is_file(), f"Missing {cp}"
        data = yaml.safe_load(cp.read_text(encoding="utf-8"))
        assert "services" in data
        assert "sim" in data["services"]
        assert "foxglove" in data["services"]

        sim = data["services"]["sim"]
        assert "ghcr.io/ohho-robotics/omnibot-sim" in sim["image"]
        assert "autonavigate:=true" in sim["command"]
        assert "sim_navigation.launch.py" in sim["command"]
        assert sim.get("depends_on") == ["foxglove"] or "foxglove" in sim.get("depends_on", {})

        foxglove = data["services"]["foxglove"]
        assert "ghcr.io/ohho-robotics/omnibot-sim" in foxglove["image"]
        assert "foxglove_bridge" in foxglove["command"]
        assert "8765:8765" in foxglove["ports"]
        assert "LIBGL_ALWAYS_SOFTWARE=1" in sim["environment"]
        assert "GALLIUM_DRIVER=llvmpipe" in sim["environment"]
        assert "sim-vr" not in data["services"]


def test_devcontainer_json_valid():
    dc_path = REPO / ".devcontainer" / "devcontainer.json"
    assert dc_path.is_file(), f"Missing {dc_path}"
    data = json.loads(dc_path.read_text(encoding="utf-8"))
    assert "name" in data
    assert "Dockerfile.sim" in data.get("build", {}).get("dockerfile", "")
    assert 8765 in data.get("forwardPorts", [])
    assert 9090 in data.get("forwardPorts", [])
    assert "post_create.sh" in data.get("postCreateCommand", "")
    assert data.get("containerEnv", {}).get("LIBGL_ALWAYS_SOFTWARE") == "1"


def test_docker_workflow_exists_and_configured():
    wf_path = REPO / ".github" / "workflows" / "docker-sim.yml"
    assert wf_path.is_file(), f"Missing {wf_path}"
    data = yaml.safe_load(wf_path.read_text(encoding="utf-8"))
    triggers = data.get("on") if "on" in data else data.get(True, {})
    assert "pull_request" in triggers
    assert "push" in triggers
    assert "main" in triggers["push"]["branches"]
    assert "v*" in triggers["push"]["tags"]

    job = data["jobs"]["docker-sim"]
    steps = job["steps"]
    step_names = [s.get("name", "") for s in steps]
    assert any("GitHub Container Registry" in n for n in step_names)
    assert any("Build Docker image" in n for n in step_names)
    assert any("Smoke test container" in n for n in step_names)
    assert any("Docker compose" in n for n in step_names)

    login = next(s for s in steps if "GitHub Container Registry" in s.get("name", ""))
    assert login.get("if") == "github.ref == 'refs/heads/main'"

    build = next(s for s in steps if s.get("name") == "Build Docker image")
    push_expr = str(build["with"]["push"])
    assert "github.ref == 'refs/heads/main'" in push_expr
    assert "github.event_name == 'push'" in push_expr

    smoke = next(s for s in steps if s.get("name") == "Smoke test container")
    script = smoke["run"]
    assert "bash -c \"" not in script
    assert "<<'SMOKE'" in script
    assert "print('Imports OK')" in script
    assert "sim_vr.launch.py" in script

    compose_check = next(s for s in steps if "Docker compose" in s.get("name", ""))
    compose_script = compose_check["run"]
    assert "docker-compose.gpu.yml config" in compose_script
    assert "omnibot-digital-twin/docker/docker-compose.gpu.yml config" in compose_script


def test_dockerignore_excludes_heavy_artifacts():
    di_path = REPO / ".dockerignore"
    assert di_path.is_file(), f"Missing {di_path}"
    content = di_path.read_text(encoding="utf-8")
    assert ".git" in content
    assert "install/" in content
    assert "build/" in content
    assert "omnibot-vr/" in content
    assert "omnibot-android/" in content


def _command_text(command):
    if isinstance(command, list):
        return " ".join(command)
    return str(command)


def test_gpu_compose_runs_sim_vr_on_egl():
    compose_paths = [
        (REPO / "docker-compose.gpu.yml", "."),
        (REPO / "omnibot-digital-twin" / "docker" / "docker-compose.gpu.yml", "../.."),
    ]
    for path, context in compose_paths:
        assert path.is_file(), f"Missing {path}"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert "sim-vr" in data["services"]
        assert "sim" not in data["services"]
        service = data["services"]["sim-vr"]
        assert service.get("network_mode") != "host"
        assert "ghcr.io/ohho-robotics/omnibot-sim" in service["image"]
        assert service["build"]["context"] == context
        assert service["build"]["dockerfile"] == "Dockerfile.sim"
        env = service["environment"]
        assert env["LIBGL_ALWAYS_SOFTWARE"] == "0"
        assert env["GALLIUM_DRIVER"] == ""
        assert env["MESA_GL_VERSION_OVERRIDE"] == ""
        assert "graphics" in env["NVIDIA_DRIVER_CAPABILITIES"]
        assert env["__EGL_VENDOR_LIBRARY_FILENAMES"].endswith("10_nvidia.json")
        assert env["__GLX_VENDOR_LIBRARY_NAME"] == "nvidia"
        assert "127.0.0.1:9090:9090" in service["ports"]
        assert "127.0.0.1:8080:8080" in service["ports"]
        assert "9090:9090" not in service["ports"]
        assert "8080:8080" not in service["ports"]
        command = _command_text(service["command"])
        assert "sim_vr.launch.py" in command
        assert "world:=apartment" in command
        assert "video:=true" in command
        assert "rosbridge_address:=0.0.0.0" in command
        devices = service["deploy"]["resources"]["reservations"]["devices"]
        assert devices[0]["driver"] == "nvidia"
        assert "gpu" in devices[0]["capabilities"]


def test_ci_sim_smoke_stays_on_software_rendering():
    text = (REPO / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
    assert 'LIBGL_ALWAYS_SOFTWARE: "1"' in text
    assert "GALLIUM_DRIVER: llvmpipe" in text


def test_sim_navigation_launch_has_autonavigate_args():
    launch_file = REPO / "omnibot-ros2" / "omnibot_navigation" / "launch" / "sim_navigation.launch.py"
    assert launch_file.is_file()
    content = launch_file.read_text(encoding="utf-8")
    assert "autonavigate" in content
    assert "foxglove" in content
    assert "route" in content
    assert "waypoint_navigator.py" in content


# ── Browser desktop (Dockerfile.sim target "desktop", compose service "desktop")


def test_dockerfile_sim_default_target_is_headless_sim():
    content = (REPO / "Dockerfile.sim").read_text(encoding="utf-8")
    froms = [line.split() for line in content.splitlines() if line.startswith("FROM ")]
    assert froms[0][1:] == ["ros:jazzy-ros-base-noble", "AS", "base"]
    assert ["FROM", "base", "AS", "desktop"] in froms
    # The last stage is the default build target: the unchanged sim image.
    assert froms[-1] == ["FROM", "base", "AS", "sim"]


def test_dockerfile_desktop_stage():
    content = (REPO / "Dockerfile.sim").read_text(encoding="utf-8")
    desktop = content.split("FROM base AS desktop", 1)[1].split("FROM base AS sim", 1)[0]
    for pkg in ("tigervnc-standalone-server", "websockify", "openbox", "tini"):
        assert pkg in desktop
    assert "NOVNC_SHA256=" in desktop and "sha256sum -c" in desktop
    assert "COPY docker/desktop/rootfs/ /" in desktop
    assert "EXPOSE 6080" in desktop
    assert "HEALTHCHECK" in desktop and "omnibot-desktop-healthcheck" in desktop
    assert "QT_QPA_PLATFORM=xcb" in desktop
    assert '"/usr/local/bin/omnibot-desktop"' in desktop


def test_desktop_rootfs_scripts():
    rootfs = REPO / "docker" / "desktop" / "rootfs"
    entry = (rootfs / "usr/local/bin/omnibot-desktop").read_text(encoding="utf-8")
    assert "-localhost" in entry  # VNC on the container loopback only
    assert "OMNIBOT_VNC_PASSWORD" in entry and "VncAuth" in entry
    assert "websockify --web /opt/novnc" in entry
    sim = (rootfs / "usr/local/bin/omnibot-desktop-sim").read_text(encoding="utf-8")
    assert "sim_desktop.launch.py" in sim
    for name in ("omnibot-desktop", "omnibot-desktop-sim", "omnibot-desktop-healthcheck"):
        raw = (rootfs / "usr/local/bin" / name).read_bytes()
        assert raw.startswith(b"#!"), name
        assert b"\r\n" not in raw, f"{name} must keep LF line endings"
    assert json.loads((rootfs / "opt/novnc/defaults.json").read_text())["autoconnect"] is True


def test_compose_desktop_service():
    data = yaml.safe_load((REPO / "docker-compose.yml").read_text(encoding="utf-8"))
    desktop = data["services"]["desktop"]
    # Only started when named, so `docker compose up` is unchanged.
    assert desktop["profiles"] == ["desktop"]
    assert desktop["build"]["dockerfile"] == "Dockerfile.sim"
    assert desktop["build"]["target"] == "desktop"
    assert "omnibot-desktop" in desktop["image"]
    assert "network_mode" not in desktop
    assert desktop["ports"] == ["127.0.0.1:${OMNIBOT_PORT:-6080}:6080"]
    assert desktop["shm_size"] == "2gb"
    assert "OMNIBOT_VNC_PASSWORD=${OMNIBOT_VNC_PASSWORD:-}" in desktop["environment"]
    assert any(v.startswith("desktop-home:/home/user") for v in desktop["volumes"])
    assert any(v.endswith(":/home/user/shared") for v in desktop["volumes"])
    assert "/usr/local/bin/omnibot-desktop-healthcheck" in desktop["healthcheck"]["test"]
    assert data["volumes"]["desktop-home"]["name"] == "omnibot-desktop-home"
    # The headless services keep their image and build target.
    assert "target" not in data["services"]["sim"].get("build", {})


def test_desktop_launchers_present():
    d = REPO / "docker"
    for name in ("omnibot-desktop.sh", "omnibot-desktop.ps1", "omnibot-desktop.cmd", "README.md"):
        assert (d / name).is_file(), name
    sh = (d / "omnibot-desktop.sh").read_text(encoding="utf-8")
    ps1 = (d / "omnibot-desktop.ps1").read_text(encoding="utf-8")
    assert "OMNIBOT_PORT" in sh and "OMNIBOT_PORT" in ps1
    assert "--profile desktop" in sh and "--profile desktop" in ps1


def test_sim_desktop_launch_and_rviz_config():
    bringup = REPO / "omnibot-ros2" / "omnibot_bringup"
    launch = (bringup / "launch" / "sim_desktop.launch.py").read_text(encoding="utf-8")
    assert "sim.launch.py" in launch
    assert "joint_state_publisher" in launch and "/arm/joint_states" in launch
    assert "rviz2" in launch and "sim_desktop.rviz" in launch
    rviz = yaml.safe_load((bringup / "config" / "sim_desktop.rviz").read_text(encoding="utf-8"))
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "odom"


def test_docker_desktop_workflow():
    wf = yaml.safe_load(
        (REPO / ".github" / "workflows" / "docker-desktop.yml").read_text(encoding="utf-8")
    )
    smoke = wf["jobs"]["smoke"]
    script = "\n".join(s.get("run", "") for s in smoke["steps"])
    assert "127.0.0.1:6080" in script
    assert "ros2 topic list" in script and "/odom" in script and "/cmd_vel" in script
    assert "healthy" in script
    build = next(s for s in smoke["steps"] if s.get("name") == "Build desktop image")
    assert build["with"]["target"] == "desktop"
    platforms = {i["platform"] for i in wf["jobs"]["publish"]["strategy"]["matrix"]["include"]}
    assert platforms == {"linux/amd64", "linux/arm64"}
    assert wf["jobs"]["publish"]["if"] == "github.event_name == 'push'"
