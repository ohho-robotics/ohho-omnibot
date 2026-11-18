#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Parse OmniBot ROS 2 launch files and bridge YAML into a Markdown table.

No ROS install required. Used by docs/architecture.md and CI freshness checks.
"""

from __future__ import annotations

import ast
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_GLOBS = (
    "omnibot-ros2/**/launch/**/*.launch.py",
    "omnibot-ai-ros2/**/launch/**/*.launch.py",
    "yahboom-python-driver/launch/**/*.launch.py",
    "ros2-bev-stitcher/launch/**/*.launch.py",
)
BRIDGE_YAML_GLOBS = (
    "omnibot-ros2/**/config/*bridge*.yaml",
    "omnibot-ros2/**/config/sim_bridge.yaml",
)


@dataclass(frozen=True)
class LaunchNode:
    package: str
    executable: str
    name: str | None


@dataclass(frozen=True)
class LaunchInclude:
    child: str  # repo-relative path


def _str_from_ast(node: ast.AST) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.JoinedStr) and len(node.values) == 1:
        return _str_from_ast(node.values[0])
    return None


def _kwargs_dict(call: ast.Call) -> dict[str, ast.AST]:
    out: dict[str, ast.AST] = {}
    for kw in call.keywords:
        if kw.arg:
            out[kw.arg] = kw.value
    return out


def _is_node_call(call: ast.Call) -> bool:
    func = call.func
    if isinstance(func, ast.Name) and func.id == "Node":
        return True
    if isinstance(func, ast.Attribute) and func.attr == "Node":
        return True
    return False


def _includes_from_launch_source(path: Path, source: str) -> list[LaunchInclude]:
    """Resolve child launch files referenced by IncludeLaunchDescription only."""
    if "IncludeLaunchDescription" not in source:
        return []
    includes: list[LaunchInclude] = []
    blocks = re.split(r"IncludeLaunchDescription\s*\(", source)
    launch_file_re = re.compile(r'["\']([\w./-]+\.launch\.py)["\']')
    join_launch_re = re.compile(
        r'os\.path\.join\([^)]*["\']launch["\']\s*,\s*["\']([^"\']+\.launch\.py)["\']'
    )
    names: list[str] = []
    for block in blocks[1:]:
        names.extend(launch_file_re.findall(block.split(")", 1)[0]))
    names.extend(join_launch_re.findall(source))
    for child_name in names:
            if child_name == path.name:
                continue
            candidates: list[Path] = []
            if "/" in child_name:
                candidates.append(path.parent / child_name)
            pkg_launch_root = path.parent
            while pkg_launch_root.name != "launch" and pkg_launch_root != REPO_ROOT:
                pkg_launch_root = pkg_launch_root.parent
            candidates.append(pkg_launch_root / child_name)
            candidates.append(pkg_launch_root / "master" / child_name)
            for candidate in candidates:
                if candidate.is_file() and candidate.resolve() != path.resolve():
                    includes.append(
                        LaunchInclude(child=candidate.relative_to(REPO_ROOT).as_posix())
                    )
                    break
    seen: set[str] = set()
    unique: list[LaunchInclude] = []
    for inc in includes:
        if inc.child not in seen:
            seen.add(inc.child)
            unique.append(inc)
    return unique


def _parse_nodes_and_includes(path: Path, source: str) -> tuple[list[LaunchNode], list[LaunchInclude]]:
    nodes: list[LaunchNode] = []
    includes: list[LaunchInclude] = []
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError:
        return nodes, includes

    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _is_node_call(node):
            kw = _kwargs_dict(node)
            pkg = _str_from_ast(kw.get("package", ast.Constant(value="")))
            exe = _str_from_ast(kw.get("executable", ast.Constant(value="")))
            if not pkg or not exe:
                continue
            name = _str_from_ast(kw.get("name", ast.Constant(value="")))
            nodes.append(LaunchNode(package=pkg, executable=exe, name=name or None))

    includes.extend(_includes_from_launch_source(path, source))

    # Deduplicate includes while preserving order
    seen_inc: set[str] = set()
    unique_includes: list[LaunchInclude] = []
    for inc in includes:
        if inc.child not in seen_inc:
            seen_inc.add(inc.child)
            unique_includes.append(inc)
    return nodes, unique_includes


def _collect_launch_files() -> list[Path]:
    files: list[Path] = []
    for pattern in LAUNCH_GLOBS:
        files.extend(REPO_ROOT.glob(pattern))
    return sorted({f.resolve() for f in files if f.is_file()})


def _collect_bridge_topics() -> list[tuple[str, str, str]]:
    """Return (source_file, ros_topic, ros_type) sorted."""
    rows: list[tuple[str, str, str]] = []
    seen_paths: set[Path] = set()
    for pattern in BRIDGE_YAML_GLOBS:
        for path in REPO_ROOT.glob(pattern):
            if path in seen_paths or not path.is_file():
                continue
            seen_paths.add(path)
            try:
                docs = yaml.safe_load(path.read_text(encoding="utf-8"))
            except yaml.YAMLError:
                continue
            if not isinstance(docs, list):
                continue
            rel = path.relative_to(REPO_ROOT).as_posix()
            for entry in docs:
                if not isinstance(entry, dict):
                    continue
                topic = entry.get("ros_topic_name")
                typ = entry.get("ros_type_name")
                if topic and typ:
                    rows.append((rel, str(topic), str(typ)))
    return sorted(rows, key=lambda r: (r[0], r[1]))


def _remappings_from_source(source: str) -> list[tuple[str, str]]:
    """Extract remappings=[(...)] string pairs from launch source."""
    pairs: list[tuple[str, str]] = []
    for m in re.finditer(
        r'remappings\s*=\s*\[(.*?)\]',
        source,
        flags=re.DOTALL,
    ):
        block = m.group(1)
        for pair in re.finditer(
            r'\(\s*["\']([^"\']+)["\']\s*,\s*["\']([^"\']+)["\']\s*\)',
            block,
        ):
            pairs.append((pair.group(1), pair.group(2)))
    return pairs


def generate_markdown() -> str:
    launch_files = _collect_launch_files()
    lines: list[str] = []
    lines.append(
        "Tables below are generated by "
        "[`scripts/generate_architecture_launch_table.py`](../scripts/generate_architecture_launch_table.py). "
        "Do not edit manually; run the script and commit, or CI will fail."
    )
    lines.append("")
    lines.append("### ROS nodes per launch file")
    lines.append("")
    lines.append(
        "| Launch file | `package` / `executable` | `name` | Includes |"
    )
    lines.append("| --- | --- | --- | --- |")

    for path in launch_files:
        rel = path.relative_to(REPO_ROOT).as_posix()
        source = path.read_text(encoding="utf-8")
        nodes, includes = _parse_nodes_and_includes(path, source)
        inc_cell = ", ".join(f"`{i.child}`" for i in includes) if includes else "-"
        if not nodes:
            lines.append(f"| `{rel}` | - | - | {inc_cell} |")
            continue
        for idx, n in enumerate(nodes):
            launch_col = f"`{rel}`" if idx == 0 else ""
            name_cell = f"`{n.name}`" if n.name else "-"
            inc = inc_cell if idx == 0 else ""
            lines.append(
                f"| {launch_col} | `{n.package}` / `{n.executable}` | {name_cell} | {inc} |"
            )

    lines.append("")
    lines.append("### Bridged topics (`ros_gz_bridge` YAML)")
    lines.append("")
    lines.append("| Config file | ROS topic | Message type |")
    lines.append("| --- | --- | --- |")
    for rel, topic, typ in _collect_bridge_topics():
        lines.append(f"| `{rel}` | `{topic}` | `{typ}` |")

    lines.append("")
    lines.append("### Selected topic remappings in launch source")
    lines.append("")
    lines.append(
        "Nav2 and the mobile-manipulation stack rename `/cmd_vel` at the mux boundary. "
        "See [`docs/reference.md`](../docs/reference.md#bring-up-control-mode)."
    )
    lines.append("")
    lines.append("| Launch file | From | To |")
    lines.append("| --- | --- | --- |")
    for path in launch_files:
        rel = path.relative_to(REPO_ROOT).as_posix()
        pairs = _remappings_from_source(path.read_text(encoding="utf-8"))
        if not pairs:
            continue
        for idx, (src, dst) in enumerate(pairs):
            launch_col = f"`{rel}`" if idx == 0 else ""
            lines.append(f"| {launch_col} | `{src}` | `{dst}` |")

    return "\n".join(lines).rstrip() + "\n"


ARCHITECTURE_DOC = REPO_ROOT / "docs" / "architecture.md"
BEGIN_MARKER = "<!-- BEGIN GENERATED_LAUNCH_TABLE -->"
END_MARKER = "<!-- END GENERATED_LAUNCH_TABLE -->"


def architecture_prologue() -> str:
    return """# OmniBot architecture

Back to [README](../README.md). Operational detail also lives in [reference.md](reference.md) and [safety.md](safety.md).

This document satisfies Linear OHH-43: system diagram, launch-derived topics table, launch graph, and known limitations. Every behavioural claim below links to source in this repository or to a CI workflow.

## System diagram

OmniBot is a ROS 2 Jazzy meta-repo: onboard drivers and perception on the Raspberry Pi ([`omnibot_pi.launch.py`](../omnibot-ros2/omnibot_bringup/launch/master/omnibot_pi.launch.py)), optional GPU policy nodes on a workstation ([`omnibot_workstation.launch.py`](../omnibot-ros2/omnibot_bringup/launch/master/omnibot_workstation.launch.py)), laptop simulation ([`sim.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py)), and pure-Python libraries tested without ROS in [`.github/workflows/ci.yml`](../.github/workflows/ci.yml).

```mermaid
flowchart TB
  subgraph clients["Teleop and ops"]
    XB["Xbox / joy"]
    AND["Android"]
    VR["Quest VR"]
    WEB["Browser / Foxglove"]
  end
  subgraph connectivity["Bridges"]
    RB["rosbridge :9090"]
    FG["foxglove :8765"]
    WVS["web_video_server"]
  end
  subgraph routing["Control routing"]
    CM["/control_mode"]
    MUX["cmd_vel_mux"]
    AMUX["arm_cmd_mux"]
    ESTOP["/emergency_stop"]
  end
  subgraph onboard["Onboard ROS 2"]
    DRV["yahboom_controller_node"]
    ARM["arm_driver_node"]
    EKF["robot_localization EKF"]
    CAM["usb_cam + Orbbec"]
    SLAM["slam_toolbox / RTAB-Map"]
    NAV["Nav2 stack"]
  end
  subgraph gpu["Workstation policies"]
    VLA["vla_node"]
    RL["rl_nav / rl_arm"]
    AG["langchain_agent"]
  end
  subgraph sim["Simulation"]
    GZ["Gazebo Harmonic"]
    BR["ros_gz_bridge"]
  end
  subgraph train["Offline training"]
    LE["learning_engine"]
    VE["vla_engine / rl_engine"]
  end
  XB --> MUX
  AND --> RB
  VR --> RB
  WEB --> RB
  WEB --> FG
  RB --> MUX
  NAV --> MUX
  VLA --> MUX
  RL --> MUX
  CM --> MUX
  CM --> AMUX
  ESTOP --> MUX
  ESTOP --> AMUX
  MUX --> DRV
  AMUX --> ARM
  DRV --> EKF
  CAM --> SLAM
  SLAM --> NAV
  GZ --> BR
  BR --> NAV
  LE --> VE
  VE -.->|ROS deploy| VLA
```

Control-mode gating is implemented in [`omnibot_hybrid`](../omnibot-ros2/omnibot_hybrid/) and documented in [reference.md § Bring-up control mode](reference.md#bring-up-control-mode). Arm safety limits are in [`omnibot_arm`](../omnibot-ros2/omnibot_arm/) and [safety.md](safety.md).

## Launch graph (common entry points)

Edges follow `IncludeLaunchDescription` in the launch sources (see generated table below).

```mermaid
flowchart LR
  sim_nav["sim_navigation.launch.py"]
  sim_map["sim_mapping.launch.py"]
  sim["sim.launch.py"]
  auto_nav["autonomous_navigation.launch.py"]
  slam_tb["slam_toolbox.launch.py"]
  robot_joy["robot_with_joy.launch.py"]
  robot["robot.launch.py"]
  joy["joy_teleop.launch.py"]
  state["state_estimation.launch.py"]
  mobile["mobile_manipulation.launch.py"]
  sim_nav --> sim
  sim_nav --> auto_nav
  sim_map --> slam_tb
  sim_map --> sim
  robot_joy --> robot
  robot_joy --> joy
  robot --> state
  mobile --> state
```

| Entry command | Launch file | Typical use |
| --- | --- | --- |
| `ros2 launch omnibot_bringup sim.launch.py` | [`sim.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py) | Headless Gazebo; CI [`sim-smoke`](../.github/workflows/ci.yml) |
| `ros2 launch omnibot_navigation sim_navigation.launch.py` | [`sim_navigation.launch.py`](../omnibot-ros2/omnibot_navigation/launch/sim_navigation.launch.py) | Nav2 in apartment sim; CI [`nav-sim`](../.github/workflows/nav-sim.yml) |
| `ros2 launch omnibot_bringup robot_with_joy.launch.py` | [`robot_with_joy.launch.py`](../omnibot-ros2/omnibot_bringup/launch/robot_with_joy.launch.py) | Physical base + Xbox teleop |
| `ros2 launch omnibot_bringup mobile_manipulation.launch.py` | [`mobile_manipulation.launch.py`](../omnibot-ros2/omnibot_bringup/launch/mobile_manipulation.launch.py) | Base mux + arm + cameras |
| `ros2 launch omnibot_bringup master/omnibot_pi.launch.py` | [`omnibot_pi.launch.py`](../omnibot-ros2/omnibot_bringup/launch/master/omnibot_pi.launch.py) | Full Pi stack (hardware) |

## Topics and services from launch files

"""


def architecture_epilogue() -> str:
    return """
ROS 2 services are created inside node implementations, not declared in launch files; this table lists nodes and bridged topics only. Nav2 action servers (for example `NavigateToPose`) come from [`autonomous_navigation.launch.py`](../omnibot-ros2/omnibot_navigation/launch/autonomous_navigation.launch.py).

## Known limitations

Honest gaps as of this branch; each item points at code or CI scope.

| Limitation | Evidence |
| --- | --- |
| MuJoCo and Isaac Sim launch backends are stubs (`not built`, exit 2). | [`mujoco.launch.py`](../omnibot-ros2/omnibot_bringup/launch/mujoco.launch.py), [`isaac.launch.py`](../omnibot-ros2/omnibot_bringup/launch/isaac.launch.py), [`test_unbuilt_launch.py`](../omnibot-ros2/omnibot_bringup/test/test_unbuilt_launch.py) |
| `sim_navigation.launch.py` uses planar holonomic drive only; `drive:=mecanum` with Nav2 did not produce `map` before startup timeout. | Comment in [`sim_navigation.launch.py`](../omnibot-ros2/omnibot_navigation/launch/sim_navigation.launch.py), [README Nav2 section](../README.md#nav2-tuning-for-the-mecanum-base) |
| Nav2 + `slam_toolbox` waypoint runs are simulation-only; not validated on the physical robot. | [README](../README.md#nav2--slam_toolbox-in-gazebo-ohh-30), [reference.md](reference.md) |
| OpenVLA / SmolVLA and Isaac Lab RL need a desktop NVIDIA GPU; not installed in CPU CI. | [`vla_engine/requirements.txt`](../omnibot-ai-engines/vla_engine/requirements.txt), [`rl_engine/requirements.txt`](../omnibot-ai-engines/rl_engine/requirements.txt), [reference.md](reference.md) |
| `omnibot-digital-twin` compose references a missing sim Dockerfile / install path. | [`docker-compose.yml`](../omnibot-digital-twin/docker/docker-compose.yml), [AGENTS.md](../AGENTS.md) |
| rosbridge defaults to loopback; `rosbridge_address:=0.0.0.0` exposes an unauthenticated WebSocket. | [`sim.launch.py`](../omnibot-ros2/omnibot_bringup/launch/sim.launch.py) docstring and arguments |
| Quest client uses MJPEG via `web_video_server`; WebRTC is roadmap-only. | [README roadmap](../README.md#roadmap) |
| BOM prices and some mounts are not documented. | [README hardware table](../README.md#hardware-bill-of-materials), issue [#19](https://github.com/ohho-robotics/OmniBot/issues/19) |
| Launch table parser reads static Python launch sources; dynamic `OpaqueFunction` node lists may be incomplete. | [`scripts/generate_architecture_launch_table.py`](../scripts/generate_architecture_launch_table.py) |

## Regenerating the launch table

```bash
python scripts/generate_architecture_launch_table.py
python scripts/generate_architecture_launch_table.py --write-doc
```

CI runs [`scripts/test_architecture_launch_table.py`](../scripts/test_architecture_launch_table.py) in the CPU job ([`ci.yml`](../.github/workflows/ci.yml)).
"""


def write_architecture_doc() -> None:
    body = (
        architecture_prologue()
        + BEGIN_MARKER
        + "\n"
        + generate_markdown()
        + END_MARKER
        + architecture_epilogue()
    )
    ARCHITECTURE_DOC.write_text(body, encoding="utf-8")


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "--write-doc":
        write_architecture_doc()
        return 0
    sys.stdout.write(generate_markdown())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
