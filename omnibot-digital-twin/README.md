# OmniBot Digital Twin

A contributor-ready simulation environment for working on OmniBot without physical hardware.

---

## Directory Layout

```
omnibot-digital-twin/
├── README.md                        # this file
├── worlds/
│   ├── omnibot_lab.sdf              # rich indoor lab (table, shelf, YCB objects)
│   └── objects/                     # reusable SDF object fragments
├── scenarios/
│   ├── pick_and_place.yaml          # VLA / manipulation task definitions
│   └── nav_corridor.yaml            # navigation benchmark scenarios
├── docker/
│   ├── Dockerfile.sim               # Ubuntu 24.04 + ROS Jazzy + Gazebo Harmonic
│   ├── docker-compose.yml           # sim + foxglove services
│   ├── docker-compose.gpu.yml       # GPU sim-vr service (EGL, ports 9090 and 8080)
│   └── post_create.sh              # DevContainer postCreate: deps + colcon build
├── configs/
│   ├── rviz/
│   │   ├── perception.rviz          # camera feeds, depth, BEV
│   │   ├── navigation.rviz          # map, costmap, path, odom
│   │   └── manipulation.rviz        # arm joints, TF tree, gripper cam
│   ├── foxglove/
│   │   └── omnibot_layout.json      # Foxglove Studio panel layout
│   └── nav2_sim_params.yaml         # simulation-tuned Nav2 params
└── scripts/
    ├── build_usd.sh                 # automates xacro → URDF → Isaac Sim USD
    └── setup_omnigraph.py           # wires OmniGraph ROS 2 bridge inside Isaac Sim
```

---

## Quick-Start (Gazebo Simulation)

### Option A — VS Code DevContainer

Open this repository in VS Code with the Dev Containers extension installed and select **Reopen in Container**. The DevContainer configuration lives in [`.devcontainer/devcontainer.json`](../../.devcontainer/devcontainer.json) based on `Dockerfile.sim`, pre-building the ROS 2 workspace, configuring headless software rendering, and forwarding ports 8765 (Foxglove), 9090 (ROSBridge), and 8080 (Web Video Server).

### Option B — Docker Compose (headless)

The compose file uses the published GHCR image (`ghcr.io/ohho-robotics/omnibot-sim:latest`) with automatic local build fallback from `Dockerfile.sim`. It includes prebuilt workspace packages, headless Gazebo Harmonic, Nav2 autonomous navigation, and the Foxglove bridge.

From the repository root:

```bash
docker compose up sim
```

Or targeting this directory directly:

```bash
docker compose -f omnibot-digital-twin/docker/docker-compose.yml up sim
```

The CPU file above forces llvmpipe. The Foxglove bridge listens on `ws://localhost:8765`, and ROSBridge listens on port 9090. For the RTX 5060 Ti VR profile (Ogre2 EGL, three cameras, rosbridge on 9090 and web_video_server on 8080, both published on 127.0.0.1), run the GPU file on its own:

```bash
docker compose -f omnibot-digital-twin/docker/docker-compose.gpu.yml up sim-vr
```

That starts `sim_vr.launch.py` (`world:=apartment`, `vr:=true`, `gui:=false`). Do not add this file on top of the CPU compose file.


### Option C — Native Ubuntu 24.04
```bash
# Prerequisites: ROS 2 Jazzy, Gazebo Harmonic, foxglove_bridge
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-foxglove-bridge
# From the repo root, after the ROS 2 quick start in the root README
source install/setup.bash
ros2 launch omnibot_bringup simulation.launch.py
```

---

## Contributor Domain Guide

| Your Area | Launch Command | Visualization |
|-----------|---------------|---------------|
| **Perception / Cameras** | `simulation.launch.py` | Foxglove or `perception.rviz` |
| **SLAM / Mapping** | `simulation.launch.py` then `slam_toolbox.launch.py` | `navigation.rviz` |
| **Nav2 Navigation** | `simulation.launch.py` then `autonomous_robot.launch.py` | `navigation.rviz` |
| **VLA / SmolVLA** | Isaac Sim + `isaac_sim.launch.py` | Foxglove |
| **Arm / Manipulation** | `simulation.launch.py` | `manipulation.rviz` |
| **Android App** | `simulation.launch.py` (ROSBridge on port 9090 auto-starts) | Android app |
| **Dataset Collection** | Isaac Sim + `data_engine/isaac_sim/collect_episodes.py` | Isaac Sim GUI |

---

## Visualization Options

### Foxglove Studio (browser — no GPU needed)
```bash
# Foxglove bridge starts automatically with simulation.launch.py
# Open:
https://app.foxglove.dev
# Connect: ws://localhost:8765
# Import layout: omnibot-digital-twin/configs/foxglove/omnibot_layout.json
```

### RViz 2 (desktop)
```bash
# Per-domain configs:
rviz2 -d omnibot-digital-twin/configs/rviz/perception.rviz
rviz2 -d omnibot-digital-twin/configs/rviz/navigation.rviz
rviz2 -d omnibot-digital-twin/configs/rviz/manipulation.rviz
```

---

## Simulation Worlds

| World | Use Case | File |
|-------|----------|------|
| `omnibot_world.sdf` (default) | Basic physics + sensor test | `omnibot-ros2/omnibot_bringup/worlds/` |
| `omnibot_lab.sdf` | Manipulation + navigation with objects | `omnibot-digital-twin/worlds/` |

To launch with the lab world (run from the repo root):
```bash
ros2 launch omnibot_bringup simulation.launch.py \
  world:=$(pwd)/omnibot-digital-twin/worlds/omnibot_lab.sdf
```

---

## Isaac Sim Setup (VLA / Training Data)

Prerequisites:
- NVIDIA GPU with ≥ 16 GB VRAM
- [NVIDIA Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/) with Isaac Sim 4.x installed
- ROS 2 Jazzy sourced in the same shell

```bash
# 1. Generate the USD asset (run once per URDF change)
bash omnibot-digital-twin/scripts/build_usd.sh

# 2. Start Isaac Sim with ROS 2 bridge enabled (via GUI or:)
#    ~/.local/share/ov/pkg/isaac-sim-*/isaac-sim.sh --enable omni.isaac.ros2_bridge

# 3. In Isaac Sim Script Editor, run:
#    omnibot-digital-twin/scripts/setup_omnigraph.py

# 4. Launch ROS 2 side
ros2 launch omnibot_bringup isaac_sim.launch.py

# 5. Collect training episodes
python3 data_engine/isaac_sim/collect_episodes.py \
  --config data_engine/isaac_sim/randomization_config.yaml \
  --output ~/datasets/omnibot
```

---

## Key Topics (all domains)

| Topic | Type | Notes |
|-------|------|-------|
| `/scan` | LaserScan | synthesized from depth via `depthimage_to_laserscan` (no physical 2D lidar) — SLAM, Nav2 |
| `/camera/front/image_raw` | Image | 640×480, 30 Hz |
| `/camera/base/bev/image_raw` | Image | 800×800 BEV composite |
| `/camera/wrist/image_raw` | Image | 320×240 gripper cam |
| `/camera/depth/points` | PointCloud2 | Astra Pro — Nav2 voxel layer |
| `/odom` | Odometry | Fused wheel odometry |
| `/map` | OccupancyGrid | SLAM Toolbox 2D map |
| `/arm/joint_states` | JointState | 6-DOF arm feedback |
| `/arm/joint_commands` | JointState | 6-DOF arm commands |
| `/mission/command` | String | e.g. `"navigate:kitchen,vla:find cup"` |

---

## Nav2 Simulation Parameters

The default `nav2_params.yaml` is tuned for hardware. Use the sim-specific
overrides for cleaner costmaps in Gazebo:

```bash
ros2 launch omnibot_navigation autonomous_robot.launch.py \
  params_file:=$(pwd)/omnibot-digital-twin/configs/nav2_sim_params.yaml
```
