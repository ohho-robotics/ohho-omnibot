# OHH-33 Evidence: Docker/DevContainer Simulation Demo Published to GHCR

## Context & Objectives

The organization previously promised single-command `docker compose up` simulations, but Docker Hub images were 404. This PR implements a CPU-only simulation demo container published to GitHub Container Registry (GHCR) and resolves claims **C140** and **C143**.

## Components Delivered

### 1. `Dockerfile.sim` (Root & `omnibot-digital-twin/docker/Dockerfile.sim`)
- **Base image**: `ros:jazzy-ros-base-noble` (Ubuntu 24.04 LTS).
- **Simulation stack**: Gazebo Harmonic (`ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`).
- **Navigation & SLAM**: Nav2 bringup, costmap 2D, DWB controller, slam_toolbox, AMCL, robot_localization.
- **Headless software rendering**: Mesa DRI, llvmpipe, and xvfb (`LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe`, `MESA_GL_VERSION_OVERRIDE=3.3`).
- **Telemetry & visualization**: `ros-jazzy-foxglove-bridge` (port 8765), `ros-jazzy-rosbridge-suite` (port 9090).
- **Workspace pre-compilation**: Copies workspace, resolves rosdep dependencies, installs Python packages in editable mode (`mecanum-kinematics`, `yahboom-python-driver`, `learning_engine`, `agent_engine`), and builds packages into `/opt/omnibot/install`.
- **Environment & entrypoint**: Automatically sources ROS 2 Jazzy and `/opt/omnibot/install/setup.bash`, chaining any local `/workspaces/OmniBot/install/setup.bash` overlay when mounted.

### 2. `docker-compose.yml` (Root & `omnibot-digital-twin/docker/docker-compose.yml`)
- **`sim` service**: Runs Gazebo Harmonic in the apartment world with Nav2 and auto-navigation to the kitchen waypoint (`sim_navigation.launch.py autonavigate:=true route:=kitchen`).
- **`foxglove` service**: Starts `foxglove_bridge` WebSocket service on port 8765 (`ws://localhost:8765`).
- **`depends_on`**: `sim` depends on `foxglove`, ensuring `docker compose up sim` brings up both simulation and telemetry bridge automatically.
- **Image**: Points to `ghcr.io/ohho-robotics/omnibot-sim:latest` with automatic local build fallback from `Dockerfile.sim`.

### 3. VS Code DevContainer (`.devcontainer/devcontainer.json`)
- Uses `Dockerfile.sim` with `--network=host` and `--ipc=host`.
- Forwards ports 8765 (Foxglove), 9090 (ROSBridge), and 8080 (Web Video Server).
- Pre-configures Python interpreter and ROS extensions.
- Executes `post_create.sh` for workspace setup.

### 4. CI/CD Workflow (`.github/workflows/docker-sim.yml`)
- **Pull Requests**: Builds the Docker image locally without pushing (`load: true`, tag `omnibot-sim:test`), validates `docker compose config`, and runs container smoke tests.
- **Tag pushes (`v*`)**: Authenticates with GHCR via `GITHUB_TOKEN`, builds and pushes `ghcr.io/ohho-robotics/omnibot-sim:${{ tag }}` and `ghcr.io/ohho-robotics/omnibot-sim:latest`.

### 5. Launch Extensions (`sim_navigation.launch.py`)
- Added arguments:
  - `autonavigate`: default `false` (set `true` to dispatch waypoint navigator upon bringup).
  - `foxglove`: default `false` (set `true` to launch foxglove_bridge in-process).
  - `route`: default `kitchen` (comma-separated waypoint sequence).
  - `waypoint_file`: default `waypoints.yaml`.
  - `loop_waypoints`: default `false`.

## Verification & Tests

1. **Python Unit Tests**:
   - `omnibot-ros2/omnibot_bringup/test/test_docker_sim.py` (6 test cases validating Dockerfiles, compose files, DevContainer, workflows, and launch arguments).
   - Full test suite passes: 243 passed in 2.64s.
   - Syntax & lint check: `ruff check .` clean.
   - README docs test: `docs_test_readme.py` passed (6 fences parsed).

2. **Docker Build & Container Smoke Test Verification**:
   - `docker build -t omnibot-sim:test -f Dockerfile.sim .`: Successfully built all 15 layers in local Docker daemon. Built 13 ROS 2 packages into `/opt/omnibot/install` and verified editable python packages.
   - Container smoke test:
     ```bash
     docker run --rm omnibot-sim:test bash -c "ros2 pkg list | grep -E 'omnibot_navigation|omnibot_bringup|omnibot_description|mecanum_drive_ros2' && ros2 launch omnibot_navigation sim_navigation.launch.py --show-args && python3 -c 'import mecanum_drive_ros2; import yahboom_ros2; import learning_engine; import agent_engine; print(\"OK\")'"
     ```
     Executed with exit code 0.

3. **Docker Compose Validation**:
   - `docker compose config`: Validated clean exit (exit code 0).
   - `docker compose -f omnibot-digital-twin/docker/docker-compose.yml config`: Validated clean exit (exit code 0).

## Human Acceptance / Pending Verification

- **TODO for Varun**: Run on a fresh Ubuntu machine with Docker:
  ```bash
  docker compose up sim
  ```
  Verify that the container pulls/starts, Gazebo sim initializes, Nav2 navigates the robot to the kitchen waypoint, and the navigation stream is visible in Foxglove Studio (`ws://localhost:8765`) within 10 minutes.
