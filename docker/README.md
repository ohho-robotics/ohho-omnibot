# OmniBot browser desktop

A container with a virtual Linux desktop that runs the OmniBot Gazebo
Harmonic sim and RViz, shown in your web browser at
<http://localhost:6080>. The host needs only Docker (Docker Desktop on
Windows and macOS). No ROS 2, Gazebo, X server or VNC client is installed on
the host.

```
browser ──http/ws──▶ 127.0.0.1:6080 ──▶ websockify + noVNC ──▶ Xtigervnc :1 (loopback)
                                                                 ├─ openbox
                                                                 ├─ Gazebo Harmonic (GUI + server)
                                                                 └─ RViz
```

Everything renders with Mesa llvmpipe (software OpenGL); no GPU is needed.

## Start

From the repository root, any one of:

| Host | Command |
|------|---------|
| Linux / macOS | `./docker/omnibot-desktop.sh` |
| Windows (PowerShell) | `.\docker\omnibot-desktop.ps1` |
| Windows (cmd / double-click) | `docker\omnibot-desktop.cmd` |
| Any (plain Compose) | `docker compose up -d desktop` |

The launchers start the `desktop` service of
[`docker-compose.yml`](../docker-compose.yml), wait until the page answers
and open the browser. The first start pulls
`ghcr.io/ohho-robotics/omnibot-desktop:latest` (amd64 and arm64). If that
tag does not exist in the registry, Compose builds it from `Dockerfile.sim`
(target `desktop`); for other pull failures (offline, DNS, registry
authentication) build it explicitly with `--build` / `-Build` or
`docker compose build desktop`.

A terminal window on the desktop shows the launch log of
`ros2 launch omnibot_bringup sim_desktop.launch.py`; Gazebo and RViz open
a few seconds later (expect up to a minute on the first start while
llvmpipe compiles shaders).

## Stop

| Launcher | Compose |
|----------|---------|
| `./docker/omnibot-desktop.sh stop` | `docker compose stop desktop` |
| `.\docker\omnibot-desktop.ps1 stop` | |
| `./docker/omnibot-desktop.sh down` (remove the container) | `docker compose rm -sf desktop` |

Other launcher commands: `restart`, `logs`, `shell` (bash in the
container), `status`, `update` (pull the newest image and restart).

## Options

Pass them to the launcher, or set the environment variables before
`docker compose up -d desktop`:

| Launcher (`.sh` / `.ps1`) | Variable | Default | Meaning |
|---|---|---|---|
| `--port N` / `-Port N` | `OMNIBOT_PORT` | `6080` | Host port of the page (bound to 127.0.0.1) |
| `--world W` / `-World W` | `OMNIBOT_WORLD` | `flat` | `flat`, `apartment` or an SDF path in the container |
| `--password[=P]` / `-Password` | `OMNIBOT_VNC_PASSWORD` | empty | Require a VNC password (random if none is given); the printed URL carries it for one-click connect (see Security) |
| `--build` / `-Build` | | | Build the image from this checkout instead of pulling |
| `--no-browser` / `-NoBrowser` | | | Do not open the browser |
| | `OMNIBOT_GUI` | `true` | Open the Gazebo client window |
| | `OMNIBOT_RVIZ` | `true` | Open RViz |
| | `OMNIBOT_RESOLUTION` | `1600x900` | Initial desktop size (the page resizes it to the window) |
| | `OMNIBOT_SHARED_DIR` | `./omnibot-shared` | Host folder mounted at `~/shared` |
| | `OMNIBOT_DESKTOP_IMAGE` | `ghcr.io/ohho-robotics/omnibot-desktop:latest` | Image to run |
| | `ROS_DOMAIN_ID` | `30` | ROS 2 domain inside the container |

Example: port 6081 and the apartment world.

```bash
OMNIBOT_PORT=6081 OMNIBOT_WORLD=apartment docker compose up -d desktop
# or
./docker/omnibot-desktop.sh --port 6081 --world apartment
```

## On the desktop

Right-click the background for the menu: Terminal, restart the sim in the
flat or apartment world, stop it, open another RViz.

From a terminal (on the desktop, or `./docker/omnibot-desktop.sh shell`):

```bash
ros2 topic list                       # /clock /cmd_vel /odom /scan /imu /joint_states /tf ...
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive with the keyboard
omnibot-desktop-sim restart world:=apartment            # extra args go to ros2 launch
```

Logs are in `/tmp/omnibot-desktop/` (`sim.log`, `xvnc.log`,
`websockify.log`).

## Persistence and files

* `omnibot-desktop-home` (named volume) is the container user's home
  (`/home/user`): shell history, RViz and Gazebo settings survive
  `stop`/`down` and image updates. Delete it with
  `docker volume rm omnibot-desktop-home`.
* `./omnibot-shared` on the host is `~/shared` in the container (git-ignored).

## Security

* The page is published on `127.0.0.1` only. Other machines on the network
  cannot reach it. Keep it that way: the page and its WebSocket are plain
  HTTP, and VNC password authentication does not encrypt the desktop stream
  or keyboard input. For remote use, prefer an SSH tunnel
  (`ssh -L 6080:127.0.0.1:6080 host`) or put a TLS-terminating reverse proxy
  with its own authentication in front; `OMNIBOT_VNC_PASSWORD` alone is not
  enough protection on a network.
* The launchers put the password in the URL (`vnc.html?password=...`) so the
  page connects with one click; it can end up in browser history and in the
  terminal output. To avoid that, open `http://localhost:<port>/` without the
  query and type the password when noVNC asks for it.
* The VNC server itself listens on the container's loopback; only websockify
  reaches it.
* rosbridge (9090) runs inside the container on its loopback and is not
  published.

## Building the image

```bash
docker compose build desktop
# or
docker build -f Dockerfile.sim --target desktop -t omnibot-desktop:local .
OMNIBOT_DESKTOP_IMAGE=omnibot-desktop:local docker compose up -d desktop
```

`Dockerfile.sim` has three stages: `base` (ROS 2 Jazzy, Gazebo Harmonic, the
built workspace), `desktop` (this image) and `sim` (the default target, the
headless image used by `docker compose up sim`, unchanged).

On Windows, build from a checkout with LF line endings (WSL, or
`git config core.autocrlf false` before cloning); with `core.autocrlf=true`
the Python node scripts copied into the image get CRLF shebangs. Building
straight from the Git URL avoids this:

```bash
docker build -f Dockerfile.sim --target desktop -t omnibot-desktop:local \
  https://github.com/ohho-robotics/OmniBot.git#main
```

## CI

[`.github/workflows/docker-desktop.yml`](../.github/workflows/docker-desktop.yml)
builds the image on every pull request that touches it, starts the Compose
service, waits for the healthcheck, checks that `http://127.0.0.1:6080`
returns 200 and that Gazebo publishes `/clock`, `/cmd_vel`, `/odom`,
`/scan`, `/joint_states` and `/tf`. On `main` and `v*` tags it builds amd64
and arm64 on native runners and pushes one multi-arch tag to
`ghcr.io/ohho-robotics/omnibot-desktop`.

## Troubleshooting

* **Port already in use**: pick another port, e.g. `--port 6081`.
* **Black or frozen page**: reload; the session keeps running. Check
  `./docker/omnibot-desktop.sh logs`.
* **Gazebo is slow**: it renders on the CPU. Close RViz
  (`OMNIBOT_RVIZ=false`) or the Gazebo client (`OMNIBOT_GUI=false`); the sim
  keeps running headless and RViz still shows it.
* **Container exits at once**: `docker logs omnibot_desktop`; the virtual
  display needs `shm_size` (set to 2 GB in the Compose file).
