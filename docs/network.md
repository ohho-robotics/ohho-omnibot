# Network: one RMW and static DDS peers

The robot is split across a Raspberry Pi (drivers, cameras, Nav2) and a GPU
workstation (policy and perception). Both machines, and the sim containers,
use **one** ROS 2 middleware: `rmw_cyclonedds_cpp`. Discovery does not depend
on multicast. Peers are the Pi and the workstation, filled from environment
variables. There are no LAN addresses in the profile.

A real Pi-plus-workstation session is not part of the automated checks.
Those checks are a two-container Docker job (see below) plus the plain
Python test that reads the profile.

## Why Cyclone DDS

`rmw_cyclonedds_cpp` is the cell default.

- Jazzy packages it. `omnibot_bringup` declares an `exec_depend` on
  `rmw_cyclonedds_cpp`, so `rosdep install` pulls
  `ros-<distro>-rmw-cyclonedds-cpp` (on this repo, `ros-jazzy-rmw-cyclonedds-cpp`).
- One XML file turns multicast off and lists peers. The same file is used on
  the Pi, the workstation, and in the sim image.
- Wi-Fi and WSL2 often drop or hairpin multicast. With multicast off, nodes
  on one computer still find each other through the `localhost` peer.
- Mixing `rmw_cyclonedds_cpp` and `rmw_fastrtps_cpp` on the two machines
  breaks discovery and QoS matching. Do not mix them.

### Fast DDS alternative (not the cell default)

Fast DDS (`rmw_fastrtps_cpp`) can do the same job with
`FASTRTPS_DEFAULT_PROFILES_FILE` and an initial-peer list. This repo does
not ship that file, so the cell cannot drift between two profiles. If a
machine cannot install Cyclone, both sides must switch together:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
```

and point `FASTRTPS_DEFAULT_PROFILES_FILE` at a profile whose
`initialPeersList` and `metatrafficUnicastLocatorList` name the other
computer, with multicast discovery disabled. Domain id rules below still
apply: Fast DDS cannot compute a multicast port above domain 232.

## Files

| File | Role |
|---|---|
| [`omnibot-ros2/omnibot_bringup/config/ros_domain.env`](../omnibot-ros2/omnibot_bringup/config/ros_domain.env) | `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, peer addresses |
| [`omnibot-ros2/omnibot_bringup/config/dds/cyclonedds.xml`](../omnibot-ros2/omnibot_bringup/config/dds/cyclonedds.xml) | Cyclone profile: multicast off, peers from `${OMNIBOT_PI_IP}` and `${OMNIBOT_WS_IP}` |
| [`scripts/net_env.sh`](../scripts/net_env.sh) | Sources `ros_domain.env` and prints the peer list. Optional `OMNIBOT_DDS_INTERFACE` renders a throwaway copy that pins the NIC |

`OMNIBOT_WORKSTATION_IP` is the name kept from the earlier domain file.
`OMNIBOT_WS_IP` is the same address and is what the XML reads. The checked-in
values `192.0.2.10` and `192.0.2.20` are documentation addresses (TEST-NET-1),
used only when the variables are unset. Export the LAN addresses before
sourcing the env file or starting compose. A value already set is kept.
When both workstation names are set, `OMNIBOT_WORKSTATION_IP` wins and
`OMNIBOT_WS_IP` is set to the same address. Do not commit LAN addresses.

There is no second env file. Sim containers set the same variables in
`Dockerfile.sim` and the compose files.

## Two-machine setup

On the Pi and on the workstation, from a checkout of this repo:

```bash
# Export LAN addresses first. TEST-NET values in the file are fallbacks.
#   export OMNIBOT_PI_IP=<pi-address>
#   export OMNIBOT_WORKSTATION_IP=<workstation-address>
source omnibot-ros2/omnibot_bringup/config/ros_domain.env
# equivalent, and prints the peer list:
# source scripts/net_env.sh

ros2 daemon stop
ros2 launch omnibot_bringup master/omnibot_pi.launch.py          # Pi
ros2 launch omnibot_bringup master/omnibot_workstation.launch.py # workstation
```

The master launches also set `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`,
`ROS_DOMAIN_ID`, and the peer variables on their child processes. Source
the env file anyway so `ros2 node list`, `ros2 topic`, and `ros2 doctor`
use the same RMW as the nodes. After changing `RMW_IMPLEMENTATION`, run
`ros2 daemon stop` so the daemon is not left on Fast DDS.

`ROS_DOMAIN_ID` is 30. Keep it in 0–232 on both sides. Above 232 the DDS
port calculation overflows.

### What the profile sets

- `AllowMulticast` is `false`. The chosen NIC is also marked `multicast="false"`.
- Interface selection is autodetermine (the interface with the default route).
  If that picks a virtual adapter (`docker0`, a vEthernet NIC), pin the real
  one and re-source:

  ```bash
  export OMNIBOT_DDS_INTERFACE=eth0   # or wlan0, the name ip link shows
  source scripts/net_env.sh
  ```

  That writes a runtime XML under the temp directory and points
  `CYCLONEDDS_URI` at it. The checked-in file is unchanged.
- Peers are `localhost`, `${OMNIBOT_PI_IP}`, and `${OMNIBOT_WS_IP}`.
  `localhost` keeps Gazebo, Nav2, and the other nodes on one computer
  visible while multicast is off.
- `MaxAutoParticipantIndex` is 120 so a full bringup does not run out of
  participant ports.
- `SocketReceiveBufferSize` maximum is 16MB and the writer watermark is 1MB,
  for fragmented camera samples. The maximum is a request. A hard minimum
  above the kernel cap makes `rmw_create_node` fail, so the profile does not
  set one. Many kernels start near a few hundred KB. Raise the cap so the
  16MB request can take effect:

  ```bash
  sysctl net.core.rmem_max
  sudo sysctl -w net.core.rmem_max=16777216
  sudo sysctl -w net.core.wmem_max=16777216
  ```

## Firewall

DDS here is UDP. The profile does not open TCP for discovery.

For domain id `D` and participant index `P` (0 through 120 with this profile):

```text
discovery multicast = 7400 + 250 * D          # unused while multicast is off
user multicast      = 7400 + 250 * D + 1      # unused while multicast is off
discovery unicast   = 7400 + 250 * D + 10 + 2 * P
user unicast        = 7400 + 250 * D + 11 + 2 * P
```

Domain 30, participants 0–120: UDP **14900–15151**. Allow that range in both
directions between the Pi and the workstation. A simple rule is UDP
14900–15199. If you change `ROS_DOMAIN_ID`, recompute the range. Do not use
a domain above 232.

These TCP ports are application sockets, not DDS discovery:

| Port | Process |
|---|---|
| 9090 | rosbridge |
| 8765 | foxglove_bridge |
| 8080 | web_video_server |

On Windows, the Quest path for 9090, 8080, and the WebRTC UDP range is
[vr-sim-network-windows.md](vr-sim-network-windows.md). That page owns the
firewall rules and the choice between mirrored WSL networking and TCP
portproxy. This page does not repeat those steps. Portproxy forwards TCP
only, so it does not carry DDS.

## WSL2 and Docker

Multicast between Wi-Fi and a WSL2 NAT address is unreliable, and the
workstation address inside the NAT is not an address the Pi can route to.
DDS is UDP, so the TCP portproxy in
[vr-sim-network-windows.md](vr-sim-network-windows.md) does not connect the
Pi to a stack that is only on the WSL virtual NIC.

Use mirrored networking from that guide when the workstation stack runs in
WSL and must see the Pi. Set `OMNIBOT_WORKSTATION_IP` to the PC LAN address
(`OMNIBOT_WS_IP` is the same address). In
default NAT mode, run the workstation DDS processes on an address the Pi
can reach, or keep them off the robot domain.

`docker compose` sim services use host networking, so they share the host
(or WSL) network stack. The profile's `localhost` peer covers nodes in
that stack. Set `OMNIBOT_PI_IP` when that sim also has to see the Pi.
`Dockerfile.sim` and the compose files set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
and `CYCLONEDDS_URI` to the checked-in profile. The GPU compose file and the
devcontainer set the same RMW. Do not point one of them at Fast DDS while
the others stay on Cyclone.

## Check

On each machine, after sourcing the env file:

```bash
echo "$RMW_IMPLEMENTATION"
echo "$CYCLONEDDS_URI"
ros2 doctor --report
ros2 node list
ros2 topic hz /camera/front/image_raw
```

`ros2 doctor --report` includes the middleware name. It should be
`rmw_cyclonedds_cpp` on the Pi, the workstation, and in the sim containers.
`ros2 node list` from the workstation should show Pi nodes (the driver, the
muxes) once both launches are up, and the Pi should show workstation nodes
(`policy_node`, and the others that launch started). `ros2 topic hz` on a
camera topic confirms samples are arriving, not only discovery.

The CI job `dds-static-peers` in [`.github/workflows/ci.yml`](../.github/workflows/ci.yml)
runs [`scripts/ci_dds_static_peers.sh`](../scripts/ci_dds_static_peers.sh)
with `ros:jazzy-ros-base-noble`. Two containers share a Docker bridge.
Multicast discovery is disabled in the DDS config. A talker and
`ros2 topic echo /chatter --once` must fail to meet when the static-peer
profile is absent, and must meet when `cyclonedds.xml` is loaded with the
two container addresses. That job also checks `RMW_IMPLEMENTATION` inside
the container that receives the sample. It does not replace `ros2 doctor`
on the real machines.
