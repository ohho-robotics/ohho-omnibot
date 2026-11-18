# Quest to a WSL2 sim on Windows

Docker inside WSL2 uses a NAT address by default, so a Quest on Wi-Fi cannot open rosbridge or the sim video on the PC. Two ways across that NAT are below. **Varun TODO:** try both on this PC and pick one. The script refuses to pick for you.

The Quest browser opens `http://<pc-ip>:8080`. The VR app connects to `ws://<pc-ip>:9090`. Those are the Windows LAN addresses, the same address the script prints.

This page and `scripts/windows/vr-sim-network.ps1` are the OHH-205 setup. Applying the script needs an elevated PowerShell on the PC and a Quest to confirm the stream. A dry run needs neither.

Related: [OHH-114](https://linear.app/ohho-robotics/issue/OHH-114/unitree-dev-environment-on-windows-wsl2-ubuntu-2204-cyclonedds-0102) (WSL2 setup). Back to [reference](reference.md).

## Varun TODO: mirrored or portproxy

| | Mirrored | Portproxy |
|---|---|---|
| When it works | Windows 11 22H2 or newer (build 22621) and a current WSL 2 (`wsl --version`) | Any WSL2 NAT install |
| TCP 9090 and 8080 | A process in WSL that listens on `0.0.0.0` is reachable on the PC's LAN IP | `netsh interface portproxy` listens on `0.0.0.0` and connects to the distro's eth0 IPv4 |
| WebRTC UDP 50000-50100 | The firewall rule is enough once the sim binds that range | `netsh interface portproxy` forwards TCP. WebRTC media is UDP, so mirrored mode is the path that carries it |
| After a reboot | The `.wslconfig` setting stays. It takes effect after `wsl --shutdown` | The proxy entries stay, and the WSL IPv4 usually changes. Run the script again after WSL starts |
| What an elevated apply changes | Sets `networkingMode=mirrored` under `[wsl2]` in `%USERPROFILE%\.wslconfig`, adds the three Private firewall rules below, and deletes portproxy listeners on 9090 and 8080 so they do not sit in front of the mirrored binds | Adds the same three firewall rules and two TCP proxies. Leaves `.wslconfig` as it is. If mirrored is already set, it skips the proxies unless `-ForcePortProxy` |
| Disruption | `-RestartWsl` runs `wsl --shutdown` only when the config text actually changes. That stops every WSL distro | The proxy and firewall rules |

Check the installed WSL with `wsl --version`. Mirrored networking arrived in WSL 2.0.

## Script

From the repo root:

```powershell
# Both plans, LAN IP, pairing code, and QR. Changes nothing. Admin is not required.
powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -WhatIf

# One plan, still nothing changed.
powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -Mode Mirrored -WhatIf
powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -Mode PortProxy -DryRun

# Apply. Use an elevated PowerShell.
powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -Mode Mirrored -RestartWsl
powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -Mode PortProxy
```

`-WhatIf` and `-DryRun` are the same dry run. Running the file again is safe: matching firewall rules and matching proxies are left in place, and a new WSL IPv4 replaces the old proxy target.

Exit codes:

| Code | Meaning |
|---|---|
| 0 | Dry run of a chosen mode, or the apply finished |
| 1 | The plan has an error, or an apply was started from a window that is not elevated |
| 2 | No `-Mode`. Nothing was changed. Choose Mirrored or PortProxy |

The script prints:

- LAN IPv4. Wi-Fi is preferred, then Ethernet. Loopback, APIPA, and virtual adapters (vEthernet, WSL, Hyper-V, Docker, VPN, and similar) are skipped.
- `http://<ip>:8080` for the Quest browser. The QR code encodes that URL.
- `ws://<ip>:9090` for the VR app (the in-app default port is 9090).
- Pairing code `OHHO-XXXX`. That is CRC-16/CCITT-FALSE (polynomial `0x1021`, init `0xFFFF`) of the ASCII string `ohho|<ip>|9090|8080`, printed as four uppercase hex digits. It is a checksum you can read back. It is not an authentication token.

Windows Terminal shows the QR with a white background and black modules. The plain URLs are printed as well.

`-WslDistro` defaults to `navdev`. If `wsl -l -v` shows another name (often `Ubuntu`) and `navdev` is only the Docker container (`docker exec navdev`), pass `-WslDistro Ubuntu`. The portproxy connect address is that distro's eth0 IPv4. With Docker `network_mode: host`, listeners in the container are on that same address. `-WslAddress` overrides auto-detect.

## Firewall rules

Inbound Allow, profile **Private** only, remote address **LocalSubnet**:

| Name | Protocol | Port |
|---|---|---|
| `OhhO VR sim rosbridge (TCP 9090)` | TCP | 9090 |
| `OhhO VR sim video (TCP 8080)` | TCP | 8080 |
| `OhhO VR sim WebRTC (UDP 50000-50100)` | UDP | 50000-50100 |

The group in Windows Defender Firewall is `OhhO VR sim`. No other ports are opened. Foxglove's 8765 is left alone. The adapter's network category is left as it is: set Wi-Fi or Ethernet to Private so these rules match. A Public network keeps the rules from applying.

`-WebRtcUdpStart` and `-WebRtcUdpEnd` change the UDP range. The sim's WebRTC stack has to bind that same range. Nothing in this repo binds it yet (`WebRtcVideoSource` still reports that signaling is not configured, and the headset falls back to MJPEG on port 8080).

rosbridge in `sim.launch.py` has no authentication when `rosbridge_address:=0.0.0.0`. The Private profile and LocalSubnet remote address are the LAN gate this script adds.

## The sim has to listen on all interfaces

`sim.launch.py` starts rosbridge on port 9090 bound to `127.0.0.1` and does not start `web_video_server`. A listener on `127.0.0.1` stays on loopback in both mirrored mode and portproxy mode. For a Quest, start rosbridge on all interfaces:

```text
ros2 launch omnibot_bringup sim.launch.py world:=apartment rosbridge_address:=0.0.0.0
```

That launch still does not serve port 8080. `simulation.launch.py` starts `web_video_server` on port 8080 and rosbridge on port 9090. The Quest browser acceptance check is `http://<pc-ip>:8080` (the server's topic list). The headset MJPEG client requests `http://<pc-ip>:8080/stream?topic=<camera>`, and the sim front camera topic is `/camera/front/image_raw`.

`omnibot-digital-twin/docker/docker-compose.yml` sets `network_mode: host` on the sim service. In that mode Docker ignores the `ports:` map, and the process bind address is what the Quest reaches. The compose file publishes `9090:9090` for the bridge-network case; host networking still needs rosbridge on `0.0.0.0`.

## Troubleshooting

### AP / client isolation

The Quest can load the internet and still fail to open the PC. Many access points, extenders, and "smart" Wi-Fi systems isolate wireless clients from each other. Turn client isolation, AP isolation, or wireless isolation off for the SSID the Quest and the PC share. A travel router or an Ethernet link from the PC to the same router, with isolation off, is the usual fix when the ISP gateway will not allow it.

### Guest networks

A guest SSID almost always blocks clients from the rest of the LAN and from each other. Put the Quest on the main LAN SSID, the same subnet as the PC.

### 5 GHz

Use the 5 GHz SSID for the Quest. 2.4 GHz is crowded and adds latency to the camera stream. If the PC is on Wi-Fi too, put it on that same 5 GHz SSID. Band steering that parks the Quest on 5 GHz and the PC on 2.4 GHz is fine only when both bands are bridged to one subnet. If the Quest times out while a phone on 5 GHz can open the URL, check that split.

### Wi-Fi 6E

Wi-Fi 6E (6 GHz) is the preferred band for a Quest 3 when the access point has a 6 GHz radio: less interference and wider channels. The PC has to be on the same L2 network. 6 GHz and 5 GHz are different radios. If the access point isolates the 6 GHz SSID from the wired LAN, a PC on Ethernet will not answer. Confirm the 6 GHz SSID is bridged to the same subnet as the PC, then use that SSID on the Quest.

### Windows network profile, VPN, and other firewalls

The rules match the Private profile only. A network Windows calls Public will not hit them. Set the category to Private in Windows settings for the Wi-Fi or Ethernet adapter that faces the Quest.

A VPN on the PC can take the route to the Quest's subnet. Disconnect it, or use a split tunnel that leaves the LAN subnet on the physical adapter, and run the script again so the printed IP is the LAN address.

A third-party firewall can ignore Windows Defender rules. Allow inbound TCP 9090, TCP 8080, and UDP 50000-50100 from the local subnet there too.

### Portproxy address went stale

After `wsl --shutdown`, sleep, or a Docker restart, `hostname -I` inside the distro often changes. The Quest still uses the PC's LAN IP. The proxy's connect address is the stale WSL IP, so 9090 and 8080 time out. Run the script again with `-Mode PortProxy`. Matching rules are kept; the connect address is updated.

### Mirrored mode still behaves like NAT

`networkingMode=mirrored` is read when WSL starts. Until `wsl --shutdown` (or `-RestartWsl` on an apply that changed the file), existing distros keep the old network. `wsl --shutdown` stops every distro, not only navdev.

On a build older than 22621 the script reports that mirrored mode is unavailable and leaves `.wslconfig` alone. Use `-Mode PortProxy` there.

### Quest browser loads nothing, or the VR status stays red

- Confirm the printed URL from a phone on the same SSID before using the headset.
- `http://` on port 8080. The sim does not serve HTTPS there.
- rosbridge still bound to `127.0.0.1` in `sim.launch.py` until `rosbridge_address:=0.0.0.0`.
- `web_video_server` has to be running for port 8080. `sim.launch.py` does not start it.
- The VR app IP is the LAN IP from the script, port 9090.
- `ROS_DOMAIN_ID=30` is what the Quest notes and the compose file use. The WebSocket itself is rosbridge, not DDS discovery.

### WebRTC

The UDP firewall range is 50000-50100 so a later WebRTC sender has a narrow hole on the Private profile. Portproxy will not forward those packets. Use `-Mode Mirrored` when WebRTC media is required. Until signaling is configured, the headset falls back to MJPEG on TCP 8080, which portproxy can carry.

### navdev

OHH-205's notes use the name `navdev` for the WSL distro. Other notes use `docker exec navdev`, which is a container. If `wsl -d navdev` fails, list distros with `wsl -l -v` and pass `-WslDistro` with that name.
