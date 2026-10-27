# ADR: Authenticated gateway in front of rosbridge, video, and signaling

Status: accepted (OHH-196)

## Context

`sim.launch.py` binds rosbridge to `127.0.0.1` and warns that `rosbridge_address:=0.0.0.0` is unauthenticated. `robot.launch.py` and `omnibot_pi.launch.py` previously published rosbridge (9090), foxglove (8765), and, on the Pi, web_video_server (8080) on every interface. Quest, Android, and the browser Connect page need those streams on a LAN and, later, over the internet. A rosbridge `authenticate` op would cover only the rosbridge socket. Video and future WebRTC signaling would stay open.

## Decision

Use a small reverse proxy, `ohho-gateway`, in this repository. It terminates TLS and checks one bearer token for HTTP and WebSocket. rosbridge, foxglove, and web_video_server bind to `127.0.0.1`. Only the gateway listens on the LAN.

The proxy is Python (`asyncio` / `aiohttp`) so the CPU CI job can run it without ROS, Caddy, or nginx. The reusable launch file is `omnibot-ros2/omnibot_bringup/launch/ohho_gateway.launch.py`. `robot.launch.py` and `master/omnibot_pi.launch.py` include it by default. `gateway:=false` is an explicit opt-out: the launch prints `OHHO-GATEWAY WARNING` and binds the three upstreams to `0.0.0.0` with no authentication.

`sim.launch.py` is unchanged. It still defaults rosbridge to `127.0.0.1`. OHH-198 can include the snippet from a `vr:=true` sim profile and should pass `rosbridge:=false` to `sim.launch.py` so port 9090 is claimed once.

### Token

Checked with SHA-256 digests compared by `hmac.compare_digest`. Accepted as `Authorization: Bearer <token>` or the query parameter `token` (Unity WebSocket clients cannot be relied on to set headers). If both are present they must match. The query parameter is stripped before the upstream request, and the `Authorization` header is not forwarded.

Sources, in order:

1. `OHHO_GATEWAY_TOKEN`
2. `--token-file` / launch arg `gateway_token_file`
3. A pairing code stored in the state directory (`~/.ohho/gateway/pairing-code`), generated on first run

The pairing code is printed at startup. A token that came from the environment or an explicit file is not printed. Request logs record the path and status only. A log filter redacts `token=` query values, bearer credentials, and the configured token. The process command line does not contain the token.

OHH-206 can later place a Supabase session token in `OHHO_GATEWAY_TOKEN` or the token file. This proxy does not call Supabase.

### TLS

The public listener is TLS on `0.0.0.0:8443` (launch args `gateway_bind`, `gateway_port`). The first run writes a self-signed ECDSA P-256 certificate and key in the state directory and reuses them so pins stay stable. Startup prints:

- certificate SHA-256, colon-separated hex, for clients that hash the raw certificate
- SPKI pin `sha256/<base64>` for OkHttp `CertificatePinner`

Plain `ws://` and `http://` bind to `127.0.0.1:9080` only. Any other plain bind address refuses to start. `gateway_tls_only:=true` (CLI `--tls-only`) does not open that port. Cloud clients use `wss://` / `https://` only. A real certificate can replace `cert.pem` and `key.pem` in the state directory; the process will not overwrite a complete pair.

### Routes

| Client | Upstream |
|---|---|
| `/rosbridge` | `ws://127.0.0.1:9090/` |
| `/video/...` | `http://127.0.0.1:8080/...` (`/video` stripped) |
| `/foxglove` | `ws://127.0.0.1:8765/` |
| `/webrtc/...` | 503 `OHH-210` until `--webrtc <base>` is set, then the same prefix strip |

MJPEG responses are streamed. The gateway sets `Cache-Control: no-store` and `Referrer-Policy: no-referrer` on proxied responses. Its own HTTP client sets `trust_env=False` so `HTTP_PROXY` cannot carry the token off the machine.

## Consequences

Without a token the gateway returns 401 and does not open the upstream connection, so a client cannot publish `/cmd_vel`. With a token, a WebSocket to `/rosbridge` is proxied to rosbridge and an HTTP GET to `/video/stream` is proxied to web_video_server.

Quest, Android, and Connect are not modified here. The exact client changes are in [OHH-196-client-followups.md](../security/OHH-196-client-followups.md).

These launch files still publish foxglove, video, or rosbridge on every interface when someone starts them directly: `perception.launch.py`, `simulation.launch.py`, `mobile_manipulation.launch.py`, `hybrid_robot.launch.py`, and `isaac_sim.launch.py`. They were left unchanged so this patch stays limited to the robot and Pi entry points.

## Non-goals

- rosbridge `authenticate` / rosauth opcodes
- Editing Unity, the Android app, or the Connect site
- Provisioning a public CA certificate
- Implementing WebRTC signaling (OHH-210) or the session registry (OHH-206)
