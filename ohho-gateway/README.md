# ohho-gateway

Authenticated TLS reverse proxy in front of rosbridge, web_video_server, and foxglove. A reserved `/webrtc/` path stays closed until a signaling upstream is configured (OHH-210).

Plain `ws://` / `http://` listens on `127.0.0.1` only. The LAN and cloud listener is TLS.

```bash
pip install -e ohho-gateway
ohho-gateway --help
```

On first start the process writes a self-signed certificate under `~/.ohho/gateway/`. On every start it prints:

- a pairing code, when the token is a generated or remembered pairing code (not when `OHHO_GATEWAY_TOKEN` or `--token-file` supplied the token)
- the certificate SHA-256 fingerprint (Unity pin)
- the SPKI pin `sha256/...` (Android OkHttp)

Clients send `Authorization: Bearer <token>` or `?token=`. Both may be sent only when they are identical. A missing or wrong token is HTTP 401 for ordinary requests and for the WebSocket upgrade, and the upstream is not contacted.

| Client path | Upstream |
|---|---|
| `wss://<host>:8443/rosbridge` | `ws://127.0.0.1:9090/` |
| `https://<host>:8443/video/...` | `http://127.0.0.1:8080/...` |
| `wss://<host>:8443/foxglove` | `ws://127.0.0.1:8765/` |
| `wss://<host>:8443/webrtc/signaling` | 503 until `--webrtc` is set |

`--tls-only` closes the plain loopback port (cloud). `--plain-host` other than loopback refuses to start.

The robot and Pi launch files include `omnibot_bringup/launch/ohho_gateway.launch.py`. See `docs/adr/OHH-196-authenticated-gateway.md`.

```bash
pytest ohho-gateway/tests -q
```
