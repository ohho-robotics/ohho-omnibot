# OHH-196 client follow-ups

The gateway in this repository is the server side of OHH-196. Quest (`omnibot-vr`), Android (`omnibot-android`), and the website Connect page are not changed here. The notes below are the patch those follow-up issues should apply.

## Contract

Default LAN base: `wss://<robot-ip>:8443` and `https://<robot-ip>:8443`.

| Purpose | URL |
|---|---|
| rosbridge | `wss://<host>:8443/rosbridge?token=<TOKEN>` |
| MJPEG | `https://<host>:8443/video/stream?topic=<TOPIC>&token=<TOKEN>` |
| Foxglove | `wss://<host>:8443/foxglove?token=<TOKEN>` |
| WebRTC signaling (reserved, OHH-210) | `wss://<host>:8443/webrtc/signaling?token=<TOKEN>` |

`TOKEN` is the pairing code printed by `ohho-gateway` at startup, or `OHHO_GATEWAY_TOKEN` / the token file when an operator set one. URL-encode the token.

Header form, when the client can set it: `Authorization: Bearer <TOKEN>`. Omit the query parameter in that case. If both are sent they must be the same string or the gateway returns 401.

A missing or wrong token is HTTP 401 on GET, POST, and on the WebSocket upgrade. The robot process is not contacted.

Cloud clients use `wss://` and `https://` only. Do not open `ws://` or `http://` toward a cloud host. The robot launch closes the plain port with `gateway_tls_only:=true`. Same-machine tools may use `ws://127.0.0.1:9080/rosbridge?token=<TOKEN>`; that port is loopback-only and still requires the token.

## Certificate pin

Startup prints two lines:

```text
ohho-gateway TLS certificate SHA-256: AA:BB:...
ohho-gateway TLS SPKI pin: sha256/<base64>
```

The certificate fingerprint is SHA-256 of the raw DER certificate (Unity `CertificateHandler` receives those bytes). The SPKI pin is SHA-256 of the SubjectPublicKeyInfo, base64, with a `sha256/` prefix (OkHttp `CertificatePinner`). They are not interchangeable.

The certificate is self-signed. Its names are `localhost`, `ohho-gateway`, and `127.0.0.1`. A LAN IP will not match the name. Clients must accept the connection only when the pin matches, and must reject every other certificate. A callback that returns true for every certificate is not a pin.

The same files (`~/.ohho/gateway/cert.pem` and `key.pem`) are reused across restarts. Replacing them with a publicly trusted pair is how a cloud deployment gets a browser-trusted name; clients then pin that new fingerprint.

## Unity (`omnibot-vr`)

Do not edit these files in the OHH-196 gateway branch. Apply the following in the Quest follow-up.

### `Assets/Scripts/Core/ConnectionManager.cs`

Today `Connect` builds `ws://{ip}:{port}` and `Awake` defaults `robot_port` to 9090.

- Add PlayerPrefs `ohho.gateway.token` and `ohho.gateway.cert_sha256` (hex, colons optional).
- Default `robot_port` to **8443**.
- Build the URL as:

```csharp
string token = Uri.EscapeDataString(PlayerPrefs.GetString("ohho.gateway.token", ""));
string url = $"wss://{ip}:{port}/rosbridge?token={token}";
ROSBridgeClient.Instance.Connect(url);
```

`ConnectionPanel` (`robot_port` default 9090) and `LastRobotStore` (default 9090) need the same default port, plus a token field and a fingerprint field. `TeleopHudController` hardcodes `int port = 9090`; pass 8443 and the token the same way.

### `Assets/Scripts/Core/ROSBridgeClient.cs`

`ConnectAsync` does `new WebSocket(url)`. Keep the token in the query string. NativeWebSocket on Quest/WebGL cannot be relied on to send `Authorization`. Do not also send a different token in a header.

`wss://` with a self-signed certificate will fail the platform trust check. Add a validation callback that hashes the server certificate with SHA-256 and compares it to `ohho.gateway.cert_sha256` with colons removed, ordinal ignore-case. Return false for every other certificate. Do not assign a callback that returns true unconditionally.

`ROSConnectionWrapper.Connect` still formats `ws://{ip}:{port}` and never opens the socket. Point it at the same `wss://.../rosbridge?token=` URL when that wrapper is wired up.

### `Assets/Scripts/Video/MjpegVideoSource.cs`

`Start` uses `http://{robotIp}:8080/stream?topic=...`. Change it to the gateway:

```csharp
string token = Uri.EscapeDataString(PlayerPrefs.GetString("ohho.gateway.token", ""));
string topic = Uri.EscapeDataString(topicName);
string url =
    $"https://{robotIp}:8443/video/stream?topic={topic}&token={token}";
```

On the `UnityWebRequest`, set `certificateHandler` to a `CertificateHandler` whose `ValidateCertificate` hashes `certificateData` with SHA-256 and compares it to the stored fingerprint. Return false otherwise. `CameraFeedController` can keep calling `MjpegVideoSource.Start(robotIp, topic)`; the source reads the token itself.

### `Assets/Scripts/Video/WebRtcVideoSource.cs`

Signaling is still a placeholder (`OnError` → MJPEG fallback). When OHH-210 lands, negotiate at `wss://{robotIp}:8443/webrtc/signaling?token=<TOKEN>` (or `https://` for an HTTP offer). Until then the gateway answers 503 on that path after a valid token, and 401 without one. Leave the MJPEG fallback in place.

## Android (`omnibot-android`)

### `RobotRepository.connect`

Today:

```kotlin
rosManager = ROSBridgeManager("ws://$ip:$port", rosListener)
```

Change the URL to:

```kotlin
val encoded = URLEncoder.encode(token, Charsets.UTF_8.name())
val url = "wss://$ip:$port/rosbridge?token=$encoded"
```

Default the port to 8443 (`Constants.DEFAULT_ROSBRIDGE_PORT` is 9090 today). `HomeFragment` also interpolates `:9090` into the status line; show `:8443`.

Build the `OkHttpClient` inside `ROSBridgeManager` with a trust manager that accepts only the gateway certificate:

- Compute SHA-256 of the leaf certificate's encoded bytes and compare to the stored certificate fingerprint, **or** pin the SPKI with `CertificatePinner.Builder().add(host, spkiPin)`.
- A `CertificatePinner` does not replace the system trust store. A self-signed LAN cert still needs a `TrustManager` / `sslSocketFactory` that trusts that one certificate. Reject every certificate whose hash does not match. Do not use a trust-all manager.
- Apply the same client to the MJPEG request.

### `CameraViewModel`

Today:

```kotlin
val url = "http://${status.ipAddress}:8080/stream?topic=/camera/front/image_raw&type=mjpeg&quality=50"
```

Change it to:

```kotlin
val url = "https://${status.ipAddress}:8443/video/stream" +
    "?topic=/camera/front/image_raw&type=mjpeg&quality=50&token=$encodedToken"
```

The other topics (`/camera/base/bev/image_raw`, `/camera/color/image_raw`, `/camera/wrist/image_raw`) keep their topic query and move under the same `/video/stream` prefix.

## Connect (browser)

The Connect page is not in this repository. It currently expects `ws://localhost:9090` for the sim, which remains valid: `sim.launch.py` still binds rosbridge to `127.0.0.1:9090` with no gateway.

For a robot or a cloud sim:

- Rosbridge: `wss://<host>:8443/rosbridge?token=<TOKEN>`
- Camera: `https://<host>:8443/video/stream?topic=/camera/front/image_raw&token=<TOKEN>`
- If the page is cloud, or the operator selected cloud, refuse a `ws://` or `http://` robot URL.
- For the self-signed LAN cert, installing `cert.pem` does not make `https://<robot-ip>:8443` succeed in a browser. The generated certificate's SAN is only `localhost`, `ohho-gateway`, and `127.0.0.1`, so a LAN IP or other hostname fails the name check even when the file is installed. The browser path needs a certificate that lists that robot host name or IP in its SAN. Replacing the state-dir pair with that certificate changes the pin, so clients must be re-paired. Unity and Android should pin the fingerprint instead of trusting the name. Do not instruct the page to ignore certificate errors.

## Install on the robot

The launch file runs `python3 -m ohho_gateway`. On the Pi or the sim host:

```bash
pip install -e ohho-gateway
```

When the gateway uses its generated or remembered pairing code, the pairing code and both pins are printed in the launch log on every start. They are not ROS parameters.
