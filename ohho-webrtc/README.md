# ohho_webrtc (OHH-210)

Robot-side WebRTC for OmniBot. A ROS 2 node subscribes to `sensor_msgs/Image` topics and an aiortc peer connection sends one H.264 track per negotiated camera. Signaling is a WebSocket on loopback. The OHH-196 gateway is the LAN and cloud face.

MJPEG through `web_video_server` is the fallback the Quest client already uses. This package is the low-latency path that client is waiting on.

## Topics

| Track | Default topic | On main today |
|---|---|---|
| chase | `/camera/chase/image_raw` | Not published. OHH-200 is still open. This is the name to publish. |
| front | `/camera/front/image_raw` | Gazebo bridge and the hybrid launch. |
| wrist | `/camera/wrist/image_raw` | `perception.launch.py` (`usb_cam` namespace `camera/wrist`). |

Override with ROS parameters `track_names` and `track_topics` (same length), or `ohho-webrtc --track chase=/some/image_raw`. See `config/webrtc_params.yaml`.

Supported image encodings are `rgb8`, `bgr8`, and `mono8`.

## Latency settings

These are fixed in the encoder choice, not optional:

- B-frames off (`bframes` must be 0; any other value is a config error)
- GOP 15 (half a second at 30 fps; allowed range 1–30)
- 3 Mbps default, allowed range 2–4 Mbps
- 1280×720 at 30 fps

Every encoded frame has the capture time burned in as `chase 1710000000123` (track name and Unix milliseconds). That is the number a glass-to-glass photo reads.

Encoder selection:

- `auto` opens `h264_nvenc` only after a child process encodes one frame with it. A missing GPU, or a driver that faults inside PyAV, stays on `libx264`. The check cannot take down the streamer.
- The same choice builds a GStreamer `webrtcbin` recipe: `nvh264enc` on an RTX desktop, `v4l2h264enc` when that element is installed (Pi), otherwise `x264enc` with `tune=zerolatency`. The recipe is logged and returned by `/health`. The peer connection that runs here is aiortc, using the PyAV codec above, with the same GOP, bitrate, and B-frame policy.
- `encoder:=v4l2` or `encoder:=nvenc` forces that recipe and falls back to x264 when the element or the NVENC probe is missing.

`aiortc` is optional. From this repo: `pip install -e "ohho-webrtc[media]"`, or `pip install aiortc`. Without it the signaling socket still comes up and answers an offer with `media_unavailable`.

## Signaling and the gateway

This process binds `127.0.0.1:8766` by default.

| Who | URL |
|---|---|
| Browser on the robot PC | `http://127.0.0.1:8766/` |
| Upstream WebSocket (what the gateway connects to) | `ws://127.0.0.1:8766/signaling` |
| Client through OHH-196 | `wss://<robot>:8443/webrtc/signaling?token=<TOKEN>` |

OHH-196 returns 503 on `/webrtc/signaling` until an upstream is set. It strips the `/webrtc` prefix, so `/webrtc/signaling` arrives here as `/signaling`. Point the gateway at this process with:

```bash
ohho-gateway --webrtc http://127.0.0.1:8766
```

The test page is also served for `https://<robot>:8443/webrtc/?token=<TOKEN>` because the gateway strips `/webrtc` and proxies the HTTP GET to `/`. The page then opens `wss://<robot>:8443/webrtc/signaling` and keeps the token query. STUN defaults to `stun:stun.l.google.com:19302`. Set `turn_urls`, `turn_username`, and `turn_credential` for a cloud client; both the browser `iceServers` list and the GStreamer recipe pick them up.

Message shapes (JSON text frames):

- Client `join`: `type`, `role` (`viewer` or `publisher`), `tracks` (list of names)
- Client `offer` / `answer`: `type`, `sdp` (starts with `v=`), `track` or `tracks`
- Client `ice`: `type`, `track`, `candidate`, `sdpMid`, `sdpMLineIndex`
- Client `list`: `type`
- Server `welcome`: `type`, `role`, `viewer`, `tracks`, `iceServers`, `signalingPath`, `gatewayPath`
- Server `error`: `type`, `error`

## Run

On a machine with ROS 2 Jazzy, from a sourced workspace that contains this package (`src/ohho_webrtc` links here):

```bash
ros2 launch ohho_webrtc webrtc_streamer.launch.py
```

Without ROS, the loopback server can still feed the test page. It does not subscribe to cameras:

```bash
pip install numpy pyyaml aiortc
python -m ohho_webrtc --test-pattern
```

Open `http://127.0.0.1:8766/`, choose `chase`, and press Connect. Chrome shows one `<video>` element.

## Glass-to-glass measurement (human, on the robot)

This checkout cannot measure it: there is no camera, no Quest, and no robot here.

1. Start the streamer on the robot so it subscribes to the chase topic (or `--test-pattern` only to check the page).
2. Put a millisecond clock in the chase camera's view.
3. Open the test page in Chrome on the LAN, either `http://<robot-pc>:8766/` while the socket is bound on that interface, or through the gateway URL above. The default bind is loopback; use the gateway for another machine.
4. Take one photo that contains both the physical clock and the Chrome video.
5. Subtract the clock reading visible inside the video from the physical clock. The overlay `chase <unix-ms>` is the stamp at capture, for the same photo.
6. The OHH-210 bar is under 120 ms.

To bind beyond loopback for that photo, set `signaling_host` to `0.0.0.0`. Prefer the gateway so the socket is not open without the pairing token.

## Tests

```bash
pytest ohho-webrtc/tests -q
```

Config parsing, track registration, signaling shapes, the loopback socket, and the GStreamer recipe run without ROS, a GPU, or a camera. The aiortc frame test runs when `aiortc` is installed and is skipped otherwise.

## Follow-ups

- OHH-200: publish `/camera/chase/image_raw`.
- OHH-196: operators pass `--webrtc http://127.0.0.1:8766` once this node is up. The gateway package is not modified here.
- Quest `WebRtcVideoSource` is still a stub. It should open `wss://<robot>:8443/webrtc/signaling?token=<TOKEN>` and speak the JSON shapes above. That edit belongs in the VR tree, not this package.
- A GStreamer `webrtcbin` process that actually runs the logged `nvh264enc` / `v4l2h264enc` pipeline, for the Pi's hardware encoder. The live path today is aiortc.
