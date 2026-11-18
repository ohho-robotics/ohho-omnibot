# omnibot_lerobot

SmolVLA policy inference and teleop dataset recording for OmniBot.

Recording is done on the robot (or the sim) by `teleop_recorder_node`. The Quest headset does not record locally and does not POST episodes anywhere.

## Recording control

Joystick control is unchanged: Xbox RB (button 7) starts, and RB again saves; LB (button 6) discards the open episode.

The same episode machine is also exposed for rosbridge. A second start while a recording is open is refused, so it cannot open another episode. Stop or discard while idle returns `success: false` and does not write a file. Discard drops the in-progress buffer and does not advance the episode index.

### Services

All three are `std_srvs/srv/Trigger` (empty request). The response `message` ends with `(episode_index=N)`, which is the episode that was started, saved, or dropped.

| Service | Effect |
|---|---|
| `/ohho/record/start` | Start one episode. Fails if the node is already `recording` or `saving`. |
| `/ohho/record/stop` | Save the open episode and advance the index. Fails if idle, if already saving, or if the buffer is empty (nothing is written). |
| `/ohho/record/discard` | Drop the open episode. Nothing is written and the index does not advance. Fails if the node is not `recording`. |

Task name, in order:

1. The latest non-empty `std_msgs/String` on `/ohho/record/task`.
2. Otherwise the `task_name` parameter (`mobile manipulation task` by default).

Episode index, in order:

1. The latest `std_msgs/Int32` on `/ohho/record/episode_index`. This value is used once, on the next successful start, then cleared. A negative value means "use the recorder's next index".
2. Otherwise the `episode_index` parameter. `-1` (the default) means the next index. A value `>= 0` is sticky: every start uses that index until the parameter is set back to `-1`.

### State topic

`/ohho/record/state` (`std_msgs/String`) is published immediately when the phase, task, or episode index changes, and at 2 Hz so `frame_count` stays current. The publisher is reliable, keep-last 1, transient-local (latched).

`data` is a JSON object:

```json
{
  "state": "recording",
  "task": "pick up the cup",
  "episode_index": 0,
  "frame_count": 12,
  "started_at": 1760438400.5
}
```

`state` is `idle`, `recording`, or `saving`. `started_at` is Unix seconds, or `0` when idle. `episode_index` while idle is the next index the recorder will use unless a start overrides it.

### rosbridge `call_service`

The headset already talks to rosbridge (port 9090 in `robot.launch.py` and `simulation.launch.py`). Do not use port 8765 for recording: that port is the Foxglove bridge.

Publish the task, then start:

```json
{"op":"publish","topic":"/ohho/record/task","type":"std_msgs/msg/String","msg":{"data":"pick up the cup"}}
```

```json
{"op":"publish","topic":"/ohho/record/episode_index","type":"std_msgs/msg/Int32","msg":{"data":4}}
```

Omit the episode-index publish, or send `data: -1`, to take the next index.

```json
{"op":"call_service","id":"record-start-1","service":"/ohho/record/start","type":"std_srvs/srv/Trigger","args":{}}
```

```json
{"op":"call_service","id":"record-stop-1","service":"/ohho/record/stop","type":"std_srvs/srv/Trigger","args":{}}
```

```json
{"op":"call_service","id":"record-discard-1","service":"/ohho/record/discard","type":"std_srvs/srv/Trigger","args":{}}
```

A start response looks like:

```json
{"op":"service_response","id":"record-start-1","service":"/ohho/record/start","result":true,"values":{"success":true,"message":"recording (episode_index=4)"}}
```

Subscribe for the HUD (state and frame count):

```json
{"op":"subscribe","id":"record-state","topic":"/ohho/record/state","type":"std_msgs/msg/String"}
```

The bridge then publishes `msg.data` as a JSON string. Parse that string. Do not treat the rosbridge envelope as the recording state.

## VR action source (`action_source:=vr`)

`action_source` defaults to `leader`. That path is unchanged: the leader arm is the arm action, Xbox axes are the base action, and RB/LB still start, save, and discard.

`action_source:=vr` records the VR teleop command instead.

| Vector | Source |
|---|---|
| Arm action (6) | Last `JointState` on `vr_arm_command_topic` (default `/arm/joint_commands/teleop`, the OHH-197 teleop input). |
| Arm fallback | `vr_arm_mux_topic` (default `/arm/joint_commands/out`) only while `/control_mode` is `teleop`. A policy or navigation command on the mux is not stored. |
| Base action (3) | Last twist on `vr_base_command_topic` (default `/cmd_vel/teleop`). `/cmd_vel/out` is the same teleop-only fallback. |
| State (9) | Follower `/arm/joint_states` plus `/odom` (`vx`, `vy`, `wz`). |

`vr_action_prefer:=teleop` (default) uses the dedicated topic whenever a sample has arrived. `vr_action_prefer:=mux` uses the mux output when control mode is teleop, which is the command the mux is forwarding. Point `vr_arm_command_topic` at the post-safety stream if teleop safety republishes on a different name. The recorder only subscribes. It does not publish joint commands or base twists.

Episode control in vr mode is the `/ohho/record` services above. A joystick is not required. If a joystick is connected, RB/LB still call the same start, save, and discard path.

Each saved VR episode is a LeRobot v2.0 dataset (parquet, mp4, and a `.npy` camera array for the quality filter). `meta/episodes.jsonl` stores:

- `operator` — `vr` unless `operator` is set
- `robot_kind` — `sim` or `real`
- `sim_world` — simulator world name, or empty
- `task` — the task string

```bash
ros2 launch omnibot_lerobot teleop_record.launch.py \
  action_source:=vr robot_kind:=sim sim_world:=warehouse \
  task_name:="pick up the cup"
```

Validate the dataset and run the OHH-99 filter (from `omnibot-ai-engines`, with `pyarrow` installed):

```bash
PYTHONPATH=omnibot-ai-engines python -m data_engine.scripts.validate_dataset \
  --dataset ~/datasets/mobile_manipulation --verbose
PYTHONPATH=omnibot-ai-engines python -m data_engine.scripts.filter_episodes \
  --dataset ~/datasets/mobile_manipulation
```

A headset episode was not recorded here. `test/test_vr_action.py` builds a 30 s episode from synthetic teleop samples and runs both commands. `test/fake_vr_teleop_smoke.py` publishes those topics under ROS 2. Run it only in the `navdev` image (`omnibot-nav-dev`) with `ROS_LOCALHOST_ONLY=1` and a `ROS_DOMAIN_ID` other than 30. That image's NumPy is 1.26, so install `pyarrow==18.1.0` there (do not upgrade NumPy). The script refuses to publish if those environment variables are missing.

## VR follow-up (OhhO-VR)

The robot is the source of truth. The headset should not record locally and should not upload a JSONL file.

Port 8765 is Foxglove (`omnibot_bringup` `robot.launch.py` and `simulation.launch.py`). Nothing on the robot accepts `POST /upload_episode`. Remove every `:8765` recording URL.

`Assets/Scripts/Core/ROSBridgeClient.cs` can publish and subscribe, and `HandleMessage` only dispatches `op: "publish"`. Add a `call_service` send, and dispatch `op: "service_response"` by `id` so the UI can read `values.success` and `values.message`.

`Assets/Scripts/UI/RecordingPanel.cs`

- Delete `bridgeHttpPort`, `ExportAllEpisodes`, and `ExportSingleEpisode` (the `UnityWebRequest` POST to `http://<robotIp>:8765/upload_episode`).
- `OnStartClicked`: publish the episode-name field to `/ohho/record/task`, then `call_service` `/ohho/record/start`.
- `OnStopSaveClicked`: `call_service` `/ohho/record/stop`.
- `OnDiscardClicked`: `call_service` `/ohho/record/discard`.
- Drive the indicator and button enables from `/ohho/record/state`, not from `EpisodeManager`.

`Assets/Scripts/UI/TeleopHudController.cs`

- `OnStartRecording`, `OnStopSave`, and `OnDiscard` should call the three services instead of `EpisodeManager`.
- `RefreshRecordingUi` should show `state`, `task`, and `frame_count` from `/ohho/record/state`.
- Delete `ExportAllEpisodes` and `ExportSingleEpisode` (the POST around the `bridgeHttpPort` field, including the request near line 322).

`Assets/Scripts/Recording/DatasetRecorder.cs` and `Assets/Scripts/Recording/ProfileDrivenRecorder.cs`

- Stop writing JSONL under `Application.persistentDataPath/recordings`. Do not flush a file on stop, and do not flush one on discard.

`Assets/Scripts/Recording/EpisodeManager.cs`

- Stop using it as the dataset state machine. The HUD follows `/ohho/record/state`.

`Assets/Scripts/UI/Teleop/ControlsPanelController.cs`

- Leave this file on `rosbag:start` / `rosbag:stop` via `/mission/command`. Those buttons are rosbag control, not the LeRobot episode recorder.

Update the VR README and `SCENE_SETUP.md` so they no longer tell operators to POST episodes to `:8765`.
