# Model adapter interface

A model runs in the policy runtime by implementing `PolicyAdapter` in [`omnibot-ai-engines/lerobot_engine/models/base.py`](../omnibot-ai-engines/lerobot_engine/models/base.py). Register the class with `@register` in [`registry.py`](../omnibot-ai-engines/lerobot_engine/models/registry.py). `policy_node` loads it with `make_policy(model_type)` and calls `select_action` once per control cycle. The same contract covers every registered backend. The interface does not name a robot vendor.

`adapter.contract` is the declaration the runtime reads: observation keys and image shape, state, action units, control rate, and the safety hooks. Those hooks are applied by [`policy_guard.py`](../omnibot-ai-ros2/omnibot_lerobot/omnibot_lerobot/policy_guard.py) and [`policy_node.py`](../omnibot-ai-ros2/omnibot_lerobot/omnibot_lerobot/policy_node.py). A model cannot turn them off. Numeric ceilings for the current node are in [safety.md](safety.md).

## Inputs

Cameras arrive as uint8 HWC. Before `select_action`, the runtime resizes each image to `image_size` `(width, height)` and converts it to float32 in `[0, 1]`, layout `(1, channels, height, width)`.

| Field | Default |
| --- | --- |
| Image keys | `observation.images.wrist`, `observation.images.bev` |
| Channels | 3 |
| Resize | 320 × 240 (width × height) |
| State key | `observation.state` |
| State shape | `(1, 9)` float32 |
| State units | six joint positions in rad, then `base_vx` m/s, `base_vy` m/s, `base_yaw_rate` rad/s |
| Task key | `None`, or `"task"` when the model is language-conditioned |

An adapter may override the keys, the image size, and the vector length. `select_action` may ignore keys it does not use. It must tolerate the shapes above.

## Outputs

`select_action` returns one finite float32 vector of shape `(action_dim,)`.

| Index | Name | Unit |
| --- | --- | --- |
| 0–5 | `joint_0` … `joint_5` | rad (position target) |
| 6 | `base_vx` | m/s |
| 7 | `base_vy` | m/s |
| 8 | `base_yaw_rate` | rad/s |

The current policy node publishes `action[:6]` on `/arm/joint_commands/policy` and `action[6:9]` on `/cmd_vel` (remapped to `/cmd_vel/vla` by the launch file). Publishing happens only after the safety hooks below.

## Control rate and latency

`control_hz` defaults to 10. The latency budget is one period, `1 / control_hz` (0.1 s at the default). `policy_node` uses its `policy_hz` parameter for that period, also 10 unless overridden.

A tick is stale when preprocess plus `select_action` takes longer than the period. The node drops that action, publishes zero base velocity, and does not publish an arm command. The reason string is `stale_inference`.

## Safety hooks

These run on every enabled tick. They are `RUNTIME_SAFETY` on the contract.

1. **Black-frame refusal.** A required image that is missing, empty, not an array, or all zeros is refused. No arm command is published. Base velocity is set to zero. The reason string is `missing_or_black_images`. The runtime does not replace a black frame with zeros and then infer.
2. **Stale-inference stop.** A tick slower than the latency budget is dropped the same way. The reason string is `stale_inference`.
3. **Action limits.** Joint targets move at most `MAX_JOINT_DELTA_RAD` (0.15 rad) from the previous command in one cycle, and they stay inside the configured joint minimum and maximum. Base velocity is multiplied by `base_vel_scale` (default 0.3) and clipped to the runtime ceilings in `policy_guard.py` (0.20 m/s linear, 0.50 rad/s angular).

## Two headless runs

Two adapters in [`reference.py`](../omnibot-ai-engines/lerobot_engine/models/reference.py) implement this contract with randomly initialised numpy weights. No checkpoint file was downloaded. No GPU was required. torch was not required.

| | `reference_linear` | `reference_mlp` |
| --- | --- | --- |
| Seed | seed 447 | seed 448 |
| Weights | `W` `(9, 11)` and bias `(9,)`, `normal(0, 0.05)`, bias zeros, float32 | `W1` `(16, 15)`, `W2` `(9, 16)`, biases zero, `tanh` hidden layer, same distribution, float32 |
| Features | 9-D state, mean of the wrist image, mean of the bird's-eye image | 9-D state, per-channel mean of each image |
| Device | `requested_device` cpu, `executed_on` cpu | `requested_device` cpu, `executed_on` cpu |
| Checkpoint | argument `random`, `downloaded_checkpoint` false, `checkpoint_file` null | argument `random`, `downloaded_checkpoint` false, `checkpoint_file` null |

Each run is one trial of the headless pick-and-place harness (`GazeboSimBackend` with an in-memory transport) and one enabled tick of `PolicyNode._inference_loop` under ROS stubs. Gazebo was not started. rclpy was not started. No robot device was opened.

The check is `omnibot-ai-ros2/omnibot_lerobot/test/test_model_adapter_runs.py`. It also refuses a black frame and drops a stale tick for each adapter. Those runs do not measure a task success rate. The trial result is a timeout because the in-memory transport never reports a cube pose.

ACT, SmolVLA, diffusion, and OpenVLA use the same `PolicyAdapter` methods. Loading their published checkpoints needs the matching library and, for the large checkpoints, a GPU. Those checkpoints were not loaded in the runs above.
