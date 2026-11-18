# ACT on the VR sim pick dataset

Train a checkpoint against the OHH-237 sim VR pick set, serve it through
`policy_node` in `serve` mode, and let the headset policy panel load, start,
stop, and monitor it. Control stays in teleop until the operator confirms.
`SafetyGate`, `teleop_safety`, and e-stop are unchanged.

## Dataset

OHH-237 committed a dataset card at `datasets/omnibot-sim-vr-pick` on branch
`ohh-237-sim-vr-pick`. This branch does not copy the card. The smoke episodes
were left in the nav-dev container and are not in git.

| Field | Value |
| --- | --- |
| Local LeRobot v2 root | `/tmp/omnibot-sim-vr-pick` |
| `repo_id` | `local/omnibot-sim-vr-pick` |
| Card path (OHH-237 branch) | `datasets/omnibot-sim-vr-pick` |
| Hugging Face dataset | `ohho-robotics/omnibot-sim-vr-pick` (not created) |
| Policy id | `act-vr-sim-pick` |
| Local checkpoint | `checkpoints/act_vr_sim_pick/best` |
| Hugging Face checkpoint | `ohho-robotics/omnibot-act-vr-sim-pick` (not created) |

Config file: `omnibot-ai-engines/lerobot_engine/configs/act_vr_sim_pick.yaml`.

### Hugging Face push (human)

After at least 50 validated episodes are on disk:

```bash
huggingface-cli login
huggingface-cli upload ohho-robotics/omnibot-sim-vr-pick /tmp/omnibot-sim-vr-pick --repo-type dataset
```

Then set `dataset.hf_published: true` in the YAML. A checkpoint push is a
separate step and was not run:

```bash
huggingface-cli upload ohho-robotics/omnibot-act-vr-sim-pick checkpoints/act_vr_sim_pick/best --repo-type model
```

## Train

CPU smoke. Two numpy steps on synthetic 9-DoF chunks. This does not load
LeRobot ACT, does not read the VR episodes, and does not start W&B.

```bash
python omnibot-ai-engines/lerobot_engine/train_act_vr.py --smoke
```

The manifest at `checkpoints/act_vr_sim_pick/train_manifest.json` records
`weights_are_lerobot_act: false` and `wandb_run: null`.

When LeRobot's ACT package imports, a separate two-step CPU smoke builds a
tiny `ACTPolicy` (ResNet-18 with no pretrained weights, one synthetic batch,
no W&B, weights not written):

```bash
python omnibot-ai-engines/lerobot_engine/train_act_vr.py --act-smoke
```

That command exits 4 when ACT is not installed. It is not a fine-tune on the
VR episodes. A run on this PC is `docs/evidence/act_cpu_smoke.json`: torch
2.14.1+cpu, CUDA unavailable, 11,287,365 parameters updated, weights not
saved, W&B not started. On that one random batch the second loss was higher
than the first (`loss_decreased` is false).

Full ACT, once a LeRobot v2 root exists and `torch` plus `lerobot` import.
`--run-full` is what actually starts `lerobot_engine/train.py`. Without it
the command only prints the argv.

```bash
python omnibot-ai-engines/lerobot_engine/train_act_vr.py --dataset-path /tmp/omnibot-sim-vr-pick --run-full
```

If the root is missing and the HF dataset is still unpublished, the process
exits 2 and prints the push steps. It does not invent a training run.

## Serve

Start OhhO Serve yourself (`ohho serve --checkpoint checkpoints/act_vr_sim_pick/best`
from ohho-os, or the stdlib mock in `lerobot_engine/serve/mock_server.py`).
Then:

```bash
ros2 launch omnibot_lerobot act_vr_sim_pick.launch.py
```

That launch sets `model_type:=serve`, checkpoint
`checkpoints/act_vr_sim_pick/best`, and `serve_url:=http://127.0.0.1:8000`.
It starts `arm_cmd_mux` and does not publish `/control_mode`. The mux still
holds policy commands while the mode is teleop.

`policy_node` subscribes to `/policy/load`. A new checkpoint string reloads
the serve adapter. The callback does not set inference enabled. `/policy/enable`
still comes from the headset Start button. `PolicyGuard` still drops black
frames, drops late actions, and clamps joint deltas to 0.15 rad. An inference
exception publishes a zero base twist.

Headset Start publishes, in order, `/policy/enable` true, `/arm/cmd_mode`
`policy`, and `/control_mode` `vla`. Stop publishes `/control_mode` `teleop`,
disables inference, and zeros base and arm commands. E-stop still blocks the
arm stream inside `ArmStreamGate`.

## Eval

Twenty sim trials, seeds 237 through 256 (the OHH-237 smoke used seed 237).
The scaffold writes a blank success column. It does not launch Gazebo and it
does not fill a rate.

```bash
python omnibot-ai-engines/lerobot_engine/eval_act_vr_sim.py --write docs/act-vr-sim-pick-trials.md
```

When `scripted_vr_pick.launch.py` from OHH-237 is available, one trial is:

```bash
ros2 launch omnibot_lerobot scripted_vr_pick.launch.py world:=apartment pick_place:=true pick_place_seed:=SEED episodes:=1
```

Score success only when the cube is in the bin. Until then the rate stays
"not run". The OHH-237 smoke did not measure that outcome, and this change
did not run the 20 trials.

## Headset

OhhO-VR branch `ohh-243-act-vr-loop`. In a sim session the policy panel lists
`act-vr-sim-pick` with checkpoint `checkpoints/act_vr_sim_pick/best`. Load
publishes `/policy/load` and `POST /load_model`. Start requires confirmation
and, on a real robot, the supervise hold. Sim does not require supervise.
Stop and e-stop return to teleop. Status text reads latency and guard trips
from `/ohho/telemetry`.
