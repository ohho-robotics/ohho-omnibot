# ACT GPU training and 20-trial eval (OHH-52)

**Needs Varun: budget + robot.**

Fine-tune the existing ACT config, then score 20 trials on the robot. Step 1
only reads files in this checkout. Step 2 is the budget checkpoint. The
training command in step 3 stays unrun until that checkpoint is filled. The
robot section stays unrun until Varun is with the robot and a `best`
checkpoint is on disk.

## Config this run uses

| | |
|---|---|
| ACT config | [`omnibot-ai-engines/lerobot_engine/configs/models/act.yaml`](../omnibot-ai-engines/lerobot_engine/configs/models/act.yaml) |
| Robot spec | [`omnibot-ai-engines/lerobot_engine/configs/robot/omnibot_mobile_manip.yaml`](../omnibot-ai-engines/lerobot_engine/configs/robot/omnibot_mobile_manip.yaml) |
| Trainer | [`omnibot-ai-engines/lerobot_engine/train.py`](../omnibot-ai-engines/lerobot_engine/train.py) |
| Inference launch | [`omnibot-ai-ros2/omnibot_lerobot/launch/policy_inference.launch.py`](../omnibot-ai-ros2/omnibot_lerobot/launch/policy_inference.launch.py) |
| Trial sheet | [results.md](results.md) |

`train.py` does not load `act.yaml`. Pass the yaml values as flags.

| `act.yaml` | Flag |
|---|---|
| `model.type: act` | `--model act` |
| `model.pretrained: lerobot/act_base` | `--checkpoint lerobot/act_base` |
| `model.device: cuda` | `--device cuda` |
| `model.chunk_size: 100` | `--chunk-size 100` |
| `training.num_epochs: 200` | `--num-epochs 200` |
| `training.batch_size: 16` | `--batch-size 16` |
| `training.lr: 1.0e-4` | `--lr 1e-4` |
| `training.grad_clip_norm: 10.0` | `--grad-clip-norm 10.0` |
| `training.save_every: 20` | `--save-every 20` |
| `training.num_workers: 4` | `--num-workers 4` |

Three yaml fields have no separate effect on the training loop:

- `training.lr_scheduler: cosine` matches `CosineAnnealingLR` inside `train.py`. There is no `--lr-scheduler` flag.
- `--chunk-size` is stored on the W&B config when `--wandb-project` is set. `make_policy` does not take `chunk_size`.
- `model.policy_hz: 20.0` is the inference rate. `train.py` does not read it. [`policy_params.yaml`](../omnibot-ai-ros2/omnibot_lerobot/config/policy_params.yaml) sets `policy_hz: 10.0`, and the inference launch does not declare `policy_hz`. Step 5 applies 20.0 on the robot checkout.

The robot spec is the 9-D mobile manipulator (`action_dim` / `state_dim` 9) with `observation.images.wrist` and `observation.images.bev`.

Checkpoints are written inside the epoch loop: `checkpoint_epoch_XXXX` every `save_every` epochs and on the final epoch, and `best/` when held-out eval loss improves. The dry-run rejects a `train.py` where that loop only calls `policy_model.train()`.

## 1. Dry-run

From the repo root:

```bash
python scripts/ohh52_act_train_dry_run.py
```

Exit 0 means the paths, the ACT values, the `train.py` flags, the epoch-loop shape, and the 20-row trial sheet match this page. The script uses the Python standard library. It prints the training command and does not start it.

To check a dataset directory without reading its files:

```bash
python scripts/ohh52_act_train_dry_run.py --dataset-path /path/to/dataset
```

`--output-dir` is reported and not created. Extra arguments are refused. `OHH52_LAUNCH_GPU=1` and `OHH52_START_TRAINING=1` are refused. A variable that claims the budget is already approved still leaves the script in validate-only mode.

## 2. Budget checkpoint

Stop here. Fill this with Varun before step 3. Keep the copy in the ticket or on the training machine. Leave this table blank in git.

| Field | Agreed value |
|---|---|
| Agreed by | |
| Date | |
| Provider | |
| Spend cap (USD) | |
| Time cap (hours) | |
| Dataset path on that machine | |
| Output directory on that machine | |
| Stop rule | First of the spend cap, the time cap, or the stop steps in section 4 |

A blank cap means the training command stays unrun. An environment variable does not clear this checkpoint.

**Needs Varun: budget + robot.**

## 3. Train

Run this on the machine named in the checkpoint, after step 1 exited 0 on this commit and every field in step 2 has a value. `DATASET_PATH` and `OUTPUT_DIR` are those two paths.

```bash
pip install -r omnibot-ai-engines/lerobot_engine/requirements.txt
PYTHONPATH=omnibot-ai-engines python omnibot-ai-engines/lerobot_engine/train.py \
  --model act \
  --checkpoint lerobot/act_base \
  --dataset-path "$DATASET_PATH" \
  --output-dir "$OUTPUT_DIR" \
  --num-epochs 200 \
  --batch-size 16 \
  --lr 1e-4 \
  --device cuda \
  --chunk-size 100 \
  --save-every 20 \
  --grad-clip-norm 10.0 \
  --num-workers 4
```

The shell is bash on the training machine. `train.py` imports `lerobot_engine`, so `PYTHONPATH` is the `omnibot-ai-engines` directory.

On disk after a finished run:

- `$OUTPUT_DIR/checkpoint_epoch_0020`, then every 20 epochs, then the final epoch (`save_checkpoint` uses `checkpoint_epoch_{epoch:04d}`)
- `$OUTPUT_DIR/best` when eval loss improves, including epochs that are not a multiple of 20

`--checkpoint` loads weights through `ACTPolicy.from_pretrained`. A new process starts the optimizer and the epoch counter over. To continue under the same cap, point `--checkpoint` at `best` or a `checkpoint_epoch_*` directory and set `--num-epochs` to the epochs still inside the cap.

W&B stays off unless you also pass `--wandb-project omnibot_act`.

## 4. How to stop

1. Press Ctrl+C once in the training terminal and wait until the process exits. When W&B was started, the `finally` block calls `wandb.finish()`. Ctrl+C raises `KeyboardInterrupt`, which is not an `Exception`, so `Training interrupted at epoch` is not printed and `Training complete` is not printed either.
2. Keep `$OUTPUT_DIR/best` and the newest `checkpoint_epoch_*`. The epoch that was in progress has no checkpoint.
3. After the process has exited, terminate the GPU instance in the provider console. An idle instance keeps billing against the cap. This repo has no provider client. The dry-run will not call one.
4. When the spend cap or the time cap hits first, do steps 1–3 immediately. A replacement machine needs a new filled checkpoint.

## 5. Twenty trials on the robot

**Needs Varun: budget + robot.**

Do this with Varun at the robot, after `$OUTPUT_DIR/best` exists. Write one row per trial in [results.md](results.md).

`mobile_manipulation.launch.py` is the bring-up in this repo (`./launch_mobile_manipulation.sh` is not in the tree). It starts in `teleop`. Policy arm and base commands are forwarded when `/control_mode` is `vla`. See [reference.md](reference.md#bring-up-control-mode) and [safety.md](safety.md).

On the robot checkout, set `policy_node.ros__parameters.policy_hz` to `20.0` in [`policy_params.yaml`](../omnibot-ai-ros2/omnibot_lerobot/config/policy_params.yaml) so the session matches `act.yaml`. The file in git is `10.0`. Leave that edit uncommitted. The launch file cannot take `policy_hz:=`.

```bash
ros2 launch omnibot_bringup mobile_manipulation.launch.py
```

```bash
ros2 launch omnibot_lerobot policy_inference.launch.py \
  model_type:=act \
  checkpoint:="$OUTPUT_DIR/best" \
  device:=cuda
```

```bash
ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'vla'"
ros2 topic pub --once /policy/enable std_msgs/msg/Bool "data: true"
```

Reset the scene, run one attempt, stop, then fill the row. Success and failure are exclusive. Intervention is `yes` when a person used the e-stop, disabled the policy, or moved the robot during the trial.

Stop a trial:

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: true"
ros2 topic pub --once /policy/enable std_msgs/msg/Bool "data: false"
ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'teleop'"
```

The arm drops torque while `/emergency_stop` is true and leaves torque off after the stop clears until `/arm/enable` is true and the stop is false. The Xbox left-stick click is the teleop e-stop in [`TRAINING_GUIDE.md`](../omnibot-ai-engines/data_engine/TRAINING_GUIDE.md). Keep that controller in hand.

Next trial:

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: false"
ros2 topic pub --once /arm/enable std_msgs/msg/Bool "data: true"
ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'vla'"
ros2 topic pub --once /policy/enable std_msgs/msg/Bool "data: true"
```

## Needs Varun: budget + robot

GPU spend and the 20-trial session both wait on Varun. Before that agreement, the command to run is the dry-run in step 1.
