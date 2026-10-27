# lerobot_engine

Direct [LeRobot](https://github.com/huggingface/lerobot)-based training,
recording, and inference for OmniBot's unified 9-DOF mobile-manipulation policy
(6 arm joints + 3 base velocities). **No ROS required** — these are standalone
scripts that run on the GPU workstation or cloud GPU instances. The deployed
inference path is the ROS `policy_node` in `omnibot_lerobot`; this engine produces
the checkpoints it loads.

## Layout

```
lerobot_engine/
├── train.py                 # Fine-tune any registered policy on a LeRobot dataset
├── train_act.py             # Dedicated ACT training runner with default config
├── train_act.sh             # Shell script runner for Linux / Colab / cloud GPU
├── record.py                # Standalone gamepad + webcam dataset recorder (no ROS)
├── infer.py                 # Standalone inference test against a live/dummy camera
├── models/                  # Model registry + adapters
│   ├── registry.py          # register() / make_policy() / list_models()
│   ├── base.py              # PolicyAdapter interface
│   ├── act.py               # ACT policy adapter (OHH-51) + ONNX export
│   ├── smolvla.py           # SmolVLA multimodal VLA adapter
│   ├── diffusion.py         # Diffusion Policy adapter
│   └── openvla.py           # OpenVLA 7B adapter
├── configs/
│   ├── models/              # act.yaml, smolvla.yaml, diffusion.yaml
│   └── robot/               # omnibot_mobile_manip.yaml (9-DOF state/action, cameras)
├── tests/                   # CPU smoke tests and integration tests
│   └── test_smoke_train.py  # 1-2 step training smoke test on synthetic fixture
├── requirements.txt         # Dependencies
└── wandb_sweep_smolvla.yaml # Bayesian sweep for SmolVLA fine-tuning
```

## Clean Environment Setup

From the repository root:

```bash
# 1. Create and activate a fresh virtual environment
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\Activate.ps1

# 2. Install dependencies
pip install --upgrade pip
pip install -r omnibot-ai-engines/lerobot_engine/requirements.txt

# 3. Editable install for data_engine
pip install -e omnibot-ai-engines/learning_engine
```

## Models

The registry (`models/registry.py`) selects the backend by name. Available
adapters: `act` (Action Chunking Transformer, recommended baseline), `smolvla`
(default VLA), `diffusion`, `openvla`.

```bash
python lerobot_engine/train.py --list-models
```

These names match `policy_node`'s `model_type` parameter, so a checkpoint trained
here drops directly into `ros2 launch omnibot_lerobot policy_inference.launch.py
model_type:=<name> checkpoint:=<path>`.

---

## ACT Training (OHH-51)

ACT (Action Chunking Transformer) predicts chunks of future 9-DOF actions (50 steps)
from synchronized wrist camera (640×480), bird's-eye view camera (800×800), and 9-DOF
proprioceptive state. It runs at ~20 Hz inference rate with low VRAM footprint (~4–6 GB).

### 1. Training with Config File (Recommended)

Configuration defaults are maintained in `configs/models/act.yaml`:

```bash
# Using the dedicated shell script:
bash lerobot_engine/train_act.sh /data/lerobot/pick_and_place_cube ~/checkpoints/act_run1

# Or with python:
python lerobot_engine/train_act.py \
    --dataset-path /data/lerobot/pick_and_place_cube \
    --output-dir ~/checkpoints/act_run1
```

Or invoke `train.py` with the `--config` flag:

```bash
python lerobot_engine/train.py \
    --config lerobot_engine/configs/models/act.yaml \
    --dataset-path /data/lerobot/pick_and_place_cube \
    --output-dir ~/checkpoints/act_run1
```

### 2. Fast CPU Smoke Test (1–2 Steps)

To verify the training pipeline in CI or on a CPU machine without downloading pretrained weights:

```bash
python lerobot_engine/train.py \
    --model act \
    --dataset-path /path/to/dataset \
    --output-dir /tmp/smoke_out \
    --smoke-test
```

Or run the test suite:

```bash
pytest omnibot-ai-engines/lerobot_engine/tests -v
```

---

## Cloud GPU & Colab Run Instructions

When training on a rented cloud GPU (RunPod, Lambda Labs, Vast.ai) or Google Colab:

### Google Colab Setup

1. Open a new notebook and select a GPU runtime (`Runtime` → `Change runtime type` → `T4` or `A100`).
2. Run the clone and setup commands:

```bash
!git clone https://github.com/ohho-robotics/OmniBot.git
%cd OmniBot
!pip install -r omnibot-ai-engines/lerobot_engine/requirements.txt
```

3. Download / sync dataset and start training:

```bash
!python omnibot-ai-engines/lerobot_engine/train_act.py \
    --dataset-path /content/dataset/pick_and_place_cube \
    --output-dir /content/checkpoints/act_colab_run1 \
    --wandb-project omnibot-act \
    --wandb-run-name act-colab-v1
```

### Documented Compute Cost Breakdown

ACT is compute-efficient compared to full VLMs. For a typical dataset of 50–100 episodes
(100 epochs, batch size 16):

| Platform | GPU | VRAM | Hourly Rate | Training Time | Estimated Total Cost |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **RunPod (Community)** | NVIDIA RTX 4090 | 24 GB | ~$0.74 / hr | ~2.5 hrs | **~$1.85** |
| **Google Colab Pro** | NVIDIA A100 | 40/80 GB | ~$1.17 / hr (compute units) | ~1.5 hrs | **~$1.75** |
| **Lambda Labs** | NVIDIA A10 | 24 GB | ~$0.60 / hr | ~3.0 hrs | **~$1.80** |
| **RunPod (Secure)** | NVIDIA RTX 3090 | 24 GB | ~$0.44 / hr | ~3.5 hrs | **~$1.54** |

> *Note: In accordance with repository parallel-work rules, no paid GPU instances were started during automated agent runs.*

---

## Public W&B Experiment Tracking

ACT training logs epoch loss, step loss, gradient norms, learning rate schedule, and model artifacts.

To log to the public W&B project:

```bash
wandb login
python lerobot_engine/train_act.py \
    --dataset-path /data/lerobot/pick_and_place_cube \
    --wandb-project omnibot-act \
    --wandb-run-name act-mobile-manip-v1
```

> **TODO (human / Varun):**
> 1. Create public W&B project at `https://wandb.ai/ohho-robotics/omnibot-act`.
> 2. Run training run on GPU and share public dashboard link in the PR/issue report.

---

## Model Export: HF Hub & ONNX

### 1. Export Checkpoint to Hugging Face Hub

Push the final trained checkpoint directly to the Hugging Face Hub:

```bash
python lerobot_engine/train_act.py \
    --dataset-path /data/lerobot/pick_and_place_cube \
    --push-to-hub \
    --hf-repo-id ohho-robotics/omnibot-act-mobile-manip
```

> **TODO (human / Varun):**
> Run `huggingface-cli login` with account write permissions before launching a full upload.

### 2. ONNX Export

Exporting to ONNX produces an optimized static graph for inference deployment on the robot's onboard Orin/PC:

```bash
python lerobot_engine/train.py \
    --config lerobot_engine/configs/models/act.yaml \
    --export-onnx \
    --onnx-output-path ~/checkpoints/act_run1/act_policy.onnx
```

The exported ONNX model accepts `observation_images_wrist` (1×3×240×320), `observation_images_bev` (1×3×240×320),
and `observation_state` (1×9), outputting `predicted_action_chunk` (1×chunk_size×9).

---

## ACT on the VR sim pick dataset (OHH-243)

Config: `configs/act_vr_sim_pick.yaml`. It points at the OHH-237 LeRobot v2
root `/tmp/omnibot-sim-vr-pick` (`local/omnibot-sim-vr-pick`). That directory
is not in git. The Hugging Face dataset `ohho-robotics/omnibot-sim-vr-pick`
is not published. See `docs/act-vr-sim-pick.md` for the human push steps.

CPU smoke (synthetic chunks, no W&B, not LeRobot ACT weights):

```bash
python omnibot-ai-engines/lerobot_engine/train_act_vr.py --smoke
```

Serve that checkpoint through the mux without leaving teleop until the
headset confirms:

```bash
ros2 launch omnibot_lerobot act_vr_sim_pick.launch.py
```

## Record (standalone, no ROS)

```bash
python lerobot_engine/record.py     # RB = start/stop, LB = discard
```

For the full ROS-integrated recording workflow (leader arm + base + ROS topics),
use `ros2 launch omnibot_lerobot teleop_record.launch.py` instead.

## Compare VR and leader-arm datasets

`tools/compare_dataset_quality.py` scores two LeRobot v2 roots and writes a
markdown table plus JSON: action jerk, action smoothness, wrist/BEV timestamp
skew, frame drops, time per episode, and OHH-99 quality-filter keep/reject
counts. It does not record and does not upload. Real VR and leader episodes
are not in this repo. The empty table and the commands to run after recording
are in [`docs/vr-vs-leader-quality.md`](../../docs/vr-vs-leader-quality.md).

```bash
python omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py --help
```

## Inference test (standalone, no ROS)

```bash
python lerobot_engine/infer.py \
    --model act \
    --checkpoint ~/checkpoints/act_run1/best
```

Overlays predicted arm + base actions on live camera or dummy frame.

## See also

- `data_engine/TRAINING_GUIDE.md` — end-to-end recording → dataset → training loop
- `data_engine/` — ROS bag → LeRobot dataset conversion
- `omnibot-ai-ros2/omnibot_lerobot/` — the ROS deployment node (`policy_node`)
