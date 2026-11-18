#!/usr/bin/env bash
# ==============================================================================
# OmniBot — ACT Policy Fine-Tuning Runner
#
# Trains Action Chunking Transformer (ACT) on OmniBot 9-DOF mobile-manipulation
# dataset (6 arm joints + 3 base velocities).
#
# Usage:
#   ./lerobot_engine/train_act.sh [DATASET_PATH] [OUTPUT_DIR] [EXTRA_FLAGS...]
#
# Examples:
#   # Run with default config (act.yaml):
#   ./lerobot_engine/train_act.sh /data/lerobot/pick_and_place_cube ~/checkpoints/act_run1
#
#   # Run on Colab / rented GPU with public W&B logging:
#   ./lerobot_engine/train_act.sh /data/lerobot/pick_and_place_cube ~/checkpoints/act_run1 \
#       --wandb-project omnibot-act --wandb-run-name act-colab-v1
#
#   # Run fast CPU smoke test (1-2 steps):
#   ./lerobot_engine/train_act.sh /tmp/dataset /tmp/out --smoke-test
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${SCRIPT_DIR}/configs/models/act.yaml"

DATASET_PATH="${1:-}"
OUTPUT_DIR="${2:-}"

EXTRA_ARGS=()
if [ $# -ge 1 ]; then
    shift
fi
if [ $# -ge 1 ]; then
    shift
fi
EXTRA_ARGS=("$@")

CMD=("python" "${SCRIPT_DIR}/train.py" "--config" "${CONFIG_PATH}")

if [ -n "${DATASET_PATH}" ]; then
    CMD+=("--dataset-path" "${DATASET_PATH}")
fi

if [ -n "${OUTPUT_DIR}" ]; then
    CMD+=("--output-dir" "${OUTPUT_DIR}")
fi

echo "=================================================="
echo "Starting OmniBot ACT training..."
echo "Config: ${CONFIG_PATH}"
if [ -n "${DATASET_PATH}" ]; then
    echo "Dataset: ${DATASET_PATH}"
fi
if [ -n "${OUTPUT_DIR}" ]; then
    echo "Output:  ${OUTPUT_DIR}"
fi
echo "=================================================="

"${CMD[@]}" "${EXTRA_ARGS[@]}"
