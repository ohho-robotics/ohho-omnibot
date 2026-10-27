#!/usr/bin/env bash
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Headless Gazebo smoke for the scripted virtual headset. Run inside the
# navdev image (omnibot-nav-dev), not on a robot and not on a Quest.
#
#   docker run --rm \
#     -e ROS_DOMAIN_ID=88 -e ROS_LOCALHOST_ONLY=1 \
#     -e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe \
#     -e QT_QPA_PLATFORM=offscreen -e MESA_GL_VERSION_OVERRIDE=3.3 \
#     -v "$PWD":/work omnibot-nav-dev:latest \
#     bash /work/scripts/ohh237_scripted_vr_pick_smoke.sh
#
# Writes a reject report under datasets/omnibot-sim-vr-pick/ when episodes
# are actually recorded. Does not push to the Hugging Face Hub.
set -uo pipefail

domain="${ROS_DOMAIN_ID:-}"
if [[ "${ROS_LOCALHOST_ONLY:-}" != "1" || -z "${domain}" ]]; then
  echo "Refusing to publish teleop topics. Set ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID to a value other than 30." >&2
  exit 2
fi
# Decimal only. ROS Jazzy parses leading zeros as octal, so 036 is domain 30.
if [[ ! "${domain}" =~ ^[0-9]+$ ]]; then
  echo "ROS_DOMAIN_ID=${domain} is outside the FastDDS range 0-232. The multicast port overflows above 232, so 237 cannot be used." >&2
  exit 2
fi
domain_id=$((10#${domain}))
if [[ "${domain}" != "${domain_id}" ]]; then
  echo "Refusing to publish teleop topics. ROS_DOMAIN_ID=${domain} is not canonical; leading zeros are octal in ROS (036 is domain 30)." >&2
  exit 2
fi
if [[ "${domain_id}" -eq 30 ]]; then
  echo "Refusing to publish teleop topics. Set ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID to a value other than 30." >&2
  exit 2
fi
if [[ "${domain_id}" -gt 232 ]]; then
  echo "ROS_DOMAIN_ID=${domain} is outside the FastDDS range 0-232. The multicast port overflows above 232, so 237 cannot be used." >&2
  exit 2
fi

export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export OHHO_SAFETY_DIR="${OHHO_SAFETY_DIR:-/tmp/ohh237-safety}"

REPO="${OMNIBOT_REPO:-/work}"
WS=/tmp/ohh237-ws
OUT=/tmp/omnibot-sim-vr-pick
REPORT_DIR="${REPO}/datasets/omnibot-sim-vr-pick"
EPISODES="${OHH237_EPISODES:-3}"
SEED="${OHH237_SEED:-237}"

set +u
# ROS setup scripts read optional variables.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "${WS}" "${OHHO_SAFETY_DIR}"
cd "${WS}"

echo "[ohh-237] colcon build"
# Copy into the container install space. --symlink-install leaves ament_python
# console scripts without a libexec directory on this image.
if ! colcon build \
  --base-paths "${REPO}/omnibot-ros2" "${REPO}/omnibot-ai-ros2" \
  --packages-select omnibot_description omnibot_arm omnibot_bringup \
    omnibot_hybrid omnibot_rl omnibot_lerobot \
  2>&1 | tee /tmp/ohh237-build.log; then
  echo "[ohh-237] colcon build failed" >&2
  exit 1
fi
# Shebangs copied from a Windows checkout contain CR. Linux exec then looks
# for python3\r. The install space is inside the container, so this does not
# rewrite the git checkout.
find "${WS}/install" -type f -name '*.py' -exec sed -i 's/\r$//' {} +
set +u
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -euo pipefail

NUMPY_BEFORE="$(python3 -c 'import numpy; print(numpy.__version__)')"
echo "[ohh-237] numpy ${NUMPY_BEFORE}"
if ! python3 -c 'import pyarrow' >/dev/null 2>&1; then
  if ! python3 -m pip --version >/dev/null 2>&1; then
    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y python3-pip
  fi
  python3 -m pip install --break-system-packages 'pyarrow==18.1.0'
fi
NUMPY_AFTER="$(python3 -c 'import numpy; print(numpy.__version__)')"
if [[ "${NUMPY_BEFORE}" != "${NUMPY_AFTER}" ]]; then
  echo "[ohh-237] numpy changed ${NUMPY_BEFORE} -> ${NUMPY_AFTER}; refusing to continue" >&2
  exit 1
fi

rm -rf "${OUT}"
mkdir -p "${OUT}"
# The open-loop script is 12.7s. Allow 20s per episode plus 360s of
# bring-up, so the default 3-episode run keeps a 420s budget and larger
# counts are not cut off. Override with OHH237_TIMEOUT (seconds).
TIMEOUT="${OHH237_TIMEOUT:-$((360 + EPISODES * 20))}"
echo "[ohh-237] launch episodes=${EPISODES} seed=${SEED} timeout=${TIMEOUT}"
set +e
timeout --signal=INT --kill-after=30 "${TIMEOUT}" \
  ros2 launch omnibot_lerobot scripted_vr_pick.launch.py \
    world:=apartment \
    pick_place:=true \
    pick_place_seed:="${SEED}" \
    episodes:="${EPISODES}" \
    output_dir:="${OUT}" \
  2>&1 | tee /tmp/ohh237-launch.log
launch_status=${PIPESTATUS[0]}
set -e
pkill -f "gz sim" 2>/dev/null || true
pkill -f "scripted_vr_headset" 2>/dev/null || true
echo "[ohh-237] launch status ${launch_status}"

if [[ ! -f "${OUT}/meta/episodes.jsonl" ]]; then
  echo "[ohh-237] no episodes were recorded" >&2
  exit 1
fi
recorded=$(grep -cve '^[[:space:]]*$' "${OUT}/meta/episodes.jsonl" || true)
if [[ "${recorded}" -ne "${EPISODES}" ]]; then
  echo "[ohh-237] recorded ${recorded} episodes, expected ${EPISODES}" >&2
  exit 1
fi
if [[ ! -f "${OUT}/meta/collection.json" ]]; then
  echo "[ohh-237] collection sidecar missing; headset did not finish" >&2
  exit 1
fi

mkdir -p "${REPORT_DIR}"
if [[ -f "${OUT}/meta/collection.json" ]]; then
  cp "${OUT}/meta/collection.json" "${REPORT_DIR}/collection.json"
fi
PYTHONPATH="${REPO}/omnibot-ai-engines:${REPO}/omnibot-ai-ros2/omnibot_lerobot${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m omnibot_lerobot.scripted_vr_pick \
    --dataset "${OUT}" \
    --report "${REPORT_DIR}/reject_report.md" \
    --json "${REPORT_DIR}/reject_report.json"
echo "[ohh-237] wrote ${REPORT_DIR}/reject_report.md"
exit "${launch_status}"
