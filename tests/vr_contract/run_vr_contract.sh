#!/usr/bin/env bash
# Headless Gazebo VR contract. No robot and no Quest.
# Starts sim_vr.launch.py, records an MCAP, and runs the scripted headset.
set -euo pipefail

OUT="${1:-vr-contract-out}"
mkdir -p "$OUT"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

# Jazzy setup.bash reads AMENT_TRACE_SETUP_FILES, which is unset. nounset
# aborts this script before Gazebo starts (OHH-21).
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

# symlink-install executes this file directly; the bit must be set in CI.
chmod +x omnibot-ros2/omnibot_arm/scripts/arm_joint_state_mirror.py || true

LAUNCH_PID=""
BAG_PID=""
MIRROR_PID=""

write_summary() {
  if [[ -n "${GITHUB_STEP_SUMMARY:-}" && -f "$OUT/report.json" ]]; then
    {
      echo "### VR contract"
      echo
      echo '```json'
      cat "$OUT/report.json"
      echo '```'
    } >> "$GITHUB_STEP_SUMMARY"
  fi
}

cleanup() {
  status=$?
  set +e
  write_summary
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 1
    done
  fi
  if [[ -n "$MIRROR_PID" ]] && kill -0 "$MIRROR_PID" 2>/dev/null; then
    kill -INT "$MIRROR_PID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 2
    kill -TERM "$LAUNCH_PID" 2>/dev/null
  fi
  pkill -TERM -f "gz sim" 2>/dev/null || true
  pkill -TERM -f "parameter_bridge" 2>/dev/null || true
  pkill -TERM -f "rosbridge_websocket" 2>/dev/null || true
  pkill -TERM -f "arm_joint_state_mirror" 2>/dev/null || true
  if [[ -n "$LAUNCH_PID" ]]; then
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT

# The gate and the /cmd_vel/out bridge moved to sim_teleop.launch.py.
# sim_vr.launch.py is the camera profile and does not start teleop_safety.
# arm:=true still starts arm_sim_driver inside sim.launch.py. The mirror
# node is the contract's /arm/joint_states source from the mux output.
ros2 launch omnibot_bringup sim_teleop.launch.py \
  world:=flat \
  gui:=false \
  drive:=planar \
  arm:=true \
  video:=false \
  vr:=false \
  rosbridge:=true \
  rosbridge_address:=127.0.0.1 \
  >"$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

python3 - "$OUT/launch.log" "$LAUNCH_PID" << 'PY'
import os
import socket
import sys
import time

log_path, pid = sys.argv[1], int(sys.argv[2])

def launch_alive(launch_pid):
    try:
        os.kill(launch_pid, 0)
    except OSError:
        return False
    return True

deadline = time.monotonic() + 120
while time.monotonic() < deadline:
    if not launch_alive(pid):
        print("sim_teleop launch exited before rosbridge opened", file=sys.stderr)
        break
    try:
        sock = socket.create_connection(("127.0.0.1", 9090), 2)
    except OSError:
        sock = None
    if sock is not None:
        sock.close()
        sys.exit(0)
    time.sleep(1)
print("rosbridge did not open on 127.0.0.1:9090", file=sys.stderr)
try:
    text = open(log_path, encoding="utf-8", errors="replace").read().splitlines()
except OSError as exc:
    print(exc, file=sys.stderr)
else:
    print("\n".join(text[-80:]), file=sys.stderr)
sys.exit(1)
PY

ros2 run omnibot_arm arm_joint_state_mirror.py >"$OUT/mirror.log" 2>&1 &
MIRROR_PID=$!

ros2 bag record -s mcap -o "$OUT/vr_contract" \
  /odom \
  /arm/joint_states \
  /ohho/safety/state \
  /cmd_vel/out \
  /cmd_vel/teleop \
  /cmd_vel/teleop_raw \
  /emergency_stop \
  /ohho/heartbeat \
  /control_mode \
  /control_mode/active \
  /arm/joint_commands/teleop \
  /arm/joint_commands/teleop_raw \
  /arm/joint_commands/out \
  >"$OUT/bag.log" 2>&1 &
BAG_PID=$!
sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "rosbag mcap record exited early" >&2
  tail -n 80 "$OUT/bag.log" >&2 || true
  exit 1
fi

set +e
python3 tests/vr_contract/virtual_headset.py \
  --url ws://127.0.0.1:9090 \
  --report "$OUT/report.json" \
  2>&1 | tee "$OUT/headset.log"
status=${PIPESTATUS[0]}
set -e
exit "$status"
