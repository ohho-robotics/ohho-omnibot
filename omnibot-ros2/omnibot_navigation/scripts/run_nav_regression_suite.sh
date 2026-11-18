#!/usr/bin/env bash
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Run all Proof v0 scenario regression suite entries (headless Gazebo + Nav2).
#
#   scripts/run_nav_regression_suite.sh [--results FILE] [--timeout S]
#
# One fresh sim per scenario. Writes JSONL results and prints a Markdown table.
# Simulation only — do not run on a robot.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_FILE="${SCRIPT_DIR}/../test/test_nav_regression_sim.py"
SCENARIOS_FILE="${SCRIPT_DIR}/../config/nav_regression_scenarios.yaml"
RESULTS="nav-regression-results.jsonl"
RUN_TIMEOUT_S=900

while [[ $# -gt 0 ]]; do
  case "$1" in
    --results) RESULTS="$2"; shift 2 ;;
    --timeout) RUN_TIMEOUT_S="$2"; shift 2 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

RESULTS="$(cd "$(dirname "${RESULTS}")" && pwd)/$(basename "${RESULTS}")"
LOG_DIR="$(dirname "${RESULTS}")/nav-regression-logs"
mkdir -p "${LOG_DIR}"
: > "${RESULTS}"

mapfile -t SCENARIO_IDS < <(
  python3 -c "
import sys
sys.path.insert(0, '${SCRIPT_DIR}')
from regression_scenario_loader import scenario_ids
for sid in scenario_ids('${SCENARIOS_FILE}'):
    print(sid)
"
)

failures=0
for scenario in "${SCENARIO_IDS[@]}"; do
  echo "=== scenario regression: ${scenario} ==="
  log="${LOG_DIR}/${scenario}.log"
  lines_before=$(wc -l < "${RESULTS}")
  OMNIBOT_NAV_REGRESSION_SCENARIO="${scenario}" OMNIBOT_NAV_RESULTS_FILE="${RESULTS}" \
    timeout --signal=INT --kill-after=60 "${RUN_TIMEOUT_S}" \
    launch_test "${TEST_FILE}" 2>&1 | tee "${log}" | grep -E "EVIDENCE|NAV_RESULT|FAIL|Ran [0-9]+ test|OK$"
  status=${PIPESTATUS[0]}
  if [[ ${status} -ne 0 ]]; then
    echo "${scenario}: launch_test exited ${status}"
    failures=$((failures + 1))
    if [[ $(wc -l < "${RESULTS}") -eq ${lines_before} ]]; then
      printf '{"scenario": "%s", "success": false, "reason": "launch_test exited %s before a result"}\n' \
        "${scenario}" "${status}" >> "${RESULTS}"
    fi
  fi
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ros_gz_bridge|parameter_bridge" 2>/dev/null || true
  sleep 3
done

echo
python3 "${SCRIPT_DIR}/summarize_nav_results.py" "${RESULTS}" || { [[ ${failures} -gt 0 ]] || failures=1; }
echo "scenario regression suite: ${failures} failure(s) across ${#SCENARIO_IDS[@]} scenarios"
[[ ${failures} -eq 0 ]]
