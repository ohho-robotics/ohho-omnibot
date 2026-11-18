#!/usr/bin/env bash
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Two containers on a Docker bridge. Multicast discovery is disabled in
# the DDS configuration for both cases, so a default multicast search
# cannot connect them.
#
#   negative: AllowMulticast false, no peers. /chatter must not arrive.
#   positive: the checked-in cyclonedds.xml, peers set to the two
#             container addresses. /chatter must arrive.
#
# The official ros:jazzy-ros-base-noble image is pulled, then
# rmw_cyclonedds_cpp and demo_nodes_cpp are installed once.
# Not for a robot. Simulation / CI only.
set -euo pipefail

command -v docker >/dev/null
command -v timeout >/dev/null

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${DDS_CI_IMAGE:-ros:jazzy-ros-base-noble}"
RUN_ID="${GITHUB_RUN_ID:-$$}"
NET="omnibot-dds-${RUN_ID}"
PREFIX="omnibot-dds-${RUN_ID}"
LOCAL_IMAGE="omnibot-dds-ci:${RUN_ID}"
XML="${ROOT}/omnibot-ros2/omnibot_bringup/config/dds/cyclonedds.xml"
POS_TIMEOUT="${DDS_POSITIVE_TIMEOUT:-30}"
NEG_TIMEOUT="${DDS_NEGATIVE_TIMEOUT:-15}"
DOMAIN_ID="${DDS_DOMAIN_ID:-42}"
HEAR_LOG="$(mktemp)"

if [ ! -f "${XML}" ]; then
  echo "missing ${XML}" >&2
  exit 1
fi

cleanup() {
  docker rm -f "${PREFIX}-a" "${PREFIX}-b" "${PREFIX}-prep" >/dev/null 2>&1 || true
  docker network rm "${NET}" >/dev/null 2>&1 || true
  docker rmi "${LOCAL_IMAGE}" >/dev/null 2>&1 || true
  rm -f "${HEAR_LOG}"
}
trap cleanup EXIT

echo "pull ${IMAGE}"
docker pull "${IMAGE}"
docker network create "${NET}"

echo "install rmw_cyclonedds_cpp and demo_nodes_cpp"
docker run --name "${PREFIX}-prep" --network "${NET}" "${IMAGE}" \
  bash -c 'set -euo pipefail
    apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-demo-nodes-cpp
  '
docker commit "${PREFIX}-prep" "${LOCAL_IMAGE}" >/dev/null
docker rm -f "${PREFIX}-prep" >/dev/null

start_sleeper() {
  docker run -d \
    --name "$1" \
    --network "${NET}" \
    --hostname "$1" \
    -v "${XML}:/dds/cyclonedds.xml:ro" \
    "${LOCAL_IMAGE}" \
    sleep infinity >/dev/null
}

start_sleeper "${PREFIX}-a"
start_sleeper "${PREFIX}-b"

IP_A="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${PREFIX}-a")"
IP_B="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${PREFIX}-b")"
if [ -z "${IP_A}" ] || [ -z "${IP_B}" ] || [ "${IP_A}" = "${IP_B}" ]; then
  echo "container addresses missing: a=${IP_A:-} b=${IP_B:-}" >&2
  exit 1
fi
echo "containers ${PREFIX}-a=${IP_A} ${PREFIX}-b=${IP_B}"

docker exec "${PREFIX}-a" grep -q 'AllowMulticast>false' /dds/cyclonedds.xml
docker exec "${PREFIX}-a" bash -c \
  'find /opt/ros/jazzy -name librmw_cyclonedds_cpp.so | grep -q .'

stop_dds() {
  docker exec "${PREFIX}-a" bash -c 'pkill -x talker || true' >/dev/null 2>&1 || true
  docker exec "${PREFIX}-b" bash -c 'pkill -f "[t]opic echo" || true' >/dev/null 2>&1 || true
  sleep 1
}

# hear_chatter TIMEOUT docker-exec-env-args...
# Prints echo output to HEAR_LOG. Returns 0 if a Hello sample arrived,
# 2 if the talker did not stay up, 1 if the wait ended with no sample.
hear_chatter() {
  timeout_s="$1"
  shift
  stop_dds
  docker exec -d "$@" "${PREFIX}-a" \
    bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp talker > /tmp/talker.log 2>&1'
  sleep 3
  if ! docker exec "${PREFIX}-a" pgrep -x talker >/dev/null; then
    echo "talker is not running" >&2
    docker exec "${PREFIX}-a" cat /tmp/talker.log >&2 || true
    return 2
  fi
  if docker exec "${PREFIX}-a" \
      bash -c 'grep -E -i -q "failed to parse|could not load library|implementation identifier does not match" /tmp/talker.log'; then
    echo "talker reported an RMW or XML error" >&2
    docker exec "${PREFIX}-a" cat /tmp/talker.log >&2 || true
    return 2
  fi
  hear_rc=0
  timeout "${timeout_s}" docker exec "$@" "${PREFIX}-b" \
    bash -c 'source /opt/ros/jazzy/setup.bash
      echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
      test "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp"
      ros2 topic echo /chatter --once' >"${HEAR_LOG}" 2>&1 || hear_rc=$?
  if [ "${hear_rc}" -eq 0 ] && grep -q "Hello" "${HEAR_LOG}"; then
    return 0
  fi
  if [ "${hear_rc}" -eq 0 ]; then
    return 1
  fi
  return "${hear_rc}"
}

# Not the checked-in profile: multicast off, no peers. Written in the
# containers so the negative case does not depend on shell quoting.
write_negative_profile() {
  docker exec -i "$1" tee /tmp/negative.xml >/dev/null <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
}

write_negative_profile "${PREFIX}-a"
write_negative_profile "${PREFIX}-b"

echo "negative: multicast disabled, no static peers"
set +e
hear_chatter "${NEG_TIMEOUT}" \
  -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
  -e "ROS_DOMAIN_ID=${DOMAIN_ID}" \
  -e "CYCLONEDDS_URI=file:///tmp/negative.xml"
neg_rc=$?
set -e
if [ "${neg_rc}" -eq 0 ] || grep -q "Hello" "${HEAR_LOG}"; then
  echo "FAIL: /chatter crossed the network without the static-peer profile" >&2
  cat "${HEAR_LOG}" >&2 || true
  exit 1
fi
if [ "${neg_rc}" -eq 2 ]; then
  echo "FAIL: negative case did not start a healthy talker" >&2
  exit 1
fi
# 124: echo waited out the window. 1: the CLI reported the topic is not published.
if [ "${neg_rc}" -eq 124 ]; then
  :
elif [ "${neg_rc}" -eq 1 ] && grep -E -q "does not appear to be published yet|No discovery mechanism is available" "${HEAR_LOG}"; then
  :
else
  echo "FAIL: negative case ended with unexpected exit ${neg_rc}" >&2
  cat "${HEAR_LOG}" >&2 || true
  docker exec "${PREFIX}-a" cat /tmp/talker.log >&2 || true
  exit 1
fi
if ! docker exec "${PREFIX}-a" pgrep -x talker >/dev/null; then
  echo "FAIL: talker exited during the negative case (discovery was not what failed)" >&2
  docker exec "${PREFIX}-a" cat /tmp/talker.log >&2 || true
  exit 1
fi
echo "negative case: no discovery (exit ${neg_rc})"

echo "positive: checked-in static-peer profile"
set +e
hear_chatter "${POS_TIMEOUT}" \
  -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
  -e "ROS_DOMAIN_ID=${DOMAIN_ID}" \
  -e "CYCLONEDDS_URI=file:///dds/cyclonedds.xml" \
  -e "OMNIBOT_PI_IP=${IP_A}" \
  -e "OMNIBOT_WS_IP=${IP_B}" \
  -e "OMNIBOT_WORKSTATION_IP=${IP_B}"
pos_rc=$?
set -e
if [ "${pos_rc}" -ne 0 ]; then
  echo "FAIL: static-peer profile did not deliver /chatter (exit ${pos_rc})" >&2
  cat "${HEAR_LOG}" >&2 || true
  docker exec "${PREFIX}-a" cat /tmp/talker.log >&2 || true
  exit 1
fi
echo "positive case: /chatter received"
grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" "${HEAR_LOG}"
echo "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
cat "${HEAR_LOG}"
