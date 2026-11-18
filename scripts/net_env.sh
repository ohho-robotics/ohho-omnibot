#!/usr/bin/env bash
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Source the cell DDS settings and print the static peer list.
#
#   source scripts/net_env.sh
#
# Optional: OMNIBOT_DDS_INTERFACE=<nic> rewrites a runtime copy of
# cyclonedds.xml so Cyclone binds that interface instead of autodetermine.
# The copy is not committed. The checked-in profile stays free of LAN addresses.
#
# Executing this file (instead of sourcing it) prints the variables and
# does not change the caller.

_omnibot_net_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${_omnibot_net_root}/omnibot-ros2/omnibot_bringup/config/ros_domain.env"

if [ -n "${OMNIBOT_DDS_INTERFACE:-}" ]; then
  case "${OMNIBOT_DDS_INTERFACE}" in
    *[!A-Za-z0-9._:-]*)
      echo "OMNIBOT_DDS_INTERFACE must be a NIC name (letters, digits, . _ : -)" >&2
      unset _omnibot_net_root
      return 1 2>/dev/null || exit 1
      ;;
  esac
  _omnibot_dds_src="${CYCLONEDDS_URI#file://}"
  _omnibot_dds_dest="${TMPDIR:-/tmp}/omnibot-cyclonedds.${UID:-0}.xml"
  export OMNIBOT_DDS_INTERFACE
  if ! OMNIBOT_DDS_TEMPLATE="${_omnibot_dds_src}" \
      OMNIBOT_DDS_RENDER="${_omnibot_dds_dest}" \
      python3 - <<'PY'
import os
import sys
from pathlib import Path

src = Path(os.environ["OMNIBOT_DDS_TEMPLATE"])
dest = Path(os.environ["OMNIBOT_DDS_RENDER"])
iface = os.environ["OMNIBOT_DDS_INTERFACE"]
text = src.read_text(encoding="utf-8")
old = '<NetworkInterface autodetermine="true" priority="default" multicast="false"/>'
new = (
    f'<NetworkInterface name="{iface}" autodetermine="false" '
    'priority="default" multicast="false"/>'
)
if old not in text:
    sys.exit("cyclonedds.xml is missing the autodetermine NetworkInterface element")
dest.write_text(text.replace(old, new, 1), encoding="utf-8")
PY
  then
    echo "failed to render the Cyclone interface overlay" >&2
    unset _omnibot_net_root _omnibot_dds_src _omnibot_dds_dest
    return 1 2>/dev/null || exit 1
  fi
  export CYCLONEDDS_URI="file://${_omnibot_dds_dest}"
  unset _omnibot_dds_src _omnibot_dds_dest
fi

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION}"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
  printf 'CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI}"
  printf 'OMNIBOT_PI_IP=%s\n' "${OMNIBOT_PI_IP}"
  printf 'OMNIBOT_WS_IP=%s\n' "${OMNIBOT_WS_IP}"
  printf 'OMNIBOT_WORKSTATION_IP=%s\n' "${OMNIBOT_WORKSTATION_IP}"
  printf 'ROS_STATIC_PEERS=%s\n' "${ROS_STATIC_PEERS}"
else
  echo "omnibot dds: RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} peers=${OMNIBOT_PI_IP},${OMNIBOT_WS_IP} CYCLONEDDS_URI=${CYCLONEDDS_URI}" >&2
fi

unset _omnibot_net_root
