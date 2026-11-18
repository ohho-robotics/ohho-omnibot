#!/usr/bin/env bash
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
#
# OmniBot browser desktop launcher (Linux and macOS). Needs only Docker.
#
#   ./docker/omnibot-desktop.sh                 start, then open the browser
#   ./docker/omnibot-desktop.sh --port 6081     use another host port
#   ./docker/omnibot-desktop.sh --world apartment
#   ./docker/omnibot-desktop.sh --password      require a random VNC password
#   ./docker/omnibot-desktop.sh --build         build the image locally
#   ./docker/omnibot-desktop.sh stop | restart | logs | shell | status | update | down
#
# It wraps `docker compose --profile desktop` on the repository's
# docker-compose.yml, so `docker compose up -d desktop` is equivalent.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/.." && pwd)"
COMPOSE_FILE="${REPO}/docker-compose.yml"
SERVICE=desktop
CONTAINER=omnibot_desktop

say() { printf '%s\n' "$*"; }
die() { printf 'omnibot-desktop: %s\n' "$*" >&2; exit 1; }

usage() { sed -n '5,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

COMMAND=start
BUILD=0
OPEN_BROWSER=1
while [ "$#" -gt 0 ]; do
    case "$1" in
        start|stop|restart|logs|shell|status|update|down) COMMAND="$1" ;;
        --port) [ "$#" -ge 2 ] || die "--port needs a number"; export OMNIBOT_PORT="$2"; shift ;;
        --port=*) export OMNIBOT_PORT="${1#*=}" ;;
        --world) [ "$#" -ge 2 ] || die "--world needs a name"; export OMNIBOT_WORLD="$2"; shift ;;
        --world=*) export OMNIBOT_WORLD="${1#*=}" ;;
        --password) export OMNIBOT_VNC_PASSWORD="${OMNIBOT_VNC_PASSWORD:-$(LC_ALL=C tr -dc 'A-Za-z0-9' </dev/urandom | head -c 8)}" ;;
        --password=*) export OMNIBOT_VNC_PASSWORD="${1#*=}" ;;
        --build) BUILD=1 ;;
        --no-browser) OPEN_BROWSER=0 ;;
        -h|--help|help) usage; exit 0 ;;
        *) die "unknown argument '$1' (see --help)" ;;
    esac
    shift
done

PORT="${OMNIBOT_PORT:-6080}"
case "${PORT}" in ''|*[!0-9]*) die "port must be a number, got '${PORT}'" ;; esac
export OMNIBOT_PORT="${PORT}"

command -v docker >/dev/null 2>&1 || die "Docker is not installed (https://docs.docker.com/get-docker/)."
docker info >/dev/null 2>&1 || die "Docker is not running. Start Docker Desktop (or the Docker service) and try again."
docker compose version >/dev/null 2>&1 || die "Docker Compose v2 is needed (docker compose)."

compose() { docker compose -f "${COMPOSE_FILE}" --profile desktop "$@"; }

url() {
    if [ -n "${OMNIBOT_VNC_PASSWORD:-}" ]; then
        printf 'http://localhost:%s/vnc.html?password=%s' "${PORT}" "${OMNIBOT_VNC_PASSWORD}"
    else
        printf 'http://localhost:%s/' "${PORT}"
    fi
}

open_url() {
    [ "${OPEN_BROWSER}" = 1 ] || return 0
    if command -v xdg-open >/dev/null 2>&1; then xdg-open "$1" >/dev/null 2>&1 || true
    elif command -v open >/dev/null 2>&1; then open "$1" || true
    fi
}

wait_for_desktop() {
    printf 'Waiting for the desktop'
    for _ in $(seq 1 180); do
        if curl --noproxy '*' -fsS -o /dev/null --max-time 3 "http://127.0.0.1:${PORT}/vnc.html" 2>/dev/null; then
            printf ' ready.\n'
            return 0
        fi
        if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null)" = "false" ]; then
            printf '\n'
            docker logs --tail 40 "${CONTAINER}" >&2 || true
            die "the container stopped while starting (see above)."
        fi
        printf '.'
        sleep 1
    done
    printf '\n'
    say "The desktop is taking longer than usual; try $(url) in a moment."
}

start() {
    mkdir -p "${OMNIBOT_SHARED_DIR:-${REPO}/omnibot-shared}"
    if [ "${BUILD}" = 1 ]; then
        compose up -d --build "${SERVICE}"
    else
        compose up -d "${SERVICE}"
    fi
    wait_for_desktop
    say ""
    say "OmniBot desktop:  $(url)"
    say "Gazebo and RViz open on the desktop (first start can take a minute)."
    say "Stop it with:     ${BASH_SOURCE[0]} stop"
    open_url "$(url)"
}

case "${COMMAND}" in
    start) start ;;
    stop) compose stop "${SERVICE}" ;;
    restart) compose stop "${SERVICE}"; start ;;
    logs) compose logs -f --tail 200 "${SERVICE}" ;;
    shell)
        if [ -t 0 ]; then docker exec -it "${CONTAINER}" bash -l
        else docker exec -i "${CONTAINER}" bash -l; fi ;;
    status)
        docker ps -a --filter "name=^/${CONTAINER}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
        docker exec "${CONTAINER}" omnibot-desktop-sim status 2>/dev/null | sed 's/^/sim: /' || true ;;
    update) compose pull "${SERVICE}"; start ;;
    down) compose rm -sf "${SERVICE}"; say "Removed the container (the omnibot-desktop-home volume is kept)." ;;
esac
