#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Scripted Quest stand-in for the Gazebo VR contract.

Connects to rosbridge and publishes the same topics OhhO-VR publishes.
No headset and no robot. The Gazebo job launches sim_teleop.launch.py
(the safety gate and the /cmd_vel/out bridge) and runs this file.
Driving goes through /cmd_vel/teleop_raw, so the base moves only when
the mux output /cmd_vel/out is what Gazebo consumes.
teleop_safety starts latched, so this client calls /ohho/estop_reset
before the first command.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path


FORWARD_M = 1.0
FORWARD_MIN_M = 0.95
FORWARD_MAX_M = 1.40
STRAFE_M = 0.30
STRAFE_MIN_M = 0.25
STRAFE_MAX_M = 0.70
DRIVE_SPEED = 0.2
JOINTS = ("arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex")
JOINT_TARGET = 0.15
JOINT_MIN_DELTA = 0.10
HELD_JOINT_TARGET = 0.45
TICK_S = 0.05
FORWARD_TIMEOUT_S = 120.0
STRAFE_TIMEOUT_S = 60.0
STOP_TIMEOUT_S = 8.0
ARM_TIMEOUT_S = 8.0
READY_TIMEOUT_S = 60.0
ESTOP_TIMEOUT_S = 6.0
LINK_LOSS_TIMEOUT_S = 4.0

TELEOP_RAW = "/cmd_vel/teleop_raw"
JOINT_RAW = "/arm/joint_commands/teleop_raw"
HEARTBEAT = "/ohho/heartbeat"
ESTOP = "/emergency_stop"
MODE = "/control_mode"
ODOM = "/odom"
JOINT_STATES = "/arm/joint_states"
SAFETY = "/ohho/safety/state"
CMD_OUT = "/cmd_vel/out"
MODE_ACTIVE = "/control_mode/active"
RESET = "/ohho/estop_reset"

TWIST = "geometry_msgs/msg/Twist"
JOINT_STATE = "sensor_msgs/msg/JointState"
EMPTY = "std_msgs/msg/Empty"
BOOL = "std_msgs/msg/Bool"
STRING = "std_msgs/msg/String"
ODOM_TYPE = "nav_msgs/msg/Odometry"
TRIGGER = "std_srvs/srv/Trigger"


class ContractError(Exception):
    """The VR topic contract did not hold."""

    def __init__(self, message):
        super().__init__(message)
        self.evidence = {}


def log(text):
    print(text, flush=True)


def twist_msg(vx, vy, wz=0.0):
    return {
        "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
    }


def joint_msg(names, positions):
    return {
        "name": list(names),
        "position": [float(position) for position in positions],
    }


def _num(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(number):
        return 0.0
    return number


def pose_of(msg):
    position = ((msg or {}).get("pose") or {}).get("pose") or {}
    position = position.get("position") or {}
    return _num(position.get("x")), _num(position.get("y"))


def speed_of(msg):
    linear = ((msg or {}).get("twist") or {}).get("twist") or {}
    linear = linear.get("linear") or {}
    return math.hypot(_num(linear.get("x")), _num(linear.get("y")))


def cmd_speed(msg):
    linear = (msg or {}).get("linear") or {}
    return math.hypot(_num(linear.get("x")), _num(linear.get("y")))


def joint_positions(msg):
    if not msg:
        return {}
    names = msg.get("name") or []
    positions = msg.get("position") or []
    found = {}
    for name, position in zip(names, positions):
        try:
            found[str(name)] = float(position)
        except (TypeError, ValueError):
            continue
    return found


def safety_of(msg):
    if not msg:
        return None
    raw = msg.get("data")
    if isinstance(raw, dict):
        payload = raw
    elif isinstance(raw, str) and raw:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return None
    else:
        return None
    if not isinstance(payload, dict):
        return None
    return {
        "armed": bool(payload.get("armed")),
        "latched": bool(payload.get("latched")),
        "reason": str(payload.get("reason") or ""),
    }


def service_ok(response):
    values = (response or {}).get("values") or {}
    if "success" in values:
        return bool(values.get("success")), str(values.get("message") or "")
    return bool((response or {}).get("result")), ""


def ros_topics_to_gz_cmd_vel(text):
    """ROS topics bridged ROS_TO_GZ onto Gazebo /cmd_vel."""
    found = []
    for chunk in text.split("\n- "):
        if 'gz_topic_name: "/cmd_vel"' not in chunk:
            continue
        if "ROS_TO_GZ" not in chunk:
            continue
        for line in chunk.splitlines():
            stripped = line.strip()
            if stripped.startswith("ros_topic_name:"):
                value = stripped.split(":", 1)[1].strip().strip('"').strip("'")
                found.append(value)
    return found


class RosbridgeClient:
    """Minimal rosbridge v2 client. websockets is imported on connect."""

    def __init__(self, url):
        self.url = url
        self._cm = None
        self._ws = None
        self._reader = None
        self._latest = {}
        self._pending = {}
        self._ids = 0

    def clock(self):
        return time.monotonic()

    async def sleep(self, seconds):
        await asyncio.sleep(seconds)

    def latest(self, topic):
        return self._latest.get(topic)

    async def connect(self):
        try:
            import websockets
        except ImportError as exc:
            raise ContractError(
                "websockets is not installed; pip install 'websockets>=13'"
            ) from exc
        deadline = time.monotonic() + 30.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                self._cm = websockets.connect(
                    self.url, open_timeout=5, max_size=1_000_000
                )
                self._ws = await self._cm.__aenter__()
                break
            except Exception as exc:  # noqa: BLE001 — retry until rosbridge is up
                last_error = exc
                self._cm = None
                self._ws = None
                await asyncio.sleep(0.5)
        else:
            raise ContractError(f"rosbridge connect failed: {last_error}")
        self._reader = asyncio.create_task(self._read())

    async def close(self):
        if self._reader is not None:
            self._reader.cancel()
            try:
                await self._reader
            except asyncio.CancelledError:
                pass
            self._reader = None
        if self._cm is not None:
            await self._cm.__aexit__(None, None, None)
            self._cm = None
            self._ws = None

    async def _read(self):
        try:
            async for raw in self._ws:
                if isinstance(raw, bytes):
                    raw = raw.decode("utf-8", "replace")
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                op = msg.get("op")
                if op == "publish":
                    topic = msg.get("topic")
                    body = msg.get("msg") or {}
                    if isinstance(body, str):
                        try:
                            body = json.loads(body)
                        except json.JSONDecodeError:
                            body = {}
                    if topic:
                        self._latest[topic] = body
                elif op == "service_response":
                    pending = self._pending.get(msg.get("id"))
                    if pending is not None and not pending.done():
                        pending.set_result(msg)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 — surface a dead socket on the next call
            log(f"rosbridge read ended: {exc}")

    async def _send(self, payload):
        if self._ws is None:
            raise ContractError("rosbridge is not connected")
        await self._ws.send(json.dumps(payload))

    async def advertise(self, topic, type_name):
        await self._send({"op": "advertise", "topic": topic, "type": type_name})

    async def subscribe(self, topic, type_name):
        await self._send({"op": "subscribe", "topic": topic, "type": type_name})

    async def publish(self, topic, msg):
        await self._send({"op": "publish", "topic": topic, "msg": msg})

    async def call_service(self, service, type_name, args=None, timeout=5.0):
        self._ids += 1
        ident = f"call-{self._ids}"
        loop = asyncio.get_running_loop()
        pending = loop.create_future()
        self._pending[ident] = pending
        await self._send(
            {
                "op": "call_service",
                "id": ident,
                "service": service,
                "type": type_name,
                "args": args or {},
            }
        )
        try:
            return await asyncio.wait_for(pending, timeout)
        except asyncio.TimeoutError as exc:
            raise ContractError(f"{service} timed out") from exc
        finally:
            self._pending.pop(ident, None)


async def _beat(link):
    await link.publish(HEARTBEAT, {})


async def _twist(link, vx, vy):
    await link.publish(TELEOP_RAW, twist_msg(vx, vy, 0.0))


async def _joints(link, target):
    await link.publish(JOINT_RAW, joint_msg(JOINTS, [target, target, target]))


def _require_joints(positions, label):
    missing = [name for name in JOINTS if name not in positions]
    if missing:
        raise ContractError(f"{label} missing joints {missing}")


async def _stop_base(link):
    deadline = link.clock() + STOP_TIMEOUT_S
    while link.clock() < deadline:
        await _beat(link)
        await _twist(link, 0.0, 0.0)
        await link.sleep(TICK_S)
        if speed_of(link.latest(ODOM)) < 0.05:
            return
    raise ContractError(
        f"base did not stop (speed {speed_of(link.latest(ODOM)):.3f} m/s)"
    )


async def _run(link, evidence):
    await link.advertise(TELEOP_RAW, TWIST)
    await link.advertise(JOINT_RAW, JOINT_STATE)
    await link.advertise(HEARTBEAT, EMPTY)
    await link.advertise(ESTOP, BOOL)
    await link.advertise(MODE, STRING)
    for topic, type_name in (
        (ODOM, ODOM_TYPE),
        (JOINT_STATES, JOINT_STATE),
        (SAFETY, STRING),
        (CMD_OUT, TWIST),
        (MODE_ACTIVE, STRING),
    ):
        await link.subscribe(topic, type_name)

    # Match scripted_vr_headset: the gate boots latched (reason startup)
    # and a heartbeat does not arm it. Clear that latch before driving.
    startup_reset = None
    startup_error = "not called"
    ready_deadline = link.clock() + READY_TIMEOUT_S
    while link.clock() < ready_deadline:
        await _beat(link)
        await link.publish(MODE, {"data": "teleop"})
        if startup_reset is None:
            try:
                response = await link.call_service(RESET, TRIGGER, {})
            except ContractError as exc:
                startup_error = str(exc)
                await link.sleep(TICK_S)
                continue
            ok, message = service_ok(response)
            if not ok:
                startup_error = message or "refused"
                await link.sleep(TICK_S)
                continue
            startup_reset = message
        await link.sleep(TICK_S)
        safety = safety_of(link.latest(SAFETY))
        joints = joint_positions(link.latest(JOINT_STATES))
        mode = (link.latest(MODE_ACTIVE) or {}).get("data")
        if (
            link.latest(ODOM) is not None
            and all(name in joints for name in JOINTS)
            and safety is not None
            and safety["armed"]
            and not safety["latched"]
            and mode == "teleop"
        ):
            break
    else:
        safety = safety_of(link.latest(SAFETY))
        evidence["startup_reset"] = startup_reset
        raise ContractError(
            "VR graph was not ready "
            f"odom={link.latest(ODOM) is not None} "
            f"joints={list(joint_positions(link.latest(JOINT_STATES)))} "
            f"safety={safety} mode={(link.latest(MODE_ACTIVE) or {}).get('data')} "
            f"startup_reset={startup_reset!r} ({startup_error})"
        )
    evidence["startup_reset"] = startup_reset
    log(
        "EVIDENCE vr_contract ready "
        f"safety={safety_of(link.latest(SAFETY))} startup_reset={startup_reset}"
    )

    start = pose_of(link.latest(ODOM))
    peak = 0.0
    forward_deadline = link.clock() + FORWARD_TIMEOUT_S
    while link.clock() < forward_deadline:
        await _beat(link)
        await _twist(link, DRIVE_SPEED, 0.0)
        await link.sleep(TICK_S)
        peak = max(peak, cmd_speed(link.latest(CMD_OUT)))
        dx = pose_of(link.latest(ODOM))[0] - start[0]
        if dx >= FORWARD_M:
            break
    await _stop_base(link)
    end = pose_of(link.latest(ODOM))
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    evidence["forward_m"] = dx
    evidence["forward_lateral_m"] = dy
    evidence["cmd_vel_out_peak"] = peak
    log(
        "EVIDENCE vr_contract forward "
        f"dx={dx:.3f} dy={dy:.3f} cmd_vel_out_peak={peak:.3f}"
    )
    if peak < 0.05:
        raise ContractError(
            "cmd_vel mux did not publish /cmd_vel/out while the headset drove"
        )
    if dx < FORWARD_MIN_M or dx > FORWARD_MAX_M or abs(dy) > 0.25:
        raise ContractError(
            f"forward travel {dx:.3f} m (lateral {dy:.3f} m); "
            "Gazebo must follow mux output /cmd_vel/out"
        )

    strafe_start = pose_of(link.latest(ODOM))
    strafe_deadline = link.clock() + STRAFE_TIMEOUT_S
    while link.clock() < strafe_deadline:
        await _beat(link)
        await _twist(link, 0.0, DRIVE_SPEED)
        await link.sleep(TICK_S)
        dy = pose_of(link.latest(ODOM))[1] - strafe_start[1]
        if abs(dy) >= STRAFE_M:
            break
    await _stop_base(link)
    strafe_end = pose_of(link.latest(ODOM))
    strafe_dx = strafe_end[0] - strafe_start[0]
    strafe_dy = strafe_end[1] - strafe_start[1]
    evidence["strafe_m"] = strafe_dy
    evidence["strafe_forward_m"] = strafe_dx
    log(f"EVIDENCE vr_contract strafe dy={strafe_dy:.3f} dx={strafe_dx:.3f}")
    if abs(strafe_dy) < STRAFE_MIN_M or abs(strafe_dy) > STRAFE_MAX_M or abs(strafe_dx) > 0.25:
        raise ContractError(
            f"strafe travel {strafe_dy:.3f} m (forward drift {strafe_dx:.3f} m)"
        )

    before = joint_positions(link.latest(JOINT_STATES))
    _require_joints(before, "pre-move joint state")
    arm_deadline = link.clock() + ARM_TIMEOUT_S
    after = before
    while link.clock() < arm_deadline:
        await _beat(link)
        await _twist(link, 0.0, 0.0)
        await _joints(link, JOINT_TARGET)
        await link.sleep(TICK_S)
        after = joint_positions(link.latest(JOINT_STATES))
        if all(name in after and after[name] - before[name] >= JOINT_MIN_DELTA for name in JOINTS):
            break
    deltas = {
        name: after.get(name, 0.0) - before.get(name, 0.0) for name in JOINTS
    }
    evidence["joint_delta_rad"] = deltas
    log(f"EVIDENCE vr_contract joints delta={deltas}")
    for name, delta in deltas.items():
        if delta < JOINT_MIN_DELTA or delta > JOINT_TARGET + 0.05:
            raise ContractError(
                f"{name} moved {delta:.3f} rad; expected about {JOINT_TARGET} rad "
                "through /arm/joint_commands/out"
            )

    held_pose = pose_of(link.latest(ODOM))
    held_joints = dict(after)
    estop_deadline = link.clock() + ESTOP_TIMEOUT_S
    latched = None
    while link.clock() < estop_deadline:
        await link.publish(ESTOP, {"data": True})
        await _beat(link)
        await _twist(link, DRIVE_SPEED, 0.0)
        await _joints(link, HELD_JOINT_TARGET)
        await link.sleep(TICK_S)
        latched = safety_of(link.latest(SAFETY))
        if (
            latched is not None
            and latched["latched"]
            and not latched["armed"]
            and latched["reason"] == "estop"
            and speed_of(link.latest(ODOM)) < 0.05
        ):
            break
    evidence["estop"] = latched
    log(f"EVIDENCE vr_contract estop safety={latched}")
    if (
        latched is None
        or not latched["latched"]
        or latched["armed"]
        or latched["reason"] != "estop"
    ):
        raise ContractError(f"e-stop did not latch /ohho/safety/state ({latched})")
    if speed_of(link.latest(ODOM)) >= 0.05:
        raise ContractError("base still moving while e-stop is latched")
    drifted = pose_of(link.latest(ODOM))
    if abs(drifted[0] - held_pose[0]) > 0.15 or abs(drifted[1] - held_pose[1]) > 0.15:
        raise ContractError("base kept moving after the e-stop latch")
    stopped_joints = joint_positions(link.latest(JOINT_STATES))
    for name in JOINTS:
        if stopped_joints.get(name, 0.0) - held_joints.get(name, 0.0) > 0.05:
            raise ContractError(f"{name} moved while e-stop was latched")

    await link.publish(ESTOP, {"data": False})
    await _beat(link)
    await link.sleep(0.2)
    still = safety_of(link.latest(SAFETY))
    evidence["estop_released"] = still
    log(f"EVIDENCE vr_contract estop_false safety={still}")
    if still is None or not still["latched"]:
        raise ContractError("publishing emergency_stop false cleared the latch")

    response = await link.call_service(RESET, TRIGGER, {})
    ok, message = service_ok(response)
    evidence["reset_message"] = message
    if not ok:
        raise ContractError(f"e-stop reset refused: {message}")
    reset_deadline = link.clock() + 5.0
    cleared = None
    while link.clock() < reset_deadline:
        await _beat(link)
        await link.sleep(TICK_S)
        cleared = safety_of(link.latest(SAFETY))
        if cleared is not None and not cleared["latched"] and cleared["armed"]:
            break
    evidence["reset"] = cleared
    log(f"EVIDENCE vr_contract reset safety={cleared} message={message}")
    if cleared is None or cleared["latched"] or not cleared["armed"]:
        raise ContractError(f"e-stop reset did not clear the latch ({cleared})")

    burst_deadline = link.clock() + 0.4
    while link.clock() < burst_deadline:
        await _beat(link)
        await _twist(link, DRIVE_SPEED, 0.0)
        await link.sleep(TICK_S)
    silent_start = pose_of(link.latest(ODOM))
    silent_deadline = link.clock() + LINK_LOSS_TIMEOUT_S
    loss = None
    while link.clock() < silent_deadline:
        await _twist(link, DRIVE_SPEED, 0.0)
        await link.sleep(TICK_S)
        loss = safety_of(link.latest(SAFETY))
        if (
            loss is not None
            and not loss["armed"]
            and not loss["latched"]
            and loss["reason"] == "heartbeat_watchdog"
            and speed_of(link.latest(ODOM)) < 0.05
        ):
            break
    travel = math.hypot(
        pose_of(link.latest(ODOM))[0] - silent_start[0],
        pose_of(link.latest(ODOM))[1] - silent_start[1],
    )
    evidence["link_loss"] = loss
    evidence["link_loss_travel_m"] = travel
    log(f"EVIDENCE vr_contract link_loss safety={loss} travel={travel:.3f}")
    if (
        loss is None
        or loss["armed"]
        or loss["latched"]
        or loss["reason"] != "heartbeat_watchdog"
    ):
        raise ContractError(
            f"stopping the heartbeat did not disarm /ohho/safety/state ({loss})"
        )
    if speed_of(link.latest(ODOM)) >= 0.05 or travel > 0.20:
        raise ContractError(
            "base kept driving after heartbeat link loss "
            f"(speed {speed_of(link.latest(ODOM)):.3f} m/s, travel {travel:.3f} m)"
        )


async def execute(link):
    evidence = {}
    try:
        await _run(link, evidence)
    except ContractError as exc:
        exc.evidence = evidence
        raise
    evidence["ok"] = True
    return evidence


def _write_report(path, payload):
    if not path:
        return
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Scripted VR headset for the Gazebo rosbridge contract"
    )
    parser.add_argument("--url", default="ws://127.0.0.1:9090")
    parser.add_argument("--report", default="")
    return parser.parse_args(argv)


async def _async_main(argv):
    args = parse_args(argv)
    link = RosbridgeClient(args.url)
    try:
        await link.connect()
        evidence = await execute(link)
    except ContractError as exc:
        payload = dict(exc.evidence)
        payload["ok"] = False
        payload["error"] = str(exc)
        _write_report(args.report, payload)
        log(f"EVIDENCE vr_contract FAIL {exc}")
        return 1
    finally:
        await link.close()
    _write_report(args.report, evidence)
    log("EVIDENCE vr_contract PASS")
    return 0


def main(argv=None):
    return asyncio.run(_async_main(argv))


if __name__ == "__main__":
    sys.exit(main())
