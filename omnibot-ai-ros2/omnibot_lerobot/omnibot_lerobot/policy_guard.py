"""Policy safety guards: camera validation, latency bounding, and command clamping.

Pure Python: no ROS, Torch, or LeRobot dependencies.
The policy node is a thin wrapper around PolicyGuard so decision logic
can be tested headless and without a ROS graph or GPU.

OHH-100 reuses MAX_JOINT_DELTA_RAD (0.15 rad per cycle) from arm_safety.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import sys
from typing import Any, Callable, Sequence
import numpy as np

# Re-use MAX_JOINT_DELTA_RAD from arm_safety (OHH-93 / OHH-100 shared constant).
# If arm_safety is not yet in sys.modules, try loading it from omnibot_arm/scripts.
if "arm_safety" not in sys.modules:
    try:
        from arm_safety import MAX_JOINT_DELTA_RAD, clamp_joint_command
    except ImportError:
        _arm_scripts = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "..",
                "omnibot-ros2",
                "omnibot_arm",
                "scripts",
            )
        )
        if os.path.isdir(_arm_scripts) and _arm_scripts not in sys.path:
            sys.path.insert(0, _arm_scripts)
            try:
                from arm_safety import MAX_JOINT_DELTA_RAD, clamp_joint_command
            except ImportError:
                MAX_JOINT_DELTA_RAD = 0.15
                clamp_joint_command = None
        else:
            MAX_JOINT_DELTA_RAD = 0.15
            clamp_joint_command = None
else:
    import arm_safety

    MAX_JOINT_DELTA_RAD = arm_safety.MAX_JOINT_DELTA_RAD
    clamp_joint_command = getattr(arm_safety, "clamp_joint_command", None)

DEFAULT_POLICY_PERIOD = 0.10  # 10 Hz
DEFAULT_IMAGE_KEYS = ["observation.images.wrist", "observation.images.bev"]
DEFAULT_BASE_VEL_SCALE = 0.3
MAX_BASE_LIN_VEL = 0.20  # m/s (Yahboom hardware limit)
MAX_BASE_ANG_VEL = 0.50  # rad/s (Yahboom hardware limit)


def is_black_frame(img: Any) -> bool:
    """Check if an image frame is missing, empty, or all zeros (black).

    Returns True if:
      - img is None
      - img is not a numpy array
      - img has size 0
      - all pixels in img are zero
    """
    if img is None:
        return True
    if not isinstance(img, np.ndarray):
        return True
    if img.size == 0:
        return True
    return not bool(np.any(img))


def check_camera_images(
    camera_images: dict[str, Any],
    required_keys: Sequence[str] | None = None,
) -> tuple[bool, list[str]]:
    """Verify that required camera frames are present and not black.

    Returns (is_valid, invalid_or_missing_keys).
    """
    if required_keys is None:
        # If camera_images has short keys 'wrist' and 'bev', check those;
        # otherwise default to observation.images.wrist / observation.images.bev
        short_keys = [k for k in ["wrist", "bev"] if k in camera_images]
        keys: Sequence[str] = (
            ["wrist", "bev"] if short_keys else DEFAULT_IMAGE_KEYS
        )
    else:
        keys = required_keys

    invalid = []
    for k in keys:
        img = camera_images.get(k)
        if is_black_frame(img):
            invalid.append(k)
    return (len(invalid) == 0, invalid)


def is_latency_exceeded(duration_sec: float, policy_period_sec: float) -> bool:
    """Return True if inference duration strictly exceeds the allowed policy period."""
    return float(duration_sec) > float(policy_period_sec)


def clamp_joint_deltas(
    commanded: Sequence[float],
    reference: Sequence[float] | None,
    max_delta: float = MAX_JOINT_DELTA_RAD,
) -> list[float]:
    """Limit joint changes to at most max_delta radians from reference."""
    if reference is None:
        return [float(x) for x in commanded]
    step = abs(float(max_delta))
    stepped = []
    for target, ref in zip(commanded, reference):
        delta = float(target) - float(ref)
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        stepped.append(float(ref) + delta)
    return stepped


def clamp_arm_action(
    commanded: Sequence[float] | np.ndarray,
    reference: Sequence[float] | np.ndarray | None,
    max_delta: float = MAX_JOINT_DELTA_RAD,
    joint_min: Sequence[float] | None = None,
    joint_max: Sequence[float] | None = None,
) -> np.ndarray:
    """Clamp arm joint targets to joint limits and max_delta per cycle."""
    cmd_list = [float(x) for x in commanded]
    ref_list = [float(x) for x in reference] if reference is not None else None

    if joint_min is not None and joint_max is not None:
        if clamp_joint_command is not None:
            try:
                res = clamp_joint_command(
                    cmd_list, joint_min, joint_max, ref_list, max_delta
                )
                return np.array(res, dtype=np.float32)
            except Exception:
                pass
        # Fallback limit clamp
        cmd_list = [
            max(float(mn), min(float(mx), val))
            for val, mn, mx in zip(cmd_list, joint_min, joint_max)
        ]

    stepped = clamp_joint_deltas(cmd_list, ref_list, max_delta)
    return np.array(stepped, dtype=np.float32)


def clamp_base_action(
    base_action: Sequence[float] | np.ndarray,
    scale: float = DEFAULT_BASE_VEL_SCALE,
    max_lin: float = MAX_BASE_LIN_VEL,
    max_ang: float = MAX_BASE_ANG_VEL,
) -> np.ndarray:
    """Scale and clip base velocity [vx, vy, vz/omega]."""
    vx = float(np.clip(float(base_action[0]) * scale, -max_lin, max_lin))
    vy = float(np.clip(float(base_action[1]) * scale, -max_lin, max_lin))
    vz = float(np.clip(float(base_action[2]) * scale, -max_ang, max_ang))
    return np.array([vx, vy, vz], dtype=np.float32)


@dataclass
class TickResult:
    """Outcome of a single policy tick."""

    should_publish: bool
    arm_command: np.ndarray | None = None
    base_command: np.ndarray | None = None
    warning: str | None = None
    dropped_reason: str | None = None
    inference_duration: float | None = None
    preprocess_duration: float | None = None
    publish_zero_base: bool = False


class PolicyGuard:
    """Stateful policy guard for arm and base safety."""

    def __init__(
        self,
        policy_period: float = DEFAULT_POLICY_PERIOD,
        max_joint_delta_rad: float = MAX_JOINT_DELTA_RAD,
        required_keys: Sequence[str] | None = None,
        joint_min: Sequence[float] | None = None,
        joint_max: Sequence[float] | None = None,
        base_vel_scale: float = DEFAULT_BASE_VEL_SCALE,
        max_lin_vel: float = MAX_BASE_LIN_VEL,
        max_ang_vel: float = MAX_BASE_ANG_VEL,
    ):
        self.policy_period = (
            float(policy_period)
            if policy_period > 0
            else DEFAULT_POLICY_PERIOD
        )
        self.max_joint_delta_rad = float(max_joint_delta_rad)
        self.required_keys = (
            list(required_keys) if required_keys is not None else None
        )
        self.joint_min = list(joint_min) if joint_min is not None else None
        self.joint_max = list(joint_max) if joint_max is not None else None
        self.base_vel_scale = float(base_vel_scale)
        self.max_lin_vel = float(max_lin_vel)
        self.max_ang_vel = float(max_ang_vel)
        self.last_arm_cmd: np.ndarray | None = None

    def reset(self) -> None:
        """Reset internal command tracking (e.g. on policy enable/task change)."""
        self.last_arm_cmd = None

    def check_images(
        self,
        camera_images: dict[str, Any],
        required_keys: Sequence[str] | None = None,
    ) -> tuple[bool, list[str]]:
        """Check image availability and quality (no black frames)."""
        keys = required_keys if required_keys is not None else self.required_keys
        return check_camera_images(camera_images, keys)

    def is_latency_exceeded(
        self, duration_sec: float, period_sec: float | None = None
    ) -> bool:
        """Check if inference elapsed time exceeded the policy period."""
        period = period_sec if period_sec is not None else self.policy_period
        return is_latency_exceeded(duration_sec, period)

    def process_arm_action(
        self,
        arm_action: Sequence[float] | np.ndarray,
        reference: Sequence[float] | np.ndarray | None = None,
    ) -> np.ndarray:
        """Clamp arm action to max_joint_delta_rad from baseline and update last_arm_cmd.

        Baseline is last_arm_cmd if available, else reference (current joint positions).
        The guard clamps both its output and its internal reference to joint limits.
        """
        baseline = (
            self.last_arm_cmd if self.last_arm_cmd is not None else reference
        )
        if (
            baseline is not None
            and self.joint_min is not None
            and self.joint_max is not None
        ):
            baseline = [
                max(float(mn), min(float(mx), float(v)))
                for v, mn, mx in zip(baseline, self.joint_min, self.joint_max)
            ]

        clamped = clamp_arm_action(
            arm_action,
            baseline,
            max_delta=self.max_joint_delta_rad,
            joint_min=self.joint_min,
            joint_max=self.joint_max,
        )
        if self.joint_min is not None and self.joint_max is not None:
            clamped = np.array(
                [
                    max(float(mn), min(float(mx), float(v)))
                    for v, mn, mx in zip(clamped, self.joint_min, self.joint_max)
                ],
                dtype=np.float32,
            )
        self.last_arm_cmd = clamped
        return clamped

    def process_base_action(
        self, base_action: Sequence[float] | np.ndarray
    ) -> np.ndarray:
        """Scale and clip base velocity action."""
        return clamp_base_action(
            base_action,
            scale=self.base_vel_scale,
            max_lin=self.max_lin_vel,
            max_ang=self.max_ang_vel,
        )

    def tick(
        self,
        camera_images: dict[str, Any],
        adapter: Any,
        arm_positions: Sequence[float] | np.ndarray,
        base_vel: Sequence[float] | np.ndarray | None = None,
        task_description: str = "",
        obs_builder: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
        now_fn: Callable[[], float] | None = None,
    ) -> TickResult:
        """Execute one policy tick decision.

        1. Validates camera images (no missing frames or black frames).
        2. Prepares observation dictionary (no black-frame zero substitution).
        3. Measures select_action duration.
        4. Drops the action if the full tick exceeds policy_period.
        5. Clamps arm targets to 0.15 rad/cycle and clips base velocities.
        """
        import time

        _now = now_fn if now_fn is not None else time.perf_counter

        # Determine required image keys from adapter if not set explicitly
        req_keys = self.required_keys
        if req_keys is None and hasattr(adapter, "image_keys"):
            req_keys = getattr(adapter, "image_keys")

        # 1. Image check: refuse missing or black frames
        valid, invalid = self.check_images(camera_images, req_keys)
        if not valid:
            warning = f"Waiting for valid images (missing or black: {invalid})"
            return TickResult(
                should_publish=False,
                publish_zero_base=True,
                base_command=np.zeros(3, dtype=np.float32),
                arm_command=None,
                warning=warning,
                dropped_reason="missing_or_black_images",
            )

        # 2. Build observation
        t_pre_0 = _now()
        if obs_builder is not None:
            obs = obs_builder(camera_images)
        else:
            obs = self._default_build_obs(
                camera_images=camera_images,
                adapter=adapter,
                arm_positions=arm_positions,
                base_vel=base_vel,
                task_description=task_description,
                image_keys=req_keys or DEFAULT_IMAGE_KEYS,
            )
        t_pre_1 = _now()
        preprocess_dur = t_pre_1 - t_pre_0

        # 3. Model inference with timing
        t_inf_0 = _now()
        raw_action = adapter.select_action(obs)
        t_inf_1 = _now()
        inference_dur = t_inf_1 - t_inf_0

        # 4. Latency check: drop stale action if the tick exceeded policy_period
        tick_dur = t_inf_1 - t_pre_0
        if self.is_latency_exceeded(tick_dur, self.policy_period):
            warning = (
                f"Policy tick ({tick_dur * 1000.0:.1f}ms) exceeded "
                f"policy period ({self.policy_period * 1000.0:.1f}ms) — action dropped."
            )
            return TickResult(
                should_publish=False,
                publish_zero_base=True,
                base_command=np.zeros(3, dtype=np.float32),
                arm_command=None,
                warning=warning,
                dropped_reason="stale_inference",
                inference_duration=inference_dur,
                preprocess_duration=preprocess_dur,
            )

        # 5. Clamping and rate limiting
        raw_action_arr = np.asarray(raw_action, dtype=np.float32).ravel()
        clamped_arm = self.process_arm_action(
            raw_action_arr[:6], reference=arm_positions
        )
        clamped_base = self.process_base_action(raw_action_arr[6:9])

        return TickResult(
            should_publish=True,
            publish_zero_base=False,
            arm_command=clamped_arm,
            base_command=clamped_base,
            inference_duration=inference_dur,
            preprocess_duration=preprocess_dur,
        )

    def _default_build_obs(
        self,
        camera_images: dict[str, Any],
        adapter: Any,
        arm_positions: Sequence[float] | np.ndarray,
        base_vel: Sequence[float] | np.ndarray | None,
        task_description: str,
        image_keys: Sequence[str],
    ) -> dict[str, Any]:
        """Simple numpy observation builder without Torch/ROS."""
        obs: dict[str, Any] = {}
        for k in image_keys:
            img = camera_images[k]
            # Convert to CHW float32 [0, 1] with batch dim (1, C, H, W)
            arr = np.asarray(img, dtype=np.float32) / 255.0
            if arr.ndim == 3:
                arr = arr.transpose(2, 0, 1)
            if arr.ndim == 3:
                arr = arr[np.newaxis, ...]
            obs[k] = arr

        bv = (
            np.asarray(base_vel, dtype=np.float32)
            if base_vel is not None
            else np.zeros(3, dtype=np.float32)
        )
        ap = np.asarray(arm_positions, dtype=np.float32)
        state = np.concatenate([ap, bv])
        state_key = getattr(adapter, "state_key", "observation.state")
        obs[state_key] = state[np.newaxis, ...]

        task_key = getattr(adapter, "task_key", "task")
        if task_key:
            obs[task_key] = task_description

        return obs
