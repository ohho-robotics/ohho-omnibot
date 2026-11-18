"""ACT (Action Chunking Transformer) policy adapter.

ACT is faster than SmolVLA (~20 Hz vs 10 Hz) and works well for precise
manipulation, but is not language-conditioned. Serves as a strong baseline
for OmniBot 9-DOF mobile manipulation (6 arm joints + 3 base velocities).
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Optional

import numpy as np

from .base import PolicyAdapter
from .registry import register

logger = logging.getLogger(__name__)

try:
    import torch
    import torch.nn as nn

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    nn = None

# Support both LeRobot >= 0.3 (lerobot.policies) and older layouts (lerobot.common.policies)
LEROBOT_ACT_AVAILABLE = False
ACTPolicy = None
ACTConfig = None
PolicyFeature = None
FeatureType = None

try:
    from lerobot.configs.policies import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    LEROBOT_ACT_AVAILABLE = True
except ImportError:
    try:
        from lerobot.common.policies.act.configuration_act import ACTConfig
        from lerobot.common.policies.act.modeling_act import ACTPolicy

        LEROBOT_ACT_AVAILABLE = True
    except ImportError:
        pass


class _FallbackACTPolicy(nn.Module if TORCH_AVAILABLE else object):
    """Lightweight fallback module when LeRobot is not installed.

    Provides the forward() and select_action() signatures for CPU smoke testing
    and offline unit verification.
    """

    def __init__(self, action_dim: int = 9, chunk_size: int = 50):
        super().__init__()
        self.action_dim = action_dim
        self.chunk_size = chunk_size
        self.fc = nn.Linear(action_dim, action_dim * chunk_size)
        self.criterion = nn.L1Loss()

    def forward(self, batch: dict[str, Any]) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        state = batch.get("observation.state")
        target_action = batch.get("action")
        if state is None or target_action is None:
            raise ValueError("batch must contain 'observation.state' and 'action'")

        batch_size = state.shape[0]
        pred = self.fc(state).view(batch_size, self.chunk_size, self.action_dim)
        loss = self.criterion(pred, target_action)
        return loss, {"l1_loss": loss}

    def select_action(self, obs: dict[str, Any]) -> torch.Tensor:
        state = obs.get("observation.state")
        if state is None:
            return torch.zeros((1, self.action_dim), dtype=torch.float32)
        if state.ndim == 1:
            state = state.unsqueeze(0)
        batch_size = state.shape[0]
        pred = self.fc(state).view(batch_size, self.chunk_size, self.action_dim)
        return pred[:, 0, :]

    def reset(self) -> None:
        pass

    def save_pretrained(self, save_directory: str) -> None:
        save_path = Path(save_directory)
        save_path.mkdir(parents=True, exist_ok=True)
        torch.save(self.state_dict(), save_path / "policy.pt")


@register("act")
class ACTAdapter(PolicyAdapter):
    """Wraps LeRobot ACTPolicy for OmniBot 9-DOF mobile manipulation."""

    def __init__(self):
        self._policy = None
        self._device = None
        self._chunk_size = 50

    def load(
        self,
        checkpoint: Optional[str] = None,
        device: str = "cuda",
        chunk_size: int = 50,
        scratch: bool = False,
        **kwargs,
    ) -> None:
        """Load weights from a checkpoint or initialize from scratch.

        Args:
            checkpoint: HuggingFace hub ID or local checkpoint path.
                        Pass None or 'scratch' to initialize from scratch.
            device: 'cuda' or 'cpu'.
            chunk_size: Action chunk horizon (default: 50).
            scratch: Force initialization from scratch without downloading pretrained weights.
        """
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch is required: pip install torch")

        dev = torch.device(
            device if (device == "cuda" and torch.cuda.is_available()) else "cpu"
        )
        self._device = dev
        self._chunk_size = chunk_size

        is_scratch = scratch or (checkpoint in (None, "", "scratch", "random", "none"))

        if LEROBOT_ACT_AVAILABLE:
            if not is_scratch and checkpoint is not None:
                try:
                    self._policy = ACTPolicy.from_pretrained(checkpoint)
                    self._policy = self._policy.to(dev)
                    self._policy.eval()
                    return
                except Exception as e:
                    logger.warning(
                        "Failed to load checkpoint '%s' (%s). Initializing from config.",
                        checkpoint,
                        e,
                    )

            # Initialize ACTPolicy from configuration
            cfg_kwargs: dict[str, Any] = {
                "chunk_size": chunk_size,
                "n_action_steps": chunk_size,
                "pretrained_backbone_weights": None if is_scratch else "ResNet18_Weights.IMAGENET1K_V1",
            }
            if PolicyFeature is not None and FeatureType is not None:
                cfg_kwargs["input_features"] = {
                    "observation.images.wrist": PolicyFeature(
                        type=FeatureType.VISUAL, shape=(3, self.image_size[1], self.image_size[0])
                    ),
                    "observation.images.bev": PolicyFeature(
                        type=FeatureType.VISUAL, shape=(3, self.image_size[1], self.image_size[0])
                    ),
                    "observation.state": PolicyFeature(
                        type=FeatureType.STATE, shape=(self.action_dim,)
                    ),
                }
                cfg_kwargs["output_features"] = {
                    "action": PolicyFeature(
                        type=FeatureType.ACTION, shape=(self.action_dim,)
                    ),
                }

            config = ACTConfig(**cfg_kwargs) if ACTConfig is not None else None
            # Provide default normalization statistics so normalize_inputs doesn't fail
            dataset_stats = {
                "observation.images.wrist": {
                    "mean": torch.zeros(3, 1, 1),
                    "std": torch.ones(3, 1, 1),
                },
                "observation.images.bev": {
                    "mean": torch.zeros(3, 1, 1),
                    "std": torch.ones(3, 1, 1),
                },
                "observation.state": {
                    "mean": torch.zeros(self.action_dim),
                    "std": torch.ones(self.action_dim),
                },
                "action": {
                    "mean": torch.zeros(self.action_dim),
                    "std": torch.ones(self.action_dim),
                },
            }
            self._policy = ACTPolicy(config, dataset_stats=dataset_stats)
            self._policy = self._policy.to(dev)
            self._policy.eval()
        else:
            # Fallback for CPU smoke test / environment without lerobot package
            logger.info("Using FallbackACTPolicy (LeRobot not installed).")
            self._policy = _FallbackACTPolicy(
                action_dim=self.action_dim, chunk_size=self._chunk_size
            ).to(dev)

    def reset(self) -> None:
        if self._policy is not None and hasattr(self._policy, "reset"):
            self._policy.reset()

    def select_action(self, obs: dict) -> np.ndarray:
        if self._policy is None:
            return np.zeros(self.action_dim, dtype=np.float32)
        dev = self._device or torch.device("cpu")
        tensor_obs = {}
        for k, v in obs.items():
            t = v.to(dev) if hasattr(v, "to") else torch.as_tensor(v, device=dev)
            if k in self.image_keys and t.ndim == 3:
                t = t.unsqueeze(0)
            elif k == "observation.state" and t.ndim == 1:
                t = t.unsqueeze(0)
            tensor_obs[k] = t
        with torch.no_grad():
            action = self._policy.select_action(tensor_obs)
        arr = action.cpu().numpy() if hasattr(action, "cpu") else np.array(action)
        return arr[0] if arr.ndim == 2 else arr

    def export_onnx(self, output_path: str | Path, opset_version: int = 14) -> bool:
        """Export the ACT policy to ONNX format for deployment.

        Returns True if export was successful, False otherwise.
        """
        if not TORCH_AVAILABLE:
            logger.error("PyTorch required for ONNX export.")
            return False

        if self._policy is None:
            logger.error("Cannot export: policy model is not loaded.")
            return False

        out_path = Path(output_path).expanduser()
        out_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            import onnx  # noqa: F401
        except ImportError:
            logger.warning(
                "ONNX package not installed. Install with: pip install onnx onnxscript onnxruntime"
            )
            return False

        try:
            self._policy.eval()
            w, h = self.image_size
            wrist_dummy = torch.zeros(1, 3, h, w, dtype=torch.float32)
            bev_dummy = torch.zeros(1, 3, h, w, dtype=torch.float32)
            state_dummy = torch.zeros(1, self.action_dim, dtype=torch.float32)

            # Exportable wrapper that takes explicit named tensors
            class _ACTONNXWrapper(nn.Module):
                def __init__(self, policy):
                    super().__init__()
                    self.policy = policy

                def forward(self, wrist, bev, state):
                    batch = {
                        "observation.images.wrist": wrist,
                        "observation.images.bev": bev,
                        "observation.state": state,
                    }
                    if hasattr(self.policy, "normalize_inputs"):
                        batch = self.policy.normalize_inputs(batch)
                    if hasattr(self.policy, "model"):
                        actions, _ = self.policy.model(batch)
                    elif hasattr(self.policy, "fc"):
                        # Fallback module
                        actions = self.policy.fc(state).view(
                            state.shape[0], -1, state.shape[-1]
                        )
                    else:
                        raise NotImplementedError("Underlying model structure not recognized for ONNX export")
                    if hasattr(self.policy, "unnormalize_outputs"):
                        actions = self.policy.unnormalize_outputs({"action": actions})["action"]
                    return actions

            wrapper = _ACTONNXWrapper(self._policy)
            torch.onnx.export(
                wrapper,
                (wrist_dummy, bev_dummy, state_dummy),
                str(out_path),
                opset_version=opset_version,
                input_names=["observation_images_wrist", "observation_images_bev", "observation_state"],
                output_names=["predicted_action_chunk"],
                dynamic_axes={
                    "observation_images_wrist": {0: "batch_size"},
                    "observation_images_bev": {0: "batch_size"},
                    "observation_state": {0: "batch_size"},
                    "predicted_action_chunk": {0: "batch_size"},
                },
            )
            logger.info("Exported ONNX policy: %s", out_path)
            return True
        except Exception as e:
            logger.warning("ONNX export encountered an error (%s).", e)
            return False

    @property
    def image_keys(self) -> list[str]:
        return ["observation.images.wrist", "observation.images.bev"]

    @property
    def task_key(self):
        return None  # ACT is not language-conditioned

    @property
    def action_dim(self) -> int:
        return 9

    @property
    def image_size(self) -> tuple[int, int]:
        return (320, 240)
