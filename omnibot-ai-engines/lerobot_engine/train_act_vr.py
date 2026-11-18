#!/usr/bin/env python3
"""Train ACT on the VR sim pick dataset, or run the CPU smoke.

Smoke (no torch, no lerobot, no W&B, synthetic 9-DoF chunks)::

    python omnibot-ai-engines/lerobot_engine/train_act_vr.py --smoke

Full run. Exits without training when the LeRobot v2 root is missing and
the Hugging Face dataset is still unpublished. Prints the human push steps.
Does not start W&B.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

_ENGINES = Path(__file__).resolve().parents[1]
if str(_ENGINES) not in sys.path:
    sys.path.insert(0, str(_ENGINES))

from lerobot_engine.act_vr_sim import (  # noqa: E402
    full_train_blockers,
    hf_dataset_push_steps,
    load_train_config,
    plan_full_train,
    resolve_dataset,
    run_lerobot_act_cpu_smoke,
    run_numpy_smoke,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=None,
        help="YAML config. Default: configs/act_vr_sim_pick.yaml",
    )
    parser.add_argument(
        "--dataset-path",
        default=None,
        help="Local LeRobot v2 root. Overrides dataset.local_episode_root.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Smoke or full checkpoint directory.",
    )
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Two CPU numpy steps on synthetic data. Does not call W&B.",
    )
    parser.add_argument(
        "--act-smoke",
        action="store_true",
        help=(
            "Two CPU steps of a tiny LeRobot ACTPolicy when that package "
            "imports. No pretrained download, no W&B, weights not saved."
        ),
    )
    parser.add_argument(
        "--run-full",
        action="store_true",
        help="Execute plan_full_train when torch and lerobot import.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = load_train_config(args.config)
    choice = resolve_dataset(
        config,
        dataset_path=args.dataset_path,
        smoke=args.smoke,
    )
    training = config.get("training") or {}
    output = Path(
        args.output_dir or training.get("output_dir") or "checkpoints/act_vr_sim_pick"
    ).expanduser()

    if args.act_smoke:
        try:
            manifest = run_lerobot_act_cpu_smoke(output, steps=2)
        except ImportError as exc:
            print(f"ACT CPU smoke not run: {exc}")
            print("Numpy smoke still works:")
            print("  python omnibot-ai-engines/lerobot_engine/train_act_vr.py --smoke")
            return 4
        print("ACT CPU smoke finished.")
        print(f"  backend: {manifest['backend']}")
        print(f"  weights_are_lerobot_act: {manifest['weights_are_lerobot_act']}")
        print(f"  checkpoint_persisted: {manifest['checkpoint_persisted']}")
        print(f"  wandb_run: {manifest['wandb_run']}")
        print(f"  parameters_changed: {manifest['parameters_changed']}")
        print(f"  parameter_count: {manifest['parameter_count']}")
        print(f"  losses: {manifest['losses']}")
        print(f"  manifest: {output / 'act_cpu_smoke.json'}")
        return 0

    if args.smoke:
        steps = int(training.get("smoke_steps") or 2)
        manifest = run_numpy_smoke(output, steps=steps)
        manifest["configured_local_episode_root"] = choice.get(
            "configured_local_episode_root", choice.get("path")
        )
        manifest["resolved_kind"] = choice["kind"]
        print("ACT VR smoke finished.")
        print(f"  backend: {manifest['backend']}")
        print(f"  weights_are_lerobot_act: {manifest['weights_are_lerobot_act']}")
        print(f"  wandb_run: {manifest['wandb_run']}")
        print(f"  losses: {manifest['losses']}")
        print(f"  manifest: {output / 'train_manifest.json'}")
        print("  dataset: synthetic_smoke (OHH-237 episodes are not in this tree)")
        return 0

    if not choice["trainable"]:
        print("No local LeRobot v2 dataset and HF dataset is unpublished.")
        print(hf_dataset_push_steps(str(choice.get("path") or "")))
        print("CPU smoke (does not use those episodes):")
        print("  python omnibot-ai-engines/lerobot_engine/train_act_vr.py --smoke")
        return 2

    blockers = full_train_blockers()
    command = plan_full_train(config, str(choice["path"]))
    if args.output_dir:
        # plan_full_train reads output from the yaml; keep the CLI override.
        for index, token in enumerate(command):
            if token == "--output-dir" and index + 1 < len(command):
                command[index + 1] = str(output)
    print("Full ACT command:")
    print("  " + " ".join(command))
    if blockers:
        print("Not started. Missing: " + ", ".join(blockers))
        print("This is not a W&B run and not an ACT weight update.")
        return 3
    if not args.run_full:
        print("Not started. Pass --run-full to execute that command.")
        return 0

    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


if __name__ == "__main__":
    raise SystemExit(main())
