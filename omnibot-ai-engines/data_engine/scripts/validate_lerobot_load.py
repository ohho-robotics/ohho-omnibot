"""
validate_lerobot_load.py
------------------------
Structural checks (``validate_dataset``) plus optional ``LeRobotDataset`` load.

Uses the same offline loading rules as ``ohho.sim_loop.validate_lerobot_dataset``.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import click

from data_engine.scripts.validate_dataset import validate


def _lerobot_revision(info: dict) -> str:
    raw = str(info.get("codebase_version", "v2.0"))
    if raw and raw[0].isdigit():
        return f"v{raw}"
    return raw


def try_load_lerobot_dataset(dataset_root: Path) -> tuple[bool, str]:
    try:
        try:
            from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        except (ImportError, ModuleNotFoundError):
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except (ImportError, ModuleNotFoundError) as exc:
        return False, f"lerobot not installed ({exc})"

    info = json.loads((dataset_root / "meta" / "info.json").read_text(encoding="utf-8"))
    repo_id = info.get("repo_id") or "local/dataset"
    revision = _lerobot_revision(info)

    prev = os.environ.get("HF_HUB_OFFLINE")
    os.environ["HF_HUB_OFFLINE"] = "1"
    try:
        ds = LeRobotDataset(repo_id=repo_id, root=str(dataset_root), revision=revision)
        n = len(ds)
        if n < 1:
            return False, "LeRobotDataset loaded 0 frames"
        return True, f"LeRobotDataset OK ({n} frames)"
    except Exception as exc:
        return False, f"LeRobotDataset load failed: {exc}"
    finally:
        if prev is None:
            os.environ.pop("HF_HUB_OFFLINE", None)
        else:
            os.environ["HF_HUB_OFFLINE"] = prev


@click.command()
@click.option("--dataset", required=True, type=click.Path(exists=True))
@click.option(
    "--require-lerobot",
    is_flag=True,
    default=False,
    help="Exit non-zero if LeRobotDataset cannot be imported/loaded",
)
@click.option("--verbose", is_flag=True, default=False)
def main(dataset: str, require_lerobot: bool, verbose: bool) -> None:
    root = Path(dataset)
    click.echo(f"Validating: {root}\n")
    if not validate(root, verbose=verbose):
        raise SystemExit(1)
    click.echo("Structural validation passed.")

    ok, msg = try_load_lerobot_dataset(root)
    click.echo(msg)
    if require_lerobot and not ok:
        raise SystemExit(1)
    if ok:
        click.echo("\nAll checks passed (structure + LeRobotDataset).")
    else:
        click.echo("\nStructural checks passed; LeRobotDataset step skipped or failed.")


if __name__ == "__main__":
    main()
