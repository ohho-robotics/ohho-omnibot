"""
measure_sync_jitter.py
----------------------
Report wrist vs BEV (and optional state) timestamp skew for a LeRobot dataset.

Hardware recordings: run on the robot after teleop capture and paste the JSON
into the dataset card (see ``data_engine/dataset_card_template.md``).

Usage:
    python -m data_engine.scripts.measure_sync_jitter --dataset /path/to/dataset
"""

from __future__ import annotations

import json
from pathlib import Path

import click

from data_engine.recording.sync_jitter import measure_dataset_skew


@click.command()
@click.option(
    "--dataset",
    required=True,
    type=click.Path(exists=True, file_okay=False),
    help="LeRobot dataset root",
)
@click.option(
    "--output",
    type=click.Path(),
    default=None,
    help="Optional path to write JSON report",
)
def main(dataset: str, output: str | None) -> None:
    root = Path(dataset)
    reports = measure_dataset_skew(root)
    payload = {
        "dataset": str(root.resolve()),
        "episodes": [r.as_dict() for r in reports],
    }
    text = json.dumps(payload, indent=2)
    click.echo(text)
    if output:
        Path(output).write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
