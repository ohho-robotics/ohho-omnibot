#!/usr/bin/env python3
"""Write the 20-trial sim eval scaffold for the ACT VR pick policy.

Does not launch Gazebo and does not fill success. A person scores each seed
after watching whether the cube ended in the bin.

    python omnibot-ai-engines/lerobot_engine/eval_act_vr_sim.py
    python omnibot-ai-engines/lerobot_engine/eval_act_vr_sim.py --write trials.md

When the OHH-237 scripted pick launch is on the machine, one trial is:

    ros2 launch omnibot_lerobot scripted_vr_pick.launch.py \\
        world:=apartment pick_place:=true pick_place_seed:=SEED episodes:=1

Seeds in the table are 237 through 256. Leave success blank until scored.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ENGINES = Path(__file__).resolve().parents[1]
if str(_ENGINES) not in sys.path:
    sys.path.insert(0, str(_ENGINES))

from lerobot_engine.act_vr_sim import render_trial_markdown  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--write",
        default=None,
        help="Write the markdown table to this path instead of stdout.",
    )
    args = parser.parse_args(argv)
    text = render_trial_markdown()
    if args.write:
        path = Path(args.write)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        print(f"Wrote {path} ({text.count(chr(10))} lines). Success column is blank.")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
