#!/usr/bin/env python3
"""Convert lerobot-calibrate JSON into an OmniBot so101_<id>.yaml file.

Reads a JSON file and writes YAML. Does not open a port or import a motor bus.
"""

import argparse
import json
import sys
from pathlib import Path

import servo_offsets


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Convert lerobot-calibrate JSON into OmniBot home-tick YAML. "
            "home_tick is (range_min + range_max) // 2 for each of the six joints. "
            "Does not connect to the arm."
        )
    )
    parser.add_argument("json_path", help="lerobot-calibrate JSON file")
    parser.add_argument(
        "--robot-id",
        required=True,
        help="Id used in so101_<id>.yaml and the robot_id field",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Destination yaml path (for example config/calibration/so101_<id>.yaml)",
    )
    args = parser.parse_args(argv)
    try:
        payload = json.loads(Path(args.json_path).read_text(encoding="utf-8"))
        motors = servo_offsets.parse_lerobot_json(payload)
        text = servo_offsets.format_calibration_yaml(args.robot_id, motors)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text, encoding="utf-8")
    except servo_offsets.CalibrationError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except json.JSONDecodeError as exc:
        print(f"error: invalid JSON in {args.json_path}: {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    flipped = [motor.joint_name for motor in motors if motor.drive_mode != 0]
    if flipped:
        print(
            "warning: drive_mode is not 0 for "
            + ", ".join(flipped)
            + ". OmniBot does not flip joint sign.",
            file=sys.stderr,
        )
    print(f"Wrote {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
