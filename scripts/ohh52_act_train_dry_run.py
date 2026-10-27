#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validate the OHH-52 ACT training configs. Do not start training.

Needs Varun: budget + robot.

Standard library only. This process does not import torch, does not execute
train.py, and does not contact a GPU provider.
"""

from __future__ import annotations

import argparse
import ast
import math
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NEED = "Needs Varun: budget + robot."

LAUNCH_ENV = ("OHH52_LAUNCH_GPU", "OHH52_START_TRAINING")
REMOTE_PREFIXES = ("http://", "https://", "s3://", "gs://", "hf://", "ssh://")

ACT_YAML = Path("omnibot-ai-engines/lerobot_engine/configs/models/act.yaml")
ROBOT_YAML = Path("omnibot-ai-engines/lerobot_engine/configs/robot/omnibot_mobile_manip.yaml")
TRAIN_PY = Path("omnibot-ai-engines/lerobot_engine/train.py")
POLICY_PARAMS = Path("omnibot-ai-ros2/omnibot_lerobot/config/policy_params.yaml")
INFERENCE_LAUNCH = Path("omnibot-ai-ros2/omnibot_lerobot/launch/policy_inference.launch.py")
BRINGUP_LAUNCH = Path("omnibot-ros2/omnibot_bringup/launch/mobile_manipulation.launch.py")
RUNBOOK = Path("docs/gpu-train-runbook.md")
RESULTS = Path("docs/results.md")

REQUIRED_PATHS = (
    ACT_YAML,
    ROBOT_YAML,
    TRAIN_PY,
    Path("omnibot-ai-engines/lerobot_engine/models/act.py"),
    Path("omnibot-ai-engines/lerobot_engine/requirements.txt"),
    POLICY_PARAMS,
    INFERENCE_LAUNCH,
    BRINGUP_LAUNCH,
    RUNBOOK,
    RESULTS,
)

ACT_EXPECT = {
    ("model", "type"): "act",
    ("model", "pretrained"): "lerobot/act_base",
    ("model", "device"): "cuda",
    ("model", "policy_hz"): 20.0,
    ("model", "chunk_size"): 100,
    ("training", "num_epochs"): 200,
    ("training", "batch_size"): 16,
    ("training", "lr"): 1.0e-4,
    ("training", "lr_scheduler"): "cosine",
    ("training", "grad_clip_norm"): 10.0,
    ("training", "save_every"): 20,
    ("training", "num_workers"): 4,
}

TRAIN_FLAGS = (
    "--model",
    "--checkpoint",
    "--dataset-path",
    "--output-dir",
    "--num-epochs",
    "--batch-size",
    "--lr",
    "--device",
    "--chunk-size",
    "--save-every",
    "--grad-clip-norm",
    "--num-workers",
)

CAMERA_KEYS = ("observation.images.wrist", "observation.images.bev")
_INT = re.compile(r"^[+-]?\d+$")
_FLOAT = re.compile(r"^[+-]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?$")


class YamlError(Exception):
    pass


def _strip_comment(line: str) -> str:
    in_single = False
    in_double = False
    out: list[str] = []
    for ch in line:
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif ch == "#" and not in_single and not in_double:
            break
        out.append(ch)
    return "".join(out).rstrip()


def _split_flow(inner: str) -> list[str]:
    parts: list[str] = []
    buf: list[str] = []
    depth = 0
    for ch in inner:
        if ch in "[{":
            depth += 1
            buf.append(ch)
        elif ch in "]}":
            depth -= 1
            buf.append(ch)
        elif ch == "," and depth == 0:
            parts.append("".join(buf).strip())
            buf = []
        else:
            buf.append(ch)
    tail = "".join(buf).strip()
    if tail:
        parts.append(tail)
    return parts


def _parse_scalar(raw: str):
    raw = raw.strip()
    if len(raw) >= 2 and raw[0] == raw[-1] and raw[0] in {'"', "'"}:
        return raw[1:-1]
    if raw.startswith("[") and raw.endswith("]"):
        inner = raw[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(part) for part in _split_flow(inner)]
    if raw in {"true", "True"}:
        return True
    if raw in {"false", "False"}:
        return False
    if raw in {"null", "~", "None"}:
        return None
    if _INT.fullmatch(raw):
        return int(raw)
    if _FLOAT.fullmatch(raw):
        return float(raw)
    return raw


def parse_yaml_subset(text: str):
    """Parse the mappings and lists used by the lerobot_engine configs."""
    lines: list[tuple[int, int, str]] = []
    for lineno, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw:
            raise YamlError(f"line {lineno}: tab indentation is not supported")
        content = _strip_comment(raw)
        if not content.strip():
            continue
        indent = len(content) - len(content.lstrip(" "))
        lines.append((lineno, indent, content.strip()))
    if not lines:
        raise YamlError("empty document")

    def parse_node(index: int, indent: int):
        _lineno, _ind, text = lines[index]
        if text.startswith("- "):
            return parse_list(index, indent)
        return parse_map(index, indent)

    def parse_map(index: int, indent: int):
        result: dict = {}
        while index < len(lines):
            lineno, ind, text = lines[index]
            if ind < indent:
                break
            if ind > indent:
                raise YamlError(f"line {lineno}: unexpected indent")
            if text.startswith("- "):
                raise YamlError(f"line {lineno}: list item where a key was expected")
            if ":" not in text:
                raise YamlError(f"line {lineno}: expected key")
            key, _, rest = text.partition(":")
            key = key.strip()
            rest = rest.strip()
            index += 1
            if rest == "":
                if index < len(lines) and lines[index][1] > ind:
                    child, index = parse_node(index, lines[index][1])
                    result[key] = child
                else:
                    result[key] = None
            else:
                result[key] = _parse_scalar(rest)
        return result, index

    def parse_list(index: int, indent: int):
        items: list = []
        while index < len(lines):
            lineno, ind, text = lines[index]
            if ind < indent or not text.startswith("- "):
                break
            if ind != indent:
                raise YamlError(f"line {lineno}: unexpected list indent")
            item_text = text[2:].strip()
            index += 1
            if item_text == "":
                if index < len(lines) and lines[index][1] > indent:
                    child, index = parse_node(index, lines[index][1])
                    items.append(child)
                else:
                    items.append(None)
                continue
            if ":" in item_text and not item_text.startswith(("'", '"', "[")):
                key, _, rest = item_text.partition(":")
                child: dict = {}
                rest = rest.strip()
                child[key.strip()] = None if rest == "" else _parse_scalar(rest)
                while index < len(lines):
                    nlineno, nind, ntext = lines[index]
                    if nind <= indent or ntext.startswith("- "):
                        break
                    if ":" not in ntext:
                        raise YamlError(f"line {nlineno}: expected key")
                    subkey, _, subrest = ntext.partition(":")
                    index += 1
                    subrest = subrest.strip()
                    if subrest == "":
                        if index < len(lines) and lines[index][1] > nind:
                            nested, index = parse_node(index, lines[index][1])
                            child[subkey.strip()] = nested
                        else:
                            child[subkey.strip()] = None
                    else:
                        child[subkey.strip()] = _parse_scalar(subrest)
                items.append(child)
                continue
            items.append(_parse_scalar(item_text))
        return items, index

    document, index = parse_map(0, lines[0][1])
    if index != len(lines):
        raise YamlError(f"line {lines[index][0]}: unparsed trailing content")
    return document


def _lookup(cfg: dict, path: tuple[str, ...]):
    cur = cfg
    for part in path:
        if not isinstance(cur, dict) or part not in cur:
            return None, False
        cur = cur[part]
    return cur, True


def _same(found, expected) -> bool:
    if isinstance(expected, float):
        return isinstance(found, (int, float)) and math.isclose(
            float(found), expected, rel_tol=0.0, abs_tol=1e-12
        )
    return found == expected


def _fmt(value) -> str:
    if isinstance(value, float):
        if math.isclose(value, 1.0e-4, rel_tol=0.0, abs_tol=1e-15):
            return "1e-4"
        if value.is_integer():
            return f"{int(value)}.0"
        return format(value, "g")
    return str(value)


def _call_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return ""


def _add_argument_flags(tree: ast.AST) -> set[str]:
    flags: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _call_name(node.func) != "add_argument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        value = node.args[0].value
        if isinstance(value, str) and value.startswith("--"):
            flags.add(value)
    return flags


def _launch_arguments(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _call_name(node.func) != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        if isinstance(node.args[0].value, str):
            names.add(node.args[0].value)
    return names


def _is_epoch_range(node: ast.AST) -> bool:
    if not isinstance(node, ast.Call) or _call_name(node.func) != "range":
        return False
    if len(node.args) != 2 or not isinstance(node.args[1], ast.BinOp):
        return False
    high = node.args[1]
    if not isinstance(high.op, ast.Add) or not isinstance(high.right, ast.Constant):
        return False
    if high.right.value != 1 or not isinstance(high.left, ast.Attribute):
        return False
    return high.left.attr == "num_epochs"


def _epoch_loop(tree: ast.AST) -> ast.For | None:
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "main":
            for child in ast.walk(node):
                if isinstance(child, ast.For) and _is_epoch_range(child.iter):
                    return child
    return None


def _loop_enumerates_train_loader(loop: ast.For) -> bool:
    for node in ast.walk(loop):
        if node is loop or not isinstance(node, ast.For):
            continue
        call = node.iter
        if not isinstance(call, ast.Call) or _call_name(call.func) != "enumerate":
            continue
        if call.args and isinstance(call.args[0], ast.Name) and call.args[0].id == "train_loader":
            return True
    return False


def _loop_calls(loop: ast.For, name: str) -> bool:
    for node in ast.walk(loop):
        if isinstance(node, ast.Call) and _call_name(node.func) == name:
            return True
    return False


def _make_policy_takes_chunk_size(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _call_name(node.func) != "make_policy":
            continue
        if any(kw.arg == "chunk_size" for kw in node.keywords):
            return True
    return False


def _train_command(act: dict) -> str:
    model = act["model"]
    training = act["training"]
    flags = [
        "--model",
        _fmt(model["type"]),
        "--checkpoint",
        _fmt(model["pretrained"]),
        "--dataset-path",
        '"$DATASET_PATH"',
        "--output-dir",
        '"$OUTPUT_DIR"',
        "--num-epochs",
        _fmt(training["num_epochs"]),
        "--batch-size",
        _fmt(training["batch_size"]),
        "--lr",
        _fmt(training["lr"]),
        "--device",
        _fmt(model["device"]),
        "--chunk-size",
        _fmt(model["chunk_size"]),
        "--save-every",
        _fmt(training["save_every"]),
        "--grad-clip-norm",
        _fmt(training["grad_clip_norm"]),
        "--num-workers",
        _fmt(training["num_workers"]),
    ]
    parts = ["PYTHONPATH=omnibot-ai-engines python omnibot-ai-engines/lerobot_engine/train.py"]
    for index in range(0, len(flags), 2):
        parts.append(f"  {flags[index]} {flags[index + 1]}")
    return " \\\n".join(parts)


def _truthy(value: str | None) -> bool:
    return (value or "").strip().lower() in {"1", "true", "yes", "y", "on"}


def _is_remote(text: str) -> bool:
    lowered = text.strip().lower()
    return lowered.startswith(REMOTE_PREFIXES)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _check_results(text: str, errors: list[str]) -> None:
    if NEED not in text:
        errors.append(f"{RESULTS.as_posix()} is missing the {NEED} note")
    header = "| Trial | Success | Failure | Intervention | Notes |"
    if header not in text:
        errors.append(f"{RESULTS.as_posix()} is missing the Success/Failure/Intervention columns")
    for trial in range(1, 21):
        if f"| {trial} |" not in text:
            errors.append(f"{RESULTS.as_posix()} is missing trial row {trial}")


def _check_local_path(label: str, raw: str, errors: list[str]) -> None:
    if _is_remote(raw):
        errors.append(f"{label} is a remote URI. This dry-run only accepts a local directory.")
        return
    path = Path(raw).expanduser()
    if not path.exists():
        errors.append(f"{label} does not exist: {path}")
    elif not path.is_dir():
        errors.append(f"{label} is not a directory: {path}")


def validate(dataset_path: str | None, output_dir: str | None) -> tuple[list[str], list[str], str]:
    errors: list[str] = []
    notes: list[str] = []
    command = ""

    for rel in REQUIRED_PATHS:
        path = ROOT / rel
        if not path.is_file():
            errors.append(f"missing {rel.as_posix()}")
    if errors:
        return errors, notes, command

    try:
        act = parse_yaml_subset(_read(ROOT / ACT_YAML))
        robot_doc = parse_yaml_subset(_read(ROOT / ROBOT_YAML))
        params = parse_yaml_subset(_read(ROOT / POLICY_PARAMS))
    except YamlError as exc:
        errors.append(f"config parse error: {exc}")
        return errors, notes, command

    for path, expected in ACT_EXPECT.items():
        found, ok = _lookup(act, path)
        dotted = ".".join(path)
        if not ok:
            errors.append(f"{ACT_YAML.as_posix()} missing {dotted}")
        elif not _same(found, expected):
            errors.append(
                f"{ACT_YAML.as_posix()} {dotted} is {found!r}, runbook expects {expected!r}"
            )

    robot, ok = _lookup(robot_doc, ("robot",))
    if not ok or not isinstance(robot, dict):
        errors.append(f"{ROBOT_YAML.as_posix()} missing robot mapping")
        robot = {}
    if robot.get("name") != "omnibot_mobile_manip":
        errors.append(f"{ROBOT_YAML.as_posix()} robot.name is not omnibot_mobile_manip")
    for key in ("action_dim", "state_dim"):
        if robot.get(key) != 9:
            errors.append(f"{ROBOT_YAML.as_posix()} robot.{key} is not 9")
    cameras = robot.get("cameras")
    keys = set()
    if isinstance(cameras, list):
        keys = {item.get("key") for item in cameras if isinstance(item, dict)}
    missing_keys = [key for key in CAMERA_KEYS if key not in keys]
    if missing_keys:
        errors.append(
            f"{ROBOT_YAML.as_posix()} cameras missing {', '.join(missing_keys)}"
        )

    train_source = _read(ROOT / TRAIN_PY)
    try:
        train_tree = ast.parse(train_source)
    except SyntaxError as exc:
        errors.append(f"{TRAIN_PY.as_posix()} does not parse: {exc}")
        return errors, notes, command

    flags = _add_argument_flags(train_tree)
    for flag in TRAIN_FLAGS:
        if flag not in flags:
            errors.append(f"{TRAIN_PY.as_posix()} has no {flag} argument")

    loop = _epoch_loop(train_tree)
    if loop is None:
        errors.append(f"{TRAIN_PY.as_posix()} has no epoch loop over args.num_epochs")
    elif not _loop_enumerates_train_loader(loop) or not _loop_calls(loop, "save_checkpoint"):
        errors.append(
            f"{TRAIN_PY.as_posix()} epoch loop does not contain the train_loader "
            "pass and save_checkpoint. Refusing a shape that would bill a full "
            "ACT run for a single pass."
        )

    if not _make_policy_takes_chunk_size(train_tree):
        notes.append(
            "--chunk-size is recorded on the W&B config only. make_policy does not take chunk_size."
        )
    if not any(
        isinstance(node, ast.Call) and _call_name(node.func) == "CosineAnnealingLR"
        for node in ast.walk(train_tree)
    ):
        errors.append(f"{TRAIN_PY.as_posix()} no longer uses CosineAnnealingLR for the cosine schedule")
    else:
        notes.append("lr_scheduler cosine is CosineAnnealingLR in train.py. There is no --lr-scheduler flag.")

    try:
        launch_tree = ast.parse(_read(ROOT / INFERENCE_LAUNCH))
    except SyntaxError as exc:
        errors.append(f"{INFERENCE_LAUNCH.as_posix()} does not parse: {exc}")
        launch_tree = None
    if launch_tree is not None:
        launch_args = _launch_arguments(launch_tree)
        for name in ("model_type", "checkpoint", "device"):
            if name not in launch_args:
                errors.append(f"{INFERENCE_LAUNCH.as_posix()} does not declare {name}")
        if "policy_hz" not in launch_args:
            hz, ok = _lookup(params, ("policy_node", "ros__parameters", "policy_hz"))
            act_hz, _act_ok = _lookup(act, ("model", "policy_hz"))
            notes.append(
                "policy_inference.launch.py does not declare policy_hz. "
                f"policy_params.yaml policy_hz is {hz if ok else 'missing'}; "
                f"act.yaml policy_hz is {act_hz}. Set 20.0 on the robot checkout "
                "for the 20-trial session and leave that edit uncommitted."
            )

    runbook = _read(ROOT / RUNBOOK)
    for snippet in (
        NEED,
        "## 2. Budget checkpoint",
        "## 4. How to stop",
        "Ctrl+C",
        "--model act",
        "--checkpoint lerobot/act_base",
        "--num-epochs 200",
        "--batch-size 16",
        "--lr 1e-4",
        "--device cuda",
        "--chunk-size 100",
        "--save-every 20",
        "--grad-clip-norm 10.0",
        "--num-workers 4",
    ):
        if snippet not in runbook:
            errors.append(f"{RUNBOOK.as_posix()} is missing {snippet!r}")
    _check_results(_read(ROOT / RESULTS), errors)

    if dataset_path:
        _check_local_path("--dataset-path", dataset_path, errors)
    if output_dir:
        if _is_remote(output_dir):
            errors.append("--output-dir is a remote URI. This dry-run does not create it.")
        else:
            path = Path(output_dir).expanduser()
            if path.exists() and not path.is_dir():
                errors.append(f"--output-dir is not a directory: {path}")
            elif path.exists():
                notes.append(f"--output-dir exists and was not modified: {path}")
            else:
                notes.append(f"--output-dir is absent and was not created: {path}")

    if not errors and isinstance(act, dict) and "model" in act and "training" in act:
        command = _train_command(act)
    return errors, notes, command


def main(argv: list[str] | None = None) -> int:
    for key in LAUNCH_ENV:
        if _truthy(os.environ.get(key)):
            print(f"Refusing {key}. This script only validates config paths.", file=sys.stderr)
            print(NEED, file=sys.stderr)
            return 2

    parser = argparse.ArgumentParser(
        description="Validate OHH-52 ACT config paths. Does not start training."
    )
    parser.add_argument(
        "--dataset-path",
        default=None,
        help="Optional local dataset directory to stat. The directory is not read.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Optional output directory to report. It is not created.",
    )
    args, unknown = parser.parse_known_args(argv)
    if unknown:
        print(
            "Refusing extra arguments. This script only validates config paths: "
            + " ".join(unknown),
            file=sys.stderr,
        )
        print(NEED, file=sys.stderr)
        return 2

    if _truthy(os.environ.get("OHH52_BUDGET_APPROVED")):
        print("OHH52_BUDGET_APPROVED is set. Validation only; training stays stopped.")

    errors, notes, command = validate(args.dataset_path, args.output_dir)
    print("OHH-52 ACT config dry-run")
    print(NEED)
    print("Budget checkpoint: not cleared by this script.")
    if notes:
        print("Notes:")
        for note in notes:
            print(f"  - {note}")
    if errors:
        print("Dry-run failed. train.py was not started.")
        for error in errors:
            print(f"ERROR: {error}")
        return 1

    print("Config paths match the ACT runbook.")
    print("Training command (not started):")
    print(command)
    print("No cloud GPU was contacted. No robot command was sent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
