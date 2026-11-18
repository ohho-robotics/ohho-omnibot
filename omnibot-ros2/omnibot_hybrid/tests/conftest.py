"""Put omnibot_hybrid on sys.path without initialising rclpy."""

from __future__ import annotations

import sys
from pathlib import Path

_TESTS = Path(__file__).resolve().parent
_ROOT = _TESTS.parent
for _path in (_TESTS, _ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
