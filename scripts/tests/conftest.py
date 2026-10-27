"""Put this repo's packages ahead of any other checkout on sys.path."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for rel in ("scripts", "omnibot-ai-engines", "yahboom-python-driver"):
    entry = str(ROOT / rel)
    if entry not in sys.path:
        sys.path.insert(0, entry)
