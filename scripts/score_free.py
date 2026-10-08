#!/usr/bin/env python3
"""Free-root row at one leg armature. Writes JSON through the live entry point.

python3 scripts/score_free.py 0.01 /tmp/arm/free010.json
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_live_row


def main(argv: list[str]) -> int:
    arm = float(argv[1])
    out = Path(argv[2]) if len(argv) > 2 else Path("/tmp/arm") / f"free_{arm:.3f}.json"
    forwarded = [
        "--t", "1.00",
        "--vx", "0.016",
        "--clearance", "0.008",
        "--dsp", "0.40",
        "--ff", "1",
        "--knee-qdd-cap", "40",
        "--armature", str(arm),
        "--seed", "none",
        "--latency", "0",
        "--mass-scale", "1",
        "--mu", "none",
        "--arm-s", "1.0",
        "--stand-s", "0.25",
        "--walk-s", "4.0",
        "--stop-s", "2.5",
        "--preview-shape", "1.0",
        "--amp", "0.016",
        "--z-quintic", "0",
        "--z-lead", "0",
        "--honor-vx", "0",
        "--out", str(out),
    ]
    return run_live_row.main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
