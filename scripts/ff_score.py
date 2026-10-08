#!/usr/bin/env python3
"""Historical feedforward row. The positional arguments match /tmp/ff_score.py.

T vx [clearance dsp preview_arm stand walk stop amp honor ff]
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_live_row
import zmp_preview


def main(argv: list[str]) -> int:
    t = float(argv[1])
    vx = float(argv[2])
    z = float(argv[3]) if len(argv) > 3 else 0.008
    dsp = float(argv[4]) if len(argv) > 4 else 0.25
    arm = float(argv[5]) if len(argv) > 5 else 1.0
    stand = float(argv[6]) if len(argv) > 6 else 0.25
    walk = float(argv[7]) if len(argv) > 7 else (1.0 + 2.05 * t)
    stop = float(argv[8]) if len(argv) > 8 else 2.40
    amp_arg = float(argv[9]) if len(argv) > 9 else -1.0
    honor = int(argv[10]) if len(argv) > 10 else 1
    ff = int(argv[11]) if len(argv) > 11 else 1
    if amp_arg < 0.0:
        amp = zmp_preview.sway_zmp_amp(t, dsp, 0.18, 0.016, 0.043, 0.0, 1.0e-4)
    else:
        amp = amp_arg
    out = Path("/tmp") / f"ff_score_{t:.2f}_{vx:.3f}.json"
    forwarded = [
        "--t", str(t),
        "--vx", str(vx),
        "--clearance", str(z),
        "--dsp", str(dsp),
        "--ff", str(ff),
        "--knee-qdd-cap", "40",
        "--armature", "0.01",
        "--seed", "none",
        "--latency", "0",
        "--mass-scale", "1",
        "--mu", "none",
        "--arm-s", str(arm),
        "--stand-s", str(stand),
        "--walk-s", str(walk),
        "--stop-s", str(stop),
        "--preview-shape", "0",
        "--amp", str(amp),
        "--z-quintic", "1",
        "--z-lead", "1",
        "--honor-vx", str(honor),
        "--out", str(out),
    ]
    return run_live_row.main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
