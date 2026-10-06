#!/usr/bin/env python3
"""Floor-edge arm gate. Does not send Day-1 stop.

Controls+MFG locked the formula. The margin was written down before
any run of this gate:

    arm_threshold_m = swing_toe_min_m − FLOOR_EDGE_MARGIN_M
    FLOOR_EDGE_MARGIN_M = 0.002   # 2 mm

``swing_toe_min_m`` is the Controls print of swing-toe minimum on the
entrance straight bout. That print has not landed. ``SWING_TOE_MIN_M``
stays None. Prefer FAIL / not locked. While it is None this gate
refuses every floor-edge stop, including the measured ``col_mat_rug``
top of 0.012 m.

That rug is a walk / step-on. The recorded hit is the left sole at
6.892 s, 11.28 N, in double support. Refusing the stop does not claim
the foot missed the rug, and it does not score ankle torque, half-edge
plant, or CoP. Those belong to Controls.

p90 is report-only. Hardware's sim swing sole p90 of about 1.73 cm is
not subtracted and is not an argument of the arm. The ~1 cm slogan is
not the arm.

The Day-1 wood-leg latch is unchanged. ``too_close`` is a different
flag and is not cleared here. This module does not call ``CommandBus``.
It does not read a geom out of XML. The 0.012 m figure is the measured
contact top, not the vision-box size. ``HAZARD_PAD_M`` stays 0.020.
The 3–5 cm buffer stays off. Head tilt stays off. Soft-pass is off.
Not kit-safe. Not go-anywhere.

No taller sill is armed. ``col_entrance_sill`` stays a wall. There is
no positive floor-edge stop in this gate.
"""
from __future__ import annotations

import hashlib
import inspect
from pathlib import Path

import ray_corridor as rc

# Locked before the run. Two millimetres. Not a clearance.
FLOOR_EDGE_MARGIN_MM = 2
FLOOR_EDGE_MARGIN_M = FLOOR_EDGE_MARGIN_MM / 1000.0

# Controls has not printed swing-toe min from the entrance bout.
# None shuts the arm. Prefer FAIL / not locked.
SWING_TOE_MIN_M: float | None = None

# Measured walk top of col_mat_rug. Step-on. Not read from the vision box.
RUG_TOP_M = 0.012

# Hardware sim swing sole p90, report only. Not the arm.
SWING_SOLE_P90_REPORT_M = 0.0173

# Arithmetic stand-in. Not the Controls print and not locked.
PROBE_SWING_TOE_MIN_M = 0.040

PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
_PLANT = (
    Path(__file__).resolve().parents[1]
    / "mujoco"
    / "ainex_hiwonder"
    / "ainex_controls_m2_145.xml"
)


def arm_threshold_m(swing_toe_min_m: float | None) -> float | None:
    """Swing-toe min minus 2 mm. None while the min is unprinted.

    A non-finite min, or a min that does not clear the margin, also
    stays shut. p90 is not accepted as this argument by the shipped arm.
    """
    if swing_toe_min_m is None:
        return None
    minimum = float(swing_toe_min_m)
    if minimum != minimum or minimum <= FLOOR_EDGE_MARGIN_M:
        return None
    return minimum - FLOOR_EDGE_MARGIN_M


def formula_arms(edge_height_m: float, swing_toe_min_m: float | None) -> bool:
    """True only when the edge is strictly taller than min − 2 mm.

    This is the formula. The shipped walk does not call it. Passing a
    guessed min here does not latch ``stop``.
    """
    threshold = arm_threshold_m(swing_toe_min_m)
    if threshold is None:
        return False
    return float(edge_height_m) > threshold


def floor_edge_stop_armed(edge_height_m: float | None) -> bool:
    """Shipped arm. False until Controls prints swing-toe min.

    ``edge_height_m`` None is an empty approach: no floor-edge stop.
    The 0.012 m rug is refused while the min is unset. A taller number
    is refused for the same reason. This does not send ``stop``.
    """
    threshold = arm_threshold_m(SWING_TOE_MIN_M)
    if threshold is None or edge_height_m is None:
        return False
    return float(edge_height_m) > threshold


def self_check() -> None:
    """Margin is 2 mm. Rug does not arm. Plant hash is cold."""
    if FLOOR_EDGE_MARGIN_MM != 2:
        raise SystemExit(f"margin moved to {FLOOR_EDGE_MARGIN_MM} mm")
    if abs(FLOOR_EDGE_MARGIN_M - 0.002) > 1e-15:
        raise SystemExit(f"margin metres {FLOOR_EDGE_MARGIN_M}")
    if SWING_TOE_MIN_M is not None:
        raise SystemExit(
            f"swing-toe min {SWING_TOE_MIN_M} locked without a Controls print"
        )
    if arm_threshold_m(SWING_TOE_MIN_M) is not None:
        raise SystemExit("unset min produced a threshold")
    if floor_edge_stop_armed(RUG_TOP_M):
        raise SystemExit("12 mm rug armed a floor-edge stop")
    if floor_edge_stop_armed(None):
        raise SystemExit("empty approach armed a floor-edge stop")
    if floor_edge_stop_armed(0.20):
        raise SystemExit("tall edge armed while swing-toe min is unset")
    if abs(RUG_TOP_M - 0.012) > 1e-15:
        raise SystemExit(f"rug top moved to {RUG_TOP_M}")
    armed_src = inspect.getsource(floor_edge_stop_armed)
    if "P90" in armed_src or "0.0173" in armed_src:
        raise SystemExit("p90 leaked into the shipped arm")
    if abs(rc.HAZARD_PAD_M - 0.020) > 1e-12:
        raise SystemExit(f"pad moved to {rc.HAZARD_PAD_M}")
    # Probe arithmetic only. 0.040 m is not the entrance bout.
    probe_threshold = arm_threshold_m(PROBE_SWING_TOE_MIN_M)
    expect = PROBE_SWING_TOE_MIN_M - FLOOR_EDGE_MARGIN_M
    if probe_threshold is None or abs(probe_threshold - expect) > 1e-12:
        raise SystemExit(f"probe threshold {probe_threshold} != {expect}")
    if formula_arms(RUG_TOP_M, PROBE_SWING_TOE_MIN_M):
        raise SystemExit("probe armed the 12 mm rug")
    if formula_arms(probe_threshold, PROBE_SWING_TOE_MIN_M):
        raise SystemExit("edge equal to min − 2 mm armed")
    if not formula_arms(probe_threshold + 1e-4, PROBE_SWING_TOE_MIN_M):
        raise SystemExit("edge taller than min − 2 mm did not pass the formula")
    if formula_arms(RUG_TOP_M, None):
        raise SystemExit("formula armed with no min")
    digest = hashlib.md5(_PLANT.read_bytes()).hexdigest()
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {PLANT_MD5}")


if __name__ == "__main__":
    self_check()
    print(
        f"self_check ok margin_mm={FLOOR_EDGE_MARGIN_MM} "
        f"swing_toe_min={SWING_TOE_MIN_M} rug_top_m={RUG_TOP_M:.3f} "
        f"rug_stop={floor_edge_stop_armed(RUG_TOP_M)} "
        f"empty_stop={floor_edge_stop_armed(None)} "
        f"p90_report_m={SWING_SOLE_P90_REPORT_M:.4f} "
        f"pad={rc.HAZARD_PAD_M:.3f} plant={PLANT_MD5} "
        f"prefer_fail=refuse_12mm_rug not_go_anywhere"
    )
