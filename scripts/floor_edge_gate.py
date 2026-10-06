#!/usr/bin/env python3
"""Floor-edge arm gate. Does not send Day-1 stop.

The margin was written down before any run of this gate:

    arm_threshold_m = mid_swing_toe_min_m − FLOOR_EDGE_MARGIN_M
    FLOOR_EDGE_MARGIN_M = 0.002   # 2 mm

The arm input is the mid-swing toe minimum, 20–80% of each swing.
Controls printed that sample. ``MID_SWING_TOE_MIN_M`` is −0.003066 m,
384 ticks, minimum at t = 3.384 s. That minimum is a loaded floor
contact inside the mid-swing window. It matches the whole-swing
minimum, so the scuff is real. It is not a lift-off or touchdown
artifact.

Subtracting 2 mm leaves −0.005066 m. That threshold is negative. It
is not installed. It would arm on the 0.012 m rug and on every taller
edge. The shipped call raises ``ScuffGait``. It does not return None
or False.

Controls' walk Prefer FAIL on the rug is 7.016 s to 7.560 s. The body
is pitched over the edge. That window is not this stop gate.

The rug height report is computed before any arm call. Mono at the
stand eye, 0.335 m, looking at a 0.012 m edge 0.30 m ahead, moves the
floor meet of the top-edge ray by about 1 cm. The finder eye bias on
the firing frame is −0.011 m. Those two are the same order. The
height interval is reported next to the true 0.012 m. When the
interval still contains that height, the estimate does not arm a stop
and the error bar is not tightened to make it separate.

Whole-swing p90 is 0.025967 m. Mid-swing p90 is 0.024671 m. Both are
report-only. Neither is subtracted.

This module does not call ``CommandBus``. It does not edit the plant.
``HAZARD_PAD_M`` stays 0.020. The 3–5 cm buffer stays off. Head tilt
stays off. Soft-pass is off. Not kit-safe. Not go-anywhere.
"""
from __future__ import annotations

import hashlib
import inspect
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import ray_corridor as rc

# Locked before the run. Two millimetres. Not a clearance.
FLOOR_EDGE_MARGIN_MM = 2
FLOOR_EDGE_MARGIN_M = FLOOR_EDGE_MARGIN_MM / 1000.0

# Entrance straight, whole swing, including lift-off and touchdown.
# Same value as the locked mid-swing min. Real scuff.
WHOLE_SWING_TOE_MIN_M = -0.003066

# Arithmetic only. Negative. Not installed as a threshold.
WHOLE_SWING_ARM_M = WHOLE_SWING_TOE_MIN_M - FLOOR_EDGE_MARGIN_M

# Locked. 20–80% of each swing, 384 ticks, min at t = 3.384 s.
# Loaded floor contact inside the window. Not a clearance that arms.
MID_SWING_TOE_MIN_M = -0.003066
MID_SWING_TICKS = 384
MID_SWING_MIN_T_S = 3.384
MID_SWING_ARM_M = MID_SWING_TOE_MIN_M - FLOOR_EDGE_MARGIN_M

# Report only. Not the arm.
WHOLE_SWING_P90_REPORT_M = 0.025967
MID_SWING_P90_REPORT_M = 0.024671

# Controls walk Prefer FAIL. Body pitched over the rug edge.
# Not this stop gate.
RUG_BODY_PITCH_T0_S = 7.016
RUG_BODY_PITCH_T1_S = 7.560

# Measured walk top of col_mat_rug. Honesty reference. Not an arm input.
RUG_TOP_M = 0.012

# Stand eye and the Hardware range used for the mono error.
# Not a measured rug range from this process, and not tuned.
STAND_CAM_Z_M = 0.335
STAND_PITCH_DEG = -14.84
EDGE_FORWARD_M = 0.30

# Firing-frame finder eye minus true range, T_detect = 0. Not tuned.
FINDER_EYE_BIAS_M = -0.011

# Probe min above the margin. Not a Controls print and not installed.
PROBE_SWING_TOE_MIN_M = 0.040

PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
_PLANT = (
    Path(__file__).resolve().parents[1]
    / "mujoco"
    / "ainex_hiwonder"
    / "ainex_controls_m2_145.xml"
)


class ScuffGait(RuntimeError):
    """Mid-swing toe min is at or under the 2 mm margin. Prefer FAIL."""


@dataclass(frozen=True)
class EdgeHeightReport:
    """Kit_cam mono height of the 12 mm edge, printed before any arm."""

    true_m: float
    estimate_m: float
    err_m: float
    lo_m: float
    hi_m: float
    floor_shift_m: float
    range_err_m: float
    cam_z_m: float
    forward_m: float
    overlaps: bool


def arm_threshold_m(swing_toe_min_m: float | None) -> float | None:
    """Mid-swing min minus 2 mm.

    None is only an explicit missing sample. The shipped mid-swing min
    is locked and is not None. A finite min at or under the margin
    raises ``ScuffGait``. p90 is not this argument.
    """
    if swing_toe_min_m is None:
        return None
    minimum = float(swing_toe_min_m)
    if minimum != minimum or minimum <= FLOOR_EDGE_MARGIN_M:
        raise ScuffGait(
            f"swing-toe min {minimum:.6f} m <= margin {FLOOR_EDGE_MARGIN_M:.3f} m; "
            "gait Prefer FAIL scuff"
        )
    threshold = minimum - FLOOR_EDGE_MARGIN_M
    if threshold <= 0.0:
        raise ScuffGait(
            f"arm {threshold:.6f} m is not positive; it would fire on every edge"
        )
    return threshold


def formula_arms(edge_height_m: float, swing_toe_min_m: float | None) -> bool:
    """True only when the edge is strictly taller than min − 2 mm.

    A min at or under 2 mm raises. None refuses. This does not latch
    ``stop``. The shipped walk does not call it.
    """
    threshold = arm_threshold_m(swing_toe_min_m)
    if threshold is None:
        return False
    return float(edge_height_m) > threshold


def floor_edge_stop_armed(edge_height_m: float | None) -> bool:
    """Shipped arm. The locked mid-swing min raises ``ScuffGait``.

    The negative min − 2 mm threshold is not installed, so this does
    not return true for the rug or for any other edge. It does not
    send ``stop``. p90 is not read.
    """
    threshold = arm_threshold_m(MID_SWING_TOE_MIN_M)
    if threshold is None or edge_height_m is None:
        return False
    return float(edge_height_m) > threshold


def negative_threshold_arms(edge_height_m: float, threshold_m: float) -> bool:
    """A non-positive threshold does not arm.

    The whole-swing arithmetic −0.005066 m would be true for the rug
    if it were compared. This function does not perform that compare.
    """
    if float(threshold_m) <= 0.0:
        return False
    return float(edge_height_m) > float(threshold_m)


def _camera() -> tuple[np.ndarray, np.ndarray]:
    pitch = math.radians(STAND_PITCH_DEG)
    rotation = rc.camera_rotation_from_imu(0.0, pitch, 0.0, 0.0, 0.0)
    origin = np.array([0.0, 0.0, STAND_CAM_Z_M], dtype=np.float64)
    return origin, rotation


def _ray_direction(u: float, v: float, rotation: np.ndarray) -> np.ndarray:
    return np.asarray(rotation, dtype=np.float64).reshape(3, 3) @ rc.pixel_direction(u, v)


def _z_at_forward(
    u: float,
    v: float,
    origin: np.ndarray,
    rotation: np.ndarray,
    forward_m: float,
) -> float:
    direction = _ray_direction(u, v, rotation)
    ahead = float(direction[0])
    if ahead <= 1e-8:
        raise RuntimeError("top-edge ray does not travel forward")
    travel = (float(forward_m) - float(origin[0])) / ahead
    if travel <= 0.0:
        raise RuntimeError("top-edge ray meets the range behind the camera")
    return float(origin[2] + travel * float(direction[2]))


def report_rug_height() -> EdgeHeightReport:
    """Mono height of a 0.012 m edge at 0.30 m. Does not arm.

    The point estimate rays the top-edge pixel and reads z at the
    floor-contact range. The error bar moves that range by the
    firing-frame finder eye bias, −0.011 m. The floor-meet shift of
    the same top-edge ray down to z = 0 is reported beside it.
    """
    origin, rotation = _camera()
    base = np.array([EDGE_FORWARD_M, 0.0, 0.0], dtype=np.float64)
    top = np.array([EDGE_FORWARD_M, 0.0, RUG_TOP_M], dtype=np.float64)
    pix_base = rc.project_point(base, origin, rotation)
    pix_top = rc.project_point(top, origin, rotation)
    if pix_base is None or pix_top is None:
        raise RuntimeError("rug edge is behind kit_cam")
    hit = rc.ray_floor_hit(pix_top[0], pix_top[1], origin, rotation)
    if hit is None:
        raise RuntimeError("top-edge ray missed the floor")
    floor_shift = float(hit[0] - EDGE_FORWARD_M)
    estimate = _z_at_forward(pix_top[0], pix_top[1], origin, rotation, EDGE_FORWARD_M)
    range_err = abs(FINDER_EYE_BIAS_M)
    low = _z_at_forward(
        pix_top[0], pix_top[1], origin, rotation, EDGE_FORWARD_M - range_err,
    )
    high = _z_at_forward(
        pix_top[0], pix_top[1], origin, rotation, EDGE_FORWARD_M + range_err,
    )
    err = max(abs(estimate - low), abs(estimate - high))
    lo = estimate - err
    hi = estimate + err
    # The bar still contains the true 12 mm height, and it is wider
    # than the 2 mm margin, so walk and stop are not separated.
    overlaps = lo < RUG_TOP_M < hi and err > FLOOR_EDGE_MARGIN_M
    return EdgeHeightReport(
        true_m=RUG_TOP_M,
        estimate_m=estimate,
        err_m=err,
        lo_m=lo,
        hi_m=hi,
        floor_shift_m=floor_shift,
        range_err_m=range_err,
        cam_z_m=STAND_CAM_Z_M,
        forward_m=EDGE_FORWARD_M,
        overlaps=overlaps,
    )


def shipped_scuff_raised(edge_height_m: float | None) -> bool:
    """True when the shipped gate raises. A returned bool is not a raise."""
    try:
        floor_edge_stop_armed(edge_height_m)
    except ScuffGait:
        return True
    return False


def _expect_scuff(swing_toe_min_m: float) -> None:
    try:
        arm_threshold_m(swing_toe_min_m)
    except ScuffGait:
        return
    raise SystemExit(f"min {swing_toe_min_m} returned a threshold")


def self_check() -> EdgeHeightReport:
    """Height report first. Then the scuff raise. Then the refused arm."""
    if FLOOR_EDGE_MARGIN_MM != 2:
        raise SystemExit(f"margin moved to {FLOOR_EDGE_MARGIN_MM} mm")
    if abs(FLOOR_EDGE_MARGIN_M - 0.002) > 1e-15:
        raise SystemExit(f"margin metres {FLOOR_EDGE_MARGIN_M}")
    report = report_rug_height()
    if abs(report.true_m - 0.012) > 1e-15:
        raise SystemExit(f"true height {report.true_m}")
    if abs(report.estimate_m - 0.012) > 1e-6:
        raise SystemExit(f"point estimate {report.estimate_m} left 0.012")
    if report.err_m <= FLOOR_EDGE_MARGIN_M:
        raise SystemExit(f"error {report.err_m} was tightened under the margin")
    if not report.overlaps:
        raise SystemExit("12 mm height error no longer overlaps the true height")
    if report.lo_m >= report.true_m or report.hi_m <= report.true_m:
        raise SystemExit("interval does not contain 0.012")
    # Height interval is on the record before any arm or scuff call.
    print(format_height(report), flush=True)
    if abs(WHOLE_SWING_TOE_MIN_M - (-0.003066)) > 1e-12:
        raise SystemExit("whole-swing print moved")
    if abs(WHOLE_SWING_ARM_M - (-0.005066)) > 1e-12:
        raise SystemExit(f"whole-swing arm arithmetic {WHOLE_SWING_ARM_M}")
    if WHOLE_SWING_ARM_M >= 0.0 or WHOLE_SWING_ARM_M >= RUG_TOP_M:
        raise SystemExit("whole-swing arm is not below the rug")
    if abs(MID_SWING_TOE_MIN_M - (-0.003066)) > 1e-12:
        raise SystemExit("mid-swing min moved")
    if abs(MID_SWING_TOE_MIN_M - WHOLE_SWING_TOE_MIN_M) > 1e-12:
        raise SystemExit("mid-swing and whole-swing mins diverged")
    if MID_SWING_TICKS != 384 or abs(MID_SWING_MIN_T_S - 3.384) > 1e-12:
        raise SystemExit("mid-swing sample tag moved")
    if abs(MID_SWING_ARM_M - (-0.005066)) > 1e-12:
        raise SystemExit(f"mid-swing arm arithmetic {MID_SWING_ARM_M}")
    if MID_SWING_ARM_M >= 0.0:
        raise SystemExit("mid-swing arm is not negative")
    if abs(WHOLE_SWING_P90_REPORT_M - 0.025967) > 1e-12:
        raise SystemExit("whole-swing p90 moved")
    if abs(MID_SWING_P90_REPORT_M - 0.024671) > 1e-12:
        raise SystemExit("mid-swing p90 moved")
    if abs(RUG_BODY_PITCH_T0_S - 7.016) > 1e-12 or abs(RUG_BODY_PITCH_T1_S - 7.560) > 1e-12:
        raise SystemExit("rug pitch window moved")
    # The negative arithmetic would pass a plain compare. It is not used.
    if not (RUG_TOP_M > MID_SWING_ARM_M):
        raise SystemExit("rug is not above the unused negative arithmetic")
    if negative_threshold_arms(RUG_TOP_M, MID_SWING_ARM_M):
        raise SystemExit("negative threshold armed the rug")
    if not shipped_scuff_raised(RUG_TOP_M):
        raise SystemExit("locked mid-swing did not raise on the rug")
    if not shipped_scuff_raised(None):
        raise SystemExit("locked mid-swing did not raise on an empty edge")
    if not shipped_scuff_raised(report.hi_m):
        raise SystemExit("locked mid-swing did not raise on the height bar")
    _expect_scuff(MID_SWING_TOE_MIN_M)
    _expect_scuff(WHOLE_SWING_TOE_MIN_M)
    _expect_scuff(FLOOR_EDGE_MARGIN_M)
    _expect_scuff(0.0)
    if arm_threshold_m(None) is not None:
        raise SystemExit("unset mid-swing produced a threshold")
    probe = arm_threshold_m(PROBE_SWING_TOE_MIN_M)
    expect = PROBE_SWING_TOE_MIN_M - FLOOR_EDGE_MARGIN_M
    if probe is None or abs(probe - expect) > 1e-12:
        raise SystemExit(f"probe threshold {probe}")
    if formula_arms(RUG_TOP_M, PROBE_SWING_TOE_MIN_M):
        raise SystemExit("probe min armed the 12 mm height")
    if formula_arms(probe, PROBE_SWING_TOE_MIN_M):
        raise SystemExit("edge equal to min − 2 mm armed")
    if not formula_arms(probe + 1e-4, PROBE_SWING_TOE_MIN_M):
        raise SystemExit("edge taller than a probe min did not pass the formula")
    if formula_arms(RUG_TOP_M, None):
        raise SystemExit("formula armed with no mid-swing min")
    armed_src = inspect.getsource(floor_edge_stop_armed)
    if "P90" in armed_src or "0.025967" in armed_src or "0.024671" in armed_src:
        raise SystemExit("shipped arm reads a p90")
    if "7.016" in armed_src or "7.560" in armed_src:
        raise SystemExit("shipped arm reads the Controls pitch window")
    if abs(rc.HAZARD_PAD_M - 0.020) > 1e-12:
        raise SystemExit(f"pad moved to {rc.HAZARD_PAD_M}")
    digest = hashlib.md5(_PLANT.read_bytes()).hexdigest()
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {PLANT_MD5}")
    return report


def format_height(report: EdgeHeightReport) -> str:
    """Height line. Does not call the arm."""
    return (
        f"BEFORE_ARM rug_height true={report.true_m:.3f} "
        f"estimate={report.estimate_m:.6f} "
        f"err={report.err_m:.6f} "
        f"interval=[{report.lo_m:.6f},{report.hi_m:.6f}] "
        f"floor_shift={report.floor_shift_m:.6f} "
        f"range_err={report.range_err_m:.3f} "
        f"cam_z={report.cam_z_m:.3f} "
        f"forward={report.forward_m:.2f} "
        f"overlaps={report.overlaps}"
    )


if __name__ == "__main__":
    # self_check prints the height line before it calls the arm.
    checked = self_check()
    print(
        f"AFTER_ARM whole_swing={WHOLE_SWING_TOE_MIN_M:.6f} "
        f"mid_swing={MID_SWING_TOE_MIN_M:.6f} "
        f"ticks={MID_SWING_TICKS} t_min={MID_SWING_MIN_T_S:.3f} "
        f"arm_unused={MID_SWING_ARM_M:.6f} "
        f"raised={shipped_scuff_raised(checked.true_m)} "
        f"rug_stop=False "
        f"mid_p90_report={MID_SWING_P90_REPORT_M:.6f} "
        f"whole_p90_report={WHOLE_SWING_P90_REPORT_M:.6f} "
        f"rug_pitch={RUG_BODY_PITCH_T0_S:.3f}-{RUG_BODY_PITCH_T1_S:.3f} "
        f"pad={rc.HAZARD_PAD_M:.3f} plant={PLANT_MD5} "
        f"prefer_fail=scuff_raises arm_refused not_go_anywhere"
    )
