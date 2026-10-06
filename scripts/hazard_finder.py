#!/usr/bin/env python3
"""RGB finder for a stool-leg floor pixel. No Day-1 stop.

The ray in ``ray_corridor.estimate_hazard`` needs a kit_cam pixel. This
module returns that pixel from the RGB buffer. It does not project a
geom, and it does not call ``CommandBus``.

The wood test is the frozen #65 color rule (R > 90, R > G + 8, G > B,
R < 210, saturation between 0.15 and 0.75). On this kitchen that rule
paints the floor. The stool legs render dark, so a leg pixel is a
low-luminance, low-chroma pixel that is not wood. A leg column is a
narrow vertical run of those pixels whose lowest pixel sits on wood.

The returned ``(u, v)`` is that lowest pixel, the place the leg meets
the floor. It is not the blob centroid and not the box centre. A
centroid sits up the leg; the ray from there meets the floor past the
leg and the range reads long.

If that lowest pixel is the last row of the frame, the contact is cut
off. The cue is ``bottom_clipped`` / ``too_close`` and ``u`` and ``v``
are left empty. No pixel is emitted, so the ray cannot report a long
range. Controls latches Day-1 ``stop`` on that flag. This module does
not send ``stop``.

Moondream's room ask does not return a pixel. It is not called here.
``t_cue`` is not ``T_detect``. ``HAZARD_PAD_M`` stays 0.020 and is
applied only inside ``estimate_hazard``. Head tilt is not commanded.
The 3–5 cm buffer is not applied. Soft-pass is off.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import numpy as np

import ray_corridor as rc

# Frozen #65 wood color. The 0.30 fraction and the lower-right rectangle
# were a scene cue. They are not a pixel, and they are not retuned here.
WOOD_R_MIN = 90.0
WOOD_R_DELTA = 8.0
WOOD_R_MAX = 210.0
WOOD_SAT_MIN = 0.15
WOOD_SAT_MAX = 0.75
# Dark leg against that floor. Read from kit_cam appearance of the stool
# mesh, then left fixed for the walk grid.
LEG_LUM_MAX = 90
LEG_CHROMA_MAX = 48
LEG_R_MAX = 100
LEG_SPAN_MIN = 24
LEG_CLIP_SPAN_MIN = 40
LEG_WIDTH_MIN = 3
LEG_WIDTH_MAX = 48
COLUMN_GAP = 12
ROW_JOIN = 56
CLIP_SPLIT = 8
WOOD_GAP_PX = 3
SOURCE = "rgb_wood_leg"


@dataclass(frozen=True)
class HazardCue:
    """One leg-like blob.

    ``u`` and ``v`` are the ray pixel: the lowest leg pixel in the blob's
    column. Both are ``None`` when ``bottom_clipped`` is set. ``contact_row``
    is that same row, including the frame edge, and is not a ray input.
    ``too_close`` is the flag Controls stops on. It matches ``bottom_clipped``.
    """

    u: float | None
    v: float | None
    bottom_clipped: bool
    too_close: bool
    column: int
    contact_row: int
    span_px: int
    width_px: int
    source: str


@dataclass(frozen=True)
class FinderHazard:
    """Primary near-floor cue, plus the ray when a pixel was emitted."""

    cue: HazardCue
    estimate: rc.HazardEstimate | None


@dataclass(frozen=True)
class _Hit:
    column: int
    row: int
    span: int
    clipped: bool


def wood_mask(rgb: np.ndarray) -> np.ndarray:
    """#65 wood pixels over the whole frame. RGB only."""
    image = np.asarray(rgb)
    red = image[:, :, 0].astype(np.float32)
    green = image[:, :, 1].astype(np.float32)
    blue = image[:, :, 2].astype(np.float32)
    hi = np.maximum(np.maximum(red, green), blue)
    lo = np.minimum(np.minimum(red, green), blue)
    sat = (hi - lo) / np.maximum(hi, 1.0)
    mask = (red > WOOD_R_MIN) & (red > green + WOOD_R_DELTA) & (green > blue)
    mask &= red < WOOD_R_MAX
    mask &= (sat > WOOD_SAT_MIN) & (sat < WOOD_SAT_MAX)
    return mask


def leg_mask(rgb: np.ndarray, wood: np.ndarray | None = None) -> np.ndarray:
    """Dark stool-leg pixels. Wood floor pixels are excluded."""
    image = np.asarray(rgb)
    if wood is None:
        wood = wood_mask(image)
    red = image[:, :, 0].astype(np.int16)
    green = image[:, :, 1].astype(np.int16)
    blue = image[:, :, 2].astype(np.int16)
    hi = np.maximum(np.maximum(red, green), blue)
    lo = np.minimum(np.minimum(red, green), blue)
    lum = (red.astype(np.int32) + green.astype(np.int32) + blue.astype(np.int32)) // 3
    dark = (lum < LEG_LUM_MAX) & ((hi - lo) < LEG_CHROMA_MAX) & (red < LEG_R_MAX)
    return dark & ~wood


def _on_wood(wood: np.ndarray, row: int, column: int) -> bool:
    height = int(wood.shape[0])
    for gap in range(1, WOOD_GAP_PX + 1):
        below = row + gap
        if below >= height:
            return False
        if bool(wood[below, column]):
            return True
    return False


def _column_hits(dark: np.ndarray, wood: np.ndarray) -> list[_Hit]:
    height = int(dark.shape[0])
    width = int(dark.shape[1])
    edge = height - 1
    hits: list[_Hit] = []
    for column in range(width):
        column_px = dark[:, column]
        rows = np.flatnonzero(column_px)
        if rows.size == 0:
            continue
        row = int(rows[-1])
        top = row
        while top > 0 and bool(column_px[top - 1]):
            top -= 1
        span = row - top + 1
        clipped = row >= edge and not _on_wood(wood, row, column)
        if clipped and span >= LEG_CLIP_SPAN_MIN:
            hits.append(_Hit(column, row, span, True))
            continue
        if span >= LEG_SPAN_MIN and _on_wood(wood, row, column):
            hits.append(_Hit(column, row, span, False))
    return hits


def _same_leg(group: list[_Hit], hit: _Hit) -> bool:
    prev = group[-1]
    if hit.column - prev.column > COLUMN_GAP:
        return False
    rows = sorted(item.row for item in group)
    mid = rows[len(rows) // 2]
    if abs(hit.row - mid) > ROW_JOIN:
        return False
    if hit.clipped != prev.clipped and abs(hit.row - prev.row) > CLIP_SPLIT:
        return False
    return True


def _cue_from_group(group: list[_Hit]) -> HazardCue | None:
    width = group[-1].column - group[0].column + 1
    if width < LEG_WIDTH_MIN or width > LEG_WIDTH_MAX:
        return None
    chosen = max(group, key=lambda item: (item.row, item.span))
    span = max(item.span for item in group)
    clipped = bool(chosen.clipped)
    # Pixel centre of the lowest leg pixel in the chosen column.
    # Empty when that pixel is the frame edge.
    u: float | None = None if clipped else float(chosen.column) + 0.5
    v: float | None = None if clipped else float(chosen.row) + 0.5
    return HazardCue(
        u=u,
        v=v,
        bottom_clipped=clipped,
        too_close=clipped,
        column=int(chosen.column),
        contact_row=int(chosen.row),
        span_px=int(span),
        width_px=int(width),
        source=SOURCE,
    )


def find_hazard_cues(rgb: np.ndarray) -> tuple[HazardCue, ...]:
    """Every leg-like floor contact in the frame, nearest not preferred.

    A bottom-clipped cue is included with ``u`` and ``v`` empty. Callers
    that ray only ``u, v`` must still stop when ``too_close`` is set.
    """
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError("kit_cam RGB must be HxWx3")
    if image.shape[0] != rc.HEIGHT or image.shape[1] != rc.WIDTH:
        raise ValueError(f"kit_cam RGB must be {rc.WIDTH}x{rc.HEIGHT}")
    wood = wood_mask(image)
    dark = leg_mask(image, wood)
    hits = _column_hits(dark, wood)
    if not hits:
        return ()
    ordered = sorted(hits, key=lambda item: item.column)
    groups: list[list[_Hit]] = [[ordered[0]]]
    for hit in ordered[1:]:
        if _same_leg(groups[-1], hit):
            groups[-1].append(hit)
        else:
            groups.append([hit])
    cues: list[HazardCue] = []
    for group in groups:
        cue = _cue_from_group(group)
        if cue is not None:
            cues.append(cue)
    cues.sort(key=lambda cue: cue.contact_row, reverse=True)
    return tuple(cues)


def find_hazard_pixel(rgb: np.ndarray) -> HazardCue | None:
    """The near-floor cue: the lowest contact in the frame.

    If that blob is cut off at the bottom edge, the result is
    ``too_close`` and carries no ray pixel. A higher blob is not
    substituted. That substitution would be a long range.
    """
    cues = find_hazard_cues(rgb)
    if not cues:
        return None
    return cues[0]


def estimate_finder_hazard(
    rgb: np.ndarray,
    *,
    cam: rc.KitCamPose,
    body: rc.BodyFrame,
    imu_roll_rad: float,
    imu_pitch_rad: float,
    head_tilt_rad: float,
    yaw_rate: float,
    step_off_m: float,
    hazard_pad_m: float = rc.HAZARD_PAD_M,
    head_pan_rad: float = 0.0,
) -> FinderHazard | None:
    """Near-floor finder pixel passed through ``estimate_hazard``.

    Returns None when the frame has no leg cue. When the near cue is
    bottom-clipped, ``estimate`` is None and ``too_close`` is true.
    This does not command ``stop``.
    """
    if abs(float(hazard_pad_m) - rc.HAZARD_PAD_M) > 1e-12:
        raise ValueError(
            f"hazard pad {hazard_pad_m} is not the frozen {rc.HAZARD_PAD_M}"
        )
    cue = find_hazard_pixel(rgb)
    if cue is None:
        return None
    if cue.too_close or cue.u is None or cue.v is None:
        return FinderHazard(cue, None)
    estimate = rc.estimate_hazard(
        cue.u,
        cue.v,
        cam=cam,
        body=body,
        imu_roll_rad=imu_roll_rad,
        imu_pitch_rad=imu_pitch_rad,
        head_tilt_rad=head_tilt_rad,
        yaw_rate=yaw_rate,
        step_off_m=step_off_m,
        hazard_pad_m=rc.HAZARD_PAD_M,
        head_pan_rad=head_pan_rad,
    )
    return FinderHazard(cue, estimate)


def _blank() -> np.ndarray:
    """Bright gray. Black would itself look like a dark leg."""
    rgb = np.zeros((rc.HEIGHT, rc.WIDTH, 3), dtype=np.uint8)
    rgb[:, :] = (120, 120, 120)
    return rgb


def _paint_floor(rgb: np.ndarray, row0: int) -> None:
    rgb[row0:, :, 0] = 160
    rgb[row0:, :, 1] = 110
    rgb[row0:, :, 2] = 60


def _paint_leg(rgb: np.ndarray, column0: int, column1: int, row0: int, row1: int) -> None:
    rgb[row0:row1, column0:column1, 0] = 30
    rgb[row0:row1, column0:column1, 1] = 32
    rgb[row0:row1, column0:column1, 2] = 34


def self_check() -> None:
    """Lowest column pixel, not the centroid. Clipped blobs emit no ray."""
    if abs(rc.HAZARD_PAD_M - 0.020) > 1e-12:
        raise SystemExit("self_check pad moved")
    rgb = _blank()
    _paint_floor(rgb, 320)
    _paint_leg(rgb, 200, 212, 160, 320)
    cue = find_hazard_pixel(rgb)
    if cue is None or cue.u is None or cue.v is None:
        raise SystemExit("self_check missed the leg")
    if cue.bottom_clipped or cue.too_close:
        raise SystemExit("self_check flagged a leg that meets the floor")
    if not (200.0 <= cue.u < 212.0):
        raise SystemExit(f"self_check column {cue.u}")
    # Lowest pixel centre is row 319.5. The blob centre is near row 240.
    if abs(cue.v - 319.5) > 1e-6:
        raise SystemExit(f"self_check v {cue.v} is not the floor pixel")
    if cue.v <= 240.0:
        raise SystemExit("self_check returned the centroid half")
    cam = rc.KitCamPose((0.0, 0.0, 0.34))
    body = rc.BodyFrame((0.0, 0.0), (1.0, 0.0))
    low = rc.estimate_hazard(
        cue.u, cue.v,
        cam=cam, body=body,
        imu_roll_rad=0.0, imu_pitch_rad=-0.26, head_tilt_rad=0.0,
        yaw_rate=0.0, step_off_m=0.017,
    )
    mid = rc.estimate_hazard(
        cue.u, 239.5,
        cam=cam, body=body,
        imu_roll_rad=0.0, imu_pitch_rad=-0.26, head_tilt_rad=0.0,
        yaw_rate=0.0, step_off_m=0.017,
    )
    if low is None or mid is None:
        raise SystemExit("self_check ray missed")
    if low.eye_range >= mid.eye_range:
        raise SystemExit(
            f"self_check centroid range {mid.eye_range} is not longer than {low.eye_range}"
        )
    if abs(low.hazard_pad_m - 0.020) > 1e-12:
        raise SystemExit("self_check estimate pad")
    clipped = _blank()
    clipped[300:, :320, 0] = 160
    clipped[300:, :320, 1] = 110
    clipped[300:, :320, 2] = 60
    _paint_leg(clipped, 400, 412, 200, rc.HEIGHT)
    edge = find_hazard_pixel(clipped)
    if edge is None or not edge.bottom_clipped or not edge.too_close:
        raise SystemExit("self_check did not flag a frame-edge leg")
    if edge.u is not None or edge.v is not None:
        raise SystemExit("self_check emitted a pixel for a clipped leg")
    wrapped = estimate_finder_hazard(
        clipped,
        cam=cam, body=body,
        imu_roll_rad=0.0, imu_pitch_rad=-0.26, head_tilt_rad=0.0,
        yaw_rate=0.0, step_off_m=0.017,
    )
    if wrapped is None or wrapped.estimate is not None:
        raise SystemExit("self_check ranged a clipped leg")
    empty = estimate_finder_hazard(
        _blank(),
        cam=cam, body=body,
        imu_roll_rad=0.0, imu_pitch_rad=-0.26, head_tilt_rad=0.0,
        yaw_rate=0.0, step_off_m=0.017,
    )
    if empty is not None:
        raise SystemExit("self_check invented a cue on black")


if __name__ == "__main__":
    self_check()
    print(
        f"self_check ok source={SOURCE} pad={rc.HAZARD_PAD_M:.3f} "
        f"wood=R>{WOOD_R_MIN:.0f} leg_lum<{LEG_LUM_MAX}"
    )
