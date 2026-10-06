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
range. A leg column is at least 48 px tall and 8–40 px wide. A
contact in the bottom band has to be taller still. A short speck, a
hairline edge, or a wide silhouette is not a leg. ``too_close`` is
that flag only for a clip
whose last-row column rays into the foot corridor. A last-row dark blob
outside the corridor is not a near-floor leg, and the flag is cleared.
A later in-corridor hit is dropped when that stance already has a
floor point from the same gait phase and the new point is farther
than the body has walked since then, plus a small margin. The first
sighting, with no prior point, is emitted only when that column goes
low-chroma within 64 px of the contact. That near-contact gate is
sim-only. It is not evidence for any-space, and it is not evidence
for real brown wood under kit exposure. Bedroom left in this sim
was dropped because that column stays brown. A wood-brown stress
flattened the stool mesh and the coffee mesh to RGB (84, 62, 42);
shading kept both stops, and that is still not an any-space clear.
Width and span are not tightened. The living-right apron stop stays
a height keep. The entrance rug is Controls. Soft-pass is off.
Controls latches Day-1 ``stop`` on the flag that remains. This
module does not send ``stop``.

Moondream's room ask does not return a pixel. It is not called here.
``t_cue`` is not ``T_detect``. ``HAZARD_PAD_M`` stays 0.020 and is
applied only inside ``estimate_hazard``. Head tilt is not commanded.
The 3–5 cm buffer is not applied. Soft-pass is off.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
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
# A real near leg in these rooms is a tall column about 11–33 px wide
# (coffee leg, stool leg). A 3–7 px column is an edge. A 47 px column
# is a silhouette. A 30 px run is a speck. A 36 px cap still lets the
# living-left and entrance-left clips through on the next frame.
LEG_SPAN_MIN = 48
LEG_CLIP_SPAN_MIN = 48
LEG_WIDTH_MIN = 8
LEG_WIDTH_MAX = 40
# Contact row this low is the near floor. The column has to be taller
# still. The kitchen stool there is hundreds of pixels.
NEAR_ROW = 440
NEAR_SPAN_MIN = 120
COLUMN_GAP = 12
ROW_JOIN = 56
CLIP_SPLIT = 8
WOOD_GAP_PX = 3
# A same-stance hit farther than the body walked since that sample, plus
# this margin, is texture drift. Coffee and the stool move 0.002–0.014 m.
# The false clips jump about 0.22 m. 0.08 is inside 0.05–0.10.
PHASE_DRIFT_MARGIN_M = 0.08
# Third 8 ms tick of a shift bout. That is the steady one-foot sample.
# The both-feet exchange is a different pitch and is not this sample.
PHASE_SAMPLE_TICK = 3
# Sim-only near-contact gate. Not evidence for any-space or for real
# brown wood under kit exposure. In these rooms the stool is gray at
# the pixel, the coffee leg is dark 14 px up, and the apron rail is
# dark by 39 px. The bedroom-left texture stays chroma ~42. 64 covers
# the apron and still misses that texture. The wood-brown stress kept
# the stool and coffee stops via shading. Not a width, span, or height
# cut. The apron stop stays.
LEG_CORE_RISE_PX = 64
LEG_CORE_CHROMA = 16
LEG_CORE_LUM = 40
SOURCE = "rgb_wood_leg"


@dataclass(frozen=True)
class HazardCue:
    """One leg-like blob.

    ``u`` and ``v`` are the ray pixel: the lowest leg pixel in the blob's
    column. Both are ``None`` when ``bottom_clipped`` is set. ``contact_row``
    is that same row, including the frame edge, and is not a ray input.
    ``too_close`` starts equal to ``bottom_clipped``. ``corridor_gate_cues``
    clears it when that column's floor ray is outside the foot corridor.
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
    if chosen.row >= NEAR_ROW and span < NEAR_SPAN_MIN:
        return None
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

    A bottom-clipped cue is included with ``u`` and ``v`` empty.
    ``too_close`` still matches the clip here. ``corridor_gate_cues``
    clears that flag when the column is outside the foot corridor.
    Callers that ray only ``u, v`` must still stop when ``too_close``
    remains set.
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


def column_has_leg_core(
    rgb: np.ndarray,
    dark: np.ndarray,
    column: int,
    contact_row: int,
) -> bool:
    """True when the column goes gray or black within ``LEG_CORE_RISE_PX``.

    The walk stays on the contiguous leg pixels above the contact. A
    brown run that ends, or that stays brown for the whole window, is
    not a core. The contact pixel itself counts.
    """
    image = np.asarray(rgb)
    height = int(dark.shape[0])
    width = int(dark.shape[1])
    col = int(column)
    row = min(int(contact_row), height - 1)
    if col < 0 or col >= width or row < 0:
        return False
    for back in range(LEG_CORE_RISE_PX):
        rr = row - back
        if rr < 0 or not bool(dark[rr, col]):
            return False
        pix = image[rr, col]
        red = int(pix[0])
        green = int(pix[1])
        blue = int(pix[2])
        chroma = max(red, green, blue) - min(red, green, blue)
        lum = (red + green + blue) // 3
        if chroma < LEG_CORE_CHROMA and lum < LEG_CORE_LUM:
            return True
    return False


def confirm_leg_columns(
    cues: tuple[HazardCue, ...],
    rgb: np.ndarray,
) -> tuple[HazardCue, ...]:
    """Drop a first-sighting column that stays brown above the contact.

    Sim-only. Not evidence for any-space or for real brown wood under
    kit exposure. In this sim the coffee leg, the stool, and the apron
    rail go low-chroma inside 64 px. The bedroom-left texture does not.
    A flat wood-brown render of those hit meshes still stopped, because
    shading made a low-chroma pixel. That is not an any-space clear.
    Width, span, and the apron height are not read. This does not send
    ``stop``.
    """
    if not cues:
        return ()
    image = np.asarray(rgb)
    dark = leg_mask(image, wood_mask(image))
    kept = [
        cue
        for cue in cues
        if column_has_leg_core(image, dark, cue.column, cue.contact_row)
    ]
    return tuple(kept)


def corridor_gate_cues(
    cues: tuple[HazardCue, ...],
    *,
    cam: rc.KitCamPose,
    body: rc.BodyFrame,
    imu_roll_rad: float,
    imu_pitch_rad: float,
    head_tilt_rad: float,
    yaw_rate: float,
    step_off_m: float,
    head_pan_rad: float = 0.0,
) -> tuple[HazardCue, ...]:
    """Clear ``too_close`` unless the clipped column is in the foot corridor.

    The last row is the near floor. A dark blob there can be a leg cut off
    by the frame, where a ray would read long, or it can be floor grain or
    a distant prop's silhouette at the side of the wide view. Only the
    in-corridor clip keeps the stop flag. The pixel stays empty either way,
    so an outside clip cannot become an optimistic long range. The pad
    stays 0.020. This does not send ``stop``.
    """
    gated: list[HazardCue] = []
    for cue in cues:
        if not cue.too_close:
            gated.append(cue)
            continue
        estimate = rc.estimate_hazard(
            float(cue.column) + 0.5,
            float(rc.HEIGHT) - 0.5,
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
        if estimate is not None and estimate.in_corridor:
            gated.append(cue)
            continue
        gated.append(replace(cue, too_close=False))
    return tuple(gated)


@dataclass
class _StanceHit:
    xy: tuple[float, float]


class SamePhaseFloor:
    """Emit a first sighting. Drop a same-stance floor point that jumped.

    The ray already applies this frame's pitch, roll, and head tilt.
    Memory is the in-corridor hits from the third tick of a shift bout,
    keyed by stance. A stance with no stored point emits every cue that
    reached it. The caller drops a brown column before that first emit.
    A hit farther than the body has walked since the last sample of
    that stance, plus ``PHASE_DRIFT_MARGIN_M``, is not emitted and is
    not written back. The sample anchor moves on every sample, including
    one that accepts nothing, so an old unmatched point does not let the
    margin grow with the whole walk. Leaving the jumped point out of
    memory keeps the next tick from matching it. Width and span are not
    read here. This does not send ``stop``.
    """

    def __init__(self) -> None:
        self._hits: dict[str, list[_StanceHit]] = {}
        self._anchor: dict[str, tuple[float, float]] = {}
        self._in_shift = False
        self._shift_ticks = 0

    def has_prior(self, stance: str) -> bool:
        """True once this stance has stored an in-corridor floor point."""
        return bool(self._hits.get(stance))

    def apply(
        self,
        cues: tuple[HazardCue, ...],
        *,
        phase: str,
        stance: str,
        body_xy: tuple[float, float],
        cam: rc.KitCamPose,
        body: rc.BodyFrame,
        imu_roll_rad: float,
        imu_pitch_rad: float,
        head_tilt_rad: float,
        yaw_rate: float,
        step_off_m: float,
        head_pan_rad: float = 0.0,
    ) -> tuple[HazardCue, ...]:
        sample = self._sample_now(phase)
        prior = self._hits.get(stance, [])
        kept: list[HazardCue] = []
        accepted: list[tuple[float, float]] = []
        for cue in cues:
            hit = _in_corridor_hit(
                cue,
                cam=cam,
                body=body,
                imu_roll_rad=imu_roll_rad,
                imu_pitch_rad=imu_pitch_rad,
                head_tilt_rad=head_tilt_rad,
                yaw_rate=yaw_rate,
                step_off_m=step_off_m,
                head_pan_rad=head_pan_rad,
            )
            if hit is None:
                kept.append(cue)
                continue
            if _jumped(hit, prior, body_xy, self._anchor.get(stance)):
                if cue.too_close:
                    kept.append(replace(cue, too_close=False))
                continue
            kept.append(cue)
            accepted.append(hit)
        if sample:
            self._commit(stance, prior, accepted, body_xy)
        return tuple(kept)

    def _sample_now(self, phase: str) -> bool:
        if phase != "shift":
            self._in_shift = False
            self._shift_ticks = 0
            return False
        if not self._in_shift:
            self._in_shift = True
            self._shift_ticks = 0
        self._shift_ticks += 1
        return self._shift_ticks == PHASE_SAMPLE_TICK

    def _commit(
        self,
        stance: str,
        prior: list[_StanceHit],
        accepted: list[tuple[float, float]],
        body_xy: tuple[float, float],
    ) -> None:
        mem = list(prior)
        used: set[int] = set()
        fresh: list[_StanceHit] = []
        anchor = self._anchor.get(stance, body_xy)
        walked = math.hypot(body_xy[0] - anchor[0], body_xy[1] - anchor[1])
        for hit in accepted:
            if mem:
                best_i = min(
                    range(len(mem)),
                    key=lambda i: math.hypot(hit[0] - mem[i].xy[0], hit[1] - mem[i].xy[1]),
                )
                dist = math.hypot(hit[0] - mem[best_i].xy[0], hit[1] - mem[best_i].xy[1])
                if dist <= walked + PHASE_DRIFT_MARGIN_M and best_i not in used:
                    mem[best_i] = _StanceHit(hit)
                    used.add(best_i)
                    continue
            fresh.append(_StanceHit(hit))
        mem.extend(fresh)
        self._hits[stance] = mem
        self._anchor[stance] = body_xy


def _in_corridor_hit(
    cue: HazardCue,
    *,
    cam: rc.KitCamPose,
    body: rc.BodyFrame,
    imu_roll_rad: float,
    imu_pitch_rad: float,
    head_tilt_rad: float,
    yaw_rate: float,
    step_off_m: float,
    head_pan_rad: float,
) -> tuple[float, float] | None:
    """Tilt-corrected floor point, or None when the cue is outside."""
    if cue.too_close:
        u = float(cue.column) + 0.5
        v = float(rc.HEIGHT) - 0.5
    elif cue.u is None or cue.v is None:
        return None
    else:
        u = float(cue.u)
        v = float(cue.v)
    estimate = rc.estimate_hazard(
        u,
        v,
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
    if estimate is None or not estimate.in_corridor:
        return None
    return estimate.hit_xy_m


def _jumped(
    hit: tuple[float, float],
    prior: list[_StanceHit],
    body_xy: tuple[float, float],
    anchor: tuple[float, float] | None,
) -> bool:
    """True when a stored same-stance point cannot explain this hit.

    ``anchor`` is the body position at the last sample of this stance.
    An unmatched point from early in the walk does not widen the margin.
    """
    if not prior or anchor is None:
        return False
    best = min(
        prior,
        key=lambda item: math.hypot(hit[0] - item.xy[0], hit[1] - item.xy[1]),
    )
    dist = math.hypot(hit[0] - best.xy[0], hit[1] - best.xy[1])
    walked = math.hypot(body_xy[0] - anchor[0], body_xy[1] - anchor[1])
    return dist > walked + PHASE_DRIFT_MARGIN_M


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
    speck = _blank()
    _paint_floor(speck, 400)
    _paint_leg(speck, 300, 308, 449, 479)
    if find_hazard_pixel(speck) is not None:
        raise SystemExit("self_check kept a short speck on the near floor")
    clipped_speck = _blank()
    _paint_floor(clipped_speck, 400)
    _paint_leg(clipped_speck, 300, 308, 430, rc.HEIGHT)
    if find_hazard_pixel(clipped_speck) is not None:
        raise SystemExit("self_check kept a short clipped speck")
    tall = _blank()
    _paint_floor(tall, 400)
    _paint_leg(tall, 300, 312, 200, 475)
    near = find_hazard_pixel(tall)
    if near is None or near.u is None or near.too_close:
        raise SystemExit("self_check dropped a tall near-floor leg")
    if abs(near.v - 474.5) > 1e-6:
        raise SystemExit(f"self_check near v {near.v}")
    if not confirm_leg_columns((near,), tall):
        raise SystemExit("self_check dropped a gray leg column")
    _check_leg_core()
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
    center = HazardCue(None, None, True, True, 320, rc.HEIGHT - 1, 80, 8, SOURCE)
    side = HazardCue(None, None, True, True, 600, rc.HEIGHT - 1, 80, 8, SOURCE)
    kept = corridor_gate_cues(
        (center, side),
        cam=cam, body=body,
        imu_roll_rad=0.0, imu_pitch_rad=-0.26, head_tilt_rad=0.0,
        yaw_rate=0.0, step_off_m=0.017,
    )
    if len(kept) != 2 or not kept[0].too_close or kept[0].u is not None:
        raise SystemExit("self_check dropped an in-corridor clip")
    if kept[1].too_close or kept[1].u is not None or kept[1].v is not None:
        raise SystemExit("self_check left a side clip as too_close")
    if not kept[1].bottom_clipped:
        raise SystemExit("self_check emitted a ray for a side clip")
    _check_same_phase_floor(cam, body)


def _paint_brown(rgb: np.ndarray, column0: int, column1: int, row0: int, row1: int) -> None:
    rgb[row0:row1, column0:column1, 0] = 84
    rgb[row0:row1, column0:column1, 1] = 62
    rgb[row0:row1, column0:column1, 2] = 42


def _check_leg_core() -> None:
    """A brown texture column is not a first-sighting leg. A dark core is."""
    brown = _blank()
    _paint_floor(brown, 300)
    _paint_brown(brown, 260, 272, 200, rc.HEIGHT)
    raw = find_hazard_cues(brown)
    if not raw or not raw[0].too_close:
        raise SystemExit("self_check brown clip was not a cue")
    if confirm_leg_columns(raw, brown):
        raise SystemExit("self_check kept a brown texture column")
    mixed = _blank()
    _paint_floor(mixed, 400)
    _paint_leg(mixed, 260, 272, 200, 400)
    _paint_brown(mixed, 260, 272, 386, 400)
    kept = confirm_leg_columns(find_hazard_cues(mixed), mixed)
    if not kept:
        raise SystemExit("self_check dropped a leg dark 14 px above the contact")
    late = _blank()
    _paint_floor(late, 400)
    _paint_leg(late, 260, 272, 200, 400)
    _paint_brown(late, 260, 272, 320, 400)
    if confirm_leg_columns(find_hazard_cues(late), late):
        raise SystemExit("self_check kept a column dark only past 64 px")
    apron = _blank()
    _paint_floor(apron, 446)
    _paint_leg(apron, 410, 418, 100, 446)
    apron[407:446, 410:418, 0] = 71
    apron[407:446, 410:418, 1] = 55
    apron[407:446, 410:418, 2] = 41
    if not confirm_leg_columns(find_hazard_cues(apron), apron):
        raise SystemExit("self_check dropped a column dark by 39 px")


def _check_same_phase_floor(cam: rc.KitCamPose, body: rc.BodyFrame) -> None:
    """First sighting stays. A 0.22 m jump against a walked body does not."""
    common = dict(
        cam=cam,
        body=body,
        imu_roll_rad=0.0,
        imu_pitch_rad=-0.26,
        head_tilt_rad=0.0,
        yaw_rate=0.0,
        step_off_m=0.017,
        head_pan_rad=0.0,
    )
    near = HazardCue(320.5, 460.5, False, False, 320, 460, 180, 16, SOURCE)
    near_hit = _in_corridor_hit(near, **common)
    if near_hit is None:
        raise SystemExit("self_check phase floor missed the near pixel")
    far: HazardCue | None = None
    far_hit: tuple[float, float] | None = None
    for row in range(180, 450, 8):
        cue = HazardCue(320.5, float(row) + 0.5, False, False, 320, row, 180, 16, SOURCE)
        hit = _in_corridor_hit(cue, **common)
        if hit is None:
            continue
        if math.hypot(hit[0] - near_hit[0], hit[1] - near_hit[1]) > 0.22:
            far = cue
            far_hit = hit
            break
    if far is None or far_hit is None:
        raise SystemExit("self_check phase floor found no 0.22 m pixel")
    origin = (0.0, 0.0)
    gate = SamePhaseFloor()
    if gate.has_prior("L"):
        raise SystemExit("self_check invented a stored floor point")
    for _tick in range(2):
        out = gate.apply((near,), phase="shift", stance="L", body_xy=origin, **common)
        if out != (near,):
            raise SystemExit("self_check dropped a first sighting")
    out = gate.apply((far,), phase="swing", stance="L", body_xy=origin, **common)
    if out != (far,):
        raise SystemExit("self_check dropped a first sighting with no stored point")
    for _tick in range(PHASE_SAMPLE_TICK):
        out = gate.apply((near,), phase="shift", stance="L", body_xy=origin, **common)
        if out != (near,):
            raise SystemExit("self_check dropped the sample that stores the first hit")
    if not gate.has_prior("L"):
        raise SystemExit("self_check did not store the first sighting")
    walked = (0.08, 0.0)
    out = gate.apply((near,), phase="swing", stance="L", body_xy=walked, **common)
    if out != (near,):
        raise SystemExit("self_check dropped a floor point inside the walked margin")
    close: HazardCue | None = None
    for row in (456, 458, 462, 464, 450, 470):
        cue = HazardCue(320.5, float(row) + 0.5, False, False, 320, row, 180, 16, SOURCE)
        hit = _in_corridor_hit(cue, **common)
        if hit is None:
            continue
        dist = math.hypot(hit[0] - near_hit[0], hit[1] - near_hit[1])
        if 0.002 <= dist <= 0.014:
            close = cue
            break
    if close is None:
        raise SystemExit("self_check found no 0.002–0.014 m neighbour")
    out = gate.apply((close,), phase="swing", stance="L", body_xy=walked, **common)
    if out != (close,):
        raise SystemExit("self_check dropped a coffee-sized floor move")
    out = gate.apply((far,), phase="swing", stance="L", body_xy=walked, **common)
    if out:
        raise SystemExit("self_check kept a 0.22 m jump")
    out = gate.apply((far,), phase="swing", stance="L", body_xy=walked, **common)
    if out:
        raise SystemExit("self_check stored a rejected jump")
    clip = HazardCue(None, None, True, True, 320, rc.HEIGHT - 1, 200, 12, SOURCE)
    clip_hit = _in_corridor_hit(clip, **common)
    if clip_hit is None:
        raise SystemExit("self_check phase floor missed the clip")
    if math.hypot(clip_hit[0] - far_hit[0], clip_hit[1] - far_hit[1]) <= PHASE_DRIFT_MARGIN_M:
        raise SystemExit("self_check clip is not far from the jumped pixel")
    other = SamePhaseFloor()
    for _tick in range(PHASE_SAMPLE_TICK):
        other.apply((far,), phase="shift", stance="L", body_xy=origin, **common)
    cleared = other.apply((clip,), phase="swing", stance="L", body_xy=origin, **common)
    if len(cleared) != 1 or cleared[0].too_close or cleared[0].u is not None:
        raise SystemExit("self_check left a jumped clip as too_close")
    again = other.apply((clip,), phase="swing", stance="L", body_xy=origin, **common)
    if len(again) != 1 or again[0].too_close:
        raise SystemExit("self_check stored the jumped clip")
    fresh = other.apply((clip,), phase="swing", stance="R", body_xy=origin, **common)
    if len(fresh) != 1 or not fresh[0].too_close:
        raise SystemExit("self_check used the other stance's floor point")
    # Empty samples still move the anchor. A 0.22 m jump stays a jump
    # after the body has walked many periods away from the first store.
    stale = SamePhaseFloor()
    for _tick in range(PHASE_SAMPLE_TICK):
        stale.apply((near,), phase="shift", stance="L", body_xy=origin, **common)
    for step in range(1, 9):
        place = (0.08 * step, 0.0)
        for _tick in range(PHASE_SAMPLE_TICK):
            stale.apply((), phase="shift", stance="L", body_xy=place, **common)
        stale.apply((), phase="swing", stance="L", body_xy=place, **common)
    late = stale.apply((far,), phase="swing", stance="L", body_xy=(0.72, 0.0), **common)
    if late:
        raise SystemExit("self_check let a jump through a stale anchor")


if __name__ == "__main__":
    self_check()
    print(
        f"self_check ok source={SOURCE} pad={rc.HAZARD_PAD_M:.3f} "
        f"wood=R>{WOOD_R_MIN:.0f} leg_lum<{LEG_LUM_MAX} "
        f"core_rise={LEG_CORE_RISE_PX}"
    )
