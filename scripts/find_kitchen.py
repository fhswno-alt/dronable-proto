#!/usr/bin/env python3
"""Prefer FAIL find-kitchen from kit_cam pixels.

A phrase "go to the kitchen" / "go to kitchen" may steer only when
mujoco/room_kitchen.xml is loaded and the current kit_cam frame shows the
kitchen. The steer is stand / stop / vel on the CommandBus in
scripts/steer_walk.py. Caps stay +0.056 / -0.032 m/s and yaw ±0.25 rad/s.
vel is resent at 10 Hz. 200 ms of silence stands.

This is not a map, not SLAM, and not an arrival. The pixel blob picks yaw
left, yaw right, or a forward burst. Named kitchen / table / chair boxes
are a sim honesty check that the blob sits on the kitchen. They are not
waypoints and they are not given to a planner. If the backsplash is missing,
split, or off the kitchen body, the finder sends no vel.

The phrase path re-scores kit_cam every 0.40 s and walks at half the
forward cap (0.028 m/s). A centered blob is that vx with yaw 0. A side blob
adds a yaw trim scaled by the bias. Full-cap slices and short slices with
stand pauses between them are not this path: the first crossed up_z 0.90
near +0.39 m, and the pauses never got past ~0.1 m. The up_z bar stays
0.90. The 1.2 s hop stays in --self-test at the full forward cap. The
world-x budget is 1.10 m, a stop, not a goal pose. Arrival is claimed only when
the backsplash fills at least half the frame and the torso is within 0.25 m
of the kitchen geom. Otherwise the summary reports end x and the remaining
gap and does not say arrived.

The visible signal under this lighting is the yellow backsplash. The wood
counter's lit face does not separate from the blue cabinet, and the red
kettle renders brown, so neither is the blob. Empty floor and a yaw that
puts the kitchen behind the camera measure zero of that yellow.

The camera stays on head_tilt_link at 0.050 0.019 0.007, fovy 104.82.
The plant file is not edited.

Run:
  MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the kitchen"
  MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the bathroom"
  MUJOCO_GL=osmesa python scripts/find_kitchen.py --scene plant "go to the kitchen"
  python scripts/find_kitchen.py --self-test
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

if "--view" not in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import steer_walk  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
ROOM_XML = ROOT / "mujoco" / "room_kitchen.xml"
PLANT_XML = steer_walk.PLANT_XML
PREVIEWS = ROOT / "previews"
ARTIFACTS = Path("/opt/cursor/artifacts")

WIDTH = 640
HEIGHT = 480
STAND_S = 0.60
SETTLE_S = 0.60
YAW_HOLD_S = 1.00
FORWARD_HOLD_S = 1.20
# Phrase path. kit_cam is scored between these slices. The gait stays in move.
# Full-cap slices crossed up_z 0.90 near +0.39 m. A 0.20–0.30 s slice plus a
# stand pause does not walk: vx slews to the cap in 0.70 s, and the pause
# either drifts backward or yaws the kitchen out of frame. Half the forward
# cap is the softer duty that stayed above up_z 0.90. vx=0 yaw does not
# change heading on this plant.
BURST_FORWARD_S = 0.40
BURST_YAW_S = 0.40
SOFT_VX = steer_walk.VX_FWD_CAP * 0.5
# Half-cap realized speed is about 0.016 m/s. The slice cap is only so a
# stalled walk cannot run forever. 280 * 0.40 s is longer than the 1.10 m
# budget at that speed, so the distance stop fires first.
MAX_FORWARD_BURSTS = 280
MAX_YAW_CORRECTIONS = 80
# |bias| at which the trim uses the full yaw cap. Inside CENTER_BIAS, yaw is 0.
YAW_BIAS_FULL = 0.35
# World-x progress stop. The kitchen near face is at x = 1.315 m, so 1.10 m
# of progress from the stand leaves a gap above the 0.25 m arrival bar.
# This is a burst budget, not a waypoint at the counter. The up_z bar is
# still 0.90, and arrival still needs both yellow >= 0.50 and gap <= 0.25 m.
PROGRESS_STOP_M = 1.10
# First sample under the plant's throttle line stops the approach.
# A lean that holds under 0.85 also trips the bus fault in this room,
# because the world COM includes the furniture.
UP_Z_ABORT = 0.90
# Arrival requires both. Half the frame is "most of the frame".
# 0.25 m is contact range for this torso, not a room crossing.
ARRIVAL_YELLOW_FRAC = 0.50
ARRIVAL_REMAINING_M = 0.25
PERCEPT_S = 0.40
BUDGET_STOP = "burst budget reached; stop"
# Stand kit_cam measures yellow fraction ~0.039 and bias ~-0.014.
# Empty floor and a yaw that hides the kitchen measure 0.
MIN_YELLOW_FRAC = 0.015
CENTER_BIAS = 0.08
MID_SHARE_CENTER = 0.45
MIN_BOX_PX = 36.0
BOX_PAD_PX = 40.0
LOWER_BLOCK_FRAC = 0.25

DecisionName = Literal["yaw_left", "yaw_right", "forward", "fail"]
SceneName = Literal["room", "plant"]
PhraseAction = Literal["find", "refuse"]

KITCHEN_PHRASES = frozenset({"go to the kitchen", "go to kitchen"})
BATH_WORDS = frozenset({"bathroom", "bath", "toilet", "restroom", "washroom"})
OTHER_ROOM_WORDS = frozenset({
    "bedroom", "hallway", "hall", "office", "garage", "living", "dining",
    "pantry", "lobby", "corridor", "stairs", "stair", "closet", "basement",
    "attic", "porch", "yard", "garden", "room",
})
MAP_WORDS = frozenset({"map", "maps", "waypoint", "waypoints", "goal", "goals"})
DOOR_WORDS = frozenset({"door", "doors", "doorway"})
STRAFE_WORDS = frozenset({"strafe", "sidestep", "sideways", "lateral", "vy"})
_POLITE_PREFIXES = ("please ", "could you ", "can you ", "would you ")

FIND_LINE = "find kitchen from kit_cam"
BATH_LINE = "refused: bathroom stays refused; no map and no motion"
OTHER_ROOM_LINE = "refused: that room stays refused; no map and no motion"
EMPTY_SCENE_LINE = "Prefer FAIL: empty floor plant; kit_cam has no kitchen; no motion"
MAP_LINE = "refused: no map"
DOOR_LINE = "refused: door is not a day-1 velocity command"
VY_LINE = "refused: no vy"
UNKNOWN_LINE = "refused: not a find-kitchen phrase"
EMPTY_PHRASE_LINE = "refused: empty phrase"
NOT_IN_FRAME_LINE = "Prefer FAIL: kitchen is not in the kit_cam frame; no motion"
NO_PIXELS_LINE = "Prefer FAIL: kit_cam pixels do not show the backsplash; no motion"
AMBIGUOUS_LINE = "Prefer FAIL: kitchen blob is ambiguous; no motion"
BLOCKED_LINE = "Prefer FAIL: backsplash fills the lower center; no forward"
HONESTY = (
    "Vision-reactive bursts from kit_cam. Not SLAM, not a map, "
    "not a waypoint planner, and not an arrival unless the backsplash "
    "fills at least half the frame and the torso is within 0.25 m of the "
    "kitchen geom. The yellow blob picks yaw and forward. "
    "The kitchen geom distance is only the arrival bar and the reported gap. "
    "A miss, a split blob, a tip, or an empty floor stops with no further vel."
)


@dataclass(frozen=True)
class PixelRead:
    frac: float
    centroid_u: float | None
    centroid_v: float | None
    bias: float
    mid_share: float
    bimodal: bool
    lower_center_blocked: bool
    decision: DecisionName
    reason: str


@dataclass(frozen=True)
class BodyBox:
    name: str
    u0: float
    u1: float
    v0: float
    v1: float
    in_frame: bool


@dataclass(frozen=True)
class KitchenScore:
    decision: DecisionName
    reason: str
    line: str
    needs_map: bool
    bias: float
    yellow_frac: float
    centroid_u: float | None
    centroid_v: float | None
    kitchen_in_frame: bool


@dataclass(frozen=True)
class PhraseResult:
    action: PhraseAction
    line: str


@dataclass(frozen=True)
class SentVel:
    t: float
    vx: float
    yaw_rate: float


@dataclass
class Attempt:
    before: np.ndarray
    after: np.ndarray
    initial: KitchenScore
    final: KitchenScore
    sent: list[SentVel]
    note: str
    min_up_z: float
    end_mode: str
    fault: bool


@dataclass
class Approach:
    """Multi-burst approach. arrival is true only when both bars hold on the stop frame."""

    before: np.ndarray
    mid: np.ndarray
    after: np.ndarray
    initial: KitchenScore
    mid_score: KitchenScore
    final: KitchenScore
    sent: list[SentVel]
    note: str
    stop_kind: BurstKind
    min_up_z: float
    end_mode: str
    fault: bool
    x0_m: float
    end_x_m: float
    end_y_m: float
    end_yaw_rad: float
    dx_m: float
    remaining_m: float
    near_face_x_m: float
    arrival: bool
    forward_bursts: int
    yaw_corrections: int


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def plant_md5_ok() -> str | None:
    if not PLANT_XML.is_file():
        return f"missing {PLANT_XML}"
    digest = _md5(PLANT_XML)
    if digest != steer_walk.PLANT_MD5:
        return f"plant md5 {digest} != {steer_walk.PLANT_MD5}"
    return None


def normalize_phrase(phrase: str) -> str:
    text = phrase.strip().lower().replace("'", "")
    text = re.sub(r"[^a-z0-9\s]+", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    changed = True
    while changed:
        changed = False
        for prefix in _POLITE_PREFIXES:
            if text.startswith(prefix):
                text = text[len(prefix):].strip()
                changed = True
        if text.endswith(" please"):
            text = text[: -len(" please")].strip()
            changed = True
    return text


def phrase_hook(phrase: str, *, scene: SceneName) -> PhraseResult:
    """Thin voice hook. Does not publish vel. The runner does, or refuses."""
    text = normalize_phrase(phrase)
    if not text:
        return PhraseResult("refuse", EMPTY_PHRASE_LINE)
    tokens = set(text.split())
    if tokens & MAP_WORDS:
        return PhraseResult("refuse", MAP_LINE)
    if tokens & DOOR_WORDS:
        return PhraseResult("refuse", DOOR_LINE)
    if tokens & STRAFE_WORDS:
        return PhraseResult("refuse", VY_LINE)
    if tokens & BATH_WORDS:
        return PhraseResult("refuse", BATH_LINE)
    if tokens & OTHER_ROOM_WORDS:
        return PhraseResult("refuse", OTHER_ROOM_LINE)
    if text not in KITCHEN_PHRASES:
        return PhraseResult("refuse", UNKNOWN_LINE)
    if scene != "room":
        return PhraseResult("refuse", EMPTY_SCENE_LINE)
    return PhraseResult("find", FIND_LINE)


def backsplash_mask(image: np.ndarray) -> np.ndarray:
    """Yellow backsplash. Empty kit_cam floor does not pass this test."""
    red = image[:, :, 0].astype(np.int16)
    green = image[:, :, 1].astype(np.int16)
    blue = image[:, :, 2].astype(np.int16)
    return (
        (red > 110)
        & (green > 100)
        & (blue < 100)
        & (red > blue + 40)
        & (green > blue + 30)
        & (np.abs(red - green) < 60)
    )


def read_pixels(image: np.ndarray) -> PixelRead:
    mask = backsplash_mask(image)
    total = int(mask.size)
    count = int(mask.sum())
    frac = float(count) / float(total) if total else 0.0
    if count == 0:
        return PixelRead(
            frac=0.0,
            centroid_u=None,
            centroid_v=None,
            bias=0.0,
            mid_share=0.0,
            bimodal=False,
            lower_center_blocked=False,
            decision="fail",
            reason="no backsplash pixels",
        )
    rows, cols = np.nonzero(mask)
    centroid_u = float(cols.mean())
    centroid_v = float(rows.mean())
    bias = (centroid_u - (WIDTH / 2.0)) / (WIDTH / 2.0)
    third = WIDTH // 3
    left = int(mask[:, :third].sum())
    mid = int(mask[:, third: 2 * third].sum())
    right = int(mask[:, 2 * third:].sum())
    mid_share = float(mid) / float(count)
    bimodal = left > 0.25 * count and right > 0.25 * count and mid < 0.15 * count
    lower = mask[int(HEIGHT * 0.70):, int(WIDTH * 0.30): int(WIDTH * 0.70)]
    blocked = bool(lower.size) and float(lower.mean()) > LOWER_BLOCK_FRAC
    if frac < MIN_YELLOW_FRAC:
        decision: DecisionName = "fail"
        reason = "backsplash fraction below the visible bar"
    elif bimodal:
        decision = "fail"
        reason = "backsplash is split left and right"
    elif abs(bias) <= CENTER_BIAS and mid_share >= MID_SHARE_CENTER:
        decision = "forward"
        reason = "backsplash is in the center of the frame"
    elif bias < 0.0:
        decision = "yaw_left"
        reason = "backsplash is left of center"
    else:
        decision = "yaw_right"
        reason = "backsplash is right of center"
    return PixelRead(
        frac=frac,
        centroid_u=centroid_u,
        centroid_v=centroid_v,
        bias=bias,
        mid_share=mid_share,
        bimodal=bimodal,
        lower_center_blocked=blocked,
        decision=decision,
        reason=reason,
    )


def _project(
    point_world: np.ndarray,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fovy_deg: float,
) -> tuple[float, float] | None:
    """Pixel (u, v) if the point is in front of kit_cam. None if behind.

    Same camera math as scripts/render_kit_cam_room.py. Used only to check
    that a pixel blob lands on a named body.
    """
    local = cam_mat.T @ (point_world - cam_pos)
    depth = -float(local[2])
    if depth <= 0.05:
        return None
    focal = (HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    u = (WIDTH / 2.0) + focal * (float(local[0]) / depth)
    v = (HEIGHT / 2.0) - focal * (float(local[1]) / depth)
    return u, v


def kitchen_clearance(model: mj.MjModel, data: mj.MjData) -> tuple[float, float] | None:
    """Torso-to-kitchen horizontal gap, and the kitchen geom's near face x.

    Read from the included scene geoms after the sim step. This is the
    arrival bar and the reported remaining distance. It does not pick a
    heading or a waypoint.
    """
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    kitchen_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "kitchen")
    if body_id < 0 or kitchen_id < 0:
        return None
    point_x = float(data.xpos[body_id][0])
    point_y = float(data.xpos[body_id][1])
    nearest = float("inf")
    near_face_x = float("inf")
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) != kitchen_id:
            continue
        half_x = float(model.geom_size[geom_id][0])
        half_y = float(model.geom_size[geom_id][1])
        rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        origin = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        delta = np.array([point_x - float(origin[0]), point_y - float(origin[1]), 0.0])
        local = rotation.T @ delta
        outside_x = max(abs(float(local[0])) - half_x, 0.0)
        outside_y = max(abs(float(local[1])) - half_y, 0.0)
        nearest = min(nearest, math.hypot(outside_x, outside_y))
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                corner = origin + rotation @ np.array([sx * half_x, sy * half_y, 0.0])
                near_face_x = min(near_face_x, float(corner[0]))
    if nearest == float("inf"):
        return None
    return nearest, near_face_x


def _body_box(model: mj.MjModel, data: mj.MjData, body_name: str) -> BodyBox | None:
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return None
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cam_id < 0:
        return None
    cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
    cam_mat = np.asarray(data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3)
    fovy = float(model.cam_fovy[cam_id])
    us: list[float] = []
    vs: list[float] = []
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) != body_id:
            continue
        size = np.asarray(model.geom_size[geom_id, :3], dtype=np.float64)
        rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        origin = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    local = np.array(
                        [sx * size[0], sy * size[1], sz * size[2]],
                        dtype=np.float64,
                    )
                    pix = _project(origin + rotation @ local, cam_pos, cam_mat, fovy)
                    if pix is None:
                        continue
                    us.append(pix[0])
                    vs.append(pix[1])
    if len(us) < 2:
        return BodyBox(body_name, 0.0, 0.0, 0.0, 0.0, False)
    u0, u1 = min(us), max(us)
    v0, v1 = min(vs), max(vs)
    on_u0 = max(u0, 0.0)
    on_u1 = min(u1, float(WIDTH - 1))
    on_v0 = max(v0, 0.0)
    on_v1 = min(v1, float(HEIGHT - 1))
    in_frame = (on_u1 - on_u0) >= MIN_BOX_PX and (on_v1 - on_v0) >= MIN_BOX_PX
    return BodyBox(body_name, u0, u1, v0, v1, in_frame)


def _inside(box: BodyBox, u: float, v: float, pad: float) -> bool:
    return (box.u0 - pad) <= u <= (box.u1 + pad) and (box.v0 - pad) <= v <= (box.v1 + pad)


def score_frame(model: mj.MjModel, data: mj.MjData, image: np.ndarray) -> KitchenScore:
    pixels = read_pixels(image)
    kitchen = _body_box(model, data, "kitchen")
    table = _body_box(model, data, "table")
    chair = _body_box(model, data, "chair")
    if kitchen is None:
        return KitchenScore(
            decision="fail",
            reason="no kitchen body in this scene",
            line=EMPTY_SCENE_LINE,
            needs_map=True,
            bias=pixels.bias,
            yellow_frac=pixels.frac,
            centroid_u=pixels.centroid_u,
            centroid_v=pixels.centroid_v,
            kitchen_in_frame=False,
        )
    if not kitchen.in_frame:
        return KitchenScore(
            decision="fail",
            reason="kitchen body is outside the kit_cam frame",
            line=NOT_IN_FRAME_LINE,
            needs_map=True,
            bias=pixels.bias,
            yellow_frac=pixels.frac,
            centroid_u=pixels.centroid_u,
            centroid_v=pixels.centroid_v,
            kitchen_in_frame=False,
        )
    if pixels.decision == "fail" and pixels.frac < MIN_YELLOW_FRAC:
        return KitchenScore(
            decision="fail",
            reason=pixels.reason,
            line=NO_PIXELS_LINE,
            needs_map=False,
            bias=pixels.bias,
            yellow_frac=pixels.frac,
            centroid_u=pixels.centroid_u,
            centroid_v=pixels.centroid_v,
            kitchen_in_frame=True,
        )
    if pixels.bimodal or pixels.decision == "fail":
        return KitchenScore(
            decision="fail",
            reason=pixels.reason,
            line=AMBIGUOUS_LINE,
            needs_map=False,
            bias=pixels.bias,
            yellow_frac=pixels.frac,
            centroid_u=pixels.centroid_u,
            centroid_v=pixels.centroid_v,
            kitchen_in_frame=True,
        )
    assert pixels.centroid_u is not None and pixels.centroid_v is not None
    on_kitchen = _inside(kitchen, pixels.centroid_u, pixels.centroid_v, BOX_PAD_PX)
    on_other = False
    for other in (table, chair):
        if other is not None and other.in_frame and _inside(other, pixels.centroid_u, pixels.centroid_v, 0.0):
            on_other = True
    if not on_kitchen or (on_other and not on_kitchen):
        return KitchenScore(
            decision="fail",
            reason="backsplash centroid is not on the kitchen body",
            line=AMBIGUOUS_LINE,
            needs_map=False,
            bias=pixels.bias,
            yellow_frac=pixels.frac,
            centroid_u=pixels.centroid_u,
            centroid_v=pixels.centroid_v,
            kitchen_in_frame=True,
        )
    if pixels.lower_center_blocked and pixels.decision == "forward":
        return KitchenScore(
            decision="fail",
            reason="backsplash blocks the lower center",
            line=BLOCKED_LINE,
            needs_map=False,
            bias=pixels.bias,
            yellow_frac=pixels.frac,
            centroid_u=pixels.centroid_u,
            centroid_v=pixels.centroid_v,
            kitchen_in_frame=True,
        )
    return KitchenScore(
        decision=pixels.decision,
        reason=pixels.reason,
        line=pixels.reason,
        needs_map=False,
        bias=pixels.bias,
        yellow_frac=pixels.frac,
        centroid_u=pixels.centroid_u,
        centroid_v=pixels.centroid_v,
        kitchen_in_frame=True,
    )


def command_for(decision: DecisionName) -> tuple[float, float] | None:
    """Bus command for a passing score. None means no motion.

    Yaw here is vx=0. The approach uses correction_command instead, because
    a pure yaw does not change heading on this plant.
    """
    if decision == "forward":
        return (steer_walk.VX_FWD_CAP, 0.0)
    if decision == "yaw_left":
        return (0.0, steer_walk.YAW_RATE_CAP)
    if decision == "yaw_right":
        return (0.0, -steer_walk.YAW_RATE_CAP)
    return None


def trim_yaw(bias: float) -> float:
    """Yaw rate for one approach slice. Sign faces the blob. Magnitude follows it.

    Full cap only when |bias| reaches YAW_BIAS_FULL. A smaller bias gets a
    smaller rate, still above the bus deadband once the blob is outside the
    center band. This is not a heading setpoint and not a path.
    """
    if abs(bias) <= CENTER_BIAS:
        return 0.0
    yaw = -bias / YAW_BIAS_FULL * steer_walk.YAW_RATE_CAP
    if yaw > steer_walk.YAW_RATE_CAP:
        return steer_walk.YAW_RATE_CAP
    if yaw < -steer_walk.YAW_RATE_CAP:
        return -steer_walk.YAW_RATE_CAP
    return yaw


def correction_command(score: KitchenScore) -> tuple[float, float] | None:
    """Approach slice: forward cap, plus a bias trim when the blob is off center."""
    if score.decision == "fail":
        return None
    return (SOFT_VX, trim_yaw(score.bias))


BurstAction = Literal["forward", "yaw_left", "yaw_right", "stop"]
BurstKind = Literal["steer", "budget", "tip", "blob", "yaw_limit", "close", "arrival"]


@dataclass(frozen=True)
class ApproachChoice:
    """Next burst. Distance is used only for the arrival bar and the close stop."""

    action: BurstAction
    kind: BurstKind
    line: str


def arrival_bars(yellow_frac: float, remaining_m: float) -> bool:
    """Both bars. Either one alone is not arrival."""
    return yellow_frac >= ARRIVAL_YELLOW_FRAC and remaining_m <= ARRIVAL_REMAINING_M


def approach_choice(
    score: KitchenScore,
    *,
    dx_m: float,
    up_z: float,
    yaw_corrections: int,
    forward_bursts: int,
    remaining_m: float,
) -> ApproachChoice:
    """Pick the next burst from the latest kit_cam score and a few stop bars.

    dx_m and remaining_m do not choose left versus right. The blob does.
    """
    if up_z < UP_Z_ABORT:
        return ApproachChoice(
            "stop",
            "tip",
            f"Prefer FAIL: up_z {up_z:.3f} is below {UP_Z_ABORT:.2f}; stop",
        )
    if score.decision == "fail":
        return ApproachChoice("stop", "blob", score.line)
    if arrival_bars(score.yellow_frac, remaining_m):
        return ApproachChoice(
            "stop",
            "arrival",
            (
                "arrival bars met: backsplash fills at least half the frame "
                f"and torso-to-kitchen is {remaining_m:.3f} m"
            ),
        )
    if remaining_m <= ARRIVAL_REMAINING_M:
        return ApproachChoice(
            "stop",
            "close",
            (
                "Prefer FAIL: torso is within "
                f"{ARRIVAL_REMAINING_M:.2f} m of the kitchen geom "
                "but the backsplash does not fill the frame; not arrival"
            ),
        )
    if dx_m >= PROGRESS_STOP_M:
        return ApproachChoice(
            "stop",
            "budget",
            (
                f"burst budget {dx_m:.3f} m reached; stop; "
                "this is not a counter pose and not arrival"
            ),
        )
    if score.decision in ("yaw_left", "yaw_right"):
        if yaw_corrections >= MAX_YAW_CORRECTIONS:
            return ApproachChoice(
                "stop",
                "yaw_limit",
                "Prefer FAIL: blob still off center after yaw corrections; no further search",
            )
        return ApproachChoice(score.decision, "steer", score.reason)
    if forward_bursts >= MAX_FORWARD_BURSTS:
        return ApproachChoice(
            "stop",
            "budget",
            "forward burst cap; stop; not arrival",
        )
    return ApproachChoice("forward", "steer", score.reason)


def _within_caps(vx: float, yaw_rate: float) -> bool:
    return (
        -steer_walk.VX_BACK_CAP - 1e-9 <= vx <= steer_walk.VX_FWD_CAP + 1e-9
        and abs(yaw_rate) <= steer_walk.YAW_RATE_CAP + 1e-9
    )


class KitCam:
    def __init__(self, model: mj.MjModel) -> None:
        self.renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)

    def grab(self, data: mj.MjData) -> np.ndarray:
        self.renderer.update_scene(data, camera="kit_cam")
        frame = np.asarray(self.renderer.render(), dtype=np.uint8).copy()
        if frame.shape != (HEIGHT, WIDTH, 3):
            raise RuntimeError(f"kit_cam frame shape {frame.shape}")
        return frame

    def close(self) -> None:
        self.renderer.close()


def _hold_stand(session: steer_walk.SteerSession, seconds: float) -> None:
    session.bus.stand(float(session.data.time))
    end = float(session.data.time) + seconds
    while float(session.data.time) < end - 1e-9:
        session.step()


def _hold_vel(
    session: steer_walk.SteerSession,
    cam: KitCam,
    vx: float,
    yaw_rate: float,
    seconds: float,
    sent: list[SentVel],
    *,
    stop_when_centered: bool = False,
    progress_x0: float | None = None,
    progress_cap_m: float | None = None,
) -> str | None:
    """Resend one vel at 10 Hz. Stop early if the kitchen leaves the frame.

    A yaw hold also stops when the blob reaches the center, or when it
    crosses to the other side. Crossing does not reverse. That would be a search.
    progress_cap_m stops a burst once world-x progress hits the budget.
    """
    if not _within_caps(vx, yaw_rate):
        return "refused: command outside the bus caps"
    end = float(session.data.time) + seconds
    last_send = -1.0
    last_look = float(session.data.time)
    while float(session.data.time) < end - 1e-9:
        now = float(session.data.time)
        if (now - last_send) >= (steer_walk.VEL_RESEND_S - 1e-9):
            refusal = session.bus.vel(vx, yaw_rate, now)
            sent.append(SentVel(now, vx, yaw_rate))
            last_send = now
            if refusal:
                return refusal
        session.step()
        if session.bus.fault:
            return f"fault: {session.bus.fault_reason}"
        if session.samples and session.samples[-1].up_z < UP_Z_ABORT:
            up_now = session.samples[-1].up_z
            return f"Prefer FAIL: up_z {up_now:.3f} during the burst; stop"
        if (
            progress_x0 is not None
            and progress_cap_m is not None
            and (float(session.data.qpos[0]) - progress_x0) >= progress_cap_m
        ):
            return BUDGET_STOP
        if (now - last_look) >= PERCEPT_S:
            last_look = now
            live = score_frame(session.model, session.data, cam.grab(session.data))
            if live.decision == "fail":
                return live.line
            if stop_when_centered and live.decision == "forward":
                return None
            if stop_when_centered and (
                (yaw_rate > 0.0 and live.decision == "yaw_right")
                or (yaw_rate < 0.0 and live.decision == "yaw_left")
            ):
                return "Prefer FAIL: yaw crossed the center; no reverse and no further search"
    return None


def run_attempt(
    session: steer_walk.SteerSession,
    cam: KitCam,
    *,
    yaw_hold_s: float = YAW_HOLD_S,
    forward_hold_s: float = FORWARD_HOLD_S,
) -> Attempt:
    """Score the stand frame, then yaw toward a side blob or step forward if centered.

    A side blob gets one short yaw. After that yaw, forward is sent only if
    the new frame is a clear center. Still biased, or a lost kitchen, stops.
    That second look is the honest try. It does not keep turning to search.
    """
    _hold_stand(session, STAND_S)
    before = cam.grab(session.data)
    initial = score_frame(session.model, session.data, before)
    sent: list[SentVel] = []
    note = initial.line
    if initial.decision == "fail":
        after = before
        return Attempt(
            before=before,
            after=after,
            initial=initial,
            final=initial,
            sent=sent,
            note=note,
            min_up_z=session.min_up_z,
            end_mode=session.bus.mode,
            fault=session.bus.fault,
        )
    current = initial
    if current.decision in ("yaw_left", "yaw_right"):
        command = command_for(current.decision)
        assert command is not None
        abort = _hold_vel(
            session, cam, command[0], command[1], yaw_hold_s, sent,
            stop_when_centered=True,
        )
        if abort:
            session.bus.stop(float(session.data.time))
            _hold_stand(session, SETTLE_S)
            final = score_frame(session.model, session.data, cam.grab(session.data))
            return Attempt(
                before=before,
                after=cam.grab(session.data),
                initial=initial,
                final=final,
                sent=sent,
                note=abort,
                min_up_z=session.min_up_z,
                end_mode=session.bus.mode,
                fault=session.bus.fault or abort.startswith("fault:"),
            )
        current = score_frame(session.model, session.data, cam.grab(session.data))
        if current.decision != "forward":
            note = (
                "Prefer FAIL: after a short yaw the kitchen is not a clear center; "
                "no forward and no further search"
            )
            session.bus.stop(float(session.data.time))
            _hold_stand(session, SETTLE_S)
            return Attempt(
                before=before,
                after=cam.grab(session.data),
                initial=initial,
                final=current,
                sent=sent,
                note=note,
                min_up_z=session.min_up_z,
                end_mode=session.bus.mode,
                fault=session.bus.fault,
            )
    command = command_for("forward")
    assert command is not None
    abort = _hold_vel(session, cam, command[0], command[1], forward_hold_s, sent)
    if abort:
        note = abort
    else:
        note = "brief forward on a centered backsplash, then stop"
    session.bus.stop(float(session.data.time))
    _hold_stand(session, SETTLE_S)
    after = cam.grab(session.data)
    final = score_frame(session.model, session.data, after)
    return Attempt(
        before=before,
        after=after,
        initial=initial,
        final=final,
        sent=sent,
        note=note,
        min_up_z=session.min_up_z,
        end_mode=session.bus.mode,
        fault=session.bus.fault or (abort is not None and abort.startswith("fault:")),
    )


def _abort_kind(abort: str) -> BurstKind:
    if abort == BUDGET_STOP:
        return "budget"
    if abort.startswith("fault:") or "up_z" in abort:
        return "tip"
    return "blob"


def _gap(model: mj.MjModel, data: mj.MjData) -> tuple[float, float]:
    found = kitchen_clearance(model, data)
    if found is None:
        return float("inf"), float("nan")
    return found


def run_approach(session: steer_walk.SteerSession, cam: KitCam) -> Approach:
    """Half-cap slices while the backsplash stays valid, with a yaw trim if it drifts.

    The gait stays in move across a slice boundary. A stand between slices
    resets the velocity slew and does not advance. Stop is the budget, or a
    Prefer FAIL. up_z under 0.90 stops. That bar is not lowered.
    """
    _hold_stand(session, STAND_S)
    before = cam.grab(session.data)
    score = score_frame(session.model, session.data, before)
    stand_score = score
    x0 = float(session.data.qpos[0])
    sent: list[SentVel] = []
    mid_image: np.ndarray | None = None
    mid_score = stand_score
    forward_bursts = 0
    yaw_corrections = 0
    note = stand_score.line
    stop_kind: BurstKind = "blob"
    if stand_score.decision == "fail":
        return _approach_result(
            session, cam, before, before, stand_score, stand_score, sent, note,
            stop_kind, x0, forward_bursts, yaw_corrections, False,
        )
    while True:
        dx = float(session.data.qpos[0]) - x0
        up_z = session.samples[-1].up_z if session.samples else 1.0
        remaining, _near = _gap(session.model, session.data)
        choice = approach_choice(
            score,
            dx_m=dx,
            up_z=up_z,
            yaw_corrections=yaw_corrections,
            forward_bursts=forward_bursts,
            remaining_m=remaining,
        )
        if choice.action == "stop":
            note = choice.line
            stop_kind = choice.kind
            break
        command = correction_command(score)
        if command is None:
            note = "Prefer FAIL: no bus command for this score"
            stop_kind = "blob"
            break
        if abs(command[1]) > 1e-9:
            yaw_corrections += 1
            hold_s = BURST_YAW_S
        else:
            forward_bursts += 1
            hold_s = BURST_FORWARD_S
        abort = _hold_vel(
            session, cam, command[0], command[1], hold_s, sent,
            stop_when_centered=False,
            progress_x0=x0,
            progress_cap_m=PROGRESS_STOP_M,
        )
        score = score_frame(session.model, session.data, cam.grab(session.data))
        moved = float(session.data.qpos[0]) - x0
        if mid_image is None and moved >= (PROGRESS_STOP_M * 0.5):
            mid_image = cam.grab(session.data)
            mid_score = score
        if abort:
            note = abort
            stop_kind = _abort_kind(abort)
            break
    if mid_image is None and (forward_bursts + yaw_corrections) > 1:
        mid_image = cam.grab(session.data)
        mid_score = score
    if mid_image is None:
        mid_image = before
        mid_score = stand_score
    return _approach_result(
        session, cam, before, mid_image, stand_score, mid_score, sent, note,
        stop_kind, x0, forward_bursts, yaw_corrections, stop_kind == "arrival",
    )


def _approach_result(
    session: steer_walk.SteerSession,
    cam: KitCam,
    before: np.ndarray,
    mid_image: np.ndarray,
    stand_score: KitchenScore,
    mid_score: KitchenScore,
    sent: list[SentVel],
    note: str,
    stop_kind: BurstKind,
    x0: float,
    forward_bursts: int,
    yaw_corrections: int,
    arrival_candidate: bool,
) -> Approach:
    session.bus.stop(float(session.data.time))
    _hold_stand(session, SETTLE_S)
    after = cam.grab(session.data)
    final = score_frame(session.model, session.data, after)
    remaining, near_face_x = _gap(session.model, session.data)
    end_x = float(session.data.qpos[0])
    arrival = arrival_bars(final.yellow_frac, remaining) and math.isfinite(remaining)
    if arrival:
        note = (
            "arrival bars met on the stop frame: backsplash fills at least half "
            f"the frame and torso-to-kitchen is {remaining:.3f} m"
        )
        stop_kind = "arrival"
    elif arrival_candidate:
        note = (
            "Prefer FAIL: the stop frame does not still meet both arrival bars; "
            "not arrival"
        )
        stop_kind = "close"
    elif stop_kind == "budget":
        note = (
            f"burst budget; dx {end_x - x0:+.3f} m; "
            f"remaining torso-to-kitchen {remaining:.3f} m; not arrival"
        )
    fault = session.bus.fault or note.startswith("fault:")
    return Approach(
        before=before,
        mid=mid_image,
        after=after,
        initial=stand_score,
        mid_score=mid_score,
        final=final,
        sent=sent,
        note=note,
        stop_kind=stop_kind,
        min_up_z=session.min_up_z,
        end_mode=session.bus.mode,
        fault=fault,
        x0_m=x0,
        end_x_m=end_x,
        end_y_m=float(session.data.qpos[1]),
        end_yaw_rad=session.yaw(),
        dx_m=end_x - x0,
        remaining_m=remaining,
        near_face_x_m=near_face_x,
        arrival=arrival,
        forward_bursts=forward_bursts,
        yaw_corrections=yaw_corrections,
    )


def _save_png(image: np.ndarray, path: Path) -> None:
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(path)


def _collapse(sent: list[SentVel]) -> list[dict[str, float | int]]:
    groups: list[dict[str, float | int]] = []
    for command in sent:
        if (
            groups
            and groups[-1]["vx"] == command.vx
            and groups[-1]["yaw_rate"] == command.yaw_rate
        ):
            groups[-1]["resends"] = int(groups[-1]["resends"]) + 1
            groups[-1]["t1"] = command.t
            continue
        groups.append({
            "t0": command.t,
            "t1": command.t,
            "vx": command.vx,
            "yaw_rate": command.yaw_rate,
            "resends": 1,
        })
    return groups


def _score_dict(score: KitchenScore) -> dict[str, object]:
    return {
        "decision": score.decision,
        "reason": score.reason,
        "line": score.line,
        "needs_map": score.needs_map,
        "bias": score.bias,
        "yellow_frac": score.yellow_frac,
        "centroid_u": score.centroid_u,
        "centroid_v": score.centroid_v,
        "kitchen_in_frame": score.kitchen_in_frame,
    }


def _attempt_dict(attempt: Attempt) -> dict[str, object]:
    return {
        "initial": _score_dict(attempt.initial),
        "final": _score_dict(attempt.final),
        "note": attempt.note,
        "min_up_z": attempt.min_up_z,
        "end_mode": attempt.end_mode,
        "fault": attempt.fault,
        "n_resends": len(attempt.sent),
        "commands": _collapse(attempt.sent),
    }


def _commands_legal(sent: list[SentVel], *, allow_walk_yaw: bool = False) -> str | None:
    for command in sent:
        if not _within_caps(command.vx, command.yaw_rate):
            return f"command outside caps vx={command.vx} yaw={command.yaw_rate}"
        if command.vx < 0.0:
            return "finder sent reverse"
        if allow_walk_yaw:
            if abs(command.vx - SOFT_VX) > 1e-9:
                return "approach forward is half the bus cap"
            continue
        if command.vx != 0.0 and command.yaw_rate != 0.0:
            return "yaw and forward were sent together"
    return None


def _paint(cx: int, cy: int, box_w: int, box_h: int, color: tuple[int, int, int]) -> np.ndarray:
    image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    image[:, :] = (70, 85, 110)
    u0 = max(0, cx - box_w // 2)
    u1 = min(WIDTH, cx + box_w // 2)
    v0 = max(0, cy - box_h // 2)
    v1 = min(HEIGHT, cy + box_h // 2)
    image[v0:v1, u0:u1] = color
    return image


def _expect(cond: bool, msg: str, failures: list[str]) -> None:
    if not cond:
        failures.append(msg)


def _fake_score(
    decision: DecisionName,
    frac: float = 0.04,
    bias: float = 0.0,
) -> KitchenScore:
    return KitchenScore(
        decision=decision,
        reason="test",
        line="Prefer FAIL: test blob" if decision == "fail" else "test",
        needs_map=False,
        bias=bias,
        yellow_frac=frac,
        centroid_u=None if decision == "fail" else 320.0,
        centroid_v=None if decision == "fail" else 140.0,
        kitchen_in_frame=decision != "fail",
    )


def test_approach_policy() -> list[str]:
    failures: list[str] = []
    centered = _fake_score("forward", 0.04, -0.01)
    left = _fake_score("yaw_left", 0.05, -0.30)
    lost = _fake_score("fail", 0.0, 0.0)
    full = _fake_score("forward", 0.62, 0.0)
    go = approach_choice(
        centered, dx_m=0.10, up_z=0.98, yaw_corrections=0,
        forward_bursts=0, remaining_m=1.20,
    )
    _expect(go.action == "forward" and go.kind == "steer", f"open burst {go}", failures)
    earlier = approach_choice(
        centered, dx_m=0.60, up_z=0.98, yaw_corrections=0,
        forward_bursts=3, remaining_m=0.70,
    )
    _expect(earlier.action == "forward", f"0.60 m is not the budget {earlier}", failures)
    budget = approach_choice(
        centered, dx_m=PROGRESS_STOP_M, up_z=0.98, yaw_corrections=0,
        forward_bursts=3, remaining_m=0.40,
    )
    _expect(budget.action == "stop" and budget.kind == "budget", f"budget {budget}", failures)
    yaw = approach_choice(
        left, dx_m=0.20, up_z=0.97, yaw_corrections=0,
        forward_bursts=1, remaining_m=1.00,
    )
    _expect(yaw.action == "yaw_left", f"yaw correct {yaw}", failures)
    hunted = approach_choice(
        left, dx_m=0.20, up_z=0.97, yaw_corrections=MAX_YAW_CORRECTIONS,
        forward_bursts=1, remaining_m=1.00,
    )
    _expect(hunted.action == "stop" and hunted.kind == "yaw_limit", f"yaw limit {hunted}", failures)
    _expect(trim_yaw(0.0) == 0.0, "center trim", failures)
    _expect(trim_yaw(0.05) == 0.0, "in-band trim", failures)
    right = trim_yaw(0.35)
    left = trim_yaw(-0.35)
    _expect(abs(right + steer_walk.YAW_RATE_CAP) < 1e-9, f"full right {right}", failures)
    _expect(abs(left - steer_walk.YAW_RATE_CAP) < 1e-9, f"full left {left}", failures)
    mild = trim_yaw(0.14)
    _expect(mild < 0.0 and abs(mild) < steer_walk.YAW_RATE_CAP, f"mild trim {mild}", failures)
    tip = approach_choice(
        centered, dx_m=0.20, up_z=0.89, yaw_corrections=0,
        forward_bursts=1, remaining_m=1.00,
    )
    _expect(tip.action == "stop" and tip.kind == "tip", f"tip {tip}", failures)
    blob = approach_choice(
        lost, dx_m=0.20, up_z=0.97, yaw_corrections=0,
        forward_bursts=1, remaining_m=1.00,
    )
    _expect(blob.action == "stop" and blob.kind == "blob", f"blob {blob}", failures)
    # Half the frame but still far: not arrival, keep steering.
    far_fill = approach_choice(
        full, dx_m=0.20, up_z=0.97, yaw_corrections=0,
        forward_bursts=1, remaining_m=0.80,
    )
    _expect(far_fill.action == "forward" and far_fill.kind == "steer", f"far fill {far_fill}", failures)
    # Close but the frame is not mostly backsplash: stop, do not claim arrival.
    close = approach_choice(
        centered, dx_m=0.20, up_z=0.97, yaw_corrections=0,
        forward_bursts=1, remaining_m=0.20,
    )
    _expect(close.action == "stop" and close.kind == "close", f"close {close}", failures)
    arrived = approach_choice(
        full, dx_m=0.20, up_z=0.97, yaw_corrections=0,
        forward_bursts=1, remaining_m=0.20,
    )
    _expect(arrived.action == "stop" and arrived.kind == "arrival", f"arrival {arrived}", failures)
    _expect(not arrival_bars(0.039, 1.25), "stand frame was called arrival", failures)
    _expect(not arrival_bars(0.80, 0.40), "full frame at 0.40 m was called arrival", failures)
    _expect(not arrival_bars(0.20, 0.10), "close but small blob was called arrival", failures)
    _expect(arrival_bars(0.50, 0.25), "both bars at the threshold did not pass", failures)
    _expect(arrival_bars(0.62, 0.20), "clear arrival bars did not pass", failures)
    return failures


def test_pixels() -> list[str]:
    failures: list[str] = []
    yellow = (150, 140, 50)
    left = read_pixels(_paint(120, 160, 160, 40, yellow))
    right = read_pixels(_paint(520, 160, 160, 40, yellow))
    center = read_pixels(_paint(320, 150, 140, 40, yellow))
    empty = read_pixels(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8))
    _expect(left.decision == "yaw_left", f"left blob {left.decision} bias {left.bias:+.2f}", failures)
    _expect(right.decision == "yaw_right", f"right blob {right.decision} bias {right.bias:+.2f}", failures)
    _expect(center.decision == "forward", f"center blob {center.decision} bias {center.bias:+.2f}", failures)
    _expect(empty.decision == "fail" and empty.frac == 0.0, "empty image did not fail", failures)
    split = _paint(100, 160, 70, 36, yellow)
    split[140:180, 500:570] = yellow
    split_read = read_pixels(split)
    _expect(split_read.bimodal and split_read.decision == "fail", "split blob was not ambiguous", failures)
    _expect(command_for("fail") is None, "fail produced a command", failures)
    _expect(command_for("yaw_left") == (0.0, steer_walk.YAW_RATE_CAP), "yaw left command", failures)
    _expect(command_for("yaw_right") == (0.0, -steer_walk.YAW_RATE_CAP), "yaw right command", failures)
    _expect(command_for("forward") == (steer_walk.VX_FWD_CAP, 0.0), "forward command", failures)
    soft = correction_command(_fake_score("forward", 0.04, 0.0))
    _expect(soft == (SOFT_VX, 0.0), f"soft forward {soft}", failures)
    _expect(abs(SOFT_VX - 0.028) < 1e-9 and SOFT_VX < steer_walk.VX_FWD_CAP, "soft vx", failures)
    _expect(steer_walk.VX_FWD_CAP == 0.056 and steer_walk.VX_BACK_CAP == 0.032, "vx caps moved", failures)
    _expect(steer_walk.YAW_RATE_CAP == 0.25, "yaw cap moved", failures)
    return failures


def test_phrases() -> list[str]:
    failures: list[str] = []
    cases: tuple[tuple[str, SceneName, PhraseAction, str], ...] = (
        ("go to the kitchen", "room", "find", FIND_LINE),
        ("go to kitchen", "room", "find", FIND_LINE),
        ("Go to the kitchen.", "room", "find", FIND_LINE),
        ("please go to the kitchen", "room", "find", FIND_LINE),
        ("go to the kitchen", "plant", "refuse", EMPTY_SCENE_LINE),
        ("go to kitchen", "plant", "refuse", EMPTY_SCENE_LINE),
        ("go to the bathroom", "room", "refuse", BATH_LINE),
        ("go to the bathroom", "plant", "refuse", BATH_LINE),
        ("go to the bedroom", "room", "refuse", OTHER_ROOM_LINE),
        ("go to the kitchen using the map", "room", "refuse", MAP_LINE),
        ("open the door", "room", "refuse", DOOR_LINE),
        ("strafe left", "room", "refuse", VY_LINE),
        ("walk forward", "room", "refuse", UNKNOWN_LINE),
        ("", "room", "refuse", EMPTY_PHRASE_LINE),
    )
    for phrase, scene, action, line in cases:
        got = phrase_hook(phrase, scene=scene)
        _expect(
            got.action == action and got.line == line,
            f"{phrase!r} scene={scene} got {got.action} {got.line}",
            failures,
        )
    return failures


def test_scenes() -> list[str]:
    failures: list[str] = []
    problem = plant_md5_ok()
    _expect(problem is None, problem or "", failures)
    if failures:
        return failures
    room = steer_walk.SteerSession(video=False, scene_xml=ROOM_XML)
    away = steer_walk.SteerSession(
        video=False, scene_xml=ROOM_XML, initial_yaw=math.pi,
    )
    side = steer_walk.SteerSession(
        video=False, scene_xml=ROOM_XML, initial_yaw=math.radians(-18.0),
    )
    empty = steer_walk.SteerSession(video=False)
    sessions = (room, away, side, empty)
    cams = [KitCam(session.model) for session in sessions]
    try:
        for session in sessions:
            _hold_stand(session, STAND_S)
        room_score = score_frame(room.model, room.data, cams[0].grab(room.data))
        away_score = score_frame(away.model, away.data, cams[1].grab(away.data))
        side_score = score_frame(side.model, side.data, cams[2].grab(side.data))
        empty_score = score_frame(empty.model, empty.data, cams[3].grab(empty.data))
    finally:
        for cam in cams:
            cam.close()
    _expect(
        room_score.decision == "forward" and room_score.kitchen_in_frame,
        f"stand pose {room_score.decision} bias {room_score.bias:+.3f} "
        f"yellow {room_score.yellow_frac:.3f} {room_score.reason}",
        failures,
    )
    _expect(not room.bus.fault, f"stand fault {room.bus.fault_reason}", failures)
    _expect(
        away_score.decision == "fail" and away_score.needs_map,
        f"yaw-away {away_score.decision} {away_score.line}",
        failures,
    )
    _expect(
        side_score.decision == "yaw_left" and side_score.bias < -CENTER_BIAS,
        f"left bias {side_score.decision} bias {side_score.bias:+.3f}",
        failures,
    )
    _expect(
        empty_score.decision == "fail" and empty_score.needs_map and not empty_score.kitchen_in_frame,
        f"empty plant {empty_score.decision} {empty_score.line}",
        failures,
    )
    _expect(_md5(PLANT_XML) == steer_walk.PLANT_MD5, "plant md5 changed during score", failures)
    gap = kitchen_clearance(room.model, room.data)
    _expect(gap is not None, "stand pose has no kitchen clearance", failures)
    if gap is not None:
        _expect(
            1.00 <= gap[0] <= 1.60,
            f"stand torso-to-kitchen {gap[0]:.3f} m is not the open-room gap",
            failures,
        )
        _expect(gap[1] > 1.0, f"kitchen near face x {gap[1]:.3f}", failures)
    return failures


def test_short_hop() -> list[str]:
    """Regression: the 1.2 s single forward hop, not the multi-burst approach."""
    failures: list[str] = []
    session = steer_walk.SteerSession(video=False, scene_xml=ROOM_XML)
    cam = KitCam(session.model)
    try:
        attempt = run_attempt(
            session, cam, yaw_hold_s=YAW_HOLD_S, forward_hold_s=FORWARD_HOLD_S,
        )
    finally:
        cam.close()
    session.assert_plant_unchanged()
    _expect(FORWARD_HOLD_S == 1.20, "short hop duration moved", failures)
    _expect(attempt.initial.decision == "forward", f"hop start {attempt.initial.decision}", failures)
    _expect(not attempt.fault, f"hop fault {attempt.note}", failures)
    _expect(attempt.end_mode == "stand", f"hop end {attempt.end_mode}", failures)
    _expect(attempt.min_up_z >= UP_Z_ABORT, f"hop min up_z {attempt.min_up_z:.3f}", failures)
    _expect(len(attempt.sent) > 0, "hop sent no vel", failures)
    _expect(
        all(command.vx == steer_walk.VX_FWD_CAP and command.yaw_rate == 0.0 for command in attempt.sent),
        "hop did not stay on forward-only vel",
        failures,
    )
    if len(attempt.sent) >= 2:
        span = attempt.sent[-1].t - attempt.sent[0].t
        _expect(span <= FORWARD_HOLD_S + 0.05, f"hop span {span:.2f} s exceeded 1.2 s", failures)
    end_x = float(session.data.qpos[0])
    _expect(end_x < 0.15, f"hop end x {end_x:.3f} m is no longer the short hop", failures)
    _expect(
        attempt.note == "brief forward on a centered backsplash, then stop",
        f"hop note {attempt.note}",
        failures,
    )
    print(
        f"[find] short hop {len(attempt.sent)} resends end x {end_x:+.3f} m "
        f"note {attempt.note}"
    )
    return failures


def test_biased_yaw() -> list[str]:
    """Kitchen on the left of the frame. One short +yaw, then stop if still off center."""
    failures: list[str] = []
    session = steer_walk.SteerSession(
        video=False, scene_xml=ROOM_XML, initial_yaw=math.radians(-18.0),
    )
    cam = KitCam(session.model)
    try:
        attempt = run_attempt(session, cam, yaw_hold_s=YAW_HOLD_S, forward_hold_s=FORWARD_HOLD_S)
    finally:
        cam.close()
    session.assert_plant_unchanged()
    _expect(attempt.initial.decision == "yaw_left", f"biased start {attempt.initial.decision}", failures)
    _expect(len(attempt.sent) > 0, "biased frame sent no vel", failures)
    if attempt.sent:
        first = attempt.sent[0]
        _expect(first.vx == 0.0 and first.yaw_rate == steer_walk.YAW_RATE_CAP, f"first vel {first}", failures)
    illegal = _commands_legal(attempt.sent)
    _expect(illegal is None, illegal or "", failures)
    _expect(not attempt.fault, f"biased fault {attempt.note}", failures)
    _expect(attempt.end_mode == "stand", f"biased end mode {attempt.end_mode}", failures)
    _expect(attempt.min_up_z >= 0.90, f"biased min up_z {attempt.min_up_z:.3f}", failures)
    yaw_commands = [command for command in attempt.sent if command.yaw_rate != 0.0]
    forward_commands = [command for command in attempt.sent if command.vx != 0.0]
    _expect(len(yaw_commands) > 0, "biased frame sent no yaw", failures)
    _expect(
        all(command.yaw_rate > 0.0 and command.vx == 0.0 for command in yaw_commands),
        "yaw was not +cap toward the left blob",
        failures,
    )
    if forward_commands:
        _expect(
            yaw_commands and forward_commands[0].t >= yaw_commands[-1].t - 1e-9,
            "forward was sent before the yaw",
            failures,
        )
    print(
        f"[find] biased trial {attempt.initial.decision} bias {attempt.initial.bias:+.3f} "
        f"resends {len(attempt.sent)} note {attempt.note}"
    )
    return failures


def self_test() -> int:
    failures: list[str] = []
    failures.extend(test_pixels())
    failures.extend(test_phrases())
    failures.extend(test_approach_policy())
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    failures.extend(test_scenes())
    failures.extend(test_short_hop())
    failures.extend(test_biased_yaw())
    digest = _md5(PLANT_XML)
    if digest != steer_walk.PLANT_MD5:
        failures.append(f"plant md5 {digest}")
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[find] self-test PASS")
    return 0


def _write_summary(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _copy_artifacts(paths: tuple[Path, ...]) -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    for path in paths:
        target = ARTIFACTS / path.name
        target.write_bytes(path.read_bytes())


def run_phrase(phrase: str, scene: SceneName) -> int:
    problem = plant_md5_ok()
    if problem:
        print(f"FAIL: {problem}")
        return 1
    hook = phrase_hook(phrase, scene=scene)
    print(hook.line)
    if hook.action == "refuse":
        if scene == "plant" and normalize_phrase(phrase) in KITCHEN_PHRASES:
            session = steer_walk.SteerSession(video=False)
            cam = KitCam(session.model)
            try:
                _hold_stand(session, STAND_S)
                image = cam.grab(session.data)
                score = score_frame(session.model, session.data, image)
            finally:
                cam.close()
            if score.decision != "fail":
                print(f"FAIL: empty plant scored {score.decision}")
                return 1
            out = PREVIEWS / "find_kitchen_prefer_fail.png"
            _save_png(image, out)
            _copy_artifacts((out,))
            print(f"prefer fail still {out.relative_to(ROOT)}  no vel")
        return 0

    before_md5 = _md5(PLANT_XML)
    room = steer_walk.SteerSession(video=False, scene_xml=ROOM_XML)
    room_cam = KitCam(room.model)
    try:
        approach = run_approach(room, room_cam)
    finally:
        room_cam.close()
    room.assert_plant_unchanged()

    empty = steer_walk.SteerSession(video=False)
    empty_cam = KitCam(empty.model)
    try:
        _hold_stand(empty, STAND_S)
        fail_image = empty_cam.grab(empty.data)
        fail_score = score_frame(empty.model, empty.data, fail_image)
    finally:
        empty_cam.close()

    before_path = PREVIEWS / "find_kitchen_before.png"
    mid_path = PREVIEWS / "find_kitchen_mid.png"
    after_path = PREVIEWS / "find_kitchen_after.png"
    fail_path = PREVIEWS / "find_kitchen_prefer_fail.png"
    summary_path = PREVIEWS / "find_kitchen_summary.json"
    _save_png(approach.before, before_path)
    _save_png(approach.mid, mid_path)
    _save_png(approach.after, after_path)
    _save_png(fail_image, fail_path)

    illegal = _commands_legal(approach.sent, allow_walk_yaw=True)
    after_md5 = _md5(PLANT_XML)
    bars_now = arrival_bars(approach.final.yellow_frac, approach.remaining_m)
    problems: list[str] = []
    if approach.initial.decision == "fail":
        problems.append(f"stand pose refused: {approach.initial.line}")
    if approach.fault:
        problems.append(f"steer fault: {approach.note}")
    if illegal:
        problems.append(illegal)
    if not approach.sent:
        problems.append("kitchen was in frame but no vel was sent")
    if fail_score.decision != "fail":
        problems.append(f"empty plant scored {fail_score.decision}")
    if after_md5 != before_md5 or after_md5 != steer_walk.PLANT_MD5:
        problems.append(f"plant md5 changed {before_md5} -> {after_md5}")
    if approach.end_mode != "stand":
        problems.append(f"end mode {approach.end_mode}")
    if approach.arrival != bars_now:
        problems.append(
            f"arrival flag {approach.arrival} does not match the bars "
            f"(yellow {approach.final.yellow_frac:.3f}, remaining {approach.remaining_m:.3f} m)"
        )
    if approach.arrival:
        problems.append("arrival was claimed; this draft does not expect the bars to pass")
    if approach.stop_kind == "budget" and not (1.00 <= approach.dx_m <= 1.25):
        problems.append(f"budget stop dx {approach.dx_m:.3f} m is outside 1.00–1.25")
    if approach.stop_kind == "budget" and approach.min_up_z < UP_Z_ABORT:
        problems.append(f"budget stop min up_z {approach.min_up_z:.3f}")

    payload: dict[str, object] = {
        "plant": str(PLANT_XML.relative_to(ROOT)),
        "plant_md5": after_md5,
        "kit_cam_pos": list(steer_walk.KIT_CAM_POS),
        "kit_cam_fovy": steer_walk.KIT_CAM_FOVY,
        "scene": str(ROOM_XML.relative_to(ROOT)),
        "phrase": phrase,
        "vx_fwd_cap": steer_walk.VX_FWD_CAP,
        "approach_vx": SOFT_VX,
        "vx_back_cap": steer_walk.VX_BACK_CAP,
        "yaw_rate_cap": steer_walk.YAW_RATE_CAP,
        "resend_hz": 1.0 / steer_walk.VEL_RESEND_S,
        "short_hop_s": FORWARD_HOLD_S,
        "burst_forward_s": BURST_FORWARD_S,
        "progress_stop_m": PROGRESS_STOP_M,
        "arrival_yellow_frac": ARRIVAL_YELLOW_FRAC,
        "arrival_remaining_m": ARRIVAL_REMAINING_M,
        "arrival": approach.arrival,
        "stop_kind": approach.stop_kind,
        "note": approach.note,
        "forward_bursts": approach.forward_bursts,
        "yaw_corrections": approach.yaw_corrections,
        "x0_m": approach.x0_m,
        "end_x_m": approach.end_x_m,
        "end_y_m": approach.end_y_m,
        "end_yaw_rad": approach.end_yaw_rad,
        "dx_m": approach.dx_m,
        "remaining_m": approach.remaining_m,
        "near_face_x_m": approach.near_face_x_m,
        "min_up_z": approach.min_up_z,
        "initial": _score_dict(approach.initial),
        "mid": _score_dict(approach.mid_score),
        "final": _score_dict(approach.final),
        "commands": _collapse(approach.sent),
        "prefer_fail_empty_plant": _score_dict(fail_score),
        "stills": {
            "before": str(before_path.relative_to(ROOT)),
            "mid": str(mid_path.relative_to(ROOT)),
            "after": str(after_path.relative_to(ROOT)),
            "prefer_fail": str(fail_path.relative_to(ROOT)),
        },
        "honesty": HONESTY,
    }
    _write_summary(summary_path, payload)
    _copy_artifacts((before_path, mid_path, after_path, fail_path))

    print(
        f"[find] stand {approach.initial.decision} bias {approach.initial.bias:+.3f} "
        f"yellow {approach.initial.yellow_frac:.3f}  {approach.note}"
    )
    for group in _collapse(approach.sent):
        print(
            f"  vel vx={float(group['vx']):+.3f} yaw_rate={float(group['yaw_rate']):+.3f} "
            f"t={float(group['t0']):.2f}..{float(group['t1']):.2f} "
            f"resends={int(group['resends'])}"
        )
    print(
        f"[find] stop {approach.stop_kind} bursts {approach.forward_bursts} "
        f"yaw_corrections {approach.yaw_corrections} "
        f"end mode {approach.end_mode} min_up_z {approach.min_up_z:.3f} "
        f"dx {approach.dx_m:+.3f} m end x {approach.end_x_m:+.3f} m "
        f"remaining {approach.remaining_m:.3f} m "
        f"arrival {approach.arrival} "
        f"empty-plant {fail_score.decision}  plant md5 {after_md5}"
    )
    if problems:
        for msg in problems:
            print(f"FAIL: {msg}")
        return 1
    print("[find] PASS  vision-reactive steer, not a map, not arrival")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Prefer FAIL find-kitchen from kit_cam")
    parser.add_argument("phrase", nargs="?", default="go to the kitchen")
    parser.add_argument("--scene", choices=("room", "plant"), default="room")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    scene: SceneName = "room" if args.scene == "room" else "plant"
    return run_phrase(args.phrase, scene)


if __name__ == "__main__":
    raise SystemExit(main())
