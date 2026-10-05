#!/usr/bin/env python3
"""Prefer FAIL find-kitchen from kit_cam pixels.

A phrase "go to the kitchen" / "go to kitchen" may steer only when
mujoco/room_kitchen.xml is loaded and the current kit_cam frame shows the
kitchen. The steer is stand / stop / vel on the CommandBus in
scripts/steer_walk.py. Caps stay +0.056 / -0.032 m/s and yaw ±0.25 rad/s.
vel is resent at 10 Hz. 200 ms of silence stands.

This is not a map, not SLAM, and not an arrival. The pixel blob picks yaw
left, yaw right, or a brief forward. Named kitchen / table / chair boxes
are a sim honesty check that the blob sits on the kitchen. They are not
waypoints and they are not given to a planner. If the backsplash is missing,
split, or off the kitchen body, the finder sends no vel.

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
PERCEPT_S = 0.40
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
    "Vision-reactive steer from one kit_cam frame. Not SLAM, not a map, "
    "not a waypoint planner, and not an arrival guarantee. "
    "The yellow backsplash blob picks the command. "
    "Projecting the named kitchen, table, and chair bodies only checks "
    "that the blob sits on the kitchen. A miss, a split blob, or an empty "
    "floor sends no vel."
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
    """Bus command for a passing score. None means no motion."""
    if decision == "forward":
        return (steer_walk.VX_FWD_CAP, 0.0)
    if decision == "yaw_left":
        return (0.0, steer_walk.YAW_RATE_CAP)
    if decision == "yaw_right":
        return (0.0, -steer_walk.YAW_RATE_CAP)
    return None


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
) -> str | None:
    """Resend one vel at 10 Hz. Stop early if the kitchen leaves the frame.

    A yaw hold also stops when the blob reaches the center, or when it
    crosses to the other side. Crossing does not reverse. That would be a search.
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


def _commands_legal(sent: list[SentVel]) -> str | None:
    for command in sent:
        if not _within_caps(command.vx, command.yaw_rate):
            return f"command outside caps vx={command.vx} yaw={command.yaw_rate}"
        if command.vx != 0.0 and command.yaw_rate != 0.0:
            return "yaw and forward were sent together"
        if command.vx < 0.0:
            return "finder sent reverse"
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
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    failures.extend(test_scenes())
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
        attempt = run_attempt(room, room_cam)
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
    after_path = PREVIEWS / "find_kitchen_after.png"
    fail_path = PREVIEWS / "find_kitchen_prefer_fail.png"
    summary_path = PREVIEWS / "find_kitchen_summary.json"
    _save_png(attempt.before, before_path)
    _save_png(attempt.after, after_path)
    _save_png(fail_image, fail_path)

    illegal = _commands_legal(attempt.sent)
    after_md5 = _md5(PLANT_XML)
    problems: list[str] = []
    if attempt.initial.decision == "fail":
        problems.append(f"stand pose refused: {attempt.initial.line}")
    if attempt.fault:
        problems.append(f"steer fault: {attempt.note}")
    if illegal:
        problems.append(illegal)
    if not attempt.sent:
        problems.append("kitchen was in frame but no vel was sent")
    if fail_score.decision != "fail":
        problems.append(f"empty plant scored {fail_score.decision}")
    if after_md5 != before_md5 or after_md5 != steer_walk.PLANT_MD5:
        problems.append(f"plant md5 changed {before_md5} -> {after_md5}")
    if attempt.min_up_z < 0.90:
        problems.append(f"min up_z {attempt.min_up_z:.3f}")
    if attempt.end_mode != "stand":
        problems.append(f"end mode {attempt.end_mode}")

    payload: dict[str, object] = {
        "plant": str(PLANT_XML.relative_to(ROOT)),
        "plant_md5": after_md5,
        "kit_cam_pos": list(steer_walk.KIT_CAM_POS),
        "kit_cam_fovy": steer_walk.KIT_CAM_FOVY,
        "scene": str(ROOM_XML.relative_to(ROOT)),
        "phrase": phrase,
        "vx_fwd_cap": steer_walk.VX_FWD_CAP,
        "vx_back_cap": steer_walk.VX_BACK_CAP,
        "yaw_rate_cap": steer_walk.YAW_RATE_CAP,
        "resend_hz": 1.0 / steer_walk.VEL_RESEND_S,
        "stand": _attempt_dict(attempt),
        "end_x_m": float(room.data.qpos[0]),
        "end_y_m": float(room.data.qpos[1]),
        "end_yaw_rad": room.yaw(),
        "prefer_fail_empty_plant": _score_dict(fail_score),
        "stills": {
            "before": str(before_path.relative_to(ROOT)),
            "after": str(after_path.relative_to(ROOT)),
            "prefer_fail": str(fail_path.relative_to(ROOT)),
        },
        "honesty": HONESTY,
    }
    _write_summary(summary_path, payload)
    _copy_artifacts((before_path, after_path, fail_path))

    print(
        f"[find] stand {attempt.initial.decision} bias {attempt.initial.bias:+.3f} "
        f"yellow {attempt.initial.yellow_frac:.3f}  {attempt.note}"
    )
    for group in _collapse(attempt.sent):
        print(
            f"  vel vx={float(group['vx']):+.3f} yaw_rate={float(group['yaw_rate']):+.3f} "
            f"t={float(group['t0']):.2f}..{float(group['t1']):.2f} "
            f"resends={int(group['resends'])}"
        )
    print(
        f"[find] end mode {attempt.end_mode} min_up_z {attempt.min_up_z:.3f} "
        f"end x {float(room.data.qpos[0]):+.3f} m "
        f"end yaw {math.degrees(room.yaw()):+.1f} deg "
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
