#!/usr/bin/env python3
"""Open-vocab room scores from a frozen kit_cam. Not a per-room colour bar.

google/owlvit-base-patch32 runs locally. Objects map to rooms in one
table, and one score cutoff applies to every scene. The measurement places
the stand pose on a straight line and reads kit_cam. It does not step the
gait and it does not publish vel. The kitchen slab path in find_kitchen.py
is not this script.

This pass does not claim arrival. It does not publish vel.
"""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco as mj
import numpy as np

import find_kitchen as fk
import steer_walk

ROOT = fk.ROOT
PLANT_XML = fk.PLANT_XML
PREVIEWS = ROOT / "previews"
ARTIFACTS = Path("/opt/cursor/artifacts")

MODEL_ID = "google/owlvit-base-patch32"
MODEL_REV = "cbc355fb364588351c5d51c7f74465e8e7ec6f72"
# One cutoff for every room. Kitchen cupboard is 0.072 and the door on that
# frame is 0.049. The empty plant peaks at 0.004. 0.06 sits in that gap.
# It is not a per-room threshold.
SCORE_MIN = 0.06
# Last poses on the scripted line. Each frame scores a room as its best
# object plus a small capped bonus for every other object at the cutoff.
# The window sums those frame scores. One margin for every room: the leader
# must clear the runner-up by this much, or the pose stays undecided.
# bed 0.102 versus bathtub 0.092 is a 0.010 gap and does not commit.
VOTE_FRAMES = 3
VOTE_MARGIN = 0.15
# Each extra object at or above the cutoff adds this much, and the extras
# stop at the cap. A pile of labels cannot add its full scores on top of
# the best hit. The cap is one cutoff.
OBJECT_BONUS = 0.03
OBJECT_BONUS_CAP = 0.06
# Bedroom only, after a straight pose stays undecided. Positive yaw faces
# the head of the bed. Negative yaw faces the dresser. Scripted looks.
# No vel. Wardrobe and rug stay off the map. Headboard scored on the door
# and on the kitchen, so it is not a prompt.
RELOOK_YAW = (0.55, 0.90, -0.75, -1.05)
# Object text, then the room it votes for. Room-name labels are not used.
# Cupboard is the cabinet hit on this kit_cam. Fridge, oven, stove, and
# kitchen sink stay in the map and score under the door.
OBJECTS: tuple[tuple[str, str], ...] = (
    ("fridge", "kitchen"),
    ("oven", "kitchen"),
    ("stove", "kitchen"),
    ("kitchen sink", "kitchen"),
    ("cupboard", "kitchen"),
    ("toilet", "bathroom"),
    ("bathtub", "bathroom"),
    ("bathroom sink", "bathroom"),
    ("bed", "bedroom"),
    ("pillow", "bedroom"),
    ("mattress", "bedroom"),
    ("upholstered bed", "bedroom"),
    ("sofa", "living"),
    ("TV", "living"),
    ("couch", "living"),
    ("door", "entrance"),
    ("hallway", "entrance"),
    ("doormat", "entrance"),
    ("shoe rack", "entrance"),
    ("coat hooks", "entrance"),
)
# Misses in a row that still reuse the last good box. Then the cue is lost.
# The gait walk is not this measurement. Detection uses scripted poses.
HOLD_SLICES = 4
# Straight kit_cam path. Freejoint x only. No vel and no gait step.
DETECT_X_M = (0.0, 0.3, 0.6, 0.9, 1.2)
SCENE_XML = {
    "kitchen": ROOT / "mujoco" / "room_kitchen.xml",
    "bathroom": ROOT / "mujoco" / "room_bathroom.xml",
    "living": ROOT / "mujoco" / "room_living.xml",
    "bedroom": ROOT / "mujoco" / "room_bedroom.xml",
    "entrance": ROOT / "mujoco" / "room_entrance.xml",
    "plant": None,
}
PHRASES = {
    "go to the kitchen": "kitchen",
    "go to kitchen": "kitchen",
    "go to the bathroom": "bathroom",
    "go to bathroom": "bathroom",
    "go to the living room": "living",
    "go to the bedroom": "bedroom",
    "go to the entrance": "entrance",
}

ARRIVAL_FRAC = 0.50
ARRIVAL_GAP_M = 0.25
UP_Z_ABORT = 0.90
# Path length, not a waypoint. Long enough to close a stand gap near 1.6 m
# if the heading is right, and short of a search.
PROGRESS_M = 2.00
SLICE_S = 0.40
STAND_S = fk.STAND_S
SETTLE_S = fk.SETTLE_S

# Stand frames with the object map and SCORE_MIN. Each room's own object
# wins. The empty plant stays under the cutoff.
STAND_TOP = {
    "kitchen": "kitchen",
    "bathroom": "bathroom",
    "living": "living",
    "bedroom": "bedroom",
    "entrance": "entrance",
    "plant": None,
}


@dataclass(frozen=True)
class Detection:
    room: str | None
    score: float
    frac: float
    bias: float
    scores: dict[str, float]
    box: tuple[float, float, float, float] | None
    object_name: str | None = None
    object_scores: dict[str, float] | None = None
    object_bias: dict[str, float] | None = None


@dataclass
class Walk:
    room: str
    before: np.ndarray
    after: np.ndarray
    stand: Detection
    final: Detection
    sent: list[fk.SentVel]
    stop_kind: str
    note: str
    min_up_z: float
    dx_m: float
    end_x_m: float
    end_y_m: float
    end_yaw_rad: float
    remaining_m: float | None
    arrival: bool
    stand_saturated: bool


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def room_of_phrase(phrase: str) -> str | None:
    return PHRASES.get(fk.normalize_phrase(phrase))


def command_from_bias(bias: float) -> tuple[float, float]:
    """Half-cap forward plus the kitchen trim. Never a pure yaw."""
    return (fk.SOFT_VX, fk.trim_yaw(bias))


def arrival_ok(stand_frac: float, stop_frac: float, gap_m: float | None) -> bool:
    """Both bars, and the cue was not already half the frame at the stand."""
    if gap_m is None or not math.isfinite(gap_m):
        return False
    if stand_frac >= ARRIVAL_FRAC:
        return False
    return stop_frac >= ARRIVAL_FRAC and gap_m <= ARRIVAL_GAP_M


def _box_gap(model: mj.MjModel, data: mj.MjData, geom_id: int, point_x: float, point_y: float) -> float:
    half_x = float(model.geom_size[geom_id][0])
    half_y = float(model.geom_size[geom_id][1])
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    delta = np.array([point_x - float(origin[0]), point_y - float(origin[1]), 0.0])
    local = rotation.T @ delta
    outside_x = max(abs(float(local[0])) - half_x, 0.0)
    outside_y = max(abs(float(local[1])) - half_y, 0.0)
    return math.hypot(outside_x, outside_y)


def body_gap(model: mj.MjModel, data: mj.MjData, room: str) -> float | None:
    """Horizontal torso gap to the named room body. Not a heading."""
    if room == "kitchen":
        found = fk.kitchen_clearance(model, data)
        return None if found is None else found[0]
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, room)
    torso_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if body_id < 0 or torso_id < 0:
        return None
    point_x = float(data.xpos[torso_id][0])
    point_y = float(data.xpos[torso_id][1])
    nearest = float("inf")
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) != body_id:
            continue
        if int(model.geom_type[geom_id]) == int(mj.mjtGeom.mjGEOM_BOX):
            gap = _box_gap(model, data, geom_id, point_x, point_y)
        else:
            origin = data.geom_xpos[geom_id]
            reach = float(model.geom_rbound[geom_id])
            gap = max(0.0, math.hypot(point_x - float(origin[0]), point_y - float(origin[1])) - reach)
        nearest = min(nearest, gap)
    if nearest == float("inf"):
        return None
    return nearest


def _box_metrics(box, image_w: int, image_h: int) -> tuple[float, float, tuple[float, float, float, float]]:
    cx, cy, width, height = (float(value) for value in box)
    x0 = (cx - width / 2.0) * image_w
    x1 = (cx + width / 2.0) * image_w
    y0 = (cy - height / 2.0) * image_h
    y1 = (cy + height / 2.0) * image_h
    x0 = min(max(x0, 0.0), float(image_w))
    x1 = min(max(x1, 0.0), float(image_w))
    y0 = min(max(y0, 0.0), float(image_h))
    y1 = min(max(y1, 0.0), float(image_h))
    frac = max(0.0, x1 - x0) * max(0.0, y1 - y0) / float(image_w * image_h)
    bias = (((x0 + x1) * 0.5) - (image_w * 0.5)) / (image_w * 0.5)
    return frac, bias, (x0, y0, x1, y1)


class RoomDetector:
    """One OWL-ViT. Object votes, one cutoff, local weights."""

    def __init__(self) -> None:
        import torch
        from transformers import OwlViTForObjectDetection, OwlViTProcessor

        torch.set_num_threads(4)
        self.torch = torch
        self.processor = OwlViTProcessor.from_pretrained(MODEL_ID, revision=MODEL_REV)
        self.model = OwlViTForObjectDetection.from_pretrained(MODEL_ID, revision=MODEL_REV)
        self.model.eval()
        self.texts = [name for name, _room in OBJECTS]

    def read(self, frame: np.ndarray) -> Detection:
        image_h, image_w = int(frame.shape[0]), int(frame.shape[1])
        inputs = self.processor(text=[self.texts], images=frame, return_tensors="pt")
        with self.torch.no_grad():
            outputs = self.model(**inputs)
        probs = self.torch.sigmoid(outputs.logits[0])
        best_q = probs.argmax(dim=0)
        best = probs.max(dim=0).values
        boxes = outputs.pred_boxes[0]
        scores = {room: 0.0 for room in STAND_TOP if room != "plant"}
        object_scores: dict[str, float] = {}
        object_bias: dict[str, float] = {}
        winner_i = 0
        winner_score = -1.0
        for index, (name, room) in enumerate(OBJECTS):
            score = float(best[index])
            object_scores[name] = score
            query = int(best_q[index])
            _frac, bias, _box = _box_metrics(boxes[query], image_w, image_h)
            object_bias[name] = bias
            if score > scores[room]:
                scores[room] = score
            if score > winner_score:
                winner_score = score
                winner_i = index
        name, room = OBJECTS[winner_i]
        if winner_score < SCORE_MIN:
            return Detection(None, winner_score, 0.0, 0.0, scores, None, None, object_scores, object_bias)
        _frac, bias, box = _box_metrics(boxes[int(best_q[winner_i])], image_w, image_h)
        return Detection(room, winner_score, _frac, bias, scores, box, name, object_scores, object_bias)


def _hold_slice(
    session: steer_walk.SteerSession,
    vx: float,
    yaw_rate: float,
    sent: list[fk.SentVel],
    x0: float,
    y0: float,
) -> str | None:
    if not fk._within_caps(vx, yaw_rate):
        return "refused: command outside the bus caps"
    if abs(vx) < 1e-9 and abs(yaw_rate) > 1e-9:
        return "refused: vx=0 yaw"
    end = float(session.data.time) + SLICE_S
    last_send = -1.0
    while float(session.data.time) < end - 1e-9:
        now = float(session.data.time)
        if (now - last_send) >= (steer_walk.VEL_RESEND_S - 1e-9):
            refusal = session.bus.vel(vx, yaw_rate, now)
            sent.append(fk.SentVel(now, vx, yaw_rate))
            last_send = now
            if refusal:
                return refusal
        session.step()
        if session.bus.fault:
            return f"fault: {session.bus.fault_reason}"
        if session.samples and session.samples[-1].up_z < UP_Z_ABORT:
            return f"Prefer FAIL: up_z {session.samples[-1].up_z:.3f} during the burst; stop"
        travelled = math.hypot(float(session.data.qpos[0]) - x0, float(session.data.qpos[1]) - y0)
        if travelled >= PROGRESS_M:
            return "budget"
    return None


def _paint_box(frame: np.ndarray, box: tuple[float, float, float, float] | None) -> np.ndarray:
    image = frame.copy()
    if box is None:
        return image
    x0, y0, x1, y1 = box
    height, width = image.shape[:2]
    x0i = int(max(0, min(width - 1, round(x0))))
    x1i = int(max(0, min(width - 1, round(x1))))
    y0i = int(max(0, min(height - 1, round(y0))))
    y1i = int(max(0, min(height - 1, round(y1))))
    image[y0i : y0i + 2, x0i:x1i] = (255, 40, 40)
    image[max(0, y1i - 2) : y1i, x0i:x1i] = (255, 40, 40)
    image[y0i:y1i, x0i : x0i + 2] = (255, 40, 40)
    image[y0i:y1i, max(0, x1i - 2) : x1i] = (255, 40, 40)
    return image


def walk_room(detector: RoomDetector, room: str) -> Walk:
    session = steer_walk.SteerSession(video=False, scene_xml=SCENE_XML[room])
    cam = fk.KitCam(session.model)
    sent: list[fk.SentVel] = []
    try:
        fk._hold_stand(session, STAND_S)
        before = cam.grab(session.data)
        stand = detector.read(before)
        x0 = float(session.data.qpos[0])
        y0 = float(session.data.qpos[1])
        gap = body_gap(session.model, session.data, room)
        saturated = stand.room == room and stand.frac >= ARRIVAL_FRAC
        stop_kind = "no_cue"
        note = "Prefer FAIL: open-vocab cue did not select this room; no vel"
        final = stand
        tracked = stand
        misses = 0
        if stand.room == room:
            while True:
                gap = body_gap(session.model, session.data, room)
                up_z = session.samples[-1].up_z if session.samples else 1.0
                if up_z < UP_Z_ABORT:
                    stop_kind = "tip"
                    note = f"Prefer FAIL: up_z {up_z:.3f} is below {UP_Z_ABORT:.2f}; stop"
                    break
                if arrival_ok(stand.frac, tracked.frac, gap):
                    stop_kind = "arrival"
                    note = "arrival bars met on the live frame"
                    break
                if gap is not None and gap <= ARRIVAL_GAP_M:
                    stop_kind = "close"
                    note = (
                        "Prefer FAIL: torso is within 0.25 m but the cue is not an approach cue; "
                        "not arrival"
                    )
                    break
                travelled = math.hypot(float(session.data.qpos[0]) - x0, float(session.data.qpos[1]) - y0)
                if travelled >= PROGRESS_M:
                    stop_kind = "budget"
                    note = f"path budget {PROGRESS_M:.2f} m reached; not arrival"
                    break
                vx, yaw_rate = command_from_bias(tracked.bias)
                abort = _hold_slice(session, vx, yaw_rate, sent, x0, y0)
                frame = cam.grab(session.data)
                seen = detector.read(frame)
                if seen.room == room:
                    tracked = seen
                    final = seen
                    misses = 0
                else:
                    misses += 1
                if abort == "budget":
                    stop_kind = "budget"
                    note = f"path budget {PROGRESS_M:.2f} m reached; not arrival"
                    break
                if abort and abort.startswith("Prefer FAIL: up_z"):
                    stop_kind = "tip"
                    note = abort
                    break
                if abort and abort.startswith("fault:"):
                    stop_kind = "tip"
                    note = abort
                    break
                if abort:
                    stop_kind = "lost"
                    note = abort
                    break
                if seen.room != room and misses > HOLD_SLICES:
                    final = seen
                    stop_kind = "lost"
                    note = "Prefer FAIL: open-vocab cue left this room; no further vel"
                    break
        session.bus.stop(float(session.data.time))
        fk._hold_stand(session, SETTLE_S)
        after = cam.grab(session.data)
        settled = detector.read(after)
        gap = body_gap(session.model, session.data, room)
        arrival = arrival_ok(stand.frac, settled.frac, gap)
        if arrival:
            stop_kind = "arrival"
            note = (
                "arrival bars met on the stop frame: cue fills at least half the frame "
                f"and torso-to-{room} is {gap:.3f} m"
            )
        elif stop_kind == "arrival":
            stop_kind = "close"
            note = "Prefer FAIL: the stop frame does not still meet both arrival bars; not arrival"
        return Walk(
            room=room,
            before=_paint_box(before, stand.box if stand.room == room else None),
            after=after,
            stand=stand,
            final=settled,
            sent=sent,
            stop_kind=stop_kind,
            note=note,
            min_up_z=session.min_up_z,
            dx_m=float(session.data.qpos[0]) - x0,
            end_x_m=float(session.data.qpos[0]),
            end_y_m=float(session.data.qpos[1]),
            end_yaw_rad=session.yaw(),
            remaining_m=gap,
            arrival=arrival,
            stand_saturated=saturated,
        )
    finally:
        cam.close()


def _expect(cond: bool, message: str, failures: list[str]) -> None:
    if not cond:
        failures.append(message)


def test_phrases() -> list[str]:
    failures: list[str] = []
    _expect(room_of_phrase("go to the bathroom") == "bathroom", "bathroom phrase", failures)
    _expect(room_of_phrase("please go to the kitchen") == "kitchen", "polite kitchen", failures)
    _expect(room_of_phrase("go to the living room") == "living", "living phrase", failures)
    _expect(room_of_phrase("go to the bedroom") == "bedroom", "bedroom phrase", failures)
    _expect(room_of_phrase("go to the entrance") == "entrance", "entrance phrase", failures)
    _expect(room_of_phrase("go to the moon") is None, "unknown phrase", failures)
    _expect(not arrival_ok(0.99, 0.99, 0.20), "saturated stand counted as arrival", failures)
    _expect(arrival_ok(0.04, 0.60, 0.20), "grown cue inside the gap was refused", failures)
    _expect(not arrival_ok(0.04, 0.60, 0.40), "gap bar was ignored", failures)
    _expect(not arrival_ok(0.04, 0.40, 0.20), "cue bar was ignored", failures)
    vx, yaw = command_from_bias(0.0)
    _expect(abs(vx - fk.SOFT_VX) < 1e-9 and yaw == 0.0, f"center command {vx} {yaw}", failures)
    vx, yaw = command_from_bias(-0.40)
    _expect(abs(vx - fk.SOFT_VX) < 1e-9 and 0.0 < yaw <= steer_walk.YAW_RATE_CAP, f"left command {vx} {yaw}", failures)
    _expect(_md5(PLANT_XML) == steer_walk.PLANT_MD5, "plant md5 changed", failures)
    _expect(_false_objects({"kitchen": 0.07, "bathroom": 0.01}, "kitchen") == [], "true room counted false", failures)
    _expect(_false_objects({"kitchen": 0.07, "bathroom": 0.08}, "kitchen") == ["bathroom"], "wrong room missed", failures)
    _expect(_false_objects({"living": 0.004}, "plant") == [], "plant under cutoff flagged", failures)
    step = _bearing_stats([0.1, 0.1, 0.4])["max_step"]
    _expect(step is not None and abs(step - 0.3) < 1e-9, "bearing step", failures)
    _expect("bus.vel" not in inspect.getsource(_set_scripted_pose), "scripted pose publishes vel", failures)
    close = {name: 0.0 for name, _room in OBJECTS}
    close["bed"] = 0.102
    close["bathtub"] = 0.092
    vote = decide_window([_frame(close)])
    _expect(vote.room is None, f"bed versus bathtub committed {vote}", failures)
    group = {name: 0.0 for name, _room in OBJECTS}
    group["toilet"] = 0.20
    group["bathroom sink"] = 0.10
    group["fridge"] = 0.15
    vote = decide_window([_frame(group)])
    _expect(vote.room is None and vote.gap < VOTE_MARGIN, f"thin toilet lead committed {vote}", failures)
    noise = {name: 0.01 for name, _room in OBJECTS}
    vote = decide_window([_frame(noise), _frame(noise), _frame(noise)])
    _expect(vote.room is None and vote.vote < SCORE_MIN, f"plant noise committed {vote}", failures)
    _expect(SCORE_MIN == 0.06, "cutoff moved", failures)
    _expect(VOTE_MARGIN == 0.15, "margin moved", failures)
    _expect(OBJECT_BONUS_CAP <= SCORE_MIN + 1e-9, "label cap exceeds the cutoff", failures)
    thin = {name: 0.0 for name, _room in OBJECTS}
    thin["bed"] = 0.10
    thin["pillow"] = 0.08
    thin["couch"] = 0.09
    thin["sofa"] = 0.08
    vote = decide_window([_frame(thin)])
    _expect(vote.room is None, f"thin bedroom lead committed {vote}", failures)
    stack = {name: 0.0 for name, _room in OBJECTS}
    stack["door"] = 0.16
    for name in ("bed", "pillow", "mattress", "upholstered bed"):
        stack[name] = 0.08
    vote = decide_window([_frame(stack)])
    _expect(vote.room != "bedroom", f"label stack outvoted the door {vote}", failures)
    clear = {name: 0.0 for name, _room in OBJECTS}
    clear["door"] = 0.20
    vote = decide_window([_frame(clear)])
    _expect(vote.room == "entrance", f"clear door did not commit {vote}", failures)
    return failures


def test_stand(detector: RoomDetector) -> list[str]:
    failures: list[str] = []
    for room, xml in SCENE_XML.items():
        session = steer_walk.SteerSession(video=False, scene_xml=xml)
        cam = fk.KitCam(session.model)
        try:
            _set_scripted_pose(session, 0.0)
            seen = detector.read(cam.grab(session.data))
        finally:
            cam.close()
        _expect(seen.room == STAND_TOP[room], f"{room} top {seen.room} scores {seen.scores}", failures)
        if room != "bathroom":
            _expect(seen.room != "bathroom", f"{room} selected bathroom", failures)
    return failures


def self_test() -> int:
    failures = test_phrases()
    detector = RoomDetector()
    failures.extend(test_stand(detector))
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[room] self-test PASS")
    print(f"[room] model {MODEL_ID} @ {MODEL_REV} score>={SCORE_MIN}")
    return 0


def _walk_dict(walk: Walk) -> dict[str, object]:
    return {
        "room": walk.room,
        "stop_kind": walk.stop_kind,
        "note": walk.note,
        "arrival": walk.arrival,
        "stand_saturated": walk.stand_saturated,
        "stand_room": walk.stand.room,
        "stand_score": walk.stand.score,
        "stand_frac": walk.stand.frac,
        "stand_bias": walk.stand.bias,
        "stand_object": walk.stand.object_name,
        "stand_scores": walk.stand.scores,
        "settled_room": walk.final.room,
        "settled_frac": walk.final.frac,
        "settled_score": walk.final.score,
        "remaining_m": walk.remaining_m,
        "min_up_z": walk.min_up_z,
        "dx_m": walk.dx_m,
        "end_x_m": walk.end_x_m,
        "end_y_m": walk.end_y_m,
        "end_yaw_rad": walk.end_yaw_rad,
        "n_commands": len(walk.sent),
        "commands": fk._collapse(walk.sent),
    }


def _set_scripted_pose(session: steer_walk.SteerSession, x: float, yaw: float = 0.0) -> None:
    """Place the frozen stand pose. Does not step the gait or publish vel."""
    session._reset_stand()
    session.data.qpos[0] = float(x)
    session.data.qpos[1] = 0.0
    half = 0.5 * float(yaw)
    session.data.qpos[3] = math.cos(half)
    session.data.qpos[6] = math.sin(half)
    session.data.qvel[:] = 0.0
    mj.mj_forward(session.model, session.data)


def _bearing_rad(bias: float, fovy_deg: float) -> float:
    focal = (fk.HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    return math.atan((bias * (fk.WIDTH / 2.0)) / focal)


def _false_objects(scores: dict[str, float], scene: str) -> list[str]:
    """Other rooms at or above the one cutoff. Empty plant has no true room."""
    return sorted(room for room, score in scores.items() if room != scene and score >= SCORE_MIN)


def _false_object_hits(object_scores: dict[str, float], scene: str) -> list[dict[str, object]]:
    """Wrong-room objects at or above the one cutoff, highest score first."""
    hits = [
        {"object": name, "room": room, "score": float(object_scores[name])}
        for name, room in OBJECTS
        if name in object_scores and room != scene and float(object_scores[name]) >= SCORE_MIN
    ]
    hits.sort(key=lambda row: float(row["score"]), reverse=True)
    return hits


@dataclass(frozen=True)
class Vote:
    room: str | None
    vote: float
    gap: float
    runner: str | None
    bearing: float | None


def _frame(object_scores: dict[str, float], object_bias: dict[str, float] | None = None) -> dict[str, object]:
    return {
        "object_scores": object_scores,
        "object_bias": object_bias or {name: 0.0 for name, _room in OBJECTS},
    }


def _frame_votes(object_scores: dict[str, float]) -> dict[str, float]:
    """Best object, plus a capped bonus for each other object at the cutoff.

    A room with nothing at the cutoff votes 0. Extra labels do not add
    their full scores.
    """
    rooms = [room for room in STAND_TOP if room != "plant"]
    best = {room: 0.0 for room in rooms}
    above: dict[str, list[float]] = {room: [] for room in rooms}
    for name, room in OBJECTS:
        score = float(object_scores.get(name, 0.0))
        if score >= SCORE_MIN:
            above[room].append(score)
            if score > best[room]:
                best[room] = score
    votes = {room: 0.0 for room in rooms}
    for room in rooms:
        if not above[room]:
            continue
        others = len(above[room]) - 1
        bonus = min(OBJECT_BONUS * others, OBJECT_BONUS_CAP)
        votes[room] = best[room] + bonus
    return votes


def _mean_bearing(frames: list[dict[str, object]], room: str, fovy_deg: float) -> float | None:
    """Score-weighted mean bearing of this room's objects that clear the cutoff."""
    total = 0.0
    weight = 0.0
    for frame in frames:
        scores = frame["object_scores"]
        bias = frame["object_bias"]
        for name, obj_room in OBJECTS:
            if obj_room != room:
                continue
            score = float(scores.get(name, 0.0))
            if score < SCORE_MIN:
                continue
            total += _bearing_rad(float(bias.get(name, 0.0)), fovy_deg) * score
            weight += score
    if weight <= 0.0:
        return None
    return total / weight


def decide_window(frames: list[dict[str, object]], fovy_deg: float = 104.82) -> Vote:
    """Commit a room from the last frames, or leave the pose undecided."""
    window = frames[-VOTE_FRAMES:]
    votes = {room: 0.0 for room in STAND_TOP if room != "plant"}
    for frame in window:
        for room, value in _frame_votes(frame["object_scores"]).items():
            votes[room] += value
    ranked = sorted(votes.items(), key=lambda item: item[1], reverse=True)
    winner, win_vote = ranked[0]
    runner, run_vote = ranked[1]
    gap = win_vote - run_vote
    if win_vote < SCORE_MIN or gap < VOTE_MARGIN:
        return Vote(None, win_vote, gap, runner, None)
    return Vote(winner, win_vote, gap, runner, _mean_bearing(window, winner, fovy_deg))


def _bearing_stats(bearings: list[float]) -> dict[str, float | None]:
    if len(bearings) < 2:
        return {"std": None, "max_step": None}
    mean = sum(bearings) / len(bearings)
    var = sum((value - mean) ** 2 for value in bearings) / len(bearings)
    steps = [abs(bearings[i] - bearings[i - 1]) for i in range(1, len(bearings))]
    return {"std": math.sqrt(var), "max_step": max(steps)}


def _read_pose(session, cam, detector: RoomDetector, x: float, yaw: float):
    _set_scripted_pose(session, x, yaw)
    seen = detector.read(cam.grab(session.data))
    frame = _frame(seen.object_scores or {}, seen.object_bias or {})
    return seen, frame


def _sample_record(x, yaw, seen: Detection, vote: Vote, fovy: float, scene: str, relook: bool) -> dict[str, object]:
    object_scores = seen.object_scores or {}
    raw_bearing = None if seen.room is None else _bearing_rad(seen.bias, fovy)
    return {
        "x_m": x,
        "yaw": yaw,
        "relook_pose": relook,
        "raw_object": seen.object_name,
        "raw_room": seen.room,
        "raw_score": seen.score,
        "room": vote.room,
        "vote": vote.vote,
        "gap": vote.gap,
        "runner": vote.runner,
        "bearing_rad": vote.bearing,
        "raw_bearing_rad": raw_bearing,
        "scores": seen.scores,
        "object_scores": object_scores,
        "object_bias": seen.object_bias or {},
        "false_objects": _false_object_hits(object_scores, scene),
        "hit": vote.room == scene,
    }


def _views(sample: dict[str, object]):
    yield sample
    for extra in sample.get("relook") or []:
        yield extra


def detect_paths(detector: RoomDetector) -> dict[str, object]:
    """Score kit_cam along a scripted line. No CommandBus vel."""
    scenes: list[dict[str, object]] = []
    for scene, xml in SCENE_XML.items():
        session = steer_walk.SteerSession(video=False, scene_xml=xml)
        cam = fk.KitCam(session.model)
        fovy = float(session.model.cam_fovy[cam.cam_id])
        samples: list[dict[str, object]] = []
        straight: list[dict[str, object]] = []
        try:
            for x in DETECT_X_M:
                seen, frame = _read_pose(session, cam, detector, x, 0.0)
                straight.append(frame)
                vote = decide_window(straight, fovy)
                sample = _sample_record(x, 0.0, seen, vote, fovy, scene, relook=False)
                sample["straight_room"] = vote.room
                sample["straight_gap"] = vote.gap
                sample["straight_hit"] = vote.room == scene
                sample["relook_intent"] = None
                sample["relook"] = []
                if scene == "bedroom" and vote.room is None:
                    sample["relook_intent"] = (
                        f"undecided; lead {vote.gap:.3f} under {VOTE_MARGIN:.2f}; keep looking"
                    )
                    looked = list(straight)
                    for yaw in RELOOK_YAW:
                        seen_y, frame_y = _read_pose(session, cam, detector, x, yaw)
                        looked.append(frame_y)
                        vote_y = decide_window(looked, fovy)
                        turned = _sample_record(x, yaw, seen_y, vote_y, fovy, scene, relook=True)
                        sample["relook"].append(turned)
                        vote = vote_y
                        seen = seen_y
                        if vote_y.room is not None:
                            break
                    sample.update({
                        "room": vote.room,
                        "vote": vote.vote,
                        "gap": vote.gap,
                        "runner": vote.runner,
                        "bearing_rad": vote.bearing,
                        "hit": vote.room == scene,
                        "yaw": sample["relook"][-1]["yaw"] if sample["relook"] else 0.0,
                    })
                samples.append(sample)
        finally:
            cam.close()
        hits = [sample for sample in samples if sample["hit"]]
        bearings = [float(sample["bearing_rad"]) for sample in hits if sample["bearing_rad"] is not None]
        stats = _bearing_stats(bearings)
        false_best: dict[str, dict[str, object]] = {}
        committed_false: dict[str, dict[str, object]] = {}
        object_max: dict[str, float] = {}
        for sample in samples:
            for view in _views(sample):
                for name, score in view["object_scores"].items():
                    object_max[name] = max(object_max.get(name, 0.0), float(score))
                for row in view["false_objects"]:
                    key = str(row["object"])
                    prev = false_best.get(key)
                    if prev is None or float(row["score"]) > float(prev["score"]):
                        false_best[key] = row
                    if view["room"] not in (None, scene):
                        committed_false[key] = false_best[key]
        if scene == "plant":
            true_object = None
            true_max = 0.0
        else:
            true_pairs = [(name, object_max.get(name, 0.0)) for name, room in OBJECTS if room == scene]
            true_object, true_max = max(true_pairs, key=lambda pair: pair[1])
        fired = sorted({
            str(name)
            for sample in hits
            for name, room in OBJECTS
            if room == scene and float(sample["object_scores"].get(name, 0.0)) >= SCORE_MIN
        })
        raw_false = sorted(false_best.values(), key=lambda row: float(row["score"]), reverse=True)
        wrong = sorted(committed_false.values(), key=lambda row: float(row["score"]), reverse=True)
        wrong_rooms = sorted({str(sample["room"]) for sample in samples if sample["room"] not in (None, scene)})
        scenes.append({
            "scene": scene,
            "poses": len(samples),
            "hits": len(hits),
            "straight_hits": sum(1 for sample in samples if sample["straight_hit"]),
            "relooks": sum(1 for sample in samples if sample["relook"]),
            "winner_object": true_object,
            "winner_score": true_max,
            "fired_objects": fired,
            "false_objects": wrong,
            "wrong_rooms": wrong_rooms,
            "raw_false_objects": raw_false,
            "min_gap": min((float(sample["gap"]) for sample in hits), default=None),
            "bearing_std_rad": stats["std"],
            "bearing_max_step_rad": stats["max_step"],
            "max_score": max(float(view["raw_score"]) for sample in samples for view in _views(sample)),
            "samples": samples,
        })
    plant = next(row for row in scenes if row["scene"] == "plant")
    return {
        "model": MODEL_ID,
        "model_rev": MODEL_REV,
        "score_min": SCORE_MIN,
        "vote_frames": VOTE_FRAMES,
        "vote_margin": VOTE_MARGIN,
        "object_bonus": OBJECT_BONUS,
        "object_bonus_cap": OBJECT_BONUS_CAP,
        "objects": [{"name": name, "room": room} for name, room in OBJECTS],
        "path_x_m": list(DETECT_X_M),
        "relook_yaw": list(RELOOK_YAW),
        "gait": False,
        "vel": False,
        "arrival": False,
        "plant_md5": _md5(PLANT_XML),
        "plant_under_cutoff": float(plant["max_score"]) < SCORE_MIN and plant["hits"] == 0,
        "scenes": scenes,
    }


def run_detect() -> int:
    if _md5(PLANT_XML) != steer_walk.PLANT_MD5:
        print("FAIL plant md5")
        return 1
    payload = detect_paths(RoomDetector())
    summary = PREVIEWS / "find_room_detect.json"
    fk._write_summary(summary, payload)
    fk._copy_artifacts((summary,))
    print(
        f"[room] detect cutoff {SCORE_MIN} plant_under {payload['plant_under_cutoff']} "
        f"md5 {payload['plant_md5']} gait {payload['gait']}"
    )
    for scene in payload["scenes"]:
        std = scene["bearing_std_rad"]
        step = scene["bearing_max_step_rad"]
        std_s = "n/a" if std is None else f"{std:.3f}"
        step_s = "n/a" if step is None else f"{step:.3f}"
        print(
            f"[room] {scene['scene']:9} hits {scene['hits']}/{scene['poses']} "
            f"straight {scene['straight_hits']} relook {scene['relooks']} "
            f"object {scene['winner_object']} score {scene['winner_score']:.3f} "
            f"fired {scene['fired_objects'] or '-'} "
            f"wrong_rooms {scene['wrong_rooms'] or '-'} "
            f"committed_false {[row['object'] for row in scene['false_objects']] or '-'} "
            f"raw_false {[row['object'] for row in scene['raw_false_objects']] or '-'} "
            f"bearing_std {std_s} max_step {step_s} max_score {scene['max_score']:.3f}"
        )
    if not payload["plant_under_cutoff"]:
        print("FAIL empty plant crossed the cutoff")
        return 1
    return 0


def prefer_fail() -> int:
    if _md5(PLANT_XML) != steer_walk.PLANT_MD5:
        print("FAIL plant md5")
        return 1
    detector = RoomDetector()
    walks = [walk_room(detector, room) for room in ("kitchen", "bathroom")]
    payload: dict[str, object] = {
        "model": MODEL_ID,
        "model_rev": MODEL_REV,
        "score_min": SCORE_MIN,
        "objects": [{"name": name, "room": room} for name, room in OBJECTS],
        "hold_slices": HOLD_SLICES,
        "arrival_frac": ARRIVAL_FRAC,
        "arrival_gap_m": ARRIVAL_GAP_M,
        "progress_m": PROGRESS_M,
        "approach_vx": fk.SOFT_VX,
        "plant_md5": _md5(PLANT_XML),
        "stand_top": STAND_TOP,
        "walks": [_walk_dict(walk) for walk in walks],
    }
    summary = PREVIEWS / "find_room_summary.json"
    fk._write_summary(summary, payload)
    stills: list[Path] = []
    for walk in walks:
        before = PREVIEWS / f"find_room_{walk.room}_before.png"
        after = PREVIEWS / f"find_room_{walk.room}_after.png"
        fk._save_png(walk.before, before)
        fk._save_png(walk.after, after)
        stills.extend((before, after))
        remaining = None if walk.remaining_m is None else f"{walk.remaining_m:.3f}"
        print(
            f"[room] {walk.room} stop {walk.stop_kind} arrival {walk.arrival} "
            f"stand {walk.stand.room} {walk.stand.frac:.3f} "
            f"settled {walk.final.room} {walk.final.frac:.3f} "
            f"remaining {remaining} min_up_z {walk.min_up_z:.3f} "
            f"dx {walk.dx_m:+.3f} commands {len(walk.sent)} {walk.note}"
        )
    fk._copy_artifacts(tuple(stills) + (summary,))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("phrase", nargs="?", default="")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--prefer-fail", action="store_true")
    parser.add_argument("--detect", action="store_true")
    parser.add_argument("--scene", default="")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.detect:
        return run_detect()
    if args.prefer_fail:
        return prefer_fail()
    room = room_of_phrase(args.phrase)
    if room is None:
        print("refused: not a room phrase")
        return 1
    if args.scene == "plant":
        print("Prefer FAIL: empty floor plant; no motion")
        return 0
    if args.scene and args.scene != room:
        print("refused: phrase does not match the loaded scene; no motion")
        return 0
    if _md5(PLANT_XML) != steer_walk.PLANT_MD5:
        print("FAIL plant md5")
        return 1
    walk = walk_room(RoomDetector(), room)
    print(
        f"[room] {walk.room} stop {walk.stop_kind} arrival {walk.arrival} "
        f"remaining {walk.remaining_m} cue {walk.final.frac:.3f} {walk.note}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
