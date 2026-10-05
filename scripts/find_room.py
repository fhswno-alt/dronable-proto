#!/usr/bin/env python3
"""Open-vocab room steer from kit_cam. Not a per-room colour bar.

google/owlvit-base-patch32 runs locally. The same five room phrases and the
library score cutoff (0.1) are used for every scene. A phrase walks only when
that room is the unique top label. The kitchen slab path in find_kitchen.py
is not this script.

Arrival still needs both bars on the stop frame: cue fraction >= 0.50 and
torso-to-room <= 0.25 m. A box that already covers half the stand frame is
not an approach cue, so that stop is not arrival. vx stays at half cap.
vx=0 yaw is not sent.
"""

from __future__ import annotations

import argparse
import hashlib
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
# OwlViTImageProcessor.post_process_object_detection default. Not fit per room.
SCORE_MIN = 0.1
LABELS = (
    "a kitchen",
    "a bathroom",
    "a living room",
    "a bedroom",
    "an entrance",
)
ROOMS = ("kitchen", "bathroom", "living", "bedroom", "entrance")
ROOM_LABEL = {
    "kitchen": "a kitchen",
    "bathroom": "a bathroom",
    "living": "a living room",
    "bedroom": "a bedroom",
    "entrance": "an entrance",
}
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

# Stand frames of this revision. "living" wins several rooms. Bathroom is
# the only phrase that selects its own scene. Kitchen does not.
STAND_TOP = {
    "kitchen": "living",
    "bathroom": "bathroom",
    "living": "living",
    "bedroom": "living",
    "entrance": "living",
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


class RoomDetector:
    """One OWL-ViT. Five labels, one cutoff, local weights."""

    def __init__(self) -> None:
        import torch
        from transformers import OwlViTForObjectDetection, OwlViTProcessor

        torch.set_num_threads(4)
        self.torch = torch
        self.processor = OwlViTProcessor.from_pretrained(MODEL_ID, revision=MODEL_REV)
        self.model = OwlViTForObjectDetection.from_pretrained(MODEL_ID, revision=MODEL_REV)
        self.model.eval()

    def read(self, frame: np.ndarray) -> Detection:
        image_h, image_w = frame.shape[:2]
        inputs = self.processor(text=[list(LABELS)], images=frame, return_tensors="pt")
        with self.torch.no_grad():
            outputs = self.model(**inputs)
        probs = self.torch.sigmoid(outputs.logits[0])
        best = probs.max(dim=0).values
        scores = {room: float(best[LABELS.index(label)]) for room, label in ROOM_LABEL.items()}
        top_room = max(scores, key=scores.__getitem__)
        top_score = scores[top_room]
        if top_score < SCORE_MIN:
            return Detection(None, top_score, 0.0, 0.0, scores, None)
        results = self.processor.image_processor.post_process_object_detection(
            outputs,
            threshold=SCORE_MIN,
            target_sizes=[(image_h, image_w)],
        )[0]
        wanted = LABELS.index(ROOM_LABEL[top_room])
        chosen_score = -1.0
        chosen_box: tuple[float, float, float, float] | None = None
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            if int(label) != wanted:
                continue
            if float(score) <= chosen_score:
                continue
            chosen_score = float(score)
            chosen_box = tuple(float(value) for value in box)
        if chosen_box is None:
            return Detection(None, top_score, 0.0, 0.0, scores, None)
        x0, y0, x1, y1 = chosen_box
        x0 = min(max(x0, 0.0), float(image_w))
        x1 = min(max(x1, 0.0), float(image_w))
        y0 = min(max(y0, 0.0), float(image_h))
        y1 = min(max(y1, 0.0), float(image_h))
        area = max(0.0, x1 - x0) * max(0.0, y1 - y0)
        frac = area / float(image_w * image_h)
        bias = (((x0 + x1) * 0.5) - (image_w * 0.5)) / (image_w * 0.5)
        return Detection(top_room, chosen_score, frac, bias, scores, (x0, y0, x1, y1))


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
        if stand.room == room:
            while True:
                gap = body_gap(session.model, session.data, room)
                up_z = session.samples[-1].up_z if session.samples else 1.0
                if up_z < UP_Z_ABORT:
                    stop_kind = "tip"
                    note = f"Prefer FAIL: up_z {up_z:.3f} is below {UP_Z_ABORT:.2f}; stop"
                    break
                if arrival_ok(stand.frac, final.frac, gap):
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
                vx, yaw_rate = command_from_bias(final.bias)
                abort = _hold_slice(session, vx, yaw_rate, sent, x0, y0)
                frame = cam.grab(session.data)
                seen = detector.read(frame)
                if seen.room == room:
                    final = seen
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
                if seen.room != room:
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
    return failures


def test_stand(detector: RoomDetector) -> list[str]:
    failures: list[str] = []
    for room, xml in SCENE_XML.items():
        session = steer_walk.SteerSession(video=False, scene_xml=xml)
        cam = fk.KitCam(session.model)
        try:
            fk._hold_stand(session, STAND_S)
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
        "labels": list(LABELS),
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
    parser.add_argument("--scene", default="")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
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
