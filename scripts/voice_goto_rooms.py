#!/usr/bin/env python3
"""Five-room yes/no search on the joined apartment. Soft-pass is off.

Reach is fixed below before any room is scored. A bout reaches only when
the settled stand COM starts outside that room's floor box in
mujoco/room_apartment.json and finishes inside the same box, upright,
with zero prop contacts, at or before the time limit, and only after a
correct per-target yes committed vel(+0.056, 0). The box is the named
room, not the plant floor from -3 m to +3 m.

Scored spawns are the doorway xy turned ±90° from the door-facing yaw.
A geometric census logs the kit_cam doorway fraction. ±90° is the offset
inside the ±60–90° band where that fraction is ~0. kit_cam is not moved.

The question is "Is there a {room} through the doorway ahead?" yes or no.
It is not an open-set room label. Search turns in place at +0.25 rad/s
with no forward velocity. The first yes ends the search. A yes while the
asked room is under 1% of the frame is a wrong yes and does not go-to.
A correct yes publishes vel(+0.056, +0.000) along the current heading
and arms the #71 latch. A stop on the speckled hall shadow by the
kitchen or bathroom doorway, with no prop in the frame, is a false stop
and is not a #71 pass.

0.150 is only the d_min bound. The plant file is only hashed.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
from pathlib import Path
from typing import TypedDict

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import check_apartment_scene as apt
import hazard_finder as hf
import lipm_gait
import mono_toe_gate as gate
import ray_corridor as rc
import room_ask
import steer_walk as sw
import voice_caller as voice

ROOT = Path(__file__).resolve().parents[1]
SUMMARY_PATH = ROOT / "previews" / "voice_goto_rooms_summary.json"
SCENE_XML = ROOT / "mujoco" / "room_apartment.xml"
TIME_LIMIT_S = 53.0
STAND_S = 1.0
UPRIGHT_UP_Z = 0.90
PROP_FORCE_N = 0.5
T_DETECT_S = 0.0
# Hall floor in front of the east openings. The light shines toward -X,
# so the wall shadow lands on this checker, outside the room box.
SHADOW_X = (0.45, 1.06)
SPECKLE_STD_MIN = 8.0
SPECKLE_MEAN_MAX = 90.0
SPECKLE_P10_MAX = 55.0
SPECKLE_SPAN_MIN = 18.0
PROP_HIT_M = 0.25
PHRASES: tuple[tuple[str, str], ...] = (
    ("kitchen", "go to the kitchen"),
    ("bathroom", "go to the bathroom"),
    ("living", "go to the living room"),
    ("bedroom", "go to the bedroom"),
    ("entrance", "go to the entrance"),
)
# ±90° is inside the requested ±60–90° band. The census must show the
# doorway fraction near 0 there. ±60° and ±75° are logged and not scored,
# because a 120° kit_cam still sees the opening at those headings.
SPAWN_OFFSETS_RAD: tuple[float, ...] = (0.5 * math.pi, -0.5 * math.pi)
CENSUS_OFFSETS_DEG: tuple[int, ...] = (0, 60, -60, 75, -75, 90, -90)
SEARCH_YAW = voice.YAW_RAD_S
SEARCH_SWEEP_RAD = 2.0 * math.pi
ASK_STEP_RAD = math.radians(20.0)
ROOM_VISIBLE_FRAC = 0.01
HIP_BAR_NM = 2.33
DOOR_Z = (0.05, 1.35)
RAY_STEP = 8
ROOM_GEOMS: dict[str, tuple[str, ...]] = {
    "kitchen": ("kitchen_", "table_", "chair_"),
    "bathroom": ("bathroom_", "sink_", "toilet_"),
    "living": ("living_", "tv_"),
    "bedroom": ("bedroom_", "nightstand_"),
    "entrance": ("entrance_", "mat_"),
}
HIP_ACTUATORS: tuple[tuple[str, str], ...] = (
    ("l_hip_yaw", "l_hip_yaw_pos"),
    ("r_hip_yaw", "r_hip_yaw_pos"),
    ("l_hip_roll", "l_hip_roll_pos"),
    ("r_hip_roll", "r_hip_roll_pos"),
)


class FloorBox(TypedDict):
    source: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


class SpawnJson(TypedDict):
    doorway_x: float
    doorway_y: float
    doorway_yaw: float
    qpos_xyz: list[float]
    yaw_rad: float
    com_xyz: list[float]
    up_z: float


class ShadowSighting(TypedDict):
    patch: str
    t: float
    u: float
    v: float
    lum_mean: float
    lum_std: float
    speckled: bool
    finder_emitted: bool
    prop_in_frame: bool
    hit_xy: list[float] | None


class AskJson(TypedDict):
    t: float
    tick: int
    heading_rad: float
    yaw_from_spawn_rad: float
    doorway_fraction: float
    asked_fraction: float
    visible_rooms: list[str]
    answer: str
    raw: str
    confidence: float | None
    seconds: float | None


class HipJson(TypedDict):
    joint: str
    actuator: str
    peak_nm: float
    abs_nm: float
    over_2_33: bool


class RoomJson(TypedDict):
    room: str
    scene: str
    phrase: str
    question: str
    yaw_offset_rad: float
    voice_parse: str
    voice_line: str
    generic_caller: str
    matched_room_words: list[str]
    recogniser_label: str
    recogniser_raw: str
    recogniser_confidence: float | None
    recogniser_seconds: float | None
    commanded_vx: float | None
    commanded_yaw: float | None
    search_yaw_rate: float
    yaw_turned_rad: float
    first_yes_t: float | None
    first_yes_tick: int | None
    first_yes_heading_rad: float | None
    wrong_yes: bool
    committed: bool
    d_min_m: float
    d_min_v_mps: float
    t_detect_s: float
    t_stop_s: float
    latch_applied: bool
    latch_pass: bool
    false_stop: bool
    prop_in_frame_at_stop: bool | None
    stop_path: str
    stop_reason: str
    stop_hit_xy: list[float] | None
    stop_on_shadow: bool
    shadow_first_sightings: list[ShadowSighting]
    doorway_fraction: float
    asked_fraction: float
    visible_rooms: list[str]
    asks: list[AskJson]
    spawn: SpawnJson
    floor_box: FloorBox
    started_outside: bool
    finished_inside: bool
    reached: bool
    reach_block: str
    contact: str
    peak_nm: float
    peak_actuator: str
    peak_joint: str
    hip_while_yawing: list[HipJson]
    min_up_z: float
    t_end: float
    end_qpos_xyz: list[float]
    end_yaw_rad: float


class DefinitionJson(TypedDict):
    reached: str
    floor_box: str
    upright: str
    prop_contact: str
    time_limit_s: float
    voice_vx_cap: float
    voice_yaw_cap: float
    d_min: str
    d_min_m: float
    false_stop: str
    yes_no: str
    search: str
    hip_bar_nm: float
    soft_pass: bool


class _Patch(TypedDict):
    room: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


class _Stop(TypedDict):
    path: str
    reason: str
    hit_xy: tuple[float, float] | None
    on_shadow: bool
    prop_in_frame: bool
    false_stop: bool
    latch_pass: bool


PATCHES: tuple[_Patch, ...] = (
    _Patch(room="kitchen", xmin=SHADOW_X[0], xmax=SHADOW_X[1], ymin=-0.56, ymax=0.34),
    _Patch(room="bathroom", xmin=SHADOW_X[0], xmax=SHADOW_X[1], ymin=1.43, ymax=2.33),
)


def _plant_md5() -> str:
    return hashlib.md5(sw.PLANT_XML.read_bytes()).hexdigest()


def _definition() -> DefinitionJson:
    d_min_m = float(rc.d_min(T_DETECT_S))
    return DefinitionJson(
        reached=(
            "Body COM xy of body_link finishes inside that room's floor "
            "box from room_apartment.json, the bout started with that COM "
            f"outside the same box, min up_z is at least {UPRIGHT_UP_Z:.2f}, "
            "prop contacts are zero, and the bout ends at or before "
            f"{TIME_LIMIT_S:.1f} s. A start inside the box is not a reach. "
            "The plant floor from -3 m to +3 m is not the box."
        ),
        floor_box=(
            "The named room floor_box in mujoco/room_apartment.json. "
            "It is not a geom and it is not passed to a command."
        ),
        upright=f"min up_z over the bout >= {UPRIGHT_UP_Z:.2f}",
        prop_contact=(
            f"A contact with normal force >= {PROP_FORCE_N:.1f} N between "
            "a robot geom and any geom other than floor."
        ),
        time_limit_s=TIME_LIMIT_S,
        voice_vx_cap=voice.FWD_MPS,
        voice_yaw_cap=voice.YAW_RAD_S,
        d_min=(
            "d_min = 0.150 * (T_detect + T_stop). 0.150 is the hardware "
            "bound in that formula, not the voice vx cap. "
            f"T_detect = {T_DETECT_S:.3f} s. T_stop = {rc.T_STOP_S:.3f} s."
        ),
        d_min_m=d_min_m,
        false_stop=(
            "A #71 stop whose floor point lies on the speckled shadow "
            "patch in front of the kitchen or bathroom doorway, with no "
            "furniture prop in the kit_cam frame. That stop is not a "
            "#71 pass. Walls are not props."
        ),
        yes_no=(
            "Per asked room: 'Is there a {room} through the doorway "
            "ahead?' yes or no. Not an open-set room name. A yes is "
            f"correct only when that room's furniture covers at least "
            f"{ROOM_VISIBLE_FRAC:.0%} of kit_cam. Any other yes is a "
            "wrong yes and does not publish forward vel."
        ),
        search=(
            "Spawn yaw is the door-facing yaw plus or minus 90 degrees. "
            "After the stand, vel(0, +0.25) turns in place. No forward "
            "vel until a correct yes. The question is asked at the spawn "
            "heading and every 20 degrees of body yaw, through one full "
            "turn. The turn sign is not taken from the door bearing. "
            "kit_cam is not moved."
        ),
        hip_bar_nm=HIP_BAR_NM,
        soft_pass=False,
    )


def _inside(x: float, y: float, box: FloorBox) -> bool:
    return box["xmin"] <= x <= box["xmax"] and box["ymin"] <= y <= box["ymax"]


def _in_patch(x: float, y: float, patch: _Patch) -> bool:
    return patch["xmin"] <= x <= patch["xmax"] and patch["ymin"] <= y <= patch["ymax"]


def _patch_of(x: float, y: float) -> str:
    for patch in PATCHES:
        if _in_patch(x, y, patch):
            return patch["room"]
    return ""


def _yaw(data: mj.MjData, body_id: int) -> float:
    rot = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    return float(math.atan2(rot[1, 0], rot[0, 0]))


def _robot_bodies(plant: mj.MjModel) -> set[str]:
    names: set[str] = set()
    for body_id in range(plant.nbody):
        name = mj.mj_id2name(plant, mj.mjtObj.mjOBJ_BODY, body_id) or ""
        if name != "":
            names.add(name)
    return names


def _prop_geom_ids(model: mj.MjModel, robot_bodies: set[str]) -> dict[int, str]:
    found: dict[int, str] = {}
    for geom_id in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or ""
        body_id = int(model.geom_bodyid[geom_id])
        body = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or ""
        if body in robot_bodies or name == "floor" or name.startswith("wall_"):
            continue
        found[geom_id] = name
    return found


def _prop_contact(model: mj.MjModel, data: mj.MjData, robot_bodies: set[str]) -> str:
    best_name = ""
    best_force = 0.0
    for index in range(data.ncon):
        con = data.contact[index]
        names: list[str] = []
        bodies: list[str] = []
        for geom_id in (int(con.geom1), int(con.geom2)):
            names.append(mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or "")
            body_id = int(model.geom_bodyid[geom_id])
            bodies.append(mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or "")
        if "floor" in names:
            continue
        robot = [body in robot_bodies for body in bodies]
        if robot[0] == robot[1]:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, index, force)
        normal = float(force[0])
        if normal < PROP_FORCE_N:
            continue
        prop = names[0] if robot[1] else names[1]
        if normal > best_force:
            best_force = normal
            best_name = prop
    if best_name == "":
        return "none"
    return f"{best_name} {best_force:.1f} N"


def _prop_in_frame(
    seg: np.ndarray,
    prop_ids: dict[int, str],
) -> bool:
    if seg.ndim != 3 or seg.shape[2] < 2:
        raise RuntimeError(f"segmentation shape {seg.shape}")
    geom_type = int(mj.mjtObj.mjOBJ_GEOM)
    types = seg[:, :, 1]
    ids = seg[:, :, 0]
    visible = ids[types == geom_type]
    if visible.size == 0:
        # This MuJoCo build stores the geom id in channel 0 and mjOBJ_GEOM
        # in channel 1. An empty type channel is a renderer change.
        if int(np.max(types)) != geom_type and int(np.max(ids)) > 0:
            raise RuntimeError("segmentation channels are not geom id, geom type")
        return False
    for geom_id in np.unique(visible):
        if int(geom_id) in prop_ids:
            return True
    return False


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _joint_name(actuator: str) -> str:
    if actuator.endswith("_pos"):
        return actuator[: -len("_pos")]
    return actuator


def _geom_room(name: str) -> str:
    for room, prefixes in ROOM_GEOMS.items():
        if name.startswith(prefixes):
            return room
    return ""


def _room_fractions(model: mj.MjModel, seg: np.ndarray) -> dict[str, float]:
    fracs = {room: 0.0 for room in ROOM_GEOMS}
    if seg.ndim != 3 or seg.shape[2] < 2:
        raise RuntimeError(f"segmentation shape {seg.shape}")
    geom_type = int(mj.mjtObj.mjOBJ_GEOM)
    types = seg[:, :, 1]
    ids = seg[:, :, 0]
    total = float(ids.size)
    visible = types == geom_type
    for geom_id in np.unique(ids[visible]):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
        room = _geom_room(name)
        if room == "":
            continue
        fracs[room] += float(np.sum((ids == geom_id) & visible)) / total
    return fracs


def _visible_rooms(fracs: dict[str, float]) -> list[str]:
    return [room for room, frac in fracs.items() if frac >= ROOM_VISIBLE_FRAC]


def _doorway_fraction(
    model: mj.MjModel,
    data: mj.MjData,
    opening: apt.Box,
    step: int = RAY_STEP,
) -> float:
    """Fraction of kit_cam rays that pass through this doorway opening.

    The plane is the hall face of the opening. A ray counts when it meets
    that rectangle before the first geom. kit_cam is not moved.
    """
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    origin = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    rot = np.asarray(data.cam_xmat[cid], dtype=np.float64).reshape(3, 3)
    plane_x = opening["xmin"] if float(origin[0]) < opening["xmin"] else opening["xmax"]
    ymin = opening["ymin"]
    ymax = opening["ymax"]
    z0, z1 = DOOR_Z
    hit = 0
    total = 0
    geomid = np.zeros(1, dtype=np.int32)
    for v in range(0, rc.HEIGHT, step):
        for u in range(0, rc.WIDTH, step):
            total += 1
            direction = rot @ rc.pixel_direction(float(u) + 0.5, float(v) + 0.5)
            if abs(float(direction[0])) < 1e-8:
                continue
            travel = (plane_x - float(origin[0])) / float(direction[0])
            if travel <= 0.05:
                continue
            point = origin + travel * direction
            if not (ymin <= float(point[1]) <= ymax and z0 <= float(point[2]) <= z1):
                continue
            dist = mj.mj_ray(model, data, origin, direction, None, 1, -1, geomid)
            if dist < 0 or float(dist) + 1e-3 >= travel:
                hit += 1
    if total == 0:
        return 0.0
    return hit / total


def _nearest_prop(
    model: mj.MjModel,
    data: mj.MjData,
    prop_ids: dict[int, str],
    hit_xy: tuple[float, float],
) -> tuple[str, float]:
    best_name = ""
    best = 1e9
    for geom_id, name in prop_ids.items():
        center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        # Box geoms use half-size. Other types use the center only.
        if int(model.geom_type[geom_id]) == int(mj.mjtGeom.mjGEOM_BOX):
            dx = max(abs(hit_xy[0] - float(center[0])) - float(half[0]), 0.0)
            dy = max(abs(hit_xy[1] - float(center[1])) - float(half[1]), 0.0)
        else:
            dx = hit_xy[0] - float(center[0])
            dy = hit_xy[1] - float(center[1])
        dist = math.hypot(dx, dy)
        if dist < best:
            best = dist
            best_name = name
    return best_name, best


def _window(rgb: np.ndarray, u: float, v: float) -> np.ndarray | None:
    height = int(rgb.shape[0])
    width = int(rgb.shape[1])
    cu = int(round(u))
    cv = int(round(v))
    if cu < 12 or cv < 12 or cu >= width - 12 or cv >= height - 12:
        return None
    return rgb[cv - 12: cv + 13, cu - 12: cu + 13]


def _speckled(window: np.ndarray) -> tuple[bool, float, float]:
    lum = window.astype(np.int32).sum(axis=2) // 3
    mean = float(lum.mean())
    std = float(lum.std())
    p10 = float(np.percentile(lum, 10))
    p90 = float(np.percentile(lum, 90))
    ok = (
        std >= SPECKLE_STD_MIN
        and mean <= SPECKLE_MEAN_MAX
        and p10 <= SPECKLE_P10_MAX
        and (p90 - p10) >= SPECKLE_SPAN_MIN
    )
    return ok, mean, std


def _speckle_self_check() -> None:
    flat = np.full((25, 25, 3), 40, dtype=np.uint8)
    ok, _mean, _std = _speckled(flat)
    if ok:
        raise SystemExit("FAIL: a flat patch counted as speckled")
    checker = np.zeros((25, 25, 3), dtype=np.uint8)
    checker[::2, ::2] = 44
    checker[1::2, 1::2] = 44
    checker[::2, 1::2] = 78
    checker[1::2, ::2] = 78
    ok, mean, std = _speckled(checker)
    if not ok:
        raise SystemExit(f"FAIL: checker shadow was not speckled mean={mean:.1f} std={std:.1f}")
    bright = np.full((25, 25, 3), 210, dtype=np.uint8)
    bright[::2] = 180
    ok, _mean, _std = _speckled(bright)
    if ok:
        raise SystemExit("FAIL: a bright wall counted as the floor shadow")


def _pose_bits(
    session: sw.SteerSession,
    model: mj.MjModel,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
) -> tuple[rc.KitCamPose, rc.BodyFrame, float, float, float, float, float, float, np.ndarray]:
    data = session.data
    tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
    pan = 0.0 if pan_id < 0 else float(data.qpos[int(model.jnt_qposadr[pan_id])])
    body_rot = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
    yaw, pitch, roll = rc.imu_from_body(body_rot)
    cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    fwd = gate.body_forward_xy(data, session.bid_body)
    toes = gate.toe_samples(session, cid)
    now = float(data.time)
    reach.append((
        now,
        next(row.offset_m for row in toes if row.side == "L"),
        next(row.offset_m for row in toes if row.side == "R"),
    ))
    step_off, _side, _when = gate._high_water(reach)
    body = rc.BodyFrame(
        (float(data.qpos[0]), float(data.qpos[1])),
        (float(fwd[0]), float(fwd[1])),
    )
    pose = rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2])))
    return pose, body, yaw, pitch, roll, tilt, pan, step_off, cam


def _cue_hit(
    cue: hf.HazardCue,
    *,
    pose: rc.KitCamPose,
    body: rc.BodyFrame,
    roll: float,
    pitch: float,
    tilt: float,
    yaw_rate: float,
    step_off: float,
    pan: float,
) -> tuple[float, float] | None:
    if cue.too_close or cue.u is None or cue.v is None:
        u = float(cue.column) + 0.5
        v = float(rc.HEIGHT) - 0.5
    else:
        u = float(cue.u)
        v = float(cue.v)
    estimate = rc.estimate_hazard(
        u, v,
        cam=pose,
        body=body,
        imu_roll_rad=roll,
        imu_pitch_rad=pitch,
        head_tilt_rad=tilt,
        yaw_rate=yaw_rate,
        step_off_m=step_off,
        hazard_pad_m=rc.HAZARD_PAD_M,
        head_pan_rad=pan,
    )
    if estimate is None:
        return None
    return estimate.hit_xy_m


def _render_rgb(renderer: mj.Renderer, data: mj.MjData) -> np.ndarray:
    renderer.disable_segmentation_rendering()
    renderer.update_scene(data, camera="kit_cam")
    return np.asarray(renderer.render(), dtype=np.uint8).copy()


def _render_seg(renderer: mj.Renderer, data: mj.MjData) -> np.ndarray:
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="kit_cam")
    seg = np.asarray(renderer.render()).copy()
    renderer.disable_segmentation_rendering()
    return seg


def _shadow_candidate(
    seen: set[str],
    *,
    rgb: np.ndarray,
    cam: np.ndarray,
    rot: np.ndarray,
) -> tuple[str, float, float, float, float] | None:
    """First unseen doorway patch whose floor sample is speckled in frame."""
    for patch in PATCHES:
        if patch["room"] in seen:
            continue
        xs = (patch["xmin"], 0.5 * (patch["xmin"] + patch["xmax"]), patch["xmax"])
        ys = (patch["ymin"], 0.5 * (patch["ymin"] + patch["ymax"]), patch["ymax"])
        for x in xs:
            for y in ys:
                pix = rc.project_point(np.array([x, y, 0.0], dtype=np.float64), cam, rot)
                if pix is None:
                    continue
                window = _window(rgb, pix[0], pix[1])
                if window is None:
                    continue
                speckled, mean, std = _speckled(window)
                if speckled:
                    return patch["room"], float(pix[0]), float(pix[1]), mean, std
    return None


def _mark_finder(
    sightings: list[ShadowSighting],
    patch_name: str,
    hit: tuple[float, float],
) -> None:
    for row in sightings:
        if row["patch"] == patch_name and not row["finder_emitted"]:
            row["finder_emitted"] = True
            row["hit_xy"] = [hit[0], hit[1]]


def _latch_cues(
    cues: tuple[hf.HazardCue, ...],
    tracks: list[gate._GapTrack],
    *,
    pose: rc.KitCamPose,
    body: rc.BodyFrame,
    roll: float,
    pitch: float,
    tilt: float,
    yaw_rate: float,
    step_off: float,
    pan: float,
    now: float,
    gate_m: float,
) -> tuple[str, tuple[float, float] | None, hf.HazardCue | None]:
    """#71 shortest in-corridor gap, then an in-corridor too_close clip.

    Returns path, floor hit, and the clip. path is empty when nothing stops.
    """
    clipped: hf.HazardCue | None = None
    for cue in cues:
        if cue.too_close and clipped is None:
            clipped = cue
        if cue.u is None or cue.v is None or cue.too_close:
            continue
        estimate = rc.estimate_hazard(
            cue.u, cue.v,
            cam=pose, body=body,
            imu_roll_rad=roll, imu_pitch_rad=pitch,
            head_tilt_rad=tilt, yaw_rate=yaw_rate,
            step_off_m=step_off, hazard_pad_m=rc.HAZARD_PAD_M,
            head_pan_rad=pan,
        )
        if estimate is None or not estimate.in_corridor:
            continue
        hit = estimate.hit_xy_m
        offer = estimate.toe_gap_m
        joined: gate._GapTrack | None = None
        joined_d = gate.TRACK_JOIN_M
        for held in tracks:
            apart = math.hypot(hit[0] - held.hit_xy[0], hit[1] - held.hit_xy[1])
            if apart <= joined_d:
                joined_d = apart
                joined = held
        if joined is None:
            tracks.append(gate._GapTrack(hit, offer, estimate.sideways_m, now))
        elif offer < joined.gap_m:
            joined.hit_xy = hit
            joined.gap_m = offer
            joined.side_m = estimate.sideways_m
            joined.t_s = now
    chosen: gate._GapTrack | None = None
    for held in tracks:
        if chosen is None or held.gap_m < chosen.gap_m:
            chosen = held
    if chosen is not None and chosen.gap_m <= gate_m:
        return "shortest", chosen.hit_xy, clipped
    if clipped is not None:
        hit = _cue_hit(
            clipped, pose=pose, body=body, roll=roll, pitch=pitch,
            tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
        )
        return "too_close", hit, clipped
    return "", None, None


def _refresh_tracks(
    tracks: list[gate._GapTrack],
    *,
    cam_xy: tuple[float, float],
    body: rc.BodyFrame,
    yaw_rate: float,
    step_off: float,
) -> list[gate._GapTrack]:
    alive: list[gate._GapTrack] = []
    for held in tracks:
        gap_now, _forward, side_now, inside = gate._pose_gap(
            held.hit_xy, cam_xy, body, yaw_rate, step_off,
        )
        if not inside:
            continue
        held.gap_m = gap_now
        held.side_m = side_now
        alive.append(held)
    return alive


def _finder_patch(
    cues: tuple[hf.HazardCue, ...],
    *,
    pose: rc.KitCamPose,
    body: rc.BodyFrame,
    roll: float,
    pitch: float,
    tilt: float,
    yaw_rate: float,
    step_off: float,
    pan: float,
) -> tuple[str, tuple[float, float] | None]:
    for cue in cues:
        hit = _cue_hit(
            cue, pose=pose, body=body, roll=roll, pitch=pitch,
            tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
        )
        if hit is None:
            continue
        name = _patch_of(hit[0], hit[1])
        if name != "":
            return name, hit
    return "", None


class _Bout:
    def __init__(self) -> None:
        self.contact = "none"
        self.peak_nm = 0.0
        self.peak_actuator = ""
        self.yaw_on = False
        self.hip_peak: dict[str, float] = {}


def _watch_step(
    model: mj.MjModel,
    data: mj.MjData,
    bout: _Bout,
    robot_bodies: set[str],
    act_idx: dict[str, int],
) -> None:
    hit = _prop_contact(model, data, robot_bodies)
    if hit != "none" and bout.contact == "none":
        bout.contact = hit
    for name, index in act_idx.items():
        if "hip_" not in name and "knee" not in name and "ank_" not in name:
            continue
        force = float(data.actuator_force[index])
        if abs(force) > abs(bout.peak_nm):
            bout.peak_nm = force
            bout.peak_actuator = name
        if bout.yaw_on and name in bout.hip_peak and abs(force) > abs(bout.hip_peak[name]):
            bout.hip_peak[name] = force


def _hip_rows(bout: _Bout) -> list[HipJson]:
    rows: list[HipJson] = []
    for joint, actuator in HIP_ACTUATORS:
        peak = float(bout.hip_peak.get(actuator, 0.0))
        rows.append(HipJson(
            joint=joint,
            actuator=actuator,
            peak_nm=peak,
            abs_nm=abs(peak),
            over_2_33=abs(peak) > HIP_BAR_NM,
        ))
    return rows


def _score_stop(
    path: str,
    reason: str,
    hit: tuple[float, float] | None,
    *,
    model: mj.MjModel,
    data: mj.MjData,
    prop_ids: dict[int, str],
    prop_in_frame: bool,
) -> _Stop:
    on_shadow = False if hit is None else _patch_of(hit[0], hit[1]) != ""
    near = ""
    dist = 1e9
    if hit is not None:
        near, dist = _nearest_prop(model, data, prop_ids, hit)
    false_stop = path in ("shortest", "too_close") and on_shadow and not prop_in_frame
    latch_pass = (
        path in ("shortest", "too_close")
        and prop_in_frame
        and not on_shadow
        and dist <= PROP_HIT_M
    )
    if false_stop:
        reason = (
            f"{reason} false_stop shadow patch, no prop in frame"
        )
    elif path in ("shortest", "too_close"):
        reason = (
            f"{reason} prop_in_frame={prop_in_frame} nearest={near} "
            f"dist={dist:.3f} on_shadow={on_shadow}"
        )
    return _Stop(
        path=path,
        reason=reason,
        hit_xy=hit,
        on_shadow=on_shadow,
        prop_in_frame=prop_in_frame,
        false_stop=false_stop,
        latch_pass=latch_pass,
    )


def _empty_stop() -> _Stop:
    return _Stop(
        path="",
        reason="",
        hit_xy=None,
        on_shadow=False,
        prop_in_frame=False,
        false_stop=False,
        latch_pass=False,
    )


def _sense(
    session: sw.SteerSession,
    model: mj.MjModel,
    renderer: mj.Renderer,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
    tracks: list[gate._GapTrack],
    phase_floor: hf.SamePhaseFloor,
    gate_m: float,
    sightings: list[ShadowSighting],
    seen: set[str],
    prop_ids: dict[int, str],
) -> tuple[str, tuple[float, float] | None, np.ndarray]:
    pose, body, yaw, pitch, roll, tilt, pan, step_off, cam = _pose_bits(
        session, model, cid, jid, pan_id, reach,
    )
    yaw_rate = float(session.bus.applied_yaw_rate)
    tracks[:] = _refresh_tracks(
        tracks, cam_xy=(float(cam[0]), float(cam[1])),
        body=body, yaw_rate=yaw_rate, step_off=step_off,
    )
    data = session.data
    rgb = _render_rgb(renderer, data)
    raw = hf.find_hazard_cues(rgb)
    cues = hf.corridor_gate_cues(
        raw, cam=pose, body=body,
        imu_roll_rad=roll, imu_pitch_rad=pitch,
        head_tilt_rad=tilt, yaw_rate=yaw_rate,
        step_off_m=step_off, head_pan_rad=pan,
    )
    if session.lipm is not None and not phase_floor.has_prior(session.lipm.stance):
        cues = hf.confirm_leg_columns(cues, rgb)
    if session.lipm is not None:
        cues = phase_floor.apply(
            cues,
            phase=session.lipm.phase,
            stance=session.lipm.stance,
            body_xy=(float(data.qpos[0]), float(data.qpos[1])),
            cam=pose, body=body,
            imu_roll_rad=roll, imu_pitch_rad=pitch,
            head_tilt_rad=tilt, yaw_rate=yaw_rate,
            step_off_m=step_off, head_pan_rad=pan,
        )
    rot = rc.camera_rotation_from_imu(yaw, pitch, roll, tilt, pan)
    candidate = _shadow_candidate(seen, rgb=rgb, cam=cam, rot=rot)
    finder_name, finder_hit = _finder_patch(
        cues, pose=pose, body=body, roll=roll, pitch=pitch,
        tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
    )
    if candidate is not None:
        seg = _render_seg(renderer, data)
        name, u, v, mean, std = candidate
        hit_list = None
        if finder_name == name and finder_hit is not None:
            hit_list = [finder_hit[0], finder_hit[1]]
        sightings.append(ShadowSighting(
            patch=name,
            t=float(data.time),
            u=u,
            v=v,
            lum_mean=mean,
            lum_std=std,
            speckled=True,
            finder_emitted=finder_name == name,
            prop_in_frame=_prop_in_frame(seg, prop_ids),
            hit_xy=hit_list,
        ))
        seen.add(name)
    elif finder_name != "" and finder_hit is not None:
        _mark_finder(sightings, finder_name, finder_hit)
    path, hit, _clip = _latch_cues(
        cues, tracks, pose=pose, body=body, roll=roll, pitch=pitch,
        tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
        now=float(data.time), gate_m=gate_m,
    )
    return path, hit, rgb


def _note_shadow(
    session: sw.SteerSession,
    model: mj.MjModel,
    renderer: mj.Renderer,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
    phase_floor: hf.SamePhaseFloor,
    sightings: list[ShadowSighting],
    seen: set[str],
    prop_ids: dict[int, str],
    rgb: np.ndarray,
) -> None:
    pose, body, yaw, pitch, roll, tilt, pan, step_off, cam = _pose_bits(
        session, model, cid, jid, pan_id, reach,
    )
    yaw_rate = float(session.bus.applied_yaw_rate)
    raw_cues = hf.corridor_gate_cues(
        hf.find_hazard_cues(rgb),
        cam=pose, body=body,
        imu_roll_rad=roll, imu_pitch_rad=pitch,
        head_tilt_rad=tilt, yaw_rate=yaw_rate,
        step_off_m=step_off, head_pan_rad=pan,
    )
    if session.lipm is not None and not phase_floor.has_prior(session.lipm.stance):
        raw_cues = hf.confirm_leg_columns(raw_cues, rgb)
    finder_name, finder_hit = _finder_patch(
        raw_cues, pose=pose, body=body, roll=roll, pitch=pitch,
        tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
    )
    rot = rc.camera_rotation_from_imu(yaw, pitch, roll, tilt, pan)
    candidate = _shadow_candidate(seen, rgb=rgb, cam=cam, rot=rot)
    if candidate is not None:
        seg = _render_seg(renderer, session.data)
        name, u, v, mean, std = candidate
        hit_list = None
        if finder_name == name and finder_hit is not None:
            hit_list = [finder_hit[0], finder_hit[1]]
        sightings.append(ShadowSighting(
            patch=name,
            t=float(session.data.time),
            u=u,
            v=v,
            lum_mean=mean,
            lum_std=std,
            speckled=True,
            finder_emitted=finder_name == name,
            prop_in_frame=_prop_in_frame(seg, prop_ids),
            hit_xy=hit_list,
        ))
        seen.add(name)
    elif finder_name != "" and finder_hit is not None:
        _mark_finder(sightings, finder_name, finder_hit)


def _look(
    session: sw.SteerSession,
    model: mj.MjModel,
    renderer: mj.Renderer,
    opening: apt.Box,
    room_name: str,
    question: str,
    asker: room_ask.RoomAsk | None,
    ask_error: str,
    *,
    tick: int,
    yaw_spawn: float,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
    phase_floor: hf.SamePhaseFloor,
    sightings: list[ShadowSighting],
    seen: set[str],
    prop_ids: dict[int, str],
) -> tuple[AskJson, dict[str, float]]:
    rgb = _render_rgb(renderer, session.data)
    _note_shadow(
        session, model, renderer, cid, jid, pan_id, reach, phase_floor,
        sightings, seen, prop_ids, rgb,
    )
    seg = _render_seg(renderer, session.data)
    fracs = _room_fractions(model, seg)
    door = _doorway_fraction(model, session.data, opening)
    heading = _yaw(session.data, session.bid_body)
    if asker is None:
        answer = "not loaded"
        raw = ask_error
        confidence = None
        seconds = None
    else:
        asked = asker.ask_yes_no(rgb, question)
        parsed = asked.get("answer")
        answer = "undecided" if parsed is None else str(parsed)
        raw = str(asked.get("raw", ""))
        conf = asked.get("confidence")
        confidence = None if conf is None else float(conf)
        sec = asked.get("seconds")
        seconds = None if sec is None else float(sec)
    row = AskJson(
        t=float(session.data.time),
        tick=tick,
        heading_rad=heading,
        yaw_from_spawn_rad=_wrap(heading - yaw_spawn),
        doorway_fraction=door,
        asked_fraction=float(fracs.get(room_name, 0.0)),
        visible_rooms=_visible_rooms(fracs),
        answer=answer,
        raw=raw,
        confidence=confidence,
        seconds=seconds,
    )
    return row, fracs


def _place(session: sw.SteerSession, x: float, y: float, yaw: float) -> None:
    session.data.qpos[0] = float(x)
    session.data.qpos[1] = float(y)
    half = 0.5 * float(yaw)
    session.data.qpos[3] = math.cos(half)
    session.data.qpos[4] = 0.0
    session.data.qpos[5] = 0.0
    session.data.qpos[6] = math.sin(half)
    session.data.qvel[:] = 0.0
    mj.mj_forward(session.model, session.data)


def _fov_census(spec: dict[str, apt.RoomSpec]) -> list[dict[str, object]]:
    """Doorway fraction before any model call. kit_cam is not moved."""
    session = sw.SteerSession(
        video=False,
        scene_xml=SCENE_XML,
        initial_yaw=0.0,
        lipm=sw.locked_kit_config(),
    )
    model = session.model
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    rows: list[dict[str, object]] = []
    try:
        for name in ROOM_GEOMS:
            room = spec[name]
            door = room["doorways"][0]
            opening = door["opening"]
            spawn = door["spawn"]
            box = room["floor_box"]
            for deg in CENSUS_OFFSETS_DEG:
                yaw = float(spawn["yaw"]) + math.radians(deg)
                _place(session, float(spawn["x"]), float(spawn["y"]), yaw)
                seg = _render_seg(renderer, session.data)
                fracs = _room_fractions(model, seg)
                com = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64)
                outside = not (
                    box["xmin"] <= float(com[0]) <= box["xmax"]
                    and box["ymin"] <= float(com[1]) <= box["ymax"]
                )
                rows.append({
                    "room": name,
                    "offset_deg": deg,
                    "doorway_fraction": _doorway_fraction(model, session.data, opening),
                    "asked_fraction": float(fracs.get(name, 0.0)),
                    "visible_rooms": _visible_rooms(fracs),
                    "started_outside": outside,
                })
    finally:
        renderer.close()
    return rows


def _assert_census(rows: list[dict[str, object]]) -> None:
    by = {(str(row["room"]), int(row["offset_deg"])): row for row in rows}
    facing = by[("kitchen", 0)]
    left = by[("kitchen", 90)]
    right = by[("kitchen", -90)]
    if float(facing["doorway_fraction"]) < 0.20:
        raise SystemExit(
            f"FAIL: facing kitchen doorway fraction {facing['doorway_fraction']}"
        )
    for row in (left, right):
        if float(row["doorway_fraction"]) > 0.02:
            raise SystemExit(
                f"FAIL: kitchen offset {row['offset_deg']} still sees the doorway "
                f"({row['doorway_fraction']})"
            )
    if float(facing["asked_fraction"]) < ROOM_VISIBLE_FRAC:
        raise SystemExit("FAIL: facing kitchen furniture is under the visibility bar")
    if float(left["asked_fraction"]) >= ROOM_VISIBLE_FRAC:
        raise SystemExit("FAIL: kitchen at +90 deg is already in frame")
    for row in rows:
        if int(row["offset_deg"]) in (90, -90) and not bool(row["started_outside"]):
            raise SystemExit(f"FAIL: {row['room']} ±90 spawn is inside its box")
    if abs(HIP_BAR_NM - 2.33) > 1e-12 or abs(HIP_BAR_NM - lipm_gait.KNEE_SAG_NM) > 1e-12:
        raise SystemExit(f"FAIL: hip bar {HIP_BAR_NM} is not 2.33")


def _run_room(
    room_name: str,
    phrase: str,
    yaw_offset: float,
    spec_room: apt.RoomSpec,
    asker: room_ask.RoomAsk | None,
    ask_error: str,
    robot_bodies: set[str],
    d_min_m: float,
) -> RoomJson:
    door = spec_room["doorways"][0]
    spawn_xy = door["spawn"]
    opening = door["opening"]
    face_yaw = float(spawn_xy["yaw"])
    box = FloorBox(
        source="room_apartment.json",
        xmin=spec_room["floor_box"]["xmin"],
        xmax=spec_room["floor_box"]["xmax"],
        ymin=spec_room["floor_box"]["ymin"],
        ymax=spec_room["floor_box"]["ymax"],
    )
    session = sw.SteerSession(
        video=False,
        scene_xml=SCENE_XML,
        initial_yaw=face_yaw + yaw_offset,
        lipm=sw.locked_kit_config(),
    )
    session.data.qpos[0] = float(spawn_xy["x"])
    session.data.qpos[1] = float(spawn_xy["y"])
    mj.mj_forward(session.model, session.data)
    model = session.model
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    pan_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_pan")
    if cid < 0 or jid < 0:
        raise SystemExit(f"FAIL {room_name}: kit_cam or head_tilt missing")
    prop_ids = _prop_geom_ids(model, robot_bodies)
    act_idx = {
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or "": i
        for i in range(model.nu)
    }
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    bout = _Bout()
    for _joint, actuator in HIP_ACTUATORS:
        if actuator not in act_idx:
            raise SystemExit(f"FAIL: missing actuator {actuator}")
        bout.hip_peak[actuator] = 0.0
    question = room_ask.yes_no_prompt(room_name)
    real = mj.mj_step

    def hook(step_model: mj.MjModel, step_data: mj.MjData) -> None:
        real(step_model, step_data)
        _watch_step(step_model, step_data, bout, robot_bodies, act_idx)

    mj.mj_step = hook
    reach: list[tuple[float, float, float]] = []
    tracks: list[gate._GapTrack] = []
    phase_floor = hf.SamePhaseFloor()
    sightings: list[ShadowSighting] = []
    seen: set[str] = set()
    asks: list[AskJson] = []
    gate_m = d_min_m
    generic = voice.parse_phrase(phrase)
    words = sorted(set(voice.normalize_phrase(phrase).split()) & voice._ROOM_WORDS)
    stop = _empty_stop()
    commanded_vx: float | None = None
    commanded_yaw: float | None = None
    latch_applied = False
    committed = False
    wrong_yes = False
    block = ""
    ticks = 0
    yaw_turned = 0.0
    first_yes_t: float | None = None
    first_yes_tick: int | None = None
    first_yes_heading: float | None = None
    stand_door = 0.0
    stand_asked = 0.0
    stand_visible: list[str] = []
    try:
        while float(session.data.time) < STAND_S - 1e-9:
            session.bus.stand(float(session.data.time))
            bout.yaw_on = False
            session.step()
            ticks += 1
        com = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64)
        yaw_spawn = _yaw(session.data, session.bid_body)
        spawn = SpawnJson(
            doorway_x=float(spawn_xy["x"]),
            doorway_y=float(spawn_xy["y"]),
            doorway_yaw=face_yaw,
            qpos_xyz=[
                float(session.data.qpos[0]),
                float(session.data.qpos[1]),
                float(session.data.qpos[2]),
            ],
            yaw_rad=yaw_spawn,
            com_xyz=[float(com[0]), float(com[1]), float(com[2])],
            up_z=float(session.data.xmat[session.bid_body].reshape(3, 3)[2, 2]),
        )
        started_outside = not _inside(float(com[0]), float(com[1]), box)
        if not started_outside:
            block = "settled COM is inside the named room box"

        def consider(row: AskJson) -> str:
            nonlocal block, wrong_yes, committed, first_yes_t, first_yes_tick, first_yes_heading
            asks.append(row)
            if row["answer"] != "yes":
                return row["answer"]
            first_yes_t = row["t"]
            first_yes_tick = row["tick"]
            first_yes_heading = row["heading_rad"]
            if row["asked_fraction"] < ROOM_VISIBLE_FRAC:
                wrong_yes = True
                visible = ", ".join(row["visible_rooms"]) or "none"
                block = (
                    f"wrong yes; {room_name} covers {row['asked_fraction']:.3f} of "
                    f"kit_cam, under {ROOM_VISIBLE_FRAC:.2f}; visible rooms: {visible}"
                )
                return "wrong"
            committed = True
            return "commit"

        if asker is None:
            block = "recogniser did not load, so no vel was published"
            look, _fracs = _look(
                session, model, renderer, opening, room_name, question,
                None, ask_error, tick=ticks, yaw_spawn=yaw_spawn,
                cid=cid, jid=jid, pan_id=pan_id, reach=reach,
                phase_floor=phase_floor, sightings=sightings, seen=seen,
                prop_ids=prop_ids,
            )
            asks.append(look)
            stand_door = look["doorway_fraction"]
            stand_asked = look["asked_fraction"]
            stand_visible = look["visible_rooms"]
        elif block == "":
            look, _fracs = _look(
                session, model, renderer, opening, room_name, question,
                asker, ask_error, tick=ticks, yaw_spawn=yaw_spawn,
                cid=cid, jid=jid, pan_id=pan_id, reach=reach,
                phase_floor=phase_floor, sightings=sightings, seen=seen,
                prop_ids=prop_ids,
            )
            stand_door = look["doorway_fraction"]
            stand_asked = look["asked_fraction"]
            stand_visible = look["visible_rooms"]
            decision = consider(look)
            prev_yaw = yaw_spawn
            next_ask = ASK_STEP_RAD
            while (
                decision == "no" or decision == "undecided"
            ) and float(session.data.time) < TIME_LIMIT_S - 1e-9 and yaw_turned < SEARCH_SWEEP_RAD:
                now = float(session.data.time)
                session.bus.vel(0.0, SEARCH_YAW, now)
                bout.yaw_on = True
                session.step()
                ticks += 1
                heading = _yaw(session.data, session.bid_body)
                yaw_turned += _wrap(heading - prev_yaw)
                prev_yaw = heading
                if bout.contact != "none":
                    stop = _Stop(
                        path="contact",
                        reason=bout.contact,
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
                    block = f"prop contact during search {bout.contact}"
                    break
                com_now = np.asarray(
                    session.data.subtree_com[session.bid_body], dtype=np.float64,
                )
                if _inside(float(com_now[0]), float(com_now[1]), box):
                    block = (
                        "COM entered the named box during the in-place search, "
                        "before a go-to"
                    )
                    stop = _Stop(
                        path="search_inside",
                        reason=block,
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
                    break
                if yaw_turned < next_ask:
                    continue
                while yaw_turned >= next_ask:
                    next_ask += ASK_STEP_RAD
                look, _fracs = _look(
                    session, model, renderer, opening, room_name, question,
                    asker, ask_error, tick=ticks, yaw_spawn=yaw_spawn,
                    cid=cid, jid=jid, pan_id=pan_id, reach=reach,
                    phase_floor=phase_floor, sightings=sightings, seen=seen,
                    prop_ids=prop_ids,
                )
                decision = consider(look)
                if decision in ("wrong", "commit"):
                    break
            else:
                if not committed and not wrong_yes and block == "" and bout.contact == "none":
                    if float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                        block = "no yes before the time limit"
                        stop = _Stop(
                            path="time",
                            reason=block,
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
                    else:
                        block = (
                            f"turned {yaw_turned:.3f} rad at yaw {SEARCH_YAW:+.2f} "
                            "with no yes"
                        )
                        stop = _Stop(
                            path="no_yes",
                            reason=block,
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
            if wrong_yes:
                stop = _Stop(
                    path="wrong_yes",
                    reason=block,
                    hit_xy=None,
                    on_shadow=False,
                    prop_in_frame=False,
                    false_stop=False,
                    latch_pass=False,
                )
        if committed and stop["path"] == "":
            commanded_vx = voice.FWD_MPS
            commanded_yaw = 0.0
            latch_applied = True
            session.bus.vel(voice.FWD_MPS, 0.0, float(session.data.time))
            while float(session.data.time) < TIME_LIMIT_S - 1e-9:
                now = float(session.data.time)
                path, hit, _frame = _sense(
                    session, model, renderer, cid, jid, pan_id, reach, tracks,
                    phase_floor, gate_m, sightings, seen, prop_ids,
                )
                if path != "":
                    seg = _render_seg(renderer, session.data)
                    prop = _prop_in_frame(seg, prop_ids)
                    stop = _score_stop(
                        path, path, hit,
                        model=model, data=session.data, prop_ids=prop_ids,
                        prop_in_frame=prop,
                    )
                    session.bus.stop(now)
                    bout.yaw_on = abs(session.bus.applied_yaw_rate) > 1e-6
                    session.step()
                    ticks += 1
                    break
                if bout.contact != "none":
                    stop = _Stop(
                        path="contact",
                        reason=bout.contact,
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
                    break
                com_now = np.asarray(
                    session.data.subtree_com[session.bid_body], dtype=np.float64,
                )
                if _inside(float(com_now[0]), float(com_now[1]), box):
                    session.bus.stop(now)
                    bout.yaw_on = abs(session.bus.applied_yaw_rate) > 1e-6
                    session.step()
                    ticks += 1
                    stop = _Stop(
                        path="inside",
                        reason="COM finished inside the named room box",
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
                    break
                session.bus.vel(voice.FWD_MPS, 0.0, now)
                bout.yaw_on = (
                    abs(session.bus.target_yaw) > 1e-6
                    or abs(session.bus.applied_yaw_rate) > 1e-6
                )
                session.step()
                ticks += 1
                if bout.contact != "none":
                    stop = _Stop(
                        path="contact",
                        reason=bout.contact,
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
                    break
            else:
                stop = _Stop(
                    path="time",
                    reason=f"time limit {TIME_LIMIT_S:.1f} s",
                    hit_xy=None,
                    on_shadow=False,
                    prop_in_frame=False,
                    false_stop=False,
                    latch_pass=False,
                )
        if bout.peak_actuator == "":
            raise SystemExit(f"FAIL {room_name}: no leg actuator force")
        com_end = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64)
        finished_inside = _inside(float(com_end[0]), float(com_end[1]), box)
        min_up = float(session.min_up_z)
        t_end = float(session.data.time)
        upright = min_up >= UPRIGHT_UP_Z
        clean = bout.contact == "none"
        in_time = t_end <= TIME_LIMIT_S + 1e-6
        reached = (
            started_outside
            and finished_inside
            and upright
            and clean
            and in_time
            and committed
            and not wrong_yes
        )
        if reached:
            block = ""
        elif block == "" and not finished_inside:
            block = f"COM did not finish inside the named box ({stop['path']})"
        elif block == "" and not upright:
            block = f"min up_z {min_up:.3f} is under {UPRIGHT_UP_Z:.2f}"
        elif block == "" and not clean:
            block = f"prop contact {bout.contact}"
        label = "not asked"
        raw_answer = ""
        confidence: float | None = None
        seconds: float | None = None
        if asks:
            chosen = asks[-1]
            for row in asks:
                if row["answer"] == "yes":
                    chosen = row
                    break
            label = chosen["answer"]
            raw_answer = chosen["raw"]
            confidence = chosen["confidence"]
            seconds = chosen["seconds"]
        voice_line = generic.line
        voice_parse = generic.kind
        if commanded_vx is not None and commanded_yaw is not None:
            voice_line = voice.bus_text("vel", commanded_vx, commanded_yaw)
            voice_parse = "vel"
        hit_list = None if stop["hit_xy"] is None else [stop["hit_xy"][0], stop["hit_xy"][1]]
        prop_at_stop: bool | None = None
        if stop["path"] in ("shortest", "too_close"):
            prop_at_stop = stop["prop_in_frame"]
        return RoomJson(
            room=room_name,
            scene=SCENE_XML.name,
            phrase=phrase,
            question=question,
            yaw_offset_rad=yaw_offset,
            voice_parse=voice_parse,
            voice_line=voice_line,
            generic_caller=generic.line,
            matched_room_words=words,
            recogniser_label=label,
            recogniser_raw=raw_answer,
            recogniser_confidence=confidence,
            recogniser_seconds=seconds,
            commanded_vx=commanded_vx,
            commanded_yaw=commanded_yaw,
            search_yaw_rate=SEARCH_YAW,
            yaw_turned_rad=yaw_turned,
            first_yes_t=first_yes_t,
            first_yes_tick=first_yes_tick,
            first_yes_heading_rad=first_yes_heading,
            wrong_yes=wrong_yes,
            committed=committed,
            d_min_m=d_min_m,
            d_min_v_mps=float(rc.V_MPS),
            t_detect_s=T_DETECT_S,
            t_stop_s=float(rc.T_STOP_S),
            latch_applied=latch_applied,
            latch_pass=stop["latch_pass"],
            false_stop=stop["false_stop"],
            prop_in_frame_at_stop=prop_at_stop,
            stop_path=stop["path"],
            stop_reason=stop["reason"],
            stop_hit_xy=hit_list,
            stop_on_shadow=stop["on_shadow"],
            shadow_first_sightings=sightings,
            doorway_fraction=stand_door,
            asked_fraction=stand_asked,
            visible_rooms=stand_visible,
            asks=asks,
            spawn=spawn,
            floor_box=box,
            started_outside=started_outside,
            finished_inside=finished_inside,
            reached=reached,
            reach_block=block,
            contact=bout.contact,
            peak_nm=bout.peak_nm,
            peak_actuator=bout.peak_actuator,
            peak_joint=_joint_name(bout.peak_actuator),
            hip_while_yawing=_hip_rows(bout),
            min_up_z=min_up,
            t_end=t_end,
            end_qpos_xyz=[
                float(session.data.qpos[0]),
                float(session.data.qpos[1]),
                float(session.data.qpos[2]),
            ],
            end_yaw_rad=_yaw(session.data, session.bid_body),
        )
    finally:
        mj.mj_step = real
        renderer.close()


def _write(payload: dict[str, object]) -> None:
    SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
    SUMMARY_PATH.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _load_asker() -> tuple[room_ask.RoomAsk | None, str]:
    try:
        return room_ask.RoomAsk(), ""
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"


def main() -> int:
    _speckle_self_check()
    before = _plant_md5()
    if before != sw.PLANT_MD5 or before != voice.PLANT_MD5:
        raise SystemExit(f"FAIL: plant md5 {before}")
    if abs(voice.FWD_MPS - 0.056) > 1e-12:
        raise SystemExit(f"FAIL: voice forward cap moved to {voice.FWD_MPS}")
    if abs(voice.YAW_RAD_S - 0.25) > 1e-12:
        raise SystemExit(f"FAIL: voice yaw cap moved to {voice.YAW_RAD_S}")
    if abs(SEARCH_YAW - 0.25) > 1e-12:
        raise SystemExit(f"FAIL: search yaw moved to {SEARCH_YAW}")
    if abs(rc.V_MPS - 0.150) > 1e-12:
        raise SystemExit(f"FAIL: d_min speed bound moved to {rc.V_MPS}")
    if not SCENE_XML.is_file():
        raise SystemExit(f"FAIL: missing {SCENE_XML}")
    spec = apt._load_spec(apt.SPEC_JSON)
    census = _fov_census(spec["rooms"])
    _assert_census(census)
    for row in census:
        print(
            f"census {row['room']} off={row['offset_deg']:+d} "
            f"door={row['doorway_fraction']:.3f} "
            f"asked={row['asked_fraction']:.3f} "
            f"visible={row['visible_rooms']} outside={row['started_outside']}",
            flush=True,
        )
    definition = _definition()
    want = sys.argv[1] if len(sys.argv) > 1 else ""
    header: dict[str, object] = {
        "prefer_fail": True,
        "soft_pass": False,
        "go_anywhere": False,
        "kit_safe": False,
        "plant_md5": before,
        "definition": definition,
        "scene": SCENE_XML.name,
        "joined_scene": True,
        "doorway_spawn": True,
        "kit_cam_moved": False,
        "recogniser": {
            "model": room_ask.MODEL_ID,
            "revision": room_ask.MODEL_REV,
            "prompt": room_ask.YES_NO_PROMPT,
            "open_set_prompt_not_used": room_ask.PROMPT,
            "room_visible_frac": ROOM_VISIBLE_FRAC,
            "ask_step_rad": ASK_STEP_RAD,
            "search_yaw": SEARCH_YAW,
            "search_sweep_rad": SEARCH_SWEEP_RAD,
            "spawn_offsets_rad": list(SPAWN_OFFSETS_RAD),
        },
        "doorway_census": census,
        "shadow_patches": list(PATCHES),
        "rooms": [],
    }
    _write(header)
    sw.apply_frozen_forward_gait()
    plant = mj.MjModel.from_xml_path(str(sw.PLANT_XML))
    robot_bodies = _robot_bodies(plant)
    asker, ask_error = _load_asker()
    if ask_error:
        print(f"recogniser not loaded: {ask_error}", flush=True)
    rows: list[RoomJson] = []
    expected = len(PHRASES) * len(SPAWN_OFFSETS_RAD)
    try:
        for name, phrase in PHRASES:
            if want != "" and want != name:
                continue
            for offset in SPAWN_OFFSETS_RAD:
                row = _run_room(
                    name, phrase, offset, spec["rooms"][name], asker, ask_error,
                    robot_bodies, float(definition["d_min_m"]),
                )
                rows.append(row)
                spawn = row["spawn"]
                hip_over = [item["joint"] for item in row["hip_while_yawing"] if item["over_2_33"]]
                print(
                    f"{row['room']} off={row['yaw_offset_rad']:+.3f}  "
                    f"door={row['doorway_fraction']:.3f}  "
                    f"label={row['recogniser_label']!r}  "
                    f"wrong_yes={row['wrong_yes']}  "
                    f"yes_t={row['first_yes_t']} yes_tick={row['first_yes_tick']}  "
                    f"yes_heading={row['first_yes_heading_rad']}  "
                    f"yaw_turned={row['yaw_turned_rad']:+.3f}  "
                    f"spawn_com={spawn['com_xyz'][0]:+.3f},{spawn['com_xyz'][1]:+.3f}  "
                    f"vx={row['commanded_vx']} yaw={row['commanded_yaw']}  "
                    f"d_min={row['d_min_m']:.4f}  stop={row['stop_path']}  "
                    f"false_stop={row['false_stop']}  "
                    f"prop_in_frame={row['prop_in_frame_at_stop']}  "
                    f"latch_pass={row['latch_pass']}  "
                    f"reached={row['reached']}  contact={row['contact']}  "
                    f"peak={row['peak_nm']:+.3f} {row['peak_joint']}  "
                    f"hip_over={hip_over}  "
                    f"shadows={len(row['shadow_first_sightings'])}",
                    flush=True,
                )
                header["rooms"] = rows
                _write(header)
    finally:
        if asker is not None:
            asker.close()
    if _plant_md5() != before:
        raise SystemExit("FAIL: plant file changed during the measure")
    if definition != _definition():
        raise SystemExit("FAIL: reach definition changed after the measure")
    reached_all = len(rows) == expected and all(row["reached"] for row in rows)
    false_any = any(row["false_stop"] for row in rows)
    wrong_any = any(row["wrong_yes"] for row in rows)
    header["rooms"] = rows
    header["reached_all"] = reached_all
    header["false_stop_any"] = false_any
    header["wrong_yes_any"] = wrong_any
    header["prefer_fail"] = (not reached_all) or false_any or wrong_any
    header["go_anywhere"] = reached_all and not false_any and not wrong_any
    header["kit_safe"] = False
    header["soft_pass"] = False
    _write(header)
    if header["go_anywhere"]:
        print("Prefer PASS  soft-pass=off  reached_all=true", flush=True)
        return 0
    print(
        f"Prefer FAIL  soft-pass=off  reached_all={str(reached_all).lower()}  "
        f"false_stop_any={str(false_any).lower()}  "
        f"wrong_yes_any={str(wrong_any).lower()}",
        flush=True,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
