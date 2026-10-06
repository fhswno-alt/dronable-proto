#!/usr/bin/env python3
"""Five-room voice go-to on the joined apartment. Soft-pass is off.

Reach is fixed below before any room is scored. A bout reaches only when
the settled stand COM starts outside that room's floor box in
mujoco/room_apartment.json and finishes inside the same box, upright,
with zero prop contacts, at or before the time limit. The box is the
named room, not the plant floor from -3 m to +3 m. The doorway spawn
in that file is the start.

The phrase is shown to Moondream on one kit_cam still. A label that
matches the asked room publishes vel(+0.056, +0.000) on CommandBus.
Yaw stays inside ±0.25. The doorway already faces the opening, so the
go-to yaw is 0. The #71 latch is the corridor-gated finder stop. A
stop on the speckled hall shadow by the kitchen or bathroom doorway,
with no prop in the frame, is a false stop and is not a #71 pass.

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


class RoomJson(TypedDict):
    room: str
    scene: str
    phrase: str
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
    spawn: SpawnJson
    floor_box: FloorBox
    started_outside: bool
    finished_inside: bool
    reached: bool
    reach_block: str
    contact: str
    peak_nm: float
    peak_actuator: str
    min_up_z: float
    t_end: float


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


def _run_room(
    room_name: str,
    phrase: str,
    spec_room: apt.RoomSpec,
    asker: room_ask.RoomAsk | None,
    ask_error: str,
    targets_unused: dict[str, float],
    robot_bodies: set[str],
    d_min_m: float,
) -> RoomJson:
    del targets_unused
    door = spec_room["doorways"][0]
    spawn_xy = door["spawn"]
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
        initial_yaw=float(spawn_xy["yaw"]),
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
    real = mj.mj_step

    def hook(step_model: mj.MjModel, step_data: mj.MjData) -> None:
        real(step_model, step_data)
        _watch_step(step_model, step_data, bout, robot_bodies, act_idx)

    mj.mj_step = hook
    driver = sw.ScriptedDriver((
        sw.DemoSegment(STAND_S, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(TIME_LIMIT_S, "vel", voice.FWD_MPS, 0.0, "forward"),
    ))
    reach: list[tuple[float, float, float]] = []
    tracks: list[gate._GapTrack] = []
    phase_floor = hf.SamePhaseFloor()
    sightings: list[ShadowSighting] = []
    seen: set[str] = set()
    gate_m = d_min_m
    generic = voice.parse_phrase(phrase)
    words = sorted(set(voice.normalize_phrase(phrase).split()) & voice._ROOM_WORDS)
    stop = _empty_stop()
    label = "not asked"
    raw_answer = ""
    confidence: float | None = None
    seconds: float | None = None
    commanded_vx: float | None = None
    commanded_yaw: float | None = None
    latch_applied = False
    block = ""
    try:
        while float(session.data.time) < STAND_S - 1e-9:
            driver.publish(session.bus, float(session.data.time))
            session.step()
        com = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64)
        spawn = SpawnJson(
            doorway_x=float(spawn_xy["x"]),
            doorway_y=float(spawn_xy["y"]),
            doorway_yaw=float(spawn_xy["yaw"]),
            qpos_xyz=[
                float(session.data.qpos[0]),
                float(session.data.qpos[1]),
                float(session.data.qpos[2]),
            ],
            yaw_rad=_yaw(session.data, session.bid_body),
            com_xyz=[float(com[0]), float(com[1]), float(com[2])],
            up_z=float(session.data.xmat[session.bid_body].reshape(3, 3)[2, 2]),
        )
        started_outside = not _inside(float(com[0]), float(com[1]), box)
        rgb = _render_rgb(renderer, session.data)
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
        if asker is None:
            label = "not loaded"
            raw_answer = ask_error
            block = "recogniser did not load, so no vel was published"
        else:
            asked = asker.ask(rgb)
            raw_room = asked.get("room")
            label = "none" if raw_room is None else str(raw_room)
            raw_answer = str(asked.get("raw", ""))
            conf = asked.get("confidence")
            confidence = None if conf is None else float(conf)
            sec = asked.get("seconds")
            seconds = None if sec is None else float(sec)
        if label != room_name:
            if block == "":
                block = (
                    f"recogniser label {label!r} is not {room_name}; "
                    "no vel was published"
                )
        elif not started_outside:
            block = "settled COM is inside the named room box"
        else:
            commanded_vx = voice.FWD_MPS
            commanded_yaw = 0.0
            latch_applied = True
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
                    session.step()
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
                    session.step()
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
                driver.publish(session.bus, now)
                session.step()
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
        reached = started_outside and finished_inside and upright and clean and in_time
        if reached:
            block = ""
        elif block == "" and not finished_inside:
            block = f"COM did not finish inside the named box ({stop['path']})"
        elif block == "" and not upright:
            block = f"min up_z {min_up:.3f} is under {UPRIGHT_UP_Z:.2f}"
        elif block == "" and not clean:
            block = f"prop contact {bout.contact}"
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
            spawn=spawn,
            floor_box=box,
            started_outside=started_outside,
            finished_inside=finished_inside,
            reached=reached,
            reach_block=block,
            contact=bout.contact,
            peak_nm=bout.peak_nm,
            peak_actuator=bout.peak_actuator,
            min_up_z=min_up,
            t_end=t_end,
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
    if abs(rc.V_MPS - 0.150) > 1e-12:
        raise SystemExit(f"FAIL: d_min speed bound moved to {rc.V_MPS}")
    if not SCENE_XML.is_file():
        raise SystemExit(f"FAIL: missing {SCENE_XML}")
    spec = apt._load_spec(apt.SPEC_JSON)
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
        "recogniser": {
            "model": room_ask.MODEL_ID,
            "revision": room_ask.MODEL_REV,
            "prompt": room_ask.PROMPT,
        },
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
    try:
        for name, phrase in PHRASES:
            if want != "" and want != name:
                continue
            row = _run_room(
                name, phrase, spec["rooms"][name], asker, ask_error,
                {}, robot_bodies, float(definition["d_min_m"]),
            )
            rows.append(row)
            spawn = row["spawn"]
            print(
                f"{row['room']}  label={row['recogniser_label']!r}  "
                f"spawn_com={spawn['com_xyz'][0]:+.3f},{spawn['com_xyz'][1]:+.3f}  "
                f"started_outside={row['started_outside']}  "
                f"vx={row['commanded_vx']} yaw={row['commanded_yaw']}  "
                f"d_min={row['d_min_m']:.4f}  stop={row['stop_path']}  "
                f"false_stop={row['false_stop']}  "
                f"prop_in_frame={row['prop_in_frame_at_stop']}  "
                f"latch_pass={row['latch_pass']}  "
                f"reached={row['reached']}  contact={row['contact']}  "
                f"peak={row['peak_nm']:+.3f} {row['peak_actuator']}  "
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
    reached_all = len(rows) == len(PHRASES) and all(row["reached"] for row in rows)
    false_any = any(row["false_stop"] for row in rows)
    header["rooms"] = rows
    header["reached_all"] = reached_all
    header["false_stop_any"] = false_any
    header["prefer_fail"] = not reached_all
    header["go_anywhere"] = reached_all and not false_any
    header["kit_safe"] = False
    header["soft_pass"] = False
    _write(header)
    if reached_all and not false_any:
        print("Prefer PASS  soft-pass=off  reached_all=true", flush=True)
        return 0
    print(
        f"Prefer FAIL  soft-pass=off  reached_all={str(reached_all).lower()}  "
        f"false_stop_any={str(false_any).lower()}",
        flush=True,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
