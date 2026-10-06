#!/usr/bin/env python3
"""Five-room voice go-to. Prefer FAIL: spawn is already on the room floor.

Reach is fixed below before any room is scored. A bout counts only when
the settled stand COM starts outside that room's floor box. These files
are one room each. Each includes the plant floor plane and nothing else
named floor. The documented stand is on that plane. There is no joined
scene and no doorway spawn in this tree. That bout is not a reach.

Voice forward stays +0.056 m/s and yaw ±0.25. 0.150 is only the d_min
bound. No vel is published for an ineligible bout. The #71 latch is not
applied to a start-inside walk. Plant file is only hashed. Soft-pass is
off. Exit status is 1.
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

import ray_corridor as rc
import render_kit_cam_room as rooms
import steer_walk as sw
import voice_caller as voice

ROOT = Path(__file__).resolve().parents[1]
SUMMARY_PATH = ROOT / "previews" / "voice_goto_rooms_summary.json"
# Frozen before the score. DEMO_SCRIPT holds forward until this time.
TIME_LIMIT_S = 53.0
UPRIGHT_UP_Z = 0.90
PROP_FORCE_N = 0.5
T_DETECT_S = 0.0
FLOOR_GEOM_NAMES = frozenset({"floor", "room_floor"})
PHRASES: tuple[tuple[str, str], ...] = (
    ("kitchen", "go to the kitchen"),
    ("bathroom", "go to the bathroom"),
    ("living", "go to the living room"),
    ("bedroom", "go to the bedroom"),
    ("entrance", "go to the entrance"),
)
GAP = (
    "No joined multi-room scene and no doorway spawn. Each room file is "
    "its own scene, and the documented stand starts on that scene's only "
    "floor plane, so the COM starts inside the room floor box. A "
    "start-inside pose is not a reach. No new scene was authored."
)


class FloorBox(TypedDict):
    geom: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


class SpawnJson(TypedDict):
    qpos_xyz: list[float]
    yaw_rad: float
    com_xyz: list[float]
    up_z: float


class RoomJson(TypedDict):
    room: str
    scene: str
    phrase: str
    voice_parse: str
    voice_line: str
    matched_room_words: list[str]
    recogniser_label: str
    commanded_vx: float | None
    commanded_yaw: float | None
    d_min_m: float
    d_min_v_mps: float
    t_detect_s: float
    t_stop_s: float
    latch_applied: bool
    spawn: SpawnJson
    floor_boxes: list[FloorBox]
    started_outside: bool
    reached: bool
    reach_block: str
    contact: str
    peak_nm: float
    peak_actuator: str
    min_up_z: float


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
    soft_pass: bool


def _plant_md5() -> str:
    return hashlib.md5(sw.PLANT_XML.read_bytes()).hexdigest()


def _definition() -> DefinitionJson:
    d_min_m = float(rc.d_min(T_DETECT_S))
    return DefinitionJson(
        reached=(
            "Body COM xy of body_link is inside that room's floor box, "
            "the bout started with that COM outside the same box, "
            f"min up_z is at least {UPRIGHT_UP_Z:.2f}, prop contacts are "
            f"zero, and the bout ends at or before {TIME_LIMIT_S:.1f} s. "
            "A start inside the box is not a reach."
        ),
        floor_box=(
            "Axis-aligned XY of every loaded plane geom named floor or "
            "room_floor. Half-size is the plane size. A single-room file "
            "has one such plane, and that plane is that room's floor. "
            "The box is not a waypoint and is not passed to a command."
        ),
        upright=f"min up_z over the bout >= {UPRIGHT_UP_Z:.2f}",
        prop_contact=(
            f"A contact with normal force >= {PROP_FORCE_N:.1f} N between "
            "a robot geom and any geom other than floor or room_floor."
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
        soft_pass=False,
    )


def _floor_boxes(model: mj.MjModel) -> list[FloorBox]:
    boxes: list[FloorBox] = []
    for geom_id in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name not in FLOOR_GEOM_NAMES:
            continue
        if int(model.geom_type[geom_id]) != int(mj.mjtGeom.mjGEOM_PLANE):
            continue
        pos = np.asarray(model.geom_pos[geom_id], dtype=np.float64)
        size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        boxes.append(
            FloorBox(
                geom=name,
                xmin=float(pos[0] - size[0]),
                xmax=float(pos[0] + size[0]),
                ymin=float(pos[1] - size[1]),
                ymax=float(pos[1] + size[1]),
            )
        )
    return boxes


def _inside(com_x: float, com_y: float, boxes: list[FloorBox]) -> bool:
    for box in boxes:
        if box["xmin"] <= com_x <= box["xmax"] and box["ymin"] <= com_y <= box["ymax"]:
            return True
    return False


def _yaw(data: mj.MjData, body_id: int) -> float:
    rot = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    return float(math.atan2(rot[1, 0], rot[0, 0]))


def _prop_contact(model: mj.MjModel, data: mj.MjData, robot_bodies: set[str]) -> str:
    best_name = ""
    best_force = 0.0
    for index in range(data.ncon):
        con = data.contact[index]
        names = []
        bodies = []
        for geom_id in (int(con.geom1), int(con.geom2)):
            names.append(mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or "")
            body_id = int(model.geom_bodyid[geom_id])
            bodies.append(mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or "")
        if names[0] in FLOOR_GEOM_NAMES or names[1] in FLOOR_GEOM_NAMES:
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


def _robot_bodies(plant: mj.MjModel) -> set[str]:
    names: set[str] = set()
    for body_id in range(plant.nbody):
        name = mj.mj_id2name(plant, mj.mjtObj.mjOBJ_BODY, body_id) or ""
        if name != "":
            names.add(name)
    return names


def _stand(
    model: mj.MjModel,
    data: mj.MjData,
    targets: dict[str, float],
    com_z: float,
    robot_bodies: set[str],
) -> tuple[SpawnJson, str, float, str, float]:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[2] = com_z
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    act_idx = {
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or "": i
        for i in range(model.nu)
    }
    for joint_name, value in targets.items():
        joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise SystemExit(f"FAIL: stand joint missing: {joint_name}")
        data.qpos[int(model.jnt_qposadr[joint_id])] = value
    sw.wg.set_ctrl(model, data, targets, act_idx)
    mj.mj_forward(model, data)
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if body_id < 0:
        raise SystemExit("FAIL: missing body_link")
    peak_abs = -1.0
    peak_signed = 0.0
    peak_name = ""
    min_up_z = 1.0
    contact = "none"
    steps = int(round(rooms.STAND_SETTLE_S / float(model.opt.timestep)))
    for _ in range(steps):
        mj.mj_step(model, data)
        up_z = float(data.xmat[body_id].reshape(3, 3)[2, 2])
        if up_z < min_up_z:
            min_up_z = up_z
        for name, index in act_idx.items():
            if "hip_" not in name and "knee" not in name and "ank_" not in name:
                continue
            force = float(data.actuator_force[index])
            if abs(force) > peak_abs:
                peak_abs = abs(force)
                peak_signed = force
                peak_name = name
        hit = _prop_contact(model, data, robot_bodies)
        if hit != "none":
            contact = hit
    if peak_name == "":
        raise SystemExit("FAIL: no leg actuator force during the stand")
    com = np.asarray(data.subtree_com[body_id], dtype=np.float64)
    spawn = SpawnJson(
        qpos_xyz=[float(data.qpos[0]), float(data.qpos[1]), float(data.qpos[2])],
        yaw_rad=_yaw(data, body_id),
        com_xyz=[float(com[0]), float(com[1]), float(com[2])],
        up_z=float(data.xmat[body_id].reshape(3, 3)[2, 2]),
    )
    return spawn, contact, peak_signed, peak_name, min_up_z


def _measure_room(
    scene: rooms.RoomScene,
    targets: dict[str, float],
    com_z: float,
    robot_bodies: set[str],
    d_min_m: float,
) -> RoomJson:
    phrase = ""
    for name, text in PHRASES:
        if name == scene.name:
            phrase = text
            break
    if phrase == "":
        raise SystemExit(f"FAIL: no phrase for {scene.name}")
    command = voice.parse_phrase(phrase)
    bus_vel = 0
    caller_bus = _CountBus()
    caller = voice.VoiceCaller(caller_bus)
    heard = caller.hear(phrase, 0.0)
    if heard != command.line:
        raise SystemExit(f"FAIL {scene.name}: hear {heard!r} != {command.line!r}")
    bus_vel = caller_bus.vel_count
    if bus_vel != 0:
        raise SystemExit(f"FAIL {scene.name}: voice published vel before a reach bout")
    model = mj.MjModel.from_xml_path(str(scene.xml_path))
    data = mj.MjData(model)
    spawn, contact, peak, peak_name, min_up_z = _stand(
        model, data, targets, com_z, robot_bodies,
    )
    boxes = _floor_boxes(model)
    if not boxes:
        raise SystemExit(f"FAIL {scene.name}: no floor plane in the loaded scene")
    com_x = spawn["com_xyz"][0]
    com_y = spawn["com_xyz"][1]
    inside = _inside(com_x, com_y, boxes)
    started_outside = not inside
    # The other reach bars are not consulted once the start is inside.
    reached = False
    if started_outside:
        raise SystemExit(
            f"FAIL {scene.name}: start is outside the floor box; "
            "this measure does not walk a joined scene that is not in the tree"
        )
    block = (
        "spawn COM is inside the only floor plane of this single-room "
        "scene; started_outside is false"
    )
    words = sorted(set(voice.normalize_phrase(phrase).split()) & voice._ROOM_WORDS)
    return RoomJson(
        room=scene.name,
        scene=scene.xml_name,
        phrase=phrase,
        voice_parse=command.kind,
        voice_line=command.line,
        matched_room_words=words,
        recogniser_label="not asked",
        commanded_vx=None,
        commanded_yaw=None,
        d_min_m=d_min_m,
        d_min_v_mps=float(rc.V_MPS),
        t_detect_s=T_DETECT_S,
        t_stop_s=float(rc.T_STOP_S),
        latch_applied=False,
        spawn=spawn,
        floor_boxes=boxes,
        started_outside=started_outside,
        reached=reached,
        reach_block=block,
        contact=contact,
        peak_nm=peak,
        peak_actuator=peak_name,
        min_up_z=min_up_z,
    )


class _CountBus:
    def __init__(self) -> None:
        self.vel_count = 0

    def stand(self, now: float) -> str | None:
        del now
        return None

    def stop(self, now: float) -> str | None:
        del now
        return None

    def vel(self, vx: float, yaw_rate: float, now: float) -> str | None:
        del vx, yaw_rate, now
        self.vel_count += 1
        return None


def _write(payload: dict[str, object]) -> None:
    SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
    SUMMARY_PATH.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    before = _plant_md5()
    if before != sw.PLANT_MD5 or before != voice.PLANT_MD5:
        raise SystemExit(f"FAIL: plant md5 {before}")
    if abs(voice.FWD_MPS - 0.056) > 1e-12:
        raise SystemExit(f"FAIL: voice forward cap moved to {voice.FWD_MPS}")
    if abs(voice.YAW_RAD_S - 0.25) > 1e-12:
        raise SystemExit(f"FAIL: voice yaw cap moved to {voice.YAW_RAD_S}")
    if abs(rc.V_MPS - 0.150) > 1e-12:
        raise SystemExit(f"FAIL: d_min speed bound moved to {rc.V_MPS}")
    scene_files = sorted(path.name for path in (ROOT / "mujoco").glob("room_*.xml"))
    definition = _definition()
    header: dict[str, object] = {
        "prefer_fail": True,
        "soft_pass": False,
        "go_anywhere": False,
        "kit_safe": False,
        "plant_md5": before,
        "definition": definition,
        "scene_files": scene_files,
        "joined_scene": False,
        "doorway_spawn": False,
        "gap": GAP,
        "rooms": [],
    }
    _write(header)
    sw.apply_frozen_forward_gait()
    targets, com_z = rooms._stand_targets()
    plant = mj.MjModel.from_xml_path(str(sw.PLANT_XML))
    robot_bodies = _robot_bodies(plant)
    d_min_m = float(definition["d_min_m"])
    rows: list[RoomJson] = []
    for name in rooms.ROOM_ORDER:
        row = _measure_room(rooms.ROOMS[name], targets, com_z, robot_bodies, d_min_m)
        rows.append(row)
        spawn = row["spawn"]
        print(
            f"{row['room']}  parse={row['voice_parse']}  "
            f"spawn_com={spawn['com_xyz'][0]:+.3f},{spawn['com_xyz'][1]:+.3f}  "
            f"qpos={spawn['qpos_xyz'][0]:+.3f},{spawn['qpos_xyz'][1]:+.3f}  "
            f"started_outside={row['started_outside']}  reached={row['reached']}  "
            f"commanded_vx=None  d_min={row['d_min_m']:.4f}  "
            f"latch={row['latch_applied']}  contact={row['contact']}  "
            f"peak={row['peak_nm']:+.3f} {row['peak_actuator']}",
            flush=True,
        )
    if _plant_md5() != before:
        raise SystemExit("FAIL: plant file changed during the measure")
    if definition != _definition():
        raise SystemExit("FAIL: reach definition changed after the measure")
    header["rooms"] = rows
    header["reached_any"] = False
    _write(header)
    print("Prefer FAIL  soft-pass=off  reached_any=false", flush=True)
    print(GAP, flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
