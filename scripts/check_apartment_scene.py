#!/usr/bin/env python3
"""Check the joined apartment scene. Soft-pass is off.

Loads mujoco/room_apartment.xml and mujoco/room_apartment.json.
Exits 1 if any assertion fails. Prints the measured numbers either way.

Assertions:
- plant file md5 is 207f3d5e9c6a72e16f7aa0c8d224f75e and stays that
- exactly one floor geom, the plant plane
- furniture geom centers match the single-room files in each room frame
- each doorway clear width is at least 0.80 m
- each doorway spawn is outside its room box, faces the opening,
  and is at least 0.50 m from the opening frame
- no rug/mat/threshold collider overlaps an opening or a spawn
  footprint expanded by 0.30 m
- spawn footprint margin lies on the floor plane and off props
- a 0.60 s quiet stand at each spawn and at the common hall start:
  min up_z >= 0.90, zero prop contacts, peak leg torque <= 2.33 Nm
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import TypedDict

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import render_kit_cam_room as rooms  # noqa: E402
import walk_gait_ainex as wg  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = rooms.steer_walk.PLANT_XML
PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
SCENE_XML = ROOT / "mujoco" / "room_apartment.xml"
SPEC_JSON = ROOT / "mujoco" / "room_apartment.json"
PREVIEW_PNG = ROOT / "previews" / "apartment_topdown.png"
PLANT_INCLUDE = "ainex_hiwonder/ainex_controls_m2_145.xml"

DOOR_MIN_M = 0.80
SETBACK_MIN_M = 0.50
MARGIN_M = 0.30
TORQUE_MAX_NM = 2.33
UPRIGHT_MIN_UP_Z = 0.90
HEADING_COS_MIN = math.cos(math.radians(20.0))
CLEAR_Z = (0.0, 0.90)
PREVIEW_HALF_Y = 3.45

_RUG_RE = re.compile(r"(rug|carpet|threshold|(^|_)mat($|_))", re.IGNORECASE)
_INCLUDE_RE = re.compile(r'<include\s+file="([^"]+)"\s*/>')


class Box(TypedDict):
    xmin: float
    xmax: float
    ymin: float
    ymax: float


class Spawn(TypedDict):
    x: float
    y: float
    yaw: float


class Doorway(TypedDict):
    name: str
    opening: Box
    clear_width_m: float
    spawn: Spawn


class RoomSpec(TypedDict):
    floor_box: Box
    doorways: list[Doorway]


class CommonStart(TypedDict):
    name: str
    x: float
    y: float
    yaw: float


class ApartmentSpec(TypedDict):
    scene: str
    description: str
    common_start: CommonStart
    rooms: dict[str, RoomSpec]


class Interval(TypedDict):
    start: float
    end: float
    name: str


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _fail(problems: list[str], message: str) -> None:
    problems.append(message)
    print(f"FAIL {message}")


def _num(raw: object, label: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise SystemExit(f"FAIL: {label} is not a number")
    return float(raw)


def _box(raw: object, label: str) -> Box:
    if not isinstance(raw, dict):
        raise SystemExit(f"FAIL: {label} is not an object")
    box = Box(
        xmin=_num(raw.get("xmin"), f"{label}.xmin"),
        xmax=_num(raw.get("xmax"), f"{label}.xmax"),
        ymin=_num(raw.get("ymin"), f"{label}.ymin"),
        ymax=_num(raw.get("ymax"), f"{label}.ymax"),
    )
    if box["xmax"] <= box["xmin"] or box["ymax"] <= box["ymin"]:
        raise SystemExit(f"FAIL: {label} is empty")
    return box


def _load_spec(path: Path) -> ApartmentSpec:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise SystemExit("FAIL: apartment spec is not an object")
    scene = raw.get("scene")
    description = raw.get("description")
    if not isinstance(scene, str) or not isinstance(description, str):
        raise SystemExit("FAIL: scene and description must be strings")
    common_raw = raw.get("common_start")
    if not isinstance(common_raw, dict):
        raise SystemExit("FAIL: common_start missing")
    name = common_raw.get("name")
    if not isinstance(name, str):
        raise SystemExit("FAIL: common_start.name missing")
    common = CommonStart(
        name=name,
        x=_num(common_raw.get("x"), "common_start.x"),
        y=_num(common_raw.get("y"), "common_start.y"),
        yaw=_num(common_raw.get("yaw"), "common_start.yaw"),
    )
    rooms_raw = raw.get("rooms")
    if not isinstance(rooms_raw, dict):
        raise SystemExit("FAIL: rooms missing")
    parsed: dict[str, RoomSpec] = {}
    for room_name, room_raw in rooms_raw.items():
        if not isinstance(room_name, str) or not isinstance(room_raw, dict):
            raise SystemExit("FAIL: room entry is not an object")
        doors_raw = room_raw.get("doorways")
        if not isinstance(doors_raw, list) or len(doors_raw) < 1:
            raise SystemExit(f"FAIL: {room_name} needs at least one doorway")
        doors: list[Doorway] = []
        for index, door_raw in enumerate(doors_raw):
            if not isinstance(door_raw, dict):
                raise SystemExit(f"FAIL: {room_name} doorway {index} is not an object")
            door_name = door_raw.get("name")
            spawn_raw = door_raw.get("spawn")
            if not isinstance(door_name, str) or not isinstance(spawn_raw, dict):
                raise SystemExit(f"FAIL: {room_name} doorway {index} is incomplete")
            doors.append(
                Doorway(
                    name=door_name,
                    opening=_box(door_raw.get("opening"), f"{room_name} opening"),
                    clear_width_m=_num(door_raw.get("clear_width_m"), f"{room_name} width"),
                    spawn=Spawn(
                        x=_num(spawn_raw.get("x"), f"{room_name} spawn.x"),
                        y=_num(spawn_raw.get("y"), f"{room_name} spawn.y"),
                        yaw=_num(spawn_raw.get("yaw"), f"{room_name} spawn.yaw"),
                    ),
                )
            )
        parsed[room_name] = RoomSpec(
            floor_box=_box(room_raw.get("floor_box"), f"{room_name} floor_box"),
            doorways=doors,
        )
    return ApartmentSpec(
        scene=scene,
        description=description,
        common_start=common,
        rooms=parsed,
    )


def _inside(x: float, y: float, box: Box) -> bool:
    return box["xmin"] <= x <= box["xmax"] and box["ymin"] <= y <= box["ymax"]


def _dist_to_box(x: float, y: float, box: Box) -> float:
    dx = max(box["xmin"] - x, 0.0, x - box["xmax"])
    dy = max(box["ymin"] - y, 0.0, y - box["ymax"])
    return math.hypot(dx, dy)


def _long_width(box: Box) -> float:
    return max(box["xmax"] - box["xmin"], box["ymax"] - box["ymin"])


def _short_width(box: Box) -> float:
    return min(box["xmax"] - box["xmin"], box["ymax"] - box["ymin"])


def _yaw_quat(yaw: float) -> list[float]:
    half = 0.5 * yaw
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def _body_name(model: mj.MjModel, body_id: int) -> str:
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or ""


def _geom_name(model: mj.MjModel, geom_id: int) -> str:
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or ""


def _group_of(model: mj.MjModel, body_id: int) -> str:
    current = body_id
    while current > 0:
        name = _body_name(model, current)
        if name.startswith("mount_"):
            return name
        if name == "shell":
            return "shell"
        current = int(model.body_parentid[current])
    return "world"


def _robot_bodies(plant: mj.MjModel) -> set[str]:
    names: set[str] = set()
    for body_id in range(plant.nbody):
        name = _body_name(plant, body_id)
        if name != "" and name != "world":
            names.add(name)
    return names


def _is_rug(name: str) -> bool:
    return _RUG_RE.search(name) is not None


def _box_corners(model: mj.MjModel, data: mj.MjData, geom_id: int) -> np.ndarray:
    pos = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    mat = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    corners: list[np.ndarray] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array([sx * size[0], sy * size[1], sz * size[2]], dtype=np.float64)
                corners.append(pos + mat @ local)
    return np.stack(corners, axis=0)


def _aabb(model: mj.MjModel, data: mj.MjData, geom_id: int) -> tuple[np.ndarray, np.ndarray]:
    geom_type = int(model.geom_type[geom_id])
    if geom_type == int(mj.mjtGeom.mjGEOM_PLANE):
        pos = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        low = np.array([pos[0] - size[0], pos[1] - size[1], -0.05], dtype=np.float64)
        high = np.array([pos[0] + size[0], pos[1] + size[1], 0.05], dtype=np.float64)
        return low, high
    if geom_type == int(mj.mjtGeom.mjGEOM_BOX):
        corners = _box_corners(model, data, geom_id)
        return corners.min(axis=0), corners.max(axis=0)
    pos = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    radius = float(model.geom_size[geom_id][0])
    return pos - radius, pos + radius


def _overlap(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray, eps: float) -> bool:
    return bool(np.all(a1 > b0 + eps) and np.all(b1 > a0 + eps))


def _overlap_xy(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray, eps: float) -> bool:
    return bool(
        a1[0] > b0[0] + eps
        and b1[0] > a0[0] + eps
        and a1[1] > b0[1] + eps
        and b1[1] > a0[1] + eps
    )


def _opening_axis(opening: Box) -> str:
    if (opening["ymax"] - opening["ymin"]) >= (opening["xmax"] - opening["xmin"]):
        return "y"
    return "x"


def _width_interval(opening: Box) -> tuple[float, float]:
    if _opening_axis(opening) == "y":
        return opening["ymin"], opening["ymax"]
    return opening["xmin"], opening["xmax"]


def _blocker_interval(
    opening: Box,
    low: np.ndarray,
    high: np.ndarray,
) -> tuple[float, float] | None:
    if _opening_axis(opening) == "y":
        if high[0] <= opening["xmin"] + 1e-4 or low[0] >= opening["xmax"] - 1e-4:
            return None
        return float(low[1]), float(high[1])
    if high[1] <= opening["ymin"] + 1e-4 or low[1] >= opening["ymax"] - 1e-4:
        return None
    return float(low[0]), float(high[0])


def _rect_corners_xy(box: Box) -> list[tuple[float, float]]:
    return [
        (box["xmin"], box["ymin"]),
        (box["xmin"], box["ymax"]),
        (box["xmax"], box["ymin"]),
        (box["xmax"], box["ymax"]),
    ]


def _overlap_box_xy(low: np.ndarray, high: np.ndarray, box: Box, eps: float) -> bool:
    return bool(
        high[0] > box["xmin"] + eps
        and low[0] < box["xmax"] - eps
        and high[1] > box["ymin"] + eps
        and low[1] < box["ymax"] - eps
    )


def _leg_actuators(model: mj.MjModel) -> dict[str, int]:
    found: dict[str, int] = {}
    for index in range(model.nu):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, index) or ""
        if "hip_" in name or "knee" in name or "ank_" in name:
            found[name] = index
    return found


def _pose(
    model: mj.MjModel,
    data: mj.MjData,
    spawn: Spawn,
    targets: dict[str, float],
    com_z: float,
    actuators: dict[str, int],
) -> None:
    mj.mj_resetData(model, data)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[0] = spawn["x"]
    data.qpos[1] = spawn["y"]
    data.qpos[2] = com_z
    data.qpos[3:7] = _yaw_quat(spawn["yaw"])
    for joint_name, value in targets.items():
        joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise SystemExit(f"FAIL: stand joint missing: {joint_name}")
        data.qpos[int(model.jnt_qposadr[joint_id])] = value
    wg.set_ctrl(model, data, targets, actuators)
    mj.mj_forward(model, data)


def _prop_hits(
    model: mj.MjModel,
    data: mj.MjData,
    robot_bodies: set[str],
) -> list[tuple[str, float]]:
    hits: list[tuple[str, float]] = []
    for index in range(data.ncon):
        contact = data.contact[index]
        geom_ids = (int(contact.geom1), int(contact.geom2))
        names = [_geom_name(model, geom_id) for geom_id in geom_ids]
        if "floor" in names:
            continue
        bodies = [
            _body_name(model, int(model.geom_bodyid[geom_id])) for geom_id in geom_ids
        ]
        robot = [body in robot_bodies for body in bodies]
        if robot[0] == robot[1]:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, index, force)
        prop = names[0] if robot[1] else names[1]
        hits.append((prop, float(force[0])))
    return hits


def _foot_margin(
    model: mj.MjModel,
    data: mj.MjData,
) -> tuple[np.ndarray, np.ndarray]:
    lows: list[np.ndarray] = []
    highs: list[np.ndarray] = []
    for name in ("l_foot_contact", "r_foot_contact"):
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise SystemExit(f"FAIL: missing {name}")
        low, high = _aabb(model, data, geom_id)
        lows.append(low)
        highs.append(high)
    low = np.minimum(lows[0], lows[1]).copy()
    high = np.maximum(highs[0], highs[1]).copy()
    low[0] -= MARGIN_M
    low[1] -= MARGIN_M
    high[0] += MARGIN_M
    high[1] += MARGIN_M
    return low, high


def _penetrations(
    model: mj.MjModel,
    data: mj.MjData,
    robot_bodies: set[str],
) -> list[str]:
    found: list[str] = []
    robot_geoms: list[int] = []
    prop_geoms: list[int] = []
    for geom_id in range(model.ngeom):
        if int(model.geom_contype[geom_id]) == 0 and int(model.geom_conaffinity[geom_id]) == 0:
            continue
        name = _geom_name(model, geom_id)
        body = _body_name(model, int(model.geom_bodyid[geom_id]))
        if name == "floor":
            continue
        if body in robot_bodies:
            robot_geoms.append(geom_id)
        else:
            prop_geoms.append(geom_id)
    fromto = np.zeros(6, dtype=np.float64)
    for robot_geom in robot_geoms:
        for prop_geom in prop_geoms:
            dist = float(mj.mj_geomDistance(model, data, robot_geom, prop_geom, 0.02, fromto))
            if dist < 1e-4:
                found.append(
                    f"{_geom_name(model, robot_geom)} vs {_geom_name(model, prop_geom)} {dist:.4f} m"
                )
    return found


def _measured_gap(
    model: mj.MjModel,
    data: mj.MjData,
    opening: Box,
    robot_bodies: set[str],
) -> tuple[float, list[str]]:
    blockers: list[Interval] = []
    problems: list[str] = []
    o0, o1 = _width_interval(opening)
    for geom_id in range(model.ngeom):
        name = _geom_name(model, geom_id)
        body = _body_name(model, int(model.geom_bodyid[geom_id]))
        if name == "floor" or body in robot_bodies or body == "world":
            continue
        if int(model.geom_contype[geom_id]) == 0:
            continue
        low, high = _aabb(model, data, geom_id)
        if float(high[2]) <= CLEAR_Z[0] + 1e-4 or float(low[2]) >= CLEAR_Z[1] - 1e-4:
            continue
        interval = _blocker_interval(opening, low, high)
        if interval is None:
            continue
        start, end = interval
        if end > o0 + 1e-4 and start < o1 - 1e-4:
            problems.append(f"{name} occupies the opening on [{start:.3f},{end:.3f}]")
        blockers.append(Interval(start=start, end=end, name=name))
    left_end = None
    left_name = ""
    right_start = None
    right_name = ""
    for blocker in blockers:
        if blocker["end"] <= o0 + 1e-3 and (left_end is None or blocker["end"] > left_end):
            left_end = blocker["end"]
            left_name = blocker["name"]
        if blocker["start"] >= o1 - 1e-3 and (right_start is None or blocker["start"] < right_start):
            right_start = blocker["start"]
            right_name = blocker["name"]
    if left_end is None or right_start is None:
        problems.append(
            f"opening is not framed (left={left_name or 'none'} right={right_name or 'none'})"
        )
        return -1.0, problems
    return float(right_start - left_end), problems


def _check_include(problems: list[str]) -> None:
    text = SCENE_XML.read_text(encoding="utf-8")
    includes = _INCLUDE_RE.findall(text)
    if includes != [PLANT_INCLUDE] or text.count("<include") != 1:
        _fail(problems, f"apartment includes {includes}, expected [{PLANT_INCLUDE}]")
    else:
        print(f"include {PLANT_INCLUDE} once")


def _check_floor(model: mj.MjModel, problems: list[str]) -> Box:
    planes: list[str] = []
    named: list[str] = []
    floor_box = Box(xmin=0.0, xmax=0.0, ymin=0.0, ymax=0.0)
    for geom_id in range(model.ngeom):
        name = _geom_name(model, geom_id)
        if name in ("floor", "room_floor") or "room_floor" in name:
            named.append(name)
        if int(model.geom_type[geom_id]) == int(mj.mjtGeom.mjGEOM_PLANE):
            planes.append(name)
            pos = np.asarray(model.geom_pos[geom_id], dtype=np.float64)
            size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
            floor_box = Box(
                xmin=float(pos[0] - size[0]),
                xmax=float(pos[0] + size[0]),
                ymin=float(pos[1] - size[1]),
                ymax=float(pos[1] + size[1]),
            )
    if planes != ["floor"] or named != ["floor"]:
        _fail(problems, f"floor geoms planes={planes} named={named}, expected one plane named floor")
    else:
        print(
            f"floor_geoms 1 name=floor "
            f"x[{floor_box['xmin']:.3f},{floor_box['xmax']:.3f}] "
            f"y[{floor_box['ymin']:.3f},{floor_box['ymax']:.3f}]"
        )
    return floor_box


def _check_relative(
    model: mj.MjModel,
    data: mj.MjData,
    plant_bodies: set[str],
    problems: list[str],
) -> None:
    worst = 0.0
    worst_name = ""
    for room_name in rooms.ROOM_ORDER:
        single = mj.MjModel.from_xml_path(str(ROOT / "mujoco" / f"room_{room_name}.xml"))
        single_data = mj.MjData(single)
        mj.mj_forward(single, single_data)
        mount_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"mount_{room_name}")
        if mount_id < 0:
            _fail(problems, f"missing body mount_{room_name}")
            continue
        mount_pos = np.asarray(data.xpos[mount_id], dtype=np.float64)
        mount_rot = np.asarray(data.xmat[mount_id], dtype=np.float64).reshape(3, 3)
        for geom_id in range(single.ngeom):
            name = _geom_name(single, geom_id)
            body = _body_name(single, int(single.geom_bodyid[geom_id]))
            if body in plant_bodies or body == "world" or name == "floor":
                continue
            apt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
            if apt_id < 0:
                _fail(problems, f"{room_name} geom {name} missing from the apartment")
                continue
            local = np.asarray(single_data.geom_xpos[geom_id], dtype=np.float64)
            expected = mount_pos + mount_rot @ local
            actual = np.asarray(data.geom_xpos[apt_id], dtype=np.float64)
            err = float(np.max(np.abs(expected - actual)))
            if err > worst:
                worst = err
                worst_name = name
            if err > 1e-4:
                _fail(problems, f"{name} moved {err:.6f} m relative to {room_name}")
            size_err = float(np.max(np.abs(single.geom_size[geom_id] - model.geom_size[apt_id])))
            if size_err > 1e-6:
                _fail(problems, f"{name} size changed by {size_err:.6f}")
    print(f"furniture_relative_max_err {worst:.6e} m ({worst_name})")


def _check_layout(
    model: mj.MjModel,
    data: mj.MjData,
    spec: ApartmentSpec,
    robot_bodies: set[str],
    floor_box: Box,
    problems: list[str],
) -> None:
    names = list(spec["rooms"])
    for i, left_name in enumerate(names):
        left = spec["rooms"][left_name]["floor_box"]
        if not (
            left["xmin"] >= floor_box["xmin"] - 1e-6
            and left["xmax"] <= floor_box["xmax"] + 1e-6
            and left["ymin"] >= floor_box["ymin"] - 1e-6
            and left["ymax"] <= floor_box["ymax"] + 1e-6
        ):
            _fail(problems, f"{left_name} floor box is outside the plant floor")
        for right_name in names[i + 1 :]:
            right = spec["rooms"][right_name]["floor_box"]
            if (
                left["xmax"] > right["xmin"] + 1e-6
                and right["xmax"] > left["xmin"] + 1e-6
                and left["ymax"] > right["ymin"] + 1e-6
                and right["ymax"] > left["ymin"] + 1e-6
            ):
                _fail(problems, f"room boxes overlap: {left_name} and {right_name}")

    geoms: list[tuple[int, str, np.ndarray, np.ndarray]] = []
    for geom_id in range(model.ngeom):
        name = _geom_name(model, geom_id)
        body = _body_name(model, int(model.geom_bodyid[geom_id]))
        if name == "floor" or body in robot_bodies or body == "world":
            continue
        if int(model.geom_contype[geom_id]) == 0:
            continue
        low, high = _aabb(model, data, geom_id)
        geoms.append((geom_id, _group_of(model, int(model.geom_bodyid[geom_id])), low, high))
        group = geoms[-1][1]
        if group.startswith("mount_"):
            room_name = group[len("mount_") :]
            box = spec["rooms"][room_name]["floor_box"]
            inset = 0.01
            if (
                float(low[0]) < box["xmin"] + inset
                or float(high[0]) > box["xmax"] - inset
                or float(low[1]) < box["ymin"] + inset
                or float(high[1]) > box["ymax"] - inset
            ):
                _fail(
                    problems,
                    f"{name} is not 0.01 m inside {room_name} "
                    f"xy[{float(low[0]):.3f},{float(high[0]):.3f}]"
                    f"[{float(low[1]):.3f},{float(high[1]):.3f}]",
                )

    for i, (gid_a, group_a, low_a, high_a) in enumerate(geoms):
        for gid_b, group_b, low_b, high_b in geoms[i + 1 :]:
            if group_a == group_b and group_a.startswith("mount_"):
                continue
            if _overlap(low_a, high_a, low_b, high_b, 1e-4):
                _fail(
                    problems,
                    f"props overlap {_geom_name(model, gid_a)} and {_geom_name(model, gid_b)}",
                )


def _check_doorways(
    model: mj.MjModel,
    data: mj.MjData,
    spec: ApartmentSpec,
    robot_bodies: set[str],
    problems: list[str],
) -> None:
    for room_name in rooms.ROOM_ORDER:
        room = spec["rooms"][room_name]
        for door in room["doorways"]:
            opening = door["opening"]
            declared = door["clear_width_m"]
            measured, blocks = _measured_gap(model, data, opening, robot_bodies)
            prefix = f"{room_name} {door['name']}"
            for block in blocks:
                _fail(problems, f"{prefix} {block}")
            width = _long_width(opening)
            short = _short_width(opening)
            print(
                f"{prefix} clear_width declared={declared:.3f} m "
                f"rect={width:.3f} m measured_gap={measured:.3f} m"
            )
            if declared < DOOR_MIN_M or width < DOOR_MIN_M or measured < DOOR_MIN_M:
                _fail(
                    problems,
                    f"{prefix} clear width {measured:.3f} m is under {DOOR_MIN_M:.2f} m",
                )
            if abs(declared - width) > 1e-3 or abs(measured - width) > 1e-3:
                _fail(problems, f"{prefix} width sources disagree")
            if short < 0.05 or short > 0.15:
                _fail(problems, f"{prefix} opening slab thickness {short:.3f} m is not a wall hole")
            spawn = door["spawn"]
            setback = _dist_to_box(spawn["x"], spawn["y"], opening)
            outside = not _inside(spawn["x"], spawn["y"], room["floor_box"])
            in_any = [
                other
                for other, other_room in spec["rooms"].items()
                if _inside(spawn["x"], spawn["y"], other_room["floor_box"])
            ]
            center = np.array(
                [
                    0.5 * (opening["xmin"] + opening["xmax"]),
                    0.5 * (opening["ymin"] + opening["ymax"]),
                ],
                dtype=np.float64,
            )
            forward = np.array([math.cos(spawn["yaw"]), math.sin(spawn["yaw"])], dtype=np.float64)
            toward = center - np.array([spawn["x"], spawn["y"]], dtype=np.float64)
            toward_n = toward / max(float(np.linalg.norm(toward)), 1e-9)
            heading = float(forward @ toward_n)
            print(
                f"{prefix} spawn x={spawn['x']:+.3f} y={spawn['y']:+.3f} "
                f"yaw={spawn['yaw']:+.3f} setback={setback:.3f} m "
                f"outside_box={outside} heading_cos={heading:.3f}"
            )
            if not outside or in_any:
                _fail(problems, f"{prefix} spawn is inside {in_any or [room_name]}")
            if setback < SETBACK_MIN_M:
                _fail(problems, f"{prefix} setback {setback:.3f} m is under {SETBACK_MIN_M:.2f} m")
            if heading < HEADING_COS_MIN:
                _fail(problems, f"{prefix} spawn is not facing the opening")


def _check_rugs_and_margin(
    model: mj.MjModel,
    data: mj.MjData,
    spec: ApartmentSpec,
    robot_bodies: set[str],
    floor_box: Box,
    targets: dict[str, float],
    com_z: float,
    actuators: dict[str, int],
    problems: list[str],
) -> None:
    rugs: list[tuple[str, np.ndarray, np.ndarray]] = []
    props: list[tuple[str, np.ndarray, np.ndarray]] = []
    for geom_id in range(model.ngeom):
        name = _geom_name(model, geom_id)
        body = _body_name(model, int(model.geom_bodyid[geom_id]))
        if name == "floor" or body in robot_bodies:
            continue
        if int(model.geom_contype[geom_id]) == 0:
            continue
        low, high = _aabb(model, data, geom_id)
        props.append((name, low, high))
        if _is_rug(name):
            rugs.append((name, low, high))
    print(f"rug_geoms {[name for name, _low, _high in rugs]}")
    for rug_name, low, high in rugs:
        for room_name in rooms.ROOM_ORDER:
            for door in spec["rooms"][room_name]["doorways"]:
                if _overlap_box_xy(low, high, door["opening"], 1e-6):
                    _fail(problems, f"{rug_name} overlaps doorway {door['name']}")

    hall = spec["common_start"]
    hall_spawn = Spawn(x=hall["x"], y=hall["y"], yaw=hall["yaw"])
    poses: list[tuple[str, Spawn]] = [("hall", hall_spawn)]
    for room_name in rooms.ROOM_ORDER:
        for door in spec["rooms"][room_name]["doorways"]:
            poses.append((f"{room_name}:{door['name']}", door["spawn"]))
    for label, spawn in poses:
        _pose(model, data, spawn, targets, com_z, actuators)
        margin_low, margin_high = _foot_margin(model, data)
        on_floor = (
            float(margin_low[0]) >= floor_box["xmin"] - 1e-6
            and float(margin_high[0]) <= floor_box["xmax"] + 1e-6
            and float(margin_low[1]) >= floor_box["ymin"] - 1e-6
            and float(margin_high[1]) <= floor_box["ymax"] + 1e-6
        )
        rug_hit = [
            name
            for name, low, high in rugs
            if _overlap_xy(margin_low, margin_high, low, high, 1e-6)
        ]
        prop_hit = [
            name
            for name, low, high in props
            if _overlap_xy(margin_low, margin_high, low, high, 1e-4)
        ]
        print(
            f"{label} footprint+{MARGIN_M:.2f} "
            f"x[{float(margin_low[0]):+.3f},{float(margin_high[0]):+.3f}] "
            f"y[{float(margin_low[1]):+.3f},{float(margin_high[1]):+.3f}] "
            f"on_floor={on_floor} rug={rug_hit or 'none'} prop={prop_hit or 'none'}"
        )
        if not on_floor:
            _fail(problems, f"{label} footprint margin leaves the floor plane")
        if rug_hit:
            _fail(problems, f"{label} footprint margin overlaps rug {rug_hit}")
        if prop_hit:
            _fail(problems, f"{label} footprint margin overlaps {prop_hit}")


def _up_z(data: mj.MjData, body_id: int) -> float:
    return float(np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)[2, 2])


def _com_xy(data: mj.MjData, body_id: int) -> tuple[float, float]:
    com = np.asarray(data.subtree_com[body_id], dtype=np.float64)
    return float(com[0]), float(com[1])


def _stand_one(
    model: mj.MjModel,
    data: mj.MjData,
    label: str,
    spawn: Spawn,
    room_box: Box | None,
    spec: ApartmentSpec,
    targets: dict[str, float],
    com_z: float,
    actuators: dict[str, int],
    robot_bodies: set[str],
    problems: list[str],
) -> None:
    _pose(model, data, spawn, targets, com_z, actuators)
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if body_id < 0:
        raise SystemExit("FAIL: missing body_link")
    penetrations = _penetrations(model, data, robot_bodies)
    if penetrations:
        _fail(problems, f"{label} spawn intersects {penetrations[0]}")
        if len(penetrations) > 1:
            print(f"  {label} further intersections {len(penetrations) - 1}")
    com_x, com_y = _com_xy(data, body_id)
    if room_box is not None and _inside(com_x, com_y, room_box):
        _fail(problems, f"{label} start COM {com_x:+.3f},{com_y:+.3f} is inside the room box")
    if room_box is None:
        for room_name, room in spec["rooms"].items():
            if _inside(spawn["x"], spawn["y"], room["floor_box"]) or _inside(com_x, com_y, room["floor_box"]):
                _fail(problems, f"{label} common start is inside {room_name}")
    steps = int(round(rooms.STAND_SETTLE_S / float(model.opt.timestep)))
    min_up = _up_z(data, body_id)
    prop_count = 0
    worst_name = ""
    worst_force = 0.0
    peak_abs = -1.0
    peak_signed = 0.0
    peak_name = ""
    for _ in range(steps):
        mj.mj_step(model, data)
        up = _up_z(data, body_id)
        if up < min_up:
            min_up = up
        for name, index in actuators.items():
            force = float(data.actuator_force[index])
            if abs(force) > peak_abs:
                peak_abs = abs(force)
                peak_signed = force
                peak_name = name
        for prop, normal in _prop_hits(model, data, robot_bodies):
            prop_count += 1
            if abs(normal) >= abs(worst_force):
                worst_force = normal
                worst_name = prop
    end_x, end_y = _com_xy(data, body_id)
    if room_box is not None and _inside(end_x, end_y, room_box):
        _fail(problems, f"{label} settled COM {end_x:+.3f},{end_y:+.3f} entered the room box")
    if room_box is None:
        for room_name, room in spec["rooms"].items():
            if _inside(end_x, end_y, room["floor_box"]):
                _fail(problems, f"{label} settled COM entered {room_name}")
    contact = "0" if prop_count == 0 else f"{prop_count} worst={worst_name} {worst_force:.2f} N"
    print(
        f"{label} stand {rooms.STAND_SETTLE_S:.2f}s "
        f"min_up_z={min_up:.6f} prop_contacts={contact} "
        f"peak_nm={peak_signed:+.4f} {peak_name} "
        f"com0={com_x:+.3f},{com_y:+.3f} com1={end_x:+.3f},{end_y:+.3f}"
    )
    if min_up < UPRIGHT_MIN_UP_Z:
        _fail(problems, f"{label} min_up_z {min_up:.6f} is under {UPRIGHT_MIN_UP_Z:.2f}")
    if prop_count != 0:
        _fail(problems, f"{label} prop_contacts={prop_count} ({worst_name})")
    if peak_abs > TORQUE_MAX_NM:
        _fail(problems, f"{label} peak leg torque {peak_signed:+.4f} Nm exceeds {TORQUE_MAX_NM:.2f}")
    if peak_name == "":
        _fail(problems, f"{label} recorded no leg actuator force")


def _world_px(x: float, y: float, width: int, height: int) -> tuple[float, float]:
    half_x = PREVIEW_HALF_Y * (float(width) / float(height))
    u = float(width) / 2.0 + (x / half_x) * (float(width) / 2.0)
    v = float(height) / 2.0 - (y / PREVIEW_HALF_Y) * (float(height) / 2.0)
    return u, v


def _write_preview(
    model: mj.MjModel,
    data: mj.MjData,
    spec: ApartmentSpec,
    targets: dict[str, float],
    com_z: float,
    actuators: dict[str, int],
    problems: list[str],
) -> None:
    from PIL import Image, ImageDraw, ImageFont

    hall_spawn = Spawn(
        x=spec["common_start"]["x"],
        y=spec["common_start"]["y"],
        yaw=spec["common_start"]["yaw"],
    )
    _pose(model, data, hall_spawn, targets, com_z, actuators)
    width = 960
    height = 720
    camera = mj.MjvCamera()
    mj.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = [0.0, 0.0, 0.0]
    camera.distance = 12.0
    camera.azimuth = 90.0
    camera.elevation = -90.0
    renderer = mj.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera)
        for slot in range(2):
            gl_cam = renderer.scene.camera[slot]
            gl_cam.pos[:] = [0.0, 0.0, 12.0]
            gl_cam.forward[:] = [0.0, 0.0, -1.0]
            gl_cam.up[:] = [0.0, 1.0, 0.0]
            gl_cam.orthographic = 1
            gl_cam.frustum_center = 0.0
            gl_cam.frustum_bottom = -PREVIEW_HALF_Y
            gl_cam.frustum_top = PREVIEW_HALF_Y
            gl_cam.frustum_near = 0.2
            gl_cam.frustum_far = 40.0
        image = np.asarray(renderer.render(), dtype=np.uint8).copy()
    finally:
        renderer.close()
    if image.shape != (height, width, 3):
        _fail(problems, f"preview shape {image.shape}")
        return

    colors: dict[str, tuple[int, int, int]] = {
        "kitchen": (255, 176, 40),
        "bathroom": (40, 210, 220),
        "living": (230, 110, 40),
        "bedroom": (190, 90, 230),
        "entrance": (40, 190, 170),
    }
    canvas = Image.fromarray(image)
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
    small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)

    def px(x: float, y: float) -> tuple[float, float]:
        return _world_px(x, y, width, height)

    for room_name in rooms.ROOM_ORDER:
        room = spec["rooms"][room_name]
        box = room["floor_box"]
        color = colors[room_name]
        p0 = px(box["xmin"], box["ymax"])
        p1 = px(box["xmax"], box["ymin"])
        draw.rectangle([p0, p1], outline=color, width=3)
        label_at = px(0.5 * (box["xmin"] + box["xmax"]), 0.5 * (box["ymin"] + box["ymax"]))
        draw.text((label_at[0] - 36, label_at[1] - 10), room_name, fill=color, font=font)
        for door in room["doorways"]:
            opening = door["opening"]
            d0 = px(opening["xmin"], opening["ymax"])
            d1 = px(opening["xmax"], opening["ymin"])
            draw.rectangle([d0, d1], outline=(255, 220, 40), width=3)
            spawn = door["spawn"]
            sx, sy = px(spawn["x"], spawn["y"])
            forward = (
                math.cos(spawn["yaw"]) * 0.45,
                math.sin(spawn["yaw"]) * 0.45,
            )
            ex, ey = px(spawn["x"] + forward[0], spawn["y"] + forward[1])
            draw.line([(sx, sy), (ex, ey)], fill=(255, 230, 60), width=3)
            draw.ellipse([sx - 5, sy - 5, sx + 5, sy + 5], fill=(255, 230, 60))
    start = spec["common_start"]
    hx, hy = px(start["x"], start["y"])
    draw.ellipse([hx - 7, hy - 7, hx + 7, hy + 7], outline=(255, 255, 255), width=2)
    draw.text((hx + 10, hy - 18), "start", fill=(255, 255, 255), font=small)
    draw.text((12, 8), "apartment top-down  boxes=rooms  arrows=outside spawns", fill=(240, 240, 240), font=small)
    PREVIEW_PNG.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(PREVIEW_PNG)
    print(f"preview {PREVIEW_PNG.relative_to(ROOT)}")


def main() -> int:
    problems: list[str] = []
    before = _md5(PLANT_XML)
    print(f"plant_md5 {before}")
    if before != PLANT_MD5 or before != rooms.steer_walk.PLANT_MD5:
        _fail(problems, f"plant md5 {before} != {PLANT_MD5}")
    if not SCENE_XML.is_file() or not SPEC_JSON.is_file():
        _fail(problems, "missing apartment scene or spec")
        return 1
    spec = _load_spec(SPEC_JSON)
    if spec["scene"] != "mujoco/room_apartment.xml":
        _fail(problems, f"spec scene {spec['scene']}")
    if set(spec["rooms"]) != set(rooms.ROOM_ORDER):
        _fail(problems, f"spec rooms {sorted(spec['rooms'])} != {list(rooms.ROOM_ORDER)}")
    _check_include(problems)

    model = mj.MjModel.from_xml_path(str(SCENE_XML))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    floor_box = _check_floor(model, problems)
    plant = mj.MjModel.from_xml_path(str(PLANT_XML))
    robot_bodies = _robot_bodies(plant)
    _check_relative(model, data, robot_bodies | {"world"}, problems)
    _check_layout(model, data, spec, robot_bodies, floor_box, problems)
    _check_doorways(model, data, spec, robot_bodies, problems)

    targets, com_z = rooms._stand_targets()
    actuators = _leg_actuators(model)
    if len(actuators) != 12:
        _fail(problems, f"leg actuators {len(actuators)} != 12")
    _check_rugs_and_margin(
        model, data, spec, robot_bodies, floor_box, targets, com_z, actuators, problems,
    )
    hall_spawn = Spawn(
        x=spec["common_start"]["x"],
        y=spec["common_start"]["y"],
        yaw=spec["common_start"]["yaw"],
    )
    _stand_one(
        model, data, "hall common_start", hall_spawn, None, spec,
        targets, com_z, actuators, robot_bodies, problems,
    )
    for room_name in rooms.ROOM_ORDER:
        room = spec["rooms"][room_name]
        for door in room["doorways"]:
            _stand_one(
                model, data, f"{room_name} {door['name']}", door["spawn"], room["floor_box"],
                spec, targets, com_z, actuators, robot_bodies, problems,
            )
    try:
        _write_preview(model, data, spec, targets, com_z, actuators, problems)
    except mj.FatalError as exc:
        _fail(problems, f"preview render {exc}")

    after = _md5(PLANT_XML)
    if after != before:
        _fail(problems, f"plant md5 changed to {after}")
    else:
        print(f"plant_md5_unchanged {after}")
    print(
        f"bars door>={DOOR_MIN_M:.2f} m setback>={SETBACK_MIN_M:.2f} m "
        f"margin={MARGIN_M:.2f} m up_z>={UPRIGHT_MIN_UP_Z:.2f} "
        f"torque<={TORQUE_MAX_NM:.2f} Nm soft_pass=off"
    )
    if problems:
        print(f"RESULT fail={len(problems)}")
        return 1
    print("RESULT pass")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
