#!/usr/bin/env python3
"""Stand the frozen plant in a vision room and save a kit_cam still.

Each room XML includes the walk plant and adds static furniture. This
script does not edit that file, the gait, the tip check, or CommandBus.
It does not build a map, and it does not claim a go-to or an arrival.

Default (no arguments) is the kitchen scene, written to
previews/kit_cam_room.png. --room picks one scene. --all renders kitchen
plus bathroom, living, bedroom, and entrance.

Exit status is 1 when a still is still the empty checkerboard, or when
that scene's named bodies do not land in the kit_cam frame.
"""
from __future__ import annotations

import argparse
import hashlib
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import steer_walk  # noqa: E402
import walk_gait_ainex as wg  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = steer_walk.PLANT_XML
PLANT_INCLUDE = "ainex_hiwonder/ainex_controls_m2_145.xml"

WIDTH = 640
HEIGHT = 480
STAND_SETTLE_S = 0.60
# Pixels whose max channel differs from the empty-plant kit_cam frame.
# Furniture behind the camera moves ~0 of the image. The room layout moves
# well above this. A soft floor shadow does not.
EMPTY_DIFF = 40
EMPTY_MIN_FRAC = 0.08
MIN_BOX_PX = 36.0

_INCLUDE_RE = re.compile(r'<include\s+file="([^"]+)"\s*/>')
_BANNED_NAME_TOKENS = ("door", "lever", "latch", "hinge")
_BANNED_ROOM_TAGS = (
    "<joint",
    "<freejoint",
    "<equality",
    "<actuator",
    "<tendon",
    "<camera",
    "<sensor",
)


@dataclass(frozen=True)
class RoomScene:
    """One vision scene. Bodies are labels, not a navigation map."""

    name: str
    xml_name: str
    png_name: str
    bodies: tuple[str, ...]
    geoms: tuple[tuple[str, str], ...]

    @property
    def xml_path(self) -> Path:
        return ROOT / "mujoco" / self.xml_name

    @property
    def png_path(self) -> Path:
        return ROOT / "previews" / self.png_name

    @property
    def room_body(self) -> str:
        return self.bodies[0]


ROOMS: dict[str, RoomScene] = {
    "kitchen": RoomScene(
        name="kitchen",
        xml_name="room_kitchen.xml",
        png_name="kit_cam_room.png",
        bodies=("kitchen", "table", "chair"),
        geoms=(
            ("kitchen", "kitchen_cabinet"),
            ("table", "table_top"),
            ("chair", "chair_seat"),
        ),
    ),
    "bathroom": RoomScene(
        name="bathroom",
        xml_name="room_bathroom.xml",
        png_name="kit_cam_room_bathroom.png",
        bodies=("bathroom", "sink", "toilet", "bathtub"),
        geoms=(
            ("bathroom", "bathroom_tile"),
            ("sink", "sink_basin"),
            ("toilet", "toilet_bowl"),
            ("bathtub", "bathtub_shell"),
        ),
    ),
    "living": RoomScene(
        name="living",
        xml_name="room_living.xml",
        png_name="kit_cam_room_living.png",
        bodies=("living", "tv"),
        geoms=(
            ("living", "living_sofa_back"),
            ("tv", "tv_screen"),
        ),
    ),
    "bedroom": RoomScene(
        name="bedroom",
        xml_name="room_bedroom.xml",
        png_name="kit_cam_room_bedroom.png",
        bodies=("bedroom", "nightstand"),
        geoms=(
            ("bedroom", "bedroom_headboard"),
            ("nightstand", "nightstand_top"),
        ),
    ),
    "entrance": RoomScene(
        name="entrance",
        xml_name="room_entrance.xml",
        png_name="kit_cam_room_entrance.png",
        bodies=("entrance", "mat", "shoes"),
        geoms=(
            ("entrance", "entrance_panel"),
            ("mat", "mat_rug"),
            ("shoes", "shoes_pair"),
        ),
    ),
}

ROOM_ORDER: tuple[str, ...] = ("kitchen", "bathroom", "living", "bedroom", "entrance")


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _stand_targets() -> tuple[dict[str, float], float]:
    """Same quiet stand SteerSession uses after the frozen gait constants."""
    steer_walk.apply_frozen_forward_gait()
    targets = wg.gait_targets(0.0, False, 0.0)
    return targets, float(wg.COM_Z)


def _hold_stand(
    model: mj.MjModel,
    data: mj.MjData,
    targets: dict[str, float],
    com_z: float,
) -> None:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[2] = com_z
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    act_idx = {
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i): i
        for i in range(model.nu)
    }
    for joint_name, value in targets.items():
        joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise SystemExit(f"FAIL: stand joint missing: {joint_name}")
        data.qpos[model.jnt_qposadr[joint_id]] = value
    wg.set_ctrl(model, data, targets, act_idx)
    mj.mj_forward(model, data)
    steps = int(round(STAND_SETTLE_S / float(model.opt.timestep)))
    for _ in range(steps):
        mj.mj_step(model, data)


def _names(model: mj.MjModel, obj_type: int) -> list[str]:
    count = {
        mj.mjtObj.mjOBJ_BODY: model.nbody,
        mj.mjtObj.mjOBJ_JOINT: model.njnt,
        mj.mjtObj.mjOBJ_GEOM: model.ngeom,
    }[obj_type]
    return [mj.mj_id2name(model, obj_type, i) or "" for i in range(count)]


def _banned_hit(name: str) -> str | None:
    low = name.lower()
    for token in _BANNED_NAME_TOKENS:
        if token in low:
            return token
    return None


def _room_xml_problems(scene: RoomScene) -> list[str]:
    """The room file may include only the frozen plant, and no mechanism."""
    problems: list[str] = []
    text = scene.xml_path.read_text(encoding="utf-8")
    includes = _INCLUDE_RE.findall(text)
    if includes != [PLANT_INCLUDE]:
        problems.append(f"includes {includes} != [{PLANT_INCLUDE}]")
    if text.count("<include") != 1:
        problems.append(f"expected 1 include tag, found {text.count('<include')}")
    for tag in _BANNED_ROOM_TAGS:
        if tag in text:
            problems.append(f"room xml contains {tag}")
    return problems


def _freeze_problems(
    scene: RoomScene,
    model: mj.MjModel,
    plant: mj.MjModel,
) -> list[str]:
    problems: list[str] = []
    digest = _md5(PLANT_XML)
    if digest != steer_walk.PLANT_MD5:
        problems.append(f"plant md5 {digest} != {steer_walk.PLANT_MD5}")
    problems.extend(_room_xml_problems(scene))
    if model.ncam != 1:
        problems.append(f"expected 1 camera, found {model.ncam}")
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cam_id < 0:
        problems.append("missing camera kit_cam")
    else:
        parent = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, int(model.cam_bodyid[cam_id])) or ""
        if parent != "head_tilt_link":
            problems.append(f"kit_cam parent {parent} != head_tilt_link")
        pos = np.asarray(model.cam_pos[cam_id], dtype=np.float64)
        expected = np.array(steer_walk.KIT_CAM_POS, dtype=np.float64)
        if float(np.max(np.abs(pos - expected))) > 1e-6:
            problems.append(f"kit_cam pos {pos.tolist()} != {steer_walk.KIT_CAM_POS}")
        if abs(float(model.cam_fovy[cam_id]) - steer_walk.KIT_CAM_FOVY) > 1e-4:
            problems.append(f"kit_cam fovy {float(model.cam_fovy[cam_id])}")
        cam_mat = np.asarray(model.cam_mat0[cam_id], dtype=np.float64).reshape(3, 3)
        expected_mat = np.array(steer_walk.KIT_CAM_AXES, dtype=np.float64).T
        if float(np.max(np.abs(cam_mat - expected_mat))) > 1e-5:
            problems.append("kit_cam xyaxes != 0 -1 0 0 0 1")
    if _names(model, mj.mjtObj.mjOBJ_JOINT) != _names(plant, mj.mjtObj.mjOBJ_JOINT):
        problems.append("room joints differ from the frozen plant")
    if model.nu != plant.nu:
        problems.append(f"room actuators {model.nu} != plant {plant.nu}")
    for body_name in scene.bodies:
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name) < 0:
            problems.append(f"missing body {body_name}")
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, body_name) < 0:
            problems.append(f"missing site {body_name}")
    for _body_name, geom_name in scene.geoms:
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, geom_name) < 0:
            problems.append(f"missing geom {geom_name}")
    for kind, names in (
        ("body", _names(model, mj.mjtObj.mjOBJ_BODY)),
        ("geom", _names(model, mj.mjtObj.mjOBJ_GEOM)),
        ("joint", _names(model, mj.mjtObj.mjOBJ_JOINT)),
    ):
        for name in names:
            token = _banned_hit(name)
            if token is not None:
                problems.append(f"{kind} name {name!r} contains {token}")
    for geom_name in ("l_foot_contact", "r_foot_contact"):
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_id < 0:
            problems.append(f"missing geom {geom_name}")
            continue
        size = model.geom_size[geom_id]
        if (
            abs(float(size[0]) - steer_walk.FOOT_HALF_X) > 1e-6
            or abs(float(size[1]) - steer_walk.FOOT_HALF_Y) > 1e-6
        ):
            problems.append(f"{geom_name} size {size[:2].tolist()} != 145×86 half-size")
    leg_names = {
        "l_hip_yaw_pos", "l_hip_roll_pos", "l_hip_pitch_pos",
        "l_knee_pos", "l_ank_pitch_pos", "l_ank_roll_pos",
        "r_hip_yaw_pos", "r_hip_roll_pos", "r_hip_pitch_pos",
        "r_knee_pos", "r_ank_pitch_pos", "r_ank_roll_pos",
    }
    for i in range(model.nu):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if name not in leg_names:
            continue
        force_range = model.actuator_forcerange[i]
        if (
            abs(float(force_range[0]) + steer_walk.LEG_TAU) > 1e-6
            or abs(float(force_range[1]) - steer_walk.LEG_TAU) > 1e-6
        ):
            problems.append(f"{name} forcerange {force_range.tolist()} != ±{steer_walk.LEG_TAU}")
    return problems


def _render(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        renderer.update_scene(data, camera="kit_cam")
        frame = renderer.render()
    finally:
        renderer.close()
    image = np.asarray(frame, dtype=np.uint8)
    if image.shape != (HEIGHT, WIDTH, 3):
        raise SystemExit(f"FAIL: kit_cam frame shape {image.shape}")
    return image


def _changed_fraction(room_img: np.ndarray, empty_img: np.ndarray) -> float:
    diff = np.abs(room_img.astype(np.int16) - empty_img.astype(np.int16))
    changed = diff.max(axis=2) > EMPTY_DIFF
    return float(np.mean(changed))


def _project(
    point_world: np.ndarray,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fovy_deg: float,
) -> tuple[float, float] | None:
    """Pixel (u, v) if the point is in front of kit_cam. None if behind."""
    local = cam_mat.T @ (point_world - cam_pos)
    depth = -float(local[2])
    if depth <= 0.05:
        return None
    focal = (HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    u = (WIDTH / 2.0) + focal * (float(local[0]) / depth)
    v = (HEIGHT / 2.0) - focal * (float(local[1]) / depth)
    return u, v


def _geom_corners(model: mj.MjModel, data: mj.MjData, geom_id: int) -> np.ndarray:
    """World points that cover one geom. Mesh geoms use their vertices.

    A box-size reading of geom_size is the scale on a mesh, not its extent.
    Projecting that scale misses the furniture and fails the in-frame check.
    """
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    gtype = int(model.geom_type[geom_id])
    if gtype == int(mj.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id < 0:
            return np.zeros((0, 3), dtype=np.float64)
        vert_adr = int(model.mesh_vertadr[mesh_id])
        vert_num = int(model.mesh_vertnum[mesh_id])
        if vert_num <= 0:
            return np.zeros((0, 3), dtype=np.float64)
        step = max(1, vert_num // 800)
        verts = np.asarray(
            model.mesh_vert[vert_adr:vert_adr + vert_num:step],
            dtype=np.float64,
        )
        # mesh_vert is already in the geom frame. geom_size on a mesh is the
        # fitted half-extent, not a second scale, so it is not applied here.
        return origin + (rotation @ verts.T).T
    size = np.asarray(model.geom_size[geom_id, :3], dtype=np.float64)
    if gtype == int(mj.mjtGeom.mjGEOM_PLANE):
        hx = float(size[0])
        hy = float(size[1])
        corners = [
            origin + rotation @ np.array([sx, sy, 0.0], dtype=np.float64)
            for sx in (-hx, hx)
            for sy in (-hy, hy)
        ]
        return np.stack(corners, axis=0)
    corners: list[np.ndarray] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array([sx * size[0], sy * size[1], sz * size[2]], dtype=np.float64)
                corners.append(origin + rotation @ local)
    return np.stack(corners, axis=0)


def _body_box_pixels(
    model: mj.MjModel,
    data: mj.MjData,
    body_name: str,
) -> tuple[float, float]:
    """Width and height, in pixels, of the body's geoms inside the kit_cam image."""
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name)
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
    cam_mat = np.asarray(data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3)
    fovy = float(model.cam_fovy[cam_id])
    us: list[float] = []
    vs: list[float] = []
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) != body_id:
            continue
        for corner in _geom_corners(model, data, geom_id):
            pix = _project(corner, cam_pos, cam_mat, fovy)
            if pix is None:
                continue
            us.append(min(max(pix[0], 0.0), float(WIDTH)))
            vs.append(min(max(pix[1], 0.0), float(HEIGHT)))
    if len(us) < 2:
        return 0.0, 0.0
    return max(us) - min(us), max(vs) - min(vs)


def _save_png(image: np.ndarray, path: Path) -> None:
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(path)


def _render_scene(
    scene: RoomScene,
    plant: mj.MjModel,
    empty_img: np.ndarray,
    before: str,
    targets: dict[str, float],
    com_z: float,
) -> int:
    if not scene.xml_path.is_file():
        print(f"FAIL {scene.name}: missing {scene.xml_path}")
        return 1
    room = mj.MjModel.from_xml_path(str(scene.xml_path))
    problems = _freeze_problems(scene, room, plant)
    if _md5(PLANT_XML) != before:
        problems.append("plant file changed while loading the room")
    if problems:
        for msg in problems:
            print(f"FAIL {scene.name}: {msg}")
        return 1

    room_data = mj.MjData(room)
    _hold_stand(room, room_data, targets, com_z)
    room_img = _render(room, room_data)
    _save_png(room_img, scene.png_path)

    fraction = _changed_fraction(room_img, empty_img)
    print(
        f"{scene.name}  kit_cam {WIDTH}x{HEIGHT}  changed_frac={fraction:.3f}  "
        f"(empty-floor fail if < {EMPTY_MIN_FRAC:.2f})  "
        f"out={scene.png_path.relative_to(ROOT)}"
    )
    if fraction < EMPTY_MIN_FRAC:
        print(f"FAIL {scene.name}: kit_cam still is empty floor only")
        return 1

    for body_name in scene.bodies:
        box_w, box_h = _body_box_pixels(room, room_data, body_name)
        print(f"  {body_name} in frame {box_w:.0f}x{box_h:.0f} px")
        if box_w < MIN_BOX_PX or box_h < MIN_BOX_PX:
            print(f"FAIL {scene.name}: {body_name} is not visible in kit_cam")
            return 1

    cam_id = mj.mj_name2id(room, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    cam_world = np.asarray(room_data.cam_xpos[cam_id], dtype=np.float64)
    print(
        f"PASS  {scene.name}  named body {scene.room_body} in frame  "
        f"plant md5 {before}  kit_cam world "
        f"{cam_world[0]:+.3f} {cam_world[1]:+.3f} {cam_world[2]:+.3f}  "
        "vision scene only, not a map, no go-to, no arrival"
    )
    return 0


def _selected(argv: list[str]) -> list[str] | None:
    parser = argparse.ArgumentParser(
        description=(
            "Render a kit_cam still of a vision room. "
            "Not a map, not a planner, and not a go-to."
        )
    )
    parser.add_argument(
        "--room",
        choices=ROOM_ORDER,
        default=None,
        help="One room. Default is kitchen.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Kitchen plus bathroom, living, bedroom, and entrance.",
    )
    args = parser.parse_args(argv)
    if args.all and args.room is not None:
        print("FAIL: pass --room or --all, not both")
        return None
    if args.all:
        return list(ROOM_ORDER)
    if args.room is None:
        return ["kitchen"]
    return [args.room]


def main(argv: list[str] | None = None) -> int:
    selected = _selected(sys.argv[1:] if argv is None else argv)
    if selected is None:
        return 1
    if not PLANT_XML.is_file():
        print(f"FAIL: missing {PLANT_XML}")
        return 1
    before = _md5(PLANT_XML)
    if before != steer_walk.PLANT_MD5:
        print(f"FAIL: plant md5 {before} != {steer_walk.PLANT_MD5}")
        return 1

    plant = mj.MjModel.from_xml_path(str(PLANT_XML))
    targets, com_z = _stand_targets()
    plant_data = mj.MjData(plant)
    _hold_stand(plant, plant_data, targets, com_z)
    empty_img = _render(plant, plant_data)

    status = 0
    for name in selected:
        status |= _render_scene(ROOMS[name], plant, empty_img, before, targets, com_z)
    if _md5(PLANT_XML) != before:
        print("FAIL: plant file changed while rendering rooms")
        return 1
    return status


if __name__ == "__main__":
    raise SystemExit(main())
