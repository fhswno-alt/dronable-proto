#!/usr/bin/env python3
"""Stand the frozen plant in the kitchen room and save a kit_cam still.

The room XML includes the walk plant. This script does not edit that file,
the gait, the tip check, or CommandBus. It does not build a map.

Exit status is 1 when the still is still the empty checkerboard, or when
the kitchen / table / chair boxes do not land in the kit_cam frame.
"""
from __future__ import annotations

import hashlib
import math
import os
import sys
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
ROOM_XML = ROOT / "mujoco" / "room_kitchen.xml"
PLANT_XML = steer_walk.PLANT_XML
OUT_PNG = ROOT / "previews" / "kit_cam_room.png"

WIDTH = 640
HEIGHT = 480
STAND_SETTLE_S = 0.60
# Pixels whose max channel differs from the empty-plant kit_cam frame.
# Furniture behind the camera moves ~0 of the image. The room layout moves
# well above this. A soft floor shadow does not.
EMPTY_DIFF = 40
EMPTY_MIN_FRAC = 0.08
MIN_BOX_PX = 36.0

FURNITURE_BODIES = ("kitchen", "table", "chair")
FURNITURE_GEOMS = {
    "kitchen": "kitchen_cabinet",
    "table": "table_top",
    "chair": "chair_seat",
}


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


def _freeze_problems(model: mj.MjModel) -> list[str]:
    problems: list[str] = []
    digest = _md5(PLANT_XML)
    if digest != steer_walk.PLANT_MD5:
        problems.append(f"plant md5 {digest} != {steer_walk.PLANT_MD5}")
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
    for body_name in FURNITURE_BODIES:
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name) < 0:
            problems.append(f"missing body {body_name}")
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, body_name) < 0:
            problems.append(f"missing site {body_name}")
    for geom_name in FURNITURE_GEOMS.values():
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, geom_name) < 0:
            problems.append(f"missing geom {geom_name}")
    for i in range(model.nbody):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) or ""
        low = name.lower()
        if "door" in low or "lever" in low:
            problems.append(f"door/lever body present: {name}")
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
    size = np.asarray(model.geom_size[geom_id, :3], dtype=np.float64)
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
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


def main() -> int:
    if not ROOM_XML.is_file():
        print(f"FAIL: missing {ROOM_XML}")
        return 1
    if not PLANT_XML.is_file():
        print(f"FAIL: missing {PLANT_XML}")
        return 1
    before = _md5(PLANT_XML)
    room = mj.MjModel.from_xml_path(str(ROOM_XML))
    plant = mj.MjModel.from_xml_path(str(PLANT_XML))
    problems = _freeze_problems(room)
    if _md5(PLANT_XML) != before:
        problems.append("plant file changed while loading the room")
    if problems:
        for msg in problems:
            print(f"FAIL: {msg}")
        return 1

    targets, com_z = _stand_targets()
    room_data = mj.MjData(room)
    plant_data = mj.MjData(plant)
    _hold_stand(room, room_data, targets, com_z)
    _hold_stand(plant, plant_data, targets, com_z)

    room_img = _render(room, room_data)
    empty_img = _render(plant, plant_data)
    _save_png(room_img, OUT_PNG)

    fraction = _changed_fraction(room_img, empty_img)
    print(
        f"kit_cam {WIDTH}x{HEIGHT}  changed_frac={fraction:.3f}  "
        f"(empty-floor fail if < {EMPTY_MIN_FRAC:.2f})  out={OUT_PNG.relative_to(ROOT)}"
    )
    if fraction < EMPTY_MIN_FRAC:
        print("FAIL: kit_cam still is empty floor only")
        return 1

    for body_name in FURNITURE_BODIES:
        box_w, box_h = _body_box_pixels(room, room_data, body_name)
        print(f"  {body_name} in frame {box_w:.0f}x{box_h:.0f} px")
        if box_w < MIN_BOX_PX or box_h < MIN_BOX_PX:
            print(f"FAIL: {body_name} is not visible in kit_cam")
            return 1

    cam_id = mj.mj_name2id(room, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    cam_world = np.asarray(room_data.cam_xpos[cam_id], dtype=np.float64)
    print(
        f"PASS  plant md5 {before}  kit_cam world "
        f"{cam_world[0]:+.3f} {cam_world[1]:+.3f} {cam_world[2]:+.3f}  "
        "room is a vision scene, not a navigation map"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
