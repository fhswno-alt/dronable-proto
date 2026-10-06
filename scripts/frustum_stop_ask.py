#!/usr/bin/env python3
"""Prefer FAIL: are bed, toilet, and stove in kit_cam at the ask stand?

Read-only. Loads a vision scene, holds the kit-gait stand for 1.0 s
(the living-collapse ASK_STOP crouch), and counts kit_cam pixels per
prop. It does not edit the plant, CommandBus, or the gait.

Classes are frozen before the pixel counts:

- missing: the prop's geoms are not in the scene
- out_of_frustum: zero visible pixels, and neither the geom center nor
  an AABB corner projects inside the image
- occluded: zero visible pixels, but some of that geometry projects
  inside the image
- tiny: some pixels, under 1% of 640×480 or a mask shorter than 36 px
- in_frame_boxy: at least that size, and every visible geom is a box
- visible_enough: at least that size, and a mesh geom is among them

``--old-root`` points at a directory of the main-freeze room XML (the
scenes behind the four kit stills SmolVLM named correctly). Those files
are not in this branch. The include still loads the current plant.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TypedDict

os.environ.setdefault("MUJOCO_GL", "osmesa")

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import mujoco as mj
import numpy as np
from PIL import Image

import explore_map as em
import steer_walk as sw

PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
WIDTH = 640
HEIGHT = 480
FRAME_PX = WIDTH * HEIGHT
TINY_FRAC = 0.01
TINY_MIN_SIDE_PX = 36
TINY_PX = int(TINY_FRAC * FRAME_PX)

# Geom names that make one room-defining prop. Frozen with the classes.
PROP_GEOMS: dict[str, tuple[str, ...]] = {
    "stove": ("kitchen_stove",),
    "toilet": ("toilet_bowl", "toilet_tank", "toilet_seat", "toilet_handle"),
    "bed": (
        "bedroom_headboard",
        "bedroom_duvet",
        "bedroom_pillow",
        "bedroom_pillow_r",
        "bedroom_mattress",
        "bedroom_bed_frame",
    ),
    "sink": ("sink_basin", "sink_pedestal", "sink_faucet", "kitchen_sink"),
    "bathtub": ("bathtub_shell", "bathtub_water"),
    "sofa": (
        "living_sofa_seat",
        "living_sofa_back",
        "living_sofa_arm_l",
        "living_sofa_arm_r",
    ),
    "tv": ("tv_stand", "tv_bezel", "tv_screen"),
    "dresser": ("dresser_case",),
}

CURRENT_SCENES: tuple[tuple[str, Path], ...] = (
    ("kitchen", em.SCENES["kitchen"]),
    ("bathroom", em.SCENES["bathroom"]),
    ("bedroom", em.SCENES["bedroom"]),
    ("living", em.SCENES["living"]),
)

DEFINING: tuple[str, ...] = ("stove", "toilet", "bed")


class GeomRow(TypedDict):
    name: str
    type: str
    pixels: int
    min_side_px: int
    center_in_frame: bool


class PropRow(TypedDict):
    prop: str
    class_name: str
    present: bool
    geom_type: str
    pixels: int
    frac: float
    min_side_px: int
    bbox: list[int] | None
    center_xyz: list[float] | None
    bearing_deg: float | None
    yaw_offset_deg: float | None
    geoms: list[str]


class SceneRow(TypedDict):
    scene: str
    xml: str
    png: str
    bus_mode: str
    stand_s: float
    stop_xy_m: list[float]
    stop_yaw_deg: float
    cam_xyz_m: list[float]
    cam_z_m: float
    pitch_deg: float
    look_yaw_deg: float
    fovy_deg: float
    props: list[PropRow]
    scene_geoms: list[GeomRow]


_PLANT_GEOM_NAMES: set[str] | None = None


def _plant_geom_names() -> set[str]:
    global _PLANT_GEOM_NAMES
    if _PLANT_GEOM_NAMES is None:
        model = mj.MjModel.from_xml_path(str(sw.PLANT_XML))
        names = {
            mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or ""
            for gid in range(model.ngeom)
        }
        _PLANT_GEOM_NAMES = {name for name in names if name}
    return _PLANT_GEOM_NAMES


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _project(
    point: np.ndarray,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fovy_deg: float,
) -> tuple[float, float, bool]:
    """Pixel (u, v) and whether it lands inside the image, in front."""
    local = cam_mat.T @ (point - cam_pos)
    depth = -float(local[2])
    if depth <= 1e-4:
        return -1.0, -1.0, False
    half_h = math.tan(math.radians(fovy_deg) * 0.5)
    half_w = half_h * (WIDTH / HEIGHT)
    x_ndc = float(local[0]) / depth / half_w
    y_ndc = float(local[1]) / depth / half_h
    u = (x_ndc + 1.0) * 0.5 * WIDTH
    v = (1.0 - y_ndc) * 0.5 * HEIGHT
    inside = 0.0 <= u < WIDTH and 0.0 <= v < HEIGHT
    return u, v, inside


def _geom_world_points(model: mj.MjModel, data: mj.MjData, gid: int) -> np.ndarray:
    """Corners (box) or mesh vertices, in world meters."""
    origin = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
    gtype = int(model.geom_type[gid])
    if gtype == int(mj.mjtGeom.mjGEOM_BOX):
        hx, hy, hz = (float(v) for v in model.geom_size[gid])
        local = np.array(
            [
                [sx * hx, sy * hy, sz * hz]
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
                for sz in (-1.0, 1.0)
            ],
            dtype=np.float64,
        )
        return origin + (rot @ local.T).T
    if gtype == int(mj.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[gid])
        adr = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        step = max(1, count // 4000)
        verts = np.asarray(model.mesh_vert[adr:adr + count:step], dtype=np.float64)
        scale = np.asarray(model.geom_size[gid], dtype=np.float64)
        local = verts * scale
        return origin + (rot @ local.T).T
    radius = float(model.geom_rbound[gid])
    offsets = np.array(
        [
            [sx * radius, sy * radius, sz * radius]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    return origin + offsets


def _mask_stats(mask: np.ndarray) -> tuple[int, int, list[int] | None]:
    count = int(np.count_nonzero(mask))
    if count == 0:
        return 0, 0, None
    ys, xs = np.nonzero(mask)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    min_side = min(bbox[2] - bbox[0] + 1, bbox[3] - bbox[1] + 1)
    return count, int(min_side), bbox


def _classify(present: bool, pixels: int, min_side: int, mesh: bool, in_frame: bool) -> str:
    if not present:
        return "missing"
    if pixels <= 0:
        return "occluded" if in_frame else "out_of_frustum"
    tiny = pixels < TINY_PX or min_side < TINY_MIN_SIDE_PX
    if tiny:
        return "tiny"
    if mesh:
        return "visible_enough"
    return "in_frame_boxy"


def _xml_inventory(path: Path) -> list[dict[str, str]]:
    root = ET.parse(path).getroot()
    rows: list[dict[str, str]] = []
    for body in root.iter("body"):
        body_name = body.get("name") or ""
        body_pos = body.get("pos") or "0 0 0"
        for geom in list(body):
            if geom.tag != "geom":
                continue
            rows.append({
                "body": body_name,
                "body_pos": body_pos,
                "geom": geom.get("name") or "",
                "type": geom.get("type") or "sphere",
                "mesh": geom.get("mesh") or "",
                "pos": geom.get("pos") or "0 0 0",
                "size": geom.get("size") or "",
            })
    shell = [
        geom.get("name") or ""
        for geom in root.iter("geom")
        if (geom.get("name") or "").startswith("room_")
    ]
    rows.append({
        "body": "",
        "body_pos": "",
        "geom": ",".join(shell),
        "type": "shell",
        "mesh": "",
        "pos": "",
        "size": str(len(shell)),
    })
    return rows


def _measure_scene(
    scene: str,
    xml_path: Path,
    png_path: Path,
    source: str | None = None,
) -> SceneRow:
    session = sw.SteerSession(
        video=False,
        scene_xml=xml_path,
        lipm=sw.locked_kit_config(),
    )
    sent: list[em.SentCommand] = []
    em._hold_stand(session, em.STAND_S, sent)
    model = session.model
    data = session.data
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
    cam_mat = np.asarray(data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3)
    look = -cam_mat[:, 2]
    pitch = math.degrees(math.atan2(float(look[2]), math.hypot(float(look[0]), float(look[1]))))
    look_yaw = math.degrees(math.atan2(float(look[1]), float(look[0])))
    robot_yaw = math.degrees(session.yaw())
    stop_x = float(data.qpos[0])
    stop_y = float(data.qpos[1])

    renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        renderer.update_scene(data, camera="kit_cam")
        rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera="kit_cam")
        seg = np.asarray(renderer.render()).copy()
    finally:
        renderer.close()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb).save(png_path)

    # MuJoCo seg image: channel 0 is the object id, channel 1 is mjOBJ_*.
    geom_id = seg[:, :, 0]
    geom_type = seg[:, :, 1]
    geom_mask = geom_type == int(mj.mjtObj.mjOBJ_GEOM)
    fovy = float(model.cam_fovy[cam_id])

    name_to_id: dict[str, int] = {}
    for gid in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or ""
        if name:
            name_to_id[name] = gid

    def one_geom(name: str) -> GeomRow | None:
        gid = name_to_id.get(name)
        if gid is None:
            return None
        mask = geom_mask & (geom_id == gid)
        pixels, min_side, _bbox = _mask_stats(mask)
        points = _geom_world_points(model, data, gid)
        center = np.asarray(data.geom_xpos[gid], dtype=np.float64)
        in_frame = _project(center, cam_pos, cam_mat, fovy)[2]
        if not in_frame:
            for corner in points:
                if _project(corner, cam_pos, cam_mat, fovy)[2]:
                    in_frame = True
                    break
        gtype = mj.mjtGeom(int(model.geom_type[gid])).name.replace("mjGEOM_", "").lower()
        return {
            "name": name,
            "type": gtype,
            "pixels": pixels,
            "min_side_px": min_side,
            "center_in_frame": in_frame,
        }

    plant_names = _plant_geom_names()
    scene_geoms: list[GeomRow] = []
    for name, gid in sorted(name_to_id.items(), key=lambda item: item[1]):
        if name in plant_names:
            continue
        row = one_geom(name)
        if row is not None:
            scene_geoms.append(row)

    props: list[PropRow] = []
    for prop, names in PROP_GEOMS.items():
        found = [row for name in names if (row := one_geom(name)) is not None]
        present = bool(found)
        pixels = sum(row["pixels"] for row in found)
        types = {row["type"] for row in found}
        mesh = "mesh" in types
        if not found:
            min_side = 0
            bbox = None
            in_frame = False
            center = None
        else:
            gids = [name_to_id[row["name"]] for row in found]
            mask = geom_mask & np.isin(geom_id, np.array(gids, dtype=geom_id.dtype))
            _pixels, min_side, bbox = _mask_stats(mask)
            pixels = _pixels
            in_frame = any(row["center_in_frame"] for row in found)
            centers = [np.asarray(data.geom_xpos[gid], dtype=np.float64) for gid in gids]
            center = np.mean(np.stack(centers, axis=0), axis=0)
        bearing: float | None = None
        offset: float | None = None
        center_xyz: list[float] | None = None
        if center is not None:
            center_xyz = [float(v) for v in center]
            bearing = math.degrees(math.atan2(float(center[1]) - stop_y, float(center[0]) - stop_x))
            offset = _wrap_deg(bearing - robot_yaw)
        geom_type = "missing" if not found else ("mesh" if mesh else "box" if types == {"box"} else "+".join(sorted(types)))
        props.append({
            "prop": prop,
            "class_name": _classify(present, pixels, min_side, mesh, in_frame),
            "present": present,
            "geom_type": geom_type,
            "pixels": pixels,
            "frac": pixels / FRAME_PX,
            "min_side_px": min_side,
            "bbox": bbox,
            "center_xyz": center_xyz,
            "bearing_deg": bearing,
            "yaw_offset_deg": offset,
            "geoms": [row["name"] for row in found],
        })

    session.assert_plant_unchanged()
    del session
    return {
        "scene": scene,
        "xml": source if source is not None else str(xml_path.relative_to(sw.ROOT)),
        "png": str(png_path.relative_to(sw.ROOT)),
        "bus_mode": "stand",
        "stand_s": em.STAND_S,
        "stop_xy_m": [stop_x, stop_y],
        "stop_yaw_deg": robot_yaw,
        "cam_xyz_m": [float(v) for v in cam_pos],
        "cam_z_m": float(cam_pos[2]),
        "pitch_deg": pitch,
        "look_yaw_deg": look_yaw,
        "fovy_deg": fovy,
        "props": props,
        "scene_geoms": scene_geoms,
    }


def _diff_named(current: list[dict[str, str]], old: list[dict[str, str]]) -> dict[str, object]:
    def names(rows: list[dict[str, str]]) -> set[str]:
        return {row["geom"] for row in rows if row["type"] != "shell" and row["geom"]}

    cur_names = names(current)
    old_names = names(old)
    return {
        "only_old": sorted(old_names - cur_names),
        "only_current": sorted(cur_names - old_names),
        "shared": sorted(cur_names & old_names),
    }


def _prop(row: SceneRow, name: str) -> PropRow:
    return next(item for item in row["props"] if item["prop"] == name)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--old-root", type=Path, default=None)
    parser.add_argument(
        "--out",
        type=Path,
        default=sw.ROOT / "previews" / "frustum_stop_ask",
    )
    args = parser.parse_args()
    digest = _md5(sw.PLANT_XML)
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest}")
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    current_rows: list[SceneRow] = []
    for scene, xml_path in CURRENT_SCENES:
        row = _measure_scene(scene, xml_path, out / f"current_{scene}.png")
        current_rows.append(row)
        stove = _prop(row, "stove")
        toilet = _prop(row, "toilet")
        bed = _prop(row, "bed")
        print(
            f"[current] {scene} z={row['cam_z_m']:.3f} pitch={row['pitch_deg']:+.2f} "
            f"xy=({row['stop_xy_m'][0]:+.3f},{row['stop_xy_m'][1]:+.3f}) "
            f"yaw={row['stop_yaw_deg']:+.2f} "
            f"stove={stove['class_name']}:{stove['pixels']} "
            f"toilet={toilet['class_name']}:{toilet['pixels']} "
            f"bed={bed['class_name']}:{bed['pixels']}",
            flush=True,
        )

    old_rows: list[SceneRow] = []
    xml_diff: dict[str, object] = {}
    old_root: Path | None = args.old_root
    if old_root is not None:
        for scene, _current in CURRENT_SCENES:
            old_xml = old_root / f"room_{scene}.xml"
            if not old_xml.is_file():
                raise SystemExit(f"missing old scene {old_xml}")
            row = _measure_scene(
                scene,
                old_xml,
                out / f"oldmesh_{scene}.png",
                source=f"bb94512:mujoco/room_{scene}.xml",
            )
            old_rows.append(row)
            defining = {
                item["prop"]: f"{item['class_name']}:{item['pixels']}"
                for item in row["props"]
                if item["prop"] in DEFINING or item["present"]
            }
            print(
                f"[oldmesh] {scene} z={row['cam_z_m']:.3f} pitch={row['pitch_deg']:+.2f} "
                f"{defining}",
                flush=True,
            )
            xml_diff[scene] = _diff_named(
                _xml_inventory(em.SCENES[scene]),
                _xml_inventory(old_xml),
            )

    payload = {
        "plant_md5": digest,
        "stand_s": em.STAND_S,
        "width": WIDTH,
        "height": HEIGHT,
        "tiny_frac": TINY_FRAC,
        "tiny_min_side_px": TINY_MIN_SIDE_PX,
        "tiny_px": TINY_PX,
        "classes": [
            "missing",
            "out_of_frustum",
            "occluded",
            "tiny",
            "in_frame_boxy",
            "visible_enough",
        ],
        "old_scenes": "bb94512" if old_root is not None else None,
        "current": current_rows,
        "oldmesh": old_rows,
        "xml_diff": xml_diff,
    }
    dest = out / "summary.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n")
    if _md5(sw.PLANT_XML) != digest:
        raise SystemExit("plant md5 changed")
    print(f"wrote {dest}", flush=True)


if __name__ == "__main__":
    main()
