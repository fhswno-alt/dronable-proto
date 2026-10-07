#!/usr/bin/env python3
"""Exploded assembly animation of the frozen AiNex walk plant.

Loads mujoco/ainex_hiwonder/ainex_controls_m2_145.xml, checks its md5, and
renders an offscreen build: every kinematic group starts pushed outward,
then seats one group at a time. Body offsets are applied only on the loaded
MjModel / MjData. The plant file, its meshes, and the controller are not
written.

The free joint on body_link is posed with MjData.qpos. Child bodies move
with MjModel.body_pos. Foot geoms move with MjModel.geom_pos.

The 145x86 boxes in the plant are sim contact pads. When the frozen outsole
STL compiles as a visual geom, the render shows that plate and hides the pad
rgba in memory only. The plant file is never written.
"""
from __future__ import annotations

import hashlib
import math
import os
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

Vec3 = npt.NDArray[np.float64]

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
PRICE_SHEET = ROOT / "docs" / "MFG_FIRST_BUILD_PRICE_SHEET.md"
OUT_DIR = ROOT / "docs" / "previews" / "assembly"

EXPECTED_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"

WIDTH = 1280
HEIGHT = 720
FPS = 30
INTRO_S = 2.00  # max-explode hold; must stay at least 1.5 s
MOVE_S = 1.25
SETTLE_S = 0.70
ORBIT_S = 4.50
ORBIT_DEG = 70.0
TITLE_S = 2.50

# 3/4 front. Distance is set so the assembled body is about 60% of the frame height.
LOOKAT = np.array([0.02, 0.0, 0.24], dtype=np.float64)
DISTANCE = 1.02
AZIMUTH0 = 148.0
ELEVATION = -14.0

# World-frame offsets from the assembled pose. Plant +Y is the left side.
# Lateral offsets are a starting point: the explode camera stays at or inside
# 1.8 m, and |y| is reduced first if a part would clip. Vertical gaps stay:
# the head is above the torso, the outsoles are below the ankles.
EXPLODE_OFFSET_M: dict[str, Vec3] = {
    "torso": np.array([0.00, 0.00, 0.16], dtype=np.float64),
    "hips_l": np.array([0.00, 0.20, -0.20], dtype=np.float64),
    "hips_r": np.array([0.00, -0.20, -0.20], dtype=np.float64),
    "left_leg": np.array([0.02, 0.36, -0.08], dtype=np.float64),
    "right_leg": np.array([0.02, -0.36, -0.08], dtype=np.float64),
    "foot_l": np.array([0.00, 0.14, -0.32], dtype=np.float64),
    "foot_r": np.array([0.00, -0.14, -0.32], dtype=np.float64),
    "arm_l": np.array([0.00, 0.28, 0.12], dtype=np.float64),
    "arm_r": np.array([0.00, -0.28, 0.12], dtype=np.float64),
    "head": np.array([0.00, 0.00, 0.32], dtype=np.float64),
}
MAX_EXPLODE_DISTANCE = 1.80
MIN_EXPLODE_BBOX_HEIGHT = 0.55
MIN_EXPLODE_BBOX_WIDTH = 0.40
MIN_GROUP_CENTROID_PX = 80.0
MIN_GROUP_AABB_GAP_PX = 40.0
KIT_CAM_LABEL_PX = 40.0

GROUP_ORDER: tuple[str, ...] = (
    "torso",
    "hips",
    "left_leg",
    "right_leg",
    "feet",
    "arms",
    "head",
)

GROUP_TITLE: dict[str, str] = {
    "torso": "Torso",
    "hips": "Hips",
    "left_leg": "Left leg",
    "right_leg": "Right leg",
    "feet": "Feet",
    "arms": "Arms",
    "head": "Head",
}

FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
MAX_MP4_BYTES = 5 * 1024 * 1024
MAX_GIF_BYTES = 8 * 1024 * 1024
TITLE_CARD = "Dronable proto: AiNex 24-DOF body, frozen sim plant, 145x86 printed feet"
OUTSOLE_STL = ROOT / "cad" / "m2_outsole" / "M2_outsole_145x86_meshAABB_notOEM.stl"
OUTSOLE_CAPTION = "Printed outsole (frozen CAD, cad/m2_outsole)"
# Pocket floor sits 0.8 mm above the plate bottom (docs/ANK_ROLL_SLEEVE_FIT.md).
POCKET_FLOOR_ABOVE_BOTTOM_M = 0.0008
OUTSOLE_RGBA: tuple[float, float, float, float] = (0.32, 0.33, 0.35, 1.0)


def _bootstrap_gl() -> str:
    """Pick egl, then osmesa, and re-exec so MuJoCo imports on that backend."""
    locked = os.environ.get("ASSEMBLY_GL_LOCKED", "")
    if locked in {"egl", "osmesa"}:
        os.environ["MUJOCO_GL"] = locked
        return locked
    probe = (
        "import mujoco as mj\n"
        "m = mj.MjModel.from_xml_string("
        "'<mujoco><worldbody><geom type=\"sphere\" size=\"0.1\"/></worldbody></mujoco>')\n"
        "d = mj.MjData(m)\n"
        "r = mj.Renderer(m, height=64, width=64)\n"
        "r.update_scene(d)\n"
        "r.render()\n"
        "r.close()\n"
    )
    for backend in ("egl", "osmesa"):
        env = os.environ.copy()
        env["MUJOCO_GL"] = backend
        env["ASSEMBLY_GL_LOCKED"] = backend
        result = subprocess.run(
            [sys.executable, "-c", probe],
            cwd="/tmp",
            env=env,
            capture_output=True,
            check=False,
        )
        if result.returncode == 0:
            os.environ["MUJOCO_GL"] = backend
            os.environ["ASSEMBLY_GL_LOCKED"] = backend
            os.execv(sys.executable, [sys.executable, *sys.argv])
        print(
            f"MUJOCO_GL={backend} unavailable ({result.returncode})",
            file=sys.stderr,
        )
    raise SystemExit(
        "FAIL: offscreen rendering unavailable. Tried MUJOCO_GL=egl, then osmesa."
    )


if __name__ == "__main__":
    _bootstrap_gl()

import mujoco as mj  # noqa: E402


@dataclass(frozen=True)
class AssemblyGroup:
    key: str
    title: str
    body_ids: tuple[int, ...]
    geom_ids: tuple[int, ...]
    servo_count: int
    subtitle: str


@dataclass(frozen=True)
class FeetVisual:
    subtitle: str
    geom_names: tuple[str, ...]
    printed_outsole: bool


@dataclass(frozen=True)
class FrameCue:
    phase: str
    group_key: str | None
    ease: float
    spin_deg: float


def plant_md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def assert_plant_md5(path: Path, when: str) -> str:
    if not path.is_file():
        raise SystemExit(f"FAIL: plant file missing at {when}: {path}")
    digest = plant_md5(path)
    if digest != EXPECTED_MD5:
        raise SystemExit(
            f"FAIL: plant md5 at {when} is {digest}, expected {EXPECTED_MD5}. "
            f"Refusing to continue. File: {path}"
        )
    return digest


def smootherstep(t: float) -> float:
    clamped = min(1.0, max(0.0, t))
    return clamped * clamped * clamped * (clamped * (clamped * 6.0 - 15.0) + 10.0)


def _body_name(model: mj.MjModel, body_id: int) -> str:
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id)
    if not name:
        raise SystemExit(f"FAIL: body {body_id} has no name")
    return name


def _geom_name(model: mj.MjModel, geom_id: int) -> str:
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id)
    if not name:
        raise SystemExit(f"FAIL: geom {geom_id} has no name")
    return name


def classify_body(name: str) -> str:
    """Map a plant body name onto one assembly group. Unknown names fail."""
    if name == "body_link":
        return "torso"
    if name.startswith("head_"):
        return "head"
    if "_hip_yaw_" in name or "_hip_roll_" in name:
        return "hips"
    if name.startswith("l_") and (
        "_hip_pitch_" in name or "_knee_" in name or "_ank_" in name
    ):
        return "left_leg"
    if name.startswith("r_") and (
        "_hip_pitch_" in name or "_knee_" in name or "_ank_" in name
    ):
        return "right_leg"
    if "_sho_" in name or "_el_" in name or "_gripper_" in name:
        return "arms"
    raise SystemExit(f"FAIL: body {name} is not in the kinematic assembly grouping")


def _servo_counts(model: mj.MjModel, bodies_by_group: dict[str, list[int]]) -> dict[str, int]:
    counts = {key: 0 for key in GROUP_ORDER}
    seen_joints: set[int] = set()
    for actuator_id in range(model.nu):
        trn_type = int(model.actuator_trntype[actuator_id])
        if trn_type != int(mj.mjtTrn.mjTRN_JOINT):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            raise SystemExit(f"FAIL: actuator {name} is not a joint servo")
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id in seen_joints:
            raise SystemExit(f"FAIL: joint {joint_id} has more than one actuator")
        seen_joints.add(joint_id)
        if int(model.jnt_type[joint_id]) != int(mj.mjtJoint.mjJNT_HINGE):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            raise SystemExit(f"FAIL: actuator {name} does not drive a hinge")
        body_id = int(model.jnt_bodyid[joint_id])
        body_name = _body_name(model, body_id)
        group_key = classify_body(body_name)
        counts[group_key] += 1
    if sum(counts.values()) != model.nu:
        raise SystemExit(
            f"FAIL: servo counts {counts} do not sum to nu={model.nu}"
        )
    return counts


def _measured_pad_mm(model: mj.MjModel) -> tuple[int, int]:
    """Contact-box planform from geom_size. Both feet must be the frozen 145x86."""
    measured: list[tuple[int, int]] = []
    for name in ("l_foot_contact", "r_foot_contact"):
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise SystemExit(f"FAIL: missing foot pad geom {name}")
        size = model.geom_size[geom_id]
        length_mm = int(round(float(size[0]) * 2000.0))
        width_mm = int(round(float(size[1]) * 2000.0))
        measured.append((length_mm, width_mm))
    if measured[0] != measured[1]:
        raise SystemExit(f"FAIL: left/right foot pads differ: {measured}")
    length_mm, width_mm = measured[0]
    if (length_mm, width_mm) != (145, 86):
        raise SystemExit(
            f"FAIL: foot contact pads measure {length_mm}x{width_mm} mm, expected 145x86"
        )
    return length_mm, width_mm


def _sim_pad_visual(model: mj.MjModel) -> FeetVisual:
    length_mm, width_mm = _measured_pad_mm(model)
    return FeetVisual(
        subtitle=f"Foot contact pad {length_mm}x{width_mm} mm (sim)",
        geom_names=("l_foot_contact", "r_foot_contact"),
        printed_outsole=False,
    )


def _read_stl_triangles(path: Path) -> npt.NDArray[np.float64]:
    """Binary STL as (triangle, corner, xyz) in file units."""
    blob = path.read_bytes()
    if len(blob) < 84:
        raise ValueError(f"{path.name} is too small to be a binary STL")
    triangle_count = int(struct.unpack_from("<I", blob, 80)[0])
    expected = 84 + triangle_count * 50
    if triangle_count < 1 or len(blob) < expected:
        raise ValueError(f"{path.name} triangle count {triangle_count} does not fit the file")
    triangles = np.empty((triangle_count, 3, 3), dtype=np.float64)
    offset = 84
    for index in range(triangle_count):
        values = struct.unpack_from("<12fH", blob, offset)
        for corner in range(3):
            base = 3 + 3 * corner
            triangles[index, corner, 0] = float(values[base])
            triangles[index, corner, 1] = float(values[base + 1])
            triangles[index, corner, 2] = float(values[base + 2])
        offset += 50
    if not bool(np.isfinite(triangles).all()):
        raise ValueError(f"{path.name} contains non-finite vertices")
    return triangles


def _outsole_vertices_m(path: Path) -> tuple[npt.NDArray[np.float64], float]:
    """Plate vertices in metres, centred on the outer AABB, and pocket-floor Z.

    File axes are length X, width Y, thickness Z. The geom position is the
    plate centre. MuJoCo may store the vertices in its inertia frame; the
    geom quaternion puts that frame back on these axes.
    """
    triangles = _read_stl_triangles(path)
    flat = triangles.reshape(-1, 3)
    low = flat.min(axis=0)
    high = flat.max(axis=0)
    span = high - low
    longest = float(span.max())
    if longest > 10.0:
        scale = 0.001
    elif longest > 0.05:
        scale = 1.0
    else:
        raise ValueError(f"outsole span {span.tolist()} is neither millimetres nor metres")
    span_m = span * scale
    expected = np.array([0.145, 0.086, 0.003], dtype=np.float64)
    if not np.allclose(span_m, expected, atol=0.0015):
        raise ValueError(f"outsole span {span_m.tolist()} m is not 145x86x3 mm")
    center = (low + high) * 0.5
    centered = (flat - center) * scale
    bottom_m = float(low[2]) * scale
    floor_m = bottom_m + POCKET_FLOOR_ABOVE_BOTTOM_M
    floor_z = floor_m - float(center[2]) * scale
    return centered, floor_z


def _mesh_geom_aabb(model: mj.MjModel, geom_id: int) -> tuple[Vec3, Vec3]:
    """Axis-aligned bounds of a mesh geom in its body frame."""
    mesh_id = int(model.geom_dataid[geom_id])
    if mesh_id < 0:
        raise ValueError(f"geom {geom_id} is not a mesh")
    address = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    vertices = np.array(model.mesh_vert[address : address + count], dtype=np.float64)
    quat = np.array(model.geom_quat[geom_id], dtype=np.float64)
    matrix = np.zeros(9, dtype=np.float64)
    mj.mju_quat2Mat(matrix, quat)
    posed = (matrix.reshape(3, 3) @ vertices.T).T
    posed += np.array(model.geom_pos[geom_id], dtype=np.float64)
    low = posed.min(axis=0)
    high = posed.max(axis=0)
    return low, high


def _foot_planform(model: mj.MjModel, side: str) -> tuple[float, float, float]:
    geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{side}_ank_roll_link_mesh")
    if geom_id < 0:
        raise ValueError(f"missing {side} ankle mesh")
    low, high = _mesh_geom_aabb(model, geom_id)
    center = (low + high) * 0.5
    return float(center[0]), float(center[1]), float(low[2])


def _outsole_sits_on_foot(model: mj.MjModel, side: str) -> None:
    """Reject a plate that is not a flat 145x86 sleeve on the ankle mesh."""
    foot_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{side}_ank_roll_link_mesh")
    plate_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{side}_printed_outsole")
    if foot_id < 0 or plate_id < 0:
        raise ValueError(f"missing {side} foot mesh or printed outsole")
    foot_low, foot_high = _mesh_geom_aabb(model, foot_id)
    plate_low, plate_high = _mesh_geom_aabb(model, plate_id)
    plate_span = plate_high - plate_low
    expected_span = np.array([0.145, 0.086, 0.003], dtype=np.float64)
    if not np.allclose(plate_span, expected_span, atol=0.002):
        raise ValueError(f"{side} outsole span {plate_span.tolist()} is not a flat 145x86 plate")
    foot_center = (foot_low + foot_high) * 0.5
    plate_center = (plate_low + plate_high) * 0.5
    if abs(float(plate_center[0] - foot_center[0])) > 0.002:
        raise ValueError(f"{side} outsole is off the foot in X")
    if abs(float(plate_center[1] - foot_center[1])) > 0.002:
        raise ValueError(f"{side} outsole is off the foot in Y")
    floor_z = float(plate_low[2]) + POCKET_FLOOR_ABOVE_BOTTOM_M
    gap_m = floor_z - float(foot_low[2])
    if abs(gap_m) > 0.0015:
        raise ValueError(f"{side} pocket floor is {gap_m * 1000.0:.2f} mm off the foot underside")


def attach_printed_outsoles(plant_xml: Path) -> tuple[mj.MjModel, FeetVisual]:
    """Compile a render model with visual-only outsole plates. Does not write XML."""
    if not OUTSOLE_STL.is_file():
        raise ValueError(f"missing {OUTSOLE_STL}")
    vertices, floor_z = _outsole_vertices_m(OUTSOLE_STL)
    probe = mj.MjModel.from_xml_path(str(plant_xml))
    positions: dict[str, tuple[float, float, float]] = {}
    for side in ("l", "r"):
        center_x, center_y, underside = _foot_planform(probe, side)
        positions[side] = (center_x, center_y, underside - floor_z)
    faces = np.arange(vertices.shape[0], dtype=np.int32)
    spec = mj.MjSpec.from_file(str(plant_xml))
    mesh = spec.add_mesh()
    mesh.name = "m2_printed_outsole"
    mesh.uservert = np.ascontiguousarray(vertices.reshape(-1), dtype=np.float64)
    mesh.userface = np.ascontiguousarray(faces, dtype=np.int32)
    mesh.inertia = mj.mjtMeshInertia.mjMESH_INERTIA_CONVEX
    for side in ("l", "r"):
        body = spec.body(f"{side}_ank_roll_link")
        geom = body.add_geom()
        geom.name = f"{side}_printed_outsole"
        geom.type = mj.mjtGeom.mjGEOM_MESH
        geom.meshname = "m2_printed_outsole"
        geom.pos = [positions[side][0], positions[side][1], positions[side][2]]
        geom.quat = [1.0, 0.0, 0.0, 0.0]
        geom.contype = 0
        geom.conaffinity = 0
        geom.group = 1
        geom.rgba = [OUTSOLE_RGBA[0], OUTSOLE_RGBA[1], OUTSOLE_RGBA[2], OUTSOLE_RGBA[3]]
    model = spec.compile()
    for side in ("l", "r"):
        _outsole_sits_on_foot(model, side)
    if int(model.nu) != 24:
        raise ValueError(f"overlay changed nu to {model.nu}")
    for name in ("l_foot_contact", "r_foot_contact"):
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise ValueError(f"missing {name} after compile")
        model.geom_rgba[geom_id, 3] = 0.0
    _measured_pad_mm(model)
    feet = FeetVisual(
        subtitle=OUTSOLE_CAPTION,
        geom_names=(
            "l_foot_contact",
            "r_foot_contact",
            "l_printed_outsole",
            "r_printed_outsole",
        ),
        printed_outsole=True,
    )
    return model, feet


def load_render_model(plant_xml: Path) -> tuple[mj.MjModel, FeetVisual]:
    """Plant XML plus, when it compiles cleanly, the frozen outsole as a visual."""
    digest = assert_plant_md5(plant_xml, "before outsole overlay")
    try:
        model, feet = attach_printed_outsoles(plant_xml)
    except (OSError, ValueError, mj.FatalError) as exc:
        print(f"printed outsole overlay unavailable ({exc}); keeping sim contact pads", flush=True)
        model = mj.MjModel.from_xml_path(str(plant_xml))
        feet = _sim_pad_visual(model)
    else:
        print(f"printed outsole overlay on ({feet.subtitle})", flush=True)
    if plant_md5(plant_xml) != digest:
        raise SystemExit("FAIL: plant file changed while building the render model")
    if "sim contact = real outsole" in feet.subtitle:
        raise SystemExit("FAIL: feet caption still equates the sim pad with the outsole")
    return model, feet


def build_groups(model: mj.MjModel, feet: FeetVisual) -> dict[str, AssemblyGroup]:
    bodies_by_group: dict[str, list[int]] = {key: [] for key in GROUP_ORDER}
    for body_id in range(1, model.nbody):
        name = _body_name(model, body_id)
        bodies_by_group[classify_body(name)].append(body_id)
    for key, body_ids in bodies_by_group.items():
        if key == "feet":
            continue
        if not body_ids:
            raise SystemExit(f"FAIL: assembly group {key} has no bodies")

    foot_ids: list[int] = []
    for name in feet.geom_names:
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise SystemExit(f"FAIL: missing geom {name}")
        foot_ids.append(geom_id)
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cam_id < 0:
        raise SystemExit("FAIL: camera kit_cam is missing from the plant")
    cam_body = _body_name(model, int(model.cam_bodyid[cam_id]))
    if classify_body(cam_body) != "head":
        raise SystemExit(f"FAIL: kit_cam sits on {cam_body}, not the head group")

    counts = _servo_counts(model, bodies_by_group)
    subtitles = {key: "" for key in GROUP_ORDER}
    subtitles["feet"] = feet.subtitle
    groups: dict[str, AssemblyGroup] = {}
    for key in GROUP_ORDER:
        groups[key] = AssemblyGroup(
            key=key,
            title=GROUP_TITLE[key],
            body_ids=tuple(bodies_by_group[key]),
            geom_ids=tuple(foot_ids) if key == "feet" else (),
            servo_count=counts[key],
            subtitle=subtitles[key],
        )
    return groups


def _offset_key(group_key: str, name: str) -> str:
    if group_key == "hips":
        if name.startswith("l_"):
            return "hips_l"
        if name.startswith("r_"):
            return "hips_r"
        raise SystemExit(f"FAIL: hip body {name} is not left or right")
    if group_key == "arms":
        if name.startswith("l_"):
            return "arm_l"
        if name.startswith("r_"):
            return "arm_r"
        raise SystemExit(f"FAIL: arm body {name} is not left or right")
    return group_key


def assert_explode_offsets() -> None:
    """Head stays above the torso and the outsoles stay below the ankles."""
    if INTRO_S < 1.5:
        raise SystemExit(f"FAIL: exploded hold is {INTRO_S:.2f}s, need at least 1.5s")
    if MAX_EXPLODE_DISTANCE > 1.80:
        raise SystemExit(f"FAIL: explode camera cap {MAX_EXPLODE_DISTANCE:.2f} m is past 1.80 m")
    head_z = float(EXPLODE_OFFSET_M["head"][2])
    torso_z = float(EXPLODE_OFFSET_M["torso"][2])
    if head_z < torso_z + 0.12:
        raise SystemExit(
            f"FAIL: head offset z {head_z:.3f} is not clearly above torso z {torso_z:.3f}"
        )
    foot_z = max(float(EXPLODE_OFFSET_M["foot_l"][2]), float(EXPLODE_OFFSET_M["foot_r"][2]))
    leg_z = min(float(EXPLODE_OFFSET_M["left_leg"][2]), float(EXPLODE_OFFSET_M["right_leg"][2]))
    if foot_z > leg_z - 0.16:
        raise SystemExit(
            f"FAIL: foot offset z {foot_z:.3f} is not clearly below the ankles ({leg_z:.3f})"
        )


def build_offsets(
    model: mj.MjModel,
    groups: dict[str, AssemblyGroup],
    specs: dict[str, Vec3] | None = None,
) -> tuple[dict[int, Vec3], dict[int, Vec3]]:
    assert_explode_offsets()
    table = EXPLODE_OFFSET_M if specs is None else specs
    body_delta: dict[int, Vec3] = {
        body_id: np.zeros(3, dtype=np.float64) for body_id in range(model.nbody)
    }
    geom_delta: dict[int, Vec3] = {
        geom_id: np.zeros(3, dtype=np.float64) for geom_id in range(model.ngeom)
    }
    for key in GROUP_ORDER:
        group = groups[key]
        for body_id in group.body_ids:
            name = _body_name(model, body_id)
            spec = _offset_key(key, name)
            body_delta[body_id] = table[spec].copy()
        for geom_id in group.geom_ids:
            name = _geom_name(model, geom_id)
            spec = "foot_l" if name.startswith("l_") else "foot_r"
            geom_delta[geom_id] = table[spec].copy()
    for body_id in range(1, model.nbody):
        if float(np.linalg.norm(body_delta[body_id])) < 1e-8:
            name = _body_name(model, body_id)
            raise SystemExit(f"FAIL: {name} has no explode offset")
    return body_delta, geom_delta


def apply_offsets(
    model: mj.MjModel,
    data: mj.MjData,
    *,
    home_qpos: Vec3,
    home_body_pos: npt.NDArray[np.float64],
    home_geom_pos: npt.NDArray[np.float64],
    home_xmat: npt.NDArray[np.float64],
    root_id: int,
    body_delta: dict[int, Vec3],
    geom_delta: dict[int, Vec3],
    progress: dict[str, float],
    groups: dict[str, AssemblyGroup],
) -> None:
    """Pose the loaded model. progress 1 = exploded, 0 = assembled."""
    scaled_body: dict[int, Vec3] = {
        body_id: np.zeros(3, dtype=np.float64) for body_id in range(model.nbody)
    }
    scaled_geom: dict[int, Vec3] = {
        geom_id: np.zeros(3, dtype=np.float64) for geom_id in range(model.ngeom)
    }
    for key, group in groups.items():
        amount = progress[key]
        for body_id in group.body_ids:
            scaled_body[body_id] = body_delta[body_id] * amount
        for geom_id in group.geom_ids:
            scaled_geom[geom_id] = geom_delta[geom_id] * amount

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[:7] = home_qpos
    data.qpos[:3] = home_qpos[:3] + scaled_body[root_id]
    for body_id in range(model.nbody):
        if body_id == root_id:
            model.body_pos[body_id] = home_body_pos[body_id]
            continue
        parent_id = int(model.body_parentid[body_id])
        parent_rot = home_xmat[parent_id].reshape(3, 3)
        local = parent_rot.T @ (scaled_body[body_id] - scaled_body[parent_id])
        model.body_pos[body_id] = home_body_pos[body_id] + local
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        # A zero geom delta means "draw with the body". Subtracting the body
        # delta here used to pin every mesh at its home world pose, so the
        # explode moved origins and left the pictures assembled.
        if float(np.linalg.norm(scaled_geom[geom_id])) < 1e-12:
            model.geom_pos[geom_id] = home_geom_pos[geom_id]
            continue
        body_rot = home_xmat[body_id].reshape(3, 3)
        local = body_rot.T @ (scaled_geom[geom_id] - scaled_body[body_id])
        model.geom_pos[geom_id] = home_geom_pos[geom_id] + local
    mj.mj_forward(model, data)


def assert_offsets_match_world(
    model: mj.MjModel,
    data: mj.MjData,
    *,
    home_xpos: npt.NDArray[np.float64],
    home_geom_xpos: npt.NDArray[np.float64],
    body_delta: dict[int, Vec3],
    geom_delta: dict[int, Vec3],
    apply_kwargs: dict[str, object],
) -> None:
    """Fail if an in-memory offset does not land on the expected world pose."""
    groups = apply_kwargs["groups"]
    if not isinstance(groups, dict):
        raise SystemExit("FAIL: internal groups missing from offset check")
    home_progress = {key: 0.0 for key in GROUP_ORDER}
    apply_offsets(model, data, progress=home_progress, body_delta=body_delta, geom_delta=geom_delta, **apply_kwargs)  # type: ignore[arg-type]
    if not np.allclose(data.xpos, home_xpos, atol=1e-5):
        raise SystemExit("FAIL: assembled pose drifted from the loaded plant")
    full_progress = {key: 1.0 for key in GROUP_ORDER}
    apply_offsets(model, data, progress=full_progress, body_delta=body_delta, geom_delta=geom_delta, **apply_kwargs)  # type: ignore[arg-type]
    for body_id in range(1, model.nbody):
        expected = home_xpos[body_id] + body_delta[body_id]
        if not np.allclose(data.xpos[body_id], expected, atol=1e-4):
            name = _body_name(model, body_id)
            raise SystemExit(
                f"FAIL: {name} exploded pose {data.xpos[body_id]} != {expected}"
            )
    for geom_id in range(model.ngeom):
        delta = geom_delta[geom_id]
        body_id = int(model.geom_bodyid[geom_id])
        if float(np.linalg.norm(delta)) < 1e-8:
            expected = home_geom_xpos[geom_id] + body_delta[body_id]
        else:
            expected = home_geom_xpos[geom_id] + delta
        if not np.allclose(data.geom_xpos[geom_id], expected, atol=1e-4):
            name = _geom_name(model, geom_id)
            raise SystemExit(
                f"FAIL: {name} exploded pose {data.geom_xpos[geom_id]} != {expected}"
            )
    apply_offsets(model, data, progress=home_progress, body_delta=body_delta, geom_delta=geom_delta, **apply_kwargs)  # type: ignore[arg-type]


def price_caption(path: Path) -> tuple[str, ...]:
    """Price box. Figures must already be in the manufacturing price sheet."""
    if not path.is_file():
        raise SystemExit(f"FAIL: price sheet missing: {path}")
    plain = path.read_text(encoding="utf-8").replace("**", "")
    required = (
        "829.99",
        "30.76",
        "22.40",
        "Standard Kit / Pi 5 (2GB)",
        "24 DOF",
        "Outsole ×4",
        "Tread ×4",
        "GBP ex VAT",
    )
    missing = [token for token in required if token not in plain]
    if missing:
        raise SystemExit(f"FAIL: price sheet is missing {missing}. Refusing to invent prices.")
    return (
        "Kit $829.99 (AiNex Standard, Pi 5 2GB, 24 DOF)",
        "· Outsoles 4x £30.76 total ex VAT",
        "· Treads 4x £22.40 total ex VAT",
    )


def label_for(group: AssemblyGroup | None, phase: str) -> tuple[str, str]:
    if phase == "exploded":
        return "Exploded view", ""
    if phase in {"hold", "spin", "orbit"} or group is None:
        return "Assembled", ""
    if group.servo_count > 0:
        word = "servo" if group.servo_count == 1 else "servos"
        title = f"{group.title}: {group.servo_count} {word}"
    else:
        title = group.title
    return title, group.subtitle


def _font(path: str, size: int) -> ImageFont.ImageFont:
    from PIL import ImageFont

    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def project_scene(
    point: Vec3,
    camera_pos: Vec3,
    camera_forward: Vec3,
    camera_up: Vec3,
    fovy_deg: float,
    *,
    reject_offscreen: bool = True,
) -> tuple[float, float] | None:
    """Pixel of a world point using the camera MuJoCo just rendered. None if behind."""
    forward = camera_forward / float(np.linalg.norm(camera_forward))
    up = camera_up / float(np.linalg.norm(camera_up))
    right = np.cross(forward, up)
    right_norm = float(np.linalg.norm(right))
    if right_norm < 1e-8:
        return None
    right = right / right_norm
    up = np.cross(right, forward)
    relative = point - camera_pos
    depth = float(np.dot(relative, forward))
    if depth <= 0.05:
        return None
    focal = (HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    pixel_x = (WIDTH / 2.0) + focal * float(np.dot(relative, right)) / depth
    pixel_y = (HEIGHT / 2.0) - focal * float(np.dot(relative, up)) / depth
    if reject_offscreen and (
        pixel_x < 16 or pixel_y < 16 or pixel_x > WIDTH - 16 or pixel_y > HEIGHT - 16
    ):
        return None
    return pixel_x, pixel_y


def _kit_cam_label_rect(
    pixel: tuple[float, float],
    font: ImageFont.ImageFont,
) -> tuple[float, float, float, float]:
    """Text box for kit_cam, kept beside the site rather than parked on the frame."""
    center_x, center_y = pixel
    radius = 9.0
    box = font.getbbox("kit_cam")
    text_w = float(box[2] - box[0])
    text_h = float(box[3] - box[1])
    text_x = center_x + radius + 4.0
    text_y = center_y - text_h / 2.0
    if text_x + text_w > WIDTH - 8:
        text_x = center_x - radius - 4.0 - text_w
    if text_y < 4:
        text_y = center_y + radius + 2.0
    if text_y + text_h > HEIGHT - 4:
        text_y = center_y - radius - text_h - 2.0
    return text_x, text_y, text_w, text_h


def _point_to_rect(px: float, py: float, rect: tuple[float, float, float, float]) -> float:
    text_x, text_y, text_w, text_h = rect
    dx = max(text_x - px, 0.0, px - (text_x + text_w))
    dy = max(text_y - py, 0.0, py - (text_y + text_h))
    return math.hypot(dx, dy)


def _draw_cam_marker(
    draw: ImageDraw.ImageDraw,
    pixel: tuple[float, float],
    font: ImageFont.ImageFont,
) -> float:
    """Ring and label at the plant's kit_cam site. Returns label-to-site distance in px."""
    center_x, center_y = pixel
    radius = 9.0
    draw.ellipse(
        [center_x - radius, center_y - radius, center_x + radius, center_y + radius],
        outline=(255, 248, 240, 255),
        width=3,
    )
    draw.ellipse(
        [center_x - 3.0, center_y - 3.0, center_x + 3.0, center_y + 3.0],
        fill=(214, 64, 48, 255),
    )
    text_x, text_y, text_w, text_h = _kit_cam_label_rect(pixel, font)
    anchor_x = text_x - 2.0 if text_x > center_x else text_x + text_w + 2.0
    draw.line(
        [(center_x, center_y), (anchor_x, text_y + text_h / 2.0)],
        fill=(255, 248, 240, 230),
        width=2,
    )
    draw.text((text_x + 1, text_y + 1), "kit_cam", font=font, fill=(0, 0, 0, 200))
    draw.text((text_x, text_y), "kit_cam", font=font, fill=(255, 255, 255, 255))
    return _point_to_rect(center_x, center_y, (text_x, text_y, text_w, text_h))


def overlay_frame(
    frame: npt.NDArray[np.uint8],
    title: str,
    subtitle: str,
    caption: tuple[str, ...],
    cam_pixel: tuple[float, float] | None,
) -> npt.NDArray[np.uint8]:
    from PIL import Image, ImageDraw

    base = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    title_font = _font(FONT_BOLD, 40)
    sub_font = _font(FONT_REGULAR, 26)
    cap_font = _font(FONT_REGULAR, 16)
    pad_x = 18
    pad_y = 12
    title_box = draw.textbbox((0, 0), title, font=title_font)
    title_w = title_box[2] - title_box[0]
    title_h = title_box[3] - title_box[1]
    sub_w = 0
    sub_h = 0
    if subtitle:
        sub_box = draw.textbbox((0, 0), subtitle, font=sub_font)
        sub_w = sub_box[2] - sub_box[0]
        sub_h = sub_box[3] - sub_box[1]
    block_w = max(title_w, sub_w) + pad_x * 2
    block_h = title_h + (sub_h + 6 if subtitle else 0) + pad_y * 2
    origin_x = 32
    origin_y = 28
    draw.rounded_rectangle(
        [origin_x, origin_y, origin_x + block_w, origin_y + block_h],
        radius=10,
        fill=(22, 24, 28, 186),
    )
    draw.text((origin_x + pad_x, origin_y + pad_y - 4), title, font=title_font, fill=(255, 255, 255, 255))
    if subtitle:
        draw.text(
            (origin_x + pad_x, origin_y + pad_y + title_h + 2),
            subtitle,
            font=sub_font,
            fill=(232, 236, 240, 255),
        )
    cap_widths: list[int] = []
    cap_heights: list[int] = []
    for line in caption:
        box = draw.textbbox((0, 0), line, font=cap_font)
        cap_widths.append(box[2] - box[0])
        cap_heights.append(box[3] - box[1])
    cap_w = max(cap_widths) + pad_x * 2
    line_gap = 4
    cap_h = sum(cap_heights) + line_gap * (len(caption) - 1) + pad_y * 2
    cap_x = 32
    cap_y = HEIGHT - cap_h - 28
    draw.rounded_rectangle(
        [cap_x, cap_y, cap_x + cap_w, cap_y + cap_h],
        radius=8,
        fill=(22, 24, 28, 170),
    )
    cursor_y = cap_y + pad_y - 2
    for line, line_h in zip(caption, cap_heights):
        draw.text((cap_x + pad_x, cursor_y), line, font=cap_font, fill=(236, 238, 241, 255))
        cursor_y += line_h + line_gap
    if cam_pixel is not None:
        _draw_cam_marker(draw, cam_pixel, cap_font)
    composed = Image.alpha_composite(base, layer).convert("RGB")
    out = np.asarray(composed, dtype=np.uint8)
    return out


def neutral_studio(model: mj.MjModel) -> None:
    """Flat gray sky and floor. In memory only; the XML is not saved."""
    if model.ntex < 1:
        raise SystemExit("FAIL: plant has no texture to recolor for the backdrop")
    sky_end = int(model.tex_adr[1]) if model.ntex > 1 else int(model.tex_data.shape[0])
    model.tex_data[0:sky_end] = 228
    floor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    if floor_id < 0:
        raise SystemExit("FAIL: plant has no floor geom")
    model.geom_matid[floor_id] = -1
    model.geom_rgba[floor_id] = np.array([0.86, 0.86, 0.87, 1.0], dtype=np.float32)
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = [0.50, 0.50, 0.51]
    model.vis.headlight.diffuse[:] = [0.58, 0.58, 0.57]
    model.vis.global_.ipd = 0.0
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT


def frame_cues() -> list[FrameCue]:
    cues: list[FrameCue] = []
    intro_n = int(round(INTRO_S * FPS))
    cues.extend(FrameCue("exploded", None, 1.0, 0.0) for _ in range(intro_n))
    for key in GROUP_ORDER:
        move_n = int(round(MOVE_S * FPS))
        for index in range(move_n):
            ease = smootherstep((index + 1) / move_n)
            cues.append(FrameCue("in", key, ease, 0.0))
        settle_n = int(round(SETTLE_S * FPS))
        cues.extend(FrameCue("in", key, 1.0, 0.0) for _ in range(settle_n))
    orbit_n = int(round(ORBIT_S * FPS))
    for index in range(orbit_n):
        spin = ORBIT_DEG * (index / max(orbit_n - 1, 1))
        cues.append(FrameCue("orbit", None, 1.0, spin))
    return cues


def progress_for(cue: FrameCue) -> dict[str, float]:
    """1 = still exploded, 0 = seated."""
    if cue.phase == "exploded":
        return {key: 1.0 for key in GROUP_ORDER}
    if cue.phase in {"hold", "spin", "orbit"}:
        return {key: 0.0 for key in GROUP_ORDER}
    progress = {key: 1.0 for key in GROUP_ORDER}
    for key in GROUP_ORDER:
        if key == cue.group_key:
            progress[key] = 1.0 - cue.ease
            break
        progress[key] = 0.0
    return progress


@dataclass(frozen=True)
class GroupGap:
    left: str
    right: str
    centroid_px: float
    aabb_gap_px: float


def _scaled_offset_table(y_scale: float, z_scale: float) -> dict[str, Vec3]:
    """Copy the explode table, shrink |y| first, and keep the vertical air gaps."""
    table: dict[str, Vec3] = {}
    for key, base in EXPLODE_OFFSET_M.items():
        scaled = base.copy()
        scaled[1] *= y_scale
        scaled[2] *= z_scale
        table[key] = scaled
    if table["head"][2] < table["torso"][2] + 0.12:
        table["head"][2] = table["torso"][2] + 0.12
    leg_z = min(float(table["left_leg"][2]), float(table["right_leg"][2]))
    for foot_key in ("foot_l", "foot_r"):
        if table[foot_key][2] > leg_z - 0.16:
            table[foot_key][2] = leg_z - 0.16
    return table


def _anchor_points(
    model: mj.MjModel,
    data: mj.MjData,
    groups: dict[str, AssemblyGroup],
) -> dict[str, Vec3]:
    """One world point per side of a group, used to judge on-screen gaps."""

    def mean_bodies(body_ids: list[int] | tuple[int, ...]) -> Vec3:
        if not body_ids:
            raise SystemExit("FAIL: explode anchor has no bodies")
        stacked = np.stack([np.array(data.xpos[body_id], dtype=np.float64) for body_id in body_ids])
        return stacked.mean(axis=0)

    def side_bodies(group_key: str, prefix: str) -> list[int]:
        return [
            body_id
            for body_id in groups[group_key].body_ids
            if _body_name(model, body_id).startswith(prefix)
        ]

    def foot_point(prefix: str) -> Vec3:
        for name in (f"{prefix}printed_outsole", f"{prefix}foot_contact"):
            geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
            if geom_id >= 0:
                return np.array(data.geom_xpos[geom_id], dtype=np.float64)
        raise SystemExit(f"FAIL: no foot geom for {prefix}")

    torso_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if torso_id < 0:
        raise SystemExit("FAIL: body_link missing")
    return {
        "torso": np.array(data.xpos[torso_id], dtype=np.float64),
        "head": mean_bodies(groups["head"].body_ids),
        "hips_l": mean_bodies(side_bodies("hips", "l_")),
        "hips_r": mean_bodies(side_bodies("hips", "r_")),
        "left_leg": mean_bodies(groups["left_leg"].body_ids),
        "right_leg": mean_bodies(groups["right_leg"].body_ids),
        "arm_l": mean_bodies(side_bodies("arms", "l_")),
        "arm_r": mean_bodies(side_bodies("arms", "r_")),
        "foot_l": foot_point("l_"),
        "foot_r": foot_point("r_"),
    }


def _gl_pixels(
    renderer: mj.Renderer,
    points: dict[str, Vec3],
    fovy_deg: float,
) -> dict[str, tuple[float, float]]:
    gl_cam = renderer.scene.camera[0]
    pos = np.array(gl_cam.pos, dtype=np.float64)
    forward = np.array(gl_cam.forward, dtype=np.float64)
    up = np.array(gl_cam.up, dtype=np.float64)
    pixels: dict[str, tuple[float, float]] = {}
    for key, point in points.items():
        pixel = project_scene(point, pos, forward, up, fovy_deg, reject_offscreen=False)
        if pixel is None:
            raise SystemExit(f"FAIL: {key} is behind the camera")
        pixels[key] = pixel
    return pixels


def _robot_bbox(frame: npt.NDArray[np.uint8]) -> tuple[float, float, bool]:
    """Height and width fractions of the robot pixels, and whether they touch the frame edge."""
    peak = frame.max(axis=2)
    chroma = peak.astype(np.int16) - frame.min(axis=2).astype(np.int16)
    mask = (peak < 140) | (chroma > 32)
    rows, cols = np.where(mask)
    if rows.size == 0:
        return 0.0, 0.0, True
    height = float(rows.max() - rows.min() + 1) / float(HEIGHT)
    width = float(cols.max() - cols.min() + 1) / float(WIDTH)
    clipped = bool(
        rows.min() <= 3 or cols.min() <= 3 or rows.max() >= HEIGHT - 4 or cols.max() >= WIDTH - 4
    )
    return height, width, clipped


def _group_members(
    anchors: dict[str, Vec3],
) -> dict[str, list[Vec3]]:
    return {
        "torso": [anchors["torso"]],
        "hips": [anchors["hips_l"], anchors["hips_r"]],
        "left_leg": [anchors["left_leg"]],
        "right_leg": [anchors["right_leg"]],
        "feet": [anchors["foot_l"], anchors["foot_r"]],
        "arms": [anchors["arm_l"], anchors["arm_r"]],
        "head": [anchors["head"]],
    }


def _aabb_gap_px(
    left: list[tuple[float, float]],
    right: list[tuple[float, float]],
) -> float:
    left_min_x = min(point[0] for point in left)
    left_max_x = max(point[0] for point in left)
    left_min_y = min(point[1] for point in left)
    left_max_y = max(point[1] for point in left)
    right_min_x = min(point[0] for point in right)
    right_max_x = max(point[0] for point in right)
    right_min_y = min(point[1] for point in right)
    right_max_y = max(point[1] for point in right)
    gap_x = max(left_min_x - right_max_x, right_min_x - left_max_x, 0.0)
    gap_y = max(left_min_y - right_max_y, right_min_y - left_max_y, 0.0)
    if gap_x == 0.0 and gap_y == 0.0:
        return 0.0
    return math.hypot(gap_x, gap_y)


def _consecutive_gaps(
    renderer: mj.Renderer,
    anchors: dict[str, Vec3],
    fovy_deg: float,
) -> list[GroupGap]:
    members = _group_members(anchors)
    pixels: dict[str, list[tuple[float, float]]] = {}
    for key, points in members.items():
        projected: list[tuple[float, float]] = []
        for point in points:
            pixel = _gl_pixels(renderer, {key: point}, fovy_deg)[key]
            projected.append(pixel)
        pixels[key] = projected
    gaps: list[GroupGap] = []
    for index in range(len(GROUP_ORDER) - 1):
        left = GROUP_ORDER[index]
        right = GROUP_ORDER[index + 1]
        left_px = pixels[left]
        right_px = pixels[right]
        left_centroid = (
            sum(point[0] for point in left_px) / len(left_px),
            sum(point[1] for point in left_px) / len(left_px),
        )
        right_centroid = (
            sum(point[0] for point in right_px) / len(right_px),
            sum(point[1] for point in right_px) / len(right_px),
        )
        centroid = math.hypot(
            left_centroid[0] - right_centroid[0],
            left_centroid[1] - right_centroid[1],
        )
        gaps.append(GroupGap(left, right, centroid, _aabb_gap_px(left_px, right_px)))
    return gaps


def _gap_ok(gap: GroupGap) -> bool:
    return gap.centroid_px >= MIN_GROUP_CENTROID_PX or gap.aabb_gap_px >= MIN_GROUP_AABB_GAP_PX


def _lookat_for(anchors: dict[str, Vec3]) -> Vec3:
    stacked = np.stack(list(anchors.values()))
    return (stacked.min(axis=0) + stacked.max(axis=0)) * 0.5


def _frame_explode(
    model: mj.MjModel,
    data: mj.MjData,
    renderer: mj.Renderer,
    groups: dict[str, AssemblyGroup],
    apply_kwargs: dict[str, object],
    body_delta: dict[int, Vec3],
    geom_delta: dict[int, Vec3],
) -> tuple[Vec3, float]:
    """Pick a camera at or inside 1.8 m. Shrink |y| before shrinking the vertical gaps."""
    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "kit_cam_site")
    if site_id < 0:
        raise SystemExit("FAIL: site kit_cam_site is missing")
    fovy_deg = float(model.vis.global_.fovy)
    camera = mj.MjvCamera()
    mj.mjv_defaultCamera(camera)
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    camera.elevation = ELEVATION
    camera.azimuth = AZIMUTH0
    distances = tuple(float(step) for step in np.round(np.arange(1.15, MAX_EXPLODE_DISTANCE + 0.001, 0.05), 2))
    y_scales = (1.0, 0.85, 0.70, 0.55, 0.42)
    z_scales = (1.0, 0.85, 0.70)
    nearest_miss = "no candidate rendered"
    for z_scale in z_scales:
        for y_scale in y_scales:
            specs = _scaled_offset_table(y_scale, z_scale)
            trial_body, trial_geom = build_offsets(model, groups, specs)
            apply_offsets(
                model,
                data,
                progress={key: 1.0 for key in GROUP_ORDER},
                body_delta=trial_body,
                geom_delta=trial_geom,
                **apply_kwargs,  # type: ignore[arg-type]
            )
            anchors = _anchor_points(model, data, groups)
            lookat = _lookat_for(anchors)
            camera.lookat[:] = lookat
            for distance in distances:
                camera.distance = distance
                renderer.update_scene(data, camera=camera)
                rgb = np.asarray(renderer.render(), dtype=np.uint8)
                height_frac, width_frac, clipped = _robot_bbox(rgb)
                site_px = _gl_pixels(
                    renderer,
                    {"kit_cam": np.array(data.site_xpos[site_id], dtype=np.float64)},
                    fovy_deg,
                )["kit_cam"]
                site_inside = 48.0 <= site_px[0] <= WIDTH - 48.0 and 48.0 <= site_px[1] <= HEIGHT - 48.0
                gaps = _consecutive_gaps(renderer, anchors, fovy_deg)
                min_centroid = min(gap.centroid_px for gap in gaps)
                filled = (
                    height_frac >= MIN_EXPLODE_BBOX_HEIGHT and width_frac >= MIN_EXPLODE_BBOX_WIDTH
                )
                separated = all(_gap_ok(gap) for gap in gaps)
                nearest_miss = (
                    f"y={y_scale:.2f} z={z_scale:.2f} d={distance:.2f} "
                    f"bbox {height_frac:.2f}x{width_frac:.2f} min centroid {min_centroid:.0f}px "
                    f"clipped={clipped} site={site_inside} gaps={separated}"
                )
                if clipped or not site_inside or not filled or not separated:
                    continue
                body_delta.clear()
                body_delta.update(trial_body)
                geom_delta.clear()
                geom_delta.update(trial_geom)
                print(
                    f"explode camera distance {distance:.2f} lookat {np.round(lookat, 3)} "
                    f"y_scale {y_scale:.2f} z_scale {z_scale:.2f}",
                    flush=True,
                )
                print(
                    f"exploded robot bbox height {height_frac:.2f} width {width_frac:.2f}",
                    flush=True,
                )
                for gap in gaps:
                    print(
                        f"gap {gap.left}-{gap.right}: centroid {gap.centroid_px:.0f}px "
                        f"aabb {gap.aabb_gap_px:.0f}px",
                        flush=True,
                    )
                return lookat, distance
    raise SystemExit(f"FAIL: explode framing did not fill the frame inside 1.8 m. Nearest: {nearest_miss}")


def render_animation(
    model: mj.MjModel,
    data: mj.MjData,
    groups: dict[str, AssemblyGroup],
    apply_kwargs: dict[str, object],
    body_delta: dict[int, Vec3],
    geom_delta: dict[int, Vec3],
    caption: tuple[str, ...],
    out_dir: Path,
) -> tuple[Path, Path, Path, Path]:
    import imageio.v2 as imageio

    out_dir.mkdir(parents=True, exist_ok=True)
    cues = frame_cues()
    mp4_path = out_dir / "assembly.mp4"
    gif_path = out_dir / "assembly.gif"
    exploded_path = out_dir / "exploded.png"
    assembled_path = out_dir / "assembled.png"
    renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
    camera = mj.MjvCamera()
    mj.mjv_defaultCamera(camera)
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = LOOKAT
    camera.distance = DISTANCE
    camera.elevation = ELEVATION
    camera.azimuth = AZIMUTH0
    writer = imageio.get_writer(
        mp4_path,
        fps=FPS,
        codec="libx264",
        quality=8,
        macro_block_size=1,
        ffmpeg_params=["-movflags", "+faststart"],
    )
    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "kit_cam_site")
    if site_id < 0:
        raise SystemExit("FAIL: site kit_cam_site is missing")
    fovy_deg = float(model.vis.global_.fovy)
    explode_lookat, explode_distance = _frame_explode(
        model,
        data,
        renderer,
        groups,
        apply_kwargs,
        body_delta,
        geom_delta,
    )
    if explode_distance > MAX_EXPLODE_DISTANCE + 1e-6:
        raise SystemExit(f"FAIL: explode camera is {explode_distance:.2f} m, cap is 1.80 m")
    wrote_exploded = False
    wrote_assembled = False
    title_frames = int(round(TITLE_S * FPS))
    total_frames = len(cues) + title_frames
    try:
        for index, cue in enumerate(cues):
            progress = progress_for(cue)
            apply_offsets(
                model,
                data,
                progress=progress,
                body_delta=body_delta,
                geom_delta=geom_delta,
                **apply_kwargs,  # type: ignore[arg-type]
            )
            spread = max(progress.values())
            camera.lookat[:] = LOOKAT + (explode_lookat - LOOKAT) * spread
            camera.distance = DISTANCE + (explode_distance - DISTANCE) * spread
            camera.azimuth = AZIMUTH0 + cue.spin_deg
            renderer.update_scene(data, camera=camera)
            rgb = np.asarray(renderer.render(), dtype=np.uint8)
            if cue.phase == "orbit" and not wrote_assembled:
                fraction = _robot_height_fraction(rgb)
                print(f"assembled height fraction {fraction:.2f}", flush=True)
                if fraction < 0.52 or fraction > 0.70:
                    raise SystemExit(
                        f"FAIL: assembled body fills {fraction:.2f} of the frame, expected about 0.60"
                    )
            group = groups[cue.group_key] if cue.group_key is not None else None
            title, subtitle = label_for(group, cue.phase)
            gl_cam = renderer.scene.camera[0]
            cam_pixel = project_scene(
                np.array(data.site_xpos[site_id], dtype=np.float64),
                np.array(gl_cam.pos, dtype=np.float64),
                np.array(gl_cam.forward, dtype=np.float64),
                np.array(gl_cam.up, dtype=np.float64),
                fovy_deg,
            )
            if cam_pixel is None:
                raise SystemExit(f"FAIL: kit_cam site left the frame at frame {index + 1}")
            label_gap = _point_to_rect(
                cam_pixel[0],
                cam_pixel[1],
                _kit_cam_label_rect(cam_pixel, _font(FONT_REGULAR, 16)),
            )
            if label_gap > KIT_CAM_LABEL_PX:
                raise SystemExit(
                    f"FAIL: kit_cam label is {label_gap:.0f}px from the head site "
                    f"at frame {index + 1}"
                )
            painted = overlay_frame(rgb, title, subtitle, caption, cam_pixel)
            writer.append_data(painted)
            if cue.phase == "exploded" and not wrote_exploded:
                _save_png(painted, exploded_path)
                wrote_exploded = True
            if cue.phase == "orbit" and not wrote_assembled:
                _save_png(painted, assembled_path)
                wrote_assembled = True
            if index % FPS == 0:
                print(f"  frame {index + 1}/{total_frames}", flush=True)
        card = _title_card()
        for _ in range(title_frames):
            writer.append_data(card)
    finally:
        writer.close()
        renderer.close()
    print(f"duration {total_frames / FPS:.2f}s frames {total_frames}", flush=True)
    if not exploded_path.is_file() or not assembled_path.is_file():
        raise SystemExit("FAIL: stills were not written")
    _shrink_mp4_if_needed(mp4_path)
    _write_gif(mp4_path, gif_path)
    return mp4_path, gif_path, exploded_path, assembled_path


def _robot_height_fraction(frame: npt.NDArray[np.uint8]) -> float:
    """Share of the frame height occupied by the robot, ignoring the gray backdrop."""
    peak = frame.max(axis=2)
    chroma = peak.astype(np.int16) - frame.min(axis=2).astype(np.int16)
    robot = (peak < 150) | (chroma > 28)
    rows = np.where(robot.any(axis=1))[0]
    if rows.size == 0:
        return 0.0
    return float(rows[-1] - rows[0] + 1) / float(frame.shape[0])


def _title_card() -> npt.NDArray[np.uint8]:
    from PIL import Image, ImageDraw

    image = Image.new("RGB", (WIDTH, HEIGHT), (228, 228, 228))
    draw = ImageDraw.Draw(image)
    font = _font(FONT_BOLD, 32)
    words = TITLE_CARD.split()
    lines: list[str] = []
    current = ""
    for word in words:
        trial = word if not current else f"{current} {word}"
        box = draw.textbbox((0, 0), trial, font=font)
        if box[2] - box[0] > WIDTH - 120 and current:
            lines.append(current)
            current = word
        else:
            current = trial
    if current:
        lines.append(current)
    heights = [draw.textbbox((0, 0), line, font=font)[3] for line in lines]
    block = sum(heights) + 12 * (len(lines) - 1)
    cursor = (HEIGHT - block) / 2
    for line, line_h in zip(lines, heights):
        box = draw.textbbox((0, 0), line, font=font)
        text_w = box[2] - box[0]
        x = (WIDTH - text_w) / 2
        draw.text((x, cursor), line, font=font, fill=(28, 30, 34))
        cursor += line_h + 12
    return np.asarray(image, dtype=np.uint8)


def _save_png(frame: npt.NDArray[np.uint8], path: Path) -> None:
    from PIL import Image

    Image.fromarray(frame).save(path)


def _shrink_mp4_if_needed(path: Path) -> None:
    import imageio_ffmpeg

    size = path.stat().st_size
    if size <= MAX_MP4_BYTES:
        return
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    temporary = path.with_suffix(".tmp.mp4")
    subprocess.run(
        [
            ffmpeg, "-y", "-i", str(path),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28",
            "-movflags", "+faststart",
            str(temporary),
        ],
        check=True,
        capture_output=True,
    )
    temporary.replace(path)
    if path.stat().st_size > MAX_MP4_BYTES:
        raise SystemExit(
            f"FAIL: {path} is {path.stat().st_size} bytes, over {MAX_MP4_BYTES}"
        )


def _write_gif(mp4_path: Path, gif_path: Path) -> None:
    """Short sped-up gif of the whole assembly. Shrink until it fits."""
    import imageio_ffmpeg

    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    attempts: tuple[tuple[float, int, int], ...] = (
        (0.50, 10, 640),
        (0.45, 8, 560),
        (0.40, 8, 480),
    )
    for speed, fps, scale in attempts:
        filtergraph = (
            f"setpts={speed}*PTS,fps={fps},scale={scale}:-1:flags=lanczos,"
            "split[s0][s1];"
            "[s0]palettegen=stats_mode=diff:max_colors=96[p];"
            "[s1][p]paletteuse=dither=bayer:bayer_scale=3"
        )
        subprocess.run(
            [
                ffmpeg, "-y", "-i", str(mp4_path),
                "-filter_complex", filtergraph,
                "-loop", "0",
                str(gif_path),
            ],
            check=True,
            capture_output=True,
        )
        if gif_path.stat().st_size <= MAX_GIF_BYTES:
            return
    raise SystemExit(
        f"FAIL: {gif_path} is {gif_path.stat().st_size} bytes, over {MAX_GIF_BYTES}"
    )


def _describe(model: mj.MjModel, groups: dict[str, AssemblyGroup]) -> None:
    print(f"MUJOCO_GL={os.environ.get('MUJOCO_GL', '')}")
    for key in GROUP_ORDER:
        group = groups[key]
        names = [_body_name(model, body_id) for body_id in group.body_ids]
        geoms = [_geom_name(model, geom_id) for geom_id in group.geom_ids]
        print(
            f"  {key}: servos={group.servo_count} bodies={names} geoms={geoms}",
            flush=True,
        )


def main() -> None:
    digest_before = assert_plant_md5(PLANT_XML, "start")
    print(f"plant md5 {digest_before} OK", flush=True)
    caption = price_caption(PRICE_SHEET)
    model, feet = load_render_model(PLANT_XML)
    data = mj.MjData(model)
    mj.mj_resetData(model, data)
    mj.mj_forward(model, data)
    groups = build_groups(model, feet)
    _describe(model, groups)
    body_delta, geom_delta = build_offsets(model, groups)
    root_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if root_id < 0:
        raise SystemExit("FAIL: body_link missing")
    if int(model.jnt_type[0]) != int(mj.mjtJoint.mjJNT_FREE):
        raise SystemExit("FAIL: expected a free joint on the root body")
    apply_kwargs: dict[str, object] = {
        "home_qpos": np.array(data.qpos[:7], dtype=np.float64).copy(),
        "home_body_pos": np.array(model.body_pos, dtype=np.float64).copy(),
        "home_geom_pos": np.array(model.geom_pos, dtype=np.float64).copy(),
        "home_xmat": np.array(data.xmat, dtype=np.float64).copy(),
        "root_id": root_id,
        "groups": groups,
    }
    home_xpos = np.array(data.xpos, dtype=np.float64).copy()
    home_geom_xpos = np.array(data.geom_xpos, dtype=np.float64).copy()
    assert_offsets_match_world(
        model,
        data,
        home_xpos=home_xpos,
        home_geom_xpos=home_geom_xpos,
        body_delta=body_delta,
        geom_delta=geom_delta,
        apply_kwargs=apply_kwargs,
    )
    neutral_studio(model)
    paths = render_animation(
        model,
        data,
        groups,
        apply_kwargs,
        body_delta,
        geom_delta,
        caption,
        OUT_DIR,
    )
    digest_after = assert_plant_md5(PLANT_XML, "end")
    print(f"plant md5 after render {digest_after} OK", flush=True)
    for path in paths:
        print(f"wrote {path} ({path.stat().st_size} bytes)", flush=True)


if __name__ == "__main__":
    main()
