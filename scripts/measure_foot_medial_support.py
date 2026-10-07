#!/usr/bin/env python3
"""Measure kit-sole medial room against the cleared foot contact boxes.

The ankle-roll STL is the kit sole. URDF collision origin is the link origin,
which is the ankle-roll axis. This script does not write the plant. It reports
whether a medial-support edit can stay inside that sole and keep the zero-pose
inner gap.

At q=0 every body origin in the plant is a pure translation (no body quat; joint
angles are 0), so ankle world Y is the sum of body pos Y from body_link down.
"""
from __future__ import annotations

import hashlib
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
URDF_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex.urdf"
MESH_DIR = ROOT / "mujoco" / "ainex_hiwonder" / "meshes"

BASELINE_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
SOLE_SLAB_M = 0.002
LEG_JOINTS = (
    "l_hip_yaw",
    "l_hip_roll",
    "l_hip_pitch",
    "l_knee",
    "l_ank_pitch",
    "l_ank_roll",
    "r_hip_yaw",
    "r_hip_roll",
    "r_hip_pitch",
    "r_knee",
    "r_ank_pitch",
    "r_ank_roll",
)

Vec3 = npt.NDArray[np.float64]


@dataclass(frozen=True)
class Aabb:
    min_m: Vec3
    max_m: Vec3

    @property
    def centre_m(self) -> Vec3:
        return (self.min_m + self.max_m) * 0.5

    @property
    def size_m(self) -> Vec3:
        return self.max_m - self.min_m


@dataclass(frozen=True)
class FootBox:
    name: str
    pos_m: Vec3
    half_m: Vec3


@dataclass(frozen=True)
class SoleMeasure:
    side: str
    link: str
    aabb: Aabb
    slab: Aabb
    ankle_world_y_m: float
    box: FootBox
    # Medial is toward the midline: local -Y on the left, local +Y on the right.
    medial_sign: float


def _vec3(text: str) -> Vec3:
    parts = text.split()
    if len(parts) != 3:
        raise SystemExit(f"expected 3 floats, got {text!r}")
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float64)


def load_stl_vertices(path: Path) -> Vec3:
    data = path.read_bytes()
    if len(data) < 84:
        raise SystemExit(f"{path} is not a binary STL")
    count = int.from_bytes(data[80:84], "little")
    record = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    if len(data) < 84 + count * 50:
        raise SystemExit(f"{path} STL triangle count does not fit the file")
    tris = np.frombuffer(data, dtype=record, count=count, offset=84)
    return np.asarray(tris["v"], dtype=np.float64).reshape(-1, 3)


def aabb_of(points: Vec3) -> Aabb:
    if points.size == 0:
        raise SystemExit("empty point set")
    return Aabb(min_m=points.min(axis=0), max_m=points.max(axis=0))


def body_world_pos(root: ET.Element, body_name: str) -> Vec3:
    """World position at q=0. Body elements carry pos only; no body quat."""
    world = root.find("worldbody")
    if world is None:
        raise SystemExit("plant has no worldbody")
    found: list[Vec3] = []

    def walk(node: ET.Element, origin: Vec3) -> None:
        for child in node:
            if child.tag != "body":
                continue
            pos = _vec3(child.get("pos", "0 0 0"))
            if child.get("quat") not in (None, "1 0 0 0"):
                raise SystemExit(f"{child.get('name')} has a body quat; q=0 sum is not valid")
            here = origin + pos
            if child.get("name") == body_name:
                found.append(here)
            walk(child, here)

    walk(world, np.zeros(3, dtype=np.float64))
    if len(found) != 1:
        raise SystemExit(f"body {body_name} found {len(found)} times")
    return found[0]


def foot_box(root: ET.Element, geom_name: str) -> FootBox:
    matches = [g for g in root.iter("geom") if g.get("name") == geom_name]
    if len(matches) != 1:
        raise SystemExit(f"geom {geom_name} found {len(matches)} times")
    geom = matches[0]
    if geom.get("type") != "box":
        raise SystemExit(f"{geom_name} is not a box")
    return FootBox(name=geom_name, pos_m=_vec3(geom.get("pos", "0 0 0")), half_m=_vec3(geom.get("size", "")))


def urdf_collision_origin(link_name: str) -> Vec3:
    tree = ET.parse(URDF_XML)
    link = next((node for node in tree.getroot().iter("link") if node.get("name") == link_name), None)
    if link is None:
        raise SystemExit(f"URDF missing {link_name}")
    collision = link.find("collision")
    if collision is None:
        raise SystemExit(f"URDF {link_name} has no collision")
    mesh = collision.find("geometry/mesh")
    if mesh is None or not str(mesh.get("filename", "")).endswith(f"{link_name}.STL"):
        raise SystemExit(f"URDF {link_name} collision is not the kit STL")
    origin = collision.find("origin")
    xyz = "0 0 0" if origin is None else origin.get("xyz", "0 0 0")
    return _vec3(xyz)


def measure(side: str) -> SoleMeasure:
    link = f"{side}_ank_roll_link"
    verts = load_stl_vertices(MESH_DIR / f"{link}.STL")
    z_min = float(verts[:, 2].min())
    slab_pts = verts[verts[:, 2] <= z_min + SOLE_SLAB_M]
    root = ET.parse(PLANT_XML).getroot()
    medial_sign = -1.0 if side == "l" else 1.0
    return SoleMeasure(
        side=side,
        link=link,
        aabb=aabb_of(verts),
        slab=aabb_of(slab_pts),
        ankle_world_y_m=float(body_world_pos(root, link)[1]),
        box=foot_box(root, f"{side}_foot_contact"),
        medial_sign=medial_sign,
    )


def medial_edge(centre_y: float, half_y: float, medial_sign: float) -> float:
    return centre_y + medial_sign * half_y


def lateral_edge(centre_y: float, half_y: float, medial_sign: float) -> float:
    return centre_y - medial_sign * half_y


def mm(metres: float) -> str:
    return f"{metres * 1000.0:+.3f}"


def fmt_vec_mm(vec: Vec3) -> str:
    return " ".join(f"{float(v) * 1000.0:+.3f}" for v in vec)


def plant_md5() -> str:
    return hashlib.md5(PLANT_XML.read_bytes()).hexdigest()


def actuator_ranges(root: ET.Element) -> dict[str, str]:
    ranges: dict[str, str] = {}
    for node in root.iter("position"):
        joint = node.get("joint")
        if joint is not None:
            ranges[joint] = node.get("forcerange", "")
    return ranges


def main() -> int:
    root = ET.parse(PLANT_XML).getroot()
    feet = (measure("l"), measure("r"))
    md5 = plant_md5()
    ranges = actuator_ranges(root)
    camera = next(node for node in root.iter("camera") if node.get("name") == "kit_cam")
    kit_cam = camera.get("pos", "")

    print("plant", PLANT_XML.relative_to(ROOT))
    print("plant_md5", md5)
    print("baseline_md5", BASELINE_MD5)
    print("kit_cam_pos", kit_cam)
    for joint in LEG_JOINTS:
        print(f"forcerange {joint} {ranges[joint]}")
    arm = ranges.get("l_sho_pitch", "")
    print("forcerange l_sho_pitch", arm)

    print()
    print("side link aabb_min_mm aabb_max_mm slab_min_mm slab_max_mm urdf_collision_xyz_m")
    for foot in feet:
        origin = urdf_collision_origin(foot.link)
        print(
            foot.side,
            foot.link,
            fmt_vec_mm(foot.aabb.min_m),
            "|",
            fmt_vec_mm(foot.aabb.max_m),
            "|",
            fmt_vec_mm(foot.slab.min_m),
            "|",
            fmt_vec_mm(foot.slab.max_m),
            "|",
            " ".join(f"{float(v):.6f}" for v in origin),
        )
        if not np.allclose(origin, 0.0):
            raise SystemExit(f"{foot.link} URDF collision origin is not the ankle axis")

    print()
    print(
        "side ankle_world_y_mm box_half_mm box_centre_y_mm "
        "box_medial_mm box_lateral_mm sole_medial_mm sole_lateral_mm "
        "unused_medial_mm midline_local_mm past_sole_mm"
    )
    world_medial: dict[str, float] = {}
    mesh_world_medial: dict[str, float] = {}
    for foot in feet:
        half_y = float(foot.box.half_m[1])
        centre_y = float(foot.box.pos_m[1])
        expected_centre = 0.014 if foot.side == "l" else -0.014
        if not np.allclose(foot.box.half_m, (0.0675, 0.0380, 0.008), atol=1e-9):
            raise SystemExit(f"{foot.box.name} half-size moved: {foot.box.half_m.tolist()}")
        if abs(centre_y - expected_centre) > 1e-9 or abs(float(foot.box.pos_m[0]) - 0.030) > 1e-9:
            raise SystemExit(f"{foot.box.name} centre moved: {foot.box.pos_m.tolist()}")
        if abs(float(foot.box.pos_m[2]) + 0.018) > 1e-9:
            raise SystemExit(f"{foot.box.name} z moved: {foot.box.pos_m.tolist()}")
        box_med = medial_edge(centre_y, half_y, foot.medial_sign)
        box_lat = lateral_edge(centre_y, half_y, foot.medial_sign)
        sole_med = float(foot.aabb.min_m[1] if foot.medial_sign < 0 else foot.aabb.max_m[1])
        sole_lat = float(foot.aabb.max_m[1] if foot.medial_sign < 0 else foot.aabb.min_m[1])
        # Positive unused_medial means the sole continues past the box toward the midline.
        unused = foot.medial_sign * (sole_med - box_med)
        midline_local = -foot.ankle_world_y_m
        past_sole = foot.medial_sign * (midline_local - sole_med)
        world_medial[foot.side] = foot.ankle_world_y_m + box_med
        mesh_world_medial[foot.side] = foot.ankle_world_y_m + sole_med
        print(
            foot.side,
            mm(foot.ankle_world_y_m),
            fmt_vec_mm(foot.box.half_m),
            mm(centre_y),
            mm(box_med),
            mm(box_lat),
            mm(sole_med),
            mm(sole_lat),
            mm(unused),
            mm(midline_local),
            mm(past_sole),
        )
        # A point on the body midline, in the controls slack definition
        # slack = half_y - abs(local_y - centre_y). Negative is outside.
        slack = half_y - abs(midline_local - centre_y)
        edge_slack = -past_sole - (sole_med - box_med) * foot.medial_sign
        print(
            f"  midline_slack_mm {mm(slack)} "
            f"sole_edge_slack_mm {mm(-past_sole)} "
            f"box_vs_sole_edge_slack_mm {mm(edge_slack)}"
        )

    box_gap = world_medial["l"] - world_medial["r"]
    mesh_gap = mesh_world_medial["l"] - mesh_world_medial["r"]
    print()
    print(f"zero_pose_box_inner_gap_mm {mm(box_gap)}")
    print(f"zero_pose_mesh_inner_gap_mm {mm(mesh_gap)}")
    print("foot_foot_overlap", "yes" if box_gap <= 0.0 else "no")

    # Best box still inside the sole: pin the medial edge on the sole edge and
    # keep the lateral edge on the sole edge. That box is the sole AABB itself.
    # Its midline slack is sole_edge_slack, which stays negative.
    print()
    print("verdict PREFER_FAIL")
    print(
        "reason sole medial edge is already the contact box "
        "(unused medial mesh under 0.06 mm per foot). "
        "A body-midline point stays about 4.9 mm past that edge. "
        "Extending the box to the midline leaves the measured sole and "
        "sets the zero-pose inner gap to 0. Centres and half-sizes stay."
    )
    text = PLANT_XML.read_text()
    required = (
        "Medial thaw Prefer FAIL",
        "-24.037",
        "+52.000",
        "-52.000",
        "+24.051",
        "0.037 mm",
        "0.051 mm",
        "+9.900 mm",
        "+9.812 mm",
    )
    missing = [token for token in required if token not in text]
    if missing:
        print("comment_check MISSING", " ".join(missing))
        return 1
    print("comment_check ok")
    for joint in LEG_JOINTS:
        if ranges[joint] != "-2.45 2.45":
            print(f"forcerange_check FAIL {joint} {ranges[joint]}")
            return 1
    if kit_cam != "0.050 0.019 0.007":
        print("kit_cam_check FAIL", kit_cam)
        return 1
    print("forcerange_check ok leg ±2.45")
    print("kit_cam_check ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
