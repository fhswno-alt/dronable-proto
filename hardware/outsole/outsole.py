#!/usr/bin/env python3
"""Parametric TPU outsole for the Hiwonder AiNex foot.

Drawing only. Does not modify the MuJoCo plant, kit cameras, or any quote.

Solids are millimetres in the ankle-roll link frame (the same frame as
``l_ank_roll_link.STL`` / ``r_ank_roll_link.STL``). Each pocket is cut from
that foot's own mesh. The right solid is not a mirror of the left.

Plant source (do not edit): commit 921f5941,
``mujoco/ainex_hiwonder/ainex_controls_m2_145.xml``
md5 207f3d5e9c6a72e16f7aa0c8d224f75e.
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Sequence

import cadquery as cq
import numpy as np
from cadquery import exporters
from numpy.typing import NDArray

REPO = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
MESH_DIR = REPO / "cad" / "vendor" / "ainex-thorobotics" / "ainex_description" / "meshes"
LEFT_MESH = MESH_DIR / "l_ank_roll_link.STL"
RIGHT_MESH = MESH_DIR / "r_ank_roll_link.STL"

# Plant contact box, commit 921f5941.
# Line 9 states the 135x76 mm mesh AABB and the ±14 mm outboard centres.
# Line 77: r_foot_contact size="0.0675 0.0380 0.008" pos="0.030 -0.014 -0.018"
# Line 107: l_foot_contact size="0.0675 0.0380 0.008" pos="0.030 0.014 -0.018"
# MuJoCo box size is the half-extent, in metres.
PLANT_COMMIT = "921f5941"
PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
PLANT_XML = "mujoco/ainex_hiwonder/ainex_controls_m2_145.xml"
OUTER_X_MM = 135.0  # 2 * 0.0675 m
OUTER_Y_MM = 76.0  # 2 * 0.0380 m
BOX_HEIGHT_MM = 16.0  # 2 * 0.008 m — collision proxy, not the rubber thickness
CENTER_X_MM = 30.0  # pos x = 0.030 m
CENTER_Y_LEFT_MM = 14.0  # pos y = +0.014 m
CENTER_Y_RIGHT_MM = -14.0  # pos y = -0.014 m
BOX_CENTER_Z_MM = -18.0  # pos z = -0.018 m
BOX_HALF_Z_MM = 8.0  # size z = 0.008 m
Z_GROUND_MM = BOX_CENTER_Z_MM - BOX_HALF_Z_MM  # -26.0, plant box bottom
Z_BOX_TOP_MM = BOX_CENTER_Z_MM + BOX_HALF_Z_MM  # -10.0, plant box top
PLANT_FRICTION_SLIDE = 1.6  # friction="1.6 0.1 0.01" on both contact geoms

# URDF collision meshes, ainex.urdf.xacro.
# Right: lines 366-374, filename on line 372.
# Left: lines 708-716, filename on line 714.
URDF_XACRO = (
    REPO
    / "cad"
    / "vendor"
    / "ainex-thorobotics"
    / "ainex_description"
    / "urdf"
    / "ainex.urdf.xacro"
)

# Design assumptions (not taken from the URDF).
CLEARANCE_PER_SIDE_MM = 0.3
WALL_MM = 1.2
WALL_LIFT_MM = 0.5  # wall underside above the lug-tip plane
CHAMFER_MM = 0.4
FLOOR_THICKNESS_MM = 1.2  # pocket floor web; must stay >= 1.0
POCKET_DEPTH_MM = 2.5  # wall height above the pocket floor
SOLE_HULL_Z_MAX_MM = -16.5  # vertices below this are the sole plate
BORDER_MM = 2.0
GROOVE_WIDTH_MM = 1.6
GROOVE_PITCH_MM = 6.4
CORNER_PLINTH_MM = 8.0
MIN_LUG_MM = 1.0
SHELF_OVERLAP_MM = 0.05
# Kit stance: each ankle origin moves this far outboard, sole kept level.
KIT_STANCE_OUTWARD_M = 0.005
LEFT_CHAIN = ("l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll")
RIGHT_CHAIN = ("r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll")

Side = Literal["left", "right"]
FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class MeshMeasure:
    """STL bounds in the ankle-roll frame, millimetres."""

    label: str
    path: Path
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    zmin: float
    zmax: float
    triangle_count: int
    hull: tuple[tuple[float, float], ...]

    @property
    def size_x(self) -> float:
        return self.xmax - self.xmin

    @property
    def size_y(self) -> float:
        return self.ymax - self.ymin

    @property
    def center_x(self) -> float:
        return 0.5 * (self.xmin + self.xmax)

    @property
    def center_y(self) -> float:
        return 0.5 * (self.ymin + self.ymax)


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle in the ankle-roll frame, millimetres."""

    xmin: float
    xmax: float
    ymin: float
    ymax: float

    @property
    def size_x(self) -> float:
        return self.xmax - self.xmin

    @property
    def size_y(self) -> float:
        return self.ymax - self.ymin

    def expanded(self, margin: float) -> Rect:
        return Rect(
            self.xmin - margin,
            self.xmax + margin,
            self.ymin - margin,
            self.ymax + margin,
        )

    def contains(self, x: float, y: float, tol: float = 0.0) -> bool:
        return (
            self.xmin - tol <= x <= self.xmax + tol
            and self.ymin - tol <= y <= self.ymax + tol
        )


@dataclass(frozen=True)
class Stack:
    """Vertical stack in the ankle-roll frame, millimetres."""

    z_ground: float
    z_wall_bottom: float
    z_groove: float
    z_floor: float
    z_top: float
    floor_thickness: float
    lug_height: float
    pocket_depth: float
    total_thickness: float

    @property
    def rubber_under_plate(self) -> float:
        return self.z_floor - self.z_ground

    @property
    def wall_lift(self) -> float:
        return self.z_wall_bottom - self.z_ground


@dataclass(frozen=True)
class FootSolid:
    """One foot's measured pocket, wall outline, and solid."""

    side: Side
    mesh: MeshMeasure
    stack: Stack
    pocket: Rect
    outline: Rect
    plant: Rect
    solid: cq.Solid
    clearance_mm: dict[str, float]


@dataclass(frozen=True)
class BuiltPair:
    left_foot: FootSolid
    right_foot: FootSolid
    ankle_y_left_mm: float
    ankle_y_right_mm: float
    gap_zero_mm: float
    gap_kit_mm: float

    @property
    def left(self) -> cq.Solid:
        return self.left_foot.solid

    @property
    def right(self) -> cq.Solid:
        return self.right_foot.solid

    @property
    def stack(self) -> Stack:
        return self.left_foot.stack

    @property
    def left_mesh(self) -> MeshMeasure:
        return self.left_foot.mesh

    @property
    def right_mesh(self) -> MeshMeasure:
        return self.right_foot.mesh


def load_stl_vertices(path: Path) -> tuple[FloatArray, int]:
    """Return triangle vertices (n, 3, 3) in millimetres and the triangle count."""

    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is too small to be a binary STL")
    count = struct.unpack_from("<I", data, 80)[0]
    byte_count = 84 + count * 50
    if len(data) < byte_count:
        raise ValueError(f"{path} is not a binary STL of {count} triangles")
    record = np.dtype([("normal", "<f4", (3,)), ("vertex", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.frombuffer(data[84:byte_count], dtype=record)
    vertices = np.asarray(triangles["vertex"], dtype=np.float64) * 1000.0
    return vertices, count


def convex_hull(points: FloatArray) -> FloatArray:
    """Andrew's monotone chain. Returns a CCW ring, without the repeated close."""

    if points.shape[1] != 2:
        raise ValueError("hull expects Nx2 points")
    unique = np.unique(np.round(points, 4), axis=0)
    if unique.shape[0] < 3:
        raise ValueError("need at least 3 points for a sole hull")
    ordered = unique[np.lexsort((unique[:, 1], unique[:, 0]))]

    def cross(origin: FloatArray, a: FloatArray, b: FloatArray) -> float:
        return float((a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0]))

    lower: list[FloatArray] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[FloatArray] = []
    for point in ordered[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    ring = lower[:-1] + upper[:-1]
    return np.vstack(ring)


def offset_convex_polygon(polygon: FloatArray, distance: float) -> FloatArray:
    """Outward offset of a CCW convex polygon, in millimetres."""

    if distance < 0.0:
        raise ValueError("clearance must be >= 0")
    count = polygon.shape[0]
    offset = np.zeros_like(polygon)
    for index in range(count):
        previous = polygon[(index - 1) % count]
        current = polygon[index]
        nxt = polygon[(index + 1) % count]
        edge_in = current - previous
        edge_out = nxt - current
        length_in = float(np.linalg.norm(edge_in))
        length_out = float(np.linalg.norm(edge_out))
        if length_in < 1e-9 or length_out < 1e-9:
            raise ValueError("sole hull has a zero-length edge")
        direction_in = edge_in / length_in
        direction_out = edge_out / length_out
        # Right of a CCW edge is outside the polygon.
        normal_in = np.array([direction_in[1], -direction_in[0]], dtype=np.float64)
        normal_out = np.array([direction_out[1], -direction_out[0]], dtype=np.float64)
        matrix = np.column_stack((direction_in, -direction_out))
        try:
            parameter = np.linalg.solve(matrix, (normal_out - normal_in) * distance)
        except np.linalg.LinAlgError as exc:
            raise ValueError("sole hull offset is degenerate") from exc
        offset[index] = current + normal_in * distance + direction_in * float(parameter[0])
    return offset


def measure_mesh(path: Path, label: str) -> MeshMeasure:
    vertices, count = load_stl_vertices(path)
    flat = vertices.reshape(-1, 3)
    low = flat.min(axis=0)
    high = flat.max(axis=0)
    sole = flat[flat[:, 2] < SOLE_HULL_Z_MAX_MM]
    hull = convex_hull(sole[:, :2])
    ring = tuple((float(point[0]), float(point[1])) for point in hull)
    return MeshMeasure(
        label=label,
        path=path,
        xmin=float(low[0]),
        xmax=float(high[0]),
        ymin=float(low[1]),
        ymax=float(high[1]),
        zmin=float(low[2]),
        zmax=float(high[2]),
        triangle_count=count,
        hull=ring,
    )


def pocket_floor_z(mesh_zmin: float) -> float:
    """Round the underside down to 0.001 mm so the floor never rises into the mesh."""

    return math.floor(mesh_zmin * 1000.0) / 1000.0


def make_stack(mesh_zmin: float) -> Stack:
    z_floor = pocket_floor_z(mesh_zmin)
    rubber = z_floor - Z_GROUND_MM
    lug = rubber - FLOOR_THICKNESS_MM
    if lug < MIN_LUG_MM:
        raise ValueError(
            f"lug height {lug:.3f} mm is under {MIN_LUG_MM:.1f} mm. "
            "The plate underside is too close to the plant ground plane to keep "
            f"a {FLOOR_THICKNESS_MM:.1f} mm pocket floor and a tread."
        )
    z_groove = z_floor - FLOOR_THICKNESS_MM
    z_top = z_floor + POCKET_DEPTH_MM
    z_wall_bottom = Z_GROUND_MM + WALL_LIFT_MM
    return Stack(
        z_ground=Z_GROUND_MM,
        z_wall_bottom=z_wall_bottom,
        z_groove=z_groove,
        z_floor=z_floor,
        z_top=z_top,
        floor_thickness=FLOOR_THICKNESS_MM,
        lug_height=lug,
        pocket_depth=POCKET_DEPTH_MM,
        total_thickness=z_top - Z_GROUND_MM,
    )


def _rect_solid(cx: float, cy: float, sx: float, sy: float, z0: float, height: float) -> cq.Solid:
    solid = (
        cq.Workplane("XY")
        .box(sx, sy, height, centered=(True, True, False))
        .translate((cx, cy, z0))
        .val()
    )
    if not isinstance(solid, cq.Solid):
        raise TypeError("expected a single box solid")
    return solid


def _subtract_interval(
    span: tuple[float, float],
    cuts: Sequence[tuple[float, float]],
) -> list[tuple[float, float]]:
    segments = [span]
    for cut_start, cut_end in cuts:
        nxt: list[tuple[float, float]] = []
        for start, end in segments:
            if cut_end <= start or cut_start >= end:
                nxt.append((start, end))
                continue
            if start < cut_start:
                nxt.append((start, min(end, cut_start)))
            if end > cut_end:
                nxt.append((max(start, cut_end), end))
        segments = [(start, end) for start, end in nxt if end - start > 0.8]
    return segments


def tread_groove_boxes(plant: Rect, stack: Stack) -> list[cq.Solid]:
    """Block-tread grooves inside the plant rectangle only."""

    x_min = plant.xmin
    x_max = plant.xmax
    y_min = plant.ymin
    y_max = plant.ymax
    inner = (
        x_min + BORDER_MM,
        x_max - BORDER_MM,
        y_min + BORDER_MM,
        y_max - BORDER_MM,
    )
    corner_cuts_x = ((x_min, x_min + CORNER_PLINTH_MM), (x_max - CORNER_PLINTH_MM, x_max))
    corner_cuts_y = ((y_min, y_min + CORNER_PLINTH_MM), (y_max - CORNER_PLINTH_MM, y_max))
    boxes: list[cq.Solid] = []
    y = inner[2] + GROOVE_PITCH_MM / 2.0
    while y + GROOVE_WIDTH_MM / 2.0 <= inner[3]:
        y0 = y - GROOVE_WIDTH_MM / 2.0
        y1 = y + GROOVE_WIDTH_MM / 2.0
        overlaps_corner = y0 < y_min + CORNER_PLINTH_MM or y1 > y_max - CORNER_PLINTH_MM
        spans = (
            _subtract_interval((inner[0], inner[1]), corner_cuts_x)
            if overlaps_corner
            else [(inner[0], inner[1])]
        )
        for start, end in spans:
            boxes.append(
                _rect_solid(
                    0.5 * (start + end),
                    y,
                    end - start,
                    GROOVE_WIDTH_MM,
                    stack.z_ground - 0.05,
                    stack.lug_height + 0.05,
                )
            )
        y += GROOVE_PITCH_MM
    x = inner[0] + GROOVE_PITCH_MM / 2.0
    while x + GROOVE_WIDTH_MM / 2.0 <= inner[1]:
        x0 = x - GROOVE_WIDTH_MM / 2.0
        x1 = x + GROOVE_WIDTH_MM / 2.0
        overlaps_corner = x0 < x_min + CORNER_PLINTH_MM or x1 > x_max - CORNER_PLINTH_MM
        spans = (
            _subtract_interval((inner[2], inner[3]), corner_cuts_y)
            if overlaps_corner
            else [(inner[2], inner[3])]
        )
        for start, end in spans:
            boxes.append(
                _rect_solid(
                    x,
                    0.5 * (start + end),
                    GROOVE_WIDTH_MM,
                    end - start,
                    stack.z_ground - 0.05,
                    stack.lug_height + 0.05,
                )
            )
        x += GROOVE_PITCH_MM
    if not boxes:
        raise ValueError("tread pattern produced no grooves")
    return boxes


def _fuse(solids: Sequence[cq.Solid]) -> cq.Shape:
    fused: cq.Shape = solids[0]
    for solid in solids[1:]:
        fused = fused.fuse(solid)
    return fused


def _one_solid(shape: cq.Shape, label: str) -> cq.Solid:
    solids = shape.Solids() if isinstance(shape, cq.Compound) else [shape]
    if len(solids) != 1 or not isinstance(solids[0], cq.Solid):
        raise ValueError(f"{label} produced {len(solids)} solids")
    return solids[0]


def _box_rect(rect: Rect, z0: float, height: float) -> cq.Solid:
    return _rect_solid(
        0.5 * (rect.xmin + rect.xmax),
        0.5 * (rect.ymin + rect.ymax),
        rect.size_x,
        rect.size_y,
        z0,
        height,
    )


def plant_rect(side: Side) -> Rect:
    """135 x 76 mm contact box in this ankle-roll frame."""

    centre_y = CENTER_Y_LEFT_MM if side == "left" else CENTER_Y_RIGHT_MM
    return Rect(
        CENTER_X_MM - OUTER_X_MM / 2.0,
        CENTER_X_MM + OUTER_X_MM / 2.0,
        centre_y - OUTER_Y_MM / 2.0,
        centre_y + OUTER_Y_MM / 2.0,
    )


def mesh_rect(mesh: MeshMeasure) -> Rect:
    return Rect(mesh.xmin, mesh.xmax, mesh.ymin, mesh.ymax)


def mesh_xy(path: Path) -> FloatArray:
    vertices, _count = load_stl_vertices(path)
    return vertices.reshape(-1, 3)[:, :2]


def side_clearance_mm(points: FloatArray, pocket: Rect, side: Side) -> dict[str, float]:
    """Minimum axis clearance of every mesh vertex to each pocket wall."""

    heel = float(np.min(points[:, 0] - pocket.xmin))
    toe = float(np.min(pocket.xmax - points[:, 0]))
    toward_min_y = float(np.min(points[:, 1] - pocket.ymin))
    toward_max_y = float(np.min(pocket.ymax - points[:, 1]))
    if side == "left":
        return {
            "heel": heel,
            "toe": toe,
            "inboard": toward_min_y,
            "outboard": toward_max_y,
        }
    return {
        "heel": heel,
        "toe": toe,
        "inboard": toward_max_y,
        "outboard": toward_min_y,
    }


def joint_origin(name: str) -> tuple[float, float, float]:
    """URDF joint origin xyz in metres. All leg origins used here have rpy 0."""

    text = URDF_XACRO.read_text(encoding="utf-8")
    match = re.search(
        rf'name="{re.escape(name)}"\s+type="revolute">\s*<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"',
        text,
    )
    if match is None:
        raise ValueError(f"joint {name} origin not found in {URDF_XACRO.name}")
    rpy = tuple(float(part) for part in match.group(2).split())
    if rpy != (0.0, 0.0, 0.0):
        raise ValueError(f"joint {name} rpy is {rpy}; zero-stance sum assumes identity")
    xyz = tuple(float(part) for part in match.group(1).split())
    if len(xyz) != 3:
        raise ValueError(f"joint {name} xyz is {xyz}")
    return xyz


def ankle_y_m(chain: Sequence[str]) -> float:
    """Body-frame ankle-roll origin Y at every leg joint angle = 0."""

    return sum(joint_origin(name)[1] for name in chain)


def inner_gap_mm(pair_left: FootSolid, pair_right: FootSolid, outward_m: float) -> tuple[float, float, float]:
    """Gap between inner outline edges. Returns (gap, left ankle y, right ankle y) in mm.

    Zero stance is the URDF with every revolute at 0, so the frames stay aligned
    and the ankle Y values add. Kit stance shifts each ankle  ``outward_m``
    further outboard and leaves the sole level (local Y still world Y).
    """

    ankle_left = ankle_y_m(LEFT_CHAIN) + outward_m
    ankle_right = ankle_y_m(RIGHT_CHAIN) - outward_m
    left_inner = ankle_left * 1000.0 + pair_left.outline.ymin
    right_inner = ankle_right * 1000.0 + pair_right.outline.ymax
    return left_inner - right_inner, ankle_left * 1000.0, ankle_right * 1000.0


def build_foot(side: Side, mesh: MeshMeasure) -> FootSolid:
    """Pocket from this plate's AABB, then a 1.2 mm wall, tread only in the plant box."""

    stack = make_stack(mesh.zmin)
    pocket = mesh_rect(mesh).expanded(CLEARANCE_PER_SIDE_MM)
    outline = pocket.expanded(WALL_MM)
    plant = plant_rect(side)
    points = mesh_xy(mesh.path)
    clearance = side_clearance_mm(points, pocket, side)
    short = {name: value for name, value in clearance.items() if value < CLEARANCE_PER_SIDE_MM - 1e-6}
    if short:
        raise ValueError(f"{side} pocket clearance under {CLEARANCE_PER_SIDE_MM} mm: {short}")

    outer = _box_rect(outline, stack.z_wall_bottom, stack.z_top - stack.z_wall_bottom)
    void = _box_rect(
        pocket,
        stack.z_wall_bottom - 0.2,
        (stack.z_top - stack.z_wall_bottom) + 0.4,
    )
    ring = _one_solid(outer.cut(void), f"{side} wall")
    chamfered = cq.Workplane(obj=ring).edges("<Z").chamfer(CHAMFER_MM)
    ring = _one_solid(chamfered.val(), f"{side} wall chamfer")

    shelf = _box_rect(
        pocket.expanded(SHELF_OVERLAP_MM),
        stack.z_groove,
        stack.floor_thickness,
    )
    tread = _box_rect(plant, stack.z_ground, (stack.z_groove - stack.z_ground) + SHELF_OVERLAP_MM)
    grooves = _fuse(tread_groove_boxes(plant, stack))
    tread = _one_solid(tread.cut(grooves), f"{side} tread")
    solid = _one_solid(ring.fuse(shelf).fuse(tread), f"{side} outsole")
    return FootSolid(
        side=side,
        mesh=mesh,
        stack=stack,
        pocket=pocket,
        outline=outline,
        plant=plant,
        solid=solid,
        clearance_mm=clearance,
    )


def build_pair() -> BuiltPair:
    left = build_foot("left", measure_mesh(LEFT_MESH, "left"))
    right = build_foot("right", measure_mesh(RIGHT_MESH, "right"))
    gap_zero, ankle_left, ankle_right = inner_gap_mm(left, right, 0.0)
    gap_kit, _, _ = inner_gap_mm(left, right, KIT_STANCE_OUTWARD_M)
    return BuiltPair(
        left_foot=left,
        right_foot=right,
        ankle_y_left_mm=ankle_left,
        ankle_y_right_mm=ankle_right,
        gap_zero_mm=gap_zero,
        gap_kit_mm=gap_kit,
    )


def export_solids(pair: BuiltPair) -> dict[str, Path]:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    paths = {
        "left_step": OUT_DIR / "outsole_left.step",
        "right_step": OUT_DIR / "outsole_right.step",
        "left_stl": OUT_DIR / "outsole_left.stl",
        "right_stl": OUT_DIR / "outsole_right.stl",
    }
    exporters.export(pair.left, str(paths["left_step"]))
    exporters.export(pair.right, str(paths["right_step"]))
    exporters.export(pair.left, str(paths["left_stl"]))
    exporters.export(pair.right, str(paths["right_stl"]))
    return paths


def bbox_tuple(solid: cq.Solid) -> tuple[float, float, float, float, float, float]:
    box = solid.BoundingBox()
    return (box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax)


def horizontal_face_areas(solid: cq.Solid, bin_mm: float = 0.02) -> list[tuple[float, float, float]]:
    """Return (z_mm, signed_normal_z, area_mm2) clustered for nearly horizontal faces."""

    buckets: dict[tuple[int, int], list[float]] = {}
    for face in solid.Faces():
        normal = face.normalAt()
        if abs(float(normal.z)) < 0.98:
            continue
        center = face.Center()
        z_value = float(center.z)
        sign = 1 if float(normal.z) > 0.0 else -1
        key = (sign, int(round(z_value / bin_mm)))
        buckets.setdefault(key, [0.0, 0.0])
        buckets[key][0] += float(face.Area())
        buckets[key][1] += float(face.Area()) * z_value
    rows: list[tuple[float, float, float]] = []
    for (sign, _bin), (area, moment) in buckets.items():
        rows.append((moment / area, float(sign), area))
    rows.sort(key=lambda row: (row[1], row[0]))
    return rows


def contact_area_mm2(solid: cq.Solid, z_ground: float) -> float:
    area = 0.0
    for z_value, normal_z, face_area in horizontal_face_areas(solid):
        if normal_z < 0.0 and abs(z_value - z_ground) < 0.05:
            area += face_area
    return area


def signed_clearance_mm(points: FloatArray, polygon: FloatArray) -> FloatArray:
    """Signed distance from each point to a polygon. Positive means inside."""

    def contains(x: float, y: float) -> bool:
        inside = False
        count = polygon.shape[0]
        previous = count - 1
        for index in range(count):
            xi = float(polygon[index, 0])
            yi = float(polygon[index, 1])
            xj = float(polygon[previous, 0])
            yj = float(polygon[previous, 1])
            if (yi > y) != (yj > y):
                crossing = (xj - xi) * (y - yi) / (yj - yi + 1e-18) + xi
                if x < crossing:
                    inside = not inside
            previous = index
        return inside

    distances = np.zeros(points.shape[0], dtype=np.float64)
    for index, point in enumerate(points):
        best = 1e9
        for edge in range(polygon.shape[0]):
            start = polygon[edge]
            end = polygon[(edge + 1) % polygon.shape[0]]
            segment = end - start
            length_sq = float(np.dot(segment, segment))
            if length_sq < 1e-12:
                continue
            parameter = float(np.clip(np.dot(point - start, segment) / length_sq, 0.0, 1.0))
            closest = start + parameter * segment
            best = min(best, float(np.linalg.norm(point - closest)))
        distances[index] = best if contains(float(point[0]), float(point[1])) else -best
    return distances


def edge_overhang_mm(mesh: MeshMeasure) -> dict[str, float]:
    """How far the mesh AABB sticks out of the plant rectangle. Positive is outside."""

    x_min = CENTER_X_MM - OUTER_X_MM / 2.0
    x_max = CENTER_X_MM + OUTER_X_MM / 2.0
    y_min = CENTER_Y_LEFT_MM - OUTER_Y_MM / 2.0
    y_max = CENTER_Y_LEFT_MM + OUTER_Y_MM / 2.0
    return {
        "heel_min_x": x_min - mesh.xmin,
        "toe_max_x": mesh.xmax - x_max,
        "inboard_min_y": y_min - mesh.ymin,
        "outboard_max_y": mesh.ymax - y_max,
    }


def _format_clearance(foot: FootSolid) -> str:
    parts = ", ".join(f"{name} {value:.3f} mm" for name, value in foot.clearance_mm.items())
    return (
        f"{foot.side} outline {foot.outline.size_x:.3f} x {foot.outline.size_y:.3f} mm "
        f"pocket {foot.pocket.size_x:.3f} x {foot.pocket.size_y:.3f} mm "
        f"floor z {foot.stack.z_floor:.3f} wall bottom z {foot.stack.z_wall_bottom:.3f} "
        f"clearance [{parts}]"
    )


def format_report(pair: BuiltPair) -> str:
    stack = pair.stack
    left = pair.left_mesh
    right = pair.right_mesh
    contact = contact_area_mm2(pair.left, stack.z_ground)
    planform = OUTER_X_MM * OUTER_Y_MM
    lines = [
        f"plant {PLANT_XML} @ {PLANT_COMMIT} md5 {PLANT_MD5}",
        f"plant tread box {OUTER_X_MM:.3f} x {OUTER_Y_MM:.3f} mm "
        f"centred ({CENTER_X_MM:.3f}, {CENTER_Y_LEFT_MM:.3f}) left / "
        f"({CENTER_X_MM:.3f}, {CENTER_Y_RIGHT_MM:.3f}) right",
        f"plant box z [{Z_GROUND_MM:.3f}, {Z_BOX_TOP_MM:.3f}] height {BOX_HEIGHT_MM:.3f} mm",
        _format_clearance(pair.left_foot),
        _format_clearance(pair.right_foot),
        f"left stack ground {stack.z_ground:.3f} wall {stack.z_wall_bottom:.3f} "
        f"groove {stack.z_groove:.3f} floor {stack.z_floor:.3f} wall top {stack.z_top:.3f}",
        f"left floor {stack.floor_thickness:.3f} mm  lug {stack.lug_height:.3f} mm  "
        f"wall {WALL_MM:.3f} mm  wall lift {stack.wall_lift:.3f} mm  "
        f"part height {stack.total_thickness:.3f} mm",
        f"right floor z {pair.right_foot.stack.z_floor:.3f} "
        f"lug {pair.right_foot.stack.lug_height:.3f} mm "
        f"part height {pair.right_foot.stack.total_thickness:.3f} mm",
        f"rubber under left plate {stack.rubber_under_plate:.3f} mm "
        f"(plant box is {BOX_HEIGHT_MM:.1f} mm; height gap "
        f"{BOX_HEIGHT_MM - stack.total_thickness:.3f} mm)",
        f"left mesh AABB {left.size_x:.3f} x {left.size_y:.3f} mm "
        f"centre ({left.center_x:.3f}, {left.center_y:.3f}) zmin {left.zmin:.4f}",
        f"right mesh AABB {right.size_x:.3f} x {right.size_y:.3f} mm "
        f"centre ({right.center_x:.3f}, {right.center_y:.3f}) zmin {right.zmin:.4f}",
        f"ankle Y zero stance L {pair.ankle_y_left_mm:.4f} mm  R {pair.ankle_y_right_mm:.4f} mm",
        f"inner gap zero stance {pair.gap_zero_mm:.4f} mm",
        f"inner gap kit stance +{KIT_STANCE_OUTWARD_M * 1000:.1f} mm/side {pair.gap_kit_mm:.4f} mm",
        f"ground contact area {contact:.1f} mm^2 / {planform:.1f} mm^2 "
        f"({100.0 * contact / planform:.1f}% of the plant box)",
        f"friction cited {PLANT_FRICTION_SLIDE:.1f} (plant), material TPU 95A",
    ]
    return "\n".join(lines)


def _vtk_render(stl_path: Path, png_path: Path, view: str) -> None:
    import vtk

    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(stl_path))
    reader.Update()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    prop = actor.GetProperty()
    prop.SetColor(0.62, 0.30, 0.08)
    prop.SetAmbient(0.28)
    prop.SetDiffuse(0.72)
    prop.SetSpecular(0.12)
    prop.SetSpecularPower(15)
    prop.EdgeVisibilityOn()
    prop.SetEdgeColor(0.22, 0.10, 0.03)
    prop.SetLineWidth(1.0)

    renderer = vtk.vtkRenderer()
    renderer.AddActor(actor)
    renderer.SetBackground(0.94, 0.95, 0.93)
    renderer.SetBackground2(0.78, 0.80, 0.82)
    renderer.GradientBackgroundOn()
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.AddRenderer(renderer)
    window.SetSize(1400, 1000)

    bounds = actor.GetBounds()
    cx = 0.5 * (bounds[0] + bounds[1])
    cy = 0.5 * (bounds[2] + bounds[3])
    cz = 0.5 * (bounds[4] + bounds[5])
    camera = renderer.GetActiveCamera()
    camera.SetFocalPoint(cx, cy, cz)
    camera.ParallelProjectionOn()
    if view == "top":
        camera.SetPosition(cx + 50.0, cy - 170.0, cz + 55.0)
        camera.SetViewUp(0.0, 1.0, 0.0)
        caption = "Left outsole, top. 1.2 mm pocket wall. Ankle-roll frame, mm."
    elif view == "bottom":
        camera.SetPosition(cx + 35.0, cy - 150.0, bounds[4] - 70.0)
        camera.SetViewUp(0.0, 1.0, 0.0)
        caption = "Left outsole, ground face. Tread only inside the 135 x 76 plant box."
    elif view == "iso":
        camera.SetPosition(cx + 160.0, cy - 190.0, cz + 120.0)
        camera.SetViewUp(0.0, 0.0, 1.0)
        camera.ParallelProjectionOff()
        caption = "Left outsole, isometric. Wall skirt sits above the lug tips."
    else:
        raise ValueError(view)
    renderer.ResetCamera()
    camera.Zoom(1.18)
    note = vtk.vtkCornerAnnotation()
    note.SetLinearFontScaleFactor(2)
    note.SetNonlinearFontScaleFactor(1)
    note.SetMaximumFontSize(18)
    note.GetTextProperty().SetColor(0.12, 0.12, 0.12)
    note.SetText(0, caption)
    renderer.AddViewProp(note)
    window.Render()
    image = vtk.vtkWindowToImageFilter()
    image.SetInput(window)
    image.SetScale(1)
    image.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(png_path))
    writer.SetInputConnection(image.GetOutputPort())
    writer.Write()


def _render_overlay(pair: BuiltPair, png_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon

    vertices, _count = load_stl_vertices(LEFT_MESH)
    fig, ax = plt.subplots(figsize=(11.2, 7.4), dpi=140)
    ax.set_aspect("equal")
    ax.set_facecolor("#f4f5f3")

    # Draw the ankle-roll mesh as a pale plan so the ankle mass near y=0 is visible.
    for triangle in vertices:
        ax.fill(
            triangle[:, 0],
            triangle[:, 1],
            facecolor="#8aa0b2",
            edgecolor="#5d7384",
            linewidth=0.15,
            alpha=0.35,
            zorder=1,
        )

    foot = pair.left_foot
    outline = np.array(
        [
            [foot.outline.xmin, foot.outline.ymin],
            [foot.outline.xmax, foot.outline.ymin],
            [foot.outline.xmax, foot.outline.ymax],
            [foot.outline.xmin, foot.outline.ymax],
        ]
    )
    plant = np.array(
        [
            [foot.plant.xmin, foot.plant.ymin],
            [foot.plant.xmax, foot.plant.ymin],
            [foot.plant.xmax, foot.plant.ymax],
            [foot.plant.xmin, foot.plant.ymax],
        ]
    )
    pocket = np.array(
        [
            [foot.pocket.xmin, foot.pocket.ymin],
            [foot.pocket.xmax, foot.pocket.ymin],
            [foot.pocket.xmax, foot.pocket.ymax],
            [foot.pocket.xmin, foot.pocket.ymax],
        ]
    )
    ax.add_patch(
        MplPolygon(
            outline,
            closed=True,
            facecolor="#c46a1a",
            edgecolor="#6b3208",
            linewidth=1.4,
            alpha=0.45,
            zorder=2,
            label=f"outline {foot.outline.size_x:.1f} x {foot.outline.size_y:.1f} mm",
        )
    )
    ax.add_patch(
        MplPolygon(
            plant,
            closed=True,
            facecolor="#e8b56a",
            edgecolor="#8a4b08",
            linewidth=1.1,
            alpha=0.55,
            zorder=3,
            label="plant tread 135 x 76 mm",
        )
    )
    ax.add_patch(
        MplPolygon(
            pocket,
            closed=True,
            facecolor="none",
            edgecolor="#1f4b99",
            linewidth=1.2,
            linestyle="--",
            zorder=4,
            label="pocket = this plate AABB + 0.3 mm",
        )
    )
    hull = np.array(pair.left_mesh.hull)
    ax.plot(
        np.append(hull[:, 0], hull[0, 0]),
        np.append(hull[:, 1], hull[0, 1]),
        color="#16324f",
        linewidth=1.0,
        zorder=4,
        label="left sole convex hull",
    )

    ax.axhline(0.0, color="#111111", linewidth=1.0, linestyle=":", zorder=5)
    ax.axhline(CENTER_Y_LEFT_MM, color="#6b3208", linewidth=1.0, zorder=5)
    dim_x = foot.outline.xmin - 12.0
    ax.annotate(
        "",
        xy=(dim_x, CENTER_Y_LEFT_MM),
        xytext=(dim_x, 0.0),
        arrowprops={"arrowstyle": "<->", "color": "#111111", "lw": 1.15},
        zorder=6,
    )
    ax.text(
        dim_x - 1.4,
        CENTER_Y_LEFT_MM / 2.0,
        "14 mm\noutboard",
        ha="right",
        va="center",
        fontsize=9,
        color="#111111",
        zorder=6,
    )
    ax.text(CENTER_X_MM, -2.8, "ankle-roll axis  y = 0", ha="center", va="top", fontsize=8, color="#111111")
    ax.plot(
        [CENTER_X_MM],
        [CENTER_Y_LEFT_MM],
        marker="o",
        color="#6b3208",
        markersize=7,
        markerfacecolor="none",
        markeredgewidth=1.4,
        zorder=6,
    )
    ax.annotate(
        "plant tread centre (30, +14)",
        xy=(CENTER_X_MM, CENTER_Y_LEFT_MM),
        xytext=(CENTER_X_MM + 18.0, CENTER_Y_LEFT_MM + 16.0),
        fontsize=8,
        color="#6b3208",
        arrowprops={"arrowstyle": "->", "color": "#6b3208", "lw": 0.8},
        zorder=6,
    )
    ax.plot([0.0], [0.0], marker="o", color="#111111", markersize=4, zorder=6)
    ax.annotate(
        "ankle-roll origin (0, 0)",
        xy=(0.0, 0.0),
        xytext=(-46.0, 8.0),
        fontsize=8,
        color="#111111",
        arrowprops={"arrowstyle": "->", "color": "#111111", "lw": 0.8},
        zorder=6,
    )
    ax.set_xlabel("x mm  (toe is +x)")
    ax.set_ylabel("y mm  (left outboard is +y)")
    ax.set_title(
        "Left foot overlay, ankle-roll frame\n"
        "Wider wall outline around this plate. Tread centre stays 14 mm outboard of the ankle-roll axis."
    )
    ax.legend(loc="upper right", frameon=True, fontsize=8)
    ax.set_xlim(foot.outline.xmin - 28.0, foot.outline.xmax + 16.0)
    ax.set_ylim(foot.outline.ymin - 14.0, foot.outline.ymax + 14.0)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


def _section_loops(stl_path: Path, cut_x: float) -> list[FloatArray]:
    """Closed YZ loops where the STL crosses the plane x = cut_x."""

    import vtk

    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(stl_path))
    reader.Update()
    plane = vtk.vtkPlane()
    plane.SetOrigin(cut_x, 0.0, 0.0)
    plane.SetNormal(1.0, 0.0, 0.0)
    cutter = vtk.vtkCutter()
    cutter.SetCutFunction(plane)
    cutter.SetInputConnection(reader.GetOutputPort())
    stripper = vtk.vtkStripper()
    stripper.SetInputConnection(cutter.GetOutputPort())
    stripper.JoinContiguousSegmentsOn()
    stripper.Update()
    poly = stripper.GetOutput()
    loops: list[FloatArray] = []
    for cell_index in range(poly.GetNumberOfCells()):
        cell = poly.GetCell(cell_index)
        count = cell.GetNumberOfPoints()
        if count < 3:
            continue
        pts = np.zeros((count, 2), dtype=np.float64)
        for point_index in range(count):
            point = cell.GetPoints().GetPoint(point_index)
            pts[point_index, 0] = point[1]
            pts[point_index, 1] = point[2]
        loops.append(pts)
    if not loops:
        raise ValueError(f"section at x={cut_x} produced no loops")
    return loops


def _render_section(stl_path: Path, png_path: Path, foot: FootSolid) -> None:
    """YZ section through the tread, with a zoom on the lifted wall."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon

    cut_x = CENTER_X_MM
    loops = _section_loops(stl_path, cut_x)
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 5.2), dpi=140)
    full, zoom = axes
    for loop in loops:
        patch = MplPolygon(loop, closed=True, facecolor="#c46a1a", edgecolor="#3d220c", linewidth=0.6)
        full.add_patch(patch)
        zoom.add_patch(
            MplPolygon(loop, closed=True, facecolor="#c46a1a", edgecolor="#3d220c", linewidth=0.8)
        )
    for axis in (full, zoom):
        axis.axhline(Z_GROUND_MM, color="#1b4f72", linewidth=1.0, linestyle="--")
        axis.axhline(foot.stack.z_wall_bottom, color="#6b3208", linewidth=0.9, linestyle=":")
        axis.set_aspect("equal")
        axis.set_facecolor("#f7f6f3")
        axis.set_xlabel("y mm")
        axis.set_ylabel("z mm")
    full.set_title(f"Section x = {cut_x:.0f} mm  (looking toward the toe)")
    full.set_xlim(foot.outline.ymin - 3.0, foot.outline.ymax + 3.0)
    full.set_ylim(Z_GROUND_MM - 1.5, foot.stack.z_top + 1.2)
    full.text(
        foot.outline.ymin + 2.0,
        Z_GROUND_MM - 1.15,
        "lug tips  z = -26",
        color="#1b4f72",
        fontsize=8,
    )
    # Zoom on the outboard wall (positive Y for the left foot).
    zoom_y = foot.outline.ymax
    zoom.set_xlim(zoom_y - 4.2, zoom_y + 1.2)
    zoom.set_ylim(Z_GROUND_MM - 0.8, foot.stack.z_wall_bottom + 2.4)
    zoom.set_title("Outboard wall, lifted off the ground")
    zoom.annotate(
        "",
        xy=(zoom_y + 0.55, foot.stack.z_wall_bottom),
        xytext=(zoom_y + 0.55, Z_GROUND_MM),
        arrowprops={"arrowstyle": "<->", "color": "#111111", "lw": 1.0},
    )
    zoom.text(
        zoom_y + 0.7,
        0.5 * (Z_GROUND_MM + foot.stack.z_wall_bottom),
        f"{foot.stack.wall_lift:.1f} mm",
        va="center",
        fontsize=8,
        color="#111111",
    )
    fig.suptitle(
        "Left outsole. The perimeter wall bottom is above the lug-tip plane. "
        "Only the centre tread reaches z = -26 mm.",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


def render_pngs(pair: BuiltPair, stl_path: Path) -> dict[str, Path]:
    paths = {
        "top": OUT_DIR / "outsole_top.png",
        "bottom": OUT_DIR / "outsole_bottom.png",
        "iso": OUT_DIR / "outsole_iso.png",
        "overlay": OUT_DIR / "outsole_overlay.png",
        "section": OUT_DIR / "outsole_section.png",
    }
    _vtk_render(stl_path, paths["top"], "top")
    _vtk_render(stl_path, paths["bottom"], "bottom")
    _vtk_render(stl_path, paths["iso"], "iso")
    _render_overlay(pair, paths["overlay"])
    _render_section(stl_path, paths["section"], pair.left_foot)
    return paths


def main() -> None:
    pair = build_pair()
    paths = export_solids(pair)
    pngs = render_pngs(pair, paths["left_stl"])
    print(format_report(pair))
    for path in (*paths.values(), *pngs.values()):
        print(f"wrote {path.relative_to(REPO)}")


if __name__ == "__main__":
    main()
