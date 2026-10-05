#!/usr/bin/env python3
"""Parametric TPU outsole for the Hiwonder AiNex foot.

Drawing only. Does not modify the MuJoCo plant, kit cameras, or any quote.

Solids are millimetres in the ankle-roll link frame (the same frame as
``l_ank_roll_link.STL`` / ``r_ank_roll_link.STL``). The right solid is the
left solid mirrored through the XZ plane, so its centre sits at local y = -14 mm.

Plant source (do not edit): commit 921f5941,
``mujoco/ainex_hiwonder/ainex_controls_m2_145.xml``
md5 207f3d5e9c6a72e16f7aa0c8d224f75e.
"""

from __future__ import annotations

import math
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
FLOOR_THICKNESS_MM = 1.2  # pocket floor web; must stay >= 1.0
POCKET_DEPTH_MM = 2.5  # cleat height above the pocket floor
SOLE_HULL_Z_MAX_MM = -16.5  # vertices below this are the sole plate
BORDER_MM = 2.0
GROOVE_WIDTH_MM = 1.6
GROOVE_PITCH_MM = 6.4
CORNER_PLINTH_MM = 8.0
MIN_LUG_MM = 1.0

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
class Stack:
    """Vertical stack in the ankle-roll frame, millimetres."""

    z_ground: float
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


@dataclass(frozen=True)
class BuiltPair:
    left: cq.Solid
    right: cq.Solid
    stack: Stack
    left_mesh: MeshMeasure
    right_mesh: MeshMeasure
    pocket_hull: tuple[tuple[float, float], ...]


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
    return Stack(
        z_ground=Z_GROUND_MM,
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


def tread_groove_boxes(stack: Stack) -> list[cq.Solid]:
    """Block-tread grooves. Corner plinths and the perimeter rail stay solid."""

    x_min = CENTER_X_MM - OUTER_X_MM / 2.0
    x_max = CENTER_X_MM + OUTER_X_MM / 2.0
    y_min = CENTER_Y_LEFT_MM - OUTER_Y_MM / 2.0
    y_max = CENTER_Y_LEFT_MM + OUTER_Y_MM / 2.0
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


def build_left(mesh: MeshMeasure | None = None) -> tuple[cq.Solid, Stack, MeshMeasure, tuple[tuple[float, float], ...]]:
    left_mesh = mesh if mesh is not None else measure_mesh(LEFT_MESH, "left")
    stack = make_stack(left_mesh.zmin)
    hull = np.array(left_mesh.hull, dtype=np.float64)
    pocket = offset_convex_polygon(hull, CLEARANCE_PER_SIDE_MM)
    base = _rect_solid(
        CENTER_X_MM,
        CENTER_Y_LEFT_MM,
        OUTER_X_MM,
        OUTER_Y_MM,
        stack.z_ground,
        stack.total_thickness,
    )
    pocket_wire = cq.Workplane("XY").workplane(offset=stack.z_floor)
    pocket_cut = pocket_wire.polyline([(float(p[0]), float(p[1])) for p in pocket]).close().extrude(
        stack.pocket_depth + 0.5
    )
    grooves = _fuse(tread_groove_boxes(stack))
    cut = base.cut(pocket_cut.val()).cut(grooves)
    solids = cut.Solids() if isinstance(cut, cq.Compound) else [cut]
    if len(solids) != 1 or not isinstance(solids[0], cq.Solid):
        raise ValueError(f"outsole boolean produced {len(solids)} solids")
    ring = tuple((float(point[0]), float(point[1])) for point in pocket)
    return solids[0], stack, left_mesh, ring


def build_pair() -> BuiltPair:
    left, stack, left_mesh, pocket = build_left()
    mirrored = left.mirror("XZ")
    right_solids = mirrored.Solids() if isinstance(mirrored, cq.Compound) else [mirrored]
    if len(right_solids) != 1 or not isinstance(right_solids[0], cq.Solid):
        raise ValueError("mirrored outsole is not a single solid")
    right_mesh = measure_mesh(RIGHT_MESH, "right")
    return BuiltPair(
        left=left,
        right=right_solids[0],
        stack=stack,
        left_mesh=left_mesh,
        right_mesh=right_mesh,
        pocket_hull=pocket,
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


def format_report(pair: BuiltPair) -> str:
    stack = pair.stack
    left = pair.left_mesh
    right = pair.right_mesh
    overhang = edge_overhang_mm(left)
    contact = contact_area_mm2(pair.left, stack.z_ground)
    planform = OUTER_X_MM * OUTER_Y_MM
    lines = [
        f"plant {PLANT_XML} @ {PLANT_COMMIT} md5 {PLANT_MD5}",
        f"outer {OUTER_X_MM:.3f} x {OUTER_Y_MM:.3f} mm centered ({CENTER_X_MM:.3f}, {CENTER_Y_LEFT_MM:.3f}) left",
        f"right centre y {CENTER_Y_RIGHT_MM:.3f} mm (mirror of left through XZ)",
        f"plant box z [{Z_GROUND_MM:.3f}, {Z_BOX_TOP_MM:.3f}] height {BOX_HEIGHT_MM:.3f} mm",
        f"stack ground {stack.z_ground:.3f} groove {stack.z_groove:.3f} "
        f"floor {stack.z_floor:.3f} cleat {stack.z_top:.3f}",
        f"floor {stack.floor_thickness:.3f} mm  lug {stack.lug_height:.3f} mm  "
        f"pocket {stack.pocket_depth:.3f} mm  total {stack.total_thickness:.3f} mm",
        f"rubber under plate {stack.rubber_under_plate:.3f} mm "
        f"(plant box is {BOX_HEIGHT_MM:.1f} mm; mismatch {BOX_HEIGHT_MM - stack.total_thickness:.3f} mm)",
        f"left mesh AABB {left.size_x:.3f} x {left.size_y:.3f} x {left.zmax - left.zmin:.3f} mm "
        f"center ({left.center_x:.3f}, {left.center_y:.3f}, {(left.zmin + left.zmax) / 2:.3f}) "
        f"zmin {left.zmin:.4f} tris {left.triangle_count}",
        f"right mesh AABB {right.size_x:.3f} x {right.size_y:.3f} "
        f"center ({right.center_x:.3f}, {right.center_y:.3f}) zmin {right.zmin:.4f}",
        "left mesh overhang vs 135x76 plant rect (positive = mesh outside): "
        + ", ".join(f"{name} {value:.3f} mm" for name, value in overhang.items()),
        f"clearance {CLEARANCE_PER_SIDE_MM:.3f} mm outside the sole convex hull",
        f"ground contact area {contact:.1f} mm^2 / {planform:.1f} mm^2 "
        f"({100.0 * contact / planform:.1f}%)",
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
        caption = "Left outsole, top. Pocket floor with four corner cleats. Ankle-roll frame, mm."
    elif view == "bottom":
        camera.SetPosition(cx + 35.0, cy - 150.0, bounds[4] - 70.0)
        camera.SetViewUp(0.0, 1.0, 0.0)
        caption = "Left outsole, ground face. Block tread, perimeter rail, solid corner plinths."
    elif view == "iso":
        camera.SetPosition(cx + 160.0, cy - 190.0, cz + 120.0)
        camera.SetViewUp(0.0, 0.0, 1.0)
        camera.ParallelProjectionOff()
        caption = "Left outsole, isometric. +X toe, +Y outboard, +Z up."
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

    x_min = CENTER_X_MM - OUTER_X_MM / 2.0
    y_min = CENTER_Y_LEFT_MM - OUTER_Y_MM / 2.0
    outer = np.array(
        [
            [x_min, y_min],
            [x_min + OUTER_X_MM, y_min],
            [x_min + OUTER_X_MM, y_min + OUTER_Y_MM],
            [x_min, y_min + OUTER_Y_MM],
        ]
    )
    ax.add_patch(
        MplPolygon(
            outer,
            closed=True,
            facecolor="#c46a1a",
            edgecolor="#6b3208",
            linewidth=1.4,
            alpha=0.55,
            zorder=2,
            label="outsole 135 x 76 mm",
        )
    )
    pocket = np.array(pair.pocket_hull)
    ax.add_patch(
        MplPolygon(
            pocket,
            closed=True,
            facecolor="none",
            edgecolor="#1f4b99",
            linewidth=1.2,
            linestyle="--",
            zorder=3,
            label="pocket = sole hull + 0.3 mm",
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
    dim_x = -52.0
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
        "outsole centre (30, +14)",
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
        "Outsole centred on the plant contact box, 14 mm outboard of the ankle-roll axis"
    )
    ax.legend(loc="upper right", frameon=True, fontsize=8)
    ax.set_xlim(-68.0, 112.0)
    ax.set_ylim(-36.0, 64.0)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


def render_pngs(pair: BuiltPair, stl_path: Path) -> dict[str, Path]:
    paths = {
        "top": OUT_DIR / "outsole_top.png",
        "bottom": OUT_DIR / "outsole_bottom.png",
        "iso": OUT_DIR / "outsole_iso.png",
        "overlay": OUT_DIR / "outsole_overlay.png",
    }
    _vtk_render(stl_path, paths["top"], "top")
    _vtk_render(stl_path, paths["bottom"], "bottom")
    _vtk_render(stl_path, paths["iso"], "iso")
    _render_overlay(pair, paths["overlay"])
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
