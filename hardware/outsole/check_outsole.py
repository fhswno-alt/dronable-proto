#!/usr/bin/env python3
"""Assert the AiNex outsole pair against the locked plant and the wall spec.

Checks the exported STEP files and a fresh build:
- wall underside is at least 0.5 mm above the lug tips
- ground contact stays inside the 135 x 76 mm plant box
- each plate has at least 0.3 mm clearance on every side of its own pocket
- inner foot-to-foot gap is positive at zero stance and at the kit stance
- pocket floor web is at least 1.0 mm
- plant blob at commit 921f5941 still hashes to 207f3d5e9c6a72e16f7aa0c8d224f75e
"""

from __future__ import annotations

import hashlib
import subprocess
import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parent))

import outsole

REPO = outsole.REPO
TOL_MM = 0.05
FLOOR_MIN_MM = 1.0


def fail(message: str) -> None:
    raise SystemExit(f"FAIL: {message}")


def require(condition: bool, message: str) -> None:
    if not condition:
        fail(message)


def as_solid(shape: cq.Shape | cq.Workplane) -> cq.Solid:
    if isinstance(shape, cq.Workplane):
        shape = shape.val()
    if isinstance(shape, cq.Compound):
        solids = shape.Solids()
        require(len(solids) == 1, f"expected 1 solid, got {len(solids)}")
        return solids[0]
    if not isinstance(shape, cq.Solid):
        fail(f"expected a solid, got {type(shape).__name__}")
    return shape


def load_step(path: Path) -> cq.Solid:
    require(path.is_file(), f"missing {path}")
    return as_solid(cq.importers.importStep(str(path)))


def assert_outline(solid: cq.Solid, foot: outsole.FootSolid) -> None:
    bounds = outsole.bbox_tuple(solid)
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    require(abs((xmax - xmin) - foot.outline.size_x) < TOL_MM, f"{foot.side} outline x {xmax - xmin:.4f}")
    require(abs((ymax - ymin) - foot.outline.size_y) < TOL_MM, f"{foot.side} outline y {ymax - ymin:.4f}")
    require(abs(zmin - outsole.Z_GROUND_MM) < TOL_MM, f"{foot.side} ground z {zmin:.4f}")
    expected = foot.mesh.size_x + 2.0 * (outsole.CLEARANCE_PER_SIDE_MM + outsole.WALL_MM)
    require(abs(foot.outline.size_x - expected) < 1e-4, f"{foot.side} length is not mesh + wall")
    expected_y = foot.mesh.size_y + 2.0 * (outsole.CLEARANCE_PER_SIDE_MM + outsole.WALL_MM)
    require(abs(foot.outline.size_y - expected_y) < 1e-4, f"{foot.side} width is not mesh + wall")
    print(
        f"OK {foot.side} outline {xmax - xmin:.3f} x {ymax - ymin:.3f} mm "
        f"z [{zmin:.3f}, {zmax:.3f}]"
    )


def assert_floor(solid: cq.Solid, label: str) -> float:
    rows = outsole.horizontal_face_areas(solid)
    up = [(z_value, area) for z_value, normal, area in rows if normal > 0.0]
    down = [(z_value, area) for z_value, normal, area in rows if normal < 0.0]
    require(len(up) >= 2 and len(down) >= 2, f"{label} missing pocket or tread faces")
    floor_z, floor_area = max(up, key=lambda item: item[1])
    groove_candidates = [z_value for z_value, _area in down if z_value > outsole.Z_GROUND_MM + 0.2]
    require(groove_candidates, f"{label} has no groove-root face")
    groove_z = min(groove_candidates, key=lambda z_value: abs(z_value - (floor_z - outsole.FLOOR_THICKNESS_MM)))
    thickness = floor_z - groove_z
    require(thickness + 1e-6 >= FLOOR_MIN_MM, f"{label} pocket floor {thickness:.3f} mm < {FLOOR_MIN_MM:.1f}")
    require(abs(thickness - outsole.FLOOR_THICKNESS_MM) < TOL_MM, f"{label} floor {thickness:.3f} mm")
    require(floor_area > 5000.0, f"{label} pocket floor area {floor_area:.1f} mm^2")
    print(f"OK {label} pocket floor {thickness:.3f} mm at z {floor_z:.3f} (area {floor_area:.1f} mm^2)")
    return thickness


def assert_wall_and_ground(solid: cq.Solid, foot: outsole.FootSolid) -> None:
    """Wall bottom stays above the lugs, and only the plant box touches z = -26."""

    plant = foot.plant
    stack = foot.stack
    outside_floor = outsole.Z_GROUND_MM + outsole.WALL_LIFT_MM
    lowest_outside = 1e9
    saw_ground = False
    for vertex in solid.Vertices():
        point = vertex.Center()
        x_value = float(point.x)
        y_value = float(point.y)
        z_value = float(point.z)
        outside = not plant.contains(x_value, y_value, tol=0.2)
        if z_value < outside_floor - 0.02:
            require(
                plant.contains(x_value, y_value, tol=0.08),
                f"{foot.side} ground vertex ({x_value:.3f}, {y_value:.3f}, {z_value:.3f}) "
                "is outside the 135x76 plant box",
            )
            saw_ground = True
        if outside:
            lowest_outside = min(lowest_outside, z_value)
    require(saw_ground, f"{foot.side} has no lug-tip vertices")
    require(
        lowest_outside >= outside_floor - 0.02,
        f"{foot.side} material outside the plant box reaches z {lowest_outside:.3f}, "
        f"under the {outsole.WALL_LIFT_MM:.1f} mm wall lift",
    )
    wall_y = foot.outline.ymax - outsole.WALL_MM / 2.0
    wall_z = stack.z_wall_bottom + 0.2
    require(
        solid.isInside((outsole.CENTER_X_MM, wall_y, wall_z)),
        f"{foot.side} outboard wall is missing above its underside",
    )
    require(
        not solid.isInside((outsole.CENTER_X_MM, wall_y, outsole.Z_GROUND_MM + 0.1)),
        f"{foot.side} wall touches the lug-tip plane",
    )
    pocket_y = 0.5 * (foot.pocket.ymin + foot.pocket.ymax)
    require(
        solid.isInside((outsole.CENTER_X_MM, pocket_y, stack.z_floor - 0.3)),
        f"{foot.side} pocket floor is missing",
    )
    require(
        not solid.isInside((outsole.CENTER_X_MM, pocket_y, stack.z_floor + 0.4)),
        f"{foot.side} pocket is filled",
    )
    print(
        f"OK {foot.side} wall bottom z {lowest_outside:.3f} "
        f"({lowest_outside - outsole.Z_GROUND_MM:.3f} mm above the lugs); "
        "ground contact stays inside 135x76"
    )


def assert_clearance(pair: outsole.BuiltPair) -> None:
    for foot in (pair.left_foot, pair.right_foot):
        for name, value in foot.clearance_mm.items():
            require(
                value + 1e-6 >= outsole.CLEARANCE_PER_SIDE_MM,
                f"{foot.side} {name} clearance {value:.4f} mm",
            )
        print(
            f"OK {foot.side} clearance "
            + ", ".join(f"{name} {value:.3f} mm" for name, value in foot.clearance_mm.items())
        )


def assert_gaps(pair: outsole.BuiltPair) -> None:
    require(pair.gap_zero_mm > 0.0, f"zero-stance gap {pair.gap_zero_mm:.4f} mm")
    require(pair.gap_kit_mm > 0.0, f"kit-stance gap {pair.gap_kit_mm:.4f} mm")
    require(
        pair.gap_kit_mm > pair.gap_zero_mm,
        "kit stance does not open the inner gap",
    )
    print(f"OK inner gap zero stance {pair.gap_zero_mm:.4f} mm")
    print(
        f"OK inner gap kit stance +{outsole.KIT_STANCE_OUTWARD_M * 1000:.1f} mm/side "
        f"{pair.gap_kit_mm:.4f} mm"
    )


def assert_plant_untouched() -> None:
    blob = subprocess.check_output(
        ["git", "show", f"{outsole.PLANT_COMMIT}:{outsole.PLANT_XML}"],
        cwd=REPO,
    )
    digest = hashlib.md5(blob).hexdigest()
    require(digest == outsole.PLANT_MD5, f"plant md5 {digest}")
    lines = blob.decode("utf-8").splitlines()
    require(
        'size="0.0675 0.0380 0.008" pos="0.030 -0.014 -0.018"' in lines[76],
        "right contact geom moved off line 77",
    )
    require(
        'size="0.0675 0.0380 0.008" pos="0.030 0.014 -0.018"' in lines[106],
        "left contact geom moved off line 107",
    )
    require("14 mm outboard" in lines[8], "plant line 9 no longer states the 14 mm offset")
    status = subprocess.check_output(
        ["git", "status", "--porcelain", "--", outsole.PLANT_XML, "kit_cam"],
        cwd=REPO,
        text=True,
    )
    require(status.strip() == "", f"plant or kit_cam is dirty:\n{status}")
    print(f"OK plant blob {outsole.PLANT_COMMIT} md5 {digest} (working tree plant and kit_cam clean)")


def assert_urdf_citations() -> None:
    lines = outsole.URDF_XACRO.read_text(encoding="utf-8").splitlines()
    require("r_ank_roll_link.STL" in lines[371], "URDF line 372 is not the right foot mesh")
    require("l_ank_roll_link.STL" in lines[713], "URDF line 714 is not the left foot mesh")
    require("<collision>" in lines[365] and "<collision>" in lines[707], "URDF collision blocks moved")
    print("OK URDF foot meshes cited at lines 372 and 714")


def main() -> None:
    assert_plant_untouched()
    assert_urdf_citations()
    pair = outsole.build_pair()
    left_step = load_step(outsole.OUT_DIR / "outsole_left.step")
    right_step = load_step(outsole.OUT_DIR / "outsole_right.step")
    for label, exported, built in (
        ("left", left_step, pair.left),
        ("right", right_step, pair.right),
    ):
        require(abs(exported.Volume() - built.Volume()) < 1.0, f"{label} STEP volume drifted from the script")
    assert_outline(left_step, pair.left_foot)
    assert_outline(right_step, pair.right_foot)
    assert_floor(left_step, "left STEP")
    assert_floor(right_step, "right STEP")
    assert_wall_and_ground(left_step, pair.left_foot)
    assert_wall_and_ground(right_step, pair.right_foot)
    assert_clearance(pair)
    assert_gaps(pair)
    contact = outsole.contact_area_mm2(left_step, pair.stack.z_ground)
    planform = outsole.OUTER_X_MM * outsole.OUTER_Y_MM
    ratio = contact / planform
    require(0.45 < ratio < 0.85, f"tread contact ratio {ratio:.3f} looks wrong")
    print(f"OK ground contact {contact:.1f} mm^2 ({100.0 * ratio:.1f}% of 135x76)")
    print(outsole.format_report(pair))
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
