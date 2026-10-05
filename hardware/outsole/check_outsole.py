#!/usr/bin/env python3
"""Assert the AiNex outsole pair against the locked plant dimensions.

Checks the exported STEP files and a fresh build:
- left/right are mirrors through the ankle-roll XZ plane
- each footprint is 135 x 76 mm
- centres are y = +14 mm and y = -14 mm (x = 30 mm)
- pocket floor web is at least 1.0 mm

Also confirms the plant blob at commit 921f5941 still hashes to
207f3d5e9c6a72e16f7aa0c8d224f75e and that this script does not dirty it.
"""

from __future__ import annotations

import hashlib
import subprocess
import sys
from pathlib import Path

import cadquery as cq
import numpy as np

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


def centre(bounds: tuple[float, float, float, float, float, float]) -> tuple[float, float, float]:
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    return (0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax))


def assert_footprint(solid: cq.Solid, centre_y: float, label: str) -> None:
    bounds = outsole.bbox_tuple(solid)
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    cx, cy, _cz = centre(bounds)
    require(abs((xmax - xmin) - outsole.OUTER_X_MM) < TOL_MM, f"{label} x size {xmax - xmin:.4f}")
    require(abs((ymax - ymin) - outsole.OUTER_Y_MM) < TOL_MM, f"{label} y size {ymax - ymin:.4f}")
    require(abs(cx - outsole.CENTER_X_MM) < TOL_MM, f"{label} centre x {cx:.4f}")
    require(abs(cy - centre_y) < TOL_MM, f"{label} centre y {cy:.4f}")
    require(abs(zmin - outsole.Z_GROUND_MM) < TOL_MM, f"{label} ground z {zmin:.4f}")
    require(zmax - zmin < outsole.BOX_HEIGHT_MM - 1.0, f"{label} thickness fills the 16 mm box")
    print(
        f"OK {label} footprint {xmax - xmin:.3f} x {ymax - ymin:.3f} mm "
        f"centre ({cx:.3f}, {cy:.3f}) z [{zmin:.3f}, {zmax:.3f}]"
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


def assert_features(solid: cq.Solid, centre_y: float, stack: outsole.Stack) -> None:
    z_cleat = 0.5 * (stack.z_floor + stack.z_top)
    x_min = outsole.CENTER_X_MM - outsole.OUTER_X_MM / 2.0
    x_max = outsole.CENTER_X_MM + outsole.OUTER_X_MM / 2.0
    y_min = centre_y - outsole.OUTER_Y_MM / 2.0
    y_max = centre_y + outsole.OUTER_Y_MM / 2.0
    corners = (
        (x_min + 0.4, y_min + 0.4),
        (x_min + 0.4, y_max - 0.4),
        (x_max - 0.4, y_min + 0.4),
        (x_max - 0.4, y_max - 0.4),
    )
    for x_value, y_value in corners:
        require(
            solid.isInside((x_value, y_value, z_cleat)),
            f"corner cleat missing at ({x_value:.2f}, {y_value:.2f})",
        )
    openings = (
        (outsole.CENTER_X_MM, y_max - 0.4),
        (outsole.CENTER_X_MM, y_min + 0.4),
        (x_max - 0.4, centre_y),
        (x_min + 0.4, centre_y),
    )
    for x_value, y_value in openings:
        require(
            not solid.isInside((x_value, y_value, z_cleat)),
            f"pocket wall still closes ({x_value:.2f}, {y_value:.2f}); plate cannot drop in",
        )
    require(
        solid.isInside((outsole.CENTER_X_MM, centre_y, stack.z_floor - 0.4)),
        "pocket floor is missing under the centre",
    )
    require(
        not solid.isInside((outsole.CENTER_X_MM, centre_y, stack.z_floor + 0.4)),
        "pocket void is filled at the centre",
    )
    print("OK corner cleats present and the four flats are open so the plate can drop in")


def assert_mirror(left: cq.Solid, right: cq.Solid, stack: outsole.Stack) -> None:
    require(abs(left.Volume() - right.Volume()) < 1e-3, "left/right volumes differ")
    mismatches = 0
    checked = 0
    z_values = (
        stack.z_ground + 0.4,
        stack.z_groove + 0.3,
        stack.z_floor - 0.3,
        stack.z_floor + 0.6,
        stack.z_top - 0.3,
    )
    for x_value in np.arange(-30.0, 95.0, 10.0):
        for y_value in np.arange(-20.0, 50.0, 8.0):
            for z_value in z_values:
                left_in = bool(left.isInside((float(x_value), float(y_value), float(z_value))))
                right_in = bool(right.isInside((float(x_value), float(-y_value), float(z_value))))
                checked += 1
                if left_in != right_in:
                    mismatches += 1
    require(mismatches == 0, f"mirror mismatch on {mismatches} of {checked} sample points")
    print(f"OK mirror through XZ on {checked} sample points, volumes match")


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


def assert_clearance(pair: outsole.BuiltPair) -> None:
    pocket = np.array(pair.pocket_hull, dtype=np.float64)
    left_hull = np.array(pair.left_mesh.hull, dtype=np.float64)
    left_gap = outsole.signed_clearance_mm(left_hull, pocket)
    require(float(left_gap.min()) > 0.29, f"left hull clearance {float(left_gap.min()):.3f} mm")
    mirrored = pocket.copy()
    mirrored[:, 1] *= -1.0
    right_hull = np.array(pair.right_mesh.hull, dtype=np.float64)
    right_gap = outsole.signed_clearance_mm(right_hull, mirrored)
    require(float(right_gap.min()) > 0.0, f"right hull intersects the pocket ({float(right_gap.min()):.3f} mm)")
    overhang = outsole.edge_overhang_mm(pair.left_mesh)
    require(overhang["heel_min_x"] > 0.2, "left heel no longer overhangs the 135 mm plant box")
    print(
        f"OK left hull clearance {float(left_gap.min()):.3f} mm; "
        f"right hull minimum {float(right_gap.min()):.3f} mm (mesh is not an exact mirror)"
    )
    print(
        "OK left mesh overhang vs plant rect: "
        + ", ".join(f"{name} {value:.3f} mm" for name, value in overhang.items())
    )


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
    assert_footprint(left_step, outsole.CENTER_Y_LEFT_MM, "left STEP")
    assert_footprint(right_step, outsole.CENTER_Y_RIGHT_MM, "right STEP")
    assert_floor(left_step, "left STEP")
    assert_floor(right_step, "right STEP")
    assert_features(left_step, outsole.CENTER_Y_LEFT_MM, pair.stack)
    assert_features(right_step, outsole.CENTER_Y_RIGHT_MM, pair.stack)
    assert_mirror(left_step, right_step, pair.stack)
    assert_clearance(pair)
    contact = outsole.contact_area_mm2(left_step, pair.stack.z_ground)
    planform = outsole.OUTER_X_MM * outsole.OUTER_Y_MM
    ratio = contact / planform
    require(0.45 < ratio < 0.85, f"tread contact ratio {ratio:.3f} looks wrong")
    print(f"OK ground contact {contact:.1f} mm^2 ({100.0 * ratio:.1f}% of 135x76)")
    print(outsole.format_report(pair))
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
