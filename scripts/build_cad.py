#!/usr/bin/env python3
"""Dronable v0 biped CAD — CadQuery parametric assembly (~415 mm AiNex-class)."""
import cadquery as cq
from cadquery import exporters
from pathlib import Path

OUT = Path("/workspace/dronable-proto/cad")
OUT.mkdir(parents=True, exist_ok=True)

# --- Envelope (mm) — AiNex Standard class guesses ---
TOTAL_H = 415.0
FOOT_L, FOOT_W, FOOT_H = 72.0, 44.0, 14.0
SHIN_L, SHIN_W, SHIN_D = 105.0, 28.0, 22.0
THIGH_L, THIGH_W, THIGH_D = 100.0, 32.0, 26.0
PELVIS_W, PELVIS_D, PELVIS_H = 80.0, 42.0, 38.0
TORSO_W, TORSO_D, TORSO_H = 74.0, 40.0, 100.0
HEAD_W, HEAD_D, HEAD_H = 50.0, 44.0, 50.0
UARM_L, UARM_R = 72.0, 14.0
FARM_L, FARM_R = 68.0, 12.0
HAND_L, HAND_W, HAND_H = 30.0, 18.0, 10.0
HIP_SPACING = 50.0  # center-to-center legs

def box_at(size, origin):
    """Axis-aligned box centered at origin (x,y,z), size (sx,sy,sz)."""
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(origin)

def cyl_y(length, radius, origin, rotate_z=0):
    """Cylinder along local Y (arm axis), origin at proximal center."""
    c = cq.Workplane("XZ").circle(radius).extrude(length)
    c = c.translate((0, 0, 0))
    # CadQuery extrude along +Z of workplane... use ZX then rotate
    c = cq.Workplane("XY").cylinder(length, radius)
    # Default cylinder is along Z; rotate to along -Y for right arm? We'll place manually.
    return c

# Build parts as solids and assemble with transforms
# Coordinate: Z up, Y forward, X right (robot facing +Y)

parts = []

# --- Feet (at ground Z=0 top of foot) ---
# Ankle at FOOT_H
z_ankle = FOOT_H
for side, sx in (("L", -HIP_SPACING / 2), ("R", HIP_SPACING / 2)):
    foot = box_at((FOOT_W, FOOT_L, FOOT_H), (sx, FOOT_L * 0.15, FOOT_H / 2))
    parts.append(foot)
    # Shin: ankle to knee
    z_knee = z_ankle + SHIN_L
    shin = box_at((SHIN_W, SHIN_D, SHIN_L), (sx, 0, z_ankle + SHIN_L / 2))
    parts.append(shin)
    # Thigh: knee to hip
    z_hip = z_knee + THIGH_L
    thigh = box_at((THIGH_W, THIGH_D, THIGH_L), (sx, 0, z_knee + THIGH_L / 2))
    parts.append(thigh)

z_hip = FOOT_H + SHIN_L + THIGH_L  # ~197

# Pelvis
pelvis = box_at((PELVIS_W, PELVIS_D, PELVIS_H), (0, 0, z_hip + PELVIS_H / 2))
parts.append(pelvis)
z_waist = z_hip + PELVIS_H

# Torso
torso = box_at((TORSO_W, TORSO_D, TORSO_H), (0, 0, z_waist + TORSO_H / 2))
parts.append(torso)
z_neck = z_waist + TORSO_H

# Head shell (rounded-ish via box + slight)
head = box_at((HEAD_W, HEAD_D, HEAD_H), (0, 2, z_neck + HEAD_H / 2))
# Soften: fillet if possible
try:
    head = head.edges("|Z").fillet(4)
except Exception:
    pass
parts.append(head)

# Eye pivots (simple cylinders protruding)
eye_z = z_neck + HEAD_H * 0.55
eye_y = HEAD_D / 2 - 2
for ex in (-12, 12):
    eye = (
        cq.Workplane("XY")
        .cylinder(6, 5)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((ex, eye_y + 3, eye_z))
    )
    parts.append(eye)

# Arms — shoulder at upper torso sides
z_shoulder = z_waist + TORSO_H * 0.75
for side, sx, sign in (("L", -TORSO_W / 2 - 8, -1), ("R", TORSO_W / 2 + 8, 1)):
    # Upper arm along -Z (hanging), slight out
    uarm = (
        cq.Workplane("XY")
        .cylinder(UARM_L, UARM_R)
        .translate((sx + sign * 5, 0, z_shoulder - UARM_L / 2))
    )
    parts.append(uarm)
    z_elbow = z_shoulder - UARM_L
    farm = (
        cq.Workplane("XY")
        .cylinder(FARM_L, FARM_R)
        .translate((sx + sign * 5, 0, z_elbow - FARM_L / 2))
    )
    parts.append(farm)
    z_wrist = z_elbow - FARM_L
    hand = box_at(
        (HAND_W, HAND_L, HAND_H),
        (sx + sign * 5, HAND_L * 0.2, z_wrist - HAND_H / 2),
    )
    parts.append(hand)

# Fuse assembly
assy = parts[0]
for p in parts[1:]:
    assy = assy.union(p)

# Optional simple door lever prop (separate, also in assembly for context)
# Door panel + lever at ~275 mm
door = (
    cq.Workplane("XY")
    .box(8, 40, 320)
    .translate((180, 80, 160))
)
lever = (
    cq.Workplane("XY")
    .box(50, 8, 10)
    .translate((155, 80, 275))
)
assy_with_prop = assy.union(door).union(lever)

print(f"Hip height ~{z_hip:.1f} mm, neck ~{z_neck:.1f} mm, top ~{z_neck + HEAD_H:.1f} mm")

# Export STEP + STL
step_path = OUT / "dronable_v0_assembly.step"
stl_path = OUT / "dronable_v0_assembly.stl"
step_body = OUT / "dronable_v0_body.step"
stl_body = OUT / "dronable_v0_body.stl"

exporters.export(assy, str(step_body))
exporters.export(assy, str(stl_body))
exporters.export(assy_with_prop, str(step_path))
exporters.export(assy_with_prop, str(stl_path))
print(f"Wrote {step_body}")
print(f"Wrote {stl_body}")
print(f"Wrote {step_path}")
print(f"Wrote {stl_path}")

# Static preview via matplotlib-ish: use trimesh render if available
try:
    import trimesh
    import numpy as np
    mesh = trimesh.load(str(stl_body))
    # Simple orthographic-ish PNG via scene
    scene = mesh.scene()
    png = scene.save_image(resolution=(800, 1000), visible=True)
    if png:
        prev = Path("/workspace/dronable-proto/previews/cad_static.png")
        prev.write_bytes(png)
        print(f"Wrote {prev}")
    else:
        # Fallback: plot vertices silhouette with matplotlib
        raise RuntimeError("no png from trimesh")
except Exception as e:
    print(f"trimesh preview note: {e}")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        import numpy as np
        from stl import mesh as stlmesh
        m = stlmesh.Mesh.from_file(str(stl_body))
        fig = plt.figure(figsize=(6, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.add_collection3d(Poly3DCollection(m.vectors, alpha=0.85, facecolor="#4a90d9", edgecolor="#222", linewidths=0.05))
        scale = m.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)
        ax.set_xlabel("X mm"); ax.set_ylabel("Y mm"); ax.set_zlabel("Z mm")
        ax.view_init(elev=15, azim=40)
        ax.set_title("Dronable v0 CAD (~415 mm)")
        prev = Path("/workspace/dronable-proto/previews/cad_static.png")
        fig.savefig(prev, dpi=120, bbox_inches="tight")
        plt.close()
        print(f"Wrote {prev} (matplotlib)")
    except Exception as e2:
        print(f"CAD preview failed: {e2}")

try:
    import trimesh
    mesh = trimesh.load(str(stl_body))
    glb = OUT / "dronable_v0_body.glb"
    mesh.export(str(glb))
    print(f"Wrote {glb}")
except Exception as ge:
    print(f"GLB export note: {ge}")
print("CAD build done.")
