#!/usr/bin/env python3
"""Resolve Hiwonder AiNex xacro → flat URDF → MuJoCo MJCF (kit meshes).

Keeps mujoco/dronable_v0.xml untouched (approx capsule model).
Outputs under mujoco/ainex_hiwonder/.
"""
from __future__ import annotations

import ast
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path("/workspace/dronable-proto")
VENDOR_URDF = ROOT / "cad/vendor/ainex-thorobotics/ainex_description/urdf/ainex.urdf.xacro"
MATERIALS = ROOT / "cad/vendor/ainex-thorobotics/ainex_description/urdf/materials.xacro"
OUT_DIR = ROOT / "mujoco/ainex_hiwonder"
OUT_URDF = OUT_DIR / "ainex.urdf"
OUT_MJCF_RAW = OUT_DIR / "ainex_raw.xml"
OUT_MJCF = OUT_DIR / "ainex.xml"

PROPS = {
    "motor_torque": "6",
    "motor_vec": "100",
    "damping": "0.02",
    "friction": "0.0",
    "arm_damping": "0.0",
    "arm_friction": "0.2",
    "M_PI": str(math.pi),
}

# Standing height: raise floating base so feet clear floor at q=0
ROOT_Z = "0.268"

REVOLUTE_JOINTS = [
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
    "l_sho_pitch", "l_sho_roll", "l_el_pitch", "l_el_yaw", "l_gripper",
    "r_sho_pitch", "r_sho_roll", "r_el_pitch", "r_el_yaw", "r_gripper",
    "head_pan", "head_tilt",
]


def resolve_xacro_text(text: str) -> str:
    text = re.sub(r"<xacro:property\b[^>]*/>\s*", "", text)
    text = re.sub(r"<xacro:include\b[^>]*/>\s*", "", text)

    env = {k: float(v) for k, v in PROPS.items()}
    env["M_PI"] = math.pi

    def repl(m: re.Match) -> str:
        expr = m.group(1).strip()
        if expr in PROPS and re.match(r"^[A-Za-z0-9_]+$", expr):
            return PROPS[expr]
        code = ast.parse(expr, mode="eval")
        for node in ast.walk(code):
            if not isinstance(
                node,
                (
                    ast.Expression, ast.BinOp, ast.UnaryOp, ast.Constant,
                    ast.Name, ast.Load, ast.Add, ast.Sub, ast.Mult, ast.Div,
                    ast.USub, ast.UAdd, ast.Pow, ast.Mod,
                ),
            ):
                raise ValueError(f"disallowed in expr: {type(node)}")
        val = eval(compile(code, "<xacro>", "eval"), {"__builtins__": {}}, env)
        return str(val)

    text = re.sub(r"\$\{([^}]+)\}", repl, text)
    text = text.replace("package://ainex_description/meshes/", "meshes/")
    text = text.replace(' xmlns:xacro="http://ros.org/wiki/xacro"', "")
    return text


def inject_materials(urdf_text: str) -> str:
    mats = MATERIALS.read_text()
    blocks = re.findall(r"<material\s+name=.*?</material>", mats, flags=re.S)
    m = re.match(r"(<\?xml[^>]*>\s*)?(<robot[^>]*>)", urdf_text, flags=re.S)
    if not m:
        raise RuntimeError("Could not find <robot> root")
    inject = "\n  <!-- materials from materials.xacro -->\n  " + "\n  ".join(blocks) + "\n"
    return urdf_text[: m.end()] + inject + urdf_text[m.end() :]


def strip_empty_base_link(urdf_text: str) -> str:
    urdf_text = re.sub(r'<link\s+name="base_link"\s*/>\s*', "", urdf_text)
    urdf_text = re.sub(
        r'<joint\s+name="base_link_to_body"\s+type="fixed">.*?</joint>\s*',
        "",
        urdf_text,
        flags=re.S,
    )
    return urdf_text


def inject_floating_base(urdf_text: str) -> str:
    """Add world + floating joint so MuJoCo does not weld body_link to world."""
    if 'type="floating"' in urdf_text:
        return urdf_text
    inject = f'''
  <!-- Floating base: MuJoCo welds URDF root to world otherwise -->
  <link name="world"/>
  <joint name="root" type="floating">
    <parent link="world"/>
    <child link="body_link"/>
    <origin xyz="0 0 {ROOT_Z}" rpy="0 0 0"/>
  </joint>
'''
    return re.sub(
        r'(<link\s*\n?\s*name="body_link">)',
        inject + r"\1",
        urdf_text,
        count=1,
    )


def write_flat_urdf() -> Path:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    flat = resolve_xacro_text(VENDOR_URDF.read_text())
    flat = inject_materials(flat)
    flat = strip_empty_base_link(flat)
    flat = inject_floating_base(flat)
    leftover = re.findall(r"\$\{[^}]+\}", flat)
    if leftover:
        raise RuntimeError(f"Unresolved substitutions: {leftover}")
    if "xacro:" in flat:
        raise RuntimeError("Residual xacro tags remain in flat URDF")
    OUT_URDF.write_text(flat)
    return OUT_URDF


def compile_urdf_to_mjcf(urdf_path: Path) -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(urdf_path))
    mujoco.mj_saveLastXML(str(OUT_MJCF_RAW), model)


def _ensure_asset_floor(asset: ET.Element) -> None:
    if asset.find("texture[@name='grid']") is not None:
        return
    sky = ET.SubElement(asset, "texture")
    sky.set("type", "skybox")
    sky.set("builtin", "gradient")
    sky.set("rgb1", "0.4 0.55 0.7")
    sky.set("rgb2", "0.05 0.08 0.12")
    sky.set("width", "512")
    sky.set("height", "512")
    tex = ET.SubElement(asset, "texture")
    tex.set("name", "grid")
    tex.set("type", "2d")
    tex.set("builtin", "checker")
    tex.set("rgb1", "0.25 0.25 0.28")
    tex.set("rgb2", "0.35 0.35 0.38")
    tex.set("width", "512")
    tex.set("height", "512")
    mat = ET.SubElement(asset, "material")
    mat.set("name", "grid")
    mat.set("texture", "grid")
    mat.set("texrepeat", "8 8")
    mat.set("reflectance", "0.02")


def postprocess_mjcf() -> dict:
    import mujoco
    import numpy as np

    tree = ET.parse(OUT_MJCF_RAW)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")
    # mesh file= already "meshes/…" — do not set meshdir

    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        wb_idx = list(root).index(root.find("worldbody"))
        root.insert(wb_idx, option)
    option.set("timestep", "0.002")
    option.set("gravity", "0 0 -9.81")
    option.set("integrator", "implicitfast")

    if root.find("visual") is None:
        visual = ET.Element("visual")
        glob = ET.SubElement(visual, "global")
        glob.set("offwidth", "960")
        glob.set("offheight", "720")
        root.insert(0, visual)

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    _ensure_asset_floor(asset)

    worldbody = root.find("worldbody")
    assert worldbody is not None

    if worldbody.find("geom[@name='floor']") is None:
        light = ET.Element("light")
        light.set("pos", "1 1 2.5")
        light.set("dir", "-0.3 -0.3 -1")
        light.set("diffuse", "0.9 0.9 0.85")
        worldbody.insert(0, light)
        floor = ET.Element("geom")
        floor.set("name", "floor")
        floor.set("type", "plane")
        floor.set("size", "3 3 0.05")
        floor.set("material", "grid")
        floor.set("friction", "1.4 0.1 0.01")
        worldbody.insert(1, floor)

    # Default joint damping / armature on hinges
    for j in root.iter("joint"):
        if j.get("type") == "free":
            continue
        if j.get("damping") is None:
            j.set("damping", "0.05")
        if j.get("armature") is None:
            j.set("armature", "0.01")

    for act in list(root.findall("actuator")):
        root.remove(act)
    actuator = ET.SubElement(root, "actuator")

    present = {j.get("name") for j in root.iter("joint") if j.get("name")}
    # freejoint may appear as <freejoint name="root"/> after compile
    for fj in root.iter("freejoint"):
        present.add(fj.get("name"))

    added, missing = [], []
    for jn in REVOLUTE_JOINTS:
        if jn not in present:
            missing.append(jn)
            continue
        motor = ET.SubElement(actuator, "position")
        motor.set("name", f"{jn}_pos")
        motor.set("joint", jn)
        motor.set("kp", "30")
        motor.set("dampratio", "1")
        motor.set("ctrlrange", "-2.09 2.09")
        # URDF effort=6 Nm (Hiwonder gazebo default) — optimistic vs HX stall
        motor.set("forcerange", "-6 6")
        added.append(jn)

    ET.indent(tree, space="  ")
    tree.write(OUT_MJCF, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(OUT_MJCF))
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)

    nhinge = sum(1 for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE)
    nfree = sum(1 for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE)
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    act_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]

    mujoco.mj_forward(model, data)
    zs = []
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if name == "floor":
            continue
        zs.append(float(data.geom_xpos[i][2]))

    bl_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body_link")
    stats = {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "njnt": int(model.njnt),
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "nmesh": int(model.nmesh),
        "nhinge": nhinge,
        "nfree": nfree,
        "joint_names": joint_names,
        "body_names": body_names,
        "act_names": act_names,
        "actuators_added": added,
        "actuators_missing_joints": missing,
        "total_mass_kg": float(model.body_mass.sum()),
        "body_link_mass": float(model.body_mass[bl_id]) if bl_id >= 0 else None,
        "geom_z_min": min(zs) if zs else None,
        "geom_z_max": max(zs) if zs else None,
        "qpos_has_nan": bool(np.isnan(data.qpos).any()),
        "steps_ok": True,
        "source_commit": "e8fe2a816797cf83054135160df5a82ec3596a69",
    }
    return stats


def main():
    print("1) Writing flat URDF…")
    urdf = write_flat_urdf()
    print(f"   → {urdf}")
    print("2) Compiling URDF → MJCF via MuJoCo…")
    compile_urdf_to_mjcf(urdf)
    print(f"   → {OUT_MJCF_RAW}")
    print("3) Post-process (floor, actuators) + smoke test…")
    stats = postprocess_mjcf()
    print(f"   → {OUT_MJCF}")
    summary = {
        k: stats[k]
        for k in (
            "nq", "nv", "nu", "njnt", "nbody", "ngeom", "nmesh",
            "nhinge", "nfree", "total_mass_kg", "body_link_mass",
            "geom_z_min", "geom_z_max",
            "actuators_missing_joints", "qpos_has_nan", "steps_ok",
        )
    }
    print(json.dumps(summary, indent=2))
    print("hinge joints:", stats["nhinge"], "actuators:", stats["nu"])
    print("joint names:", stats["joint_names"])
    (OUT_DIR / "smoke_stats.json").write_text(json.dumps(stats, indent=2) + "\n")
    print("DONE")


if __name__ == "__main__":
    main()
