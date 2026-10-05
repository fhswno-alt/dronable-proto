#!/usr/bin/env python3
"""Compare compiled AiNex plant mass/inertia to the Hiwonder URDF.

Read-only. Does not edit plant XML. Loads:

- main ``mujoco/ainex_hiwonder/ainex_controls_m2_145.xml``
- the PR #43 thaw tip of that same file (git show; mesh symlink only)

and the vendored URDF, which ``git hash-object`` must match the Hiwonder
blob cited in ``docs/PLANT_MASS_VS_KIT.md``.

Run from anywhere:

    python3 scripts/plant_mass_vs_kit.py
"""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

# The repo root is named mujoco/ at the top level and would shadow the
# pip package if it sits on sys.path.
_ROOT = Path(__file__).resolve().parents[1]
sys.path = [p for p in sys.path if Path(p or ".").resolve() != _ROOT]

import mujoco  # noqa: E402

ROOT = _ROOT
MAIN_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
MESH_DIR = ROOT / "mujoco" / "ainex_hiwonder" / "meshes"
VENDOR_URDF = (
    ROOT
    / "cad"
    / "vendor"
    / "ainex-thorobotics"
    / "ainex_description"
    / "urdf"
    / "ainex.urdf.xacro"
)
THAW_SPEC = "origin/cursor/plant-thaw-legs-foot-6f10:mujoco/ainex_hiwonder/ainex_controls_m2_145.xml"
EXPECTED_URDF_BLOB = "e0c4b4301ac1d9d574e7ef73033cc41f5eca1863"
G = 9.81
CROUCH_M = 0.015
FLAG_PCT = 15.0

# Arms/head from Hiwonder init_pose.yaml at the cited commit. Legs are solved
# below for the 0.015 m crouch; these joints are not part of that solve.
INIT_ARM: dict[str, float] = {
    "l_sho_pitch": 0.0,
    "l_sho_roll": 1.293,
    "l_el_pitch": -0.10,
    "l_el_yaw": -1.926,
    "l_gripper": 0.0,
    "r_sho_pitch": 0.0,
    "r_sho_roll": -1.293,
    "r_el_pitch": 0.10,
    "r_el_yaw": 1.926,
    "r_gripper": 0.0,
    "head_pan": 0.0,
    "head_tilt": 0.0,
}

LEG_JOINTS: tuple[str, ...] = (
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

# Role tags for the per-link table. Ankle pitch is its own row so it is not
# folded into the shank or the foot.
LINK_ROLE: dict[str, str] = {
    "body_link": "torso",
    "l_hip_yaw_link": "hip",
    "r_hip_yaw_link": "hip",
    "l_hip_roll_link": "hip",
    "r_hip_roll_link": "hip",
    "l_hip_pitch_link": "thigh",
    "r_hip_pitch_link": "thigh",
    "l_knee_link": "shank",
    "r_knee_link": "shank",
    "l_ank_pitch_link": "ankle",
    "r_ank_pitch_link": "ankle",
    "l_ank_roll_link": "foot",
    "r_ank_roll_link": "foot",
}

LEG_ROLES = frozenset({"hip", "thigh", "shank", "ankle", "foot"})


@dataclass(frozen=True)
class UrdfLink:
    name: str
    mass_kg: float
    com_m: tuple[float, float, float]
    inertia: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True)
class UrdfJointDyn:
    name: str
    damping: float
    friction: float


@dataclass(frozen=True)
class CompiledLink:
    name: str
    mass_kg: float
    com_m: tuple[float, float, float]
    principal_kgm2: tuple[float, float, float]
    inertia_body: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True)
class JointDyn:
    name: str
    armature: float
    damping: float
    frictionloss: float


def _mat3(rows: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    out: list[tuple[float, float, float]] = []
    for r in range(3):
        out.append((float(rows[r, 0]), float(rows[r, 1]), float(rows[r, 2])))
    return (out[0], out[1], out[2])


def _quat_to_mat(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3]))
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _principal(inertia: np.ndarray) -> tuple[float, float, float]:
    vals = np.linalg.eigvalsh(inertia)
    desc = np.sort(vals)[::-1]
    return (float(desc[0]), float(desc[1]), float(desc[2]))


def md5_file(path: Path) -> str:
    digest = hashlib.md5()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def git_blob(path: Path) -> str:
    raw = subprocess.check_output(["git", "hash-object", str(path)], cwd=ROOT)
    return raw.decode().strip()


def _xacro_props(tree: ET.ElementTree) -> dict[str, float]:
    props: dict[str, float] = {}
    for el in tree.iter():
        local = el.tag.split("}")[-1]
        if local != "property":
            continue
        name = el.get("name")
        raw = el.get("value")
        if name is None or raw is None:
            continue
        try:
            props[name] = float(raw)
        except ValueError:
            continue
    return props


def _subst_float(raw: str | None, props: dict[str, float]) -> float:
    if raw is None:
        return float("nan")
    text = raw.strip()
    if text.startswith("${") and text.endswith("}"):
        key = text[2:-1].strip()
        if key not in props:
            raise SystemExit(f"unresolved xacro property {key}")
        return props[key]
    return float(text)


def parse_urdf(path: Path) -> tuple[dict[str, UrdfLink], dict[str, UrdfJointDyn]]:
    tree = ET.parse(path)
    props = _xacro_props(tree)
    links: dict[str, UrdfLink] = {}
    for link in tree.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if name is None or inertial is None:
            continue
        origin = inertial.find("origin")
        mass_el = inertial.find("mass")
        inertia_el = inertial.find("inertia")
        if origin is None or mass_el is None or inertia_el is None:
            continue
        xyz = origin.get("xyz")
        if xyz is None:
            continue
        com = tuple(float(v) for v in xyz.split())
        if len(com) != 3:
            raise SystemExit(f"bad com on {name}")
        ixx = float(inertia_el.get("ixx") or "nan")
        ixy = float(inertia_el.get("ixy") or "nan")
        ixz = float(inertia_el.get("ixz") or "nan")
        iyy = float(inertia_el.get("iyy") or "nan")
        iyz = float(inertia_el.get("iyz") or "nan")
        izz = float(inertia_el.get("izz") or "nan")
        matrix = (
            (ixx, ixy, ixz),
            (ixy, iyy, iyz),
            (ixz, iyz, izz),
        )
        links[name] = UrdfLink(
            name=name,
            mass_kg=float(mass_el.get("value") or "nan"),
            com_m=(com[0], com[1], com[2]),
            inertia=matrix,
        )
    joints: dict[str, UrdfJointDyn] = {}
    for joint in tree.findall("joint"):
        name = joint.get("name")
        dyn = joint.find("dynamics")
        if name is None or dyn is None:
            continue
        joints[name] = UrdfJointDyn(
            name=name,
            damping=_subst_float(dyn.get("damping"), props),
            friction=_subst_float(dyn.get("friction"), props),
        )
    return links, joints


def load_model(xml_path: Path) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(xml_path))


def compiled_links(model: mujoco.MjModel) -> dict[str, CompiledLink]:
    out: dict[str, CompiledLink] = {}
    for bid in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
        if name is None or name == "world":
            continue
        principal = np.array(model.body_inertia[bid], dtype=np.float64)
        rot = _quat_to_mat(np.array(model.body_iquat[bid], dtype=np.float64))
        inertia = rot @ np.diag(principal) @ rot.T
        com = model.body_ipos[bid]
        out[name] = CompiledLink(
            name=name,
            mass_kg=float(model.body_mass[bid]),
            com_m=(float(com[0]), float(com[1]), float(com[2])),
            principal_kgm2=(float(principal[0]), float(principal[1]), float(principal[2])),
            inertia_body=_mat3(inertia),
        )
    return out


def joint_dyn(model: mujoco.MjModel, names: Sequence[str]) -> list[JointDyn]:
    rows: list[JointDyn] = []
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise SystemExit(f"missing joint {name}")
        dof = int(model.jnt_dofadr[jid])
        rows.append(
            JointDyn(
                name=name,
                armature=float(model.dof_armature[dof]),
                damping=float(model.dof_damping[dof]),
                frictionloss=float(model.dof_frictionloss[dof]),
            )
        )
    return rows


def reflected_link_inertia(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> tuple[float, float, float]:
    """Return (M_ii, armature, M_ii - armature) at the current pose.

    M_ii is the joint-space inertia diagonal. With every other acceleration
    at zero it is the reflected inertia of the distal subtree about the
    joint axis, plus MuJoCo armature.
    """
    mujoco.mj_forward(model, data)
    nv = int(model.nv)
    full = np.zeros((nv, nv), dtype=np.float64)
    # MuJoCo 3.14: mj_fullM(model, data, dst). Sparse inertia is data.M.
    mujoco.mj_fullM(model, data, full)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    dof = int(model.jnt_dofadr[jid])
    total = float(full[dof, dof])
    arm = float(model.dof_armature[dof])
    return total, arm, total - arm


def _body_id(model: mujoco.MjModel, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise SystemExit(f"missing body {name}")
    return int(bid)


def _geom_id(model: mujoco.MjModel, name: str) -> int:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if gid < 0:
        raise SystemExit(f"missing geom {name}")
    return int(gid)


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise SystemExit(f"missing joint {name}")
    return int(jid)


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, angles: dict[str, float]) -> None:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[3] = 1.0
    data.qpos[2] = 0.268
    for name, value in angles.items():
        jid = _joint_id(model, name)
        data.qpos[int(model.jnt_qposadr[jid])] = value
    mujoco.mj_forward(model, data)


def leg_pitch_angles(flex: float, fwd: float) -> dict[str, float]:
    """Flat-foot sagittal crouch. Signs follow ``to_joint`` in ainex_stand_then_step."""
    return {
        "l_hip_pitch": -fwd,
        "l_knee": flex,
        "l_ank_pitch": -fwd + flex,
        "r_hip_pitch": fwd,
        "r_knee": -flex,
        "r_ank_pitch": fwd + (-flex),
    }


def sole_metrics(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> tuple[float, float, float, float]:
    """Foot-contact center relative to body origin, and sole bottom Z in world.

    Returns (dx, dy, sole_z, foot_z_axis_world_z). Sole Z assumes a level box:
    contact-center Z minus the box half-height.
    """
    body = data.xpos[_body_id(model, "body_link")]
    gid = _geom_id(model, f"{side}_foot_contact")
    center = data.geom_xpos[gid]
    half_z = float(model.geom_size[gid][2])
    z_axis = float(data.geom_xmat[gid][8])
    return (
        float(center[0] - body[0]),
        float(center[1] - body[1]),
        float(center[2] - half_z),
        z_axis,
    )


def solve_crouch(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    straight = dict(INIT_ARM)
    set_pose(model, data, straight)
    dx0, dy0, sole0, _up0 = sole_metrics(model, data, "r")
    body0 = float(data.xpos[_body_id(model, "body_link")][2])
    drop0 = body0 - sole0

    def cost(flex: float, fwd: float) -> float:
        angles = dict(INIT_ARM)
        angles.update(leg_pitch_angles(flex, fwd))
        set_pose(model, data, angles)
        dx, dy, sole, up = sole_metrics(model, data, "r")
        body_z = float(data.xpos[_body_id(model, "body_link")][2])
        drop = body_z - sole
        return (
            (drop - (drop0 - CROUCH_M)) ** 2
            + (dx - dx0) ** 2
            + (dy - dy0) ** 2
            + (1.0 - up) ** 2
        )

    best_c = float("inf")
    best_flex = 0.0
    best_fwd = 0.0
    for flex in np.linspace(0.0, 1.4, 29):
        for fwd in np.linspace(-0.6, 0.6, 25):
            c = cost(float(flex), float(fwd))
            if c < best_c:
                best_c = c
                best_flex = float(flex)
                best_fwd = float(fwd)
    span_f = 0.08
    span_w = 0.08
    for _ in range(6):
        for flex in np.linspace(best_flex - span_f, best_flex + span_f, 17):
            for fwd in np.linspace(best_fwd - span_w, best_fwd + span_w, 17):
                c = cost(float(flex), float(fwd))
                if c < best_c:
                    best_c = c
                    best_flex = float(flex)
                    best_fwd = float(fwd)
        span_f *= 0.45
        span_w *= 0.45
    angles = dict(INIT_ARM)
    angles.update(leg_pitch_angles(best_flex, best_fwd))
    angles["_flex"] = best_flex
    angles["_fwd"] = best_fwd
    angles["_cost"] = best_c
    return angles


def apply_roll(angles: dict[str, float], side: str, hip_roll: float, ank_roll: float) -> dict[str, float]:
    out = {k: v for k, v in angles.items() if not k.startswith("_")}
    prefix = "r_" if side == "r" else "l_"
    out[f"{prefix}hip_roll"] = hip_roll
    out[f"{prefix}ank_roll"] = ank_roll
    return out


def solve_ank_for_level(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base: dict[str, float],
    side: str,
    hip_roll: float,
) -> float:
    best_up = -2.0
    best_ank = hip_roll
    for ank in np.linspace(-1.5, 1.5, 61):
        set_pose(model, data, apply_roll(base, side, hip_roll, float(ank)))
        _dx, _dy, _sole, up = sole_metrics(model, data, side)
        if up > best_up:
            best_up = up
            best_ank = float(ank)
    span = 0.08
    for _ in range(5):
        for ank in np.linspace(best_ank - span, best_ank + span, 21):
            set_pose(model, data, apply_roll(base, side, hip_roll, float(ank)))
            _dx, _dy, _sole, up = sole_metrics(model, data, side)
            if up > best_up:
                best_up = up
                best_ank = float(ank)
        span *= 0.4
    return best_ank


def foot_y_offset(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> float:
    _dx, dy, _sole, _up = sole_metrics(model, data, side)
    return dy


def solve_sway(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    crouch: dict[str, float],
    side: str,
    sway_m: float,
) -> dict[str, float]:
    """Shift the pelvis ``sway_m`` toward the stance foot, foot kept level.

    Positive sway reduces the lateral gap between body origin and that foot.
    """
    set_pose(model, data, {k: v for k, v in crouch.items() if not k.startswith("_")})
    y0 = foot_y_offset(model, data, side)
    # Right foot lives at negative Y. Moving it toward the body increases Y.
    # Left foot lives at positive Y. Moving it toward the body decreases Y.
    target = y0 + sway_m if side == "r" else y0 - sway_m

    def err(hip_roll: float) -> tuple[float, float]:
        ank = solve_ank_for_level(model, data, crouch, side, hip_roll)
        set_pose(model, data, apply_roll(crouch, side, hip_roll, ank))
        y = foot_y_offset(model, data, side)
        return (y - target) ** 2, ank

    best = float("inf")
    best_hip = 0.0
    best_ank = 0.0
    for hip in np.linspace(-1.2, 1.2, 49):
        e, ank = err(float(hip))
        if e < best:
            best = e
            best_hip = float(hip)
            best_ank = ank
    span = 0.06
    for _ in range(6):
        for hip in np.linspace(best_hip - span, best_hip + span, 21):
            e, ank = err(float(hip))
            if e < best:
                best = e
                best_hip = float(hip)
                best_ank = ank
        span *= 0.45
    pose = apply_roll(crouch, side, best_hip, best_ank)
    pose["_hip_roll"] = best_hip
    pose["_ank_roll"] = best_ank
    pose["_sway_err_m2"] = best
    return pose


def solve_step_angle(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    crouch: dict[str, float],
    step_m: float,
) -> dict[str, float]:
    """Hip-pitch amplitude that moves the right foot +step_m in X.

    Knee flex and ankle pitch keep the sole height and a level foot.
    Amplitude is the change in ``r_hip_pitch`` from the crouch pose.
    """
    base = {k: v for k, v in crouch.items() if not k.startswith("_")}
    set_pose(model, data, base)
    dx0, _dy0, sole0, _up0 = sole_metrics(model, data, "r")
    hip0 = float(base["r_hip_pitch"])
    knee0 = float(base["r_knee"])

    def cost(d_hip: float, d_knee: float) -> float:
        angles = dict(base)
        hip = hip0 + d_hip
        knee = knee0 + d_knee
        angles["r_hip_pitch"] = hip
        angles["r_knee"] = knee
        # r_ank axis is +Y while hip and knee axes are -Y, so flat foot is
        # q_ank = q_hip + q_knee (see leg_pitch_angles).
        angles["r_ank_pitch"] = hip + knee
        set_pose(model, data, angles)
        dx, _dy, sole, up = sole_metrics(model, data, "r")
        return (dx - (dx0 + step_m)) ** 2 + (sole - sole0) ** 2 + (1.0 - up) ** 2

    best = float("inf")
    best_h = 0.0
    best_k = 0.0
    for d_hip in np.linspace(-0.5, 0.5, 41):
        for d_knee in np.linspace(-0.5, 0.5, 41):
            c = cost(float(d_hip), float(d_knee))
            if c < best:
                best = c
                best_h = float(d_hip)
                best_k = float(d_knee)
    span_h = 0.04
    span_k = 0.04
    for _ in range(5):
        for d_hip in np.linspace(best_h - span_h, best_h + span_h, 15):
            for d_knee in np.linspace(best_k - span_k, best_k + span_k, 15):
                c = cost(float(d_hip), float(d_knee))
                if c < best:
                    best = c
                    best_h = float(d_hip)
                    best_k = float(d_knee)
        span_h *= 0.45
        span_k *= 0.45
    return {
        "d_hip_rad": best_h,
        "d_knee_rad": best_k,
        "cost": best,
        "hip_from_rad": hip0,
    }


def static_hip_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    side: str,
    which: str,
    total_mass: float,
) -> dict[str, float]:
    """Quasi-static stance-leg hip torque from GRF plus distal link weight.

    CoP is the foot-contact box center. GRF is (0, 0, M g) with M the full
    robot mass (single support, no other contact). Distal bodies are the
    subtree of the joint, including the joint's own child link.
    """
    prefix = f"{side}_"
    if which == "roll":
        joint = f"{prefix}hip_roll"
        distal = (
            f"{prefix}hip_roll_link",
            f"{prefix}hip_pitch_link",
            f"{prefix}knee_link",
            f"{prefix}ank_pitch_link",
            f"{prefix}ank_roll_link",
        )
    elif which == "pitch":
        joint = f"{prefix}hip_pitch"
        distal = (
            f"{prefix}hip_pitch_link",
            f"{prefix}knee_link",
            f"{prefix}ank_pitch_link",
            f"{prefix}ank_roll_link",
        )
    else:
        raise SystemExit(which)
    jid = _joint_id(model, joint)
    anchor = np.array(data.xanchor[jid], dtype=np.float64)
    axis = np.array(data.xaxis[jid], dtype=np.float64)
    gid = _geom_id(model, f"{side}_foot_contact")
    cop = np.array(data.geom_xpos[gid], dtype=np.float64)
    grf = np.array([0.0, 0.0, total_mass * G], dtype=np.float64)
    tau = np.cross(cop - anchor, grf)
    distal_mass = 0.0
    for name in distal:
        bid = _body_id(model, name)
        mass = float(model.body_mass[bid])
        distal_mass += mass
        com = np.array(data.xipos[bid], dtype=np.float64)
        weight = np.array([0.0, 0.0, -mass * G], dtype=np.float64)
        tau = tau + np.cross(com - anchor, weight)
    signed = float(np.dot(tau, axis))
    return {
        "tau_nm": signed,
        "abs_nm": abs(signed),
        "distal_mass_kg": distal_mass,
        "cop_minus_hip_m": (
            float(cop[0] - anchor[0]),
            float(cop[1] - anchor[1]),
            float(cop[2] - anchor[2]),
        ),
    }


def pct(sim: float, kit: float) -> float:
    if abs(kit) < 1e-15:
        return float("nan")
    return 100.0 * (sim - kit) / kit


def inertia_np(rows: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]) -> np.ndarray:
    return np.array(rows, dtype=np.float64)


def materialize_thaw() -> Path:
    xml = subprocess.check_output(["git", "show", THAW_SPEC], cwd=ROOT)
    tmp = Path(tempfile.mkdtemp(prefix="plant-thaw-"))
    meshes = tmp / "meshes"
    meshes.symlink_to(MESH_DIR)
    path = tmp / "ainex_controls_m2_145.xml"
    path.write_bytes(xml)
    return path


def robot_mass(links: dict[str, CompiledLink]) -> float:
    return float(sum(link.mass_kg for link in links.values()))


def leg_mass(links: dict[str, CompiledLink]) -> float:
    total = 0.0
    for name, link in links.items():
        if LINK_ROLE.get(name) in LEG_ROLES:
            total += link.mass_kg
    return total


def main() -> None:
    blob = git_blob(VENDOR_URDF)
    if blob != EXPECTED_URDF_BLOB:
        raise SystemExit(f"vendor URDF blob {blob} != Hiwonder {EXPECTED_URDF_BLOB}")
    urdf_links, urdf_joints = parse_urdf(VENDOR_URDF)
    thaw_xml = materialize_thaw()
    plants: dict[str, Path] = {"main": MAIN_XML, "thaw": thaw_xml}
    compiled: dict[str, dict[str, CompiledLink]] = {}
    dyn: dict[str, list[JointDyn]] = {}
    md5s: dict[str, str] = {}
    for label, path in plants.items():
        md5s[label] = md5_file(path)
        model = load_model(path)
        compiled[label] = compiled_links(model)
        dyn[label] = joint_dyn(model, LEG_JOINTS)

    main_links = compiled["main"]
    thaw_links = compiled["thaw"]
    link_rows: list[dict[str, float | str | bool | tuple[float, float, float]]] = []
    flags: list[str] = []
    for name in sorted(urdf_links):
        kit = urdf_links[name]
        if name not in main_links:
            raise SystemExit(f"plant missing {name}")
        sim = main_links[name]
        thaw = thaw_links[name]
        kit_i = inertia_np(kit.inertia)
        sim_i = inertia_np(sim.inertia_body)
        kit_p = _principal(kit_i)
        sim_p = _principal(sim_i)
        # MuJoCo stores principal moments in its own axis order. Compare sorted.
        sim_stored = tuple(sorted(sim.principal_kgm2, reverse=True))
        mass_delta = sim.mass_kg - kit.mass_kg
        mass_pct = pct(sim.mass_kg, kit.mass_kg)
        princ_pct = tuple(pct(sim_p[i], kit_p[i]) for i in range(3))
        tensor_max = float(np.max(np.abs(sim_i - kit_i)))
        com_delta = tuple(sim.com_m[i] - kit.com_m[i] for i in range(3))
        role = LINK_ROLE.get(name, "other")
        heavier = mass_pct > FLAG_PCT
        inertia_high = any(p > FLAG_PCT for p in princ_pct)
        if role in LEG_ROLES and (heavier or inertia_high):
            flags.append(name)
        thaw_mass_delta = abs(thaw.mass_kg - sim.mass_kg)
        link_rows.append(
            {
                "name": name,
                "role": role,
                "sim_mass_kg": sim.mass_kg,
                "kit_mass_kg": kit.mass_kg,
                "mass_delta_kg": mass_delta,
                "mass_pct": mass_pct,
                "sim_com_m": sim.com_m,
                "kit_com_m": kit.com_m,
                "com_delta_m": com_delta,
                "sim_principal_kgm2": sim_p,
                "kit_principal_kgm2": kit_p,
                "principal_pct": princ_pct,
                "sim_principal_stored_kgm2": sim_stored,
                "tensor_max_abs": tensor_max,
                "thaw_mass_delta_kg": thaw_mass_delta,
                "flag": role in LEG_ROLES and (heavier or inertia_high),
            }
        )

    # Dynamics + reflected inertia on the main plant (thaw armature is checked equal).
    model = load_model(MAIN_XML)
    data = mujoco.MjData(model)
    crouch = solve_crouch(model, data)
    crouch_pose = {k: v for k, v in crouch.items() if not k.startswith("_")}
    set_pose(model, data, crouch_pose)
    dx, dy, sole, up = sole_metrics(model, data, "r")
    dx_l, dy_l, sole_l, up_l = sole_metrics(model, data, "l")
    body_z = float(data.xpos[_body_id(model, "body_link")][2])
    set_pose(model, data, dict(INIT_ARM))
    _dx_s, _dy_s, sole_s, _up_s = sole_metrics(model, data, "r")
    body_s = float(data.xpos[_body_id(model, "body_link")][2])
    straight_drop = body_s - sole_s
    crouch_drop = body_z - sole

    total = robot_mass(main_links)
    legs = leg_mass(main_links)
    reflected: dict[str, dict[str, float]] = {}
    for pose_name, pose in (("straight_arms_down", dict(INIT_ARM)), ("crouch", crouch_pose)):
        set_pose(model, data, pose)
        for joint in ("r_hip_roll", "r_hip_pitch", "l_hip_roll", "l_hip_pitch"):
            total_i, arm, link_i = reflected_link_inertia(model, data, joint)
            reflected[f"{pose_name}:{joint}"] = {
                "M_ii": total_i,
                "armature": arm,
                "I_link": link_i,
                "arm_over_link": arm / link_i if link_i else float("nan"),
            }

    step = solve_step_angle(model, data, crouch, 0.02)
    sway_poses: dict[str, dict[str, float]] = {}
    sway_angles: dict[str, dict[str, float]] = {}
    for sway in (0.02, 0.04):
        pose = solve_sway(model, data, crouch, "r", sway)
        key = f"{sway:.2f}"
        sway_poses[key] = pose
        clean = {k: v for k, v in pose.items() if not k.startswith("_")}
        set_pose(model, data, clean)
        _dx, y, _sole, up_s = sole_metrics(model, data, "r")
        set_pose(model, data, crouch_pose)
        y_ref = foot_y_offset(model, data, "r")
        sway_angles[key] = {
            "hip_roll_rad": float(pose["_hip_roll"]),
            "ank_roll_rad": float(pose["_ank_roll"]),
            "y_from_m": y_ref,
            "y_m": y,
            "gap_closed_m": y - y_ref,
            "foot_up_z": up_s,
            "err_m": math.sqrt(float(pose["_sway_err_m2"])),
        }

    static: dict[str, dict[str, float]] = {}
    for sway_key, pose in sway_poses.items():
        clean = {k: v for k, v in pose.items() if not k.startswith("_")}
        set_pose(model, data, clean)
        for which in ("roll", "pitch"):
            tor = static_hip_torque(model, data, "r", which, total)
            static[f"{sway_key}:{which}"] = tor

    # Zero-sway reference (symmetric crouch, right foot still the support).
    set_pose(model, data, crouch_pose)
    for which in ("roll", "pitch"):
        static[f"0.00:{which}"] = static_hip_torque(model, data, "r", which, total)

    periods_s = (0.300, 0.400, 0.500, 0.600)
    a_pitch = abs(float(step["d_hip_rad"]))
    arm_torque: list[dict[str, float]] = []
    for period in periods_s:
        omega2 = (2.0 * math.pi / period) ** 2
        for sway_key, ang in sway_angles.items():
            a_roll = abs(float(ang["hip_roll_rad"]))
            # Reflected link inertia at the crouch pose (symmetric). Sway changes
            # it only slightly; the crouch value is the one quoted in the ratio.
            i_roll = reflected["crouch:r_hip_roll"]["I_link"]
            i_pitch = reflected["crouch:r_hip_pitch"]["I_link"]
            arm = reflected["crouch:r_hip_roll"]["armature"]
            tau_arm_roll = arm * a_roll * omega2
            tau_arm_pitch = arm * a_pitch * omega2
            tau_link_roll = i_roll * a_roll * omega2
            tau_link_pitch = i_pitch * a_pitch * omega2
            st_roll = static[f"{sway_key}:roll"]["abs_nm"]
            st_pitch = static[f"{sway_key}:pitch"]["abs_nm"]
            arm_torque.append(
                {
                    "period_s": period,
                    "sway_m": float(sway_key),
                    "a_roll_rad": a_roll,
                    "a_pitch_rad": a_pitch,
                    "alpha_roll": a_roll * omega2,
                    "alpha_pitch": a_pitch * omega2,
                    "tau_arm_roll_nm": tau_arm_roll,
                    "tau_arm_pitch_nm": tau_arm_pitch,
                    "tau_link_roll_nm": tau_link_roll,
                    "tau_link_pitch_nm": tau_link_pitch,
                    "tau_static_roll_nm": st_roll,
                    "tau_static_pitch_nm": st_pitch,
                    "tau_sum_roll_nm": st_roll + tau_arm_roll,
                    "tau_sum_pitch_nm": st_pitch + tau_arm_pitch,
                }
            )

    dyn_rows = []
    thaw_by_name = {row.name: row for row in dyn["thaw"]}
    for row in dyn["main"]:
        other = thaw_by_name[row.name]
        urdf = urdf_joints.get(row.name)
        dyn_rows.append(
            {
                "name": row.name,
                "main_armature": row.armature,
                "main_damping": row.damping,
                "main_frictionloss": row.frictionloss,
                "thaw_armature": other.armature,
                "thaw_damping": other.damping,
                "thaw_frictionloss": other.frictionloss,
                "urdf_damping": None if urdf is None else urdf.damping,
                "urdf_friction": None if urdf is None else urdf.friction,
            }
        )

    kit_mass = float(sum(link.mass_kg for link in urdf_links.values()))
    report: dict[str, object] = {
        "mujoco": mujoco.mj_versionString(),
        "md5": md5s,
        "urdf_blob": blob,
        "n_urdf_links": len(urdf_links),
        "n_plant_links_main": len(main_links),
        "mass_main_kg": total,
        "mass_thaw_kg": robot_mass(thaw_links),
        "mass_urdf_kg": kit_mass,
        "leg_mass_main_kg": legs,
        "leg_mass_fraction": legs / total,
        "one_leg_fraction_r": (
            sum(main_links[n].mass_kg for n in main_links if n.startswith("r_") and LINK_ROLE.get(n) in LEG_ROLES)
            / total
        ),
        "flags": flags,
        "crouch": {
            "flex_rad": crouch["_flex"],
            "fwd_rad": crouch["_fwd"],
            "cost": crouch["_cost"],
            "straight_body_minus_sole_m": straight_drop,
            "crouch_body_minus_sole_m": crouch_drop,
            "drop_m": straight_drop - crouch_drop,
            "r_foot_dx_m": dx,
            "r_foot_dy_m": dy,
            "l_foot_dy_m": dy_l,
            "r_foot_up_z": up,
            "l_foot_up_z": up_l,
            "sole_r": sole,
            "sole_l": sole_l,
            "angles": {k: crouch_pose[k] for k in ("r_hip_pitch", "r_knee", "r_ank_pitch", "l_hip_pitch", "l_knee", "l_ank_pitch")},
        },
        "step": step,
        "sway": sway_angles,
        "static": {
            k: {
                "abs_nm": v["abs_nm"],
                "tau_nm": v["tau_nm"],
                "distal_mass_kg": v["distal_mass_kg"],
                "cop_minus_hip_m": v["cop_minus_hip_m"],
            }
            for k, v in static.items()
        },
        "reflected": reflected,
        "arm_torque": arm_torque,
        "joints": dyn_rows,
        "links": link_rows,
        "max_tensor_abs": max(float(row["tensor_max_abs"]) for row in link_rows),
        "max_abs_mass_delta_main_thaw": max(float(row["thaw_mass_delta_kg"]) for row in link_rows),
        "max_abs_com_delta": 0.0,
    }
    max_com = 0.0
    for row in link_rows:
        com_delta = row["com_delta_m"]
        if not isinstance(com_delta, tuple):
            raise SystemExit("com_delta_m must be a 3-tuple")
        for component in com_delta:
            max_com = max(max_com, abs(float(component)))
    report["max_abs_com_delta"] = max_com
    json.dump(report, sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
