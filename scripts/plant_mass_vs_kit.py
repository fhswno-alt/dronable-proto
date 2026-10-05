#!/usr/bin/env python3
"""Compare compiled AiNex plant mass/inertia to the Hiwonder URDF.

Read-only. Does not edit plant XML. Loads:

- main ``mujoco/ainex_hiwonder/ainex_controls_m2_145.xml``
- the PR #43 thaw tip of that same file (git show; mesh symlink only)

and the vendored URDF, which ``git hash-object`` must match the Hiwonder
blob cited in ``docs/PLANT_MASS_VS_KIT.md``. The same run prints the knee
and ankle-pitch static torques and the 2 cm knee-lift inertia term.

Run from anywhere:

    python3 scripts/plant_mass_vs_kit.py
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
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


@dataclass(frozen=True)
class LegLengths:
    """Analytic thigh / calf / sole chain. Metres and radians."""

    thigh_m: float
    calf_m: float
    ankle_m: float
    hip_pitch_offset_m: float
    hip_offset_angle_rad: float


# OP3 link lengths at ROBOTIS-OP3 3bc2bd51, op3_kinematics_dynamics.cpp.
# Thigh is the knee-link xz distance (relative pos 0.0001, 0, -0.11015).
# Calf is |ankle-pitch z| = 0.110. Ankle is |leg-end z| = 0.0305.
# hip_offset_angle_rad_ = atan2(0.0001, 0.11015); hip_pitch_offset_m_ = 0.0001.
OP3_LENGTHS = LegLengths(
    thigh_m=math.hypot(0.0001, 0.11015),
    calf_m=0.110,
    ankle_m=0.0305,
    hip_pitch_offset_m=0.0001,
    hip_offset_angle_rad=math.atan2(0.0001, 0.11015),
)

# param.yaml after loadWalkingParam. Degrees are converted by DEGREE2RADIAN
# (M_PI/180) in ROBOTIS-Math robotis_math_base.h.
OP3_X_OFFSET_M = -0.020
OP3_Y_OFFSET_M = 0.015
OP3_Z_OFFSET_M = 0.035
OP3_HIP_PITCH_OFFSET_RAD = 7.0 * math.pi / 180.0

# +0.34 rad knee on the stand that measured it: steer_walk.apply_frozen_forward_gait
# at d2e393b sets HIP_BIAS_FWD = 0.06 and KNEE_STANCE = 0.40, then lipm_gait adds
# 0.34 rad of knee and sets the ankle to hip+knee. gait_targets also sets hip roll.
KNEE_STAND_FLEX_RAD = 0.40
HIP_BIAS_FWD_RAD = 0.06
KNEE_EXTRA_FLEX_RAD = 0.34
STAND_HIP_ROLL_L_RAD = -0.05
STAND_HIP_ROLL_R_RAD = 0.05

_LEG_R: tuple[str, ...] = ("r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll")
_LEG_L: tuple[str, ...] = ("l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll")
STEP_HEIGHT_M = 0.02
RAIL_NM = 2.45
# HX-35H product page: static maximum 35 kg·cm at 11.1 V. 1 kgf·cm = 0.0980665 N·m,
# so 35 kgf·cm = 3.4323275 N·m. The stall bound uses 3.43 N·m, that figure to 0.01 N·m.
STALL_NM = 3.43
KGF_CM_TO_NM = 0.0980665
HX35H_STATIC_KGF_CM = 35.0
# walking_param.yaml period_time is 400 ms. move(1) replaces it with dsp_ratio[0],
# whose period is 300 ms. Both use the yaml crouch and z_move_amplitude.
KIT_YAML_PERIOD_S = 0.400
KIT_MOVE1_PERIOD_S = 0.300
KIT_PERIOD_S = KIT_MOVE1_PERIOD_S
KIT_DSP = 0.2
KIT_TRAJECTORY_STEP_S = 0.008
KIT_SERVO_CYCLE_S = 0.02
# walking_param_sim.yaml is not the real-kit preset.
KIT_SIM_PERIOD_S = 1.500
KIT_Y_SWAP_M = 0.02
KIT_Z_MOVE_M = 0.02
KIT_Z_OFFSET_M = 0.025
KIT_X_OFFSET_M = 0.0
KIT_Y_OFFSET_M = -0.005
KIT_HIP_PITCH_DEG = 15.0
KIT_X_AMP_CAP_M = 0.02
LIFT_GRID_M = (0.005, 0.010, 0.015, 0.020)


def _rot_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _sign(value: float) -> float:
    return 1.0 if value >= 0.0 else -1.0


def lengths_from_model(model: mujoco.MjModel) -> LegLengths:
    """Same three lengths ``lengths_from_model`` reads on the OP3 IK port."""
    knee = _body_id(model, "r_knee_link")
    ank = _body_id(model, "r_ank_pitch_link")
    foot = _geom_id(model, "r_foot_contact")
    knee_pos = np.asarray(model.body_pos[knee], dtype=np.float64)
    calf = abs(float(model.body_pos[ank][2]))
    sole_z = float(model.geom_pos[foot][2] - model.geom_size[foot][2])
    ankle = abs(sole_z)
    hip_x = float(knee_pos[0])
    return LegLengths(
        thigh_m=float(math.hypot(hip_x, float(knee_pos[2]))),
        calf_m=calf,
        ankle_m=ankle,
        hip_pitch_offset_m=hip_x,
        hip_offset_angle_rad=math.atan2(hip_x, abs(float(knee_pos[2]))),
    )


def ik_leg(
    lengths: LegLengths,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    """``OP3KinematicsDynamics::calcInverseKinematicsForLeg``.

    Six angles before the joint-direction multiply: hip yaw, hip roll,
    hip pitch, knee, ankle pitch, ankle roll.
    """
    r06 = _rpy(roll, pitch, yaw)
    p06 = np.array([x, y, z], dtype=np.float64) + lengths.ankle_m * r06[:, 2]
    p60 = -r06.T @ p06
    out = np.zeros(6, dtype=np.float64)
    out[5] = math.atan2(float(p60[1]), float(p60[2]))
    r05 = r06 @ _rot_x(-float(out[5]))
    out[0] = math.atan2(-float(r05[0, 1]), float(r05[1, 1]))
    p03 = _rot_z(float(out[0])) @ np.array([lengths.hip_pitch_offset_m, 0.0, 0.0], dtype=np.float64)
    p36 = p06 - p03
    dist = float(np.linalg.norm(p36))
    if dist < 1e-8:
        raise SystemExit("OP3 IK: zero hip-to-ankle distance")
    cos_knee = (lengths.thigh_m ** 2 + lengths.calf_m ** 2 - dist ** 2) / (2.0 * lengths.thigh_m * lengths.calf_m)
    if abs(cos_knee) > 1.0 + 1e-6:
        raise SystemExit(f"OP3 IK: knee cosine {cos_knee} out of range")
    cos_knee = max(-1.0, min(1.0, cos_knee))
    out[3] = -math.acos(cos_knee) + math.pi
    sin_arg = lengths.thigh_m * math.sin(math.pi - float(out[3])) / dist
    if abs(sin_arg) > 1.0 + 1e-6:
        raise SystemExit(f"OP3 IK: knee sine {sin_arg} out of range")
    sin_arg = max(-1.0, min(1.0, sin_arg))
    alpha = math.asin(sin_arg)
    p63 = -r06.T @ p36
    horiz = math.sqrt(float(p63[1]) ** 2 + float(p63[2]) ** 2)
    out[4] = -math.atan2(float(p63[0]), _sign(float(p63[2])) * horiz) - alpha
    r13 = _rot_z(-float(out[0])) @ r05 @ _rot_y(-(float(out[4]) + float(out[3])))
    out[1] = math.atan2(float(r13[2, 1]), float(r13[1, 1]))
    out[2] = math.atan2(float(r13[0, 2]), float(r13[0, 0]))
    out[2] += lengths.hip_offset_angle_rad
    out[3] -= lengths.hip_offset_angle_rad
    if not np.all(np.isfinite(out)):
        raise SystemExit("OP3 IK: non-finite angle")
    return out


def _axis_sum(model: mujoco.MjModel, joint: str) -> float:
    jid = _joint_id(model, joint)
    axis = np.asarray(model.jnt_axis[jid], dtype=np.float64)
    return float(axis[0] + axis[1] + axis[2])


def _apply_ik_leg(
    raw: np.ndarray,
    directions: dict[str, float],
    names: tuple[str, ...],
    hip_pitch_offset: float,
) -> dict[str, float]:
    """Direction multiply, then the walking module's hip-pitch offset."""
    out: dict[str, float] = {}
    for i, name in enumerate(names):
        out[name] = float(raw[i]) * directions[name]
    hip_name = names[2]
    out[hip_name] = out[hip_name] - directions[hip_name] * hip_pitch_offset
    return out


def op3_endpoints(lengths: LegLengths, z_offset: float, x_offset: float, y_offset: float) -> tuple[np.ndarray, np.ndarray]:
    """Foot targets with the gait sinusoids at zero.

    ``computeLegAngle`` uses ``ep_z = z_offset - leg_length``. x and y are the
    init offsets. Roll, pitch, and yaw offsets are 0 in param.yaml.
    """
    leg = lengths.thigh_m + lengths.calf_m + lengths.ankle_m
    right = np.array([x_offset, -y_offset / 2.0, z_offset - leg, 0.0, 0.0, 0.0], dtype=np.float64)
    left = np.array([x_offset, y_offset / 2.0, z_offset - leg, 0.0, 0.0, 0.0], dtype=np.float64)
    return right, left


def op3_pose(
    model: mujoco.MjModel,
    lengths: LegLengths,
    z_offset: float,
    x_offset: float,
    y_offset: float,
    hip_pitch_offset: float,
) -> dict[str, float]:
    directions = {name: _axis_sum(model, name) for name in _LEG_R + _LEG_L}
    right, left = op3_endpoints(lengths, z_offset, x_offset, y_offset)
    raw_r = ik_leg(lengths, float(right[0]), float(right[1]), float(right[2]), float(right[3]), float(right[4]), float(right[5]))
    raw_l = ik_leg(lengths, float(left[0]), float(left[1]), float(left[2]), float(left[3]), float(left[4]), float(left[5]))
    angles = dict(INIT_ARM)
    angles.update(_apply_ik_leg(raw_r, directions, _LEG_R, hip_pitch_offset))
    angles.update(_apply_ik_leg(raw_l, directions, _LEG_L, hip_pitch_offset))
    return angles


def swing_peak_rise(foot_height: float, period_s: float, dsp: float) -> dict[str, float]:
    """Peak swing-foot rise above the stance foot, OP3 ``wSin`` on z.

    ``updateMovementParam`` sets ``z_move_amplitude_ = foot_height / 2`` and
    ``z_move_amplitude_shift_ = z_move_amplitude_ / 2``, with phase ``π/2`` and
    ``z_move_period_time_ = period * ssp_ratio / 2``. Over the left-swing window
    the sine goes from −1 to +1 and back, so the foot rises by ``foot_height``
    above the value held on the stance foot. The internal half is not a half-height step.
    """
    ssp = 1.0 - dsp
    start = (1.0 - ssp) * period_s / 4.0
    end = (1.0 + ssp) * period_s / 4.0
    z_period = period_s * ssp / 2.0
    mag = foot_height / 2.0
    shift = mag / 2.0
    phase0 = math.pi / 2.0

    def wsin(time: float) -> float:
        phase = phase0 + 2.0 * math.pi / z_period * start
        return mag * math.sin(2.0 * math.pi / z_period * time - phase) + shift

    endpoint = wsin(start)
    if abs(wsin(end) - endpoint) > 1e-9:
        raise SystemExit("swing z does not return to the same height at both ends")
    peak = endpoint
    for i in range(1001):
        z = wsin(start + (end - start) * i / 1000.0)
        if z > peak:
            peak = z
    rise = peak - endpoint
    if abs(rise - foot_height) > 1e-6:
        raise SystemExit(f"swing rise {rise} != commanded {foot_height}")
    return {
        "commanded_m": foot_height,
        "endpoint_m": endpoint,
        "peak_m": peak,
        "rise_m": rise,
    }


def armature_ceiling(
    limit_nm: float,
    static_ss_nm: float,
    amplitude_rad: float,
    link_i: float,
    period_s: float,
) -> float:
    """Largest armature with ``static_ss + (I_link + J) A (2π/T)² <= limit``."""
    alpha = amplitude_rad * (2.0 * math.pi / period_s) ** 2
    if alpha <= 0.0:
        raise SystemExit("knee amplitude is zero")
    return (limit_nm - static_ss_nm) / alpha - link_i


def clock0_z_added(period_s: float, dsp: float, foot_height: float, z_swap: float) -> float:
    """``swap.z + right_leg_move.z`` at time 0, from the yaml gait terms.

    ``updateMovementParam`` halves ``foot_height`` into ``z_move_amplitude_``
    and sets the shift to half of that. At time 0 the right foot's z sample
    is frozen at ``r_ssp_start``. This is not part of the init-offset pose.
    """
    ssp = 1.0 - dsp
    l_ssp_start = (1.0 - ssp) * period_s / 4.0
    r_ssp_start = (3.0 - ssp) * period_s / 4.0
    z_move_period = period_s * ssp / 2.0
    z_move_amp = foot_height / 2.0
    z_move_shift = z_move_amp / 2.0
    z_phase = math.pi / 2.0
    swap_z = z_swap * math.sin(-1.5 * math.pi) + z_swap

    def wsin(time: float, period: float, phase: float, mag: float, shift: float) -> float:
        return mag * math.sin(2.0 * math.pi / period * time - phase) + shift

    phase_r = z_phase + 2.0 * math.pi / z_move_period * r_ssp_start
    move_r = wsin(r_ssp_start, z_move_period, phase_r, z_move_amp, z_move_shift)
    phase_l = z_phase + 2.0 * math.pi / z_move_period * l_ssp_start
    move_l = wsin(l_ssp_start, z_move_period, phase_l, z_move_amp, z_move_shift)
    return swap_z + move_r if abs(move_r - move_l) < 1e-9 else float("nan")


def _pose_only(angles: dict[str, float]) -> dict[str, float]:
    return {k: float(v) for k, v in angles.items() if not k.startswith("_")}


def knee_crouch_pose(flex: float, fwd: float) -> dict[str, float]:
    """Level-sole sagittal pose plus the stand hip-roll from ``gait_targets``."""
    angles = dict(INIT_ARM)
    angles.update(leg_pitch_angles(flex, fwd))
    angles["l_hip_roll"] = STAND_HIP_ROLL_L_RAD
    angles["r_hip_roll"] = STAND_HIP_ROLL_R_RAD
    return angles


def body_sole_drop(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> float:
    sole = sole_metrics(model, data, side)[2]
    body_z = float(data.xpos[_body_id(model, "body_link")][2])
    return body_z - sole


def static_joint_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint: str,
    distal: tuple[str, ...],
    grf_z: float,
    foot_side: str,
) -> dict[str, float]:
    """GRF at the contact-box center plus distal link weight, on the joint axis."""
    jid = _joint_id(model, joint)
    anchor = np.array(data.xanchor[jid], dtype=np.float64)
    axis = np.array(data.xaxis[jid], dtype=np.float64)
    gid = _geom_id(model, f"{foot_side}_foot_contact")
    cop = np.array(data.geom_xpos[gid], dtype=np.float64)
    grf = np.array([0.0, 0.0, grf_z], dtype=np.float64)
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
        "grf_z_n": grf_z,
        "cop_x_m": float(cop[0] - anchor[0]),
        "cop_y_m": float(cop[1] - anchor[1]),
        "cop_z_m": float(cop[2] - anchor[2]),
    }


def _distal(side: str, which: str) -> tuple[str, tuple[str, ...]]:
    prefix = f"{side}_"
    if which == "knee":
        return (
            f"{prefix}knee",
            (f"{prefix}knee_link", f"{prefix}ank_pitch_link", f"{prefix}ank_roll_link"),
        )
    if which == "ank_pitch":
        return (
            f"{prefix}ank_pitch",
            (f"{prefix}ank_pitch_link", f"{prefix}ank_roll_link"),
        )
    raise SystemExit(which)


def support_torques(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pose: dict[str, float],
    total_mass: float,
) -> dict[str, dict[str, float]]:
    """Equal double-support split, and single support on each foot."""
    set_pose(model, data, _pose_only(pose))
    half = 0.5 * total_mass * G
    full = total_mass * G
    out: dict[str, dict[str, float]] = {}
    for side in ("r", "l"):
        for which in ("knee", "ank_pitch"):
            joint, distal = _distal(side, which)
            out[f"ds:{side}:{which}"] = static_joint_torque(model, data, joint, distal, half, side)
            out[f"ss:{side}:{which}"] = static_joint_torque(model, data, joint, distal, full, side)
            out[f"swing:{side}:{which}"] = static_joint_torque(model, data, joint, distal, 0.0, side)
    return out


def solve_ank_pitch_level(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base: dict[str, float],
    side: str,
    knee: float,
) -> float:
    prefix = "r_" if side == "r" else "l_"
    joint = f"{prefix}ank_pitch"
    best_up = -2.0
    best = float(base[joint])

    def up_at(ank: float) -> float:
        angles = _pose_only(base)
        angles[f"{prefix}knee"] = knee
        angles[joint] = ank
        set_pose(model, data, angles)
        return sole_metrics(model, data, side)[3]

    for ank in np.linspace(-2.0, 2.0, 81):
        up = up_at(float(ank))
        if up > best_up:
            best_up = up
            best = float(ank)
    span = 0.08
    for _ in range(6):
        for ank in np.linspace(best - span, best + span, 21):
            up = up_at(float(ank))
            if up > best_up:
                best_up = up
                best = float(ank)
        span *= 0.4
    return best


def solve_knee_lift(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base: dict[str, float],
    side: str,
    height_m: float,
    knee_lo: float | None = None,
    knee_hi: float | None = None,
) -> dict[str, float]:
    """Knee change that raises this sole by ``height_m``.

    Hip pitch, hip roll, and hip yaw stay at the pose. Ankle pitch is the
    value that makes the contact box's world Z axis as vertical as that
    one joint can. The search is on the right knee for the rail.

    ``knee_lo`` and ``knee_hi`` restrict the search. The default window is
    the stance knee ±1.2 rad, which is what the 2 cm table uses. A second
    root exists once the knee extends through straight; the kit-walk bound
    passes a window that stays on the flexed side of the stance knee.
    """
    prefix = "r_" if side == "r" else "l_"
    clean = _pose_only(base)
    set_pose(model, data, clean)
    sole0 = sole_metrics(model, data, side)[2]
    knee0 = float(clean[f"{prefix}knee"])
    target = sole0 + height_m

    def evaluate(knee: float) -> tuple[float, float, float, float]:
        ank = solve_ank_pitch_level(model, data, clean, side, knee)
        angles = dict(clean)
        angles[f"{prefix}knee"] = knee
        angles[f"{prefix}ank_pitch"] = ank
        set_pose(model, data, angles)
        _dx, _dy, sole, up = sole_metrics(model, data, side)
        return sole - target, ank, sole, up

    best = float("inf")
    best_knee = knee0
    best_ank = float(clean[f"{prefix}ank_pitch"])
    best_sole = sole0
    best_up = 0.0
    lo = knee0 - 1.2 if knee_lo is None else knee_lo
    hi = knee0 + 1.2 if knee_hi is None else knee_hi
    for knee in np.linspace(lo, hi, 49):
        err, ank, sole, up = evaluate(float(knee))
        if abs(err) < best:
            best = abs(err)
            best_knee = float(knee)
            best_ank = ank
            best_sole = sole
            best_up = up
    span = 0.06
    for _ in range(6):
        for knee in np.linspace(best_knee - span, best_knee + span, 21):
            err, ank, sole, up = evaluate(float(knee))
            if abs(err) < best:
                best = abs(err)
                best_knee = float(knee)
                best_ank = ank
                best_sole = sole
                best_up = up
        span *= 0.45
    return {
        "knee0_rad": knee0,
        "knee_rad": best_knee,
        "d_knee_rad": best_knee - knee0,
        "ank_rad": best_ank,
        "sole0_m": sole0,
        "sole_m": best_sole,
        "height_m": best_sole - sole0,
        "height_err_m": best_sole - target,
        "foot_up_z": best_up,
    }


def knee_dynamic(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pose: dict[str, float],
    lift: dict[str, float],
    static_abs: dict[str, float],
) -> dict[str, object]:
    """Armature and link inertia at the pose, times the 2 cm knee amplitude."""
    set_pose(model, data, _pose_only(pose))
    total_i, arm, link_i = reflected_link_inertia(model, data, "r_knee")
    amplitude = abs(float(lift["d_knee_rad"]))
    rows: list[dict[str, float]] = []
    for period in (0.300, 0.400, 0.500, 0.600):
        omega = 2.0 * math.pi / period
        alpha = amplitude * omega * omega
        tau_arm = arm * alpha
        tau_link = link_i * alpha
        tau_full = total_i * alpha
        viscous = 0.08 * amplitude * omega
        row: dict[str, float] = {
            "period_s": period,
            "alpha_rad_s2": alpha,
            "tau_arm_nm": tau_arm,
            "tau_link_nm": tau_link,
            "tau_full_nm": tau_full,
            "tau_viscous_nm": viscous,
        }
        for key, static_nm in static_abs.items():
            row[f"{key}_plus_arm_nm"] = static_nm + tau_arm
            row[f"{key}_plus_full_nm"] = static_nm + tau_full
            row[f"{key}_plus_full_visc_nm"] = static_nm + tau_full + viscous
        rows.append(row)
    return {
        "M_ii": total_i,
        "armature": arm,
        "I_link": link_i,
        "arm_over_link": arm / link_i if link_i else float("nan"),
        "amplitude_rad": amplitude,
        "rows": rows,
    }


# Trial armatures for the sensitivity table. Neither value is written into a plant.
# 0.01 is the converter default already on the hinges. 0.045 is the Menagerie OP3
# system-ID result cited in the doc; it is not an HX-35H measurement.
ARMATURE_TRIALS: tuple[tuple[str, float], ...] = (
    ("converter_default", 0.01),
    ("menagerie_op3", 0.045),
)


def armature_sensitivity(
    pose_name: str,
    amplitude: float,
    link_i: float,
    plant_arm: float,
    published_rows: list[dict[str, float]],
    static_ds: float,
    static_ss: float,
    rail_nm: float,
) -> list[dict[str, float | str | bool]]:
    """Static knee torque plus (I_link + trial armature) * α.

    I_link and the 2 cm knee amplitude stay at the values from ``knee_dynamic``.
    α = A (2π / T)², the same sinusoid as that section. Viscous damping is not
    added here; the published period table's "full" columns leave it out too.
    """
    rows_out: list[dict[str, float | str | bool]] = []
    for label, trial in ARMATURE_TRIALS:
        for period in (0.300, 0.400, 0.500, 0.600):
            omega = 2.0 * math.pi / period
            alpha = amplitude * omega * omega
            tau_arm = trial * alpha
            tau_link = link_i * alpha
            tau_full = (link_i + trial) * alpha
            ds_peak = static_ds + tau_full
            ss_peak = static_ss + tau_full
            if abs(trial - plant_arm) < 1e-15:
                match = next(row for row in published_rows if abs(row["period_s"] - period) < 1e-12)
                if abs(ds_peak - match["ds_plus_full_nm"]) > 1e-9:
                    raise SystemExit(f"{pose_name} {period} ds peak drifted from the knee table")
                if abs(ss_peak - match["ss_plus_full_nm"]) > 1e-9:
                    raise SystemExit(f"{pose_name} {period} ss peak drifted from the knee table")
            rows_out.append(
                {
                    "pose": pose_name,
                    "armature_label": label,
                    "armature": trial,
                    "period_s": period,
                    "alpha_rad_s2": alpha,
                    "tau_arm_nm": tau_arm,
                    "tau_link_nm": tau_link,
                    "tau_full_nm": tau_full,
                    "ds_peak_nm": ds_peak,
                    "ss_peak_nm": ss_peak,
                    "ds_clears_rail": ds_peak <= rail_nm + 1e-12,
                    "ss_clears_rail": ss_peak <= rail_nm + 1e-12,
                }
            )
    return rows_out


def pose_snapshot(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pose: dict[str, float],
) -> dict[str, float]:
    set_pose(model, data, _pose_only(pose))
    out: dict[str, float] = {}
    for name in (
        "r_hip_pitch",
        "r_knee",
        "r_ank_pitch",
        "r_hip_roll",
        "l_hip_pitch",
        "l_knee",
        "l_ank_pitch",
        "l_hip_roll",
    ):
        out[name] = float(pose.get(name, 0.0))
    for side in ("r", "l"):
        dx, dy, sole, up = sole_metrics(model, data, side)
        out[f"{side}_dx_m"] = dx
        out[f"{side}_dy_m"] = dy
        out[f"{side}_drop_m"] = body_sole_drop(model, data, side)
        out[f"{side}_up_z"] = up
        out[f"{side}_sole_z_m"] = sole
    return out


def build_knee_report(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    crouch_pose: dict[str, float],
    total_mass: float,
    rail_nm: float,
) -> dict[str, object]:
    """Static knee/ankle torque and the 2 cm knee-lift inertia term."""
    lengths = lengths_from_model(model)
    op3 = op3_pose(
        model,
        lengths,
        OP3_Z_OFFSET_M,
        OP3_X_OFFSET_M,
        OP3_Y_OFFSET_M,
        OP3_HIP_PITCH_OFFSET_RAD,
    )
    z_clock = clock0_z_added(0.600, 0.2, 0.060, 0.006)
    op3_clock = op3_pose(
        model,
        lengths,
        OP3_Z_OFFSET_M + z_clock,
        OP3_X_OFFSET_M,
        OP3_Y_OFFSET_M,
        OP3_HIP_PITCH_OFFSET_RAD,
    )
    # Cross-check against the PR #42 stand: z_offset 0.015, other offsets 0.
    ik_z015 = op3_pose(model, lengths, 0.015, 0.0, 0.0, 0.0)
    native = op3_pose(
        model,
        OP3_LENGTHS,
        OP3_Z_OFFSET_M,
        OP3_X_OFFSET_M,
        OP3_Y_OFFSET_M,
        OP3_HIP_PITCH_OFFSET_RAD,
    )
    stand_034 = knee_crouch_pose(KNEE_STAND_FLEX_RAD, HIP_BIAS_FWD_RAD)
    crouch_034 = knee_crouch_pose(KNEE_STAND_FLEX_RAD + KNEE_EXTRA_FLEX_RAD, HIP_BIAS_FWD_RAD)
    poses: dict[str, dict[str, float]] = {
        "op3_init": op3,
        "crouch_015": crouch_pose,
        "knee_034": crouch_034,
    }
    snapshots: dict[str, dict[str, float]] = {}
    torques: dict[str, dict[str, dict[str, float]]] = {}
    lifts: dict[str, dict[str, float]] = {}
    dynamics: dict[str, dict[str, object]] = {}
    for name, pose in poses.items():
        snapshots[name] = pose_snapshot(model, data, pose)
        torques[name] = support_torques(model, data, pose, total_mass)
        lifts[name] = solve_knee_lift(model, data, pose, "r", STEP_HEIGHT_M)
        static_abs = {
            "ds": torques[name]["ds:r:knee"]["abs_nm"],
            "ss": torques[name]["ss:r:knee"]["abs_nm"],
        }
        dynamics[name] = knee_dynamic(model, data, pose, lifts[name], static_abs)
    sensitivity: list[dict[str, float | str | bool]] = []
    for name, dynamic in dynamics.items():
        published = dynamic["rows"]
        if not isinstance(published, list):
            raise SystemExit(f"{name} dynamics rows missing")
        sensitivity.extend(
            armature_sensitivity(
                name,
                float(dynamic["amplitude_rad"]),
                float(dynamic["I_link"]),
                float(dynamic["armature"]),
                published,
                torques[name]["ds:r:knee"]["abs_nm"],
                torques[name]["ss:r:knee"]["abs_nm"],
                rail_nm,
            )
        )
    set_pose(model, data, _pose_only(stand_034))
    stand_drop = body_sole_drop(model, data, "r")
    set_pose(model, data, _pose_only(crouch_034))
    crouch_drop = body_sole_drop(model, data, "r")
    set_pose(model, data, dict(INIT_ARM))
    straight_drop = body_sole_drop(model, data, "r")
    kit_bound = kit_walk_armature_bound(model, data, lengths, total_mass, dynamics, torques)
    return {
        "lengths_ainex": {
            "thigh_m": lengths.thigh_m,
            "calf_m": lengths.calf_m,
            "ankle_m": lengths.ankle_m,
            "hip_pitch_offset_m": lengths.hip_pitch_offset_m,
            "hip_offset_angle_rad": lengths.hip_offset_angle_rad,
            "leg_m": lengths.thigh_m + lengths.calf_m + lengths.ankle_m,
        },
        "lengths_op3": {
            "thigh_m": OP3_LENGTHS.thigh_m,
            "calf_m": OP3_LENGTHS.calf_m,
            "ankle_m": OP3_LENGTHS.ankle_m,
            "leg_m": OP3_LENGTHS.thigh_m + OP3_LENGTHS.calf_m + OP3_LENGTHS.ankle_m,
        },
        "weight_n": total_mass * G,
        "half_weight_n": 0.5 * total_mass * G,
        "rail_nm": rail_nm,
        "clock0_z_added_m": z_clock,
        "native_angles": {k: float(native[k]) for k in ("r_hip_pitch", "r_knee", "r_ank_pitch", "l_hip_pitch", "l_knee", "l_ank_pitch")},
        "ik_z015_angles": {k: float(ik_z015[k]) for k in ("r_hip_pitch", "r_knee", "r_ank_pitch", "l_knee")},
        "stand_034_drop_m": stand_drop,
        "knee_034_drop_m": crouch_drop,
        "knee_034_delta_drop_m": stand_drop - crouch_drop,
        "straight_drop_m": straight_drop,
        "snapshots": snapshots,
        "torques": torques,
        "lifts": lifts,
        "dynamics": dynamics,
        "armature_sensitivity": sensitivity,
        "kit_walk_bound": kit_bound,
        "op3_clock_snapshot": pose_snapshot(model, data, op3_clock),
        "op3_clock_torque_ds_knee": support_torques(model, data, op3_clock, total_mass)["ds:r:knee"],
    }


def kit_walk_armature_bound(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    lengths: LegLengths,
    total_mass: float,
    dynamics: dict[str, dict[str, object]],
    torques: dict[str, dict[str, dict[str, float]]],
) -> dict[str, object]:
    """Max knee armature at the yaml period and at move(1), single support.

    The stance is the walking-yaml IK (z offset, hip-pitch offset, x and y),
    sinusoids at zero. The lift is the hip-held sole rise. The yaml period is
    400 ms. move(1) replaces that period with 300 ms and keeps the same lift.
    """
    rise = swing_peak_rise(KIT_Z_MOVE_M, KIT_YAML_PERIOD_S, KIT_DSP)
    rise_move1 = swing_peak_rise(KIT_Z_MOVE_M, KIT_MOVE1_PERIOD_S, KIT_DSP)
    rise_015 = swing_peak_rise(0.015, KIT_MOVE1_PERIOD_S, KIT_DSP)
    pose = op3_pose(
        model,
        lengths,
        KIT_Z_OFFSET_M,
        KIT_X_OFFSET_M,
        KIT_Y_OFFSET_M,
        KIT_HIP_PITCH_DEG * math.pi / 180.0,
    )
    snapshot = pose_snapshot(model, data, pose)
    kit_tau = support_torques(model, data, pose, total_mass)
    static_ss = kit_tau["ss:r:knee"]["abs_nm"]
    static_ds = kit_tau["ds:r:knee"]["abs_nm"]
    set_pose(model, data, _pose_only(pose))
    _total_i, _arm, link_i = reflected_link_inertia(model, data, "r_knee")
    rows: list[dict[str, float | str | bool]] = []
    for height in LIFT_GRID_M:
        knee0 = float(_pose_only(pose)["r_knee"])
        lift = solve_knee_lift(model, data, pose, "r", height, knee_lo=knee0 - 1.2, knee_hi=knee0)
        if abs(float(lift["height_err_m"])) > 1e-4:
            raise SystemExit(f"kit lift {height} residual {lift['height_err_m']}")
        amplitude = abs(float(lift["d_knee_rad"]))
        by_period: dict[str, dict[str, float]] = {}
        for period in (KIT_MOVE1_PERIOD_S, KIT_YAML_PERIOD_S):
            alpha = amplitude * (2.0 * math.pi / period) ** 2
            by_period[f"{period:.3f}"] = {
                "period_s": period,
                "alpha_rad_s2": alpha,
                "j_max_rail": armature_ceiling(RAIL_NM, static_ss, amplitude, link_i, period),
                "j_max_stall": armature_ceiling(STALL_NM, static_ss, amplitude, link_i, period),
                "tau_at_j0_nm": static_ss + link_i * alpha,
            }
        move1 = by_period[f"{KIT_MOVE1_PERIOD_S:.3f}"]
        rows.append(
            {
                "lift_m": height,
                "achieved_m": float(lift["height_m"]),
                "amplitude_rad": amplitude,
                "alpha_rad_s2": move1["alpha_rad_s2"],
                "I_link": link_i,
                "static_ss_nm": static_ss,
                "rail_nm": RAIL_NM,
                "stall_nm": STALL_NM,
                "j_max_rail": move1["j_max_rail"],
                "j_max_stall": move1["j_max_stall"],
                "tau_at_j0_nm": move1["tau_at_j0_nm"],
                "periods": by_period,
                "is_yaml_z_move": abs(height - KIT_Z_MOVE_M) < 1e-12,
                "is_app_speed4_z": abs(height - 0.015) < 1e-12,
            }
        )
    published: list[dict[str, float | str]] = []
    for name, dynamic in dynamics.items():
        amplitude = float(dynamic["amplitude_rad"])
        link = float(dynamic["I_link"])
        static_ss_pose = torques[name]["ss:r:knee"]["abs_nm"]
        published.append(
            {
                "pose": name,
                "amplitude_rad": amplitude,
                "I_link": link,
                "static_ss_nm": static_ss_pose,
                "j_max_rail": armature_ceiling(RAIL_NM, static_ss_pose, amplitude, link, KIT_PERIOD_S),
                "j_max_stall": armature_ceiling(STALL_NM, static_ss_pose, amplitude, link, KIT_PERIOD_S),
            }
        )
    return {
        "yaml_period_s": KIT_YAML_PERIOD_S,
        "move1_period_s": KIT_MOVE1_PERIOD_S,
        "trajectory_step_s": KIT_TRAJECTORY_STEP_S,
        "servo_control_cycle_s": KIT_SERVO_CYCLE_S,
        "sim_period_s": KIT_SIM_PERIOD_S,
        "period_s": KIT_PERIOD_S,
        "dsp_ratio": KIT_DSP,
        "y_swap_m": KIT_Y_SWAP_M,
        "z_move_m": KIT_Z_MOVE_M,
        "z_offset_m": KIT_Z_OFFSET_M,
        "x_offset_m": KIT_X_OFFSET_M,
        "y_offset_m": KIT_Y_OFFSET_M,
        "hip_pitch_deg": KIT_HIP_PITCH_DEG,
        "x_amplitude_cap_m": KIT_X_AMP_CAP_M,
        "stall_kgf_cm": HX35H_STATIC_KGF_CM,
        "stall_nm_from_kgf_cm": HX35H_STATIC_KGF_CM * KGF_CM_TO_NM,
        "stall_nm": STALL_NM,
        "rail_nm": RAIL_NM,
        "swing_rise_yaml": rise,
        "swing_rise_move1": rise_move1,
        "swing_rise_0_015": rise_015,
        "snapshot": snapshot,
        "static_ds_nm": static_ds,
        "static_ss_nm": static_ss,
        "cop_x_m": kit_tau["ss:r:knee"]["cop_x_m"],
        "I_link": link_i,
        "rows": rows,
        "published_pose_2cm": published,
    }


# Stance width offsets, outward positive. +Y is the robot's left. A positive
# offset adds to the left foot's world Y and subtracts from the right foot's.
STANCE_Y_OFFSETS_M = (0.0, -0.005, 0.005, 0.018)


def _geom_y_half_extent(model: mujoco.MjModel, data: mujoco.MjData, gid: int) -> float:
    """Half-width of a box along world Y, including orientation."""
    rot = np.array(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
    size = np.array(model.geom_size[gid][:3], dtype=np.float64)
    return float(np.abs(rot[1]).dot(size))


def _foot_pair_gap(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    """Left box min-Y minus right box max-Y. Negative means the boxes overlap."""
    gl = _geom_id(model, "l_foot_contact")
    gr = _geom_id(model, "r_foot_contact")
    left_y = float(data.geom_xpos[gl][1])
    right_y = float(data.geom_xpos[gr][1])
    left_half = _geom_y_half_extent(model, data, gl)
    right_half = _geom_y_half_extent(model, data, gr)
    left_inner = left_y - left_half
    right_inner = right_y + right_half
    return {
        "left_inner_y_m": left_inner,
        "right_inner_y_m": right_inner,
        "gap_m": left_inner - right_inner,
    }


def _foot_collision(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float | int | bool]:
    """Whether the two foot boxes are allowed to contact, and whether they do."""
    gl = _geom_id(model, "l_foot_contact")
    gr = _geom_id(model, "r_foot_contact")
    contype_l = int(model.geom_contype[gl])
    contype_r = int(model.geom_contype[gr])
    conaff_l = int(model.geom_conaffinity[gl])
    conaff_r = int(model.geom_conaffinity[gr])
    can = (contype_l & conaff_r) != 0 and (contype_r & conaff_l) != 0
    fromto = np.zeros(6, dtype=np.float64)
    dist = float(mujoco.mj_geomDistance(model, data, gl, gr, 1.0, fromto))
    mujoco.mj_collision(model, data)
    n_pair = 0
    for i in range(int(data.ncon)):
        contact = data.contact[i]
        pair = {int(contact.geom1), int(contact.geom2)}
        if pair == {gl, gr}:
            n_pair += 1
    return {
        "contype_l": contype_l,
        "contype_r": contype_r,
        "conaffinity_l": conaff_l,
        "conaffinity_r": conaff_r,
        "filters_allow_contact": can,
        "geom_distance_m": dist,
        "n_foot_contacts": n_pair,
        "feet_collide": can and (dist < 0.0 or n_pair > 0),
    }


def _solve_outward(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    side: str,
    y_target: float,
) -> tuple[float, float]:
    """Hip roll and ankle roll that put this foot's box center on ``y_target``.

    The sole stays level. The search is the zero-pose leg, sagittal joints at 0.
    """
    base: dict[str, float] = {}

    def err(hip_roll: float) -> tuple[float, float, float]:
        ank = solve_ank_for_level(model, data, base, side, hip_roll)
        set_pose(model, data, apply_roll(base, side, hip_roll, ank))
        y = foot_y_offset(model, data, side)
        _dx, _dy, _sole, up = sole_metrics(model, data, side)
        return (y - y_target) ** 2, ank, up

    best = float("inf")
    best_hip = 0.0
    best_ank = 0.0
    for hip in np.linspace(-0.35, 0.35, 29):
        e, ank, _up = err(float(hip))
        if e < best:
            best = e
            best_hip = float(hip)
            best_ank = ank
    span = 0.04
    for _ in range(5):
        for hip in np.linspace(best_hip - span, best_hip + span, 15):
            e, ank, _up = err(float(hip))
            if e < best:
                best = e
                best_hip = float(hip)
                best_ank = ank
        span *= 0.45
    if best > (1e-4) ** 2:
        raise SystemExit(f"{side} stance offset missed by {math.sqrt(best)} m")
    return best_hip, best_ank


def _foot_frame_row(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    side: str,
) -> dict[str, float | int]:
    """Ankle axis, hip roll, and foot box in the world frame.

    At the zero pose the pelvis (``body_link``) sits at world XY origin, so
    these XY values are also pelvis-frame coordinates.
    """
    prefix = f"{side}_"
    ank = _joint_id(model, f"{prefix}ank_roll")
    hip = _joint_id(model, f"{prefix}hip_roll")
    gid = _geom_id(model, f"{prefix}foot_contact")
    axis = np.array(data.xanchor[ank], dtype=np.float64)
    hip_axis = np.array(data.xanchor[hip], dtype=np.float64)
    center = np.array(data.geom_xpos[gid], dtype=np.float64)
    size = np.array(model.geom_size[gid][:3], dtype=np.float64)
    pelvis = np.array(data.xpos[_body_id(model, "body_link")], dtype=np.float64)
    return {
        "ankle_x_m": float(axis[0]),
        "ankle_y_m": float(axis[1]),
        "ankle_z_m": float(axis[2]),
        "hip_x_m": float(hip_axis[0]),
        "hip_y_m": float(hip_axis[1]),
        "hip_z_m": float(hip_axis[2]),
        "box_x_m": float(center[0]),
        "box_y_m": float(center[1]),
        "box_z_m": float(center[2]),
        "box_hx_m": float(size[0]),
        "box_hy_m": float(size[1]),
        "box_hz_m": float(size[2]),
        "offset_x_m": float(center[0] - axis[0]),
        "offset_y_m": float(center[1] - axis[1]),
        "pelvis_x_m": float(pelvis[0]),
        "pelvis_y_m": float(pelvis[1]),
        "contype": int(model.geom_contype[gid]),
        "conaffinity": int(model.geom_conaffinity[gid]),
    }


def _stl_aabb(path: Path) -> dict[str, list[float]]:
    """Axis-aligned bounds of a binary STL, in the file's frame (metres)."""
    blob = path.read_bytes()
    if len(blob) < 84:
        raise SystemExit(f"{path} is not a binary STL")
    n = int.from_bytes(blob[80:84], "little")
    if len(blob) < 84 + n * 50:
        raise SystemExit(f"{path} STL triangle count does not fit")
    lo = [float("inf"), float("inf"), float("inf")]
    hi = [float("-inf"), float("-inf"), float("-inf")]
    for i in range(n):
        base = 84 + i * 50 + 12
        for k in range(3):
            x, y, z = struct.unpack_from("<fff", blob, base + k * 12)
            for axis, value in enumerate((x, y, z)):
                lo[axis] = min(lo[axis], float(value))
                hi[axis] = max(hi[axis], float(value))
    center = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
    full = [hi[i] - lo[i] for i in range(3)]
    return {"min_m": lo, "max_m": hi, "center_m": center, "full_m": full, "triangles": n}


def foot_box_vs_ankle(
    plants: dict[str, mujoco.MjModel],
) -> dict[str, object]:
    """Foot box against the ankle-roll axis at the zero pose, plus stance gaps.

    No plant XML is written. The y offsets are posed with hip roll and ankle
    roll; sagittal joints stay at zero.
    """
    out: dict[str, object] = {
        "pose": "qpos zero; these plants have no keyframe",
        "frame": "+X forward, +Y robot left, +Z up. Pelvis XY is the world origin at this pose.",
        "outward_positive": "positive offset adds to left-foot Y and subtracts from right-foot Y",
        "offset_y_sign": "box Y minus ankle-axis Y; positive means the box center is left of that ankle",
        "offset_x_sign": "box X minus ankle-axis X; positive is forward of the ankle axis",
        "gap_sign": "left box min world-Y minus right box max world-Y; negative is overlap",
        "plants": {},
    }
    plant_rows: dict[str, object] = {}
    for label, model in plants.items():
        data = mujoco.MjData(model)
        set_pose(model, data, {})
        feet = {side: _foot_frame_row(model, data, side) for side in ("r", "l")}
        gaps: list[dict[str, float | bool | int]] = []
        for offset in STANCE_Y_OFFSETS_M:
            if abs(offset) < 1e-15:
                set_pose(model, data, {})
            else:
                y_l = float(feet["l"]["box_y_m"]) + offset
                y_r = float(feet["r"]["box_y_m"]) - offset
                hip_l, ank_l = _solve_outward(model, data, "l", y_l)
                hip_r, ank_r = _solve_outward(model, data, "r", y_r)
                set_pose(
                    model,
                    data,
                    {
                        "l_hip_roll": hip_l,
                        "l_ank_roll": ank_l,
                        "r_hip_roll": hip_r,
                        "r_ank_roll": ank_r,
                    },
                )
            gap = _foot_pair_gap(model, data)
            hit = _foot_collision(model, data)
            y_l_now = float(data.geom_xpos[_geom_id(model, "l_foot_contact")][1])
            y_r_now = float(data.geom_xpos[_geom_id(model, "r_foot_contact")][1])
            gaps.append(
                {
                    "offset_m": offset,
                    "left_box_y_m": y_l_now,
                    "right_box_y_m": y_r_now,
                    **gap,
                    **hit,
                }
            )
        plant_rows[label] = {"feet": feet, "gaps": gaps}
    out["plants"] = plant_rows
    meshes = {}
    for side in ("r", "l"):
        meshes[side] = _stl_aabb(MESH_DIR / f"{side}_ank_roll_link.STL")
    out["urdf_mesh_aabb"] = meshes
    return out


def joint_force_limit(model: mujoco.MjModel, joint: str) -> float:
    jid = _joint_id(model, joint)
    limit = np.asarray(model.jnt_actfrcrange[jid], dtype=np.float64)
    return float(max(abs(float(limit[0])), abs(float(limit[1]))))


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

    thaw_model = load_model(thaw_xml)
    foot_box = foot_box_vs_ankle({"main": model, "thaw": thaw_model})
    for side in ("r", "l"):
        gid_main = _geom_id(model, f"{side}_foot_contact")
        gid_thaw = _geom_id(thaw_model, f"{side}_foot_contact")
        if not np.allclose(model.geom_pos[gid_main], thaw_model.geom_pos[gid_thaw]):
            raise SystemExit(f"{side} foot contact pos differs between plants")
        if abs(float(model.geom_size[gid_main][2]) - float(thaw_model.geom_size[gid_thaw][2])) > 1e-12:
            raise SystemExit(f"{side} foot contact half-z differs between plants")
    thaw_rail = joint_force_limit(thaw_model, "r_knee")
    main_rail = joint_force_limit(model, "r_knee")
    knee_report = build_knee_report(model, data, crouch_pose, total, thaw_rail)
    knee_report["main_rail_nm"] = main_rail
    knee_report["thaw_rail_nm"] = thaw_rail

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
        "knee": knee_report,
        "foot_box": foot_box,
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
