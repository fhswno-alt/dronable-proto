#!/usr/bin/env python3
"""Compiled-plant snapshot for the live-row JSON.

The twelve leg joints are stored as ``dof_armature``, ``kv``, ``kp``,
``forcerange``, ``actfrcrange`` and ``ctrlrange``. ``kv`` is
``-actuator_biasprm[actuator, 2]`` and ``kp`` is ``actuator_gainprm[actuator, 0]``
on the joint's ``_pos`` actuator. ``forcerange`` is the actuator force
range and ``actfrcrange`` is ``jnt_actfrcrange``.

``compiled_md5`` is the md5 of 253 little-endian float64 values: those
leg arrays, then body mass, body inertia, the floor and foot friction,
dof damping, ``opt.timestep`` and ``opt.integrator``. The field names and
the pack order match the voice056 manifest so the two JSON files diff
on values.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import steer_walk as sw

# File armature. A load-time override that leaves this value compiles the
# same kv as MjModel.from_xml_path.
FILE_ARMATURE_KGM2 = 0.01
COMPILE_XML = "MjModel.from_xml_path"
REF_MD5_ARMATURE_0_01 = "03ed33386178ab8d05db76a7307f1c1d"
REF_MD5_ARMATURE_0_025 = "f4fb2b70b5312a851e16316a38a88283"

QUANTITIES = (
    "dof_armature",
    "kv",
    "kp",
    "forcerange",
    "actfrcrange",
    "ctrlrange",
    "body_mass",
    "body_inertia",
    "geom_friction",
    "dof_damping",
    "opt.timestep",
    "opt.integrator",
)
_JOINT_FIELDS = ("dof_armature", "kv", "kp")
_RANGE_FIELDS = ("forcerange", "actfrcrange", "ctrlrange")
_INERTIA_AXES = ("ix", "iy", "iz")
_FRICTION_COLS = ("slide", "torsion", "roll")
_FOOT_GEOMS = ("r_foot_contact", "l_foot_contact")
FRONT_KEYS = (
    "xml_md5",
    "compiled_md5",
    "dof_armature",
    "kv",
    "kp",
    "forcerange",
    "actfrcrange",
    "ctrlrange",
    "mujoco_version",
    "compiled_md5_by_mujoco_version",
    "perturbation",
    "body_mass",
    "body_inertia",
    "geom_friction",
    "dof_damping",
    "opt",
    "compiled_order",
)


def compile_note(leg_armature: float | None) -> str:
    """How this model was compiled. The entry point always passes a number."""
    if leg_armature is None:
        return COMPILE_XML
    return (
        "MjSpec.from_file, twelve leg joint armatures set to "
        f"{float(leg_armature):g} before compile. "
        "model.dof_armature is not written after compile. The XML file is not written."
    )


def perturbation_note(leg_armature: float | None, perturb: object | None) -> str:
    """Label for hashed-array edits. The file armature with no mass or μ edit is ``none``."""
    parts: list[str] = []
    if leg_armature is not None and abs(float(leg_armature) - FILE_ARMATURE_KGM2) > 1e-12:
        parts.append(f"armature_{float(leg_armature):g}")
    if perturb is not None:
        scale = float(getattr(perturb, "mass_scale", 1.0))
        if abs(scale - 1.0) > 1e-12:
            parts.append(f"mass_scale_{scale:g}")
        friction = getattr(perturb, "friction", None)
        if friction is not None:
            parts.append(f"friction_{float(friction):g}")
    if not parts:
        return "none"
    return ",".join(parts)


def _integrator_name(code: int) -> str:
    for name in (
        "mjINT_EULER",
        "mjINT_RK4",
        "mjINT_IMPLICIT",
        "mjINT_IMPLICITFAST",
        "mjINT_DISCRETE",
    ):
        if int(getattr(mj.mjtIntegrator, name)) == int(code):
            return name
    return str(code)


def _leg_joints(model: mj.MjModel) -> list[tuple[int, str]]:
    wanted = set(sw.LEG_JOINT_NAMES)
    found: list[tuple[int, str]] = []
    for jid in range(model.njnt):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, jid) or ""
        if name in wanted:
            found.append((int(jid), name))
    if len(found) != len(sw.LEG_JOINT_NAMES):
        raise RuntimeError(f"compiled model has {len(found)} leg joints")
    return found


def _dof_names(model: mj.MjModel) -> list[str]:
    names: list[str] = []
    njnt = int(model.njnt)
    for jid in range(njnt):
        adr = int(model.jnt_dofadr[jid])
        nxt = int(model.jnt_dofadr[jid + 1]) if jid + 1 < njnt else int(model.nv)
        jname = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, jid) or f"joint{jid}"
        width = nxt - adr
        if width == 1:
            names.append(jname)
            continue
        for k in range(width):
            names.append(f"{jname}:{k}")
    if len(names) != int(model.nv):
        raise RuntimeError(f"dof labels {len(names)} != nv {model.nv}")
    return names


def _friction_geoms(model: mj.MjModel) -> list[tuple[int, str]]:
    floor = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor"))
    if floor < 0:
        raise RuntimeError("compiled model has no floor geom")
    feet: list[tuple[int, str]] = []
    for name in _FOOT_GEOMS:
        gid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name))
        if gid < 0:
            raise RuntimeError(f"compiled model has no {name}")
        feet.append((gid, name))
    feet.sort()
    return [(floor, "floor"), *feet]


def _pair(row: np.ndarray) -> list[float]:
    return [float(row[0]), float(row[1])]


def field_values(manifest: dict) -> dict[str, np.ndarray]:
    """Float64 values of each hashed quantity, in compiled_order."""
    order = manifest["compiled_order"]
    joints = list(order["joints"])
    bodies = list(order["bodies"])
    geoms = list(order["geoms"])
    dofs = list(order["dofs"])
    out: dict[str, np.ndarray] = {}
    for field in _JOINT_FIELDS:
        out[field] = np.array([manifest[field][name] for name in joints], dtype=np.float64)
    for field in _RANGE_FIELDS:
        out[field] = np.array(
            [float(end) for name in joints for end in manifest[field][name]],
            dtype=np.float64,
        )
    out["body_mass"] = np.array([manifest["body_mass"][name] for name in bodies], dtype=np.float64)
    out["body_inertia"] = np.array(
        [float(component) for name in bodies for component in manifest["body_inertia"][name]],
        dtype=np.float64,
    )
    out["geom_friction"] = np.array(
        [float(component) for name in geoms for component in manifest["geom_friction"][name]],
        dtype=np.float64,
    )
    out["dof_damping"] = np.array(
        [manifest["dof_damping"][name] for name in dofs], dtype=np.float64
    )
    out["opt.timestep"] = np.array([manifest["opt"]["timestep"]], dtype=np.float64)
    out["opt.integrator"] = np.array([float(manifest["opt"]["integrator"])], dtype=np.float64)
    return out


def packed_values(manifest: dict) -> np.ndarray:
    """The float64 values whose little-endian bytes are ``compiled_md5``."""
    fields = field_values(manifest)
    return np.concatenate([fields[name] for name in QUANTITIES])


def compiled_md5(manifest: dict) -> str:
    packed = np.ascontiguousarray(packed_values(manifest))
    return hashlib.md5(packed.tobytes()).hexdigest()


def changed_fields(manifest: dict, baseline: dict) -> list[str]:
    """Hashed fields that differ at rtol 1e-9, in pack order."""
    left = field_values(manifest)
    right = field_values(baseline)
    changed: list[str] = []
    for field in QUANTITIES:
        a = left[field]
        b = right[field]
        if a.shape != b.shape or not np.all(np.isclose(a, b, rtol=1e-9, atol=0.0)):
            changed.append(field)
    return changed


def compiled_manifest(
    model: mj.MjModel,
    *,
    xml_md5: str,
    compile: str,
    perturbation: str,
    compare_baseline: bool = True,
) -> dict:
    """Snapshot ``model`` after compile and after any mass or friction edit."""
    joints = _leg_joints(model)
    joint_names = [name for _, name in joints]
    dof_armature: dict[str, float] = {}
    kv: dict[str, float] = {}
    kp: dict[str, float] = {}
    forcerange: dict[str, list[float]] = {}
    actfrcrange: dict[str, list[float]] = {}
    ctrlrange: dict[str, list[float]] = {}
    for jid, name in joints:
        dof = int(model.jnt_dofadr[jid])
        aid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name + "_pos"))
        if aid < 0:
            raise RuntimeError(f"missing actuator {name}_pos")
        dof_armature[name] = float(model.dof_armature[dof])
        kv[name] = float(-model.actuator_biasprm[aid, 2])
        kp[name] = float(model.actuator_gainprm[aid, 0])
        forcerange[name] = _pair(model.actuator_forcerange[aid])
        actfrcrange[name] = _pair(model.jnt_actfrcrange[jid])
        ctrlrange[name] = _pair(model.actuator_ctrlrange[aid])
    bodies = [
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) or f"body{i}"
        for i in range(model.nbody)
    ]
    body_mass = {name: float(model.body_mass[i]) for i, name in enumerate(bodies)}
    body_inertia = {
        name: [float(x) for x in model.body_inertia[i]]
        for i, name in enumerate(bodies)
    }
    geom_ids = _friction_geoms(model)
    geom_friction = {
        name: [float(x) for x in model.geom_friction[gid, :3]]
        for gid, name in geom_ids
    }
    dof_names = _dof_names(model)
    dof_damping = {name: float(model.dof_damping[i]) for i, name in enumerate(dof_names)}
    code = int(model.opt.integrator)
    arms = list(dof_armature.values())
    armature_kgm2 = float(arms[0])
    order = {
        "joints": joint_names,
        "joint_rule": "ascending MuJoCo joint id of the twelve leg hinges in the compiled model",
        "bodies": bodies,
        "body_rule": "ascending body id, world body first",
        "inertia_layout": "ix, iy, iz principal moments, C order, one body at a time",
        "geoms": [name for _, name in geom_ids],
        "geom_rule": "floor, then the two foot contact boxes, ascending geom id",
        "friction_layout": "slide, torsion, roll",
        "dofs": dof_names,
        "dof_rule": (
            "ascending dof id. The free joint root occupies dofs 0-5 as root:0 "
            "through root:5, three translations then three rotations. Each hinge "
            "is one dof under its joint name."
        ),
        "quantities": list(QUANTITIES),
        "kv": "-actuator_biasprm[actuator, 2] on the joint _pos actuator",
        "kp": "actuator_gainprm[actuator, 0] on the joint _pos actuator",
        "range_layout": "lo then hi, C order, one pair per joint",
        "integrator": "mjtIntegrator cast to float64. mjINT_IMPLICITFAST is 3",
        "dtype": "float64",
        "byte_order": "little",
        "n": 0,
        "nbytes": 0,
        "hash": "md5",
        "compile": compile,
        "perturbation": perturbation,
        "armature_kgm2": armature_kgm2,
    }
    manifest: dict = {
        "xml_md5": str(xml_md5),
        "compiled_md5": "",
        "dof_armature": dof_armature,
        "kv": kv,
        "kp": kp,
        "forcerange": forcerange,
        "actfrcrange": actfrcrange,
        "ctrlrange": ctrlrange,
        "mujoco_version": mj.__version__,
        "compiled_md5_by_mujoco_version": {},
        "perturbation": perturbation,
        "body_mass": body_mass,
        "body_inertia": body_inertia,
        "geom_friction": geom_friction,
        "dof_damping": dof_damping,
        "opt": {
            "timestep": float(model.opt.timestep),
            "integrator": code,
            "integrator_name": _integrator_name(code),
        },
        "compiled_order": order,
    }
    packed = packed_values(manifest)
    digest = hashlib.md5(np.ascontiguousarray(packed).tobytes()).hexdigest()
    order["n"] = int(packed.size)
    order["nbytes"] = int(packed.nbytes)
    manifest["compiled_md5"] = digest
    manifest["compiled_md5_by_mujoco_version"] = {mj.__version__: digest}
    if compare_baseline and digest != REF_MD5_ARMATURE_0_01:
        baseline_model = mj.MjModel.from_xml_path(str(sw.PLANT_XML))
        baseline = compiled_manifest(
            baseline_model,
            xml_md5=xml_md5,
            compile=COMPILE_XML,
            perturbation="none",
            compare_baseline=False,
        )
        order["baseline_compiled_md5"] = baseline["compiled_md5"]
        order["changed_vs_baseline"] = changed_fields(manifest, baseline)
    return manifest


def merge_compiled(payload: dict, manifest: dict | None) -> None:
    """Put the compiled fields on ``payload`` without replacing ``tip_sha``."""
    if not isinstance(manifest, dict) or "compiled_md5" not in manifest:
        return
    front = {key: manifest[key] for key in FRONT_KEYS if key in manifest}
    rest = {key: value for key, value in payload.items() if key not in front}
    payload.clear()
    payload.update(front)
    payload.update(rest)


def manifest_from_traceback(exc: BaseException) -> dict | None:
    """The snapshot stored on the walker before a plan abort."""
    tb = exc.__traceback__
    while tb is not None:
        for value in tb.tb_frame.f_locals.values():
            manifest = getattr(value, "compiled_manifest", None)
            if isinstance(manifest, dict) and "compiled_md5" in manifest:
                return manifest
        tb = tb.tb_next
    return None


def _self_check() -> None:
    xml_md5 = hashlib.md5(sw.PLANT_XML.read_bytes()).hexdigest()
    if xml_md5 != sw.PLANT_MD5:
        raise SystemExit(f"plant md5 {xml_md5} != {sw.PLANT_MD5}")
    file_model = mj.MjModel.from_xml_path(str(sw.PLANT_XML))
    file_manifest = compiled_manifest(
        file_model, xml_md5=xml_md5, compile=COMPILE_XML, perturbation="none",
    )
    spec_01 = compiled_manifest(
        sw.compile_plant(sw.PLANT_XML, 0.01),
        xml_md5=xml_md5,
        compile=compile_note(0.01),
        perturbation=perturbation_note(0.01, None),
    )
    spec_25 = compiled_manifest(
        sw.compile_plant(sw.PLANT_XML, 0.025),
        xml_md5=xml_md5,
        compile=compile_note(0.025),
        perturbation=perturbation_note(0.025, None),
    )
    if file_manifest["compiled_md5"] != REF_MD5_ARMATURE_0_01:
        raise SystemExit(f"from_xml_path {file_manifest['compiled_md5']}")
    if spec_01["compiled_md5"] != REF_MD5_ARMATURE_0_01:
        raise SystemExit(f"MjSpec 0.01 {spec_01['compiled_md5']}")
    if spec_25["compiled_md5"] != REF_MD5_ARMATURE_0_025:
        raise SystemExit(f"MjSpec 0.025 {spec_25['compiled_md5']}")
    if spec_01["compiled_order"]["n"] != 253 or spec_01["compiled_order"]["nbytes"] != 2024:
        raise SystemExit("pack is not 253 float64 / 2024 bytes")
    if "baseline_compiled_md5" in spec_01["compiled_order"]:
        raise SystemExit("0.01 snapshot should match the file hash with no baseline block")
    if spec_25["compiled_order"]["changed_vs_baseline"] != ["dof_armature", "kv"]:
        raise SystemExit(spec_25["compiled_order"]["changed_vs_baseline"])
    if spec_25["compiled_order"]["baseline_compiled_md5"] != REF_MD5_ARMATURE_0_01:
        raise SystemExit("0.025 baseline is not the 0.01 hash")
    expect_compile = (
        "MjSpec.from_file, twelve leg joint armatures set to 0.025 before compile. "
        "model.dof_armature is not written after compile. The XML file is not written."
    )
    if spec_25["compiled_order"]["compile"] != expect_compile:
        raise SystemExit(spec_25["compiled_order"]["compile"])
    if spec_25["perturbation"] != "armature_0.025" or spec_01["perturbation"] != "none":
        raise SystemExit("perturbation labels")
    kv01 = spec_01["kv"]
    if kv01["r_hip_roll"] != 1.702689162101357 or kv01["r_knee"] != 1.4573414651387:
        raise SystemExit(f"0.01 kv {kv01['r_hip_roll']} {kv01['r_knee']}")
    kv25 = spec_25["kv"]
    if kv25["r_hip_roll"] != 2.301988354170677 or kv25["r_knee"] != 2.1963251457861634:
        raise SystemExit(f"0.025 kv {kv25['r_hip_roll']} {kv25['r_knee']}")
    for manifest in (file_manifest, spec_01, spec_25):
        again = json.loads(json.dumps(manifest))
        if compiled_md5(again) != manifest["compiled_md5"]:
            raise SystemExit("json round-trip changed the hash")
        if again["compiled_md5_by_mujoco_version"][mj.__version__] != manifest["compiled_md5"]:
            raise SystemExit("version key")
    payload = {"tip_sha": "ours", "plant_md5": xml_md5, "mujoco_version": mj.__version__}
    merge_compiled(payload, spec_01)
    if payload["tip_sha"] != "ours" or payload["compiled_md5"] != REF_MD5_ARMATURE_0_01:
        raise SystemExit("merge overwrote the tip or dropped the hash")
    if list(payload)[0] != "xml_md5":
        raise SystemExit(f"front key {list(payload)[0]}")

    class _Walker:
        compiled_manifest = spec_25

    def _raise(walker: _Walker) -> None:
        raise RuntimeError("plan inconsistent")

    try:
        _raise(_Walker())
    except RuntimeError as exc:
        found = manifest_from_traceback(exc)
    if found is None or found["compiled_md5"] != REF_MD5_ARMATURE_0_025:
        raise SystemExit("traceback did not keep the compiled snapshot")
    print(
        f"xml {xml_md5} mujoco {mj.__version__} "
        f"0.01 {REF_MD5_ARMATURE_0_01} 0.025 {REF_MD5_ARMATURE_0_025}"
    )


if __name__ == "__main__":
    _self_check()
