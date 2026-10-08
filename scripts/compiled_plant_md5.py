#!/usr/bin/env python3
"""Compare a compiled plant against the version-keyed reference hashes.

``previews/compiled_refs.json`` maps ``mujoco.__version__`` to the md5 of the
253 little-endian float64 values at armature 0.01 and at armature 0.025.
``check_compiled`` compares ``compiled_md5`` first. On a mismatch it compares
each ``compiled_order`` field at rtol 1e-9 and names the field and index.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REFS_PATH = ROOT / "previews" / "compiled_refs.json"

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
RTOL = 1e-9


def load_refs(path: Path | None = None) -> dict:
    """Return ``{mujoco_version: {armature_0.01: md5, armature_0.025: md5}}``."""
    return json.loads((path or REFS_PATH).read_text())


def ref_md5(version: str, armature: str, refs: dict | None = None) -> str:
    """Look up one reference hash. ``version`` is the exact ``mujoco.__version__``."""
    table = refs if refs is not None else load_refs()
    try:
        return table[version][armature]
    except KeyError as exc:
        raise KeyError(f"no compiled ref for mujoco {version!r} {armature!r}") from exc


def _labels(order: dict) -> dict[str, list[str]]:
    joints = list(order["joints"])
    bodies = list(order["bodies"])
    geoms = list(order["geoms"])
    dofs = list(order["dofs"])
    labels: dict[str, list[str]] = {}
    for field in _JOINT_FIELDS:
        labels[field] = list(joints)
    for field in _RANGE_FIELDS:
        labels[field] = [f"{name}:{end}" for name in joints for end in ("lo", "hi")]
    labels["body_mass"] = list(bodies)
    labels["body_inertia"] = [
        f"{name}:{axis}" for name in bodies for axis in _INERTIA_AXES
    ]
    labels["geom_friction"] = [
        f"{name}:{col}" for name in geoms for col in _FRICTION_COLS
    ]
    labels["dof_damping"] = list(dofs)
    labels["opt.timestep"] = ["timestep"]
    labels["opt.integrator"] = ["integrator"]
    return labels


def field_values(manifest: dict) -> dict[str, np.ndarray]:
    """Float64 values of each compiled_order quantity, in the hash order."""
    order = manifest["compiled_order"]
    labels = _labels(order)
    out: dict[str, np.ndarray] = {}
    for field in _JOINT_FIELDS:
        out[field] = np.array(
            [manifest[field][name] for name in labels[field]], dtype=np.float64
        )
    for field in _RANGE_FIELDS:
        pairs = [
            float(end)
            for name in order["joints"]
            for end in manifest[field][name]
        ]
        out[field] = np.array(pairs, dtype=np.float64)
    out["body_mass"] = np.array(
        [manifest["body_mass"][name] for name in order["bodies"]], dtype=np.float64
    )
    out["body_inertia"] = np.array(
        [
            float(component)
            for name in order["bodies"]
            for component in manifest["body_inertia"][name]
        ],
        dtype=np.float64,
    )
    out["geom_friction"] = np.array(
        [
            float(component)
            for name in order["geoms"]
            for component in manifest["geom_friction"][name]
        ],
        dtype=np.float64,
    )
    out["dof_damping"] = np.array(
        [manifest["dof_damping"][name] for name in order["dofs"]], dtype=np.float64
    )
    out["opt.timestep"] = np.array([manifest["opt"]["timestep"]], dtype=np.float64)
    out["opt.integrator"] = np.array(
        [float(manifest["opt"]["integrator"])], dtype=np.float64
    )
    for field in QUANTITIES:
        if out[field].size != len(labels[field]):
            raise ValueError(f"{field} has {out[field].size} values, labels {len(labels[field])}")
    return out


def packed_values(manifest: dict) -> np.ndarray:
    """The 253 float64 values whose little-endian bytes are ``compiled_md5``."""
    fields = field_values(manifest)
    return np.concatenate([fields[name] for name in QUANTITIES])


def check_compiled(
    manifest: dict,
    reference: dict,
    *,
    rtol: float = RTOL,
) -> dict:
    """Compare ``compiled_md5`` first.

    A matching hash returns ``{"match": True, "compiled_md5": ...}`` and does
    not walk the arrays. On a mismatch each compiled_order field is compared
    with ``numpy.isclose(..., rtol=rtol, atol=0)``. Differences name the
    field, the index inside that field, and the compiled_order label.
    """
    got = str(manifest["compiled_md5"])
    expect = str(reference["compiled_md5"])
    if got == expect:
        return {"match": True, "compiled_md5": got}
    got_fields = field_values(manifest)
    ref_fields = field_values(reference)
    got_labels = _labels(manifest["compiled_order"])
    ref_labels = _labels(reference["compiled_order"])
    differences: list[dict] = []
    for field in QUANTITIES:
        left = got_fields[field]
        right = ref_fields[field]
        labels = got_labels[field]
        if left.shape != right.shape or labels != ref_labels[field]:
            differences.append(
                {
                    "field": field,
                    "index": None,
                    "name": None,
                    "reason": "shape",
                    "got_n": int(left.size),
                    "ref_n": int(right.size),
                }
            )
            continue
        mismatch = np.flatnonzero(~np.isclose(left, right, rtol=rtol, atol=0.0))
        for index in mismatch:
            i = int(index)
            differences.append(
                {
                    "field": field,
                    "index": i,
                    "name": labels[i],
                    "got": float(left[i]),
                    "ref": float(right[i]),
                }
            )
    return {
        "match": False,
        "compiled_md5": {"got": got, "ref": expect},
        "rtol": rtol,
        "differences": differences,
    }


def _self_check() -> None:
    import hashlib

    refs = load_refs()
    version = "3.14.0"
    base = json.loads((ROOT / "previews" / "run_manifest_a5a9183_voice056.json").read_text())
    sens = json.loads((ROOT / "previews" / "compiled_md5_armature_0.025.json").read_text())
    if base["mujoco_version"] != version or sens["mujoco_version"] != version:
        raise SystemExit("manifest mujoco_version is not 3.14.0")
    for manifest, key in ((base, "armature_0.01"), (sens, "armature_0.025")):
        packed = packed_values(manifest)
        if packed.size != 253:
            raise SystemExit(f"{key} packed {packed.size} values")
        digest = hashlib.md5(np.ascontiguousarray(packed).tobytes()).hexdigest()
        if digest != manifest["compiled_md5"]:
            raise SystemExit(f"{key} packed md5 {digest} != {manifest['compiled_md5']}")
        if digest != ref_md5(version, key, refs):
            raise SystemExit(f"{key} ref lookup {ref_md5(version, key, refs)} != {digest}")
        if manifest["compiled_md5_by_mujoco_version"][version] != digest:
            raise SystemExit(f"{key} is not keyed by {version}")
    same = check_compiled(base, base)
    if not same["match"]:
        raise SystemExit(f"baseline does not match itself: {same}")
    diff = check_compiled(sens, base)
    if diff["match"]:
        raise SystemExit("0.025 matched the 0.01 hash")
    fields = sorted({row["field"] for row in diff["differences"]})
    if fields != ["dof_armature", "kv"]:
        raise SystemExit(f"expected dof_armature and kv, got {fields}")
    for field in fields:
        idx = [row["index"] for row in diff["differences"] if row["field"] == field]
        if idx != list(range(12)):
            raise SystemExit(f"{field} indices {idx}")
    nudged = json.loads(json.dumps(sens))
    nudged["compiled_md5"] = "0" * 32
    nudged["kv"]["r_hip_yaw"] = float(nudged["kv"]["r_hip_yaw"]) * (1.0 + 1e-12)
    nudged["kp"]["r_knee"] = float(nudged["kp"]["r_knee"]) * (1.0 + 1e-8)
    nudged["body_mass"]["world"] = 1e-15
    inside = check_compiled(nudged, sens)
    named = [(row["field"], row["index"], row["name"]) for row in inside["differences"]]
    expect = [("kp", 3, "r_knee"), ("body_mass", 0, "world")]
    if named != expect:
        raise SystemExit(f"rtol rows {named} != {expect}")
    print(
        f"mujoco {version} armature_0.01 {refs[version]['armature_0.01']} "
        f"armature_0.025 {refs[version]['armature_0.025']}"
    )
    print("check_compiled: self match; 0.025 differs on dof_armature and kv, indices 0-11")


if __name__ == "__main__":
    _self_check()
