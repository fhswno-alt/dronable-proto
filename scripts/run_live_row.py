#!/usr/bin/env python3
"""One live voice056 row. The arguments are the whole rollout.

Writes JSON with the git tip, the plant md5, the MuJoCo version, every
argument, the right hip-roll signed force, the tip, and the DC line.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import mujoco as mj

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lipm_gait
import score_com_zmp as sc
import steer_walk as sw
import zmp_preview


def _tip_sha() -> str:
    root = Path(__file__).resolve().parents[1]
    out = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=root, text=True,
    )
    return out.strip()


def _plant_md5() -> str:
    digest = hashlib.md5()
    digest.update(sw.PLANT_XML.read_bytes())
    return digest.hexdigest()


def _opt_float(text: str) -> float | None:
    if text.strip().lower() in ("none", "off", "-", "null"):
        return None
    return float(text)


def _opt_int(text: str) -> int | None:
    if text.strip().lower() in ("none", "off", "-", "null"):
        return None
    return int(text)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Score one live voice056 row.")
    p.add_argument("--t", type=float, required=True, help="Step period, seconds.")
    p.add_argument("--vx", type=float, required=True, help="Commanded speed, m/s.")
    p.add_argument("--clearance", type=float, required=True, help="Swing height, m.")
    p.add_argument("--dsp", type=float, required=True, help="Double-support fraction.")
    p.add_argument("--ff", type=int, required=True, choices=(0, 1), help="Feedforward 0 or 1.")
    p.add_argument(
        "--knee-qdd-cap", type=str, required=True,
        help="Knee and leg q̈ cap in rad/s^2, or none.",
    )
    p.add_argument("--armature", type=float, required=True, help="Leg armature, kg m^2.")
    p.add_argument("--seed", type=str, required=True, help="Hinge seed, or none.")
    p.add_argument("--latency", type=int, required=True, help="Command latency, ticks.")
    p.add_argument("--mass-scale", type=float, required=True, help="Body mass scale.")
    p.add_argument("--mu", type=str, required=True, help="Sliding friction, or none.")
    p.add_argument("--arm-s", type=float, default=1.0)
    p.add_argument("--stand-s", type=float, default=0.25)
    p.add_argument("--walk-s", type=float, default=None)
    p.add_argument("--stop-s", type=float, default=2.40)
    p.add_argument("--preview-shape", type=float, default=0.0)
    p.add_argument("--preview-r", type=float, default=1.0e-4)
    p.add_argument("--amp", type=float, default=None, help="ZMP amplitude. Default is the sway solve.")
    p.add_argument("--z-quintic", type=int, default=0, choices=(0, 1))
    p.add_argument("--z-lead", type=int, default=0, choices=(0, 1))
    p.add_argument("--honor-vx", type=int, default=0, choices=(0, 1))
    p.add_argument("--crouch", type=float, default=0.025)
    p.add_argument("--hip-pitch-deg", type=float, default=15.0)
    p.add_argument("--gait", type=str, default="voice056")
    p.add_argument("--out", type=Path, required=True)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cap = _opt_float(args.knee_qdd_cap)
    seed = _opt_int(args.seed)
    mu = _opt_float(args.mu)
    walk_s = float(args.walk_s) if args.walk_s is not None else (1.0 + 2.05 * float(args.t))
    if cap is None:
        lipm_gait.WALK_KNEE_QDD_MAX = 1.0e9
    else:
        lipm_gait.WALK_KNEE_QDD_MAX = float(cap)
    if args.amp is None:
        amp = float(zmp_preview.sway_zmp_amp(
            float(args.t), float(args.dsp), 0.18, 0.016, 0.043, 0.0, float(args.preview_r),
        ))
    else:
        amp = float(args.amp)
    perturb = None
    if (
        seed is not None
        or abs(float(args.mass_scale) - 1.0) > 1e-12
        or mu is not None
        or int(args.latency) != 0
    ):
        perturb = sc.Perturb(
            label="row",
            seed=seed,
            mass_scale=float(args.mass_scale),
            friction=mu,
            latency_ticks=int(args.latency),
        )
    recorded = {
        "t": float(args.t),
        "vx": float(args.vx),
        "clearance": float(args.clearance),
        "dsp": float(args.dsp),
        "ff": int(args.ff),
        "knee_qdd_cap": cap,
        "armature": float(args.armature),
        "seed": seed,
        "latency": int(args.latency),
        "mass_scale": float(args.mass_scale),
        "mu": mu,
        "arm_s": float(args.arm_s),
        "stand_s": float(args.stand_s),
        "walk_s": walk_s,
        "stop_s": float(args.stop_s),
        "preview_shape": float(args.preview_shape),
        "preview_r": float(args.preview_r),
        "amp": amp,
        "z_quintic": int(args.z_quintic),
        "z_lead": int(args.z_lead),
        "honor_vx": int(args.honor_vx),
        "crouch": float(args.crouch),
        "hip_pitch_deg": float(args.hip_pitch_deg),
        "gait": str(args.gait),
    }
    payload: dict[str, object] = {
        "tip_sha": _tip_sha(),
        "plant_md5": _plant_md5(),
        "mujoco_version": mj.__version__,
        "soft_pass": 0,
        "args": recorded,
    }
    try:
        result = sc.run_attempt(
            period_s=float(args.t),
            dsp=float(args.dsp),
            amp_m=amp,
            z_m=float(args.clearance),
            arm_s=float(args.arm_s),
            stand_s=float(args.stand_s),
            walk_s=walk_s,
            stop_s=float(args.stop_s),
            vx_m_s=float(args.vx),
            preview_r=float(args.preview_r),
            preview_shape=float(args.preview_shape),
            crouch_m=float(args.crouch),
            hip_pitch_deg=float(args.hip_pitch_deg),
            gait_name=str(args.gait),
            z_quintic=bool(args.z_quintic),
            z_lead=bool(args.z_lead),
            honor_vx=bool(args.honor_vx),
            id_ff=bool(args.ff),
            leg_armature=float(args.armature),
            perturb=perturb,
        )
    except lipm_gait.PlanInconsistent as exc:
        payload["abort"] = str(exc)
        payload["r_hip_roll_signed"] = None
        payload["tip_ok"] = None
        payload["dc_ok"] = None
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(payload, indent=2, default=str) + "\n")
        print(json.dumps(payload, default=str), flush=True)
        return 2
    joint = result.get("joint_signed") or {}
    hip = joint.get("r_hip_roll") if isinstance(joint, dict) else None
    payload["abort"] = None
    payload["r_hip_roll_signed"] = None if not isinstance(hip, dict) else hip.get("value")
    payload["r_hip_roll_signed_t"] = None if not isinstance(hip, dict) else hip.get("t")
    payload["r_hip_roll_over"] = None if not isinstance(hip, dict) else hip.get("over")
    payload["signed_over"] = result.get("signed_over")
    payload["signed_value"] = result.get("signed_value")
    payload["signed_joint"] = result.get("signed_joint")
    payload["signed_t"] = result.get("signed_t")
    payload["signed_ok"] = result.get("signed_ok")
    payload["tip_ok"] = result.get("tip_ok")
    payload["min_up_z"] = result.get("min_up_z")
    payload["fault"] = result.get("fault")
    payload["dc_ok"] = result.get("dc_ok")
    payload["dc_excess"] = result.get("dc_excess")
    payload["dc_joint"] = result.get("dc_joint")
    payload["dc_tau"] = result.get("dc_tau")
    payload["dc_qvel"] = result.get("dc_qvel")
    payload["dc_limit"] = result.get("dc_limit")
    payload["dc_t"] = result.get("dc_t")
    payload["score"] = result
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, default=str) + "\n")
    print(
        f"tip {payload['tip_sha']} seed {seed} "
        f"r_hip_roll {payload['r_hip_roll_signed']} over {payload['r_hip_roll_over']} "
        f"signed_over {payload['signed_over']} tip_ok {payload['tip_ok']} "
        f"dc_ok {payload['dc_ok']} dc {payload['dc_joint']} "
        f"{payload['dc_excess']} fault {payload['fault']!r}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
