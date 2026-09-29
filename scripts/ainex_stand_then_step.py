#!/usr/bin/env python3
"""AiNex kit-matched: stand hold → minimal open-loop step cue (headless).

Prefers mujoco/ainex_hiwonder/ainex_controls.xml (HX torque clips, foot
contact boxes, L=orange/R=green) over bare ainex.xml.

Does NOT launch an interactive viewer. Uses MUJOCO_GL=egl (or osmesa).

Outputs (under mujoco/ainex_hiwonder/ unless --out-dir):
  stand_first_frame.png  — kinematic stand pose, 3/4 view
  stand_side.png         — side view
  stand_held.png         — after 2.5 s PD hold under gravity
  stand_walk_progress.png — after stand + open-loop step cue
  stand_then_step_note.txt — honest what-worked / what-fell

Usage:
  MUJOCO_GL=egl python scripts/ainex_stand_then_step.py
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
XML_DEFAULT = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls.xml"
OUT_DEFAULT = ROOT / "mujoco" / "ainex_hiwonder"

CTRL_HZ = 50.0
COM_Z = 0.238          # foot boxes near floor in stand pose
HOLD_S = 2.5
STEP_S = 2.0
GAIT_T = 0.55
HIP_BIAS_FWD = 0.10
HIP_PITCH_AMP = 0.28   # modest open-loop cue (not full walk acceptance)
KNEE_STANCE = 0.32
KNEE_SWING = 0.85
ARM_ROLL = 1.45        # |sho_roll| — arms down from T-pose

# Soft upright/height assist during step only (NOT production balance).
USE_ASSIST_ON_STEP = True


def act_name(jname: str) -> str:
    return f"{jname}_pos"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def to_joint(side: str, fwd: float, flex: float) -> tuple[float, float, float]:
    """Map signed (fwd thigh, knee flex) → (hip_j, knee_j, ank_j) for AiNex axes."""
    if side == "L":
        hip, knee = -fwd, +flex
    else:
        hip, knee = +fwd, -flex
    return hip, knee, hip + knee


def stand_targets() -> dict[str, float]:
    """Home / ready stand: slight knee bend, arms down, slight hip abduct."""
    q = {
        "head_pan": 0.0,
        "head_tilt": 0.0,
        "l_sho_pitch": 0.0,
        "l_sho_roll": -ARM_ROLL,
        "l_el_pitch": 0.40,
        "l_el_yaw": 0.0,
        "l_gripper": 0.0,
        "r_sho_pitch": 0.0,
        "r_sho_roll": +ARM_ROLL,
        "r_el_pitch": 0.40,
        "r_el_yaw": 0.0,
        "r_gripper": 0.0,
        "l_hip_yaw": 0.0,
        "l_hip_roll": -0.05,
        "r_hip_yaw": 0.0,
        "r_hip_roll": 0.05,
    }
    for side in ("L", "R"):
        hip, knee, ank = to_joint(side, HIP_BIAS_FWD, KNEE_STANCE)
        p = "l_" if side == "L" else "r_"
        q[f"{p}hip_pitch"] = hip
        q[f"{p}knee"] = knee
        q[f"{p}ank_pitch"] = ank
        q[f"{p}ank_roll"] = 0.0
    return q


def step_targets(t_step: float, amp: float = 1.0) -> dict[str, float]:
    """Very simple open-loop step: COM lateral shift via hip roll + alternating hip/knee."""
    q = stand_targets()
    a = clamp(amp, 0.0, 1.0)
    if a <= 0.0:
        return q

    phi = (t_step / GAIT_T) % 1.0
    # Lateral weight-shift cue (helps before swing)
    shift = 0.08 * a * math.sin(2.0 * math.pi * phi)
    q["l_hip_roll"] = -0.05 - shift
    q["r_hip_roll"] = 0.05 - shift

    for side in ("L", "R"):
        pref = "l_" if side == "L" else "r_"
        p = phi if side == "L" else (phi + 0.5) % 1.0
        if p < 0.5:
            s = p / 0.5
            s = s * s * (3.0 - 2.0 * s)
            # stance: fwd from +A (ahead) → -A (behind) so body rolls +X over plant
            fwd = HIP_BIAS_FWD + HIP_PITCH_AMP * a * (1.0 - 2.0 * s)
            flex = KNEE_STANCE
            sw = 0.0
            ank_extra = 0.0
        else:
            s = (p - 0.5) / 0.5
            sw = math.sin(math.pi * (s ** 0.55))
            s_sm = s * s * (3.0 - 2.0 * s)
            # swing: recover from behind → ahead
            fwd = HIP_BIAS_FWD + HIP_PITCH_AMP * a * (-1.0 + 2.0 * s_sm)
            flex = KNEE_STANCE + (KNEE_SWING - KNEE_STANCE) * sw
            ank_extra = 0.12 * sw

        hip, knee, ank = to_joint(side, fwd, flex)
        ank = ank + ank_extra if side == "L" else ank - ank_extra
        q[f"{pref}hip_pitch"] = clamp(hip, -1.5, 1.5)
        q[f"{pref}knee"] = clamp(knee, -2.0, 2.0)
        q[f"{pref}ank_pitch"] = clamp(ank, -1.2, 1.2)
        if side == "L":
            q[f"{pref}hip_roll"] = -0.05 - shift - 0.04 * sw
        else:
            q[f"{pref}hip_roll"] = 0.05 - shift + 0.04 * sw
        q[f"{pref}sho_pitch"] = -0.20 * (fwd - HIP_BIAS_FWD)

    return q


def set_ctrl(model: mj.MjModel, data: mj.MjData, qdes: dict[str, float], act_idx: dict[str, int]):
    for jname, val in qdes.items():
        an = act_name(jname)
        if an in act_idx:
            data.ctrl[act_idx[an]] = val


def apply_qpos(model: mj.MjModel, data: mj.MjData, qdes: dict[str, float]):
    for jn, val in qdes.items():
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = val


def soft_assist(model: mj.MjModel, data: mj.MjData, z_des: float = COM_Z):
    """Temporary upright/height assist — NOT a real balance controller."""
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    R = data.xmat[bid].reshape(3, 3)
    up = R[:, 2]
    axis = np.cross(up, np.array([0.0, 0.0, 1.0]))
    lean = float(np.linalg.norm(axis))
    if lean > 0.25:
        axis = axis * (0.25 / lean)
    omega = data.qvel[3:6]
    torque = np.clip(50.0 * axis - 6.0 * omega, -8.0, 8.0)
    mass = mj.mj_getTotalmass(model)
    z = data.qpos[2]
    fz = 0.88 * mass * 9.81 + 160.0 * (z_des - z) - 28.0 * data.qvel[2]
    fz = float(np.clip(fz, -35.0, 50.0))
    fy = float(np.clip(-12.0 * data.qpos[1] - 4.0 * data.qvel[1], -8.0, 8.0))
    fx = float(np.clip(-3.0 * data.qvel[0], -6.0, 6.0))
    data.qfrc_applied[0:6] = [fx, fy, fz, torque[0], torque[1], torque[2]]
    # soft upright lock
    tgt = np.array([1.0, 0.0, 0.0, 0.0])
    cur = np.array([float(v) for v in data.qpos[3:7]])
    if np.dot(cur, tgt) < 0:
        tgt = -tgt
    blend = 0.45 if up[2] > 0.85 else 0.80
    out = cur + blend * (tgt - cur)
    out = out / (np.linalg.norm(out) + 1e-12)
    data.qpos[3:7] = out
    data.qvel[3:6] *= (1.0 - 0.80 * blend)


def _burn(img: np.ndarray, lines: list[str]) -> np.ndarray:
    from PIL import Image, ImageDraw, ImageFont

    out = Image.fromarray(img)
    draw = ImageDraw.Draw(out)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)
        font_sm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11)
    except Exception:
        font = ImageFont.load_default()
        font_sm = font
    y = 6
    for i, line in enumerate(lines):
        f = font if i == 0 else font_sm
        draw.text((7, y + 1), line, fill=(0, 0, 0), font=f)
        draw.text((6, y), line, fill=(255, 230, 80), font=f)
        y += 16 if i == 0 else 14
    return np.asarray(out, dtype=np.uint8)


def render_cam(model, data, renderer, lookat, distance, azimuth, elevation) -> np.ndarray:
    cam = mj.MjvCamera()
    mj.mjv_defaultCamera(cam)
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    mj.mj_forward(model, data)
    renderer.update_scene(data, cam)
    return np.ascontiguousarray(renderer.render().copy(), dtype=np.uint8)


def body_up_z(model, data) -> float:
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    return float(data.xmat[bid].reshape(3, 3)[2, 2])


def main():
    ap = argparse.ArgumentParser(description="AiNex stand hold + minimal step (headless)")
    ap.add_argument("--model", type=str, default=str(XML_DEFAULT))
    ap.add_argument("--out-dir", type=str, default=str(OUT_DEFAULT))
    ap.add_argument("--hold", type=float, default=HOLD_S)
    ap.add_argument("--step", type=float, default=STEP_S)
    ap.add_argument("--no-assist", action="store_true")
    args = ap.parse_args()

    xml = Path(args.model)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not xml.exists():
        print(f"ERROR: missing {xml}", file=sys.stderr)
        sys.exit(1)

    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    act_idx = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(model.nu)}

    # --- load quality ---
    mesh_names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_MESH, i) for i in range(model.nmesh)]
    empty = [n for i, n in enumerate(mesh_names) if model.mesh_vertnum[i] == 0]
    hinge = sum(1 for i in range(model.njnt) if model.jnt_type[i] == mj.mjtJoint.mjJNT_HINGE)
    mass = float(mj.mj_getTotalmass(model))
    print(f"[stand] model={xml}")
    print(f"[stand] nq={model.nq} nv={model.nv} nu={model.nu} nhinge={hinge} nmesh={model.nmesh} ngeom={model.ngeom}")
    print(f"[stand] mass={mass:.4f} kg  empty_meshes={empty or 'none'}")
    if model.nmesh != 25 or empty:
        print("ERROR: expected 25 resolved meshes", file=sys.stderr)
        sys.exit(1)

    q_stand = stand_targets()
    # Init kinematic stand
    data.qpos[:] = 0.0
    data.qpos[0:3] = [0.0, 0.0, COM_Z]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    apply_qpos(model, data, q_stand)
    set_ctrl(model, data, q_stand, act_idx)
    mj.mj_forward(model, data)

    renderer = mj.Renderer(model, height=720, width=960)
    import imageio.v2 as imageio

    look = np.array([0.0, 0.0, 0.12], dtype=np.float64)

    # 1) First-frame standing (3/4 view)
    img = render_cam(model, data, renderer, look, 1.15, 140.0, -18.0)
    img = _burn(img, [
        "AiNex kit-matched STAND (kinematic)",
        f"model={xml.name}  nmesh={model.nmesh}  mass={mass:.3f}kg",
        f"COM_z={COM_Z:.3f}  knee={KNEE_STANCE}  |sho_roll|={ARM_ROLL}",
        "L=orange R=green  Hiwonder STL (NOT OEM STEP)",
    ])
    p_first = out_dir / "stand_first_frame.png"
    imageio.imwrite(p_first, img)
    print(f"[stand] wrote {p_first}")

    # 2) Side view
    img = render_cam(model, data, renderer, look, 1.10, 90.0, -12.0)
    img = _burn(img, [
        "AiNex STAND side view (kinematic)",
        f"forward=+X  body_z={data.qpos[2]:.3f}  up_z={body_up_z(model, data):.3f}",
    ])
    p_side = out_dir / "stand_side.png"
    imageio.imwrite(p_side, img)
    print(f"[stand] wrote {p_side}")

    # 3) Hold under gravity with PD (no assist — honest stand)
    sim_dt = float(model.opt.timestep)
    steps_per_ctrl = max(1, int(round((1.0 / CTRL_HZ) / sim_dt)))
    hold_s = float(args.hold)
    n_hold = int(hold_s * CTRL_HZ)
    fell_hold = False
    z_trace = []
    up_trace = []

    for k in range(n_hold):
        set_ctrl(model, data, q_stand, act_idx)
        for _ in range(steps_per_ctrl):
            mj.mj_step(model, data)
            data.qfrc_applied[:] = 0.0
        z = float(data.qpos[2])
        up = body_up_z(model, data)
        z_trace.append(z)
        up_trace.append(up)
        if (not np.isfinite(data.qpos).all()) or z < 0.10 or up < 0.35:
            fell_hold = True
            break

    z_end = float(data.qpos[2])
    up_end = body_up_z(model, data)
    held = (not fell_hold) and up_end > 0.75 and z_end > 0.18
    status = "HOLDING" if held else ("FALLING/TIPPING" if fell_hold or up_end < 0.55 else "UNSTABLE")
    img = render_cam(model, data, renderer, look, 1.15, 140.0, -18.0)
    img = _burn(img, [
        f"AiNex STAND held {data.time:.2f}s under gravity — {status}",
        f"body_z={z_end:.3f} (start {COM_Z:.3f})  up_z={up_end:.3f}",
        f"PD position actuators only  assist=OFF  fell={fell_hold}",
        f"z_min={min(z_trace):.3f} z_max={max(z_trace):.3f}" if z_trace else "",
    ])
    p_held = out_dir / "stand_held.png"
    imageio.imwrite(p_held, img)
    print(f"[stand] hold t={data.time:.2f}s z={z_end:.3f} up={up_end:.3f} status={status}")
    print(f"[stand] wrote {p_held}")

    # 4) Minimal open-loop step cue
    # If hold already fell, reset to stand before stepping so we still show a cue.
    if fell_hold or up_end < 0.6:
        print("[stand] reset to kinematic stand before step cue (hold failed)")
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[0:3] = [0.0, 0.0, COM_Z]
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        apply_qpos(model, data, q_stand)
        set_ctrl(model, data, q_stand, act_idx)
        mj.mj_forward(model, data)
        # brief settle with assist so feet plant
        for _ in range(int(0.4 * CTRL_HZ)):
            set_ctrl(model, data, q_stand, act_idx)
            for __ in range(steps_per_ctrl):
                soft_assist(model, data, COM_Z)
                mj.mj_step(model, data)
                data.qfrc_applied[:] = 0.0

    use_assist = USE_ASSIST_ON_STEP and (not args.no_assist)
    t0_step = float(data.time)
    x0 = float(data.qpos[0])
    step_s = float(args.step)
    n_step = int(step_s * CTRL_HZ)
    fell_step = False
    hip_l_adr = int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "l_hip_pitch")])
    hip_r_adr = int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "r_hip_pitch")])
    knee_l_adr = int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "l_knee")])
    knee_r_adr = int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "r_knee")])
    bid_lf = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link")
    bid_rf = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link")
    hip_hist = []
    foot_lead = []

    for k in range(n_step):
        t_step = float(data.time) - t0_step
        # ramp amp over 0.35 s
        amp = clamp(t_step / 0.35, 0.0, 1.0)
        qdes = step_targets(t_step, amp)
        set_ctrl(model, data, qdes, act_idx)
        for _ in range(steps_per_ctrl):
            if use_assist:
                soft_assist(model, data, COM_Z)
            mj.mj_step(model, data)
            data.qfrc_applied[:] = 0.0
        z = float(data.qpos[2])
        up = body_up_z(model, data)
        hip_hist.append((float(data.qpos[hip_l_adr]), float(data.qpos[hip_r_adr]),
                         float(data.qpos[knee_l_adr]), float(data.qpos[knee_r_adr])))
        foot_lead.append(float(data.xpos[bid_lf, 0]) > float(data.xpos[bid_rf, 0]))
        if (not np.isfinite(data.qpos).all()) or z < 0.10 or up < 0.30:
            fell_step = True
            break

    dx = float(data.qpos[0]) - x0
    z_s = float(data.qpos[2])
    up_s = body_up_z(model, data)
    hh = np.asarray(hip_hist, dtype=np.float64) if hip_hist else np.zeros((0, 4))
    hip_ptp = float(max(np.ptp(hh[:, 0]), np.ptp(hh[:, 1]))) if len(hh) else 0.0
    knee_ptp = float(max(np.ptp(hh[:, 2]), np.ptp(hh[:, 3]))) if len(hh) else 0.0
    lead_flips = int(np.sum(np.diff(np.asarray(foot_lead, dtype=np.int8)) != 0)) if len(foot_lead) > 1 else 0
    step_okish = (not fell_step) and up_s > 0.55 and hip_ptp > 0.12 and knee_ptp > 0.10

    img = render_cam(
        model, data, renderer,
        lookat=np.array([float(data.qpos[0]), 0.0, 0.12]),
        distance=1.15, azimuth=90.0, elevation=-12.0,
    )
    img = _burn(img, [
        f"AiNex stand→step cue t={data.time:.2f}s  {'ARTICULATING' if step_okish else 'FELL/WEAK'}",
        f"dx={dx:+.3f}m  z={z_s:.3f}  up_z={up_s:.3f}  assist={'ON' if use_assist else 'OFF'}",
        f"hip_ptp={hip_ptp:.3f} knee_ptp={knee_ptp:.3f} foot_lead_flips={lead_flips}",
        "open-loop COM-shift + alt hip/knee — NOT a closed-loop walk",
    ])
    p_prog = out_dir / "stand_walk_progress.png"
    imageio.imwrite(p_prog, img)
    print(f"[stand] step dx={dx:+.4f} hip_ptp={hip_ptp:.3f} knee_ptp={knee_ptp:.3f} fell={fell_step}")
    print(f"[stand] wrote {p_prog}")

    # Honest note
    worked = []
    failed = []
    worked.append(f"Load clean: {model.nmesh}/25 meshes, {hinge} hinges, {model.nu} actuators, mass={mass:.3f} kg")
    worked.append(f"Stand pose defined (knee={KNEE_STANCE}, |sho_roll|={ARM_ROLL}, hip_fwd={HIP_BIAS_FWD})")
    worked.append(f"First-frame + side renders saved")
    if held:
        worked.append(f"PD stand hold ~{hold_s:.1f}s under gravity (no assist): z={z_end:.3f} up_z={up_end:.3f}")
    else:
        failed.append(f"Stand hold tipped/fell: status={status} z={z_end:.3f} up_z={up_end:.3f}")
    if step_okish:
        worked.append(
            f"Open-loop step cue ran {step_s:.1f}s: hip_ptp={hip_ptp:.3f} knee_ptp={knee_ptp:.3f} "
            f"dx={dx:+.3f}m lead_flips={lead_flips} assist={'ON' if use_assist else 'OFF'}"
        )
        if use_assist and abs(dx) > 0.12:
            failed.append(
                f"Translation under soft assist likely includes foot skate "
                f"(dx={dx:+.3f}m in {step_s:.1f}s) — NOT claimed as walking"
            )
    else:
        failed.append(
            f"Step cue weak/fell: fell={fell_step} up={up_s:.3f} hip_ptp={hip_ptp:.3f} knee_ptp={knee_ptp:.3f}"
        )
        if not use_assist:
            failed.append("Without soft assist, open-loop step tips (expect fall) — balance not solved")
    failed.append(
        "True walking not achieved — open-loop only; no capture-point / ZMP / RL; "
        "URDF effort ±6 vs HX clips ±2.1 Nm; mesh collision off (boxes only)"
    )
    next_steps = [
        "Tune stand COM_Z + foot box height so plant is flat without assist",
        "Port walk_gait_ainex.py CPG with WORLD_FIXED acceptance (already axis-mapped)",
        "Add soft lateral COM shift before swing; raise swing clearance if feet scuff",
        "Optional: expose IMU/camera sites from URDF fixed frames",
    ]

    note = out_dir / "stand_then_step_note.txt"
    lines = [
        "AiNex stand → step progress note",
        f"model: {xml}",
        f"geometry: Hiwonder URDF+25 STL (NOT OEM STEP); Path A buy FROZEN",
        "",
        "WORKED:",
        *[f"  + {w}" for w in worked],
        "",
        "DID NOT / LIMITS:",
        *[f"  - {f}" for f in failed],
        "",
        "NEXT:",
        *[f"  → {n}" for n in next_steps],
        "",
        f"PNGs: {p_first.name}, {p_side.name}, {p_held.name}, {p_prog.name}",
        f"script: scripts/ainex_stand_then_step.py",
        f"related: scripts/walk_gait_ainex.py (full CPG attempt on same model)",
    ]
    note.write_text("\n".join(lines) + "\n")
    print(f"[stand] wrote {note}")

    stats = {
        "model": str(xml),
        "nmesh": int(model.nmesh),
        "nhinge": hinge,
        "nu": int(model.nu),
        "mass_kg": mass,
        "empty_meshes": empty,
        "com_z": COM_Z,
        "stand_targets": q_stand,
        "hold_s": hold_s,
        "hold_status": status,
        "hold_z": z_end,
        "hold_up_z": up_end,
        "held": held,
        "step_s": step_s,
        "step_dx": dx,
        "step_z": z_s,
        "step_up_z": up_s,
        "step_hip_ptp": hip_ptp,
        "step_knee_ptp": knee_ptp,
        "step_lead_flips": lead_flips,
        "step_fell": fell_step,
        "step_assist": use_assist,
        "pngs": {
            "first": str(p_first),
            "side": str(p_side),
            "held": str(p_held),
            "progress": str(p_prog),
        },
        "note": str(note),
    }
    stats_path = out_dir / "stand_then_step_stats.json"
    stats_path.write_text(json.dumps(stats, indent=2))
    print(f"[stand] stats → {stats_path}")

    renderer.close()
    print("[stand] DONE")
    # exit 0 even if tipped — artifacts + honest note are the deliverable
    sys.exit(0)


if __name__ == "__main__":
    main()
