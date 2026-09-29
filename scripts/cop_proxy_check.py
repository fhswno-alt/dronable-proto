#!/usr/bin/env python3
"""AiNex CoP / stance-load PROXY falsifier (kit signals only vs MuJoCo truth).

Validates whether Path-A kit-available signals — joint q, actuator force
(HX torque / qfrc_actuator), and simulated bus load = |actuator_force| —
can track true foot contact forces / CoP well enough for skate/unload detect.

No FSR, no Orin, no NN. Reuses walk_gait_ainex stand/gait helpers.

Usage:
  MUJOCO_GL=egl .venv/bin/python scripts/cop_proxy_check.py
  MUJOCO_GL=egl .venv/bin/python scripts/cop_proxy_check.py --hz 100 --no-walk
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
sys.path.insert(0, str(ROOT / "scripts"))

import walk_gait_ainex as gait  # noqa: E402

XML_DEFAULT = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls.xml"
DOCS = ROOT / "docs"
OUT_DIR = ROOT / "mujoco" / "ainex_hiwonder"

# Leg actuators used for bus-load / stance proxy (kit HX leg chain)
LEG_ACTS = {
    "L": [
        "l_hip_yaw_pos",
        "l_hip_roll_pos",
        "l_hip_pitch_pos",
        "l_knee_pos",
        "l_ank_pitch_pos",
        "l_ank_roll_pos",
    ],
    "R": [
        "r_hip_yaw_pos",
        "r_hip_roll_pos",
        "r_hip_pitch_pos",
        "r_knee_pos",
        "r_ank_pitch_pos",
        "r_ank_roll_pos",
    ],
}
# Support-biased subset (gravity + plant) — stronger unload signal than full chain
SUPPORT_ACTS = {
    "L": ["l_knee_pos", "l_ank_pitch_pos", "l_ank_roll_pos", "l_hip_pitch_pos"],
    "R": ["r_knee_pos", "r_ank_pitch_pos", "r_ank_roll_pos", "r_hip_pitch_pos"],
}
ANK_ACTS = {
    "L": ("l_ank_pitch_pos", "l_ank_roll_pos"),
    "R": ("r_ank_pitch_pos", "r_ank_roll_pos"),
}

FOOT_GEOMS = {
    "L": ("l_foot_contact", "l_toe_viz"),
    "R": ("r_foot_contact", "r_toe_viz"),
}
FOOT_BODY = {"L": "l_ank_roll_link", "R": "r_ank_roll_link"}

# Verdict thresholds (stance load-share & CoP-y during disturb window)
CORR_USEABLE = 0.70
CORR_MARGINAL = 0.40
RMSE_SHARE_USEABLE = 0.12
RMSE_SHARE_MARGINAL = 0.22
RMSE_COPY_USEABLE = 0.025  # m
RMSE_COPY_MARGINAL = 0.050


def act_idx_map(model: mj.MjModel) -> dict[str, int]:
    out = {}
    for i in range(model.nu):
        n = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i)
        if n:
            out[n] = i
    return out


def apply_qpos(model: mj.MjModel, data: mj.MjData, qdes: dict[str, float]) -> None:
    for jn, val in qdes.items():
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = val


def body_up_z(model: mj.MjModel, data: mj.MjData) -> float:
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    R = data.xmat[bid].reshape(3, 3)
    return float(R[2, 2])


def _geom_ids(model: mj.MjModel, names: tuple[str, ...]) -> set[int]:
    ids = set()
    for n in names:
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, n)
        if gid >= 0:
            ids.add(gid)
    return ids


def true_foot_wrench(
    model: mj.MjModel,
    data: mj.MjData,
    gid_floor: int,
    foot_gids: set[int],
) -> tuple[float, float, float, float]:
    """Return (Fz, CoP_x, CoP_y, Fx_tang) for one foot from MuJoCo contacts.

    Contact force from mj_contactForce is in the contact frame; axis0 = normal.
    World force = frame[:,0]*fn + frame[:,1]*ft1 + frame[:,2]*ft2 (MuJoCo stores
    frame as 3x3 with columns = axes).
    """
    fz_sum = 0.0
    fx_sum = 0.0
    mx = 0.0  # sum Fz * x
    my = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if gid_floor not in (g1, g2):
            continue
        other = g2 if g1 == gid_floor else g1
        if other not in foot_gids:
            continue
        cf = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, cf)
        # frame: 9 floats, row-major; MuJoCo docs: frame[0:3]=normal
        frame = np.array(c.frame, dtype=np.float64).reshape(3, 3)
        # columns are axes: normal, tangent1, tangent2
        f_world = frame[:, 0] * cf[0] + frame[:, 1] * cf[1] + frame[:, 2] * cf[2]
        # Floor reaction on robot: if geom1 is floor, contactForce is on geom1;
        # normal points from geom1→geom2 roughly. Use world +Z component magnitude
        # as normal load; sign so positive = upward support on robot.
        fz = float(-f_world[2]) if g1 == gid_floor else float(f_world[2])
        # Prefer contact-frame normal magnitude with +Z alignment
        n_world = frame[:, 0]
        # Ensure normal points roughly +Z (floor up)
        if n_world[2] < 0:
            n_world = -n_world
            fn = -cf[0]
            ft1, ft2 = -cf[1], -cf[2]
        else:
            fn, ft1, ft2 = cf[0], cf[1], cf[2]
        f_world = frame[:, 0] * fn + frame[:, 1] * ft1 + frame[:, 2] * ft2
        # Reaction ON the foot (robot): opposite of force on floor
        if g1 == gid_floor:
            f_on_foot = -f_world
        else:
            f_on_foot = f_world
        fz_i = float(f_on_foot[2])
        if fz_i < 0:
            # compressive normal should push foot up; flip if needed
            fz_i = -fz_i
            f_on_foot = -f_on_foot
        fz_sum += fz_i
        fx_sum += float(f_on_foot[0])
        mx += fz_i * float(c.pos[0])
        my += fz_i * float(c.pos[1])
    if fz_sum < 1e-6:
        return 0.0, float("nan"), float("nan"), 0.0
    return fz_sum, mx / fz_sum, my / fz_sum, fx_sum


def bus_load(data: mj.MjData, act_idx: dict[str, int], names: list[str]) -> float:
    s = 0.0
    for n in names:
        i = act_idx.get(n)
        if i is not None:
            s += abs(float(data.actuator_force[i]))
    return s


def ank_torques(
    data: mj.MjData, act_idx: dict[str, int], side: str
) -> tuple[float, float]:
    ap, ar = ANK_ACTS[side]
    tp = float(data.actuator_force[act_idx[ap]]) if ap in act_idx else 0.0
    tr = float(data.actuator_force[act_idx[ar]]) if ar in act_idx else 0.0
    return tp, tr


def proxy_from_kit(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    mass: float,
    foot_xy: dict[str, np.ndarray],
) -> dict[str, float]:
    """Kit-only proxy: bus |τ|, support |τ|, ankle-torque CoP guess."""
    bus_L = bus_load(data, act_idx, LEG_ACTS["L"])
    bus_R = bus_load(data, act_idx, LEG_ACTS["R"])
    sup_L = bus_load(data, act_idx, SUPPORT_ACTS["L"])
    sup_R = bus_load(data, act_idx, SUPPORT_ACTS["R"])
    eps = 1e-9
    share_bus = bus_L / (bus_L + bus_R + eps)
    share_sup = sup_L / (sup_L + sup_R + eps)

    # Weight estimate split by support bus → Fz proxy
    mg = mass * 9.81
    fz_L_p = share_sup * mg
    fz_R_p = (1.0 - share_sup) * mg

    # Ankle-torque local CoP offsets (quasi-static; axis signs from AiNex URDF).
    # L ank_pitch axis +Y, R ank_pitch axis -Y mirrored; roll +X both.
    # Empirically calibrate sign against quiet-stand residual below.
    tp_L, tr_L = ank_torques(data, act_idx, "L")
    tp_R, tr_R = ank_torques(data, act_idx, "R")

    # Local CoP relative to ankle body: τ ≈ r × F → for F≈(0,0,Fz):
    #   τ_x ≈ -r_y * Fz  → r_y ≈ -τ_x / Fz
    #   τ_y ≈  r_x * Fz  → r_x ≈  τ_y / Fz
    # Map actuator torque (about joint axis) ≈ body τ component.
    def local_cop(tp: float, tr: float, fz: float, side: str) -> tuple[float, float]:
        fz_c = max(fz, 0.5)
        # pitch joint ≈ τ_y world for upright foot; roll ≈ τ_x
        # Mirrored R hip/ank pitch: flip pitch sign on R
        # Pitch axes mirrored L(+Y)/R(-Y): flip R. Empirical: CoP-x anti-corr
        # without overall minus on rx (quiet-stand probe).
        sx = -1.0 if side == "R" else 1.0
        rx = -sx * (tp / fz_c)
        ry = -(tr / fz_c)
        return rx, ry

    rx_L, ry_L = local_cop(tp_L, tr_L, fz_L_p, "L")
    rx_R, ry_R = local_cop(tp_R, tr_R, fz_R_p, "R")

    # World CoP from each foot ankle xy + local offset, then force-weighted
    pL = foot_xy["L"]
    pR = foot_xy["R"]
    cop_L = np.array([pL[0] + rx_L, pL[1] + ry_L])
    cop_R = np.array([pR[0] + rx_R, pR[1] + ry_R])
    wL, wR = fz_L_p, fz_R_p
    wsum = wL + wR + eps
    cop_xy = (wL * cop_L + wR * cop_R) / wsum

    # Coarser CoP-y from load share alone (no ankle τ): interpolate foot y
    cop_y_share = share_sup * pL[1] + (1.0 - share_sup) * pR[1]

    return {
        "bus_L": bus_L,
        "bus_R": bus_R,
        "sup_L": sup_L,
        "sup_R": sup_R,
        "share_bus": share_bus,
        "share_sup": share_sup,
        "fz_L_p": fz_L_p,
        "fz_R_p": fz_R_p,
        "cop_x_p": float(cop_xy[0]),
        "cop_y_p": float(cop_xy[1]),
        "cop_y_share": float(cop_y_share),
        "tp_L": tp_L,
        "tr_L": tr_L,
        "tp_R": tp_R,
        "tr_R": tr_R,
    }


def corr_rmse(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """Pearson corr and RMSE; NaN-safe."""
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 5:
        return float("nan"), float("nan")
    aa, bb = a[m], b[m]
    if np.std(aa) < 1e-12 or np.std(bb) < 1e-12:
        c = float("nan")
    else:
        c = float(np.corrcoef(aa, bb)[0, 1])
    rmse = float(np.sqrt(np.mean((aa - bb) ** 2)))
    return c, rmse


def ema_series(x: np.ndarray, alpha: float = 0.20) -> np.ndarray:
    """Causal EMA; alpha=1 → raw."""
    y = np.array(x, dtype=np.float64, copy=True)
    if len(y) == 0:
        return y
    for i in range(1, len(y)):
        if np.isfinite(y[i]) and np.isfinite(y[i - 1]):
            y[i] = alpha * y[i] + (1.0 - alpha) * y[i - 1]
    return y


def sample_truth_and_proxy(
    model, data, act_idx, gid_floor, foot_gids, mass, bid_feet
) -> dict[str, float]:
    fz_L, cx_L, cy_L, fx_L = true_foot_wrench(model, data, gid_floor, foot_gids["L"])
    fz_R, cx_R, cy_R, fx_R = true_foot_wrench(model, data, gid_floor, foot_gids["R"])
    fz_tot = fz_L + fz_R
    if fz_tot > 1e-6:
        share_T = fz_L / fz_tot
        if math.isfinite(cx_L) and math.isfinite(cx_R):
            cop_x = (fz_L * cx_L + fz_R * cx_R) / fz_tot
            cop_y = (fz_L * cy_L + fz_R * cy_R) / fz_tot
        elif math.isfinite(cx_L):
            cop_x, cop_y = cx_L, cy_L
            share_T = 1.0
        elif math.isfinite(cx_R):
            cop_x, cop_y = cx_R, cy_R
            share_T = 0.0
        else:
            cop_x = cop_y = float("nan")
    else:
        share_T = float("nan")
        cop_x = cop_y = float("nan")

    foot_xy = {
        "L": data.xpos[bid_feet["L"], :2].copy(),
        "R": data.xpos[bid_feet["R"], :2].copy(),
    }
    p = proxy_from_kit(model, data, act_idx, mass, foot_xy)
    out = {
        "t": float(data.time),
        "fz_L": fz_L,
        "fz_R": fz_R,
        "share_T": share_T,
        "cop_x_T": cop_x,
        "cop_y_T": cop_y,
        "fx_L": fx_L,
        "fx_R": fx_R,
        "z": float(data.qpos[2]),
        "up_z": body_up_z(model, data),
        **p,
    }
    return out


def run_episode(
    model: mj.MjModel,
    *,
    mode: str,
    hz: float,
    settle_s: float,
    disturb_t: float,
    disturb_hold: float,
    disturb_fy: float,
    disturb_fx: float,
    duration: float,
    gait_ramp_s: float,
) -> tuple[list[dict], dict]:
    data = mj.MjData(model)
    act_idx = act_idx_map(model)
    mass = float(mj.mj_getTotalmass(model))
    gid_floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    foot_gids = {s: _geom_ids(model, FOOT_GEOMS[s]) for s in ("L", "R")}
    bid_feet = {
        s: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, FOOT_BODY[s]) for s in ("L", "R")
    }
    bid_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")

    q0 = gait.gait_targets(0.0, gait_on=False, amp=0.0)
    data.qpos[0:3] = [0.0, 0.0, gait.COM_Z]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    apply_qpos(model, data, q0)
    gait.set_ctrl(model, data, q0, act_idx)
    mj.mj_forward(model, data)

    ctrl_dt = 1.0 / hz
    steps_per = max(1, int(round(ctrl_dt / model.opt.timestep)))
    logs: list[dict] = []
    tip_t = None
    disturb_on = False

    n_ctrl = int(math.ceil(duration * hz))
    for k in range(n_ctrl):
        t = data.time
        # targets
        if mode == "stand":
            qdes = gait.gait_targets(0.0, gait_on=False, amp=0.0)
        else:
            # walk: settle then ramp open-loop gait (no ankle CoP / plant — proxy sense-only)
            if t < settle_s:
                qdes = gait.gait_targets(0.0, gait_on=False, amp=0.0)
            else:
                tw = t - settle_s
                amp = min(1.0, tw / max(gait_ramp_s, 1e-3))
                qdes = gait.gait_targets(tw, gait_on=True, amp=amp)

        gait.set_ctrl(model, data, qdes, act_idx)

        # clear external wrenches then apply disturb on free joint (stand mode)
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        if mode == "stand" and disturb_t <= t < disturb_t + disturb_hold:
            disturb_on = True
            # force on torso body (world)
            data.xfrc_applied[bid_body, 0] = disturb_fx
            data.xfrc_applied[bid_body, 1] = disturb_fy

        for _ in range(steps_per):
            mj.mj_step(model, data)

        row = sample_truth_and_proxy(
            model, data, act_idx, gid_floor, foot_gids, mass, bid_feet
        )
        row["disturb"] = 1.0 if (
            mode == "stand" and disturb_t <= row["t"] < disturb_t + disturb_hold
        ) else 0.0
        row["mode"] = mode
        logs.append(row)

        if tip_t is None and (row["up_z"] < 0.50 or row["z"] < 0.12):
            tip_t = row["t"]
            if mode == "walk":
                break  # stop at tip for walk window

    meta = {
        "mode": mode,
        "hz": hz,
        "mass_kg": mass,
        "t_final": float(logs[-1]["t"]) if logs else 0.0,
        "tip_t": tip_t,
        "n_samples": len(logs),
        "disturb_applied": disturb_on if mode == "stand" else False,
        "disturb_fy": disturb_fy if mode == "stand" else None,
        "disturb_fx": disturb_fx if mode == "stand" else None,
        "disturb_t": disturb_t if mode == "stand" else None,
        "disturb_hold": disturb_hold if mode == "stand" else None,
        "com_z_cmd": gait.COM_Z,
    }
    return logs, meta


def window_mask(logs: list[dict], t0: float, t1: float) -> np.ndarray:
    t = np.array([r["t"] for r in logs])
    return (t >= t0) & (t <= t1)


def summarize(logs: list[dict], meta: dict, win: tuple[float, float] | None) -> dict:
    if not logs:
        return {"empty": True, **meta}
    t = np.array([r["t"] for r in logs])
    if win is None:
        m = np.ones(len(logs), dtype=bool)
        wlabel = "full"
    else:
        m = window_mask(logs, win[0], win[1])
        wlabel = f"{win[0]:.2f}-{win[1]:.2f}s"

    def col(key: str) -> np.ndarray:
        return np.array([r[key] for r in logs], dtype=np.float64)

    share_T = col("share_T")
    share_sup = col("share_sup")
    share_bus = col("share_bus")
    cop_y_T = col("cop_y_T")
    cop_y_p = col("cop_y_p")
    cop_y_share = col("cop_y_share")
    cop_x_T = col("cop_x_T")
    cop_x_p = col("cop_x_p")
    fz_L = col("fz_L")
    fz_R = col("fz_R")
    fz_Lp = col("fz_L_p")
    fz_Rp = col("fz_R_p")

    metrics = {}
    share_sup_ema = ema_series(share_sup, alpha=0.20)
    cop_y_p_ema = ema_series(cop_y_p, alpha=0.20)
    for name, a, b in [
        ("share_sup_vs_T", share_sup, share_T),
        ("share_sup_ema_vs_T", share_sup_ema, share_T),
        ("share_bus_vs_T", share_bus, share_T),
        ("cop_y_ankle_vs_T", cop_y_p, cop_y_T),
        ("cop_y_ankle_ema_vs_T", cop_y_p_ema, cop_y_T),
        ("cop_y_share_vs_T", cop_y_share, cop_y_T),
        ("cop_x_ankle_vs_T", cop_x_p, cop_x_T),
        ("fz_L_vs_p", fz_L, fz_Lp),
        ("fz_R_vs_p", fz_R, fz_Rp),
    ]:
        c, rmse = corr_rmse(a[m], b[m])
        metrics[name] = {"corr": c, "rmse": rmse, "n": int(m.sum())}

    # Unload detection: when true share crosses 0.35/0.65, does proxy agree?
    # Binary agreement on "L unloaded" = share_T < 0.35
    st = share_T[m]
    sp = share_sup[m]
    valid = np.isfinite(st) & np.isfinite(sp)
    unload_agree = float("nan")
    if valid.sum() >= 5:
        L_un_T = st[valid] < 0.35
        L_un_P = sp[valid] < 0.35
        R_un_T = st[valid] > 0.65
        R_un_P = sp[valid] > 0.65
        agree = (L_un_T == L_un_P) & (R_un_T == R_un_P)
        # also majority-load agreement
        maj_T = st[valid] >= 0.5
        maj_P = sp[valid] >= 0.5
        unload_agree = float(np.mean(maj_T == maj_P))

    return {
        **meta,
        "window": wlabel,
        "metrics": metrics,
        "majority_load_agree": unload_agree,
        "share_T_mean": float(np.nanmean(share_T[m])) if m.any() else None,
        "share_sup_mean": float(np.nanmean(share_sup[m])) if m.any() else None,
        "cop_y_T_ptp": float(np.nanmax(cop_y_T[m]) - np.nanmin(cop_y_T[m]))
        if m.any() and np.isfinite(cop_y_T[m]).any()
        else None,
        "cop_y_p_ptp": float(np.nanmax(cop_y_p[m]) - np.nanmin(cop_y_p[m]))
        if m.any() and np.isfinite(cop_y_p[m]).any()
        else None,
    }


def grade_verdict(stand_sum: dict, walk_sum: dict | None) -> tuple[str, list[str]]:
    """USEABLE / MARGINAL / FAIL from numeric gates."""
    notes: list[str] = []
    m = stand_sum.get("metrics", {})
    share = m.get("share_sup_vs_T", {})
    copy = m.get("cop_y_ankle_vs_T", {})
    share_c = share.get("corr")
    share_r = share.get("rmse")
    copy_c = copy.get("corr")
    copy_r = copy.get("rmse")
    agree = stand_sum.get("majority_load_agree")

    def ok_use(c, r, r_thr):
        return (
            c is not None
            and math.isfinite(c)
            and c >= CORR_USEABLE
            and r is not None
            and math.isfinite(r)
            and r <= r_thr
        )

    def ok_marg(c, r, r_thr):
        return (
            c is not None
            and math.isfinite(c)
            and c >= CORR_MARGINAL
            and r is not None
            and math.isfinite(r)
            and r <= r_thr
        )

    share_u = ok_use(share_c, share_r, RMSE_SHARE_USEABLE)
    share_m = ok_marg(share_c, share_r, RMSE_SHARE_MARGINAL)
    cop_u = ok_use(copy_c, copy_r, RMSE_COPY_USEABLE)
    cop_m = ok_marg(copy_c, copy_r, RMSE_COPY_MARGINAL)
    agree_ok = agree is not None and math.isfinite(agree) and agree >= 0.80

    notes.append(
        f"stand disturb share_sup vs truth: corr={share_c:.3f} RMSE={share_r:.3f}"
        if share_c is not None and math.isfinite(share_c or float("nan"))
        else "stand disturb share_sup: insufficient samples"
    )
    se = m.get("share_sup_ema_vs_T", {})
    if se.get("corr") is not None and math.isfinite(se.get("corr") or float("nan")):
        notes.append(
            f"stand disturb share_sup EMA(α=0.2): corr={se['corr']:.3f} RMSE={se['rmse']:.3f}"
        )
    notes.append(
        f"stand disturb CoP-y (ankle-τ proxy) vs truth: corr={copy_c:.3f} RMSE={copy_r:.4f} m"
        if copy_c is not None and math.isfinite(copy_c or float("nan"))
        else "stand disturb CoP-y: insufficient samples"
    )
    if agree is not None and math.isfinite(agree):
        notes.append(f"stand majority-load agreement={agree:.3f}")

    walk_share_u = walk_share_m = False
    if walk_sum and "metrics" in walk_sum:
        ws = walk_sum["metrics"].get("share_sup_vs_T", {})
        we = walk_sum["metrics"].get("share_sup_ema_vs_T", {})
        wc, wr = ws.get("corr"), ws.get("rmse")
        wec, wer = we.get("corr"), we.get("rmse")
        walk_share_u = ok_use(wc, wr, RMSE_SHARE_USEABLE) or ok_use(
            wec, wer, RMSE_SHARE_USEABLE
        )
        walk_share_m = ok_marg(wc, wr, RMSE_SHARE_MARGINAL) or ok_marg(
            wec, wer, RMSE_SHARE_MARGINAL
        )
        if wc is not None and math.isfinite(wc or float("nan")):
            notes.append(
                f"walk share_sup vs truth: corr={wc:.3f} RMSE={wr:.3f} "
                f"(t_final={walk_sum.get('t_final')}, tip_t={walk_sum.get('tip_t')})"
            )
            if wec is not None and math.isfinite(wec or float("nan")):
                notes.append(
                    f"walk share_sup EMA(α=0.2): corr={wec:.3f} RMSE={wer:.3f}"
                )
            wcy = walk_sum["metrics"].get("cop_y_ankle_vs_T", {})
            if wcy.get("corr") is not None and math.isfinite(wcy.get("corr") or float("nan")):
                notes.append(
                    f"walk CoP-y ankle-τ: corr={wcy['corr']:.3f} RMSE={wcy['rmse']:.4f} m"
                )
        else:
            notes.append("walk: no usable share metric")

    # Primary: stance unload/load-share for skate detection; CoP-y secondary
    if share_u and agree_ok and (walk_sum is None or walk_share_u or walk_share_m):
        if cop_u or cop_m:
            return "USEABLE", notes
        return "USEABLE", notes + [
            "CoP-xy ankle-τ weak but L/R load-share strong enough for unload/skate detect"
        ]
    if share_m or (share_u and not agree_ok) or (cop_m and share_m):
        return "MARGINAL", notes
    return "FAIL", notes


def write_markdown(
    path: Path,
    verdict: str,
    notes: list[str],
    stand_sum: dict,
    walk_sum: dict | None,
    quiet_sum: dict,
    args: argparse.Namespace,
) -> None:
    def fmt_m(block: dict, key: str) -> str:
        m = block.get("metrics", {}).get(key, {})
        c, r, n = m.get("corr"), m.get("rmse"), m.get("n")
        if c is None or (isinstance(c, float) and not math.isfinite(c)):
            return "n/a"
        return f"corr={c:.3f}, RMSE={r:.4f}, n={n}"

    lines = [
        "# CoP proxy falsifier — AiNex kit signals vs MuJoCo truth",
        "",
        f"**When:** Sun 27 Sep 2026 Europe/London (BST)  ",
        f"**Model:** `mujoco/ainex_hiwonder/ainex_controls.xml` (HX ±2.1 leg / ±0.7 arm)  ",
        f"**Script:** `scripts/cop_proxy_check.py`  ",
        f"**Log rate:** {args.hz:.0f} Hz  ",
        "**Scope:** AI sensing lane — no purchases, no Orin, no NN. Path A spend frozen.",
        "",
        "## VERDICT",
        "",
        f"**{verdict}**",
        "",
    ]
    if verdict == "USEABLE":
        lines.append(
            "Proxy (support-chain `|actuator_force|` load-share + optional ankle-τ CoP) "
            "tracks true contact load-share well enough for skate / stance-unload detection "
            "at 50–100 Hz under the tested disturb + early-walk windows."
        )
    elif verdict == "MARGINAL":
        lines.append(
            "Some correlation exists; usable only with filtering / calibration and not "
            "as a sole skate detector. Prefer capture-point from IMU+q, or add FSR."
        )
    else:
        lines.append(
            "Path A ankles remain effectively **blind** for CoP / unload without add-on "
            "sensing (FSR / force ankles). Kit HX bus load + joint q do **not** track "
            "true contact CoP/Fz well enough for skate detection in this falsifier."
        )
    lines += ["", "### Key numbers", ""]
    for n in notes:
        lines.append(f"- {n}")
    lines += [
        "",
        "## Method",
        "",
        "### Truth (MuJoCo)",
        "- Per-foot normal load `Fz` and CoP-xy from `mj_contactForce` on "
        "`l_foot_contact` / `r_foot_contact` (+ toe spheres) vs `floor`.",
        "- Load-share_T = Fz_L / (Fz_L + Fz_R); combined CoP = force-weighted contact positions.",
        "",
        "### Proxy (kit-available only)",
        "- `actuator_force` on position actuators (= HX torque / load channel stand-in).",
        "- Bus load = `|actuator_force|` summed on leg chain.",
        "- **share_sup**: `|τ|` on {knee, ank_pitch, ank_roll, hip_pitch} → L/(L+R).",
        "- **CoP ankle-τ**: split `mg` by share_sup, then local `r ≈ τ/Fz` at each ankle, "
        "mapped to world via foot body xy (quasi-static).",
        "- Joint `q` used only for FK foot xy (same as kit encoders).",
        "",
        "### Episodes",
        f"1. **Quiet stand** settle {args.settle:.2f}s (no disturb) — sanity.",
        f"2. **Stand + disturb:** Fy={args.fy} N, Fx={args.fx} N, hold={args.hold:.2f}s "
        f"at t={args.disturb_t:.2f}s on `body_link` (xfrc); window = disturb→+1.0s or tip.",
        f"3. **Open-loop walk** via `walk_gait_ainex.gait_targets` (no ankle-CoP servo, "
        f"no plant damper — sense-only); log until tip or {args.walk_dur:.1f}s.",
        "",
        "## Results tables",
        "",
        "### Quiet stand (pre-disturb / no push)",
        "",
        f"| Metric | Value |",
        f"|--------|-------|",
        f"| share_sup vs T | {fmt_m(quiet_sum, 'share_sup_vs_T')} |",
        f"| share_bus vs T | {fmt_m(quiet_sum, 'share_bus_vs_T')} |",
        f"| CoP-y ankle-τ vs T | {fmt_m(quiet_sum, 'cop_y_ankle_vs_T')} |",
        f"| CoP-y share-only vs T | {fmt_m(quiet_sum, 'cop_y_share_vs_T')} |",
        f"| majority-load agree | {quiet_sum.get('majority_load_agree')} |",
        "",
        "### Stand disturb window",
        "",
        f"| Metric | Value |",
        f"|--------|-------|",
        f"| window | {stand_sum.get('window')} |",
        f"| tip_t | {stand_sum.get('tip_t')} |",
        f"| share_sup vs T | {fmt_m(stand_sum, 'share_sup_vs_T')} |",
        f"| share_sup EMA(α=0.2) vs T | {fmt_m(stand_sum, 'share_sup_ema_vs_T')} |",
        f"| share_bus vs T | {fmt_m(stand_sum, 'share_bus_vs_T')} |",
        f"| CoP-y ankle-τ vs T | {fmt_m(stand_sum, 'cop_y_ankle_vs_T')} |",
        f"| CoP-y ankle EMA vs T | {fmt_m(stand_sum, 'cop_y_ankle_ema_vs_T')} |",
        f"| CoP-y share-only vs T | {fmt_m(stand_sum, 'cop_y_share_vs_T')} |",
        f"| CoP-x ankle-τ vs T | {fmt_m(stand_sum, 'cop_x_ankle_vs_T')} |",
        f"| Fz_L vs proxy | {fmt_m(stand_sum, 'fz_L_vs_p')} |",
        f"| Fz_R vs proxy | {fmt_m(stand_sum, 'fz_R_vs_p')} |",
        f"| majority-load agree | {stand_sum.get('majority_load_agree')} |",
        f"| CoP-y truth ptp | {stand_sum.get('cop_y_T_ptp')} m |",
        "",
    ]
    if walk_sum:
        lines += [
            "### Walk (first seconds until tip/skate)",
            "",
            f"| Metric | Value |",
            f"|--------|-------|",
            f"| t_final / tip_t | {walk_sum.get('t_final')} / {walk_sum.get('tip_t')} |",
            f"| share_sup vs T | {fmt_m(walk_sum, 'share_sup_vs_T')} |",
            f"| share_sup EMA(α=0.2) vs T | {fmt_m(walk_sum, 'share_sup_ema_vs_T')} |",
            f"| share_bus vs T | {fmt_m(walk_sum, 'share_bus_vs_T')} |",
            f"| CoP-y ankle-τ vs T | {fmt_m(walk_sum, 'cop_y_ankle_vs_T')} |",
            f"| CoP-y ankle EMA vs T | {fmt_m(walk_sum, 'cop_y_ankle_ema_vs_T')} |",
            f"| CoP-y share-only vs T | {fmt_m(walk_sum, 'cop_y_share_vs_T')} |",
            f"| majority-load agree | {walk_sum.get('majority_load_agree')} |",
            "",
        ]
    lines += [
        "## Gates used",
        "",
        f"- USEABLE: share corr ≥ {CORR_USEABLE} and RMSE ≤ {RMSE_SHARE_USEABLE}, "
        f"majority-load agree ≥ 0.80; CoP-y supportive if corr ≥ {CORR_USEABLE} / "
        f"RMSE ≤ {RMSE_COPY_USEABLE} m.",
        f"- MARGINAL: share corr ≥ {CORR_MARGINAL} and RMSE ≤ {RMSE_SHARE_MARGINAL}.",
        "- FAIL: below marginal → ankles blind without add-on sensing.",
        "",
        "## Contact / model notes",
        "",
        "- Foot boxes `l_foot_contact` / `r_foot_contact` already `contype=1 conaffinity=1 "
        "condim=3` in `ainex_controls.xml`; no geom edits required for this run.",
        "- Truth CoP uses contact positions × normal loads; proxy never reads `cfrc_ext` / contacts.",
        "",
        "## Next AI / Controls action",
        "",
    ]
    if verdict == "FAIL":
        lines.append(
            "Treat kit HX bus-load / ankle-τ as a **hard falsifier** for Path-A CoP sensing: "
            "do not plan skate/unload detection on actuator_force alone. Controls should "
            "drive the **capture-point / ZMP regulator from IMU + joint q** (proposal #1) "
            "without relying on a force CoP; optionally word Dave's Path-A note as "
            "*ankles blind without FSR/add-on (~$10–30) if CP-from-IMU still tips <2 s*. "
            "Keep proposal #3 closed unless a cheap FSR trial is greenlit — no Orin, no NN."
        )
    elif verdict == "MARGINAL":
        lines.append(
            "If using the proxy at all, low-pass share_sup (~5–10 Hz) and use it only as a "
            "**binary unload hint** beside an IMU capture-point regulator — not as primary "
            "CoP. Parallel: implement CP/ZMP from IMU+q (proposal #1); keep FSR as the "
            "cheap sensing unlock if CP still fails tip-threshold. No NN, no Orin."
        )
    else:
        lines.append(
            "Feed `share_sup` (and filtered CoP-y if RMSE allows) into the capture-point / "
            "stance machine as an unload/skate flag at 50 Hz alongside IMU lean — still "
            "prefer CP regulation of hip/ankle/swing from IMU+q as the primary stabilizer. "
            "Re-check proxy on hardware HX load registers before claiming kit sensing closed."
        )
    lines += [
        "",
        "## Artifacts",
        "",
        "- `scripts/cop_proxy_check.py`",
        "- `docs/COP_PROXY_FALSIFIER.md` (this file)",
        "- `mujoco/ainex_hiwonder/cop_proxy_stats.json`",
        "",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="AiNex CoP proxy falsifier")
    ap.add_argument("--xml", type=Path, default=XML_DEFAULT)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--settle", type=float, default=0.80)
    ap.add_argument("--disturb-t", type=float, default=1.00)
    ap.add_argument("--hold", type=float, default=0.60)
    ap.add_argument("--fy", type=float, default=3.8, help="lateral disturb force N")
    ap.add_argument("--fx", type=float, default=0.0)
    ap.add_argument("--stand-dur", type=float, default=3.5)
    ap.add_argument("--walk-dur", type=float, default=3.0)
    ap.add_argument("--gait-ramp", type=float, default=0.40)
    ap.add_argument("--no-walk", action="store_true")
    ap.add_argument("--out-docs", type=Path, default=DOCS / "COP_PROXY_FALSIFIER.md")
    ap.add_argument("--out-json", type=Path, default=OUT_DIR / "cop_proxy_stats.json")
    args = ap.parse_args()

    if not args.xml.is_file():
        print(f"ERROR: missing model {args.xml}", file=sys.stderr)
        return 2

    print(f"[cop_proxy] load {args.xml}")
    model = mj.MjModel.from_xml_path(str(args.xml))
    print(
        f"[cop_proxy] nq={model.nq} nv={model.nv} nu={model.nu} "
        f"mass={mj.mj_getTotalmass(model):.4f} kg  hz={args.hz}"
    )

    # --- 1) Quiet stand (no disturb) for baseline ---
    quiet_logs, quiet_meta = run_episode(
        model,
        mode="stand",
        hz=args.hz,
        settle_s=args.settle,
        disturb_t=99.0,
        disturb_hold=0.0,
        disturb_fy=0.0,
        disturb_fx=0.0,
        duration=args.settle + 0.6,
        gait_ramp_s=args.gait_ramp,
    )
    quiet_sum = summarize(quiet_logs, quiet_meta, (0.40, args.settle + 0.5))
    print(
        f"[cop_proxy] quiet: share corr="
        f"{quiet_sum.get('metrics', {}).get('share_sup_vs_T', {}).get('corr')} "
        f"agree={quiet_sum.get('majority_load_agree')}"
    )

    # --- 2) Stand + disturb ---
    stand_logs, stand_meta = run_episode(
        model,
        mode="stand",
        hz=args.hz,
        settle_s=args.settle,
        disturb_t=args.disturb_t,
        disturb_hold=args.hold,
        disturb_fy=args.fy,
        disturb_fx=args.fx,
        duration=args.stand_dur,
        gait_ramp_s=args.gait_ramp,
    )
    t1 = args.disturb_t + args.hold + 0.5
    if stand_meta.get("tip_t") is not None:
        t1 = min(t1, float(stand_meta["tip_t"]))
    stand_sum = summarize(stand_logs, stand_meta, (args.disturb_t, t1))
    print(
        f"[cop_proxy] disturb: tip_t={stand_meta.get('tip_t')} "
        f"share={stand_sum.get('metrics', {}).get('share_sup_vs_T')} "
        f"cop_y={stand_sum.get('metrics', {}).get('cop_y_ankle_vs_T')}"
    )

    # --- 3) Walk snippet ---
    walk_sum = None
    walk_logs = []
    if not args.no_walk:
        walk_logs, walk_meta = run_episode(
            model,
            mode="walk",
            hz=args.hz,
            settle_s=args.settle,
            disturb_t=99.0,
            disturb_hold=0.0,
            disturb_fy=0.0,
            disturb_fx=0.0,
            duration=args.settle + args.walk_dur,
            gait_ramp_s=args.gait_ramp,
        )
        # analyze from gait start to tip/end
        tw0 = args.settle
        # Known skate/tip horizon ~2.3 s of gait; cap analysis there
        tw1 = min(
            float(walk_meta.get("t_final", args.settle + args.walk_dur)),
            args.settle + min(args.walk_dur, 2.30),
        )
        if walk_meta.get("tip_t") is not None:
            tw1 = min(tw1, float(walk_meta["tip_t"]))
        walk_sum = summarize(walk_logs, walk_meta, (tw0, tw1))
        print(
            f"[cop_proxy] walk: tip_t={walk_meta.get('tip_t')} t_final={walk_meta.get('t_final')} "
            f"share={walk_sum.get('metrics', {}).get('share_sup_vs_T')}"
        )

    verdict, notes = grade_verdict(stand_sum, walk_sum)
    print(f"[cop_proxy] VERDICT: {verdict}")
    for n in notes:
        print(f"  - {n}")

    args.out_docs.parent.mkdir(parents=True, exist_ok=True)
    write_markdown(args.out_docs, verdict, notes, stand_sum, walk_sum, quiet_sum, args)
    print(f"[cop_proxy] wrote {args.out_docs}")

    payload = {
        "verdict": verdict,
        "notes": notes,
        "quiet": quiet_sum,
        "stand_disturb": stand_sum,
        "walk": walk_sum,
        "gates": {
            "CORR_USEABLE": CORR_USEABLE,
            "CORR_MARGINAL": CORR_MARGINAL,
            "RMSE_SHARE_USEABLE": RMSE_SHARE_USEABLE,
            "RMSE_SHARE_MARGINAL": RMSE_SHARE_MARGINAL,
            "RMSE_COPY_USEABLE": RMSE_COPY_USEABLE,
            "RMSE_COPY_MARGINAL": RMSE_COPY_MARGINAL,
        },
        "hz": args.hz,
        "model": str(args.xml),
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
    print(f"[cop_proxy] wrote {args.out_json}")
    print("[cop_proxy] DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
