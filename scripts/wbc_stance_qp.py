#!/usr/bin/env python3
"""Stance friction-cone wrench QP → joint position trim (sim WBC-lite).

Replaces additive residual-on-CPG for Gate E attempts. See docs/WBC_STANCE_QP_PLAN.md.

Flow:
  1) Desired CoM / CP wrench from gait phase + DCM tracking.
  2) Solve foot forces in friction pyramid (μ).
  3) Map f → τ = Jᵀ f → Δctrl under position actuators; HX forcerange is MuJoCo's;
     we clip |τ|≤2.1 and position setpoints to ±2.09 legs only.

No xfrc / assist / freeze. No residual K/MLP.
"""
from __future__ import annotations

import math
from typing import Callable

import numpy as np
from scipy.optimize import minimize

# Leg joint order for Jacobian columns
LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll")

# Position-actuator kp proxies (match XML order of magnitude)
KP_PROXY = {
    "hip_yaw": 40.0,
    "hip_roll": 40.0,
    "hip_pitch": 45.0,
    "knee": 45.0,
    "ank_pitch": 35.0,
    "ank_roll": 35.0,
}

HX_TAU = 2.1
CTRL_LIM = 2.09


def foot_floor_contact(model, data, bid_foot, gid_floor, z_thr: float = 0.025) -> bool:
    if gid_floor is not None and gid_floor >= 0:
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1 = int(model.geom_bodyid[g1])
            b2 = int(model.geom_bodyid[g2])
            if (b1 == bid_foot or b2 == bid_foot) and (g1 == gid_floor or g2 == gid_floor):
                return True
    return float(data.xpos[bid_foot, 2]) < z_thr


def _leg_dof_cols(model, side: str) -> tuple[list[str], list[int]]:
    import mujoco as mj
    pref = "l_" if side == "L" else "r_"
    names, cols = [], []
    for jn in LEG_JOINTS:
        full = f"{pref}{jn}"
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, full)
        if jid < 0:
            continue
        names.append(full)
        cols.append(int(model.jnt_dofadr[jid]))
    return names, cols


def desired_com_wrench(
    model,
    data,
    *,
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
    mass: float,
    lat_fn: Callable[[float], float],
    omega: float,
    kp_com: float = 40.0,
    kd_com: float = 8.0,
    scrub_k: float = 25.0,
) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """Return (f_des_world[3], n_des_world[3], stance_sides).

    f_des from CoM PD toward support + gravity; soft skate scrub baked into
    later per-foot cost, not here.
    """
    com = np.asarray(data.subtree_com[0], dtype=np.float64)
    v = np.asarray(data.qvel[0:3], dtype=np.float64)
    lat = float(lat_fn(phi))
    cL = foot_floor_contact(model, data, bid_lf, gid_floor)
    cR = foot_floor_contact(model, data, bid_rf, gid_floor)
    sides: list[str] = []
    # Prefer true SS (exclusive contact) — DS uses both but weaker scrub later
    if cL and not cR:
        sides = ["L"]
    elif cR and not cL:
        sides = ["R"]
    elif cL and cR:
        sides = ["L", "R"]
    else:
        if lat >= 0:
            sides = ["L"]
        else:
            sides = ["R"]

    # Support center (stance feet only)
    pts = []
    if "L" in sides:
        pts.append(data.xpos[bid_lf, :2].copy())
    if "R" in sides:
        pts.append(data.xpos[bid_rf, :2].copy())
    support = np.mean(np.stack(pts, axis=0), axis=0)

    # DCM / CP
    w = max(float(omega), 0.5)
    cp = com[:2] + v[:2] / w
    # Want CP → support; equivalent accel command on CoM
    # a ≈ ω² (support - cp) scaled — mild gains under HX
    a_xy = (w * w) * (support - cp) * float(amp)
    # Blend with CoM PD toward support
    a_xy = 0.55 * a_xy + 0.45 * (
        kp_com * (support - com[:2]) + kd_com * (0.0 - v[:2])
    ) / max(mass, 1e-3)
    a_xy = np.clip(a_xy, -6.0, 6.0)

    f_des = np.array([
        mass * float(a_xy[0]),
        mass * float(a_xy[1]),
        mass * 9.81,
    ], dtype=np.float64)
    # Soft yaw/roll moment desire ≈ 0 about CoM (keep upright)
    n_des = np.zeros(3, dtype=np.float64)
    return f_des, n_des, sides


def solve_stance_forces(
    *,
    foot_pos: dict[str, np.ndarray],
    com: np.ndarray,
    f_des: np.ndarray,
    n_des: np.ndarray,
    sides: list[str],
    mu: float,
    foot_vx: dict[str, float] | None = None,
    scrub_k: float = 18.0,
    w_force: float = 1.0,
    w_moment: float = 0.35,
    w_reg: float = 1e-3,
    w_scrub: float = 0.25,
) -> dict[str, np.ndarray]:
    """Friction-pyramid QP for stance foot forces. Returns {side: f_xyz}."""
    if not sides:
        return {}
    n = len(sides)
    # x = [fx,fy,fz] * n
    mu = float(max(mu, 0.05))

    def unpack(x: np.ndarray) -> dict[str, np.ndarray]:
        out = {}
        for i, s in enumerate(sides):
            out[s] = x[3 * i : 3 * i + 3].copy()
        return out

    def cost(x: np.ndarray) -> float:
        forces = unpack(x)
        f_sum = np.zeros(3)
        n_sum = np.zeros(3)
        for s, f in forces.items():
            f_sum += f
            r = foot_pos[s] - com
            n_sum += np.cross(r, f)
        e_f = f_sum - f_des
        e_n = n_sum - n_des
        c = w_force * float(e_f @ e_f) + w_moment * float(e_n @ e_n) + w_reg * float(x @ x)
        if foot_vx and w_scrub > 0:
            for s, f in forces.items():
                # Prefer horizontal force opposing foot slip (inside cone via constraints)
                vx = float(foot_vx.get(s, 0.0))
                # target fx ≈ -scrub_k * vx * (fz/mg scale soft)
                fx_tgt = -scrub_k * vx
                c += w_scrub * (f[0] - fx_tgt) ** 2
        return c

    cons = []
    for i, s in enumerate(sides):
        b = 3 * i

        def fz_pos(x, b=b):
            return x[b + 2] - 0.5  # at least 0.5 N

        def cone_px(x, b=b, m=mu):
            return m * x[b + 2] - x[b]

        def cone_nx(x, b=b, m=mu):
            return m * x[b + 2] + x[b]

        def cone_py(x, b=b, m=mu):
            return m * x[b + 2] - x[b + 1]

        def cone_ny(x, b=b, m=mu):
            return m * x[b + 2] + x[b + 1]

        cons.extend([
            {"type": "ineq", "fun": fz_pos},
            {"type": "ineq", "fun": cone_px},
            {"type": "ineq", "fun": cone_nx},
            {"type": "ineq", "fun": cone_py},
            {"type": "ineq", "fun": cone_ny},
        ])

    # Soft vertical load share init
    fz0 = float(f_des[2]) / max(n, 1)
    x0 = np.zeros(3 * n)
    for i in range(n):
        x0[3 * i + 2] = max(fz0, 1.0)

    res = minimize(
        cost,
        x0,
        method="SLSQP",
        constraints=cons,
        options={"maxiter": 40, "ftol": 1e-7, "disp": False},
    )
    x = res.x if res.success else x0
    # Project into cone if solver soft-failed
    out = unpack(x)
    for s, f in out.items():
        fz = max(float(f[2]), 0.5)
        fx = float(np.clip(f[0], -mu * fz, mu * fz))
        fy = float(np.clip(f[1], -mu * fz, mu * fz))
        out[s] = np.array([fx, fy, fz], dtype=np.float64)
    return out


def forces_to_ctrl_trim(
    model,
    data,
    act_idx: dict,
    *,
    forces: dict[str, np.ndarray],
    bid: dict[str, int],
    act_name_fn: Callable[[str], str],
    trim_clip: float = 0.12,
    tau_scale: float = 1.0,
) -> dict:
    """Apply Δctrl from τ = Jᵀ f / kp on stance legs. Returns diag."""
    import mujoco as mj

    diag = {"applied": False, "tau_max": 0.0}
    jacp = np.zeros((3, model.nv))
    for side, f in forces.items():
        names, cols = _leg_dof_cols(model, side)
        if len(cols) < 2:
            continue
        mj.mj_jacBody(model, data, jacp, None, bid[side])
        J = jacp[:, cols].copy()  # 3 x n
        tau = J.T @ f  # n
        tau = tau_scale * tau
        diag["tau_max"] = max(diag["tau_max"], float(np.max(np.abs(tau))))
        for jn, ti in zip(names, tau):
            # HX torque authority
            ti = float(np.clip(ti, -HX_TAU, HX_TAU))
            short = jn.split("_", 1)[1] if "_" in jn else jn  # hip_pitch etc after l_/r_
            # jn is l_hip_pitch → kp key hip_pitch
            key = "_".join(jn.split("_")[1:])
            kp = KP_PROXY.get(key, 40.0)
            dq = float(np.clip(ti / max(kp, 1.0), -trim_clip, trim_clip))
            an = act_name_fn(jn)
            if an not in act_idx:
                continue
            i = act_idx[an]
            data.ctrl[i] = float(np.clip(data.ctrl[i] + dq, -CTRL_LIM, CTRL_LIM))
            diag["applied"] = True
            diag[f"dq_{jn}"] = dq
    # legs only clip (never crush arms)
    for name, i in act_idx.items():
        nl = name.lower()
        if any(k in nl for k in ("hip_", "knee", "ank_")):
            data.ctrl[i] = float(np.clip(data.ctrl[i], -CTRL_LIM, CTRL_LIM))
    return diag


def apply_wbc_stance_qp(
    model,
    data,
    act_idx: dict,
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
    *,
    mu: float = 0.8,
    mass: float | None = None,
    lat_fn: Callable[[float], float] | None = None,
    omega: float = 3.2,
    scrub_k: float = 18.0,
    trim_clip: float = 0.12,
    tau_scale: float = 1.0,
    kp_com: float = 40.0,
    kd_com: float = 8.0,
    act_name_fn: Callable[[str], str] | None = None,
    w_scrub: float = 0.35,
) -> dict:
    """Full stance WBC-lite step. Call after gait+CP (+ optional VIK)."""
    import mujoco as mj

    diag = {"applied": False, "mu": mu}
    if amp < 0.05:
        return diag
    if act_name_fn is None:
        act_name_fn = lambda j: f"{j}_pos"
    if lat_fn is None:
        lat_fn = lambda p: math.cos(2.0 * math.pi * p)
    if mass is None:
        mass = float(mj.mj_getTotalmass(model))

    f_des, n_des, sides = desired_com_wrench(
        model, data,
        phi=phi, amp=amp, bid_lf=bid_lf, bid_rf=bid_rf, gid_floor=gid_floor,
        mass=mass, lat_fn=lat_fn, omega=omega, kp_com=kp_com, kd_com=kd_com,
        scrub_k=scrub_k,
    )
    diag["sides"] = list(sides)
    diag["f_des"] = f_des.tolist()

    bid = {"L": bid_lf, "R": bid_rf}
    foot_pos = {s: np.asarray(data.xpos[bid[s]], dtype=np.float64).copy() for s in sides}
    com = np.asarray(data.subtree_com[0], dtype=np.float64).copy()
    foot_vx = {s: float(data.cvel[bid[s]][3]) for s in sides}

    # Soften scrub in double support to protect CSF50 floor
    w_sc = w_scrub if len(sides) == 1 else 0.15 * w_scrub
    forces = solve_stance_forces(
        foot_pos=foot_pos,
        com=com,
        f_des=f_des,
        n_des=n_des,
        sides=sides,
        mu=mu,
        foot_vx=foot_vx,
        scrub_k=scrub_k,
        w_scrub=w_sc,
    )
    diag["forces"] = {s: forces[s].tolist() for s in forces}
    trim = forces_to_ctrl_trim(
        model, data, act_idx,
        forces=forces, bid=bid, act_name_fn=act_name_fn,
        trim_clip=trim_clip, tau_scale=tau_scale,
    )
    diag.update(trim)
    return diag
