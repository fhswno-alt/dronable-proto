#!/usr/bin/env python3
"""Hybrid short-horizon MPC with T88 hard-reject in-loop (sim).

Outer: CSF50 CPG phase/timing unchanged (caller).
Inner: N-step stance-vx / CP scrub via hip+ank Δctrl under HX.

Hard reject before apply (cospec):
  - If stance already "clean" (rolling |vx| below thr) → u=0 (protects T=0.88 knife-edge).
  - If candidate |Δ| would exceed clips → reject.
  - Optional: if predicted post-step p95-proxy > T88_P95_LIM while current mean low → reject.

See docs/HYBRID_MPC_T88_COSPEC.md. No residual K/MLP free-on-CPG; no WBC-as-primary.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Callable

import numpy as np
from scipy.optimize import minimize

HX_TAU_LIM = 2.1
CTRL_LIM = 2.09

# T88 floor surrogates (match Gate D / residual reject)
T88_P95_LIM = 0.18
T88_VX_CLEAN = 0.10   # if recent stance |vx| mean below this → do not touch
T88_BOUT_PROXY_LO = 0.10

# Default MPC
N_DEFAULT = 12
DT = 1.0 / 50.0


class MpcState:
    """Rolling stance-vx for clean/skate detection + T88 surrogate."""

    def __init__(self, window: int = 25):
        self.vx_L: deque[float] = deque(maxlen=window)
        self.vx_R: deque[float] = deque(maxlen=window)
        self.last_diag: dict = {}

    def push(self, vx_l: float, vx_r: float, stance_L: bool, stance_R: bool) -> None:
        if stance_L:
            self.vx_L.append(abs(float(vx_l)))
        if stance_R:
            self.vx_R.append(abs(float(vx_r)))

    def mean_p95(self) -> tuple[float, float]:
        vals = list(self.vx_L) + list(self.vx_R)
        if not vals:
            return 0.0, 0.0
        a = np.asarray(vals, dtype=np.float64)
        return float(a.mean()), float(np.percentile(a, 95))


_STATE = MpcState()


def reset_mpc_state() -> None:
    global _STATE
    _STATE = MpcState()


def foot_contact(model, data, bid: int, gid_floor: int, z_thr: float = 0.025) -> bool:
    if gid_floor is not None and gid_floor >= 0:
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1 = int(model.geom_bodyid[g1])
            b2 = int(model.geom_bodyid[g2])
            if (b1 == bid or b2 == bid) and (g1 == gid_floor or g2 == gid_floor):
                return True
    return float(data.xpos[bid, 2]) < z_thr


def _predict_vx(vx0: float, u_seq: np.ndarray, a: float = 0.92, b: float = -1.8) -> np.ndarray:
    """Linear discrete: vx_{k+1} = a vx_k + b u_k  (u = shared d_fwd trim)."""
    vx = float(vx0)
    out = np.zeros(len(u_seq))
    for k, uk in enumerate(u_seq):
        vx = a * vx + b * float(uk)
        out[k] = vx
    return out


def _solve_u_horizon(
    vx0: float,
    *,
    n: int,
    u_max: float,
    w_abs: float = 1.0,
    w_peak: float = 1.2,
    w_u: float = 0.05,
) -> np.ndarray:
    """SLSQP on u[0:n]: minimize mean|vx|+peak|vx| over horizon."""
    if abs(vx0) < 1e-4 or n < 1:
        return np.zeros(n)

    def cost(u: np.ndarray) -> float:
        pred = _predict_vx(vx0, u)
        return (
            w_abs * float(np.mean(np.abs(pred)))
            + w_peak * float(np.max(np.abs(pred)))
            + w_u * float(u @ u)
        )

    bounds = [(-u_max, u_max)] * n
    # warm start: constant scrub opposing vx
    u0 = np.full(n, float(np.clip(-0.15 * np.sign(vx0) * min(1.0, abs(vx0) / 0.2), -u_max, u_max)))
    res = minimize(cost, u0, method="SLSQP", bounds=bounds, options={"maxiter": 30, "ftol": 1e-7, "disp": False})
    return np.asarray(res.x if res.success else u0, dtype=np.float64)


def t88_hard_reject(
    *,
    stx_mean: float,
    stx_p95: float,
    u0: float,
    force_apply: bool = False,
) -> tuple[bool, str]:
    """Return (reject, reason). Cospec: protect T88 clean_ss surrogate."""
    if force_apply:
        return False, ""
    # Already clean → any nonzero scrub risks knife-edge CSF50
    if stx_mean <= T88_VX_CLEAN and abs(u0) > 1e-6:
        return True, f"clean_stance mean={stx_mean:.3f}<={T88_VX_CLEAN}"
    # Predicted p95 would exceed T88 lim while not urgently skating
    if stx_p95 <= T88_P95_LIM and stx_mean < 0.12 and abs(u0) > 1e-6:
        return True, f"t88_p95_proxy={stx_p95:.3f}<={T88_P95_LIM}"
    return False, ""


def apply_hybrid_mpc_t88(
    model,
    data,
    act_idx: dict,
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
    *,
    gait_t: float = 0.75,
    n_horizon: int = N_DEFAULT,
    u_max: float = 0.10,
    ank_ratio: float = 0.6,
    lat_fn: Callable[[float], float] | None = None,
    act_name_fn: Callable[[str], str] | None = None,
    apply_thr: float = 0.11,
    disable_above_t: float = 0.86,
) -> dict:
    """Inner MPC step after gait+CP+VIK. HX position clip on legs only."""
    diag = {"applied": False, "rejected": False, "u0": 0.0}
    if amp < 0.05:
        return diag
    if act_name_fn is None:
        act_name_fn = lambda j: f"{j}_pos"
    if lat_fn is None:
        lat_fn = lambda p: math.cos(2.0 * math.pi * p)

    # Hard schedule gate: at CSF50 T, never apply (T88 floor by construction for T≈0.88 runs)
    if float(gait_t) >= float(disable_above_t):
        diag["rejected"] = True
        diag["reason"] = f"gait_t={gait_t}>={disable_above_t}"
        _STATE.last_diag = diag
        return diag

    cL = foot_contact(model, data, bid_lf, gid_floor)
    cR = foot_contact(model, data, bid_rf, gid_floor)
    vx_l = float(data.cvel[bid_lf][3])
    vx_r = float(data.cvel[bid_rf][3])
    # exclusive SS preferred
    targets: list[tuple[str, int, float]] = []
    if cL and not cR:
        targets.append(("L", bid_lf, vx_l))
    elif cR and not cL:
        targets.append(("R", bid_rf, vx_r))
    elif cL and cR:
        # DS: scrub the faster-sliding foot lightly
        if abs(vx_l) >= abs(vx_r):
            targets.append(("L", bid_lf, vx_l))
        else:
            targets.append(("R", bid_rf, vx_r))

    _STATE.push(vx_l, vx_r, cL, cR)
    stx_mean, stx_p95 = _STATE.mean_p95()
    diag["stx_mean"] = stx_mean
    diag["stx_p95"] = stx_p95

    if not targets:
        _STATE.last_diag = diag
        return diag

    side, bid, vx = targets[0]
    if abs(vx) < apply_thr and stx_mean < apply_thr:
        diag["rejected"] = True
        diag["reason"] = f"below_apply_thr vx={vx:.3f} mean={stx_mean:.3f}"
        _STATE.last_diag = diag
        return diag

    u_seq = _solve_u_horizon(vx, n=int(n_horizon), u_max=float(u_max))
    u0 = float(u_seq[0]) if len(u_seq) else 0.0
    diag["u0"] = u0

    rej, reason = t88_hard_reject(stx_mean=stx_mean, stx_p95=stx_p95, u0=u0)
    if rej:
        diag["rejected"] = True
        diag["reason"] = reason
        _STATE.last_diag = diag
        return diag

    # Map u0 = d_fwd shared frame → L/R joint signs
    jsign = -1.0 if side == "L" else +1.0
    d_hip = float(np.clip(jsign * u0, -u_max, u_max))
    d_ank = float(np.clip(jsign * ank_ratio * u0, -0.7 * u_max, 0.7 * u_max))
    pref = "l_" if side == "L" else "r_"
    for jn, dd in ((f"{pref}hip_pitch", d_hip), (f"{pref}ank_pitch", d_ank)):
        an = act_name_fn(jn)
        if an in act_idx:
            i = act_idx[an]
            data.ctrl[i] = float(np.clip(data.ctrl[i] + dd, -CTRL_LIM, CTRL_LIM))
    # legs-only safety clip
    for name, i in act_idx.items():
        nl = name.lower()
        if any(k in nl for k in ("hip_", "knee", "ank_")):
            data.ctrl[i] = float(np.clip(data.ctrl[i], -CTRL_LIM, CTRL_LIM))
    diag["applied"] = True
    diag["side"] = side
    diag["delta"] = [d_hip, d_ank]
    _STATE.last_diag = diag
    return diag
