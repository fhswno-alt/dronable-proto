"""Contact stepping bars for the #102 walk scorer.

Period in ``op3_walk.update_time`` is one left-plus-right cycle: both
single-support windows sit inside ``period``. ``gait_manager_traj`` says the
same thing. Each foot therefore swings once per period. A no-slip walk at
speed ``vx`` places that foot ``vx * period`` further along x. That is the
per-swing airborne travel.

``vx * period / 2`` is the spacing between opposite footfalls, the
stance-to-stance step. At 0.056 m/s and 3.60 s that spacing is 0.1008 m.
It is not the distance one swing foot travels.

The IK does not command either of those. ``kit_bus_step`` sets
``x_amp = min(0.020, vx / 7.50)``, and the swing sine runs about
``±x_amp`` relative to the hip, so the commanded foot travel is about
``2 * x_amp`` (15 mm at vx 0.056).
"""
from __future__ import annotations

import math

import numpy as np

STEP_FRACTION_BAR = 0.90
SLIP_BAR_M = 0.002
CLEAR_BAR_M = 0.008
STRIDE_TOL = 0.20
SS_CONTACT_BAR = 3
EDGE_DWELL_M = 0.005
EDGE_ZERO_M = 0.0001
TILT_BAR_RAD = math.radians(1.0)
UPRIGHT_MATCH_RAD = math.radians(5.0)
STOP_UP_Z = 0.90
KIT_BODY_PER_X = 7.50
KIT_X_RAIL_M = 0.020
# HX-35H no-load speed is 0.18 s per 60°. 60° is π/3 rad, so the rate is
# (π/3)/0.18 ≈ 5.8178 rad/s. The bar is the stated 5.82 rad/s. The plant
# has no joint velocity limit; this cutoff lives in the scorer.
LEG_QVEL_BAR = 5.82
# Corner ticks reported while the speed-torque line is unset. These are
# report thresholds, not a motor curve.
TAU_CORNER_NM = 2.0
QVEL_CORNER_RAD_S = 4.0
SPEED_TORQUE_PENDING = "speed-torque line pending HW datasheet"


def hinge_speed_report(
    names: tuple[str, ...] | list[str],
    t: np.ndarray,
    omega_abs: np.ndarray,
    stage: np.ndarray,
) -> dict[str, object]:
    """Peak |qvel| on each leg hinge against ``LEG_QVEL_BAR``.

    ``omega_abs`` has shape ``(n, len(names))`` and is already absolute.
    ``stage`` is one label per tick: the preview stage on a preview row, or
    the gait phase on a voice bout that has no preview stage. Headroom is
    ``LEG_QVEL_BAR - peak``. A joint at the bar still passes.
    """
    labels = [str(name) for name in names]
    times = np.asarray(t, dtype=np.float64)
    omega = np.asarray(omega_abs, dtype=np.float64)
    stages = np.asarray(stage, dtype=object)
    reasons: list[str] = []
    joints: list[dict[str, object]] = []
    if times.shape[0] < 1 or omega.ndim != 2 or omega.shape[0] != times.shape[0]:
        reasons.append("no hinge-speed samples")
        return {"bar_rad_s": LEG_QVEL_BAR, "joints": joints, "fail_reasons": reasons, "passes": False}
    if omega.shape[1] != len(labels) or stages.shape[0] != times.shape[0]:
        reasons.append("hinge-speed trace does not cover the 12 leg hinges")
        return {"bar_rad_s": LEG_QVEL_BAR, "joints": joints, "fail_reasons": reasons, "passes": False}
    for col, name in enumerate(labels):
        series = omega[:, col]
        idx = int(np.argmax(series))
        peak = float(series[idx])
        headroom = float(LEG_QVEL_BAR - peak)
        label = str(stages[idx])
        joints.append({
            "joint": name,
            "peak_rad_s": peak,
            "t_s": float(times[idx]),
            "stage": label,
            "headroom_rad_s": headroom,
        })
        if peak > LEG_QVEL_BAR + 1e-12:
            reasons.append(
                f"peak |qvel| {peak:.4f} rad/s on {name} at {float(times[idx]):.3f} s "
                f"stage {label} is over {LEG_QVEL_BAR:.2f} rad/s "
                f"(headroom {headroom:.4f} rad/s)"
            )
    joints.sort(key=lambda row: float(row["peak_rad_s"]), reverse=True)
    return {
        "bar_rad_s": LEG_QVEL_BAR,
        "joints": joints,
        "fail_reasons": reasons,
        "passes": not reasons,
    }


def _corner_tick(
    times: np.ndarray,
    tau: np.ndarray,
    omega: np.ndarray,
    stages: np.ndarray,
    pick: np.ndarray,
    *,
    rank: np.ndarray,
) -> dict[str, object] | None:
    if not np.any(pick):
        return None
    chosen = np.flatnonzero(pick)
    idx = int(chosen[int(np.argmax(rank[pick]))])
    return {
        "t_s": float(times[idx]),
        "tau_nm": float(tau[idx]),
        "qvel_rad_s": float(omega[idx]),
        "stage": str(stages[idx]),
    }


def speed_torque_check(
    names: tuple[str, ...] | list[str],
    t: np.ndarray,
    tau_abs: np.ndarray,
    omega_abs: np.ndarray,
    stage: np.ndarray,
    *,
    no_load_speed: float | None = None,
    stall_torque: float | None = None,
    voltage: float | None = None,
) -> dict[str, object]:
    """Speed-torque check. The motor line stays unset until HW supplies it.

    ``tau_abs`` is the physical servo torque ``|kp·(q_des−q) − kv·ω|``.
    The sum ``|kp·e| + |kv·ω|`` is a separate conservative column and is
    not what this check compares.

    With ``no_load_speed``, ``stall_torque``, or ``voltage`` left None, the
    status is ``SPEED_TORQUE_PENDING`` and nothing is failed. Each joint
    still reports the tick with the largest ``|qvel|`` at signed ``|τ| ≥ 2``
    Nm and the tick with the largest signed ``|τ|`` at ``|qvel| ≥ 4`` rad/s.

    When all three are set, each tick must satisfy
    ``|qvel| ≤ no_load_speed * (1 − |τ| / stall_torque)`` with that signed
    ``|τ|``. Voltage is recorded and does not scale that line. The worst
    margin is folded into the row verdict.
    """
    labels = [str(name) for name in names]
    times = np.asarray(t, dtype=np.float64)
    tau = np.asarray(tau_abs, dtype=np.float64)
    omega = np.asarray(omega_abs, dtype=np.float64)
    stages = np.asarray(stage, dtype=object)
    pending = no_load_speed is None or stall_torque is None or voltage is None
    empty: dict[str, object] = {
        "status": SPEED_TORQUE_PENDING if pending else "speed-torque line",
        "no_load_speed": no_load_speed,
        "stall_torque": stall_torque,
        "voltage": voltage,
        "tau_corner_nm": TAU_CORNER_NM,
        "qvel_corner_rad_s": QVEL_CORNER_RAD_S,
        "corners": [],
        "worst": None,
        "fail_reasons": [],
        "passes": True,
    }
    if times.shape[0] < 1 or tau.ndim != 2 or tau.shape != omega.shape or tau.shape[1] != len(labels):
        empty["fail_reasons"] = ["no speed-torque samples"]
        empty["passes"] = False
        empty["status"] = "no speed-torque samples"
        return empty
    if stages.shape[0] != times.shape[0]:
        empty["fail_reasons"] = ["speed-torque stage trace does not match the ticks"]
        empty["passes"] = False
        empty["status"] = "no speed-torque samples"
        return empty
    corners: list[dict[str, object]] = []
    for col, name in enumerate(labels):
        joint_tau = tau[:, col]
        joint_omega = omega[:, col]
        known = np.isfinite(joint_tau) & np.isfinite(joint_omega)
        corners.append({
            "joint": name,
            "max_qvel_at_tau": _corner_tick(
                times, joint_tau, joint_omega, stages,
                known & (joint_tau >= TAU_CORNER_NM - 1e-12),
                rank=joint_omega,
            ),
            "max_tau_at_qvel": _corner_tick(
                times, joint_tau, joint_omega, stages,
                known & (joint_omega >= QVEL_CORNER_RAD_S - 1e-12),
                rank=joint_tau,
            ),
        })
    empty["corners"] = corners
    if pending:
        return empty
    no_load = float(no_load_speed)
    stall = float(stall_torque)
    if not math.isfinite(no_load) or not math.isfinite(stall) or stall <= 0.0 or no_load < 0.0:
        empty["status"] = "speed-torque line is not usable"
        empty["fail_reasons"] = ["speed-torque line is not usable"]
        empty["passes"] = False
        return empty
    worst: dict[str, object] | None = None
    for col, name in enumerate(labels):
        joint_tau = tau[:, col]
        joint_omega = omega[:, col]
        known = np.isfinite(joint_tau) & np.isfinite(joint_omega)
        for idx in np.flatnonzero(known):
            limit = no_load * (1.0 - float(joint_tau[idx]) / stall)
            margin = limit - float(joint_omega[idx])
            if worst is None or margin < float(worst["margin_rad_s"]):
                worst = {
                    "joint": name,
                    "t_s": float(times[idx]),
                    "stage": str(stages[idx]),
                    "tau_nm": float(joint_tau[idx]),
                    "qvel_rad_s": float(joint_omega[idx]),
                    "limit_rad_s": float(limit),
                    "margin_rad_s": float(margin),
                }
    empty["worst"] = worst
    if worst is not None and float(worst["margin_rad_s"]) < -1e-12:
        empty["fail_reasons"] = [
            f"speed-torque margin {float(worst['margin_rad_s']):.4f} rad/s on "
            f"{worst['joint']} at {float(worst['t_s']):.3f} s stage {worst['stage']} "
            f"(|τ| {float(worst['tau_nm']):.4f} Nm, |qvel| {float(worst['qvel_rad_s']):.4f} rad/s, "
            f"limit {float(worst['limit_rad_s']):.4f} rad/s)"
        ]
        empty["passes"] = False
    return empty


def forcerange_clamp_report(
    names: tuple[str, ...] | list[str],
    force: np.ndarray,
    lo: np.ndarray,
    hi: np.ndarray,
    *,
    eps_nm: float = 1e-3,
) -> list[dict[str, object]]:
    """Fraction of ticks whose applied actuator force sits on forcerange.

    ``force`` is ``data.actuator_force`` after the control tick, shape
    ``(n, n_joints)``. A tick is clamped when that force is within
    ``eps_nm`` of the low or high rail. The denominator is every tick in
    the bout.
    """
    labels = [str(name) for name in names]
    applied = np.asarray(force, dtype=np.float64)
    low = np.asarray(lo, dtype=np.float64)
    high = np.asarray(hi, dtype=np.float64)
    rows: list[dict[str, object]] = []
    n = int(applied.shape[0]) if applied.ndim == 2 else 0
    for col, name in enumerate(labels):
        if applied.ndim != 2 or applied.shape[1] <= col or n < 1:
            rows.append({
                "joint": name,
                "forcerange_lo_nm": None,
                "forcerange_hi_nm": None,
                "n_clamped": 0,
                "n_ticks": n,
                "fraction": None,
            })
            continue
        series = applied[:, col]
        rail_lo = float(low[col])
        rail_hi = float(high[col])
        on_rail = np.isfinite(series) & (
            (series >= rail_hi - eps_nm) | (series <= rail_lo + eps_nm)
        )
        n_clamped = int(np.sum(on_rail))
        finite = series[np.isfinite(series)]
        peak = float(np.max(np.abs(finite))) if finite.size else None
        rows.append({
            "joint": name,
            "forcerange_lo_nm": rail_lo,
            "forcerange_hi_nm": rail_hi,
            "n_clamped": n_clamped,
            "n_ticks": n,
            "fraction": float(n_clamped / n),
            "peak_abs_nm": peak,
        })
    return rows


def trunk_speed_line(
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    mode: np.ndarray,
    *,
    period_s: float,
    vx_cmd_m_s: float,
) -> dict[str, object]:
    """Actual trunk speed over the bus ``move`` window.

    Forward is the trunk x axis, ``(cos yaw, sin yaw)``, taken at the later
    tick. Displacement sums those steps between adjacent ticks that are both
    in ``move``. Time is the sum of those step durations. The ratio is
    actual divided by commanded ``vx``. The line is a report. It is not a
    cutoff: a slow step still fails the existing ``vx·T`` stride bar.
    """
    times = np.asarray(t, dtype=np.float64)
    xx = np.asarray(x, dtype=np.float64)
    yy = np.asarray(y, dtype=np.float64)
    heading = np.asarray(yaw, dtype=np.float64)
    moving = np.array([str(item) == "move" for item in np.asarray(mode, dtype=object)])
    fwd = 0.0
    dur = 0.0
    n = int(times.shape[0])
    if xx.shape[0] != n or yy.shape[0] != n or heading.shape[0] != n or moving.shape[0] != n:
        n = 0
    for i in range(1, n):
        if not (bool(moving[i]) and bool(moving[i - 1])):
            continue
        dx = float(xx[i] - xx[i - 1])
        dy = float(yy[i] - yy[i - 1])
        c = math.cos(float(heading[i]))
        s = math.sin(float(heading[i]))
        fwd += dx * c + dy * s
        dur += float(times[i] - times[i - 1])
    actual = (fwd / dur) if dur > 1e-9 else None
    cmd = float(vx_cmd_m_s)
    ratio = (actual / cmd) if actual is not None and abs(cmd) > 1e-9 else None
    return {
        "period_s": float(period_s),
        "vx_cmd_m_s": cmd,
        "actual_vx_m_s": actual,
        "ratio": ratio,
        "fwd_m": fwd,
        "move_s": dur,
        "axis": "trunk heading",
    }


def expected_steps(vx_m_s: float, period_s: float) -> dict[str, float]:
    """World-frame distances implied by a full L+R period."""
    vx = float(vx_m_s)
    period = float(period_s)
    x_amp = 0.0
    if abs(vx) > 1e-4:
        x_amp = min(KIT_X_RAIL_M, abs(vx) / KIT_BODY_PER_X)
    return {
        "period_s": period,
        "vx_m_s": vx,
        "per_swing_m": vx * period,
        "stance_to_stance_m": vx * period / 2.0,
        "commanded_x_amp_m": x_amp,
        "commanded_hip_travel_m": 2.0 * x_amp,
    }


def _pct(values: np.ndarray, q: float) -> float | None:
    if values.size == 0:
        return None
    return float(np.percentile(values, q))


def summarize_trace(
    trace: dict[str, np.ndarray],
    *,
    vx_m_s: float,
    period_s: float,
    q_stand_err: np.ndarray | None = None,
) -> dict[str, object]:
    """Reduce one bout's per-tick contact trace to the stepping bars.

    ``trace`` arrays share one length. Required keys: ``t``, ``x_l``, ``x_r``,
    ``y_l``, ``y_r``, ``n_l``, ``n_r``, ``z_l``, ``z_r``, ``declared``,
    ``mode``, ``roll``, ``pitch``, ``roll_l``, ``pitch_l``, ``roll_r``,
    ``pitch_r``, ``up_z``, ``zmp_act``, ``com_act``, ``cop_l``, ``cop_r``.
    Margins are NaN when that tick has no CoP on that polygon.
    ``declared`` and ``mode`` are unicode arrays.
    """
    t = np.asarray(trace["t"], dtype=np.float64)
    n = int(t.shape[0])
    expect = expected_steps(vx_m_s, period_s)
    empty = _empty(expect)
    if n < 2:
        empty["fail_reasons"] = ["no stepping samples"]
        return empty
    swings = _swings(trace)
    scored = [row for row in swings if str(row["mode"]) == "move"]
    air_fwd, contact_fwd = _forward_split(trace)
    total_fwd = air_fwd + contact_fwd
    fraction = (air_fwd / total_fwd) if total_fwd > 1e-6 else 0.0
    slips = [float(row["slip_m"]) for row in scored if row["slip_m"] is not None]
    clears = [float(row["clear_m"]) for row in scored if row["clear_m"] is not None]
    airs = [float(row["airborne_dx_m"]) for row in scored]
    seps = [float(row["sep_m"]) for row in swings]
    x_l = np.asarray(trace["x_l"], dtype=np.float64)
    x_r = np.asarray(trace["x_r"], dtype=np.float64)
    sep_series = np.abs(x_l - x_r)
    move = np.asarray([str(item) == "move" for item in trace["mode"]])
    sep_bout = float(np.max(sep_series)) if sep_series.size else None
    sep_move = float(np.max(sep_series[move])) if np.any(move) else sep_bout
    worst_slip = max(slips) if slips else None
    min_clear = min(clears) if clears else None
    honest_clear = max(clears) if clears else None
    target = float(expect["per_swing_m"])
    stride_lo = target * (1.0 - STRIDE_TOL)
    stride_hi = target * (1.0 + STRIDE_TOL)
    stride_bad = [
        row for row in scored
        if not (stride_lo <= float(row["airborne_dx_m"]) <= stride_hi)
    ]
    ss = _single_support(trace)
    stop = _stop_window(trace, q_stand_err)
    mismatch = _mismatch(trace)
    act = _margin_stats(trace["zmp_act"], trace["com_act"])
    reasons: list[str] = []
    if not scored:
        reasons.append("no airborne swing while the bus is moving; the feet do not step")
    if fraction + 1e-12 < STEP_FRACTION_BAR:
        reasons.append(
            f"step fraction {fraction:.3f} is under {STEP_FRACTION_BAR:.2f} "
            f"(airborne forward {air_fwd:.4f} m, contact forward {contact_fwd:.4f} m)"
        )
    if worst_slip is None:
        reasons.append("no stance-slip sample; there is no loaded stance under a swing")
    elif worst_slip > SLIP_BAR_M + 1e-12:
        reasons.append(f"worst stance slip {worst_slip * 1000.0:.2f} mm is over 2 mm")
    if min_clear is None:
        reasons.append("no sole-clearance sample over 20–80% of a swing")
    elif min_clear + 1e-12 < CLEAR_BAR_M:
        honest = "unmeasured" if honest_clear is None else f"{honest_clear * 1000.0:.2f} mm"
        reasons.append(
            f"sole clearance min {min_clear * 1000.0:.2f} mm is under 8 mm; "
            f"honest max of the per-step minima is {honest}"
        )
    if target > 1e-6 and stride_bad:
        worst = min(stride_bad, key=lambda row: float(row["airborne_dx_m"]))
        reasons.append(
            f"airborne advance {float(worst['airborne_dx_m']):.4f} m on {worst['side']} "
            f"at {float(worst['t_lift']):.3f} s is outside ±20% of vx·T "
            f"({target:.4f} m). vx·T/2 is the stance-to-stance spacing "
            f"({expect['stance_to_stance_m']:.4f} m), not this travel"
        )
    if ss["n"] <= 0:
        reasons.append("no actual single support; a foot never leaves the floor alone")
    elif int(ss["min_contacts"]) < SS_CONTACT_BAR:
        reasons.append(
            f"stance contact count min {ss['min_contacts']} is under {SS_CONTACT_BAR}"
        )
    reasons.extend(stop["fail_reasons"])
    if act["zmp_min_m"] is None or float(act["zmp_min_m"]) < 0.0 or float(act["zmp_out"]) > 0.0:
        reasons.append(
            f"actual-stance ZMP margin {act['zmp_min_m']} outside {act['zmp_out']}"
        )
    if act["com_min_m"] is None or float(act["com_min_m"]) < 0.0 or float(act["com_out"]) > 0.0:
        reasons.append(
            f"actual-stance CoM margin {act['com_min_m']} outside {act['com_out']}"
        )
    return {
        "expected": expect,
        "n_swings": len(swings),
        "n_scored_swings": len(scored),
        "step_fraction": fraction,
        "airborne_forward_m": air_fwd,
        "contact_forward_m": contact_fwd,
        "worst_slip_m": worst_slip,
        "min_clear_m": min_clear,
        "honest_max_clear_m": honest_clear,
        "airborne_min_m": min(airs) if airs else None,
        "airborne_median_m": float(np.median(np.asarray(airs))) if airs else None,
        "sep_peak_m": max(seps) if seps else None,
        "sep_bout_m": sep_bout,
        "sep_move_m": sep_move,
        "mismatch_fraction": mismatch,
        "ss": ss,
        "stop": {k: v for k, v in stop.items() if k != "fail_reasons"},
        "actual": act,
        "swings": scored[:24],
        "fail_reasons": reasons,
        "passes": not reasons,
    }


def _empty(expect: dict[str, float]) -> dict[str, object]:
    return {
        "expected": expect,
        "n_swings": 0,
        "n_scored_swings": 0,
        "step_fraction": 0.0,
        "airborne_forward_m": 0.0,
        "contact_forward_m": 0.0,
        "worst_slip_m": None,
        "min_clear_m": None,
        "honest_max_clear_m": None,
        "airborne_min_m": None,
        "airborne_median_m": None,
        "sep_peak_m": None,
        "sep_bout_m": None,
        "sep_move_m": None,
        "mismatch_fraction": 1.0,
        "ss": {"n": 0, "min_contacts": 0, "cop_min_m": None, "cop_p5_m": None,
               "cop_p50_m": None, "cop_p95_m": None, "edge_fraction": None,
               "edge_zero_fraction": None, "tilt_max_rad": None, "tilt_over_1deg": None},
        "stop": {},
        "actual": {"zmp_min_m": None, "com_min_m": None, "zmp_out": 1.0, "com_out": 1.0},
        "swings": [],
        "fail_reasons": ["no stepping samples"],
        "passes": False,
    }


def _forward_split(trace: dict[str, np.ndarray]) -> tuple[float, float]:
    air = 0.0
    contact = 0.0
    for side in ("l", "r"):
        x = np.asarray(trace[f"x_{side}"], dtype=np.float64)
        ncon = np.asarray(trace[f"n_{side}"], dtype=np.int32)
        dx = np.diff(x)
        # The interval belongs to the destination tick's contact.
        end_air = ncon[1:] == 0
        forward = dx > 0.0
        air += float(np.sum(dx[forward & end_air]))
        contact += float(np.sum(dx[forward & ~end_air]))
    return air, contact


def _swings(trace: dict[str, np.ndarray]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    t = np.asarray(trace["t"], dtype=np.float64)
    mode = trace["mode"]
    for side, other in (("l", "r"), ("r", "l")):
        ncon = np.asarray(trace[f"n_{side}"], dtype=np.int32)
        x = np.asarray(trace[f"x_{side}"], dtype=np.float64)
        xo = np.asarray(trace[f"x_{other}"], dtype=np.float64)
        yo = np.asarray(trace[f"y_{other}"], dtype=np.float64)
        z = np.asarray(trace[f"z_{side}"], dtype=np.float64)
        i = 1
        n = int(ncon.shape[0])
        while i < n:
            if ncon[i - 1] > 0 and ncon[i] == 0:
                j = i + 1
                while j < n and ncon[j] == 0:
                    j += 1
                if j >= n or (j - i) < 2:
                    i = max(j, i + 1)
                    continue
                span = j - i
                lo = i + int(math.floor(0.20 * span))
                hi = i + int(math.ceil(0.80 * span))
                hi = min(n, max(lo + 1, hi))
                clear = float(np.min(z[lo:hi])) if hi > lo else None
                no = np.asarray(trace[f"n_{other}"], dtype=np.int32)
                slip = 0.0
                loaded = 0
                for k in range(i, j):
                    if int(no[k]) <= 0:
                        continue
                    loaded += 1
                    slip += math.hypot(float(xo[k] - xo[k - 1]), float(yo[k] - yo[k - 1]))
                sep = float(np.max(np.abs(x[i:j] - xo[i:j]))) if j > i else 0.0
                rows.append({
                    "side": side.upper(),
                    "t_lift": float(t[i]),
                    "t_land": float(t[j - 1]),
                    "airborne_dx_m": float(x[j - 1] - x[i]),
                    "slip_m": None if loaded == 0 else slip,
                    "clear_m": clear,
                    "sep_m": sep,
                    "mode": str(mode[i]),
                })
                i = j
            else:
                i += 1
    return rows


def _single_support(trace: dict[str, np.ndarray]) -> dict[str, object]:
    n_l = np.asarray(trace["n_l"], dtype=np.int32)
    n_r = np.asarray(trace["n_r"], dtype=np.int32)
    left = (n_l > 0) & (n_r == 0)
    right = (n_r > 0) & (n_l == 0)
    ss = left | right
    n = int(np.sum(ss))
    if n <= 0:
        return {
            "n": 0, "min_contacts": 0, "cop_min_m": None, "cop_p5_m": None,
            "cop_p50_m": None, "cop_p95_m": None, "edge_fraction": None,
            "edge_zero_fraction": None, "tilt_max_rad": None, "tilt_over_1deg": None,
        }
    contacts = np.where(left, n_l, n_r)
    cop = np.where(left, trace["cop_l"], trace["cop_r"])
    cop_ss = np.asarray(cop[ss], dtype=np.float64)
    cop_ok = cop_ss[np.isfinite(cop_ss)]
    roll = np.where(left, trace["roll_l"], trace["roll_r"])
    pitch = np.where(left, trace["pitch_l"], trace["pitch_r"])
    tilt = np.maximum(np.abs(roll[ss]), np.abs(pitch[ss]))
    edge = np.isfinite(cop_ss) & (cop_ss < EDGE_DWELL_M)
    zero = np.isfinite(cop_ss) & (np.abs(cop_ss) <= EDGE_ZERO_M)
    return {
        "n": n,
        "min_contacts": int(np.min(contacts[ss])),
        "cop_min_m": None if cop_ok.size == 0 else float(np.min(cop_ok)),
        "cop_p5_m": _pct(cop_ok, 5),
        "cop_p50_m": _pct(cop_ok, 50),
        "cop_p95_m": _pct(cop_ok, 95),
        "edge_fraction": float(np.mean(edge)) if cop_ss.size else None,
        "edge_zero_fraction": float(np.mean(zero)) if cop_ss.size else None,
        "tilt_max_rad": float(np.max(tilt)),
        "tilt_over_1deg": float(np.mean(tilt > TILT_BAR_RAD)),
    }


def _mismatch(trace: dict[str, np.ndarray]) -> float:
    n_l = np.asarray(trace["n_l"], dtype=np.int32)
    n_r = np.asarray(trace["n_r"], dtype=np.int32)
    declared = trace["declared"]
    bad = 0
    for i in range(int(n_l.shape[0])):
        actual = _actual_label(int(n_l[i]), int(n_r[i]))
        label = str(declared[i])
        if label in ("stand", "ds"):
            match = actual == "ds"
        elif label in ("ss_L", "ss_R"):
            match = actual == label
        else:
            match = False
        if not match:
            bad += 1
    return bad / float(n_l.shape[0])


def _actual_label(n_l: int, n_r: int) -> str:
    if n_l > 0 and n_r > 0:
        return "ds"
    if n_l > 0:
        return "ss_L"
    if n_r > 0:
        return "ss_R"
    return "flight"


def _margin_stats(zmp: np.ndarray, com: np.ndarray) -> dict[str, object]:
    z = np.asarray(zmp, dtype=np.float64)
    c = np.asarray(com, dtype=np.float64)
    z_ok = z[np.isfinite(z)]
    c_ok = c[np.isfinite(c)]
    z_out = float(np.mean(~np.isfinite(z) | (z < 0.0)))
    c_out = float(np.mean(~np.isfinite(c) | (c < 0.0))) if c.size else 1.0
    return {
        "zmp_min_m": None if z_ok.size == 0 else float(np.min(z_ok)),
        "com_min_m": None if c_ok.size == 0 else float(np.min(c_ok)),
        "zmp_out": z_out,
        "com_out": c_out,
    }


def _stop_window(
    trace: dict[str, np.ndarray],
    q_err: np.ndarray | None,
) -> dict[str, object]:
    t = np.asarray(trace["t"], dtype=np.float64)
    t1 = float(t[-1]) - 1.0
    window = t >= t1 - 1e-12
    if not np.any(window):
        window = np.ones(t.shape[0], dtype=bool)
    # Stand reference is the first 0.40 s, before the walk.
    stand = t <= 0.40 + 1e-12
    if not np.any(stand):
        stand = window
    pitch = np.asarray(trace["pitch"], dtype=np.float64)
    roll = np.asarray(trace["roll"], dtype=np.float64)
    up = np.asarray(trace["up_z"], dtype=np.float64)
    n_l = np.asarray(trace["n_l"], dtype=np.int32)
    n_r = np.asarray(trace["n_r"], dtype=np.int32)
    stand_pitch = float(np.median(pitch[stand]))
    stand_roll = float(np.median(roll[stand]))
    pitch_w = pitch[window]
    roll_w = roll[window]
    min_up = float(np.min(up[window]))
    min_l = int(np.min(n_l[window]))
    min_r = int(np.min(n_r[window]))
    pitch_off = float(np.max(np.abs(pitch_w - stand_pitch)))
    roll_off = float(np.max(np.abs(roll_w - stand_roll)))
    q_max = None
    if q_err is not None and q_err.shape[0] == t.shape[0]:
        q_max = float(np.max(q_err[window]))
    both = min_l >= SS_CONTACT_BAR and min_r >= SS_CONTACT_BAR
    level = min_up >= STOP_UP_Z and pitch_off <= UPRIGHT_MATCH_RAD and roll_off <= UPRIGHT_MATCH_RAD
    lean = pitch_off > UPRIGHT_MATCH_RAD or roll_off > UPRIGHT_MATCH_RAD or min_up < STOP_UP_Z
    upright = both and level
    returns = upright and (q_max is None or q_max <= 0.10)
    reasons: list[str] = []
    if not upright:
        reasons.append(
            "stop does not end upright: "
            f"final 1 s min up_z {min_up:.3f}, contacts L/R {min_l}/{min_r}, "
            f"trunk pitch off the stand by {math.degrees(pitch_off):.2f} deg, "
            f"roll off by {math.degrees(roll_off):.2f} deg"
        )
    if lean:
        pose = "freezes in a lean"
    elif not both:
        pose = "final second does not hold both feet"
    elif returns:
        pose = "returns to the stand pose"
    else:
        pose = "upright, freezes off the stand pose"
    return {
        "min_up_z": min_up,
        "min_contacts_l": min_l,
        "min_contacts_r": min_r,
        "pitch_off_rad": pitch_off,
        "roll_off_rad": roll_off,
        "stand_pitch_rad": stand_pitch,
        "stand_roll_rad": stand_roll,
        "final_pitch_rad": float(pitch_w[-1]),
        "final_roll_rad": float(roll_w[-1]),
        "q_stand_err_rad": q_max,
        "upright": upright,
        "pose": pose,
        "fail_reasons": reasons,
    }


def self_test() -> int:
    failures: list[str] = []

    def expect(ok: bool, msg: str) -> None:
        if not ok:
            failures.append(msg)

    got = expected_steps(0.056, 3.60)
    expect(abs(got["per_swing_m"] - 0.2016) < 1e-9, "per-swing travel is vx·T")
    expect(abs(got["stance_to_stance_m"] - 0.1008) < 1e-9, "stance spacing is vx·T/2")
    expect(abs(got["commanded_hip_travel_m"] - 2.0 * (0.056 / 7.50)) < 1e-12, "IK travel is 2·x_amp")
    n = 50
    t = np.arange(n, dtype=np.float64) * 0.008
    skate = _base_trace(t)
    skate["x_l"] = t * 0.056
    skate["x_r"] = t * 0.056
    skate["n_l"][:] = 4
    skate["n_r"][:] = 4
    skate["mode"][:] = "move"
    skate["declared"][:] = "ds"
    skate["zmp_act"][:] = 0.01
    skate["com_act"][:] = 0.01
    skate["cop_l"][:] = 0.01
    skate["cop_r"][:] = 0.01
    report = summarize_trace(skate, vx_m_s=0.056, period_s=3.60)
    expect(float(report["step_fraction"]) < 0.01, "a skate has no airborne advance")
    expect(report["passes"] is False, "a skate fails the bars")
    step = _base_trace(t)
    step["mode"][:] = "move"
    step["n_l"][:] = 4
    step["n_r"][:] = 4
    # One left swing, airborne travel = vx·T, stance foot still.
    step["n_l"][10:30] = 0
    step["x_l"][10:30] = np.linspace(0.0, 0.2016, 20)
    step["x_l"][30:] = 0.2016
    step["z_l"][10:30] = 0.012
    step["declared"][:] = "ds"
    step["declared"][10:30] = "ss_R"
    step["zmp_act"][:] = 0.01
    step["com_act"][:] = 0.01
    step["cop_r"][:] = 0.02
    stepped = summarize_trace(step, vx_m_s=0.056, period_s=3.60)
    expect(float(stepped["step_fraction"]) >= 0.90, "a real swing clears the step fraction")
    expect(int(stepped["n_scored_swings"]) == 1, "one swing event")
    n_h = 4
    ht = np.arange(n_h, dtype=np.float64) * 0.008
    omega = np.zeros((n_h, 2), dtype=np.float64)
    omega[1, 0] = LEG_QVEL_BAR
    omega[2, 1] = LEG_QVEL_BAR + 0.01
    hinges = hinge_speed_report(
        ("l_knee", "r_knee"), ht, omega, np.array(["walk"] * n_h, dtype=object),
    )
    by_joint = {str(row["joint"]): row for row in hinges["joints"]}
    expect(hinges["passes"] is False, "a hinge over 5.82 rad/s fails")
    expect(len(hinges["fail_reasons"]) == 1, "only the over-speed hinge fails")
    expect("r_knee" in str(hinges["fail_reasons"][0]), "the fail names r_knee")
    expect(abs(float(by_joint["l_knee"]["headroom_rad_s"])) < 1e-12, "5.82 rad/s has zero headroom")
    expect(float(by_joint["r_knee"]["headroom_rad_s"]) < 0.0, "over-speed headroom is negative")
    expect(str(by_joint["r_knee"]["stage"]) == "walk", "peak keeps its stage")
    nt = 21
    tt = np.arange(nt, dtype=np.float64)
    line = trunk_speed_line(
        tt,
        np.linspace(0.0, 0.084, nt),
        np.zeros(nt),
        np.zeros(nt),
        np.array(["move"] * nt, dtype=object),
        period_s=20.0,
        vx_cmd_m_s=0.056,
    )
    expect(line["actual_vx_m_s"] is not None and abs(float(line["actual_vx_m_s"]) - 0.0042) < 1e-9, "0.084 m in 20 s is 0.0042 m/s")
    expect(line["ratio"] is not None and abs(float(line["ratio"]) - (0.0042 / 0.056)) < 1e-9, "ratio against 0.056")
    turned = trunk_speed_line(
        tt,
        np.zeros(nt),
        np.linspace(0.0, -0.084, nt),
        np.full(nt, -0.5 * math.pi),
        np.array(["move"] * nt, dtype=object),
        period_s=0.5,
        vx_cmd_m_s=0.056,
    )
    expect(turned["actual_vx_m_s"] is not None and abs(float(turned["actual_vx_m_s"]) - 0.0042) < 1e-9, "heading-frame forward, not world +x")
    n_st = 3
    st_t = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    st_tau = np.array([[3.0, 0.4], [2.1, 0.2], [0.5, 1.5]], dtype=np.float64)
    st_qv = np.array([[0.2, 4.5], [1.5, 6.0], [0.1, 1.0]], dtype=np.float64)
    st_stage = np.array(["walk", "walk", "stop"], dtype=object)
    pending = speed_torque_check(
        ("l_knee", "r_knee"), st_t, st_tau, st_qv, st_stage,
        no_load_speed=None, stall_torque=None, voltage=None,
    )
    expect(pending["status"] == SPEED_TORQUE_PENDING, "unset line stays pending")
    expect(pending["passes"] is True, "a pending line does not fail the row")
    expect(pending["no_load_speed"] is None and pending["stall_torque"] is None and pending["voltage"] is None, "line parameters stay unset")
    by_corner = {str(row["joint"]): row for row in pending["corners"]}
    l_at_tau = by_corner["l_knee"]["max_qvel_at_tau"]
    r_at_qv = by_corner["r_knee"]["max_tau_at_qvel"]
    expect(isinstance(l_at_tau, dict) and abs(float(l_at_tau["qvel_rad_s"]) - 1.5) < 1e-12, "largest |qvel| at |τ|≥2")
    expect(isinstance(r_at_qv, dict) and abs(float(r_at_qv["tau_nm"]) - 0.4) < 1e-12, "largest |τ| at |qvel|≥4")
    supplied = speed_torque_check(
        ("l_knee",), st_t, st_tau[:, :1], st_qv[:, :1], st_stage,
        no_load_speed=1.0, stall_torque=10.0, voltage=1.0,
    )
    expect(supplied["passes"] is False, "a supplied line can fail")
    worst = supplied["worst"]
    expect(isinstance(worst, dict) and worst["joint"] == "l_knee", "worst joint is named")
    expect(isinstance(worst, dict) and abs(float(worst["t_s"]) - 0.2) < 1e-12, "worst tick is named")
    expect(isinstance(worst, dict) and float(worst["margin_rad_s"]) < 0.0, "worst margin is negative")
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[step-bars] self-test ok")
    return 0


def _base_trace(t: np.ndarray) -> dict[str, np.ndarray]:
    n = int(t.shape[0])
    z = np.zeros(n, dtype=np.float64)
    mode = np.array(["stand"] * n, dtype=object)
    declared = np.array(["stand"] * n, dtype=object)
    return {
        "t": t,
        "x_l": z.copy(), "x_r": z.copy(), "y_l": z.copy(), "y_r": z.copy(),
        "n_l": np.zeros(n, dtype=np.int32), "n_r": np.zeros(n, dtype=np.int32),
        "z_l": z.copy(), "z_r": z.copy(),
        "declared": declared, "mode": mode,
        "roll": z.copy(), "pitch": z.copy(),
        "roll_l": z.copy(), "pitch_l": z.copy(), "roll_r": z.copy(), "pitch_r": z.copy(),
        "up_z": np.ones(n, dtype=np.float64),
        "zmp_act": np.full(n, np.nan), "com_act": np.full(n, np.nan),
        "cop_l": np.full(n, np.nan), "cop_r": np.full(n, np.nan),
    }


def as_arrays(columns: dict[str, list[object]]) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    for key, values in columns.items():
        if key in ("declared", "mode"):
            out[key] = np.asarray(values, dtype=object)
        elif key in ("n_l", "n_r"):
            out[key] = np.asarray(values, dtype=np.int32)
        else:
            out[key] = np.asarray(list(values), dtype=np.float64)
    return out


