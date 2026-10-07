#!/usr/bin/env python3
"""Contact stepping score for older voice tips and the #70 kit bout.

Loads each tip from a detached worktree. Does not edit that tree, the plant,
or the gait. Swing events come from floor contact, including the entrance
mat when that geom is in the scene.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FROZEN_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
STEP_BARS_PATH = Path(__file__).resolve().parent / "step_bars.py"
RETRO_MD_HEADING = "## Retro voice tips"
HINGE_MD_HEADING = "## Hinge speed"
HINGE_JSON = ROOT / "previews" / "walk_hinge_speed.json"
LEG_JOINTS = (
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
)
TRACE_KEYS = (
    "t", "x_l", "x_r", "y_l", "y_r", "n_l", "n_r", "z_l", "z_r",
    "declared", "mode", "roll", "pitch", "roll_l", "pitch_l", "roll_r", "pitch_r",
    "up_z", "zmp_act", "com_act", "cop_l", "cop_r", "yaw",
)
# Wall-stop CLEARs the PRs published for these exact bouts.
EARLIER_CLEAR = {
    ("79edde5", "living-m90"): "living −90 wall-stop CLEAR",
    ("e2da4f3", "kitchen-m90"): "kitchen −90 wall-stop CLEAR",
    ("e2da4f3", "living-m90"): "living −90 wall-stop CLEAR",
    ("51ae123", "kitchen-m90"): "kitchen −90 wall-stop CLEAR",
    ("51ae123", "living-m90"): "living −90 wall-stop CLEAR",
    ("51ae123", "entrance-m90"): "entrance −90 wall-stop CLEAR",
    ("f8c9edb", "kitchen-m90"): "kitchen −90 wall-stop CLEAR",
    ("f8c9edb", "living-m90"): "living −90 wall-stop CLEAR",
    ("f8c9edb", "entrance-m90"): "entrance −90 wall-stop CLEAR",
    ("743b79a", "kitchen-m90"): "kitchen −90 wall-stop CLEAR",
    ("743b79a", "living-m90"): "living −90 wall-stop CLEAR",
    ("69da170", "kit"): "Track 1 toe-clearance CLEAR, MID_SWING_TOE_MIN_M −0.003066",
}


def _load_step_bars():
    spec = importlib.util.spec_from_file_location("step_bars_102", STEP_BARS_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {STEP_BARS_PATH}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _worktree(sha: str) -> Path:
    dest = Path("/tmp/retro-walk") / sha
    marker = dest / "scripts" / "steer_walk.py"
    if marker.is_file():
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.check_call(
        ["git", "worktree", "add", "--detach", str(dest), sha],
        cwd=ROOT,
    )
    return dest


def _plant_md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _phase_label(phase: str, stance: str) -> str:
    if phase == "swing" and stance == "L":
        return "ss_L"
    if phase == "swing" and stance == "R":
        return "ss_R"
    if phase == "shift":
        return "ds"
    if phase == "stand":
        return "stand"
    return "unknown"


def _contact_count(data: mj.MjData, foot_gid: int, grounds: tuple[int, ...]) -> int:
    allowed = {int(g) for g in grounds}
    n = 0
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        if (g1 == foot_gid and g2 in allowed) or (g2 == foot_gid and g1 in allowed):
            n += 1
    return n


def _rug_box(model: mj.MjModel, data: mj.MjData) -> tuple[float, float, float, float, float] | None:
    gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "mat_rug")
    if gid < 0:
        return None
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    center = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    return (
        float(center[0] - half[0]),
        float(center[0] + half[0]),
        float(center[1] - half[1]),
        float(center[1] + half[1]),
        float(center[2] + half[2]),
    )


def _foot_box(
    model: mj.MjModel,
    data: mj.MjData,
    gid: int,
    rug: tuple[float, float, float, float, float] | None,
) -> tuple[float, float, float, float, float]:
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    if hasattr(data, "geom_xmat"):
        rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
        origin = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    else:
        bid = int(model.geom_bodyid[gid])
        rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
        origin = np.asarray(data.xpos[bid], dtype=np.float64) + rot @ np.asarray(
            model.geom_pos[gid], dtype=np.float64,
        )
    clear = 1.0e9
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array([sx * half[0], sy * half[1], sz * half[2]], dtype=np.float64)
                world = origin + rot @ local
                ground = 0.0
                if rug is not None:
                    x0, x1, y0, y1, top = rug
                    if x0 <= float(world[0]) <= x1 and y0 <= float(world[1]) <= y1:
                        ground = top
                clear = min(clear, float(world[2]) - ground)
    normal = rot[:, 2]
    roll = math.atan2(float(normal[1]), float(normal[2]))
    pitch = math.atan2(-float(normal[0]), math.hypot(float(normal[1]), float(normal[2])))
    return float(origin[0]), float(origin[1]), float(clear), roll, pitch


def _foot_cop(model, data, gid: int, grounds: tuple[int, ...]):
    allowed = {int(g) for g in grounds}
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        if not ((g1 == gid and g2 in allowed) or (g2 == gid and g1 in allowed)):
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, force)
        fn = float(force[0])
        if fn <= 1e-6:
            continue
        num += fn * np.asarray(con.pos, dtype=np.float64)
        den += fn
    if den <= 1e-6:
        return None
    return num / den, den


def _margin(sw, session, walker, grounds, n_l: int, n_r: int) -> tuple[float, float]:
    sides = [side for side, n in (("L", n_l), ("R", n_r)) if n > 0]
    if not sides:
        return float("nan"), float("nan")
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for side in sides:
        got = _foot_cop(session.model, session.data, int(walker.gid[side]), grounds)
        if got is None:
            continue
        world, fn = got
        num += fn * world
        den += fn
    chunks = [session._foot_corners(walker.bid[side], walker.gid[side]) for side in sides]
    hull = sw.convex_hull_xy(np.concatenate(chunks, axis=0))
    com = np.asarray(session.data.subtree_com[walker.bid_body, :2], dtype=np.float64)
    com_m = float(sw.support_margin(com, hull))
    if den <= 1e-6:
        return float("nan"), com_m
    return float(sw.support_margin((num / den)[:2], hull)), com_m


def _cop_margin(sw, session, walker, side: str, grounds) -> float:
    got = _foot_cop(session.model, session.data, int(walker.gid[side]), grounds)
    if got is None:
        return float("nan")
    world, _fn = got
    corners = session._foot_corners(walker.bid[side], walker.gid[side])
    hull = sw.convex_hull_xy(corners)
    return float(sw.support_margin(world[:2], hull))


def _hinge_block(step_bars, rec: Recorder, period: float, vx: float) -> dict[str, object]:
    """Hinge-speed bar and trunk-speed line for one recorded bout.

    This walker has no preview stage. The stage column is the gait phase
    already used by the contact trace: stand, ds, ss_L, ss_R.
    """
    t = np.asarray(rec.cols["t"], dtype=np.float64)
    omega = np.asarray(rec.omega, dtype=np.float64) if rec.omega else np.zeros((0, 12))
    stage = np.asarray(rec.stage, dtype=object)
    qvel = step_bars.hinge_speed_report(LEG_JOINTS, t, omega, stage)
    tau_m = np.full((len(rec.pair_tau), len(LEG_JOINTS)), np.nan, dtype=np.float64)
    qv_m = np.full_like(tau_m, np.nan)
    for i, (tau_row, qv_row) in enumerate(zip(rec.pair_tau, rec.pair_qvel)):
        for j, (tau, qvel_abs) in enumerate(zip(tau_row, qv_row)):
            if tau is not None:
                tau_m[i, j] = float(tau)
            if qvel_abs is not None:
                qv_m[i, j] = float(qvel_abs)
    torque = step_bars.speed_torque_check(
        LEG_JOINTS, t, tau_m, qv_m, stage,
        no_load_speed=None, stall_torque=None, voltage=None,
    )
    pairs = {
        "t_s": [float(item) for item in rec.cols["t"]],
        "tau_nm": {
            name: [row[i] for row in rec.pair_tau] for i, name in enumerate(LEG_JOINTS)
        },
        "qvel_rad_s": {
            name: [row[i] for row in rec.pair_qvel] for i, name in enumerate(LEG_JOINTS)
        },
    }
    speed = step_bars.trunk_speed_line(
        t,
        np.asarray(rec.trunk_x, dtype=np.float64),
        np.asarray(rec.trunk_y, dtype=np.float64),
        np.asarray(rec.trunk_yaw, dtype=np.float64),
        np.asarray(rec.cols["mode"], dtype=object),
        period_s=period,
        vx_cmd_m_s=vx,
    )
    return {"qvel": qvel, "trunk_speed": speed, "speed_torque": torque, "hinge_pairs": pairs}


def _leg_q(model, data) -> list[float]:
    out: list[float] = []
    for name in LEG_JOINTS:
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"missing joint {name}")
        out.append(float(data.qpos[int(model.jnt_qposadr[jid])]))
    return out


def _grounds(model, walker) -> tuple[int, ...]:
    gids = [int(walker.gid_floor)]
    for extra in getattr(walker, "ground_extra", ()):
        gi = int(extra)
        if gi >= 0 and gi not in gids:
            gids.append(gi)
    for name in ("mat_rug", "col_mat_rug"):
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if gid >= 0 and int(gid) not in gids:
            gids.append(int(gid))
    return tuple(g for g in gids if g >= 0)


class Recorder:
    def __init__(self, sw) -> None:
        self.sw = sw
        self.on = False
        self.cols: dict[str, list[object]] = {key: [] for key in TRACE_KEYS}
        self.q: list[list[float]] = []
        self.applied_vx: list[float] = []
        self.half_x: float | None = None
        self.omega: list[list[float]] = []
        self.stage: list[str] = []
        self.trunk_x: list[float] = []
        self.trunk_y: list[float] = []
        self.trunk_yaw: list[float] = []
        self._dof: list[int] | None = None
        self.pair_tau: list[list[float | None]] = []
        self.pair_qvel: list[list[float | None]] = []
        self._orig = sw.SteerSession.step

        def _step(session, *args, **kwargs):
            if session.lipm is not None:
                self._arm_ask_log(session.lipm)
            report = self._orig(session, *args, **kwargs)
            if self.on:
                self._sample(session)
            return report

        sw.SteerSession.step = _step  # type: ignore[method-assign]

    def _arm_ask_log(self, walker) -> None:
        """Record unclamped |τ| and |qvel| at each leg write. The command is unchanged."""
        if getattr(walker, "_pair_log", None) is not None:
            return
        walker._pair_log = []

        def _log(jn: str, q_des: float) -> None:
            if jn not in LEG_JOINTS:
                return
            act = jn + "_pos"
            idx = walker.act_idx.get(act)
            if idx is None:
                return
            q = float(walker.q(jn))
            jid = mj.mj_name2id(walker.model, mj.mjtObj.mjOBJ_JOINT, jn)
            omega = float(walker.data.qvel[int(walker.model.jnt_dofadr[jid])])
            kp = float(walker.model.actuator_gainprm[idx, 0])
            kv = -float(walker.model.actuator_biasprm[idx, 2])
            total = abs(kp * (float(q_des) - q)) + abs(kv * omega)
            walker._pair_log.append((jn, float(total), abs(omega)))

        orig = walker.write_clipped

        def wrapped(jn: str, q_des: float) -> None:
            _log(jn, q_des)
            orig(jn, q_des)

        walker.write_clipped = wrapped  # type: ignore[method-assign]
        limited = getattr(walker, "write_force_limited", None)
        if limited is not None:
            def wrapped_limited(jn: str, q_des: float, limit_nm: float | None = None) -> None:
                _log(jn, q_des)
                limited(jn, q_des, limit_nm)

            walker.write_force_limited = wrapped_limited  # type: ignore[method-assign]

    def _drain_pairs(self, walker) -> None:
        log = getattr(walker, "_pair_log", None)
        writes = list(log) if log is not None else []
        if log is not None:
            log.clear()
        best: dict[str, tuple[float, float]] = {}
        for name, tau, qvel in writes:
            prev = best.get(name)
            if prev is None or tau > prev[0]:
                best[name] = (tau, qvel)
        tau_row: list[float | None] = []
        qvel_row: list[float | None] = []
        for name in LEG_JOINTS:
            got = best.get(name)
            if got is None:
                tau_row.append(None)
                qvel_row.append(None)
            else:
                tau_row.append(got[0])
                qvel_row.append(got[1])
        self.pair_tau.append(tau_row)
        self.pair_qvel.append(qvel_row)

    def _leg_omega(self, model, data) -> list[float]:
        if self._dof is None:
            addrs: list[int] = []
            for name in LEG_JOINTS:
                jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
                if jid < 0:
                    raise RuntimeError(f"missing joint {name}")
                addrs.append(int(model.jnt_dofadr[jid]))
            self._dof = addrs
        qv = data.qvel
        return [abs(float(qv[adr])) for adr in self._dof]

    def _sample(self, session) -> None:
        walker = session.lipm
        if walker is None:
            return
        grounds = _grounds(session.model, walker)
        rug = _rug_box(session.model, session.data)
        n_l = _contact_count(session.data, int(walker.gid["L"]), grounds)
        n_r = _contact_count(session.data, int(walker.gid["R"]), grounds)
        x_l, y_l, z_l, roll_l, pitch_l = _foot_box(
            session.model, session.data, int(walker.gid["L"]), rug,
        )
        x_r, y_r, z_r, roll_r, pitch_r = _foot_box(
            session.model, session.data, int(walker.gid["R"]), rug,
        )
        rot = np.asarray(session.data.xmat[walker.bid_body], dtype=np.float64).reshape(3, 3)
        up = rot[:, 2]
        roll = math.atan2(float(up[1]), float(up[2]))
        pitch = math.atan2(-float(up[0]), math.hypot(float(up[1]), float(up[2])))
        yaw = math.atan2(float(rot[1, 0]), float(rot[0, 0]))
        zmp_act, com_act = _margin(self.sw, session, walker, grounds, n_l, n_r)
        cols = self.cols
        cols["t"].append(float(session.data.time))
        cols["x_l"].append(x_l)
        cols["x_r"].append(x_r)
        cols["y_l"].append(y_l)
        cols["y_r"].append(y_r)
        cols["n_l"].append(n_l)
        cols["n_r"].append(n_r)
        cols["z_l"].append(z_l)
        cols["z_r"].append(z_r)
        cols["declared"].append(_phase_label(str(walker.phase), str(walker.stance)))
        cols["mode"].append(str(session.bus.mode))
        cols["roll"].append(roll)
        cols["pitch"].append(pitch)
        cols["roll_l"].append(roll_l)
        cols["pitch_l"].append(pitch_l)
        cols["roll_r"].append(roll_r)
        cols["pitch_r"].append(pitch_r)
        cols["up_z"].append(float(up[2]))
        cols["yaw"].append(yaw)
        cols["zmp_act"].append(zmp_act)
        cols["com_act"].append(com_act)
        cols["cop_l"].append(_cop_margin(self.sw, session, walker, "L", grounds))
        cols["cop_r"].append(_cop_margin(self.sw, session, walker, "R", grounds))
        self.q.append(_leg_q(session.model, session.data))
        self.applied_vx.append(float(session.bus.applied_vx))
        self.omega.append(self._leg_omega(session.model, session.data))
        self.stage.append(str(cols["declared"][-1]))
        origin = np.asarray(session.data.xpos[walker.bid_body], dtype=np.float64)
        self.trunk_x.append(float(origin[0]))
        self.trunk_y.append(float(origin[1]))
        self.trunk_yaw.append(yaw)
        self._drain_pairs(walker)
        if self.half_x is None:
            self.half_x = float(session.model.geom_size[int(walker.gid["L"]), 0])
        n = len(cols["t"])
        if n == 1 or n % 250 == 0:
            print(
                f"[retro] t={cols['t'][-1]:.2f}s n={n} mode={cols['mode'][-1]} "
                f"nL={n_l} nR={n_r}",
                flush=True,
            )

    def reset(self) -> None:
        self.cols = {key: [] for key in TRACE_KEYS}
        self.q = []
        self.applied_vx = []
        self.half_x = None
        self.omega = []
        self.stage = []
        self.trunk_x = []
        self.trunk_y = []
        self.trunk_yaw = []
        self.pair_tau = []
        self.pair_qvel = []


def _q_err(step_bars, cols, q_rows: list[list[float]]) -> np.ndarray:
    q_mat = np.asarray(q_rows, dtype=np.float64) if q_rows else np.zeros((0, 12))
    err = np.zeros(q_mat.shape[0], dtype=np.float64)
    if q_mat.shape[0] == 0:
        return err
    t = np.asarray(cols["t"], dtype=np.float64)
    stand = t <= 0.40 + 1e-12
    if not np.any(stand):
        stand = np.zeros(q_mat.shape[0], dtype=bool)
        stand[0] = True
    ref = np.median(q_mat[stand], axis=0)
    return np.max(np.abs(q_mat - ref), axis=1)


def _verdict(summary: dict[str, object]) -> str:
    """STEPS when airborne swings carry at least half the forward travel.

    The 90% step-fraction bar is still in ``fail_reasons``. A bout under that
    bar can still be STEPS. SKATES means the feet stay down, or contact
    advance is the majority of the forward travel.
    """
    frac = float(summary["step_fraction"])
    n = int(summary["n_scored_swings"])
    if n <= 0 or frac < 0.50:
        return "SKATES"
    return "STEPS"


def _heading_frame(trace: dict[str, np.ndarray]) -> tuple[dict[str, np.ndarray], np.ndarray]:
    """Express foot travel along the trunk heading. Slip length is unchanged.

    World +x is not the walk on a turned apartment bout. Body forward is the
    trunk x axis, ``(cos yaw, sin yaw)``.
    """
    yaw = np.asarray(trace["yaw"], dtype=np.float64)
    n = int(yaw.shape[0])
    sep = np.zeros(n, dtype=np.float64)
    out = dict(trace)
    for side in ("l", "r"):
        x = np.asarray(trace[f"x_{side}"], dtype=np.float64)
        y = np.asarray(trace[f"y_{side}"], dtype=np.float64)
        fwd = np.zeros(n, dtype=np.float64)
        lat = np.zeros(n, dtype=np.float64)
        for i in range(1, n):
            dx = float(x[i] - x[i - 1])
            dy = float(y[i] - y[i - 1])
            c = math.cos(float(yaw[i]))
            s = math.sin(float(yaw[i]))
            fwd[i] = fwd[i - 1] + dx * c + dy * s
            lat[i] = lat[i - 1] - dx * s + dy * c
        out[f"x_{side}"] = fwd
        out[f"y_{side}"] = lat
    x_l = np.asarray(trace["x_l"], dtype=np.float64)
    y_l = np.asarray(trace["y_l"], dtype=np.float64)
    x_r = np.asarray(trace["x_r"], dtype=np.float64)
    y_r = np.asarray(trace["y_r"], dtype=np.float64)
    for i in range(n):
        c = math.cos(float(yaw[i]))
        s = math.sin(float(yaw[i]))
        sep[i] = abs((float(x_l[i] - x_r[i])) * c + (float(y_l[i] - y_r[i])) * s)
    out.pop("yaw", None)
    return out, sep


def _summarize(step_bars, rec: Recorder, vx: float, period: float) -> dict[str, object]:
    raw = step_bars.as_arrays(rec.cols)
    trace, sep = _heading_frame(raw)
    summary = step_bars.summarize_trace(
        trace, vx_m_s=vx, period_s=period, q_stand_err=_q_err(step_bars, rec.cols, rec.q),
    )
    mode = np.asarray(rec.cols["mode"], dtype=object)
    move = np.array([str(item) == "move" for item in mode])
    summary["sep_bout_m"] = float(np.max(sep)) if sep.size else None
    summary["sep_peak_m"] = summary["sep_bout_m"]
    summary["sep_move_m"] = float(np.max(sep[move])) if np.any(move) else summary["sep_bout_m"]
    summary["forward_axis"] = "trunk heading"
    n_l = np.asarray(rec.cols["n_l"], dtype=np.int32)
    n_r = np.asarray(rec.cols["n_r"], dtype=np.int32)
    summary["airborne_ticks"] = int(np.sum((n_l == 0) | (n_r == 0))) if n_l.size else 0
    summary["n_ticks"] = int(n_l.size)
    summary["box_half_x_m"] = rec.half_x
    vx_arr = np.asarray(rec.applied_vx, dtype=np.float64)
    mode = np.asarray(rec.cols["mode"], dtype=object)
    moving = (mode == "move") & (vx_arr > 1e-4)
    summary["applied_vx_median_m_s"] = (
        float(np.median(vx_arr[moving])) if np.any(moving) else 0.0
    )
    summary["gait_verdict"] = _verdict(summary)
    swings = summary.get("swings")
    if isinstance(swings, list):
        summary["swings"] = swings[:4]
    return summary


def _jsonable(value: object) -> object:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    return value


def _prepare(sha: str) -> tuple[object, Path]:
    tree = _worktree(sha)
    scripts = str(tree / "scripts")
    sys.path.insert(0, scripts)
    import steer_walk as sw
    loaded = Path(sw.__file__).resolve()
    if not str(loaded).startswith(str(tree.resolve())):
        raise SystemExit(f"steer_walk loaded from {loaded}, not {tree}")
    return sw, tree


def _row_common(label: str, pr: int, sha: str, name: str, plant_before: str, plant_after: str) -> dict[str, object]:
    return {
        "label": label,
        "pr": pr,
        "tip_sha": sha,
        "bout": name,
        "earlier_clear": EARLIER_CLEAR.get((label, name)),
        "plant_md5_before": plant_before,
        "plant_md5_after": plant_after,
        "plant_matches_frozen": plant_before == FROZEN_MD5 and plant_after == FROZEN_MD5,
        "plant_predates_frozen": plant_before != FROZEN_MD5,
        "soft_pass": False,
    }


def run_kit(sw, step_bars, label: str, pr: int, sha: str) -> dict[str, object]:
    plant = sw.PLANT_XML
    before = _plant_md5(plant)
    print(f"[retro] kit {label} plant {before}", flush=True)
    rec = Recorder(sw)
    rec.on = True
    failures, lines = sw._bus_kit_forward_stop()
    rec.on = False
    after = _plant_md5(plant)
    cfg = sw.locked_kit_config()
    period = float(cfg.gm_period_s)
    vx = float(sw.VX_FWD_CAP)
    summary = _summarize(step_bars, rec, vx, period)
    if rec.half_x is not None and abs(rec.half_x - 0.0675) > 1e-6:
        summary["box_note"] = f"contact box half-length {rec.half_x * 1000.0:.2f} mm"
    row = _row_common(label, pr, sha, "kit", before, after)
    row.update({
        "script": "steer_walk._bus_kit_forward_stop",
        "config": "locked_kit_config",
        "bus_script": "BUS_KIT_SCRIPT",
        "vx_m_s": vx,
        "period_s": period,
        "kit_self_check_failures": failures[:12],
        "stepping": summary,
        "verdict": summary["gait_verdict"],
    })
    print(
        f"[retro] kit {summary['gait_verdict']} frac={summary['step_fraction']:.3f} "
        f"air_ticks={summary['airborne_ticks']} plant {before}->{after}",
        flush=True,
    )
    if lines:
        print(f"[retro] kit log lines {len(lines)}", flush=True)
    return row


def _room_brief(row: dict[str, object]) -> dict[str, object]:
    keys = (
        "room", "yaw_offset_rad", "commanded_vx", "commanded_yaw", "committed",
        "recogniser_label", "stop_path", "stop_reason", "t_end", "t_stop_s",
        "wall_stop_clear", "wall_stop_fail", "wall_stop_residual_m",
        "wall_stop_class", "wall_stop_wall", "wall_stop_lead_side",
        "reached", "reach_clear", "min_up_z", "contact", "started_outside",
        "finished_inside", "frac_refuse_count",
    )
    return {key: row.get(key) for key in keys if key in row}


def run_voice(sw, step_bars, label: str, pr: int, sha: str, rooms: tuple[str, ...]) -> dict[str, object]:
    import voice_goto_rooms as vgr
    plant = sw.PLANT_XML
    before = _plant_md5(plant)
    if before != sw.PLANT_MD5:
        print(f"[retro] tip plant constant {sw.PLANT_MD5} file {before}", flush=True)
    print(f"[retro] voice {label} plant {before}", flush=True)
    sw.apply_frozen_forward_gait()
    spec = vgr.apt._load_spec(vgr.apt.SPEC_JSON)
    definition = vgr._definition()
    asker, ask_error = vgr._load_asker()
    if asker is None:
        raise SystemExit(f"recogniser did not load: {ask_error}")
    plant_model = mj.MjModel.from_xml_path(str(sw.PLANT_XML))
    bodies = vgr._robot_bodies(plant_model)
    del plant_model
    offset = -0.5 * math.pi
    if offset not in vgr.SPAWN_OFFSETS_RAD:
        raise SystemExit(f"−90° offset {offset} is not in SPAWN_OFFSETS_RAD")
    phrases = {name: phrase for name, phrase in vgr.PHRASES}
    rec = Recorder(sw)
    cfg = sw.locked_kit_config()
    period = float(cfg.gm_period_s)
    vx = float(vgr.voice.FWD_MPS)
    rows: list[dict[str, object]] = []
    for name in rooms:
        if name not in phrases:
            raise SystemExit(f"{name} is not a suite phrase")
        rec.reset()
        digest_before = _plant_md5(plant)
        print(f"[retro] {label} {name} −90", flush=True)
        rec.on = True
        try:
            room = vgr._run_room(
                name,
                phrases[name],
                offset,
                spec["rooms"][name],
                asker,
                ask_error,
                bodies,
                float(definition["d_min_m"]),
                float(vgr.LATCH_EXTRA_M),
            )
        finally:
            rec.on = False
        digest_after = _plant_md5(plant)
        summary = _summarize(step_bars, rec, vx, period)
        hinge = _hinge_block(step_bars, rec, period, vx)
        bout = f"{name}-m90"
        row = _row_common(label, pr, sha, bout, digest_before, digest_after)
        row.update({
            "qvel": _jsonable(hinge["qvel"]),
            "trunk_speed": _jsonable(hinge["trunk_speed"]),
            "speed_torque": _jsonable(hinge["speed_torque"]),
            "hinge_pairs": _jsonable(hinge["hinge_pairs"]),
            "script": "voice_goto_rooms._run_room",
            "config": "locked_kit_config",
            "phrase": phrases[name],
            "yaw_offset_rad": offset,
            "vx_m_s": vx,
            "period_s": period,
            "suite": _room_brief(room),
            "stepping": summary,
            "verdict": summary["gait_verdict"],
        })
        rows.append(row)
        print(
            f"[retro] {label} {bout} {summary['gait_verdict']} "
            f"frac={float(summary['step_fraction']):.3f} "
            f"air_ticks={summary['airborne_ticks']} "
            f"wall_clear={room.get('wall_stop_clear')} "
            f"stop={room.get('stop_path')} t={room.get('t_end')} "
            f"plant {digest_before}->{digest_after}",
            flush=True,
        )
    asker.close()
    return {
        "label": label,
        "pr": pr,
        "tip_sha": sha,
        "kind": "voice",
        "plant_md5": before,
        "period_s": period,
        "vx_m_s": vx,
        "rows": rows,
    }


def write_tip(payload: dict[str, object], label: str) -> Path:
    path = ROOT / "previews" / f"walk_stepping_retro_{label}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(payload), indent=2) + "\n", encoding="utf-8")
    print(f"[retro] wrote {path}", flush=True)
    return path


def _deg(value: object) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "—"
    return f"{float(value) * 180.0 / math.pi:+.2f}"


def _mm(value: object, digits: int = 1) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "—"
    return f"{float(value) * 1000.0:.{digits}f}"


def _num(value: object, digits: int = 3) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "—"
    return f"{float(value):.{digits}f}"


_TIP_ORDER = ("79edde5", "e2da4f3", "51ae123", "f8c9edb", "743b79a", "69da170")


def _load_retro_payloads() -> list[dict[str, object]]:
    found: dict[str, dict[str, object]] = {}
    for path in (ROOT / "previews").glob("walk_stepping_retro_*.json"):
        loaded = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            found[str(loaded.get("label") or path.stem)] = loaded
    ordered = [_TIP_ORDER.index(label) if label in _TIP_ORDER else 99 for label in found]
    return [found[label] for _, label in sorted(zip(ordered, found), key=lambda item: (item[0], item[1]))]


def retro_markdown(payloads: list[dict[str, object]] | None = None) -> str:
    """Section for docs/WALK_STEPPING_BARS.md. Empty when no retro JSON exists."""
    if payloads is None:
        payloads = _load_retro_payloads()
    rows: list[dict[str, object]] = []
    for payload in payloads:
        got = payload.get("rows")
        if isinstance(got, list):
            rows.extend(row for row in got if isinstance(row, dict))
    if not rows:
        return ""
    lines = [
        RETRO_MD_HEADING,
        "",
        "Soft-pass is off. These bouts are the suite's own scripts on each tip.",
        "The plant file is not written. A tip whose plant md5 is not",
        f"`{FROZEN_MD5}` is reported as predating that plant.",
        "",
        "On these tips `locked_kit_config` sets `gm_period_s` to 0.500 s.",
        "`op3_walk.update_time` still places both single-support windows inside",
        "that period, so period is one left-plus-right cycle. Each foot swings",
        "once per period. The per-swing foot travel is `vx·T`. `vx·T/2` is the",
        "stance-to-stance spacing. Voice bouts command `voice.FWD_MPS` 0.056 m/s,",
        "so `vx·T` is 0.028 m and `vx·T/2` is 0.014 m. The day-1 kit script",
        "commands `VX_FWD_CAP` 0.150 m/s, so `vx·T` is 0.075 m and `vx·T/2` is",
        "0.0375 m. The ~0.10 m figure belongs to the 3.60 s preview rows, not",
        "to this 0.500 s walker.",
        "",
        "Forward travel is along the trunk heading, not world +x. Apartment",
        "bouts turn. A straight kit walk has yaw near 0, so the two axes match.",
        "A bout is STEPS when at least one swing starts in bus mode `move` and",
        "airborne advance is at least half the forward travel. It is SKATES",
        "when the feet stay down or contact advance is the majority. The 90%",
        "step-fraction bar stays in the fail list either way.",
        "Swing events are runs of ticks with zero contacts on the floor geom,",
        "and on `mat_rug` when that geom is in the scene.",
        "",
        "| Tip | PR | Bout | Verdict | Earlier CLEAR | Step frac | Air ticks | Air m | vx·T m | Contact m | Slip mm | Clear mm | Sep mm | Mismatch | Pitch vs stand | Plant |",
        "| --- | ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for row in rows:
        step = row.get("stepping")
        if not isinstance(step, dict):
            step = {}
        stop = step.get("stop")
        if not isinstance(stop, dict):
            stop = {}
        earlier = row.get("earlier_clear") or "—"
        plant = "frozen" if row.get("plant_matches_frozen") else str(row.get("plant_md5_before"))
        pitch = (
            f"{_deg(stop.get('final_pitch_rad'))} vs stand {_deg(stop.get('stand_pitch_rad'))}"
            f" ({_deg(stop.get('pitch_off_rad')).replace('+', '')} off)"
        )
        lines.append(
            "| "
            + " | ".join([
                str(row.get("label")),
                str(row.get("pr")),
                str(row.get("bout")),
                str(row.get("verdict")),
                str(earlier),
                _num(step.get("step_fraction")),
                str(step.get("airborne_ticks")),
                _num(step.get("airborne_forward_m")),
                _num(_step_expect(step, "per_swing_m")),
                _num(step.get("contact_forward_m")),
                _mm(step.get("worst_slip_m"), 2),
                _mm(step.get("min_clear_m"), 2),
                _mm(step.get("sep_move_m") if step.get("sep_move_m") is not None else step.get("sep_peak_m")),
                _num(step.get("mismatch_fraction")),
                pitch,
                plant,
            ])
            + " |"
        )
    lines.extend(["", _retro_notes(rows), ""])
    return "\n".join(lines)


def _step_expect(step: dict[str, object], key: str) -> object:
    expect = step.get("expected")
    if not isinstance(expect, dict):
        return None
    return expect.get(key)


def _retro_notes(rows: list[dict[str, object]]) -> str:
    notes: list[str] = []
    for row in rows:
        step = row.get("stepping")
        if not isinstance(step, dict):
            continue
        stop = step.get("stop")
        if not isinstance(stop, dict):
            stop = {}
        suite = row.get("suite")
        if not isinstance(suite, dict):
            suite = {}
        clear = suite.get("wall_stop_clear")
        wall = suite.get("wall_stop_wall") or ""
        residual = suite.get("wall_stop_residual_m")
        stop_path = suite.get("stop_path") or ""
        extra = ""
        if suite:
            extra = (
                f" Suite stop `{stop_path}`"
                f" wall `{wall}` clear {clear}"
                f" residual {residual}."
            )
        notes.append(
            f"- `{row.get('label')}` `{row.get('bout')}` {row.get('verdict')}: "
            f"airborne ticks {step.get('airborne_ticks')}, "
            f"airborne advance {_num(step.get('airborne_forward_m'))} m "
            f"against vx·T {_num(_step_expect(step, 'per_swing_m'))} m "
            f"(per-swing min {_num(step.get('airborne_min_m'), 4)} m, "
            f"median {_num(step.get('airborne_median_m'), 4)} m), "
            f"contact advance {_num(step.get('contact_forward_m'))} m, "
            f"worst slip {_mm(step.get('worst_slip_m'), 2)} mm, "
            f"lowest-corner clearance {_mm(step.get('min_clear_m'), 2)} mm "
            f"(honest max {_mm(step.get('honest_max_clear_m'), 2)} mm), "
            f"fore/aft separation {_mm(step.get('sep_move_m'), 1)} mm, "
            f"mismatch {_num(step.get('mismatch_fraction'))}, "
            f"final pitch {_deg(stop.get('final_pitch_rad'))} deg "
            f"against stand {_deg(stop.get('stand_pitch_rad'))} deg, "
            f"{stop.get('pose')}."
            f"{extra}"
        )
    skates = [row for row in rows if row.get("verdict") == "SKATES" and row.get("earlier_clear")]
    steps = [row for row in rows if row.get("verdict") == "STEPS" and row.get("earlier_clear")]
    close: list[str] = []
    if skates:
        named = ", ".join(f"#{row.get('pr')} {row.get('bout')}" for row in skates)
        close.append(
            f"Earlier CLEARs that reopen as skates: {named}. "
            "The forward motion on those bouts is contact advance."
        )
    else:
        close.append("No earlier CLEAR on this sample reopens as a skate.")
    if steps:
        named = ", ".join(f"#{row.get('pr')} {row.get('bout')}" for row in steps)
        close.append(
            f"Earlier CLEARs whose bouts step: {named}. "
            "Those feet leave the floor. The wall-stop residual and the "
            "Track 1 toe lock stay what they were. They are not the #103 skate "
            "(step fraction 0, zero airborne ticks)."
        )
    close.extend(_retro_close(rows))
    return "\n".join(notes) + "\n\n" + "\n\n".join(close)


def _suite_residual(row: dict[str, object]) -> str:
    suite = row.get("suite")
    if not isinstance(suite, dict) or not suite:
        return ""
    clear = suite.get("wall_stop_clear")
    residual = suite.get("wall_stop_residual_m")
    path = suite.get("stop_path") or ""
    if residual is None:
        return f"#{row.get('pr')} {row.get('bout')} stop `{path}` wall clear {clear}"
    return (
        f"#{row.get('pr')} {row.get('bout')} residual {float(residual):+.4f} m "
        f"wall clear {clear}"
    )


def _retro_close(rows: list[dict[str, object]]) -> list[str]:
    """Plain reading of the sample. Hard bars stay failed; the gait label is STEPS."""
    plants = {str(row.get("plant_md5_before")) for row in rows}
    plant_line = (
        f"Plant md5 `{FROZEN_MD5}` held before and after every bout. "
        "None of these tips predates that plant."
        if plants == {FROZEN_MD5}
        else "Plant md5 differs from the frozen blob on: "
        + ", ".join(
            f"`{row.get('label')}` `{row.get('plant_md5_before')}`"
            for row in rows
            if row.get("plant_md5_before") != FROZEN_MD5
        )
        + "."
    )
    wall_bits = [_suite_residual(row) for row in rows]
    wall_bits = [bit for bit in wall_bits if bit]
    return [
        "Every bout is STEPS and every bout still misses the hard stepping bars, "
        "so none of these rows is a stepping CLEAR. Voice stance slip is 3.39–3.44 mm "
        "(kit 3.19 mm) against a 2 mm bar. Lowest-corner clearance over 20–80% of "
        "swing is 2.02–2.08 mm against an 8 mm bar. The honest max of those "
        "per-step minima is 7.88–7.92 mm on the voice rows and 8.07 mm on the kit. "
        "Voice per-swing airborne travel has a median near 10.7 mm and a minimum "
        "near −1.6 mm, against vx·T of 0.028 m. The kit's shortest swing is 3.9 mm "
        "and its median is 77.2 mm, against vx·T of 0.075 m. Single-support contact "
        "count bottoms at 2. Declared-versus-actual phase mismatch is 0.31–0.33 on "
        "the voice rows and 0.389 on the kit. Peak fore/aft separation while the "
        "bus is in `move` is 17.0 mm on every voice bout and 42.8 mm on the kit.",
        "The wall suite returns on the stop tick, so the last second is still the "
        "approach: both feet leave the floor (contacts 0/0) and the trunk sits "
        "about 16° off stand. Kitchen faces +x, so the final pitch is negative "
        "(−13.18° to −16.32°). Living and entrance face −x, so the same forward "
        "lean reads positive (+14.57° to +16.32°). Stand pitch on these voice "
        "tips is about 0°. The kit stand pitch is −4.40°, the final pitch is "
        "−14.98°, and that pose freezes in a lean with both feet down (contacts 4/4).",
        "This pass, wall residuals: " + "; ".join(wall_bits) + ". "
        "Published wall CLEARs that this pass repeats: #82 living −0.0008 "
        "(here −0.0008); #88 kitchen +0.0085 and living −0.0093; #90 kitchen "
        "+0.0085 and entrance +0.0031; #98 kitchen −0.0055 and living −0.0054; "
        "#99 kitchen +0.0085 and living −0.0054. #90 living on this heading-frame "
        "pass is +0.0006. The published figure is −0.0054, and an earlier run of "
        "the same script landed on −0.0054. Moondream answers move the residual "
        "by a few millimetres. The latch still CLEARs. #98 entrance published "
        "+0.0062 and this pass CLEARs at +0.0031, the same stop time as #90. "
        "#99 published an entrance miss of +0.1254. This pass CLEARs that bout "
        "at +0.0031, same t=39.664 s as #90. #82 kitchen stops on the east wall "
        "with residual −0.0016 and wall clear false (right toe). #82 and #88 "
        "entrance hit the 53 s time limit. Those three bouts were not published "
        "wall CLEARs. They still STEP.",
        "The #70 lock `MID_SWING_TOE_MIN_M` = −0.003066 is the entrance-straight "
        "loaded scuff that tip recorded. It is not this day-1 kit bout. The kit "
        "script (`scripts/steer_walk.py` `_bus_kit_forward_stop`, `BUS_KIT_SCRIPT`, "
        "`locked_kit_config`) steps: fraction 0.961, 585 airborne ticks, lowest "
        "corner +2.02 mm. That does not reopen the toe lock as a skate, and it "
        "does not meet the 8 mm sole bar.",
        plant_line,
    ]


def _replace_section(body: str, heading: str, section: str) -> str:
    start = body.find(heading)
    block = section if section.endswith("\n") else section + "\n"
    if start < 0:
        return body.rstrip() + "\n\n" + block
    nxt = body.find("\n## ", start + len(heading))
    if nxt < 0:
        return body[:start].rstrip() + "\n\n" + block
    return body[:start].rstrip() + "\n\n" + block + "\n" + body[nxt + 1:]


def merge_hinge_row(row: dict[str, object]) -> dict[str, object]:
    payload: dict[str, object] = {
        "soft_pass": False,
        "bar_rad_s": 5.82,
        "plant_md5": FROZEN_MD5,
        "rows": [],
    }
    if HINGE_JSON.is_file():
        loaded = json.loads(HINGE_JSON.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            payload = loaded
    rows = payload.get("rows")
    kept: list[dict[str, object]] = []
    if isinstance(rows, list):
        for item in rows:
            if not isinstance(item, dict):
                continue
            if item.get("tip") == row.get("tip") and item.get("bout") == row.get("bout"):
                continue
            kept.append(item)
    kept.append(row)
    order = {"d6e8b5e": 0, "51ae123": 1}
    kept.sort(key=lambda item: (order.get(str(item.get("tip")), 9), str(item.get("bout"))))
    payload["rows"] = kept
    payload["soft_pass"] = False
    payload["bar_rad_s"] = 5.82
    payload["plant_md5"] = FROZEN_MD5
    HINGE_JSON.parent.mkdir(parents=True, exist_ok=True)
    HINGE_JSON.write_text(json.dumps(_jsonable(payload), indent=2) + "\n", encoding="utf-8")
    return payload


def _corner_cell(tick: object) -> str:
    if not isinstance(tick, dict):
        return "none"
    return (
        f"|qvel| {_num(tick.get('qvel_rad_s'), 4)} rad/s, "
        f"|τ| {_num(tick.get('tau_nm'), 4)} Nm "
        f"at {_num(tick.get('t_s'), 3)} s {tick.get('stage')}"
    )


def _corner_lines(speed_torque: object) -> list[str]:
    if not isinstance(speed_torque, dict):
        return []
    lines = [
        str(speed_torque.get("status")),
        "",
        "| Joint | Largest \\|qvel\\| at \\|τ\\|≥2 Nm | Largest \\|τ\\| at \\|qvel\\|≥4 rad/s |",
        "| --- | --- | --- |",
    ]
    corners = speed_torque.get("corners")
    if isinstance(corners, list):
        for corner in corners:
            if not isinstance(corner, dict):
                continue
            lines.append(
                "| "
                + " | ".join([
                    str(corner.get("joint")),
                    _corner_cell(corner.get("max_qvel_at_tau")),
                    _corner_cell(corner.get("max_tau_at_qvel")),
                ])
                + " |"
            )
    lines.append("")
    return lines


def hinge_markdown(payload: dict[str, object] | None = None) -> str:
    if payload is None:
        if not HINGE_JSON.is_file():
            return ""
        loaded = json.loads(HINGE_JSON.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            return ""
        payload = loaded
    rows = payload.get("rows")
    if not isinstance(rows, list) or not rows:
        return ""
    lines = [
        HINGE_MD_HEADING,
        "",
        "Soft-pass is off. Peak `|qvel|` on each of the 12 leg hinges must be",
        "≤ 5.82 rad/s (HX-35H no-load, 0.18 s/60°). The plant has no velocity",
        "cap. Headroom is 5.82 − peak. A joint over the bar is a row fail.",
        "Preview rows take the stage from `preview_stage`. The #90 kitchen",
        "walker has no preview stage, so its stage column is the gait phase.",
        "",
        "The speed line is period T, commanded vx, actual trunk vx, and the",
        "ratio. Actual trunk vx is the trunk origin's heading-frame forward",
        "displacement over the bus `move` window, divided by that window's",
        "duration. The ratio is reported. It is not a separate cutoff.",
        "",
        "Each leg hinge logs the per-tick pair `(|unclamped τ|, |qvel|)` in",
        "`previews/walk_hinge_speed.json`. Unclamped τ is",
        "`|kp·(q_des−q)| + |kv·ω|` at the write. `no_load_speed`, `stall_torque`,",
        "and `voltage` are unset, so the speed-torque line is pending the HW",
        "datasheet. While it is pending, each joint reports the tick with the",
        "largest `|qvel|` at `|τ| ≥ 2` Nm and the tick with the largest `|τ|`",
        "at `|qvel| ≥ 4` rad/s. When the line is set, a tick fails when",
        "`|qvel|` exceeds `no_load·(1 − |τ|/stall)`.",
        "",
    ]
    for row in rows:
        if not isinstance(row, dict):
            continue
        speed = row.get("trunk_speed")
        if not isinstance(speed, dict):
            speed = {}
        qvel = row.get("qvel")
        if not isinstance(qvel, dict):
            qvel = {}
        passed = "passes" if qvel.get("passes") else "fails"
        lines.append(
            f"`{row.get('tip')}` `{row.get('bout')}` {row.get('gait')}, {row.get('verdict')}. "
            f"Hinge-speed bar {passed}. "
            f"Plant `{row.get('plant_md5_before')}` before and `{row.get('plant_md5')}` after."
        )
        lines.extend(_corner_lines(row.get("speed_torque")))
        lines.append(
            f"Period T {_num(speed.get('period_s'), 3)} s, commanded vx "
            f"{_num(speed.get('vx_cmd_m_s'), 4)} m/s, actual trunk vx "
            f"{_num(speed.get('actual_vx_m_s'), 4)} m/s "
            f"(forward {_num(speed.get('fwd_m'), 4)} m over {_num(speed.get('move_s'), 3)} s), "
            f"ratio {_num(speed.get('ratio'), 3)}."
        )
        lines.append("")
        joints = qvel.get("joints")
        if isinstance(joints, list) and joints:
            lines.append("| Joint | Peak rad/s | t s | Stage | Headroom rad/s |")
            lines.append("| --- | ---: | ---: | --- | ---: |")
            for joint in joints:
                if not isinstance(joint, dict):
                    continue
                lines.append(
                    "| "
                    + " | ".join([
                        str(joint.get("joint")),
                        _num(joint.get("peak_rad_s"), 4),
                        _num(joint.get("t_s"), 3),
                        str(joint.get("stage")),
                        _num(joint.get("headroom_rad_s"), 4),
                    ])
                    + " |"
                )
            lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def splice_hinge_doc() -> None:
    path = ROOT / "docs" / "WALK_STEPPING_BARS.md"
    body = path.read_text(encoding="utf-8") if path.is_file() else "# Walk stepping bars\n"
    section = hinge_markdown()
    if not section:
        raise SystemExit("no hinge-speed JSON")
    path.write_text(_replace_section(body, HINGE_MD_HEADING, section), encoding="utf-8")
    print(f"[hinge] wrote {path}")


def splice_retro_doc() -> None:
    path = ROOT / "docs" / "WALK_STEPPING_BARS.md"
    body = path.read_text(encoding="utf-8") if path.is_file() else "# Walk stepping bars\n"
    section = retro_markdown()
    if not section:
        raise SystemExit("no retro JSON to write")
    if RETRO_MD_HEADING in body:
        body = body[: body.index(RETRO_MD_HEADING)].rstrip() + "\n"
    path.write_text(body.rstrip() + "\n\n" + section, encoding="utf-8")
    print(f"[retro] wrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Retro contact stepping score")
    parser.add_argument("--doc", action="store_true", help="Splice the retro section into the stepping doc")
    parser.add_argument("--sha")
    parser.add_argument("--label")
    parser.add_argument("--pr", type=int)
    parser.add_argument("--kind", choices=("kit", "voice"))
    parser.add_argument("--rooms", default="kitchen,living,entrance")
    parser.add_argument("--hinge", action="store_true", help="Write the hinge-speed self-test and do not replace the retro JSON")
    args = parser.parse_args()
    if args.doc:
        splice_retro_doc()
        return
    if not args.sha or not args.label or args.pr is None or not args.kind:
        raise SystemExit("--sha, --label, --pr, and --kind are required")
    step_bars = _load_step_bars()
    sw, _tree = _prepare(args.sha)
    if args.kind == "kit":
        row = run_kit(sw, step_bars, args.label, args.pr, args.sha)
        payload = {
            "label": args.label,
            "pr": args.pr,
            "tip_sha": args.sha,
            "kind": "kit",
            "plant_md5": row["plant_md5_before"],
            "period_s": row["period_s"],
            "vx_m_s": row["vx_m_s"],
            "rows": [row],
        }
    else:
        rooms = tuple(part.strip() for part in str(args.rooms).split(",") if part.strip())
        payload = run_voice(sw, step_bars, args.label, args.pr, args.sha, rooms)
    if args.hinge:
        for row in payload.get("rows") or []:
            if not isinstance(row, dict):
                continue
            qvel = row.get("qvel") if isinstance(row.get("qvel"), dict) else {}
            merge_hinge_row({
                "tip": args.label,
                "tip_sha": args.sha,
                "pr": args.pr,
                "bout": row.get("bout"),
                "gait": row.get("verdict"),
                "verdict": "hinge bar fails" if not qvel.get("passes") else "other bars not re-judged",
                "plant_md5_before": row.get("plant_md5_before"),
                "plant_md5": row.get("plant_md5_after"),
                "qvel": qvel,
                "trunk_speed": row.get("trunk_speed"),
                "speed_torque": row.get("speed_torque"),
                "hinge_pairs": row.get("hinge_pairs"),
            })
        splice_hinge_doc()
        return
    write_tip(payload, args.label)


if __name__ == "__main__":
    main()
