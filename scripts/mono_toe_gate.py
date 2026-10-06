#!/usr/bin/env python3
"""Toe-gap range gate. Day-1 stop only. Plant file is not edited.

The mono range is the ground distance from kit_cam implied by one pixel
row, the live camera height and position, and the live world pitch.
World pitch is the torso IMU pitch plus head_tilt at that frame.
Height is kit_cam world z. The floor plane is z = 0.

The compare uses the furthest toe either foot reaches during the
walk, not the sole at one frame. On this kitchen walk that high
water is +0.017 m (right foot, t = 3.056 s). A full gait period is
0.400 s. After the gait is up the right toe peaks near +0.015 m and
the left near +0.013 m. Stand is −0.016 m. The unstopped hit is the
left ankle, so a single-frame planted sole is the wrong offset.

    cam_z = kit_cam world z at the frame (floor plane z = 0)
    eye_range = cam_z / tan(depression(row, IMU + head_tilt))
    step_off = max forward toe offset of either foot since t = 1 s
    toe_gap = eye_range − step_off
    compare (toe_gap − buffer) with d_min
    buffer in {0.03, 0.05}
    d_min = 0.150 * (T_detect + T_stop)

0.1263 m at T_detect = 0 is the floor, not a safe gap. The buffer is
a fixed slip allowance. It is not a measured kit odometry error.
T_detect 0.033 s is one 30 fps frame and 0.100 s is the blind-zone
warning. Neither is a kit measurement. 0.342 ms is not plugged in.

The row model ignores the column. The buffered stops fire on
col_chair_stool_b_leg_0, whose eye range reads about 0.18 m short of
the true camera-to-floor gap. That early trip is not a calibrated
toe gap. Near the center, leg_2 at t = 5.824 s reads 0.015 m short.
The head is not tilted. A −10° walk stays off.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("DISPLAY", ":1")
os.environ.setdefault("MUJOCO_GL", "glfw")

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import mujoco as mj
import numpy as np

import gait_manager_traj as gm
import steer_walk as sw

SCENE = Path("/tmp/m62/mujoco/room_kitchen.xml")
# Range uses kit_cam world z at the frame. Floor plane is z = 0.
# Stand eye height on this plant is about 0.335 m. That number is not
# the height in the formula.
WIDTH = 640
HEIGHT = 480
FOVY_DEG = 104.82
V_MPS = 0.150
# Empty-plant all-leg hold, worst of the gait-period grid, was 0.832 s.
# The live-height kitchen dead-reckon stop settled in 0.842 s. The
# gate uses the longer one. Updated again if a rerun settles later.
T_STOP_S = 0.842
# One left-plus-right cycle. The toe offset is the furthest either foot
# reaches ahead of kit_cam during that cycle, not the sole at one frame.
GAIT_PERIOD_S = gm.GM_PERIOD_S
# MFG slip buffer. 0.126 m is the T_detect=0 floor, not a safe gap.
# 0.03 m is about one commanded step. 0.05 m is the larger trial.
BUFFERS_M = (0.03, 0.05)
# Bottom band. A contact here is about to leave the floor of the frame.
NEAR_PX = 32
LEG_TOKS = ("hip_", "knee", "ank_")
SPEED_EPS = 0.02
HOLD_S = 0.20


def d_min(t_detect_s: float, t_stop_s: float = T_STOP_S) -> float:
    return V_MPS * (float(t_detect_s) + float(t_stop_s))


def _high_water(samples: list[tuple[float, float, float]]) -> tuple[float, str, float]:
    """Furthest toe ahead of the camera since the walk started.

    Each sample is (t, left offset, right offset). Positive is ahead.
    """
    best = -1.0e9
    side = "?"
    when = 0.0
    for t, left, right in samples:
        if left > best:
            best, side, when = left, "L", t
        if right > best:
            best, side, when = right, "R", t
    if best < -1.0e8:
        return 0.0, "?", 0.0
    return best, side, when


def _period_lines(samples: list[tuple[float, float, float]]) -> list[str]:
    """Max toe offset in each gait period after the walk starts."""
    if not samples:
        return []
    t0 = 1.0
    last = samples[-1][0]
    lines: list[str] = []
    while t0 <= last + 1e-9:
        t1 = t0 + GAIT_PERIOD_S
        window = [row for row in samples if t0 - 1e-9 <= row[0] < t1]
        if window:
            left = max(row[1] for row in window)
            right = max(row[2] for row in window)
            either = max(left, right)
            side = "L" if left >= right else "R"
            lines.append(
                f"period {t0:.2f}-{t1:.2f} L={left:+.3f} R={right:+.3f} "
                f"max={either:+.3f} {side}"
            )
        t0 = t1
    return lines


def _fy() -> float:
    return (HEIGHT / 2.0) / math.tan(math.radians(FOVY_DEG) / 2.0)


def torso_pitch(data: mj.MjData, bid: int) -> float:
    """IMU pitch of body_link. Negative is nose-down."""
    rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
    forward = rot[:, 0]
    return math.atan2(float(forward[2]), float(math.hypot(forward[0], forward[1])))


def body_forward_xy(data: mj.MjData, bid: int) -> np.ndarray:
    rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
    forward = rot[:2, 0].copy()
    norm = float(np.linalg.norm(forward))
    if norm < 1e-9:
        return np.array([1.0, 0.0], dtype=np.float64)
    return forward / norm


def range_from_row(row: float, pitch: float, height: float) -> float | None:
    """Horizontal floor range from the point under the eye.

    Pitch is world pitch (IMU torso plus head_tilt). Height is kit_cam
    world z at this frame. The floor plane is z = 0.
    """
    if height <= 1e-4:
        return None
    beta = math.atan((row - (HEIGHT / 2.0)) / _fy())
    depression = -(pitch - beta)
    if depression <= math.radians(1.0):
        return None
    return height / math.tan(depression)


def _project(data: mj.MjData, cid: int, point: np.ndarray) -> tuple[float, float] | None:
    rot = np.asarray(data.cam_xmat[cid], dtype=np.float64).reshape(3, 3)
    cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    local = rot.T @ (point - cam)
    depth = -float(local[2])
    if depth <= 1e-4:
        return None
    fy = _fy()
    u = (WIDTH / 2.0) + fy * (float(local[0]) / depth)
    v = (HEIGHT / 2.0) - fy * (float(local[1]) / depth)
    return u, v


@dataclass(frozen=True)
class ToeSample:
    side: str
    swing: bool
    offset_m: float
    toe_xy: tuple[float, float]


def _box_bottom_corners(
    model: mj.MjModel, data: mj.MjData, bid: int, gid: int,
) -> list[np.ndarray]:
    rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(data.xpos[bid], dtype=np.float64)
    pos = np.asarray(model.geom_pos[gid], dtype=np.float64)
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    corners: list[np.ndarray] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            local = np.array(
                [
                    pos[0] + sx * half[0],
                    pos[1] + sy * half[1],
                    pos[2] - half[2],
                ],
                dtype=np.float64,
            )
            corners.append(origin + rot @ local)
    return corners


def toe_samples(session: sw.SteerSession, cid: int) -> list[ToeSample]:
    """Forward offset of each sole's leading bottom corner, eye to toe.

    Positive means the toe is ahead of kit_cam along body forward.
    """
    data = session.data
    model = session.model
    cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    fwd = body_forward_xy(data, session.bid_body)
    swing = session.lipm._gm_swing if session.lipm is not None else None
    rows: list[ToeSample] = []
    for side, bid, gid in (
        ("L", session.bid_lf, session.gid_lfoot),
        ("R", session.bid_rf, session.gid_rfoot),
    ):
        best_off = -1e9
        best_xy = (0.0, 0.0)
        for corner in _box_bottom_corners(model, data, bid, gid):
            delta = corner[:2] - cam[:2]
            off = float(np.dot(delta, fwd))
            if off > best_off:
                best_off = off
                best_xy = (float(corner[0]), float(corner[1]))
        rows.append(ToeSample(side, swing == side, best_off, best_xy))
    return rows


def _leg_floors(model: mj.MjModel, data: mj.MjData) -> list[tuple[str, np.ndarray]]:
    rows: list[tuple[str, np.ndarray]] = []
    for gid in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or ""
        if "stool" not in name or "_leg_" not in name:
            continue
        center = np.asarray(data.geom_xpos[gid], dtype=np.float64)
        half_z = float(model.geom_size[gid, 2])
        point = center.copy()
        point[2] = center[2] - half_z
        rows.append((name, point))
    return rows


def _is_wall(name: str) -> bool:
    return name.startswith("col_room_wall") or name == "col_entrance_sill"


def _prop_hit(model: mj.MjModel, data: mj.MjData) -> tuple[str, str, float] | None:
    best: tuple[str, str, float] | None = None
    for i in range(data.ncon):
        con = data.contact[i]
        n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(con.geom1)) or ""
        n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(con.geom2)) or ""
        other = [n for n in (n1, n2) if n.startswith("col_") and not _is_wall(n)]
        if not other:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, force)
        fn = float(force[0])
        if fn < 0.5:
            continue
        robot = n1 if n2 == other[0] else n2
        if best is None or fn > best[2]:
            best = (robot, other[0], fn)
    return best


@dataclass
class Sight:
    name: str
    row: float
    eye_m: float
    toe_gap_m: float
    body_xy: tuple[float, float]
    fwd: tuple[float, float]
    in_near_band: bool
    t: float
    gt_eye_m: float
    gt_toe_m: float
    offset_m: float
    cam_z: float


def walk(
    t_detect: float,
    t_end: float = 9.0,
    arm: bool = True,
    buffer_m: float = 0.0,
) -> dict[str, object]:
    session = sw.SteerSession(video=False, scene_xml=SCENE, lipm=sw.locked_kit_config())
    model = session.model
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    peaks: dict[str, tuple[float, float]] = {}
    stop_peaks: dict[str, tuple[float, float]] = {}
    real = mj.mj_step
    contact: tuple[float, str, str, float] | None = None
    stop_mark = {"t": None}
    com_hist: list[tuple[float, float, float]] = []
    prev_com: np.ndarray | None = None
    prev_t: float | None = None

    def hook(m: mj.MjModel, d: mj.MjData) -> None:
        nonlocal contact, prev_com, prev_t
        real(m, d)
        t_now = float(d.time)
        # World subtree_com includes the room. Use the body.
        com = np.asarray(d.subtree_com[session.bid_body, :2], dtype=np.float64).copy()
        speed = 0.0
        if prev_com is not None and prev_t is not None and t_now > prev_t:
            speed = float(np.linalg.norm(com - prev_com) / (t_now - prev_t))
        com_hist.append((t_now, float(com[0]), float(com[1]), speed))
        prev_com = com
        prev_t = t_now
        hit = _prop_hit(m, d)
        if hit is not None and contact is None:
            contact = (t_now, hit[0], hit[1], hit[2])
        for i in range(m.nu):
            name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
            if not any(tok in name for tok in LEG_TOKS):
                continue
            force = float(d.actuator_force[i])
            prev = peaks.get(name)
            if prev is None or abs(force) > abs(prev[0]):
                peaks[name] = (force, t_now)
            mark = stop_mark["t"]
            if mark is not None and t_now + 1e-9 >= float(mark):
                prev_s = stop_peaks.get(name)
                if prev_s is None or abs(force) > abs(prev_s[0]):
                    stop_peaks[name] = (force, t_now)

    mj.mj_step = hook
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t_end, "vel", sw.VX_FWD_CAP, -sw.YAW_RATE_CAP, "turn"),
    ))
    logs: list[str] = []
    issued: float | None = None
    issued_xy: tuple[float, float] | None = None
    reason = ""
    min_foot_leg = 1e9
    latched: dict[str, Sight] = {}
    bias_log: list[str] = []
    reach: list[tuple[float, float, float]] = []
    gate = d_min(t_detect)
    try:
        while float(session.data.time) < t_end - 1e-9:
            now = float(session.data.time)
            if issued is None and now >= 1.0 - 1e-9:
                choice = _decide(
                    session, cid, jid, latched, gate, bias_log, reach, buffer_m,
                )
                if arm and choice is not None:
                    session.bus.stop(now)
                    issued = now
                    issued_xy = (float(session.data.qpos[0]), float(session.data.qpos[1]))
                    stop_mark["t"] = now
                    reason = choice
                    logs.append(f"STOP t={now:.3f} {choice}")
                    logs.append(_pose_line(session, cid, jid))
                    for lname, sight in latched.items():
                        if lname not in choice:
                            continue
                        logs.append(
                            f"stop_bias {lname} eye={sight.eye_m:.3f} "
                            f"eye_gt={sight.gt_eye_m:.3f} "
                            f"eye_bias={sight.eye_m - sight.gt_eye_m:+.3f} "
                            f"cmp={sight.toe_gap_m:.3f} true_min={sight.gt_toe_m:.3f}"
                        )
            if issued is None:
                driver.publish(session.bus, now)
            session.step()
            if _want_log(float(session.data.time)):
                logs.append(_fmt(session, cid, jid, latched))
                t_log = float(session.data.time)
                if abs(t_log - 1.00) < 0.012 or abs(t_log - 5.90) < 0.012:
                    logs.append(_pose_line(session, cid, jid))
            if issued is not None:
                for _name, point in _leg_floors(model, session.data):
                    for row in toe_samples(session, cid):
                        dist = math.hypot(point[0] - row.toe_xy[0], point[1] - row.toe_xy[1])
                        if dist < min_foot_leg:
                            min_foot_leg = dist
            if contact is not None and issued is None:
                break
    finally:
        mj.mj_step = real

    t_stop = _settle(com_hist, issued)
    if issued is not None:
        speeds = [
            (row[0] - issued, row[3])
            for row in com_hist
            if issued - 1e-9 <= row[0] <= issued + 1.2
        ]
        step = max(1, len(speeds) // 8)
        logs.append(
            "speed " + " ".join(f"{t:.2f}:{v:.3f}" for t, v in speeds[::step])
        )
    stop_worst = max(stop_peaks.items(), key=lambda kv: abs(kv[1][0])) if stop_peaks else None
    stop_over = [
        (n, f, t) for n, (f, t) in stop_peaks.items() if abs(f) > sw.lipm_gait.LEG_STOP_NM + 1e-3
    ]
    return {
        "issued": issued,
        "reason": reason,
        "contact": contact,
        "t_stop": t_stop,
        "min_up_z": session.min_up_z,
        "yaw": math.degrees(session.yaw()),
        "xy": (float(session.data.qpos[0]), float(session.data.qpos[1])),
        "issued_xy": issued_xy,
        "min_foot_leg": None if min_foot_leg > 10 else min_foot_leg,
        "fault": session.bus.fault_reason,
        "stop_worst": None if stop_worst is None else (stop_worst[0], stop_worst[1][0], stop_worst[1][1]),
        "stop_over": stop_over,
        "gate_m": gate,
        "logs": logs,
        "bias": bias_log,
        "latched": {
            name: (
                f"latch t={sight.t:.3f} {name} row={sight.row:.0f} "
                f"eye={sight.eye_m:.3f} eye_gt={sight.gt_eye_m:.3f} "
                f"eye_bias={sight.eye_m - sight.gt_eye_m:+.3f} "
                f"toe={sight.toe_gap_m:.3f} toe_gt={sight.gt_toe_m:.3f} "
                f"toe_bias={sight.toe_gap_m - sight.gt_toe_m:+.3f} "
                f"off={sight.offset_m:+.3f} cam_z={sight.cam_z:.3f} "
                f"near={int(sight.in_near_band)}"
            )
            for name, sight in latched.items()
        },
        "plant": sw._md5(sw.PLANT_XML),
        "buffer_m": buffer_m,
        "step_off": _high_water(reach),
        "periods": _period_lines(reach),
    }


def _want_log(t: float) -> bool:
    if abs(t - 1.0) < 0.012 or abs(t - 1.90) < 0.012:
        return True
    if 5.40 <= t <= 6.20 and abs((t * 50) - round(t * 50)) < 0.2 and abs(t * 10 - round(t * 10)) < 0.08:
        return True
    return False


def _decide(
    session: sw.SteerSession,
    cid: int,
    jid: int,
    latched: dict[str, Sight],
    gate: float,
    bias_log: list[str],
    reach: list[tuple[float, float, float]],
    buffer_m: float,
) -> str | None:
    data = session.data
    model = session.model
    tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
    pitch = torso_pitch(data, session.bid_body) + tilt
    cam_z = float(data.cam_xpos[cid][2])
    toes = toe_samples(session, cid)
    now = float(data.time)
    if now >= 1.0 - 1e-9:
        left = next(row.offset_m for row in toes if row.side == "L")
        right = next(row.offset_m for row in toes if row.side == "R")
        reach.append((now, left, right))
    step_off, step_side, _step_t = _high_water(reach)
    # Instantaneous furthest sole. The swing foot is often behind the eye.
    # The gate does not use it. The compare uses the gait-period high water.
    toe = max(toes, key=lambda row: row.offset_m)
    fwd = body_forward_xy(data, session.bid_body)
    seen: set[str] = set()
    best: tuple[float, str] | None = None
    edge = range_from_row(float(HEIGHT - 1), pitch, cam_z)
    for name, point in _leg_floors(model, data):
        pix = _project(data, cid, point)
        if pix is None:
            continue
        u, v = pix
        if not (0.0 <= u < WIDTH and 0.0 <= v < HEIGHT):
            continue
        eye = range_from_row(v, pitch, cam_z)
        if eye is None:
            continue
        seen.add(name)
        # eye_range minus the furthest toe this gait has reached, minus the
        # fixed slip buffer. Compared with d_min.
        gap = eye - step_off - buffer_m
        near = v >= (HEIGHT - NEAR_PX)
        margin = 0.0
        if near and edge is not None:
            margin = max(0.0, eye - edge)
        body_xy = (float(data.qpos[0]), float(data.qpos[1]))
        cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
        gt_eye = float(math.hypot(point[0] - cam[0], point[1] - cam[1]))
        gt_toe = min(
            float(math.hypot(point[0] - row.toe_xy[0], point[1] - row.toe_xy[1]))
            for row in toes
        )
        sight = Sight(
            name, v, eye, gap, body_xy, (float(fwd[0]), float(fwd[1])), near,
            float(data.time), gt_eye, gt_toe, step_off, cam_z,
        )
        prev = latched.get(name)
        latched[name] = sight
        if near and (prev is None or not prev.in_near_band or sight.t - prev.t >= 0.04):
            inst_gap = eye - toe.offset_m
            inst_gt = float(math.hypot(
                point[0] - toe.toe_xy[0], point[1] - toe.toe_xy[1],
            ))
            bias_log.append(
                f"bias t={sight.t:.3f} {name} row={v:.0f} "
                f"eye={eye:.3f} eye_gt={gt_eye:.3f} eye_bias={eye - gt_eye:+.3f} "
                f"inst_toe={inst_gap:.3f} inst_gt={inst_gt:.3f} "
                f"inst_bias={inst_gap - inst_gt:+.3f} "
                f"step_off={step_off:+.3f} {step_side} buf={buffer_m:.3f} "
                f"cmp={gap:.3f} true_min={gt_toe:.3f} cam_z={cam_z:.3f}"
            )
        trigger = gap
        why = "toe_gap"
        if near and gap <= gate + margin and gap > gate:
            trigger = gap
            why = "near_edge"
            if best is None or trigger < best[0]:
                best = (
                    trigger,
                    f"{why} {name} cmp={gap:.3f} margin={margin:.3f} "
                    f"eye={eye:.3f} cam_z={cam_z:.3f} step_off={step_off:+.3f} "
                    f"{step_side} buf={buffer_m:.3f}",
                )
            continue
        if gap <= gate and (best is None or gap < best[0]):
            best = (
                gap,
                f"{why} {name} cmp={gap:.3f} eye={eye:.3f} cam_z={cam_z:.3f} "
                f"step_off={step_off:+.3f} {step_side} buf={buffer_m:.3f} row={v:.0f}",
            )
    for name, sight in list(latched.items()):
        if name in seen or not sight.in_near_band:
            continue
        body_xy = (float(data.qpos[0]), float(data.qpos[1]))
        advance = float(np.dot(
            np.array(body_xy, dtype=np.float64) - np.array(sight.body_xy, dtype=np.float64),
            np.array(sight.fwd, dtype=np.float64),
        ))
        dr = sight.eye_m - step_off - buffer_m - advance
        if dr <= gate and (best is None or dr < best[0]):
            best = (
                dr,
                f"dead_reckon {name} cmp={dr:.3f} advance={advance:.3f} "
                f"cam_z={cam_z:.3f} step_off={step_off:+.3f} {step_side} "
                f"buf={buffer_m:.3f}",
            )
    if best is None:
        return None
    return best[1]


def _pose_line(session: sw.SteerSession, cid: int, jid: int) -> str:
    """Toe minus camera, in the body frame. +x is forward."""
    data = session.data
    rot = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
    cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    toes = toe_samples(session, cid)
    bits: list[str] = []
    for row in toes:
        world = np.array([row.toe_xy[0], row.toe_xy[1], 0.0], dtype=np.float64)
        # Height of the sole corner is not in ToeSample. Forward/lateral use xy.
        delta_xy = np.array([row.toe_xy[0] - cam[0], row.toe_xy[1] - cam[1], 0.0])
        local = rot.T @ delta_xy
        mark = "*" if row.swing else ""
        bits.append(f"{row.side}{mark} x={local[0]:+.3f} y={local[1]:+.3f}")
    gaps: list[str] = []
    for name, point in _leg_floors(session.model, data):
        dists = [
            math.hypot(point[0] - row.toe_xy[0], point[1] - row.toe_xy[1])
            for row in toes
        ]
        gaps.append(f"{name} {min(dists):.3f}")
    tilt = float(data.qpos[int(session.model.jnt_qposadr[jid])])
    pitch = torso_pitch(data, session.bid_body) + tilt
    toe = max(toes, key=lambda row: row.offset_m)
    edge = range_from_row(float(HEIGHT - 1), pitch, float(cam[2]))
    near = "near=none"
    if edge is not None:
        near = f"near_eye={edge:.3f} near_toe={edge - toe.offset_m:.3f} off={toe.offset_m:+.3f}"
    return (
        f"pose cam_z={cam[2]:.3f} pitch={math.degrees(pitch):+.2f} {near} "
        + " ".join(bits)
        + " | "
        + " ".join(gaps)
    )


def _fmt(session: sw.SteerSession, cid: int, jid: int, latched: dict[str, Sight]) -> str:
    del latched
    data = session.data
    model = session.model
    tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
    pitch = torso_pitch(data, session.bid_body) + tilt
    cam_z = float(data.cam_xpos[cid][2])
    toes = toe_samples(session, cid)
    toe = max(toes, key=lambda row: row.offset_m)
    parts = " ".join(
        f"{row.side}{'*' if row.swing else ''}={row.offset_m:+.3f}" for row in toes
    )
    edge = range_from_row(float(HEIGHT - 1), pitch, cam_z)
    near = "near=none" if edge is None else f"near_eye={edge:.3f} near_toe={edge - toe.offset_m:.3f}"
    nearest = ""
    best_gap: float | None = None
    for name, point in _leg_floors(model, data):
        pix = _project(data, cid, point)
        if pix is None:
            continue
        u, v = pix
        if not (0.0 <= u < WIDTH and 0.0 <= v < HEIGHT):
            continue
        eye = range_from_row(v, pitch, cam_z)
        if eye is None:
            continue
        gap = eye - toe.offset_m
        gt = float(math.hypot(point[0] - toe.toe_xy[0], point[1] - toe.toe_xy[1]))
        if best_gap is None or gap < best_gap:
            best_gap = gap
            nearest = (
                f" {name} row={v:.0f} col={u:.0f} eye={eye:.3f} "
                f"toe_gap={gap:.3f} gt_toe={gt:.3f}"
            )
    return (
        f"t={float(data.time):.2f} pitch={math.degrees(pitch):+.2f} "
        f"cam_z={cam_z:.3f} {near} "
        f"toes {parts} gate_toe={toe.side}{' swing' if toe.swing else ''} "
        f"{toe.offset_m:+.3f}{nearest}"
    )


def _settle(hist: list[tuple[float, float, float, float]], issued: float | None) -> float | None:
    if issued is None:
        return None
    after = [row for row in hist if row[0] + 1e-9 >= issued]
    if len(after) < 2:
        return None
    quiet = 0.0
    prev = after[0]
    for row in after[1:]:
        dt = row[0] - prev[0]
        if row[3] < SPEED_EPS:
            quiet += dt
            if quiet >= HOLD_S:
                return row[0] - issued
        else:
            quiet = 0.0
        prev = row
    return None


def _print_result(result: dict[str, object], t_detect: float, mode: str) -> None:
    step = result["step_off"]
    assert isinstance(step, tuple)
    print(
        f"plant {result['plant']} mode={mode} T_detect={t_detect} "
        f"buffer={result['buffer_m']} d_min={result['gate_m']:.4f} "
        f"step_off={step[0]:+.3f} {step[1]} t={step[2]:.3f} "
        f"issued={result['issued']} reason={result['reason']} "
        f"contact={result['contact']} T_stop_run={result['t_stop']} "
        f"min_up_z={result['min_up_z']:.3f} yaw={result['yaw']:+.1f} "
        f"xy={result['xy']} issued_xy={result['issued_xy']} "
        f"min_foot_leg={result['min_foot_leg']} fault={result['fault']} "
        f"stop_worst={result['stop_worst']} stop_over={len(result['stop_over'])}"
    )
    periods = result["periods"]
    assert isinstance(periods, list)
    for line in periods:
        print(line)
    logs = result["logs"]
    assert isinstance(logs, list)
    for line in logs:
        print(line)
    bias = result["bias"]
    assert isinstance(bias, list)
    for line in bias:
        print(line)


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "gate"
    if mode == "stress":
        for t_detect in (0.0, 0.033, 0.100):
            for buffer_m in BUFFERS_M:
                result = walk(t_detect, arm=True, buffer_m=buffer_m)
                _print_result(result, t_detect, mode)
                print("---")
        return
    t_detect = 0.0 if len(sys.argv) < 3 else float(sys.argv[2])
    buffer_m = 0.0 if len(sys.argv) < 4 else float(sys.argv[3])
    arm = mode != "reach"
    t_end = 6.6 if mode == "reach" else 9.0
    result = walk(t_detect, t_end=t_end, arm=arm, buffer_m=buffer_m)
    _print_result(result, t_detect, mode)


if __name__ == "__main__":
    main()
