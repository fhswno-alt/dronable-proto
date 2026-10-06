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

``walk_latch`` is the sim projection of a stool-leg floor point. It
stops when ``in_corridor`` and ``toe_gap_m <= d_min``. That gap already
includes the 20 mm pad and the live step offset. ``latch-pad`` reruns
that projection with a frozen pad of 0.035 m and does not replace the
0.020 m pad.

Day-1 stop for the kitchen walk is ``walk_finder``. The pixel comes
from ``hazard_finder.find_hazard_cues`` on the kit_cam RGB frame.
Each in-corridor hit keeps a world floor point. A later frame may
replace that point only when the gap gets shorter. Every tick the
point is re-read in the current body x, y, and yaw. It is dropped
when that reading leaves the corridor. A new cue joins a saved
point only when the two floor positions are within 0.035 m after
that move. The radius is a constant frozen before the run. Sim leg
names are not an input; a name is attached only when the stop is
reported. ``stop`` is sent when the shortest remaining gap is at or
under ``d_min``, or when a cue sets ``too_close``. The pad stays
0.020. No 3–5 cm buffer is added. ``finder-long`` is a stress on
that latch: the frame after the gap first falls to ``d_min + 0.05``
adds 0.10 m to that floor point's toe gap, then the finder is
normal again. The older row-model stress remains in this file and
is not the latch.

On the open ±0.25 walks, existing furniture-leg centers enter the
strip between 0.0867 m and 0.108 m on kitchen left, both living
turns, bathroom right, and entrance right. The 0.020 m pad already
counts a point as inside the stand corridor until |sideways| passes
0.1067 m. The only centers past that line are 0.57 m or more ahead,
so widening the outside edge is not what stops these walks. No room
collider was moved.
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
import hazard_finder as hf
import ray_corridor as rc
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


def _prop_floor_centers(
    model: mj.MjModel, data: mj.MjData,
) -> list[tuple[str, np.ndarray]]:
    """Report-only floor centers of non-wall colliders. Not a stop input."""
    rows: list[tuple[str, np.ndarray]] = []
    for gid in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or ""
        if not name.startswith("col_") or _is_wall(name):
            continue
        center = np.asarray(data.geom_xpos[gid], dtype=np.float64)
        point = center.copy()
        point[2] = 0.0
        rows.append((name, point))
    return rows


def _leg_floor_centers(
    model: mj.MjModel, data: mj.MjData,
) -> list[tuple[str, np.ndarray]]:
    """Stool-leg floor points at the geom's xy, z = 0. Size is not read."""
    rows: list[tuple[str, np.ndarray]] = []
    for gid in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or ""
        if "stool" not in name or "_leg_" not in name:
            continue
        center = np.asarray(data.geom_xpos[gid], dtype=np.float64)
        point = center.copy()
        point[2] = 0.0
        rows.append((name, point))
    return rows


def walk_latch(
    t_detect: float,
    t_end: float = 9.0,
    hazard_pad_m: float = rc.HAZARD_PAD_M,
) -> dict[str, object]:
    """Day-1 stop on ray_corridor. No extra buffer. Head tilt stays off.

    The pixel is the sim projection of a stool-leg floor point. It is not
    an RGB finder. ``toe_gap_m`` already includes the pad and the live
    step offset, so this latch does not subtract them again. The shipped
    pad is 0.020 m. A stress run may pass another frozen constant.
    """
    if not SCENE.is_file():
        raise SystemExit(f"missing kitchen scene {SCENE}")
    session = sw.SteerSession(video=False, scene_xml=SCENE, lipm=sw.locked_kit_config())
    model = session.model
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    pan_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_pan")
    peaks: dict[str, tuple[float, float]] = {}
    stop_peaks: dict[str, tuple[float, float]] = {}
    real = mj.mj_step
    contact: tuple[float, str, str, float] | None = None
    stop_mark: dict[str, float | None] = {"t": None}
    com_hist: list[tuple[float, float, float, float]] = []
    prev_com: np.ndarray | None = None
    prev_t: float | None = None

    def hook(m: mj.MjModel, d: mj.MjData) -> None:
        nonlocal contact, prev_com, prev_t
        real(m, d)
        t_now = float(d.time)
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
    gate = rc.d_min(t_detect)
    pad = float(hazard_pad_m)
    reach: list[tuple[float, float, float]] = []
    latched: dict[str, rc.DeadReckonState] = {}
    seen_names: set[str] = set()
    issued: float | None = None
    reason = ""
    false_early = ""
    in_corridor_names: set[str] = set()
    stop_frame = ""
    leg0_closest = ""
    leg0_side_abs = 1e9
    min_foot_leg = 1e9
    head_tilt_peak = 0.0
    try:
        while float(session.data.time) < t_end - 1e-9:
            now = float(session.data.time)
            if issued is None and now >= 1.0 - 1e-9:
                data = session.data
                tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
                pan = 0.0 if pan_id < 0 else float(data.qpos[int(model.jnt_qposadr[pan_id])])
                head_tilt_peak = max(head_tilt_peak, abs(tilt))
                body_rot = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
                _yaw, pitch, roll = rc.imu_from_body(body_rot)
                cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
                fwd = body_forward_xy(data, session.bid_body)
                toes = toe_samples(session, cid)
                reach.append((
                    now,
                    next(row.offset_m for row in toes if row.side == "L"),
                    next(row.offset_m for row in toes if row.side == "R"),
                ))
                step_off, step_side, _when = _high_water(reach)
                body = rc.BodyFrame((float(data.qpos[0]), float(data.qpos[1])), (float(fwd[0]), float(fwd[1])))
                pose = rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2])))
                yaw_rate = float(session.bus.applied_yaw_rate)
                seen_names = set()
                best: tuple[float, str] | None = None
                frame_bits: list[str] = []
                for name, point in _leg_floor_centers(model, data):
                    pix = rc.project_point(
                        point,
                        cam,
                        rc.camera_rotation_from_imu(_yaw, pitch, roll, tilt, pan),
                    )
                    if pix is None:
                        continue
                    u, v = pix
                    if not (0.0 <= u < rc.WIDTH and 0.0 <= v < rc.HEIGHT):
                        continue
                    est = rc.estimate_hazard(
                        u,
                        v,
                        cam=pose,
                        body=body,
                        imu_roll_rad=roll,
                        imu_pitch_rad=pitch,
                        head_tilt_rad=tilt,
                        yaw_rate=yaw_rate,
                        step_off_m=step_off,
                        hazard_pad_m=pad,
                        head_pan_rad=pan,
                    )
                    if est is None:
                        continue
                    seen_names.add(name)
                    gt = float(math.hypot(point[0] - cam[0], point[1] - cam[1]))
                    short = name.rsplit("_", 1)[-1]
                    frame_bits.append(
                        f"{short}:{'in' if est.in_corridor else 'out'} "
                        f"gap={est.toe_gap_m:.3f} side={est.sideways_m:+.3f}"
                    )
                    if est.in_corridor:
                        in_corridor_names.add(name)
                    if "leg_0" in name and abs(est.sideways_m) < leg0_side_abs:
                        leg0_side_abs = abs(est.sideways_m)
                        leg0_closest = (
                            f"t={now:.3f} side={est.sideways_m:+.3f} "
                            f"gap={est.toe_gap_m:.3f} "
                            f"{'in' if est.in_corridor else 'out'}"
                        )
                    latched[name] = rc.DeadReckonState(
                        est.forward_m, est.sideways_m, est.in_corridor,
                        body.origin_xy_m, body.forward_xy, now,
                    )
                    if est.in_corridor and est.toe_gap_m <= gate:
                        if best is None or est.toe_gap_m < best[0]:
                            best = (
                                est.toe_gap_m,
                                f"in_corridor {name} toe_gap={est.toe_gap_m:.3f} "
                                f"eye={est.eye_range:.3f} gt_eye={gt:.3f} "
                                f"side={est.sideways_m:+.3f} step_off={step_off:+.3f} "
                                f"{step_side} pad={pad:.3f}",
                            )
                    elif (not est.in_corridor) and est.toe_gap_m <= gate and false_early == "":
                        false_early = (
                            f"{name} toe_gap={est.toe_gap_m:.3f} eye={est.eye_range:.3f} "
                            f"gt_eye={gt:.3f} side={est.sideways_m:+.3f}"
                        )
                for name, state in list(latched.items()):
                    if name in seen_names or not state.in_corridor:
                        continue
                    gap = rc.dead_reckon_gap(
                        state, body.origin_xy_m, step_off, pad,
                    )
                    if gap <= gate and (best is None or gap < best[0]):
                        best = (
                            gap,
                            f"dead_reckon {name} toe_gap={gap:.3f} "
                            f"step_off={step_off:+.3f} {step_side} pad={pad:.3f}",
                        )
                if best is not None:
                    stop_frame = " ".join(frame_bits)
                    session.bus.stop(now)
                    issued = now
                    stop_mark["t"] = now
                    reason = best[1]
            if issued is None:
                driver.publish(session.bus, now)
            session.step()
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

    stop_worst = max(stop_peaks.items(), key=lambda kv: abs(kv[1][0])) if stop_peaks else None
    stop_over = [
        (n, f, t) for n, (f, t) in stop_peaks.items()
        if abs(f) > sw.lipm_gait.LEG_STOP_NM + 1e-3
    ]
    return {
        "plant": sw._md5(sw.PLANT_XML),
        "t_detect": t_detect,
        "d_min": gate,
        "issued": issued,
        "reason": reason,
        "false_early": false_early,
        "contact": contact,
        "t_stop": _settle(com_hist, issued),
        "min_up_z": session.min_up_z,
        "min_foot_leg": None if min_foot_leg > 10 else min_foot_leg,
        "stop_worst": None if stop_worst is None else (stop_worst[0], stop_worst[1][0], stop_worst[1][1]),
        "stop_over": len(stop_over),
        "step_off": _high_water(reach),
        "head_tilt_peak": head_tilt_peak,
        "yaw": math.degrees(session.yaw()),
        "pad": pad,
        "in_corridor": sorted(in_corridor_names),
        "stop_frame": stop_frame,
        "leg0_closest": leg0_closest,
    }


# #69 match window. Used only to name a geom in the bias log. The stop
# does not read a projected floor point, and it does not read this name.
FINDER_MATCH_PX = 48.0
# Association radius for the shortest-gap latch. Frozen before the
# run. A new cue joins a saved track only when the floor positions,
# after this tick's x/y/yaw re-read, are within this distance. Sim
# leg names are not a key. The kit has no leg ids.
TRACK_JOIN_M = 0.035
# Long-read stress. Not a pad and not the 3–5 cm slip buffer.
# The arm is the first frame whose shortest in-corridor gap is at
# or under d_min + LONG_ARM_M. The next frame adds LONG_READ_M to
# the toe gap of a cue that joins that floor point. One frame, then
# the finder is normal again.
LONG_READ_M = 0.10
LONG_ARM_M = 0.05


@dataclass
class _GapTrack:
    """World floor point. The gap is recomputed from body x, y, and yaw.

    No leg name. Association is the floor position against TRACK_JOIN_M.
    """

    hit_xy: tuple[float, float]
    gap_m: float
    side_m: float
    t_s: float


def _pose_gap(
    hit_xy: tuple[float, float],
    cam_xy: tuple[float, float],
    body: rc.BodyFrame,
    yaw_rate: float,
    step_off_m: float,
) -> tuple[float, float, float, bool]:
    """Toe gap of a fixed floor point in the current body pose.

    Forward is from the camera. Sideways is from the body origin.
    Both use the current heading, so yaw is in the reading. This is
    not a forward-only subtract.
    """
    fx, fy = body.forward_xy
    norm = math.hypot(fx, fy)
    if norm < 1e-9:
        return (1e9, 0.0, 0.0, False)
    forward_xy = (fx / norm, fy / norm)
    left_xy = rc.left_from_forward(forward_xy)
    dx = hit_xy[0] - cam_xy[0]
    dy = hit_xy[1] - cam_xy[1]
    forward = dx * forward_xy[0] + dy * forward_xy[1]
    ox, oy = body.origin_xy_m
    sideways = (hit_xy[0] - ox) * left_xy[0] + (hit_xy[1] - oy) * left_xy[1]
    inside = rc.in_foot_corridor(sideways, rc.corridor_edges(yaw_rate), rc.HAZARD_PAD_M)
    toe_gap = forward - rc.HAZARD_PAD_M - step_off_m
    return (toe_gap, forward, sideways, inside)
# Sim-projection latch times on this same walk, pad 0.020. The finder
# report compares against these. They are not a second gate.
SIM_LATCH_S = {0.0: 5.904, 0.033: 5.888, 0.100: 5.856}
# Cue-only finder latch on this walk, before the shortest-gap rule.
# Report comparison only. Not a second gate.
CUE_LATCH_S = {0.0: 5.864, 0.033: 5.848, 0.100: 5.736}


def walk_finder(
    t_detect: float,
    t_end: float = 9.0,
    long_read_m: float = 0.0,
    scene: Path | None = None,
    yaw_rate: float | None = None,
) -> dict[str, object]:
    """Day-1 stop on the shortest in-corridor finder gap. Pad stays 0.020.

    A world floor point is kept per hit. A new frame can only move that
    point closer. Each tick re-reads it with body x, y, and yaw, and
    drops it when it leaves the corridor. A new cue joins a saved point
    only inside TRACK_JOIN_M. ``too_close`` still stops. A stool-leg
    name is written into the report after the stop, from the saved
    floor point, and is not used to keep or clear a track.

    ``long_read_m`` is the one-frame stress. Zero leaves the latch
    alone. A positive value adds that many metres to the toe gap of
    the floor point that first reached d_min + 0.05, on the next
    frame only. The match is the floor position, not a leg name.

    ``scene`` and ``yaw_rate`` select the room and the commanded yaw.
    The default is the kitchen walk at −0.25. The stop rule does not
    change with the room.
    """
    scene_path = SCENE if scene is None else scene
    yaw_cmd = -sw.YAW_RATE_CAP if yaw_rate is None else yaw_rate
    if not scene_path.is_file():
        raise SystemExit(f"missing scene {scene_path}")
    if abs(rc.HAZARD_PAD_M - 0.020) > 1e-12:
        raise SystemExit(f"pad moved to {rc.HAZARD_PAD_M}")
    session = sw.SteerSession(video=False, scene_xml=scene_path, lipm=sw.locked_kit_config())
    model = session.model
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    pan_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_pan")
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    peaks: dict[str, tuple[float, float]] = {}
    stop_peaks: dict[str, tuple[float, float]] = {}
    real = mj.mj_step
    contact: tuple[float, str, str, float] | None = None
    stop_mark: dict[str, float | None] = {"t": None}
    com_hist: list[tuple[float, float, float, float]] = []
    prev_com: np.ndarray | None = None
    prev_t: float | None = None

    def hook(m: mj.MjModel, d: mj.MjData) -> None:
        nonlocal contact, prev_com, prev_t
        real(m, d)
        t_now = float(d.time)
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
        sw.DemoSegment(t_end, "vel", sw.VX_FWD_CAP, yaw_cmd, "turn"),
    ))
    gate = rc.d_min(t_detect)
    reach: list[tuple[float, float, float]] = []
    issued: float | None = None
    reason = ""
    path = ""
    matched = ""
    too_close = False
    false_early = ""
    min_foot_leg = 1e9
    head_tilt_peak = 0.0
    # finder eye − true camera-to-floor range. Positive is a long read.
    bias_n = 0
    bias_long = 0
    bias_max = -1e9
    bias_min = 1e9
    bias_long_note = ""
    in_n = 0
    in_long = 0
    in_max = -1e9
    in_min = 1e9
    in_long_note = ""
    delay_n = 0
    delay_gt = 1e9
    delay_note = ""
    fire_eye_bias: float | None = None
    fire_gap_bias: float | None = None
    fire_true: float | None = None
    cross: dict[float, str] = {}
    cross_levels = (0.0, 0.033, 0.100)
    tracks: list[_GapTrack] = []
    arm_xy: tuple[float, float] | None = None
    arm_gap: float | None = None
    arm_t: float | None = None
    inject_next = False
    inject_note = ""
    try:
        while float(session.data.time) < t_end - 1e-9:
            now = float(session.data.time)
            if issued is None and now >= 1.0 - 1e-9:
                inject_this = inject_next
                inject_next = False
                data = session.data
                tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
                pan = 0.0 if pan_id < 0 else float(data.qpos[int(model.jnt_qposadr[pan_id])])
                head_tilt_peak = max(head_tilt_peak, abs(tilt))
                body_rot = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
                yaw, pitch, roll = rc.imu_from_body(body_rot)
                cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
                fwd = body_forward_xy(data, session.bid_body)
                toes = toe_samples(session, cid)
                reach.append((
                    now,
                    next(row.offset_m for row in toes if row.side == "L"),
                    next(row.offset_m for row in toes if row.side == "R"),
                ))
                step_off, step_side, _when = _high_water(reach)
                body = rc.BodyFrame((float(data.qpos[0]), float(data.qpos[1])), (float(fwd[0]), float(fwd[1])))
                pose = rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2])))
                yaw_rate = float(session.bus.applied_yaw_rate)
                rot = rc.camera_rotation_from_imu(yaw, pitch, roll, tilt, pan)
                cam_xy = (float(cam[0]), float(cam[1]))
                alive: list[_GapTrack] = []
                for held in tracks:
                    gap_now, _forward, side_now, inside = _pose_gap(
                        held.hit_xy, cam_xy, body, yaw_rate, step_off,
                    )
                    if not inside:
                        continue
                    held.gap_m = gap_now
                    held.side_m = side_now
                    alive.append(held)
                tracks = alive
                renderer.disable_segmentation_rendering()
                renderer.disable_depth_rendering()
                renderer.update_scene(data, camera="kit_cam")
                rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
                cues = hf.find_hazard_cues(rgb)
                primary = cues[0] if cues else None

                def nearest(
                    u: float, v: float,
                ) -> tuple[str, float, float | None, float | None, float | None]:
                    best_px: tuple[str, float, float, float | None, float | None] | None = None
                    for name, point in _leg_floor_centers(model, data):
                        pix = rc.project_point(point, cam, rot)
                        if pix is None:
                            continue
                        dist = math.hypot(pix[0] - u, pix[1] - v)
                        true_m = float(math.hypot(point[0] - cam[0], point[1] - cam[1]))
                        gt = rc.estimate_hazard(
                            pix[0], pix[1],
                            cam=pose, body=body,
                            imu_roll_rad=roll, imu_pitch_rad=pitch,
                            head_tilt_rad=tilt, yaw_rate=yaw_rate,
                            step_off_m=step_off, hazard_pad_m=rc.HAZARD_PAD_M,
                            head_pan_rad=pan,
                        )
                        eye = None if gt is None else gt.eye_range
                        gap = None if gt is None else gt.toe_gap_m
                        if best_px is None or dist < best_px[1]:
                            best_px = (name, dist, true_m, eye, gap)
                    if best_px is None:
                        return ("none", 1e9, None, None, None)
                    return best_px

                def track(est: rc.HazardEstimate, name: str, dist: float, true_m: float | None, gt_gap: float | None) -> tuple[float | None, float | None]:
                    nonlocal bias_n, bias_long, bias_max, bias_min, bias_long_note
                    nonlocal in_n, in_long, in_max, in_min, in_long_note
                    nonlocal delay_n, delay_gt, delay_note
                    if dist > FINDER_MATCH_PX or true_m is None:
                        return (None, None)
                    eye_bias = est.eye_range - true_m
                    gap_bias = None if gt_gap is None else est.toe_gap_m - gt_gap
                    note = (
                        f"t={now:.3f} {name} eye={est.eye_range:.3f} true={true_m:.3f} "
                        f"eye_bias={eye_bias:+.3f} toe_gap={est.toe_gap_m:.3f} gt_gap={gt_gap} "
                        f"gap_bias={gap_bias} side={est.sideways_m:+.3f} "
                        f"{'in' if est.in_corridor else 'out'}"
                    )
                    bias_n += 1
                    bias_max = max(bias_max, eye_bias)
                    bias_min = min(bias_min, eye_bias)
                    if eye_bias > 1e-3:
                        bias_long += 1
                        if bias_long_note == "" or eye_bias > bias_max - 1e-9:
                            bias_long_note = note
                    if est.in_corridor:
                        in_n += 1
                        in_max = max(in_max, eye_bias)
                        in_min = min(in_min, eye_bias)
                        if eye_bias > 1e-3:
                            in_long += 1
                            if in_long_note == "" or eye_bias >= in_max - 1e-12:
                                in_long_note = note
                        if (
                            gt_gap is not None
                            and gt_gap <= rc.d_min(0.0)
                            and est.toe_gap_m > rc.d_min(0.0)
                        ):
                            delay_n += 1
                            if gt_gap < delay_gt:
                                delay_gt = gt_gap
                                delay_note = note
                    return (eye_bias, gap_bias)

                clipped: hf.HazardCue | None = None
                for cue in cues:
                    if cue.too_close and clipped is None:
                        clipped = cue
                    if cue.u is None or cue.v is None or cue.too_close:
                        continue
                    est = rc.estimate_hazard(
                        cue.u, cue.v,
                        cam=pose, body=body,
                        imu_roll_rad=roll, imu_pitch_rad=pitch,
                        head_tilt_rad=tilt, yaw_rate=yaw_rate,
                        step_off_m=step_off, hazard_pad_m=rc.HAZARD_PAD_M,
                        head_pan_rad=pan,
                    )
                    if est is None:
                        continue
                    name, dist, true_m, gt_eye, gt_gap = nearest(cue.u, cue.v)
                    eye_bias, gap_bias = track(est, name, dist, true_m, gt_gap)
                    if est.in_corridor:
                        hit = est.hit_xy_m
                        offer_gap = est.toe_gap_m
                        injected = False
                        apart_arm = 1e9
                        if inject_this and arm_xy is not None and long_read_m > 0.0:
                            apart_arm = math.hypot(hit[0] - arm_xy[0], hit[1] - arm_xy[1])
                            if apart_arm <= TRACK_JOIN_M:
                                offer_gap = est.toe_gap_m + long_read_m
                                injected = True
                        joined: _GapTrack | None = None
                        joined_d = TRACK_JOIN_M
                        for held in tracks:
                            apart = math.hypot(hit[0] - held.hit_xy[0], hit[1] - held.hit_xy[1])
                            if apart <= joined_d:
                                joined_d = apart
                                joined = held
                        if joined is None:
                            outcome = "new_track"
                            tracks.append(_GapTrack(hit, offer_gap, est.sideways_m, now))
                        elif offer_gap < joined.gap_m:
                            outcome = f"replaced {joined.gap_m:.3f}"
                            joined.hit_xy = hit
                            joined.gap_m = offer_gap
                            joined.side_m = est.sideways_m
                            joined.t_s = now
                        else:
                            outcome = f"kept={joined.gap_m:.3f}"
                        if injected and inject_note == "":
                            inject_note = (
                                f"arm_t={arm_t:.3f} arm_gap={arm_gap:.3f} "
                                f"inject_t={now:.3f} real={est.toe_gap_m:.3f} "
                                f"offered={offer_gap:.3f} {outcome} "
                                f"join={apart_arm:.3f}"
                            )
                    elif (not est.in_corridor) and est.toe_gap_m <= gate and false_early == "":
                        false_early = (
                            f"{name} px={dist:.1f} toe_gap={est.toe_gap_m:.3f} "
                            f"gt_gap={gt_gap} eye={est.eye_range:.3f} true={true_m} "
                            f"eye_bias={eye_bias} gap_bias={gap_bias} side={est.sideways_m:+.3f}"
                        )
                chosen: _GapTrack | None = None
                for held in tracks:
                    if chosen is None or held.gap_m < chosen.gap_m:
                        chosen = held
                if chosen is not None:
                    for level in cross_levels:
                        if level in cross:
                            continue
                        if chosen.gap_m <= rc.d_min(level):
                            cross[level] = (
                                f"t={now:.3f} gap={chosen.gap_m:.3f} "
                                f"side={chosen.side_m:+.3f} born={chosen.t_s:.3f} "
                                f"tracks={len(tracks)}"
                            )
                if inject_this and inject_note == "":
                    inject_note = (
                        f"arm_t={arm_t:.3f} arm_gap={arm_gap:.3f} "
                        f"inject_t={now:.3f} no_join"
                    )
                if (
                    long_read_m > 0.0
                    and arm_xy is None
                    and chosen is not None
                    and chosen.gap_m <= gate + LONG_ARM_M
                ):
                    arm_xy = chosen.hit_xy
                    arm_gap = chosen.gap_m
                    arm_t = now
                    inject_next = True
                if (
                    long_read_m > 0.0
                    and inject_note == ""
                    and arm_t is not None
                    and abs(arm_t - now) < 1e-6
                    and chosen is not None
                    and chosen.gap_m <= gate
                ):
                    inject_note = (
                        f"arm_t={arm_t:.3f} arm_gap={arm_gap:.3f} same_frame_as_stop"
                    )
                if chosen is not None and chosen.gap_m <= gate:
                    def report_leg(hit_xy: tuple[float, float]) -> tuple[str, float]:
                        best_name = "none"
                        best = 1e9
                        for leg, point in _prop_floor_centers(model, data):
                            apart = math.hypot(hit_xy[0] - point[0], hit_xy[1] - point[1])
                            if apart < best:
                                best = apart
                                best_name = leg
                        return best_name, best

                    leg_name, leg_dist = report_leg(chosen.hit_xy)
                    true_m = None
                    gt_gap = None
                    for leg, point in _prop_floor_centers(model, data):
                        if leg != leg_name:
                            continue
                        true_m = float(math.hypot(point[0] - cam[0], point[1] - cam[1]))
                        gt_gap = _pose_gap(
                            (float(point[0]), float(point[1])),
                            cam_xy, body, yaw_rate, step_off,
                        )[0]
                        break
                    reports = []
                    for held in sorted(tracks, key=lambda row: row.gap_m):
                        rname, rdist = report_leg(held.hit_xy)
                        reports.append(
                            f"{rname}@{rdist:.3f} gap={held.gap_m:.3f} born={held.t_s:.3f}"
                        )
                    finder_eye = math.hypot(chosen.hit_xy[0] - cam[0], chosen.hit_xy[1] - cam[1])
                    fire_eye_bias = None if true_m is None else finder_eye - true_m
                    fire_gap_bias = None if gt_gap is None else chosen.gap_m - gt_gap
                    fire_true = true_m
                    session.bus.stop(now)
                    issued = now
                    stop_mark["t"] = now
                    path = "shortest"
                    reason = (
                        f"shortest report={leg_name} world={leg_dist:.3f} "
                        f"toe_gap={chosen.gap_m:.3f} gt_gap={gt_gap} "
                        f"eye={finder_eye:.3f} true={true_m} "
                        f"eye_bias={fire_eye_bias} gap_bias={fire_gap_bias} "
                        f"side={chosen.side_m:+.3f} born={chosen.t_s:.3f} "
                        f"assoc={TRACK_JOIN_M:.3f} tracks={len(tracks)} "
                        f"[{'; '.join(reports)}] "
                        f"step_off={step_off:+.3f} {step_side} "
                        f"pad={rc.HAZARD_PAD_M:.3f} long={long_read_m:.3f} "
                        f"inject=[{inject_note or 'none'}]"
                    )
                    too_close = clipped is not None
                    matched = reason
                elif clipped is not None:
                    u = float(clipped.column) + 0.5
                    v = float(clipped.contact_row) + 0.5
                    name, dist, true_m, gt_eye, gt_gap = nearest(u, v)
                    named = name if dist <= FINDER_MATCH_PX else f"unmatched {name}"
                    session.bus.stop(now)
                    issued = now
                    stop_mark["t"] = now
                    path = "too_close"
                    too_close = True
                    reason = (
                        f"too_close {named} px={dist:.1f} col={clipped.column} "
                        f"row={clipped.contact_row} true={true_m} gt_eye={gt_eye} "
                        f"gt_gap={gt_gap} step_off={step_off:+.3f} {step_side}"
                    )
                    matched = reason
                    if primary is not None and primary is not clipped:
                        reason += (
                            f" primary_row={primary.contact_row} "
                            f"primary_too_close={primary.too_close}"
                        )
            if issued is None:
                driver.publish(session.bus, now)
            session.step()
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
        renderer.close()

    stop_worst = max(stop_peaks.items(), key=lambda kv: abs(kv[1][0])) if stop_peaks else None
    stop_over = [
        (n, f, t) for n, (f, t) in stop_peaks.items()
        if abs(f) > sw.lipm_gait.LEG_STOP_NM + 1e-3
    ]
    sim_t = SIM_LATCH_S.get(t_detect)
    late = None if issued is None or sim_t is None else issued - sim_t
    cue_t = CUE_LATCH_S.get(t_detect)
    late_cue = None if issued is None or cue_t is None else issued - cue_t
    walk_worst = max(peaks.items(), key=lambda kv: abs(kv[1][0])) if peaks else None
    return {
        "plant": sw._md5(sw.PLANT_XML),
        "scene": scene_path.name,
        "yaw_cmd": yaw_cmd,
        "t_detect": t_detect,
        "d_min": gate,
        "issued": issued,
        "path": path,
        "reason": reason,
        "matched": matched,
        "too_close": too_close,
        "false_early": false_early,
        "contact": contact,
        "t_stop": _settle(com_hist, issued),
        "min_up_z": session.min_up_z,
        "min_foot_leg": None if min_foot_leg > 10 else min_foot_leg,
        "stop_worst": None if stop_worst is None else (stop_worst[0], stop_worst[1][0], stop_worst[1][1]),
        "walk_worst": None if walk_worst is None else (walk_worst[0], walk_worst[1][0], walk_worst[1][1]),
        "stop_over": len(stop_over),
        "step_off": _high_water(reach),
        "head_tilt_peak": head_tilt_peak,
        "yaw": math.degrees(session.yaw()),
        "late_vs_sim_s": late,
        "late_vs_cue_s": late_cue,
        "long_read": long_read_m,
        "arm_t": arm_t,
        "arm_gap": arm_gap,
        "inject_note": inject_note,
        "fire_eye_bias": fire_eye_bias,
        "fire_gap_bias": fire_gap_bias,
        "fire_true": fire_true,
        "bias_n": bias_n,
        "bias_long": bias_long,
        "bias_max": None if bias_n == 0 else bias_max,
        "bias_min": None if bias_n == 0 else bias_min,
        "bias_long_note": bias_long_note,
        "in_n": in_n,
        "in_long": in_long,
        "in_max": None if in_n == 0 else in_max,
        "in_min": None if in_n == 0 else in_min,
        "in_long_note": in_long_note,
        "delay_n": delay_n,
        "delay_note": delay_note,
        "cross": {f"{key:.3f}": text for key, text in cross.items()},
    }


def _print_finder(result: dict[str, object]) -> None:
    step = result["step_off"]
    assert isinstance(step, tuple)
    print(
        f"plant {result['plant']} mode=finder scene={result['scene']} "
        f"yaw_cmd={result['yaw_cmd']:+.3f} T_detect={result['t_detect']} "
        f"d_min={result['d_min']:.4f} buffer=0 pad={rc.HAZARD_PAD_M:.3f} "
        f"assoc={TRACK_JOIN_M:.3f} "
        f"step_off={step[0]:+.3f} {step[1]} t={step[2]:.3f} "
        f"path={result['path']} too_close={result['too_close']} "
        f"issued={result['issued']} late_vs_sim_s={result['late_vs_sim_s']} "
        f"late_vs_cue_s={result['late_vs_cue_s']} long={result['long_read']} "
        f"arm_t={result['arm_t']} arm_gap={result['arm_gap']} "
        f"inject=[{result['inject_note'] or 'none'}] "
        f"reason={result['reason']} false_early={result['false_early'] or 'none'} "
        f"contact={result['contact']} T_stop_run={result['t_stop']} "
        f"min_up_z={result['min_up_z']:.3f} min_foot_leg={result['min_foot_leg']} "
        f"yaw={result['yaw']:+.1f} head_tilt_peak={result['head_tilt_peak']:.4f} "
        f"stop_worst={result['stop_worst']} walk_worst={result['walk_worst']} "
        f"stop_over={result['stop_over']} "
        f"fire_eye_bias={result['fire_eye_bias']} fire_gap_bias={result['fire_gap_bias']} "
        f"fire_true={result['fire_true']} "
        f"bias n={result['bias_n']} long={result['bias_long']} "
        f"max={result['bias_max']} min={result['bias_min']} "
        f"in_n={result['in_n']} in_long={result['in_long']} "
        f"in_max={result['in_max']} in_min={result['in_min']} "
        f"in_long_note=[{result['in_long_note']}] long_note=[{result['bias_long_note']}] "
        f"delay_n={result['delay_n']} delay_note=[{result['delay_note']}] "
        f"cross={result['cross']}"
    )


def _print_latch(result: dict[str, object]) -> None:
    step = result["step_off"]
    assert isinstance(step, tuple)
    print(
        f"plant {result['plant']} mode=latch T_detect={result['t_detect']} "
        f"d_min={result['d_min']:.4f} buffer=0 pad={result['pad']:.3f} "
        f"step_off={step[0]:+.3f} {step[1]} t={step[2]:.3f} "
        f"issued={result['issued']} reason={result['reason']} "
        f"false_early={result['false_early'] or 'none'} "
        f"contact={result['contact']} T_stop_run={result['t_stop']} "
        f"min_up_z={result['min_up_z']:.3f} min_foot_leg={result['min_foot_leg']} "
        f"yaw={result['yaw']:+.1f} head_tilt_peak={result['head_tilt_peak']:.4f} "
        f"stop_worst={result['stop_worst']} stop_over={result['stop_over']} "
        f"in_corridor={result['in_corridor']} leg0_closest=[{result['leg0_closest']}] "
        f"stop_frame=[{result['stop_frame']}]"
    )


def walk_open(scene: Path, yaw_rate: float, t_end: float = 9.0) -> dict[str, object]:
    """Unstopped walk. Names the first prop contact. Not a second gate."""
    if not scene.is_file():
        raise SystemExit(f"missing scene {scene}")
    session = sw.SteerSession(video=False, scene_xml=scene, lipm=sw.locked_kit_config())
    model = session.model
    real = mj.mj_step
    contact: tuple[float, str, str, float] | None = None

    def hook(m: mj.MjModel, d: mj.MjData) -> None:
        nonlocal contact
        real(m, d)
        if contact is not None:
            return
        hit = _prop_hit(m, d)
        if hit is not None:
            contact = (float(d.time), hit[0], hit[1], hit[2])

    mj.mj_step = hook
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t_end, "vel", sw.VX_FWD_CAP, yaw_rate, "open"),
    ))
    props = 0
    for gid in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith("col_") and not _is_wall(name):
            props += 1
    try:
        while float(session.data.time) < t_end - 1e-9:
            now = float(session.data.time)
            if contact is None:
                driver.publish(session.bus, now)
            session.step()
            if contact is not None:
                break
    finally:
        mj.mj_step = real
    return {
        "props": props,
        "contact": contact,
        "x": float(session.data.qpos[0]),
        "y": float(session.data.qpos[1]),
        "yaw": math.degrees(session.yaw()),
        "min_up_z": session.min_up_z,
        "t": float(session.data.time),
    }


def _rooms() -> None:
    """Same Day-1 finder chain on every furnished room and approach.

    T_detect is 0, the floor of d_min. The stop rule is unchanged.
    """
    hf.self_check()
    rc.self_check()
    want_room = sys.argv[2] if len(sys.argv) > 2 else ""
    want_yaw = sys.argv[3] if len(sys.argv) > 3 else ""
    rooms = ("kitchen", "bathroom", "living", "bedroom", "entrance")
    bouts = (
        ("straight", 0.0),
        ("left", -sw.YAW_RATE_CAP),
        ("right", sw.YAW_RATE_CAP),
    )
    root = Path("/tmp/m62/mujoco")
    for room in rooms:
        if want_room and room != want_room:
            continue
        scene = root / f"room_{room}.xml"
        for label, yaw in bouts:
            if want_yaw and label != want_yaw:
                continue
            opened = walk_open(scene, yaw)
            print(
                f"OPEN room={room} approach={label} yaw_cmd={yaw:+.3f} "
                f"props={opened['props']} contact={opened['contact']} "
                f"t={opened['t']:.3f} xy={opened['x']:+.3f},{opened['y']:+.3f} "
                f"yaw={opened['yaw']:+.1f} min_up_z={opened['min_up_z']:.3f}",
                flush=True,
            )
            result = walk_finder(0.0, scene=scene, yaw_rate=yaw)
            _print_finder(result)
            reason = str(result["reason"])
            report = "none"
            if "report=" in reason:
                report = reason.split("report=", 1)[1].split(" ", 1)[0]
            open_hit = opened["contact"]
            open_name = None if open_hit is None else open_hit[2]
            print(
                f"BOUT room={room} approach={label} stop={result['issued']} "
                f"path={result['path']} latch={report} "
                f"finder_contact={result['contact']} open_contact={open_name} "
                f"stop_worst={result['stop_worst']} stop_over={result['stop_over']} "
                f"walk_worst={result['walk_worst']} too_close={result['too_close']}",
                flush=True,
            )


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "gate"
    if mode == "rooms-open":
        want_room = sys.argv[2] if len(sys.argv) > 2 else ""
        want_yaw = sys.argv[3] if len(sys.argv) > 3 else ""
        root = Path("/tmp/m62/mujoco")
        bouts = (
            ("straight", 0.0),
            ("left", -sw.YAW_RATE_CAP),
            ("right", sw.YAW_RATE_CAP),
        )
        for room in ("kitchen", "bathroom", "living", "bedroom", "entrance"):
            if want_room and room != want_room:
                continue
            scene = root / f"room_{room}.xml"
            for label, yaw in bouts:
                if want_yaw and label != want_yaw:
                    continue
                opened = walk_open(scene, yaw)
                print(
                    f"OPEN room={room} approach={label} yaw_cmd={yaw:+.3f} "
                    f"props={opened['props']} contact={opened['contact']} "
                    f"t={opened['t']:.3f} xy={opened['x']:+.3f},{opened['y']:+.3f} "
                    f"yaw={opened['yaw']:+.1f} min_up_z={opened['min_up_z']:.3f}",
                    flush=True,
                )
        return
    if mode == "rooms":
        _rooms()
        return
    if mode == "finder":
        hf.self_check()
        rc.self_check()
        for t_detect in (0.0, 0.033, 0.100):
            _print_finder(walk_finder(t_detect))
        return
    if mode == "finder-long":
        # One injected long frame. Not the shipped latch and not a buffer.
        hf.self_check()
        rc.self_check()
        for t_detect in (0.0, 0.033, 0.100):
            _print_finder(walk_finder(t_detect, long_read_m=LONG_READ_M))
        return
    if mode == "latch":
        rc.self_check()
        for t_detect in (0.0, 0.033, 0.100):
            _print_latch(walk_latch(t_detect))
        return
    if mode == "latch-pad":
        # Frozen for this stress only. Not a collider radius and not the
        # shipped 20 mm stand-in. Does not size the 3–5 cm buffer.
        pad = 0.035 if len(sys.argv) < 3 else float(sys.argv[2])
        rc.self_check()
        for t_detect in (0.0, 0.033, 0.100):
            _print_latch(walk_latch(t_detect, hazard_pad_m=pad))
        return
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
