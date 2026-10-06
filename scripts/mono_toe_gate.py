#!/usr/bin/env python3
"""Toe-gap range gate. Day-1 stop only. Plant file is not edited.

The mono range is the ground distance from kit_cam implied by one pixel
row, the live camera height and position, and the live world pitch.
World pitch is the torso IMU pitch plus head_tilt at that frame.
Height is kit_cam world z. The floor plane is z = 0.

The compare is the leading-toe gap, not the eye gap. On this plant the
sole is not ahead of the eye at stand: both feet measure −0.016 m
along body forward (the lean puts the eye ahead of the sole). At
t = 5.90 s the further-ahead sole is +0.006 m and the swing sole is
−0.037 m. The gate uses the further-ahead sole. Camera height at that
sample is 0.332 m, not the 0.335 m stand figure.

    cam_z = kit_cam world z at the frame (floor plane z = 0)
    eye_range = cam_z / tan(depression(row, IMU + head_tilt))
    near_edge = cam_z / tan(depression(last row, IMU + head_tilt))
    toe_offset = (leading sole bottom − cam_xy) · body_forward
    toe_gap = eye_range − toe_offset
    near_toe = near_edge − toe_offset
    d_min = 0.150 * (T_detect + T_stop)

near_edge is the horizontal distance from the point under the eye.
The compare is still the toe gap, so near_toe and toe_gap both
subtract toe_offset before they are checked against d_min. A stool-leg
row in the bottom 32 pixels also stops when toe_gap is within
(eye_range − near_edge) of d_min. That is the same check as
near_toe <= d_min, and it is only armed when the row is a stool leg.
The robot's own feet are not a contact. After that band leaves the
frame, dead-reckon subtracts body travel along the latched forward.

Height is not frozen at 0.335 m. That figure is the stand eye height
on this plant. A −10° head_tilt walk is not enabled: its near edge is
inside the toe zone, so the swing foot sits in the bottom of the
frame, and a wood-color rule on that frame can see the feet. The
stills that keep the room name are a stand plus a head hold, not a
walk. Restoring the head for a room ask is a session joint, not a
Day-1 bus key.

T_stop is 0.842 s. The empty-plant grid peaked at 0.832 s. The
live-height kitchen stop settled in 0.842 s, and that is the longer
one. T_detect stays a parameter. 0.1263 m at T_detect = 0 is not a
locked margin.
The pixel row in this probe is the sim projection of a group-3
stool-leg bottom. It is not an RGB finder. The head is not tilted.
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
# Bottom band. A contact here is about to leave the floor of the frame.
NEAR_PX = 32
LEG_TOKS = ("hip_", "knee", "ank_")
SPEED_EPS = 0.02
HOLD_S = 0.20


def d_min(t_detect_s: float, t_stop_s: float = T_STOP_S) -> float:
    return V_MPS * (float(t_detect_s) + float(t_stop_s))


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


def walk(t_detect: float, t_end: float = 9.0, arm: bool = True) -> dict[str, object]:
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
    gate = d_min(t_detect)
    try:
        while float(session.data.time) < t_end - 1e-9:
            now = float(session.data.time)
            if issued is None and arm and now >= 1.0 - 1e-9:
                choice = _decide(session, cid, jid, latched, gate)
                if choice is not None:
                    session.bus.stop(now)
                    issued = now
                    issued_xy = (float(session.data.qpos[0]), float(session.data.qpos[1]))
                    stop_mark["t"] = now
                    reason = choice
                    logs.append(f"STOP t={now:.3f} {choice}")
                    logs.append(_pose_line(session, cid, jid))
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
        "plant": sw._md5(sw.PLANT_XML),
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
) -> str | None:
    data = session.data
    model = session.model
    tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
    pitch = torso_pitch(data, session.bid_body) + tilt
    cam_z = float(data.cam_xpos[cid][2])
    toes = toe_samples(session, cid)
    # The toe that sits furthest ahead. The swing foot is often behind the eye.
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
        gap = eye - toe.offset_m
        near = v >= (HEIGHT - NEAR_PX)
        margin = 0.0
        if near and edge is not None:
            margin = max(0.0, eye - edge)
        body_xy = (float(data.qpos[0]), float(data.qpos[1]))
        sight = Sight(
            name, v, eye, gap, body_xy, (float(fwd[0]), float(fwd[1])), near,
        )
        latched[name] = sight
        trigger = gap
        why = "toe_gap"
        if near and gap <= gate + margin and gap > gate:
            trigger = gap
            why = "near_edge"
            if best is None or trigger < best[0]:
                best = (trigger, f"{why} {name} toe_gap={gap:.3f} margin={margin:.3f} eye={eye:.3f} cam_z={cam_z:.3f} off={toe.offset_m:+.3f} {toe.side}{' swing' if toe.swing else ''}")
            continue
        if gap <= gate and (best is None or gap < best[0]):
            best = (gap, f"{why} {name} toe_gap={gap:.3f} eye={eye:.3f} cam_z={cam_z:.3f} off={toe.offset_m:+.3f} row={v:.0f} {toe.side}{' swing' if toe.swing else ''}")
    for name, sight in list(latched.items()):
        if name in seen or not sight.in_near_band:
            continue
        body_xy = (float(data.qpos[0]), float(data.qpos[1]))
        advance = float(np.dot(
            np.array(body_xy, dtype=np.float64) - np.array(sight.body_xy, dtype=np.float64),
            np.array(sight.fwd, dtype=np.float64),
        ))
        dr = sight.toe_gap_m - advance
        if dr <= gate and (best is None or dr < best[0]):
            best = (dr, f"dead_reckon {name} toe_gap={dr:.3f} advance={advance:.3f} cam_z={cam_z:.3f} off={toe.offset_m:+.3f}")
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


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "gate"
    t_detect = 0.0 if len(sys.argv) < 3 else float(sys.argv[2])
    arm = mode != "log"
    result = walk(t_detect, arm=arm)
    print(
        f"plant {result['plant']} mode={mode} T_detect={t_detect} "
        f"d_min={result['gate_m']:.4f} issued={result['issued']} reason={result['reason']} "
        f"contact={result['contact']} T_stop_run={result['t_stop']} "
        f"min_up_z={result['min_up_z']:.3f} yaw={result['yaw']:+.1f} "
        f"xy={result['xy']} issued_xy={result['issued_xy']} "
        f"min_foot_leg={result['min_foot_leg']} fault={result['fault']} "
        f"stop_worst={result['stop_worst']} stop_over={len(result['stop_over'])}"
    )
    logs = result["logs"]
    assert isinstance(logs, list)
    for line in logs:
        print(line)


if __name__ == "__main__":
    main()
