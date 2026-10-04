#!/usr/bin/env python3
"""Day-1 steerable walk on the frozen AiNex M145 plant.

Velocity only. No waypoints, goals, maps, door commands, or joint targets.

Plant (Hardware freeze, plus the approved kit_cam copy):
  mujoco/ainex_hiwonder/ainex_controls_m2_145.xml
  Foot contact box 145×86 mm (half-size 0.0725 × 0.043 m, friction 1.6).
  Toe spheres are visual (contype 0) and do not hold weight.
  Legs ±2.1 Nm (hip/knee kp 40–45, ankle kp 35). Arms/head ±0.7 Nm.
  kit_cam is a child of head_tilt_link: pos 0.050 0.019 0.007, xyaxes
  0 -1 0 0 0 1, fovy 104.82, plus a zero-mass non-contact site. No door
  geometry. This file does not load gate_f / OptC / door plants. The demo
  mp4 uses the offscreen renderer aimed at body_link, not kit_cam.

Command bus (latest command wins). Voice will call this same API later:
  stand(now)              still; vx=0, yaw_rate=0. Power-on default.
  stop(now)               same as stand; drops velocity the same tick.
  vel(vx, yaw_rate, now)  m/s and rad/s. +vx is forward (+X). +yaw_rate is
                          turn left. No vy.

AI resends vel at 10 Hz while moving. If no command arrives for 200 ms the
next tick is stop/stand. Each tick returns applied_vx, applied_yaw_rate, and
mode in {stand, move, fault}. A refused command prints one line and is not
applied.

Clamps (what we actually apply — not the raw request):
  Forward vx  ≤ VX_FWD_CAP (0.056 m/s). Lowered from 0.080: that command
                yaws off and tips near 1.2 m (support margin about −0.09 m).
                CPG amplitude is |applied_vx| / GAIT_AMP_VX (0.080), so full
                stick is amplitude 0.70 of the gait that used to be full stick
                at 0.080. Realized body speed is about 0.04 m/s, not 0.056.
                applied_vx is the clamped command; the summary's mean body vx
                and forward Δyaw are the measurements.
  Reverse vx  ≥ -VX_BACK_CAP (0.032 m/s) → amplitude 0.40 of the same CPG
                (still |vx| / 0.080), sagittal mirror of the forward gait.
                The stance-slip damper is raised to REVERSE_PLANT_KD only
                while reversing. Realized retreat is about two to three body
                lengths, then stop. Not a Gate Q pass.
  |yaw_rate|  ≤ YAW_RATE_CAP (0.25 rad/s). While walking forward this scales
                the outside step longer than the inside step and adds a
                hip-yaw bias clipped at YAW_HIP_CLIP. Heading that remains
                after stop is the measured turn.
  Deadband and slew reuse TELEOP_DEADBAND (0.08) and TELEOP_RATE_LIMIT (1.5)
  from scripts/walk_gait.py. Those constants are joint-space (rad, rad/s).
  Velocity uses the same fraction of each cap:
    |vx| < 0.08 * VX_FWD_CAP  → 0
    |yaw_rate| < 0.08 * YAW_RATE_CAP → 0
  and slews no faster than the plant clamp, which is below TELEOP_RATE_LIMIT.
  Hip-yaw offset itself slews at TELEOP_RATE_LIMIT rad/s and ignores a desired
  offset smaller than TELEOP_DEADBAND rad. stand/stop/timeout skip the slew
  and zero velocity the same tick. Forward and turn then ease into stand over
  SETTLE_BLEND_S while the stance damper stays at SETTLE_PLANT_KD for
  SETTLE_DAMPER_S (100 N/(m/s), 1.20 s). Reverse, if the cut would land in a
  phase that pitches, finishes at most one mirrored step with the command
  already at 0, then uses that same damper. With neither, some phases pitch
  the COM about 8 cm out of the foot boxes and the existing tip check latches.
  The damper is off again after that window; quiet stand does not keep it.

Gait: scripts/walk_gait_ainex.py gait_targets. Forward uses a shorter cadence
than Gate D CSF50 (that basin crawled at ~1 cm/s): T=0.55 s, hip amp 0.24 rad,
DS=0.1375 s, stance-slip damper 25 N/(m/s) instead of CSF50's 115. CP swing
and mild stance VIK stay on. Feet, friction, kp, and ±2.1 Nm are unchanged;
the legs still pin at 2.1 Nm. Balance assist stays off (it locks yaw and
fakes speed). Residual npz stays off. This is not a clean-walk or Gate E
PASS. If many legs pin, or up_z approaches a tip, applied velocity is capped
(not compounded). Tip / collapse latches mode=fault and stands.

Honesty: pure yaw (vx=0) still runs a reduced forward CPG because this plant
has no turn-in-place gait, so some +X creep is expected. Reverse mirrors hip
and ankle pitch about the stand pose and keeps swing knee flexion; clearance
and heading are not a retreat proof.

Keyboard (same bus). MuJoCo's viewer callback is press-only, so keys latch
until Space:
  W  vx = +VX_FWD_CAP, yaw unchanged
  S  vx = -VX_BACK_CAP, yaw unchanged
  A  yaw_rate = +YAW_RATE_CAP (left), vx unchanged
  D  yaw_rate = -YAW_RATE_CAP (right), vx unchanged
  Space  stop / stand, clear latches
The view loop resends the latch every control tick so the 200 ms watchdog
does not trip while a key is latched.

Run:
  MUJOCO_GL=osmesa python scripts/steer_walk.py
  MUJOCO_GL=osmesa python scripts/steer_walk.py --out previews/steer_walk_forward.mp4
  MUJOCO_GL=osmesa python scripts/steer_walk.py --no-video
  MUJOCO_GL=glfw  python scripts/steer_walk.py --view
  python scripts/steer_walk.py --self-test
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

# GL before mujoco import. --view needs a window; headless uses OSMesa.
if "--view" in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "glfw")
else:
    os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import walk_gait_ainex as wg  # noqa: E402
from walk_gait import TELEOP_DEADBAND, TELEOP_RATE_LIMIT  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
# Walk plant after kit_cam moved to 0.050 0.019 0.007. Feet, friction,
# forcerange, kp, and mass are the same as the e3feef97… insert. Pre-camera
# freeze was fc94709c84f5598d4474ecfc4bb41fdc.
PLANT_MD5 = "71b2c86d133ebc603f58b99c53e496f3"
KIT_CAM_POS = (0.050, 0.019, 0.007)
KIT_CAM_FOVY = 104.82
# xyaxes "0 -1 0 0 0 1" → camera-frame columns (x, y, z). Look is −Z = +X.
KIT_CAM_AXES = (
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
)
PREVIEWS = ROOT / "previews"

# Frozen contact box (half-size, m) and servo ranges. Checked, never written.
FOOT_HALF_X = 0.0725
FOOT_HALF_Y = 0.0430
FOOT_FRICTION = 1.6
LEG_TAU = 2.1
ARM_TAU = 0.7
SAT_FRAC = 0.98

# vx that maps to CPG amplitude 1. Full-stick forward is below this: 0.080
# yaws off and tips near Δx 1.2 m. Do not divide amplitude by VX_FWD_CAP.
GAIT_AMP_VX = 0.08
# Full-stick forward command. |applied_vx| / GAIT_AMP_VX is the CPG amplitude,
# so this cap is amplitude 0.70. Realized body speed is slower than the
# command (~0.04 m/s). Not a torque-limit increase.
VX_FWD_CAP = 0.056
# |vx| / GAIT_AMP_VX = 0.40. This command, with REVERSE_PLANT_KD, is the
# upright retreat. A larger reverse command shortens the distance before a tip.
VX_BACK_CAP = 0.032
# Stance-slip damper (N per m/s) used only while vx < 0. Forward stays at
# the value in apply_frozen_forward_gait (25). Not a forcerange change.
REVERSE_PLANT_KD = 100.0
YAW_RATE_CAP = 0.25
# Hip-yaw bias at full stick. Must clear TELEOP_DEADBAND (0.08 rad).
# 0.35 rad tips this cadence. 0.12 rad clears TELEOP_DEADBAND and, with
# TURN_STEP_ASYM, holds a walk-turn instead of a pure pelvis twist.
YAW_HIP_CLIP = 0.12
YAW_HIP_GAIN = YAW_HIP_CLIP / YAW_RATE_CAP
# Full-stick outside/inside step scale. +yaw lengthens the right step.
TURN_STEP_ASYM = 0.20
# Subtracted from both hip yaws only while turning left. The open-loop gait
# already drifts left; without this the left command stacks and tips.
TURN_LEFT_BIAS = 0.06
# vx=0 and yaw!=0: reduced forward CPG so a step exists to yaw on.
INPLACE_YAW_AMP = 0.35

# Joint-space constants, scaled into velocity units (see module docstring).
DEADBAND_VX = TELEOP_DEADBAND * VX_FWD_CAP
DEADBAND_YAW = TELEOP_DEADBAND * YAW_RATE_CAP
# Plant slew is tighter than TELEOP_RATE_LIMIT so stand→full gait is not one frame.
VX_SLEW = 0.08  # m/s^2  (0 → 0.056 cap in 0.70 s)
YAW_SLEW = 0.40  # rad/s^2

COMMAND_TIMEOUT_S = 0.200
VEL_RESEND_S = 0.10
CTRL_DT = 1.0 / wg.CTRL_HZ

# After stop, applied_vx is already 0. Forward and turn blend into stand
# over SETTLE_BLEND_S while the stance damper is held at SETTLE_PLANT_KD
# for SETTLE_DAMPER_S. Measured with the damper off: some phases pitch
# (support margin about −0.08 m) and the tip check latches. The damper is
# off again after that window. Reverse cuts near phase 0.29–0.40 and
# 0.84–0.95 pitch, so those finish at most one mirrored step first, then
# use the same damper. Clamps are unchanged.
SETTLE_BLEND_S = 0.70
REVERSE_SETTLE_BLEND_S = 0.45
SETTLE_PLANT_KD = 100.0
SETTLE_PLANT_AMP = 0.8
SETTLE_COAST_MAX_S = 0.55
# Damper stays on after the pose blend. Some reverse cuts are upright at the
# end of the blend and pitch once the wrench drops; 1.20 s covers that.
SETTLE_DAMPER_S = 1.20
# Tip / fall. up_z 0.85 is the existing upright bar; a short dip slows the
# gait, a real tip latches fault. Grace covers the drop onto the feet.
SETTLE_GRACE_S = 0.40
TIP_UP_Z = 0.72
TIP_HOLD_S = 0.12
COLLAPSE_Z = 0.14
AIRBORNE_FAULT_S = 0.20
COP_SLOW_MARGIN = 0.012  # m, COM inside support hull
COP_FAULT_MARGIN = -0.04

ModeName = Literal["stand", "move", "fault"]

KEY_W = 87
KEY_A = 65
KEY_S = 83
KEY_D = 68
KEY_SPACE = 32

_REFUSED_COMMANDS = frozenset({
    "waypoint", "waypoints", "goal", "goto", "map", "door",
    "joint", "joints", "vy", "pose", "path",
})


@dataclass(frozen=True)
class TickReport:
    """One control tick. applied_* are what the gait used, after clamps."""

    applied_vx: float
    applied_yaw_rate: float
    mode: ModeName

    def line(self) -> str:
        return (
            f"applied_vx={self.applied_vx:+.4f} "
            f"applied_yaw_rate={self.applied_yaw_rate:+.4f} "
            f"mode={self.mode}"
        )


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _slew(current: float, target: float, rate: float, dt: float) -> float:
    step = rate * dt
    return current + _clamp(target - current, -step, step)


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


class CommandBus:
    """Velocity command bus. Latest command wins. Clock is the caller's `now`."""

    def __init__(self) -> None:
        self.target_vx = 0.0
        self.target_yaw = 0.0
        self.applied_vx = 0.0
        self.applied_yaw_rate = 0.0
        self.mode: ModeName = "stand"
        self.fault = False
        self.fault_reason = ""
        self.last_cmd_time = 0.0
        self._snapped = True

    def stand(self, now: float) -> str | None:
        self._snap_zero(now)
        return None

    def stop(self, now: float) -> str | None:
        return self.stand(now)

    def vel(self, vx: float, yaw_rate: float, now: float, **extra: object) -> str | None:
        if extra:
            keys = ", ".join(sorted(str(k) for k in extra))
            return f"refused: vel accepts vx and yaw_rate only (got {keys})"
        if self.fault:
            return "refused: fault — velocity dropped; send stand"
        if not _finite(vx) or not _finite(yaw_rate):
            return "refused: non-finite vx or yaw_rate"
        vx_c = float(vx)
        yaw_c = float(yaw_rate)
        if abs(vx_c) < DEADBAND_VX:
            vx_c = 0.0
        else:
            vx_c = _clamp(vx_c, -VX_BACK_CAP, VX_FWD_CAP)
        if abs(yaw_c) < DEADBAND_YAW:
            yaw_c = 0.0
        else:
            yaw_c = _clamp(yaw_c, -YAW_RATE_CAP, YAW_RATE_CAP)
        self.last_cmd_time = now
        if vx_c == 0.0 and yaw_c == 0.0:
            self._snap_zero(now)
            return None
        self.target_vx = vx_c
        self.target_yaw = yaw_c
        self._snapped = False
        if not self.fault:
            self.mode = "move"
        return None

    def submit(self, name: str, now: float, **fields: object) -> str | None:
        if name in _REFUSED_COMMANDS:
            return f"refused: {name} is not a day-1 velocity command"
        if name == "stand":
            return self.stand(now)
        if name == "stop":
            return self.stop(now)
        if name == "vel":
            vx = fields.get("vx", 0.0)
            yaw = fields.get("yaw_rate", 0.0)
            extra = {k: v for k, v in fields.items() if k not in ("vx", "yaw_rate")}
            if not _finite(vx) or not _finite(yaw):
                return "refused: non-finite vx or yaw_rate"
            return self.vel(float(vx), float(yaw), now, **extra)
        return f"refused: unknown command {name}"

    def tick(self, now: float, dt: float) -> TickReport:
        if self.fault:
            self.applied_vx = 0.0
            self.applied_yaw_rate = 0.0
            self.target_vx = 0.0
            self.target_yaw = 0.0
            self.mode = "fault"
            return self.report()
        if (now - self.last_cmd_time) > COMMAND_TIMEOUT_S:
            self._snap_zero(now)
            # Timeout is not a fresh command; keep the old stamp so we stay stopped.
            self.last_cmd_time = now - COMMAND_TIMEOUT_S - dt
        if self._snapped:
            self.applied_vx = 0.0
            self.applied_yaw_rate = 0.0
        else:
            self.applied_vx = _slew(self.applied_vx, self.target_vx, VX_SLEW, dt)
            self.applied_yaw_rate = _slew(
                self.applied_yaw_rate, self.target_yaw, YAW_SLEW, dt
            )
        self._update_mode()
        return self.report()

    def limit_applied(self, ceiling: float) -> TickReport:
        """Cap applied velocity at ceiling × target. Does not compound across ticks."""
        cap = _clamp(float(ceiling), 0.0, 1.0)
        if self.fault or self._snapped:
            return self.report()
        vx_lim = abs(self.target_vx) * cap
        yaw_lim = abs(self.target_yaw) * cap
        if abs(self.applied_vx) > vx_lim:
            self.applied_vx = math.copysign(vx_lim, self.applied_vx)
        if abs(self.applied_yaw_rate) > yaw_lim:
            self.applied_yaw_rate = math.copysign(yaw_lim, self.applied_yaw_rate)
        self._update_mode()
        return self.report()

    def declare_fault(self, now: float, reason: str) -> TickReport:
        self.fault = True
        self.fault_reason = reason
        self._snap_zero(now)
        self.mode = "fault"
        return self.report()

    def clear_fault(self) -> None:
        if not self.fault:
            return
        self.fault = False
        self.fault_reason = ""
        self.mode = "stand"

    def report(self) -> TickReport:
        return TickReport(self.applied_vx, self.applied_yaw_rate, self.mode)

    def _snap_zero(self, now: float) -> None:
        self.last_cmd_time = now
        self.target_vx = 0.0
        self.target_yaw = 0.0
        self.applied_vx = 0.0
        self.applied_yaw_rate = 0.0
        self._snapped = True
        if not self.fault:
            self.mode = "stand"

    def _update_mode(self) -> None:
        if self.fault:
            self.mode = "fault"
            return
        moving = (
            abs(self.target_vx) > 0.0
            or abs(self.target_yaw) > 0.0
            or abs(self.applied_vx) > 1e-6
            or abs(self.applied_yaw_rate) > 1e-6
        )
        self.mode = "move" if moving else "stand"


class KeyboardLatch:
    """Press-only keys → CommandBus. Latches until Space (viewer has no key-up)."""

    def __init__(self) -> None:
        self.vx_latch = 0.0
        self.yaw_latch = 0.0
        self._stop = False

    def on_press(self, keycode: int) -> None:
        if keycode == KEY_SPACE:
            self.vx_latch = 0.0
            self.yaw_latch = 0.0
            self._stop = True
            return
        if keycode == KEY_W:
            self.vx_latch = VX_FWD_CAP
        elif keycode == KEY_S:
            self.vx_latch = -VX_BACK_CAP
        elif keycode == KEY_A:
            self.yaw_latch = YAW_RATE_CAP
        elif keycode == KEY_D:
            self.yaw_latch = -YAW_RATE_CAP

    def publish(self, bus: CommandBus, now: float) -> str | None:
        if self._stop:
            self._stop = False
            return bus.stop(now)
        if self.vx_latch == 0.0 and self.yaw_latch == 0.0:
            return None
        return bus.vel(self.vx_latch, self.yaw_latch, now)


@dataclass(frozen=True)
class DemoSegment:
    t_end: float
    kind: Literal["stand", "stop", "vel"]
    vx: float
    yaw_rate: float
    label: str


# Sneak peek: stand → forward → stop. Several body lengths at the lowered
# clamp (amplitude 0.70). 0.080 tips before 2 m. Yaw stays on the API.
DEMO_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(53.0, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(56.0, "stop", 0.0, 0.0, "stop"),
)
# Stop on a gait phase that pitched (support margin about −0.08 m) before
# the settle damper. Same phase as t=2.35, one visible walk later.
# Stand 1 s, forward, stop, hold stand past the damper window.
STOP_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(6.75, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(9.5, "stop", 0.0, 0.0, "stop"),
)
REVERSE_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(17.0, "vel", -VX_BACK_CAP, 0.0, "reverse"),
    DemoSegment(19.0, "stop", 0.0, 0.0, "stop"),
)
# Left turn at amplitude 0.70 needs a longer window to clear ~0.55 rad.
# Raising TURN_STEP_ASYM at this speed yaws the wrong way.
TURN_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(12.0, "vel", VX_FWD_CAP, YAW_RATE_CAP, "turn"),
    DemoSegment(14.5, "stop", 0.0, 0.0, "stop"),
)
CLIP_SCRIPTS: dict[str, tuple[DemoSegment, ...]] = {
    "forward": DEMO_SCRIPT,
    "stop": STOP_SCRIPT,
    "reverse": REVERSE_SCRIPT,
    "turn": TURN_SCRIPT,
}


class ScriptedDriver:
    """Resend vel at 10 Hz. stand once; stop once. Silence is the watchdog's job."""

    def __init__(self, segments: tuple[DemoSegment, ...]) -> None:
        self.segments = segments
        self._last_send = -1.0
        self._stop_sent = False
        self._stand_sent = False

    def segment(self, now: float) -> DemoSegment:
        for seg in self.segments:
            if now < seg.t_end - 1e-9:
                return seg
        return self.segments[-1]

    def publish(self, bus: CommandBus, now: float) -> str | None:
        seg = self.segment(now)
        if seg.kind == "stand":
            if not self._stand_sent:
                self._stand_sent = True
                self._last_send = now
                return bus.stand(now)
            return None
        if seg.kind == "stop":
            if not self._stop_sent:
                self._stop_sent = True
                self._last_send = now
                return bus.stop(now)
            return None
        self._stop_sent = False
        if (now - self._last_send) >= (VEL_RESEND_S - 1e-9):
            self._last_send = now
            return bus.vel(seg.vx, seg.yaw_rate, now)
        return None


def convex_hull_xy(points: np.ndarray) -> np.ndarray:
    uniq = sorted({(float(p[0]), float(p[1])) for p in np.asarray(points, dtype=np.float64)})
    if len(uniq) <= 1:
        return np.asarray(uniq, dtype=np.float64).reshape(-1, 2)

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in uniq:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(uniq):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    return np.asarray(hull, dtype=np.float64).reshape(-1, 2)


def support_margin(point: np.ndarray, hull: np.ndarray) -> float:
    """Signed distance to a CCW hull. Positive means inside."""
    pt = np.asarray(point, dtype=np.float64)
    if len(hull) < 3:
        if len(hull) == 0:
            return -1.0
        return -float(np.min(np.linalg.norm(hull - pt[:2], axis=1)))
    min_left = float("inf")
    for i in range(len(hull)):
        a = hull[i]
        b = hull[(i + 1) % len(hull)]
        edge_x = float(b[0] - a[0])
        edge_y = float(b[1] - a[1])
        length = math.hypot(edge_x, edge_y)
        if length < 1e-12:
            continue
        cross = edge_x * (float(pt[1]) - float(a[1])) - edge_y * (float(pt[0]) - float(a[0]))
        min_left = min(min_left, cross / length)
    if min_left == float("inf"):
        return -1.0
    return float(min_left)


def mirror_sagittal(q_walk: dict[str, float], q_stand: dict[str, float]) -> dict[str, float]:
    """Flip hip/ankle pitch about the stand pose. Knee flexion (clearance) stays."""
    out = dict(q_walk)
    for side in ("l_", "r_"):
        for joint in ("hip_pitch", "ank_pitch"):
            key = f"{side}{joint}"
            out[key] = (2.0 * q_stand[key]) - q_walk[key]
    return out


def apply_frozen_forward_gait() -> None:
    """Faster upright forward on the frozen actuators. Does not edit the plant.

    Gate D CSF50 (T=0.88, hip 0.116, DS=0.31, plant_kd=115) stays on the
    ±2.1 Nm clip and crawls at about 1 cm/s because the stance-slip damper
    brakes the body. This basin shortens the step and lowers that damper.
    It is not a Gate E cadence pass and not a clean-walk claim.
    """
    wg.GAIT_T = 0.55
    wg.STEP_LEN = 0.040
    wg.HIP_PITCH_AMP = 0.24
    wg.HIP_BIAS_FWD = 0.06
    wg.DS_S = 0.1375
    wg.COM_SHIFT_AMP = 0.275
    wg.COM_SHIFT_LEAD = 0.23
    wg.KNEE_STANCE = 0.40
    wg.KNEE_SWING = 0.80
    wg.COM_Z = 0.225
    wg.PLANT_KD = 25.0
    wg.USE_CP_SWING = True
    wg.USE_STANCE_VIK = True
    wg.STANCE_VIK_KP = 0.45
    wg.STANCE_VIK_CLIP = 0.03
    wg.USE_RESIDUAL_STANCE_VX = False
    wg.USE_HIP_STRAT = False
    wg.USE_CAPTURE_STEP = False
    wg.USE_WBC_STANCE = False
    wg.USE_HYBRID_MPC = False


def script_bounds(script: tuple[DemoSegment, ...] = DEMO_SCRIPT) -> dict[str, tuple[float, float]]:
    t0 = 0.0
    bounds: dict[str, tuple[float, float]] = {}
    for seg in script:
        bounds[seg.label] = (t0, seg.t_end)
        t0 = seg.t_end
    return bounds


def gait_amp_and_dir(report: TickReport) -> tuple[float, int]:
    """Map applied velocity to CPG amplitude and sagittal sign (+1 forward)."""
    if report.mode != "move":
        return 0.0, 0
    if abs(report.applied_vx) >= 1e-6:
        direction = 1 if report.applied_vx > 0.0 else -1
        amp = min(1.0, abs(report.applied_vx) / GAIT_AMP_VX)
        return amp, direction
    if abs(report.applied_yaw_rate) >= 1e-6:
        amp = INPLACE_YAW_AMP * min(1.0, abs(report.applied_yaw_rate) / YAW_RATE_CAP)
        return amp, 1
    return 0.0, 0


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _kit_cam_problems(model: mj.MjModel) -> list[str]:
    """Hardware kit_cam on head_tilt_link. Site is not a body and not a contact."""
    problems: list[str] = []
    if model.ncam != 1:
        problems.append(f"expected 1 kit_cam, found {model.ncam}")
        return problems
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cid < 0:
        problems.append("missing camera kit_cam")
        return problems
    body = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, int(model.cam_bodyid[cid])) or ""
    if body != "head_tilt_link":
        problems.append(f"kit_cam parent {body} != head_tilt_link")
    pos = np.asarray(model.cam_pos[cid], dtype=np.float64)
    if float(np.max(np.abs(pos - np.array(KIT_CAM_POS)))) > 1e-6:
        problems.append(f"kit_cam pos {pos.tolist()} != {KIT_CAM_POS}")
    if abs(float(model.cam_fovy[cid]) - KIT_CAM_FOVY) > 1e-4:
        problems.append(f"kit_cam fovy {float(model.cam_fovy[cid])} != {KIT_CAM_FOVY}")
    mat = np.asarray(model.cam_mat0[cid], dtype=np.float64).reshape(3, 3)
    expected = np.array(KIT_CAM_AXES, dtype=np.float64).T
    if float(np.max(np.abs(mat - expected))) > 1e-5:
        problems.append("kit_cam xyaxes != 0 -1 0 0 0 1")
    if model.nsite != 1:
        problems.append(f"expected 1 kit_cam_site, found {model.nsite} sites")
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "kit_cam_site")
    if sid < 0:
        problems.append("missing site kit_cam_site")
    else:
        sbody = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, int(model.site_bodyid[sid])) or ""
        if sbody != "head_tilt_link":
            problems.append(f"kit_cam_site parent {sbody} != head_tilt_link")
        spos = np.asarray(model.site_pos[sid], dtype=np.float64)
        if float(np.max(np.abs(spos - np.array(KIT_CAM_POS)))) > 1e-6:
            problems.append(f"kit_cam_site pos {spos.tolist()} != {KIT_CAM_POS}")
    return problems


def plant_problems(model: mj.MjModel, xml_path: Path = PLANT_XML) -> list[str]:
    """Read-only freeze checks. Empty list means the loaded plant matches Hardware."""
    problems: list[str] = []
    if xml_path.resolve() != PLANT_XML.resolve():
        problems.append(f"plant path {xml_path} is not the frozen M145 walk body")
    digest = _md5(xml_path)
    if digest != PLANT_MD5:
        problems.append(f"plant md5 {digest} != {PLANT_MD5}")
    for i in range(model.nbody):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) or ""
        low = name.lower()
        if "door" in low or "lever" in low:
            problems.append(f"door/lever body present: {name}")
    problems.extend(_kit_cam_problems(model))
    expected_kp = {
        "l_hip_yaw_pos": 40.0, "l_hip_roll_pos": 40.0, "l_hip_pitch_pos": 45.0,
        "l_knee_pos": 45.0, "l_ank_pitch_pos": 35.0, "l_ank_roll_pos": 35.0,
        "r_hip_yaw_pos": 40.0, "r_hip_roll_pos": 40.0, "r_hip_pitch_pos": 45.0,
        "r_knee_pos": 45.0, "r_ank_pitch_pos": 35.0, "r_ank_roll_pos": 35.0,
    }
    for i in range(model.nu):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        fr = model.actuator_forcerange[i]
        kp = float(model.actuator_gainprm[i, 0])
        if name in expected_kp:
            if abs(kp - expected_kp[name]) > 1e-6:
                problems.append(f"{name} kp {kp} != {expected_kp[name]}")
            if abs(float(fr[0]) + LEG_TAU) > 1e-6 or abs(float(fr[1]) - LEG_TAU) > 1e-6:
                problems.append(f"{name} forcerange {fr.tolist()} != ±{LEG_TAU}")
        elif name.startswith(("l_sho", "r_sho", "l_el", "r_el", "l_gripper", "r_gripper", "head_")):
            if abs(float(fr[0]) + ARM_TAU) > 1e-6 or abs(float(fr[1]) - ARM_TAU) > 1e-6:
                problems.append(f"{name} forcerange {fr.tolist()} != ±{ARM_TAU}")
    for gname in ("l_foot_contact", "r_foot_contact"):
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, gname)
        if gid < 0:
            problems.append(f"missing geom {gname}")
            continue
        size = model.geom_size[gid]
        if abs(float(size[0]) - FOOT_HALF_X) > 1e-6 or abs(float(size[1]) - FOOT_HALF_Y) > 1e-6:
            problems.append(f"{gname} size {size[:2].tolist()} != 145×86 half-size")
        if abs(float(model.geom_friction[gid, 0]) - FOOT_FRICTION) > 1e-6:
            problems.append(f"{gname} friction {model.geom_friction[gid, 0]} != {FOOT_FRICTION}")
    for gname in ("l_toe_viz", "r_toe_viz"):
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, gname)
        if gid < 0:
            problems.append(f"missing geom {gname}")
        elif int(model.geom_contype[gid]) != 0:
            problems.append(f"{gname} contype {int(model.geom_contype[gid])} (toe must not hold weight)")
    return problems


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float
    up_z: float
    margin: float
    applied_vx: float
    applied_yaw_rate: float
    mode: ModeName
    body_vx: float


class SteerSession:
    """One frozen-plant sim. Commands go through `bus`; `step` is one 50 Hz tick."""

    def __init__(self, *, video: bool) -> None:
        problems = []
        if not PLANT_XML.is_file():
            problems.append(f"missing {PLANT_XML}")
        else:
            # Hash before load so a missing file is a clean refusal.
            pass
        if problems:
            raise SystemExit("refused: " + "; ".join(problems))
        self.model = mj.MjModel.from_xml_path(str(PLANT_XML))
        self.data = mj.MjData(self.model)
        problems = plant_problems(self.model, PLANT_XML)
        if problems:
            raise SystemExit("refused: " + "; ".join(problems))
        self._force_checksum = float(np.sum(np.abs(self.model.actuator_forcerange)))
        apply_frozen_forward_gait()
        self.bus = CommandBus()
        self.act_idx = {
            mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(self.model.nu)
        }
        self.bid_body = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "body_link")
        self.bid_lf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link")
        self.bid_rf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link")
        self.gid_lfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact")
        self.gid_rfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")
        self.gid_floor = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "floor")
        self.foot_local = {
            "L": (
                self.bid_lf,
                self.gid_lfoot,
                np.array(self.model.geom_pos[self.gid_lfoot], dtype=np.float64),
                np.array(self.model.geom_size[self.gid_lfoot, :2], dtype=np.float64),
            ),
            "R": (
                self.bid_rf,
                self.gid_rfoot,
                np.array(self.model.geom_pos[self.gid_rfoot], dtype=np.float64),
                np.array(self.model.geom_size[self.gid_rfoot, :2], dtype=np.float64),
            ),
        }
        self.q_stand = wg.gait_targets(0.0, False, 0.0)
        self.gait_t = 0.0
        self._gait_live = False
        self._blend = 0.0
        self._q_live: dict[str, float] | None = None
        self._last_amp = 0.0
        self._last_dir = 0
        self._coast_until = -1.0
        self._damper_until = -1.0
        self.hip_yaw_cmd = 0.0
        self.cop_bias: np.ndarray | None = None
        self._cop_samples: list[np.ndarray] = []
        self.tip_hold = 0.0
        self.air_hold = 0.0
        self.cop_out_hold = 0.0
        self.samples: list[PoseSample] = []
        self.max_leg_tau = 0.0
        self.max_arm_tau = 0.0
        self.min_margin = float("inf")
        self.min_up_z = 1.0
        self.max_cop_excursion = 0.0
        self.cop_excursion = 0.0
        self.fault_announced = False
        self.sim_dt = float(self.model.opt.timestep)
        self.steps_per_ctrl = max(1, int(round(CTRL_DT / self.sim_dt)))
        self._reset_stand()
        self.renderer: mj.Renderer | None = None
        self.cam = mj.MjvCamera()
        mj.mjv_defaultCamera(self.cam)
        self.cam.distance = 1.25
        self.cam.azimuth = 135.0
        self.cam.elevation = -18.0
        if video:
            self.renderer = mj.Renderer(self.model, height=480, width=640)

    def _reset_stand(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[2] = wg.COM_Z
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for jn, val in self.q_stand.items():
            jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self.data.qpos[self.model.jnt_qposadr[jid]] = val
        wg.set_ctrl(self.model, self.data, self.q_stand, self.act_idx)
        mj.mj_forward(self.model, self.data)

    def assert_plant_unchanged(self) -> None:
        now = float(np.sum(np.abs(self.model.actuator_forcerange)))
        if abs(now - self._force_checksum) > 1e-9:
            raise RuntimeError("actuator forcerange changed during steer")

    def step(self) -> TickReport:
        now = float(self.data.time)
        report = self.bus.tick(now, CTRL_DT)
        ceiling = self._safety_ceiling()
        if ceiling < 0.999 and report.mode == "move":
            report = self.bus.limit_applied(ceiling)
        amp, direction = self._motion_after_stop(report)
        qdes = self._targets(amp, direction, report.applied_yaw_rate)
        wg.set_ctrl(self.model, self.data, qdes, self.act_idx)
        self._servos(amp, direction)
        self._substep(amp, direction)
        self._update_bias(now)
        margin = self._support_margin()
        up_z = self._up_z()
        self._track_torques()
        self._track_cop_excursion()
        reason = self._fault_reason(now, up_z, margin)
        if reason is not None and not self.bus.fault:
            report = self.bus.declare_fault(now, reason)
            self.hip_yaw_cmd = 0.0
            wg.set_ctrl(self.model, self.data, self.q_stand, self.act_idx)
        elif self.bus.fault and self._recovered(up_z):
            self.bus.clear_fault()
            report = self.bus.report()
        body_vx = self._body_forward_speed()
        self.samples.append(
            PoseSample(
                t=now,
                x=float(self.data.qpos[0]),
                y=float(self.data.qpos[1]),
                yaw=self.yaw(),
                up_z=up_z,
                margin=margin,
                applied_vx=report.applied_vx,
                applied_yaw_rate=report.applied_yaw_rate,
                mode=report.mode,
                body_vx=body_vx,
            )
        )
        self.min_margin = min(self.min_margin, margin)
        self.min_up_z = min(self.min_up_z, up_z)
        return report

    def yaw(self) -> float:
        w, x, y, z = (float(v) for v in self.data.qpos[3:7])
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def render(self, lines: list[str]) -> np.ndarray:
        if self.renderer is None:
            raise RuntimeError("renderer not created")
        self.cam.lookat[:] = self.data.xpos[self.bid_body]
        # mj_step already forwarded. A second mj_forward here changes the
        # contact warm-start and tips this gait before the stop.
        self.renderer.update_scene(self.data, self.cam)
        raw = np.ascontiguousarray(self.renderer.render().copy(), dtype=np.uint8)
        return wg._burn_overlay(raw, lines)

    def _targets(self, amp: float, direction: int, yaw_rate: float) -> dict[str, float]:
        if amp > 0.02:
            if not self._gait_live:
                self.gait_t = 0.0
                self._gait_live = True
            else:
                self.gait_t += CTRL_DT
            qdes = wg.gait_targets(self.gait_t, True, amp)
            if direction < 0:
                qdes = mirror_sagittal(qdes, self.q_stand)
            else:
                self._asymmetric_step(qdes, yaw_rate, amp)
            self._q_live = dict(qdes)
            self._blend = 1.0
        else:
            self._gait_live = False
            qdes = self._settle_targets()
        yaw_cmd = self._hip_yaw(yaw_rate)
        if yaw_rate > 1e-3 and direction >= 0:
            yaw_cmd -= TURN_LEFT_BIAS
        qdes["l_hip_yaw"] = qdes.get("l_hip_yaw", 0.0) + yaw_cmd
        qdes["r_hip_yaw"] = qdes.get("r_hip_yaw", 0.0) + yaw_cmd
        return qdes

    def _reverse_phase_stands(self) -> bool:
        """Reverse phases where an immediate blend already ends in stand.

        Measured on this cadence. Late swing of either foot (about 0.29–0.40
        and 0.84–0.95) pitches. Windows sit inside the passes.
        """
        phi = (self.gait_t / max(float(wg.GAIT_T), 1e-6)) % 1.0
        return phi < 0.22 or (0.50 <= phi < 0.74)

    def _motion_after_stop(self, report: TickReport) -> tuple[float, int]:
        """Bus amplitude, plus a reverse step-finish after stop.

        stand/stop/timeout already report applied_vx = 0. Forward cuts blend
        in place (the settle damper holds them). A reverse cut in a pitching
        phase keeps the last mirrored step until the phase stands up, or one
        gait cycle, whichever comes first.
        """
        amp, direction = gait_amp_and_dir(report)
        now = float(self.data.time)
        if amp > 0.02:
            self._last_amp = amp
            self._last_dir = direction
            self._coast_until = -1.0
            return amp, direction
        # Below the gait threshold the bus amplitude must still be returned.
        # Zeroing it drops the first slew tick's ankle servo and the turn
        # diverges. Reverse step-finish is the only time we replace it.
        if (
            self._last_dir < 0
            and self._gait_live
            and self._last_amp > 0.02
            and not self._reverse_phase_stands()
        ):
            if self._coast_until < 0.0:
                self._coast_until = now + SETTLE_COAST_MAX_S
            if now < self._coast_until:
                return self._last_amp, -1
        return amp, direction

    def _settle_targets(self) -> dict[str, float]:
        """Ease the last step into the stand pose. applied_vx is already 0."""
        blend_s = SETTLE_BLEND_S if self._last_dir >= 0 else REVERSE_SETTLE_BLEND_S
        self._blend = max(0.0, self._blend - CTRL_DT / blend_s)
        if self._q_live is None or self._blend <= 0.0:
            return dict(self.q_stand)
        qdes: dict[str, float] = {}
        for key, stand in self.q_stand.items():
            qdes[key] = stand + self._blend * (self._q_live.get(key, stand) - stand)
        return qdes

    def _asymmetric_step(self, qdes: dict[str, float], yaw_rate: float, amp: float) -> None:
        """Lengthen the outside step. +yaw (left) lengthens the right step."""
        if amp <= 0.05 or abs(yaw_rate) < 1e-4 or TURN_STEP_ASYM <= 0.0:
            return
        frac = _clamp(yaw_rate / YAW_RATE_CAP, -1.0, 1.0)
        for side, side_sign in (("l_", -1.0), ("r_", 1.0)):
            scale = max(0.25, 1.0 + TURN_STEP_ASYM * frac * side_sign)
            for joint in ("hip_pitch", "ank_pitch"):
                key = f"{side}{joint}"
                stand = self.q_stand[key]
                qdes[key] = stand + scale * (qdes.get(key, stand) - stand)

    def _hip_yaw(self, yaw_rate: float) -> float:
        """Common-mode hip yaw. Both joints use axis -Z, so +cmd yaws the body left."""
        if abs(yaw_rate) < 1e-6:
            self.hip_yaw_cmd = 0.0
            return 0.0
        desired = _clamp(YAW_HIP_GAIN * yaw_rate, -YAW_HIP_CLIP, YAW_HIP_CLIP)
        if abs(desired) < TELEOP_DEADBAND:
            desired = 0.0
        step = TELEOP_RATE_LIMIT * CTRL_DT
        self.hip_yaw_cmd += _clamp(desired - self.hip_yaw_cmd, -step, step)
        return self.hip_yaw_cmd

    def _servos(self, amp: float, direction: int) -> None:
        phi = (self.gait_t / wg.GAIT_T) % 1.0 if amp > 0.02 else 0.0
        lat = wg.lateral_com_target(phi) if amp > 0.02 else None
        # Match walk_gait_ainex: CoP gain follows gait amp (0 while standing).
        servo_amp = 0.0 if self.bus.fault else amp
        if servo_amp > 0.0 and float(self.data.time) > 0.15:
            wg.ankle_cop_servo(
                self.model, self.data, self.act_idx, servo_amp,
                self.bid_lf, self.bid_rf, self.gid_floor,
                lat=lat, bias_xy=self.cop_bias,
            )
        if amp > 0.05 and direction >= 0 and wg.USE_CP_SWING:
            wg.apply_cp_swing_placement(
                self.model, self.data, self.act_idx, phi, amp,
                self.bid_lf, self.bid_rf,
            )
        if amp > 0.05 and direction >= 0 and wg.USE_STANCE_VIK:
            wg.apply_stance_jacobian_vik(
                self.model, self.data, self.act_idx, phi, amp,
                self.bid_lf, self.bid_rf, self.gid_floor,
            )

    def _substep(self, amp: float, direction: int) -> None:
        # Stop already zeroed applied_vx. Hold the stance damper for
        # SETTLE_DAMPER_S after the blend starts so a mid-step cut does not
        # pitch out of the foot boxes. Reverse may still be finishing a step
        # (amp > 0) and must not start this window until that step ends.
        now = float(self.data.time)
        # direction != 0 means a step is still being commanded (including the
        # slew up and a reverse step-finish). The settle damper is only for
        # the tick the bus has already zeroed.
        if amp > 0.05 or direction != 0:
            self._damper_until = -1.0
        elif self._damper_until < 0.0 and self._blend > 0.5 and self._last_dir != 0:
            self._damper_until = now + SETTLE_DAMPER_S
        settling = self._damper_until >= 0.0 and now < self._damper_until
        if settling:
            phi = (self.gait_t / max(float(wg.GAIT_T), 1e-6)) % 1.0
            plant_amp = SETTLE_PLANT_AMP
        else:
            phi = (self.gait_t / wg.GAIT_T) % 1.0 if amp > 0.02 else 0.0
            plant_amp = amp
        saved_kd = float(wg.PLANT_KD)
        if settling:
            wg.PLANT_KD = SETTLE_PLANT_KD
        elif direction < 0:
            wg.PLANT_KD = REVERSE_PLANT_KD
        try:
            self._substep_plant(plant_amp, phi)
        finally:
            wg.PLANT_KD = saved_kd

    def _substep_plant(self, amp: float, phi: float) -> None:
        for _ in range(self.steps_per_ctrl):
            self.data.qfrc_applied[:] = 0.0
            self.data.xfrc_applied[:] = 0.0
            if amp > 0.05:
                wg.stance_plant(
                    self.model, self.data, phi, amp,
                    self.bid_lf, self.bid_rf,
                    self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                )
            mj.mj_step(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0
            self.data.xfrc_applied[:] = 0.0

    def _update_bias(self, now: float) -> None:
        if self.cop_bias is not None or now >= wg.ANK_COP_BIAS_SETTLE_S:
            if self.cop_bias is None and self._cop_samples:
                self.cop_bias = np.mean(np.stack(self._cop_samples, axis=0), axis=0)
            return
        com = np.asarray(self.data.subtree_com[0, :2], dtype=np.float64)
        support, _, _, _ = wg._support_feet(
            self.model, self.data, self.bid_lf, self.bid_rf, self.gid_floor, None,
        )
        self._cop_samples.append(com - support)
        if now >= wg.ANK_COP_BIAS_SETTLE_S - 1.5 * CTRL_DT and self._cop_samples:
            self.cop_bias = np.mean(np.stack(self._cop_samples, axis=0), axis=0)

    def _up_z(self) -> float:
        return float(self.data.xmat[self.bid_body].reshape(3, 3)[2, 2])

    def _body_forward_speed(self) -> float:
        rot = self.data.xmat[self.bid_body].reshape(3, 3)
        lin = np.asarray(self.data.cvel[self.bid_body][3:6], dtype=np.float64)
        return float(np.dot(lin, rot[:, 0]))

    def _foot_corners(self, bid: int, gid: int) -> np.ndarray:
        pos = np.asarray(self.model.geom_pos[gid], dtype=np.float64)
        half = np.asarray(self.model.geom_size[gid, :2], dtype=np.float64)
        rot = self.data.xmat[bid].reshape(3, 3)
        origin = np.asarray(self.data.xpos[bid], dtype=np.float64)
        pts: list[np.ndarray] = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                local = np.array([
                    pos[0] + sx * half[0],
                    pos[1] + sy * half[1],
                    pos[2],
                ], dtype=np.float64)
                pts.append((origin + rot @ local)[:2])
        return np.stack(pts, axis=0)

    def _support_margin(self) -> float:
        c_l = wg.foot_floor_contact(self.model, self.data, self.bid_lf, self.gid_floor)
        c_r = wg.foot_floor_contact(self.model, self.data, self.bid_rf, self.gid_floor)
        chunks: list[np.ndarray] = []
        if c_l:
            chunks.append(self._foot_corners(self.bid_lf, self.gid_lfoot))
        if c_r:
            chunks.append(self._foot_corners(self.bid_rf, self.gid_rfoot))
        if not chunks:
            return -1.0
        hull = convex_hull_xy(np.concatenate(chunks, axis=0))
        com = np.asarray(self.data.subtree_com[0, :2], dtype=np.float64)
        return support_margin(com, hull)

    def _track_cop_excursion(self) -> None:
        """Contact CoP vs the 145×86 box. Toe geoms are not in this sum."""
        frame_max = 0.0
        for bid, gid, _pos, _half in self.foot_local.values():
            num = np.zeros(3, dtype=np.float64)
            den = 0.0
            for i in range(self.data.ncon):
                con = self.data.contact[i]
                if int(con.geom1) != gid and int(con.geom2) != gid:
                    continue
                force = np.zeros(6, dtype=np.float64)
                mj.mj_contactForce(self.model, self.data, i, force)
                fn = float(force[0])
                if fn <= 1e-6:
                    continue
                num += fn * np.asarray(con.pos, dtype=np.float64)
                den += fn
            if den <= 1e-6:
                continue
            world = num / den
            rot = self.data.xmat[bid].reshape(3, 3)
            local = rot.T @ (world - np.asarray(self.data.xpos[bid], dtype=np.float64))
            center = np.asarray(self.model.geom_pos[gid, :2], dtype=np.float64)
            half = np.asarray(self.model.geom_size[gid, :2], dtype=np.float64)
            delta = np.abs(local[:2] - center) - half
            frame_max = max(frame_max, float(np.max(delta)))
        self.cop_excursion = frame_max
        self.max_cop_excursion = max(self.max_cop_excursion, frame_max)

    def _track_torques(self) -> None:
        for i in range(self.model.nu):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
            tau = abs(float(self.data.actuator_force[i]))
            if any(tok in name for tok in ("hip_", "knee", "ank_")):
                self.max_leg_tau = max(self.max_leg_tau, tau)
            elif name.startswith(("l_sho", "r_sho", "l_el", "r_el", "l_gripper", "r_gripper", "head_")):
                self.max_arm_tau = max(self.max_arm_tau, tau)

    def _safety_ceiling(self) -> float:
        """Fraction of the commanded velocity the plant may use this tick.

        Touching ±2.1 Nm is normal for this position-servo gait (the hip command
        is force-clipped). A broad stall, a COM outside the foot boxes, or a
        dropping up-vector caps the command. The cap does not compound.
        """
        ceiling = 1.0
        n_sat = 0
        for i in range(self.model.nu):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
            if any(tok in name for tok in ("hip_", "knee", "ank_")):
                lim = LEG_TAU
            elif name.startswith(("l_sho", "r_sho", "l_el", "r_el", "l_gripper", "r_gripper", "head_")):
                lim = ARM_TAU
            else:
                continue
            if abs(float(self.data.actuator_force[i])) >= SAT_FRAC * lim:
                n_sat += 1
        if n_sat >= 6:
            ceiling = min(ceiling, 0.70)
        # COM-vs-hull goes a couple of centimetres outside during a normal
        # CSF50 step while the contact CoP stays inside the 145×86 box.
        # Throttle on that proxy would crush the locked gait. Tip is up_z.
        if self._up_z() < 0.90:
            ceiling = min(ceiling, 0.55)
        return ceiling

    def _fault_reason(self, now: float, up_z: float, margin: float) -> str | None:
        if not np.isfinite(self.data.qpos).all():
            return "non-finite state"
        z = float(self.data.qpos[2])
        if z < 0.08 or z > 0.55:
            return f"body z={z:.3f} out of range"
        if now < SETTLE_GRACE_S:
            return None
        if z < COLLAPSE_Z:
            return f"collapsed z={z:.3f}"
        if up_z < TIP_UP_Z:
            self.tip_hold += CTRL_DT
        else:
            self.tip_hold = 0.0
        if self.tip_hold >= TIP_HOLD_S:
            return f"tip up_z={up_z:.2f}"
        c_l = wg.foot_floor_contact(self.model, self.data, self.bid_lf, self.gid_floor)
        c_r = wg.foot_floor_contact(self.model, self.data, self.bid_rf, self.gid_floor)
        if not c_l and not c_r:
            self.air_hold += CTRL_DT
        else:
            self.air_hold = 0.0
        if self.air_hold >= AIRBORNE_FAULT_S:
            return "airborne"
        if margin < COP_FAULT_MARGIN:
            self.cop_out_hold += CTRL_DT
        else:
            self.cop_out_hold = 0.0
        # Contact CoP is checked separately. COM outside the hull is a tip
        # only together with a falling torso — the locked gait's COM leaves
        # the single-support hull by a few centimetres while still upright.
        if self.cop_out_hold >= TIP_HOLD_S and up_z < 0.85:
            return f"COM outside support and tipping margin={margin:.3f}"
        if self.cop_excursion > 0.008:
            return f"contact CoP outside 145×86 box by {self.cop_excursion:.4f} m"
        return None

    def _recovered(self, up_z: float) -> bool:
        z = float(self.data.qpos[2])
        return up_z > 0.92 and z > 0.18 and self.tip_hold == 0.0


def _segment_window(
    samples: list[PoseSample], t0: float, t1: float,
) -> list[PoseSample]:
    return [s for s in samples if t0 - 1e-9 <= s.t < t1 - 1e-9]


@dataclass
class RunSummary:
    plant: str
    plant_md5: str
    fault: bool
    fault_reason: str
    dx_forward_m: float
    mean_body_vx_forward: float
    dyaw_forward_rad: float
    yaw_forward_min_rad: float
    yaw_forward_max_rad: float
    dyaw_end_rad: float
    dyaw_turn_rad: float
    gait_amp_vx: float
    dx_turn_m: float
    min_up_z: float
    min_support_margin_m: float
    max_contact_cop_outside_box_m: float
    max_leg_tau_nm: float
    max_arm_tau_nm: float
    delta_x_m: float
    mean_vx_m_s: float
    tip: bool
    cop_in_box: bool
    peak_torque_nm: float
    end_mode: str
    end_margin_m: float
    vx_fwd_cap: float
    vx_back_cap: float
    yaw_rate_cap: float
    deadband_vx: float
    deadband_yaw: float
    vx_slew: float
    yaw_slew: float
    teleop_deadband: float
    teleop_rate_limit: float
    honesty: str


def summarize(session: SteerSession, script: tuple[DemoSegment, ...] = DEMO_SCRIPT) -> RunSummary:
    bounds = script_bounds(script)
    if "reverse" in bounds:
        fwd_t = bounds["reverse"]
    elif "forward" in bounds:
        fwd_t = bounds["forward"]
    elif "turn" in bounds:
        fwd_t = bounds["turn"]
    else:
        fwd_t = (1.2, 4.5)
    fwd = _segment_window(session.samples, fwd_t[0], fwd_t[1])
    turn_bounds = bounds.get("turn")
    turn = _segment_window(session.samples, turn_bounds[0], turn_bounds[1]) if turn_bounds is not None else []
    dx_fwd = (fwd[-1].x - fwd[0].x) if len(fwd) >= 2 else 0.0
    mean_vx = float(np.mean([s.body_vx for s in fwd])) if fwd else 0.0
    straight = bounds.get("forward")
    straight_w = _segment_window(session.samples, straight[0], straight[1]) if straight is not None else []
    dyaw_fwd = (straight_w[-1].yaw - straight_w[0].yaw) if len(straight_w) >= 2 else 0.0
    dyaw_fwd = (dyaw_fwd + math.pi) % (2.0 * math.pi) - math.pi
    if straight_w:
        yaw_lo = min(s.yaw for s in straight_w)
        yaw_hi = max(s.yaw for s in straight_w)
    else:
        yaw_lo = 0.0
        yaw_hi = 0.0
    dyaw = (turn[-1].yaw - turn[0].yaw) if len(turn) >= 2 else 0.0
    dx_turn = (turn[-1].x - turn[0].x) if len(turn) >= 2 else 0.0
    # Wrap yaw delta to [-pi, pi]
    dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi
    dyaw_end = 0.0
    if session.samples:
        dyaw_end = session.samples[-1].yaw - session.samples[0].yaw
        dyaw_end = (dyaw_end + math.pi) % (2.0 * math.pi) - math.pi
    tip = bool(session.min_up_z < 0.85 or (session.bus.fault and "tip" in session.bus.fault_reason))
    cop_in_box = bool(session.max_cop_excursion <= 0.001)
    tail = session.samples[-1] if session.samples else None
    honesty = (
        "applied_vx is the clamped command, not odometry. "
        f"Forward clamp is {VX_FWD_CAP:.3f} m/s, lowered from 0.080 because "
        "0.080 yaws off and tips near 1.2 m (support margin about −0.09 m). "
        f"CPG amplitude is |applied_vx| / {GAIT_AMP_VX:.3f}, so full forward "
        f"stick is amplitude {VX_FWD_CAP / GAIT_AMP_VX:.2f} and reverse "
        f"{-VX_BACK_CAP:.3f} m/s stays amplitude {VX_BACK_CAP / GAIT_AMP_VX:.2f} "
        f"with stance damper {REVERSE_PLANT_KD:.0f} N/(m/s). "
        f"Yaw cap is ±{YAW_RATE_CAP:.2f} rad/s; hip-yaw clip {YAW_HIP_CLIP:.2f} rad "
        f"plus outside-step scale {TURN_STEP_ASYM:.2f}. "
        f"Measured motion Δx={dx_fwd:+.3f} m, mean body vx={mean_vx:+.3f} m/s "
        f"(not the command). Straight-forward net yaw drift="
        f"{math.degrees(dyaw_fwd):+.2f} deg; yaw during that window "
        f"{math.degrees(yaw_lo):+.1f} to {math.degrees(yaw_hi):+.1f} deg. "
        f"Heading at the end of the clip is {math.degrees(dyaw_end):+.2f} deg from the start. "
        f"Turn-window Δyaw={math.degrees(dyaw):+.2f} deg. "
        f"tip={tip}; CoP in box={cop_in_box} "
        f"(outside {session.max_cop_excursion:.4f} m); "
        f"peak leg torque={session.max_leg_tau:.2f} Nm (limit {LEG_TAU}). "
        "Stance-slip damper is 25 N/(m/s) while walking forward, down from the "
        "CSF50 crawl's 115. After stop it is "
        f"{SETTLE_PLANT_KD:.0f} N/(m/s) for {SETTLE_DAMPER_S:.2f} s "
        f"(pose blend {SETTLE_BLEND_S:.2f} s), then off. A reverse stop finishes "
        "at most one step first if that phase would pitch. "
        "Foot box, friction, kp, and ±2.1 Nm are unchanged."
    )
    if session.bus.fault:
        honesty += f" FAULT: {session.bus.fault_reason}."
    return RunSummary(
        plant=str(PLANT_XML.relative_to(ROOT)),
        plant_md5=PLANT_MD5,
        fault=bool(session.bus.fault),
        fault_reason=session.bus.fault_reason,
        dx_forward_m=float(dx_fwd),
        mean_body_vx_forward=float(mean_vx),
        dyaw_forward_rad=float(dyaw_fwd),
        yaw_forward_min_rad=float(yaw_lo),
        yaw_forward_max_rad=float(yaw_hi),
        dyaw_end_rad=float(dyaw_end),
        dyaw_turn_rad=float(dyaw),
        gait_amp_vx=GAIT_AMP_VX,
        dx_turn_m=float(dx_turn),
        min_up_z=float(session.min_up_z),
        min_support_margin_m=float(session.min_margin) if math.isfinite(session.min_margin) else -1.0,
        max_contact_cop_outside_box_m=float(session.max_cop_excursion),
        max_leg_tau_nm=float(session.max_leg_tau),
        max_arm_tau_nm=float(session.max_arm_tau),
        delta_x_m=float(dx_fwd),
        mean_vx_m_s=float(mean_vx),
        tip=tip,
        cop_in_box=cop_in_box,
        peak_torque_nm=float(session.max_leg_tau),
        end_mode=tail.mode if tail is not None else "stand",
        end_margin_m=float(tail.margin) if tail is not None else 0.0,
        vx_fwd_cap=VX_FWD_CAP,
        vx_back_cap=VX_BACK_CAP,
        yaw_rate_cap=YAW_RATE_CAP,
        deadband_vx=DEADBAND_VX,
        deadband_yaw=DEADBAND_YAW,
        vx_slew=VX_SLEW,
        yaw_slew=YAW_SLEW,
        teleop_deadband=float(TELEOP_DEADBAND),
        teleop_rate_limit=float(TELEOP_RATE_LIMIT),
        honesty=honesty,
    )


def run_demo(
    *,
    duration: float,
    out_mp4: Path | None,
    log_path: Path | None,
    summary_path: Path | None,
    script: tuple[DemoSegment, ...] = DEMO_SCRIPT,
) -> RunSummary:
    video = out_mp4 is not None
    session = SteerSession(video=video)
    segments = _clip_script(script, duration)
    driver = ScriptedDriver(segments)
    print(
        f"[steer] plant={PLANT_XML.name} md5={PLANT_MD5} "
        f"vx_cap=+{VX_FWD_CAP:.3f}/-{VX_BACK_CAP:.3f} yaw_cap=±{YAW_RATE_CAP:.2f} "
        f"slew vx={VX_SLEW:.2f} yaw={YAW_SLEW:.2f} "
        f"(TELEOP_RATE_LIMIT={TELEOP_RATE_LIMIT} deadband={TELEOP_DEADBAND})"
    )
    print(
        "[steer] gait=forward T=0.55 hip=0.24 ds=0.1375 plant_kd=25 "
        f"amp=|vx|/{GAIT_AMP_VX:.3f} "
        "assist=OFF ankle_cop=ON cp_swing=ON stance_vik=ON residual=OFF door=OFF"
    )
    n_ctrl = int(duration * wg.CTRL_HZ)
    last_print = -1.0
    last_mode: ModeName | None = None
    frames: list[np.ndarray] = []
    refusals: list[str] = []
    for _ in range(n_ctrl):
        now = float(session.data.time)
        refusal = driver.publish(session.bus, now)
        if refusal:
            print(refusal)
            refusals.append(refusal)
        report = session.step()
        if report.mode == "fault" and not session.fault_announced:
            print(f"fault: {session.bus.fault_reason}")
            session.fault_announced = True
        seg = driver.segment(now)
        if report.mode != last_mode or (now - last_print) >= (VEL_RESEND_S - 1e-9):
            print(f"t={now:.2f} {seg.label} {report.line()}")
            last_print = now
            last_mode = report.mode
        if session.renderer is not None and (len(session.samples) % 2 == 0):
            lines = [
                f"Day-1 steer  {seg.label}  M145 frozen  no door",
                report.line(),
                f"t={now:.2f}s  x={session.data.qpos[0]:+.3f}  yaw={math.degrees(session.yaw()):+.1f} deg",
                "W/S vx  A/D yaw  space stop  |  voice uses the same bus",
            ]
            frames.append(session.render(lines))
        if session.bus.fault and float(session.data.time) > now + 0.4:
            # Keep a short fault tail so the clip shows the stop, then end.
            pass
    session.assert_plant_unchanged()
    summary = summarize(session, script)
    print("[steer] " + summary.honesty)
    if log_path is not None:
        _write_log(log_path, session, summary, refusals)
    if summary_path is not None:
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(asdict(summary), indent=2) + "\n", encoding="utf-8")
        print(f"[steer] wrote {summary_path}")
    if out_mp4 is not None and frames:
        _write_mp4(frames, out_mp4)
        print(f"[steer] wrote {out_mp4} ({len(frames)} frames)")
    return summary


def _clip_script(script: tuple[DemoSegment, ...], duration: float) -> tuple[DemoSegment, ...]:
    out: list[DemoSegment] = []
    for seg in script:
        if seg.t_end <= duration + 1e-9:
            out.append(seg)
        else:
            out.append(DemoSegment(duration, seg.kind, seg.vx, seg.yaw_rate, seg.label))
            break
    if not out:
        out.append(DemoSegment(duration, "stand", 0.0, 0.0, "stand"))
    return tuple(out)


def _write_log(
    path: Path,
    session: SteerSession,
    summary: RunSummary,
    refusals: list[str],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "Day-1 steer log",
        summary.honesty,
        f"fault={summary.fault} reason={summary.fault_reason or '-'}",
        "t mode applied_vx applied_yaw_rate x y yaw_deg up_z margin body_vx",
    ]
    for s in session.samples:
        lines.append(
            f"{s.t:.3f} {s.mode} {s.applied_vx:+.4f} {s.applied_yaw_rate:+.4f} "
            f"{s.x:+.4f} {s.y:+.4f} {math.degrees(s.yaw):+.2f} {s.up_z:.3f} "
            f"{s.margin:+.4f} {s.body_vx:+.4f}"
        )
    if refusals:
        lines.append("refusals:")
        lines.extend(refusals)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[steer] wrote {path}")


def _write_mp4(frames: list[np.ndarray], out_mp4: Path) -> None:
    import imageio.v2 as imageio

    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="steer_frames_") as tmp:
        folder = Path(tmp)
        for i, frame in enumerate(frames):
            imageio.imwrite(folder / f"frame_{i:05d}.png", frame)
        wg._encode_mp4_ffmpeg(folder, out_mp4, fps=25)


def run_view(duration: float) -> None:
    try:
        from mujoco import viewer
    except ImportError:
        print("refused: mujoco.viewer unavailable", file=sys.stderr)
        raise SystemExit(1)
    session = SteerSession(video=False)
    keys = KeyboardLatch()

    def on_key(keycode: int) -> None:
        keys.on_press(keycode)

    print("[steer] view  W/S=±vx  A/D=±yaw (left/right)  space=stop  latched until space")
    print("[steer] voice later calls CommandBus.stand / stop / vel — same bus")
    import time
    with viewer.launch_passive(session.model, session.data, key_callback=on_key) as handle:
        wall0 = time.time()
        while handle.is_running() and float(session.data.time) < duration:
            now = float(session.data.time)
            refusal = keys.publish(session.bus, now)
            if refusal:
                print(refusal)
            report = session.step()
            if report.mode == "fault" and not session.fault_announced:
                print(f"fault: {session.bus.fault_reason}")
                session.fault_announced = True
            if int(now * wg.CTRL_HZ) % 10 == 0:
                print(f"t={now:.2f} {report.line()}")
            handle.sync()
            sleep = (wall0 + float(session.data.time)) - time.time()
            if sleep > 0:
                time.sleep(sleep)
    session.assert_plant_unchanged()
    print(f"[steer] view done t={session.data.time:.2f}s mode={session.bus.mode}")


def _expect(cond: bool, msg: str, failures: list[str]) -> None:
    if not cond:
        failures.append(msg)


def test_bus() -> list[str]:
    failures: list[str] = []
    _expect(VX_SLEW <= TELEOP_RATE_LIMIT, "vx slew exceeds TELEOP_RATE_LIMIT", failures)
    _expect(YAW_SLEW <= TELEOP_RATE_LIMIT, "yaw slew exceeds TELEOP_RATE_LIMIT", failures)
    _expect(YAW_HIP_CLIP > TELEOP_DEADBAND, "hip yaw clip is inside joint deadband", failures)
    bus = CommandBus()
    report = bus.tick(0.0, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "power-on is not stand", failures)
    refusal = bus.vel(0.10, 0.0, 0.0, vy=0.2)
    _expect(refusal == "refused: vel accepts vx and yaw_rate only (got vy)", f"vy refusal got {refusal}", failures)
    refusal = bus.submit("waypoint", 0.0, x=1.0)
    _expect(refusal is not None and refusal.startswith("refused:"), f"waypoint got {refusal}", failures)
    refusal = bus.vel(float("nan"), 0.0, 0.0)
    _expect(refusal == "refused: non-finite vx or yaw_rate", f"nan got {refusal}", failures)
    bus.vel(5.0, 5.0, 0.0)
    _expect(bus.target_vx == VX_FWD_CAP and bus.target_yaw == YAW_RATE_CAP, "caps not applied", failures)
    report = bus.tick(0.0, CTRL_DT)
    _expect(report.mode == "move", "vel did not enter move", failures)
    _expect(abs(report.applied_vx - VX_SLEW * CTRL_DT) < 1e-9, "vx slew wrong", failures)
    _expect(report.applied_vx < VX_FWD_CAP, "slew jumped to cap", failures)
    bus.stop(0.02)
    report = bus.tick(0.02, CTRL_DT)
    _expect(
        report.mode == "stand" and report.applied_vx == 0.0 and report.applied_yaw_rate == 0.0,
        "stop did not zero same tick",
        failures,
    )
    bus.vel(VX_FWD_CAP, 0.0, 1.0)
    bus.tick(1.0, CTRL_DT)
    report = bus.tick(1.0 + COMMAND_TIMEOUT_S + 0.02, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "timeout did not stand", failures)
    bus2 = CommandBus()
    bus2.vel(DEADBAND_VX * 0.5, 0.0, 0.0)
    report = bus2.tick(0.0, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "deadband did not zero vx", failures)
    full = TickReport(VX_FWD_CAP, 0.0, "move")
    amp, direction = gait_amp_and_dir(full)
    _expect(
        abs(amp - VX_FWD_CAP / GAIT_AMP_VX) < 1e-9 and direction == 1,
        f"full-stick amp {amp} (divisor must stay {GAIT_AMP_VX})",
        failures,
    )
    back = TickReport(-VX_BACK_CAP, 0.0, "move")
    amp, direction = gait_amp_and_dir(back)
    _expect(
        abs(amp - VX_BACK_CAP / GAIT_AMP_VX) < 1e-9 and direction == -1,
        f"reverse amp {amp}",
        failures,
    )
    _expect(GAIT_AMP_VX + 1e-12 >= VX_FWD_CAP, "amplitude reference below the forward clamp", failures)
    bus2.declare_fault(0.1, "tip")
    refusal = bus2.vel(0.1, 0.0, 0.1)
    _expect(refusal is not None and refusal.startswith("refused: fault"), f"fault vel got {refusal}", failures)
    report = bus2.tick(0.12, CTRL_DT)
    _expect(report.mode == "fault" and report.applied_vx == 0.0, "fault tick not zero", failures)
    return failures


def test_keys() -> list[str]:
    failures: list[str] = []
    bus = CommandBus()
    keys = KeyboardLatch()
    keys.on_press(KEY_W)
    refusal = keys.publish(bus, 0.0)
    _expect(refusal is None, "W refused", failures)
    report = bus.tick(0.0, CTRL_DT)
    _expect(report.mode == "move" and report.applied_vx > 0.0, "W did not command +vx", failures)
    keys.on_press(KEY_A)
    keys.publish(bus, 0.02)
    _expect(bus.target_vx == VX_FWD_CAP and bus.target_yaw == YAW_RATE_CAP, "W+A latch", failures)
    _expect(bus.target_yaw > 0.0, "A is not turn-left (positive)", failures)
    keys.on_press(KEY_SPACE)
    keys.publish(bus, 0.04)
    report = bus.tick(0.04, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "space did not stop", failures)
    keys.on_press(KEY_S)
    keys.publish(bus, 0.06)
    _expect(bus.target_vx == -VX_BACK_CAP, "S is not reverse cap", failures)
    keys.on_press(KEY_D)
    keys.publish(bus, 0.08)
    _expect(bus.target_yaw == -YAW_RATE_CAP, "D is not turn-right", failures)
    return failures


def test_hull() -> list[str]:
    failures: list[str] = []
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    hull = convex_hull_xy(square)
    mid = support_margin(np.array([0.5, 0.5]), hull)
    edge = support_margin(np.array([0.0, 0.5]), hull)
    out = support_margin(np.array([1.2, 0.5]), hull)
    _expect(abs(mid - 0.5) < 1e-6, f"mid margin {mid}", failures)
    _expect(abs(edge) < 1e-6, f"edge margin {edge}", failures)
    _expect(abs(out - (-0.2)) < 1e-6, f"outside margin {out}", failures)
    q_stand = {
        "l_hip_pitch": -0.07, "l_ank_pitch": 0.35,
        "r_hip_pitch": 0.07, "r_ank_pitch": 0.35,
        "l_knee": 0.42,
    }
    q_walk = dict(q_stand)
    q_walk["l_hip_pitch"] = -0.07 + 0.12
    q_walk["l_ank_pitch"] = 0.35 + 0.12
    mirrored = mirror_sagittal(q_walk, q_stand)
    _expect(mirrored["l_knee"] == 0.42, "knee was mirrored", failures)
    _expect(abs(mirrored["l_hip_pitch"] - (-0.07 - 0.12)) < 1e-9, "hip mirror", failures)
    return failures


def test_plant_file() -> list[str]:
    if not PLANT_XML.is_file():
        return [f"missing {PLANT_XML}"]
    model = mj.MjModel.from_xml_path(str(PLANT_XML))
    return plant_problems(model, PLANT_XML)


def test_smoke_sim() -> list[str]:
    """Stand, forward, stop on the frozen plant. No video."""
    failures: list[str] = []
    duration = DEMO_SCRIPT[-1].t_end
    session = SteerSession(video=False)
    driver = ScriptedDriver(DEMO_SCRIPT)
    n_ctrl = int(duration * wg.CTRL_HZ)
    bounds = script_bounds()
    fwd_t = bounds["forward"]
    saw_forward = False
    for _ in range(n_ctrl):
        now = float(session.data.time)
        refusal = driver.publish(session.bus, now)
        if refusal:
            failures.append(refusal)
        report = session.step()
        if fwd_t[0] + 1.0 <= now < fwd_t[1] - 0.1 and report.applied_vx > 0.5 * VX_FWD_CAP and report.mode == "move":
            saw_forward = True
    session.assert_plant_unchanged()
    summary = summarize(session)
    print("[steer] smoke " + summary.honesty)
    _expect(not summary.fault, f"smoke fault: {summary.fault_reason}", failures)
    _expect(not summary.tip, f"tipped up_z={summary.min_up_z:.3f}", failures)
    _expect(saw_forward, "forward command was not applied", failures)
    _expect(summary.dx_forward_m > 2.0, f"forward Δx={summary.dx_forward_m:.3f} m", failures)
    _expect(summary.mean_body_vx_forward > 0.03, f"mean vx={summary.mean_body_vx_forward:.3f}", failures)
    _expect(
        abs(summary.dyaw_forward_rad) < 0.20,
        f"forward yaw drift={summary.dyaw_forward_rad:.3f} rad",
        failures,
    )
    _expect(
        abs(summary.yaw_forward_min_rad) < 0.45 and abs(summary.yaw_forward_max_rad) < 0.45,
        f"forward yaw span {summary.yaw_forward_min_rad:.3f}..{summary.yaw_forward_max_rad:.3f}",
        failures,
    )
    _expect(abs(summary.dyaw_end_rad) < 0.20, f"end heading {summary.dyaw_end_rad:.3f} rad", failures)
    _expect(summary.min_up_z >= 0.90, f"min up_z={summary.min_up_z:.3f}", failures)
    _expect(summary.cop_in_box, "CoP left the foot box", failures)
    _expect(summary.max_leg_tau_nm <= LEG_TAU + 1e-3, "leg torque above freeze", failures)
    _expect(summary.max_contact_cop_outside_box_m <= 0.005, "CoP left the foot box", failures)
    tail = session.samples[-1]
    _expect(tail.mode == "stand" and abs(tail.applied_vx) < 1e-6, "did not end in stand", failures)
    _expect(tail.margin > 0.02, f"forward stop margin {tail.margin:+.3f}", failures)
    failures.extend(test_stop_settle())
    failures.extend(test_reverse_and_turn())
    return failures


def test_stop_settle() -> list[str]:
    """Forward→stop on phases that used to pitch, plus a 200 ms silence stop.

    The tip check is unchanged. These cuts previously latched
    'COM outside support and tipping' with margin about −0.08 m.
    """
    failures: list[str] = []
    # Gait phases that faulted with an immediate blend and the damper off.
    for stop_at in (2.00, 2.25, 2.35, 2.55, 2.60, 2.80, 2.90):
        script = (
            DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
            DemoSegment(stop_at, "vel", VX_FWD_CAP, 0.0, "forward"),
            DemoSegment(stop_at + 2.5, "stop", 0.0, 0.0, "stop"),
        )
        session, summary = _run_script(script)
        tail = session.samples[-1]
        _expect(not summary.fault, f"stop@{stop_at:.2f} fault: {summary.fault_reason}", failures)
        _expect(not summary.tip, f"stop@{stop_at:.2f} tip up_z={summary.min_up_z:.3f}", failures)
        _expect(summary.min_up_z >= 0.90, f"stop@{stop_at:.2f} min up_z={summary.min_up_z:.3f}", failures)
        _expect(summary.cop_in_box, f"stop@{stop_at:.2f} CoP left the box", failures)
        _expect(tail.mode == "stand", f"stop@{stop_at:.2f} ended {tail.mode}", failures)
        _expect(tail.margin > 0.02, f"stop@{stop_at:.2f} end margin {tail.margin:+.3f}", failures)
    # Reverse cuts that pitched without the step-finish and settle damper.
    for stop_at in (2.30, 3.40, 3.80):
        rev_script = (
            DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
            DemoSegment(stop_at, "vel", -VX_BACK_CAP, 0.0, "reverse"),
            DemoSegment(stop_at + 2.6, "stop", 0.0, 0.0, "stop"),
        )
        rev, rev_sum = _run_script(rev_script)
        _expect(not rev_sum.fault, f"reverse@{stop_at:.2f} fault: {rev_sum.fault_reason}", failures)
        _expect(not rev_sum.tip, f"reverse@{stop_at:.2f} tip up_z={rev_sum.min_up_z:.3f}", failures)
        _expect(rev_sum.min_up_z >= 0.90, f"reverse@{stop_at:.2f} min up_z={rev_sum.min_up_z:.3f}", failures)
        _expect(rev_sum.cop_in_box, f"reverse@{stop_at:.2f} CoP left the box", failures)
        _expect(rev.samples[-1].mode == "stand", f"reverse@{stop_at:.2f} ended {rev.samples[-1].mode}", failures)
        _expect(rev.samples[-1].margin > 0.02, f"reverse@{stop_at:.2f} end margin {rev.samples[-1].margin:+.3f}", failures)
    rev_script = (
        DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        DemoSegment(6.5, "vel", -VX_BACK_CAP, 0.0, "reverse"),
        DemoSegment(9.0, "stop", 0.0, 0.0, "stop"),
    )
    rev, rev_sum = _run_script(rev_script)
    _expect(not rev_sum.fault, f"short reverse fault: {rev_sum.fault_reason}", failures)
    _expect(not rev_sum.tip, f"short reverse tip up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.min_up_z >= 0.90, f"short reverse min up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.cop_in_box, "short reverse CoP left the box", failures)
    _expect(rev.samples[-1].mode == "stand", "short reverse did not end in stand", failures)
    _expect(rev_sum.dx_forward_m < -0.05, f"short reverse Δx={rev_sum.dx_forward_m:.3f}", failures)
    # Voice path: resend vel at 10 Hz, then silence. The 200 ms watchdog stops.
    session = SteerSession(video=False)
    last_send = -1.0
    t_silent = 2.35
    duration = 5.0
    n_ctrl = int(duration * wg.CTRL_HZ)
    for _ in range(n_ctrl):
        now = float(session.data.time)
        if now < 1.0 - 1e-9:
            if last_send < 0.0:
                session.bus.stand(now)
                last_send = now
        elif now < t_silent - 1e-9:
            if (now - last_send) >= (VEL_RESEND_S - 1e-9):
                session.bus.vel(VX_FWD_CAP, 0.0, now)
                last_send = now
        session.step()
    tail = session.samples[-1]
    _expect(not session.bus.fault, f"silence fault: {session.bus.fault_reason}", failures)
    _expect(tail.mode == "stand" and abs(tail.applied_vx) < 1e-6, "silence did not end in stand", failures)
    _expect(session.min_up_z >= 0.90, f"silence min up_z={session.min_up_z:.3f}", failures)
    _expect(session.max_cop_excursion <= 0.001, "silence CoP left the box", failures)
    _expect(tail.margin > 0.02, f"silence end margin {tail.margin:+.3f}", failures)
    print(
        f"[steer] stop-settle phases ok, short reverse Δx={rev_sum.dx_forward_m:+.3f} m "
        f"up={rev_sum.min_up_z:.3f}, silence end={tail.mode}"
    )
    return failures


def _run_script(script: tuple[DemoSegment, ...]) -> tuple[SteerSession, RunSummary]:
    session = SteerSession(video=False)
    driver = ScriptedDriver(script)
    n_ctrl = int(script[-1].t_end * wg.CTRL_HZ)
    for _ in range(n_ctrl):
        now = float(session.data.time)
        driver.publish(session.bus, now)
        session.step()
    session.assert_plant_unchanged()
    return session, summarize(session, script)


def test_reverse_and_turn() -> list[str]:
    """Upright retreat and a walking left/right turn. No video."""
    failures: list[str] = []
    rev, rev_sum = _run_script(REVERSE_SCRIPT)
    print("[steer] reverse " + rev_sum.honesty)
    _expect(not rev_sum.fault, f"reverse fault: {rev_sum.fault_reason}", failures)
    _expect(not rev_sum.tip, f"reverse tip up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.dx_forward_m < -0.70, f"reverse Δx={rev_sum.dx_forward_m:.3f} m", failures)
    _expect(rev_sum.min_up_z >= 0.90, f"reverse min up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.cop_in_box, "reverse CoP left the foot box", failures)
    _expect(rev.samples[-1].mode == "stand", "reverse did not end in stand", failures)

    turn, turn_sum = _run_script(TURN_SCRIPT)
    print("[steer] turn " + turn_sum.honesty)
    _expect(not turn_sum.fault, f"turn fault: {turn_sum.fault_reason}", failures)
    _expect(not turn_sum.tip, f"turn tip up_z={turn_sum.min_up_z:.3f}", failures)
    _expect(turn_sum.dyaw_turn_rad > 0.55, f"turn Δyaw={turn_sum.dyaw_turn_rad:.3f} rad", failures)
    _expect(turn_sum.dx_turn_m > 0.15, f"turn Δx={turn_sum.dx_turn_m:.3f} m", failures)
    _expect(turn_sum.min_up_z >= 0.90, f"turn min up_z={turn_sum.min_up_z:.3f}", failures)
    _expect(turn_sum.cop_in_box, "turn CoP left the foot box", failures)
    _expect(turn.samples[-1].mode == "stand", "turn did not end in stand", failures)
    turn_end = [s for s in turn.samples if s.t < TURN_SCRIPT[1].t_end][-1]
    unwind = abs(turn.samples[-1].yaw - turn_end.yaw)
    _expect(unwind < 0.20, f"heading unwound {unwind:.3f} rad after stop", failures)

    # Right is the weaker direction: the open-loop gait drifts left, so the
    # same stick does not mirror. This duration stays upright.
    right_script = (
        DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        DemoSegment(5.5, "vel", VX_FWD_CAP, -YAW_RATE_CAP, "turn"),
        DemoSegment(7.5, "stop", 0.0, 0.0, "stop"),
    )
    _right, right_sum = _run_script(right_script)
    print(
        f"[steer] right Δyaw={right_sum.dyaw_turn_rad:+.3f} rad "
        f"Δx={right_sum.dx_turn_m:+.3f} m up={right_sum.min_up_z:.3f} "
        f"fault={right_sum.fault_reason or '-'}"
    )
    _expect(not right_sum.fault, f"right fault: {right_sum.fault_reason}", failures)
    _expect(not right_sum.tip, f"right tip up_z={right_sum.min_up_z:.3f}", failures)
    _expect(right_sum.dyaw_turn_rad < -0.15, f"right Δyaw={right_sum.dyaw_turn_rad:.3f}", failures)
    _expect(right_sum.min_up_z >= 0.90, f"right min up_z={right_sum.min_up_z:.3f}", failures)
    _expect(right_sum.cop_in_box, "right CoP left the foot box", failures)
    return failures


def self_test() -> int:
    failures: list[str] = []
    failures.extend(test_bus())
    failures.extend(test_keys())
    failures.extend(test_hull())
    failures.extend(test_plant_file())
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    failures.extend(test_smoke_sim())
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[steer] self-test PASS")
    return 0


def main() -> None:
    ap = argparse.ArgumentParser(description="Day-1 velocity steer on frozen M145 (no door)")
    ap.add_argument("--view", action="store_true", help="Interactive viewer + keyboard")
    ap.add_argument("--duration", type=float, default=None, help="Seconds (demo default is the scripted clip, view default 120)")
    ap.add_argument("--clip", choices=tuple(CLIP_SCRIPTS), default="forward")
    ap.add_argument("--out", type=str, default=None)
    ap.add_argument("--no-video", action="store_true", help="Headless sim without mp4")
    ap.add_argument("--self-test", action="store_true", help="Command bus, freeze checks, short sim")
    ap.add_argument("--log", type=str, default=None)
    ap.add_argument("--summary", type=str, default=None)
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(self_test())
    if args.view:
        run_view(120.0 if args.duration is None else float(args.duration))
        return
    script = CLIP_SCRIPTS[args.clip]
    stem = {
        "forward": "steer_walk_forward",
        "stop": "steer_walk_stop",
        "reverse": "steer_walk_reverse",
        "turn": "steer_walk_turn",
    }[args.clip]
    duration = script[-1].t_end if args.duration is None else float(args.duration)
    out = None if args.no_video else Path(args.out or (PREVIEWS / f"{stem}.mp4"))
    run_demo(
        duration=duration,
        out_mp4=out,
        log_path=Path(args.log or (PREVIEWS / f"{stem}_log.txt")),
        summary_path=Path(args.summary or (PREVIEWS / f"{stem}_summary.json")),
        script=script,
    )


if __name__ == "__main__":
    main()
