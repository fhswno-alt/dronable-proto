#!/usr/bin/env python3
"""Hiwonder GaitManager foot trajectory, from the published walker.

The numbers are the published ones, not a sweep:

- ``walking_param.yaml``: period 400 ms, dsp_ratio 0.2, step_fb_ratio 0.028,
  z_move 0.02 m, y_swap 0.02 m, z_swap 0.006 m, pelvis_offset 5 deg,
  hip_pitch_offset 15 deg.
- ``gait_manager.py`` speed table: [300, 0.2, 0.02], [400, 0.2, 0.02],
  [500, 0.2, 0.02], [600, 0.1, 0.04].
- ``gait_control_demo.py`` forward walk: set_step([400, 0.2, 0.02], x=0.02,
  y=0, rot=0, arm_swap=30).
- Straight-walk gears in ``ainex_controller.py``: 300 ms x=0.012 z=0.015;
  400 ms x=0.013; 500 ms x=0.015; 600 ms x=0.015. dsp 0.2, y_swap 0.02,
  z 0.02 except the 300 ms gear.

The shape is the ROBOTIS OP2 ``wSin`` schedule that ``walking_module.so``
embeds (same field names, including ``hit_pitch_offset_``). Period in that
code is one left-plus-right cycle. ``step_fb_ratio`` stays 0.028.

``hip_pitch_offset`` of 15 deg is on the live OP3 stand and the walk.
This clock does not add the trim; the live path is ``op3_walk``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# walking_param.yaml and gait_control_demo.set_step. Do not retune these.
GM_PERIOD_S = 0.400
GM_DSP = 0.20
GM_Y_SWAP_M = 0.020
GM_X_M = 0.020
GM_Z_M = 0.020
GM_Z_SWAP_M = 0.006
GM_STEP_FB = 0.028
GM_PELVIS_DEG = 5.0
GM_ARM_DEG = 30.0
GM_HIP_PITCH_DEG = 15.0
# gait_manager.py dsp_ratio table: period ms, dsp, y_swap m.
GM_DSP_TABLE: tuple[tuple[float, float, float], ...] = (
    (0.300, 0.20, 0.020),
    (0.400, 0.20, 0.020),
    (0.500, 0.20, 0.020),
    (0.600, 0.10, 0.040),
)
# ainex_controller.py straight walk (x only): period s, x m, z m.
GM_SPEED_GEARS: tuple[tuple[float, float, float], ...] = (
    (0.300, 0.012, 0.015),
    (0.400, 0.013, 0.020),
    (0.500, 0.015, 0.020),
    (0.600, 0.015, 0.020),
)

_X_SWAP_PHASE = math.pi
_X_MOVE_PHASE = math.pi / 2
_Y_SWAP_PHASE = 0.0
_Y_MOVE_PHASE = math.pi / 2
_Z_SWAP_PHASE = 3.0 * math.pi / 2
_Z_MOVE_PHASE = math.pi / 2
_A_MOVE_PHASE = math.pi / 2


def wsin(time: float, period: float, period_shift: float, mag: float, mag_shift: float) -> float:
    """ROBOTIS ``WalkingModule::wSin``. Period is seconds."""
    if period <= 1e-9:
        return mag_shift
    return mag * math.sin(2.0 * math.pi / period * time - period_shift) + mag_shift


@dataclass
class FootShift:
    """Body-frame offset from the t=0 pose of this cycle. Metres and radians.

    ``x_abs`` is the published endpoint, not that delta. It is 0 when the
    foot is under the hip. t=0 is already split fore-aft by ±x_move, so the
    delta is still ±x_move at mid-swing.
    """

    x: float
    y: float
    z: float
    yaw: float
    pelvis_roll: float
    x_abs: float = 0.0


class GaitManagerClock:
    """One published cycle. ``x_amp`` is the demo's x_move_amplitude in metres."""

    def __init__(
        self,
        period_s: float = GM_PERIOD_S,
        dsp: float = GM_DSP,
        y_swap: float = GM_Y_SWAP_M,
        z_move: float = GM_Z_M,
        z_swap: float = GM_Z_SWAP_M,
        step_fb: float = GM_STEP_FB,
        pelvis_deg: float = GM_PELVIS_DEG,
    ) -> None:
        self.period_cmd = float(period_s)
        self.dsp_cmd = float(dsp)
        self.y_swap_cmd = float(y_swap)
        self.z_move_cmd = float(z_move)
        self.z_swap_cmd = float(z_swap)
        self.step_fb = float(step_fb)
        self.pelvis = math.radians(pelvis_deg)
        self.x_cmd = 0.0
        self.y_cmd = 0.0
        self.angle_rad = 0.0
        self.time = 0.0
        self.previous_x = 0.0
        self._x_move = 0.0
        self._x_swap = 0.0
        self._y_move = 0.0
        self._y_move_shift = 0.0
        self._y_swap = 0.0
        self._z_move = 0.0
        self._z_move_shift = 0.0
        self._z_swap = 0.0
        self._z_swap_shift = 0.0
        self._a_move = 0.0
        self._a_move_shift = 0.0
        self._zero_l = (0.0, 0.0, 0.0, 0.0)
        self._zero_r = (0.0, 0.0, 0.0, 0.0)
        self._update_time()
        self._update_movement()
        self._capture_zero()

    def _update_time(self) -> None:
        period = self.period_cmd
        dsp = min(0.95, max(0.0, self.dsp_cmd))
        ssp = 1.0 - dsp
        self.period = period
        self.x_swap_period = period / 2.0
        self.x_move_period = period * ssp
        self.y_swap_period = period
        self.y_move_period = period * ssp
        self.z_swap_period = period / 2.0
        self.z_move_period = period * ssp / 2.0
        self.a_move_period = period * ssp
        self.l_ssp_start = (1.0 - ssp) * period / 4.0
        self.l_ssp_end = (1.0 + ssp) * period / 4.0
        self.r_ssp_start = (3.0 - ssp) * period / 4.0
        self.r_ssp_end = (3.0 + ssp) * period / 4.0
        self.phase1 = (self.l_ssp_start + self.l_ssp_end) / 2.0
        self.phase2 = (self.l_ssp_end + self.r_ssp_start) / 2.0
        self.phase3 = (self.r_ssp_start + self.r_ssp_end) / 2.0
        self.pelvis_swing = self.pelvis * 0.35

    def _update_movement(self) -> None:
        x = self.x_cmd
        x_swap = x * self.step_fb
        if self.previous_x == 0.0:
            x *= 0.5
            x_swap *= 0.5
        self._x_move = x
        self._x_swap = x_swap
        y = self.y_cmd / 2.0
        self._y_move = y
        self._y_move_shift = y if y > 0.0 else -y
        self._y_swap = self.y_swap_cmd + self._y_move_shift * 0.04
        self._z_move = self.z_move_cmd / 2.0
        self._z_move_shift = self._z_move / 2.0
        self._z_swap = self.z_swap_cmd
        self._z_swap_shift = self.z_swap_cmd
        a = self.angle_rad / 2.0
        self._a_move = a
        self._a_move_shift = a if a > 0.0 else -a

    def _phase_update(self, dt: float) -> None:
        t = self.time
        half = dt / 2.0
        if t == 0.0:
            self._update_time()
            self._update_movement()
        elif abs(t - self.phase1) <= half:
            self._update_movement()
        elif abs(t - self.phase2) <= half:
            self._update_time()
            self.time = self.phase2
        elif abs(t - self.phase3) <= half:
            self._update_movement()

    def set_command(self, x_amp: float, y_amp: float = 0.0, angle_deg: float = 0.0) -> None:
        self.x_cmd = float(x_amp)
        self.y_cmd = float(y_amp)
        self.angle_rad = math.radians(angle_deg)

    def _raw(self, t: float) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float], float, float]:
        """Absolute (x, y, z, yaw) of each foot, plus pelvis roll L/R. Not yet differenced."""
        sx = wsin(t, self.x_swap_period, _X_SWAP_PHASE, self._x_swap, 0.0)
        sy = wsin(t, self.y_swap_period, _Y_SWAP_PHASE, self._y_swap, 0.0)
        sz = wsin(t, self.z_swap_period, _Z_SWAP_PHASE, self._z_swap, self._z_swap_shift)

        # The branch structure of computeLegAngle, including which time each
        # channel is frozen to. Pelvis roll is radians (5 deg published).
        if t <= self.l_ssp_start:
            left = self._leg(t_x=self.l_ssp_start, t_z=self.l_ssp_start, phase_t=self.l_ssp_start, sign=1.0, sx=sx, sy=sy, sz=sz)
            right = self._leg(t_x=self.l_ssp_start, t_z=self.r_ssp_start, phase_t=self.l_ssp_start, sign=-1.0, sx=sx, sy=sy, sz=sz, z_phase_t=self.r_ssp_start)
            pel_l, pel_r = 0.0, 0.0
        elif t <= self.l_ssp_end:
            left = self._leg(t_x=t, t_z=t, phase_t=self.l_ssp_start, sign=1.0, sx=sx, sy=sy, sz=sz)
            right = self._leg(t_x=t, t_z=self.r_ssp_start, phase_t=self.l_ssp_start, sign=-1.0, sx=sx, sy=sy, sz=sz, z_phase_t=self.r_ssp_start)
            pel_l = wsin(
                t, self.z_move_period,
                _Z_MOVE_PHASE + 2.0 * math.pi / self.z_move_period * self.l_ssp_start,
                self.pelvis_swing / 2.0, self.pelvis_swing / 2.0,
            )
            pel_r = wsin(
                t, self.z_move_period,
                _Z_MOVE_PHASE + 2.0 * math.pi / self.z_move_period * self.l_ssp_start,
                -self.pelvis / 2.0, -self.pelvis / 2.0,
            )
        elif t <= self.r_ssp_start:
            left = self._leg(t_x=self.l_ssp_end, t_z=self.l_ssp_end, phase_t=self.l_ssp_start, sign=1.0, sx=sx, sy=sy, sz=sz)
            right = self._leg(t_x=self.l_ssp_end, t_z=self.r_ssp_start, phase_t=self.l_ssp_start, sign=-1.0, sx=sx, sy=sy, sz=sz, z_phase_t=self.r_ssp_start)
            pel_l, pel_r = 0.0, 0.0
        elif t <= self.r_ssp_end:
            left = self._leg(t_x=t, t_z=self.l_ssp_end, phase_t=self.r_ssp_start, sign=1.0, sx=sx, sy=sy, sz=sz, extra_pi=True, z_phase_t=self.l_ssp_start)
            right = self._leg(t_x=t, t_z=t, phase_t=self.r_ssp_start, sign=-1.0, sx=sx, sy=sy, sz=sz, extra_pi=True)
            pel_l = wsin(
                t, self.z_move_period,
                _Z_MOVE_PHASE + 2.0 * math.pi / self.z_move_period * self.r_ssp_start,
                self.pelvis / 2.0, self.pelvis / 2.0,
            )
            pel_r = wsin(
                t, self.z_move_period,
                _Z_MOVE_PHASE + 2.0 * math.pi / self.z_move_period * self.r_ssp_start,
                -self.pelvis_swing / 2.0, -self.pelvis_swing / 2.0,
            )
        else:
            left = self._leg(t_x=self.r_ssp_end, t_z=self.l_ssp_end, phase_t=self.r_ssp_start, sign=1.0, sx=sx, sy=sy, sz=sz, extra_pi=True, z_phase_t=self.l_ssp_start)
            right = self._leg(t_x=self.r_ssp_end, t_z=self.r_ssp_end, phase_t=self.r_ssp_start, sign=-1.0, sx=sx, sy=sy, sz=sz, extra_pi=True)
            pel_l, pel_r = 0.0, 0.0
        return left, right, pel_l, pel_r

    def _leg(
        self,
        t_x: float,
        t_z: float,
        phase_t: float,
        sign: float,
        sx: float,
        sy: float,
        sz: float,
        extra_pi: bool = False,
        z_phase_t: float | None = None,
    ) -> tuple[float, float, float, float]:
        z_anchor = phase_t if z_phase_t is None else z_phase_t
        extra = math.pi if extra_pi else 0.0
        x_phase = _X_MOVE_PHASE + 2.0 * math.pi / max(self.x_move_period, 1e-6) * phase_t + extra
        y_phase = _Y_MOVE_PHASE + 2.0 * math.pi / max(self.y_move_period, 1e-6) * phase_t + extra
        z_phase = _Z_MOVE_PHASE + 2.0 * math.pi / max(self.z_move_period, 1e-6) * z_anchor
        a_phase = _A_MOVE_PHASE + 2.0 * math.pi / max(self.a_move_period, 1e-6) * phase_t + extra
        # Same single negation as op3_walk._leg_move. A second negation of
        # the shift same-signs both feet.
        x = wsin(t_x, self.x_move_period, x_phase, sign * self._x_move, 0.0)
        y = wsin(t_x, self.y_move_period, y_phase, sign * self._y_move, sign * self._y_move_shift)
        z = wsin(t_z, self.z_move_period, z_phase, self._z_move, self._z_move_shift)
        yaw = wsin(t_x, self.a_move_period, a_phase, sign * self._a_move, sign * self._a_move_shift)
        return sx + x, sy + y, sz + z, yaw

    def _capture_zero(self) -> None:
        left, right, _, _ = self._raw(0.0)
        self._zero_l = left
        self._zero_r = right

    def sample(self) -> tuple[FootShift, FootShift, str]:
        left, right, pel_l, pel_r = self._raw(self.time)
        fl = FootShift(
            left[0] - self._zero_l[0],
            left[1] - self._zero_l[1],
            left[2] - self._zero_l[2],
            left[3] - self._zero_l[3],
            pel_l,
            x_abs=left[0],
        )
        fr = FootShift(
            right[0] - self._zero_r[0],
            right[1] - self._zero_r[1],
            right[2] - self._zero_r[2],
            right[3] - self._zero_r[3],
            pel_r,
            x_abs=right[0],
        )
        if self.l_ssp_start < self.time <= self.l_ssp_end:
            phase = "L"
        elif self.r_ssp_start < self.time <= self.r_ssp_end:
            phase = "R"
        else:
            phase = "D"
        return fl, fr, phase

    def advance(self, dt: float) -> tuple[FootShift, FootShift, str]:
        self._phase_update(dt)
        out = self.sample()
        self.time += dt
        if self.time >= self.period - 1e-12:
            self.time = 0.0
            self.previous_x = self.x_cmd * 0.5
            self._update_movement()
            self._capture_zero()
        return out


def foot_gap(period: float, x: float, z: float, dsp: float, y_swap: float) -> tuple[float, float]:
    """Peak swing-minus-stance foot height, and swing-foot x travel, on cycle 2."""
    clock = GaitManagerClock(period_s=period, dsp=dsp, y_swap=y_swap, z_move=z, z_swap=GM_Z_SWAP_M)
    clock.set_command(x)
    clock.previous_x = x * 0.5
    clock._update_movement()
    clock._capture_zero()
    gaps: list[float] = []
    xs: list[float] = []
    n = max(2, int(round(period / 0.002)))
    for _ in range(n):
        fl, fr, phase = clock.advance(period / n)
        if phase == "L":
            gaps.append(fl.z - fr.z)
            xs.append(fl.x)
    if not gaps:
        return 0.0, 0.0
    return max(gaps), max(xs) - min(xs)


if __name__ == "__main__":
    gap, xpp = foot_gap(0.400, 0.020, 0.020, 0.20, 0.020)
    print(f"demo 400 ms swing-stance z {gap * 100:.2f} cm, swing x travel {xpp * 100:.2f} cm")
    if abs(gap - 0.020) > 0.003:
        raise SystemExit(f"foot gap {gap} is not the published 2 cm")
