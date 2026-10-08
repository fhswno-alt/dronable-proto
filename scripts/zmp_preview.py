#!/usr/bin/env python3
"""Lateral cart-table preview. CoM y from a ZMP reference. No plant write.

Kajita preview control on the 3-state cart table. The input is CoM jerk.
The ZMP reference is the only trajectory this file owns. Joint targets
stay in the walker. y_swap is not this channel.
"""
from __future__ import annotations

import math

import numpy as np

G = 9.81
# Zero-pose contact box, plant cold. Not a measured fit and not a plant edit.
# Centres are ±0.043 m in world y. Each box runs from ±0.005 m to ±0.081 m,
# so 5 mm off the midline is the inner edge (margin 0). The box centre is
# the 43 mm lateral sway. Fore-aft trunk CoM is already inside the box.
BOX_CENTER_Y_M = 0.0430
INNER_EDGE_Y_M = 0.0050
# Past the inner edge, so the margin is positive when the foot is allowed to lift.
ENTRY_Y_M = 0.0080


class ZmpPreview:
    """One lateral axis. ``step`` returns the CoM position in metres."""

    def __init__(
        self, zc_m: float, dt_s: float, horizon: int, r_weight: float = 1.0e-4,
    ) -> None:
        if zc_m < 0.05:
            raise ValueError(f"preview height {zc_m} m is below 0.05 m")
        if dt_s <= 0.0:
            raise ValueError(f"preview dt {dt_s} s is not positive")
        if horizon < 8:
            raise ValueError(f"preview horizon {horizon} is below 8")
        if r_weight <= 0.0:
            raise ValueError(f"preview R {r_weight} is not positive")
        self.zc_m = float(zc_m)
        self.dt_s = float(dt_s)
        self.horizon = int(horizon)
        self.r_weight = float(r_weight)
        a_state, b_state, c_state, k_gain, f_gain = _preview_gains(
            self.zc_m, self.dt_s, self.horizon, self.r_weight,
        )
        self._a = a_state
        self._b = b_state
        self._c = c_state
        self._k = k_gain
        self._f = f_gain
        self._x = np.zeros(3, dtype=np.float64)
        self._e = 0.0

    def reset(self) -> None:
        self._x[:] = 0.0
        self._e = 0.0

    @property
    def com_m(self) -> float:
        return float(self._x[0])

    @property
    def com_vel_m_s(self) -> float:
        return float(self._x[1])

    @property
    def com_acc_m_s2(self) -> float:
        return float(self._x[2])

    def step(self, zmp_future_m: np.ndarray) -> float:
        """Advance one tick. ``zmp_future_m[0]`` is the ZMP due now."""
        ref = np.asarray(zmp_future_m, dtype=np.float64).reshape(-1)
        if ref.shape[0] != self.horizon:
            raise ValueError(
                f"ZMP future length {ref.shape[0]} != horizon {self.horizon}"
            )
        state = np.zeros(4, dtype=np.float64)
        state[0] = self._e
        state[1:] = self._x
        jerk = float(-self._k @ state + self._f @ ref)
        self._x = self._a @ self._x + self._b * jerk
        # Tracking error uses the ZMP that was due on this tick.
        self._e = self._e + float(self._c @ self._x) - float(ref[0])
        return float(self._x[0])


def _preview_gains(
    zc_m: float, dt_s: float, horizon: int, r_weight: float = 1.0e-4,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Discrete gains. Q weights ZMP error. R weights jerk."""
    a_state = np.array(
        [
            [1.0, dt_s, 0.5 * dt_s * dt_s],
            [0.0, 1.0, dt_s],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    b_state = np.array(
        [dt_s ** 3 / 6.0, 0.5 * dt_s * dt_s, dt_s],
        dtype=np.float64,
    )
    c_state = np.array([1.0, 0.0, -zc_m / G], dtype=np.float64)
    a_aug = np.zeros((4, 4), dtype=np.float64)
    b_aug = np.zeros(4, dtype=np.float64)
    a_aug[0, 0] = 1.0
    a_aug[0, 1:] = c_state @ a_state
    b_aug[0] = float(c_state @ b_state)
    a_aug[1:, 1:] = a_state
    b_aug[1:] = b_state
    q_weight = 1.0
    q_aug = np.zeros((4, 4), dtype=np.float64)
    q_aug[0, 0] = q_weight
    p_riccati = q_aug.copy()
    for _ in range(4000):
        bt_p = b_aug @ p_riccati
        s_inv = r_weight + float(bt_p @ b_aug)
        k_iter = (bt_p @ a_aug) / s_inv
        p_next = a_aug.T @ p_riccati @ (a_aug - np.outer(b_aug, k_iter)) + q_aug
        if float(np.max(np.abs(p_next - p_riccati))) < 1e-12:
            p_riccati = p_next
            break
        p_riccati = p_next
    bt_p = b_aug @ p_riccati
    s_inv = r_weight + float(bt_p @ b_aug)
    k_gain = (bt_p @ a_aug) / s_inv
    a_closed = a_aug - np.outer(b_aug, k_gain)
    # Future ZMP enters the error state with a +1, so the preview
    # vector is built from that column of P.
    carry = p_riccati[:, 0].copy()
    f_gain = np.zeros(horizon, dtype=np.float64)
    for i in range(horizon):
        f_gain[i] = float(b_aug @ carry) / s_inv
        carry = a_closed.T @ carry
    return a_state, b_state, c_state, k_gain, f_gain


def track_constant(zc_m: float, dt_s: float, target_m: float, seconds: float) -> float:
    """CoM after a constant ZMP. Used to check the gain sign."""
    horizon = max(8, int(round(1.6 / dt_s)))
    preview = ZmpPreview(zc_m, dt_s, horizon)
    future = np.full(horizon, float(target_m), dtype=np.float64)
    com = 0.0
    n = int(round(seconds / dt_s))
    for _ in range(n):
        com = preview.step(future)
    return com


def cycle_zmp(
    t_s: float,
    period_s: float,
    l_ssp_start_s: float,
    l_ssp_end_s: float,
    r_ssp_start_s: float,
    r_ssp_end_s: float,
    amp_m: float,
    shape: float = 1.0,
) -> float:
    """ZMP y over one walk cycle. +y is toward the left foot.

    Left swing (right stance) holds ``-amp``. Right swing holds ``+amp``.
    Double support is a raised cosine between those holds. The value is
    already at ``-amp`` through the opening double support, which is the
    first stance before the left foot lifts.
    """
    period = float(period_s)
    if period <= 1e-6:
        return 0.0
    t = float(t_s) % period
    amp = float(amp_m)
    l0 = float(l_ssp_start_s)
    l1 = float(l_ssp_end_s)
    r0 = float(r_ssp_start_s)
    r1 = float(r_ssp_end_s)
    if l0 < t <= l1:
        return -amp
    if r0 < t <= r1:
        return amp
    if l1 < t <= r0:
        return _cosine(t, l1, r0, -amp, amp, shape)
    # Wrap-around double support: after the right swing, back to the right foot.
    span = (period - r1) + l0
    if span <= 1e-9:
        return -amp
    if t > r1:
        u_time = t - r1
    else:
        u_time = (period - r1) + t
    return _cosine_u(u_time / span, amp, -amp, shape)


def sway_zmp_amp(
    period_s: float,
    dsp: float,
    zc_m: float,
    com_target_m: float,
    zmp_cap_m: float = BOX_CENTER_Y_M,
    shape: float = 0.0,
    r_weight: float = 1.0e-4,
) -> float:
    """ZMP amplitude whose steady CoM peak is ``com_target_m``.

    The cap is the foot-box centre. A short single support does not
    carry the CoM out to that centre: the preview orbit is the sway,
    and a smaller ZMP reference is used when the orbit would overshoot
    the target. If the cap still falls short of the target, the cap is
    the amplitude.
    """
    cap = max(0.0, float(zmp_cap_m))
    target = max(0.0, float(com_target_m))
    if cap <= 1e-6:
        return 0.0

    def peak(amp: float) -> float:
        period = float(period_s)
        ssp = 1.0 - min(0.95, max(0.0, float(dsp)))
        l0 = (1.0 - ssp) * period / 4.0
        l1 = (1.0 + ssp) * period / 4.0
        r0 = (3.0 - ssp) * period / 4.0
        r1 = (3.0 + ssp) * period / 4.0
        dt = 0.008
        horizon = max(8, int(round(1.6 / dt)))
        preview = ZmpPreview(float(zc_m), dt, horizon, float(r_weight))
        n = int(round(max(3.0, 4.0 * period) / dt))
        hold = max(1, int(round(2.0 * period / dt)))
        acc = 0.0
        for i in range(n):
            t = i * dt
            future = np.empty(horizon, dtype=np.float64)
            for k in range(horizon):
                future[k] = cycle_zmp(
                    t + k * dt, period, l0, l1, r0, r1, amp, shape,
                )
            com = abs(preview.step(future))
            if i >= n - hold:
                acc = max(acc, com)
        return acc

    if peak(cap) <= target:
        return cap
    lo = 0.0
    hi = cap
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        if peak(mid) < target:
            lo = mid
        else:
            hi = mid
    return hi


def arm_zmp(t_before_s: float, arm_s: float, amp_m: float) -> float:
    """ZMP during the pre-lift shift. ``t_before_s`` is negative and ends at 0.

    The shift finishes on the first stance foot (right, ``-amp``) before
    the clock starts.
    """
    arm = max(float(arm_s), 1e-6)
    u = 1.0 - min(1.0, max(0.0, -float(t_before_s) / arm))
    s = u * u * (3.0 - 2.0 * u)
    return -float(amp_m) * s


def _cosine(
    t_s: float, t0_s: float, t1_s: float, y0_m: float, y1_m: float, shape: float = 1.0,
) -> float:
    span = float(t1_s) - float(t0_s)
    if span <= 1e-9:
        return float(y1_m)
    u = min(1.0, max(0.0, (float(t_s) - float(t0_s)) / span))
    return _cosine_u(u, y0_m, y1_m, shape)


def _cosine_u(u: float, y0_m: float, y1_m: float, shape: float = 1.0) -> float:
    """Blend a ramp and a raised cosine. ``shape`` 1 is the cosine.

    The cosine peaks at pi/2 times the mean rate. A lower shape spends
    more of the double support at a steady rate, so the hip-roll peak
    drops. 0 is a straight ramp.
    """
    u_clamped = min(1.0, max(0.0, u))
    cosine = 0.5 - 0.5 * math.cos(math.pi * u_clamped)
    blend = min(1.0, max(0.0, float(shape)))
    w = (1.0 - blend) * u_clamped + blend * cosine
    return float(y0_m) + (float(y1_m) - float(y0_m)) * w


if __name__ == "__main__":
    got = track_constant(0.22, 0.008, 0.018, 2.0)
    print(f"constant 0.018 -> com {got:+.6f} m")
