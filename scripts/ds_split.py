"""Double-support splits of one planned ZMP wrench.

The net contact wrench is fixed by the LIPM force at the planned ZMP.
How that wrench is shared by the two feet is not. Knee torque follows
the share. Three shares are computed for the same wrench:

(a) force split by where the ZMP sits on the foot-to-foot line
(b) the share whose ankle torques have the smallest Euclidean norm
(c) the share that minimises the maximum |leg-joint torque| with the
    armature term removed

(c) is a linear program: minimise t subject to |τ_bare| ≤ t, each
foot's CoP inside its box, a four-sided friction pyramid of coefficient
μ, unilateral normals, and the summed wrench equal to the planned one.
The pyramid is the cone the QP can enforce with linear inequalities.
A wall is a tick where this t exceeds the sag bar. Splits (a) and (b)
do not name one.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import LinearConstraint, minimize, linprog

# Declared contact box, metres. Half-sizes, matching the plant geom.
BOX_HX_M = 0.0675
BOX_HY_M = 0.038
# Friction coefficient of the planned cone. The plant geom is 1.6.
# This is the constraint on the split, not a geom edit.
DS_FRICTION_MU = 1.2
_CORNERS = ((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0))


def pyramid_matrix(n_corners: int, mu: float) -> np.ndarray:
    """A_ub @ f <= 0. Each corner is (fx, fy, fz) in the sole frame."""
    rows = []
    mu = float(mu)
    for i in range(n_corners):
        b = 3 * i
        # fz >= 0
        row = np.zeros(3 * n_corners)
        row[b + 2] = -1.0
        rows.append(row)
        for axis in (0, 1):
            for sign in (1.0, -1.0):
                row = np.zeros(3 * n_corners)
                row[b + axis] = sign
                row[b + 2] = -mu
                rows.append(row)
    return np.vstack(rows)


_A_UB = pyramid_matrix(8, DS_FRICTION_MU)


def corner_wrench_map(rot: np.ndarray, hx: float, hy: float) -> np.ndarray:
    """Map 4 sole-frame corner forces to a world wrench at the bottom centre.

    Returns (6, 12). The wrench is [force, torque] in world frame.
    """
    blocks = []
    rot = np.asarray(rot, dtype=np.float64).reshape(3, 3)
    for sx, sy in _CORNERS:
        r = np.array([sx * float(hx), sy * float(hy), 0.0], dtype=np.float64)
        skew = np.array(
            [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]],
            dtype=np.float64,
        )
        block = np.zeros((6, 3), dtype=np.float64)
        block[:3, :] = rot
        block[3:, :] = rot @ skew
        blocks.append(block)
    return np.concatenate(blocks, axis=1)


def _clamp_cop(local_xy: np.ndarray, hx: float, hy: float) -> np.ndarray:
    out = np.array(local_xy, dtype=np.float64, copy=True)
    out[0] = min(float(hx), max(-float(hx), float(out[0])))
    out[1] = min(float(hy), max(-float(hy), float(out[1])))
    return out


def linear_split(
    centers: list[np.ndarray],
    rots: list[np.ndarray],
    halves: list[tuple[float, float]],
    p_zmp: np.ndarray,
    force: np.ndarray,
) -> dict[str, np.ndarray | float]:
    """Split ``force`` by the ZMP's position on the foot-to-foot line.

    The CoP of each share is the minimum-offset pair whose force-weighted
    position is the ZMP, then clamped into that foot's box. Clamping can
    leave a residual, which is part of what this split is.
    """
    c_l = np.asarray(centers[0], dtype=np.float64)
    c_r = np.asarray(centers[1], dtype=np.float64)
    d = c_l[:2] - c_r[:2]
    den = float(d @ d)
    if den < 1e-10:
        alpha = 0.5
    else:
        alpha = float((np.asarray(p_zmp[:2]) - c_r[:2]) @ d / den)
        alpha = min(1.0, max(0.0, alpha))
    c_mid = alpha * c_l[:2] + (1.0 - alpha) * c_r[:2]
    # Minimum-offset CoPs. w = α² + (1-α)².
    w = alpha * alpha + (1.0 - alpha) * (1.0 - alpha)
    if w < 1e-12:
        shift_l = np.zeros(2)
        shift_r = np.zeros(2)
    else:
        gap = c_mid - np.asarray(p_zmp[:2], dtype=np.float64)
        shift_l = -alpha * gap / w
        shift_r = -(1.0 - alpha) * gap / w
    forces = (float(alpha) * force, (1.0 - alpha) * force)
    wrenches = []
    cops = []
    weighted = np.zeros(2, dtype=np.float64)
    weight = 0.0
    for center, rot, half, share, shift in (
        (c_l, rots[0], halves[0], forces[0], shift_l),
        (c_r, rots[1], halves[1], forces[1], shift_r),
    ):
        rot = np.asarray(rot, dtype=np.float64).reshape(3, 3)
        local = rot.T @ np.array([shift[0], shift[1], 0.0], dtype=np.float64)
        cop = _clamp_cop(local[:2], half[0], half[1])
        cops.append(cop)
        r_w = rot @ np.array([cop[0], cop[1], 0.0], dtype=np.float64)
        torque = np.cross(r_w, share)
        wrenches.append(np.concatenate([share, torque]))
        # Weight by vertical force so a zero share does not pull the CoP.
        fz = float(share[2])
        weighted += fz * (center[:2] + r_w[:2])
        weight += fz
    if weight > 1e-8:
        cop_net = weighted / weight
    else:
        cop_net = c_mid
    residual = float(np.linalg.norm(cop_net - np.asarray(p_zmp[:2])))
    return {
        "alpha": alpha,
        "wrenches": wrenches,
        "cops": cops,
        "residual_m": residual,
    }


def barycentric_forces(
    linear: dict,
    rots: list[np.ndarray],
    halves: list[tuple[float, float]],
) -> np.ndarray:
    """Corner forces that reproduce the linear split, for a warm start."""
    x = np.zeros(24, dtype=np.float64)
    for foot, (rot, half, wrench) in enumerate(zip(rots, halves, linear["wrenches"])):
        rot = np.asarray(rot, dtype=np.float64).reshape(3, 3)
        f_w = np.asarray(wrench[:3], dtype=np.float64)
        f_s = rot.T @ f_w
        cop = np.asarray(linear["cops"][foot], dtype=np.float64)
        hx = max(float(half[0]), 1e-6)
        hy = max(float(half[1]), 1e-6)
        u = min(1.0, max(0.0, 0.5 * (float(cop[0]) / hx + 1.0)))
        v = min(1.0, max(0.0, 0.5 * (float(cop[1]) / hy + 1.0)))
        weights = (u * v, u * (1.0 - v), (1.0 - u) * v, (1.0 - u) * (1.0 - v))
        for k, wgt in enumerate(weights):
            x[12 * foot + 3 * k : 12 * foot + 3 * k + 3] = wgt * f_s
    return x


def solve_minimax(
    J: np.ndarray,
    bare0: np.ndarray,
    G: np.ndarray,
    target: np.ndarray,
    x0: np.ndarray | None = None,
) -> tuple[np.ndarray | None, float]:
    """Minimise the maximum |bare torque|. Returns (x, t) or (None, inf)."""
    n = int(J.shape[1])
    n_j = int(J.shape[0])
    # Variables [x (n), t].
    c = np.zeros(n + 1, dtype=np.float64)
    c[-1] = 1.0
    # |bare0 - J x| <= t
    a_ub = np.zeros((2 * n_j + _A_UB.shape[0], n + 1), dtype=np.float64)
    b_ub = np.zeros(a_ub.shape[0], dtype=np.float64)
    a_ub[:n_j, :n] = -J
    a_ub[:n_j, -1] = -1.0
    b_ub[:n_j] = -bare0
    a_ub[n_j : 2 * n_j, :n] = J
    a_ub[n_j : 2 * n_j, -1] = -1.0
    b_ub[n_j : 2 * n_j] = bare0
    a_ub[2 * n_j :, :n] = _A_UB
    a_eq = np.zeros((G.shape[0], n + 1), dtype=np.float64)
    a_eq[:, :n] = G
    res = linprog(
        c, A_ub=a_ub, b_ub=b_ub, A_eq=a_eq, b_eq=np.asarray(target, dtype=np.float64),
        bounds=[(None, None)] * n + [(0.0, None)], method="highs",
    )
    if not res.success or res.x is None:
        return None, float("inf")
    return np.asarray(res.x[:n], dtype=np.float64), float(res.x[-1])


def solve_ankle_norm(
    J_ank: np.ndarray,
    bare_ank: np.ndarray,
    G: np.ndarray,
    target: np.ndarray,
    x0: np.ndarray,
) -> np.ndarray | None:
    """Minimum Euclidean ankle torque inside the same cone and wrench."""
    n = int(J_ank.shape[1])
    H = J_ank.T @ J_ank
    # A hair of force regularisation picks one solution in the ankle nullspace.
    H = H + 1e-8 * np.eye(n)
    rhs = J_ank.T @ np.asarray(bare_ank, dtype=np.float64)
    kkt_top = np.concatenate([H, G.T], axis=1)
    kkt_bot = np.concatenate([G, np.zeros((G.shape[0], G.shape[0]))], axis=1)
    kkt = np.concatenate([kkt_top, kkt_bot], axis=0)
    sol = np.concatenate([rhs, np.asarray(target, dtype=np.float64)])
    try:
        ans = np.linalg.solve(kkt, sol)
        x_eq = ans[:n]
    except np.linalg.LinAlgError:
        x_eq = np.array(x0, dtype=np.float64, copy=True)
    if np.all(_A_UB @ x_eq <= 1e-7) and np.linalg.norm(G @ x_eq - target) <= 1e-5:
        return x_eq

    def cost(x: np.ndarray) -> float:
        err = bare_ank - J_ank @ x
        return float(err @ err + 1e-8 * (x @ x))

    def grad(x: np.ndarray) -> np.ndarray:
        err = bare_ank - J_ank @ x
        return -2.0 * (J_ank.T @ err) + 2e-8 * x

    eq = LinearConstraint(G, target, target)
    ineq = LinearConstraint(_A_UB, -np.inf * np.ones(_A_UB.shape[0]), np.zeros(_A_UB.shape[0]))
    res = minimize(
        cost, x_eq, jac=grad, method="trust-constr",
        constraints=[eq, ineq],
        options={"maxiter": 60, "gtol": 1e-8, "verbose": 0},
    )
    if not res.success and np.linalg.norm(G @ res.x - target) > 1e-4:
        return None
    return np.asarray(res.x, dtype=np.float64)
