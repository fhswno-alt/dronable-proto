#!/usr/bin/env python3
"""Full-pixel floor ray and foot-corridor hazard. Estimator only.

Controls latches Day-1 stop. This module does not call ``CommandBus``.
It does not command ``head_tilt``. Soft-pass is off. The plant file is
not read and not edited.

The ray starts at a kit_cam pixel ``(u, v)``. Intrinsics are the frozen
camera: 640×480, fovy 104.82, square pixels. The direction is rotated by
the full camera attitude built from body yaw, IMU roll, IMU pitch, and
``head_tilt`` (and ``head_pan`` if it is not zero). Pitch is the torso
IMU pitch: negative is nose-down. The ray intersects the floor plane
``z = 0``.

``eye_range`` is the horizontal distance from the camera to that hit.
``forward_m`` is the same hit in the body frame, ahead of the camera.
``sideways_m`` is the hit's lateral offset from the body origin, body
left positive. ``toe_gap_m`` subtracts the frozen hazard pad and the
caller-supplied step offset:

    toe_gap_m = forward_m − hazard_pad − step_off

``step_off`` is the period high-water toe ahead of the camera. On this
kitchen tip that high water is +0.017 m on the right foot at t ≈ 3.056 s.
A caller that only has ``eye_range`` applies that +0.017 m itself and
still does not have the corridor or the pad. Do not subtract it twice.

The hazard pad is a constant frozen before the run. It inflates the
hit toward the robot (approach) and sideways. It is not a ``col_*``
radius and it is not read from any geom. 20 mm is a stand-in for the
near edge of a stool leg. It is not a measured kit radius and it is
not a sized buffer.

The foot corridor uses the Controls stand measurement of the outer
``l_foot_contact`` / ``r_foot_contact`` edges after the 14 mm outboard
shift already in the plant: ±0.0867 m. The inboard edges are ±0.0096 m
and are not a hole in the corridor. At commanded yaw ±0.25 the outside
edge widens to 0.108 m (stand outer 0.0867 plus the 0.021 m outside-step
sweep). Kitchen yaw −0.25 puts that widen on the left.

``T_stop`` = 0.842 s is the kitchen body-COM settle used in
``d_min = 0.150 × (T_detect + T_stop)``. ``T_detect`` and ``buffer`` stay
parameters. This module does not apply a buffer and does not pick a kit
``T_detect``. ``t_cue`` is not ``T_detect``. A −10° head-down walk is not
enabled here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

WIDTH = 640
HEIGHT = 480
FOVY_DEG = 104.82
V_MPS = 0.150
# Kitchen body-COM settle. Formula sample, not a new measurement.
T_STOP_S = 0.842
# Frozen before the run. Not a col_* half-extent and not loaded from XML.
HAZARD_PAD_M = 0.020
# Outer edges of the planted soles, body frame, metres. Left is +Y.
CORRIDOR_OUTER_M = 0.0867
CORRIDOR_INBOARD_M = 0.0096
CORRIDOR_OUTSIDE_M = 0.108
YAW_CAP = 0.25
# kit_cam xyaxes "0 -1 0 0 0 1". Columns are camera axes in the head frame.
# Look direction is −Z of the camera, which is +X of the head.
_MOUNT = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


def d_min(t_detect_s: float, t_stop_s: float = T_STOP_S) -> float:
    """Clear distance at commanded 0.150 m/s. ``t_detect_s`` is not filled in."""
    return V_MPS * (float(t_detect_s) + float(t_stop_s))


def focal_px() -> float:
    """Vertical focal length in pixels. Square pixels, so fx = fy."""
    return (HEIGHT / 2.0) / math.tan(math.radians(FOVY_DEG) / 2.0)


def _rot_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
        dtype=np.float64,
    )


def _rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
        dtype=np.float64,
    )


def _rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _rot_axis(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    ax = np.array(axis, dtype=np.float64)
    norm = float(np.linalg.norm(ax))
    x, y, z = (float(v) for v in (ax / norm))
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return np.array(
        [
            [c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
        ],
        dtype=np.float64,
    )


def imu_from_body(rotation: np.ndarray) -> tuple[float, float, float]:
    """Yaw, pitch, roll of a body rotation. Pitch is negative nose-down."""
    rot = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    forward = rot[:, 0]
    left = rot[:, 1]
    yaw = math.atan2(float(forward[1]), float(forward[0]))
    pitch = math.atan2(float(forward[2]), float(math.hypot(forward[0], forward[1])))
    roll = math.atan2(float(left[2]), float(rot[2, 2]))
    return yaw, pitch, roll


def body_rotation(yaw_rad: float, pitch_rad: float, roll_rad: float) -> np.ndarray:
    """Body rotation from yaw and IMU pitch/roll. Pitch is negative nose-down."""
    return _rot_z(yaw_rad) @ _rot_y(-pitch_rad) @ _rot_x(roll_rad)


def camera_rotation_from_imu(
    yaw_rad: float,
    pitch_rad: float,
    roll_rad: float,
    head_tilt_rad: float,
    head_pan_rad: float = 0.0,
) -> np.ndarray:
    """kit_cam world rotation. Columns are camera X, Y, Z in the world.

    Roll and pitch both enter. ``head_tilt`` is the head joint, added on
    the head's pitch axis. This does not command that joint.
    """
    body = body_rotation(yaw_rad, pitch_rad, roll_rad)
    pan = _rot_axis((0.0, 0.0, -1.0), head_pan_rad)
    tilt = _rot_axis((0.0, -1.0, 0.0), head_tilt_rad)
    return body @ pan @ tilt @ _MOUNT


def pixel_direction(u: float, v: float) -> np.ndarray:
    """Unit ray in the camera frame. +X right, +Y up, look along −Z."""
    f = focal_px()
    cx = WIDTH / 2.0
    cy = HEIGHT / 2.0
    direction = np.array(
        [(u - cx) / f, -(v - cy) / f, -1.0],
        dtype=np.float64,
    )
    return direction / float(np.linalg.norm(direction))


def project_point(
    point_m: np.ndarray,
    cam_pos_m: np.ndarray,
    cam_rot: np.ndarray,
) -> tuple[float, float] | None:
    """Pixel of a world point. None when it is behind the camera."""
    local = np.asarray(cam_rot, dtype=np.float64).reshape(3, 3).T @ (
        np.asarray(point_m, dtype=np.float64) - np.asarray(cam_pos_m, dtype=np.float64)
    )
    depth = -float(local[2])
    if depth <= 1e-6:
        return None
    f = focal_px()
    u = (WIDTH / 2.0) + f * (float(local[0]) / depth)
    v = (HEIGHT / 2.0) - f * (float(local[1]) / depth)
    return u, v


def ray_floor_hit(
    u: float,
    v: float,
    cam_pos_m: np.ndarray,
    cam_rot: np.ndarray,
) -> np.ndarray | None:
    """World point where the pixel ray meets z = 0. None if it misses the floor."""
    origin = np.asarray(cam_pos_m, dtype=np.float64)
    direction = np.asarray(cam_rot, dtype=np.float64).reshape(3, 3) @ pixel_direction(u, v)
    if float(direction[2]) >= -1e-8:
        return None
    travel = -float(origin[2]) / float(direction[2])
    if travel <= 0.0:
        return None
    return origin + travel * direction


def left_from_forward(forward_xy: tuple[float, float]) -> tuple[float, float]:
    """Body left in the ground plane. +Y when the robot faces +X."""
    return (-forward_xy[1], forward_xy[0])


@dataclass(frozen=True)
class KitCamPose:
    """Optical centre. Attitude is rebuilt from the IMU, not from this pose."""

    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class BodyFrame:
    """Body origin and horizontal forward. Left is derived."""

    origin_xy_m: tuple[float, float]
    forward_xy: tuple[float, float]


@dataclass(frozen=True)
class CorridorEdges:
    left_outer_m: float
    right_outer_m: float
    inboard_m: float
    yaw_rate: float
    outside: str


@dataclass(frozen=True)
class HazardEstimate:
    toe_gap_m: float
    eye_range: float
    forward_m: float
    sideways_m: float
    approach_m: float
    in_corridor: bool
    hazard_pad_m: float
    step_off_m: float
    hit_xy_m: tuple[float, float]


@dataclass(frozen=True)
class DeadReckonState:
    """Last in-frame hit. Sideways is not updated after the pixel leaves."""

    forward_m: float
    sideways_m: float
    in_corridor: bool
    origin_xy_m: tuple[float, float]
    forward_xy: tuple[float, float]
    t_s: float


def corridor_edges(yaw_rate: float) -> CorridorEdges:
    """Outer half-widths. ±0.25 widens only the outside edge to 0.108 m."""
    left = CORRIDOR_OUTER_M
    right = CORRIDOR_OUTER_M
    outside = "none"
    if yaw_rate <= -YAW_CAP + 1e-6:
        left = CORRIDOR_OUTSIDE_M
        outside = "L"
    elif yaw_rate >= YAW_CAP - 1e-6:
        right = CORRIDOR_OUTSIDE_M
        outside = "R"
    return CorridorEdges(left, right, CORRIDOR_INBOARD_M, float(yaw_rate), outside)


def in_foot_corridor(sideways_m: float, edges: CorridorEdges, pad_m: float) -> bool:
    """True when the pad-inflated lateral interval meets the corridor."""
    lo = sideways_m - pad_m
    hi = sideways_m + pad_m
    right_edge = -edges.right_outer_m
    left_edge = edges.left_outer_m
    return hi >= right_edge and lo <= left_edge


def estimate_hazard(
    u: float,
    v: float,
    *,
    cam: KitCamPose,
    body: BodyFrame,
    imu_roll_rad: float,
    imu_pitch_rad: float,
    head_tilt_rad: float,
    yaw_rate: float,
    step_off_m: float,
    hazard_pad_m: float = HAZARD_PAD_M,
    head_pan_rad: float = 0.0,
) -> HazardEstimate | None:
    """Ray plus corridor. Returns None when the pixel misses the floor.

    The attitude is IMU roll and pitch plus ``head_tilt``. A pose matrix
    is not accepted, so a pitch-only model cannot be substituted.
    ``hazard_pad_m`` defaults to the frozen stand-in. Pass that same
    constant; do not pass a collider size.
    """
    fx, fy = body.forward_xy
    norm = math.hypot(fx, fy)
    if norm < 1e-9:
        return None
    forward_xy = (fx / norm, fy / norm)
    left_xy = left_from_forward(forward_xy)
    yaw = math.atan2(forward_xy[1], forward_xy[0])
    rotation = camera_rotation_from_imu(
        yaw, imu_pitch_rad, imu_roll_rad, head_tilt_rad, head_pan_rad,
    )
    origin = np.array(cam.position_m, dtype=np.float64)
    hit = ray_floor_hit(u, v, origin, rotation)
    if hit is None:
        return None
    delta_cam = hit[:2] - origin[:2]
    forward_m = float(delta_cam[0] * forward_xy[0] + delta_cam[1] * forward_xy[1])
    delta_body = hit[:2] - np.array(body.origin_xy_m, dtype=np.float64)
    sideways_m = float(delta_body[0] * left_xy[0] + delta_body[1] * left_xy[1])
    eye_range = float(math.hypot(float(delta_cam[0]), float(delta_cam[1])))
    approach_m = forward_m - float(hazard_pad_m)
    edges = corridor_edges(yaw_rate)
    inside = in_foot_corridor(sideways_m, edges, float(hazard_pad_m))
    return HazardEstimate(
        toe_gap_m=approach_m - float(step_off_m),
        eye_range=eye_range,
        forward_m=forward_m,
        sideways_m=sideways_m,
        approach_m=approach_m,
        in_corridor=inside,
        hazard_pad_m=float(hazard_pad_m),
        step_off_m=float(step_off_m),
        hit_xy_m=(float(hit[0]), float(hit[1])),
    )


def dead_reckon_gap(
    state: DeadReckonState,
    origin_xy_now: tuple[float, float],
    step_off_m: float,
    hazard_pad_m: float = HAZARD_PAD_M,
) -> float:
    """Forward gap after the pixel leaves, using the heading stored at leave.

    Turning after that frame is not folded into ``sideways_m``. The
    corridor flag stays the one from the last in-frame hit.
    """
    dx = origin_xy_now[0] - state.origin_xy_m[0]
    dy = origin_xy_now[1] - state.origin_xy_m[1]
    advance = dx * state.forward_xy[0] + dy * state.forward_xy[1]
    return state.forward_m - float(hazard_pad_m) - advance - float(step_off_m)


def _assert_close(name: str, got: float, want: float, tol: float = 1e-6) -> None:
    if abs(got - want) > tol:
        raise SystemExit(f"self_check {name}: {got} != {want}")


def self_check() -> None:
    """Closed-form ray, corridor, and pad. No plant and no collider size."""
    yaw, pitch, roll = 0.40, -0.22, 0.07
    tilt = -0.03
    rot = camera_rotation_from_imu(yaw, pitch, roll, tilt)
    back = imu_from_body(body_rotation(yaw, pitch, roll))
    _assert_close("yaw", back[0], yaw, 1e-9)
    _assert_close("pitch", back[1], pitch, 1e-9)
    _assert_close("roll", back[2], roll, 1e-9)
    cam = np.array([0.15, -0.04, 0.335], dtype=np.float64)
    target = np.array([0.85, 0.22, 0.0], dtype=np.float64)
    pix = project_point(target, cam, rot)
    if pix is None:
        raise SystemExit("self_check project missed")
    hit = ray_floor_hit(pix[0], pix[1], cam, rot)
    if hit is None:
        raise SystemExit("self_check ray missed")
    if float(np.max(np.abs(hit - target))) > 1e-6:
        raise SystemExit(f"self_check ray {hit} != {target}")
    forward = (math.cos(yaw), math.sin(yaw))
    body = BodyFrame((0.10, -0.02), forward)
    est = estimate_hazard(
        pix[0],
        pix[1],
        cam=KitCamPose((float(cam[0]), float(cam[1]), float(cam[2]))),
        body=body,
        imu_roll_rad=roll,
        imu_pitch_rad=pitch,
        head_tilt_rad=tilt,
        yaw_rate=0.0,
        step_off_m=0.017,
    )
    if est is None:
        raise SystemExit("self_check estimate missed")
    if abs(est.hazard_pad_m - HAZARD_PAD_M) > 1e-12:
        raise SystemExit("self_check pad was replaced")
    _assert_close("toe", est.toe_gap_m, est.forward_m - HAZARD_PAD_M - 0.017)
    _assert_close("eye", est.eye_range, math.hypot(target[0] - cam[0], target[1] - cam[1]))
    straight = corridor_edges(0.0)
    right_turn = corridor_edges(-0.25)
    left_turn = corridor_edges(0.25)
    if straight.outside != "none" or abs(straight.left_outer_m - 0.0867) > 1e-12:
        raise SystemExit("self_check straight corridor")
    if right_turn.outside != "L" or abs(right_turn.left_outer_m - 0.108) > 1e-12:
        raise SystemExit("self_check kitchen outside")
    if abs(right_turn.right_outer_m - 0.0867) > 1e-12:
        raise SystemExit("self_check kitchen inside edge moved")
    if left_turn.outside != "R" or abs(left_turn.right_outer_m - 0.108) > 1e-12:
        raise SystemExit("self_check left turn outside")
    if in_foot_corridor(0.10, straight, 0.0):
        raise SystemExit("self_check 0.10 m should sit outside ±0.0867")
    if not in_foot_corridor(0.10, right_turn, 0.0):
        raise SystemExit("self_check 0.10 m should sit inside the left 0.108")
    if not in_foot_corridor(0.10, straight, 0.020):
        raise SystemExit("self_check pad should pull 0.10 m into ±0.0867")
    if in_foot_corridor(0.20, straight, HAZARD_PAD_M):
        raise SystemExit("self_check 0.20 m is not a foot hazard")
    state = DeadReckonState(0.40, 0.05, True, (0.0, 0.0), (1.0, 0.0), 1.0)
    gap = dead_reckon_gap(state, (0.10, 0.0), 0.017)
    _assert_close("dead", gap, 0.40 - HAZARD_PAD_M - 0.10 - 0.017)
    _assert_close("d_min0", d_min(0.0), 0.1263, 1e-6)


if __name__ == "__main__":
    self_check()
    print(
        f"self_check ok pad={HAZARD_PAD_M:.3f} "
        f"outer=±{CORRIDOR_OUTER_M:.4f} outside={CORRIDOR_OUTSIDE_M:.3f} "
        f"inboard=±{CORRIDOR_INBOARD_M:.4f} d_min(T_detect=0)={d_min(0.0):.4f}"
    )
