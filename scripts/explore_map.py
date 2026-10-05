#!/usr/bin/env python3
"""Explore a space and build a partial map from kit_cam.

This is not a room finder and not go-anywhere. The only commands are
stand, stop, and vel(vx, yaw_rate) on CommandBus in scripts/steer_walk.py,
resent at 10 Hz. Caps stay +0.056 / -0.032 m/s and yaw ±0.25 rad/s.
On the empty plant the walk is the claimed nav-multi chain:
vel(+0.056, +0.25) for 12.5 s, then vel(+0.056, -0.25) for 11 s,
after a 1.0 s stand so the left arc starts at t = 16 s. A 0.6 s
stand starts that arc early and the left hold only reaches about
+35 deg. Furnished scenes cannot hold those windows (up_z crosses
0.90). They use the same vel(+0.056, -0.25) for 8 s, right first.
A short vel(+0.028, +0.25) left command is swallowed here. vx=0
yaw does not change heading either, so this script never sends it.

The plant file is not edited. kit_cam stays on head_tilt_link at
0.050 0.019 0.007. Scenes are vision only (room_kitchen.xml and the
other room includes on main). Body names are not read. Nothing here
is a waypoint, an arrival, or a map of pre-placed rooms.

What the map actually is:
  - World-frame cells (0.10 m) painted when a kit_cam ray meets the
    floor plane inside 0.30–1.80 m. Low-saturation hits are free.
    Saturated floor hits (a mat, for example) are feature cells.
  - Elevated color, including the kitchen backsplash, does not meet
    that plane in range. It is stored as a camera-ray bearing, not a
    cell. query_kitchen_like_yellow() reports that bearing.
  - Frontier cells are unknown cells next to a free or walked cell.
    The next vel aims at one of those cells. It does not aim at yellow.
  - The walked trail uses the sim freejoint. That is odometry in this
    sim, not visual SLAM.

Prefer FAIL: a longer floor fan, plus a bearing if yellow was in
frame, is not a house map and not an arrival. The last-mile finder
may query query_kitchen_like_yellow() and frontier_cells(). That
query is not a waypoint.

Run:
  MUJOCO_GL=osmesa python scripts/explore_map.py --self-test
  MUJOCO_GL=osmesa python scripts/explore_map.py --demo
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

if "--view" not in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import steer_walk  # noqa: E402
import walk_gait_ainex as wg  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = steer_walk.PLANT_XML
PREVIEWS = ROOT / "previews"
ARTIFACTS = Path("/opt/cursor/artifacts")

WIDTH = 640
HEIGHT = 480
# 1.0 s is the claimed nav-multi stand, so the left arc starts at t = 16 s.
# A 0.6 s stand starts that same vel 0.4 s earlier. On this plant the 12.5 s
# left hold then only reached about +35 deg instead of about +75 deg.
STAND_S = 1.00
SETTLE_S = 1.20
PERCEPT_S = 0.40
FRAME_EVERY_S = 0.20
UP_Z_ABORT = 0.90
# Claimed walk speed. Half-cap (+0.028) stays upright, but a left yaw
# on that speed is swallowed. The windows below are the ones Controls
# already measured: left ~+75.7 deg in 12.5 s, chained right ~-55 deg
# in 11 s. Caps are not raised.
EXPLORE_VX = steer_walk.VX_FWD_CAP
# Finder last-mile stays at half the forward cap. Full-cap finder
# bursts crossed up_z 0.90. This is not the explore schedule.
FINDER_HALF_VX = steer_walk.VX_FWD_CAP * 0.5
# A yaw arc "tracked" when realized heading moves more than this on
# the commanded side. The swallowed left command was about -12 deg.
YAW_TRACK_DEG = 20.0
DEMO_WALK_S = (
    steer_walk.CLAIMED_APPROACH_S
    + steer_walk.CLAIMED_LEFT_ARC_S
    + steer_walk.CLAIMED_RESUME_S
    + steer_walk.CLAIMED_RIGHT_ARC_S
    + steer_walk.CLAIMED_RESUME_S
)
# Furnished scenes cross up_z 0.90 on the claimed 11 s / 12.5 s windows
# (kitchen right 11 s reached 0.899, bathroom 0.889; a cold left hold
# stayed near +5 to +11 deg and then leaned). The same vel(+0.056, -0.25)
# for 8 s stayed at about 0.919 / 0.934 and the heading followed right.
# This is not a new cap and not the claimed 11 s window.
FURNISHED_RIGHT_S = 8.0

CELL_M = 0.10
X_MIN = -0.80
X_MAX = 2.60
Y_MIN = -1.80
Y_MAX = 1.80
MIN_RANGE_M = 0.30
MAX_RANGE_M = 1.80
FRONTIER_MIN_M = 0.40
FRONTIER_MAX_M = 1.60
FORWARD_CONE_RAD = 1.20
YAW_FULL_ERR_RAD = 0.50
BLOCK_ERR_RAD = 0.40
RAY_STRIDE = 8
FLOOR_SAT = 12
FEATURE_SAT = 18
MIN_YELLOW_FRAC = 0.015
MAP_PX = 14

# Same pixel test find_kitchen uses for the backsplash. A bearing, not a room id.
YELLOW_RED_MIN = 110
YELLOW_GREEN_MIN = 100
YELLOW_BLUE_MAX = 100

UNKNOWN = 0
FREE = 1
FEATURE = 2

SceneName = Literal["plant", "kitchen", "bathroom", "living", "bedroom", "entrance"]
CommandName = Literal["stand", "stop", "vel"]

SCENES: dict[str, Path | None] = {
    "plant": None,
    "kitchen": ROOT / "mujoco" / "room_kitchen.xml",
    "bathroom": ROOT / "mujoco" / "room_bathroom.xml",
    "living": ROOT / "mujoco" / "room_living.xml",
    "bedroom": ROOT / "mujoco" / "room_bedroom.xml",
    "entrance": ROOT / "mujoco" / "room_entrance.xml",
}
DEMO_SCENES: tuple[SceneName, ...] = ("plant", "kitchen", "bathroom")
STAND_SCENES: tuple[SceneName, ...] = (
    "plant",
    "kitchen",
    "bathroom",
    "living",
    "bedroom",
    "entrance",
)

HONESTY = (
    "Partial kit_cam map. Floor cells are a ground-plane paint. "
    "Kitchen-like yellow is a camera-ray bearing, not a waypoint and not arrival. "
    "Frontiers are the edge of that paint. Pose is the sim freejoint, not SLAM. "
    "Explore holds the claimed vel(+0.056, ±0.25) windows "
    "(left 12.5 s, then right 11 s). vx=0 yaw is not used. Not go-anywhere."
)


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass(frozen=True)
class RobotPose:
    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class YellowQuery:
    """Finder query. bearing_rad is a ray direction, not a goal pose."""

    seen: bool
    max_fraction: float
    bearing_rad: float | None
    elevation_rad: float | None
    ground_cell_ij: tuple[int, int] | None
    note: str


@dataclass(frozen=True)
class FrontierCell:
    i: int
    j: int
    x: float
    y: float
    distance_m: float
    bearing_rad: float


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    yaw_rate: float
    reason: str
    frontier_ij: tuple[int, int] | None


@dataclass(frozen=True)
class ExplorePhase:
    name: str
    duration_s: float
    vx: float
    yaw_rate: float


@dataclass(frozen=True)
class PhaseRecord:
    name: str
    commanded_vx: float
    commanded_yaw: float
    duration_s: float
    dx_m: float
    dy_m: float
    dyaw_deg: float
    tracked: bool | None


@dataclass(frozen=True)
class SentCommand:
    t: float
    name: CommandName
    vx: float
    yaw_rate: float


@dataclass(frozen=True)
class IntegrateResult:
    yellow_frac: float
    sky_frac: float
    other_chromatic_frac: float
    low_sat_frac: float
    free_cells: int
    feature_cells: int
    traversed_cells: int
    frontier_cells: int
    blocked: bool
    new_free: int
    new_feature: int


@dataclass
class ExploreMap:
    """Occupancy plus a yellow-bearing log. No room names, no waypoints."""

    nx: int
    ny: int
    state: np.ndarray
    traversed: np.ndarray
    yellow_max_fraction: float
    yellow_bearing_rad: float | None
    yellow_elevation_rad: float | None
    yellow_ground_ij: tuple[int, int] | None
    other_chromatic_max: float
    last_blocked: bool
    hold_ij: tuple[int, int] | None

    @classmethod
    def empty(cls) -> ExploreMap:
        nx = int(round((X_MAX - X_MIN) / CELL_M))
        ny = int(round((Y_MAX - Y_MIN) / CELL_M))
        return cls(
            nx=nx,
            ny=ny,
            state=np.zeros((nx, ny), dtype=np.int8),
            traversed=np.zeros((nx, ny), dtype=np.bool_),
            yellow_max_fraction=0.0,
            yellow_bearing_rad=None,
            yellow_elevation_rad=None,
            yellow_ground_ij=None,
            other_chromatic_max=0.0,
            last_blocked=False,
            hold_ij=None,
        )

    def cell_index(self, x: float, y: float) -> tuple[int, int] | None:
        i = int(math.floor((x - X_MIN) / CELL_M))
        j = int(math.floor((y - Y_MIN) / CELL_M))
        if i < 0 or j < 0 or i >= self.nx or j >= self.ny:
            return None
        return i, j

    def cell_center(self, i: int, j: int) -> tuple[float, float]:
        return (X_MIN + (i + 0.5) * CELL_M, Y_MIN + (j + 0.5) * CELL_M)

    def mark_traversed(self, x: float, y: float) -> None:
        ij = self.cell_index(x, y)
        if ij is None:
            return
        self.traversed[ij[0], ij[1]] = True

    def query_kitchen_like_yellow(self) -> YellowQuery:
        seen = self.yellow_max_fraction >= MIN_YELLOW_FRAC
        if seen:
            note = (
                "kitchen-like yellow is in the kit_cam log. "
                "bearing_rad is the camera ray, not a waypoint. "
                "ground_cell_ij is set only if that ray met the floor "
                f"inside {MAX_RANGE_M:.2f} m."
            )
        else:
            note = (
                f"no kitchen-like yellow at or above {MIN_YELLOW_FRAC:.3f} of a frame. "
                "This is not a room label."
            )
        return YellowQuery(
            seen=seen,
            max_fraction=self.yellow_max_fraction,
            bearing_rad=self.yellow_bearing_rad if seen else None,
            elevation_rad=self.yellow_elevation_rad if seen else None,
            ground_cell_ij=self.yellow_ground_ij if seen else None,
            note=note,
        )

    def frontier_cells(self, x: float, y: float) -> tuple[FrontierCell, ...]:
        """Unknown cells beside free or walked cells, inside the explore ring."""
        known_free = (self.state == FREE) | self.traversed
        cells: list[FrontierCell] = []
        for i in range(self.nx):
            for j in range(self.ny):
                if int(self.state[i, j]) != UNKNOWN or bool(self.traversed[i, j]):
                    continue
                if not _has_free_neighbor(known_free, i, j):
                    continue
                cx, cy = self.cell_center(i, j)
                distance = math.hypot(cx - x, cy - y)
                if distance < FRONTIER_MIN_M or distance > FRONTIER_MAX_M:
                    continue
                cells.append(
                    FrontierCell(
                        i=i,
                        j=j,
                        x=cx,
                        y=cy,
                        distance_m=distance,
                        bearing_rad=math.atan2(cy - y, cx - x),
                    )
                )
        return tuple(cells)

    def counts(self, x: float, y: float) -> tuple[int, int, int, int]:
        free = int(np.count_nonzero(self.state == FREE))
        feature = int(np.count_nonzero(self.state == FEATURE))
        walked = int(np.count_nonzero(self.traversed))
        return free, feature, walked, len(self.frontier_cells(x, y))

    def integrate(
        self,
        pose: RobotPose,
        frame: np.ndarray,
        cam_pos: np.ndarray,
        cam_mat: np.ndarray,
        fovy_deg: float,
    ) -> IntegrateResult:
        if frame.shape != (HEIGHT, WIDTH, 3):
            raise RuntimeError(f"kit_cam frame shape {frame.shape}")
        masks = classify_frame(frame)
        yellow_frac = float(masks.yellow.mean())
        sky_frac = float(masks.sky.mean())
        other_frac = float(masks.other.mean())
        low_sat_frac = float(masks.low_sat.mean())
        self.other_chromatic_max = max(self.other_chromatic_max, other_frac)
        self._remember_yellow(frame, masks.yellow, cam_pos, cam_mat, fovy_deg)
        self.mark_traversed(pose.x, pose.y)
        free_before = int(np.count_nonzero(self.state == FREE))
        feature_before = int(np.count_nonzero(self.state == FEATURE))
        self._paint_rays(masks, cam_pos, cam_mat, fovy_deg)
        self.last_blocked = _lower_center_blocked(masks, cam_pos, cam_mat, fovy_deg)
        free, feature, walked, frontiers = self.counts(pose.x, pose.y)
        return IntegrateResult(
            yellow_frac=yellow_frac,
            sky_frac=sky_frac,
            other_chromatic_frac=other_frac,
            low_sat_frac=low_sat_frac,
            free_cells=free,
            feature_cells=feature,
            traversed_cells=walked,
            frontier_cells=frontiers,
            blocked=self.last_blocked,
            new_free=free - free_before,
            new_feature=feature - feature_before,
        )

    def _remember_yellow(
        self,
        frame: np.ndarray,
        yellow: np.ndarray,
        cam_pos: np.ndarray,
        cam_mat: np.ndarray,
        fovy_deg: float,
    ) -> None:
        frac = float(yellow.mean())
        if frac <= self.yellow_max_fraction or frac <= 0.0:
            return
        rows, cols = np.nonzero(yellow)
        u = float(cols.mean())
        v = float(rows.mean())
        direction = _pixel_direction(u, v, fovy_deg, cam_mat)
        bearing = math.atan2(float(direction[1]), float(direction[0]))
        horizontal = math.hypot(float(direction[0]), float(direction[1]))
        elevation = math.atan2(float(direction[2]), horizontal)
        ground = _ground_point(cam_pos, direction)
        cell: tuple[int, int] | None = None
        if ground is not None:
            cell = self.cell_index(ground[0], ground[1])
        self.yellow_max_fraction = frac
        self.yellow_bearing_rad = bearing
        self.yellow_elevation_rad = elevation
        self.yellow_ground_ij = cell
        del frame

    def _paint_rays(
        self,
        masks: FrameMasks,
        cam_pos: np.ndarray,
        cam_mat: np.ndarray,
        fovy_deg: float,
    ) -> None:
        u, v, directions = _strided_directions(fovy_deg, cam_mat, RAY_STRIDE)
        hit = _ground_points(cam_pos, directions)
        sat = masks.sat[v, u]
        yellow = masks.yellow[v, u]
        sky = masks.sky[v, u]
        valid = np.isfinite(hit[:, 0])
        feature_hit = valid & (sat >= FEATURE_SAT) & ~yellow & ~sky
        free_hit = valid & (sat < FLOOR_SAT) & ~yellow & ~sky & ~feature_hit
        for index in np.flatnonzero(free_hit):
            ij = self.cell_index(float(hit[index, 0]), float(hit[index, 1]))
            if ij is None:
                continue
            if int(self.state[ij[0], ij[1]]) == UNKNOWN:
                self.state[ij[0], ij[1]] = FREE
        for index in np.flatnonzero(feature_hit):
            ij = self.cell_index(float(hit[index, 0]), float(hit[index, 1]))
            if ij is None:
                continue
            self.state[ij[0], ij[1]] = FEATURE


@dataclass(frozen=True)
class FrameMasks:
    yellow: np.ndarray
    sky: np.ndarray
    other: np.ndarray
    low_sat: np.ndarray
    sat: np.ndarray


def classify_frame(frame: np.ndarray) -> FrameMasks:
    """Pixel classes from color only. No body names."""
    red = frame[:, :, 0].astype(np.int16)
    green = frame[:, :, 1].astype(np.int16)
    blue = frame[:, :, 2].astype(np.int16)
    sat = np.maximum(np.maximum(red, green), blue) - np.minimum(np.minimum(red, green), blue)
    yellow = (
        (red > YELLOW_RED_MIN)
        & (green > YELLOW_GREEN_MIN)
        & (blue < YELLOW_BLUE_MAX)
        & (red > blue + 40)
        & (green > blue + 30)
        & (np.abs(red - green) < 60)
    )
    # Empty-plant sky measured near rgb (64, 96, 128). Cyan tile is red-poor
    # and does not pass red >= 40.
    sky = (
        ~yellow
        & (blue >= 90)
        & (blue >= green)
        & (green + 8 >= red)
        & (red >= 40)
        & (red <= 110)
        & (green >= 60)
        & (sat >= 20)
    )
    other = (sat >= FEATURE_SAT) & ~yellow & ~sky
    low_sat = (sat < FLOOR_SAT) & ~yellow & ~sky & ~other
    return FrameMasks(
        yellow=yellow,
        sky=sky,
        other=other,
        low_sat=low_sat,
        sat=sat,
    )


def _pixel_direction(
    u: float,
    v: float,
    fovy_deg: float,
    cam_mat: np.ndarray,
) -> np.ndarray:
    focal = (HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    ray = np.array(
        [
            (u - (WIDTH / 2.0)) / focal,
            -((v - (HEIGHT / 2.0)) / focal),
            -1.0,
        ],
        dtype=np.float64,
    )
    ray /= np.linalg.norm(ray)
    return cam_mat @ ray


def _strided_directions(
    fovy_deg: float,
    cam_mat: np.ndarray,
    stride: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    vs = np.arange(0, HEIGHT, stride, dtype=np.float64)
    us = np.arange(0, WIDTH, stride, dtype=np.float64)
    grid_u, grid_v = np.meshgrid(us, vs)
    u = grid_u.reshape(-1)
    v = grid_v.reshape(-1)
    focal = (HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    ray = np.stack(
        (
            (u - (WIDTH / 2.0)) / focal,
            -((v - (HEIGHT / 2.0)) / focal),
            np.full(u.shape, -1.0, dtype=np.float64),
        ),
        axis=1,
    )
    ray /= np.linalg.norm(ray, axis=1, keepdims=True)
    world = ray @ cam_mat.T
    return u.astype(np.int32), v.astype(np.int32), world


def _ground_point(cam_pos: np.ndarray, direction: np.ndarray) -> np.ndarray | None:
    dz = float(direction[2])
    if dz >= -1e-3:
        return None
    t = -float(cam_pos[2]) / dz
    if t < MIN_RANGE_M or t > MAX_RANGE_M:
        return None
    return cam_pos + t * direction


def _ground_points(cam_pos: np.ndarray, directions: np.ndarray) -> np.ndarray:
    dz = directions[:, 2]
    t = np.full(dz.shape, np.nan, dtype=np.float64)
    looking_down = dz < -1e-3
    t[looking_down] = -float(cam_pos[2]) / dz[looking_down]
    ok = (t >= MIN_RANGE_M) & (t <= MAX_RANGE_M)
    points = np.full((directions.shape[0], 2), np.nan, dtype=np.float64)
    points[ok, 0] = cam_pos[0] + t[ok] * directions[ok, 0]
    points[ok, 1] = cam_pos[1] + t[ok] * directions[ok, 1]
    return points


def _lower_center_blocked(
    masks: FrameMasks,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fovy_deg: float,
) -> bool:
    """Saturated floor hits in the lower center, inside 0.80 m."""
    u, v, directions = _strided_directions(fovy_deg, cam_mat, RAY_STRIDE)
    hit = _ground_points(cam_pos, directions)
    in_window = (v >= int(HEIGHT * 0.70)) & (u >= int(WIDTH * 0.30)) & (u < int(WIDTH * 0.70))
    near = np.isfinite(hit[:, 0])
    # Range is already capped; recompute distance from the camera's ground point.
    dist = np.hypot(hit[:, 0] - float(cam_pos[0]), hit[:, 1] - float(cam_pos[1]))
    near = near & (dist <= 0.80) & in_window
    if int(near.sum()) < 8:
        return False
    sat = masks.sat[v, u]
    blocked = int(((sat >= FEATURE_SAT) & near).sum())
    return (blocked / float(near.sum())) > 0.25


def _has_free_neighbor(known_free: np.ndarray, i: int, j: int) -> bool:
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        ni = i + di
        nj = j + dj
        if ni < 0 or nj < 0 or ni >= known_free.shape[0] or nj >= known_free.shape[1]:
            continue
        if bool(known_free[ni, nj]):
            return True
    return False


def choose_velocity(feature_map: ExploreMap, x: float, y: float, yaw: float) -> VelocityCommand:
    """Frontier side as a claimed-cap walk-yaw.

    The demo does not replan from this every percept. A short left trim
    was commanded and the body did not follow. The held windows are
    claimed_explore_phases(). This function still does not read a
    bearing log, and it does not produce a pure yaw.
    """
    if not _within_caps(EXPLORE_VX, 0.0):
        raise RuntimeError("explore vx is outside the bus caps")
    frontiers = feature_map.frontier_cells(x, y)
    cone = [
        cell
        for cell in frontiers
        if abs(_wrap(cell.bearing_rad - yaw)) <= FORWARD_CONE_RAD
    ]
    if feature_map.last_blocked:
        cone = [
            cell
            for cell in cone
            if abs(_wrap(cell.bearing_rad - yaw)) >= BLOCK_ERR_RAD
        ]
    target = _select_frontier(cone, yaw, feature_map.hold_ij)
    if target is None:
        feature_map.hold_ij = None
        if feature_map.last_blocked:
            return VelocityCommand(
                EXPLORE_VX,
                steer_walk.YAW_RATE_CAP,
                "lower center is saturated; claimed walk-yaw left",
                None,
            )
        return VelocityCommand(
            EXPLORE_VX,
            0.0,
            "no forward frontier; claimed forward",
            None,
        )
    feature_map.hold_ij = (target.i, target.j)
    err = _wrap(target.bearing_rad - yaw)
    if abs(err) < max(steer_walk.DEADBAND_YAW, 0.05):
        yaw_rate = 0.0
        reason = "frontier is ahead; claimed forward, yaw 0"
    elif err > 0.0:
        yaw_rate = steer_walk.YAW_RATE_CAP
        reason = "frontier is left; claimed walk-yaw left"
    else:
        yaw_rate = -steer_walk.YAW_RATE_CAP
        reason = "frontier is right; claimed walk-yaw right"
    if not _within_caps(EXPLORE_VX, yaw_rate):
        raise RuntimeError(f"command outside caps vx={EXPLORE_VX} yaw={yaw_rate}")
    return VelocityCommand(EXPLORE_VX, yaw_rate, reason, (target.i, target.j))


def claimed_explore_phases() -> tuple[ExplorePhase, ...]:
    """nav-multi order. Left then right. A second left, or right then left, is not claimed."""
    vx = EXPLORE_VX
    yaw = steer_walk.YAW_RATE_CAP
    return (
        ExplorePhase("approach", steer_walk.CLAIMED_APPROACH_S, vx, 0.0),
        ExplorePhase("arc-left", steer_walk.CLAIMED_LEFT_ARC_S, vx, yaw),
        ExplorePhase("mid", steer_walk.CLAIMED_RESUME_S, vx, 0.0),
        ExplorePhase("arc-right", steer_walk.CLAIMED_RIGHT_ARC_S, vx, -yaw),
        ExplorePhase("resume", steer_walk.CLAIMED_RESUME_S, vx, 0.0),
    )


def explore_schedule(walk_s: float, scene: str = "plant") -> tuple[ExplorePhase, ...]:
    """Plant uses the claimed left-then-right chain. Furnished scenes yaw right first.

    A full-cap left hold is swallowed or leans in the kitchen and bathroom.
    The claimed 11 s right window also crosses up_z 0.90 there. The shorter
    right hold uses the same vel and stays under that bar.
    """
    if scene != "plant":
        duration = min(FURNISHED_RIGHT_S, walk_s)
        if duration <= 1e-9:
            return ()
        return (
            ExplorePhase(
                "arc-right",
                duration,
                EXPLORE_VX,
                -steer_walk.YAW_RATE_CAP,
            ),
        )
    remaining = walk_s
    chosen: list[ExplorePhase] = []
    for phase in claimed_explore_phases():
        if remaining <= 1e-9:
            break
        duration = min(phase.duration_s, remaining)
        chosen.append(ExplorePhase(phase.name, duration, phase.vx, phase.yaw_rate))
        remaining -= duration
    return tuple(chosen)


def last_mile_from_map(
    feature_map: ExploreMap,
    x: float,
    y: float,
    yaw: float,
) -> VelocityCommand | None:
    """Finder query. None means the map has no kitchen-like yellow yet.

    Always calls query_kitchen_like_yellow() and frontier_cells().
    The aim is the frontier nearest the logged camera ray when one sits
    inside the forward cone of that ray. Otherwise the aim is the ray.
    vx stays at the finder half cap. A pure yaw is not returned.
    """
    yellow = feature_map.query_kitchen_like_yellow()
    frontiers = feature_map.frontier_cells(x, y)
    if not yellow.seen or yellow.bearing_rad is None:
        return None
    aim = yellow.bearing_rad
    chosen_ij: tuple[int, int] | None = None
    best_err: float | None = None
    best_bearing: float | None = None
    for cell in frontiers:
        err = abs(_wrap(cell.bearing_rad - yellow.bearing_rad))
        if best_err is None or err < best_err:
            best_err = err
            best_bearing = cell.bearing_rad
            chosen_ij = (cell.i, cell.j)
    if (
        best_bearing is not None
        and best_err is not None
        and best_err <= FORWARD_CONE_RAD
    ):
        aim = best_bearing
        reason = "map yellow logged; half-cap walk-yaw toward the frontier nearest that ray"
    else:
        chosen_ij = None
        reason = "map yellow logged; half-cap walk-yaw toward the camera ray"
    heading_err = _wrap(aim - yaw)
    yaw_rate = _clamp(
        heading_err / YAW_FULL_ERR_RAD * steer_walk.YAW_RATE_CAP,
        -steer_walk.YAW_RATE_CAP,
        steer_walk.YAW_RATE_CAP,
    )
    if abs(yaw_rate) < steer_walk.DEADBAND_YAW:
        yaw_rate = 0.0
        reason = "map yellow logged; aim is ahead; half-cap forward"
    if not _within_caps(FINDER_HALF_VX, yaw_rate):
        raise RuntimeError(f"last-mile command outside caps vx={FINDER_HALF_VX} yaw={yaw_rate}")
    return VelocityCommand(FINDER_HALF_VX, yaw_rate, reason, chosen_ij)


def yaw_tracked(commanded_yaw: float, dyaw_deg: float) -> bool | None:
    """True when realized heading follows the commanded side. None on a straight hold."""
    if abs(commanded_yaw) <= 1e-9:
        return None
    if commanded_yaw > 0.0:
        return dyaw_deg > YAW_TRACK_DEG
    return dyaw_deg < -YAW_TRACK_DEG


def _select_frontier(
    cone: list[FrontierCell],
    yaw: float,
    hold_ij: tuple[int, int] | None,
) -> FrontierCell | None:
    if not cone:
        return None
    if hold_ij is not None:
        for cell in cone:
            if (cell.i, cell.j) == hold_ij:
                return cell
    def sort_key(cell: FrontierCell) -> tuple[float, float]:
        err = _wrap(cell.bearing_rad - yaw)
        return (round(cell.distance_m, 3), -err)

    return min(cone, key=sort_key)


def _within_caps(vx: float, yaw_rate: float) -> bool:
    return (
        -steer_walk.VX_BACK_CAP - 1e-9 <= vx <= steer_walk.VX_FWD_CAP + 1e-9
        and abs(yaw_rate) <= steer_walk.YAW_RATE_CAP + 1e-9
        and not (abs(vx) <= 1e-9 and abs(yaw_rate) > 1e-9)
    )


class KitCam:
    def __init__(self, model: mj.MjModel) -> None:
        self.renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
        self.cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
        if self.cam_id < 0:
            raise RuntimeError("missing kit_cam")

    def grab(self, model: mj.MjModel, data: mj.MjData) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        self.renderer.update_scene(data, camera="kit_cam")
        frame = np.asarray(self.renderer.render(), dtype=np.uint8).copy()
        if frame.shape != (HEIGHT, WIDTH, 3):
            raise RuntimeError(f"kit_cam frame shape {frame.shape}")
        cam_pos = np.asarray(data.cam_xpos[self.cam_id], dtype=np.float64).copy()
        cam_mat = np.asarray(data.cam_xmat[self.cam_id], dtype=np.float64).reshape(3, 3).copy()
        fovy = float(model.cam_fovy[self.cam_id])
        return frame, cam_pos, cam_mat, fovy

    def close(self) -> None:
        self.renderer.close()


def _pose(session: steer_walk.SteerSession) -> RobotPose:
    return RobotPose(
        t=float(session.data.time),
        x=float(session.data.qpos[0]),
        y=float(session.data.qpos[1]),
        yaw=session.yaw(),
    )


def _hold_stand(session: steer_walk.SteerSession, seconds: float, sent: list[SentCommand]) -> None:
    now = float(session.data.time)
    refusal = session.bus.stand(now)
    if refusal:
        raise RuntimeError(refusal)
    sent.append(SentCommand(now, "stand", 0.0, 0.0))
    end = now + seconds
    while float(session.data.time) < end - 1e-9:
        session.step()


def _send_vel(
    session: steer_walk.SteerSession,
    command: VelocityCommand,
    sent: list[SentCommand],
) -> None:
    if not _within_caps(command.vx, command.yaw_rate):
        raise RuntimeError(f"refused local command {command}")
    now = float(session.data.time)
    refusal = session.bus.vel(command.vx, command.yaw_rate, now)
    if refusal:
        raise RuntimeError(refusal)
    sent.append(SentCommand(now, "vel", command.vx, command.yaw_rate))


@dataclass
class SceneRun:
    scene: str
    stop_reason: str
    start: RobotPose
    stop_pose: RobotPose
    end: RobotPose
    min_up_z: float
    fault: bool
    fault_reason: str
    end_mode: str
    yellow: YellowQuery
    before: IntegrateResult
    after: IntegrateResult
    sent: list[SentCommand]
    trail: list[tuple[float, float]]
    kit_before: np.ndarray
    kit_mid: np.ndarray
    kit_after: np.ndarray
    map_before: np.ndarray
    map_mid: np.ndarray
    map_after: np.ndarray
    frames: list[np.ndarray]
    other_chromatic_max: float
    phases: list[PhaseRecord]

    def delta_x(self) -> float:
        return self.end.x - self.start.x

    def delta_y(self) -> float:
        return self.end.y - self.start.y

    def delta_yaw(self) -> float:
        return _wrap(self.end.yaw - self.start.yaw)

    def vx_zero_yaw_sends(self) -> int:
        return sum(1 for command in self.sent if command.name == "vel" and abs(command.vx) <= 1e-9 and abs(command.yaw_rate) > 1e-9)


def run_explore(
    scene: SceneName,
    walk_s: float,
    *,
    record_frames: bool,
) -> SceneRun:
    scene_xml = SCENES[scene]
    session = steer_walk.SteerSession(video=False, scene_xml=scene_xml)
    cam = KitCam(session.model)
    feature_map = ExploreMap.empty()
    sent: list[SentCommand] = []
    frames: list[np.ndarray] = []
    try:
        _hold_stand(session, STAND_S, sent)
        start = _pose(session)
        frame, cam_pos, cam_mat, fovy = cam.grab(session.model, session.data)
        before = feature_map.integrate(start, frame, cam_pos, cam_mat, fovy)
        kit_before = frame
        map_before = render_map(feature_map, start, [(start.x, start.y)])
        if record_frames:
            frames.append(frame.copy())
        walk_end = float(session.data.time) + walk_s
        last_send = -1.0
        last_look = float(session.data.time)
        last_frame = -1.0
        kit_mid = frame
        map_mid = map_before
        trail: list[tuple[float, float]] = [(start.x, start.y)]
        schedule = explore_schedule(walk_s, scene)
        schedule_s = sum(phase.duration_s for phase in schedule)
        mid_time = float(session.data.time) + (schedule_s * 0.5)
        if not schedule:
            raise RuntimeError("explore schedule is empty")
        phase_i = 0
        phase_origin = start
        phase_t0 = float(session.data.time)
        phases: list[PhaseRecord] = []
        command = VelocityCommand(
            schedule[0].vx, schedule[0].yaw_rate, schedule[0].name, None,
        )
        stop_reason = "claimed walk-yaw schedule"

        def _close_phase(pose: RobotPose) -> None:
            phase = schedule[phase_i]
            phases.append(
                PhaseRecord(
                    name=phase.name,
                    commanded_vx=phase.vx,
                    commanded_yaw=phase.yaw_rate,
                    duration_s=float(pose.t - phase_t0),
                    dx_m=pose.x - phase_origin.x,
                    dy_m=pose.y - phase_origin.y,
                    dyaw_deg=math.degrees(_wrap(pose.yaw - phase_origin.yaw)),
                    tracked=yaw_tracked(
                        phase.yaw_rate,
                        math.degrees(_wrap(pose.yaw - phase_origin.yaw)),
                    ),
                )
            )

        while float(session.data.time) < walk_end - 1e-9 and phase_i < len(schedule):
            now = float(session.data.time)
            phase = schedule[phase_i]
            if now >= (phase_t0 + phase.duration_s) - 1e-9:
                pose_now = _pose(session)
                _close_phase(pose_now)
                phase_i += 1
                if phase_i >= len(schedule):
                    break
                phase_origin = pose_now
                phase_t0 = now
                phase = schedule[phase_i]
                command = VelocityCommand(phase.vx, phase.yaw_rate, phase.name, None)
            if (now - last_send) >= (steer_walk.VEL_RESEND_S - 1e-9):
                _send_vel(session, command, sent)
                last_send = now
            report = session.step()
            pose = _pose(session)
            feature_map.mark_traversed(pose.x, pose.y)
            if len(trail) == 0 or math.hypot(pose.x - trail[-1][0], pose.y - trail[-1][1]) >= 0.02:
                trail.append((pose.x, pose.y))
            up_z = session.samples[-1].up_z if session.samples else 1.0
            if report.mode == "fault" or session.bus.fault:
                stop_reason = f"fault: {session.bus.fault_reason or report.mode}"
                _close_phase(pose)
                break
            if up_z < UP_Z_ABORT:
                stop_reason = f"up_z {up_z:.3f} below {UP_Z_ABORT:.2f}"
                _close_phase(pose)
                break
            if (now - last_look) >= (PERCEPT_S - 1e-9):
                frame, cam_pos, cam_mat, fovy = cam.grab(session.model, session.data)
                feature_map.integrate(pose, frame, cam_pos, cam_mat, fovy)
                last_look = now
                if pose.t >= mid_time and kit_mid is kit_before:
                    kit_mid = frame.copy()
                    map_mid = render_map(feature_map, pose, trail)
            if record_frames and (now - last_frame) >= (FRAME_EVERY_S - 1e-9):
                if (now - last_look) > 1e-6:
                    frame, cam_pos, cam_mat, fovy = cam.grab(session.model, session.data)
                frames.append(frame.copy())
                last_frame = now
        else:
            if phase_i < len(schedule) and stop_reason == "claimed walk-yaw schedule":
                _close_phase(_pose(session))
        if stop_reason == "claimed walk-yaw schedule" and scene != "plant":
            stop_reason = "furnished right-first window"
        elif abs(walk_s - DEMO_WALK_S) > 1e-6 and stop_reason == "claimed walk-yaw schedule":
            stop_reason = "walk budget"
        stop_pose = _pose(session)
        now = float(session.data.time)
        refusal = session.bus.stop(now)
        if refusal:
            raise RuntimeError(refusal)
        sent.append(SentCommand(now, "stop", 0.0, 0.0))
        settle_end = now + SETTLE_S
        while float(session.data.time) < settle_end - 1e-9:
            session.step()
        end = _pose(session)
        frame, cam_pos, cam_mat, fovy = cam.grab(session.model, session.data)
        after = feature_map.integrate(end, frame, cam_pos, cam_mat, fovy)
        if record_frames:
            frames.append(frame.copy())
        kit_after = frame
        map_after = render_map(feature_map, end, trail)
        if kit_mid is kit_before:
            kit_mid = frame.copy()
            map_mid = map_after
        session.assert_plant_unchanged()
        return SceneRun(
            scene=scene,
            stop_reason=stop_reason,
            start=start,
            stop_pose=stop_pose,
            end=end,
            min_up_z=float(session.min_up_z),
            fault=bool(session.bus.fault),
            fault_reason=session.bus.fault_reason,
            end_mode=session.bus.mode,
            yellow=feature_map.query_kitchen_like_yellow(),
            before=before,
            after=after,
            sent=sent,
            trail=trail,
            kit_before=kit_before,
            kit_mid=kit_mid,
            kit_after=kit_after,
            map_before=map_before,
            map_mid=map_mid,
            map_after=map_after,
            frames=frames,
            other_chromatic_max=feature_map.other_chromatic_max,
            phases=phases,
        )
    finally:
        cam.close()


def render_map(
    feature_map: ExploreMap,
    pose: RobotPose,
    trail: list[tuple[float, float]],
) -> np.ndarray:
    from PIL import Image, ImageDraw

    width = feature_map.ny * MAP_PX
    height = feature_map.nx * MAP_PX
    image = Image.new("RGB", (width, height), (24, 26, 30))
    draw = ImageDraw.Draw(image)
    frontiers = {(cell.i, cell.j) for cell in feature_map.frontier_cells(pose.x, pose.y)}
    for i in range(feature_map.nx):
        for j in range(feature_map.ny):
            kind = int(feature_map.state[i, j])
            if (i, j) in frontiers:
                color = (40, 170, 190)
            elif kind == FEATURE:
                color = (210, 120, 40)
            elif bool(feature_map.traversed[i, j]):
                color = (70, 120, 78)
            elif kind == FREE:
                color = (186, 184, 170)
            else:
                continue
            col = j * MAP_PX
            row = (feature_map.nx - 1 - i) * MAP_PX
            draw.rectangle((col, row, col + MAP_PX - 1, row + MAP_PX - 1), fill=color)
    if len(trail) >= 2:
        draw.line([_world_px(feature_map, x, y) for x, y in trail], fill=(230, 230, 220), width=2)
    _draw_robot(draw, feature_map, pose)
    yellow = feature_map.query_kitchen_like_yellow()
    if yellow.seen and yellow.bearing_rad is not None:
        length = 0.90
        x1 = pose.x + length * math.cos(yellow.bearing_rad)
        y1 = pose.y + length * math.sin(yellow.bearing_rad)
        draw.line(
            [_world_px(feature_map, pose.x, pose.y), _world_px(feature_map, x1, y1)],
            fill=(240, 210, 40),
            width=3,
        )
    draw.text((6, 6), "free gray  walked green  feature orange  frontier cyan", fill=(230, 230, 230))
    draw.text((6, 18), "yellow line = camera ray, not a goal", fill=(240, 210, 40))
    return np.asarray(image, dtype=np.uint8)


def _world_px(feature_map: ExploreMap, x: float, y: float) -> tuple[float, float]:
    col = (y - Y_MIN) / CELL_M * MAP_PX
    row = (X_MAX - x) / CELL_M * MAP_PX
    return col, row


def _draw_robot(draw: ImageDraw.ImageDraw, feature_map: ExploreMap, pose: RobotPose) -> None:
    tip_x = pose.x + 0.16 * math.cos(pose.yaw)
    tip_y = pose.y + 0.16 * math.sin(pose.yaw)
    left_x = pose.x + 0.08 * math.cos(pose.yaw + 2.4)
    left_y = pose.y + 0.08 * math.sin(pose.yaw + 2.4)
    right_x = pose.x + 0.08 * math.cos(pose.yaw - 2.4)
    right_y = pose.y + 0.08 * math.sin(pose.yaw - 2.4)
    points = [
        _world_px(feature_map, tip_x, tip_y),
        _world_px(feature_map, left_x, left_y),
        _world_px(feature_map, right_x, right_y),
    ]
    draw.polygon(points, fill=(220, 70, 60))


def _save_png(image: np.ndarray, path: Path) -> None:
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(path)


def _sheet(run: SceneRun) -> np.ndarray:
    from PIL import Image

    def fit(image: np.ndarray, width: int, height: int) -> Image.Image:
        pic = Image.fromarray(image)
        return pic.resize((width, height), Image.Resampling.NEAREST)

    kit_w, kit_h = 480, 360
    map_w, map_h = 480, 360
    row1 = [fit(run.kit_before, kit_w, kit_h), fit(run.kit_mid, kit_w, kit_h), fit(run.kit_after, kit_w, kit_h)]
    row2 = [fit(run.map_before, map_w, map_h), fit(run.map_mid, map_w, map_h), fit(run.map_after, map_w, map_h)]
    sheet = Image.new("RGB", (kit_w * 3, kit_h * 2), (12, 12, 14))
    for index, pic in enumerate(row1):
        sheet.paste(pic, (index * kit_w, 0))
    for index, pic in enumerate(row2):
        sheet.paste(pic, (index * map_w, kit_h))
    return np.asarray(sheet, dtype=np.uint8)


def _write_mp4(frames: list[np.ndarray], out_mp4: Path) -> None:
    if len(frames) < 2:
        raise RuntimeError("not enough kit_cam frames for a clip")
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is not installed")
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="explore_frames_") as tmp:
        folder = Path(tmp)
        for index, frame in enumerate(frames):
            _save_png(frame, folder / f"frame_{index:05d}.png")
        wg._encode_mp4_ffmpeg(folder, out_mp4, fps=5)


def _command_stats(sent: list[SentCommand]) -> dict[str, object]:
    vels = [command for command in sent if command.name == "vel"]
    yaw_values = [command.yaw_rate for command in vels]
    vx_values = sorted({round(command.vx, 6) for command in vels})
    return {
        "stand_sends": sum(1 for command in sent if command.name == "stand"),
        "stop_sends": sum(1 for command in sent if command.name == "stop"),
        "vel_sends": len(vels),
        "vx_zero_yaw_sends": sum(
            1 for command in vels if abs(command.vx) <= 1e-9 and abs(command.yaw_rate) > 1e-9
        ),
        "vx_values": vx_values,
        "max_abs_yaw_command": max((abs(value) for value in yaw_values), default=0.0),
        "replans": [
            {"t": command.t, "vx": command.vx, "yaw_rate": command.yaw_rate}
            for command in vels
            if not vels or command.t == vels[0].t or abs(command.yaw_rate - _previous_yaw(vels, command)) > 1e-9
        ],
    }


def _previous_yaw(vels: list[SentCommand], command: SentCommand) -> float:
    previous = 0.0
    for item in vels:
        if item.t >= command.t - 1e-9 and item is command:
            return previous
        previous = item.yaw_rate
    return previous


def _run_payload(run: SceneRun) -> dict[str, object]:
    yellow = run.yellow
    return {
        "scene": run.scene,
        "stop_reason": run.stop_reason,
        "prefer_fail": True,
        "go_anywhere": False,
        "arrival_claimed": False,
        "dx_m": run.delta_x(),
        "dy_m": run.delta_y(),
        "dyaw_rad": run.delta_yaw(),
        "dyaw_deg": math.degrees(run.delta_yaw()),
        "start": asdict(run.start),
        "stop_pose": asdict(run.stop_pose),
        "end": asdict(run.end),
        "min_up_z": run.min_up_z,
        "up_z_abort": UP_Z_ABORT,
        "fault": run.fault,
        "fault_reason": run.fault_reason,
        "tip": bool(run.fault and "tip" in run.fault_reason) or run.min_up_z < 0.85,
        "end_mode": run.end_mode,
        "yellow": {
            "seen": yellow.seen,
            "max_fraction": yellow.max_fraction,
            "bearing_rad": yellow.bearing_rad,
            "elevation_rad": yellow.elevation_rad,
            "ground_cell_ij": list(yellow.ground_cell_ij) if yellow.ground_cell_ij is not None else None,
            "note": yellow.note,
        },
        "map_before": asdict(run.before),
        "map_after": asdict(run.after),
        "other_chromatic_max": run.other_chromatic_max,
        "trail_points": len(run.trail),
        "commands": _command_stats(run.sent),
        "phases": [asdict(phase) for phase in run.phases],
        "yaw_tracked": {
            phase.name: phase.tracked
            for phase in run.phases
            if phase.tracked is not None
        },
        "limit": _scene_limit(run),
    }


def _scene_limit(run: SceneRun) -> str:
    yellow = run.yellow
    free = run.after.free_cells
    feature = run.after.feature_cells
    walked = run.after.traversed_cells
    if run.scene == "plant" and not yellow.seen:
        return (
            f"Prefer FAIL: empty floor. Vision-free cells {free}, floor-feature cells {feature}, "
            f"walked cells {walked}. No kitchen-like yellow. Frontiers are the edge of that floor paint, "
            "not rooms. Not go-anywhere."
        )
    if yellow.seen and yellow.ground_cell_ij is None:
        return (
            f"Prefer FAIL: kitchen-like yellow fraction {yellow.max_fraction:.3f} at bearing "
            f"{yellow.bearing_rad:.3f} rad, elevation {yellow.elevation_rad:.3f} rad. "
            "The ray does not meet the floor in range, so there is no yellow cell and no waypoint. "
            f"Vision-free cells {free}, floor-feature cells {feature}. Not arrival and not go-anywhere."
        )
    if yellow.seen:
        return (
            f"Prefer FAIL: yellow fraction {yellow.max_fraction:.3f} painted a floor cell "
            f"{yellow.ground_cell_ij}. That cell is a ground-plane hit, not a room waypoint. Not arrival."
        )
    return (
        f"Prefer FAIL: no kitchen-like yellow. Other chromatic fraction peaked at "
        f"{run.other_chromatic_max:.3f} and is not labeled as a room. "
        f"Vision-free cells {free}, floor-feature cells {feature}, walked cells {walked}. "
        "Not go-anywhere."
    )


def _print_run(run: SceneRun) -> None:
    phase_bits = " ".join(
        f"{phase.name} dyaw={phase.dyaw_deg:+.1f}deg"
        f"{'' if phase.tracked is None else (' track' if phase.tracked else ' SWALLOWED')}"
        for phase in run.phases
    )
    print(
        f"[explore] {run.scene}: {_scene_limit(run)} "
        f"dx={run.delta_x():+.3f} m dy={run.delta_y():+.3f} m "
        f"dyaw={math.degrees(run.delta_yaw()):+.2f} deg "
        f"min_up_z={run.min_up_z:.3f} fault={run.fault} "
        f"stop={run.stop_reason} end_mode={run.end_mode} "
        f"phases[{phase_bits}]"
    )


def _expect(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def _paint_free_rect(feature_map: ExploreMap, x0: float, x1: float, y0: float, y1: float) -> None:
    x = x0
    while x <= x1 + 1e-9:
        y = y0
        while y <= y1 + 1e-9:
            ij = feature_map.cell_index(x, y)
            if ij is not None:
                feature_map.state[ij[0], ij[1]] = FREE
            y += CELL_M * 0.5
        x += CELL_M * 0.5


def test_policy_uses_frontiers_not_yellow() -> list[str]:
    """A yellow bearing on the opposite side must not pick the command."""
    failures: list[str] = []
    source = inspect.getsource(choose_velocity)
    _expect("yellow" not in source, "choose_velocity mentions yellow", failures)
    _expect("kitchen" not in source and "waypoint" not in source, "choose_velocity mentions a room goal", failures)
    # Free paint covers the left side and the far field. The near unknown
    # edge is in front-right. The logged yellow bearing points left.
    right = ExploreMap.empty()
    right.yellow_max_fraction = 0.20
    right.yellow_bearing_rad = math.radians(80.0)
    right.yellow_elevation_rad = 0.4
    _paint_free_rect(right, -0.2, 1.4, -0.05, 1.4)
    command = choose_velocity(right, 0.50, 0.0, 0.0)
    _expect(abs(command.vx - EXPLORE_VX) < 1e-9, f"right-map vx {command.vx}", failures)
    _expect(command.yaw_rate < 0.0, f"frontier on the right should yaw right, got {command.yaw_rate} ({command.reason})", failures)
    _expect(command.frontier_ij is not None, "missing right frontier", failures)
    left = ExploreMap.empty()
    left.yellow_max_fraction = 0.20
    left.yellow_bearing_rad = math.radians(-80.0)
    left.yellow_elevation_rad = 0.4
    _paint_free_rect(left, -0.2, 1.4, -1.4, 0.05)
    left_command = choose_velocity(left, 0.50, 0.0, 0.0)
    _expect(abs(left_command.vx - EXPLORE_VX) < 1e-9, f"left-map vx {left_command.vx}", failures)
    _expect(left_command.yaw_rate > 0.0, f"frontier on the left should yaw left, got {left_command.yaw_rate} ({left_command.reason})", failures)
    zero = VelocityCommand(0.0, steer_walk.YAW_RATE_CAP, "illegal", None)
    _expect(not _within_caps(zero.vx, zero.yaw_rate), "vx=0 yaw was accepted", failures)
    return failures


def test_classify_colors() -> list[str]:
    failures: list[str] = []
    yellow = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    yellow[:, :] = (230, 210, 40)
    masks = classify_frame(yellow)
    _expect(float(masks.yellow.mean()) > 0.99, "yellow image missed", failures)
    sky = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    sky[:, :] = (70, 100, 140)
    sky_masks = classify_frame(sky)
    _expect(float(sky_masks.sky.mean()) > 0.99, "sky image missed", failures)
    _expect(float(sky_masks.yellow.mean()) == 0.0, "sky counted as yellow", failures)
    cyan = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    cyan[:, :] = (10, 180, 190)
    cyan_masks = classify_frame(cyan)
    _expect(float(cyan_masks.yellow.mean()) == 0.0, "cyan counted as yellow", failures)
    _expect(float(cyan_masks.sky.mean()) == 0.0, "cyan counted as sky", failures)
    _expect(float(cyan_masks.other.mean()) > 0.99, "cyan was not other chromatic", failures)
    floor = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    floor[:, :] = (80, 80, 82)
    floor_masks = classify_frame(floor)
    _expect(float(floor_masks.low_sat.mean()) > 0.99, "gray floor missed", failures)
    _expect(float(floor_masks.yellow.mean()) == 0.0, "gray counted as yellow", failures)
    return failures


def _stand_integrate(scene: SceneName) -> tuple[ExploreMap, IntegrateResult, np.ndarray]:
    session = steer_walk.SteerSession(video=False, scene_xml=SCENES[scene])
    cam = KitCam(session.model)
    sent: list[SentCommand] = []
    try:
        _hold_stand(session, STAND_S, sent)
        pose = _pose(session)
        frame, cam_pos, cam_mat, fovy = cam.grab(session.model, session.data)
        feature_map = ExploreMap.empty()
        result = feature_map.integrate(pose, frame, cam_pos, cam_mat, fovy)
        session.assert_plant_unchanged()
        return feature_map, result, frame
    finally:
        cam.close()


def test_stand_scenes() -> list[str]:
    failures: list[str] = []
    for scene in STAND_SCENES:
        feature_map, result, _frame = _stand_integrate(scene)
        yellow = feature_map.query_kitchen_like_yellow()
        _expect(result.free_cells > 20, f"{scene} free cells {result.free_cells}", failures)
        if scene == "kitchen":
            _expect(yellow.seen, f"kitchen yellow not seen ({yellow.max_fraction:.4f})", failures)
            _expect(0.02 <= yellow.max_fraction <= 0.12, f"kitchen yellow frac {yellow.max_fraction:.4f}", failures)
            _expect(yellow.ground_cell_ij is None, f"kitchen yellow painted a cell {yellow.ground_cell_ij}", failures)
            _expect(yellow.bearing_rad is not None and abs(yellow.bearing_rad) < 0.35, f"kitchen bearing {yellow.bearing_rad}", failures)
        else:
            _expect(not yellow.seen, f"{scene} yellow seen {yellow.max_fraction:.4f}", failures)
        if scene == "plant":
            _expect(result.feature_cells == 0, f"plant feature cells {result.feature_cells}", failures)
        if scene == "entrance":
            _expect(result.feature_cells > 0, f"entrance mat did not paint a feature cell ({result.feature_cells})", failures)
        _expect(result.frontier_cells > 0, f"{scene} has no frontiers", failures)
    return failures


def test_short_walk() -> list[str]:
    failures: list[str] = []
    run = run_explore("plant", 2.0, record_frames=False)
    _expect(run.delta_x() > 0.0, f"short walk dx {run.delta_x():.4f}", failures)
    _expect(run.min_up_z >= UP_Z_ABORT, f"short walk min up_z {run.min_up_z:.3f}", failures)
    _expect(not run.fault, f"short walk fault {run.fault_reason}", failures)
    _expect(run.end_mode == "stand", f"short walk end mode {run.end_mode}", failures)
    _expect(run.vx_zero_yaw_sends() == 0, "short walk sent vx=0 yaw", failures)
    _expect(
        all(abs(command.vx - EXPLORE_VX) < 1e-9 for command in run.sent if command.name == "vel"),
        "short walk vx",
        failures,
    )
    _expect(
        all(abs(command.yaw_rate) < 1e-9 for command in run.sent if command.name == "vel"),
        "short walk left the approach yaw",
        failures,
    )
    _expect(len(run.phases) == 1 and run.phases[0].name == "approach", f"short phases {run.phases}", failures)
    _expect(not run.yellow.seen, "short walk saw yellow on the empty plant", failures)
    _expect(run.after.free_cells >= run.before.free_cells, "map lost free cells", failures)
    return failures


def test_caps_and_plant() -> list[str]:
    failures: list[str] = []
    digest = _md5(PLANT_XML)
    _expect(digest == steer_walk.PLANT_MD5, f"plant md5 {digest}", failures)
    _expect(abs(EXPLORE_VX - 0.056) < 1e-9, f"explore vx {EXPLORE_VX}", failures)
    _expect(abs(FINDER_HALF_VX - 0.028) < 1e-9, f"finder half vx {FINDER_HALF_VX}", failures)
    _expect(abs(steer_walk.CLAIMED_LEFT_ARC_S - 12.5) < 1e-9, "left window moved", failures)
    _expect(abs(steer_walk.CLAIMED_RIGHT_ARC_S - 11.0) < 1e-9, "right window moved", failures)
    _expect(steer_walk.VX_FWD_CAP == 0.056 and steer_walk.VX_BACK_CAP == 0.032, "vx caps", failures)
    _expect(steer_walk.YAW_RATE_CAP == 0.25, "yaw cap", failures)
    _expect(abs(steer_walk.VEL_RESEND_S - 0.10) < 1e-9, "resend", failures)
    _expect(steer_walk.KIT_CAM_POS == (0.050, 0.019, 0.007), "kit_cam pos", failures)
    return failures


def test_claimed_schedule() -> list[str]:
    failures: list[str] = []
    phases = claimed_explore_phases()
    names = [phase.name for phase in phases]
    _expect(
        names == ["approach", "arc-left", "mid", "arc-right", "resume"],
        f"phase order {names}",
        failures,
    )
    by_name = {phase.name: phase for phase in phases}
    left = by_name["arc-left"]
    right = by_name["arc-right"]
    _expect(abs(left.duration_s - 12.5) < 1e-9 and abs(left.vx - EXPLORE_VX) < 1e-9, "left window", failures)
    _expect(abs(left.yaw_rate - steer_walk.YAW_RATE_CAP) < 1e-9, "left yaw", failures)
    _expect(abs(right.duration_s - 11.0) < 1e-9 and right.yaw_rate < 0.0, "right window", failures)
    _expect(names.index("arc-left") < names.index("arc-right"), "right-then-left is not the claimed chain", failures)
    _expect(all(_within_caps(phase.vx, phase.yaw_rate) for phase in phases), "schedule outside caps", failures)
    short = explore_schedule(2.0, "plant")
    _expect(len(short) == 1 and short[0].name == "approach" and abs(short[0].duration_s - 2.0) < 1e-9, f"short schedule {short}", failures)
    _expect(abs(sum(phase.duration_s for phase in phases) - DEMO_WALK_S) < 1e-9, "demo length", failures)
    room = explore_schedule(DEMO_WALK_S, "kitchen")
    _expect(len(room) == 1 and room[0].name == "arc-right", f"room schedule {room}", failures)
    if room:
        _expect(abs(room[0].vx - EXPLORE_VX) < 1e-9 and room[0].yaw_rate < 0.0, "room right vel", failures)
        _expect(room[0].duration_s < steer_walk.CLAIMED_RIGHT_ARC_S, "room window is the 11 s hold", failures)
        _expect(abs(room[0].duration_s - FURNISHED_RIGHT_S) < 1e-9, "room window", failures)
    return failures


def test_last_mile_query() -> list[str]:
    """Logged yellow is queried. An empty map is a Prefer FAIL with no command."""
    failures: list[str] = []
    source = inspect.getsource(last_mile_from_map)
    _expect("query_kitchen_like_yellow" in source, "last mile does not query yellow", failures)
    _expect("frontier_cells" in source, "last mile does not query frontiers", failures)
    _expect("waypoint" not in source, "last mile mentions a waypoint", failures)
    empty = ExploreMap.empty()
    _paint_free_rect(empty, 0.3, 1.2, -0.4, 0.4)
    _expect(last_mile_from_map(empty, 0.2, 0.0, 0.0) is None, "empty map produced a command", failures)
    logged = ExploreMap.empty()
    logged.yellow_max_fraction = 0.08
    logged.yellow_bearing_rad = 0.0
    logged.yellow_elevation_rad = 0.5
    missed = last_mile_from_map(logged, 0.2, 0.0, 0.0)
    _expect(missed is not None, "logged yellow with no frontier returned None", failures)
    if missed is not None:
        _expect(abs(missed.vx - FINDER_HALF_VX) < 1e-9, f"last-mile vx {missed.vx}", failures)
        _expect(missed.frontier_ij is None, "ray aim invented a frontier", failures)
        _expect(abs(missed.yaw_rate) <= steer_walk.YAW_RATE_CAP + 1e-9, "last-mile yaw cap", failures)
    painted = ExploreMap.empty()
    painted.yellow_max_fraction = 0.08
    painted.yellow_bearing_rad = 0.0
    painted.yellow_elevation_rad = 0.5
    _paint_free_rect(painted, 0.3, 1.2, -0.3, 0.3)
    aimed = last_mile_from_map(painted, 0.2, 0.0, 0.0)
    _expect(aimed is not None and aimed.frontier_ij is not None, f"frontier aim {aimed}", failures)
    if aimed is not None:
        _expect(abs(aimed.vx - FINDER_HALF_VX) < 1e-9, f"frontier aim vx {aimed.vx}", failures)
        _expect(_within_caps(aimed.vx, aimed.yaw_rate), "frontier aim outside caps", failures)
    return failures


def self_test() -> int:
    failures: list[str] = []
    failures.extend(test_caps_and_plant())
    failures.extend(test_classify_colors())
    failures.extend(test_policy_uses_frontiers_not_yellow())
    failures.extend(test_claimed_schedule())
    failures.extend(test_last_mile_query())
    failures.extend(test_stand_scenes())
    failures.extend(test_short_walk())
    if _md5(PLANT_XML) != steer_walk.PLANT_MD5:
        failures.append("plant md5 changed during self-test")
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[explore] self-test PASS")
    return 0


def _save_run(run: SceneRun) -> list[Path]:
    paths = [
        PREVIEWS / f"explore_map_{run.scene}_kit_before.png",
        PREVIEWS / f"explore_map_{run.scene}_kit_mid.png",
        PREVIEWS / f"explore_map_{run.scene}_kit_after.png",
        PREVIEWS / f"explore_map_{run.scene}_map_before.png",
        PREVIEWS / f"explore_map_{run.scene}_map_mid.png",
        PREVIEWS / f"explore_map_{run.scene}_map_after.png",
        PREVIEWS / f"explore_map_{run.scene}_sheet.png",
    ]
    images = (
        run.kit_before,
        run.kit_mid,
        run.kit_after,
        run.map_before,
        run.map_mid,
        run.map_after,
        _sheet(run),
    )
    for path, image in zip(paths, images):
        _save_png(image, path)
    if run.scene == "kitchen" and run.frames:
        clip = PREVIEWS / "explore_map_kitchen.mp4"
        _write_mp4(run.frames, clip)
        paths.append(clip)
    return paths


def demo(walk_s: float) -> int:
    before = _md5(PLANT_XML)
    if before != steer_walk.PLANT_MD5:
        print(f"FAIL plant md5 {before}")
        return 1
    payloads: list[dict[str, object]] = []
    saved: list[Path] = []
    problems: list[str] = []
    for scene in DEMO_SCENES:
        run = run_explore(scene, walk_s, record_frames=scene == "kitchen")
        _print_run(run)
        payloads.append(_run_payload(run))
        saved.extend(_save_run(run))
        if run.fault or run.min_up_z < UP_Z_ABORT:
            problems.append(f"{scene} min_up_z {run.min_up_z:.3f} fault {run.fault_reason}")
        if run.vx_zero_yaw_sends() != 0:
            problems.append(f"{scene} sent vx=0 yaw")
        if run.end_mode != "stand":
            problems.append(f"{scene} end mode {run.end_mode}")
    after = _md5(PLANT_XML)
    summary: dict[str, object] = {
        "plant_md5": after,
        "plant_md5_unchanged": after == steer_walk.PLANT_MD5 == before,
        "kit_cam_pos": list(steer_walk.KIT_CAM_POS),
        "kit_cam_fovy": steer_walk.KIT_CAM_FOVY,
        "vx_fwd_cap": steer_walk.VX_FWD_CAP,
        "vx_back_cap": steer_walk.VX_BACK_CAP,
        "yaw_rate_cap": steer_walk.YAW_RATE_CAP,
        "explore_vx": EXPLORE_VX,
        "finder_half_vx": FINDER_HALF_VX,
        "claimed_left_arc_s": steer_walk.CLAIMED_LEFT_ARC_S,
        "claimed_right_arc_s": steer_walk.CLAIMED_RIGHT_ARC_S,
        "resend_hz": 1.0 / steer_walk.VEL_RESEND_S,
        "walk_s": walk_s,
        "cell_m": CELL_M,
        "honesty": HONESTY,
        "prefer_fail_bar": {
            "tonight": (
                "A real land is not this slice. The map is a ground-plane floor fan "
                "from one camera, frontiers on the edge of that fan, and a yellow "
                "bearing when the backsplash is in frame. The walk holds the claimed "
                "vel(+0.056, ±0.25) windows so the body can yaw. Rooms are separate "
                "XML files, not one space. Pose is the sim freejoint. vx=0 does not turn. "
                "Arrival is still yellow >= 0.50 and torso-to-kitchen <= 0.25 m."
            ),
            "later": (
                "A later land would need one continuous space, metric occupied cells "
                "for elevated furniture (depth or parallax, not a floor-plane guess), "
                "and a pose that is not the simulator freejoint. The finder can query "
                "this interface. That query is not a waypoint and not arrival."
            ),
        },
        "runs": payloads,
    }
    summary_path = PREVIEWS / "explore_map_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    saved.append(summary_path)
    if ARTIFACTS.is_dir() or True:
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        for path in saved:
            target = ARTIFACTS / path.name
            target.write_bytes(path.read_bytes())
    print(f"[explore] wrote {summary_path.relative_to(ROOT)}")
    if after != steer_walk.PLANT_MD5:
        problems.append(f"plant md5 changed to {after}")
    if problems:
        for message in problems:
            print(f"FAIL {message}")
        return 1
    print("[explore] demo PASS (Prefer FAIL on go-anywhere; map is partial)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Explore from kit_cam and build a partial map")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--walk-s", type=float, default=DEMO_WALK_S)
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    return demo(float(args.walk_s))


if __name__ == "__main__":
    raise SystemExit(main())
