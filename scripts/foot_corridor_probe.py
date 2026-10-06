#!/usr/bin/env python3
"""Plant corridor half-widths. No ray, no stop, no head tilt.

Reads the outer edges of l_foot_contact and r_foot_contact. The 14 mm
outboard shift is the geom pos already in the plant. This does not
estimate a stance width and then add a foot width.

The Day-1 stop is not latched here. The column-aware ray belongs to
the AI estimator. This probe only prints the widths that latch will use.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("DISPLAY", ":1")
os.environ.setdefault("MUJOCO_GL", "glfw")

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import mujoco as mj
import numpy as np

import steer_walk as sw

SIDES = (
    ("L", "l_foot_contact"),
    ("R", "r_foot_contact"),
)


def _corners(model: mj.MjModel, data: mj.MjData, gid: int) -> np.ndarray:
    center = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    pts: list[np.ndarray] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array(
                    [sx * half[0], sy * half[1], sz * half[2]],
                    dtype=np.float64,
                )
                pts.append(center + rot @ local)
    return np.stack(pts, axis=0)


def _body_y(data: mj.MjData, bid: int, points: np.ndarray) -> np.ndarray:
    origin = np.asarray(data.xpos[bid], dtype=np.float64)
    rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
    local = (points - origin) @ rot
    return local[:, 1]


def _edges(session: sw.SteerSession) -> dict[str, float]:
    model = session.model
    data = session.data
    out: dict[str, float] = {}
    for side, name in SIDES:
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        ys = _body_y(data, session.bid_body, _corners(model, data, gid))
        out[f"{side}_out"] = float(ys.max() if side == "L" else ys.min())
        out[f"{side}_in"] = float(ys.min() if side == "L" else ys.max())
    return out


def _print_authored(model: mj.MjModel) -> None:
    for _side, name in SIDES:
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        pos = np.asarray(model.geom_pos[gid], dtype=np.float64)
        half = np.asarray(model.geom_size[gid], dtype=np.float64)
        print(
            f"authored {name} pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
            f"half=({half[0]:.4f},{half[1]:.4f},{half[2]:.4f}) "
            f"local_outer_y={pos[1] + np.sign(pos[1]) * half[1]:+.4f}"
        )


def _walk(yaw: float) -> tuple[dict[str, float], dict[str, float]]:
    session = sw.SteerSession(video=False, lipm=sw.locked_kit_config())
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(4.0, "vel", sw.VX_FWD_CAP, yaw, "turn"),
    ))
    stand: dict[str, float] | None = None
    peak: dict[str, float] | None = None
    while float(session.data.time) < 4.0 - 1e-9:
        now = float(session.data.time)
        driver.publish(session.bus, now)
        session.step()
        edges = _edges(session)
        t = float(session.data.time)
        if stand is None and t >= 1.0 - 1e-3:
            stand = edges
            peak = dict(edges)
            _print_authored(session.model)
        if peak is not None and t >= 1.0:
            if edges["L_out"] > peak["L_out"]:
                peak["L_out"] = edges["L_out"]
            if edges["R_out"] < peak["R_out"]:
                peak["R_out"] = edges["R_out"]
    assert stand is not None and peak is not None
    return stand, peak


def main() -> None:
    print(f"plant {sw._md5(sw.PLANT_XML)}")
    for yaw in (sw.YAW_RATE_CAP, -sw.YAW_RATE_CAP):
        stand, peak = _walk(yaw)
        l_half = stand["L_out"]
        r_half = -stand["R_out"]
        # +yaw is a left turn. The outside foot is the right foot.
        if yaw > 0.0:
            sweep = stand["R_out"] - peak["R_out"]
            side = "R"
        else:
            sweep = peak["L_out"] - stand["L_out"]
            side = "L"
        print(
            f"yaw={yaw:+.2f} stand_L_out={stand['L_out']:+.4f} "
            f"stand_L_in={stand['L_in']:+.4f} stand_R_out={stand['R_out']:+.4f} "
            f"stand_R_in={stand['R_in']:+.4f} half_L={l_half:.4f} half_R={r_half:.4f} "
            f"outside={side} sweep={sweep:+.4f} "
            f"outside_half={((r_half if side == 'R' else l_half) + sweep):.4f}"
        )


if __name__ == "__main__":
    main()
