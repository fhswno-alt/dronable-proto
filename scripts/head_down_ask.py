#!/usr/bin/env python3
"""Prefer FAIL: walk head-down floor edge, then Moondream at restore.

The head joint is moved inside the session. The plant file, the gait
caps, and the CommandBus command set are unchanged. Day1 commands stay
stand, stop, and vel. head_tilt is not a bus key.

Negative head_tilt depresses kit_cam (axis 0 -1 0). The walk row is
head_tilt 0, the kit crouch. Down rows are -10 deg and -15 deg from
that. Restore is +0.25 rad, the earlier near-level ask pose.

Near-edge is the horizontal distance from the camera's ground point to
where the bottom-center ray meets the floor plane z = 0. It is a
geometric FOV number, not a furniture range.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import mujoco as mj
import numpy as np
from PIL import Image

import explore_map as em
import steer_walk as sw

PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
HEAD_JOINT = "head_tilt"
HEAD_ACT = "head_tilt_pos"
# Frozen before the grid. Negative depresses. +0.25 rad is the #58 level pose.
DOWN_10 = -math.radians(10.0)
DOWN_15 = -math.radians(15.0)
RESTORE = 0.25
STAND_HEAD = 0.0
WALK_S = 2.0
ASK_ROOMS = ("kitchen", "bathroom", "bedroom", "living")
POSES = (
    ("stand", STAND_HEAD),
    ("down10", DOWN_10),
    ("restore", RESTORE),
)


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _pitch(cam_mat: np.ndarray) -> float:
    look = -cam_mat[:, 2]
    return math.degrees(math.atan2(float(look[2]), math.hypot(float(look[0]), float(look[1]))))


def _near_floor(cam_pos: np.ndarray, cam_mat: np.ndarray, fovy_deg: float) -> float | None:
    """Horizontal meters from the camera nadir to the bottom-center floor hit."""
    half_h = math.tan(math.radians(fovy_deg) * 0.5)
    direction = cam_mat @ np.array([0.0, -half_h, -1.0], dtype=np.float64)
    if direction[2] >= -1e-8:
        return None
    scale = -float(cam_pos[2]) / float(direction[2])
    if scale <= 0.0:
        return None
    hit = cam_pos + scale * direction
    return float(math.hypot(hit[0] - cam_pos[0], hit[1] - cam_pos[1]))


def _cam(model: mj.MjModel, data: mj.MjData) -> tuple[np.ndarray, np.ndarray, float]:
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    pos = np.asarray(data.cam_xpos[cam_id], dtype=np.float64).copy()
    mat = np.asarray(data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3).copy()
    return pos, mat, float(model.cam_fovy[cam_id])


def _head_q(model: mj.MjModel, data: mj.MjData) -> float:
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, HEAD_JOINT)
    return float(data.qpos[int(model.jnt_qposadr[jid])])


def _set_head_goal(session: sw.SteerSession, rad: float) -> None:
    session.q_stand[HEAD_JOINT] = rad
    if session.lipm is not None:
        session.lipm.q_stand[HEAD_JOINT] = rad


def _hold(session: sw.SteerSession, seconds: float) -> None:
    end = float(session.data.time) + seconds
    while float(session.data.time) < end - 1e-9:
        session.step()


def _walk_fov(head: float, yaw_rate: float = 0.0) -> dict[str, float | None]:
    session = sw.SteerSession(video=False, lipm=sw.locked_kit_config())
    _set_head_goal(session, head)
    sent: list[em.SentCommand] = []
    em._hold_stand(session, em.STAND_S, sent)
    last_send = -1.0
    end = em.STAND_S + WALK_S
    rows: list[tuple[float, float, float]] = []
    head_tau = 0.0
    act = session.act_idx[HEAD_ACT]
    while float(session.data.time) < end - 1e-9:
        now = float(session.data.time)
        if (now - last_send) >= (sw.VEL_RESEND_S - 1e-9):
            refusal = session.bus.vel(sw.VX_FWD_CAP, yaw_rate, now)
            if refusal:
                raise SystemExit(refusal)
            last_send = now
        session.step()
        head_tau = max(head_tau, abs(float(session.data.actuator_force[act])))
        if float(session.data.time) < em.STAND_S + 0.5:
            continue
        pos, mat, fovy = _cam(session.model, session.data)
        near = _near_floor(pos, mat, fovy)
        rows.append((float(pos[2]), _pitch(mat), -1.0 if near is None else near))
    session.assert_plant_unchanged()
    if not rows:
        raise SystemExit("no walk samples")
    z = sorted(r[0] for r in rows)
    pitch = sorted(r[1] for r in rows)
    near = sorted(r[2] for r in rows)
    mid = len(rows) // 2
    return {
        "head_cmd_rad": head,
        "yaw_rate": yaw_rate,
        "head_q_rad": _head_q(session.model, session.data),
        "n": float(len(rows)),
        "cam_z_m": z[mid],
        "pitch_deg": pitch[mid],
        "near_m": near[mid],
        "head_tau_nm": head_tau,
        "min_up_z": float(session.min_up_z),
        "fault": 1.0 if session.bus.fault else 0.0,
    }


def _render_pose(scene: str, head: float) -> tuple[np.ndarray, dict[str, float]]:
    session = sw.SteerSession(
        video=False, scene_xml=em.SCENES[scene], lipm=sw.locked_kit_config(),
    )
    _set_head_goal(session, head)
    sent: list[em.SentCommand] = []
    em._hold_stand(session, em.STAND_S, sent)
    # The slew limit on the head needs a short hold after the stand.
    _hold(session, 0.40)
    pos, mat, fovy = _cam(session.model, session.data)
    near = _near_floor(pos, mat, fovy)
    renderer = mj.Renderer(session.model, height=480, width=640)
    try:
        renderer.update_scene(session.data, camera="kit_cam")
        rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
    finally:
        renderer.close()
    act = session.act_idx[HEAD_ACT]
    info = {
        "head_cmd_rad": head,
        "head_q_rad": _head_q(session.model, session.data),
        "cam_z_m": float(pos[2]),
        "pitch_deg": _pitch(mat),
        "near_m": -1.0 if near is None else near,
        "head_tau_nm": abs(float(session.data.actuator_force[act])),
        "min_up_z": float(session.min_up_z),
    }
    session.assert_plant_unchanged()
    return rgb, info


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=sw.ROOT / "previews" / "head_down_ask")
    parser.add_argument("--fov-only", action="store_true")
    args = parser.parse_args()
    digest = _md5(sw.PLANT_XML)
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest}")
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    fov = {
        "stand": _walk_fov(STAND_HEAD),
        "yaw": _walk_fov(STAND_HEAD, -0.25),
        "down10": _walk_fov(DOWN_10),
        "down15": _walk_fov(DOWN_15),
    }
    for name, row in fov.items():
        print(
            f"plant md5 {digest} fov {name} pitch={row['pitch_deg']:.2f} deg "
            f"near={row['near_m']:.3f} m cam_z={row['cam_z_m']:.3f} "
            f"q={row['head_q_rad']:.3f} tau={row['head_tau_nm']:.3f} Nm "
            f"min_up_z={row['min_up_z']:.3f}",
            flush=True,
        )

    poses: dict[str, dict[str, dict[str, float | str | None]]] = {}
    if not args.fov_only:
        import room_ask

        asker = room_ask.RoomAsk()
        try:
            for scene in ASK_ROOMS:
                poses[scene] = {}
                for pose, head in POSES:
                    rgb, info = _render_pose(scene, head)
                    Image.fromarray(rgb).save(out / f"{scene}_{pose}.png")
                    result = asker.ask(rgb)
                    poses[scene][pose] = {
                        "room": result["room"],
                        "raw": result["raw"],
                        "pitch_deg": info["pitch_deg"],
                        "near_m": info["near_m"],
                        "cam_z_m": info["cam_z_m"],
                        "head_q_rad": info["head_q_rad"],
                    }
                    print(
                        f"plant md5 {digest} ask {scene} {pose} "
                        f"room={result['room']!r} raw={result['raw']!r} "
                        f"pitch={info['pitch_deg']:.2f} near={info['near_m']:.3f}",
                        flush=True,
                    )
        finally:
            asker.close()

    if _md5(sw.PLANT_XML) != digest:
        raise SystemExit("plant md5 changed")
    payload = {
        "plant_md5": digest,
        "fov_walk": fov,
        "asks": poses,
        "down10_rad": DOWN_10,
        "down15_rad": DOWN_15,
        "restore_rad": RESTORE,
        "prompt": None if args.fov_only else __import__("room_ask").PROMPT,
    }
    dest = out / "summary.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {dest}", flush=True)


if __name__ == "__main__":
    main()
