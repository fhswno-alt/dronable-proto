#!/usr/bin/env python3
"""Prefer FAIL: does kit_cam see the kitchen stool legs before the foot hits?

Replays the locked-kit kitchen walk from the collision probe: 1.0 s stand,
then 8.0 s of vel(+0.150, -0.25), resent at 10 Hz. It does not edit the
plant, the room XML, the gait, the bus, or the collision proxies.

The size classes are the earlier frustum bar, frozen before this walk:

- out_of_frustum: zero pixels, and the part does not project into the image
- occluded: zero pixels, but some of it projects into the image
- tiny: some pixels, under 3072 (1% of 640x480) or a mask shorter than 36 px
- visible_enough: a mesh region that clears that bar

Group-3 collision boxes are not drawn. Pixel counts are the visual mesh
`chair_stool_b`. A pixel of that mesh is a leg/rail pixel when its
reconstructed world z is below the seat box (z < 0.2562), which is the
authored split in room_kitchen.xml, not a threshold picked after the counts.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path
from typing import TypedDict

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
WIDTH = 640
HEIGHT = 480
FRAME_PX = WIDTH * HEIGHT
TINY_FRAC = 0.01
TINY_MIN_SIDE_PX = 36
TINY_PX = int(TINY_FRAC * FRAME_PX)
# Seat-box bottom on col_chair_stool_b. Legs and rails are authored below it.
LEG_Z_MAX = 0.3686 - 0.1124
STAND_S = 1.0
WALK_S = 8.0
SAMPLE_S = 0.10
VX = 0.150
YAW = -0.25
STOOL_GEOM = "chair_stool_b"
HIT_LEG = "col_chair_stool_b_leg_2"
HIT_RAIL = "col_chair_stool_b_rail_yp"
LEG_GEOMS = (
    "col_chair_stool_b_leg_0",
    "col_chair_stool_b_leg_1",
    "col_chair_stool_b_leg_2",
    "col_chair_stool_b_leg_3",
)
RAIL_GEOMS = (
    "col_chair_stool_b_rail_xn",
    "col_chair_stool_b_rail_xp",
    "col_chair_stool_b_rail_yn",
    "col_chair_stool_b_rail_yp",
)


class PartCount(TypedDict):
    pixels: int
    min_side_px: int
    bbox: list[int] | None
    class_name: str
    bearing_deg: float | None
    center_z_m: float | None


class SampleRow(TypedDict):
    t: float
    up_z: float
    xy: list[float]
    yaw_deg: float
    cam_z: float
    pitch_deg: float
    stool: PartCount
    legs: PartCount
    seat: PartCount
    stool_pixel_z_min: float | None
    stool_pixel_z_max: float | None
    legs_pixel_mean_z: float | None
    seat_pixel_mean_z: float | None
    hit_leg_projects: bool
    hit_rail_projects: bool


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _classify(pixels: int, min_side: int, projects: bool) -> str:
    if pixels <= 0:
        return "occluded" if projects else "out_of_frustum"
    if pixels < TINY_PX or min_side < TINY_MIN_SIDE_PX:
        return "tiny"
    return "visible_enough"


def _mask_stats(mask: np.ndarray) -> tuple[int, int, list[int] | None]:
    count = int(np.count_nonzero(mask))
    if count == 0:
        return 0, 0, None
    ys, xs = np.nonzero(mask)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    min_side = min(bbox[2] - bbox[0] + 1, bbox[3] - bbox[1] + 1)
    return count, int(min_side), bbox


def _project(
    point: np.ndarray,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fovy_deg: float,
) -> tuple[float, float, bool, float]:
    local = cam_mat.T @ (point - cam_pos)
    depth = -float(local[2])
    if depth <= 1e-4:
        return -1.0, -1.0, False, depth
    half_h = math.tan(math.radians(fovy_deg) * 0.5)
    half_w = half_h * (WIDTH / HEIGHT)
    x_ndc = float(local[0]) / depth / half_w
    y_ndc = float(local[1]) / depth / half_h
    u = (x_ndc + 1.0) * 0.5 * WIDTH
    v = (1.0 - y_ndc) * 0.5 * HEIGHT
    inside = 0.0 <= u < WIDTH and 0.0 <= v < HEIGHT
    return u, v, inside, depth


def _box_corners(model: mj.MjModel, data: mj.MjData, gid: int) -> np.ndarray:
    origin = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
    hx, hy, hz = (float(v) for v in model.geom_size[gid])
    local = np.array(
        [
            [sx * hx, sy * hy, sz * hz]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    return origin + (rot @ local.T).T


def _projects(points: np.ndarray, cam_pos: np.ndarray, cam_mat: np.ndarray, fovy: float) -> bool:
    for point in points:
        if _project(point, cam_pos, cam_mat, fovy)[2]:
            return True
    return False


def _world_z_from_depth(
    xs: np.ndarray,
    ys: np.ndarray,
    depth: np.ndarray,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fovy_deg: float,
) -> np.ndarray:
    """World z of each pixel. Depth is the camera-plane distance, meters."""
    half_h = math.tan(math.radians(fovy_deg) * 0.5)
    half_w = half_h * (WIDTH / HEIGHT)
    z = depth[ys, xs].astype(np.float64)
    x_ndc = ((xs.astype(np.float64) + 0.5) / WIDTH) * 2.0 - 1.0
    y_ndc = 1.0 - ((ys.astype(np.float64) + 0.5) / HEIGHT) * 2.0
    local = np.stack(
        (x_ndc * z * half_w, y_ndc * z * half_h, -z),
        axis=1,
    )
    world = cam_pos + local @ cam_mat.T
    return world[:, 2]


def _part(
    pixels: int,
    min_side: int,
    bbox: list[int] | None,
    projects: bool,
    bearing: float | None,
    center_z: float | None,
) -> PartCount:
    return {
        "pixels": pixels,
        "min_side_px": min_side,
        "bbox": bbox,
        "class_name": _classify(pixels, min_side, projects),
        "bearing_deg": bearing,
        "center_z_m": center_z,
    }


def _bearing(point: np.ndarray, origin_xy: tuple[float, float], yaw_deg: float) -> float:
    bearing = math.degrees(math.atan2(float(point[1]) - origin_xy[1], float(point[0]) - origin_xy[0]))
    return _wrap_deg(bearing - yaw_deg)


def _contacts(model: mj.MjModel, data: mj.MjData, ankle_bodies: set[int], prop_names: dict[int, str]) -> list[tuple[str, str, float]]:
    rows: list[tuple[str, str, float]] = []
    for i in range(data.ncon):
        contact = data.contact[i]
        g1 = int(contact.geom1)
        g2 = int(contact.geom2)
        b1 = int(model.geom_bodyid[g1])
        b2 = int(model.geom_bodyid[g2])
        prop = ""
        ankle = ""
        if g1 in prop_names and b2 in ankle_bodies:
            prop = prop_names[g1]
            ankle = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b2) or ""
        elif g2 in prop_names and b1 in ankle_bodies:
            prop = prop_names[g2]
            ankle = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b1) or ""
        else:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, force)
        rows.append((ankle, prop, float(force[0])))
    return rows


def _sample(
    model: mj.MjModel,
    data: mj.MjData,
    renderer: mj.Renderer,
    stool_id: int,
    leg_ids: list[int],
    rail_ids: list[int],
    hit_leg_id: int,
    hit_rail_id: int,
) -> tuple[SampleRow, np.ndarray]:
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
    cam_mat = np.asarray(data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3)
    look = -cam_mat[:, 2]
    pitch = math.degrees(math.atan2(float(look[2]), math.hypot(float(look[0]), float(look[1]))))
    fovy = float(model.cam_fovy[cam_id])
    renderer.disable_segmentation_rendering()
    renderer.disable_depth_rendering()
    renderer.update_scene(data, camera="kit_cam")
    rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="kit_cam")
    seg = np.asarray(renderer.render()).copy()
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="kit_cam")
    depth = np.asarray(renderer.render(), dtype=np.float32).copy()
    renderer.disable_depth_rendering()

    geom_id = seg[:, :, 0]
    geom_type = seg[:, :, 1]
    stool_mask = (geom_type == int(mj.mjtObj.mjOBJ_GEOM)) & (geom_id == stool_id)
    pixels, min_side, bbox = _mask_stats(stool_mask)
    ys, xs = np.nonzero(stool_mask)
    leg_mask = np.zeros_like(stool_mask)
    seat_mask = np.zeros_like(stool_mask)
    stool_z_min: float | None = None
    stool_z_max: float | None = None
    legs_mean_z: float | None = None
    seat_mean_z: float | None = None
    if pixels > 0:
        world_z = _world_z_from_depth(xs, ys, depth, cam_pos, cam_mat, fovy)
        stool_z_min = float(np.min(world_z))
        stool_z_max = float(np.max(world_z))
        leg_sel = world_z < LEG_Z_MAX
        leg_mask[ys[leg_sel], xs[leg_sel]] = True
        seat_mask[ys[~leg_sel], xs[~leg_sel]] = True
        if bool(np.any(leg_sel)):
            legs_mean_z = float(np.mean(world_z[leg_sel]))
        if bool(np.any(~leg_sel)):
            seat_mean_z = float(np.mean(world_z[~leg_sel]))
    leg_px, leg_side, leg_bbox = _mask_stats(leg_mask)
    seat_px, seat_side, seat_bbox = _mask_stats(seat_mask)

    stool_origin = np.asarray(data.geom_xpos[stool_id], dtype=np.float64)
    robot_xy = (float(data.qpos[0]), float(data.qpos[1]))
    yaw_deg = math.degrees(math.atan2(
        2.0 * (float(data.qpos[3]) * float(data.qpos[6]) + float(data.qpos[4]) * float(data.qpos[5])),
        1.0 - 2.0 * (float(data.qpos[5]) ** 2 + float(data.qpos[6]) ** 2),
    ))
    leg_points = np.concatenate([_box_corners(model, data, gid) for gid in leg_ids], axis=0)
    rail_points = np.concatenate([_box_corners(model, data, gid) for gid in rail_ids], axis=0)
    hit_leg_pts = _box_corners(model, data, hit_leg_id)
    hit_rail_pts = _box_corners(model, data, hit_rail_id)
    stool_projects = _projects(np.vstack((leg_points, rail_points, stool_origin.reshape(1, 3))), cam_pos, cam_mat, fovy)
    legs_project = _projects(leg_points, cam_pos, cam_mat, fovy)
    # The visual mesh origin sits on the seat. Seat projection uses that point.
    seat_projects = _projects(np.array([stool_origin]), cam_pos, cam_mat, fovy) or seat_px > 0

    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    up_z = float(data.xmat[body_id].reshape(3, 3)[2, 2])
    row: SampleRow = {
        "t": float(data.time),
        "up_z": up_z,
        "xy": [float(data.qpos[0]), float(data.qpos[1])],
        "yaw_deg": yaw_deg,
        "cam_z": float(cam_pos[2]),
        "pitch_deg": pitch,
        "stool": _part(pixels, min_side, bbox, stool_projects, _bearing(stool_origin, robot_xy, yaw_deg), float(stool_origin[2])),
        "legs": _part(
            leg_px,
            leg_side,
            leg_bbox,
            legs_project,
            _bearing(np.mean(leg_points, axis=0), robot_xy, yaw_deg),
            float(np.mean(leg_points[:, 2])),
        ),
        "seat": _part(seat_px, seat_side, seat_bbox, seat_projects, _bearing(stool_origin, robot_xy, yaw_deg), float(stool_origin[2])),
        "stool_pixel_z_min": stool_z_min,
        "stool_pixel_z_max": stool_z_max,
        "legs_pixel_mean_z": legs_mean_z,
        "seat_pixel_mean_z": seat_mean_z,
        "hit_leg_projects": _projects(hit_leg_pts, cam_pos, cam_mat, fovy),
        "hit_rail_projects": _projects(hit_rail_pts, cam_pos, cam_mat, fovy),
    }
    return row, rgb


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=sw.ROOT / "previews" / "stool_leg_frustum")
    args = parser.parse_args()
    digest = _md5(sw.PLANT_XML)
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest}")
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)
    kitchen = em.SCENES["kitchen"]
    session = sw.SteerSession(video=False, scene_xml=kitchen, lipm=sw.locked_kit_config())
    model = session.model
    data = session.data
    stool_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, STOOL_GEOM)
    if stool_id < 0:
        raise SystemExit(f"missing {STOOL_GEOM}")
    leg_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name) for name in LEG_GEOMS]
    rail_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name) for name in RAIL_GEOMS]
    hit_leg_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, HIT_LEG)
    hit_rail_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, HIT_RAIL)
    if min(leg_ids + rail_ids + [hit_leg_id, hit_rail_id]) < 0:
        raise SystemExit("missing stool collision geom")
    prop_names = {gid: mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or "" for gid in leg_ids + rail_ids}
    ankle_bodies = {
        mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link"),
        mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link"),
    }
    renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
    sent: list[em.SentCommand] = []
    em._hold_stand(session, STAND_S, sent)
    walk_end = STAND_S + WALK_S
    last_send = -1.0
    # Sample on a 0.10 s grid from the end of the stand. Starting at 0 would
    # catch up one control tick at a time and stamp a burst of 8 ms frames.
    next_sample = float(data.time)
    samples: list[SampleRow] = []
    rgbs: list[np.ndarray] = []
    frames: dict[str, np.ndarray] = {}
    contacts: list[dict[str, float | str]] = []
    first_contact_t: float | None = None
    min_up_z = 1.0
    try:
        while float(data.time) < walk_end - 1e-9:
            now = float(data.time)
            if now + 1e-9 >= next_sample:
                row, rgb = _sample(
                    model, data, renderer, stool_id, leg_ids, rail_ids, hit_leg_id, hit_rail_id,
                )
                samples.append(row)
                rgbs.append(rgb)
                if abs(now - STAND_S) < SAMPLE_S * 0.5 and "stand" not in frames:
                    frames["stand"] = rgb
                next_sample += SAMPLE_S
            if (now - last_send) >= (sw.VEL_RESEND_S - 1e-9):
                # After the tip the bus drops velocity. Keep stepping on stand
                # through the rest of the 8 s window so the later stool
                # contacts are on the same clock as the collision probe.
                if session.bus.fault:
                    session.bus.stand(now)
                else:
                    refusal = session.bus.vel(VX, YAW, now)
                    if refusal:
                        raise SystemExit(refusal)
                last_send = now
            session.step()
            min_up_z = min(min_up_z, session._up_z())
            for ankle, prop, normal in _contacts(model, data, ankle_bodies, prop_names):
                contacts.append({"t": float(data.time), "ankle": ankle, "prop": prop, "normal_n": normal})
                if first_contact_t is None:
                    first_contact_t = float(data.time)
        # One more sample at the end of the vel window.
        row, rgb = _sample(model, data, renderer, stool_id, leg_ids, rail_ids, hit_leg_id, hit_rail_id)
        samples.append(row)
        rgbs.append(rgb)
        frames["end"] = rgb
    finally:
        renderer.close()

    def _first(kind: str, part: str) -> float | None:
        for row in samples:
            if first_contact_t is not None and row["t"] >= first_contact_t - 1e-9:
                break
            counted = row["legs"] if part == "legs" else row["stool"]
            if counted["class_name"] == kind:
                return row["t"]
        return None

    first_leg_ok = _first("visible_enough", "legs")
    first_stool_ok = _first("visible_enough", "stool")
    before = [row for row in samples if first_contact_t is None or row["t"] < first_contact_t - 1e-9]
    best_legs = max(before, key=lambda row: int(row["legs"]["pixels"])) if before else None
    contact_row = None
    if first_contact_t is not None:
        contact_row = min(samples, key=lambda row: abs(row["t"] - first_contact_t))

    if "stand" in frames:
        Image.fromarray(frames["stand"]).save(out / "kitchen_stand.png")
    if "end" in frames:
        Image.fromarray(frames["end"]).save(out / "kitchen_end.png")
    if best_legs is not None:
        Image.fromarray(rgbs[samples.index(best_legs)]).save(out / "kitchen_best_legs.png")
    if contact_row is not None:
        Image.fromarray(rgbs[samples.index(contact_row)]).save(out / "kitchen_contact.png")
    peaks: dict[str, float] = {}
    for item in contacts:
        key = f"{item['ankle']}|{item['prop']}"
        peaks[key] = max(peaks.get(key, 0.0), float(item["normal_n"]))
    episodes: list[dict[str, float | str]] = []
    for item in contacts:
        t = float(item["t"])
        normal = float(item["normal_n"])
        if (
            not episodes
            or item["ankle"] != episodes[-1]["ankle"]
            or item["prop"] != episodes[-1]["prop"]
            or t - float(episodes[-1]["t_end"]) > 0.12
        ):
            episodes.append(
                {
                    "ankle": str(item["ankle"]),
                    "prop": str(item["prop"]),
                    "t_start": t,
                    "t_peak": t,
                    "t_end": t,
                    "peak_n": normal,
                }
            )
            continue
        episodes[-1]["t_end"] = t
        if normal > float(episodes[-1]["peak_n"]):
            episodes[-1]["peak_n"] = normal
            episodes[-1]["t_peak"] = t
    session.assert_plant_unchanged()
    if _md5(sw.PLANT_XML) != digest:
        raise SystemExit("plant md5 changed")
    payload = {
        "plant_md5": digest,
        "scene": "kitchen",
        "stand_s": STAND_S,
        "walk_s": WALK_S,
        "vx": VX,
        "yaw_rate": YAW,
        "sample_s": SAMPLE_S,
        "leg_z_max_m": LEG_Z_MAX,
        "tiny_px": TINY_PX,
        "tiny_min_side_px": TINY_MIN_SIDE_PX,
        "min_up_z": min_up_z,
        "fault": session.bus.fault,
        "fault_reason": session.bus.fault_reason,
        "end_xy": [float(data.qpos[0]), float(data.qpos[1])],
        "end_yaw_deg": samples[-1]["yaw_deg"] if samples else None,
        "first_contact_t": first_contact_t,
        "first_stool_visible_enough_t": first_stool_ok,
        "first_legs_visible_enough_t": first_leg_ok,
        "lead_s_stool": None if first_stool_ok is None or first_contact_t is None else first_contact_t - first_stool_ok,
        "lead_s_legs": None if first_leg_ok is None or first_contact_t is None else first_contact_t - first_leg_ok,
        "best_legs_before_contact": best_legs,
        "nearest_contact_sample": contact_row,
        "contact_peaks_n": peaks,
        "contact_episodes": episodes,
        "n_contact_samples": len(contacts),
        "samples": samples,
    }
    dest = out / "summary.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n")
    print(
        f"first_contact={first_contact_t} stool_ok={first_stool_ok} legs_ok={first_leg_ok} "
        f"min_up_z={min_up_z:.3f} fault={session.bus.fault_reason!r} samples={len(samples)}",
        flush=True,
    )
    for episode in episodes:
        print(
            f"episode {episode['ankle']} {episode['prop']} "
            f"peak={float(episode['peak_n']):.1f}N t={float(episode['t_peak']):.2f}",
            flush=True,
        )
    if best_legs is not None:
        print(
            f"best legs before hit t={best_legs['t']:.2f} "
            f"legs={best_legs['legs']['class_name']}:{best_legs['legs']['pixels']} "
            f"stool={best_legs['stool']['class_name']}:{best_legs['stool']['pixels']} "
            f"seat={best_legs['seat']['class_name']}:{best_legs['seat']['pixels']}",
            flush=True,
        )
    print(f"wrote {dest}", flush=True)


if __name__ == "__main__":
    main()
