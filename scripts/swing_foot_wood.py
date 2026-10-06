#!/usr/bin/env python3
"""Does the #65 wood rule fire on the robot's own swing foot?

Locked-kit walk, session head_tilt -10 deg. The wood rule is the frozen
#65 test: fraction >= 0.30 in x >= 352, y >= 192. It sees RGB only.

Foot and stool counts are labels from the segmentation buffer. They are
not inputs to the rule. The plant file is not edited.
"""
from __future__ import annotations

import argparse
import hashlib
import json
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
import head_down_ask as hd
import steer_walk as sw
import stool_leg_frustum as fr
import stool_rgb_detect as wood

PLANT_MD5 = wood.PLANT_MD5
HEAD = hd.DOWN_10
SETTLE_S = 0.50
CAPTURE_S = 1.00
SAMPLE_S = 0.02
FOOT_BODIES = (
    "l_ank_pitch_link",
    "l_ank_roll_link",
    "r_ank_pitch_link",
    "r_ank_roll_link",
)
FOOT_GEOMS = (
    "l_foot_contact",
    "r_foot_contact",
    "l_toe_viz",
    "r_toe_viz",
    "l_ank_pitch_link_mesh",
    "l_ank_roll_link_mesh",
    "r_ank_pitch_link_mesh",
    "r_ank_roll_link_mesh",
)


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _phase(session: sw.SteerSession) -> str:
    walker = session.lipm.op3 if session.lipm is not None else None
    if walker is None:
        return "unknown"
    period = float(walker.period) if float(walker.period) > 1e-6 else 0.5
    t = float(walker.time) % period
    if walker.l_ssp_start < t <= walker.l_ssp_end:
        return "L"
    if walker.r_ssp_start < t <= walker.r_ssp_end:
        return "R"
    return "DS"


def _ids(model: mj.MjModel, names: tuple[str, ...]) -> set[int]:
    found: set[int] = set()
    for name in names:
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if gid >= 0:
            found.add(gid)
        bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            continue
        for gid in range(model.ngeom):
            if int(model.geom_bodyid[gid]) == bid:
                found.add(gid)
    return found


def _foot_v(
    model: mj.MjModel,
    data: mj.MjData,
    gid: int,
    cam_pos: np.ndarray,
    cam_mat: np.ndarray,
    fovy: float,
) -> tuple[float, int, int]:
    """Min image row of corners in front of the camera, plus front and inside counts.

    A corner behind the camera is not a row. The image is 480 rows, so a
    front-row above 479 is still below the frame.
    """
    if gid < 0:
        return -1.0, 0, 0
    if int(model.geom_type[gid]) == int(mj.mjtGeom.mjGEOM_BOX):
        points = fr._box_corners(model, data, gid)
    else:
        points = np.asarray(data.geom_xpos[gid], dtype=np.float64).reshape(1, 3)
    front: list[float] = []
    inside = 0
    for point in points:
        _u, v, hit, depth = fr._project(point, cam_pos, cam_mat, fovy)
        if depth <= 1e-4:
            continue
        front.append(v)
        if hit:
            inside += 1
    if not front:
        return -1.0, 0, inside
    return float(min(front)), len(front), inside


def _capture(
    scene: str | None,
    yaw_rate: float,
    out: Path,
    tag: str,
) -> tuple[list[dict[str, float | str | bool]], dict[str, float]]:
    scene_xml = None if scene is None else em.SCENES[scene]
    session = sw.SteerSession(video=False, scene_xml=scene_xml, lipm=sw.locked_kit_config())
    hd._set_head_goal(session, HEAD)
    sent: list[em.SentCommand] = []
    em._hold_stand(session, em.STAND_S, sent)
    model = session.model
    data = session.data
    foot_ids = _ids(model, FOOT_BODIES + FOOT_GEOMS)
    left_ids = {gid for gid in foot_ids if "l_" in (mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or "")}
    right_ids = foot_ids - left_ids
    stool_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "chair_stool_b")
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    fovy = float(model.cam_fovy[cam_id])
    l_box = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact")
    r_box = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")
    renderer = mj.Renderer(model, height=480, width=640)
    last_send = -1.0
    next_sample = em.STAND_S + SETTLE_S
    end = next_sample + CAPTURE_S
    rows: list[dict[str, float | str | bool]] = []
    saved_phase: set[str] = set()
    try:
        while float(data.time) < end - 1e-9:
            now = float(data.time)
            if (now - last_send) >= (sw.VEL_RESEND_S - 1e-9):
                refusal = session.bus.vel(sw.VX_FWD_CAP, yaw_rate, now)
                if refusal:
                    raise SystemExit(refusal)
                last_send = now
            if now + 1e-9 >= next_sample and now + 1e-9 >= em.STAND_S + SETTLE_S:
                renderer.disable_segmentation_rendering()
                renderer.update_scene(data, camera="kit_cam")
                rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
                renderer.enable_segmentation_rendering()
                renderer.update_scene(data, camera="kit_cam")
                seg = np.asarray(renderer.render()).copy()
                renderer.disable_segmentation_rendering()
                geom = seg[:, :, 0]
                kind = seg[:, :, 1]
                roi_geom = geom[wood.WOOD_Y0:, wood.WOOD_X0:]
                roi_kind = kind[wood.WOOD_Y0:, wood.WOOD_X0:]
                is_geom = roi_kind == int(mj.mjtObj.mjOBJ_GEOM)
                positive, frac = wood.rgb_positive(rgb)
                patch = rgb[wood.WOOD_Y0:, wood.WOOD_X0:].astype(np.float32)
                red, green, blue = patch[:, :, 0], patch[:, :, 1], patch[:, :, 2]
                hi = np.maximum(np.maximum(red, green), blue)
                lo = np.minimum(np.minimum(red, green), blue)
                sat = (hi - lo) / np.maximum(hi, 1.0)
                wood_mask = (red > 90.0) & (red > green + 8.0) & (green > blue) & (red < 210.0)
                wood_mask &= (sat > 0.15) & (sat < 0.75)
                foot = is_geom & np.isin(roi_geom, list(foot_ids))
                left = is_geom & np.isin(roi_geom, list(left_ids)) if left_ids else np.zeros_like(foot)
                right = is_geom & np.isin(roi_geom, list(right_ids)) if right_ids else np.zeros_like(foot)
                stool = is_geom & (roi_geom == stool_id) if stool_id >= 0 else np.zeros_like(foot)
                roi_n = int(wood_mask.size)
                foot_px = int(np.count_nonzero(foot))
                foot_wood = int(np.count_nonzero(wood_mask & foot))
                stool_px = int(np.count_nonzero(stool))
                # Full-frame foot, so a foot below the ROI is still counted.
                full_foot = (kind == int(mj.mjtObj.mjOBJ_GEOM)) & np.isin(geom, list(foot_ids))
                ys, xs = np.nonzero(full_foot)
                bbox = None if ys.size == 0 else [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
                phase = _phase(session)
                cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
                cam_mat = np.asarray(data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3)
                l_v, l_front, l_inside = _foot_v(model, data, l_box, cam_pos, cam_mat, fovy)
                r_v, r_front, r_inside = _foot_v(model, data, r_box, cam_pos, cam_mat, fovy)
                row: dict[str, float | str | bool] = {
                    "t": float(data.time),
                    "phase": phase,
                    "wood_frac": frac,
                    "positive": positive,
                    "foot_px": foot_px,
                    "left_px": int(np.count_nonzero(left)),
                    "right_px": int(np.count_nonzero(right)),
                    "foot_wood_px": foot_wood,
                    "stool_px": stool_px,
                    "foot_bbox": bbox,
                    "l_foot_v": l_v,
                    "r_foot_v": r_v,
                    "l_front": l_front,
                    "r_front": r_front,
                    "l_inside": l_inside,
                    "r_inside": r_inside,
                    "foot_in_image": bool(bbox is not None),
                    "only_foot_floor": foot_px > 0 and stool_px == 0,
                }
                rows.append(row)
                if phase not in saved_phase:
                    Image.fromarray(rgb).save(out / f"{tag}_{phase}.png")
                    saved_phase.add(phase)
                next_sample += SAMPLE_S
            session.step()
    finally:
        renderer.close()
    session.assert_plant_unchanged()
    meta = {
        "min_up_z": float(session.min_up_z),
        "fault": bool(session.bus.fault),
    }
    return rows, meta


def _summary(rows: list[dict[str, float | str | bool]]) -> dict[str, float | int]:
    def _max(phase: str, key: str) -> float:
        picked = [r for r in rows if r["phase"] == phase]
        if not picked:
            return -1.0
        return float(max(float(r[key]) for r in picked))

    foot_only = [r for r in rows if bool(r["only_foot_floor"])]
    fired = [r for r in foot_only if bool(r["positive"])]
    foot_caused = [
        r for r in fired
        if float(r["wood_frac"]) >= wood.WOOD_FRAC
        and (float(r["wood_frac"]) * (480 - wood.WOOD_Y0) * (640 - wood.WOOD_X0) - float(r["foot_wood_px"]))
        < wood.WOOD_FRAC * (480 - wood.WOOD_Y0) * (640 - wood.WOOD_X0)
    ]
    return {
        "n": len(rows),
        "positive": sum(1 for r in rows if r["positive"]),
        "foot_in_roi": sum(1 for r in rows if int(r["foot_px"]) > 0),
        "foot_only_positive": len(fired),
        "foot_only_n": len(foot_only),
        "max_foot_px_L": _max("L", "left_px"),
        "max_foot_px_R": _max("R", "right_px"),
        "max_wood": max((float(r["wood_frac"]) for r in rows), default=-1.0),
        "max_foot_wood_px": max((int(r["foot_wood_px"]) for r in rows), default=0),
        "max_stool_px": max((int(r["stool_px"]) for r in rows), default=0),
        "foot_in_image": sum(1 for r in rows if r["foot_in_image"]),
        "l_foot_v_min": min((float(r["l_foot_v"]) for r in rows if float(r["l_foot_v"]) >= 0), default=-1.0),
        "l_foot_v_max": max((float(r["l_foot_v"]) for r in rows if float(r["l_foot_v"]) >= 0), default=-1.0),
        "r_foot_v_min": min((float(r["r_foot_v"]) for r in rows if float(r["r_foot_v"]) >= 0), default=-1.0),
        "r_foot_v_max": max((float(r["r_foot_v"]) for r in rows if float(r["r_foot_v"]) >= 0), default=-1.0),
        "foot_corner_inside": sum(int(r["l_inside"]) + int(r["r_inside"]) for r in rows),
        "foot_wood_would_change_bar": len(foot_caused),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=sw.ROOT / "previews" / "swing_foot_wood")
    args = parser.parse_args()
    digest = _md5(sw.PLANT_XML)
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest}")
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)
    runs = {
        "empty_straight": _capture(None, 0.0, out, "empty_straight"),
        "kitchen_yaw": _capture("kitchen", -0.25, out, "kitchen_yaw"),
    }
    if _md5(sw.PLANT_XML) != digest:
        raise SystemExit("plant md5 changed")
    payload = {
        "plant_md5": digest,
        "head_tilt_rad": HEAD,
        "wood_frac_bar": wood.WOOD_FRAC,
        "roi": [wood.WOOD_X0, wood.WOOD_Y0, 640, 480],
        "runs": {
            name: {"rows": rows, "summary": _summary(rows), "meta": meta}
            for name, (rows, meta) in runs.items()
        },
    }
    dest = out / "summary.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n")
    for name, (rows, meta) in runs.items():
        summary = payload["runs"][name]["summary"]
        print(
            f"plant md5 {digest} {name} n={summary['n']} positive={summary['positive']} "
            f"foot_in_roi={summary['foot_in_roi']} foot_in_image={summary['foot_in_image']} "
            f"foot_only_positive={summary['foot_only_positive']}/{summary['foot_only_n']} "
            f"max_foot_wood_px={summary['max_foot_wood_px']} max_wood={summary['max_wood']:.3f} "
            f"max_stool_px={summary['max_stool_px']} "
            f"Lv={summary['l_foot_v_min']:.0f}-{summary['l_foot_v_max']:.0f} "
            f"Rv={summary['r_foot_v_min']:.0f}-{summary['r_foot_v_max']:.0f} "
            f"corners_inside={summary['foot_corner_inside']} "
            f"min_up_z={meta['min_up_z']:.3f} fault={meta['fault']}",
            flush=True,
        )
        for row in rows:
            if int(row["foot_px"]) > 0 or bool(row["positive"]):
                print(
                    f"  t={float(row['t']):.3f} {row['phase']} wood={float(row['wood_frac'])*100:.1f}% "
                    f"fire={row['positive']} foot={row['foot_px']} L={row['left_px']} R={row['right_px']} "
                    f"foot_wood={row['foot_wood_px']} stool={row['stool_px']} bbox={row['foot_bbox']}",
                    flush=True,
                )
    print(f"wrote {dest}", flush=True)


if __name__ == "__main__":
    main()
