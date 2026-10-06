#!/usr/bin/env python3
"""Prefer FAIL: finder pixel vs sim-projected stool-leg floor.

Same kitchen yaw −0.25 collision walk as ``walk_latch``. The ray is fed
the RGB finder pixel from ``hazard_finder``, and again the sim projection,
and the two ``estimate_hazard`` results are compared. This script does
not call ``CommandBus.stop``. Head tilt stays off. Buffer stays off.
``HAZARD_PAD_M`` stays 0.020. The plant file is not edited.

``t_cue`` is not ``T_detect``. A numeric miss here is the result, not a
soft pass. Not kit-safe. Not go-anywhere.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("DISPLAY", ":1")
os.environ.setdefault("MUJOCO_GL", "glfw")

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

import hazard_finder as hf
import mono_toe_gate as gate
import ray_corridor as rc
import steer_walk as sw

SAMPLE_S = 0.05
MATCH_PX = 48.0
T_END = 7.2
OUT = sw.ROOT / "previews" / "finder_ray"


@dataclass(frozen=True)
class GtLeg:
    short: str
    name: str
    u: float
    v: float
    estimate: rc.HazardEstimate


@dataclass(frozen=True)
class Match:
    short: str
    gt_u: float
    gt_v: float
    gt_eye: float
    gt_gap: float
    gt_side: float
    gt_in: bool
    pixel_err: float | None
    finder_u: float | None
    finder_v: float | None
    finder_eye: float | None
    finder_gap: float | None
    finder_side: float | None
    finder_in: bool | None
    clipped: bool
    missed: bool


def _plant_md5() -> str:
    return hashlib.md5(sw.PLANT_XML.read_bytes()).hexdigest()


def _short(name: str) -> str:
    return name.rsplit("_", 1)[-1]


def _gt_legs(
    model: mj.MjModel,
    data: mj.MjData,
    cid: int,
    session: sw.SteerSession,
    step_off: float,
) -> list[GtLeg]:
    body_rot = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
    yaw, pitch, roll = rc.imu_from_body(body_rot)
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
    pan_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_pan")
    pan = 0.0 if pan_id < 0 else float(data.qpos[int(model.jnt_qposadr[pan_id])])
    cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    rot = rc.camera_rotation_from_imu(yaw, pitch, roll, tilt, pan)
    fwd = gate.body_forward_xy(data, session.bid_body)
    body = rc.BodyFrame(
        (float(data.qpos[0]), float(data.qpos[1])),
        (float(fwd[0]), float(fwd[1])),
    )
    pose = rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2])))
    yaw_rate = float(session.bus.applied_yaw_rate)
    rows: list[GtLeg] = []
    for name, point in gate._leg_floor_centers(model, data):
        if "stool_b_leg_" not in name:
            continue
        pix = rc.project_point(point, cam, rot)
        if pix is None:
            continue
        u, v = pix
        if not (0.0 <= u < rc.WIDTH and 0.0 <= v < rc.HEIGHT):
            continue
        est = rc.estimate_hazard(
            u, v,
            cam=pose, body=body,
            imu_roll_rad=roll, imu_pitch_rad=pitch,
            head_tilt_rad=tilt, yaw_rate=yaw_rate,
            step_off_m=step_off, head_pan_rad=pan,
        )
        if est is None:
            continue
        rows.append(GtLeg(_short(name), name, u, v, est))
    return rows


def _pose(
    model: mj.MjModel,
    data: mj.MjData,
    cid: int,
    session: sw.SteerSession,
) -> tuple[rc.KitCamPose, rc.BodyFrame, float, float, float, float, float]:
    body_rot = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
    _yaw, pitch, roll = rc.imu_from_body(body_rot)
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
    pan_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_pan")
    pan = 0.0 if pan_id < 0 else float(data.qpos[int(model.jnt_qposadr[pan_id])])
    cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    fwd = gate.body_forward_xy(data, session.bid_body)
    body = rc.BodyFrame(
        (float(data.qpos[0]), float(data.qpos[1])),
        (float(fwd[0]), float(fwd[1])),
    )
    pose = rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2])))
    return pose, body, roll, pitch, tilt, pan, float(session.bus.applied_yaw_rate)


def _match(
    gt: GtLeg,
    cues: tuple[hf.HazardCue, ...],
    pose: rc.KitCamPose,
    body: rc.BodyFrame,
    roll: float,
    pitch: float,
    tilt: float,
    pan: float,
    yaw_rate: float,
    step_off: float,
) -> Match:
    best: tuple[float, hf.HazardCue] | None = None
    for cue in cues:
        du = (float(cue.column) + 0.5) - gt.u
        dv = (float(cue.contact_row) + 0.5) - gt.v
        dist = math.hypot(du, dv)
        if best is None or dist < best[0]:
            best = (dist, cue)
    missed = best is None or best[0] > MATCH_PX
    if missed or best is None:
        return Match(
            gt.short, gt.u, gt.v, gt.estimate.eye_range, gt.estimate.toe_gap_m,
            gt.estimate.sideways_m, gt.estimate.in_corridor,
            None, None, None, None, None, None, None, False, True,
        )
    dist, cue = best
    finder_eye: float | None = None
    finder_gap: float | None = None
    finder_side: float | None = None
    finder_in: bool | None = None
    if cue.u is not None and cue.v is not None and not cue.too_close:
        est = rc.estimate_hazard(
            cue.u, cue.v,
            cam=pose, body=body,
            imu_roll_rad=roll, imu_pitch_rad=pitch,
            head_tilt_rad=tilt, yaw_rate=yaw_rate,
            step_off_m=step_off, head_pan_rad=pan,
        )
        if est is not None:
            finder_eye = est.eye_range
            finder_gap = est.toe_gap_m
            finder_side = est.sideways_m
            finder_in = est.in_corridor
    return Match(
        gt.short, gt.u, gt.v, gt.estimate.eye_range, gt.estimate.toe_gap_m,
        gt.estimate.sideways_m, gt.estimate.in_corridor,
        dist, cue.u, cue.v, finder_eye, finder_gap, finder_side, finder_in,
        cue.too_close, False,
    )


def _draw(rgb: np.ndarray, gts: list[GtLeg], cues: tuple[hf.HazardCue, ...], path: Path) -> None:
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    for gt in gts:
        x = int(round(gt.u))
        y = int(round(gt.v))
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), outline=(40, 220, 80))
        draw.text((x + 6, y - 8), gt.short, fill=(40, 220, 80))
    for cue in cues:
        x = cue.column
        y = cue.contact_row
        color = (255, 40, 40) if cue.too_close else (255, 210, 40)
        draw.line((x - 6, y, x + 6, y), fill=color)
        draw.line((x, y - 6, x, y + 6), fill=color)
    image.save(path)


def _summary_leg(rows: list[Match]) -> dict[str, float | int | bool | None]:
    matched = [row for row in rows if not row.missed]
    ranged = [row for row in matched if row.finder_eye is not None and row.finder_in is not None]
    errs = [float(row.pixel_err) for row in matched if row.pixel_err is not None]
    gt_in = [row for row in rows if row.gt_in]
    both_in = [row for row in ranged if row.gt_in and row.finder_in]
    lost = [row for row in ranged if row.gt_in and not row.finder_in]
    gained = [row for row in ranged if row.finder_in and not row.gt_in]
    return {
        "n": len(rows),
        "matched": len(matched),
        "missed": sum(1 for row in rows if row.missed),
        "clipped": sum(1 for row in matched if row.clipped),
        "pixel_err_med": None if not errs else float(np.median(np.array(errs, dtype=np.float64))),
        "pixel_err_max": None if not errs else float(max(errs)),
        "gt_in": len(gt_in),
        "both_in": len(both_in),
        "gt_in_finder_out": len(lost),
        "finder_in_gt_out": len(gained),
        "finder_in": sum(1 for row in ranged if row.finder_in),
        "eye_delta_med": None if not ranged else float(np.median(np.array(
            [float(row.finder_eye) - row.gt_eye for row in ranged], dtype=np.float64,
        ))),
        "gap_delta_med": None if not ranged else float(np.median(np.array(
            [float(row.finder_gap) - row.gt_gap for row in ranged], dtype=np.float64,
        ))),
        "closest_side": None if not ranged else float(max(float(row.finder_side) for row in ranged)),
        "closest_side_abs_gt": None if not rows else float(min(abs(row.gt_side) for row in rows)),
    }


def main() -> None:
    hf.self_check()
    rc.self_check()
    digest = _plant_md5()
    if digest != "207f3d5e9c6a72e16f7aa0c8d224f75e":
        raise SystemExit(f"plant md5 {digest}")
    if abs(rc.HAZARD_PAD_M - 0.020) > 1e-12:
        raise SystemExit("pad moved")
    OUT.mkdir(parents=True, exist_ok=True)
    session = sw.SteerSession(video=False, scene_xml=gate.SCENE, lipm=sw.locked_kit_config())
    model = session.model
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(T_END, "vel", sw.VX_FWD_CAP, -sw.YAW_RATE_CAP, "turn"),
    ))
    reach: list[tuple[float, float, float]] = []
    samples: list[dict[str, object]] = []
    by_leg: dict[str, list[Match]] = {"0": [], "1": [], "2": [], "3": []}
    contact: tuple[float, str, str, float] | None = None
    head_peak = 0.0
    next_sample = 1.0
    saved: set[str] = set()
    gate_m = rc.d_min(0.0)
    try:
        while float(session.data.time) < T_END - 1e-9 and contact is None:
            now = float(session.data.time)
            driver.publish(session.bus, now)
            if now + 1e-9 >= 1.0 and now + 1e-6 >= next_sample:
                data = session.data
                tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
                head_peak = max(head_peak, abs(tilt))
                toes = gate.toe_samples(session, cid)
                reach.append((
                    now,
                    next(row.offset_m for row in toes if row.side == "L"),
                    next(row.offset_m for row in toes if row.side == "R"),
                ))
                step_off, step_side, step_t = gate._high_water(reach)
                renderer.disable_segmentation_rendering()
                renderer.disable_depth_rendering()
                renderer.update_scene(data, camera="kit_cam")
                rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
                cues = hf.find_hazard_cues(rgb)
                primary = hf.find_hazard_pixel(rgb)
                pose, body, roll, pitch, tilt_r, pan, yaw_rate = _pose(model, data, cid, session)
                gts = _gt_legs(model, data, cid, session, step_off)
                matches = [
                    _match(gt, cues, pose, body, roll, pitch, tilt_r, pan, yaw_rate, step_off)
                    for gt in gts
                ]
                for row in matches:
                    by_leg.setdefault(row.short, []).append(row)
                primary_est: rc.HazardEstimate | None = None
                if primary is not None and primary.u is not None and primary.v is not None:
                    primary_est = rc.estimate_hazard(
                        primary.u, primary.v,
                        cam=pose, body=body,
                        imu_roll_rad=roll, imu_pitch_rad=pitch,
                        head_tilt_rad=tilt_r, yaw_rate=yaw_rate,
                        step_off_m=step_off, head_pan_rad=pan,
                    )
                would_latch = False
                latch_why = ""
                if primary is not None and primary.too_close:
                    would_latch = True
                    latch_why = f"too_close col={primary.column} row={primary.contact_row}"
                elif primary_est is not None and primary_est.in_corridor and primary_est.toe_gap_m <= gate_m:
                    would_latch = True
                    latch_why = (
                        f"pixel ({primary.u:.0f},{primary.v:.0f}) "
                        f"gap={primary_est.toe_gap_m:.3f} side={primary_est.sideways_m:+.3f}"
                    )
                in_cues: list[str] = []
                for cue in cues:
                    if cue.u is None or cue.v is None:
                        continue
                    est = rc.estimate_hazard(
                        cue.u, cue.v,
                        cam=pose, body=body,
                        imu_roll_rad=roll, imu_pitch_rad=pitch,
                        head_tilt_rad=tilt_r, yaw_rate=yaw_rate,
                        step_off_m=step_off, head_pan_rad=pan,
                    )
                    if est is not None and est.in_corridor:
                        in_cues.append(
                            f"({cue.u:.0f},{cue.v:.0f}) gap={est.toe_gap_m:.3f} side={est.sideways_m:+.3f}"
                        )
                sample: dict[str, object] = {
                    "t": now,
                    "step_off": step_off,
                    "step_side": step_side,
                    "step_t": step_t,
                    "head_tilt": tilt,
                    "n_cues": len(cues),
                    "n_clipped": sum(1 for cue in cues if cue.too_close),
                    "primary_too_close": None if primary is None else primary.too_close,
                    "primary_u": None if primary is None else primary.u,
                    "primary_v": None if primary is None else primary.v,
                    "primary_column": None if primary is None else primary.column,
                    "primary_row": None if primary is None else primary.contact_row,
                    "primary_eye": None if primary_est is None else primary_est.eye_range,
                    "primary_gap": None if primary_est is None else primary_est.toe_gap_m,
                    "primary_side": None if primary_est is None else primary_est.sideways_m,
                    "primary_in": None if primary_est is None else primary_est.in_corridor,
                    "would_latch_tdetect0": would_latch,
                    "latch_why": latch_why,
                    "finder_in_corridor": in_cues,
                    "legs": [
                        {
                            "leg": row.short,
                            "gt_u": row.gt_u,
                            "gt_v": row.gt_v,
                            "gt_eye": row.gt_eye,
                            "gt_gap": row.gt_gap,
                            "gt_side": row.gt_side,
                            "gt_in": row.gt_in,
                            "pixel_err": row.pixel_err,
                            "finder_u": row.finder_u,
                            "finder_v": row.finder_v,
                            "finder_eye": row.finder_eye,
                            "finder_gap": row.finder_gap,
                            "finder_side": row.finder_side,
                            "finder_in": row.finder_in,
                            "clipped": row.clipped,
                            "missed": row.missed,
                        }
                        for row in matches
                    ],
                }
                samples.append(sample)
                stamp = f"{now:.2f}"
                if stamp.startswith(("1.90", "4.50", "5.50", "5.90")) or (
                    primary is not None and primary.too_close and "clip" not in saved
                ):
                    _draw(rgb, gts, cues, OUT / f"frame_{stamp}.png")
                    saved.add(stamp)
                    if primary is not None and primary.too_close:
                        saved.add("clip")
                bits = " ".join(
                    f"{row.short}:{'miss' if row.missed else ('clip' if row.clipped else f'{row.pixel_err:.0f}px')} "
                    f"gt={'IN' if row.gt_in else 'out'}"
                    + (
                        ""
                        if row.finder_in is None
                        else f"/{'IN' if row.finder_in else 'out'}"
                    )
                    for row in matches
                )
                print(
                    f"t={now:.3f} cues={len(cues)} clip={sample['n_clipped']} "
                    f"primary={latch_why or 'none'} latch={would_latch} {bits}",
                    flush=True,
                )
                next_sample += SAMPLE_S
            session.step()
            hit = gate._prop_hit(model, session.data)
            if hit is not None:
                contact = (float(session.data.time), hit[0], hit[1], hit[2])
    finally:
        renderer.close()
    if _plant_md5() != digest:
        raise SystemExit("plant md5 changed")
    step = gate._high_water(reach)
    leg_summary = {key: _summary_leg(rows) for key, rows in by_leg.items()}
    first_latch = next((row for row in samples if bool(row["would_latch_tdetect0"])), None)
    leg0_in = [
        row for row in by_leg.get("0", [])
        if row.finder_in is True
    ]
    leg2_ranged_in = [
        row for row in by_leg.get("2", [])
        if row.gt_in and row.finder_in is True
    ]
    payload: dict[str, object] = {
        "plant_md5": digest,
        "pad_m": rc.HAZARD_PAD_M,
        "buffer_m": 0.0,
        "head_tilt_commanded": False,
        "head_tilt_peak_rad": head_peak,
        "d_min_tdetect0": gate_m,
        "sample_s": SAMPLE_S,
        "match_px": MATCH_PX,
        "step_off": {"m": step[0], "side": step[1], "t": step[2]},
        "contact": None if contact is None else {
            "t": contact[0], "robot": contact[1], "prop": contact[2], "n": contact[3],
        },
        "stop_sent": False,
        "first_would_latch": None if first_latch is None else {
            "t": first_latch["t"],
            "why": first_latch["latch_why"],
            "primary_too_close": first_latch["primary_too_close"],
            "primary_u": first_latch["primary_u"],
            "primary_v": first_latch["primary_v"],
            "primary_gap": first_latch["primary_gap"],
            "primary_in": first_latch["primary_in"],
        },
        "leg0_finder_in_corridor_n": len(leg0_in),
        "leg2_both_in_corridor_n": len(leg2_ranged_in),
        "legs": leg_summary,
        "samples": samples,
    }
    dest = OUT / "summary.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n")
    print(
        f"plant {digest} pad={rc.HAZARD_PAD_M:.3f} buffer=0 head_peak={head_peak:.4f} "
        f"step_off={step[0]:+.3f} {step[1]} t={step[2]:.3f} contact={contact} "
        f"stop_sent=False samples={len(samples)}",
        flush=True,
    )
    for key in ("0", "1", "2", "3"):
        row = leg_summary[key]
        print(
            f"leg_{key} n={row['n']} matched={row['matched']} miss={row['missed']} "
            f"clip={row['clipped']} err_med={row['pixel_err_med']} err_max={row['pixel_err_max']} "
            f"gt_in={row['gt_in']} both_in={row['both_in']} lost={row['gt_in_finder_out']} "
            f"gained={row['finder_in_gt_out']} eye_dmed={row['eye_delta_med']} "
            f"gap_dmed={row['gap_delta_med']}",
            flush=True,
        )
    if first_latch is not None:
        print(
            f"first would-latch T_detect=0 t={first_latch['t']} why={first_latch['latch_why']}",
            flush=True,
        )
    print(
        f"leg_0 finder in_corridor samples={len(leg0_in)} "
        f"leg_2 both in_corridor samples={len(leg2_ranged_in)}",
        flush=True,
    )
    print(f"wrote {dest}", flush=True)


if __name__ == "__main__":
    main()
