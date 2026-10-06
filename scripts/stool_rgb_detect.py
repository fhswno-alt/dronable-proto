#!/usr/bin/env python3
"""RGB-only stool cue latency. Not the mesh seg/depth path.

The 28.8 ms figure in the stop probe is kit_cam RGB plus MuJoCo
segmentation and depth. Those masks do not exist on the real kit.
This script times a detector whose only input is the RGB buffer.

The rule was frozen after reading four saved stills from that walk,
before this grid was scored:

    lower-right rectangle: x >= 352, y >= 192 on a 640x480 frame
    a pixel is wood when R > 90, R > G + 8, G > B, R < 210,
    and (max-min)/max is between 0.15 and 0.75
    positive when the wood fraction in that rectangle is >= 0.30

On those stills the stand frame (legs tiny) was 26.9% and the cue
frame (legs visible_enough) was 32.0%. The cue frame helped place
0.30, so a positive there is not an independent hit. Frames other
than that still are the check. No threshold was moved after the grid.

T_detect_rgb is wall-clock from that RGB buffer in hand, through this
rule, to CommandBus.stop's return, and only when the rule is positive.
The geometric leg/rail class is a label. It is not a detector input.
The ~4.9 s cue-to-contact lead is not T_detect_rgb. The mesh 28.8 ms
is not T_detect_rgb. T_stop and d_min are Controls' numbers, not
remeasured here.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import mujoco as mj
import numpy as np

import explore_map as em
import steer_walk as sw
import stool_leg_frustum as sl

PLANT_MD5 = sl.PLANT_MD5
# Frozen from the four stills. Do not retune from the grid below.
WOOD_X0 = 352
WOOD_Y0 = 192
WOOD_FRAC = 0.30
# Controls' published gate. Not remeasured in this script.
T_STOP_S = 0.830
D_MIN_M = 0.129
V_MPS = 0.150


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def wood_fraction(rgb: np.ndarray) -> float:
    """Wood fraction in the lower-right rectangle. RGB only."""
    patch = rgb[WOOD_Y0:, WOOD_X0:]
    red = patch[:, :, 0].astype(np.float32)
    green = patch[:, :, 1].astype(np.float32)
    blue = patch[:, :, 2].astype(np.float32)
    hi = np.maximum(np.maximum(red, green), blue)
    lo = np.minimum(np.minimum(red, green), blue)
    sat = (hi - lo) / np.maximum(hi, 1.0)
    wood = (red > 90.0) & (red > green + 8.0) & (green > blue) & (red < 210.0)
    wood &= (sat > 0.15) & (sat < 0.75)
    return float(np.count_nonzero(wood)) / float(wood.size)


def rgb_positive(rgb: np.ndarray) -> tuple[bool, float]:
    frac = wood_fraction(rgb)
    return frac >= WOOD_FRAC, frac


def _prop_contacts(model: mj.MjModel, data: mj.MjData) -> list[tuple[str, str, float]]:
    rows: list[tuple[str, str, float]] = []
    for i in range(data.ncon):
        contact = data.contact[i]
        n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
        n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
        if n1.startswith("col_"):
            prop, other = n1, n2
        elif n2.startswith("col_"):
            prop, other = n2, n1
        else:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, force)
        rows.append((other, prop, float(force[0])))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=sw.ROOT / "previews" / "stool_rgb_detect")
    args = parser.parse_args()
    digest = _md5(sw.PLANT_XML)
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest}")
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    session = sw.SteerSession(video=False, scene_xml=em.SCENES["kitchen"], lipm=sw.locked_kit_config())
    model = session.model
    data = session.data
    stool_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, sl.STOOL_GEOM)
    leg_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name) for name in sl.LEG_GEOMS]
    rail_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name) for name in sl.RAIL_GEOMS]
    hit_leg_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, sl.HIT_LEG)
    hit_rail_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, sl.HIT_RAIL)
    if min([stool_id, hit_leg_id, hit_rail_id, *leg_ids, *rail_ids]) < 0:
        raise SystemExit("missing stool geom")

    renderer = mj.Renderer(model, height=sl.HEIGHT, width=sl.WIDTH)
    sent: list[em.SentCommand] = []
    em._hold_stand(session, sl.STAND_S, sent)
    walk_end = sl.STAND_S + sl.WALK_S
    last_send = -1.0
    next_sample = float(data.time)
    samples: list[dict[str, float | str | bool | None]] = []
    contacts: list[dict[str, float | str]] = []
    first_contact_t: float | None = None
    t_detect_rgb: float | None = None
    t_detect_rule_s: float | None = None
    t_detect_stop_s: float | None = None
    cue_fired = False
    try:
        while float(data.time) < walk_end - 1e-9:
            now = float(data.time)
            if now + 1e-9 >= next_sample:
                row, rgb, _mesh_s = sl._sample_timed(
                    model, data, renderer, stool_id, leg_ids, rail_ids, hit_leg_id, hit_rail_id,
                )
                # Geometric class is a label. The detector sees rgb only.
                t0 = time.perf_counter()
                positive, frac = rgb_positive(rgb)
                rule_s = time.perf_counter() - t0
                stop_s = None
                gt = row["legs"]["class_name"]
                is_cue = t_detect_rgb is None and t_detect_rule_s is None and gt == "visible_enough"
                if is_cue:
                    # Re-time from the buffer already in hand, including the
                    # Day1 stop write when the rule is positive. A fresh bus
                    # keeps this labeling walk on the unstopped trajectory.
                    bus = sw.CommandBus()
                    t0 = time.perf_counter()
                    positive, frac = rgb_positive(rgb)
                    t_rule = time.perf_counter()
                    if positive:
                        refusal = bus.stop(float(row["t"]))
                        if refusal:
                            raise SystemExit(refusal)
                        t_detect_rgb = time.perf_counter() - t0
                        cue_fired = True
                    t_detect_rule_s = t_rule - t0
                    t_detect_stop_s = None if t_detect_rgb is None else t_detect_rgb - t_detect_rule_s
                    stop_s = t_detect_stop_s
                    rule_s = t_detect_rule_s
                samples.append(
                    {
                        "t": float(row["t"]),
                        "gt_legs": gt,
                        "gt_pixels": int(row["legs"]["pixels"]),
                        "gt_min_side": int(row["legs"]["min_side_px"]),
                        "rgb_positive": positive,
                        "wood_frac": frac,
                        "rule_s": rule_s,
                        "stop_write_s": stop_s,
                    }
                )
                next_sample += sl.SAMPLE_S
            if session.bus.fault:
                session.bus.stand(now)
            elif (now - last_send) >= (sw.VEL_RESEND_S - 1e-9):
                refusal = session.bus.vel(sl.VX, sl.YAW, now)
                if refusal:
                    raise SystemExit(refusal)
                last_send = now
                sent.append(em.SentCommand(now, "vel", sl.VX, sl.YAW))
            session.step()
            if first_contact_t is None:
                hits = _prop_contacts(model, data)
                if hits:
                    first_contact_t = float(data.time)
                    contacts.extend(
                        {"t": first_contact_t, "geom": g, "prop": p, "normal_n": n}
                        for g, p, n in hits
                    )
    finally:
        renderer.close()

    session.assert_plant_unchanged()
    if _md5(sw.PLANT_XML) != digest:
        raise SystemExit("plant md5 changed")

    cue_rows = [s for s in samples if s["gt_legs"] == "visible_enough"]
    t_cue = float(cue_rows[0]["t"]) if cue_rows else None
    before = [s for s in samples if t_cue is None or float(s["t"]) < t_cue - 1e-9]
    # Approach labels stop at the published contact. The fall is separate.
    approach = [
        s for s in samples
        if t_cue is not None and float(s["t"]) + 1e-9 >= t_cue
        and (first_contact_t is None or float(s["t"]) < first_contact_t - 1e-9)
    ]
    fp = [s for s in before if s["rgb_positive"]]
    fn = [s for s in approach if not s["rgb_positive"]]
    first_rgb = next((s for s in samples if s["rgb_positive"]), None)
    margin_rhs = None if t_detect_rgb is None else V_MPS * (t_detect_rgb + T_STOP_S)
    payload = {
        "plant_md5": digest,
        "method": "lower-right wood fraction >= 0.30 on kit_cam RGB",
        "wood_x0": WOOD_X0,
        "wood_y0": WOOD_Y0,
        "wood_frac_bar": WOOD_FRAC,
        "fit_note": "0.30 sits between the stand still (26.9%) and the cue still (32.0%). The cue still is not an independent test.",
        "mesh_path_s": 0.028835650000473834,
        "mesh_path_is_kit_T_detect": False,
        "t_cue": t_cue,
        "T_detect_rgb_s": t_detect_rgb,
        "rule_s": t_detect_rule_s,
        "stop_write_s": t_detect_stop_s,
        "cue_fired": cue_fired,
        "first_rgb_positive_t": None if first_rgb is None else first_rgb["t"],
        "first_contact_t": first_contact_t,
        "fp_before_cue": len(fp),
        "n_before_cue": len(before),
        "fn_cue_to_contact": len(fn),
        "n_cue_to_contact": len(approach),
        "T_stop_s_controls": T_STOP_S,
        "d_min_m_controls": D_MIN_M,
        "v_mps": V_MPS,
        "margin_rhs_m": margin_rhs,
        "margin_holds": None if margin_rhs is None else bool(D_MIN_M + 1e-12 >= margin_rhs),
        "min_up_z": float(session.min_up_z),
        "fault_reason": session.bus.fault_reason,
        "samples": samples,
    }
    dest = out / "summary.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n")
    print(
        f"plant md5 {digest} t_cue={t_cue} T_detect_rgb_s={t_detect_rgb} "
        f"cue_fired={cue_fired} first_rgb={None if first_rgb is None else first_rgb['t']} "
        f"first_contact={first_contact_t}",
        flush=True,
    )
    print(
        f"plant md5 {digest} fp_before_cue={len(fp)}/{len(before)} "
        f"fn_cue_to_contact={len(fn)}/{len(approach)}",
        flush=True,
    )
    if margin_rhs is not None:
        print(
            f"plant md5 {digest} margin d_min {D_MIN_M:.3f} m vs "
            f"v*(T_detect_rgb+T_stop) {margin_rhs:.3f} m holds={D_MIN_M + 1e-12 >= margin_rhs}",
            flush=True,
        )
    print(f"wrote {dest}", flush=True)


if __name__ == "__main__":
    main()
