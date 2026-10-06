#!/usr/bin/env python3
"""Wire the kitchen stool leg/rail cue to a Day1 CommandBus stop.

Same locked-kit walk as the frustum probe: 1.0 s stand, then
vel(+0.150, -0.25) until the leg/rail band is visible_enough. That sim
time is t_cue. It is not T_detect.

T_detect is wall-clock on that one frame. It starts when the kit_cam RGB
buffer is in hand (the cue is already in the image, or it is not this
frame) and ends when CommandBus.stop returns. The work in that span is
the leg/rail read from the visual mesh (segmentation and depth, group-3
boxes are not the mask) plus the bus write. No Moondream. The ~4.9 s from
t_cue to the old contact is not detection time. Controls measures T_stop
(gait) separately. This script does not report a gait stopping distance.

After the stop, vel is not sent again. The sim runs on to t = 9.0 s, the
same window as the contact probe. Plant, gait, and clamps stay as they are.
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
from PIL import Image

import explore_map as em
import steer_walk as sw
import stool_leg_frustum as sl

PLANT_MD5 = sl.PLANT_MD5
KNEE_ACTUATORS = ("l_knee_pos", "r_knee_pos")
TAU_BAR_NM = 2.33


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _prop_contacts(model: mj.MjModel, data: mj.MjData) -> list[tuple[str, str, float]]:
    rows: list[tuple[str, str, float]] = []
    for i in range(data.ncon):
        contact = data.contact[i]
        g1 = int(contact.geom1)
        g2 = int(contact.geom2)
        n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g1) or ""
        n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g2) or ""
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


def _episodes(contacts: list[dict[str, float | str]]) -> list[dict[str, float | str]]:
    episodes: list[dict[str, float | str]] = []
    for item in contacts:
        t = float(item["t"])
        normal = float(item["normal_n"])
        if (
            not episodes
            or item["geom"] != episodes[-1]["geom"]
            or item["prop"] != episodes[-1]["prop"]
            or t - float(episodes[-1]["t_end"]) > 0.12
        ):
            episodes.append(
                {
                    "geom": str(item["geom"]),
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
    return episodes


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=sw.ROOT / "previews" / "stool_leg_stop")
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
    knee_idx = {name: session.act_idx[name] for name in KNEE_ACTUATORS}
    leg_idx: dict[str, int] = {}
    for i in range(model.nu):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if any(tok in name for tok in ("hip_", "knee", "ank_")):
            leg_idx[name] = i

    renderer = mj.Renderer(model, height=sl.HEIGHT, width=sl.WIDTH)
    sent: list[em.SentCommand] = []
    em._hold_stand(session, sl.STAND_S, sent)
    walk_end = sl.STAND_S + sl.WALK_S
    last_send = -1.0
    next_sample = float(data.time)
    stopped = False
    t_cue: float | None = None
    t_stop: float | None = None
    t_detect_s: float | None = None
    cue_row: sl.SampleRow | None = None
    cue_rgb: np.ndarray | None = None
    contacts: list[dict[str, float | str]] = []
    knee_peak = {name: 0.0 for name in KNEE_ACTUATORS}
    knee_peak_t = {name: 0.0 for name in KNEE_ACTUATORS}
    leg_peak = {name: 0.0 for name in leg_idx}
    leg_peak_t = {name: 0.0 for name in leg_idx}
    try:
        while float(data.time) < walk_end - 1e-9:
            now = float(data.time)
            if (not stopped) and now + 1e-9 >= next_sample:
                row, rgb, post_rgb_s = sl._sample_timed(
                    model, data, renderer, stool_id, leg_ids, rail_ids, hit_leg_id, hit_rail_id,
                )
                next_sample += sl.SAMPLE_S
                if row["legs"]["class_name"] == "visible_enough":
                    t_cue = float(row["t"])
                    t_write = time.perf_counter()
                    refusal = session.bus.stop(t_cue)
                    if refusal:
                        raise SystemExit(refusal)
                    t_stop = float(data.time)
                    # post_rgb_s ends before the write. Add the write. The sim
                    # clock does not move inside this span.
                    t_detect_s = post_rgb_s + (time.perf_counter() - t_write)
                    stopped = True
                    cue_row = row
                    cue_rgb = rgb
                    sent.append(em.SentCommand(t_stop, "stop", 0.0, 0.0))
            if not stopped and (now - last_send) >= (sw.VEL_RESEND_S - 1e-9):
                refusal = session.bus.vel(sl.VX, sl.YAW, now)
                if refusal:
                    raise SystemExit(refusal)
                last_send = now
                sent.append(em.SentCommand(now, "vel", sl.VX, sl.YAW))
            session.step()
            for name, idx in knee_idx.items():
                force = abs(float(data.actuator_force[idx]))
                if force > knee_peak[name]:
                    knee_peak[name] = force
                    knee_peak_t[name] = float(data.time)
            for name, idx in leg_idx.items():
                force = abs(float(data.actuator_force[idx]))
                if force > leg_peak[name]:
                    leg_peak[name] = force
                    leg_peak_t[name] = float(data.time)
            for geom, prop, normal in _prop_contacts(model, data):
                contacts.append(
                    {"t": float(data.time), "geom": geom, "prop": prop, "normal_n": normal}
                )
    finally:
        renderer.close()

    if cue_rgb is not None:
        Image.fromarray(cue_rgb).save(out / "kitchen_cue_stop.png")
    session.assert_plant_unchanged()
    if _md5(sw.PLANT_XML) != digest:
        raise SystemExit("plant md5 changed")

    episodes = _episodes(contacts)
    over_bar = [
        {
            "actuator": name,
            "peak_nm": leg_peak[name],
            "t": leg_peak_t[name],
        }
        for name in leg_idx
        if leg_peak[name] > TAU_BAR_NM
    ]
    over_bar.sort(key=lambda item: float(item["peak_nm"]), reverse=True)
    payload = {
        "plant_md5": digest,
        "scene": "kitchen",
        "vx": sl.VX,
        "yaw_rate": sl.YAW,
        "t_cue": t_cue,
        "T_detect_s": t_detect_s,
        "t_stop": t_stop,
        "sim_s_cue_to_stop": None if t_cue is None or t_stop is None else t_stop - t_cue,
        "cue_to_old_contact_s": None if t_cue is None else 6.848 - t_cue,
        "nomenclature": {
            "t_cue": "sim time of the first leg/rail visible_enough sample",
            "T_detect": "wall-clock seconds from that frame's RGB buffer to CommandBus.stop; not the cue-to-contact lead",
            "t_stop": "sim time of the Day1 stop write",
            "T_stop": "not measured here; Controls owns gait stop distance and time",
        },
        "cue": cue_row,
        "n_prop_contacts": len(contacts),
        "contact_episodes": episodes,
        "min_up_z": float(session.min_up_z),
        "fault": bool(session.bus.fault),
        "fault_reason": session.bus.fault_reason,
        "end_xy": [float(data.qpos[0]), float(data.qpos[1])],
        "end_yaw_deg": float(np.degrees(session.yaw())),
        "end_mode": session.bus.mode,
        "knee_peaks": [
            {"actuator": name, "peak_nm": knee_peak[name], "t": knee_peak_t[name]}
            for name in KNEE_ACTUATORS
        ],
        "leg_peaks_over_2_33_nm": over_bar,
        "max_leg_tau_nm": float(session.max_leg_tau),
    }
    dest = out / "summary.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n")

    print(f"plant md5 {digest} t_cue={t_cue} T_detect_s={t_detect_s} t_stop={t_stop}", flush=True)
    if cue_row is not None:
        legs = cue_row["legs"]
        print(
            f"plant md5 {digest} cue legs {legs['class_name']} {legs['pixels']} px "
            f"side {legs['min_side_px']} bearing {legs['bearing_deg']}",
            flush=True,
        )
    print(
        f"plant md5 {digest} prop_contacts={len(contacts)} episodes={len(episodes)} "
        f"min_up_z={session.min_up_z:.3f} fault={session.bus.fault_reason!r} "
        f"end_mode={session.bus.mode}",
        flush=True,
    )
    for name in KNEE_ACTUATORS:
        print(
            f"plant md5 {digest} knee {name} peak {knee_peak[name]:.3f} Nm at t={knee_peak_t[name]:.2f} s",
            flush=True,
        )
    for item in over_bar:
        print(
            f"plant md5 {digest} leg {item['actuator']} peak {float(item['peak_nm']):.3f} Nm "
            f"at t={float(item['t']):.2f} s over {TAU_BAR_NM:.2f}",
            flush=True,
        )
    if not over_bar:
        print(f"plant md5 {digest} no leg actuator over {TAU_BAR_NM:.2f} Nm", flush=True)
    for episode in episodes:
        print(
            f"plant md5 {digest} contact {episode['geom']} {episode['prop']} "
            f"peak {float(episode['peak_n']):.1f} N at t={float(episode['t_peak']):.2f} s",
            flush=True,
        )
    print(f"wrote {dest}", flush=True)


if __name__ == "__main__":
    main()
