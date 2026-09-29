#!/usr/bin/env python3
"""Gate M: open-hold disturbance reject on ±30° companion (no range bump).

Criteria: docs/GATE_M_AI_CRITERIA.md
- M00 open ≥25° + tip≥8 + coupling
- M01 hold ≥1.0 s @ ≥25°; declared disturbance (≤0.5 s impulse / ≤1.0 s window)
- M02 after disturb: tip≥8; panel ≥15° within 2.0 s; then close Δ ≥10°
- M03 2/2 consecutive cycles; continuous MP4

Disturbance (Controls choice): **small panel push** — declared xfrc on
`door_panel_link` (closing-sense +X world force), applied only inside custom
substep during `disturb` phase. No undeclared xfrc. Soft-pass forbidden.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
from pathlib import Path

import numpy as np
import mujoco as mj

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
os.environ.setdefault("MUJOCO_GL", "glfw")

from score_gate_f import ITER, PLANT_F, CKPT, GateFEnv, body_x_for_cam_dist, CTRL_HZ
from score_gate_g import ik_reach_pose, _hand_lever_contact
from score_gate_h import _hinge_id, _pin_arms_head, OPEN_CMD, CLOSE_CMD
from score_gate_i import _panel_hinge_id, _angle_deg
from score_gate_k import (
    _hud,
    _panel_cam,
    _world_cam,
    _mark_base_edge,
    _arm_seed,
    _panel_spring,
    _sha16,
)

COMPANION_LOCK_MD5 = "59cc408eda07037a58f92ad27da045d6"
CKPT_SHA16 = "9ffaa1a21b607bf6"

# Declared disturbance (document in note/table)
DISTURB_TYPE = "panel_push_xfrc_door_panel_link"
DISTURB_FORCE_N = np.array([1.2, 0.0, 0.0])  # +X world ≈ closing sense on free edge
DISTURB_S = 0.35  # ≤0.5 s impulse window
REJECT_S = 2.0    # observe reject window after disturb ends


def _md5(p: Path) -> str:
    return hashlib.md5(p.read_bytes()).hexdigest()


def _substep_m(env: GateFEnv, arm_q: dict, *, disturb_xfrc: np.ndarray | None = None):
    """Like score_gate_h._substep but re-applies declared disturb xfrc after clear."""
    m, d = env.model, env.data
    n = env.steps_per_ctrl
    bid_panel = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "door_panel_link")
    for _ in range(n):
        for jn, val in arm_q.items():
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                d.qpos[m.jnt_qposadr[jid]] = val
                d.qvel[m.jnt_dofadr[jid]] = 0.0
        d.qpos[m.jnt_qposadr[env.jid_tilt]] = env.head_tilt
        d.qfrc_applied[:] = 0
        d.xfrc_applied[:] = 0
        if disturb_xfrc is not None and bid_panel >= 0:
            d.xfrc_applied[bid_panel, :3] = disturb_xfrc
        mj.mj_step(m, d)


def _hud_m(frame, dx, dy, x, y, t, bout, panel_deg, lever_deg, phase, disturb_on: bool):
    title = f"kit · Gate M · {phase}"
    if disturb_on:
        title = f"kit · Gate M · DISTURB ON · {phase}"
    out = _hud(frame, dx, dy, x, y, t, bout, panel_deg, lever_deg, title=title)
    if disturb_on:
        from PIL import Image, ImageDraw, ImageFont
        img = Image.fromarray(out)
        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 22)
        except Exception:
            font = ImageFont.load_default()
        draw.rectangle([6, 130, 340, 158], fill=(180, 0, 0))
        draw.text((10, 132), "DISTURBANCE ON (declared panel push)", fill=(255, 255, 0), font=font)
        out = np.asarray(img)
    return out


def _render_m(env, qpos_log, xy_log, bout_log, t_log, phase_log, disturb_log, stride=5, fps=10):
    import imageio.v2 as imageio
    ITER.mkdir(parents=True, exist_ok=True)
    r = mj.Renderer(env.model, height=480, width=640)
    wcam = _world_cam(env.model)
    pcam = _panel_cam(env.model)
    jid_p = _panel_hinge_id(env.model)
    jid_l = _hinge_id(env.model)
    x0, y0 = xy_log[0]
    paths = {
        "kit": ITER / "M03_kit.mp4",
        "panel": ITER / "M03_panel.mp4",
        "world": ITER / "M03_world.mp4",
        "dual": ITER / "M03.mp4",
        "dual_s9": ITER / "M03_dual_s9.mp4",
    }
    n = len(qpos_log)
    print(f"[M] stream-render n={n} stride={stride} → M03*.mp4", flush=True)
    writers = {
        k: imageio.get_writer(str(p), fps=fps, codec="libx264", quality=8, macro_block_size=1)
        for k, p in paths.items()
    }
    try:
        for i in range(0, n, stride):
            env.data.qpos[:] = qpos_log[i]
            env.data.qvel[:] = 0
            mj.mj_forward(env.model, env.data)
            x, y = xy_log[i]
            dx, dy = x - x0, y - y0
            panel_deg = _angle_deg(env.model, env.data, jid_p)
            lever_deg = _angle_deg(env.model, env.data, jid_l)
            tt, bb = t_log[i], bout_log[i]
            ph = phase_log[i]
            dist_on = bool(disturb_log[i])
            r.update_scene(env.data, camera=env.cid)
            kit = _hud_m(
                r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg, ph, dist_on
            )
            r.update_scene(env.data, camera=wcam)
            world = _hud(
                _mark_base_edge(r.render().copy()),
                dx, dy, x, y, tt, bb, panel_deg, lever_deg,
                title=f"§9 feet · {ph}" + (" · DISTURB" if dist_on else ""),
            )
            r.update_scene(env.data, camera=pcam)
            panel = _hud(
                r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg,
                title=f"oblique free-edge · {ph}" + (" · DISTURB ON" if dist_on else ""),
            )
            if dist_on:
                from PIL import Image, ImageDraw
                for arr_name, arr in (("panel", panel),):
                    im = Image.fromarray(arr)
                    dr = ImageDraw.Draw(im)
                    dr.rectangle([6, 130, 300, 152], fill=(180, 0, 0))
                    dr.text((10, 132), "DISTURBANCE ON", fill=(255, 255, 0))
                    panel = np.asarray(im)
            dual = np.concatenate([kit, panel], axis=1)
            dual_s9 = np.concatenate([kit, world], axis=1)
            writers["kit"].append_data(kit)
            writers["world"].append_data(world)
            writers["panel"].append_data(panel)
            writers["dual"].append_data(dual)
            writers["dual_s9"].append_data(dual_s9)
            if i % 500 == 0:
                print(f"[M] render {i}/{n} phase={ph} panel={panel_deg:+.1f} disturb={dist_on}", flush=True)
    finally:
        for w in writers.values():
            w.close()
        r.close()
    for tag in ("M00", "M01", "M02"):
        dst = ITER / f"{tag}.mp4"
        if dst.exists() or dst.is_symlink():
            dst.unlink()
        try:
            dst.symlink_to("M03.mp4")
        except OSError:
            import shutil
            shutil.copy2(paths["dual"], dst)
    print("[M] wrote M03.mp4 (+ M00–M02 → M03)", flush=True)
    return {k: str(p.relative_to(ROOT)) for k, p in paths.items()}


def score_disturb_reject(n_cycles: int = 2) -> dict:
    arm_q, _, tilt = _arm_seed()
    cam_dist = 0.11
    t_settle_open, t_settle_close = 0.8, 1.2
    push_s, hold_s = 10.0, 2.0
    disturb_s = DISTURB_S
    reject_s = REJECT_S
    close_s, recover_s = 6.5, 2.5
    push_to, rev_cmd_end = 29.8, -4.0
    soft_close = True
    close_pull_scale = 0.22
    g_push, g_close, g_hold = 1.35, 1.15, 1.40
    cycle_s = push_s + hold_s + disturb_s + reject_s + close_s + recover_s
    episode_s = t_settle_open + t_settle_close + n_cycles * cycle_s

    env = GateFEnv(
        gait_t=0.75, episode_s=episode_s, stand_hold=episode_s, ramp_t=0.0,
        head_tilt_deg=tilt, walk=False, body_x0=body_x_for_cam_dist(cam_dist),
    )
    env.reset()
    m, d = env.model, env.data
    spring = _panel_spring(m)
    jid_p = _panel_hinge_id(m)
    jid_l = _hinge_id(m)
    assert jid_p >= 0 and jid_l >= 0
    for i in range(m.nu):
        n = mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "panel" in n.lower():
            raise RuntimeError(f"panel actuator: {n}")

    d.qpos[m.jnt_qposadr[jid_p]] = 0.0
    d.qpos[m.jnt_qposadr[jid_l]] = 0.0
    mj.mj_forward(m, d)

    x0 = float(d.qpos[env.q_free])
    y0 = float(d.qpos[env.q_free + 1])
    sid_hand = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "l_hand_site")
    reach = ["l_sho_pitch", "l_sho_roll", "l_el_pitch", "l_el_yaw"]
    jids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, j) for j in reach]
    panel_hinge_xy = np.array([0.45, -0.05])

    def arc(deg: float) -> np.ndarray:
        rest = np.array([0.40, 0.0]) - panel_hinge_xy
        th = np.radians(deg)
        c, s = np.cos(th), np.sin(th)
        xy = panel_hinge_xy + np.array([[c, -s], [s, c]]) @ rest
        return np.array([xy[0], xy[1], 0.275])

    cycles = []
    for _ in range(n_cycles):
        cycles.append({
            "peak_panel_deg": 0.0,
            "max_lever_deg": 0.0,
            "hold25_s": 0.0,
            "_hold_run": 0.0,
            "disturb_t0": None,
            "disturb_t1": None,
            "panel_at_disturb_start": None,
            "panel_min_during_disturb": None,
            "panel_min_during_reject": None,
            "panel_at_reject_end": None,
            "time_to_panel15_after_disturb_s": None,
            "_seen15_after": False,
            "min_abs_during_close": None,
            "close_delta_deg": 0.0,
            "contact_rising_s": 0.0,
            "contact_s": 0.0,
            "coupled_ok": False,
            "open25_ok": False,
            "hold_ok": False,
            "disturb_applied_ok": False,
            "reject_panel15_ok": False,
            "close_ok": False,
            "pass": False,
            "_prev_abs": 0.0,
        })

    qpos_log, xy_log, bout_log, t_log, phase_log, disturb_log = [], [], [], [], [], []
    tip_run = tip = 0.0
    tip_after_disturb_ok = True  # cumulative; per-cycle checked via global tip
    global_max_panel = global_max_lever = 0.0
    global_rising = global_contact = 0.0
    prev_abs = 0.0
    t0 = t_settle_open + t_settle_close
    disturb_force_applied_s = 0.0

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        disturb_xfrc = None
        if t < t_settle_open:
            gcmd, phase, cmd, a_rev = OPEN_CMD, "settle_open", 0.0, 0.0
            bi = -1
        elif t < t0:
            a = (t - t_settle_open) / t_settle_close
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            phase, cmd, a_rev, bi = "settle_close", 0.0, 0.0, -1
        else:
            tb = t - t0
            bi = min(int(tb // cycle_s), n_cycles - 1)
            local = tb - bi * cycle_s
            if local < push_s:
                gcmd, phase = CLOSE_CMD, "push"
                cmd = push_to * (local / push_s)
                a_rev = 0.0
            elif local < push_s + hold_s:
                gcmd, phase, cmd, a_rev = CLOSE_CMD, "hold", push_to, 0.0
            elif local < push_s + hold_s + disturb_s:
                gcmd, phase, cmd, a_rev = CLOSE_CMD, "disturb", push_to, 0.0
                disturb_xfrc = DISTURB_FORCE_N
                disturb_force_applied_s += 1 / CTRL_HZ
            elif local < push_s + hold_s + disturb_s + reject_s:
                gcmd, phase, cmd, a_rev = CLOSE_CMD, "reject", push_to, 0.0
            elif local < push_s + hold_s + disturb_s + reject_s + close_s:
                a_rev = (local - push_s - hold_s - disturb_s - reject_s) / close_s
                if soft_close:
                    gcmd = CLOSE_CMD * (1.0 - 0.55 * a_rev) + OPEN_CMD * (0.55 * a_rev)
                else:
                    gcmd = CLOSE_CMD * (1.0 - 0.85 * a_rev) + OPEN_CMD * (0.85 * a_rev)
                phase = "close"
                ang_now = abs(_angle_deg(m, d, jid_p))
                cmd = ang_now + (rev_cmd_end - ang_now) * a_rev
            else:
                a_rec = (local - push_s - hold_s - disturb_s - reject_s - close_s) / recover_s
                gcmd = OPEN_CMD * (1.0 - a_rec) + CLOSE_CMD * a_rec
                phase, cmd, a_rev = "recover", 0.0, a_rec

        mj.mj_forward(m, d)
        if phase in ("push", "hold", "disturb", "reject", "close", "recover"):
            if phase == "close":
                target = arc(cmd)
                if soft_close:
                    target = target + np.array([
                        0.03 * close_pull_scale,
                        -0.05 * a_rev * close_pull_scale,
                        0.0,
                    ])
                g = g_close
            elif phase == "recover":
                target = arc(0.0) + np.array([-0.02, 0.02, 0.0]) * (1.0 - a_rev)
                g = g_hold
            else:
                # push/hold/disturb/reject: keep pushing open arc
                target = arc(cmd)
                g = g_push if phase == "push" else g_hold
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = target - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), err)
                for jid, ddq, jn in zip(jids, dq, reach):
                    lo, hi = m.jnt_range[jid]
                    nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + g * ddq, lo, hi))
                    d.qpos[m.jnt_qposadr[jid]] = nv
                    arm_q[jn] = nv
            except np.linalg.LinAlgError:
                pass

        _pin_arms_head(env, arm_q, gcmd)
        _substep_m(env, arm_q, disturb_xfrc=disturb_xfrc)

        qpos_log.append(d.qpos.copy())
        xy_log.append((float(d.qpos[env.q_free]), float(d.qpos[env.q_free + 1])))
        bout_log.append(bi)
        t_log.append(t)
        phase_log.append(phase)
        disturb_log.append(disturb_xfrc is not None)

        Rmat = d.xmat[env.bid_body].reshape(3, 3)
        up = float(Rmat[2, 2])
        if up > 0.5:
            tip_run += 1 / CTRL_HZ
            tip = max(tip, tip_run)
        else:
            tip_run = 0.0
            if phase in ("disturb", "reject", "close") and bi >= 0:
                tip_after_disturb_ok = False

        ang_p = _angle_deg(m, d, jid_p)
        ang_l = _angle_deg(m, d, jid_l)
        global_max_panel = max(global_max_panel, abs(ang_p))
        global_max_lever = max(global_max_lever, abs(ang_l))
        contacting = _hand_lever_contact(m, d)
        if contacting:
            global_contact += 1 / CTRL_HZ
            if abs(ang_p) > prev_abs + 1e-4:
                global_rising += 1 / CTRL_HZ

        if bi >= 0:
            b = cycles[bi]
            if phase in ("push", "hold"):
                if abs(ang_p) > abs(b["peak_panel_deg"]):
                    b["peak_panel_deg"] = ang_p
                b["max_lever_deg"] = max(b["max_lever_deg"], abs(ang_l))
                if abs(ang_p) >= 25.0 - 1e-6:
                    b["_hold_run"] += 1 / CTRL_HZ
                    b["hold25_s"] = max(b["hold25_s"], b["_hold_run"])
                else:
                    b["_hold_run"] = 0.0
                if contacting:
                    b["contact_s"] += 1 / CTRL_HZ
                    if abs(ang_p) > b["_prev_abs"] + 1e-4:
                        b["contact_rising_s"] += 1 / CTRL_HZ
            elif phase == "disturb":
                if b["disturb_t0"] is None:
                    b["disturb_t0"] = t
                    b["panel_at_disturb_start"] = abs(ang_p)
                b["disturb_t1"] = t
                if b["panel_min_during_disturb"] is None:
                    b["panel_min_during_disturb"] = abs(ang_p)
                else:
                    b["panel_min_during_disturb"] = min(b["panel_min_during_disturb"], abs(ang_p))
                if contacting:
                    b["contact_s"] += 1 / CTRL_HZ
            elif phase == "reject":
                if b["panel_min_during_reject"] is None:
                    b["panel_min_during_reject"] = abs(ang_p)
                else:
                    b["panel_min_during_reject"] = min(b["panel_min_during_reject"], abs(ang_p))
                b["panel_at_reject_end"] = abs(ang_p)
                if (not b["_seen15_after"]) and abs(ang_p) >= 15.0 - 1e-6:
                    # time since disturb end
                    t_end = b["disturb_t1"] if b["disturb_t1"] is not None else t
                    b["time_to_panel15_after_disturb_s"] = t - t_end
                    # if already ≥15 at reject start, time≈0
                    if abs(ang_p) >= 15 and b["time_to_panel15_after_disturb_s"] < 0:
                        b["time_to_panel15_after_disturb_s"] = 0.0
                    b["_seen15_after"] = True
                if contacting:
                    b["contact_s"] += 1 / CTRL_HZ
            elif phase == "close":
                if b["min_abs_during_close"] is None:
                    b["min_abs_during_close"] = abs(ang_p)
                else:
                    b["min_abs_during_close"] = min(b["min_abs_during_close"], abs(ang_p))
                if contacting:
                    b["contact_s"] += 1 / CTRL_HZ
            b["_prev_abs"] = abs(ang_p)
        prev_abs = abs(ang_p)

    tip_ok = tip >= 8.0 and tip_after_disturb_ok
    for i, b in enumerate(cycles):
        if b["min_abs_during_close"] is None:
            b["min_abs_during_close"] = abs(b["peak_panel_deg"])
        b["close_delta_deg"] = abs(b["peak_panel_deg"]) - float(b["min_abs_during_close"])
        b["open25_ok"] = abs(b["peak_panel_deg"]) >= 25.0 - 1e-6
        b["hold_ok"] = b["hold25_s"] >= 1.0 - 1e-9
        b["disturb_applied_ok"] = (
            b["disturb_t0"] is not None
            and b["disturb_t1"] is not None
            and (b["disturb_t1"] - b["disturb_t0"]) >= 0.2
        )
        # reject: panel ≥15 within 2.0 s after disturb ends
        # If panel stayed ≥15 entire reject, time_to is 0 at first reject sample
        if b["time_to_panel15_after_disturb_s"] is None:
            # never recovered to 15 during reject — check if already ≥15 at end of disturb
            if b["panel_min_during_disturb"] is not None and b["panel_min_during_disturb"] >= 15:
                b["time_to_panel15_after_disturb_s"] = 0.0
                b["_seen15_after"] = True
        # Also: if first reject frame already ≥15, set 0
        if b["_seen15_after"] and b["time_to_panel15_after_disturb_s"] is None:
            b["time_to_panel15_after_disturb_s"] = 0.0
        b["reject_panel15_ok"] = (
            b["time_to_panel15_after_disturb_s"] is not None
            and b["time_to_panel15_after_disturb_s"] <= 2.0 + 1e-9
        )
        b["close_ok"] = b["close_delta_deg"] >= 10.0 - 1e-6
        b["coupled_ok"] = b["contact_rising_s"] > 0.1 and b["contact_s"] > 0.5
        b["tip_ok"] = tip_ok
        b["within_30"] = abs(b["peak_panel_deg"]) <= 30.0 + 0.05
        b["pass"] = bool(
            b["open25_ok"] and b["hold_ok"] and b["disturb_applied_ok"]
            and b["reject_panel15_ok"] and b["close_ok"] and b["coupled_ok"]
            and b["tip_ok"] and b["within_30"]
        )
        for k in list(b.keys()):
            if k.startswith("_"):
                del b[k]
        b["cycle_index"] = i

    n_pass = sum(1 for b in cycles if b["pass"])
    dx = float(d.qpos[env.q_free]) - x0
    dy = float(d.qpos[env.q_free + 1]) - y0
    print(
        f"[M] physics done n_pass={n_pass}/{n_cycles} tip={tip:.2f}s "
        f"Δx={dx*1000:+.1f}mm Δy={dy*1000:+.1f}mm disturb_applied_s={disturb_force_applied_s:.2f}",
        flush=True,
    )
    for b in cycles:
        print(
            f"  cyc{b['cycle_index']}: peak={b['peak_panel_deg']:+.2f} hold={b['hold25_s']:.2f} "
            f"dist_min={b['panel_min_during_disturb']} t15={b['time_to_panel15_after_disturb_s']} "
            f"Δclose={b['close_delta_deg']:.2f} pass={b['pass']}",
            flush=True,
        )

    mp4s = _render_m(env, qpos_log, xy_log, bout_log, t_log, phase_log, disturb_log)
    del qpos_log[:]

    return {
        "tip": tip,
        "tip_ok": tip_ok,
        "tip_after_disturb_ok": tip_after_disturb_ok,
        "max_abs_panel_deg": global_max_panel,
        "max_abs_lever_deg": global_max_lever,
        "cycles": cycles,
        "n_cycles": n_cycles,
        "n_pass": n_pass,
        "success_rate": f"{n_pass}/{n_cycles}",
        "all_cycles_pass": n_pass == n_cycles,
        "coupled_ok": global_rising > 0.2 and global_contact > 1.0,
        "hand_lever_contact_s": global_contact,
        "contact_while_panel_rising_s": global_rising,
        "panel_qpos_scripted": False,
        "panel_actuator": False,
        "stand_xy_hold": False,
        "free_joint_xy_pinned": False,
        "base_dx_m": dx,
        "base_dy_m": dy,
        "panel_spring": spring,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "fall": tip < 1.0,
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": tilt,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "companion_md5": _md5(PLANT_F),
        "within_30deg_range": global_max_panel <= 30.0 + 0.05,
        "cam_dist_cmd": cam_dist,
        "disturbance": {
            "type": DISTURB_TYPE,
            "body": "door_panel_link",
            "force_N_world": DISTURB_FORCE_N.tolist(),
            "duration_s": DISTURB_S,
            "applied_s_total": disturb_force_applied_s,
            "declared": True,
            "undeclared_xfrc": False,
            "note": "Closing-sense +X push on door_panel_link via xfrc during disturb phase only; custom substep re-applies after env clear",
        },
        "controller": {
            "push_s": push_s, "hold_s": hold_s, "disturb_s": disturb_s,
            "reject_s": reject_s, "close_s": close_s, "recover_s": recover_s,
            "push_to": push_to, "cam_dist": cam_dist, "xy_pin": False,
        },
        "mp4s": mp4s,
        "episode_s": episode_s,
    }


def main():
    ITER.mkdir(parents=True, exist_ok=True)
    sha = _sha16(CKPT)
    plant_md5 = _md5(PLANT_F)
    print(
        f"[M] HONEST disturb-reject plant={PLANT_F.name} md5={plant_md5} ckpt={sha} "
        f"disturb={DISTURB_TYPE} F={DISTURB_FORCE_N.tolist()}N Δt={DISTURB_S}s",
        flush=True,
    )
    if sha != CKPT_SHA16:
        print(f"[M] WARN ckpt sha {sha} != {CKPT_SHA16}", flush=True)
    if plant_md5 != COMPANION_LOCK_MD5:
        print(f"[M] WARN md5 {plant_md5} != lock {COMPANION_LOCK_MD5}", flush=True)

    ep = score_disturb_reject(n_cycles=2)
    tip_ok = ep["tip_ok"]
    no_cheat = (
        not ep["panel_qpos_scripted"] and not ep["panel_actuator"]
        and not ep["stand_xy_hold"] and not ep["free_joint_xy_pinned"]
        and not ep["assist"] and not ep["freeze"] and not ep["vision_in_walk_obs"]
        and ep["within_30deg_range"]
        and ep["disturbance"]["declared"] and not ep["disturbance"]["undeclared_xfrc"]
    )
    cycles = ep["cycles"]
    c0 = cycles[0] if cycles else {}

    def st(r):
        return "PASS" if r.get("pass") else "FAIL"

    l00 = {
        "tag": "M00",
        "pass": bool(c0.get("open25_ok") and tip_ok and c0.get("coupled_ok") and no_cheat),
        "peak_panel_deg": c0.get("peak_panel_deg"),
        "tip": ep["tip"],
        "coupled_ok": c0.get("coupled_ok"),
        "mp4": "previews/ainex_walk/iterate/M00.mp4",
    }
    l01 = {
        "tag": "M01",
        "pass": bool(c0.get("hold_ok") and c0.get("disturb_applied_ok") and tip_ok and no_cheat),
        "hold25_s": c0.get("hold25_s"),
        "disturbance": ep["disturbance"],
        "panel_at_disturb_start": c0.get("panel_at_disturb_start"),
        "panel_min_during_disturb": c0.get("panel_min_during_disturb"),
        "mp4": "previews/ainex_walk/iterate/M01.mp4",
    }
    l02 = {
        "tag": "M02",
        "pass": bool(
            c0.get("reject_panel15_ok") and c0.get("close_ok") and tip_ok and no_cheat
            and ep["tip_after_disturb_ok"]
        ),
        "time_to_panel15_after_disturb_s": c0.get("time_to_panel15_after_disturb_s"),
        "close_delta_deg": c0.get("close_delta_deg"),
        "tip_after_disturb_ok": ep["tip_after_disturb_ok"],
        "mp4": "previews/ainex_walk/iterate/M02.mp4",
    }
    l03 = {
        "tag": "M03",
        "pass": bool(ep["all_cycles_pass"] and tip_ok and no_cheat),
        "success_rate": ep["success_rate"],
        "n_pass": ep["n_pass"],
        "n_cycles": ep["n_cycles"],
        "cycles": cycles,
        "mp4": "previews/ainex_walk/iterate/M03.mp4",
    }
    rows = {"M00": l00, "M01": l01, "M02": l02, "M03": l03}
    overall = all(r["pass"] for r in rows.values())
    verdict = "GATE_M_PASS" if overall else "GATE_M_FAIL"

    table = {
        "verdict": verdict,
        "overall_pass": overall,
        "ai_can_lock": overall,
        "criteria": "docs/GATE_M_AI_CRITERIA.md",
        "plant": str(PLANT_F.relative_to(ROOT)),
        "companion_md5": plant_md5,
        "companion_lock_md5_expected": COMPANION_LOCK_MD5,
        "gate_e_plant_untouched": "mujoco/ainex_hiwonder/ainex_controls_m2_145.xml",
        "ckpt_sha16": sha,
        "stand_xy_hold": False,
        "free_joint_xy_pinned": False,
        "soft_pass_used": False,
        "disturbance": ep["disturbance"],
        "tip": ep["tip"],
        "tip_ok": tip_ok,
        "base_dx_m": ep["base_dx_m"],
        "base_dy_m": ep["base_dy_m"],
        "max_abs_panel_deg": ep["max_abs_panel_deg"],
        "panel_spring": ep["panel_spring"],
        "controller": ep["controller"],
        "rows": rows,
        "cycles": cycles,
        "mp4s": ep["mp4s"],
        "watch_primary": "M03.mp4",
        "open_angle_criterion": "free_edge_stripe_d_screen_x_ge_25px_at_plus30",
        "non_claims": [
            "full_door_open", "latch", "90_deg", "walk_through", "UK_handle",
            "hinge_range_bump", "Pi", "Orin",
        ],
        "scored_at_bst": "2026-09-28",
    }
    (ITER / "GATE_M_TABLE.json").write_text(json.dumps(table, indent=2, default=str))

    cyc_lines = "\n".join(
        f"| {b['cycle_index']} | {b['peak_panel_deg']:+.2f} | {b['hold25_s']:.2f} | "
        f"{b.get('panel_min_during_disturb')} | {b.get('time_to_panel15_after_disturb_s')} | "
        f"{b['close_delta_deg']:.2f} | {b['coupled_ok']} | **{st(b)}** |"
        for b in cycles
    )
    dist = ep["disturbance"]
    note = f'''# Gate M Controls — open-hold disturbance reject (sim)

**When:** Mon 28 Sep 2026 Europe/London (BST)  
**Role:** Founding Controls  
**Criteria:** `docs/GATE_M_AI_CRITERIA.md`  
**Plant:** `{PLANT_F.relative_to(ROOT)}` · md5 `{plant_md5}` (lock `{COMPANION_LOCK_MD5}`)  
**Gate E plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Ckpt:** sha16 **`{sha}`** · `head_tilt` · assist/freeze OFF · vision off · glfw  
**stand_xy_hold:** **false** · Soft-pass: **not used**

## Verdict: **{verdict}**

| Tag | Result |
|-----|--------|
| M00 open ≥25° + tip + coupling | **{st(l00)}** |
| M01 hold ≥1.0 s + declared disturbance | **{st(l01)}** |
| M02 reject (≥15° in 2 s) + close Δ ≥10° | **{st(l02)}** |
| M03 2/2 consecutive cycles | **{st(l03)}** ({ep["success_rate"]}) |

| Cycle | peak panel° | hold≥25° s | panel min@disturb | t→≥15° after disturb | close Δ° | coupled | Status |
|-------|-------------|------------|-------------------|----------------------|----------|---------|--------|
{cyc_lines}

**tip**={ep["tip"]:.2f}s · tip_after_disturb_ok={ep["tip_after_disturb_ok"]} · Δx={ep["base_dx_m"]*1000:+.1f} mm · Δy={ep["base_dy_m"]*1000:+.1f} mm · max panel={ep["max_abs_panel_deg"]:.2f}°

## Disturbance (declared)

| Item | Value |
|------|--------|
| Type | **{dist["type"]}** (Controls choice: panel push, not free-base nudge) |
| Body | `{dist["body"]}` |
| Force | {dist["force_N_world"]} N world (+X closing-sense) |
| Duration | **{dist["duration_s"]} s** (≤0.5 s) |
| Applied total | {dist["applied_s_total"]:.2f} s across cycles |
| Honesty | Declared only; custom `_substep_m` re-applies after env xfrc clear; **no** undeclared xfrc |

## Controller
push {ep["controller"]["push_s"]}s / hold {ep["controller"]["hold_s"]}s / disturb {ep["controller"]["disturb_s"]}s / reject {ep["controller"]["reject_s"]}s / close {ep["controller"]["close_s"]}s / recover {ep["controller"]["recover_s"]}s; cam_dist={ep["cam_dist_cmd"]}. No XY pin. No plant/spring/range invent.

## Video
- **watch_primary:** `M03.mp4` (kit ‖ oblique; HUD panel° + XY; **DISTURBANCE ON** banner during push)
- Also: `M03_kit.mp4`, `M03_panel.mp4`, `M03_world.mp4`, `M03_dual_s9.mp4`
- Open-angle: inherit K (green stripe Δ screen-X ≥~25 px @ +30°)

## Honesty / non-claims
No panel actuator / scripted panel qpos. No M145 edit. No hinge-range bump. Not full door-open / latch / 90° / walk-through / UK / Pi. Soft-pass forbidden.

**AI can lock:** {"YES (Controls pack claims PASS — AI owns lock)" if overall else "NO — pack FAIL; iterate controller (no plant invent)"}
'''
    (ITER / "GATE_M_CONTROLS_NOTE.md").write_text(note)
    print(f"=== {verdict} === ai_can_lock={overall}", flush=True)
    for tag, r in rows.items():
        print(f"  {tag}: {st(r)}", flush=True)
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
