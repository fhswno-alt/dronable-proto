#!/usr/bin/env python3
"""Gate L: leave / re-grasp while holding panel on ±30° companion (no range bump).

Criteria: docs/GATE_L_AI_CRITERIA.md
- L00 open ≥25° + tip≥8 + coupling (K retain)
- L01 leave contact ≥0.5 s with panel ≥20°; no tip
- L02 re-grasp + close Δ ≥10°; tip≥8
- L03 2/2 consecutive leave→re-grasp→close cycles; continuous MP4

NO Stand XY pin. Soft-pass forbidden. M145 / spring / range untouched.
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
from score_gate_h import _hinge_id, _pin_arms_head, _substep, OPEN_CMD, CLOSE_CMD
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


def _md5(p: Path) -> str:
    return hashlib.md5(p.read_bytes()).hexdigest()


def _render_l(
    env: GateFEnv,
    qpos_log: list,
    xy_log: list,
    bout_log: list,
    t_log: list,
    phase_log: list,
    stride: int = 5,
    fps: int = 10,
):
    """Stream-encode L03 dual (kit‖oblique) + kit + panel + world."""
    import imageio.v2 as imageio

    ITER.mkdir(parents=True, exist_ok=True)
    r = mj.Renderer(env.model, height=480, width=640)
    wcam = _world_cam(env.model)
    pcam = _panel_cam(env.model)
    jid_p = _panel_hinge_id(env.model)
    jid_l = _hinge_id(env.model)
    x0, y0 = xy_log[0]
    paths = {
        "kit": ITER / "L03_kit.mp4",
        "panel": ITER / "L03_panel.mp4",
        "world": ITER / "L03_world.mp4",
        "dual": ITER / "L03.mp4",  # watch_primary
        "dual_s9": ITER / "L03_dual_s9.mp4",
    }
    n = len(qpos_log)
    print(f"[L] stream-render n={n} stride={stride} → L03*.mp4", flush=True)
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
            title_kit = f"kit · L gate · {ph}"
            title_panel = f"oblique free-edge · {ph} · judge stripe ΔX"
            r.update_scene(env.data, camera=env.cid)
            kit = _hud(
                r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg, title=title_kit
            )
            r.update_scene(env.data, camera=wcam)
            world = _hud(
                _mark_base_edge(r.render().copy()),
                dx, dy, x, y, tt, bb, panel_deg, lever_deg,
                title="§9 feet world · base EDGE",
            )
            r.update_scene(env.data, camera=pcam)
            panel = _hud(
                r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg, title=title_panel
            )
            dual = np.concatenate([kit, panel], axis=1)
            dual_s9 = np.concatenate([kit, world], axis=1)
            writers["kit"].append_data(kit)
            writers["world"].append_data(world)
            writers["panel"].append_data(panel)
            writers["dual"].append_data(dual)
            writers["dual_s9"].append_data(dual_s9)
            if i % 500 == 0:
                print(f"[L] render frame {i}/{n} phase={ph} panel={panel_deg:+.1f}°", flush=True)
    finally:
        for w in writers.values():
            w.close()
        r.close()
    # Also copy watch-primary aliases
    for tag in ("L00", "L01", "L02"):
        # symlink/copy dual as per-tag convenience (same continuous run)
        src = paths["dual"]
        dst = ITER / f"{tag}.mp4"
        if dst.exists() or dst.is_symlink():
            dst.unlink()
        try:
            dst.symlink_to(src.name)
        except OSError:
            import shutil
            shutil.copy2(src, dst)
    print("[L] wrote L03.mp4 (+ L00–L02 → L03) L03_kit/panel/world/dual_s9", flush=True)
    return {k: str(p.relative_to(ROOT)) for k, p in paths.items()}


def score_leave_regrasp(n_cycles: int = 2) -> dict:
    """Settle + N×(push, hold, leave, regrasp, close). Free-joint x,y UNPINNED."""
    arm_q, _cam_seed, tilt = _arm_seed()
    cam_dist = 0.11
    t_settle_open, t_settle_close = 0.8, 1.2
    push_s, hold_s = 10.0, 1.5
    leave_s, regrasp_s, close_s = 1.2, 3.5, 6.5
    recover_s = 2.0  # inter-cycle reapproach / settle near lever after close
    push_to, rev_cmd_end = 29.8, -4.0
    soft_close = True
    close_pull_scale = 0.22
    g_push, g_close, g_leave, g_regrasp = 1.35, 1.15, 1.05, 1.25
    # leave retreat offsets (hand away from lever while panel held by spring)
    leave_off = np.array([-0.08, 0.05, 0.03])
    cycle_s = push_s + hold_s + leave_s + regrasp_s + close_s + recover_s
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
            "hold_open_s": 0.0,
            "_hold_run": 0.0,
            "leave_no_contact_s": 0.0,
            "leave_no_contact_panel20_s": 0.0,
            "_leave_nc_run": 0.0,
            "_leave_nc20_run": 0.0,
            "min_panel_during_leave_ok": None,
            "regrasp_contact_s": 0.0,
            "min_abs_during_close": None,
            "close_delta_deg": 0.0,
            "contact_rising_s": 0.0,
            "contact_s": 0.0,
            "panel_at_leave_start": None,
            "coupled_ok": False,
            "open25_ok": False,
            "hold_ok": False,
            "leave_ok": False,
            "regrasp_ok": False,
            "close_ok": False,
            "pass": False,
            "_prev_abs": 0.0,
            "_seen_leave": False,
            "_seen_regrasp": False,
        })

    qpos_log, xy_log, bout_log, t_log, phase_log = [], [], [], [], []
    tip_run = tip = 0.0
    global_max_panel = global_max_lever = 0.0
    global_rising = global_contact = 0.0
    prev_abs = 0.0
    t0 = t_settle_open + t_settle_close

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        if t < t_settle_open:
            gcmd, phase, cmd, a_rev = OPEN_CMD, "settle_open", 0.0, 0.0
            bi = -1
            local = 0.0
        elif t < t0:
            a = (t - t_settle_open) / t_settle_close
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            phase, cmd, a_rev, bi = "settle_close", 0.0, 0.0, -1
            local = 0.0
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
            elif local < push_s + hold_s + leave_s:
                # leave: open gripper, retreat hand
                a_leave = (local - push_s - hold_s) / leave_s
                gcmd = CLOSE_CMD * (1.0 - a_leave) + OPEN_CMD * a_leave
                phase, cmd, a_rev = "leave", push_to, a_leave
            elif local < push_s + hold_s + leave_s + regrasp_s:
                a_rg = (local - push_s - hold_s - leave_s) / regrasp_s
                gcmd = OPEN_CMD * (1.0 - a_rg) + CLOSE_CMD * a_rg
                # track toward current panel angle (may have drifted)
                ang_now = abs(_angle_deg(m, d, jid_p))
                cmd = max(ang_now, 18.0)  # aim near present open
                phase, a_rev = "regrasp", a_rg
            elif local < push_s + hold_s + leave_s + regrasp_s + close_s:
                a_rev = (local - push_s - hold_s - leave_s - regrasp_s) / close_s
                if soft_close:
                    gcmd = CLOSE_CMD * (1.0 - 0.55 * a_rev) + OPEN_CMD * (0.55 * a_rev)
                else:
                    gcmd = CLOSE_CMD * (1.0 - 0.85 * a_rev) + OPEN_CMD * (0.85 * a_rev)
                phase = "close"
                ang_now = abs(_angle_deg(m, d, jid_p))
                cmd = ang_now + (rev_cmd_end - ang_now) * a_rev
            else:
                # recover: soft approach near closed lever for next cycle workspace
                a_rec = (local - push_s - hold_s - leave_s - regrasp_s - close_s) / recover_s
                gcmd = OPEN_CMD * (1.0 - a_rec) + CLOSE_CMD * a_rec
                phase, cmd, a_rev = "recover", 0.0, a_rec

        mj.mj_forward(m, d)
        if phase in ("push", "hold", "leave", "regrasp", "close", "recover"):
            if phase == "leave":
                # retreat from open arc
                target = arc(cmd) + leave_off * min(1.0, 0.3 + 0.7 * a_rev)
                g = g_leave
            elif phase == "regrasp":
                target = arc(cmd)
                # blend from leave offset back to lever
                target = target + leave_off * (1.0 - a_rev)
                g = g_regrasp
            elif phase == "close":
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
                g = g_regrasp
            else:
                target = arc(cmd)
                g = g_push
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = target - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            lam = 1e-3
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
                for jid, ddq, jn in zip(jids, dq, reach):
                    lo, hi = m.jnt_range[jid]
                    nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + g * ddq, lo, hi))
                    d.qpos[m.jnt_qposadr[jid]] = nv
                    arm_q[jn] = nv
            except np.linalg.LinAlgError:
                pass

        _pin_arms_head(env, arm_q, gcmd)
        _substep(env, arm_q)
        qpos_log.append(d.qpos.copy())
        xy_log.append((float(d.qpos[env.q_free]), float(d.qpos[env.q_free + 1])))
        bout_log.append(bi)
        t_log.append(t)
        phase_log.append(phase)

        Rmat = d.xmat[env.bid_body].reshape(3, 3)
        up = float(Rmat[2, 2])
        if up > 0.5:
            tip_run += 1 / CTRL_HZ
            tip = max(tip, tip_run)
        else:
            tip_run = 0.0

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
                    b["hold_open_s"] = max(b["hold_open_s"], b["_hold_run"])
                else:
                    b["_hold_run"] = 0.0
                if contacting:
                    b["contact_s"] += 1 / CTRL_HZ
                    if abs(ang_p) > b["_prev_abs"] + 1e-4:
                        b["contact_rising_s"] += 1 / CTRL_HZ
            elif phase == "leave":
                b["_seen_leave"] = True
                if b["panel_at_leave_start"] is None:
                    b["panel_at_leave_start"] = abs(ang_p)
                if not contacting:
                    b["_leave_nc_run"] += 1 / CTRL_HZ
                    b["leave_no_contact_s"] = max(b["leave_no_contact_s"], b["_leave_nc_run"])
                    if abs(ang_p) >= 20.0 - 1e-6:
                        b["_leave_nc20_run"] += 1 / CTRL_HZ
                        b["leave_no_contact_panel20_s"] = max(
                            b["leave_no_contact_panel20_s"], b["_leave_nc20_run"]
                        )
                        if b["min_panel_during_leave_ok"] is None:
                            b["min_panel_during_leave_ok"] = abs(ang_p)
                        else:
                            b["min_panel_during_leave_ok"] = min(
                                b["min_panel_during_leave_ok"], abs(ang_p)
                            )
                    else:
                        b["_leave_nc20_run"] = 0.0
                else:
                    b["_leave_nc_run"] = 0.0
                    b["_leave_nc20_run"] = 0.0
            elif phase == "regrasp":
                b["_seen_regrasp"] = True
                if contacting:
                    b["regrasp_contact_s"] += 1 / CTRL_HZ
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

    tip_ok = tip >= 8.0
    for i, b in enumerate(cycles):
        if b["min_abs_during_close"] is None:
            b["min_abs_during_close"] = abs(b["peak_panel_deg"])
        b["close_delta_deg"] = abs(b["peak_panel_deg"]) - float(b["min_abs_during_close"])
        b["open25_ok"] = abs(b["peak_panel_deg"]) >= 25.0 - 1e-6
        b["hold_ok"] = b["hold_open_s"] >= 0.5 - 1e-9
        b["leave_ok"] = b["leave_no_contact_panel20_s"] >= 0.5 - 1e-9
        b["regrasp_ok"] = b["regrasp_contact_s"] >= 0.3  # re-establish grasp before close
        b["close_ok"] = b["close_delta_deg"] >= 10.0 - 1e-6
        b["coupled_ok"] = b["contact_rising_s"] > 0.1 and b["contact_s"] > 0.5
        b["tip_ok"] = tip_ok
        b["within_30"] = abs(b["peak_panel_deg"]) <= 30.0 + 0.05
        b["cycle_pass"] = bool(
            b["open25_ok"] and b["hold_ok"] and b["leave_ok"] and b["regrasp_ok"]
            and b["close_ok"] and b["coupled_ok"] and b["tip_ok"] and b["within_30"]
        )
        b["pass"] = b["cycle_pass"]
        for k in list(b.keys()):
            if k.startswith("_"):
                del b[k]
        b["cycle_index"] = i

    n_pass = sum(1 for b in cycles if b["pass"])
    dx = float(d.qpos[env.q_free]) - x0
    dy = float(d.qpos[env.q_free + 1]) - y0
    print(
        f"[L] physics done n_pass={n_pass}/{n_cycles} tip={tip:.2f}s "
        f"Δx={dx*1000:+.1f}mm Δy={dy*1000:+.1f}mm",
        flush=True,
    )
    for b in cycles:
        print(
            f"  cyc{b['cycle_index']}: peak={b['peak_panel_deg']:+.2f} hold={b['hold_open_s']:.2f} "
            f"leave20={b['leave_no_contact_panel20_s']:.2f} regrasp={b['regrasp_contact_s']:.2f} "
            f"Δclose={b['close_delta_deg']:.2f} pass={b['pass']}",
            flush=True,
        )

    mp4s = _render_l(env, qpos_log, xy_log, bout_log, t_log, phase_log)
    del qpos_log[:]

    return {
        "tip": tip,
        "tip_ok": tip_ok,
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
        "coupling_path": "hand contact → door_lever → door_lever_link child of door_panel_link → door_panel_hinge",
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "fall": tip < 1.0,
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": tilt,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "companion_md5": _md5(PLANT_F),
        "companion_lock_md5": COMPANION_LOCK_MD5,
        "hw_spring_ref": "docs/GATE_J_HARDWARE_PANEL_SPRING.md",
        "within_30deg_range": global_max_panel <= 30.0 + 0.05,
        "cam_dist_cmd": cam_dist,
        "controller": {
            "push_s": push_s, "hold_s": hold_s, "leave_s": leave_s,
            "regrasp_s": regrasp_s, "close_s": close_s, "recover_s": recover_s,
            "push_to": push_to, "leave_off": leave_off.tolist(),
            "soft_close": soft_close, "xy_pin": False,
        },
        "mp4s": mp4s,
        "episode_s": episode_s,
    }


def main():
    ITER.mkdir(parents=True, exist_ok=True)
    sha = _sha16(CKPT)
    plant_md5 = _md5(PLANT_F)
    print(
        f"[L] HONEST leave/re-grasp plant={PLANT_F.name} md5={plant_md5} "
        f"ckpt={sha} spring damp={_panel_spring(mj.MjModel.from_xml_path(str(PLANT_F)))['damping']}",
        flush=True,
    )
    if sha != CKPT_SHA16:
        print(f"[L] WARN ckpt sha {sha} != frozen {CKPT_SHA16}", flush=True)
    if plant_md5 != COMPANION_LOCK_MD5:
        print(
            f"[L] WARN companion md5 {plant_md5} != Gate K lock {COMPANION_LOCK_MD5}",
            flush=True,
        )

    ep = score_leave_regrasp(n_cycles=2)
    tip_ok = ep["tip_ok"]
    no_cheat = (
        not ep["panel_qpos_scripted"]
        and not ep["panel_actuator"]
        and not ep["stand_xy_hold"]
        and not ep["free_joint_xy_pinned"]
        and not ep["assist"]
        and not ep["freeze"]
        and not ep["vision_in_walk_obs"]
        and ep["within_30deg_range"]
    )
    cycles = ep["cycles"]
    c0 = cycles[0] if cycles else {}

    # L00: open ≥25 + tip + coupling (any first-cycle open)
    l00 = {
        "tag": "L00",
        "pass": bool(
            c0.get("open25_ok") and tip_ok and c0.get("coupled_ok") and no_cheat
        ),
        "peak_panel_deg": c0.get("peak_panel_deg"),
        "tip": ep["tip"],
        "coupled_ok": c0.get("coupled_ok"),
        "mp4": "previews/ainex_walk/iterate/L00.mp4",
    }
    # L01: leave ≥0.5s no-contact with panel≥20
    l01 = {
        "tag": "L01",
        "pass": bool(c0.get("leave_ok") and tip_ok and no_cheat),
        "leave_no_contact_panel20_s": c0.get("leave_no_contact_panel20_s"),
        "leave_no_contact_s": c0.get("leave_no_contact_s"),
        "panel_at_leave_start": c0.get("panel_at_leave_start"),
        "min_panel_during_leave_ok": c0.get("min_panel_during_leave_ok"),
        "tip": ep["tip"],
        "mp4": "previews/ainex_walk/iterate/L01.mp4",
    }
    # L02: re-grasp + close Δ≥10
    l02 = {
        "tag": "L02",
        "pass": bool(
            c0.get("regrasp_ok") and c0.get("close_ok") and tip_ok and no_cheat
        ),
        "regrasp_contact_s": c0.get("regrasp_contact_s"),
        "close_delta_deg": c0.get("close_delta_deg"),
        "tip": ep["tip"],
        "mp4": "previews/ainex_walk/iterate/L02.mp4",
    }
    # L03: 2/2 cycles
    l03 = {
        "tag": "L03",
        "pass": bool(ep["all_cycles_pass"] and tip_ok and no_cheat),
        "success_rate": ep["success_rate"],
        "n_pass": ep["n_pass"],
        "n_cycles": ep["n_cycles"],
        "cycles": cycles,
        "mp4": "previews/ainex_walk/iterate/L03.mp4",
    }

    rows = {"L00": l00, "L01": l01, "L02": l02, "L03": l03}
    overall = all(r["pass"] for r in rows.values())
    verdict = "GATE_L_PASS" if overall else "GATE_L_FAIL"

    table = {
        "verdict": verdict,
        "overall_pass": overall,
        "ai_can_lock": overall,  # Controls claim only — AI owns lock
        "criteria": "docs/GATE_L_AI_CRITERIA.md",
        "plant": str(PLANT_F.relative_to(ROOT)),
        "companion_md5": plant_md5,
        "companion_lock_md5_expected": COMPANION_LOCK_MD5,
        "gate_e_plant_untouched": "mujoco/ainex_hiwonder/ainex_controls_m2_145.xml",
        "ckpt_sha16": sha,
        "stand_xy_hold": False,
        "free_joint_xy_pinned": False,
        "soft_pass_used": False,
        "panel_actuator": False,
        "panel_qpos_scripted": False,
        "within_30deg_range": ep["within_30deg_range"],
        "tip": ep["tip"],
        "tip_ok": tip_ok,
        "base_dx_m": ep["base_dx_m"],
        "base_dy_m": ep["base_dy_m"],
        "max_abs_panel_deg": ep["max_abs_panel_deg"],
        "max_abs_lever_deg": ep["max_abs_lever_deg"],
        "panel_spring": ep["panel_spring"],
        "controller": ep["controller"],
        "rows": rows,
        "cycles": cycles,
        "mp4s": ep["mp4s"],
        "watch_primary": "L03.mp4",
        "open_angle_criterion": "free_edge_stripe_d_screen_x_ge_25px_at_plus30",
        "non_claims": [
            "full_door_open", "latch", "90_deg", "walk_through", "UK_handle",
            "hinge_range_bump", "Pi", "Orin",
        ],
        "scored_at_bst": "2026-09-28",
    }
    (ITER / "GATE_L_TABLE.json").write_text(json.dumps(table, indent=2, default=str))

    def st(r):
        return "PASS" if r["pass"] else "FAIL"

    cyc_lines = "\n".join(
        f"| {b['cycle_index']} | {b['peak_panel_deg']:+.2f} | {b['hold_open_s']:.2f} | "
        f"{b['leave_no_contact_panel20_s']:.2f} | {b['regrasp_contact_s']:.2f} | "
        f"{b['close_delta_deg']:.2f} | {b['coupled_ok']} | **{st(b)}** |"
        for b in cycles
    )
    note = f'''# Gate L Controls — leave / re-grasp (sim)

**When:** Mon 28 Sep 2026 Europe/London (BST)  
**Role:** Founding Controls  
**Criteria:** `docs/GATE_L_AI_CRITERIA.md`  
**Plant:** `{PLANT_F.relative_to(ROOT)}` · md5 `{plant_md5}` (Gate K lock `{COMPANION_LOCK_MD5}`)  
**Gate E plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Ckpt:** sha16 **`{sha}`** · `head_tilt` · assist/freeze OFF · vision off · glfw  
**stand_xy_hold:** **false** · Soft-pass: **not used**

## Verdict: **{verdict}**

| Tag | Result |
|-----|--------|
| L00 open ≥25° + tip + coupling | **{st(l00)}** |
| L01 leave ≥0.5 s (panel ≥20°) | **{st(l01)}** |
| L02 re-grasp + close Δ ≥10° | **{st(l02)}** |
| L03 2/2 consecutive cycles | **{st(l03)}** ({ep["success_rate"]}) |

| Cycle | peak panel° | hold≥25° s | leave nc@≥20° s | regrasp contact s | close Δ° | coupled | Status |
|-------|-------------|------------|-----------------|-------------------|----------|---------|--------|
{cyc_lines}

**tip**={ep["tip"]:.2f}s · Δx={ep["base_dx_m"]*1000:+.1f} mm · Δy={ep["base_dy_m"]*1000:+.1f} mm · max panel={ep["max_abs_panel_deg"]:.2f}°

## Controller
push {ep["controller"]["push_s"]}s / hold {ep["controller"]["hold_s"]}s / leave {ep["controller"]["leave_s"]}s / regrasp {ep["controller"]["regrasp_s"]}s / close {ep["controller"]["close_s"]}s; cam_dist={ep["cam_dist_cmd"]}; leave_off={ep["controller"]["leave_off"]}. No XY pin. No plant/spring/range invent.

## Video
- **watch_primary:** `L03.mp4` (kit ‖ oblique free-edge; HUD panel° + XY)
- Also: `L03_kit.mp4`, `L03_panel.mp4`, `L03_world.mp4`, `L03_dual_s9.mp4`
- Open-angle: same K criterion (green stripe Δ screen-X ≥~25 px @ +30°) — judge on oblique pane.

## Honesty / non-claims
No panel actuator / scripted panel qpos. No M145 edit. No hinge-range bump. Not full door-open / latch / 90° / walk-through / UK / Pi. Soft-pass forbidden.

**AI can lock:** {"YES (Controls pack claims PASS — AI owns lock)" if overall else "NO — pack FAIL; iterate controller (no plant invent)"}
'''
    (ITER / "GATE_L_CONTROLS_NOTE.md").write_text(note)
    print(f"=== {verdict} === ai_can_lock={overall}", flush=True)
    for tag, r in rows.items():
        print(f"  {tag}: {st(r)}", flush=True)
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
