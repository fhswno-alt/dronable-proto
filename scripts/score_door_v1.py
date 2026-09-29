#!/usr/bin/env python3
"""Gate Door v1: Prefer FAIL panel push/pull prove (hospital-style).

Criteria: docs/GATE_DOOR_V1_AI_CRITERIA.md
Plant: installed live gate_f_push.md5 adb24309…; gate_f.xml is lever-era archive only.
Soft-pass: OFF forever. No lever contact scored. No panel actuator / scripted
panel qpos during bout (inter-bout rest reset OK). No latch / 90° / walk-through.

Bars:
  1 closed start |door_panel_hinge| ≤ 2°
  2 approach/reach to door_panel_push_site (document Δ); tip≥8 if stepping
  3 hand × door_panel_push_face contact ≥ 0.3 s during open
  4 open |hinge| ≥ 25° within ±30°; tip≥8
  5 hold ≥ 1.0 s @ panel ≥ 20°; tip≥8
  6 multi-bout 2/2 consecutive
  7 plant honesty (md5 lock; lever contype 0; push_face contype 2; no panel act)
  8 continuous MP4 both bouts; HUD hinge + closed/contact/open marks
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Any
from zoneinfo import ZoneInfo

import numpy as np
import mujoco as mj

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
os.environ.setdefault("MUJOCO_GL", "glfw")

import score_gate_f as sgf  # noqa: E402
from score_gate_f import (  # noqa: E402
    CTRL_HZ,
    CKPT,
    ITER,
    GateFEnv,
    body_x_for_cam_dist,
)
from score_gate_g import ARM_L, ARM_R  # noqa: E402
from score_gate_h import OPEN_CMD, CLOSE_CMD, _pin_arms_head, _substep  # noqa: E402
from score_gate_i import _panel_hinge_id, _angle_deg  # noqa: E402
from score_gate_k import _hud, _panel_cam  # noqa: E402

PUSH_MD5 = "adb24309b489d56615c194e92676d040"
ARCHIVE_LEVER_MD5 = "59cc408eda07037a58f92ad27da045d6"
WALK_MD5 = "fc94709c84f5598d4474ecfc4bb41fdc"
CKPT_SHA16 = "9ffaa1a21b607bf6"
CLOSED_EPS_DEG = 2.0
OPEN_DEG = 25.0
HOLD_DEG = 20.0
HOLD_S = 1.0
CONTACT_S = 0.3
TZ = ZoneInfo("Europe/London")

PLANT_ARCHIVE = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145_gate_f.xml"  # lever-era ARCHIVE
PLANT_LIVE = PLANT_ARCHIVE  # alias: never score Door v1 against archive
PLANT_PUSH = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145_gate_f_push.xml"  # Door v1 live
PLANT_WALK = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
PROGRESS = ITER / "GATE_DOOR_V1_PROGRESS.md"
SCORE_DOC = ROOT / "docs" / "GATE_DOOR_V1_AI_SCORE.md"
DONE_JSON = ITER / "GATE_DOOR_V1_DONE.json"
SCORE_JSON = ITER / "GATE_DOOR_V1_SCORE.json"


def _md5(p: Path) -> str:
    return hashlib.md5(p.read_bytes()).hexdigest()


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _now() -> str:
    return datetime.now(TZ).strftime("%a %d %b %Y %H:%M %Z")


def resolve_plant() -> tuple[Path, str, str]:
    """Door v1 plant-of-record = gate_f_push (Hardware install receipt).

    Score ONLY against gate_f_push.xml. Lever-era gate_f.xml is ARCHIVE — never
    load it for Door v1. Walk M145 untouched.
    """
    push = _md5(PLANT_PUSH)
    assert push == PUSH_MD5, f"push plant md5 drift: {push}"
    walk = _md5(PLANT_WALK)
    assert walk == WALK_MD5, f"walk plant touched: {walk}"
    archive = _md5(PLANT_LIVE)  # gate_f.xml = lever-era ARCHIVE (not Door live)
    assert archive == ARCHIVE_LEVER_MD5, (
        f"lever-era archive gate_f.md5 drift (must stay 59cc…): {archive}"
    )
    return (
        PLANT_PUSH,
        push,
        "INSTALLED live=gate_f_push md5 adb24309…; archive gate_f 59cc… KEPT; walk fc94709c… KEPT "
        "(docs/GATE_DOOR_V1_HARDWARE_INSTALL.md)",
    )


def _assert_plant_honesty(model: mj.MjModel) -> dict:
    gid_lev = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "door_lever")
    gid_pf = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "door_panel_push_face")
    assert gid_lev >= 0 and gid_pf >= 0
    lev_ct = int(model.geom_contype[gid_lev])
    pf_ct = int(model.geom_contype[gid_pf])
    assert lev_ct == 0, f"door_lever contype must be 0, got {lev_ct}"
    assert pf_ct == 2, f"door_panel_push_face contype must be 2, got {pf_ct}"
    for i in range(model.nu):
        n = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "panel" in n.lower():
            raise RuntimeError(f"panel actuator present (cheat): {n}")
    return {
        "door_lever_contype": lev_ct,
        "door_panel_push_face_contype": pf_ct,
        "panel_actuator": False,
    }


def _hand_push_face_contact(model: mj.MjModel, data: mj.MjData) -> bool:
    gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "door_panel_push_face")
    hands = {
        mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "l_hand_contact"),
        mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_hand_contact"),
    }
    hands.discard(-1)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 == gid and g2 in hands) or (g2 == gid and g1 in hands):
            return True
    return False


def _hand_lever_contact(model: mj.MjModel, data: mj.MjData) -> bool:
    """Detect (but do NOT score as open) any residual lever contact."""
    gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "door_lever")
    if gid < 0 or int(model.geom_contype[gid]) == 0:
        return False
    hands = {
        mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "l_hand_contact"),
        mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_hand_contact"),
    }
    hands.discard(-1)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 == gid and g2 in hands) or (g2 == gid and g1 in hands):
            return True
    return False


def _banner(frame: np.ndarray, y0: int, fill: tuple, text: str) -> np.ndarray:
    from PIL import Image, ImageDraw, ImageFont

    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 18
        )
    except Exception:
        font = ImageFont.load_default()
    draw.rectangle([6, y0, 420, y0 + 28], fill=fill)
    draw.text((10, y0 + 4), text, fill=(255, 255, 0), font=font)
    return np.asarray(img)


def _door_hud(
    frame: np.ndarray,
    *,
    t: float,
    bout: int,
    panel_deg: float,
    phase: str,
    mark: str,
    reach_d: float,
    contact_on: bool,
) -> np.ndarray:
    title = f"Door v1 · {phase} · {mark}"
    out = _hud(
        frame, 0.0, 0.0, 0.0, 0.0, t, bout,
        panel_deg=panel_deg, lever_deg=None, title=title,
    )
    # mark strip
    if mark == "CLOSED":
        out = _banner(out, 130, (0, 90, 40), "CLOSED |hinge|≤2°")
    elif mark == "CONTACT":
        out = _banner(out, 130, (140, 80, 0), "CONTACT hand×push_face")
    elif mark == "OPEN":
        out = _banner(out, 130, (0, 110, 180), "OPEN |hinge|≥25°")
    elif mark == "HOLD":
        out = _banner(out, 130, (0, 140, 200), "HOLD ≥20°")
    # reach line
    from PIL import Image, ImageDraw, ImageFont

    img = Image.fromarray(out)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 13
        )
    except Exception:
        font = ImageFont.load_default()
    draw.rectangle([6, 162, 420, 180], fill=(0, 0, 0))
    draw.text(
        (10, 162),
        f"Δpush_site={reach_d * 1000:.0f}mm  contact={'Y' if contact_on else 'n'}",
        fill=(200, 200, 200),
        font=font,
    )
    return np.asarray(img)


def _render_door(
    env: GateFEnv,
    qpos_log: list,
    meta_log: list[dict],
    out_path: Path,
    *,
    stride: int = 5,
    fps: int = 10,
) -> Path:
    import imageio.v2 as imageio

    ITER.mkdir(parents=True, exist_ok=True)
    r = mj.Renderer(env.model, height=480, width=640)
    pcam = _panel_cam(env.model)
    jid_p = _panel_hinge_id(env.model)
    n = len(qpos_log)
    dual = out_path
    kit_p = out_path.with_name(out_path.stem + "_kit.mp4")
    panel_p = out_path.with_name(out_path.stem + "_panel.mp4")
    print(f"[DOOR] stream-render n={n} stride={stride} → {out_path.name}", flush=True)
    writers = {
        "dual": imageio.get_writer(str(dual), fps=fps, codec="libx264", quality=8, macro_block_size=1),
        "kit": imageio.get_writer(str(kit_p), fps=fps, codec="libx264", quality=8, macro_block_size=1),
        "panel": imageio.get_writer(str(panel_p), fps=fps, codec="libx264", quality=8, macro_block_size=1),
    }
    try:
        for i in range(0, n, stride):
            env.data.qpos[:] = qpos_log[i]
            env.data.qvel[:] = 0
            mj.mj_forward(env.model, env.data)
            md = meta_log[i]
            panel_deg = _angle_deg(env.model, env.data, jid_p)
            r.update_scene(env.data, camera=env.cid)
            kit = _door_hud(
                r.render().copy(),
                t=md["t"], bout=md["bout"], panel_deg=panel_deg,
                phase=md["phase"], mark=md["mark"], reach_d=md["reach"],
                contact_on=md["contact"],
            )
            r.update_scene(env.data, camera=pcam)
            pan = _door_hud(
                r.render().copy(),
                t=md["t"], bout=md["bout"], panel_deg=panel_deg,
                phase=md["phase"], mark=md["mark"], reach_d=md["reach"],
                contact_on=md["contact"],
            )
            dual_fr = np.concatenate([kit, pan], axis=1)
            writers["kit"].append_data(kit)
            writers["panel"].append_data(pan)
            writers["dual"].append_data(dual_fr)
    finally:
        for w in writers.values():
            w.close()
        r.close()
    return dual


def score_episode(
    *,
    tag: str,
    plant: Path,
    # Prefer FAIL tunable knobs (timing/gain/IK — NO plant invent)
    cam_dist: float = 0.14,
    push_to: float = -28.0,
    gain: float = 1.4,
    pen: float = 0.020,
    lead_deg: float = 4.0,
    rest_xy: tuple[float, float] = (-0.014, 0.045),
    push_z: float = 0.18,
    push_s: float = 8.0,
    hold_s: float = 2.5,
    rev_s: float = 5.5,
    recover_s: float = 3.5,
    settle_open: float = 0.5,
    settle_close: float = 0.7,
    tilt_deg: float = -19.0,
    n_cycles: int = 2,
) -> dict[str, Any]:
    """Continuous 2-bout closed→push→open≥25→hold on push face."""
    sgf.PLANT_F = plant
    hinge_xy = np.array([0.45, -0.05])
    rest = np.array(list(rest_xy))
    z = float(push_z)

    def arc(deg: float) -> np.ndarray:
        th = np.radians(deg)
        c, s = np.cos(th), np.sin(th)
        xy = hinge_xy + np.array([[c, -s], [s, c]]) @ rest
        return np.array([xy[0], xy[1], z])

    cycle_s = push_s + hold_s + rev_s + recover_s
    episode_s = settle_open + settle_close + n_cycles * cycle_s
    env = GateFEnv(
        gait_t=0.75,
        episode_s=episode_s,
        stand_hold=episode_s,
        ramp_t=0.0,
        head_tilt_deg=tilt_deg,
        walk=False,
        body_x0=body_x_for_cam_dist(cam_dist, lever_x=0.424),
    )
    env.reset()
    m, d = env.model, env.data
    honesty = _assert_plant_honesty(m)
    jid_p = _panel_hinge_id(m)
    assert jid_p >= 0
    # rest reset only — never script panel qpos during bout
    d.qpos[m.jnt_qposadr[jid_p]] = 0.0
    d.qvel[m.jnt_dofadr[jid_p]] = 0.0
    mj.mj_forward(m, d)

    sid_hand = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "l_hand_site")
    sid_push = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "door_panel_push_site")
    jids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, j) for j in ARM_L]
    arm_q: dict[str, float] = {}
    warm = [-1.5, 0.6, -1.0, 0.2]
    for jid, val, jn in zip(jids, warm, ARM_L):
        d.qpos[m.jnt_qposadr[jid]] = val
        arm_q[jn] = val
    for jn in ARM_R:
        jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
        arm_q[jn] = float(d.qpos[m.jnt_qposadr[jid]])

    # IK warm to approach pose (not penetrating)
    for _ in range(120):
        mj.mj_forward(m, d)
        tgt = arc(0.0) + np.array([-0.025, 0.0, 0.0])
        err = tgt - d.site_xpos[sid_hand]
        jacp = np.zeros((3, m.nv))
        mj.mj_jacSite(m, d, jacp, None, sid_hand)
        cols = [m.jnt_dofadr[j] for j in jids]
        J = jacp[:, cols]
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), err)
            for jid, dd, jn in zip(jids, dq, ARM_L):
                lo, hi = m.jnt_range[jid]
                nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + 0.55 * dd, lo, hi))
                d.qpos[m.jnt_qposadr[jid]] = nv
                arm_q[jn] = nv
        except np.linalg.LinAlgError:
            pass

    bouts: list[dict] = [
        {
            "bout": i,
            "closed_start_deg": None,
            "max_abs_panel_deg": 0.0,
            "peak_panel_deg": 0.0,
            "contact_s": 0.0,
            "contact_while_rising_s": 0.0,
            "hold20_s": 0.0,
            "_hold_run": 0.0,
            "_prev_abs": 0.0,
            "lever_contact_s": 0.0,
            "min_reach_m": 9.0,
            "open25_ok": False,
            "hold_ok": False,
            "contact_ok": False,
            "closed_ok": False,
            "within_30": True,
            "pass": False,
        }
        for i in range(n_cycles)
    ]

    qpos_log: list[np.ndarray] = []
    meta_log: list[dict] = []
    tip_run = tip = 0.0
    global_max = 0.0
    t0 = settle_open + settle_close
    panel_scripted_during_bout = False

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        if t < settle_open:
            gcmd, phase, cmd, bi = OPEN_CMD, "settle_open", 0.0, -1
            local = 0.0
        elif t < t0:
            a = (t - settle_open) / settle_close
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            phase, cmd, bi, local = "settle_close", 0.0, -1, 0.0
        else:
            tb = t - t0
            bi = min(int(tb // cycle_s), n_cycles - 1)
            local = tb - bi * cycle_s
            if local < push_s:
                gcmd, phase = CLOSE_CMD, "push"
                a = (local / push_s) ** 0.7
                cmd = float(np.clip(push_to, -29.0, 29.0)) * a
            elif local < push_s + hold_s:
                gcmd, phase, cmd = CLOSE_CMD, "hold", float(np.clip(push_to, -29.0, 29.0))
            elif local < push_s + hold_s + rev_s:
                a = (local - push_s - hold_s) / rev_s
                gcmd = CLOSE_CMD * (1.0 - 0.75 * a) + OPEN_CMD * (0.75 * a)
                phase = "rev"
                cmd = push_to * (1.0 - a)
            else:
                a = (local - push_s - hold_s - rev_s) / recover_s
                gcmd = OPEN_CMD * (1.0 - a) + CLOSE_CMD * a
                phase, cmd = "recover", 0.0

        # Inter-bout rest reset ONLY (criteria: reset OK between) — not mid-bout
        if bi >= 0 and phase == "recover" and local >= push_s + hold_s + rev_s + 0.50 * recover_s:
            d.qpos[m.jnt_qposadr[jid_p]] = 0.0
            d.qvel[m.jnt_dofadr[jid_p]] = 0.0
            # note: this is declared inter-bout reset, not mid-bout scripted open

        mj.mj_forward(m, d)

        # Sample closed at bout entry BEFORE push IK/physics (Prefer FAIL bar 1)
        if bi >= 0:
            b = bouts[bi]
            if b["closed_start_deg"] is None and local < 1.5 / CTRL_HZ:
                b["closed_start_deg"] = abs(_angle_deg(m, d, jid_p))

        # Arm IK — no penetration during settle (avoid pre-opening before closed sample)
        do_ik = phase in ("push", "hold", "rev", "recover")
        if do_ik:
            if phase == "recover":
                tgt = arc(0.0) + np.array([-0.04, 0.02, 0.02])
                g = 1.0
            elif phase == "rev":
                th = np.radians(cmd)
                c, s = np.cos(th), np.sin(th)
                retreat = (1.0 - abs(cmd) / (abs(push_to) + 1e-9))
                tgt = arc(cmd) + np.array([c, s, 0.0]) * pen * 0.2 + np.array(
                    [-0.03, 0.0, 0.02]
                ) * retreat
                g = 1.1
            else:
                th_lead = np.radians(cmd + np.sign(push_to) * lead_deg)
                c, s = np.cos(th_lead), np.sin(th_lead)
                xy = hinge_xy + np.array([[c, -s], [s, c]]) @ rest
                th = np.radians(cmd)
                into = np.array([np.cos(th), np.sin(th), 0.0]) * pen
                tgt = np.array([xy[0], xy[1], z]) + into
                g = gain
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = tgt - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), err)
                for jid, dd, jn in zip(jids, dq, ARM_L):
                    lo, hi = m.jnt_range[jid]
                    nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + g * dd, lo, hi))
                    d.qpos[m.jnt_qposadr[jid]] = nv
                    arm_q[jn] = nv
            except np.linalg.LinAlgError:
                pass

        _pin_arms_head(env, arm_q, gcmd)
        _substep(env, arm_q)

        ang = _angle_deg(m, d, jid_p)
        global_max = max(global_max, abs(ang))
        reach = float(np.linalg.norm(d.site_xpos[sid_hand] - d.site_xpos[sid_push]))
        contact_on = _hand_push_face_contact(m, d)
        lever_on = _hand_lever_contact(m, d)

        Rmat = d.xmat[env.bid_body].reshape(3, 3)
        up = float(Rmat[2, 2])
        if up > 0.5:
            tip_run += 1.0 / CTRL_HZ
            tip = max(tip, tip_run)
        else:
            tip_run = 0.0

        # sample closed at first frame of each bout (before push force accumulates)
        if bi >= 0:
            b = bouts[bi]
            # closed_start_deg already sampled pre-push above
            b["max_abs_panel_deg"] = max(b["max_abs_panel_deg"], abs(ang))
            if abs(ang) > abs(b["peak_panel_deg"]):
                b["peak_panel_deg"] = ang
            b["min_reach_m"] = min(b["min_reach_m"], reach)
            if contact_on and phase in ("push", "hold"):
                b["contact_s"] += 1.0 / CTRL_HZ
                if abs(ang) > b["_prev_abs"] + 1e-4:
                    b["contact_while_rising_s"] += 1.0 / CTRL_HZ
            if lever_on:
                b["lever_contact_s"] += 1.0 / CTRL_HZ
            if abs(ang) >= HOLD_DEG and phase in ("push", "hold"):
                b["_hold_run"] += 1.0 / CTRL_HZ
                b["hold20_s"] = max(b["hold20_s"], b["_hold_run"])
            else:
                if phase in ("push", "hold"):
                    b["_hold_run"] = 0.0
            b["_prev_abs"] = abs(ang)
            if abs(ang) > 30.0 + 0.05:
                b["within_30"] = False

        # HUD mark
        if bi < 0:
            mark = "SETTLE"
        elif abs(ang) <= CLOSED_EPS_DEG and phase in ("push", "recover", "rev") and local < 0.3:
            mark = "CLOSED"
        elif phase == "hold" and abs(ang) >= HOLD_DEG:
            mark = "HOLD"
        elif abs(ang) >= OPEN_DEG:
            mark = "OPEN"
        elif contact_on:
            mark = "CONTACT"
        else:
            mark = phase.upper()

        qpos_log.append(d.qpos.copy())
        meta_log.append(
            {
                "t": t,
                "bout": bi,
                "phase": phase,
                "mark": mark,
                "reach": reach,
                "contact": contact_on,
            }
        )

    tip_ok = tip >= 8.0
    for b in bouts:
        cs = b["closed_start_deg"]
        b["closed_ok"] = cs is not None and cs <= CLOSED_EPS_DEG + 1e-6
        b["open25_ok"] = b["max_abs_panel_deg"] >= OPEN_DEG - 1e-6
        b["hold_ok"] = b["hold20_s"] >= HOLD_S - 1e-9
        b["contact_ok"] = b["contact_s"] >= CONTACT_S - 1e-9
        b["pass"] = bool(
            tip_ok
            and b["closed_ok"]
            and b["open25_ok"]
            and b["hold_ok"]
            and b["contact_ok"]
            and b["within_30"]
            and b["lever_contact_s"] < 1e-9  # lever contype 0 → should be 0
            and not panel_scripted_during_bout
        )
        # scrub internal
        b.pop("_hold_run", None)
        b.pop("_prev_abs", None)

    n_pass = sum(1 for b in bouts if b["pass"])
    multi_ok = n_pass == n_cycles
    smoke10 = any(b["max_abs_panel_deg"] >= 10.0 and b["contact_s"] >= CONTACT_S for b in bouts)

    video = ITER / f"{tag}.mp4"
    _render_door(env, qpos_log, meta_log, video)

    return {
        "tag": tag,
        "tip": tip,
        "tip_ok": tip_ok,
        "bouts": bouts,
        "n_pass": n_pass,
        "n_cycles": n_cycles,
        "multi_2of2": multi_ok,
        "smoke10_note": smoke10,
        "global_max_abs_panel_deg": global_max,
        "min_reach_m": min(b["min_reach_m"] for b in bouts),
        "plant": str(plant.relative_to(ROOT)),
        "plant_md5": _md5(plant),
        "honesty": honesty,
        "panel_qpos_scripted_during_bout": panel_scripted_during_bout,
        "panel_actuator": False,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "soft_pass": False,
        "lever_scored_as_open": False,
        "latch_claim": False,
        "walk_through_claim": False,
        "full_90_claim": False,
        "mp4": str(video.relative_to(ROOT)),
        "knobs": {
            "cam_dist": cam_dist,
            "push_to": push_to,
            "gain": gain,
            "pen": pen,
            "lead_deg": lead_deg,
            "rest_xy": list(rest_xy),
            "push_z": push_z,
            "push_s": push_s,
            "hold_s": hold_s,
            "rev_s": rev_s,
            "recover_s": recover_s,
        },
        "pass": bool(multi_ok and tip_ok),
    }


def append_progress(text: str) -> None:
    ITER.mkdir(parents=True, exist_ok=True)
    if not PROGRESS.exists():
        PROGRESS.write_text(
            f"# Gate Door v1 — Controls Prefer FAIL progress\n\n"
            f"**Soft-pass:** OFF · **Spend:** none\n"
            f"**Criteria:** `docs/GATE_DOOR_V1_AI_CRITERIA.md`\n\n"
        )
    with PROGRESS.open("a") as f:
        f.write(text)
        if not text.endswith("\n"):
            f.write("\n")


def write_score(ep: dict, install_status: str, attempts: list[dict], disposition: str) -> None:
    live_md5 = _md5(PLANT_LIVE)
    rows = []
    for b in ep["bouts"]:
        rows.append(
            f"| {b['bout']} | {b['closed_start_deg']} | {b['max_abs_panel_deg']:.2f} | "
            f"{b['contact_s']:.2f} | {b['hold20_s']:.2f} | {b['min_reach_m']*1000:.0f} | "
            f"{'PASS' if b['pass'] else 'FAIL'} |"
        )
    verdict = "PASS" if ep["pass"] else "PREFER_FAIL"
    doc = f"""# Gate Door v1 — AI SCORE

**When:** {_now()}  
**Role:** Founding Controls (Prefer FAIL prove)  
**Criteria:** `docs/GATE_DOOR_V1_AI_CRITERIA.md`  
**Disposition:** **{disposition}** · Soft-pass **OFF**

## Plant

| Item | Value |
|------|-------|
| Score plant | `{ep['plant']}` |
| Score plant md5 | `{ep['plant_md5']}` (lock **adb24309…**) |
| Archive gate_f.md5 (lever-era) | `{live_md5}` |
| Install status | {install_status} |
| Walk M145 md5 | `{_md5(PLANT_WALK)}` (untouched) |
| door_lever contype | **{ep['honesty']['door_lever_contype']}** |
| door_panel_push_face contype | **{ep['honesty']['door_panel_push_face_contype']}** |
| panel actuator | **{ep['honesty']['panel_actuator']}** |

## Verdict: **{verdict}**

| Bout | closed° | max\\|θ\\|° | contact s | hold≥20° s | Δpush mm | Result |
|------|---------|-----------|-----------|------------|----------|--------|
{chr(10).join(rows)}

- tip = **{ep['tip']:.2f}** s (bar ≥8) · tip_ok={ep['tip_ok']}
- multi 2/2 = **{ep['multi_2of2']}**
- optional smoke ≥10° + contact = {ep['smoke10_note']} (not March unlock alone)
- attempts this run = **{len(attempts)}**

## Honesty

- Soft-pass: **OFF** (never)
- Lever contact scored as open: **NO**
- Panel qpos scripted during bout: **NO** (inter-bout rest reset only)
- No latch / 90° / walk-through / UK height claim
- Push site Z ≈ {ep['knobs']['push_z']} (panel center, not lever 0.275)
- Contact pair: `l/r_hand_contact` × `door_panel_push_face`

## Artifacts

- `scripts/score_door_v1.py`
- `{ep['mp4']}` (+ `_kit` / `_panel`)
- `previews/ainex_walk/iterate/GATE_DOOR_V1_SCORE.json`
- `previews/ainex_walk/iterate/GATE_DOOR_V1_PROGRESS.md`
- `previews/ainex_walk/iterate/GATE_DOOR_V1_DONE.json` {"(written)" if disposition=="SUCCESS" else "(not on Prefer FAIL)"}

## Attempts summary

"""
    for a in attempts:
        doc += (
            f"- **{a['tag']}**: pass={a['pass']} max={a['global_max_abs_panel_deg']:.2f}° "
            f"2/2={a['multi_2of2']} tip={a['tip']:.2f} knobs={a['knobs']}\n"
        )
    SCORE_DOC.write_text(doc)
    SCORE_JSON.write_text(
        json.dumps(
            {
                "verdict": verdict,
                "disposition": disposition,
                "criteria": "docs/GATE_DOOR_V1_AI_CRITERIA.md",
                "install_status": install_status,
                "live_md5": live_md5,
                "episode": ep,
                "attempts": [
                    {
                        "tag": a["tag"],
                        "pass": a["pass"],
                        "multi_2of2": a["multi_2of2"],
                        "global_max_abs_panel_deg": a["global_max_abs_panel_deg"],
                        "tip": a["tip"],
                        "bouts": a["bouts"],
                        "knobs": a["knobs"],
                        "mp4": a["mp4"],
                    }
                    for a in attempts
                ],
                "soft_pass": False,
                "when": _now(),
            },
            indent=2,
            default=str,
        )
    )
    if disposition == "SUCCESS":
        DONE_JSON.write_text(
            json.dumps(
                {
                    "gate": "DOOR_V1",
                    "status": "DONE",
                    "verdict": "PASS",
                    "plant_md5": ep["plant_md5"],
                    "live_md5": live_md5,
                    "install_status": install_status,
                    "tip": ep["tip"],
                    "multi_2of2": True,
                    "mp4": ep["mp4"],
                    "score_doc": str(SCORE_DOC.relative_to(ROOT)),
                    "when": _now(),
                    "soft_pass": False,
                },
                indent=2,
            )
        )


ATTEMPT_KNOBS = [
    # Attempt 1 — baseline E-like
    dict(
        cam_dist=0.14, push_to=-28.0, gain=1.4, pen=0.020, lead_deg=4.0,
        rest_xy=(-0.014, 0.045), push_z=0.18, push_s=8.0, hold_s=2.5,
        rev_s=5.5, recover_s=3.5,
    ),
    # Attempt 2 — longer hold + deeper pen + slower push
    dict(
        cam_dist=0.13, push_to=-28.5, gain=1.45, pen=0.022, lead_deg=5.0,
        rest_xy=(-0.013, 0.048), push_z=0.175, push_s=9.0, hold_s=3.0,
        rev_s=6.0, recover_s=4.0,
    ),
    # Attempt 3 — free-edge-er Y, more lead, longer recover for clean closed sample
    dict(
        cam_dist=0.12, push_to=-28.5, gain=1.48, pen=0.023, lead_deg=5.5,
        rest_xy=(-0.012, 0.052), push_z=0.17, push_s=9.5, hold_s=3.2,
        rev_s=6.5, recover_s=4.5,
    ),
    # Attempt 4 (if needed) — milder gain, ensure closed sample + hold
    dict(
        cam_dist=0.135, push_to=-27.5, gain=1.35, pen=0.019, lead_deg=3.5,
        rest_xy=(-0.015, 0.042), push_z=0.18, push_s=8.5, hold_s=3.5,
        rev_s=6.0, recover_s=4.0,
    ),
]


def main() -> int:
    ITER.mkdir(parents=True, exist_ok=True)
    plant, plant_md5, install_status = resolve_plant()
    sgf.PLANT_F = plant
    sha = _sha16(CKPT)
    assert sha == CKPT_SHA16, sha
    assert plant_md5 == PUSH_MD5, plant_md5

    append_progress(
        f"\n## Run start — {_now()}\n\n"
        f"- install_status: **{install_status}**\n"
        f"- score plant: `{plant.relative_to(ROOT)}` md5 `{plant_md5}`\n"
        f"- archive gate_f.md5 (lever-era): `{_md5(PLANT_ARCHIVE)}`\n"f"- Door v1 live: gate_f_push `{plant_md5}`\n"
        f"- walk M145 md5: `{_md5(PLANT_WALK)}` (untouched)\n"
        f"- soft-pass: OFF\n"
    )
    print(f"[DOOR] install={install_status}", flush=True)
    print(f"[DOOR] plant={plant.name} md5={plant_md5}", flush=True)

    attempts: list[dict] = []
    best: dict | None = None

    for i, knobs in enumerate(ATTEMPT_KNOBS, start=1):
        # re-resolve plant mid-run in case Hardware swaps
        plant2, md5_2, inst2 = resolve_plant()
        if md5_2 == PUSH_MD5:
            plant, plant_md5, install_status = plant2, md5_2, inst2
            sgf.PLANT_F = plant
        tag = f"DOOR_V1_A{i:02d}"
        print(f"[DOOR] attempt {i}/{len(ATTEMPT_KNOBS)} {tag} knobs={knobs}", flush=True)
        ep = score_episode(tag=tag, plant=plant, **knobs)
        attempts.append(ep)
        (ITER / f"{tag}.json").write_text(json.dumps(ep, indent=2, default=str))

        bout_lines = []
        for b in ep["bouts"]:
            bout_lines.append(
                f"  - bout{b['bout']}: closed={b['closed_start_deg']} "
                f"max={b['max_abs_panel_deg']:.2f} contact={b['contact_s']:.2f} "
                f"hold20={b['hold20_s']:.2f} pass={b['pass']}"
            )
        append_progress(
            f"\n### Attempt {i} — `{tag}` — {_now()}\n\n"
            f"- pass={ep['pass']} multi_2of2={ep['multi_2of2']} tip={ep['tip']:.2f}\n"
            f"- global_max={ep['global_max_abs_panel_deg']:.2f}° min_reach={ep['min_reach_m']*1000:.0f}mm\n"
            f"- knobs: `{json.dumps(ep['knobs'])}`\n"
            f"- mp4: `{ep['mp4']}`\n"
            + "\n".join(bout_lines)
            + "\n"
        )
        print(
            f"[DOOR] {tag}: pass={ep['pass']} 2/2={ep['multi_2of2']} "
            f"max={ep['global_max_abs_panel_deg']:.2f} tip={ep['tip']:.2f}",
            flush=True,
        )
        for line in bout_lines:
            print(line, flush=True)

        if best is None or (
            (ep["n_pass"], ep["global_max_abs_panel_deg"], ep["tip"])
            > (best["n_pass"], best["global_max_abs_panel_deg"], best["tip"])
        ):
            best = ep

        if ep["pass"]:
            # final plant check before SCORE write
            plant_f, md5_f, inst_f = resolve_plant()
            if md5_f == PUSH_MD5:
                plant, plant_md5, install_status = plant_f, md5_f, inst_f
                ep["plant"] = str(plant.relative_to(ROOT))
                ep["plant_md5"] = plant_md5
            write_score(ep, install_status, attempts, "SUCCESS")
            append_progress(
                f"\n## SUCCESS on attempt {i}\n\n"
                f"- SCORE: `docs/GATE_DOOR_V1_AI_SCORE.md`\n"
                f"- DONE: `previews/ainex_walk/iterate/GATE_DOOR_V1_DONE.json`\n"
            )
            print("=== DOOR_V1_SUCCESS ===", flush=True)
            return 0

        # stop early only after ≥3 serious attempts
        if i >= 3 and best is not None and best["n_pass"] < 2:
            # continue to 4th if we have a near-miss worth one more tune
            if i == 3 and best["global_max_abs_panel_deg"] >= 20:
                continue
            break

    # Prefer FAIL after ≥3
    assert best is not None
    plant_f, md5_f, inst_f = resolve_plant()
    if md5_f == PUSH_MD5:
        install_status = inst_f
        best["plant"] = str(plant_f.relative_to(ROOT))
        best["plant_md5"] = md5_f
    write_score(best, install_status, attempts, "STUCK_REPEATED")
    append_progress(
        f"\n## STUCK_REPEATED after {len(attempts)} attempts — {_now()}\n\n"
        f"- best n_pass={best['n_pass']}/2 max={best['global_max_abs_panel_deg']:.2f}° "
        f"tip={best['tip']:.2f}\n"
        f"- Prefer FAIL SCORE draft written; ping Dave (wants to be present)\n"
        f"- Option C NOT invented mid-score\n"
    )
    print("=== DOOR_V1_STUCK_REPEATED ===", flush=True)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
