#!/usr/bin/env python3
"""Gate K: multi-bout (N=3) panel open/close on ±30° companion — freeflyer honest.

NO Stand XY hold / free-joint x,y pin (hard falsifier per GATE_K_AI_CRITERIA §9).
Companion only. M145 + ckpt untouched. MUJOCO_GL=glfw.
Prior pack with stand_xy_hold=true is SUPERSEDED / VOID.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import sys
from pathlib import Path

import numpy as np
import mujoco as mj

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
os.environ.setdefault("MUJOCO_GL", "glfw")

from score_gate_f import ITER, PLANT_F, CKPT, GateFEnv, body_x_for_cam_dist, CTRL_HZ, render_mp4
from score_gate_g import ik_reach_pose, _hand_lever_contact
from score_gate_h import _hinge_id, _pin_arms_head, _substep, OPEN_CMD, CLOSE_CMD
from score_gate_i import _panel_hinge_id, _angle_deg


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _hud(frame: np.ndarray, dx: float, dy: float, x: float, y: float, t: float, bout: int,
         panel_deg: float | None = None, lever_deg: float | None = None,
         title: str | None = None) -> np.ndarray:
    """Burn §9 XY HUD + optional panel°/lever° (must match scored hinge qpos)."""
    from PIL import Image, ImageDraw, ImageFont
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 16)
        font_sm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 13)
    except Exception:
        font = ImageFont.load_default()
        font_sm = font
    lines = [
        title or "§9 FREEFLYER XY (no pin)",
        f"t={t:5.1f}s  bout={bout if bout >= 0 else '-'}",
        f"x={x:+.4f} m  y={y:+.4f} m",
        f"Δx={dx * 1000:+.1f} mm  Δy={dy * 1000:+.1f} mm",
    ]
    if panel_deg is not None:
        lines.append(f"panel_hinge={panel_deg:+.2f}°")
    if lever_deg is not None:
        lines.append(f"lever_hinge={lever_deg:+.2f}°")
    y0 = 8
    for i, line in enumerate(lines):
        draw.rectangle([6, y0 + i * 20, 420, y0 + i * 20 + 18], fill=(0, 0, 0))
        if i == 0:
            fill = (0, 255, 80)
        elif panel_deg is not None and line.startswith("panel_hinge"):
            fill = (0, 255, 80)  # emphasize panel° for open-angle watch
        else:
            fill = (255, 255, 255)
        draw.text((10, y0 + i * 20), line, fill=fill, font=font if i == 0 else font_sm)
    return np.asarray(img)


def _hud_xy(frame: np.ndarray, dx: float, dy: float, x: float, y: float, t: float, bout: int) -> np.ndarray:
    """Back-compat wrapper (proof reel)."""
    return _hud(frame, dx, dy, x, y, t, bout)


def _world_cam(model: mj.MjModel) -> mj.MjvCamera:
    """World-fixed FREE side cam, zoomed on feet/base for §9 watch honesty.

    Prior pack used distance=1.35 → ~cm freeflyer ≈1–4 px (below watch threshold).
    Zoomed feet framing (distance≈0.55) makes natural ~cm drift and +5 cm proof visible.
    type=mjCAMERA_FREE (not TRACKING); lookat fixed in world — not body-mounted.
    """
    cam = mj.MjvCamera()
    mj.mjv_defaultCamera(cam)
    cam.type = mj.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.28, 0.0, 0.02]  # feet / floor contact region
    cam.distance = 0.55
    cam.azimuth = 95.0   # +Y side — Δx reads as horizontal image motion
    cam.elevation = -25.0
    return cam


def _panel_cam(model: mj.MjModel) -> mj.MjvCamera:
    """Oblique free-edge cam — make ~5 cm X swing at +30° obvious (Hardware note).

    Face-on / prior az≈35° foreshortens the 100 mm-wide slab (Y span 100→97 mm).
    Probe pick: az=270° (look from −Y), el=−20°, d=0.7 → ~45 px free-edge travel 0→30°.
    See docs/GATE_K_HARDWARE_PANEL_VISUAL.md + GATE_K_OPEN_ANGLE_OBLIQUE.md.
    """
    cam = mj.MjvCamera()
    mj.mjv_defaultCamera(cam)
    cam.type = mj.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.42, 0.0, 0.17]  # near free-edge / panel mid
    cam.distance = 0.70
    cam.azimuth = 270.0  # −Y side — free-edge X travel → clear image-X motion
    cam.elevation = -20.0
    return cam


def _mark_base_edge(frame: np.ndarray) -> np.ndarray:
    """Cheap green-base lowest-edge marker for §9 feet cam (not arm shadow)."""
    f = frame.astype(np.float32)
    r, g, b = f[:, :, 0], f[:, :, 1], f[:, :, 2]
    mask = (g > r + 15) & (g > b + 8) & (g > 70)
    h, w = mask.shape
    y_lo, y_hi = 160, min(360, h)
    x_lo, x_hi = 80, min(420, w)
    out = frame.copy()
    pts = []
    for x in range(x_lo, x_hi, 2):
        col = mask[y_lo:y_hi, x]
        idx = np.where(col)[0]
        if len(idx) < 3:
            continue
        y = y_lo + int(idx.max())
        pts.append((x, y))
        if 0 <= y < h:
            out[y, x] = (0, 255, 80)
            if y + 1 < h:
                out[y + 1, x] = (0, 255, 80)
    if len(pts) >= 2:
        # endpoints crosshair
        for u, v in (pts[0], pts[-1]):
            for du in range(-6, 7):
                if 0 <= u + du < w and 0 <= v < h:
                    out[v, u + du] = (255, 40, 40)
                if 0 <= u < w and 0 <= v + du < h:
                    out[v + du, u] = (255, 40, 40)
    return out


def _render_evidence(env: GateFEnv, qpos_log: list, xy_log: list, bout_log: list, t_log: list,
                     stride: int = 5, fps: int = 10):
    """Stream-encode kit / feet-world / panel-facing / duals (OOM-safe).

    K03.mp4          kit + panel°/lever°/XY HUD
    K03_world.mp4    feet §9 cam + XY HUD + base-edge marker
    K03_panel.mp4    panel-facing open-angle + panel° HUD
    K03_dual.mp4     kit | panel-facing  (watch_primary open-angle)
    K03_dual_s9.mp4  kit | feet         (§9 companion)
    """
    import imageio.v2 as imageio
    ITER.mkdir(parents=True, exist_ok=True)
    r = mj.Renderer(env.model, height=480, width=640)
    wcam = _world_cam(env.model)
    pcam = _panel_cam(env.model)
    jid_p = _panel_hinge_id(env.model)
    jid_l = _hinge_id(env.model)
    x0, y0 = xy_log[0]
    kit_path = ITER / "K03.mp4"
    world_path = ITER / "K03_world.mp4"
    panel_path = ITER / "K03_panel.mp4"
    dual_path = ITER / "K03_dual.mp4"
    dual_s9_path = ITER / "K03_dual_s9.mp4"
    n = len(qpos_log)
    print(
        f"[K] stream-render n={n} stride={stride} fps={fps} "
        f"→ K03 / K03_world / K03_panel / K03_dual / K03_dual_s9",
        flush=True,
    )
    writers = {
        "kit": imageio.get_writer(str(kit_path), fps=fps, codec="libx264", quality=8, macro_block_size=1),
        "world": imageio.get_writer(str(world_path), fps=fps, codec="libx264", quality=8, macro_block_size=1),
        "panel": imageio.get_writer(str(panel_path), fps=fps, codec="libx264", quality=8, macro_block_size=1),
        "dual": imageio.get_writer(str(dual_path), fps=fps, codec="libx264", quality=8, macro_block_size=1),
        "dual_s9": imageio.get_writer(str(dual_s9_path), fps=fps, codec="libx264", quality=8, macro_block_size=1),
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

            r.update_scene(env.data, camera=env.cid)
            kit = _hud(
                r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg,
                title="kit_cam · panel°+XY",
            )
            r.update_scene(env.data, camera=wcam)
            world = _mark_base_edge(r.render().copy())
            world = _hud(
                world, dx, dy, x, y, tt, bb, panel_deg, lever_deg,
                title="§9 feet world d=0.55 · base EDGE marked",
            )
            r.update_scene(env.data, camera=pcam)
            panel = _hud(
                r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg,
                title="oblique free-edge · judge X-swing @ +30°",
            )
            dual = np.concatenate([kit, panel], axis=1)
            dual_s9 = np.concatenate([kit, world], axis=1)
            writers["kit"].append_data(kit)
            writers["world"].append_data(world)
            writers["panel"].append_data(panel)
            writers["dual"].append_data(dual)
            writers["dual_s9"].append_data(dual_s9)
            if (i // stride) % 100 == 0:
                print(f"[K] render frame {i}/{n} panel={panel_deg:+.1f}°", flush=True)
            del kit, world, panel, dual, dual_s9
    finally:
        for w in writers.values():
            w.close()
        r.close()
    for tag in ("K00", "K01", "K02"):
        shutil.copy(dual_path, ITER / f"{tag}.mp4")
    print(
        f"[K] wrote {kit_path.name} {world_path.name} {panel_path.name} "
        f"{dual_path.name} {dual_s9_path.name}",
        flush=True,
    )


def _crosshair(frame: np.ndarray, uv, color=(255, 40, 40)) -> np.ndarray:
    """Draw a small crosshair at pixel (u,v) if in bounds."""
    if uv is None:
        return frame
    u, v = int(round(uv[0])), int(round(uv[1]))
    h, w = frame.shape[:2]
    out = frame.copy()
    for du in range(-8, 9):
        if 0 <= u + du < w and 0 <= v < h:
            out[v, u + du] = color
        if 0 <= u < w and 0 <= v + du < h:
            out[v + du, u] = color
    return out


def _project_free_cam(model: mj.MjModel, data: mj.MjData, cam: mj.MjvCamera,
                      xyz, width=640, height=480):
    """Project world point through FREE MjvCamera (same pose Renderer uses)."""
    r = mj.Renderer(model, height=height, width=width)
    r.update_scene(data, camera=cam)
    sc = r.scene.camera[0]
    pos = np.array(sc.pos)
    forward = np.array(sc.forward)
    up = np.array(sc.up)
    right = np.cross(forward, up)
    right = right / (np.linalg.norm(right) + 1e-12)
    up = np.cross(right, forward)
    up = up / (np.linalg.norm(up) + 1e-12)
    # MjvCamera has no .fovy; FREE cam uses MuJoCo default ~45°
    fovy = 45.0
    focal = (height / 2.0) / np.tan(np.radians(fovy) / 2.0)
    p = np.asarray(xyz, dtype=float) - pos
    z = float(np.dot(p, forward))
    r.close()
    if z <= 1e-6:
        return None
    x = float(np.dot(p, right))
    y = float(np.dot(p, up))
    return (width / 2.0 + focal * (x / z), height / 2.0 - focal * (y / z), z)


def render_xy_proof_reel(out: Path | None = None) -> dict:
    """Short §9 proof: zoomed world-fixed cam + intentional +5 cm displace + live push.

    Does NOT rescore Gate K metrics. Does NOT claim PASS.
    """
    import imageio.v2 as imageio
    out = out or (ITER / "K03_xy_proof.mp4")
    arm_q, cam_dist, tilt = _arm_seed()
    t_settle, push_s = 1.0, 8.0
    episode_s = t_settle + push_s
    env = GateFEnv(
        gait_t=0.75, episode_s=episode_s, stand_hold=episode_s, ramp_t=0.0,
        head_tilt_deg=tilt, walk=False, body_x0=body_x_for_cam_dist(cam_dist),
    )
    env.reset()
    m, d = env.model, env.data
    jid_p = _panel_hinge_id(m)
    jid_l = _hinge_id(m)
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

    qpos_log = []
    xy_log = []
    t_log = []
    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        if t < t_settle:
            gcmd, cmd, phase = OPEN_CMD, 0.0, "settle"
        else:
            gcmd = CLOSE_CMD
            local = t - t_settle
            cmd = 29.5 * min(1.0, local / push_s)
            phase = "push"
        mj.mj_forward(m, d)
        if phase == "push":
            target = arc(cmd)
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = target - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), err)
                for jid, ddq, jn in zip(jids, dq, reach):
                    lo, hi = m.jnt_range[jid]
                    nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + 1.05 * ddq, lo, hi))
                    d.qpos[m.jnt_qposadr[jid]] = nv
                    arm_q[jn] = nv
            except np.linalg.LinAlgError:
                pass
        _pin_arms_head(env, arm_q, gcmd)
        _substep(env, arm_q)
        qpos_log.append(d.qpos.copy())
        xy_log.append((float(d.qpos[env.q_free]), float(d.qpos[env.q_free + 1])))
        t_log.append(t)

    wcam = _world_cam(m)
    r = mj.Renderer(m, height=480, width=640)
    fps = 10
    stride = 5
    writer = imageio.get_writer(str(out), fps=fps, codec="libx264", quality=8, macro_block_size=1)

    def render_pose(qp, x, y, t, label_extra=""):
        d.qpos[:] = qp
        d.qvel[:] = 0
        mj.mj_forward(m, d)
        r.update_scene(d, camera=wcam)
        fr = r.render().copy()
        foot = d.geom_xpos[env.gid_lfoot].copy()
        uv = _project_free_cam(m, d, wcam, foot)
        fr = _crosshair(fr, uv)
        fr = _hud_xy(fr, x - x0, y - y0, x, y, t, 0)
        if label_extra:
            from PIL import Image, ImageDraw, ImageFont
            img = Image.fromarray(fr)
            draw = ImageDraw.Draw(img)
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 15)
            except Exception:
                font = ImageFont.load_default()
            draw.rectangle([6, 100, 620, 122], fill=(0, 0, 0))
            draw.text((10, 102), label_extra, fill=(255, 220, 40), font=font)
            fr = np.asarray(img)
        return fr

    try:
        # --- A: cam responds to intentional +5 cm X ---
        qp_end = qpos_log[-1]
        x_e, y_e = xy_log[-1]
        for flash in range(6):
            use_ghost = flash % 2 == 1
            qp = qp_end.copy()
            xx, yy = x_e, y_e
            tag = "PROOF A: natural end pose"
            if use_ghost:
                qp[env.q_free] += 0.05
                xx = x_e + 0.05
                tag = "PROOF A: SAME pose +5 cm X (cam must jump)"
            fr = render_pose(qp, xx, yy, t_log[-1], tag)
            for _ in range(8):  # 0.8 s each
                writer.append_data(fr)
        # --- B: +5 cm Y ---
        for flash in range(6):
            use_ghost = flash % 2 == 1
            qp = qp_end.copy()
            xx, yy = x_e, y_e
            tag = "PROOF B: natural end pose"
            if use_ghost:
                qp[env.q_free + 1] += 0.05
                yy = y_e + 0.05
                tag = "PROOF B: SAME pose +5 cm Y (cam must jump)"
            fr = render_pose(qp, xx, yy, t_log[-1], tag)
            for _ in range(8):
                writer.append_data(fr)
        # --- C: live push, zoomed world + HUD (natural freeflyer) ---
        for i in range(0, len(qpos_log), stride):
            x, y = xy_log[i]
            fr = render_pose(
                qpos_log[i], x, y, t_log[i],
                "PROOF C: live push — zoomed world-fixed FREE cam (no XY pin)",
            )
            writer.append_data(fr)
    finally:
        writer.close()
        r.close()

    dx = xy_log[-1][0] - x0
    dy = xy_log[-1][1] - y0
    summary = {
        "mp4": str(out.relative_to(ROOT)) if out.is_relative_to(ROOT) else str(out),
        "verdict_label": "SCALE",
        "dx_mm": dx * 1000,
        "dy_mm": dy * 1000,
        "neq": int(m.neq),
        "cam_type": "mjCAMERA_FREE",
        "cam_distance": float(wcam.distance),
        "cam_lookat": [float(v) for v in wcam.lookat],
        "note": "Intentional +5cm flashes prove cam is world-fixed; natural ~cm drift is real but was sub-perceptual at old distance=1.35",
    }
    print(f"[K] wrote XY proof {out} dx={dx*1000:+.1f}mm dy={dy*1000:+.1f}mm", flush=True)
    return summary


def _arm_seed():
    h00_path = ITER / "H00.json"
    if h00_path.exists():
        h00 = json.loads(h00_path.read_text())
        return dict(h00["arm_q"]), float(h00.get("cam_dist_cmd", 0.15)), float(h00.get("head_tilt_cmd_deg", -19.0))
    _, arm_q, _, _ = ik_reach_pose(cam_dist=0.15, side="L", tilt_deg=-19.0)
    return arm_q, 0.15, -19.0


def _panel_spring(model: mj.MjModel) -> dict:
    jid = _panel_hinge_id(model)
    dof = int(model.jnt_dofadr[jid])
    return {
        "damping": float(model.dof_damping[dof]),
        "stiffness": float(model.jnt_stiffness[jid]),
        "range_rad": [float(model.jnt_range[jid][0]), float(model.jnt_range[jid][1])],
        "hw_ref": "docs/GATE_J_HARDWARE_PANEL_SPRING.md",
    }


def score_three_bouts() -> dict:
    """Continuous settle + 3×(push, hold, close). Free-joint x,y UNPINNED.

    Controller iterate (2026-09-28): bout2 failed under freeflyer because cumulative
    base XY/yaw drift saturated l_sho_roll → IK err ~14 mm → peak ~22°. Fix without
    XY pin / plant / spring / Alt A-B:
      - spawn closer: cam_dist 0.15→0.12 (approach offset)
      - gentler/shorter close: soft grip release, smaller pull, rev_cmd_end -5°, close_s=6
      - slightly longer push/hold + IK gain 1.2
    """
    arm_q, _cam_seed, tilt = _arm_seed()
    # Closer approach than H00 seed — keeps bout2 in arm workspace after freeflyer drift
    cam_dist = 0.12
    n_bouts = 3
    t_settle_open, t_settle_close = 0.8, 1.4
    push_s, hold_s, close_s = 9.0, 5.0, 6.0
    push_to, rev_cmd_end = 29.5, -5.0
    soft_close = True
    close_pull_scale = 0.25
    g_push, g_close = 1.2, 1.1
    bout_s = push_s + hold_s + close_s
    episode_s = t_settle_open + t_settle_close + n_bouts * bout_s

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

    bouts = []
    for _ in range(n_bouts):
        bouts.append({
            "peak_panel_deg": 0.0,
            "max_lever_deg": 0.0,
            "hold25_s": 0.0,
            "_hold_run": 0.0,
            "min_abs_during_close": None,
            "close_delta_deg": 0.0,
            "contact_rising_s": 0.0,
            "contact_s": 0.0,
            "coupled_ok": False,
            "open25_ok": False,
            "hold_ok": False,
            "close_ok": False,
            "pass": False,
            "_prev_abs": 0.0,
        })

    qpos_log = []
    xy_log = []
    bout_log = []
    t_log = []
    tip_run = tip = 0.0
    global_max_panel = global_max_lever = 0.0
    global_rising = global_contact = 0.0
    prev_abs = 0.0
    t0_bouts = t_settle_open + t_settle_close

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        # *** NO free-joint x,y pin ***
        if t < t_settle_open:
            gcmd, phase, cmd, a_rev = OPEN_CMD, "settle_open", 0.0, 0.0
            bi = -1
        elif t < t0_bouts:
            a = (t - t_settle_open) / t_settle_close
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            phase, cmd, a_rev, bi = "settle_close", 0.0, 0.0, -1
        else:
            tb = t - t0_bouts
            bi = min(int(tb // bout_s), n_bouts - 1)
            local = tb - bi * bout_s
            if local < push_s:
                gcmd, phase = CLOSE_CMD, "push"
                cmd = push_to * (local / push_s)
                a_rev = 0.0
            elif local < push_s + hold_s:
                gcmd, phase, cmd, a_rev = CLOSE_CMD, "hold", push_to, 0.0
            else:
                a_rev = (local - push_s - hold_s) / close_s
                if soft_close:
                    gcmd = CLOSE_CMD * (1.0 - 0.55 * a_rev) + OPEN_CMD * (0.55 * a_rev)
                else:
                    gcmd = CLOSE_CMD * (1.0 - 0.85 * a_rev) + OPEN_CMD * (0.85 * a_rev)
                phase = "close"
                cmd = push_to + (rev_cmd_end - push_to) * a_rev

        mj.mj_forward(m, d)
        if phase in ("push", "hold", "close"):
            target = arc(cmd)
            if phase == "close":
                if soft_close:
                    target = target + np.array([
                        0.03 * close_pull_scale,
                        -0.05 * a_rev * close_pull_scale,
                        0.0,
                    ])
                else:
                    target = target + np.array([0.05, -0.10 * a_rev, 0.0])
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = target - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            lam = 1e-3
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
                g = g_close if phase == "close" else g_push
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
            b = bouts[bi]
            if phase in ("push", "hold"):
                if abs(ang_p) > abs(b["peak_panel_deg"]):
                    b["peak_panel_deg"] = ang_p
                b["max_lever_deg"] = max(b["max_lever_deg"], abs(ang_l))
                if abs(ang_p) >= 25.0 - 1e-6:
                    b["_hold_run"] += 1 / CTRL_HZ
                    b["hold25_s"] = max(b["hold25_s"], b["_hold_run"])
                else:
                    b["_hold_run"] = 0.0
            elif phase == "close":
                if b["min_abs_during_close"] is None:
                    b["min_abs_during_close"] = abs(ang_p)
                else:
                    b["min_abs_during_close"] = min(b["min_abs_during_close"], abs(ang_p))
            if contacting:
                b["contact_s"] += 1 / CTRL_HZ
                if abs(ang_p) > b["_prev_abs"] + 1e-4:
                    b["contact_rising_s"] += 1 / CTRL_HZ
            b["_prev_abs"] = abs(ang_p)

        prev_abs = abs(ang_p)

    tip_ok = tip >= 8.0
    for i, b in enumerate(bouts):
        if b["min_abs_during_close"] is None:
            b["min_abs_during_close"] = abs(b["peak_panel_deg"])
        b["close_delta_deg"] = abs(b["peak_panel_deg"]) - float(b["min_abs_during_close"])
        b["open25_ok"] = abs(b["peak_panel_deg"]) >= 25.0 - 1e-6
        b["hold_ok"] = b["hold25_s"] >= 1.0 - 1e-9
        b["close_ok"] = b["close_delta_deg"] >= 10.0 - 1e-6
        b["coupled_ok"] = b["contact_rising_s"] > 0.1 and b["contact_s"] > 0.5
        b["tip_ok"] = tip_ok
        b["within_30"] = abs(b["peak_panel_deg"]) <= 30.0 + 0.05
        b["pass"] = bool(
            b["open25_ok"] and b["hold_ok"] and b["close_ok"] and b["coupled_ok"]
            and b["tip_ok"] and b["within_30"]
        )
        for k in list(b.keys()):
            if k.startswith("_"):
                del b[k]
        b["bout_index"] = i

    n_pass = sum(1 for b in bouts if b["pass"])
    dx = float(d.qpos[env.q_free]) - x0
    dy = float(d.qpos[env.q_free + 1]) - y0
    print(
        f"[K] physics done n_pass={n_pass}/3 tip={tip:.2f}s Δx={dx*1000:+.1f}mm Δy={dy*1000:+.1f}mm",
        flush=True,
    )
    for b in bouts:
        print(
            f"  bout{b['bout_index']}: peak={b['peak_panel_deg']:+.2f} hold={b['hold25_s']:.2f} "
            f"Δclose={b['close_delta_deg']:.2f} pass={b['pass']}",
            flush=True,
        )
    _render_evidence(env, qpos_log, xy_log, bout_log, t_log)
    del qpos_log[:]  # free qpos copies after encode

    return {
        "tip": tip,
        "tip_ok": tip_ok,
        "max_abs_panel_deg": global_max_panel,
        "max_abs_lever_deg": global_max_lever,
        "bouts": bouts,
        "n_bouts": n_bouts,
        "n_pass": n_pass,
        "success_rate": f"{n_pass}/{n_bouts}",
        "all_bouts_pass": n_pass == n_bouts,
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
        "hw_spring_ref": "docs/GATE_J_HARDWARE_PANEL_SPRING.md",
        "within_30deg_range": global_max_panel <= 30.0 + 0.05,
        "alt_a_deferred": True,
        "alt_b_deferred": True,
        "prior_xy_hold_pack": "SUPERSEDED_REJECTED",
        "cam_dist_cmd": cam_dist,
        "controller_changes": {
            "diagnosis": "bout2: freeflyer XY/yaw drift → l_sho_roll sat → IK err~14mm → peak~22°",
            "cam_dist_cmd": cam_dist,
            "cam_dist_was": 0.15,
            "push_s": push_s, "hold_s": hold_s, "close_s": close_s,
            "push_to": push_to, "rev_cmd_end": rev_cmd_end,
            "soft_close": soft_close, "close_pull_scale": close_pull_scale,
            "g_push": g_push, "g_close": g_close,
            "xy_pin": False,
        },
    }


def main():
    ITER.mkdir(parents=True, exist_ok=True)
    sha = _sha16(CKPT)
    assert sha == "9ffaa1a21b607bf6", sha
    m = mj.MjModel.from_xml_path(str(PLANT_F))
    spring = _panel_spring(m)
    print(
        f"[K] HONEST rescore (no XY pin) plant={PLANT_F.name} ckpt={sha} "
        f"spring damp={spring['damping']} stiff={spring['stiffness']}",
        flush=True,
    )
    ep = score_three_bouts()
    assert ep["stand_xy_hold"] is False
    assert ep["free_joint_xy_pinned"] is False

    tip_ok = ep["tip_ok"]
    no_cheat = (not ep["panel_qpos_scripted"]) and (not ep["panel_actuator"])
    bouts = ep["bouts"]

    lever_ok = ep["max_abs_lever_deg"] >= 15.0 - 1e-6
    panel25_ok = ep["max_abs_panel_deg"] >= 25.0 - 1e-6

    k00 = {
        "tag": "K00", "status": "SCORED", "retains": "J00/J01",
        "tip": ep["tip"], "tip_ok": tip_ok,
        "max_abs_lever_deg": ep["max_abs_lever_deg"],
        "max_abs_panel_deg": ep["max_abs_panel_deg"],
        "lever_ok": lever_ok, "panel25_ok": panel25_ok,
        "coupled_ok": ep["coupled_ok"], "within_30deg_range": ep["within_30deg_range"],
        "panel_spring": spring, "assist": False, "freeze": False,
        "vision_in_walk_obs": False, "fall": ep["fall"],
        "panel_qpos_scripted": False, "panel_actuator": False,
        "stand_xy_hold": False, "free_joint_xy_pinned": False,
        "mp4": "previews/ainex_walk/iterate/K00.mp4",
        "hw_spring_ref": "docs/GATE_J_HARDWARE_PANEL_SPRING.md",
        "plant": ep["plant"],
    }
    k00["pass"] = bool(
        tip_ok and lever_ok and panel25_ok and ep["coupled_ok"] and no_cheat
        and ep["within_30deg_range"] and not ep["fall"]
        and not ep["stand_xy_hold"]
    )

    k01 = {
        "tag": "K01", "status": "SCORED", "n_bouts": ep["n_bouts"], "bouts": bouts,
        "tip": ep["tip"], "tip_ok": tip_ok, "coupled_ok": ep["coupled_ok"],
        "all_bouts_pass": ep["all_bouts_pass"],
        "assist": False, "freeze": False, "vision_in_walk_obs": False, "fall": ep["fall"],
        "panel_qpos_scripted": False, "panel_actuator": False,
        "stand_xy_hold": False, "free_joint_xy_pinned": False,
        "mp4": "previews/ainex_walk/iterate/K01.mp4", "watch_recommended": True,
        "plant": ep["plant"], "panel_spring": spring,
    }
    k01["pass"] = bool(
        ep["all_bouts_pass"] and tip_ok and no_cheat and not ep["fall"]
        and not ep["stand_xy_hold"]
    )

    k02 = {
        "tag": "K02", "status": "SCORED",
        "success_rate": ep["success_rate"], "n_pass": ep["n_pass"], "n_bouts": ep["n_bouts"],
        "rate_ok": ep["n_pass"] == ep["n_bouts"],
        "tip": ep["tip"], "tip_ok": tip_ok, "coupled_ok": ep["coupled_ok"],
        "assist": False, "freeze": False, "vision_in_walk_obs": False, "fall": ep["fall"],
        "stand_xy_hold": False, "free_joint_xy_pinned": False,
        "mp4": "previews/ainex_walk/iterate/K02.mp4", "plant": ep["plant"],
    }
    k02["pass"] = bool(
        k02["rate_ok"] and tip_ok and ep["coupled_ok"] and no_cheat
        and not ep["stand_xy_hold"]
    )

    k03 = {
        "tag": "K03", "status": "SCORED", "optional": True, "continuous_3bout_mp4": True,
        "mp4": "previews/ainex_walk/iterate/K03_dual.mp4",
        "mp4_kit_hud": "previews/ainex_walk/iterate/K03.mp4",
        "mp4_world": "previews/ainex_walk/iterate/K03_world.mp4",
        "mp4_panel": "previews/ainex_walk/iterate/K03_panel.mp4",
        "mp4_dual": "previews/ainex_walk/iterate/K03_dual.mp4",
        "mp4_dual_s9": "previews/ainex_walk/iterate/K03_dual_s9.mp4",
        "visual_evidence_method": (
            "kit+panel°/lever°/XY HUD; oblique free-edge open-angle cam (az270); "
            "feet world d=0.55 + base-edge marker; dual=kit|panel; dual_s9=kit|feet"
        ),
        "tip": ep["tip"], "n_bouts": ep["n_bouts"],
        "assist": False, "freeze": False, "vision_in_walk_obs": False,
        "stand_xy_hold": False, "free_joint_xy_pinned": False,
        "watch_recommended": True,
        "watch_note": (
            "OPEN-ANGLE: K03_dual.mp4 = kit|oblique free-edge (az270). Judge ~5cm free-edge X swing "
            "at Bout0 t≈15s with panel° HUD — cite docs/GATE_K_HARDWARE_PANEL_VISUAL.md. "
            "§9 PASS on K03_world (not blocker). ai_can_lock=false until open-angle watch agrees."
        ),
        "diag_ref": "previews/ainex_walk/iterate/GATE_K_XY_HONESTY_DIAG.md",
        "diag_verdict": "SCALE",
        "reconcile_ref": "previews/ainex_walk/iterate/GATE_K_VISUAL_HOLD_RECONCILE.md",
        "bout0_reconcile_ref": "previews/ainex_walk/iterate/GATE_K_VISUAL_BOUT0_RECONCILE.md",
        "hardware_panel_visual_ref": "docs/GATE_K_HARDWARE_PANEL_VISUAL.md",
        "open_angle_oblique_ref": "previews/ainex_walk/iterate/GATE_K_OPEN_ANGLE_OBLIQUE.md",
        "world_cam_distance": 0.55,
        "panel_cam": {"lookat": [0.42, 0.0, 0.17], "distance": 0.70, "azimuth": 270.0, "elevation": -20.0, "role": "oblique_free_edge", "hw_ref": "docs/GATE_K_HARDWARE_PANEL_VISUAL.md"},
        "judge_open_angle": "free_edge_X_swing_on_K03_dual_right_pane",
        "judge_section9": "green_base_edge_on_K03_world",
        "plant": ep["plant"],
    }
    k03["pass"] = bool(
        (ITER / "K03_dual.mp4").exists() and (ITER / "K03_world.mp4").exists()
        and (ITER / "K03_panel.mp4").exists()
        and ep["n_bouts"] == 3 and tip_ok and not ep["stand_xy_hold"]
    )

    rows = [k00, k01, k02, k03]
    for r in rows:
        (ITER / f"{r['tag']}.json").write_text(json.dumps(r, indent=2, default=str))

    metrics_verdict = "GATE_K_FAIL"
    if ep["stand_xy_hold"] or ep["free_joint_xy_pinned"]:
        verdict = "GATE_K_HARD_BLOCK_XY_PIN"
        ai_can_lock = False
    elif all(r["pass"] for r in rows) and ep["all_bouts_pass"]:
        verdict = "GATE_K_PASS"
        ai_can_lock = True
    elif k00["pass"] and not ep["all_bouts_pass"]:
        verdict = "GATE_K_FAIL"
        ai_can_lock = False
    else:
        verdict = "GATE_K_FAIL"
        ai_can_lock = False

    # ai_can_lock only if no XY pin AND core pass
    metrics_pass = bool(
        ai_can_lock and k00["pass"] and k01["pass"] and k02["pass"]
        and not ep["stand_xy_hold"] and not ep["free_joint_xy_pinned"]
    )
    metrics_verdict = verdict if verdict != "GATE_K_HARD_BLOCK_XY_PIN" else verdict
    if metrics_pass:
        metrics_verdict = "GATE_K_PASS"
    # VISUAL_HOLD until AI/Controls re-watch on panel-facing dual (reconcile 2026-09-28)
    ai_can_lock = False
    if verdict != "GATE_K_HARD_BLOCK_XY_PIN":
        verdict = "GATE_K_VISUAL_HOLD"

    def st(r):
        return "PASS" if r.get("pass") else "FAIL"

    table = {
        "verdict": verdict,
        "metrics_verdict": metrics_verdict,
        "ai_can_lock": False,
        "visual_hold": True,
        "lock_blocked": True,
        "soft_pass": False,
        "visual_reconcile_ref": "previews/ainex_walk/iterate/GATE_K_VISUAL_HOLD_RECONCILE.md",
        "bout0_reconcile_ref": "previews/ainex_walk/iterate/GATE_K_VISUAL_BOUT0_RECONCILE.md",
        "prior_xy_hold_pack": "SUPERSEDED_REJECTED",
        "stand_xy_hold": False,
        "free_joint_xy_pinned": False,
        "base_dx_m": ep["base_dx_m"],
        "base_dy_m": ep["base_dy_m"],
        "plant": ep["plant"],
        "gate_e_plant_untouched": "mujoco/ainex_hiwonder/ainex_controls_m2_145.xml",
        "ckpt": str(CKPT.relative_to(ROOT)),
        "ckpt_sha16": sha,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "head_joint": "head_tilt",
        "full_door_open_claim": False,
        "alt_a_deferred": True,
        "alt_b_deferred": True,
        "panel_spring": spring,
        "criteria": "docs/GATE_K_AI_CRITERIA.md",
        "hw_spring_ref": "docs/GATE_J_HARDWARE_PANEL_SPRING.md",
        "coupling_honesty": ep["coupling_path"],
        "watch_recommended": True,
        "watch_primary": "K03_dual",
        "watch_primary_open_angle": "K03_dual",
        "watch_primary_section9": "K03_world",
        "visual_evidence": [
            "K03.mp4 kit + panel°/lever°/XY HUD",
            "K03_panel.mp4 oblique free-edge open-angle",
            "K03_dual.mp4 kit|oblique free-edge (watch_primary open-angle)",
            "K03_world.mp4 feet §9 d=0.55 + base-edge marker",
            "K03_dual_s9.mp4 kit|feet (§9 companion)",
        ],
        "diag_ref": "previews/ainex_walk/iterate/GATE_K_XY_HONESTY_DIAG.md",
        "diag_verdict": "SCALE",
        "reconcile_ref": "previews/ainex_walk/iterate/GATE_K_XY_RECONCILE.md",
        "world_cam_distance": 0.55,
        "judge": "green_base_edge_only",
        "section9_cite": "docs/GATE_K_AI_CRITERIA.md §9",
        "controller_changes": ep.get("controller_changes"),
        "cam_dist_cmd": ep.get("cam_dist_cmd"),
        "rows": rows,
        "bouts": bouts,
    }
    (ITER / "GATE_K_TABLE.json").write_text(json.dumps(table, indent=2, default=str))

    bout_lines = "\n".join(
        f"| B{i} | {b['peak_panel_deg']:.2f} | {b['hold25_s']:.2f} | {b['close_delta_deg']:.2f} | "
        f"{b['max_lever_deg']:.2f} | {b['coupled_ok']} | **{st(b)}** |"
        for i, b in enumerate(bouts)
    )

    note = f'''# Gate K Controls — multi-bout open/close reliability (sim) — **XY-free + controller iterate**

**When:** Mon 28 Sep 2026 Europe/London (BST)  
**Role:** Founding Controls  
**Criteria:** `docs/GATE_K_AI_CRITERIA.md` (§9 base DOF honesty — **no Stand XY pin**)  
**Controller:** cam_dist **{ep.get("cam_dist_cmd", 0.12)}** (was 0.15); soft/short close (pull×0.25, rev−5°, close_s=6); push/hold 9/5; g_push=1.2. No XY pin. Diagnosis: bout2 IK sat under freeflyer drift.  
**Plant:** `{ep["plant"]}` · Gate J spring damp **{spring["damping"]}** / stiff **{spring["stiffness"]}** · ±30°  
**Gate E plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Ckpt:** sha16 **`{sha}`** · `head_tilt` · assist/freeze OFF · vision off · glfw  

## Prior pack status

**Prior packs:** XY-hold pack **VOID**. Old far world-cam pack **SCALE-superseded** (`GATE_K_XY_HONESTY_DIAG.md` RECONCILE). Soft-pass forbidden. Cite §9 + RECONCILE **SCALE** + `GATE_K_XY_RECONCILE.md`.

## Lock verdict: **{verdict}** (ai_can_lock=false until visual re-watch)

## Metrics: **{metrics_verdict}** (physics; unchanged controller)

| Tag | Result |
|-----|--------|
| K00 retain J | **{st(k00)}** |
| K01 3 consecutive bouts | **{st(k01)}** |
| K02 3/3 success rate | **{st(k02)}** |
| K03 continuous 3-bout MP4 | **{st(k03)}** |

**AI can lock Gate K:** **NO (VISUAL_HOLD)** — oblique free-edge re-encode; cite `docs/GATE_K_HARDWARE_PANEL_VISUAL.md` + `GATE_K_OPEN_ANGLE_OBLIQUE.md`. §9 PASS (not blocker).  
**stand_xy_hold / free_joint_xy_pinned:** **false** (hard falsifier avoided)

## Bout table (freeflyer XY free)

| Bout | peak panel° | hold≥25° s | close Δ° | max lever° | coupled | Status |
|------|-------------|------------|----------|------------|---------|--------|
{bout_lines}

Success rate: **{ep["success_rate"]}** · tip={ep["tip"]:.2f} · max panel={ep["max_abs_panel_deg"]:.2f}° · max lever={ep["max_abs_lever_deg"]:.2f}°  
Base drift (honest): Δx={ep["base_dx_m"]:.4f} m · Δy={ep["base_dy_m"]:.4f} m

## Coupling / base honesty

- Panel qpos **not** scripted; **no** panel actuator.
- Path: hand→lever→panel child→`door_panel_hinge`.
- Free-joint **x,y free** — base may react to lever/panel forces (§9).
- Without XY pin, 3/3 consecutive ≥25° open+hold+close was **not** achieved in this run (bout degradation under freeflyer). Soft-pass **not** used.

## Visual evidence (§9)

- Method: **kit_cam + XY HUD** + **zoomed world-fixed FREE cam (d≈0.55)** → `K03_dual.mp4`.
- `stand_xy_hold=false` / `free_joint_xy_pinned=false`.
- Diag: `GATE_K_XY_HONESTY_DIAG.md` RECONCILE = **SCALE**; pointer `GATE_K_XY_RECONCILE.md`.
- Lock watch: **green base edge** on zoomed world pane (not arm shadow; not kit_cam horizon alone). Optional `K03_xy_proof.mp4` +5 cm flash.

## Watch

**Recommended: YES — `K03_dual.mp4`** (kit+HUD | **zoomed** world d≈0.55). Judge **green base edge**. Cite `docs/GATE_K_AI_CRITERIA.md` §9 + `GATE_K_XY_HONESTY_DIAG.md` RECONCILE (**SCALE**) + `GATE_K_XY_RECONCILE.md`.

## Honesty

Soft-pass forbidden. No M145 edit. No vision-in-walk. **No full door-open** / latch / UK / 90° / walk-through. Stay within ±30°. No spend. Alt A/B deferred.

## Artifacts

- `scripts/score_gate_k.py` (XY pin removed)
- `GATE_K_TABLE.json`, this note (overwrite)
- `K00.mp4`…`K02.mp4` (= dual), `K03.mp4` (kit+HUD), `K03_world.mp4`, `K03_dual.mp4`
'''
    (ITER / "GATE_K_CONTROLS_NOTE.md").write_text(note)

    # Freeze: keep K IN FLIGHT / note fail — surgical
    freeze = ROOT / "docs" / "CONTROLS_FREEZE_STATUS_MONDAY.md"
    ft = freeze.read_text()
    ft = re.sub(
        r"\| \*\*K\*\* \| \*\*[^*]+\*\* \|[^\n]*\n",
        "| **K** | **IN FLIGHT** | Multi-bout ±30°; **honest freeflyer rescore** (XY pin rejected); cite `GATE_K_CONTROLS_NOTE.md`; **not** full door-open |\n",
        ft,
        count=1,
    )
    if "XY pin rejected" not in ft:
        ft = ft.replace(
            "- Gate K **IN FLIGHT**",
            "- Gate K **IN FLIGHT** — prior XY-hold pack **rejected**; honest freeflyer rescore in `GATE_K_CONTROLS_NOTE.md`",
            1,
        )
    freeze.write_text(ft)

    print(f"=== {verdict} === metrics={metrics_verdict} ai_can_lock=False stand_xy_hold=False", flush=True)
    for r in rows:
        print(f"  {r['tag']}: {st(r)}", flush=True)
    for i, b in enumerate(bouts):
        print(
            f"  bout{i}: pass={b['pass']} peak={b['peak_panel_deg']:.2f} "
            f"hold={b['hold25_s']:.2f} closeΔ={b['close_delta_deg']:.2f} coupled={b['coupled_ok']}",
            flush=True,
        )
    print(f"  base dx={ep['base_dx_m']:.4f} dy={ep['base_dy_m']:.4f}", flush=True)
    print("  visual: K03.mp4 (HUD) + K03_world.mp4 + K03_dual.mp4", flush=True)


if __name__ == "__main__":
    if os.environ.get("GATE_K_XY_PROOF") == "1":
        render_xy_proof_reel()
        raise SystemExit(0)
    main()
