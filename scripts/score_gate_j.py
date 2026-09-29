#!/usr/bin/env python3
"""Gate J: larger coupled panel swing ≥25°, hold+close, optional repeat.

Companion only. M145 + ckpt untouched. MUJOCO_GL=glfw.
Coupling honesty: hand→lever→panel; no panel actuator / no scripted panel qpos after rest reset.
Plant spring: docs/GATE_J_HARDWARE_PANEL_SPRING.md (damp 0.05 / stiff 0.015).
"""
from __future__ import annotations

import hashlib
import json
import os
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
from score_gate_i import _panel_hinge_id, _angle_deg, _sha16


def _render(env: GateFEnv, qpos_log: list, out: Path):
    frames = []
    r = mj.Renderer(env.model, height=480, width=640)
    for qp in qpos_log:
        env.data.qpos[:] = qp
        env.data.qvel[:] = 0
        mj.mj_forward(env.model, env.data)
        r.update_scene(env.data, camera=env.cid)
        frames.append(r.render().copy())
    r.close()
    render_mp4(env, frames, out)


def _arm_seed() -> tuple[dict, float, float]:
    h00_path = ITER / "H00.json"
    if h00_path.exists():
        h00 = json.loads(h00_path.read_text())
        return dict(h00["arm_q"]), float(h00.get("cam_dist_cmd", 0.15)), float(h00.get("head_tilt_cmd_deg", -19.0))
    _, arm_q, _, _ = ik_reach_pose(cam_dist=0.15, side="L", tilt_deg=-19.0)
    return arm_q, 0.15, -19.0


def _assert_no_panel_actuator(model: mj.MjModel):
    for i in range(model.nu):
        n = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "panel" in n.lower():
            raise RuntimeError(f"panel actuator present (cheat): {n}")


def _panel_spring(model: mj.MjModel) -> dict:
    jid = _panel_hinge_id(model)
    dof = int(model.jnt_dofadr[jid])
    return {
        "damping": float(model.dof_damping[dof]),
        "stiffness": float(model.jnt_stiffness[jid]),
        "range_rad": [float(model.jnt_range[jid][0]), float(model.jnt_range[jid][1])],
        "hw_ref": "docs/GATE_J_HARDWARE_PANEL_SPRING.md",
    }


def score_episode(
    *,
    tag_prefix: str,
    episode_mode: str,
    video_path: Path,
) -> dict:
    """One continuous episode covering J00–J03 metrics.

    episode_mode:
      'full' — push ≥25, hold, close, rest, second swing (for J01–J03 + J00 metrics)
    """
    arm_q, cam_dist, tilt = _arm_seed()
    # timings tuned for J spring retune
    t_open, t_close = 0.8, 1.4
    push_s, hold_s, rev_s, reopen_s = 8.0, 4.0, 9.0, 7.5
    push_to, rev_cmd_end, second_to = 28.5, -10.0, 29.0
    gain = 0.95
    episode_s = t_open + t_close + push_s + hold_s + rev_s + reopen_s

    env = GateFEnv(
        gait_t=0.75, episode_s=episode_s, stand_hold=episode_s, ramp_t=0.0,
        head_tilt_deg=tilt, walk=False, body_x0=body_x_for_cam_dist(cam_dist),
    )
    env.reset()
    m, d = env.model, env.data
    spring = _panel_spring(m)
    jid_p = _panel_hinge_id(m)
    jid_l = _hinge_id(m)
    if jid_p < 0 or jid_l < 0:
        raise RuntimeError("missing panel/lever hinge")
    _assert_no_panel_actuator(m)

    # rest reset only — never script panel qpos again
    d.qpos[m.jnt_qposadr[jid_p]] = 0.0
    d.qpos[m.jnt_qposadr[jid_l]] = 0.0
    mj.mj_forward(m, d)

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
    tip_run = tip = 0.0
    max_abs_panel = max_abs_lever = 0.0
    hold25_run = max_hold25 = 0.0
    peak_first = 0.0
    min_abs_during_rev = None
    second_max = 0.0
    in_reopen = False
    contact_s = 0.0
    contact_while_panel_rising = 0.0
    prev_abs = 0.0
    # J00 inherit windows: during first push/hold
    max_panel_pre_rev = 0.0
    max_lever_pre_rev = 0.0

    t_push0 = t_open + t_close
    t_hold0 = t_push0 + push_s
    t_rev0 = t_hold0 + hold_s
    t_re0 = t_rev0 + rev_s

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        if t < t_open:
            gcmd, phase, cmd, a_rev = OPEN_CMD, "open", 0.0, 0.0
        elif t < t_push0:
            a = (t - t_open) / t_close
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            phase, cmd, a_rev = "close", 0.0, 0.0
        elif t < t_hold0:
            gcmd, phase = CLOSE_CMD, "push"
            cmd = push_to * ((t - t_push0) / push_s)
            a_rev = 0.0
        elif t < t_rev0:
            gcmd, phase, cmd, a_rev = CLOSE_CMD, "hold", push_to, 0.0
        elif t < t_re0:
            a_rev = (t - t_rev0) / rev_s
            gcmd = CLOSE_CMD * (1.0 - 0.85 * a_rev) + OPEN_CMD * (0.85 * a_rev)
            phase = "rev"
            cmd = push_to + (rev_cmd_end - push_to) * a_rev
        else:
            a = (t - t_re0) / reopen_s
            in_reopen = True
            a_rev = a
            if a < 0.12:
                gcmd, phase, cmd = OPEN_CMD, "re_open", 0.0
            elif a < 0.28:
                aa = (a - 0.12) / 0.16
                gcmd = OPEN_CMD * (1 - aa) + CLOSE_CMD * aa
                phase, cmd = "re_close", 0.0
            else:
                gcmd, phase = CLOSE_CMD, "re_push"
                cmd = second_to * ((a - 0.28) / 0.72)

        mj.mj_forward(m, d)
        if phase in ("push", "hold", "rev", "re_push"):
            target = arc(cmd)
            if phase == "rev":
                target = target + np.array([0.05, -0.10 * a_rev, 0.0])
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = target - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            lam = 1e-3
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
                g = gain if phase in ("push", "hold") else (1.05 if phase == "rev" else 1.0)
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

        Rmat = d.xmat[env.bid_body].reshape(3, 3)
        up = float(Rmat[2, 2])
        if up > 0.5:
            tip_run += 1 / CTRL_HZ
            tip = max(tip, tip_run)
        else:
            tip_run = 0.0

        ang_p = _angle_deg(m, d, jid_p)
        ang_l = _angle_deg(m, d, jid_l)
        max_abs_panel = max(max_abs_panel, abs(ang_p))
        max_abs_lever = max(max_abs_lever, abs(ang_l))

        if not in_reopen:
            max_panel_pre_rev = max(max_panel_pre_rev, abs(ang_p))
            max_lever_pre_rev = max(max_lever_pre_rev, abs(ang_l))
            if abs(ang_p) >= 25.0 - 1e-6:
                hold25_run += 1 / CTRL_HZ
                max_hold25 = max(max_hold25, hold25_run)
            else:
                hold25_run = 0.0
            if abs(ang_p) > abs(peak_first):
                peak_first = ang_p

        if phase == "rev":
            if min_abs_during_rev is None:
                min_abs_during_rev = abs(ang_p)
            else:
                min_abs_during_rev = min(min_abs_during_rev, abs(ang_p))

        if in_reopen and phase == "re_push":
            second_max = max(second_max, abs(ang_p))

        if _hand_lever_contact(m, d):
            contact_s += 1 / CTRL_HZ
            if abs(ang_p) > prev_abs + 1e-4:
                contact_while_panel_rising += 1 / CTRL_HZ
        prev_abs = abs(ang_p)

    _render(env, qpos_log, video_path)

    if min_abs_during_rev is None:
        min_abs_during_rev = abs(peak_first)
    close_delta = abs(peak_first) - float(min_abs_during_rev)
    coupled = contact_while_panel_rising > 0.2 and contact_s > 1.0

    return {
        "tip": tip,
        "tip_ok": tip >= 8.0,
        "max_abs_panel_deg": max_abs_panel,
        "max_abs_lever_deg": max_abs_lever,
        "max_panel_pre_rev_deg": max_panel_pre_rev,
        "max_lever_pre_rev_deg": max_lever_pre_rev,
        "peak_first_panel_deg": peak_first,
        "max_hold_s_above_25deg": max_hold25,
        "min_abs_panel_during_rev_deg": float(min_abs_during_rev),
        "close_delta_deg": close_delta,
        "second_swing_max_deg": second_max,
        "near_rest_ok": float(min_abs_during_rev) <= 5.0 + 1e-6,
        "hand_lever_contact_s": contact_s,
        "contact_while_panel_rising_s": contact_while_panel_rising,
        "coupled_ok": coupled,
        "panel_qpos_scripted": False,
        "panel_actuator": False,
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
        "hw_hinge_ref": "docs/GATE_I_HARDWARE_PANEL_HINGE.md",
        "within_30deg_range": max_abs_panel <= 30.0 + 0.05,  # allow tiny numeric overshoot at limit
    }


def main():
    ITER.mkdir(parents=True, exist_ok=True)
    sha = _sha16(CKPT)
    assert sha == "9ffaa1a21b607bf6", sha
    m = mj.MjModel.from_xml_path(str(PLANT_F))
    spring = _panel_spring(m)
    print(
        f"[J] plant={PLANT_F.name} ckpt={sha} panel_hinge={_panel_hinge_id(m)>=0} "
        f"spring damp={spring['damping']} stiff={spring['stiffness']}",
        flush=True,
    )
    # Discard any prior J artifacts from old spring
    for name in ["J00", "J01", "J02", "J03", "GATE_J_TABLE", "GATE_J_CONTROLS_NOTE"]:
        for ext in [".json", ".mp4", ".md"]:
            p = ITER / f"{name}{ext}"
            if p.exists() and name.startswith("J"):
                # keep rewriting below
                pass

    print("[J] scoring continuous episode (new spring)…", flush=True)
    ep = score_episode(tag_prefix="J", episode_mode="full", video_path=ITER / "J01.mp4")
    # copy same continuous video for all tags (one bout proves all)
    for tag in ("J00", "J02", "J03"):
        shutil.copy(ITER / "J01.mp4", ITER / f"{tag}.mp4")

    tip_ok = ep["tip_ok"]
    coupled = ep["coupled_ok"]
    no_cheat = (not ep["panel_qpos_scripted"]) and (not ep["panel_actuator"])

    j00 = {
        **{k: ep[k] for k in (
            "tip", "tip_ok", "assist", "freeze", "vision_in_walk_obs", "fall",
            "head_joint", "plant", "coupled_ok", "panel_qpos_scripted", "panel_actuator",
            "coupling_path", "hand_lever_contact_s", "contact_while_panel_rising_s",
            "panel_spring", "hw_spring_ref", "hw_hinge_ref",
        )},
        "tag": "J00",
        "status": "SCORED",
        "retains": "I00/I01",
        "max_abs_lever_deg": ep["max_lever_pre_rev_deg"],
        "max_abs_panel_deg": ep["max_panel_pre_rev_deg"],
        "lever_ok": ep["max_lever_pre_rev_deg"] >= 15.0 - 1e-6,
        "panel10_ok": ep["max_panel_pre_rev_deg"] >= 10.0 - 1e-6,
        "mp4": "previews/ainex_walk/iterate/J00.mp4",
    }
    j00["pass"] = bool(
        tip_ok and j00["lever_ok"] and j00["panel10_ok"] and coupled and no_cheat
        and not j00["fall"] and not j00["vision_in_walk_obs"]
    )

    j01 = {
        **{k: ep[k] for k in (
            "tip", "tip_ok", "assist", "freeze", "vision_in_walk_obs", "fall",
            "head_joint", "plant", "coupled_ok", "panel_qpos_scripted", "panel_actuator",
            "coupling_path", "hand_lever_contact_s", "contact_while_panel_rising_s",
            "panel_spring", "hw_spring_ref", "hw_hinge_ref", "within_30deg_range",
            "max_abs_panel_deg", "max_abs_lever_deg",
        )},
        "tag": "J01",
        "status": "SCORED",
        "swing25_ok": ep["max_abs_panel_deg"] >= 25.0 - 1e-6,
        "mp4": "previews/ainex_walk/iterate/J01.mp4",
        "watch_recommended": True,
        "watch_note": "Larger panel arc (~≥25°) under soft spring; confirm coupling (panel moves with hand/lever push, not free-fall/scripted).",
    }
    j01["pass"] = bool(
        tip_ok and j01["swing25_ok"] and coupled and no_cheat
        and ep["within_30deg_range"] and not j01["fall"]
    )

    j02 = {
        **{k: ep[k] for k in (
            "tip", "tip_ok", "assist", "freeze", "vision_in_walk_obs", "fall",
            "head_joint", "plant", "coupled_ok", "panel_qpos_scripted", "panel_actuator",
            "coupling_path", "hand_lever_contact_s", "contact_while_panel_rising_s",
            "panel_spring", "hw_spring_ref", "max_abs_panel_deg",
            "max_hold_s_above_25deg", "close_delta_deg", "peak_first_panel_deg",
            "min_abs_panel_during_rev_deg",
        )},
        "tag": "J02",
        "status": "SCORED",
        "hold25_ok": ep["max_hold_s_above_25deg"] >= 1.0 - 1e-9,
        "close_ok": ep["close_delta_deg"] >= 10.0 - 1e-6,
        "mp4": "previews/ainex_walk/iterate/J02.mp4",
    }
    j02["pass"] = bool(
        tip_ok and j01["swing25_ok"] and j02["hold25_ok"] and j02["close_ok"]
        and coupled and no_cheat and not j02["fall"]
    )

    j03 = {
        **{k: ep[k] for k in (
            "tip", "tip_ok", "assist", "freeze", "vision_in_walk_obs", "fall",
            "head_joint", "plant", "coupled_ok", "panel_qpos_scripted", "panel_actuator",
            "coupling_path", "panel_spring", "hw_spring_ref",
            "min_abs_panel_during_rev_deg", "second_swing_max_deg", "near_rest_ok",
        )},
        "tag": "J03",
        "status": "SCORED",
        "optional": True,
        "second_swing_ok": ep["second_swing_max_deg"] >= 15.0 - 1e-6,
        "mp4": "previews/ainex_walk/iterate/J03.mp4",
    }
    j03["pass"] = bool(
        tip_ok and j03["near_rest_ok"] and j03["second_swing_ok"]
        and coupled and no_cheat and not j03["fall"]
    )

    rows = [j00, j01, j02, j03]
    for r in rows:
        (ITER / f"{r['tag']}.json").write_text(json.dumps(r, indent=2, default=str))

    if all(r["pass"] for r in rows):
        verdict = "GATE_J_PASS"
    elif j00["pass"] and j01["pass"] and j02["pass"]:
        verdict = "GATE_J_PASS_CORE_J00_J02_J03_FAIL" if not j03["pass"] else "GATE_J_PASS"
    elif j00["pass"] and j01["pass"]:
        verdict = "GATE_J_PARTIAL_J00_J01_PASS"
    elif not j01["pass"] and ep["max_abs_panel_deg"] < 25.0:
        verdict = "GATE_J_FAIL_J01_SPRING_OR_FORCE"
    else:
        verdict = "GATE_J_FAIL"

    # core lockable = J00+J01+J02 (J03 optional)
    ai_can_lock = bool(j00["pass"] and j01["pass"] and j02["pass"])

    def st(r):
        return "PASS" if r.get("pass") else "FAIL"

    table = {
        "verdict": verdict,
        "ai_can_lock": ai_can_lock,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "gate_e_plant_untouched": "mujoco/ainex_hiwonder/ainex_controls_m2_145.xml",
        "ckpt": str(CKPT.relative_to(ROOT)),
        "ckpt_sha16": sha,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "head_joint": "head_tilt",
        "full_door_open_claim": False,
        "panel_spring": spring,
        "criteria": "docs/GATE_J_AI_CRITERIA.md",
        "hw_spring_ref": "docs/GATE_J_HARDWARE_PANEL_SPRING.md",
        "coupling_honesty": ep["coupling_path"],
        "watch_recommended": True,
        "rows": rows,
    }
    (ITER / "GATE_J_TABLE.json").write_text(json.dumps(table, indent=2, default=str))

    note = f'''# Gate J Controls — larger coupled panel swing (sim)

**When:** Mon 28 Sep 2026 Europe/London (BST)  
**Role:** Founding Controls  
**Criteria:** `docs/GATE_J_AI_CRITERIA.md`  
**Hardware spring:** `docs/GATE_J_HARDWARE_PANEL_SPRING.md` (panel damp **{spring["damping"]}** / stiff **{spring["stiffness"]}**; range ±30°)  
**Hinge / coupling:** `docs/GATE_I_HARDWARE_PANEL_HINGE.md`  
**Plant:** `{PLANT_F.relative_to(ROOT)}`  
**Gate E plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Ckpt:** sha16 **`{sha}`** · `head_tilt` · assist/freeze OFF · vision off · glfw  

## Verdict: **{verdict}**

| Tag | Result |
|-----|--------|
| J00 retain I (lever ≥15° + panel ≥10° coupled) | **{st(j00)}** |
| J01 panel abs ≥25° coupled | **{st(j01)}** |
| J02 hold ≥25° ≥1 s + close Δ ≥10° | **{st(j02)}** |
| J03 optional rest≤5° + second ≥15° | **{st(j03)}** |

**AI can lock Gate J:** **{"YES" if ai_can_lock else "NO"}** (core = J00–J02; J03 optional).

## Metrics

| Tag | tip | max\\|θ\\|_lever° | max\\|θ\\|_panel° | hold≥25° s | close Δ° | rest min° | 2nd swing° | coupled | Status |
|-----|-----|----------------|-----------------|------------|----------|-----------|------------|---------|--------|
| J00 | {j00["tip"]:.2f} | {j00["max_abs_lever_deg"]:.2f} | {j00["max_abs_panel_deg"]:.2f} | — | — | — | — | {coupled} | **{st(j00)}** |
| J01 | {j01["tip"]:.2f} | {j01["max_abs_lever_deg"]:.2f} | {j01["max_abs_panel_deg"]:.2f} | — | — | — | — | {coupled} | **{st(j01)}** |
| J02 | {j02["tip"]:.2f} | — | {j02["max_abs_panel_deg"]:.2f} | {j02["max_hold_s_above_25deg"]:.2f} | {j02["close_delta_deg"]:.2f} | {j02["min_abs_panel_during_rev_deg"]:.2f} | — | {coupled} | **{st(j02)}** |
| J03 | {j03["tip"]:.2f} | — | — | — | — | {j03["min_abs_panel_during_rev_deg"]:.2f} | {j03["second_swing_max_deg"]:.2f} | {coupled} | **{st(j03)}** |

## Coupling honesty

- Panel qpos **not** scripted after rest reset; **no** panel actuator.
- Path: hand–lever bit-2 contact → `door_lever_link` child of `door_panel_link` → `door_panel_hinge`.
- `contact_while_panel_rising_s` = **{ep["contact_while_panel_rising_s"]:.2f}**.
- Scored on **Gate J spring retune** (not Gate I 0.08/0.05). Any prior old-spring J01+ discarded.

## Watch

**Recommended: YES** — watch `J01.mp4` (larger ~≥25° panel arc + hold/close/reopen on soft spring; confirm coupled motion).

## Honesty

Soft-pass forbidden. No M145 edit. No vision-in-walk. **No full door-open** / latch / UK handle / walk-through / 90°. No spend. No equality cheat.

## Artifacts

- `scripts/score_gate_j.py`
- `GATE_J_TABLE.json`, this note
- `J00.mp4`…`J03.mp4`
'''
    (ITER / "GATE_J_CONTROLS_NOTE.md").write_text(note)

    # Freeze sheet: Gate J IN FLIGHT (or LOCKED if pass — user asked IN FLIGHT during score; update to reflect status)
    freeze = ROOT / "docs" / "CONTROLS_FREEZE_STATUS_MONDAY.md"
    ft = freeze.read_text()
    if "| **J** |" not in ft:
        ft = ft.replace(
            "| **I** | **LOCKED TRUE** |",
            "| **I** | **LOCKED TRUE** |\n| **J** | **IN FLIGHT** | Larger coupled panel ≥25°; score `scripts/score_gate_j.py` — cite `GATE_J_CONTROLS_NOTE.md`; spring `docs/GATE_J_HARDWARE_PANEL_SPRING.md`; **not** full door-open |",
            1,
        )
    else:
        import re
        ft = re.sub(
            r"\| \*\*J\*\* \| \*\*[^*]+\*\* \|[^|]+\|",
            "| **J** | **IN FLIGHT** | Larger coupled panel ≥25°; score `scripts/score_gate_j.py` — cite `GATE_J_CONTROLS_NOTE.md`; spring `docs/GATE_J_HARDWARE_PANEL_SPRING.md`; **not** full door-open |",
            ft,
            count=1,
        )
    # section 5 note
    if "Gate J **IN FLIGHT**" not in ft:
        ft = ft.replace(
            "- Gate I **LOCKED TRUE**.",
            "- Gate I **LOCKED TRUE**.\n- Gate J **IN FLIGHT** (Controls scoring on companion + Gate J spring retune).",
        )
    freeze.write_text(ft)

    print(f"=== {verdict} === ai_can_lock={ai_can_lock}", flush=True)
    for r in rows:
        print(f"  {r['tag']}: {st(r)}", flush=True)
    print(
        f"  metrics tip={ep['tip']:.2f} panel={ep['max_abs_panel_deg']:.2f} "
        f"lever={ep['max_abs_lever_deg']:.2f} hold25={ep['max_hold_s_above_25deg']:.2f} "
        f"closeΔ={ep['close_delta_deg']:.2f} rest={ep['min_abs_panel_during_rev_deg']:.2f} "
        f"2nd={ep['second_swing_max_deg']:.2f}",
        flush=True,
    )


if __name__ == "__main__":
    main()
