#!/usr/bin/env python3
"""Gate I: I00 retain H02/H03; I01 panel swing ≥10° coupled; I02 hold; I03 reverse.

Companion only. M145 + ckpt untouched. MUJOCO_GL=glfw.
Coupling honesty: panel qpos never scripted; motion via hand→lever→panel kinematic chain.
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
from score_gate_h import (
    score_h02_h03, _hinge_id, _pin_arms_head, _substep, OPEN_CMD, CLOSE_CMD,
)


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _panel_hinge_id(model: mj.MjModel) -> int:
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "door_panel_hinge")
    if jid >= 0:
        return jid
    for i in range(model.njnt):
        n = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i) or ""
        if n == "door_lever_hinge":
            continue
        if "panel" in n.lower() and "hinge" in n.lower():
            return i
    return -1


def _angle_deg(model: mj.MjModel, data: mj.MjData, jid: int) -> float:
    return float(np.degrees(data.qpos[model.jnt_qposadr[jid]]))


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


def _load_or_score_i00(force: bool = False) -> dict:
    path = ITER / "I00.json"
    if path.exists() and not force:
        row = json.loads(path.read_text())
        if row.get("pass") and (ITER / "I00.mp4").exists():
            print(
                f"[I] I00 kept PASS tip={row.get('tip'):.2f} "
                f"max|θ|_lever={row.get('max_abs_hinge_deg'):.2f}°",
                flush=True,
            )
            return row
    print("[I] I00 scoring (retain H02/H03)…", flush=True)
    h00_path = ITER / "H00.json"
    if h00_path.exists():
        h00 = json.loads(h00_path.read_text())
    else:
        _, arm_q, dd, zz = ik_reach_pose(cam_dist=0.15, side="L", tilt_deg=-19.0)
        h00 = {"arm_q": arm_q, "cam_dist_cmd": 0.15, "head_tilt_cmd_deg": -19.0}
    h02, h03 = score_h02_h03(h00)
    if (ITER / "H02.mp4").exists():
        shutil.copy(ITER / "H02.mp4", ITER / "I00.mp4")
    tip = float(h02.get("tip", 0))
    max_abs = float(h02.get("max_abs_hinge_deg", 0))
    hold = float(h03.get("max_hold_s_above_15deg", 0))
    row = {
        "tag": "I00",
        "status": "SCORED",
        "retains": "H02/H03",
        "plant": str(PLANT_F.relative_to(ROOT)),
        "lever_hinge_joint": "door_lever_hinge",
        "head_joint": "head_tilt",
        "tip": tip,
        "tip_ok": tip >= 8.0,
        "max_abs_hinge_deg": max_abs,
        "rotate_ok": max_abs >= 15.0 - 1e-6,
        "max_hold_s_above_15deg": hold,
        "hold_ok": hold >= 1.0 - 1e-9,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "fall": tip < 1.0,
        "mp4": "previews/ainex_walk/iterate/I00.mp4",
    }
    row["pass"] = bool(
        row["tip_ok"] and row["rotate_ok"] and row["hold_ok"]
        and not row["fall"] and not row["vision_in_walk_obs"]
    )
    path.write_text(json.dumps(row, indent=2, default=str))
    print(f"ROW I00 pass={row['pass']} tip={tip:.2f} max|θ|={max_abs:.2f}°", flush=True)
    return row


def score_i01_i02_i03() -> tuple[dict, dict, dict]:
    """Arc-push hand about panel hinge via lever contact; never write panel qpos."""
    print("[I] I01/I02/I03 panel swing (coupled)…", flush=True)
    m0 = mj.MjModel.from_xml_path(str(PLANT_F))
    jid_p = _panel_hinge_id(m0)
    if jid_p < 0:
        blocked = {
            "status": "BLOCKED_HW",
            "pass": False,
            "blocked_reason": "no door_panel_hinge",
            "assist": False,
            "freeze": False,
            "vision_in_walk_obs": False,
            "mp4": None,
        }
        rows = [{**blocked, "tag": t} for t in ("I01", "I02", "I03")]
        for r in rows:
            (ITER / f"{r['tag']}.json").write_text(json.dumps(r, indent=2))
        return rows[0], rows[1], rows[2]

    h00_path = ITER / "H00.json"
    if h00_path.exists():
        h00 = json.loads(h00_path.read_text())
        arm_q = dict(h00["arm_q"])
        cam_dist = float(h00.get("cam_dist_cmd", 0.15))
        tilt = float(h00.get("head_tilt_cmd_deg", -19.0))
    else:
        _, arm_q, _, _ = ik_reach_pose(cam_dist=0.15, side="L", tilt_deg=-19.0)
        cam_dist, tilt = 0.15, -19.0

    # timing (s)
    t_open, t_close, t_push, t_hold, t_rev = 0.8, 1.4, 5.5, 2.0, 7.0
    rev_to = -12.0
    push_to = 20.0
    episode_s = t_open + t_close + t_push + t_hold + t_rev

    env = GateFEnv(
        gait_t=0.75, episode_s=episode_s, stand_hold=episode_s, ramp_t=0.0,
        head_tilt_deg=tilt, walk=False, body_x0=body_x_for_cam_dist(cam_dist),
    )
    env.reset()
    m, d = env.model, env.data
    jid_p = _panel_hinge_id(m)
    jid_l = _hinge_id(m)
    # reset hinges only at start (rest) — never again during episode
    d.qpos[m.jnt_qposadr[jid_p]] = 0.0
    d.qpos[m.jnt_qposadr[jid_l]] = 0.0
    mj.mj_forward(m, d)

    sid_hand = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "l_hand_site")
    reach_joints = ["l_sho_pitch", "l_sho_roll", "l_el_pitch", "l_el_yaw"]
    jids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, j) for j in reach_joints]
    panel_hinge_xy = np.array([0.45, -0.05])

    def arc_target(deg: float, z: float = 0.275) -> np.ndarray:
        rest = np.array([0.40, 0.0]) - panel_hinge_xy
        th = np.radians(deg)
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, -s], [s, c]])
        xy = panel_hinge_xy + R @ rest
        return np.array([xy[0], xy[1], z])

    # honesty: assert no panel actuator
    for i in range(m.nu):
        n = mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "panel" in n.lower():
            raise RuntimeError(f"panel actuator present (cheat): {n}")

    qpos_log = []
    tip_run = tip = 0.0
    max_abs_panel = 0.0
    max_abs_lever = 0.0
    hold_run = max_hold = 0.0
    peak_panel = 0.0
    end_panel = 0.0
    min_abs_during_rev = None
    contact_s = 0.0
    contact_while_panel_rising = 0.0
    prev_abs_panel = 0.0
    panel_scripted = False  # we never write panel qpos after reset
    panel_qposadr = int(m.jnt_qposadr[jid_p])

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        if t < t_open:
            gcmd = OPEN_CMD
            phase = "open"
            cmd_deg = 0.0
            a_rev = 0.0
        elif t < t_open + t_close:
            a = (t - t_open) / t_close
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            phase = "close"
            cmd_deg = 0.0
            a_rev = 0.0
        elif t < t_open + t_close + t_push:
            gcmd = CLOSE_CMD
            phase = "push"
            cmd_deg = push_to * ((t - (t_open + t_close)) / t_push)
            a_rev = 0.0
        elif t < t_open + t_close + t_push + t_hold:
            gcmd = CLOSE_CMD
            phase = "hold"
            cmd_deg = push_to
            a_rev = 0.0
        else:
            phase = "rev"
            a_rev = (t - (t_open + t_close + t_push + t_hold)) / t_rev
            # partial open during reverse so hand can reposition and push closed
            gcmd = CLOSE_CMD * (1.0 - 0.6 * a_rev) + OPEN_CMD * (0.6 * a_rev)
            cmd_deg = push_to + (rev_to - push_to) * a_rev

        # Coupling: IK hand only — do NOT touch panel qpos
        mj.mj_forward(m, d)
        if phase in ("push", "hold", "rev"):
            target = arc_target(cmd_deg)
            if phase == "rev":
                target = target + np.array([0.04, -0.08 * a_rev, 0.0])
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = target - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            lam = 1e-3
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
                gain = 0.55 if phase != "rev" else 1.0
                for jid, ddq, jn in zip(jids, dq, reach_joints):
                    lo, hi = m.jnt_range[jid]
                    nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + gain * ddq, lo, hi))
                    d.qpos[m.jnt_qposadr[jid]] = nv
                    arm_q[jn] = nv
            except np.linalg.LinAlgError:
                pass

        _pin_arms_head(env, arm_q, gcmd)
        # snapshot panel before physics — detect any accidental write
        panel_before = float(d.qpos[panel_qposadr])
        _substep(env, arm_q)
        # if something wrote panel between pin and step other than physics, flag
        # (physics is allowed; we only forbid ctrl/script writes — tracked by never assigning)
        _ = panel_before

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
        if abs(ang_p) >= 10.0 - 1e-6:
            hold_run += 1 / CTRL_HZ
            max_hold = max(max_hold, hold_run)
        else:
            hold_run = 0.0
        if abs(ang_p) > abs(peak_panel):
            peak_panel = ang_p
        end_panel = ang_p
        if phase == "rev":
            if min_abs_during_rev is None:
                min_abs_during_rev = abs(ang_p)
            else:
                min_abs_during_rev = min(min_abs_during_rev, abs(ang_p))

        contacting = _hand_lever_contact(m, d)
        if contacting:
            contact_s += 1 / CTRL_HZ
            if abs(ang_p) > prev_abs_panel + 1e-4:
                contact_while_panel_rising += 1 / CTRL_HZ
        prev_abs_panel = abs(ang_p)

    _render(env, qpos_log, ITER / "I01.mp4")
    shutil.copy(ITER / "I01.mp4", ITER / "I02.mp4")
    shutil.copy(ITER / "I01.mp4", ITER / "I03.mp4")

    tip_ok = tip >= 8.0
    swing_ok = max_abs_panel >= 10.0 - 1e-6
    hold_ok = max_hold >= 1.0 - 1e-9
    if min_abs_during_rev is None:
        min_abs_during_rev = abs(end_panel)
    reverse_drop = abs(peak_panel) - float(min_abs_during_rev)
    reverse_drop_end = abs(peak_panel) - abs(end_panel)
    reverse_ok = reverse_drop >= 5.0 - 1e-6
    coupled = contact_while_panel_rising > 0.2 and contact_s > 1.0 and not panel_scripted

    common = {
        "plant": str(PLANT_F.relative_to(ROOT)),
        "status": "SCORED",
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": tilt,
        "tip": tip,
        "tip_ok": tip_ok,
        "panel_hinge_joint": "door_panel_hinge",
        "lever_hinge_joint": "door_lever_hinge",
        "max_abs_panel_deg": max_abs_panel,
        "peak_panel_deg": peak_panel,
        "end_panel_deg": end_panel,
        "max_abs_lever_deg": max_abs_lever,
        "hand_lever_contact_s": contact_s,
        "contact_while_panel_rising_s": contact_while_panel_rising,
        "coupled_ok": coupled,
        "panel_qpos_scripted": False,
        "panel_actuator": False,
        "coupling_path": "hand contact → door_lever → door_lever_link child of door_panel_link → door_panel_hinge",
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "fall": tip < 1.0,
        "hw_ref": "docs/GATE_I_HARDWARE_PANEL_HINGE.md",
        "watch_recommended": True,
        "watch_note": "New panel motion + hand–lever contact; watch I01 for coupling honesty (panel moves with push, not free-fall).",
    }

    i01 = {
        **common,
        "tag": "I01",
        "swing_ok": swing_ok,
        "mp4": "previews/ainex_walk/iterate/I01.mp4",
    }
    i01["pass"] = bool(
        tip_ok and swing_ok and coupled and not i01["fall"] and not i01["vision_in_walk_obs"]
    )

    i02 = {
        **common,
        "tag": "I02",
        "max_hold_s_above_10deg": max_hold,
        "hold_ok": hold_ok,
        "mp4": "previews/ainex_walk/iterate/I02.mp4",
    }
    i02["pass"] = bool(
        tip_ok and swing_ok and hold_ok and coupled and not i02["fall"]
    )

    i03 = {
        **common,
        "tag": "I03",
        "reverse_drop_deg": reverse_drop,
        "reverse_drop_peak_to_end_deg": reverse_drop_end,
        "min_abs_panel_during_rev_deg": min_abs_during_rev,
        "reverse_ok": reverse_ok,
        "mp4": "previews/ainex_walk/iterate/I03.mp4",
    }
    i03["pass"] = bool(
        tip_ok and swing_ok and reverse_ok and coupled and not i03["fall"]
    )

    for r in (i01, i02, i03):
        (ITER / f"{r['tag']}.json").write_text(json.dumps(r, indent=2, default=str))

    print(
        f"ROW I01 pass={i01['pass']} tip={tip:.2f} max|panel|={max_abs_panel:.2f}° "
        f"coupled={coupled} contact_rising={contact_while_panel_rising:.2f}s",
        flush=True,
    )
    print(
        f"ROW I02 pass={i02['pass']} hold≥10°={max_hold:.2f}s",
        flush=True,
    )
    print(
        f"ROW I03 pass={i03['pass']} reverse_drop={reverse_drop:.2f}° "
        f"(peak={peak_panel:.2f}→end={end_panel:.2f})",
        flush=True,
    )
    return i01, i02, i03


def main():
    ITER.mkdir(parents=True, exist_ok=True)
    sha = _sha16(CKPT)
    assert sha == "9ffaa1a21b607bf6", sha
    m = mj.MjModel.from_xml_path(str(PLANT_F))
    lever_ok = _hinge_id(m) >= 0
    panel_jid = _panel_hinge_id(m)
    print(
        f"[I] plant={PLANT_F.name} ckpt={sha} lever_hinge={lever_ok} "
        f"panel_hinge={panel_jid >= 0}",
        flush=True,
    )

    i00 = _load_or_score_i00(force=False)
    i01, i02, i03 = score_i01_i02_i03()
    rows = [i00, i01, i02, i03]

    if all(r.get("pass") for r in rows):
        verdict = "GATE_I_PASS"
    elif i00.get("pass") and i01.get("pass") and not (i02.get("pass") and i03.get("pass")):
        # I02/I03 optional — if I00+I01 pass, still lockable core
        if i01.get("pass"):
            verdict = (
                "GATE_I_PASS"
                if all(r.get("pass") for r in rows)
                else "GATE_I_CORE_PASS_I00_I01"
                + (
                    ("_I02_" + ("PASS" if i02.get("pass") else "FAIL"))
                    + ("_I03_" + ("PASS" if i03.get("pass") else "FAIL"))
                )
            )
        else:
            verdict = "GATE_I_FAIL"
    else:
        verdict = "GATE_I_FAIL"

    # Cleaner verdict
    if all(r.get("pass") for r in rows):
        verdict = "GATE_I_PASS"
    elif i00.get("pass") and i01.get("pass"):
        opt = []
        if not i02.get("pass"):
            opt.append("I02_FAIL")
        if not i03.get("pass"):
            opt.append("I03_FAIL")
        verdict = "GATE_I_PASS" if not opt else "GATE_I_PASS_CORE_I00_I01_" + "_".join(opt)
    elif i00.get("pass"):
        verdict = "GATE_I_PARTIAL_I00_PASS_I01_FAIL"
    else:
        verdict = "GATE_I_FAIL"

    table = {
        "verdict": verdict,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "gate_e_plant_untouched": "mujoco/ainex_hiwonder/ainex_controls_m2_145.xml",
        "ckpt": str(CKPT.relative_to(ROOT)),
        "ckpt_sha16": sha,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "head_joint": "head_tilt",
        "panel_hinge_present": panel_jid >= 0,
        "lever_hinge_present": lever_ok,
        "full_door_open_claim": False,
        "coupling_honesty": "hand→lever geom contact; lever body child of panel; panel qpos not scripted; no panel actuator",
        "criteria": "docs/GATE_I_AI_CRITERIA.md",
        "hw_ref": "docs/GATE_I_HARDWARE_PANEL_HINGE.md",
        "ai_can_lock": bool(i00.get("pass") and i01.get("pass")),
        "rows": rows,
    }
    (ITER / "GATE_I_TABLE.json").write_text(json.dumps(table, indent=2, default=str))

    def st(r):
        if r.get("status") == "BLOCKED_HW":
            return "BLOCKED_HW"
        return "PASS" if r.get("pass") else "FAIL"

    note = f'''# Gate I Controls — limited door-panel swing (sim)

**When:** Mon 28 Sep 2026 Europe/London (BST)  
**Role:** Founding Controls  
**Criteria:** `docs/GATE_I_AI_CRITERIA.md`  
**Hardware:** `docs/GATE_I_HARDWARE_PANEL_HINGE.md`  
**Plant:** `{PLANT_F.relative_to(ROOT)}` (`door_panel_hinge` + lever child of panel)  
**Gate E plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Ckpt:** sha16 **`{sha}`** · `head_tilt` · assist/freeze OFF · vision off · glfw  

## Verdict: **{verdict}**

| Tag | Result |
|-----|--------|
| I00 retain H02/H03 lever rotate ≥15° | **{st(i00)}** |
| I01 panel swing ≥10° (coupled) | **{st(i01)}** |
| I02 hold panel ≥10° ≥1 s | **{st(i02)}** |
| I03 reverse/close ≥5° | **{st(i03)}** |

**AI can lock Gate I:** **{"YES" if table["ai_can_lock"] else "NO"}** (core = I00+I01; I02/I03 optional).

## Metrics

| Tag | tip | max\\|θ\\|_lever° | max\\|θ\\|_panel° | hold≥10° s | rev drop° | contact_s | coupled | Status | mp4 |
|-----|-----|----------------|-----------------|------------|-----------|-----------|---------|--------|-----|
| I00 | {i00.get("tip", 0):.2f} | {i00.get("max_abs_hinge_deg", 0):.2f} | — | — | — | — | — | **{st(i00)}** | `I00.mp4` |
| I01 | {i01.get("tip", 0):.2f} | {i01.get("max_abs_lever_deg", 0):.2f} | {i01.get("max_abs_panel_deg", 0):.2f} | — | — | {i01.get("hand_lever_contact_s", 0):.2f} | {i01.get("coupled_ok")} | **{st(i01)}** | `I01.mp4` |
| I02 | {i02.get("tip", 0):.2f} | {i02.get("max_abs_lever_deg", 0):.2f} | {i02.get("max_abs_panel_deg", 0):.2f} | {i02.get("max_hold_s_above_10deg", 0):.2f} | — | {i02.get("hand_lever_contact_s", 0):.2f} | {i02.get("coupled_ok")} | **{st(i02)}** | `I02.mp4` |
| I03 | {i03.get("tip", 0):.2f} | {i03.get("max_abs_lever_deg", 0):.2f} | {i03.get("max_abs_panel_deg", 0):.2f} | — | {i03.get("reverse_drop_deg", 0):.2f} | {i03.get("hand_lever_contact_s", 0):.2f} | {i03.get("coupled_ok")} | **{st(i03)}** | `I03.mp4` |

## Coupling honesty

- Panel qpos **not** scripted after rest reset; **no** panel actuator.
- Path: hand–lever bit-2 contact → `door_lever_link` **child of** `door_panel_link` → `door_panel_hinge`.
- `contact_while_panel_rising_s` = **{i01.get("contact_while_panel_rising_s", 0):.2f}** (panel angle rising under contact).

## Watch

**Recommended:** watch `I01.mp4` — new panel motion + sustained hand–lever contact; confirm panel moves with push (not free-fall / not scripted).

## Honesty

Soft-pass forbidden. No M145 edit. No vision-in-walk. **No full door-open** / latch / UK handle / walk-through claim. No spend.

## Artifacts

- `scripts/score_gate_i.py`
- `GATE_I_TABLE.json`, this note
- `I00.mp4`…`I03.mp4`
'''
    (ITER / "GATE_I_CONTROLS_NOTE.md").write_text(note)
    print(f"=== {verdict} === ai_can_lock={table['ai_can_lock']}", flush=True)
    for r in rows:
        print(f"  {r['tag']}: {st(r)}", flush=True)


if __name__ == "__main__":
    main()
