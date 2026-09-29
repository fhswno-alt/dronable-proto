#!/usr/bin/env python3
"""Gate H: H00 reach, H01 grasp hold ≥1s, H02 lever rotate ≥15°, H03 hold rotate ≥1s.

Companion plant only. M145 + ckpt untouched. MUJOCO_GL=glfw.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco as mj
import walk_gait_ainex as wg

from score_gate_f import (
    CTRL_HZ, ITER, PLANT_F, CKPT, render_mp4, body_x_for_cam_dist, GateFEnv,
    cam_to_lever_horiz, cam_to_lever_dist, lever_in_ego, off_axis_deg,
)
from score_gate_g import (
    ik_reach_pose, _hand_lever_contact, _min_hand_lever_dist, _hold_reach_episode,
)

DEG15 = math.radians(15.0)
OPEN_CMD = -1.2
CLOSE_CMD = 0.0


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _hinge_id(model: mj.MjModel) -> int:
    return mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "door_lever_hinge")


def _hinge_angle_deg(model: mj.MjModel, data: mj.MjData) -> float:
    jid = _hinge_id(model)
    if jid < 0:
        return 0.0
    return float(math.degrees(data.qpos[model.jnt_qposadr[jid]]))


def _pin_arms_head(env: GateFEnv, arm_q: dict, gcmd: float | None = None):
    m, d = env.model, env.data
    for jn, val in arm_q.items():
        jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            d.qpos[m.jnt_qposadr[jid]] = val
            d.qvel[m.jnt_dofadr[jid]] = 0.0
    d.qpos[m.jnt_qposadr[env.jid_tilt]] = env.head_tilt
    qdes = wg.gait_targets(float(d.time), False, 0.0)
    qdes.update(arm_q)
    wg.set_ctrl(m, d, qdes, env.act_idx)
    d.ctrl[env.aid_tilt] = env.head_tilt
    if gcmd is not None:
        d.ctrl[env.act_idx["l_gripper_pos"]] = gcmd
        d.ctrl[env.act_idx["r_gripper_pos"]] = gcmd


def _substep(env: GateFEnv, arm_q: dict, n: int | None = None):
    m, d = env.model, env.data
    n = n or env.steps_per_ctrl
    for _ in range(n):
        for jn, val in arm_q.items():
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                d.qpos[m.jnt_qposadr[jid]] = val
                d.qvel[m.jnt_dofadr[jid]] = 0.0
        d.qpos[m.jnt_qposadr[env.jid_tilt]] = env.head_tilt
        d.qfrc_applied[:] = 0
        d.xfrc_applied[:] = 0
        mj.mj_step(m, d)


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


def score_h00() -> dict:
    print("[H] H00…", flush=True)
    _, arm_q, dd, zz = ik_reach_pose(cam_dist=0.15, side="L", tilt_deg=-19.0)
    row = _hold_reach_episode(
        tag="H00", arm_q=arm_q, cam_dist=0.15, tilt_deg=-19.0,
        episode_s=9.0, enable_physics_contact=True, video=True,
    )
    row["retains"] = "G02"
    row["ik_seed_dist"] = dd
    row["ik_seed_z"] = zz
    row["pass"] = bool(
        row["tip_ok"] and row["reach_ok"] and not row["fall"]
        and row["min_hand_lever_dist_m"] <= 0.05
        and 0.25 <= row["hand_z_at_best"] <= 0.30
        and not row["vision_in_walk_obs"]
    )
    (ITER / "H00.json").write_text(json.dumps(row, indent=2, default=str))
    print(f"ROW H00 pass={row['pass']} tip={row['tip']:.2f} d={row['min_hand_lever_dist_m']:.4f} z={row['hand_z_at_best']:.3f}", flush=True)
    return row


def score_h01(h00: dict) -> dict:
    print("[H] H01 grasp hold…", flush=True)
    arm_q = h00["arm_q"]
    cam_dist = float(h00.get("cam_dist_cmd", 0.15))
    tilt = float(h00.get("head_tilt_cmd_deg", -19.0))
    env = GateFEnv(
        gait_t=0.75, episode_s=9.0, stand_hold=9.0, ramp_t=0.0,
        head_tilt_deg=tilt, walk=False, body_x0=body_x_for_cam_dist(cam_dist),
    )
    env.reset()
    m, d = env.model, env.data
    qpos_log = []
    tip_run = tip = 0.0
    contact_run = max_close_contact = 0.0
    min_dist = 9.0
    hand_z_best = 0.0
    RAMP_S = 1.6

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        if t < 0.8:
            gcmd = OPEN_CMD
            closing = False
        elif t < 0.8 + RAMP_S:
            a = (t - 0.8) / RAMP_S
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            closing = True
        else:
            gcmd = CLOSE_CMD
            closing = True
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
        dd, _, zz = _min_hand_lever_dist(m, d)
        if dd < min_dist:
            min_dist, hand_z_best = dd, zz
        if _hand_lever_contact(m, d):
            contact_run += 1 / CTRL_HZ
            if closing:
                max_close_contact = max(max_close_contact, contact_run)
        else:
            contact_run = 0.0

    _render(env, qpos_log, ITER / "H01.mp4")
    tip_ok = tip >= 8.0
    grasp_ok = max_close_contact >= 1.0 - 1e-9
    row = {
        "tag": "H01",
        "plant": str(PLANT_F.relative_to(ROOT)),
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": tilt,
        "tip": tip,
        "tip_ok": tip_ok,
        "min_hand_lever_dist_m": min_dist,
        "hand_z_at_best": hand_z_best,
        "max_contact_s_while_closing": max_close_contact,
        "grasp_hold_ok": grasp_ok,
        "gripper_open_cmd": OPEN_CMD,
        "gripper_close_cmd": CLOSE_CMD,
        "gripper_proxy": "l/r_gripper_pos open=-1.2 → close=0",
        "open_then_close_cmd": True,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "fall": tip < 1.0,
        "arm_q": arm_q,
        "cam_dist_cmd": cam_dist,
        "mp4": "previews/ainex_walk/iterate/H01.mp4",
    }
    row["pass"] = bool(tip_ok and grasp_ok and not row["fall"] and not row["vision_in_walk_obs"])
    (ITER / "H01.json").write_text(json.dumps(row, indent=2, default=str))
    print(f"ROW H01 pass={row['pass']} tip={tip:.2f} contact_close={max_close_contact:.2f}s", flush=True)
    return row


def score_h02_h03(h00: dict) -> tuple[dict, dict]:
    """Push lever about Z via hand tangential motion while grasping; measure hinge angle."""
    print("[H] H02/H03 lever rotate…", flush=True)
    m0 = mj.MjModel.from_xml_path(str(PLANT_F))
    jid_h = _hinge_id(m0)
    if jid_h < 0:
        blocked = {
            "status": "BLOCKED_HW",
            "pass": False,
            "blocked_reason": "no door_lever_hinge",
            "assist": False,
            "freeze": False,
            "vision_in_walk_obs": False,
            "mp4": None,
        }
        h02 = {**blocked, "tag": "H02"}
        h03 = {**blocked, "tag": "H03"}
        (ITER / "H02.json").write_text(json.dumps(h02, indent=2))
        (ITER / "H03.json").write_text(json.dumps(h03, indent=2))
        return h02, h03

    arm_q = dict(h00["arm_q"])
    cam_dist = float(h00.get("cam_dist_cmd", 0.15))
    tilt = float(h00.get("head_tilt_cmd_deg", -19.0))
    # Reach joints for IK perturbation: sho/el of L arm
    reach_joints = ["l_sho_pitch", "l_sho_roll", "l_el_pitch", "l_el_yaw"]

    env = GateFEnv(
        gait_t=0.75, episode_s=12.0, stand_hold=12.0, ramp_t=0.0,
        head_tilt_deg=tilt, walk=False, body_x0=body_x_for_cam_dist(cam_dist),
    )
    env.reset()
    m, d = env.model, env.data
    # zero hinge
    d.qpos[m.jnt_qposadr[jid_h]] = 0.0
    sid_hand = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "l_hand_site")
    sid_lev = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "door_lever_site")
    jids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, j) for j in reach_joints]

    qpos_log = []
    tip_run = tip = 0.0
    max_abs_deg = 0.0
    hold_run = max_hold = 0.0
    peak_deg = 0.0

    # Phase times
    # 0-1.0 open+settle, 1.0-2.6 close, 2.6-8.0 push rotate, 8.0-12 hold
    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        if t < 1.0:
            gcmd = OPEN_CMD
            push = 0.0
        elif t < 2.6:
            a = (t - 1.0) / 1.6
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            push = 0.0
        elif t < 8.0:
            gcmd = CLOSE_CMD
            push = (t - 2.6) / 5.4  # 0→1
        else:
            gcmd = CLOSE_CMD
            push = 1.0

        # Tangential push: IK hand toward +Y around shaft (rotate hinge +)
        # Target = lever site + push * (0, 0.08, 0) in world
        mj.mj_forward(m, d)
        lev = d.site_xpos[sid_lev].copy()
        target = lev + np.array([0.0, 0.10 * push, 0.0])
        # one IK step on arm
        if push > 0 and sid_hand >= 0:
            jacp = np.zeros((3, m.nv))
            mj.mj_jacSite(m, d, jacp, None, sid_hand)
            p = d.site_xpos[sid_hand].copy()
            err = target - p
            cols = [m.jnt_dofadr[j] for j in jids]
            J = jacp[:, cols]
            lam = 1e-3
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
                for jid, ddq, jn in zip(jids, dq, reach_joints):
                    lo, hi = m.jnt_range[jid]
                    nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + 0.35 * ddq, lo, hi))
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

        ang = _hinge_angle_deg(m, d)
        max_abs_deg = max(max_abs_deg, abs(ang))
        if abs(ang) > abs(peak_deg):
            peak_deg = ang
        # H03: hold ≥15° continuously
        if abs(ang) >= 15.0 - 1e-6:
            hold_run += 1 / CTRL_HZ
            max_hold = max(max_hold, hold_run)
        else:
            hold_run = 0.0

    _render(env, qpos_log, ITER / "H02.mp4")
    # also copy as H03 video (same episode covers both)
    import shutil
    shutil.copy(ITER / "H02.mp4", ITER / "H03.mp4")

    tip_ok = tip >= 8.0
    rotate_ok = max_abs_deg >= 15.0 - 1e-6
    hold_ok = max_hold >= 1.0 - 1e-9

    h02 = {
        "tag": "H02",
        "plant": str(PLANT_F.relative_to(ROOT)),
        "status": "SCORED",
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": tilt,
        "tip": tip,
        "tip_ok": tip_ok,
        "lever_hinge_joint": "door_lever_hinge",
        "max_abs_hinge_deg": max_abs_deg,
        "peak_hinge_deg": peak_deg,
        "rotate_ok": rotate_ok,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "fall": tip < 1.0,
        "mp4": "previews/ainex_walk/iterate/H02.mp4",
        "hw_ref": "docs/GATE_H_HARDWARE_LEVER_HINGE.md",
    }
    h02["pass"] = bool(tip_ok and rotate_ok and not h02["fall"] and not h02["vision_in_walk_obs"])

    h03 = {
        "tag": "H03",
        "plant": str(PLANT_F.relative_to(ROOT)),
        "status": "SCORED",
        "head_joint": "head_tilt",
        "tip": tip,
        "tip_ok": tip_ok,
        "max_abs_hinge_deg": max_abs_deg,
        "max_hold_s_above_15deg": max_hold,
        "hold_ok": hold_ok,
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "fall": tip < 1.0,
        "mp4": "previews/ainex_walk/iterate/H03.mp4",
        "hw_ref": "docs/GATE_H_HARDWARE_LEVER_HINGE.md",
    }
    h03["pass"] = bool(tip_ok and hold_ok and rotate_ok and not h03["fall"])

    (ITER / "H02.json").write_text(json.dumps(h02, indent=2, default=str))
    (ITER / "H03.json").write_text(json.dumps(h03, indent=2, default=str))
    print(
        f"ROW H02 pass={h02['pass']} tip={tip:.2f} max|θ|={max_abs_deg:.2f}°",
        flush=True,
    )
    print(
        f"ROW H03 pass={h03['pass']} tip={tip:.2f} hold≥15°={max_hold:.2f}s",
        flush=True,
    )
    return h02, h03


def main():
    ITER.mkdir(parents=True, exist_ok=True)
    sha = _sha16(CKPT)
    assert sha == "9ffaa1a21b607bf6", sha
    print(f"[H] plant={PLANT_F.name} ckpt={sha} hinge={_hinge_id(mj.MjModel.from_xml_path(str(PLANT_F)))>=0}", flush=True)

    # stub pointer
    stub = ROOT / "docs" / "CONTROLS_POST_G_MUST_PROVE_STUB.md"
    stub.write_text(
        "# Controls post–Gate G stub — **superseded**\n\n"
        "**AI named Gate H** — `docs/GATE_H_AI_CRITERIA.md`. Score via `scripts/score_gate_h.py`.\n"
    )

    h00 = score_h00()
    h01 = score_h01(h00)
    h02, h03 = score_h02_h03(h00)
    rows = [h00, h01, h02, h03]

    if all(r.get("pass") for r in rows):
        verdict = "GATE_H_PASS"
    elif h00.get("pass") and h01.get("pass") and not h02.get("pass"):
        verdict = "GATE_H_PARTIAL_H00_H01_PASS_H02_FAIL"
    else:
        verdict = "GATE_H_FAIL"

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
        "hw_hinge_ref": "docs/GATE_H_HARDWARE_LEVER_HINGE.md",
        "rows": rows,
    }
    (ITER / "GATE_H_TABLE.json").write_text(json.dumps(table, indent=2, default=str))

    def st(r):
        return "PASS" if r.get("pass") else "FAIL"

    def fmt(r):
        md = r.get("min_hand_lever_dist_m")
        md_s = f"{md:.4f}" if isinstance(md, (int, float)) else "—"
        th = r.get("max_abs_hinge_deg")
        th_s = f"{th:.2f}" if isinstance(th, (int, float)) else "—"
        cs = r.get("max_contact_s_while_closing", r.get("max_hold_s_above_15deg"))
        cs_s = f"{cs:.2f}" if isinstance(cs, (int, float)) else "—"
        return (
            f"| {r['tag']} | {r.get('tip', 0):.2f} | {md_s} | {r.get('hand_z_at_best', float('nan')):.3f} | "
            f"{th_s} | {cs_s} | **{st(r)}** | `{r.get('mp4')}` |"
        )

    note = f'''# Gate H Controls — grasp + lever rotate (sim)

**When:** Mon 28 Sep 2026 Europe/London (BST)  
**Role:** Founding Controls  
**Criteria:** `docs/GATE_H_AI_CRITERIA.md`  
**Plant:** `{PLANT_F.relative_to(ROOT)}` (+ `door_lever_hinge` per `docs/GATE_H_HARDWARE_LEVER_HINGE.md`)  
**Gate E plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Ckpt:** sha16 **`{sha}`** · `head_tilt` · assist/freeze OFF · vision off · glfw  

## Verdict: **{verdict}**

| Tag | Result |
|-----|--------|
| H00 retain G02 reach | **{st(h00)}** |
| H01 grasp hold ≥1 s + close cmd | **{st(h01)}** |
| H02 lever rotate ≥15° | **{st(h02)}** |
| H03 hold ≥15° ≥1 s | **{st(h03)}** |

## Table

| Tag | tip | min_d | hand Z | max\\|θ\\|° | hold/contact_s | Status | mp4 |
|-----|-----|-------|--------|---------|----------------|--------|-----|
{chr(10).join(fmt(r) for r in rows)}

## Details

### H00
min_d **{h00.get("min_hand_lever_dist_m", float("nan")):.4f} m**, hand Z **{h00.get("hand_z_at_best", float("nan")):.3f} m**, tip={h00.get("tip")}.

### H01
Gripper open −1.2 → close 0; max contact while closing **{h01.get("max_contact_s_while_closing", 0):.2f} s**.

### H02 / H03
Hinge `door_lever_hinge` (Z). max |θ| **{h02.get("max_abs_hinge_deg", 0):.2f}°** (need ≥15). Hold ≥15° for **{h03.get("max_hold_s_above_15deg", 0):.2f} s**.

## Honesty
Soft-pass forbidden. No M145 edit. No vision-in-walk. No full door-open / latch / UK handle claim. No spend.

## Artifacts
- `scripts/score_gate_h.py`
- `GATE_H_TABLE.json`, this note
- `H00.mp4`…`H03.mp4`
'''
    (ITER / "GATE_H_CONTROLS_NOTE.md").write_text(note)
    print(f"=== {verdict} ===", flush=True)
    for r in rows:
        print(f"  {r['tag']}: {st(r)}", flush=True)


if __name__ == "__main__":
    main()
