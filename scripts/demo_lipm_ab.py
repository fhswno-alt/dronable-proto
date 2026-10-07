#!/usr/bin/env python3
"""A/B the LIPM schedule against the frozen CPG on the same CommandBus.

LIPM00 is the existing open-loop gait (unchanged, including its root wrench).
LIPM01 is the vendor 2 cm clearance. LIPM02 commands 4 cm and is not the bar.
LIPM03 is the 2 cm clearance plus a contralateral arm swing.

Soft-pass is off. The plant file is not edited. A row that stays upright and
clears about 2 cm still Prefer FAILs if the body does not step forward.
"""
from __future__ import annotations

import json

import mujoco as mj
import numpy as np

import lipm_gait as lg
import steer_walk as sw

FORWARD: tuple[sw.DemoSegment, ...] = (
    sw.DemoSegment(0.4, "stand", 0.0, 0.0, "stand"),
    sw.DemoSegment(8.4, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
)

ScoreRow = dict[str, float | int | bool | str]


def _contact_normal(session: sw.SteerSession, gid: int) -> float:
    total = 0.0
    for i in range(session.data.ncon):
        contact = session.data.contact[i]
        if int(contact.geom1) != gid and int(contact.geom2) != gid:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(session.model, session.data, i, force)
        total += float(force[0])
    return total


def _sample_cpg(duration: float) -> ScoreRow:
    """LIPM00. Swing sole is the lighter foot while the other is planted."""
    session = sw.SteerSession(video=False, lipm=None)
    driver = sw.ScriptedDriver(FORWARD)
    n_ctrl = int(round(duration / sw.CTRL_DT))
    soles: list[float] = []
    slips: list[float] = []
    gid = {"L": session.gid_lfoot, "R": session.gid_rfoot}
    bid = {"L": session.bid_lf, "R": session.bid_rf}
    prev = {
        "L": np.array(session.data.geom_xpos[gid["L"]][:2], dtype=np.float64),
        "R": np.array(session.data.geom_xpos[gid["R"]][:2], dtype=np.float64),
    }
    sat_ticks = 0
    sho: list[float] = []
    for _ in range(n_ctrl):
        now = float(session.data.time)
        driver.publish(session.bus, now)
        session.step()
        if session.samples and session.samples[-1].leg_tau >= 0.98 * 2.1:
            sat_ticks += 1
        fn: dict[str, float] = {}
        sole: dict[str, float] = {}
        for side in ("L", "R"):
            fn[side] = _contact_normal(session, gid[side])
            sole[side] = lg.sole_clearance(
                session.model, session.data, bid[side], gid[side],
            )
            xy = np.array(session.data.geom_xpos[gid[side]][:2], dtype=np.float64)
            if fn[side] > 8.0:
                slips.append(float(np.linalg.norm(xy - prev[side]) / sw.CTRL_DT))
            prev[side] = xy
        if fn["L"] <= 8.0 < fn["R"]:
            soles.append(sole["L"])
        elif fn["R"] <= 8.0 < fn["L"]:
            soles.append(sole["R"])
        sho.append(_sho(session))
    session.assert_plant_unchanged()
    sole_arr = np.asarray(soles, dtype=np.float64) if soles else np.zeros(1)
    return {
        "name": "LIPM00",
        "schedule": "cpg",
        "clear_cmd_m": 0.0,
        "arms": True,
        "n_swing_samples": len(soles),
        "sole_median_m": float(np.median(sole_arr)) if soles else 0.0,
        "sole_p90_m": float(np.percentile(sole_arr, 90)) if soles else 0.0,
        "stance_slip_m_s": float(np.median(np.asarray(slips, dtype=np.float64))) if slips else 0.0,
        "sat_rate": sat_ticks / max(1, n_ctrl),
        "min_up_z": float(session.min_up_z),
        "dx_m": float(session.data.qpos[0]),
        "dyaw_rad": float(session.yaw()),
        "fault": bool(session.bus.fault),
        "fault_reason": session.bus.fault_reason,
        "arm_ptp_rad": (max(sho) - min(sho)) if len(sho) > 1 else 0.0,
    }


def _sho(session: sw.SteerSession) -> float:
    jid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, "l_sho_pitch")
    return float(session.data.qpos[session.model.jnt_qposadr[jid]])


def _sample_lipm(cfg: lg.LipmConfig, duration: float) -> ScoreRow:
    session = sw.SteerSession(video=False, lipm=cfg)
    driver = sw.ScriptedDriver(FORWARD)
    n_ctrl = int(round(duration / sw.CTRL_DT))
    for _ in range(n_ctrl):
        now = float(session.data.time)
        driver.publish(session.bus, now)
        session.step()
    assert session.lipm is not None
    session.assert_plant_unchanged()
    row: ScoreRow = {
        "schedule": "lipm",
        "dx_m": float(session.data.qpos[0]),
        "dyaw_rad": float(session.yaw()),
        "fault": bool(session.bus.fault),
        "fault_reason": session.bus.fault_reason,
    }
    for key, val in session.lipm.score().items():
        row[key] = val
    row["min_up_z"] = float(session.min_up_z)
    return row


def main() -> None:
    duration = FORWARD[-1].t_end
    rows: list[ScoreRow] = [
        _sample_cpg(duration),
        _sample_lipm(lg.LipmConfig("LIPM01", clear_m=0.020, arms=False), duration),
        _sample_lipm(lg.LipmConfig("LIPM02", clear_m=0.040, arms=False), duration),
        _sample_lipm(lg.LipmConfig("LIPM03", clear_m=0.020, arms=True), duration),
    ]
    payload: dict[str, object] = {
        "plant_md5": sw.PLANT_MD5,
        "plant_file_edited": False,
        "soft_pass": False,
        "bar": (
            "Vendor GaitManager look: sole about 2 cm, step at most 2 cm, "
            "period 300–600 ms, forward Δx, no skate, upright, arms not frozen. "
            "A shuffle or an in-place march is a Prefer FAIL. 5 cm is not the bar."
        ),
        "hx_running_nm": 2.45,
        "leg_forcerange_nm": 2.45,
        "toe_moment_nm": 2.24,
        "window_s": duration,
        "vx": sw.VX_FWD_CAP,
        "rows": rows,
    }
    out = sw.PREVIEWS / "lipm_ab.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[lipm] wrote {out}")
    for row in rows:
        print(
            f"[lipm] {row['name']} sole_p90={float(row['sole_p90_m']):.4f} "
            f"med={float(row['sole_median_m']):.4f} "
            f"slip={float(row['stance_slip_m_s']):.4f} "
            f"sat={float(row['sat_rate']):.3f} "
            f"up={float(row['min_up_z']):.3f} "
            f"dx={float(row['dx_m']):+.4f} "
            f"unload={float(row.get('rear_unload_frac', -1)):.2f} "
            f"fault={row['fault']}"
        )


if __name__ == "__main__":
    main()
