"""Deterministic eval of open-loop or PPO policy → Gate E / T88 metrics."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts" / "learned_gate_e"))

from ss_step_ainex import analyze_ss  # noqa: E402
from score_auth_envelope import t88_ok, gate_e, shape_to_params, GRO01, CSF50  # noqa: E402
from env_ainex import AinexResidualEnv  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"


def _write_timeseries(env: AinexResidualEnv, path: Path) -> None:
    log = env.log
    n = len(log["t"])
    path.parent.mkdir(parents=True, exist_ok=True)
    # columns matching analyze_ss: 5,6=zL,zR; 21,22=cL,cR; 26=amp
    with path.open("w") as f:
        f.write("t," + ",".join(f"c{i}" for i in range(1, 30)) + "\n")
        for i in range(n):
            row = [0.0] * 30
            row[0] = log["t"][i]
            row[5] = log["zL"][i]
            row[6] = log["zR"][i]
            row[21] = log["cL"][i]
            row[22] = log["cR"][i]
            row[26] = log["amp"][i]
            f.write(",".join(f"{x:.6g}" for x in row) + "\n")


def _metrics_from_env(env: AinexResidualEnv, info: dict) -> dict:
    log = env.log
    # tip: continuous upright run; post-gait keeps the same run (match walk_gait)
    tip_run = 0.0
    tip_max = 0.0
    tip_post = 0.0
    gait_started = False
    for a, u in zip(log["amp"], log["up"]):
        if a > 0.15:
            gait_started = True
        if u > 0.5:
            tip_run += 1.0 / 50.0
            tip_max = max(tip_max, tip_run)
            if gait_started:
                tip_post = max(tip_post, tip_run)
        else:
            tip_run = 0.0

    # stance vx: match walk_gait (weight-bearing via lateral_com_target)
    import walk_gait_ainex as wg
    vxL, vxR = [], []
    stand_hold, gait_t = 0.80, float(getattr(env, "gait_t", 0.88))
    for i in range(len(log["t"])):
        if log["amp"][i] <= 0.2:
            continue
        phi = (max(0.0, log["t"][i] - stand_hold) / max(gait_t, 1e-6)) % 1.0
        lat = wg.lateral_com_target(phi)
        if lat > 0.0 and log["cL"][i] > 0.5:
            vxL.append(abs(log["vxL"][i]))
        if lat < 0.0 and log["cR"][i] > 0.5:
            vxR.append(abs(log["vxR"][i]))

    def mean_p95(xs):
        if not xs:
            return 9.0, 9.0
        a = np.asarray(xs, dtype=np.float64)
        return float(a.mean()), float(np.percentile(a, 95))

    mL, pL = mean_p95(vxL)
    mR, pR = mean_p95(vxR)
    skate = (mL > 0.08 or mR > 0.08 or pL > 0.18 or pR > 0.18)

    # hip corr (match walk_gait: corr(-qL, +qR) on hip pitch)
    hip_corr = 0.0
    if len(log.get("hipL", [])) > 10:
        fwd_l = -np.asarray(log["hipL"], dtype=np.float64)
        fwd_r = +np.asarray(log["hipR"], dtype=np.float64)
        # steady-state window
        i0 = int((0.80 + 1.40 + 0.2) * 50)
        if len(fwd_l) > i0 + 5:
            hip_corr = float(np.corrcoef(fwd_l[i0:], fwd_r[i0:])[0, 1])

    # foot lead cycles
    lead = 0
    if len(log["t"]) > 10:
        # approximate with x from feet not logged — use hip phase alternation proxy
        # better: use contact alternation flips / 2
        cL = np.asarray(log["cL"])
        cR = np.asarray(log["cR"])
        ssL = (cL < 0.5) & (cR > 0.5)
        ssR = (cR < 0.5) & (cL > 0.5)
        side = []
        i, n = 0, len(ssL)
        while i < n:
            if ssL[i]:
                j = i
                while j < n and ssL[j]:
                    j += 1
                side.append("L")
                i = j
            elif ssR[i]:
                j = i
                while j < n and ssR[j]:
                    j += 1
                side.append("R")
                i = j
            else:
                i += 1
        lead = sum(1 for a, b in zip(side, side[1:]) if a != b)

    dx = float(log["x"][-1] - log["x"][0]) if log["x"] else 0.0
    return {
        "tip_free_post_gait_max_s": float(tip_post),
        "dx_m": dx,
        "stance_foot_vx_mean_L": mL,
        "stance_foot_vx_mean_R": mR,
        "stance_foot_vx_p95_L": pL,
        "stance_foot_vx_p95_R": pR,
        "skate": bool(skate),
        "hip_q_corr_LR": float(hip_corr) if hip_corr == hip_corr else 0.0,
        "foot_lead_cycles": int(lead),
        "flipped": bool(info.get("fall")),
        "exploded": bool(info.get("fall")),
        "balance_assist": False,
        "stance_freeze": False,
    }


def run_episode(
    *,
    tag: str,
    gait_t: float,
    policy=None,
    shape: str = "GRO01",
    episode_s: float = 9.0,
    use_vik: bool = True,
) -> dict:
    env = AinexResidualEnv(
        shape=shape, gait_t=gait_t, episode_s=episode_s,
        stand_hold=0.80, ramp_t=1.40, use_vik=use_vik,
    )
    obs, _ = env.reset()
    done = False
    info: dict = {}
    while not done:
        if policy is None:
            action = np.zeros(12, dtype=np.float32)
        else:
            action, _ = policy.predict(obs, deterministic=True)
        obs, reward, term, trunc, info = env.step(action)
        done = term or trunc

    stats = _metrics_from_env(env, info)
    csv = OUT / f"{tag}_timeseries.csv"
    _write_timeseries(env, csv)
    ss = analyze_ss(csv, stats)
    bout_L = float(ss.get("ss_mean_bout_s_L") or 0)
    bout_R = float(ss.get("ss_mean_bout_s_R") or 0)
    tip = float(stats["tip_free_post_gait_max_s"])
    dx = float(stats["dx_m"])
    mL = float(stats["stance_foot_vx_mean_L"])
    mR = float(stats["stance_foot_vx_mean_R"])
    pL = float(stats["stance_foot_vx_p95_L"])
    pR = float(stats["stance_foot_vx_p95_R"])
    row = {
        "tag": tag,
        "T": gait_t,
        "shape": shape,
        "policy": "open_loop" if policy is None else "ppo",
        "tip": tip,
        "dx": dx,
        "bout_L": bout_L,
        "bout_R": bout_R,
        "bout_min": min(bout_L, bout_R) if (bout_L and bout_R) else 0.0,
        "stx_L": mL, "stx_R": mR,
        "stx_mean": 0.5 * (mL + mR),
        "p95_L": pL, "p95_R": pR,
        "stx_p95": 0.5 * (pL + pR),
        "hip_corr": float(stats["hip_q_corr_LR"]),
        "skate": bool(stats["skate"]),
        "foot_lead": int(stats["foot_lead_cycles"]),
        "n_bouts_L": int(ss.get("ss_bouts_L") or 0),
        "n_bouts_R": int(ss.get("ss_bouts_R") or 0),
        "peak_clear_L": float(ss.get("ss_peak_clear_m_L") or 0),
        "peak_clear_R": float(ss.get("ss_peak_clear_m_R") or 0),
        "flipped": bool(stats["flipped"]),
        "exploded": bool(stats["exploded"]),
        "auth_sat_rate": float(info.get("sat_rate", 0)),
        "assist": False, "freeze": False,
        "mp4": None,
        "ckpt": None,
    }
    row["t88_ok"] = t88_ok(row) if abs(gait_t - 0.88) < 1e-6 else None
    row["gate_e"] = gate_e(row) if gait_t <= 0.75 + 1e-9 else False
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{tag}.json").write_text(json.dumps(row, indent=2))
    (OUT / f"{tag}_ss.json").write_text(json.dumps({**row, **ss}, indent=2, default=str))
    (ITER / f"{tag}.json").write_text(json.dumps(row, indent=2))
    (ITER / f"{tag}_ss.json").write_text(json.dumps({**row, **ss}, indent=2, default=str))
    print(
        f"ROW {tag}: tip={row['tip']:.2f} dx={row['dx']:+.3f} bout={row['bout_min']:.3f} "
        f"stx={row['stx_mean']:.3f} p95={row['stx_p95']:.3f} skate={row['skate']} "
        f"hip={row['hip_corr']:.3f} ge={row['gate_e']} t88={row['t88_ok']} sat={row['auth_sat_rate']:.4f}",
        flush=True,
    )
    return row


def run_ol_via_walk_gait(tag: str, gait_t: float, shape_name: str = "GRO01", video: bool = False) -> dict:
    """Authoritative open-loop baseline via score_auth_envelope.run_one."""
    from score_auth_envelope import run_one
    shape = GRO01 if shape_name == "GRO01" else CSF50
    params = shape_to_params(shape, gait_t)
    # inject step_len
    if "step_len" in shape:
        # walk_gait uses -- step via hip; run_one doesn't pass step_len — OK
        pass
    m = run_one(params, tag=tag, auth_k=1.0, fric=None, duration=9.0, video=video)
    m["policy"] = "open_loop"
    m["shape"] = shape_name
    (OUT / f"{tag}_ss.json").write_text(json.dumps(m, indent=2, default=str))
    return m


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="RL_eval")
    ap.add_argument("--gait-t", type=float, default=0.75)
    ap.add_argument("--ckpt", type=str, default=None)
    ap.add_argument("--walk-gait", action="store_true")
    args = ap.parse_args()
    if args.walk_gait:
        run_ol_via_walk_gait(args.tag, args.gait_t)
    else:
        pol = None
        if args.ckpt:
            from stable_baselines3 import PPO
            pol = PPO.load(args.ckpt)
        run_episode(tag=args.tag, gait_t=args.gait_t, policy=pol)
