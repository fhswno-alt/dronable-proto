#!/usr/bin/env python3
"""T5-D1 OUTER-PRIMARY+CLEAR-CORRECTOR — Prefer FAIL cospec.

Cospec: docs/GATE_Q_AI_COSPEC_T5D.md (AI no-veto)
Family: reverse-native outer PRIMARY (CSF+ALIP; Placo N/A) owns F,T + footholds;
        residual = clear mid-swing + late-swing TDVM/SLR corrector only.
Bootstrap corrector warm-start: T5C_s43_BC sha16 5e0c0726953840ed (log which).
Outer is NEW — do not just retrain T5-C.
Budget: ≤4h wall OR ≤2e6 steps (first hit). Soft-pass NEVER.
Eval: dual-ckpt approach=E7lock · retreat=T5D1 via GATE_Q_T5D1_CKPT.
New sha16 ONLY after Prefer FAIL fair set honestly beats E7lock.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "scripts" / "learned_gate_e")]
os.environ.setdefault("MUJOCO_GL", "glfw")

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv

from env_ainex import AinexT5D1OuterPrimaryEnv, make_t5d1_env  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"
OUT.mkdir(parents=True, exist_ok=True)
PROGRESS = ITER / "GATE_Q_T5D1_PROGRESS.md"
E7LOCK = OUT / "ppo_gate_e_locked.zip"
E7BEST = OUT / "ppo_gate_e_best.zip"
E7SHA16 = "9ffaa1a21b607bf6"
T5C_S43_BC = OUT / "ppo_t5c_seed43_bc.zip"
T5C_S43_BC_SHA = "5e0c0726953840ed"
T5C_S47_03 = OUT / "ppo_t5c_best_T5C_s47_03_near_miss.zip"
T5B_02 = OUT / "ppo_t5b_best_T5B_02_near_miss.zip"
PY = str(ROOT / ".venv" / "bin" / "python")


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _bootstrap() -> tuple[Path, str]:
    """Corrector warm-start only — outer is NEW. Log which."""
    if T5C_S43_BC.exists():
        sha = _sha16(T5C_S43_BC)
        assert sha == T5C_S43_BC_SHA, f"T5C_s43_BC sha mismatch {sha}≠{T5C_S43_BC_SHA}"
        print(f"[T5D1] bootstrap_corrector=T5C_s43_BC {T5C_S43_BC.name} sha16={sha}", flush=True)
        return T5C_S43_BC, f"T5C_s43_BC:{sha}"
    if T5C_S47_03.exists():
        sha = _sha16(T5C_S47_03)
        print(f"[T5D1] bootstrap_corrector=T5C_s47_03 {T5C_S47_03.name} sha16={sha}", flush=True)
        return T5C_S47_03, f"T5C_s47_03:{sha}"
    if T5B_02.exists():
        sha = _sha16(T5B_02)
        print(f"[T5D1] bootstrap_corrector=T5B_02 {T5B_02.name} sha16={sha}", flush=True)
        return T5B_02, f"T5B_02:{sha}"
    if E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16:
        print(f"[T5D1] bootstrap_corrector=E7lock sha16={E7SHA16}", flush=True)
        return E7LOCK, f"E7lock:{E7SHA16}"
    raise FileNotFoundError("No T5C_s43_BC / T5C_s47_03 / T5B_02 / E7lock bootstrap")


def _write_progress(lines: list[str]) -> None:
    hdr = (
        "# Gate Q T5-D1 OUTER-PRIMARY+CLEAR-CORRECTOR — live progress\n\n"
        f"**Cospec:** `docs/GATE_Q_AI_COSPEC_T5D.md` · Soft-pass **off** · Bars KEPT\n"
        f"**Family:** OUTER=CSF+ALIP primary (Placo N/A) · residual=clear corrector only\n"
        f"**Bootstrap corrector:** T5C_s43_BC `{T5C_S43_BC_SHA}` if present · outer is NEW\n"
        f"**Budget:** ≤4h wall OR ≤2e6 steps (first hit) · Early stop on ret_ok both\n"
        f"**Dual-ckpt:** approach=E7lock · retreat=`GATE_Q_T5D1_CKPT`\n"
        f"**Planted:** cancel×0.70 + vel-oppose · zero planted soft-XY\n"
        f"**Rollback:** E7lock `{E7SHA16}` until Prefer-FAIL beat\n\n"
    )
    body = "\n".join(lines) + "\n"
    if not PROGRESS.exists():
        PROGRESS.write_text(hdr + body)
        return
    prev = PROGRESS.read_text()
    if not prev.startswith("# Gate Q T5-D1"):
        PROGRESS.write_text(hdr + body + ("\n" + prev if prev else ""))
    else:
        PROGRESS.write_text(prev.rstrip() + "\n\n" + body)


def _parse_probe(log_text: str) -> dict:
    out: dict = {
        "apps": [], "ret_cf": [], "skate": [], "tip_ret": [], "ret_ok": [],
        "mid": [], "eps": [], "cam_ret": [], "raw": {},
        "outer_F": [], "outer_T": [], "outer_n_replan": [], "outer_logged": False,
    }
    for m in re.finditer(
        r"RET_OK_BREAKDOWN cyc(\d+): ret_ok=(\w+).*?clear_frac_ok=\w+\(cf=([0-9.]+).*?"
        r"skate_ok=\w+\(mean/p95=([0-9.]+)/([0-9.]+)",
        log_text,
    ):
        out["ret_ok"].append(m.group(2) == "True")
        out["ret_cf"].append(float(m.group(3)))
        out["skate"].append((float(m.group(4)), float(m.group(5))))
    for m in re.finditer(r"PRE_GAP cyc(\d+): planted_middle_s=([0-9.]+)", log_text):
        out["mid"].append(float(m.group(2)))
    for m in re.finditer(
        r"cyc(\d+): Δstep=.*?clear_frac=([0-9.]+).*?tip_app=([0-9.]+).*?"
        r"clear_frac_ret=([0-9.]+).*?skate_ret=([0-9.]+)/([0-9.]+).*?tip_ret=([0-9.]+).*?"
        r"cam_ret=([0-9.]+).*?ret_ok=(\w+)",
        log_text,
    ):
        out["apps"].append(float(m.group(2)))
        out.setdefault("tip_app", []).append(float(m.group(3)))
        if len(out["ret_cf"]) < int(m.group(1)) + 1:
            out["ret_cf"].append(float(m.group(4)))
            out["skate"].append((float(m.group(5)), float(m.group(6))))
            out["ret_ok"].append(m.group(9) == "True")
        out["tip_ret"].append(float(m.group(7)))
        out["cam_ret"].append(float(m.group(8)))
    for m in re.finditer(r"cam@preSS=([0-9.]+).*?dxc_pre=([0-9.]+)", log_text):
        out.setdefault("cam_preSS", []).append(float(m.group(1)))
        out.setdefault("dxc_pre", []).append(float(m.group(2)))
    steals = re.findall(r"steal=(True|False)", log_text)
    out["eps"] = [s == "True" for s in steals]
    maxgs = [float(x) for x in re.findall(r"maxG=([0-9.]+)", log_text)]
    out["maxG"] = maxgs
    out["eps_steal"] = any(out["eps"]) or any(g > 0.02 + 1e-12 for g in maxgs)
    # Outer F/T/footholds logs (Prefer FAIL if missing / idle)
    for m in re.finditer(
        r"T5D1_OUTER(?:_FT)? cyc\d+:.*?F=([0-9.\-eE]+).*?(?:T=|T=alip_replan).*?n_replan=(\d+)",
        log_text,
    ):
        out["outer_F"].append(float(m.group(1)))
        out["outer_n_replan"].append(int(m.group(2)))
        out["outer_logged"] = True
    for m in re.finditer(
        r"T5D1_OUTER cyc\d+: fp_prior=([0-9.\-eE]+).*?du_fp=([0-9.\-eE]+).*?n_replan=(\d+)",
        log_text,
    ):
        out["outer_F"].append(float(m.group(1)))
        out.setdefault("outer_du", []).append(float(m.group(2)))
        out["outer_n_replan"].append(int(m.group(3)))
        out["outer_logged"] = True
    # Also accept ALIP/T5B-style tags as outer F proxy when T5D1 path active
    if not out["outer_logged"]:
        for m in re.finditer(
            r"\[Q\] (?:T5D1_OUTER|T5C_TEACHER|T5B_MODELBASE) cyc\d+: "
            r"fp_prior=([0-9.\-eE]+).*?du_fp=([0-9.\-eE]+).*?n_replan=(\d+)",
            log_text,
        ):
            out["outer_F"].append(float(m.group(1)))
            out.setdefault("outer_du", []).append(float(m.group(2)))
            out["outer_n_replan"].append(int(m.group(3)))
            out["outer_logged"] = True
    out["outer_idle"] = (
        (not out["outer_logged"])
        or (sum(out["outer_n_replan"]) < 2)
        or (all(abs(f) < 1e-5 for f in out["outer_F"]) if out["outer_F"] else True)
    )
    m = re.search(r"n_pass=(\d+)/(\d+)", log_text)
    out["n_pass"] = (int(m.group(1)), int(m.group(2))) if m else None
    out["ret_ok_both"] = bool(out["ret_ok"]) and all(out["ret_ok"][:2]) and len(out["ret_ok"]) >= 2
    return out


def _bars_beat_e7lock(m: dict) -> bool:
    """Honest beat: ret_ok both + apps held + plant-cam ≤ε + outer logged. Soft-pass NEVER."""
    if not m.get("ret_ok_both"):
        return False
    if m.get("outer_idle"):
        return False  # Prefer FAIL: outer idle / residual owns polarity
    if len(m.get("apps", [])) < 2:
        return False
    if m["apps"][0] < 0.548 - 0.02 or m["apps"][1] < 0.630 - 0.02:
        return False
    for cf in m.get("ret_cf", [])[:2]:
        if cf < 0.55 - 1e-9:
            return False
    for sk in m.get("skate", [])[:2]:
        if sk[0] > 0.08 + 1e-9 or sk[1] > 0.18 + 1e-9:
            return False
    for tip in m.get("tip_ret", [])[:2]:
        if tip < 8.0 - 1e-9:
            return False
    if m.get("eps_steal"):
        return False
    return True


def run_gate_q_probe(tag: str, t5d1_ckpt: Path | None, wall_t0: float) -> dict:
    env = os.environ.copy()
    env["GATE_Q_SKIP_VIDEO"] = "1"
    env["MUJOCO_GL"] = "glfw"
    for k in (
        "GATE_Q_RET_CSF50", "GATE_Q_RET_ALIP_TVR", "GATE_Q_RET_REV_PHASE",
        "GATE_Q_RET_CLEAR_BURST", "GATE_Q_RET_BURST_PLANT_CAP",
        "GATE_Q_RET_MAX_PLANT_DWELL", "GATE_Q_RET_SWING_PLACE_M",
    ):
        env[k] = "0"
    for k in ("GATE_Q_T4_CKPT", "GATE_Q_T5_CKPT", "GATE_Q_T5B_CKPT", "GATE_Q_T5C_CKPT"):
        env.pop(k, None)
    if t5d1_ckpt is not None:
        env["GATE_Q_T5D1_CKPT"] = str(t5d1_ckpt.resolve())
        env["GATE_Q_T5D1_VX_CMD"] = "-0.08"
        env["GATE_Q_T5D1_OUTER_PRIMARY"] = "1"
        env["GATE_Q_T5B_MODELBASE"] = "1"
    else:
        env.pop("GATE_Q_T5D1_CKPT", None)
        env.pop("GATE_Q_T5D1_OUTER_PRIMARY", None)
    log_path = ITER / f"GATE_Q_{tag}_PROBE.log"
    t0 = time.time()
    print(f"[T5D1-eval] {tag} ckpt={t5d1_ckpt} …", flush=True)
    proc = subprocess.run(
        [PY, str(ROOT / "scripts" / "score_gate_q.py")],
        cwd=str(ROOT), env=env, capture_output=True, text=True, timeout=900,
    )
    text = (proc.stdout or "") + "\n" + (proc.stderr or "")
    log_path.write_text(text)
    metrics = _parse_probe(text)
    metrics["tag"] = tag
    metrics["exit"] = proc.returncode
    metrics["wall_s_probe"] = time.time() - t0
    metrics["wall_s_train"] = time.time() - wall_t0
    metrics["ckpt"] = str(t5d1_ckpt) if t5d1_ckpt else "E7lock"
    metrics["ckpt_sha16"] = _sha16(t5d1_ckpt) if t5d1_ckpt and t5d1_ckpt.exists() else E7SHA16
    metrics["outer_src"] = AinexT5D1OuterPrimaryEnv.OUTER_SRC
    apps = metrics.get("apps") or []
    rcf = metrics.get("ret_cf") or []
    sk = metrics.get("skate") or []
    tip = metrics.get("tip_ret") or []
    mid = metrics.get("mid") or []
    line = (
        f"### {tag} · wall={metrics['wall_s_train']:.0f}s · sha16=`{metrics['ckpt_sha16']}`\n"
        f"- apps={apps[:2]} · ret_cf={rcf[:2]} · skate={sk[:2]} · tip_ret={tip[:2]}\n"
        f"- mid={mid[:2]} · ret_ok={metrics.get('ret_ok')} · ret_ok_both={metrics.get('ret_ok_both')}\n"
        f"- eps_steal={metrics.get('eps_steal')} maxG={metrics.get('maxG', [])[:4]}\n"
        f"- outer_src=`{metrics['outer_src']}` outer_logged={metrics.get('outer_logged')} "
        f"outer_idle={metrics.get('outer_idle')} outer_F={metrics.get('outer_F', [])[:4]} "
        f"n_replan={metrics.get('outer_n_replan', [])[:4]}\n"
        f"- probe_wall={metrics['wall_s_probe']:.0f}s · exit={proc.returncode} · log=`{log_path.name}`\n"
    )
    _write_progress([line])
    (OUT / f"T5D1_{tag}_metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    print(
        f"[T5D1-eval] {tag} apps={apps[:2]} rcf={rcf[:2]} sk={sk[:2]} "
        f"ret_ok={metrics.get('ret_ok')} both={metrics.get('ret_ok_both')} "
        f"outer_idle={metrics.get('outer_idle')} eps_steal={metrics.get('eps_steal')}",
        flush=True,
    )
    return metrics


def _env_kwargs(seed: int) -> dict:
    return dict(
        gait_t=0.75, seed=seed,
        episode_s=3.5, stand_hold=0.60, ramp_t=0.80,
        action_scale=0.028,  # corrector band
        use_vik=True, use_plant=True, use_cp=True,
        vx_cmd=-0.07, target_dx=0.08, rev_phase=True,
        clear_only_residual=True, vx_cmd_rand=True,
        vx_lo=-0.05, vx_hi=-0.03,
        use_slr=True, use_ank=True, use_dxb=True, dxb_cap=0.035,
        use_alip=True, use_dcm=True, use_ank_nmpc=True,
        n_replan=3, alip_dx_cap=0.035, alip_smooth=0.35,
        outer_dx_cap=0.035, outer_track_gain=0.14, corrector_scale=0.028,
        residual_primary_forbid=True,
    )


class T5D1Callback(BaseCallback):
    def __init__(self, total_steps: int, eval_every: int, wall_s: float, wall_t0: float):
        super().__init__()
        self.total_steps = total_steps
        self.eval_every = eval_every
        self.wall_s = wall_s
        self.t0 = wall_t0
        self.last_eval = 0
        self.history: list[dict] = []
        self.best: dict | None = None
        self.stop_ret_ok = False
        self.probe_idx = 0

    def _curriculum(self) -> None:
        frac = min(1.0, self.num_timesteps / max(1, int(0.7 * self.total_steps)))
        target = 0.06 + 0.18 * frac
        ep_s = 3.0 + 2.5 * frac
        vx_lo = -0.05 - 0.04 * frac
        vx_hi = -0.03 - 0.02 * frac
        dxb = 0.028 + 0.012 * frac
        for e in self.training_env.envs:
            env = e
            while hasattr(env, "env"):
                env = env.env
            if hasattr(env, "set_curriculum"):
                env.set_curriculum(target_dx=target, episode_s=ep_s, vx_lo=vx_lo, vx_hi=vx_hi)
            if hasattr(env, "dxb_cap"):
                env.dxb_cap = float(dxb)
            if hasattr(env, "outer_dx_cap"):
                env.outer_dx_cap = float(dxb)
            # Keep corrector scale demoted
            if hasattr(env, "action_scale") and env.action_scale > 0.04:
                env.action_scale = float(getattr(env, "corrector_scale", 0.028))

    def _on_step(self) -> bool:
        self._curriculum()
        if time.time() - self.t0 >= self.wall_s:
            print(f"[T5D1] wall limit {self.wall_s}s at steps={self.num_timesteps}", flush=True)
            return False
        if self.num_timesteps - self.last_eval >= self.eval_every and self.num_timesteps > 0:
            self.last_eval = self.num_timesteps
            self.probe_idx += 1
            tag = f"T5D1_{self.probe_idx:02d}"
            ckpt = OUT / f"ppo_t5d1_step{self.num_timesteps}"
            self.model.save(str(ckpt))
            zip_path = Path(str(ckpt) + ".zip")
            _write_progress([
                f"### train land · steps={self.num_timesteps} · "
                f"wall={time.time()-self.t0:.0f}s · ckpt=`{zip_path.name}` · Prefer FAIL `{tag}`\n"
            ])
            metrics = run_gate_q_probe(tag, zip_path, self.t0)
            metrics["steps"] = self.num_timesteps
            self.history.append(metrics)
            (OUT / "t5d1_train_history.json").write_text(
                json.dumps(self.history, indent=2, default=str))
            score = self._rank(metrics)
            if self.best is None or score > self._rank(self.best):
                self.best = metrics
                self.model.save(str(OUT / "ppo_t5d1_best"))
                _write_progress([
                    f"**best-so-far** → `{tag}` rank={score:.3f} "
                    f"sha16=`{metrics['ckpt_sha16']}` skate={metrics.get('skate')} "
                    f"rcf={metrics.get('ret_cf')} outer_idle={metrics.get('outer_idle')} "
                    f"eps_steal={metrics.get('eps_steal')}\n"
                ])
            if metrics.get("ret_ok_both") and _bars_beat_e7lock(metrics):
                print("[T5D1] ret_ok both under bars+ε+outer — early stop", flush=True)
                self.stop_ret_ok = True
                self.model.save(str(OUT / "ppo_t5d1_retok"))
                return False
        return True

    @staticmethod
    def _rank(m: dict) -> float:
        rcf = m.get("ret_cf") or [0.0, 0.0]
        sk = m.get("skate") or [(1.0, 1.0), (1.0, 1.0)]
        apps = m.get("apps") or [0.0, 0.0]
        tip = m.get("tip_ret") or [0.0, 0.0]
        sk_pen = 0.0
        sk_under = 0.0
        for i, pair in enumerate(sk[:2]):
            a, b = pair
            w_mean = 12.0 if i == 0 else 14.0
            w_p95 = 20.0 if i == 0 else 16.0
            sk_pen += max(0.0, a - 0.08) * w_mean + max(0.0, b - 0.18) * w_p95
            if a <= 0.08 + 1e-9 and b <= 0.18 + 1e-9:
                sk_under += 8.0 if i == 0 else 6.0
        cf_s = 0.0
        for c in rcf[:2]:
            if c >= 0.55 - 1e-9:
                cf_s += 1.0 + 0.3 * min(c, 1.0)
            else:
                cf_s -= 5.0 * (0.55 - c)
        tip_s = sum(1.0 for t in tip[:2] if t >= 8.0)
        apps_s = 0.0
        if len(apps) >= 2:
            apps_ok = (apps[0] >= 0.548 - 0.02) and (apps[1] >= 0.630 - 0.02)
            apps_s = 2.0 if apps_ok else -8.0
        eps_pen = 6.0 if m.get("eps_steal") else 0.0
        outer_pen = 8.0 if m.get("outer_idle") else 0.0
        outer_bonus = 3.0 if m.get("outer_logged") and not m.get("outer_idle") else 0.0
        ret_b = 10.0 if m.get("ret_ok_both") else 0.0
        return sk_under + cf_s + tip_s + apps_s + ret_b + outer_bonus - sk_pen - eps_pen - outer_pen


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--wall-hours", type=float, default=4.0)
    ap.add_argument("--eval-every", type=int, default=150_000)
    ap.add_argument("--n-envs", type=int, default=2)
    ap.add_argument("--skip-r0", action="store_true")
    ap.add_argument("--lr", type=float, default=7e-5)
    ap.add_argument("--seed", type=int, default=43)
    args = ap.parse_args()

    wall_s = args.wall_hours * 3600.0
    boot, boot_label = _bootstrap()
    assert E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16, "E7lock rollback must stay present"

    _write_progress([
        f"## Start · {time.strftime('%Y-%m-%d %H:%M:%S %Z')}\n",
        f"- bootstrap_corrector=`{boot.name}` label=`{boot_label}` sha16=`{_sha16(boot)}`\n",
        f"- family=**T5-D1 OUTER-PRIMARY+CLEAR-CORRECTOR** · OUTER=`{AinexT5D1OuterPrimaryEnv.OUTER_SRC}`\n",
        f"- residual=corrector-only scale=0.028 · clear≥2cm · planted cancel×0.70 · soft-pass NEVER\n",
        f"- steps≤{args.steps} · wall≤{args.wall_hours}h · seed={args.seed}\n",
        f"- NOT reopening R*/S1–S3/T5-A/B/C twins · no plant invent · no H2\n",
        f"- companion md5 `59cc408eda07037a58f92ad27da045d6` · walk plant `fc94709c84f5598d4474ecfc4bb41fdc`\n",
        f"- dual-ckpt approach=E7lock · retreat=`GATE_Q_T5D1_CKPT`\n",
        f"- eval_every={args.eval_every} n_envs={args.n_envs} lr={args.lr}\n",
        f"- Prefer FAIL if outer idle and residual owns polarity\n",
    ])

    t0 = time.time()
    if not args.skip_r0:
        print("[T5D1] T5D1_R0 = E7lock baseline (flags OFF)", flush=True)
        r0 = run_gate_q_probe("T5D1_R0", None, t0)
        r0_ok = (
            len(r0.get("apps", [])) >= 2
            and abs(r0["apps"][0] - 0.548) < 0.02
            and abs(r0["apps"][1] - 0.630) < 0.05
        )
        _write_progress([
            f"**T5D1_R0 ≡ E7lock:** {'YES' if r0_ok else 'CHECK'} apps={r0.get('apps')} "
            f"rcf={r0.get('ret_cf')} skate={r0.get('skate')}\n"
        ])
        # Note vs T5C_s43_BC near-miss through T5D1 dual-ckpt path
        if T5C_S43_BC.exists():
            print("[T5D1] T5D1_T5C_s43_BC note (via GATE_Q_T5D1_CKPT)", flush=True)
            note = run_gate_q_probe("T5D1_T5C_s43_BC", T5C_S43_BC, t0)
            _write_progress([
                f"**T5D1_T5C_s43_BC note:** apps={note.get('apps')} rcf={note.get('ret_cf')} "
                f"skate={note.get('skate')} ret_ok={note.get('ret_ok')} "
                f"outer_idle={note.get('outer_idle')} eps_steal={note.get('eps_steal')}\n"
            ])

    def make(rank):
        kw = _env_kwargs(args.seed)
        return make_t5d1_env(rank=rank, **kw)

    print(
        f"[T5D1] residual PPO corrector start from {boot.name} "
        f"OUTER={AinexT5D1OuterPrimaryEnv.OUTER_SRC} lr={args.lr} "
        f"steps≤{args.steps} wall≤{wall_s}s",
        flush=True,
    )
    env = DummyVecEnv([make(i) for i in range(args.n_envs)])
    model = PPO.load(str(boot), env=env, device="cpu")
    model.learning_rate = args.lr

    # Enforce corrector scale on envs at start
    for e in env.envs:
        ee = e
        while hasattr(ee, "env"):
            ee = ee.env
        if hasattr(ee, "action_scale"):
            ee.action_scale = 0.028
        if hasattr(ee, "corrector_scale"):
            ee.corrector_scale = 0.028

    cb = T5D1Callback(args.steps, args.eval_every, wall_s, t0)
    model.learn(
        total_timesteps=args.steps,
        callback=cb,
        progress_bar=False,
        reset_num_timesteps=True,
    )
    elapsed = time.time() - t0
    final = OUT / "ppo_t5d1_final"
    model.save(str(final))
    final_zip = Path(str(final) + ".zip")

    fin_m = run_gate_q_probe("T5D1_FINAL", final_zip, t0)
    fin_m["steps"] = int(model.num_timesteps)
    cb.history.append(fin_m)
    (OUT / "t5d1_train_history.json").write_text(
        json.dumps(cb.history, indent=2, default=str))

    best = cb.best or fin_m
    if best.get("ckpt") and best["ckpt"] != "E7lock":
        src = Path(best["ckpt"])
        if src.exists():
            shutil.copy2(src, OUT / "ppo_t5d1_best.zip")
            tag = best.get("tag", "near_miss")
            shutil.copy2(src, OUT / f"ppo_t5d1_best_{tag}_near_miss.zip")

    beat = _bars_beat_e7lock(best) if best else False
    installed = False
    installed_sha = None
    # Cospec: do NOT install new sha16 on Prefer FAIL — keep E7lock
    if beat and best.get("ckpt") and best["ckpt"] != "E7lock":
        src = Path(best["ckpt"])
        if src.exists():
            shutil.copy2(src, E7BEST)
            new_sha = _sha16(E7BEST)
            sgq = ROOT / "scripts" / "score_gate_q.py"
            txt = sgq.read_text()
            txt2 = re.sub(
                r'CKPT_SHA16 = "9ffaa1a21b607bf6"',
                f'CKPT_SHA16 = "{new_sha}"',
                txt, count=1,
            )
            if txt2 != txt:
                sgq.write_text(txt2)
            installed = True
            installed_sha = new_sha
            _write_progress([
                f"## HONEST BEAT — installed sha16=`{new_sha}` into ppo_gate_e_best + CKPT_SHA16\n"
                f"- rollback locked remains `{E7SHA16}` at `ppo_gate_e_locked.zip`\n"
            ])

    if beat:
        disposition = "ret_ok both — honest beat"
        outcome = "RET_OK_BOTH"
    else:
        sk = best.get("skate") or []
        rcf = best.get("ret_cf") or []
        near = (
            bool(rcf) and min(rcf[:2] or [0]) >= 0.50
            and any((s[0] <= 0.09 and s[1] <= 0.20) for s in sk[:2])
        )
        if best.get("outer_idle"):
            disposition = "Prefer FAIL — outer idle / residual polarity risk"
        elif near:
            disposition = "near-miss Prefer FAIL"
        else:
            disposition = "Prefer FAIL train wall"
        outcome = "PREFER_FAIL_SCORED"

    done = {
        "outcome": outcome,
        "disposition": disposition,
        "family": "T5-D1 OUTER-PRIMARY+CLEAR-CORRECTOR",
        "outer_src": AinexT5D1OuterPrimaryEnv.OUTER_SRC,
        "bootstrap_corrector": str(boot),
        "bootstrap_label": boot_label,
        "bootstrap_sha16": _sha16(boot),
        "opts": ["OUTER_CSF+ALIP", "CLEAR_CORRECTOR", "TDVM/SLR", "cancel×0.70", "FOOT_LIFT≥2cm"],
        "wall_s": elapsed,
        "steps": int(model.num_timesteps),
        "steps_cap": args.steps,
        "wall_h_cap": args.wall_hours,
        "seed": args.seed,
        "ret_ok_early_stop": cb.stop_ret_ok,
        "best": best,
        "history_n": len(cb.history),
        "installed_sha16": installed_sha,
        "frozen_sha16_kept": E7SHA16 if not installed else None,
        "soft_pass": False,
        "bars_kept": True,
        "outer_logged_best": bool(best.get("outer_logged")),
        "outer_idle_best": bool(best.get("outer_idle")),
        "ret_ok_both": bool(best.get("ret_ok_both")),
        "vs_E7lock": "honest_beat" if beat else "Prefer FAIL (no ret_ok both under bars+ε+outer)",
        "next_lever": (
            None if beat else "T5-D2 ICP step timing+location OR Dave T5-D4/H2 named unlock — not residual twin; not auto H2"
        ),
    }
    (OUT / "T5D1_DONE.json").write_text(json.dumps(done, indent=2, default=str))
    _write_progress([
        f"## DONE · disposition=**{disposition}** · outcome=`{outcome}` · "
        f"wall={elapsed:.0f}s ({elapsed/3600:.2f}h) · steps={model.num_timesteps}\n",
        f"- installed_new_sha16={installed} · best_sha=`{best.get('ckpt_sha16')}`\n",
        f"- best apps={best.get('apps')} rcf={best.get('ret_cf')} skate={best.get('skate')} "
        f"ret_ok={best.get('ret_ok')} eps_steal={best.get('eps_steal')}\n",
        f"- outer_logged={best.get('outer_logged')} outer_idle={best.get('outer_idle')} "
        f"outer_F={best.get('outer_F')} OUTER=`{AinexT5D1OuterPrimaryEnv.OUTER_SRC}`\n",
        f"- soft_pass=false · bars_kept=true · frozen_E7lock=`{E7SHA16}`\n",
        f"- next_lever={done['next_lever']}\n",
    ])
    print(
        f"=== T5D1 DONE === {outcome} {disposition} wall={elapsed:.0f}s "
        f"installed={installed} best_sha={best.get('ckpt_sha16')} "
        f"outer_idle={best.get('outer_idle')}",
        flush=True,
    )
    env.close()


if __name__ == "__main__":
    main()
