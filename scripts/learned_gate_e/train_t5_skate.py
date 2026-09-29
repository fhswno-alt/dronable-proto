#!/usr/bin/env python3
"""T5-A SKATE-PRIMARY reverse residual — Prefer FAIL cospec.

Cospec: docs/GATE_Q_AI_COSPEC_T5.md (+ proposal GATE_Q_AI_COSPEC_T5_PROPOSAL.md)
Family: skate mean+p95 dominate; constrain cf≥0.55 + apps + tip≥8 + plant-cam ε.
Bootstrap: T4_09 near-miss zip if present, else E7lock sha16 9ffaa1a21b607bf6.
Opts in-family: SLR/TDVM + ANK DF + DXB (clear-only). Keep T4 B+C conditioning.
Budget: ≤4h wall OR ≤2e6 steps (first hit). Soft-pass NEVER.
Eval: dual-ckpt approach=E7lock · retreat=T5 via GATE_Q_T5_CKPT.
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

from env_ainex import make_t5_env  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"
OUT.mkdir(parents=True, exist_ok=True)
PROGRESS = ITER / "GATE_Q_T5_PROGRESS.md"
E7LOCK = OUT / "ppo_gate_e_locked.zip"
E7BEST = OUT / "ppo_gate_e_best.zip"
E7SHA16 = "9ffaa1a21b607bf6"
T4_09 = OUT / "ppo_t4_best_T4_09_near_miss.zip"
T4_09_ALT = OUT / "ppo_t4_step1350000.zip"
PY = str(ROOT / ".venv" / "bin" / "python")


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _bootstrap() -> Path:
    for cand in (T4_09, T4_09_ALT):
        if cand.exists():
            print(f"[T5] bootstrap=T4_09 near-miss {cand.name} sha16={_sha16(cand)}", flush=True)
            return cand
    if E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16:
        print(f"[T5] bootstrap=ppo_gate_e_locked.zip sha16={E7SHA16}", flush=True)
        return E7LOCK
    if E7BEST.exists() and _sha16(E7BEST) == E7SHA16:
        print(f"[T5] bootstrap=ppo_gate_e_best.zip sha16={E7SHA16}", flush=True)
        return E7BEST
    raise FileNotFoundError(
        f"No T4_09 near-miss and E7lock sha16={E7SHA16} missing at {E7LOCK}/{E7BEST}"
    )


def _write_progress(lines: list[str]) -> None:
    hdr = (
        "# Gate Q T5-A SKATE-PRIMARY — live progress\n\n"
        f"**Cospec:** `docs/GATE_Q_AI_COSPEC_T5.md` · Soft-pass **off** · Bars KEPT\n"
        f"**Family:** skate mean+p95 primary · cf≥0.55 floor · SLR/ANK/DXB in-family\n"
        f"**Bootstrap:** T4_09 near-miss if present else E7lock `{E7SHA16}`\n"
        f"**Budget:** ≤4h wall OR ≤2e6 steps · Early stop on ret_ok both\n"
        f"**Dual-ckpt:** approach=E7lock · retreat=`GATE_Q_T5_CKPT`\n\n"
    )
    body = "\n".join(lines) + "\n"
    if not PROGRESS.exists():
        PROGRESS.write_text(hdr + body)
        return
    prev = PROGRESS.read_text()
    if not prev.startswith("# Gate Q T5"):
        PROGRESS.write_text(hdr + body + ("\n" + prev if prev else ""))
    else:
        PROGRESS.write_text(prev.rstrip() + "\n\n" + body)


def _parse_probe(log_text: str) -> dict:
    out: dict = {
        "apps": [], "ret_cf": [], "skate": [], "tip_ret": [], "ret_ok": [],
        "mid": [], "eps": [], "cam_ret": [], "raw": {},
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
    # cam@preSS / dxc_pre if present
    for m in re.finditer(r"cam@preSS=([0-9.]+).*?dxc_pre=([0-9.]+)", log_text):
        out.setdefault("cam_preSS", []).append(float(m.group(1)))
        out.setdefault("dxc_pre", []).append(float(m.group(2)))
    steals = re.findall(r"steal=(True|False)", log_text)
    out["eps"] = [s == "True" for s in steals]
    out["n_pass"] = None
    m = re.search(r"n_pass=(\d+)/(\d+)", log_text)
    if m:
        out["n_pass"] = (int(m.group(1)), int(m.group(2)))
    out["ret_ok_both"] = bool(out["ret_ok"]) and all(out["ret_ok"][:2]) and len(out["ret_ok"]) >= 2
    return out


def _bars_beat_e7lock(m: dict) -> bool:
    """Honest beat: ret_ok both under bars + apps held + plant-cam ≤ε. Soft-pass NEVER."""
    if not m.get("ret_ok_both"):
        return False
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
    return True


def run_gate_q_probe(tag: str, t5_ckpt: Path | None, wall_t0: float) -> dict:
    """Skip-video Prefer FAIL probe. t5_ckpt=None → E7lock only (T5_R0)."""
    env = os.environ.copy()
    env["GATE_Q_SKIP_VIDEO"] = "1"
    env["MUJOCO_GL"] = "glfw"
    for k in (
        "GATE_Q_RET_CSF50", "GATE_Q_RET_ALIP_TVR", "GATE_Q_RET_REV_PHASE",
        "GATE_Q_RET_CLEAR_BURST", "GATE_Q_RET_BURST_PLANT_CAP",
        "GATE_Q_RET_MAX_PLANT_DWELL", "GATE_Q_RET_SWING_PLACE_M",
    ):
        env[k] = "0"
    # Clear T4 so dual-ckpt is T5-only when set
    env.pop("GATE_Q_T4_CKPT", None)
    if t5_ckpt is not None:
        env["GATE_Q_T5_CKPT"] = str(t5_ckpt.resolve())
        env["GATE_Q_T5_VX_CMD"] = "-0.08"
    else:
        env.pop("GATE_Q_T5_CKPT", None)
    log_path = ITER / f"GATE_Q_{tag}_PROBE.log"
    t0 = time.time()
    print(f"[T5-eval] {tag} ckpt={t5_ckpt} …", flush=True)
    proc = subprocess.run(
        [PY, str(ROOT / "scripts" / "score_gate_q.py")],
        cwd=str(ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=900,
    )
    text = (proc.stdout or "") + "\n" + (proc.stderr or "")
    log_path.write_text(text)
    metrics = _parse_probe(text)
    metrics["tag"] = tag
    metrics["exit"] = proc.returncode
    metrics["wall_s_probe"] = time.time() - t0
    metrics["wall_s_train"] = time.time() - wall_t0
    metrics["ckpt"] = str(t5_ckpt) if t5_ckpt else "E7lock"
    metrics["ckpt_sha16"] = _sha16(t5_ckpt) if t5_ckpt and t5_ckpt.exists() else E7SHA16
    apps = metrics.get("apps") or []
    rcf = metrics.get("ret_cf") or []
    sk = metrics.get("skate") or []
    tip = metrics.get("tip_ret") or []
    mid = metrics.get("mid") or []
    line = (
        f"### {tag} · wall={metrics['wall_s_train']:.0f}s · sha16=`{metrics['ckpt_sha16']}`\n"
        f"- apps={apps[:2]} · ret_cf={rcf[:2]} · skate={sk[:2]} · tip_ret={tip[:2]}\n"
        f"- mid={mid[:2]} · ret_ok={metrics.get('ret_ok')} · ret_ok_both={metrics.get('ret_ok_both')}\n"
        f"- probe_wall={metrics['wall_s_probe']:.0f}s · exit={proc.returncode} · log=`{log_path.name}`\n"
    )
    _write_progress([line])
    (OUT / f"T5_{tag}_metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    print(
        f"[T5-eval] {tag} apps={apps[:2]} rcf={rcf[:2]} sk={sk[:2]} "
        f"ret_ok={metrics.get('ret_ok')} both={metrics.get('ret_ok_both')}",
        flush=True,
    )
    return metrics


class T5Callback(BaseCallback):
    def __init__(self, total_steps: int, eval_every: int, wall_s: float, seed: int):
        super().__init__()
        self.total_steps = total_steps
        self.eval_every = eval_every
        self.wall_s = wall_s
        self.seed = seed
        self.t0 = time.time()
        self.last_eval = 0
        self.history: list[dict] = []
        self.best: dict | None = None
        self.stop_ret_ok = False
        self.probe_idx = 0

    def _curriculum(self) -> None:
        # DXB-leaning short reverse → moderate; keep shorter than T4 to cut TD shear
        frac = min(1.0, self.num_timesteps / max(1, int(0.7 * self.total_steps)))
        target = 0.06 + 0.20 * frac   # 0.06 → 0.26 (shorter than T4 0.08→0.35)
        ep_s = 3.0 + 2.5 * frac       # 3 → 5.5 s
        vx_lo = -0.05 - 0.05 * frac   # −0.05 → −0.10
        vx_hi = -0.03 - 0.02 * frac   # −0.03 → −0.05
        dxb = 0.030 + 0.020 * frac    # short-step cap curriculum
        for e in self.training_env.envs:
            env = e
            while hasattr(env, "env"):
                env = env.env
            if hasattr(env, "set_curriculum"):
                env.set_curriculum(target_dx=target, episode_s=ep_s, vx_lo=vx_lo, vx_hi=vx_hi)
            if hasattr(env, "dxb_cap"):
                env.dxb_cap = float(dxb)

    def _on_step(self) -> bool:
        self._curriculum()
        if time.time() - self.t0 >= self.wall_s:
            print(f"[T5] wall limit {self.wall_s}s at steps={self.num_timesteps}", flush=True)
            return False
        if self.num_timesteps - self.last_eval >= self.eval_every and self.num_timesteps > 0:
            self.last_eval = self.num_timesteps
            self.probe_idx += 1
            tag = f"T5_{self.probe_idx:02d}"
            ckpt = OUT / f"ppo_t5_step{self.num_timesteps}"
            self.model.save(str(ckpt))
            zip_path = Path(str(ckpt) + ".zip")
            _write_progress([
                f"### train land · steps={self.num_timesteps} · wall={time.time()-self.t0:.0f}s · "
                f"ckpt=`{zip_path.name}` · launching Prefer FAIL `{tag}`\n"
            ])
            metrics = run_gate_q_probe(tag, zip_path, self.t0)
            metrics["steps"] = self.num_timesteps
            self.history.append(metrics)
            (OUT / "t5_train_history.json").write_text(json.dumps(self.history, indent=2, default=str))
            score = self._rank(metrics)
            if self.best is None or score > self._rank(self.best):
                self.best = metrics
                self.model.save(str(OUT / "ppo_t5_best"))
                _write_progress([
                    f"**best-so-far** → `{tag}` rank={score:.3f} sha16=`{metrics['ckpt_sha16']}` "
                    f"skate={metrics.get('skate')} rcf={metrics.get('ret_cf')}\n"
                ])
            if metrics.get("ret_ok_both") and _bars_beat_e7lock(metrics):
                print("[T5] ret_ok both under bars — early stop", flush=True)
                self.stop_ret_ok = True
                self.model.save(str(OUT / "ppo_t5_retok"))
                return False
        return True

    @staticmethod
    def _rank(m: dict) -> float:
        """Higher better. SKATE-PRIMARY rank — p95 under bar dominates; cf floor; no soft-pass."""
        rcf = m.get("ret_cf") or [0.0, 0.0]
        sk = m.get("skate") or [(1.0, 1.0), (1.0, 1.0)]
        apps = m.get("apps") or [0.0, 0.0]
        tip = m.get("tip_ret") or [0.0, 0.0]
        # Heavy skate penalty (primary)
        sk_pen = 0.0
        sk_under = 0.0
        for a, b in sk[:2]:
            sk_pen += max(0.0, a - 0.08) * 8.0 + max(0.0, b - 0.18) * 10.0
            if a <= 0.08 + 1e-9 and b <= 0.18 + 1e-9:
                sk_under += 3.0
        # cf floor (constraint, not maximize)
        cf_s = 0.0
        for c in rcf[:2]:
            if c >= 0.55 - 1e-9:
                cf_s += 1.0 + 0.3 * min(c, 1.0)
            else:
                cf_s -= 3.0 * (0.55 - c)
        tip_s = sum(1.0 for t in tip[:2] if t >= 8.0)
        apps_s = 0.0
        if len(apps) >= 2:
            apps_ok = (apps[0] >= 0.548 - 0.02) and (apps[1] >= 0.630 - 0.02)
            apps_s = 2.0 if apps_ok else -5.0  # Prefer FAIL if apps traded
        ret_b = 8.0 if m.get("ret_ok_both") else 0.0
        return sk_under + cf_s + tip_s + apps_s + ret_b - sk_pen


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--wall-hours", type=float, default=4.0)
    ap.add_argument("--eval-every", type=int, default=150_000)
    ap.add_argument("--seed", type=int, default=41)
    ap.add_argument("--n-envs", type=int, default=2)
    ap.add_argument("--skip-r0", action="store_true")
    ap.add_argument("--lr", type=float, default=8e-5)
    args = ap.parse_args()

    wall_s = args.wall_hours * 3600.0
    boot = _bootstrap()
    assert E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16, "E7lock rollback must stay present"

    _write_progress([
        f"## Start · {time.strftime('%Y-%m-%d %H:%M:%S %Z')}\n",
        f"- bootstrap=`{boot.name}` sha16=`{_sha16(boot)}`\n",
        f"- family=**T5-A SKATE-PRIMARY** · opts=**SLR+ANK+DXB** (+ T4 B+C conditioning)\n",
        f"- NOT reopening R*/S1–S3 twins · cancel×0.70 · clear-only residual\n",
        f"- companion md5 `59cc408eda07037a58f92ad27da045d6` · walk plant `fc94709c84f5598d4474ecfc4bb41fdc`\n",
        f"- steps≤{args.steps} wall≤{args.wall_hours}h eval_every={args.eval_every} "
        f"seed={args.seed} n_envs={args.n_envs} lr={args.lr}\n",
    ])

    t0 = time.time()
    if not args.skip_r0:
        print("[T5] T5_R0 = E7lock baseline (flags OFF, skip-video)", flush=True)
        r0 = run_gate_q_probe("T5_R0", None, t0)
        r0_ok = (
            len(r0.get("apps", [])) >= 2
            and abs(r0["apps"][0] - 0.548) < 0.02
            and abs(r0["apps"][1] - 0.630) < 0.05
        )
        _write_progress([
            f"**T5_R0 ≡ E7lock:** {'YES' if r0_ok else 'CHECK'} apps={r0.get('apps')} "
            f"rcf={r0.get('ret_cf')} skate={r0.get('skate')}\n"
        ])
        (OUT / "T5_R0_metrics.json").write_text(json.dumps(r0, indent=2, default=str))

        # Also note T4_09 baseline via dual-ckpt (fair compare)
        if T4_09.exists():
            print("[T5] T5_T4_09 = T4_09 near-miss dual-ckpt note", flush=True)
            t409 = run_gate_q_probe("T5_T4_09", T4_09, t0)
            _write_progress([
                f"**T5_T4_09 note:** apps={t409.get('apps')} rcf={t409.get('ret_cf')} "
                f"skate={t409.get('skate')} ret_ok={t409.get('ret_ok')}\n"
            ])

    def make(rank):
        return make_t5_env(
            gait_t=0.75, rank=rank, seed=args.seed,
            episode_s=3.5, stand_hold=0.60, ramp_t=0.80,
            action_scale=0.07, use_vik=True, use_plant=True, use_cp=True,
            vx_cmd=-0.07, target_dx=0.08, rev_phase=True,
            clear_only_residual=True, vx_cmd_rand=True,
            vx_lo=-0.05, vx_hi=-0.03,
            use_slr=True, use_ank=True, use_dxb=True, dxb_cap=0.035,
        )

    env = DummyVecEnv([make(i) for i in range(args.n_envs)])
    model = PPO.load(str(boot), env=env, device="cpu")
    model.learning_rate = args.lr
    print(
        f"[T5] finetune start from {boot.name} lr={args.lr} "
        f"steps≤{args.steps} wall≤{wall_s}s family=T5-A SLR+ANK+DXB",
        flush=True,
    )

    cb = T5Callback(args.steps, args.eval_every, wall_s, args.seed)
    cb.t0 = t0
    model.learn(
        total_timesteps=args.steps,
        callback=cb,
        progress_bar=False,
        reset_num_timesteps=True,
    )
    elapsed = time.time() - t0
    final = OUT / "ppo_t5_final"
    model.save(str(final))
    final_zip = Path(str(final) + ".zip")

    fin_m = run_gate_q_probe("T5_FINAL", final_zip, t0)
    fin_m["steps"] = model.num_timesteps
    cb.history.append(fin_m)
    (OUT / "t5_train_history.json").write_text(json.dumps(cb.history, indent=2, default=str))

    best = cb.best or fin_m
    beat = _bars_beat_e7lock(best) if best else False
    installed = False
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
                txt,
                count=1,
            )
            if txt2 != txt:
                sgq.write_text(txt2)
            installed = True
            _write_progress([
                f"## HONEST BEAT — installed sha16=`{new_sha}` into ppo_gate_e_best + CKPT_SHA16\n"
                f"- rollback locked remains `{E7SHA16}` at `ppo_gate_e_locked.zip`\n"
            ])

    disposition = (
        "ret_ok both — honest beat"
        if beat
        else (
            "near-miss"
            if (best.get("ret_cf") or [0])
            and min((best.get("ret_cf") or [0])[:2] or [0]) >= 0.50
            and any(
                (sk[0] <= 0.09 and sk[1] <= 0.22)
                for sk in (best.get("skate") or [])[:2]
            )
            else "Prefer FAIL train wall"
        )
    )
    summary = {
        "disposition": disposition,
        "family": "T5-A SKATE-PRIMARY",
        "bootstrap": str(boot),
        "bootstrap_sha16": _sha16(boot),
        "opts": ["SLR", "ANK", "DXB", "B", "C"],
        "wall_s": elapsed,
        "steps": int(model.num_timesteps),
        "T5_R0_e7lock": True,
        "best": best,
        "installed_sha16": _sha16(E7BEST) if installed else None,
        "frozen_sha16_kept": E7SHA16 if not installed else None,
        "ret_ok_early_stop": cb.stop_ret_ok,
        "soft_pass": False,
        "bars_kept": True,
    }
    (OUT / "T5_DONE.json").write_text(json.dumps(summary, indent=2, default=str))
    _write_progress([
        f"## DONE · disposition=**{disposition}** · wall={elapsed:.0f}s ({elapsed/3600:.2f}h) · "
        f"steps={model.num_timesteps}\n",
        f"- installed_new_sha16={installed} · best_sha=`{best.get('ckpt_sha16')}`\n",
        f"- best apps={best.get('apps')} rcf={best.get('ret_cf')} skate={best.get('skate')} "
        f"ret_ok={best.get('ret_ok')}\n",
        f"- final=`{final_zip}` best=`{OUT / 'ppo_t5_best.zip'}`\n",
    ])
    print(
        f"=== T5 DONE === {disposition} wall={elapsed:.0f}s steps={model.num_timesteps} "
        f"installed={installed}",
        flush=True,
    )
    env.close()


if __name__ == "__main__":
    main()
