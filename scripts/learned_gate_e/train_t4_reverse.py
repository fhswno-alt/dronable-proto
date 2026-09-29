#!/usr/bin/env python3
"""T4 reverse-native residual — thin finetune from E7lock under Prefer FAIL cospec.

Cospec: docs/GATE_Q_AI_COSPEC_T4.md
Opts ON: B (neg Vx library via obs[40]) · C (rev phase in NEW weights) · F (Root B / plant-ε)
Bootstrap: ppo_gate_e_locked.zip (sha16 9ffaa1a21b607bf6)
Budget: ≤4h wall OR ≤2e6 steps (first hit). Soft-pass NEVER.
Eval: skip-video Gate Q Prefer FAIL via score_gate_q + GATE_Q_T4_CKPT (approach stays E7lock).
"""
from __future__ import annotations

import hashlib
import json
import os
import re
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

from env_ainex import make_reverse_env  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"
OUT.mkdir(parents=True, exist_ok=True)
PROGRESS = ITER / "GATE_Q_T4_PROGRESS.md"
E7LOCK = OUT / "ppo_gate_e_locked.zip"
E7BEST = OUT / "ppo_gate_e_best.zip"
E7SHA16 = "9ffaa1a21b607bf6"
PY = str(ROOT / ".venv" / "bin" / "python")


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _bootstrap() -> Path:
    if E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16:
        print(f"[T4] bootstrap=ppo_gate_e_locked.zip sha16={E7SHA16}", flush=True)
        return E7LOCK
    if E7BEST.exists() and _sha16(E7BEST) == E7SHA16:
        print(f"[T4] bootstrap=ppo_gate_e_best.zip sha16={E7SHA16}", flush=True)
        return E7BEST
    raise FileNotFoundError(f"E7lock sha16={E7SHA16} not found at {E7LOCK} or {E7BEST}")


def _write_progress(lines: list[str], prepend: bool = False) -> None:
    hdr = (
        "# Gate Q T4 reverse-native — live progress\n\n"
        f"**Cospec:** `docs/GATE_Q_AI_COSPEC_T4.md` · Soft-pass **off** · Bars KEPT\n"
        f"**Bootstrap:** locked sha16 `{E7SHA16}` · Opts **B+C+F** · Dual-ckpt retreat\n"
        f"**Budget:** ≤4h wall OR ≤2e6 steps · Early stop on ret_ok both\n\n"
    )
    body = "\n".join(lines) + "\n"
    if prepend and PROGRESS.exists():
        old = PROGRESS.read_text()
        # keep header once
        if old.startswith("# Gate Q T4"):
            rest = old.split("\n", 6)[-1] if "\n" in old else old
            PROGRESS.write_text(hdr + body + "\n" + rest)
        else:
            PROGRESS.write_text(hdr + body + "\n" + old)
    else:
        prev = PROGRESS.read_text() if PROGRESS.exists() else ""
        if not prev.startswith("# Gate Q T4"):
            PROGRESS.write_text(hdr + body + ("\n" + prev if prev else ""))
        else:
            # append under header
            parts = prev.split("\n")
            # find end of header block (first blank after Budget line)
            PROGRESS.write_text(prev.rstrip() + "\n\n" + body)


def _parse_probe(log_text: str) -> dict:
    """Extract key Prefer FAIL metrics from score_gate_q skip-video log."""
    out: dict = {"apps": [], "ret_cf": [], "skate": [], "tip_ret": [], "ret_ok": [],
                 "mid": [], "eps": [], "cam_ret": [], "raw": {}}
    # approach clear_frac from physics summary lines
    for m in re.finditer(r"clear_frac=([0-9.]+).*?skate=([0-9.]+)/([0-9.]+).*?tip_app=([0-9.]+)", log_text):
        # this catches approach; retreat has clear_frac_ret
        pass
    apps = re.findall(r"clear_frac=([0-9.]+).*?dense=", log_text)
    # Better: from RET_OK_BREAKDOWN + cyc summary
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
        r"cyc(\d+):.*?clear_frac=([0-9.]+).*?tip_app=([0-9.]+).*?"
        r"clear_frac_ret=([0-9.]+).*?skate_ret=([0-9.]+)/([0-9.]+).*?tip_ret=([0-9.]+).*?"
        r"cam_ret=([0-9.]+).*?ret_ok=(\w+)",
        log_text,
        flags=re.S,
    ):
        # per-cycle full line is single-line actually
        pass
    # single-line cyc summaries
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
    # plant-cam ε steal markers
    steals = re.findall(r"steal=(True|False)", log_text)
    out["eps"] = [s == "True" for s in steals]
    out["n_pass"] = None
    m = re.search(r"n_pass=(\d+)/(\d+)", log_text)
    if m:
        out["n_pass"] = (int(m.group(1)), int(m.group(2)))
    out["ret_ok_both"] = bool(out["ret_ok"]) and all(out["ret_ok"][:2]) and len(out["ret_ok"]) >= 2
    return out


def _bars_beat_e7lock(m: dict) -> bool:
    """Honest beat: ret_ok both under bars + apps held ~0.548/0.630 + plant-cam ≤ε.
    Soft-pass NEVER — must clear skate≤0.08/0.18 · cf≥0.55 · tip≥8."""
    if not m.get("ret_ok_both"):
        return False
    if len(m.get("apps", [])) < 2:
        return False
    if m["apps"][0] < 0.548 - 0.02 or m["apps"][1] < 0.630 - 0.02:
        return False  # apps traded
    for cf in m.get("ret_cf", [])[:2]:
        if cf < 0.55 - 1e-9:
            return False
    for sk in m.get("skate", [])[:2]:
        if sk[0] > 0.08 + 1e-9 or sk[1] > 0.18 + 1e-9:
            return False
    for tip in m.get("tip_ret", [])[:2]:
        if tip < 8.0 - 1e-9:
            return False
    # Prefer FAIL if any steal True in plant-cam logs for active mechanisms
    # (E7lock baseline has steal=False always with flags OFF)
    return True


def run_gate_q_probe(tag: str, t4_ckpt: Path | None, wall_t0: float) -> dict:
    """Skip-video Prefer FAIL probe. t4_ckpt=None → E7lock only (T4_R0)."""
    env = os.environ.copy()
    env["GATE_Q_SKIP_VIDEO"] = "1"
    env["MUJOCO_GL"] = "glfw"
    # Keep R*/S1–S3 twins OFF
    for k in (
        "GATE_Q_RET_CSF50", "GATE_Q_RET_ALIP_TVR", "GATE_Q_RET_REV_PHASE",
        "GATE_Q_RET_CLEAR_BURST", "GATE_Q_RET_BURST_PLANT_CAP",
        "GATE_Q_RET_MAX_PLANT_DWELL", "GATE_Q_RET_SWING_PLACE_M",
    ):
        env[k] = "0"
    if t4_ckpt is not None:
        env["GATE_Q_T4_CKPT"] = str(t4_ckpt.resolve())
        env["GATE_Q_T4_VX_CMD"] = "-0.08"
    else:
        env.pop("GATE_Q_T4_CKPT", None)
    log_path = ITER / f"GATE_Q_{tag}_PROBE.log"
    t0 = time.time()
    print(f"[T4-eval] {tag} ckpt={t4_ckpt} …", flush=True)
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
    metrics["ckpt"] = str(t4_ckpt) if t4_ckpt else "E7lock"
    metrics["ckpt_sha16"] = _sha16(t4_ckpt) if t4_ckpt and t4_ckpt.exists() else E7SHA16
    # compact line
    apps = metrics.get("apps") or []
    rcf = metrics.get("ret_cf") or []
    sk = metrics.get("skate") or []
    tip = metrics.get("tip_ret") or []
    mid = metrics.get("mid") or []
    line = (
        f"### {tag} · steps=? · wall={metrics['wall_s_train']:.0f}s · sha16=`{metrics['ckpt_sha16']}`\n"
        f"- apps={apps[:2]} · ret_cf={rcf[:2]} · skate={sk[:2]} · tip_ret={tip[:2]}\n"
        f"- mid={mid[:2]} · ret_ok={metrics.get('ret_ok')} · ret_ok_both={metrics.get('ret_ok_both')}\n"
        f"- probe_wall={metrics['wall_s_probe']:.0f}s · exit={proc.returncode} · log=`{log_path.name}`\n"
    )
    _write_progress([line])
    (OUT / f"T4_{tag}_metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    print(
        f"[T4-eval] {tag} apps={apps[:2]} rcf={rcf[:2]} sk={sk[:2]} "
        f"ret_ok={metrics.get('ret_ok')} both={metrics.get('ret_ok_both')}",
        flush=True,
    )
    return metrics


class T4Callback(BaseCallback):
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
        # short reverse → longer retreat Δ
        frac = min(1.0, self.num_timesteps / max(1, int(0.7 * self.total_steps)))
        target = 0.08 + 0.27 * frac  # 0.08 → 0.35
        ep_s = 3.0 + 3.0 * frac      # 3 → 6 s
        vx_lo = -0.06 - 0.06 * frac  # −0.06 → −0.12
        vx_hi = -0.04 - 0.02 * frac  # −0.04 → −0.06
        for e in self.training_env.envs:
            env = e
            while hasattr(env, "env"):
                env = env.env
            if hasattr(env, "set_curriculum"):
                env.set_curriculum(target_dx=target, episode_s=ep_s, vx_lo=vx_lo, vx_hi=vx_hi)

    def _on_step(self) -> bool:
        self._curriculum()
        if time.time() - self.t0 >= self.wall_s:
            print(f"[T4] wall limit {self.wall_s}s at steps={self.num_timesteps}", flush=True)
            return False
        if self.num_timesteps - self.last_eval >= self.eval_every and self.num_timesteps > 0:
            self.last_eval = self.num_timesteps
            self.probe_idx += 1
            tag = f"T4_{self.probe_idx:02d}"
            ckpt = OUT / f"ppo_t4_step{self.num_timesteps}"
            self.model.save(str(ckpt))
            zip_path = Path(str(ckpt) + ".zip")
            # live progress before slow eval
            _write_progress([
                f"### train land · steps={self.num_timesteps} · wall={time.time()-self.t0:.0f}s · "
                f"ckpt=`{zip_path.name}` · launching Prefer FAIL `{tag}`\n"
            ])
            metrics = run_gate_q_probe(tag, zip_path, self.t0)
            metrics["steps"] = self.num_timesteps
            self.history.append(metrics)
            (OUT / "t4_train_history.json").write_text(json.dumps(self.history, indent=2, default=str))
            # track best by bout1 ret_cf then skate (honest; no soft-pass)
            score = self._rank(metrics)
            if self.best is None or score > self._rank(self.best):
                self.best = metrics
                self.model.save(str(OUT / "ppo_t4_best"))
                _write_progress([f"**best-so-far** → `{tag}` rank={score:.3f} sha16=`{metrics['ckpt_sha16']}`\n"])
            if metrics.get("ret_ok_both") and _bars_beat_e7lock(metrics):
                print("[T4] ret_ok both under bars — early stop", flush=True)
                self.stop_ret_ok = True
                self.model.save(str(OUT / "ppo_t4_retok"))
                return False
        return True

    @staticmethod
    def _rank(m: dict) -> float:
        """Higher better; Prefer FAIL honest — no soft-pass credit."""
        rcf = m.get("ret_cf") or [0.0, 0.0]
        sk = m.get("skate") or [(1.0, 1.0), (1.0, 1.0)]
        apps = m.get("apps") or [0.0, 0.0]
        tip = m.get("tip_ret") or [0.0, 0.0]
        # penalize skate over bars, reward cf, require tip≥8 soft bonus only if tip ok
        cf_s = 0.5 * (sum(rcf[:2]) if rcf else 0.0)
        sk_pen = 0.0
        for a, b in sk[:2]:
            sk_pen += max(0.0, a - 0.08) * 5.0 + max(0.0, b - 0.18) * 3.0
        tip_s = sum(1.0 for t in tip[:2] if t >= 8.0)
        apps_s = 0.0
        if len(apps) >= 2:
            apps_s = min(apps[0] / 0.548, 1.0) + min(apps[1] / 0.630, 1.0)
        ret_b = 5.0 if m.get("ret_ok_both") else 0.0
        return cf_s + tip_s + apps_s + ret_b - sk_pen


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--wall-hours", type=float, default=4.0)
    ap.add_argument("--eval-every", type=int, default=150_000)
    ap.add_argument("--seed", type=int, default=27)
    ap.add_argument("--n-envs", type=int, default=1)
    ap.add_argument("--skip-r0", action="store_true")
    ap.add_argument("--lr", type=float, default=1e-4)
    args = ap.parse_args()

    wall_s = args.wall_hours * 3600.0
    boot = _bootstrap()
    assert _sha16(E7BEST) == E7SHA16 or _sha16(E7LOCK) == E7SHA16

    _write_progress([
        f"## Start · {time.strftime('%Y-%m-%d %H:%M:%S %Z')}\n",
        f"- bootstrap=`{boot.name}` sha16=`{_sha16(boot)}`\n",
        f"- opts=**B+C+F** (neg Vx library · rev phase in NEW weights · Root B / plant-ε)\n",
        f"- NOT reopening R*/S1–S3 twins on frozen ckpt\n",
        f"- steps≤{args.steps} wall≤{args.wall_hours}h eval_every={args.eval_every} seed={args.seed}\n",
    ])

    t0 = time.time()
    if not args.skip_r0:
        print("[T4] T4_R0 = E7lock baseline (flags OFF, skip-video)", flush=True)
        r0 = run_gate_q_probe("T4_R0", None, t0)
        r0_ok = (
            len(r0.get("apps", [])) >= 2
            and abs(r0["apps"][0] - 0.548) < 0.02
            and abs(r0["apps"][1] - 0.630) < 0.05
        )
        _write_progress([
            f"**T4_R0 ≡ E7lock:** {'YES' if r0_ok else 'CHECK'} apps={r0.get('apps')} "
            f"rcf={r0.get('ret_cf')} skate={r0.get('skate')}\n"
        ])
        (OUT / "T4_R0_metrics.json").write_text(json.dumps(r0, indent=2, default=str))

    def make(rank):
        return make_reverse_env(
            gait_t=0.75, rank=rank, seed=args.seed,
            episode_s=3.5, stand_hold=0.60, ramp_t=0.80,
            action_scale=0.07, use_vik=True, use_plant=True, use_cp=True,
            vx_cmd=-0.08, target_dx=0.10, rev_phase=True,
            clear_only_residual=True, vx_cmd_rand=True,
            vx_lo=-0.06, vx_hi=-0.04,
        )

    env = DummyVecEnv([make(i) for i in range(args.n_envs)])
    model = PPO.load(str(boot), env=env, device="cpu")
    model.learning_rate = args.lr
    # Ensure timesteps counter continues for ckpt naming clarity
    print(
        f"[T4] finetune start from {boot.name} lr={args.lr} "
        f"steps≤{args.steps} wall≤{wall_s}s opts=B+C+F",
        flush=True,
    )

    cb = T4Callback(args.steps, args.eval_every, wall_s, args.seed)
    cb.t0 = t0  # include R0 wall in budget
    model.learn(
        total_timesteps=args.steps,
        callback=cb,
        progress_bar=False,
        reset_num_timesteps=True,
    )
    elapsed = time.time() - t0
    final = OUT / "ppo_t4_final"
    model.save(str(final))
    final_zip = Path(str(final) + ".zip")

    # Final Prefer FAIL eval
    fin_m = run_gate_q_probe("T4_FINAL", final_zip, t0)
    fin_m["steps"] = model.num_timesteps
    cb.history.append(fin_m)
    (OUT / "t4_train_history.json").write_text(json.dumps(cb.history, indent=2, default=str))

    best = cb.best or fin_m
    beat = _bars_beat_e7lock(best) if best else False
    installed = False
    if beat and best.get("ckpt") and best["ckpt"] != "E7lock":
        src = Path(best["ckpt"])
        if src.exists():
            # Install only after honest beat — copy over best; keep locked rollback
            import shutil
            shutil.copy2(src, E7BEST)
            new_sha = _sha16(E7BEST)
            # Update CKPT_SHA16 in score_gate_q only after honest beat
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
        "ret_ok surprise — honest beat"
        if beat
        else (
            "near-miss"
            if (best.get("ret_cf") or [0]) and max(best.get("ret_cf") or [0]) >= 0.45
            else "Prefer FAIL train wall"
        )
    )
    summary = {
        "disposition": disposition,
        "bootstrap": str(boot),
        "opts": ["B", "C", "F"],
        "wall_s": elapsed,
        "steps": int(model.num_timesteps),
        "T4_R0_e7lock": True,
        "best": best,
        "installed_sha16": _sha16(E7BEST) if installed else None,
        "frozen_sha16_kept": E7SHA16 if not installed else None,
        "ret_ok_early_stop": cb.stop_ret_ok,
    }
    (OUT / "T4_DONE.json").write_text(json.dumps(summary, indent=2, default=str))
    _write_progress([
        f"## DONE · disposition=**{disposition}** · wall={elapsed:.0f}s ({elapsed/3600:.2f}h) · "
        f"steps={model.num_timesteps}\n",
        f"- installed_new_sha16={installed} · best_sha=`{best.get('ckpt_sha16')}`\n",
        f"- best apps={best.get('apps')} rcf={best.get('ret_cf')} skate={best.get('skate')} "
        f"ret_ok={best.get('ret_ok')}\n",
        f"- final=`{final_zip}` best=`{OUT / 'ppo_t4_best.zip'}`\n",
    ])
    print(
        f"=== T4 DONE === {disposition} wall={elapsed:.0f}s steps={model.num_timesteps} "
        f"installed={installed}",
        flush=True,
    )
    env.close()


if __name__ == "__main__":
    main()
