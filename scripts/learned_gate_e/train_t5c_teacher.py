#!/usr/bin/env python3
"""T5-C reverse teacher BC + multi-seed — Prefer FAIL cospec.

Cospec: docs/GATE_Q_AI_COSPEC_T5C.md (no veto)
Family: BC from CSF/Placo short-reverse oracle → residual PPO; multi-seed Prefer FAIL.
Bootstrap: T5B_02 near-miss zip if present, else T5_06, else T4_09, else E7lock.
Opts: teacher BC (A) + multi-seed (E); may carry ALIP/DCM/ANK + SLR/DXB conditioning.
Budget: ≤4h wall OR ≤2e6 steps PER SEED (first hit). Soft-pass NEVER.
Eval: dual-ckpt approach=E7lock · retreat=T5C via GATE_Q_T5C_CKPT.
Attack: bout0 skate p95 0.183→≤0.18 + kill plant-cam ε WITHOUT trading bout1 skate PASS / apps / cf.
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

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "scripts" / "learned_gate_e")]
os.environ.setdefault("MUJOCO_GL", "glfw")

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv

from env_ainex import AinexT5CTeacherEnv, make_t5c_env  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"
OUT.mkdir(parents=True, exist_ok=True)
PROGRESS = ITER / "GATE_Q_T5C_PROGRESS.md"
E7LOCK = OUT / "ppo_gate_e_locked.zip"
E7BEST = OUT / "ppo_gate_e_best.zip"
E7SHA16 = "9ffaa1a21b607bf6"
T5B_02 = OUT / "ppo_t5b_best_T5B_02_near_miss.zip"
T5B_02_SHA = "a18be16ca662454f"
T5_06 = OUT / "ppo_t5_best_T5_06_near_miss.zip"
T5_06_ALT = OUT / "ppo_t5_step900000.zip"
T4_09 = OUT / "ppo_t4_best_T4_09_near_miss.zip"
T4_09_ALT = OUT / "ppo_t4_step1350000.zip"
PY = str(ROOT / ".venv" / "bin" / "python")

# Prefer 3 seeds; ≥2 required by cospec
DEFAULT_SEEDS = [41, 43, 47]


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _bootstrap() -> Path:
    if T5B_02.exists():
        sha = _sha16(T5B_02)
        print(f"[T5C] bootstrap=T5B_02 near-miss {T5B_02.name} sha16={sha}", flush=True)
        return T5B_02
    for cand in (T5_06, T5_06_ALT):
        if cand.exists():
            print(f"[T5C] bootstrap=T5_06 near-miss {cand.name} sha16={_sha16(cand)}", flush=True)
            return cand
    for cand in (T4_09, T4_09_ALT):
        if cand.exists():
            print(f"[T5C] bootstrap=T4_09 near-miss {cand.name} sha16={_sha16(cand)}", flush=True)
            return cand
    if E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16:
        print(f"[T5C] bootstrap=ppo_gate_e_locked.zip sha16={E7SHA16}", flush=True)
        return E7LOCK
    if E7BEST.exists() and _sha16(E7BEST) == E7SHA16:
        print(f"[T5C] bootstrap=ppo_gate_e_best.zip sha16={E7SHA16}", flush=True)
        return E7BEST
    raise FileNotFoundError("No T5B_02/T5_06/T4_09 and E7lock missing")


def _write_progress(lines: list[str]) -> None:
    hdr = (
        "# Gate Q T5-C TEACHER-BC+MULTI-SEED — live progress\n\n"
        f"**Cospec:** `docs/GATE_Q_AI_COSPEC_T5C.md` · Soft-pass **off** · Bars KEPT\n"
        f"**Family:** reverse teacher BC + multi-seed Prefer FAIL · skate-honest\n"
        f"**Bootstrap:** T5B_02 near-miss if present else T5_06 else T4_09 else E7lock `{E7SHA16}`\n"
        f"**Budget:** ≤4h wall OR ≤2e6 steps **per seed** · Early stop on ret_ok both\n"
        f"**Dual-ckpt:** approach=E7lock · retreat=`GATE_Q_T5C_CKPT`\n"
        f"**Attack:** bout0 skate p95 + plant-cam ε · hold bout1 skate PASS + cf/apps\n\n"
    )
    body = "\n".join(lines) + "\n"
    if not PROGRESS.exists():
        PROGRESS.write_text(hdr + body)
        return
    prev = PROGRESS.read_text()
    if not prev.startswith("# Gate Q T5-C"):
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
    for m in re.finditer(r"cam@preSS=([0-9.]+).*?dxc_pre=([0-9.]+)", log_text):
        out.setdefault("cam_preSS", []).append(float(m.group(1)))
        out.setdefault("dxc_pre", []).append(float(m.group(2)))
    steals = re.findall(r"steal=(True|False)", log_text)
    out["eps"] = [s == "True" for s in steals]
    # T5B/T5C plant-cam suite maxG
    maxgs = [float(x) for x in re.findall(r"maxG=([0-9.]+)", log_text)]
    out["maxG"] = maxgs
    out["eps_steal"] = any(out["eps"]) or any(g > 0.02 + 1e-12 for g in maxgs)
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
    # plant-cam ε Prefer FAIL
    if m.get("eps_steal"):
        return False
    return True


def run_gate_q_probe(tag: str, t5c_ckpt: Path | None, wall_t0: float, seed: int | None = None) -> dict:
    """Skip-video Prefer FAIL probe. t5c_ckpt=None → E7lock only (T5C_R0)."""
    env = os.environ.copy()
    env["GATE_Q_SKIP_VIDEO"] = "1"
    env["MUJOCO_GL"] = "glfw"
    for k in (
        "GATE_Q_RET_CSF50", "GATE_Q_RET_ALIP_TVR", "GATE_Q_RET_REV_PHASE",
        "GATE_Q_RET_CLEAR_BURST", "GATE_Q_RET_BURST_PLANT_CAP",
        "GATE_Q_RET_MAX_PLANT_DWELL", "GATE_Q_RET_SWING_PLACE_M",
    ):
        env[k] = "0"
    # Clear prior dual-ckpt aliases — T5C only when set
    for k in ("GATE_Q_T4_CKPT", "GATE_Q_T5_CKPT", "GATE_Q_T5B_CKPT"):
        env.pop(k, None)
    if t5c_ckpt is not None:
        env["GATE_Q_T5C_CKPT"] = str(t5c_ckpt.resolve())
        env["GATE_Q_T5C_VX_CMD"] = "-0.08"
        env["GATE_Q_T5B_VX_CMD"] = "-0.08"
        env["GATE_Q_T5_VX_CMD"] = "-0.08"
    else:
        env.pop("GATE_Q_T5C_CKPT", None)
    log_path = ITER / f"GATE_Q_{tag}_PROBE.log"
    t0 = time.time()
    print(f"[T5C-eval] {tag} seed={seed} ckpt={t5c_ckpt} …", flush=True)
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
    metrics["seed"] = seed
    metrics["exit"] = proc.returncode
    metrics["wall_s_probe"] = time.time() - t0
    metrics["wall_s_train"] = time.time() - wall_t0
    metrics["ckpt"] = str(t5c_ckpt) if t5c_ckpt else "E7lock"
    metrics["ckpt_sha16"] = _sha16(t5c_ckpt) if t5c_ckpt and t5c_ckpt.exists() else E7SHA16
    metrics["bc_teacher"] = AinexT5CTeacherEnv.TEACHER_SRC
    apps = metrics.get("apps") or []
    rcf = metrics.get("ret_cf") or []
    sk = metrics.get("skate") or []
    tip = metrics.get("tip_ret") or []
    mid = metrics.get("mid") or []
    line = (
        f"### {tag} · seed={seed} · wall={metrics['wall_s_train']:.0f}s · sha16=`{metrics['ckpt_sha16']}`\n"
        f"- apps={apps[:2]} · ret_cf={rcf[:2]} · skate={sk[:2]} · tip_ret={tip[:2]}\n"
        f"- mid={mid[:2]} · ret_ok={metrics.get('ret_ok')} · ret_ok_both={metrics.get('ret_ok_both')}\n"
        f"- eps_steal={metrics.get('eps_steal')} maxG={metrics.get('maxG', [])[:4]} · "
        f"bc_teacher=`{metrics['bc_teacher']}`\n"
        f"- probe_wall={metrics['wall_s_probe']:.0f}s · exit={proc.returncode} · log=`{log_path.name}`\n"
    )
    _write_progress([line])
    (OUT / f"T5C_{tag}_metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    print(
        f"[T5C-eval] {tag} apps={apps[:2]} rcf={rcf[:2]} sk={sk[:2]} "
        f"ret_ok={metrics.get('ret_ok')} both={metrics.get('ret_ok_both')} "
        f"eps_steal={metrics.get('eps_steal')}",
        flush=True,
    )
    return metrics


def collect_teacher_demos(n_episodes: int, seed: int, n_envs: int = 1) -> tuple[np.ndarray, np.ndarray, dict]:
    """Roll oracle teacher; Prefer FAIL if any plant-XY credit (should be impossible)."""
    obs_list: list[np.ndarray] = []
    act_list: list[np.ndarray] = []
    plant_reject = 0
    teacher_n = 0
    plant_xy = False
    env = AinexT5CTeacherEnv(
        gait_t=0.75, seed=seed,
        episode_s=3.5, stand_hold=0.60, ramp_t=0.80,
        action_scale=0.07, use_vik=True, use_plant=True, use_cp=True,
        vx_cmd=-0.07, target_dx=0.08, rev_phase=True,
        clear_only_residual=True, vx_cmd_rand=True,
        vx_lo=-0.05, vx_hi=-0.03,
        use_slr=True, use_ank=True, use_dxb=True, dxb_cap=0.030,
        use_alip=True, use_dcm=True, use_ank_nmpc=True,
        n_replan=2, alip_dx_cap=0.030, alip_smooth=0.45,
        teacher_dx_cap=0.030, use_footstep_timing=True,
    )
    for ep in range(n_episodes):
        obs, _ = env.reset(seed=seed + 1000 + ep)
        done = False
        steps = 0
        while not done and steps < env.n_ctrl + 5:
            act = env.teacher_action()
            # honesty: teacher zeros when planted
            any_clear, _, _ = env._sole_clear()
            if not any_clear and float(np.linalg.norm(act)) > 1e-8:
                plant_xy = True
            obs_list.append(np.asarray(obs, dtype=np.float32))
            act_list.append(np.asarray(act, dtype=np.float32))
            obs, _r, term, trunc, info = env.step(act)
            done = bool(term or trunc)
            steps += 1
            if done:
                plant_reject += int(info.get("teacher_plant_reject", 0))
                teacher_n += int(info.get("teacher_n", 0))
    env.close()
    meta = {
        "n_demo": len(obs_list),
        "n_episodes": n_episodes,
        "teacher_src": AinexT5CTeacherEnv.TEACHER_SRC,
        "teacher_n": teacher_n,
        "teacher_plant_reject": plant_reject,
        "teacher_plant_xy_credit": plant_xy,
        "seed": seed,
    }
    if plant_xy:
        raise RuntimeError("Prefer FAIL: teacher banked plant XY as stepped")
    return np.stack(obs_list), np.stack(act_list), meta


def bc_fit_policy(model: PPO, obs: np.ndarray, acts: np.ndarray, *,
                  epochs: int = 25, batch: int = 256, lr: float = 3e-4) -> float:
    """Supervised MSE on actor mean — BC warm-start. Returns final loss."""
    device = model.device
    policy = model.policy
    opt = torch.optim.Adam(policy.parameters(), lr=lr)
    n = obs.shape[0]
    last_loss = None
    for ep in range(epochs):
        perm = np.random.permutation(n)
        losses = []
        for i in range(0, n, batch):
            idx = perm[i:i + batch]
            o = torch.as_tensor(obs[idx], device=device)
            a = torch.as_tensor(acts[idx], device=device)
            # SB3: get distribution mean
            features = policy.extract_features(o)
            if hasattr(policy, "mlp_extractor"):
                latent_pi, _ = policy.mlp_extractor(features)
                mean = policy.action_net(latent_pi)
            else:
                dist = policy.get_distribution(o)
                mean = dist.distribution.mean
            loss = torch.mean((mean - a) ** 2)
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(float(loss.item()))
        last_loss = float(np.mean(losses)) if losses else 0.0
        if (ep + 1) % 5 == 0 or ep == 0:
            print(f"[T5C-BC] epoch={ep+1}/{epochs} bc_loss={last_loss:.5f} n_demo={n}", flush=True)
    return float(last_loss if last_loss is not None else 0.0)


class T5CCallback(BaseCallback):
    def __init__(self, total_steps: int, eval_every: int, wall_s: float, seed: int,
                 seed_tag: str, wall_t0: float):
        super().__init__()
        self.total_steps = total_steps
        self.eval_every = eval_every
        self.wall_s = wall_s
        self.seed = seed
        self.seed_tag = seed_tag
        self.t0 = wall_t0
        self.last_eval = 0
        self.history: list[dict] = []
        self.best: dict | None = None
        self.stop_ret_ok = False
        self.probe_idx = 0

    def _curriculum(self) -> None:
        frac = min(1.0, self.num_timesteps / max(1, int(0.7 * self.total_steps)))
        target = 0.06 + 0.18 * frac   # Placo-short: 0.06 → 0.24
        ep_s = 3.0 + 2.5 * frac
        vx_lo = -0.05 - 0.04 * frac
        vx_hi = -0.03 - 0.02 * frac
        dxb = 0.025 + 0.010 * frac   # keep short
        # Decay BC mix over first 20% of steps
        bc_mix = max(0.0, 0.55 * (1.0 - self.num_timesteps / max(1, int(0.20 * self.total_steps))))
        for e in self.training_env.envs:
            env = e
            while hasattr(env, "env"):
                env = env.env
            if hasattr(env, "set_curriculum"):
                env.set_curriculum(target_dx=target, episode_s=ep_s, vx_lo=vx_lo, vx_hi=vx_hi)
            if hasattr(env, "dxb_cap"):
                env.dxb_cap = float(dxb)
            if hasattr(env, "teacher_dx_cap"):
                env.teacher_dx_cap = float(dxb)
            if hasattr(env, "set_bc_mix"):
                env.set_bc_mix(bc_mix)

    def _on_step(self) -> bool:
        self._curriculum()
        if time.time() - self.t0 >= self.wall_s:
            print(f"[T5C] seed={self.seed} wall limit {self.wall_s}s at steps={self.num_timesteps}", flush=True)
            return False
        if self.num_timesteps - self.last_eval >= self.eval_every and self.num_timesteps > 0:
            self.last_eval = self.num_timesteps
            self.probe_idx += 1
            tag = f"T5C_s{self.seed}_{self.probe_idx:02d}"
            ckpt = OUT / f"ppo_t5c_seed{self.seed}_step{self.num_timesteps}"
            self.model.save(str(ckpt))
            zip_path = Path(str(ckpt) + ".zip")
            _write_progress([
                f"### train land · seed={self.seed} · steps={self.num_timesteps} · "
                f"wall={time.time()-self.t0:.0f}s · ckpt=`{zip_path.name}` · Prefer FAIL `{tag}`\n"
            ])
            metrics = run_gate_q_probe(tag, zip_path, self.t0, seed=self.seed)
            metrics["steps"] = self.num_timesteps
            self.history.append(metrics)
            (OUT / f"t5c_seed{self.seed}_history.json").write_text(
                json.dumps(self.history, indent=2, default=str))
            score = self._rank(metrics)
            if self.best is None or score > self._rank(self.best):
                self.best = metrics
                self.model.save(str(OUT / f"ppo_t5c_seed{self.seed}_best"))
                _write_progress([
                    f"**best-so-far seed={self.seed}** → `{tag}` rank={score:.3f} "
                    f"sha16=`{metrics['ckpt_sha16']}` skate={metrics.get('skate')} "
                    f"rcf={metrics.get('ret_cf')} eps_steal={metrics.get('eps_steal')}\n"
                ])
            if metrics.get("ret_ok_both") and _bars_beat_e7lock(metrics):
                print(f"[T5C] seed={self.seed} ret_ok both under bars+ε — early stop", flush=True)
                self.stop_ret_ok = True
                self.model.save(str(OUT / f"ppo_t5c_seed{self.seed}_retok"))
                return False
        return True

    @staticmethod
    def _rank(m: dict) -> float:
        """Higher better. Attack bout0 skate p95 + ε; hold bout1 skate PASS + cf/apps."""
        rcf = m.get("ret_cf") or [0.0, 0.0]
        sk = m.get("skate") or [(1.0, 1.0), (1.0, 1.0)]
        apps = m.get("apps") or [0.0, 0.0]
        tip = m.get("tip_ret") or [0.0, 0.0]
        sk_pen = 0.0
        sk_under = 0.0
        # bout0 ATTACK (primary wall from T5B_02: p95 0.183)
        if len(sk) >= 1:
            a0, b0 = sk[0]
            sk_pen += max(0.0, a0 - 0.08) * 12.0 + max(0.0, b0 - 0.18) * 20.0
            if a0 <= 0.08 + 1e-9 and b0 <= 0.18 + 1e-9:
                sk_under += 8.0
            # near-miss credit for closing 0.183→0.18
            if b0 <= 0.183 + 1e-9:
                sk_under += 1.5 * max(0.0, 0.183 - b0) / 0.003
        # bout1 HOLD (must not trade T5B_02 PASS)
        if len(sk) >= 2:
            a1, b1 = sk[1]
            sk_pen += max(0.0, a1 - 0.08) * 14.0 + max(0.0, b1 - 0.18) * 16.0
            if a1 <= 0.08 + 1e-9 and b1 <= 0.18 + 1e-9:
                sk_under += 6.0
            else:
                sk_pen += 4.0  # Prefer FAIL if bout1 skate traded
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
        ret_b = 10.0 if m.get("ret_ok_both") else 0.0
        return sk_under + cf_s + tip_s + apps_s + ret_b - sk_pen - eps_pen


def train_one_seed(boot: Path, seed: int, steps: int, wall_s: float, eval_every: int,
                   n_envs: int, lr: float, bc_episodes: int, wall_t0: float) -> dict:
    """BC warm-start + PPO residual for one seed. Budget per seed."""
    seed_t0 = time.time()
    remaining = max(60.0, wall_s - (seed_t0 - wall_t0) * 0.0)  # per-seed wall from now
    # Enforce per-seed wall independently
    seed_wall = min(wall_s, 4.0 * 3600.0)

    _write_progress([
        f"## Seed {seed} start · {time.strftime('%Y-%m-%d %H:%M:%S %Z')}\n",
        f"- steps≤{steps} wall≤{seed_wall/3600:.2f}h · bc_episodes={bc_episodes}\n",
        f"- teacher_src=`{AinexT5CTeacherEnv.TEACHER_SRC}`\n",
    ])

    # --- BC phase ---
    print(f"[T5C] seed={seed} collecting teacher demos n_ep={bc_episodes} …", flush=True)
    obs_d, act_d, demo_meta = collect_teacher_demos(bc_episodes, seed=seed)
    if demo_meta["teacher_plant_xy_credit"]:
        raise RuntimeError("Prefer FAIL: teacher plant XY credit")
    _write_progress([
        f"- BC demos: n_demo={demo_meta['n_demo']} teacher_n={demo_meta['teacher_n']} "
        f"plant_reject={demo_meta['teacher_plant_reject']} "
        f"teacher_src=`{demo_meta['teacher_src']}`\n",
    ])
    (OUT / f"t5c_seed{seed}_bc_meta.json").write_text(json.dumps(demo_meta, indent=2))

    def make(rank):
        return make_t5c_env(
            gait_t=0.75, rank=rank, seed=seed,
            episode_s=3.5, stand_hold=0.60, ramp_t=0.80,
            action_scale=0.07, use_vik=True, use_plant=True, use_cp=True,
            vx_cmd=-0.07, target_dx=0.08, rev_phase=True,
            clear_only_residual=True, vx_cmd_rand=True,
            vx_lo=-0.05, vx_hi=-0.03,
            use_slr=True, use_ank=True, use_dxb=True, dxb_cap=0.030,
            use_alip=True, use_dcm=True, use_ank_nmpc=True,
            n_replan=2, alip_dx_cap=0.030, alip_smooth=0.45,
            teacher_dx_cap=0.030, use_footstep_timing=True,
        )

    env = DummyVecEnv([make(i) for i in range(n_envs)])
    model = PPO.load(str(boot), env=env, device="cpu")
    model.learning_rate = lr

    print(f"[T5C] seed={seed} BC fit …", flush=True)
    bc_loss = bc_fit_policy(model, obs_d, act_d, epochs=25, batch=256, lr=max(lr, 1e-4))
    bc_ckpt = OUT / f"ppo_t5c_seed{seed}_bc"
    model.save(str(bc_ckpt))
    _write_progress([f"- BC done: bc_loss={bc_loss:.5f} ckpt=`{bc_ckpt.name}.zip`\n"])
    (OUT / f"t5c_seed{seed}_bc_meta.json").write_text(json.dumps({
        **demo_meta, "bc_loss": bc_loss,
    }, indent=2))

    # Quick probe after BC (before residual RL)
    bc_zip = Path(str(bc_ckpt) + ".zip")
    bc_m = run_gate_q_probe(f"T5C_s{seed}_BC", bc_zip, wall_t0, seed=seed)
    bc_m["bc_loss"] = bc_loss
    bc_m["steps"] = 0

    print(
        f"[T5C] seed={seed} residual PPO start from BC lr={lr} steps≤{steps} wall≤{seed_wall}s",
        flush=True,
    )
    # Set initial BC mix on envs
    for e in env.envs:
        ee = e
        while hasattr(ee, "env"):
            ee = ee.env
        if hasattr(ee, "set_bc_mix"):
            ee.set_bc_mix(0.55)

    cb = T5CCallback(steps, eval_every, seed_wall, seed, f"s{seed}", seed_t0)
    model.learn(
        total_timesteps=steps,
        callback=cb,
        progress_bar=False,
        reset_num_timesteps=True,
    )
    elapsed = time.time() - seed_t0
    final = OUT / f"ppo_t5c_seed{seed}_final"
    model.save(str(final))
    final_zip = Path(str(final) + ".zip")

    fin_m = run_gate_q_probe(f"T5C_s{seed}_FINAL", final_zip, wall_t0, seed=seed)
    fin_m["steps"] = int(model.num_timesteps)
    fin_m["bc_loss"] = bc_loss
    cb.history.append(fin_m)
    (OUT / f"t5c_seed{seed}_history.json").write_text(
        json.dumps([bc_m] + cb.history, indent=2, default=str))

    best = cb.best or fin_m
    # Also consider BC probe if better
    if cb._rank(bc_m) > cb._rank(best):
        best = bc_m

    # Copy seed best near-miss
    if best.get("ckpt") and best["ckpt"] != "E7lock":
        src = Path(best["ckpt"])
        if src.exists():
            shutil.copy2(src, OUT / f"ppo_t5c_seed{seed}_best.zip")

    summary = {
        "seed": seed,
        "bc_loss": bc_loss,
        "bc_meta": demo_meta,
        "wall_s": elapsed,
        "steps": int(model.num_timesteps),
        "best": best,
        "ret_ok_early_stop": cb.stop_ret_ok,
        "history_n": len(cb.history),
    }
    (OUT / f"T5C_seed{seed}_DONE.json").write_text(json.dumps(summary, indent=2, default=str))
    _write_progress([
        f"## Seed {seed} DONE · wall={elapsed:.0f}s ({elapsed/3600:.2f}h) · steps={model.num_timesteps}\n",
        f"- best sha=`{best.get('ckpt_sha16')}` apps={best.get('apps')} rcf={best.get('ret_cf')} "
        f"skate={best.get('skate')} ret_ok={best.get('ret_ok')} eps_steal={best.get('eps_steal')}\n",
        f"- bc_loss={bc_loss:.5f} · early_stop_ret_ok={cb.stop_ret_ok}\n",
    ])
    env.close()
    return summary


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000, help="per-seed step cap")
    ap.add_argument("--wall-hours", type=float, default=4.0, help="per-seed wall hours")
    ap.add_argument("--eval-every", type=int, default=150_000)
    ap.add_argument("--seeds", type=str, default=",".join(str(s) for s in DEFAULT_SEEDS))
    ap.add_argument("--n-envs", type=int, default=2)
    ap.add_argument("--skip-r0", action="store_true")
    ap.add_argument("--lr", type=float, default=7e-5)
    ap.add_argument("--bc-episodes", type=int, default=40)
    args = ap.parse_args()

    seeds = [int(x) for x in args.seeds.split(",") if x.strip()]
    if len(seeds) < 2:
        raise SystemExit("cospec requires ≥2 seeds (prefer 3)")
    wall_s = args.wall_hours * 3600.0
    boot = _bootstrap()
    assert E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16, "E7lock rollback must stay present"

    _write_progress([
        f"## Start · {time.strftime('%Y-%m-%d %H:%M:%S %Z')}\n",
        f"- bootstrap=`{boot.name}` sha16=`{_sha16(boot)}`\n",
        f"- family=**T5-C TEACHER-BC+MULTI-SEED** · teacher=`{AinexT5CTeacherEnv.TEACHER_SRC}`\n",
        f"- seeds={seeds} · steps≤{args.steps}/seed · wall≤{args.wall_hours}h/seed\n",
        f"- NOT reopening R*/S1–S3/T5-A/T5-B twins · cancel×0.70 · clear-only · soft-pass NEVER\n",
        f"- companion md5 `59cc408eda07037a58f92ad27da045d6` · walk plant `fc94709c84f5598d4474ecfc4bb41fdc`\n",
        f"- attack: bout0 skate p95≤0.18 + ε kill · hold bout1 skate PASS + cf≥0.55 + apps\n",
        f"- eval_every={args.eval_every} n_envs={args.n_envs} lr={args.lr} bc_ep={args.bc_episodes}\n",
    ])

    t0 = time.time()
    if not args.skip_r0:
        print("[T5C] T5C_R0 = E7lock baseline (flags OFF, skip-video)", flush=True)
        r0 = run_gate_q_probe("T5C_R0", None, t0, seed=None)
        r0_ok = (
            len(r0.get("apps", [])) >= 2
            and abs(r0["apps"][0] - 0.548) < 0.02
            and abs(r0["apps"][1] - 0.630) < 0.05
        )
        _write_progress([
            f"**T5C_R0 ≡ E7lock:** {'YES' if r0_ok else 'CHECK'} apps={r0.get('apps')} "
            f"rcf={r0.get('ret_cf')} skate={r0.get('skate')}\n"
        ])
        (OUT / "T5C_R0_metrics.json").write_text(json.dumps(r0, indent=2, default=str))

        if T5B_02.exists():
            print("[T5C] T5C_T5B_02 = T5B_02 near-miss dual-ckpt note (via GATE_Q_T5C_CKPT)", flush=True)
            # Score T5B_02 through T5C dual-ckpt path for fair compare
            t5b02 = run_gate_q_probe("T5C_T5B_02", T5B_02, t0, seed=None)
            _write_progress([
                f"**T5C_T5B_02 note:** apps={t5b02.get('apps')} rcf={t5b02.get('ret_cf')} "
                f"skate={t5b02.get('skate')} ret_ok={t5b02.get('ret_ok')} "
                f"eps_steal={t5b02.get('eps_steal')}\n"
            ])

    seed_summaries = []
    global_best = None
    for seed in seeds:
        print(f"\n========== T5C SEED {seed} ==========", flush=True)
        summary = train_one_seed(
            boot, seed, args.steps, wall_s, args.eval_every,
            args.n_envs, args.lr, args.bc_episodes, t0,
        )
        seed_summaries.append(summary)
        b = summary.get("best") or {}
        if global_best is None or T5CCallback._rank(b) > T5CCallback._rank(global_best):
            global_best = b
        # Early global stop if any seed hits ret_ok both honestly
        if summary.get("ret_ok_early_stop") and _bars_beat_e7lock(b):
            print(f"[T5C] seed={seed} honest ret_ok both — stopping remaining seeds", flush=True)
            break

    elapsed = time.time() - t0
    best = global_best or {}
    beat = _bars_beat_e7lock(best) if best else False
    installed = False
    installed_sha = None
    if beat and best.get("ckpt") and best["ckpt"] != "E7lock":
        src = Path(best["ckpt"])
        if src.exists():
            shutil.copy2(src, E7BEST)
            # Also keep locked rollback untouched; update scorer tip only if CKPT_SHA16 points at best
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
            installed_sha = new_sha
            _write_progress([
                f"## HONEST BEAT — installed sha16=`{new_sha}` into ppo_gate_e_best + CKPT_SHA16\n"
                f"- rollback locked remains `{E7SHA16}` at `ppo_gate_e_locked.zip`\n"
            ])

    # Save global best near-miss zip
    if best.get("ckpt") and best["ckpt"] != "E7lock":
        src = Path(best["ckpt"])
        if src.exists():
            shutil.copy2(src, OUT / "ppo_t5c_best.zip")
            tag = best.get("tag", "near_miss")
            shutil.copy2(src, OUT / f"ppo_t5c_best_{tag}_near_miss.zip")

    # Disposition
    if beat:
        disposition = "ret_ok both — honest beat"
    else:
        sk = best.get("skate") or []
        rcf = best.get("ret_cf") or []
        near = (
            bool(rcf) and min(rcf[:2] or [0]) >= 0.50
            and any((s[0] <= 0.09 and s[1] <= 0.20) for s in sk[:2])
        )
        disposition = "near-miss" if near else "Prefer FAIL train wall"

    # Aggregate history
    all_hist = []
    for s in seeds:
        hp = OUT / f"t5c_seed{s}_history.json"
        if hp.exists():
            all_hist.extend(json.loads(hp.read_text()))
    (OUT / "t5c_train_history.json").write_text(json.dumps(all_hist, indent=2, default=str))

    done = {
        "disposition": disposition,
        "family": "T5-C TEACHER-BC+MULTI-SEED",
        "bootstrap": str(boot),
        "bootstrap_sha16": _sha16(boot),
        "teacher_src": AinexT5CTeacherEnv.TEACHER_SRC,
        "opts": ["BC_TEACHER", "MULTI_SEED", "FOOTSTEP", "ALIP", "DCM", "ANK-NMPC", "SLR", "DXB", "ANK"],
        "seeds": seeds,
        "seeds_completed": [s["seed"] for s in seed_summaries],
        "wall_s_total": elapsed,
        "steps_per_seed_cap": args.steps,
        "wall_h_per_seed_cap": args.wall_hours,
        "seed_summaries": [
            {
                "seed": s["seed"],
                "steps": s["steps"],
                "wall_s": s["wall_s"],
                "bc_loss": s["bc_loss"],
                "ret_ok_early_stop": s["ret_ok_early_stop"],
                "best_tag": (s.get("best") or {}).get("tag"),
                "best_sha16": (s.get("best") or {}).get("ckpt_sha16"),
                "best_skate": (s.get("best") or {}).get("skate"),
                "best_ret_cf": (s.get("best") or {}).get("ret_cf"),
                "best_ret_ok": (s.get("best") or {}).get("ret_ok"),
                "best_eps_steal": (s.get("best") or {}).get("eps_steal"),
            }
            for s in seed_summaries
        ],
        "best": best,
        "installed_sha16": installed_sha,
        "frozen_sha16_kept": E7SHA16 if not installed else None,
        "soft_pass": False,
        "bars_kept": True,
        "vs_E7lock": "honest_beat" if beat else "Prefer FAIL (no ret_ok both under bars+ε)",
        "vs_T5B_02": "see SCORE packet",
    }
    (OUT / "T5C_DONE.json").write_text(json.dumps(done, indent=2, default=str))
    _write_progress([
        f"## DONE · disposition=**{disposition}** · wall={elapsed:.0f}s ({elapsed/3600:.2f}h)\n",
        f"- seeds={seeds} completed={[s['seed'] for s in seed_summaries]}\n",
        f"- installed_new_sha16={installed} · best_sha=`{best.get('ckpt_sha16')}`\n",
        f"- best apps={best.get('apps')} rcf={best.get('ret_cf')} skate={best.get('skate')} "
        f"ret_ok={best.get('ret_ok')} eps_steal={best.get('eps_steal')}\n",
        f"- soft_pass=false · bars_kept=true · frozen_E7lock=`{E7SHA16}`\n",
    ])
    print(
        f"=== T5C DONE === {disposition} wall={elapsed:.0f}s seeds={seeds} "
        f"installed={installed} best_sha={best.get('ckpt_sha16')}",
        flush=True,
    )


if __name__ == "__main__":
    main()
