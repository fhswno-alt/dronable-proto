#!/usr/bin/env python3
"""PPO residual Gate E trainer — curriculum T=0.88→0.75, kit HX 1.0×, M145 only."""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts" / "learned_gate_e"))

os.environ.setdefault("MUJOCO_GL", "glfw")

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv

from env_ainex import make_env  # noqa: E402
from eval_policy import run_episode, run_ol_via_walk_gait  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"
OUT.mkdir(parents=True, exist_ok=True)


class CurriculumCallback(BaseCallback):
    def __init__(self, total_steps: int, eval_every: int, wall_limit_s: float, verbose=0):
        super().__init__(verbose)
        self.total_steps = total_steps
        self.eval_every = eval_every
        self.wall_limit_s = wall_limit_s
        self.t0 = time.time()
        self.best_ge = False
        self.history = []
        self.last_eval = 0
        self.best_stx = 9.0

    def _gait_t(self) -> float:
        frac = min(1.0, self.num_timesteps / max(1, int(0.7 * self.total_steps)))
        return float(0.88 - 0.13 * frac)

    def _on_step(self) -> bool:
        gt = self._gait_t()
        for e in self.training_env.envs:
            env = e
            while hasattr(env, "env"):
                env = env.env
            if hasattr(env, "set_gait_t"):
                env.set_gait_t(gt)

        if time.time() - self.t0 >= self.wall_limit_s:
            print(f"[train] wall limit {self.wall_limit_s}s hit at steps={self.num_timesteps}", flush=True)
            return False

        if (self.num_timesteps - self.last_eval >= self.eval_every
                and self.num_timesteps > 0):
            self.last_eval = self.num_timesteps
            self._eval()
            if self.best_ge:
                print("[train] Gate E unlocked — early stop", flush=True)
                return False
        return True

    def _eval(self) -> None:
        ckpt = OUT / f"ppo_step{self.num_timesteps}"
        self.model.save(str(ckpt))
        print(f"[eval] steps={self.num_timesteps} gait_t_train≈{self._gait_t():.3f}", flush=True)
        r88 = run_episode(tag=f"RL_ckpt{self.num_timesteps}_T88",
                          gait_t=0.88, policy=self.model, episode_s=9.0)
        r75 = run_episode(tag=f"RL_ckpt{self.num_timesteps}_T75",
                          gait_t=0.75, policy=self.model, episode_s=9.0)
        rec = {
            "steps": self.num_timesteps,
            "wall_s": time.time() - self.t0,
            "t88_ok": r88.get("t88_ok"),
            "gate_e": r75.get("gate_e"),
            "t88": {k: r88[k] for k in ("tip", "dx", "bout_min", "stx_mean", "stx_p95", "skate", "hip_corr", "t88_ok")},
            "t75": {k: r75[k] for k in ("tip", "dx", "bout_min", "stx_mean", "stx_p95", "skate", "hip_corr", "gate_e")},
            "ckpt": str(ckpt) + ".zip",
        }
        self.history.append(rec)
        (OUT / "train_history.json").write_text(json.dumps(self.history, indent=2))
        if r75["stx_mean"] < self.best_stx and r88.get("t88_ok"):
            self.best_stx = r75["stx_mean"]
            self.model.save(str(OUT / "ppo_best_stx"))
        if r88.get("t88_ok") and r75.get("gate_e"):
            self.best_ge = True
            self.model.save(str(OUT / "ppo_gate_e_best"))


def _fmt_row(r: dict) -> str:
    ge_s = "PASS" if r.get("gate_e") else ("n/a" if r.get("T", 1) > 0.75 else "FAIL")
    t88 = r.get("t88_ok")
    t88s = "yes" if t88 else ("no" if t88 is False else "—")
    return (
        f"| {r.get('tag')} | {r.get('policy','?')} | {r.get('T',0):.2f} | {r.get('tip',0):.2f} | "
        f"{r.get('dx',0):+.3f} | {r.get('bout_min',0):.3f} | {r.get('stx_mean',0):.3f} | "
        f"{r.get('stx_p95',0):.3f} | {str(r.get('skate')).lower()} | {ge_s} | {t88s} |"
    )


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--wall-hours", type=float, default=4.0)
    ap.add_argument("--n-envs", type=int, default=1)
    ap.add_argument("--eval-every", type=int, default=100_000)
    ap.add_argument("--seed", type=int, default=27)
    ap.add_argument("--skip-baseline", action="store_true")
    args = ap.parse_args()

    wall_s = args.wall_hours * 3600.0
    print(f"[train] steps≤{args.steps} wall≤{wall_s}s n_envs={args.n_envs}", flush=True)

    table = []
    if not args.skip_baseline:
        print("[baseline] RL00 via walk_gait (authoritative)", flush=True)
        try:
            b88 = run_ol_via_walk_gait("RL00_OL_T88", 0.88, video=True)
            b75 = run_ol_via_walk_gait("RL00_OL_T75", 0.75, video=True)
            table.extend([b88, b75])
        except Exception as e:
            print(f"[baseline] walk_gait failed ({e}); env OL fallback", flush=True)
            b88 = run_episode(tag="RL00_OL_T88", gait_t=0.88, policy=None)
            b75 = run_episode(tag="RL00_OL_T75", gait_t=0.75, policy=None)
            table.extend([b88, b75])
        # also env OL for apples-to-apples with policy eval path
        run_episode(tag="RL00_envOL_T88", gait_t=0.88, policy=None)
        run_episode(tag="RL00_envOL_T75", gait_t=0.75, policy=None)

    def make(rank):
        return make_env(
            gait_t=0.88, rank=rank, seed=args.seed,
            episode_s=6.0, use_vik=True, stand_hold=0.80, ramp_t=1.40,
        )

    env = DummyVecEnv([make(i) for i in range(args.n_envs)])

    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=256,
        n_epochs=8,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        verbose=1,
        seed=args.seed,
        policy_kwargs=dict(net_arch=[64, 64]),
        device="cpu",
    )

    cb = CurriculumCallback(args.steps, args.eval_every, wall_s)
    t0 = time.time()
    model.learn(total_timesteps=args.steps, callback=cb, progress_bar=False)
    elapsed = time.time() - t0
    final_ckpt = OUT / "ppo_final"
    model.save(str(final_ckpt))
    print(f"[train] done steps={model.num_timesteps} wall={elapsed:.1f}s", flush=True)

    # Final eval
    f88 = run_episode(tag="RL01_final_T88", gait_t=0.88, policy=model, episode_s=9.0)
    f75 = run_episode(tag="RL01_final_T75", gait_t=0.75, policy=model, episode_s=9.0)
    table.extend([f88, f75])

    # Collect ckpt rows
    for rec in cb.history:
        for suffix in ("_T88", "_T75"):
            tag = f"RL_ckpt{rec['steps']}{suffix}"
            p = ITER / f"{tag}_ss.json"
            if p.exists():
                table.append(json.loads(p.read_text()))

    # best stx ckpt if exists
    best_path = OUT / "ppo_best_stx.zip"
    if best_path.exists():
        from stable_baselines3 import PPO as PPOLoad
        best = PPOLoad.load(str(best_path))
        b88p = run_episode(tag="RL02_beststx_T88", gait_t=0.88, policy=best, episode_s=9.0)
        b75p = run_episode(tag="RL02_beststx_T75", gait_t=0.75, policy=best, episode_s=9.0)
        table.extend([b88p, b75p])

    seen = set()
    uniq = []
    for r in table:
        t = r.get("tag")
        if t in seen:
            continue
        seen.add(t)
        uniq.append(r)

    (ITER / "LEARNED_GATE_E_TABLE.json").write_text(json.dumps(uniq, indent=2, default=str))
    ge = any(r.get("gate_e") for r in uniq)
    outcome = "GATE_E_UNLOCK" if ge else "HARD_FALSIFIED"

    rows = "\n".join(_fmt_row(r) for r in uniq)
    t88_final = "YES" if f88.get("t88_ok") else "NO"
    note = f'''# Learned / RL Gate E (sim) — PPO residual

**When:** Sun 27 Sep 2026 Europe/London (BST)
**Role:** Founding Controls
**Cospec:** `docs/LEARNED_GATE_E_SIM_COSPEC.md`
**Plant:** locked `ainex_controls_m2_145.xml`
**Authority:** k_auth=1.0; assist/freeze/xfrc OFF
**Policy:** PPO MLP [64,64] residual Δctrl on 12 legs (±0.08 rad); open-loop GRO01 + ankle CoP + CP swing + VIK ankle + plant damper
**Curriculum:** T 0.88→0.75 over 70% of budget
**Budget:** steps≤{args.steps}, wall≤{args.wall_hours}h; **actual steps={model.num_timesteps}, wall={elapsed:.0f}s ({elapsed/3600:.2f}h)**
**Non-claim:** sim only — not Pi/Orin/NN-first kit

## Verdict

| Claim | Result |
|-------|--------|
| Final policy T88-ok | **{t88_final}** |
| Gate E at T≤0.75 | **{"YES" if ge else "NO"}** |
| Outcome | **learned Gate E {outcome.replace("_", " ")}** |

{"Gate E UNLOCKED — freeze checkpoint. Still sim-only; no Pi claim." if ge else "Learned / RL Gate E **HARD-FALSIFIED** under kit HX on M145 after budget."}

- Final checkpoint: `{final_ckpt}.zip`
- Best stx (T88-ok): `{OUT / "ppo_best_stx.zip"}` (if created)
- Gate E best: `{OUT / "ppo_gate_e_best.zip"}` (if created)
- History: `{OUT / "train_history.json"}`

## Table

| Tag | policy | T | tip | dx | bout_min | stx mean | stx p95 | skate | Gate E | T88 ok |
|-----|--------|---|-----|-----|----------|----------|---------|-------|--------|--------|
{rows}

## Stop rule

{"**Sim learned policy unlocks Gate E** — no Pi claim." if ge else "**Learned Gate E HARD-FALSIFIED.** Classic+learned falsifier stack complete under kit HX on M145. Next lever needs Dave direction."}

## Artifacts

- `scripts/learned_gate_e/env_ainex.py`, `train_ppo.py`, `eval_policy.py`
- `LEARNED_GATE_E_TABLE.json`, `LEARNED_GATE_E_NOTE.md`
- Checkpoints under `previews/ainex_walk/iterate/learned_gate_e/`
'''
    (ITER / "LEARNED_GATE_E_NOTE.md").write_text(note)
    print(f"=== FINAL === {outcome} steps={model.num_timesteps} wall={elapsed:.0f}s", flush=True)
    env.close()


if __name__ == "__main__":
    main()
