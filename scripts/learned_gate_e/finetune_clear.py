#!/usr/bin/env python3
"""Short fine-tune from near-Gate-E ckpt with clearance-shaped reward."""
from __future__ import annotations
import json, os, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "scripts" / "learned_gate_e")]
os.environ.setdefault("MUJOCO_GL", "glfw")

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv
from env_ainex import make_env
from eval_policy import run_episode

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"


class CB(BaseCallback):
    def __init__(self, total, eval_every, wall_s):
        super().__init__()
        self.total = total
        self.eval_every = eval_every
        self.wall_s = wall_s
        self.t0 = time.time()
        self.last = 0
        self.history = []
        self.best_ge = False

    def _on_step(self):
        # hold T=0.75 for fine-tune (already annealed)
        for e in self.training_env.envs:
            env = e
            while hasattr(env, "env"):
                env = env.env
            if hasattr(env, "set_gait_t"):
                env.set_gait_t(0.75)
        if time.time() - self.t0 >= self.wall_s:
            return False
        if self.num_timesteps - self.last >= self.eval_every and self.num_timesteps > 0:
            self.last = self.num_timesteps
            ckpt = OUT / f"ppo_ft{self.num_timesteps}"
            self.model.save(str(ckpt))
            r88 = run_episode(tag=f"RL_ft{self.num_timesteps}_T88", gait_t=0.88, policy=self.model)
            r75 = run_episode(tag=f"RL_ft{self.num_timesteps}_T75", gait_t=0.75, policy=self.model)
            rec = {"steps": self.num_timesteps, "wall_s": time.time()-self.t0,
                   "t88_ok": r88.get("t88_ok"), "gate_e": r75.get("gate_e"),
                   "t88": r88, "t75": r75, "ckpt": str(ckpt)+".zip"}
            self.history.append(rec)
            (OUT / "finetune_history.json").write_text(json.dumps([
                {k: (v if k not in ("t88","t75") else {kk:v[kk] for kk in ("tip","dx","bout_min","stx_mean","stx_p95","skate","hip_corr","peak_clear_L","peak_clear_R","gate_e","t88_ok") if kk in v})
                 for k,v in rec.items()}
                for rec in self.history
            ], indent=2, default=str))
            print(f"[ft-eval] clearL/R={r75.get('peak_clear_L'):.4f}/{r75.get('peak_clear_R'):.4f} hip={r75.get('hip_corr'):.3f} ge={r75.get('gate_e')} t88={r88.get('t88_ok')}", flush=True)
            if r88.get("t88_ok") and r75.get("gate_e"):
                self.best_ge = True
                self.model.save(str(OUT / "ppo_gate_e_best"))
                return False
            if r75.get("gate_e"):
                # Gate E without T88 — still record
                self.model.save(str(OUT / "ppo_gate_e_not88"))
        return True


def main():
    ckpt = OUT / "ppo_ft1700001"
    if not (OUT / "ppo_ft1700001.zip").exists():
        # find latest
        zips = sorted(OUT.glob("ppo_step*.zip"), key=lambda p: p.stat().st_mtime)
        ckpt = zips[-1].with_suffix("")
        print("using", ckpt)
    env = DummyVecEnv([make_env(gait_t=0.75, rank=0, seed=42, episode_s=6.0,
                                use_vik=True, stand_hold=0.80, ramp_t=1.40,
                                action_scale=0.07)])
    model = PPO.load(str(ckpt), env=env, device="cpu")
    # slightly lower LR for fine-tune
    model.learning_rate = 1e-4
    cb = CB(total=300_000, eval_every=40_000, wall_s=0.75 * 3600)
    t0 = time.time()
    model.learn(total_timesteps=300_000, callback=cb, progress_bar=False, reset_num_timesteps=False)
    elapsed = time.time() - t0
    model.save(str(OUT / "ppo_ft_final"))
    f88 = run_episode(tag="RL03_ft_final_T88", gait_t=0.88, policy=model)
    f75 = run_episode(tag="RL03_ft_final_T75", gait_t=0.75, policy=model)
    print(f"FT done wall={elapsed:.0f}s ge={f75.get('gate_e')} clear={f75.get('peak_clear_L'):.4f}/{f75.get('peak_clear_R'):.4f}", flush=True)
    (OUT / "finetune_done.json").write_text(json.dumps({"wall_s": elapsed, "f88": f88, "f75": f75, "history": cb.history}, indent=2, default=str))
    env.close()

if __name__ == "__main__":
    main()
