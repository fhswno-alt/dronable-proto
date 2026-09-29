#!/usr/bin/env python3
"""Render continuous mp4 for a PPO residual episode."""
from __future__ import annotations
import os, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "scripts" / "learned_gate_e")]
os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco as mj
from stable_baselines3 import PPO
from env_ainex import AinexResidualEnv


def render(tag: str, gait_t: float, ckpt: str | None, out: Path, episode_s: float = 9.0):
    env = AinexResidualEnv(gait_t=gait_t, episode_s=episode_s, action_scale=0.07)
    pol = PPO.load(ckpt) if ckpt else None
    obs, _ = env.reset()
    # offscreen renderer
    w, h = 640, 480
    r = mj.Renderer(env.model, height=h, width=w)
    frames = []
    cam = mj.MjvCamera()
    mj.mjv_defaultCamera(cam)
    cam.type = mj.mjtCamera.mjCAMERA_FREE
    cam.distance = 1.05
    cam.azimuth = 90
    cam.elevation = -15
    cam.lookat[:] = [0.0, 0.0, 0.22]
    done = False
    while not done:
        if pol is None:
            action = np.zeros(12, dtype=np.float32)
        else:
            action, _ = pol.predict(obs, deterministic=True)
        obs, _, term, trunc, info = env.step(action)
        done = term or trunc
        cam.lookat[0] = float(env.data.qpos[0])
        r.update_scene(env.data, camera=cam)
        frames.append(r.render().copy())
    r.close()
    out.parent.mkdir(parents=True, exist_ok=True)
    # write via imageio / opencv / ffmpeg
    try:
        import imageio.v2 as imageio
        imageio.mimsave(str(out), frames, fps=50)
    except Exception:
        # fallback ffmpeg raw
        import tempfile, subprocess
        tmp = Path(tempfile.mkdtemp()) / "f"
        tmp.mkdir(exist_ok=True) if False else None
        raw = out.with_suffix(".rawdir")
        raw.mkdir(exist_ok=True)
        for i, fr in enumerate(frames):
            from PIL import Image
            Image.fromarray(fr).save(raw / f"{i:05d}.png")
        subprocess.run([
            "ffmpeg", "-y", "-framerate", "50", "-i", str(raw / "%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out),
        ], check=True, capture_output=True)
    print(f"wrote {out} nframes={len(frames)} dx={info.get('dx')}", flush=True)
    return out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--gait-t", type=float, required=True)
    ap.add_argument("--ckpt", default=None)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    render(args.tag, args.gait_t, args.ckpt, Path(args.out))
