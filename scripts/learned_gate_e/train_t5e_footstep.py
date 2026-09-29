#!/usr/bin/env python3
"""T5-E FOOTSTEP-SEQ — Prefer FAIL cospec (Walk This Way / Placo short |dx_back|).

Cospec: docs/GATE_Q_AI_COSPEC_T5E.md (AI no-veto · Dave greenlit ~20:29 BST)
Family: external retreat ordered clear TD sequence PRIMARY;
        residual = clear mid-swing + late-swing TDVM/SLR corrector only.
Bootstrap corrector: T5D1_13 near-miss sha16 45799fc897157a3a (corrector only).
Sequence outer is NEW — Prefer FAIL if seq_idle / residual-primary / D1/D2 twin.
Budget: ≤4h wall OR ≤2e6 steps (first hit). Soft-pass NEVER. H2 OUT.
Eval: dual-ckpt approach=E7lock · retreat=T5E via GATE_Q_T5E_CKPT.
Attack: beat T5D1_13 maxG 0.055 while HOLDING skate ≤0.08/0.18, ret_ok both,
        cf≥0.55, tip≥8, apps ~0.548/0.630. Prefer FAIL if skate/ret_ok traded.
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

from env_ainex import AinexT5EFootstepSeqEnv, make_t5e_env  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
OUT = ITER / "learned_gate_e"
OUT.mkdir(parents=True, exist_ok=True)
PROGRESS = ITER / "GATE_Q_T5E_PROGRESS.md"
E7LOCK = OUT / "ppo_gate_e_locked.zip"
E7BEST = OUT / "ppo_gate_e_best.zip"
E7SHA16 = "9ffaa1a21b607bf6"
T5D1_NM = OUT / "ppo_t5d1_best_T5D1_13_near_miss.zip"
T5D1_NM_SHA = "45799fc897157a3a"
T5D2_NM = OUT / "ppo_t5d2_best_T5D2_13_near_miss.zip"
T5C_S43_BC = OUT / "ppo_t5c_seed43_bc.zip"
T5C_S43_BC_SHA = "5e0c0726953840ed"
T5C_S47_03 = OUT / "ppo_t5c_best_T5C_s47_03_near_miss.zip"
T5B_02 = OUT / "ppo_t5b_best_T5B_02_near_miss.zip"
PY = str(ROOT / ".venv" / "bin" / "python")


def _sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def _bootstrap() -> tuple[Path, str]:
    """Corrector warm-start only — sequence outer is NEW. Log which."""
    if T5D1_NM.exists():
        sha = _sha16(T5D1_NM)
        assert sha == T5D1_NM_SHA, f"T5D1_13 sha mismatch {sha}≠{T5D1_NM_SHA}"
        print(f"[T5E] bootstrap_corrector=T5D1_13 {T5D1_NM.name} sha16={sha}", flush=True)
        return T5D1_NM, f"T5D1_13:{sha}"
    if T5C_S43_BC.exists():
        sha = _sha16(T5C_S43_BC)
        assert sha == T5C_S43_BC_SHA, f"T5C_s43_BC sha mismatch {sha}≠{T5C_S43_BC_SHA}"
        print(f"[T5E] bootstrap_corrector=T5C_s43_BC {T5C_S43_BC.name} sha16={sha}", flush=True)
        return T5C_S43_BC, f"T5C_s43_BC:{sha}"
    if T5C_S47_03.exists():
        sha = _sha16(T5C_S47_03)
        print(f"[T5E] bootstrap_corrector=T5C_s47_03 {T5C_S47_03.name} sha16={sha}", flush=True)
        return T5C_S47_03, f"T5C_s47_03:{sha}"
    if T5B_02.exists():
        sha = _sha16(T5B_02)
        print(f"[T5E] bootstrap_corrector=T5B_02 {T5B_02.name} sha16={sha}", flush=True)
        return T5B_02, f"T5B_02:{sha}"
    if E7LOCK.exists() and _sha16(E7LOCK) == E7SHA16:
        print(f"[T5E] bootstrap_corrector=E7lock sha16={E7SHA16}", flush=True)
        return E7LOCK, f"E7lock:{E7SHA16}"
    raise FileNotFoundError("No T5D1_13 / T5C_s43_BC / T5C_s47_03 / T5B_02 / E7lock bootstrap")


def _write_progress(lines: list[str]) -> None:
    hdr = (
        "# Gate Q T5-E FOOTSTEP-SEQ — live progress\n\n"
        f"**Cospec:** `docs/GATE_Q_AI_COSPEC_T5E.md` · Soft-pass **off** · Bars KEPT · H2 OUT\n"
        f"**Family:** OUTER=FOOTSTEP-SEQ (Placo-style short |dx_back| TD list) · residual=clear corrector only\n"
        f"**Bootstrap corrector:** T5D1_13 `{T5D1_NM_SHA}` if present · sequence outer NEW\n"
        f"**Budget:** ≤4h wall OR ≤2e6 steps (first hit) · Early stop on ret_ok both AND ε clean\n"
        f"**Dual-ckpt:** approach=E7lock · retreat=`GATE_Q_T5E_CKPT`\n"
        f"**Planted:** cancel×0.70 + vel-oppose · zero planted soft-XY\n"
        f"**Attack:** beat T5D1_13 maxG 0.055 while HOLDING skate/ret_ok (Prefer FAIL if traded)\n"
        f"**Rollback:** E7lock `{E7SHA16}` until Prefer-FAIL beat · Prefer FAIL if seq_idle\n\n"
    )
    body = "\n".join(lines) + "\n"
    if not PROGRESS.exists():
        PROGRESS.write_text(hdr + body)
        return
    prev = PROGRESS.read_text()
    if not prev.startswith("# Gate Q T5-E"):
        PROGRESS.write_text(hdr + body + ("\n" + prev if prev else ""))
    else:
        PROGRESS.write_text(prev.rstrip() + "\n\n" + body)


def _parse_probe(log_text: str) -> dict:
    out: dict = {
        "apps": [], "ret_cf": [], "skate": [], "tip_ret": [], "ret_ok": [],
        "mid": [], "eps": [], "cam_ret": [], "raw": {},
        "outer_F": [], "outer_T": [], "outer_n_replan": [], "outer_logged": False,
        "seq_idle": True, "seq_logged": False,
        "seq_err": [], "seq_n_track": [], "seq_n_advance": [],
        "seq_next_F": [], "seq_list_id": [], "seq_footholds": [],
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

    # T5E FOOTSTEP-SEQ logs (Prefer FAIL if missing / idle)
    for m in re.finditer(
        r"T5E_SEQ cyc\d+: list_id=(\S+) n=(\d+) idx=(\d+) "
        r"next_F=([0-9.\-eE]+) next_side=(\S+) next_err=([0-9.\-eE]+) "
        r"n_track=(\d+) n_advance=(\d+) seq_idle=(\w+)",
        log_text,
    ):
        out["seq_list_id"].append(m.group(1))
        out["seq_next_F"].append(float(m.group(4)))
        out["seq_err"].append(float(m.group(6)))
        out["seq_n_track"].append(int(m.group(7)))
        out["seq_n_advance"].append(int(m.group(8)))
        idle = m.group(9) in ("True", "true", "1")
        out["seq_logged"] = True
        out["seq_idle"] = bool(out.get("seq_idle", True) and idle) if out["seq_n_track"] else idle
        # once any bout has seq_idle=False, mark engaged
        if not idle:
            out["seq_idle"] = False
        out["outer_F"].append(float(m.group(4)))
        out["outer_n_replan"].append(int(m.group(7)))
        out["outer_logged"] = True
    for m in re.finditer(
        r"T5E_OUTER_FT cyc\d+: F=([0-9.\-eE]+).*?n_replan=(\d+).*?seq_idle=(\w+)",
        log_text,
    ):
        out["outer_F"].append(float(m.group(1)))
        out["outer_n_replan"].append(int(m.group(2)))
        out["outer_logged"] = True
        if m.group(3) in ("False", "false", "0"):
            out["seq_idle"] = False
            out["seq_logged"] = True

    # footholds=[...]
    for m in re.finditer(r"footholds=\[([^\]]*)\]", log_text):
        xs = [float(x) for x in re.findall(r"[0-9.\-eE]+", m.group(1))]
        if xs:
            out["seq_footholds"].append(xs)

    # Fallback: ALIP/T5E_SEQ plant suite also logs as T5E_SEQ tag
    if not out["outer_logged"]:
        for m in re.finditer(
            r"\[Q\] T5E_SEQ cyc\d+:.*?fp_prior=([0-9.\-eE]+).*?n_replan=(\d+)",
            log_text,
        ):
            out["outer_F"].append(float(m.group(1)))
            out["outer_n_replan"].append(int(m.group(2)))
            out["outer_logged"] = True

    out["outer_idle"] = bool(out.get("seq_idle", True)) or (
        (not out["outer_logged"])
        or (sum(out["outer_n_replan"]) < 2)
    )
    m = re.search(r"n_pass=(\d+)/(\d+)", log_text)
    out["n_pass"] = (int(m.group(1)), int(m.group(2))) if m else None
    out["ret_ok_both"] = bool(out["ret_ok"]) and all(out["ret_ok"][:2]) and len(out["ret_ok"]) >= 2
    # Recompute seq_idle: Prefer FAIL if never logged or all idle
    if out["seq_logged"] and out["seq_n_track"]:
        out["seq_idle"] = all(n < 2 for n in out["seq_n_track"][:2]) if out["seq_n_track"] else True
        # if any bout tracked, not fully idle
        if any(n >= 2 for n in out["seq_n_track"]):
            out["seq_idle"] = False
    return out


def _bars_beat_e7lock(m: dict) -> bool:
    """Honest beat: ret_ok both + apps held + plant-cam ≤ε + seq engaged. Soft-pass NEVER."""
    if not m.get("ret_ok_both"):
        return False
    if m.get("seq_idle") or m.get("outer_idle"):
        return False  # Prefer FAIL: seq idle / residual owns polarity / D1 twin
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
    # Hold D1 locomotor: maxG must beat 0.055 (ε clean already enforced via eps_steal)
    maxgs = m.get("maxG") or []
    if any(g > 0.02 + 1e-12 for g in maxgs[:4]):
        return False
    return True


def run_gate_q_probe(tag: str, t5e_ckpt: Path | None, wall_t0: float) -> dict:
    env = os.environ.copy()
    env["GATE_Q_SKIP_VIDEO"] = "1"
    env["MUJOCO_GL"] = "glfw"
    for k in (
        "GATE_Q_RET_CSF50", "GATE_Q_RET_ALIP_TVR", "GATE_Q_RET_REV_PHASE",
        "GATE_Q_RET_CLEAR_BURST", "GATE_Q_RET_BURST_PLANT_CAP",
        "GATE_Q_RET_MAX_PLANT_DWELL", "GATE_Q_RET_SWING_PLACE_M",
        "GATE_Q_T5D2_ICP", "GATE_Q_T5D1_OUTER_PRIMARY",
    ):
        env[k] = "0"
    for k in (
        "GATE_Q_T4_CKPT", "GATE_Q_T5_CKPT", "GATE_Q_T5B_CKPT", "GATE_Q_T5C_CKPT",
        "GATE_Q_T5D1_CKPT", "GATE_Q_T5D2_CKPT",
    ):
        env.pop(k, None)
    if t5e_ckpt is not None:
        env["GATE_Q_T5E_CKPT"] = str(t5e_ckpt.resolve())
        env["GATE_Q_T5E_VX_CMD"] = "-0.08"
        env["GATE_Q_T5E_FOOTSTEP_SEQ"] = "1"
        env["GATE_Q_T5E_DX_BACK"] = "0.028"
        env["GATE_Q_T5E_TARGET_DX"] = "0.18"
        env["GATE_Q_T5B_MODELBASE"] = "1"
    else:
        env.pop("GATE_Q_T5E_CKPT", None)
        env.pop("GATE_Q_T5E_FOOTSTEP_SEQ", None)
    log_path = ITER / f"GATE_Q_{tag}_PROBE.log"
    t0 = time.time()
    print(f"[T5E-eval] {tag} ckpt={t5e_ckpt} …", flush=True)
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
    metrics["ckpt"] = str(t5e_ckpt) if t5e_ckpt else "E7lock"
    metrics["ckpt_sha16"] = _sha16(t5e_ckpt) if t5e_ckpt and t5e_ckpt.exists() else E7SHA16
    metrics["outer_src"] = AinexT5EFootstepSeqEnv.OUTER_SRC
    apps = metrics.get("apps") or []
    rcf = metrics.get("ret_cf") or []
    sk = metrics.get("skate") or []
    tip = metrics.get("tip_ret") or []
    mid = metrics.get("mid") or []
    maxG = metrics.get("maxG") or []
    line = (
        f"### {tag} · wall={metrics['wall_s_train']:.0f}s · sha16=`{metrics['ckpt_sha16']}`\n"
        f"- apps={apps[:2]} · ret_cf={rcf[:2]} · skate={sk[:2]} · tip_ret={tip[:2]}\n"
        f"- mid={mid[:2]} · ret_ok={metrics.get('ret_ok')} · ret_ok_both={metrics.get('ret_ok_both')}\n"
        f"- eps_steal={metrics.get('eps_steal')} maxG={maxG[:4]}\n"
        f"- outer_src=`{metrics['outer_src']}` outer_logged={metrics.get('outer_logged')} "
        f"outer_idle={metrics.get('outer_idle')} outer_F={metrics.get('outer_F', [])[:4]}\n"
        f"- seq_logged={metrics.get('seq_logged')} seq_idle={metrics.get('seq_idle')} "
        f"next_F={metrics.get('seq_next_F', [])[:4]} err={metrics.get('seq_err', [])[:4]} "
        f"n_track={metrics.get('seq_n_track', [])[:4]} n_adv={metrics.get('seq_n_advance', [])[:4]} "
        f"list_id={metrics.get('seq_list_id', [])[:2]}\n"
        f"- probe_wall={metrics['wall_s_probe']:.0f}s · exit={proc.returncode} · log=`{log_path.name}`\n"
    )
    _write_progress([line])
    (OUT / f"T5E_{tag}_metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    print(
        f"[T5E-eval] {tag} apps={apps[:2]} rcf={rcf[:2]} sk={sk[:2]} "
        f"ret_ok={metrics.get('ret_ok')} both={metrics.get('ret_ok_both')} "
        f"seq_idle={metrics.get('seq_idle')} eps_steal={metrics.get('eps_steal')} "
        f"maxG={maxG[:4]}",
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
        use_slr=True, use_ank=True, use_dxb=True, dxb_cap=0.028,
        use_alip=True, use_dcm=True, use_ank_nmpc=True,
        n_replan=3, alip_dx_cap=0.028, alip_smooth=0.35,
        outer_dx_cap=0.028, outer_track_gain=0.14, corrector_scale=0.028,
        residual_primary_forbid=True,
        seq_dx_back=0.028, seq_n_max=10, seq_n_min=3,
        seq_lat_bos=0.0, seq_advance_err=0.018, seq_enabled=True,
    )


class T5ECallback(BaseCallback):
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
        dxb = 0.022 + 0.010 * frac  # SHORT_DX co-constraint (not sole ε lever)
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
            if hasattr(env, "seq_dx_back"):
                env.seq_dx_back = float(dxb)
            if hasattr(env, "action_scale") and env.action_scale > 0.04:
                env.action_scale = float(getattr(env, "corrector_scale", 0.028))

    def _on_step(self) -> bool:
        self._curriculum()
        if time.time() - self.t0 >= self.wall_s:
            print(f"[T5E] wall limit {self.wall_s}s at steps={self.num_timesteps}", flush=True)
            return False
        if self.num_timesteps - self.last_eval >= self.eval_every and self.num_timesteps > 0:
            self.last_eval = self.num_timesteps
            self.probe_idx += 1
            tag = f"T5E_{self.probe_idx:02d}"
            ckpt = OUT / f"ppo_t5e_step{self.num_timesteps}"
            self.model.save(str(ckpt))
            zip_path = Path(str(ckpt) + ".zip")
            _write_progress([
                f"### train land · steps={self.num_timesteps} · "
                f"wall={time.time()-self.t0:.0f}s · ckpt=`{zip_path.name}` · Prefer FAIL `{tag}`\n"
            ])
            metrics = run_gate_q_probe(tag, zip_path, self.t0)
            metrics["steps"] = self.num_timesteps
            self.history.append(metrics)
            (OUT / "t5e_train_history.json").write_text(
                json.dumps(self.history, indent=2, default=str))
            score = self._rank(metrics)
            if self.best is None or score > self._rank(self.best):
                self.best = metrics
                self.model.save(str(OUT / "ppo_t5e_best"))
                _write_progress([
                    f"**best-so-far** → `{tag}` rank={score:.3f} "
                    f"sha16=`{metrics['ckpt_sha16']}` skate={metrics.get('skate')} "
                    f"rcf={metrics.get('ret_cf')} seq_idle={metrics.get('seq_idle')} "
                    f"eps_steal={metrics.get('eps_steal')} maxG={(metrics.get('maxG') or [])[:4]}\n"
                ])
            # Early stop: ret_ok both AND ε clean (cospec)
            if metrics.get("ret_ok_both") and _bars_beat_e7lock(metrics):
                print("[T5E] ret_ok both under bars+ε+seq — early stop", flush=True)
                self.stop_ret_ok = True
                self.model.save(str(OUT / "ppo_t5e_retok"))
                return False
        return True

    @staticmethod
    def _rank(m: dict) -> float:
        """Rank: HOLD skate/ret_ok/cf/tip/apps first; then ε; Prefer FAIL seq_idle."""
        rcf = m.get("ret_cf") or [0.0, 0.0]
        sk = m.get("skate") or [(1.0, 1.0), (1.0, 1.0)]
        apps = m.get("apps") or [0.0, 0.0]
        tip = m.get("tip_ret") or [0.0, 0.0]
        maxgs = m.get("maxG") or [1.0]
        sk_pen = 0.0
        sk_under = 0.0
        for i, pair in enumerate(sk[:2]):
            a, b = pair
            w_mean = 14.0 if i == 0 else 16.0  # heavier than D2 — hold D1 wins
            w_p95 = 22.0 if i == 0 else 18.0
            sk_pen += max(0.0, a - 0.08) * w_mean + max(0.0, b - 0.18) * w_p95
            if a <= 0.08 + 1e-9 and b <= 0.18 + 1e-9:
                sk_under += 10.0 if i == 0 else 8.0
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
        # Attack maxG 0.055 — reward under, Prefer FAIL cosmetics alone
        eps_pen = 8.0 if m.get("eps_steal") else 0.0
        maxg0 = max(maxgs[:2]) if maxgs else 1.0
        eps_attack = 0.0
        if maxg0 <= 0.02 + 1e-12:
            eps_attack = 6.0
        elif maxg0 < 0.055:
            eps_attack = 3.0 * (1.0 - maxg0 / 0.055)
        else:
            eps_pen += 4.0 * (maxg0 - 0.055) / 0.055
        seq_pen = 10.0 if m.get("seq_idle") else 0.0
        seq_bonus = 4.0 if m.get("seq_logged") and not m.get("seq_idle") else 0.0
        ret_b = 12.0 if m.get("ret_ok_both") else 0.0
        # Prefer FAIL if skate/ret_ok traded for ε cosmetics (D2 lesson)
        traded = 0.0
        if m.get("eps_steal") is False or (maxg0 < 0.055):
            # ε improving — but if skate over or no ret_ok, heavy penalty
            sk_over = any(s[0] > 0.08 + 1e-9 or s[1] > 0.18 + 1e-9 for s in sk[:2])
            if sk_over or not m.get("ret_ok_both"):
                traded = 15.0
        return (
            sk_under + cf_s + tip_s + apps_s + ret_b + seq_bonus + eps_attack
            - sk_pen - eps_pen - seq_pen - traded
        )


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
        f"- family=**T5-E FOOTSTEP-SEQ** · OUTER=`{AinexT5EFootstepSeqEnv.OUTER_SRC}` · cite=`{AinexT5EFootstepSeqEnv.CITE}`\n",
        f"- residual=corrector-only scale=0.028 · clear≥2cm · planted cancel×0.70 · soft-pass NEVER\n",
        f"- steps≤{args.steps} · wall≤{args.wall_hours}h · seed={args.seed}\n",
        f"- NOT reopening R*/S1–S3/T5-A/B/C/T5-D1/D2 twins · no plant invent · H2 OUT\n",
        f"- companion md5 `59cc408eda07037a58f92ad27da045d6` · walk plant `fc94709c84f5598d4474ecfc4bb41fdc`\n",
        f"- dual-ckpt approach=E7lock · retreat=`GATE_Q_T5E_CKPT`\n",
        f"- eval_every={args.eval_every} n_envs={args.n_envs} lr={args.lr}\n",
        f"- Attack: plant-cam ε / Root B (beat maxG 0.055) while HOLDING skate/ret_ok\n",
        f"- Prefer FAIL if seq_idle / residual-primary / D1/D2 twin / skate traded for ε\n",
    ])

    t0 = time.time()
    if not args.skip_r0:
        print("[T5E] T5E_R0 = E7lock baseline (flags OFF)", flush=True)
        r0 = run_gate_q_probe("T5E_R0", None, t0)
        r0_ok = (
            len(r0.get("apps", [])) >= 2
            and abs(r0["apps"][0] - 0.548) < 0.02
            and abs(r0["apps"][1] - 0.630) < 0.05
        )
        _write_progress([
            f"**T5E_R0 ≡ E7lock:** {'YES' if r0_ok else 'CHECK'} apps={r0.get('apps')} "
            f"rcf={r0.get('ret_cf')} skate={r0.get('skate')}\n"
        ])
        if T5D1_NM.exists():
            print("[T5E] T5E_T5D1_13 note (via GATE_Q_T5E_CKPT)", flush=True)
            note13 = run_gate_q_probe("T5E_T5D1_13", T5D1_NM, t0)
            _write_progress([
                f"**T5E_T5D1_13 note:** apps={note13.get('apps')} rcf={note13.get('ret_cf')} "
                f"skate={note13.get('skate')} ret_ok={note13.get('ret_ok')} "
                f"seq_idle={note13.get('seq_idle')} outer_idle={note13.get('outer_idle')} "
                f"eps_steal={note13.get('eps_steal')} maxG={(note13.get('maxG') or [])[:4]}\n"
            ])
        if T5D2_NM.exists():
            print("[T5E] T5E_T5D2_13 note (via GATE_Q_T5E_CKPT)", flush=True)
            note12 = run_gate_q_probe("T5E_T5D2_13", T5D2_NM, t0)
            _write_progress([
                f"**T5E_T5D2_13 note:** apps={note12.get('apps')} rcf={note12.get('ret_cf')} "
                f"skate={note12.get('skate')} ret_ok={note12.get('ret_ok')} "
                f"seq_idle={note12.get('seq_idle')} eps_steal={note12.get('eps_steal')} "
                f"maxG={(note12.get('maxG') or [])[:4]}\n"
            ])

    def make(rank):
        kw = _env_kwargs(args.seed)
        return make_t5e_env(rank=rank, **kw)

    print(
        f"[T5E] residual PPO corrector start from {boot.name} "
        f"OUTER={AinexT5EFootstepSeqEnv.OUTER_SRC} cite={AinexT5EFootstepSeqEnv.CITE} "
        f"lr={args.lr} steps≤{args.steps} wall≤{wall_s}s",
        flush=True,
    )
    env = DummyVecEnv([make(i) for i in range(args.n_envs)])
    model = PPO.load(str(boot), env=env, device="cpu")
    model.learning_rate = args.lr

    for e in env.envs:
        ee = e
        while hasattr(ee, "env"):
            ee = ee.env
        if hasattr(ee, "action_scale"):
            ee.action_scale = 0.028
        if hasattr(ee, "corrector_scale"):
            ee.corrector_scale = 0.028

    cb = T5ECallback(args.steps, args.eval_every, wall_s, t0)
    model.learn(
        total_timesteps=args.steps,
        callback=cb,
        progress_bar=False,
        reset_num_timesteps=True,
    )
    elapsed = time.time() - t0
    final = OUT / "ppo_t5e_final"
    model.save(str(final))
    final_zip = Path(str(final) + ".zip")

    fin_m = run_gate_q_probe("T5E_FINAL", final_zip, t0)
    fin_m["steps"] = int(model.num_timesteps)
    cb.history.append(fin_m)
    (OUT / "t5e_train_history.json").write_text(
        json.dumps(cb.history, indent=2, default=str))

    best = cb.best or fin_m
    if best.get("ckpt") and best["ckpt"] != "E7lock":
        src = Path(best["ckpt"])
        if src.exists():
            shutil.copy2(src, OUT / "ppo_t5e_best.zip")
            tag = best.get("tag", "near_miss")
            shutil.copy2(src, OUT / f"ppo_t5e_best_{tag}_near_miss.zip")

    beat = _bars_beat_e7lock(best) if best else False
    installed = False
    installed_sha = None
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
        if best.get("seq_idle") or best.get("outer_idle"):
            disposition = "Prefer FAIL — seq_idle / residual polarity / D1-D2 twin risk"
        elif near:
            disposition = "near-miss Prefer FAIL"
        else:
            disposition = "Prefer FAIL train wall"
        outcome = "PREFER_FAIL_SCORED"

    maxG_best = (best.get("maxG") or [])[:4]
    done = {
        "outcome": outcome,
        "disposition": disposition,
        "family": "T5-E FOOTSTEP-SEQ",
        "outer_src": AinexT5EFootstepSeqEnv.OUTER_SRC,
        "bootstrap_corrector": str(boot),
        "bootstrap_label": boot_label,
        "bootstrap_sha16": _sha16(boot),
        "opts": ["SEQ_OUTER", "SHORT_DX", "CLEAR_CORRECTOR", "TDVM/SLR", "cancel×0.70", "FOOT_LIFT≥2cm", "LAT_BOS_opt"],
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
        "seq_logged_best": bool(best.get("seq_logged")),
        "seq_idle_best": bool(best.get("seq_idle", True)),
        "seq_err_best": best.get("seq_err"),
        "seq_n_track_best": best.get("seq_n_track"),
        "seq_n_advance_best": best.get("seq_n_advance"),
        "maxG_best": maxG_best,
        "cite": AinexT5EFootstepSeqEnv.CITE,
        "attack": "plant-cam ε / Root B (beat T5D1_13 maxG 0.055) while HOLDING skate/ret_ok",
        "h2_unlocked": False,
        "ret_ok_both": bool(best.get("ret_ok_both")),
        "vs_E7lock": "honest_beat" if beat else "Prefer FAIL (no ret_ok both under bars+ε+seq)",
        "vs_T5D1_13": "note — attack maxG 0.055; hold skate/ret_ok",
        "vs_T5D2_13": "note — ≠ ICP twin; Prefer FAIL if skate traded",
        "next_lever": (
            None if beat else "ask Dave H2 named unlock / alt DCM-VRP-DS / park — not residual twin; not auto H2; do not park Q without Dave"
        ),
    }
    (OUT / "T5E_DONE.json").write_text(json.dumps(done, indent=2, default=str))
    _write_progress([
        f"## DONE · disposition=**{disposition}** · outcome=`{outcome}` · "
        f"wall={elapsed:.0f}s ({elapsed/3600:.2f}h) · steps={model.num_timesteps}\n",
        f"- installed_new_sha16={installed} · best_sha=`{best.get('ckpt_sha16')}`\n",
        f"- best apps={best.get('apps')} rcf={best.get('ret_cf')} skate={best.get('skate')} "
        f"ret_ok={best.get('ret_ok')} eps_steal={best.get('eps_steal')} maxG={maxG_best}\n",
        f"- outer_logged={best.get('outer_logged')} outer_idle={best.get('outer_idle')} "
        f"OUTER=`{AinexT5EFootstepSeqEnv.OUTER_SRC}`\n",
        f"- seq_logged={best.get('seq_logged')} seq_idle={best.get('seq_idle')} "
        f"err={best.get('seq_err')} n_track={best.get('seq_n_track')} n_adv={best.get('seq_n_advance')}\n",
        f"- soft_pass=false · bars_kept=true · frozen_E7lock=`{E7SHA16}` · H2 OUT\n",
        f"- next_lever={done['next_lever']}\n",
    ])
    print(
        f"=== T5E DONE === {outcome} {disposition} wall={elapsed:.0f}s "
        f"installed={installed} best_sha={best.get('ckpt_sha16')} "
        f"seq_idle={best.get('seq_idle')} maxG={maxG_best}",
        flush=True,
    )
    env.close()


if __name__ == "__main__":
    main()
