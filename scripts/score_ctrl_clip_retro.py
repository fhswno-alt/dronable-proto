#!/usr/bin/env python3
"""Ctrl-range clip on the two re-verdicted bouts.

A control tick fails when a leg writer command before the ctrlrange clip,
or ``data.ctrl`` entering ``mj_step``, has absolute value greater than 2.09.
Exactly ±2.09 is inside the range. The writer command is the value
``write_clipped`` / ``write_force_limited`` holds just before that clip.
If clipping that value does not reproduce the stored ctrl, the bout is
``raw ctrl unavailable``.

This does not replace the hinge JSON or the stepping doc.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("DISPLAY", ":1")
os.environ.setdefault("MUJOCO_GL", "glfw")

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import mujoco as mj

LEG_JOINTS = (
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
)
CTRL_ABS = 2.09
D6_SHA = "d6e8b5ebb801250814552fc728886a11dd0350c2"
KITCHEN_SHA = "51ae123add3cc6bb9c3e5cd0f1d6b4035e65b29c"


class ClipStats:
    def __init__(self) -> None:
        self.ticks = 0
        self.clip_ticks = 0
        self.clip_substeps = 0
        self.substeps = 0
        self.writes = 0
        self.raw_unavailable = False
        self.max_writer_abs = 0.0
        self.max_step_abs = 0.0
        self._open = False
        self._tick_clipped = False
        self._idx: dict[int, list[int]] = {}

    def begin(self) -> None:
        self._open = True
        self._tick_clipped = False

    def end(self) -> None:
        if not self._open:
            return
        self._open = False
        self.ticks += 1
        if self._tick_clipped:
            self.clip_ticks += 1

    def indices(self, model: mj.MjModel) -> list[int]:
        key = id(model)
        cached = self._idx.get(key)
        if cached is not None:
            return cached
        found: list[int] = []
        for name in LEG_JOINTS:
            idx = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name + "_pos"))
            if idx >= 0:
                found.append(idx)
        self._idx[key] = found
        return found

    def note_step(self, model: mj.MjModel, data: mj.MjData) -> None:
        if not self._open:
            return
        self.substeps += 1
        hit = False
        for idx in self.indices(model):
            raw = abs(float(data.ctrl[idx]))
            if raw > self.max_step_abs:
                self.max_step_abs = raw
            if raw > CTRL_ABS + 1e-9:
                hit = True
        if hit:
            self.clip_substeps += 1
            self._tick_clipped = True

    def note_write(self, raw: float | None) -> None:
        if not self._open:
            return
        self.writes += 1
        if raw is None:
            self.raw_unavailable = True
            return
        if abs(raw) > self.max_writer_abs:
            self.max_writer_abs = abs(raw)
        if abs(raw) > CTRL_ABS + 1e-9:
            self._tick_clipped = True

    def payload(self, bout: str, tip: str) -> dict[str, object]:
        fraction: float | str
        if self.raw_unavailable:
            fraction = "raw ctrl unavailable"
        else:
            fraction = (self.clip_ticks / self.ticks) if self.ticks else 0.0
        return {
            "bout": bout,
            "tip_sha": tip,
            "ctrl_abs_rad": CTRL_ABS,
            "n_ticks": self.ticks,
            "clip_ticks": self.clip_ticks,
            "clip_fraction": fraction,
            "clip_substeps": self.clip_substeps,
            "substeps": self.substeps,
            "writes": self.writes,
            "max_abs_writer_raw": None if self.raw_unavailable else self.max_writer_abs,
            "max_abs_data_ctrl": self.max_step_abs,
            "passes": (not self.raw_unavailable) and self.clip_ticks == 0,
            "definition": (
                "|ctrl_raw| > 2.09 on a control tick is a fail. ctrl_raw is the "
                "writer command before the ctrlrange clip when that command "
                "reproduces the stored ctrl, and data.ctrl entering mj_step."
            ),
        }


def _predict_cmd(walker, jn: str, q_des: float, limit_nm: float | None, force: bool):
    act = f"{jn}_pos"
    idx = walker.act_idx.get(act)
    if idx is None or jn not in LEG_JOINTS:
        return None
    if not force and jn.endswith(("knee", "hip_pitch", "ank_pitch")):
        return int(idx), float(q_des)
    if not force:
        q = walker.q(jn)
        band = walker.e_sat.get(act, 0.0)
        return int(idx), float(min(q + band, max(q - band, float(q_des))))
    q = walker.q(jn)
    jid = mj.mj_name2id(walker.model, mj.mjtObj.mjOBJ_JOINT, jn)
    omega = float(walker.data.qvel[int(walker.model.jnt_dofadr[jid])])
    kp = float(walker.model.actuator_gainprm[idx, 0])
    kv = -float(walker.model.actuator_biasprm[idx, 2])
    tau = abs(float(walker.model.actuator_forcerange[idx, 1]))
    limit = float(sys.modules[walker.__class__.__module__].SAT_FRAC) * tau
    if limit_nm is not None:
        limit = min(limit, float(limit_nm))
    if kp < 1e-6:
        cmd = float(q_des)
    else:
        e_des = float(q_des) - q
        e_lo = (-limit + kv * omega) / kp
        e_hi = (limit + kv * omega) / kp
        if e_lo > e_hi:
            e_lo, e_hi = e_hi, e_lo
        cmd = q + min(e_hi, max(e_lo, e_des))
    return int(idx), float(cmd)


def _check_write(walker, idx: int, cmd: float, stats: ClipStats) -> None:
    lo = float(walker.model.actuator_ctrlrange[idx, 0])
    hi = float(walker.model.actuator_ctrlrange[idx, 1])
    clipped = min(hi, max(lo, cmd))
    stored = float(walker.data.ctrl[idx])
    if abs(clipped - stored) > 1e-8:
        stats.note_write(None)
        return
    stats.note_write(cmd)


def install(stats: ClipStats, walker_cls, session_cls) -> None:
    orig_step = mj.mj_step
    orig_clipped = walker_cls.write_clipped
    orig_limited = walker_cls.write_force_limited
    orig_session = session_cls.step

    def capturing_step(model, data, nstep=1):
        stats.note_step(model, data)
        return orig_step(model, data, nstep)

    def wrapped_clipped(self, jn, q_des):
        pred = _predict_cmd(self, jn, q_des, None, False)
        orig_clipped(self, jn, q_des)
        if pred is not None:
            _check_write(self, pred[0], pred[1], stats)

    def wrapped_limited(self, jn, q_des, limit_nm=None):
        pred = _predict_cmd(self, jn, q_des, limit_nm, True)
        orig_limited(self, jn, q_des, limit_nm)
        if pred is not None:
            _check_write(self, pred[0], pred[1], stats)

    def wrapped_session(self, *args, **kwargs):
        stats.begin()
        try:
            return orig_session(self, *args, **kwargs)
        finally:
            stats.end()

    mj.mj_step = capturing_step
    walker_cls.write_clipped = wrapped_clipped
    walker_cls.write_force_limited = wrapped_limited
    session_cls.step = wrapped_session


def run_d6(out: Path) -> dict[str, object]:
    os.environ["WALK_GAIT_SCRIPTS"] = str(Path("/tmp/tiebreak-walk") / D6_SHA / "scripts")
    # The worktree is created by the same helper the tiebreak uses.
    sys.path.insert(0, str(SCRIPTS))
    import score_tiebreak_d6e8b5e as tie
    tree = tie._worktree()
    os.environ["WALK_GAIT_SCRIPTS"] = str(tree / "scripts")
    # score_walk_smoothness reads the env at import.
    if "score_walk_smoothness" in sys.modules:
        raise SystemExit("score_walk_smoothness was imported before the tip path was set")
    import score_walk_smoothness as sws
    loaded = Path(sws.steer_walk.__file__).resolve()
    if not str(loaded).startswith(str(tree.resolve())):
        raise SystemExit(f"gait loaded from {loaded}")
    stats = ClipStats()
    install(stats, sws.lipm_gait.LipmWalker, sws.steer_walk.SteerSession)
    spec = sws.PreviewRowSpec(
        name="voice-20",
        period_s=20.0,
        dsp=0.35,
        amp_m=0.043,
        z_m=0.018,
        arm_s=2.40,
        vx_m_s=0.056,
        stand_s=0.40,
        walk_s=22.00,
        stop_s=8.00,
        preview_shape=0.0,
        source="ctrl-clip re-verdict of d6e8b5e voice-20",
    )
    cfg = sws.steer_walk.voice_preview_config()
    row = sws.run_preview_row(spec, tip_sha=D6_SHA, lipm_config=cfg)
    payload = stats.payload("d6e8b5e voice-20", D6_SHA)
    payload["plant_md5"] = row.get("plant_md5")
    payload["n_samples"] = row.get("n_samples")
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2), flush=True)
    return payload


def run_kitchen(out: Path) -> dict[str, object]:
    import score_retro_voice as retro
    sw, tree = retro._prepare(KITCHEN_SHA)
    import lipm_gait
    stats = ClipStats()
    install(stats, lipm_gait.LipmWalker, sw.SteerSession)
    step_bars = retro._load_step_bars()
    retro.run_voice(sw, step_bars, "51ae123", 90, KITCHEN_SHA, ("kitchen",))
    payload = stats.payload("51ae123 kitchen-m90", KITCHEN_SHA)
    payload["plant_md5"] = retro._plant_md5(Path(sw.PLANT_XML))
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2), flush=True)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Ctrl-range clip on one re-verdict bout")
    parser.add_argument("--bout", choices=("d6e8b5e", "kitchen"), required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    if args.bout == "d6e8b5e":
        run_d6(path)
    else:
        run_kitchen(path)


if __name__ == "__main__":
    main()
