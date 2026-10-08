#!/usr/bin/env python3
"""One planned-feedforward bout. Prints the bar set and the planned τ_req peaks."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import score_com_zmp as sc
import zmp_preview

period = float(sys.argv[1])
vx = float(sys.argv[2])
out = Path(sys.argv[3])
mass = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
seed = sys.argv[5] if len(sys.argv) > 5 else "-"
mu = sys.argv[6] if len(sys.argv) > 6 else "-"
lat = int(sys.argv[7]) if len(sys.argv) > 7 else 0
rug = int(sys.argv[8]) if len(sys.argv) > 8 else 0

amp = zmp_preview.sway_zmp_amp(period, 0.40, 0.18, 0.016, 0.043, 0.0, 1.0e-4)
stand = 0.25
walk = 1.0 + 2.05 * period
stop = 4.5
perturb = None
if mass != 1.0 or seed != "-" or mu != "-" or lat != 0 or rug:
    perturb = sc.Perturb(
        label="perturb",
        seed=None if seed == "-" else int(seed),
        mass_scale=mass,
        friction=None if mu == "-" else float(mu),
        latency_ticks=lat,
        rug=bool(rug),
    )
result = sc.run_attempt(
    period_s=period,
    dsp=0.40,
    amp_m=float(amp),
    z_m=0.008,
    arm_s=1.0,
    stand_s=stand,
    walk_s=walk,
    stop_s=stop,
    vx_m_s=vx,
    preview_r=1.0e-4,
    preview_shape=0.0,
    crouch_m=0.025,
    gait_name="voice056",
    z_quintic=True,
    z_lead=True,
    honor_vx=True,
    id_ff=True,
    perturb=perturb,
)
keep = (
    "signed_ok", "signed_value", "signed_joint", "signed_t", "clamp_ok", "clamp_max",
    "dc_ok", "dc_excess", "dc_joint", "dc_tau", "dc_qvel", "step_ok", "n_steps",
    "clear_mm", "slip_mm", "step_frac", "phase_mis", "sep_mm", "flat",
    "flat_spread_mm", "flat_pitch_deg", "stance_n", "len_ok", "tip_ok", "min_up_z",
    "stop_up_z", "vx_ratio", "vx_mean", "limit_frac", "limit_ok", "ctrl_ok",
    "ctrl_clip_n", "ctrl_clip_frac", "fault", "mfg_ok", "soft_pass", "plant_md5",
    "id_plan_tau", "id_plan_joint", "id_plan_phase", "id_plan_t", "id_plan_term",
    "id_bind_joint", "id_bind_phase", "id_bind_term", "id_bind_tau", "id_bind_t",
    "id_req_wall", "id_req_wall_n", "id_req_wall_joint", "id_req_wall_tau",
    "id_req_wall_bare", "id_req_wall_phase", "id_req_wall_t",
    "id_req_knee_tau", "id_req_knee_joint", "id_req_knee_phase", "id_req_knee_t",
    "id_req_knee_qdd", "id_req_knee_bare", "id_req_knee_abs",
    "id_stop_span", "id_stop_qdd", "id_resid_max", "id_resid_exact_n",
    "id_resid_over_n", "id_phys_n", "id_req_with", "id_req_bare",
    "ik_fail", "pitch_deg",
)
slim = {k: result.get(k) for k in keep}
out.write_text(json.dumps(slim, indent=2) + "\n")
print(
    f"T {period:.2f} vx {vx:.3f} mass {mass} seed {seed} mu {mu} lat {lat} rug {rug} "
    f"signed {slim['signed_value']:+.3f} {slim['signed_joint']} ok {slim['signed_ok']} "
    f"clamp {slim['clamp_ok']} dc {slim['dc_ok']} step {slim['step_ok']} "
    f"tip {slim['tip_ok']} up {slim['min_up_z']:.3f} stop_up {slim['stop_up_z']:.3f} "
    f"flat {slim['flat']} vxr {slim['vx_ratio']:.3f} lim {slim['limit_frac']:.4f} "
    f"clip {slim['ctrl_clip_frac']:.4f} fault {slim['fault']!r}"
)
print(
    f"  knee τ {slim['id_req_knee_tau']:+.3f} {slim['id_req_knee_joint']} "
    f"{slim['id_req_knee_phase']} t {slim['id_req_knee_t']:.3f} "
    f"qdd {slim['id_req_knee_qdd']:+.2f} bare {slim['id_req_knee_bare']:+.3f} "
    f"span {slim['id_stop_span']:.3f} qdd_cap {slim['id_stop_qdd']:.2f} "
    f"wall {slim['id_req_wall']} {slim['id_req_wall_joint']} "
    f"bare {slim['id_req_wall_bare']:+.3f} {slim['id_req_wall_phase']}"
)
for name, rec in sorted((slim["id_req_with"] or {}).items()):
    bare = (slim["id_req_bare"] or {}).get(name, {})
    print(
        f"  {name:16s} with {rec['tau']:+7.3f} t {rec['t']:.3f} {rec['phase']:16s} "
        f"bare {bare.get('bare', float('nan')):+7.3f} t {bare.get('t', float('nan')):.3f} "
        f"{bare.get('phase', '')}"
    )
