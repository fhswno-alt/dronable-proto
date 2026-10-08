"""Cadence grid on the committed voice formula. Plant file is not written."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import score_com_zmp as sc
import zmp_preview as zp

PERIODS = (0.5, 0.6, 0.8, 1.0, 1.2, 1.6, 2.0)
VXS = (0.016, 0.024, 0.032, 0.040, 0.048, 0.056)
DSP = 0.25
Z = 0.008
CROUCH = 0.025
ARM = 1.0
ZC = 0.18


_AMP = {}


def amp_for(T, dsp=DSP):
    key = (round(float(T), 4), round(float(dsp), 4))
    if key not in _AMP:
        _AMP[key] = zp.sway_zmp_amp(T, dsp, ZC, 0.016, shape=0.0)
    return _AMP[key]


def one(T, vx, z=Z, dsp=DSP, crouch=CROUCH, perturb=None):
    amp = amp_for(T, dsp)
    walk = ARM + 2.05 * T
    res = sc.run_attempt(
        period_s=T,
        dsp=dsp,
        amp_m=amp,
        z_m=z,
        arm_s=ARM,
        stand_s=0.25,
        walk_s=walk,
        stop_s=2.4,
        vx_m_s=vx,
        preview_r=1e-4,
        preview_shape=0.0,
        crouch_m=crouch,
        hip_pitch_deg=15.0,
        gait_name="voice056",
        perturb=perturb,
    )
    return res


def line(res, extra=""):
    fails = []
    for key, name in (
        ("ask_ok", "ask"),
        ("margins_ok", "com"),
        ("cop_p5_ok", "cop"),
        ("step_ok", "step"),
        ("jerk_ok", "jerk"),
        ("tip_ok", "tip"),
    ):
        if not res[key]:
            fails.append(name)
    tag = "PASS" if res["cleared"] else "FAIL:" + ",".join(fails)
    print(
        f"T {float(res['period_s']):.2f} vx {float(res['vx_m_s']):.3f} "
        f"amp {float(res['amp_m'])*1000:.1f} z {float(res['z_m'])*1000:.0f} "
        f"dsp {float(res['dsp']):.2f} "
        f"ask {float(res['ask_nm']):.2f} {res['ask_worst']} "
        f"clr {float(res['clear_mm']):.2f} pk {float(res['clear_max_mm']):.2f} "
        f"frac {float(res['step_frac']):.3f} slip {float(res['slip_mm']):.2f} "
        f"ph {float(res['phase_mis']):.3f} nst {float(res['stance_n']):.0f} "
        f"com {float(res['com_min_m'])*1000:.1f} p5 {float(res['cop_p5_mm']):.1f} "
        f"n {float(res['n_steps']):.0f} place {float(res['place_mm']):.1f}/{float(res['cmd_mm']):.1f} "
        f"air {float(res['air_mm']):.1f}/{float(res['stride_mm']):.1f} "
        f"flat {res['flat']} spread {float(res['flat_spread_mm']):.2f} "
        f"pitch {float(res['pitch_deg']):.2f} up {float(res['min_up_z']):.3f} "
        f"jerk {float(res['jerk_rad_s3']):.0f}/{float(res['jerk_rms']):.0f} {res['jerk_joint']} "
        f"comj {float(res['torso_jerk_m_s3']):.1f}/{float(res['torso_jerk_rms']):.1f} "
        f"xamp {float(res['x_amp_mm']):.2f} {tag} {extra}",
        flush=True,
    )


print("AMPS", flush=True)
for T in PERIODS:
    print(f"  T {T:.2f} amp {amp_for(T)*1000:.2f} mm", flush=True)

print("GRID", flush=True)
for T in PERIODS:
    for vx in VXS:
        line(one(T, vx))

print("DSP", flush=True)
for dsp in (0.10, 0.15, 0.20, 0.25, 0.30):
    line(one(0.5, 0.056, dsp=dsp), extra=f"dsp-slice")

print("CROUCH", flush=True)
for crouch in (0.008, 0.012, 0.018, 0.025):
    try:
        line(one(0.5, 0.056, crouch=crouch), extra=f"crouch {crouch}")
    except Exception as exc:
        print(f"crouch {crouch} EXC {type(exc).__name__}: {exc}", flush=True)

print("Z", flush=True)
for z in (0.003, 0.004, 0.006, 0.008, 0.010):
    line(one(0.5, 0.056, z=z), extra=f"z-slice")
    line(one(0.5, 0.032, z=z), extra=f"z-slice-032")

print("PERTURB design", flush=True)
cells = [
    ("nominal", None),
    ("mass-5", sc.Perturb(label="mass-5", mass_scale=0.95)),
    ("mass+5", sc.Perturb(label="mass+5", mass_scale=1.05)),
    ("mu1.2", sc.Perturb(label="mu1.2", friction=1.2)),
    ("lat+1", sc.Perturb(label="lat+1", latency_ticks=1)),
    ("lat-1", sc.Perturb(label="lat-1", latency_ticks=-1)),
]
for name, pert in cells:
    line(one(0.5, 0.056, perturb=pert), extra=name)
print("PERTURB frac-cell", flush=True)
for name, pert in cells:
    line(one(0.5, 0.032, perturb=pert), extra=name)
print("DONE", flush=True)
