# CoP proxy falsifier — AiNex kit signals vs MuJoCo truth

**When:** Sun 27 Sep 2026 Europe/London (BST)  
**Model:** `mujoco/ainex_hiwonder/ainex_controls.xml` (HX ±2.1 leg / ±0.7 arm)  
**Script:** `scripts/cop_proxy_check.py`  
**Log rate:** 50 Hz  
**Scope:** AI sensing lane — no purchases, no Orin, no NN. Path A spend frozen.

## VERDICT

**MARGINAL**

Some correlation exists; usable only with filtering / calibration and not as a sole skate detector. Prefer capture-point from IMU+q, or add FSR.

### Key numbers

- stand disturb share_sup vs truth: corr=0.829 RMSE=0.161
- stand disturb share_sup EMA(α=0.2): corr=0.832 RMSE=0.186
- stand disturb CoP-y (ankle-τ proxy) vs truth: corr=0.980 RMSE=0.0095 m
- stand majority-load agreement=1.000
- walk share_sup vs truth: corr=0.386 RMSE=0.400 (t_final=3.80, tip_t=None)
- walk share_sup EMA(α=0.2): corr=0.706 RMSE=0.368
- walk CoP-y ankle-τ: corr=0.653 RMSE=0.0419 m
- stand disturb CoP-x ankle-τ vs truth: corr=0.487 RMSE=0.0208 m

**Why not USEABLE:** stand CoP-y is strong (corr≈0.98, RMSE≈9.5 mm) and majority-load agree=1.0, but **share RMSE=0.16 > 0.12** USEABLE gate, and **walk** share RMSE≈0.37–0.40 (EMA corr helps to ~0.71 but RMSE stays bad). Not sole skate/unload detector through tip horizon.

## Method

### Truth (MuJoCo)
- Per-foot normal load `Fz` and CoP-xy from `mj_contactForce` on `l_foot_contact` / `r_foot_contact` (+ toe spheres) vs `floor`.
- Load-share_T = Fz_L / (Fz_L + Fz_R); combined CoP = force-weighted contact positions.

### Proxy (kit-available only)
- `actuator_force` on position actuators (= HX torque / load channel stand-in).
- Bus load = `|actuator_force|` summed on leg chain.
- **share_sup**: `|τ|` on {knee, ank_pitch, ank_roll, hip_pitch} → L/(L+R).
- **CoP ankle-τ**: split `mg` by share_sup, then local `r ≈ τ/Fz` at each ankle, mapped to world via foot body xy (quasi-static).
- Joint `q` used only for FK foot xy (same as kit encoders).
- Ankle-pitch → CoP-x sign calibrated vs quiet-stand truth (R axis mirrored).

### Episodes
1. **Quiet stand** settle 0.80s (no disturb) — sanity.
2. **Stand + disturb:** Fy=3.8 N, Fx=0.0 N, hold=0.60s at t=1.00s on `body_link` (xfrc); window = disturb→+1.0s or tip.
3. **Open-loop walk** via `walk_gait_ainex.gait_targets` (no ankle-CoP servo, no plant damper — sense-only); log until tip or 3.0s.

## Results tables

### Quiet stand (pre-disturb / no push)

| Metric | Value |
|--------|-------|
| share_sup vs T | corr=0.996, RMSE=0.0037, n=45 |
| share_bus vs T | corr=0.997, RMSE=0.0018, n=45 |
| CoP-y ankle-τ vs T | corr=0.998, RMSE=0.0003, n=45 |
| CoP-y share-only vs T | corr=0.991, RMSE=0.0001, n=45 |
| majority-load agree | 1.0 |

### Stand disturb window

| Metric | Value |
|--------|-------|
| window | 1.00-1.68s |
| tip_t | 1.68 |
| share_sup vs T | corr=0.829, RMSE=0.1611, n=35 |
| share_sup EMA(α=0.2) vs T | corr=0.832, RMSE=0.1862, n=35 |
| share_bus vs T | corr=0.859, RMSE=0.1764, n=35 |
| CoP-y ankle-τ vs T | corr=0.980, RMSE=0.0095, n=35 |
| CoP-y ankle EMA vs T | corr=0.874, RMSE=0.0190, n=35 |
| CoP-y share-only vs T | corr=0.863, RMSE=0.0245, n=35 |
| CoP-x ankle-τ vs T | corr=0.487, RMSE=0.0208, n=35 |
| Fz_L vs proxy | corr=0.729, RMSE=3.8850, n=35 |
| Fz_R vs proxy | corr=0.830, RMSE=3.7009, n=35 |
| majority-load agree | 1.0 |
| CoP-y truth ptp | 0.06885325029275803 m |

### Walk (first seconds until tip/skate)

| Metric | Value |
|--------|-------|
| t_final / tip_t | 3.80 / None |
| share_sup vs T | corr=0.386, RMSE=0.3996, n=116 |
| share_sup EMA(α=0.2) vs T | corr=0.706, RMSE=0.3677, n=116 |
| share_bus vs T | corr=0.255, RMSE=0.4190, n=116 |
| CoP-y ankle-τ vs T | corr=0.653, RMSE=0.0419, n=116 |
| CoP-y ankle EMA vs T | corr=0.603, RMSE=0.0449, n=116 |
| CoP-y share-only vs T | corr=0.615, RMSE=0.0447, n=116 |
| majority-load agree | 0.6206896551724138 |

## Gates used

- USEABLE: share corr ≥ 0.7 and RMSE ≤ 0.12, majority-load agree ≥ 0.80; CoP-y supportive if corr ≥ 0.7 / RMSE ≤ 0.025 m.
- MARGINAL: share corr ≥ 0.4 and RMSE ≤ 0.22.
- FAIL: below marginal → ankles blind without add-on sensing.

## Contact / model notes

- Foot boxes `l_foot_contact` / `r_foot_contact` already `contype=1 conaffinity=1 condim=3` in `ainex_controls.xml`; no geom edits required for this run.
- Truth CoP uses contact positions × normal loads; proxy never reads `cfrc_ext` / contacts.

## Next AI / Controls action

If using the proxy at all, low-pass share_sup (~5–10 Hz) and use it only as a **binary unload hint** beside an IMU capture-point regulator — not as primary CoP. Parallel: implement CP/ZMP from IMU+q (proposal #1); keep FSR as the cheap sensing unlock if CP still fails tip-threshold. No NN, no Orin.

## Artifacts

- `scripts/cop_proxy_check.py`
- `docs/COP_PROXY_FALSIFIER.md` (this file)
- `mujoco/ainex_hiwonder/cop_proxy_stats.json`

