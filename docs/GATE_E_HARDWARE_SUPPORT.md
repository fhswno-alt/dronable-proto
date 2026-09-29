# Gate E Hardware Support — M2 plant lock

**Date:** 2026-09-27 (Europe/London)  
**Scope:** Hardware support for Controls' Gate E cadence / skate-kill work. This note does **not** claim Gate E.

## Plant lock

- **Locked plant:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`
- **Contact footprint:** both `l_foot_contact` and `r_foot_contact` use MuJoCo box half-sizes `0.0725 × 0.0430 m`, i.e. **145 × 86 mm** planform. This matches the locked M2 plant and CAD naming/spec.
- **Sim friction:** contact boxes and floor use `friction="1.6 0.1 0.01"`. This is the current simulation assumption, not a measured TPU/plant coefficient.
- **Current recorded baseline:** `previews/ainex_walk/iterate/BEST_SS_clean_walk.json` reports `skate: false`, `ss_peak_clear_m_L/R = 17.001 / 16.804 mm`, `ss_mean_clear_m_L/R = 9.735 / 10.035 mm`, 15 single-support bouts per side, and `ss_duty_total = 55.45%` (contact duty L/R `73.03% / 73.33%`). These are baseline metrics only; they do not constitute a Gate E result.

## Manufacturing-ready CAD pack

Quote-ready mesh-derived / **not OEM STEP** files are in `cad/m2_outsole/`:

- `README.md`
- `M2_outsole_145x86_meshAABB_notOEM.step` and `.stl` — 145 × 86 × 3 mm plate
- `M2_tread_145x86_TPU_notOEM.step` and `.stl` — 145 × 86 × 1.5 mm TPU 95A tread
- `M1_bumper_50x28x8_TPU_notOEM.step` and `.stl` — survival bumper, if required

The exported STL bounds verify 145 × 86 × 3 mm for the plate and 145 × 86 × 1.5 mm for the tread. README and filenames agree with the locked 145 × 86 mm footprint.

## Hardware changes only if Controls asks

Hardware will turn around a targeted variant only on a Controls request, and only for a demonstrated Gate E skate-kill / cadence or fit need:

1. **Contact size** — revise the 145 × 86 mm contact geometry only if the sim evidence identifies a footprint/fit issue.
2. **Sole thickness** — revise the 3 mm plate / 1.5 mm tread stack only if clearance, contact timing, or fit evidence requires it.
3. **Friction** — provide an explicitly labelled friction A/B assumption or material variant; do not treat `1.6 0.1 0.01` as measured hardware data.
4. **Meshsole fallback** — prepare a mesh-derived sole collision fallback if the box proxy is the source of a false skate/contact result.

## Hardware will not do

- No Path A purchase while spend is frozen.
- No Feetech actuator swap.
- No Onshape spend.
- No plant resize unless Controls demonstrates that the locked M2 geometry is the cause and requests the change.

**Controls offer:** Hardware will review and return any requested plant-geometry tweak for Gate E skate-kill / cadence within **24 hours** of a sufficiently specified request (target dimensions, evidence, and requested A/B).

## Known simulation-to-CAD risk

The XML contact box has a full vertical size of **16 mm** (`size z="0.008"`), while the CAD plate+tread nominal stack is **4.5 mm**. Planform dimensions match, but this vertical box dimension is a collision proxy rather than a CAD thickness representation. Controls should flag whether it affects clearance/contact timing before requesting a hardware change; Hardware will not alter the locked plant spec pre-emptively.


## Residual honesty plant A/B (`cadsole`)

For residual stance-vx work that needs CAD-honest sole *thickness* without changing the locked 145×86 planform, use:

- **A/B plant:** `mujoco/ainex_hiwonder/ainex_controls_m2_145_cadsole.xml`
- **Change:** contact half-size z `0.008` → `0.00225` (4.5 mm full); `pos` z `−0.018` → `−0.02375` so sole bottom stays at `−0.026`.
- **SOLE_OFFSET:** remains `0.026` (see `docs/PLANT_CADSOLE_AB.md`).
- **Load:** `walk_gait_ainex.py --model mujoco/ainex_hiwonder/ainex_controls_m2_145_cadsole.xml`
- **Not** a Gate E claim; locked plant for Gate E scoring stays `ainex_controls_m2_145.xml`.

## AUTH → contact honesty

HX authority envelope is **HARD-FALSIFIED** (sat cleared at `k_auth=10`, skate still fails — see Controls AUTH note). Next lever is **plant/contact honesty** on the locked M145 family (still sim).

- Hardware pre-staged GEO pack: `docs/PLANT_CONTACT_HONESTY_PACK.md`
- GEO01 cadsole (existing), GEO02 planform (`ainex_controls_m2_160x90.xml`, `…_155x86.xml`, optional `…_160x90_cadsole.xml`), GEO03 softsole (`ainex_controls_m2_145_softsole.xml`)
- Load via `walk_gait_ainex.py --model …`. **`SOLE_OFFSET = 0.026`** on all rows; sole bottom stays **−0.026**.
- Locked score plant remains `ainex_controls_m2_145.xml` until AI cospec locks a winner. **No Gate E claim** from shipping these XMLs.
