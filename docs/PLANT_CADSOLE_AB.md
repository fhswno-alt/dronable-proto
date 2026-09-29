# Plant A/B: M145 CAD-honest sole vertical (`cadsole`)

**Date:** 2026-09-27 (Europe/London)  
**Purpose:** Residual stance-vx honesty A/B — same 145×86 planform as locked M145, contact box vertical matched to CAD plate+tread (~4.5 mm) instead of the locked 16 mm collision proxy.  
**Not a Gate E claim.** Does not replace the locked plant. Sim / Controls residual work only.

## Files

| Role | Path |
|------|------|
| Locked baseline | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| This A/B plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145_cadsole.xml` |
| CAD stack reference | `cad/m2_outsole/` — 3 mm plate + 1.5 mm tread = **4.5 mm** |

## Geometry delta (feet only)

| Field | Locked M145 | `cadsole` |
|-------|-------------|-----------|
| Planform (full) | 145 × 86 mm | **unchanged** |
| Half-size xy | `0.0725 0.0430` | **unchanged** |
| Half-size z | `0.008` (16 mm full) | **`0.00225`** (4.5 mm full) |
| Contact `pos` | `0.030 0.0 -0.018` | `0.030 0.0 **-0.02375**` |
| Sole bottom (body frame) | `pos_z − half_z = −0.026` | **same −0.026** |
| Friction / HX / PD / meshes | as locked | unchanged |

`pos` z was shifted so sole bottom (and therefore standing clearance reference) stays at −0.026 m in the ankle-roll body frame. Only the contact *thickness* changes.

## `SOLE_OFFSET` for Controls

`scripts/ss_step_ainex.py` hardcodes:

```python
SOLE_OFFSET = 0.026  # foot-body z − sole bottom (box pos −0.018 + half-h 0.008)
```

**Recommendation for `cadsole`:** keep **`SOLE_OFFSET = 0.026`** (no script change).  
Formula: `SOLE_OFFSET = -(contact_pos_z) + contact_half_z`  
→ locked: `0.018 + 0.008 = 0.026`  
→ cadsole: `0.02375 + 0.00225 = 0.026`

Related (not SOLE_OFFSET, but plant-coupled): `scripts/walk_gait_ainex.py` uses `CONTACT_Z_THR = 0.025` for planted-foot heuristics based on foot-body z. With sole bottom preserved, that threshold remains appropriate for this A/B.

There is **no** CLI/env override for `SOLE_OFFSET` today — it is a module constant in `ss_step_ainex.py` only (used in post-hoc clearance analysis, not in MuJoCo load). Prefer doc-only; do not change the default unless a future plant moves the sole bottom.

## How Controls loads it

Walker (`walk_gait_ainex.py`) accepts `--model`:

```bash
MUJOCO_GL=egl .venv/bin/python scripts/walk_gait_ainex.py \
  --model mujoco/ainex_hiwonder/ainex_controls_m2_145_cadsole.xml \
  --duration 9 --tag RES_cadsole_T75 ...
```

`ss_step_ainex.py` hardcodes `PLANT = .../ainex_controls_m2_145.xml` in `build_cmd` (`--model` always that path). For cadsole A/B via ss_step, either:

1. Call `walk_gait_ainex.py` directly with `--model ..._cadsole.xml` (preferred), or  
2. Temporarily point `PLANT` / pass an equivalent `--model` if Controls adds a flag later.

Geom names stay `l_foot_contact` / `r_foot_contact` (scripts that `mj_name2id` those names need no rename).

Env commonly used with these scripts: `MUJOCO_GL=egl` (EGL offscreen). No plant-specific env var.

## Honesty / non-claims

- Locked Gate E plant remains `ainex_controls_m2_145.xml`.  
- This file only falsifies or confirms whether the **16 mm vertical box** was inflating clearance / contact timing vs a ~4.5 mm CAD stack.  
- Do **not** treat a residual pass or fail on `cadsole` as Gate E without re-scoring on the locked plant (or an explicit dual-plant protocol).  
- No residual success claimed by shipping this plant.

## Script touch

**Doc-only.** No default change in `ss_step_ainex.py` / `walk_gait_ainex.py` — locked M145 stays default; Controls passes `--model` for the A/B.
