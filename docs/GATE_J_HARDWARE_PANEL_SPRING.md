# Gate J Hardware — panel soft-centering spring retune (companion)

**When:** Mon 28 Sep 2026 ~02:00 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Scope:** Gate **J01 panel abs ≥25°** under honest hand→lever coupling within plant ±30°. Soften companion panel centering spring only. **Sim only. No spend. No M145 hop.**  
**Criteria:** `docs/GATE_J_AI_CRITERIA.md` (J01 ≥25°; coupling honesty; still not full open)  
**Prereq hinge:** `docs/GATE_I_HARDWARE_PANEL_HINGE.md` (range ±30°; lever child of panel; Gate I soft params historical)

## Files

| Role | Path |
|------|------|
| Locked walk/score plant (**untouched**) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| **Gate F/G/H/I/J companion (edit here)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Gate J criteria | `docs/GATE_J_AI_CRITERIA.md` |
| Gate I panel hinge (range / coupling; soft params superseded here) | `docs/GATE_I_HARDWARE_PANEL_HINGE.md` |

**This doc owns the Gate J spring retune.** Gate I doc remains historical for hinge geometry / coupling / ±30° range; softness line there points here.

## Why retune

Gate I light center (`door_panel_hinge` damping=0.08, stiffness=0.05) allowed:

| Drive | Result (pre-J) |
|-------|----------------|
| 0.05 N·m torque on panel hinge | ~26.6° (passes ≥25° direct) |
| **2 N +Y xfrc on `door_lever_link`** | **~21.3°** (under J01 25°) |

Honest J01 path is **hand→lever contact→lever child of panel→`door_panel_hinge`**, not torque-on-panel. Spring needed light soften so modest lever-body force (order 1–3 N +Y) reaches ≥25° and can hold near that for J02 (≥1 s with continued contact force).

## Soft params (before → after)

| Joint | Param | Gate I (before) | Gate J (after) |
|-------|-------|-----------------|----------------|
| **`door_panel_hinge`** | damping | 0.08 | **0.05** |
| **`door_panel_hinge`** | stiffness | 0.05 | **0.015** |
| `door_panel_hinge` | range | ±0.5236 rad (±30°) | **unchanged** |
| `door_panel_hinge` | springref / armature | 0 / 0.0001 | unchanged |
| **`door_lever_hinge`** | damping / stiffness | 0.02 / 0.05 | **unchanged** (did not block coupling) |

Range **not** widened. Prefer spring retune only — smoke proved ≥25° inside ±30° with soft springs.

## Smoke (companion load)

```text
md5sum mujoco/ainex_hiwonder/ainex_controls_m2_145.xml
# expect fc94709c84f5598d4474ecfc4bb41fdc

.venv/bin/python — load ainex_controls_m2_145_gate_f.xml
# rest door_lever_site ≈ (0.40, 0, 0.275)
# no panel actuator; lever body parent == door_panel_link
# continuous +Y xfrc on door_lever_link for ~3 s; peak |door_panel_hinge| deg
```

### Results (Gate J soft params)

| Drive | Peak \|panel\| | Final / hold note |
|-------|----------------|-------------------|
| Panel hinge torque 0.05 N·m | ~30.4° (saturates near range) | baseline; still not free-fall |
| Panel hinge torque 0.08 N·m | ~30.6° | range-limited |
| Lever +Y xfrc **1.5 N** | **24.8°** | short of 25° |
| Lever +Y xfrc **2.0 N** | **25.4°** | hold ≥25° ≈ **2.0 s** |
| Lever +Y xfrc **2.5 N** | **25.8°** | hold ≥25° ≈ **2.3 s** |
| Lever +Y xfrc **3.0 N** | **26.0°** | hold ≥25° ≈ **2.5 s** |

**PASS** for Hardware J spring confirm: modest **2.0–2.5 N** +Y on lever body → panel abs **≥25°** within ±30°, hold ≥1 s with continued force (J02-ready plant). Coupling: lever child of panel; **no** panel actuator / no scripted panel qpos.

Centering check: after 2 s @ 2.5 N (~25.8°), release → returns toward rest (~14° after 2 s release); free rest drift ≈0°. Light center remains — not free-fall flop.

## Coupling / plant honesty

- `door_lever_link` parent = `door_panel_link` (kinematic child; Gate I unchanged).
- No `door_panel*` actuator in companion `<actuator>` block.
- Contype scheme unchanged (lever bit 2; panel visual-only).
- Locked M145 md5 **`fc94709c84f5598d4474ecfc4bb41fdc`** — not edited.
- Foot / HX / kit_cam / Gate E walk plant: untouched.

## Explicit non-claims

- **Not full door open / latch / walk-through / UK handle height** — J still limited swing ≥25° inside ±30°.
- Not M145 hop; not STEP/PO/spend; not Pi/Orin/NN-first; vision stays out of walk.
- Soft spring is **not** a lock — retune allows ≥25° under hand force through lever; Controls still owns J00–J03 prove (tip≥8, video, hold+close).
- Not equality-constraint cheat; not scripted panel actuator.
- Gate I lock stays limited-swing context — J does not reopen I as full-open.

## Names for Controls J01/J02

| Use | Name |
|-----|------|
| Panel hinge (≥25° abs) | `door_panel_hinge` |
| Lever contact body (xfrc / hand) | `door_lever_link` |
| Lever site | `door_lever_site` rest ≈ `(0.40, 0, 0.275)` |
| Load plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
