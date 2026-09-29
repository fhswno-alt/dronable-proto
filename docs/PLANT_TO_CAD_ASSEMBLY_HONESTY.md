# Plant ↔ CAD assembly honesty (founder one-pager)

**Date:** 2026-09-28 (Europe/London)  
**Audience:** Dave (software founder) — “what does the robot look like?” without buying parts  
**Owner:** Hardware · plant lock owned by Controls Gate E work  
**Scope:** Sim plant vs CAD/mesh honesty only. **No Path A / spend ask.**

---

## 1. Locked sim plant (what Controls scores on)

| Field | Value |
|-------|-------|
| File | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| Planform | **145 × 86 mm** contact boxes (`l_foot_contact` / `r_foot_contact`; half `0.0725 × 0.0430`) |
| Vertical | **16 mm** box collision proxy (half `0.008`) — **not** CAD sole thickness |
| Friction | `1.6 0.1 0.01` on feet + floor (sim assumption, not measured TPU) |
| Actuators | Kit-matched **24 DOF** meshes; **HX clips** ±2.1 Nm legs / ±0.7 arm-head |
| Sole bottom | −0.026 m in ankle-roll frame (`SOLE_OFFSET = 0.026`) |

This is the body Gate E learned/RL is running against. **Hardware must not change it** while Controls is mid Gate E on locked M145.

Visual meshes = Hiwonder URDF STLs (kit-matched look). Contact = simple boxes under the feet. You can “see” the robot in MuJoCo previews without owning hardware.

---

## 2. CAD pack (what Manufacturing would fab — quote-ready, not bought)

| Item | Spec | Path |
|------|------|------|
| Outsole plate | **145 × 86 × 3 mm** | `cad/m2_outsole/M2_outsole_145x86_meshAABB_notOEM.{step,stl}` |
| Tread | **145 × 86 × 1.5 mm** TPU 95A | `…/M2_tread_145x86_TPU_notOEM.{step,stl}` |
| Stack height | **(3 + 1.5) = 4.5 mm** | README agrees |
| Bumper (optional) | 50 × 28 × 8 mm TPU | `…/M1_bumper_…` |

**Match:** planform **145 × 86** = locked plant.  
**Mismatch (honest):** CAD stack is **4.5 mm** tall; sim contact box is **16 mm**. Planform is what we mean by “M145 footprint”; vertical is a collision proxy, not a thickness claim.

All labels: **mesh-derived / not OEM STEP**.

---

## 3. Onshape / mesh folder (not OEM STEP)

- Dronable Onshape folder name: **`AiNex Hiwonder mesh-derived (NOT OEM STEP)`** — host for mesh-derived foot solids next to `l_ank_roll_link` / `r_ank_roll_link` fit check.
- **No public OEM AiNex STEP** and **no public shareable articulated Onshape assembly URL** found; kit STEP remains order-gated.
- Foot visuals in sim: `mujoco/ainex_hiwonder/meshes/{l,r}_ank_roll_link.STL` (collision off; contact = boxes above).
- Import CAD STEP beside ank_roll meshes for sleeve/pocket fit — still **not** claiming OEM geometry.

---

## 4. Falsified plant variants (honesty A/B only — do not reopen as buy asks)

| Tag | File | What it falsified | Gate E? |
|-----|------|-------------------|---------|
| GEO00 | `ainex_controls_m2_145.xml` | Locked baseline | Score plant; Gate E **FALSE** |
| GEO01 | `…_m2_145_cadsole.xml` | 16 mm → **4.5 mm** vertical | No unlock |
| GEO02a | `…_m2_160x90.xml` | Planform **160 × 90** | No unlock |
| GEO02b | `…_m2_155x86.xml` | Planform **155 × 86** (+10 mm L) | No unlock |
| GEO02c | `…_m2_160x90_cadsole.xml` | 160×90 + 4.5 mm | No unlock |
| GEO03 | `…_m2_145_softsole.xml` | Soft `solref`/`solimp` | No unlock |

Contact/geometry honesty pack = **HARD-FALSIFIED** for Gate E under kit HX (`CONTACT_GEOMETRY_HONESTY_COSPEC.md`). Variants stay on disk for audit; **none unlocked Gate E**; **do not reopen as spend / Path A / resize-to-buy**.

Index: `mujoco/ainex_hiwonder/FOOT_CONTACT_VARIANTS.md` · pack: `docs/PLANT_CONTACT_HONESTY_PACK.md`.

---

## 5. What Hardware changes next (idle rule)

| Condition | Hardware action |
|-----------|-----------------|
| Learned/RL **wins** and Controls **freezes a different plant XML** | Turn around that plant’s CAD/mesh note (still sim-first) |
| AI **cospec** explicitly asks for a plant geometry change | Ship requested XML ± CAD note within ~24 h |
| Otherwise | **Idle on plant XML** — do not touch M145 contact, friction, or HX clips |

Hardware does **not** resize the foot, thicken the sole, or swap friction “to help Gate E” while Controls is mid learned/RL on locked M145.

---

## 6. Gate status (as of this note)

| Gate | Status | Plant |
|------|--------|-------|
| **D** | **TRUE** — CSF50 / `BEST_SS_clean_walk` | Locked **M145** |
| **E** | **FALSE** (hard falsifier on classic family; learned/RL in progress) | Still scoring on **M145** |

Curriculum: `docs/TELEOP_AUTONOMY_CURRICULUM.md`. Gate E criteria: `docs/GATE_E_AI_CRITERIA.md`. Support note: `docs/GATE_E_HARDWARE_SUPPORT.md`.

---

## Assembly-understanding checklist (before any buy)

Things that must be true *before* Path A / parts spend — **none of these are a buy request**:

1. **Sim plant claim locked** — which XML is the score body (today: M145); Gate E result on that plant stated.
2. **Foot planform agreement** — CAD plate/tread planform matches locked contact (today: 145×86).
3. **Vertical honesty noted** — 16 mm box vs 4.5 mm CAD stack documented; Controls signed off if it matters.
4. **OEM vs mesh-derived** — STEP/mesh source labelled; no silent “OEM” claim on mesh AABB parts.
5. **Bus / actuators** — HX serial bus only (HX-35H / HX-35HM / HX-12H); no Feetech for unit one.
6. **HX torque clips in sim** — ±2.1 / ±0.7 match kit-class authority used for Gate claims.
7. **Mass / COM** — kit envelope (~2.45 kg, public dims) vs sim inertials reviewed for walk claims. → measured **2.35 kg** / COM Z **0.225 m** foot-grounded: `docs/PLANT_MASS_COM_HAND_REACH.md`.
8. **Hand reach Z (Gate F)** — `l/r_gripper_link` vs lever band 0.25–0.30 m AFF; F02 L~0.291 plant-honest without plant hop. → `docs/PLANT_MASS_COM_HAND_REACH.md`.
9. **Foot stack BOM** — plate + tread (+ optional bumper) dimensions and material frozen in CAD README.
10. **ank_roll fit** — CAD sleeve pocket checked against `*_ank_roll_link` mesh AABB (not guessed). → **PASS** mesh-derived dry-fit: `docs/ANK_ROLL_SLEEVE_FIT.md` (pocket 137×78 vs ~135.08×76.04; ~0.96 mm/side; zip-tie path, not OEM).
11. **Dave greenlight** — explicit assembly/buy ask; Hardware does not invent Path A from a sim unlock alone.

---

## Bottom line for Dave

We already know what the robot **looks like** in sim (24-DOF kit meshes) and what the **foot print** is (145×86 locked, CAD plate+tread ready as mesh-derived solids). The only deliberate lie we keep is the **16 mm tall contact box** vs a **4.5 mm** CAD sole — called out, A/B’d, and **not** the Gate E unlock. Hardware sits idle on that plant until learned/RL or AI says otherwise. **Gate D holds on M145; Gate E is still false.**
