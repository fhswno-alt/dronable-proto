# Gate Q — Hardware honesty: planted middle under frozen M145 + companion

**When:** Mon 28 Sep 2026 ~14:44 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**To:** Controls · AI · Dave · room  
**Scope:** READ-ONLY plant honesty one-pager. **No XML edit. No STEP invent. No criteria soften. No Path A / spend / PO.**

Aligns with: `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` (~14:22) · `docs/GATE_Q_AI_CRITERIA.md` · `docs/HARDWARE_FREEZE_STATUS_MONDAY.md` · Controls `previews/ainex_walk/iterate/GATE_Q_A28_RETOK_DIAG.md` / `GATE_Q_CONTROLS_NOTE.md`

---

## Disposition (Hardware)

Gate Q is **Prefer FAIL — structural** (AI ~14:22). Residual-family wall: destination cam on stepped retreat still comes from a **planted middle** (Root B), not from clear-foot Δ. Criteria **KEPT** (skate ≤0.08/0.18, clear_frac≥0.55 — no soften). Soft-pass **off**.

**Hardware posture:** body is fixed. This note explains what "planted middle" means in plant terms, why Hardware will **not** invent plant soft-fixes to clear Q, and what controller-facing observations follow from existing metrics — so Controls/AI can reopen only with a **retreat-native controller family**, not a plant wall.

Curriculum **D–P TRUE** (sim). Gate Q **OPEN — Prefer FAIL parked** (not lock TRUE, not LOCKED FALSE).

---

## 1. What "planted middle" means in plant terms

On Gate Q retreat under frozen M145 + companion, Controls PRE_GAP (H0 / E7lock) measures a long **both-soles-down** window between the first SS cluster and the late reburst:

| Bout | planted_middle_s | cam@preSS | dxc_pre | cam_gain_in_gap | Cite |
|------|------------------|-----------|---------|-----------------|------|
| 0 | **34.9** (34.88) | 0.159 | 0.057 | **0.176** | AI Prefer FAIL · A28 RETOK H0 |
| 1 | **26.7** | 0.162 | 0.074 | **0.167** | same |

**Plant reading (not a controller ask):**

| Phrase | Plant meaning |
|--------|----------------|
| **Feet down** | Both `l_foot_contact` / `r_foot_contact` boxes stay flush on floor (SOLE_OFFSET 0.026; clear above plant rest ≪ 2 cm SS credit bar). No visible sole daylight; no dwell-clear. |
| **CoP / contact-XY shove** | Free-joint XY advances while soles are planted — contact tangential forces + residual/gait soft-XY move cam→door. Controls already gates this as **not** stepped Δ (`dx_plant` vs `dx_clear`; contact-XY cancel×0.70 KEPT — do not reopen ↑). |
| **vs clear step** | Honest stepped retreat needs sole clear ≥2 cm above plant rest, dwell ≥150–250 ms, multi-frame daylight, and ≥~70% of retreat cam/base Δ while clear (`docs/GATE_Q_AI_CRITERIA.md` §5). First cluster only banks **cam~0.16 / dxc~0.07** — far short of dest cam ≳0.45–0.55 from clear. Remaining climb is **plant ≫ clear**. |

That continuous cam-from-plant window is **Root B**. Softening skate / clear_frac / planted soft-XY would credit the planted middle as stepped retreat. AI forbids it. Hardware agrees — and will not change the plant to make the slide look like a step.

E7lock best metrics (for context only — still FAIL): bout1 ret cf **0.649** + joint HIT; skate **0.098/0.221** blocks `ret_ok`; bout0 ret cf **0.468**. Cite: `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md`.

---

## 2. Why Hardware will NOT invent plant fixes to "help" Q

| Tempting plant change | Why Hardware refuses |
|-----------------------|----------------------|
| Soft-XY / solref-solimp softsole / GEO03 reopen | Softens contact so planted shove carries cam with less skate signature — credits Root B as stepped. GEO contact honesty pack stays **HARD-FALSIFIED / frozen**; no GEO reopen without Dave + AI cospec. |
| Foot-lift geom / taller contact / STEP invent | Fake sole clearance or CAD stack change would trip "foot-lift" without a real swing-clear controller. MFG correctly: inventing STEP/geom fights Prefer FAIL. Foot STEP stays **145×86 / ~4.5 mm CAD**; XML **16 mm** = sim proxy only. |
| Friction bump / floor or foot µ edit | Changes skate bar physics without clear Δ. Locked friction `1.6 0.1 0.01` (floor + feet) stays. |
| Hinge / spring / range bump (±30° → wider, spring softer/stiffer) | Out of Gate Q scope. Panel range bump = Alt A **deferred**. Q is retreat locomotion honesty, not door open. |
| Contact-box resize / planform | Would reopen M145 walk plant that Gate E won. **Forbidden.** |

**Rule:** any plant edit that makes planted-middle cam look like stepped retreat is a soft-pass by another name. Hardware will not ship it. Reopen path is **controller-only**: genuinely new retreat-native residual schedule under same companion md5 + frozen ckpt + cancel×0.70 + soft-pass off (AI Prefer FAIL §Next disposition).

---

## 3. Frozen plant facts (body fixed — Controls can trust these)

Verified live this turn (~14:44 BST):

| Plant | Path | md5 |
|-------|------|-----|
| **Walk / Gate E (M145)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` |
| **Companion F–Q** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` |

### Foot / contact (M145 = companion feet; HX untouched)

| Fact | Value | Source |
|------|-------|--------|
| Planform | **145 × 86 mm** (half `0.0725 × 0.0430`) | `docs/PLANT_CONTACT_HONESTY_PACK.md` GEO00 |
| Contact box height | **16 mm** XML proxy (half `0.008`) — **not** fab target | same · `docs/M2_FAB_QC_FROZEN_145.md` |
| Contact `pos` | `0.030 0.0 -0.018` | same |
| Sole bottom / `SOLE_OFFSET` | **−0.026** / **0.026** | same |
| Friction (floor + feet) | `1.6 0.1 0.01` | same |
| Soft params | MuJoCo defaults — **no** geom `solref`/`solimp` on locked score plant | same |
| CAD STEP | 145×86 · stack **~4.5 mm** — **FROZEN**; no PO | `docs/M2_FAB_QC_FROZEN_145.md` |

### Panel / lever (companion only)

| Fact | Value | Source |
|------|-------|--------|
| `door_panel_hinge` range | **±0.5236 rad (±30°)** — **unchanged** | `docs/GATE_I_HARDWARE_PANEL_HINGE.md` |
| Panel spring (Gate J) | damp **0.05** / stiff **0.015** · springref 0 | `docs/GATE_J_HARDWARE_PANEL_SPRING.md` |
| Lever child-of-panel | `door_lever_link` parent = `door_panel_link` | Gate I/J |
| Lever hinge | damp 0.02 / stiff 0.05 · range ±30° | Gate H/I |
| Free-edge stripe | visual-only (Gate K); contype 0 | `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` |
| Panel actuator | **none** | Gate J honesty |

### Stack pins Controls already holds

| Pin | Value |
|-----|-------|
| Gate E ckpt sha16 | `9ffaa1a21b607bf6` |
| Contact-XY cancel | ×0.70 (do not reopen ↑) |
| CLEAR_TRACK_SWING | 1 KEEP |
| Abandoned | FREEZE / qvel0 / damp↑ / kd↑ / gap-planted |

**Hardware will not edit either XML for Gate Q.** Confirm plant-of-record only.

---

## 4. Controller-facing observations (NOT plant change asks)

Hardware reading of existing PRE_GAP / E7lock metrics — for Controls/AI brainstorm of a **retreat-native** family. None of these ask for geom/STEP/friction/range:

1. **Cam gain is during plant ≫ clear.** Both bouts: `cam_gain_in_gap` ~0.17 while first cluster only banks cam@preSS ~0.16. Destination cam is earned in the planted middle, not in SS clear windows. A retreat-native schedule must put dest cam into **clear Δ before** that gap (or eliminate the long DS plant entirely) — without cancel↑ / FREEZE / gap-planted.

2. **First SS cluster is real but under-banked.** Dense lifts exist (bout1 ~166–169 s per E7lock tip) yet `dxc_pre` ~0.07 ≪ 0.15 target. Observation: clear windows are short relative to the 26–35 s plant middle — controller problem (authority / phasing / clear-through), not missing foot geom.

3. **Skate is the Root B signature of planted travel.** E7lock skate 0.096–0.098 / 0.221–0.240 fails ≤0.08/0.18 precisely where planted_middle_s is long. Softening the bar would hide plant shove; Hardware will not change µ to hide it either.

4. **Cancel×0.70 + frozen residual already define the body+contact contract.** Fair tip set H1–H10 (front-load / clear-through) did not beat E7lock under that contract (`GATE_Q_A28_RETOK_DIAG.md`). Next family must be **new**, not a rename of abandoned levers — still on these md5s.

---

## 5. Explicit non-claims / freeze line

| Claim | Verdict |
|-------|---------|
| Demo / investor walk / room-walk / full door open / latch / 90° / walk-through / UK | **No** |
| Gates **D–P** | **LOCKED TRUE** (sim) — unchanged |
| Gate **Q** | **Prefer FAIL parked** — criteria KEPT; not lock TRUE; not LOCKED FALSE |
| Soft-pass / skate soften / clear_frac soften / planted soft-XY credit | **Off / forbidden** |
| Plant invent / STEP invent / Path A / spend / PO | **No** |
| Hardware plant XML edit this turn | **None** — md5s must match §3 after any Hardware run |

**Dave one-liner:** Hardware confirms planted middle = feet-down contact-XY cam shove under frozen M145+companion; will not invent soft-XY/foot-lift/friction/range to clear Q. Body fixed (md5s above). Controls/AI: reopen only with retreat-native controller family. Soft-pass off. No spend.

**Refs:** `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` · `docs/GATE_Q_AI_CRITERIA.md` · `docs/GATE_Q_AI_TIP_E7lock.md` · `docs/AI_GATE_Q_STATUS_MONDAY.md` · `docs/HARDWARE_FREEZE_STATUS_MONDAY.md` · `docs/MFG_FREEZE_STATUS_MONDAY.md` · `docs/PLANT_CONTACT_HONESTY_PACK.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md` · `docs/GATE_I_HARDWARE_PANEL_HINGE.md` · `docs/M2_FAB_QC_FROZEN_145.md` · `previews/ainex_walk/iterate/GATE_Q_A28_RETOK_DIAG.md` · `previews/ainex_walk/iterate/GATE_Q_CONTROLS_NOTE.md`
