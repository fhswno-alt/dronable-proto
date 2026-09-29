# Hardware — Dave solo handoff (usage limits)

**When:** Mon 28 Sep 2026 ~19:13 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Purpose:** Keep Hardware progress honest if Grok Bot hits usage limits. **Not** a plant invent / Path A / PO. Soft-pass off.

Live freeze sheet: `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`  
Planted-middle honesty: `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md`  
MFG twin handoff: `docs/MFG_DAVE_USAGE_LIMIT_HANDOFF.md`  
Google Doc: https://docs.google.com/document/d/16_kH44UaOuSKz-JIWu_zYsmZTr_M8F4bNe0Nr8YXWG4/edit

---

## Current Hardware state (do not invent)

| Item | State |
|------|--------|
| Walk plant M145 | **FROZEN** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` · md5 `fc94709c84f5598d4474ecfc4bb41fdc` · planform **145×86** · contact/HX untouched |
| Companion F–Q | **FROZEN** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` · md5 `59cc408eda07037a58f92ad27da045d6` · panel hinge ±30° · spring damp **0.05** / stiff **0.015** · lever child-of-panel · Gate K free-edge stripe (visual-only) |
| Foot STEP / CAD | **FROZEN** 145×86 · ~4.5 mm CAD stack (XML 16 mm = sim proxy only — do not fab to 16 mm) |
| Curriculum | **D–P TRUE** (sim). **Gate Q** Prefer FAIL iterate (Controls/AI) — **not** a Hardware unlock by itself |
| Soft-pass / Path A / PO | **OFF** until you explicitly say otherwise |

---

## What you can do alone (no bots)

### A. Protect the freeze (default)

1. Do **not** edit either plant XML or invent soft-XY / friction / foot-lift / range / STEP dims to clear Gate Q.
2. Re-check md5s anytime:
   - `md5sum mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` → expect `fc94709c84f5598d4474ecfc4bb41fdc`
   - `md5sum mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` → expect `59cc408eda07037a58f92ad27da045d6`
3. Read `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` before any temptation to “help” Q with plant.

### B. If Controls/AI name a real geom ask

Only when they name a concrete change (e.g. sole friction tag, visual tell, range bump with AI cospec):

1. Require the **named** ask in writing + AI criteria still KEPT (no soft-XY credit).
2. Edit companion first unless they explicitly need M145.
3. Update `docs/HARDWARE_FREEZE_STATUS_MONDAY.md` with new md5 + ping MFG for QC.
4. Do **not** soften skate / clear_frac bars via plant.

### C. Honest review you can run without bots

| Check | How |
|-------|-----|
| Freeze sheet | Open `docs/HARDWARE_FREEZE_STATUS_MONDAY.md` |
| Door hinge / spring | `docs/GATE_I_HARDWARE_PANEL_HINGE.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md` |
| Edge stripe (K) | `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` |
| Planted middle (Q) | `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` |
| FOV / look | Locked walk + companion stills under Controls FOV notes — do not retune head without AI |

### D. Vision Pro (later, off critical path)

When you want spatial preview: ask Hardware for USD/USDZ of **frozen** M145 + companion meshes / FOV camera pose (with Controls trajectories). No fab/PO. Do not block Gate Q on this.

---

## If limits hit mid Gate Q

1. Leave plants frozen — Q Prefer FAIL is a **controller** problem unless someone freezes new dims with AI cospec.
2. Your only Hardware decision is: **keep freeze** (default) vs **explicit named plant ask** after reading the honesty note.
3. Ping Founding Hardware Engineer in DM when limits reset with: (a) any plant XML you edited + new md5, (b) any geom you want ship/QC, (c) Vision Pro pack yes/no.

---

## Do **not** do while waiting

- Soften Gate Q bars / invent plant to “look busy”
- Reopen GEO contact honesty / softsole without Dave + AI cospec
- Fab soles to the 16 mm sim contact thickness
- Assume Prefer FAIL park = Path A or plant unlock
- Spend / PO / Path A thaw

---

## One-line Hardware policy

**Body stays frozen; spend stays off. When bots are dark, protect the md5s and only change plant if you deliberately name the geom.**
