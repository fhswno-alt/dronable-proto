# Gate Door v1 — Hardware install receipt

**When:** Wed 30 Sep 2026 ~11:15 Europe/London (BST)  
**Who ACK'd:** Dave (chat ACK — INSTALL Option C as the live Door v1 companion)  
**Actor:** Founding Hardware Engineer (box install)  
**Scope:** SIM-ONLY plant-of-record pointer. Soft-pass **OFF**. Park **OFF**. **No spend.** No Prefer FAIL Door SCORE claim from Hardware. No Door LOCK. No STEP/fab.

**Prior receipt (demoted):** Wed 30 Sep 2026 ~01:13 BST pointed live at Option B `…_gate_f_optb.xml`. That file is now **ARCHIVE**. Its bytes were **not** overwritten and were **not** reinstalled as live.  
**Earlier receipt:** Tue 29 Sep 2026 ~22:08 BST pointed live at Option A `…_gate_f_push.xml`. That file stays **ARCHIVE**. Bytes unchanged.

---

## Install semantics

| Item | Path | md5 | State |
|------|------|-----|-------|
| **Live companion (Door v1)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **INSTALLED** plant-of-record (Option C, full panel, bit 2) |
| **Option B archive** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **ARCHIVE** — bytes unchanged; **not** live |
| **Option A archive** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **ARCHIVE** — bytes unchanged; **not** live |
| **Lever-era archive (F–P history)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** — left on disk; **not** overwritten |
| **Walk / M145** | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** / frozen — do not touch |

**Install = point live companion at `…_gate_f_optc.xml`.** Did **not** copy Option C over Option A, Option B, lever-era `…_gate_f.xml`, or the walk plant. Did **not** overwrite any plant XML bytes.

---

## What landed

- Contact **Option C** (full panel): `door_panel` **contype 2 / conaffinity 2** (bit 2)
- `l/r_hand_contact` stay **bit 2** and pair with `door_panel`
- `door_panel_push_face` is **visual 0** (contype 0 / conaffinity 0) — site kept for reach Δ; **not** the contact pair
- Hands do **not** pair with `door_panel_push_face`, `door_lever`, or feet
- Lever visual (contype 0) · feet bit 1 **KEPT**
- Panel hinge ±30° + Gate J spring kept (damp **0.05** / stiff **0.015**)
- No latch / no knob torque score path
- H2 still **READY-NOT-INSTALLED** (separate Dave pick)
- Soft-pass **off**; park **off**; spend **none**

---

## Load check (this install)

| Plant | md5 | MuJoCo |
|-------|-----|--------|
| Live `…_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **LOAD OK** · nq=**33** · nu=**24** |
| Archive `…_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` (unchanged) | bytes **KEPT** — not reloaded as live |
| Archive `…_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` (unchanged) | bytes **KEPT** — not live |

Contact pairing on optc (bitmask): `l/r_hand_contact` (2) × `door_panel` (2) = **pair**. × `door_panel_push_face` / `door_lever` / `l_foot_contact` / `r_foot_contact` = **no pair**.

Honesty fingerprint locked in `scripts/score_door_v1.py`: panel contype **2**, push_face visual **0**, hand × `door_panel` bit 2. Score default plant is optc md5 `6a3d4a70…`. This install did **not** run a Prefer FAIL Door SCORE.

---

## Explicit non-claims

- **No Prefer FAIL Door SCORE** from Hardware this install (Controls owns scoring). No SUCCESS and no FAIL claim.
- **No Door LOCK**
- Soft-pass **OFF**
- Park **OFF** — Door v1 is installed, not parked
- Dave multi-try scoring preference for Controls: Prefer FAIL Door — **no FAIL report on first miss**; multiple attempts; report FAIL only after repeated setbacks; report success; boss present for either outcome
- Prefer FAIL numeric bars stay as written in `docs/GATE_DOOR_V1_AI_CRITERIA.md` (closed ≤2° · contact ≥0.3 s · open ≥25° · hold ≥1 s @ ≥20° · 2/2). This install does not soften them. The score-script contact geom for a later Controls run is `door_panel` (bit 2), not the visual push face.
- Gate Q H2 / DCM-VRP-DS / park still **separate** Dave pick
- No STEP/fab

---

## Refs

- Option C pack: `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_DRAFT.md` (**INSTALLED**)
- Option C checklist (executed): `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_INSTALL_READY.md`
- Option B pack (archive): `docs/GATE_DOOR_V1_HARDWARE_OPTION_B_DRAFT.md`
- Option A directive (archive history): `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md`
- Freeze: `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`
- AI criteria: `docs/GATE_DOOR_V1_AI_CRITERIA.md`

**One-liner:** Dave ACK ~11:15 BST 30 Sep 2026; live companion = `gate_f_optc` md5 `6a3d4a70…` (**INSTALLED**); Option B `gate_f_optb` md5 `ddf084cd…` **ARCHIVE** (bytes unchanged); Option A `gate_f_push` md5 `adb24309…` **ARCHIVE**; lever-era `gate_f` `59cc408e…` KEPT; walk `fc94709c…` KEPT; hand × `door_panel` bit 2; push_face visual 0; soft-pass off; park off; no spend; no Door SCORE claim.
