# Gate Door v1 — Hardware install receipt

**When:** Wed 30 Sep 2026 ~01:13 Europe/London (BST)  
**Who ACK'd:** Dave (voice ACK — INSTALL Option B as the live Door v1 companion)  
**Actor:** Founding Hardware Engineer (box install)  
**Scope:** SIM-ONLY plant-of-record pointer. Soft-pass **OFF**. **No spend.** No Prefer FAIL Door SCORE claim from Hardware. No Door LOCK. No STEP/fab.

**Prior receipt (demoted):** Tue 29 Sep 2026 ~22:08 BST pointed live at Option A `…_gate_f_push.xml`. That file is now **ARCHIVE**. Its bytes were **not** overwritten and were **not** reinstalled as live.

---

## Install semantics

| Item | Path | md5 | State |
|------|------|-----|-------|
| **Live companion (Door v1)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **INSTALLED** plant-of-record (Option B, bit 4) |
| **Option A archive** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **ARCHIVE** — bytes unchanged; **not** live |
| **Option C** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **READY-NOT-INSTALLED / HELD** — not installed |
| **Lever-era archive (F–P history)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** — left on disk; **not** overwritten |
| **Walk / M145** | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** / frozen — do not touch |

**Install = point live companion at `…_gate_f_optb.xml`.** Did **not** copy Option B over `…_gate_f_push.xml`. Did **not** overwrite lever-era `…_gate_f.xml` or the walk plant. Did **not** install Option C.

---

## What landed

- Contact **Option B** (bit-4 `door_panel_push_face` contype/conaffinity 4; `l/r_hand_contact` retargeted to bit 4)
- Hands pair with `door_panel_push_face`. Hands do **not** pair with `door_panel`, `door_lever`, or feet
- Panel visual (contype 0) · lever visual (contype 0) · feet bit 1 **KEPT**
- Panel hinge ±30° + Gate J spring kept (damp **0.05** / stiff **0.015**)
- No latch / no knob torque score path
- H2 still **READY-NOT-INSTALLED** (separate Dave pick)
- Soft-pass **off**; spend **none**

---

## Load check (this install)

| Plant | md5 | MuJoCo |
|-------|-----|--------|
| Live `…_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **LOAD OK** · nq=**33** · nu=**24** |
| Archive `…_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` (unchanged) | **LOAD OK** · nq=**33** · nu=**24** (archived Option A still loads; live scoring uses optb) |

Contact pairing on optb (bitmask): `l/r_hand_contact` (4) × `door_panel_push_face` (4) = **pair**. × `door_panel` / `door_lever` / `l_foot_contact` / `r_foot_contact` = **no pair**.

---

## Explicit non-claims

- **No Prefer FAIL Door SCORE** from Hardware this install (Controls owns scoring). No SUCCESS and no FAIL claim.
- **No Door LOCK**
- Soft-pass **OFF**
- Dave multi-try scoring preference for Controls: Prefer FAIL Door — **no FAIL report on first miss**; multiple attempts; report FAIL only after repeated setbacks; report success; boss present for either outcome
- Prefer FAIL bars stay as written in `docs/GATE_DOOR_V1_AI_CRITERIA.md` (closed ≤2° · contact ≥0.3 s · open ≥25° · hold ≥1 s @ ≥20° · 2/2). This install does not soften them.
- Gate Q H2 / DCM-VRP-DS / park still **separate** Dave pick
- No STEP/fab

---

## Refs

- Option B pack: `docs/GATE_DOOR_V1_HARDWARE_OPTION_B_DRAFT.md` (**INSTALLED**)
- Option A directive (archive history): `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md`
- Freeze: `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`
- Controls §5 (Option A cospec, historical): `docs/GATE_DOOR_V1_CONTROLS_COSPEC.md`
- AI criteria: `docs/GATE_DOOR_V1_AI_CRITERIA.md`
- Option C held: `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_DRAFT.md`

**One-liner:** Dave ACK ~01:13 BST 30 Sep 2026; live companion = `gate_f_optb` md5 `ddf084cd…` (**INSTALLED**); Option A `gate_f_push` md5 `adb24309…` **ARCHIVE** (bytes unchanged); Option C **HELD**; lever-era `gate_f` archived; walk KEPT; soft-pass off; no spend; no Door SCORE claim.
