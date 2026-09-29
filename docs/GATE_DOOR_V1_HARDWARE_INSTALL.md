# Gate Door v1 — Hardware install receipt

**When:** Tue 29 Sep 2026 ~22:08 Europe/London (BST)  
**Who ACK'd:** Dave (voice ACK — install push/pull hospital-door companion NOW)  
**Actor:** Founding Hardware Engineer (box install)  
**Scope:** SIM-ONLY plant-of-record pointer. Soft-pass **OFF**. **No spend / no PO.** No Prefer FAIL Door SCORE claim from Hardware.

---

## Install semantics

| Item | Path | md5 | State |
|------|------|-----|-------|
| **Live companion (Door v1)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **INSTALLED** plant-of-record |
| **Lever-era archive (F–P history)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** — left on disk; **not** overwritten |
| **Walk / M145** | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** / frozen — do not touch |

**Install = point live companion at `…_gate_f_push.xml`.** Did **not** overwrite lever-era `…_gate_f.xml` with push content.

---

## What landed

- Contact **Option A** (bit-2 `door_panel_push_face`; lever contype 0 visual)
- Panel hinge ±30° + Gate J spring kept
- No latch / no knob torque score path
- H2 still **READY-NOT-INSTALLED** (separate Dave pick)
- Soft-pass **off**; spend **none**

---

## Explicit non-claims

- **No Prefer FAIL Door SCORE** from Hardware this install (Controls owns scoring)
- Dave multi-try scoring preference for Controls: Prefer FAIL Door — **no FAIL report on first miss**; multiple attempts; report FAIL only after repeated setbacks; report success; boss present for either outcome
- Gate Q H2 / DCM-VRP-DS / park still **separate** Dave pick
- No STEP/fab / no Path A

---

## Refs

- Directive: `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` (**INSTALLED**)
- Freeze: `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`
- Controls §5: `docs/GATE_DOOR_V1_CONTROLS_COSPEC.md`
- AI no-veto: `docs/GATE_DOOR_V1_AI_COSPEC.md` · criteria `docs/GATE_DOOR_V1_AI_CRITERIA.md`

**One-liner:** Dave ACK ~22:08 BST; live companion = `gate_f_push` md5 `adb24309…`; lever-era `gate_f` archived; walk KEPT; soft-pass off; no spend; no Door SCORE claim.
