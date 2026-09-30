# Gate Door v1 — Hardware Option C install checklist (INSTALLED)

**INSTALLED** — Dave chat ACK ~11:15 BST Wed 30 Sep 2026. Live companion = `…_gate_f_optc.xml` md5 `6a3d4a70d4797b806dcc2580f46468aa`.

**Pre-staged:** Wed 30 Sep 2026 ~02:11 Europe/London (BST)  
**Installed:** Wed 30 Sep 2026 ~11:15 Europe/London (BST)  
**Who:** Founding Hardware Engineer  
**Scope:** Pointer install executed. Soft-pass **OFF**. Park **OFF**. **No spend.** No plant XML bytes written. No Prefer FAIL Door SCORE claim from Hardware. No Door LOCK. No STEP/fab.

This file was the checklist before the named ACK. The ACK executed it. The install record is `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`.

**Live:** Option C `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` md5 `6a3d4a70d4797b806dcc2580f46468aa` (**INSTALLED**). Option B is **ARCHIVE**.

---

## Current freeze (Option C installed)

| Item | Path | md5 | State |
|------|------|-----|-------|
| **Live companion (Door v1)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **INSTALLED** plant-of-record (Option C, bit 2) |
| **Option B archive** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **ARCHIVE** — bytes unchanged; **not** live |
| **Option A archive** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **ARCHIVE** — bytes unchanged; **not** live |
| **Lever-era archive (F–P history)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** — left on disk; **not** overwritten |
| **Walk / M145** | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** / frozen — do not touch |

md5 locks above were re-checked on disk this install. All five files match. No XML write.

---

## What Option C is (installed fork, bytes unchanged)

Full-panel contact. Not a bigger invent tonight — the fork already exists.

| Item | Value |
|------|-------|
| Path | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` |
| md5 | `6a3d4a70d4797b806dcc2580f46468aa` |
| `door_panel` | **contype 2 / conaffinity 2** (bit 2) — contactable panel volume |
| `door_panel_push_face` | **visual 0** (contype 0 / conaffinity 0) — site kept for reach Δ |
| `l/r_hand_contact` | **bit 2** (contype 2 / conaffinity 2) — pair with `door_panel`, not the visual push face |
| Lever | contype 0 (no knob torque) |
| Hinge / spring | ±30° · damp **0.05** / stiff **0.015** **KEPT** |
| Latch | **None** |

Contact pair **now live** is `l/r_hand_contact` × `door_panel` (bit 2). Push-face geom is visual-only and is **not** the contact pair. Numeric Prefer FAIL bars in `docs/GATE_DOOR_V1_AI_CRITERIA.md` were **not** rewritten and were **not** softened. No Door SCORE was claimed.

---

## Install checklist (executed ~11:15 BST 30 Sep 2026)

Dave named ACK: install Option C. Soft-pass off. Park off. No spend.

1. **Point live → `gate_f_optc`.** Live companion is `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` md5 `6a3d4a70d4797b806dcc2580f46468aa`. Install changed the plant-of-record **pointer**. Did **not** copy Option C bytes over any other file. **DONE.**
2. **Demote Option B to ARCHIVE.** `…_gate_f_optb.xml` md5 `ddf084cdac71cb0998aa6a44a65594c0` left the live pointer and is **ARCHIVE**. Bytes **KEPT**. Not deleted, not edited, not reinstalled as live. **DONE.**
3. **Do not overwrite A / B / C / walk / lever-era.** No writes:
   - Option A `…_gate_f_push.xml` (`adb24309b489d56615c194e92676d040`) stays **ARCHIVE**
   - Option B `…_gate_f_optb.xml` (`ddf084cd…`) bytes **KEPT**
   - Option C `…_gate_f_optc.xml` (`6a3d4a70…`) bytes **KEPT** — pointer at the existing file
   - Walk `…_m2_145.xml` (`fc94709c84f5598d4474ecfc4bb41fdc`) **KEPT**
   - Lever-era `…_gate_f.xml` (`59cc408eda07037a58f92ad27da045d6`) **ARCHIVE**
4. **Soft-pass stays OFF.** No bar soften. Prefer FAIL bars stay as written (closed ≤2° · contact ≥0.3 s · open ≥25° · hold ≥1 s @ ≥20° · 2/2). **DONE.**
5. **No Door SCORE claim from Hardware.** No SUCCESS and no FAIL claim on the install itself. Controls owns scoring. **DONE (no claim).**
6. **Prefer FAIL re-score under hand × `door_panel` bit 2.** Score default path now locks optc and the hand × `door_panel` bit-2 fingerprint. Controls has **not** re-scored. Push-face geom is visual-only and is **not** the contact pair. Multi-try rule unchanged: no FAIL report on first miss; FAIL only after repeated setbacks; report success; boss present for either outcome. **NOT DONE — no SCORE from this install.**
7. **Paper trail.** Live install receipt, freeze sheets, and Option C draft state moved from HELD to **INSTALLED**. **DONE.**

Park was **not** taken.

---

## Why this checklist existed

Overnight context (not a Hardware SCORE): D-series Prefer FAIL **STUCK** (phantom PARTIAL). D-twin Prefer FAIL was spinning **control-only**. AI/Controls reported intermittent contact on the thin −X push face. Option B bit-4 alone did not clear phantoms — bit 4 isolates which geom can earn contact credit; it does not enlarge the face. Geometry honesty: `docs/GATE_DOOR_V1_HARDWARE_PUSH_FACE_CONTACT_HONESTY.md`.

Option C is the already-drafted full-panel escalation. Dave ACK ~11:15 BST installed it. Park was **OFF**. No mid-score invent. No plant byte write.

---

## Explicit non-claims

- **Installed** as the live pointer. Option C is **INSTALLED**.
- **Not** a plant XML edit. A, B, C, walk, and lever-era bytes were not overwritten.
- **Not** a Door LOCK.
- **Not** a Prefer FAIL Door SCORE (no SUCCESS, no FAIL) from Hardware.
- **Not** a soft-pass. Bars **KEPT**. Park **OFF**.
- **Not** spend / PO / STEP / fab / H2 unlock / Gate Q loco pick.

---

## Refs

- Option C draft (**INSTALLED**): `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_DRAFT.md`
- Push-face contact honesty (docs only, pre-install measurement): `docs/GATE_DOOR_V1_HARDWARE_PUSH_FACE_CONTACT_HONESTY.md`
- Install receipt (live): `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`
- Option B pack (**ARCHIVE**): `docs/GATE_DOOR_V1_HARDWARE_OPTION_B_DRAFT.md`
- Freeze: `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`
- AI criteria (numeric bars KEPT; not a SCORE): `docs/GATE_DOOR_V1_AI_CRITERIA.md`

**One-liner:** Option C install checklist **INSTALLED** ~11:15 BST 30 Sep 2026; live = `gate_f_optc` `6a3d4a70…`; Option B `ddf084cd…` ARCHIVE; Option A `adb24309…` ARCHIVE; walk `fc94709c…` KEPT; full-panel = `door_panel` contype 2, push_face visual 0, hands bit 2; soft-pass off; park off; no spend; no Door SCORE claim.
