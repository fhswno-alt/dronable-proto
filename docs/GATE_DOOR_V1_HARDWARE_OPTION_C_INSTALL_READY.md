# Gate Door v1 — Hardware Option C install receipt (pre-staged)

**READY-NOT-INSTALLED / HELD — do NOT install until Dave named ACK**

**When:** Wed 30 Sep 2026 ~02:11 Europe/London (BST)  
**Who:** Founding Hardware Engineer (overnight prep)  
**Scope:** DOCS-ONLY pre-stage. Soft-pass **OFF**. **No spend.** **Option C is not installed.** Live plant pointer **unchanged**. No plant XML bytes written. No Prefer FAIL Door SCORE claim from Hardware. No Door LOCK. No STEP/fab.

This file is the install checklist **before** a named ACK. It is not an install record. Do not treat it as permission to point live at Option C.

**Live tonight:** Option B `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml` md5 `ddf084cdac71cb0998aa6a44a65594c0` (**INSTALLED**, Dave ACK ~01:13 BST 30 Sep 2026). Receipt of that install: `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`.

---

## Current freeze (Option C still held)

| Item | Path | md5 | State |
|------|------|-----|-------|
| **Live companion (Door v1)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **INSTALLED** plant-of-record (Option B, bit 4) |
| **Option C (this checklist)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **READY-NOT-INSTALLED / HELD** — do **not** install |
| **Option A archive** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **ARCHIVE** — bytes unchanged; **not** live |
| **Lever-era archive (F–P history)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** — left on disk; **not** overwritten |
| **Walk / M145** | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** / frozen — do not touch |

md5 locks above were re-checked on disk this prep. All five files match. No XML write.

---

## What Option C is (held fork, already on disk)

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

Contact pair **after** a future install would be `l/r_hand_contact` × `door_panel` (bit 2). Until Dave named ACK, the **live** declared pair stays Option B: hands × `door_panel_push_face` (bit 4). AI criteria in `docs/GATE_DOOR_V1_AI_CRITERIA.md` still name that live push-face pair. This receipt does not edit those bars.

---

## Install checklist (mirror of Option B install semantics)

Execute **only** after a Dave **named ACK** that says install Option C. Every line below is **NOT DONE**.

1. **Point live → `gate_f_optc`.** Live companion becomes `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` md5 `6a3d4a70d4797b806dcc2580f46468aa`. Install means change the plant-of-record **pointer**. Do **not** copy Option C bytes over any other file. **NOT DONE.**
2. **Demote Option B to ARCHIVE.** `…_gate_f_optb.xml` md5 `ddf084cdac71cb0998aa6a44a65594c0` leaves the live pointer and becomes **ARCHIVE**. Bytes **KEPT**. Do not delete, edit, or reinstall it as live. **NOT DONE.**
3. **Do not overwrite A / B / C / walk / lever-era.** Forbidden writes:
   - Option A `…_gate_f_push.xml` (`adb24309b489d56615c194e92676d040`) stays **ARCHIVE**
   - Option B `…_gate_f_optb.xml` (`ddf084cd…`) bytes **KEPT** (state change is pointer-only, and only after ACK)
   - Option C `…_gate_f_optc.xml` (`6a3d4a70…`) bytes **KEPT** — point at the existing file
   - Walk `…_m2_145.xml` (`fc94709c84f5598d4474ecfc4bb41fdc`) **KEPT**
   - Lever-era `…_gate_f.xml` (`59cc408eda07037a58f92ad27da045d6`) **ARCHIVE**
4. **Soft-pass stays OFF.** No bar soften. Prefer FAIL bars stay as written (closed ≤2° · contact ≥0.3 s · open ≥25° · hold ≥1 s @ ≥20° · 2/2).
5. **No Door SCORE claim from Hardware.** No SUCCESS and no FAIL claim on the install itself. Controls owns scoring.
6. **Prefer FAIL re-score under hand × `door_panel` bit 2.** After the pointer moves, Controls re-scores Prefer FAIL against `l/r_hand_contact` × `door_panel` (bit 2). Push-face geom is visual-only on Option C and is **not** the contact pair. Multi-try rule unchanged: no FAIL report on first miss; FAIL only after repeated setbacks; report success; boss present for either outcome. **NOT DONE — no re-score from this prep.**
7. **Paper trail after ACK only.** Update the live install receipt, freeze sheet, and Option C draft state from HELD to INSTALLED. Do that in the install turn, not in this pre-stage.

Until step 1 happens, live remains Option B.

---

## Why this checklist exists and why it is held

Overnight context (not a Hardware SCORE): D-series Prefer FAIL **STUCK** (phantom PARTIAL). D-twin Prefer FAIL is spinning **control-only**. AI/Controls report intermittent contact on the thin −X push face. Option B bit-4 alone did not clear phantoms — bit 4 isolates which geom can earn contact credit; it does not enlarge the face. Geometry honesty: `docs/GATE_DOOR_V1_HARDWARE_PUSH_FACE_CONTACT_HONESTY.md`.

Option C is the already-drafted full-panel escalation. It stays **HELD** until Dave names the ACK. Control-only hold (Hardware idle on live Option B) remains the other path. No mid-score invent.

---

## Explicit non-claims

- **Not installed.** Option C is **READY-NOT-INSTALLED / HELD**.
- **Not** a live pointer change. Live stays Option B `gate_f_optb` / `ddf084cd…`.
- **Not** a plant XML edit. A, B, C, walk, and lever-era bytes were not overwritten.
- **Not** a Door LOCK.
- **Not** a Prefer FAIL Door SCORE (no SUCCESS, no FAIL) from Hardware.
- **Not** a soft-pass. Bars **KEPT**.
- **Not** spend / PO / STEP / fab / H2 unlock / Gate Q loco pick.

---

## Refs

- Option C draft (still HELD): `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_DRAFT.md`
- Push-face contact honesty (docs only): `docs/GATE_DOOR_V1_HARDWARE_PUSH_FACE_CONTACT_HONESTY.md`
- Option B install (live): `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`
- Option B pack: `docs/GATE_DOOR_V1_HARDWARE_OPTION_B_DRAFT.md`
- Freeze: `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`
- AI criteria (live pair still push-face): `docs/GATE_DOOR_V1_AI_CRITERIA.md`

**One-liner:** Option C install checklist **PRE-STAGED / HELD** — do **not** install until Dave named ACK; live stays Option B `gate_f_optb` `ddf084cd…`; optc `6a3d4a70…` bytes unchanged; Option A `adb24309…` ARCHIVE; walk `fc94709c…` KEPT; full-panel = `door_panel` contype 2, push_face visual 0, hands bit 2; soft-pass off; no spend; no Door SCORE claim.
