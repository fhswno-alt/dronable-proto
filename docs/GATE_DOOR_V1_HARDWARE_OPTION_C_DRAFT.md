# Gate Door v1 — Hardware Option C (INSTALLED)

**INSTALLED** Wed 30 Sep 2026 ~11:15 Europe/London (BST) — Dave chat ACK. Live companion = `…_gate_f_optc.xml` md5 `6a3d4a70d4797b806dcc2580f46468aa`. Soft-pass **OFF**. Park **OFF**. **No spend.** No Prefer FAIL Door SCORE claim. No Door LOCK. Receipt: `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`.

Draft history below is the ~22:48 BST 29 Sep pack. Install did **not** edit the XML (md5 unchanged).

**When (draft):** Tue 29 Sep 2026 ~22:48 Europe/London (BST)  
**Who:** Founding Hardware Engineer  
**Trigger:** Prefer FAIL **STUCK_REPEATED** after fair C01–C04 on Option A (`gate_f_push`) — escalation named in `docs/GATE_DOOR_V1_AI_CRITERIA.md`  
**Scope:** SIM-ONLY companion **fork**. Soft-pass **OFF**. **No spend.**

**Pre-stage (Wed 30 Sep 2026 ~02:11 BST):** install checklist `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_INSTALL_READY.md` was HELD until this ACK. Executed ~11:15 BST. Push-face contact honesty (`docs/GATE_DOOR_V1_HARDWARE_PUSH_FACE_CONTACT_HONESTY.md`) remains a docs-only measurement of the Option B face; it did not write plant bytes.

---

## Live freeze (Option C installed)

| Item | Path | md5 | State |
|------|------|-----|-------|
| **Live Door v1 (Option C)** | `…_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **INSTALLED** — current live pointer |
| Option B archive | `…_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **ARCHIVE** — bytes unchanged; not live |
| Option A archive | `…_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **ARCHIVE** — bytes unchanged |
| Lever-era archive | `…_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** |
| Walk M145 | `…_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** |

---

## Option C fork (this pack)

| Item | Value |
|------|-------|
| Path | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` |
| md5 | `6a3d4a70d4797b806dcc2580f46468aa` |
| State | **INSTALLED** (Dave ACK ~11:15 BST 30 Sep 2026) — was READY-NOT-INSTALLED / HELD at draft |
| Contact | `door_panel` **contype=2 / conaffinity=2 / condim=3** (full panel ↔ `l/r_hand_contact` bit 2) |
| Push-face tell | `door_panel_push_face` demoted to **visual-only** (contype 0) — site KEPT for reach Δ |
| Lever | Still **contype 0** (no knob torque) |
| Hinge / spring | ±30° · damp 0.05 / stiff 0.015 **KEPT** |
| Latch | **None** |

Load check: MuJoCo loads; `nq=33` `nu=24`; panel contype 2; push_face 0; lever 0.

---

## Why

Option A thin −X push face is the honesty surface Controls scored against. C-series Prefer FAIL stuck with bars KEPT (coupled open vs uncoupled phantom / near-miss under 25°). Criteria escalation: Prefer FAIL on thin push face → **Option C full-panel contact** named draft — **not** mid-score invent. Fair set closed → draft OK.

---

## Explicit non-claims

- **Installed** as the live pointer only. Plant XML bytes were **not** edited.
- **Not** Door LOCK / Prefer FAIL SUCCESS or FAIL claim
- **Not** bar soften / soft-pass / park
- **Not** H2 unlock / Gate Q loco pick / spend / PO / STEP fab

---

## Install semantics (Dave ACK ~11:15 BST 30 Sep 2026)

1. Point live companion at `…_gate_f_optc.xml`. Demote Option B `gate_f_optb` to **ARCHIVE** (bytes **KEPT**). Do **not** overwrite A / B / C / walk / lever-era bytes. **Done.** Checklist: `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_INSTALL_READY.md`.  
2. Update freeze sheets + install receipt. **Done.**  
3. Controls Prefer FAIL re-score under `hand × door_panel` **bit 2**. **Not done — no SCORE claim from this install.**  
4. Soft-pass stays **OFF**. Park stays **OFF**. No Door SCORE claim from Hardware.

---

## Dave next-lever menu (Hardware view)

| Option | Hardware note |
|--------|----------------|
| **D-series control-only** | Live plant is now Option C; control-only hold was the other path and was not the ACK |
| **Option C** | This pack — **INSTALLED** |
| **Park Door v1** | **OFF** — not taken |

**One-liner:** Option C full-panel contact fork **INSTALLED** md5 `6a3d4a70…`; live = `gate_f_optc`; Option B `gate_f_optb` `ddf084cd…` **ARCHIVE**; Option A `gate_f_push` `adb24309…` **ARCHIVE**; soft-pass off; park off; no spend; no Door LOCK.
