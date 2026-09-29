# Gate Door v1 — Hardware Option C draft (READY-NOT-INSTALLED)

**When:** Tue 29 Sep 2026 ~22:48 Europe/London (BST)  
**Who:** Founding Hardware Engineer  
**Trigger:** Prefer FAIL **STUCK_REPEATED** after fair C01–C04 on Option A (`gate_f_push`) — escalation named in `docs/GATE_DOOR_V1_AI_CRITERIA.md`  
**Scope:** SIM-ONLY companion **fork**. Soft-pass **OFF**. **No spend / no PO.** **NOT installed.** Live freeze KEPT.

---

## Live freeze (unchanged)

| Item | Path | md5 | State |
|------|------|-----|-------|
| Live Door v1 (Option A) | `…_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **INSTALLED / KEPT** |
| Lever-era archive | `…_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** |
| Walk M145 | `…_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** |

---

## Option C fork (this pack)

| Item | Value |
|------|-------|
| Path | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` |
| md5 | `6a3d4a70d4797b806dcc2580f46468aa` |
| State | **READY-NOT-INSTALLED** |
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

- **Not installed** — needs Dave ACK + AI/Controls cospec before any score plant swap
- **Not** Door LOCK / Prefer FAIL SUCCESS claim
- **Not** bar soften / soft-pass / plant invent on live `gate_f_push`
- **Not** H2 unlock / Gate Q loco pick / spend / PO / STEP fab

---

## Install semantics (only after Dave ACK)

1. Point live companion at `…_gate_f_optc.xml` (do **not** overwrite `gate_f_push` or lever-era archive).  
2. Update freeze sheets + install receipt.  
3. Controls re-score Prefer FAIL vs updated contact pair (`hand × door_panel`).  
4. Soft-pass stays **OFF**.

---

## Dave next-lever menu (Hardware view)

| Option | Hardware note |
|--------|----------------|
| **D-series control-only** | Live Option A plant stays; Hardware idle |
| **Option C** | This pack — READY-NOT-INSTALLED; install only on named ACK |
| **Park Door v1** | Live `gate_f_push` stays INSTALLED; no further Door Prefer FAIL until reopen |

**One-liner:** Option C full-panel contact fork READY-NOT-INSTALLED md5 `6a3d4a70…`; live `gate_f_push` `adb24309…` KEPT; soft-pass off; no spend; no Door LOCK.
