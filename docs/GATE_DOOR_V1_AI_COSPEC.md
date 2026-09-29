# Gate Door v1 — AI cospec (NO-VETO)

**When:** Tue 29 Sep 2026 ~21:56 Europe/London (BST)  
**From:** Founding AI Scientist · **To:** Controls · Hardware · Dave  
**Refs:** `docs/GATE_DOOR_V1_CONTROLS_COSPEC.md` · `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` · `docs/AI_DOOR_V1_PUSH_PULL_CONFIRM.md`  
**Fork:** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040` (verified)  
**Status:** **AI NO-VETO** · Soft-pass **OFF** · Spend **none** · **NO INSTALL** until Dave ACK of named plant swap

Live freeze **KEPT** until Dave ACK:
- Walk / M145 `fc94709c84f5598d4474ecfc4bb41fdc`
- Companion F–Q `59cc408eda07037a58f92ad27da045d6`

---

## Formal no-veto (Controls §5)

| Ask | AI disposition |
|-----|----------------|
| 1. Door-task v1 = panel push/pull only; drop G03/H02/I–J lever-coupled as required | **NO-VETO CONFIRM** |
| Open measure = `door_panel_hinge` (±30° class) | **NO-VETO CONFIRM** |
| 2. Closed detect ordered: `|hinge|≈0` primary · optional hand×push_face · optional push_site distance | **NO-VETO ACCEPT** — closed band **ε = 2°** (`|door_panel_hinge| ≤ 0.035 rad`) |
| 3. Contact **Option A** (bit-2 push face; lever contype 0) | **NO-VETO ACK** — Option C only after Prefer FAIL miss on thin −X face; Option B parked |
| 4. Lever hinge keep inert visual DOF | **NO-VETO KEEP** |
| 5. Cam keep lever-height look-down first Prefer FAIL | **NO-VETO KEEP** — panel-center rebind only as separate post-SCORE honesty item |
| 6. F–P LOCKED TRUE = lever-era history on live `gate_f` | **NO-VETO CONFIRM** — do not auto-relabel as March door PASS |
| 7. March 2027 bar = walk + panel push/pull open (±30° class); knob/latch deferred; soft-pass off | **NO-VETO CONFIRM** |

**No Prefer FAIL amend** on Controls §5. Criteria path: **`docs/GATE_DOOR_V1_AI_CRITERIA.md`** (new row family — not silent amend of F–P locks).

---

## Install gate (unchanged)

1. Controls §5 cospec ✓  
2. **This AI no-veto** ✓  
3. **Dave ACK** named swap → live companion = `…_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040`  
4. Until then: READY-NOT-INSTALLED · live freeze KEPT

---

## Orthogonal

Gate Q locomotion Prefer FAIL bars **KEPT**. Next lever still Dave: **H2** A/B · **DCM-VRP-DS** · **park**. H2 **not** auto-unlocked by this door pack. Soft-pass off · E7lock frozen.
