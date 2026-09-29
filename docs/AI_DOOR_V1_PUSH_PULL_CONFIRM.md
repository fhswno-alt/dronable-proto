# AI confirm — v1 door = hospital push/pull only

**Date:** 2026-09-29 ~21:53 BST  
**From:** Founding AI Scientist (response to Dave voice / Hardware room post)  
**Soft-pass:** off · **Spend:** none · **Live freeze:** untouched until Hardware push/pull honesty cospec + AI no-veto rewrite

## Confirm

March 2027 zero-to-one door = **detect closed → push or pull the panel open in front** (hospital-style).  
**Out of v1 critical path:** doorknob / lever **torque**, latch modeling, knob height, and hinge-param tuning as zero-to-one blockers (later adjustment vars).

AI owns criteria. Historical **F–P LOCKED TRUE (sim)** remain **lever-era sim history** — **not** a March 2027 hospital-door claim. Live companion / M145 md5s **unchanged**. No plant invent. Soft-pass never.

## Flag — AI criteria / curriculum that assumed lever/knob-class

| Layer | Assumed | Doc | v1 disposition |
|-------|---------|-----|----------------|
| **Gate F** | Look+approach **prop lever** @ 250–300 mm AFF; hands reach lever Z | `GATE_F_AI_CRITERIA.md` | **Re-scope** → panel face / push target in ego @ working stand (no lever Z band) |
| **Gate G** | Hand reach to `door_lever`; **G03** light hand–lever contact | `GATE_G_AI_CRITERIA.md` | **Re-scope** → hand/palm → **panel** light contact (no lever geom) |
| **Gate H** | Grasp hold + **lever rotate ≥15°** (`door_lever_hinge`) | `GATE_H_AI_CRITERIA.md` | **OUT of v1 path** (lever torque era). Historical only. Replace with panel push/pull force honesty later |
| **Gate I** | Panel swing **coupled through lever** (lever child of panel) | `GATE_I_AI_CRITERIA.md` | **Re-scope** → panel swing from **direct panel contact** (no lever coupling honesty) |
| **Gate J–K** | Larger / multi-bout swing; hand→lever→panel→`door_panel_hinge` | `GATE_{J,K}_AI_CRITERIA.md` | **Re-scope** coupling string; panel hinge + spring honesty can stay if Hardware keeps them |
| **Gate L–N** | Leave/**re-grasp on lever**; disturb reject with lever grasp | `GATE_{L,M,N}_AI_CRITERIA.md` | **Re-scope** → leave/re-contact **panel**; re-grasp→re-push/pull |
| **Gate O–P** | Approach then **N compose** (lever grasp path) | `GATE_{O,P}_AI_CRITERIA.md` | **Re-scope** compose to push/pull open; stepped approach honesty can keep |
| **Gate Q** | Stepped approach→**N compose (lever)**→stepped retreat | `GATE_Q_AI_CRITERIA.md` | Walk/retreat Prefer FAIL bars = **locomotion** (keep). **Compose prereq** was lever-era → rewrite to push/pull compose when Hardware pack lands |
| Curriculum sheet | F–H lever / G03 / H hinge | `TELEOP_AUTONOMY_CURRICULUM.md` (if present) + lock notes | Label F–P locks **lever-era history**; new v1 door row = push/pull |
| Perception / FOV | Lever-centered look-down @ 275 mm AFF | `FOV_REBIND_*`, Gate F FOV rows | Retarget optical axis to **panel push patch** (height TBD with Hardware) — not knob height product |
| System brief | Standard hands for lever grasp; 250–300 mm AFF | `HARDWARE_SYSTEM_DESIGN_BRIEF.md` | Later edit with Hardware — not blocking freeze |

**Already never March unlock (unchanged hard non-claims):** latch release, 90° full open, walk-through, UK handle height product, investor room-walk script, spend/PO.

**Not found as named MJCF DOF:** separate rotary doorknob / latch-bolt joint. Dave “knob torque” maps to our **lever grasp/rotate (H+)** stack — same OUT.

## What can stay (pending Hardware honesty)

- Panel geom + `door_panel_hinge` + soft spring (if Hardware’s push/pull pack keeps them) — panel must still swing.
- Stepped locomotion honesty (Gate E lineage, P/Q approach/retreat bars, skate/clear_frac) — orthogonal to mechanism.
- Companion / walk plant **LIVE FREEZE** until named cospec install.
- Gate Q Prefer FAIL next lever (**H2 / DCM-VRP-DS / park**) — **still Dave’s pick**; locomotion wall, orthogonal until compose re-scope.

## AI next (no idle invent)

1. **Shipped this confirm** — flag list above.  
2. **Await Hardware** READY-NOT-INSTALLED push/pull companion fork + inventory.  
3. Same-turn: draft `docs/GATE_DOOR_V1_AI_CRITERIA.md` (or amend F→Q) = detect closed → panel push/pull open; formal **no-veto** only after Hardware pack + Controls kit-facing proposal.  
4. Do **not** auto-relabel F–P locks as March door PASS. Do **not** soften Gate Q Prefer FAIL bars. Do **not** unlock H2 without Dave.

## Owner split

| Owner | Next |
|-------|------|
| Hardware | Push/pull panel-contact honesty pack (in flight) |
| AI | This confirm · criteria rewrite after pack |
| Controls | No new door score until AI no-veto; Gate Q loco Prefer FAIL on Dave pick only |
| MFG | Lever fab PARKED (done); panel prop quote only if cospec freezes one |

---

## Update (~21:56 BST) — AI NO-VETO landed

Formal no-veto: `docs/GATE_DOOR_V1_AI_COSPEC.md`  
Criteria: `docs/GATE_DOOR_V1_AI_CRITERIA.md`  
Controls §5: Option A · closed ε=2° · F–P historical · open=`door_panel_hinge` · soft-pass off.  
**NO INSTALL** until Dave ACK of named swap to `…_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040`.
