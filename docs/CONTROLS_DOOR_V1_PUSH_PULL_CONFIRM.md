# Controls confirm — v1 door = hospital push/pull only

**Date:** 2026-09-29 ~21:52 BST  
**From:** Controls (response to Hardware / Dave voice)  
**Soft-pass:** off · **Spend:** none · **Live freeze:** untouched until Hardware push/pull honesty cospec

## Confirm

March 2027 zero-to-one door task = **detect closed → push/pull open in front** (hospital-style panel).  
**Out of v1 critical path:** doorknob torque, latch modeling, lever-torque ops, knob height / hinge param as zero-to-one vars (later).

Controls will **not** score new door work against hand→lever→panel coupling until AI rewrites criteria + Hardware ships a push/pull panel-contact companion pack (cospec first). Gate Q walk/retreat Prefer FAIL path stays locomotion-only until that re-scope lands; E7lock frozen; soft-pass off.

## What already assumed hand→lever / knob-class (flag)

| Layer | Assumption | Paths / notes |
|-------|------------|---------------|
| Curriculum F–H | Lever approach / reach+contact / grasp+rotate | `TELEOP_AUTONOMY_CURRICULUM.md`; locks G03 hand–lever, H grasp+`door_lever_hinge` |
| Curriculum I–P | Coupling honesty **hand→lever→panel→`door_panel_hinge`**; leave/re-grasp on lever | `GATE_{I,J,K,L,M,N,O,P}_AI_CRITERIA.md` — LOCKED TRUE (sim history; may need re-scope label) |
| Gate Q compose inherit | Q = stepped approach→**N compose**→retreat; N compose is lever grasp path | `GATE_Q_AI_CRITERIA.md` — walk bars themselves are locomotion; compose prereq is lever |
| Companion plant | `door_lever` contactable bit-2; `door_lever_hinge`; panel driven via lever child | `ainex_controls_m2_145_gate_f*.xml`; `GATE_G_HARDWARE_LEVER_CONTACT.md`; `GATE_H_HARDWARE_LEVER_HINGE.md`; `GATE_I_HARDWARE_PANEL_HINGE.md` |
| Fab / MFG | Lever demo prop STEP QC; hinged lever/panel fab docs | `cad/gate_f_lever/`; `GATE_F_LEVER_FAB_QC.md`; `GATE_H_LEVER_HINGE_FAB_QC.md` — MFG already PARKED for v1 |
| System brief | Standard hands for lever grasp; teleop lever 250–300 mm AFF | `HARDWARE_SYSTEM_DESIGN_BRIEF.md` |

**Already out of scope (curriculum hard falsifiers):** latch claim, 90°, walk-through, UK handle height, full open — those were never March unlock; Dave’s directive additionally drops **lever/knob torque ops** as the v1 mechanism.

**Not assumed:** separate rotary doorknob DOF / latch bolt torque in MJCF (none found as a named joint). “Knob torque” in Dave’s ask maps to our **lever grasp/rotate (Gate H+)** stack.

## Controls disposition

1. Confirm curriculum+sim **must re-scope** to push/pull panel contact for v1 door; historical F–P locks stay as **lever-era sim history**, not March door claim.  
2. Live companion md5 `59cc408eda07037a58f92ad27da045d6` **unchanged** until Hardware push/pull honesty pack + AI cospec.  
3. No plant invent / no soft-pass / no PO.  
4. Gate Q Prefer FAIL next lever (H2 / DCM-VRP-DS / park) still awaits **Dave named pick** — orthogonal to door mechanism until compose re-scope.  
5. Waiting Hardware: propose push/pull panel-contact honesty pack (Controls will score only after AI no-veto cospec).

## Owner split

| Owner | Next |
|-------|------|
| Hardware | Inventory + propose push/pull panel-contact honesty pack (freeze holds) |
| AI | Curriculum / criteria rewrite: v1 = push/pull; lever gates → historical or later |
| Controls | No new door score until cospec; Gate Q locomotion Prefer FAIL only on Dave pick |
| MFG | Lever fab PARKED (already); quote panel prop only if/when cospec freezes one |
