# Hardware freeze status — Monday afternoon sync

**When:** Tue 29 Sep 2026 ~21:52 Europe/London (BST) — Dave push/pull v1 directive (drop knob ops); draft fork READY-NOT-INSTALLED; live freeze KEPT; T5-E Prefer FAIL / H2 still pending Dave pick
**Audience:** Dave · Founding Hardware · room  
**Scope:** SIM-ONLY status sheet. **No Path A / thaw / greenlight / spend / PO.**

Verified live on disk this turn:
- Walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` → md5 `fc94709c84f5598d4474ecfc4bb41fdc`
- Companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` → md5 `59cc408eda07037a58f92ad27da045d6`

---

## 1. Curriculum (Hardware view)

| Gate | Status | Hardware note |
|------|--------|---------------|
| **D–E** | **HOLDS / TRUE** | Locked M145 walk plant — **do not edit** |
| **F–P** | **LOCKED TRUE** | Companion plant-of-record frozen at md5 `59cc408e…` — F–P rows only |
| **Q** | **Prefer FAIL / T5-E SCORED** | T5-E Prefer FAIL (`docs/GATE_Q_AI_SCORE_T5E.md`): SEQ on; skate under; maxG 0.031; ret_ok_both=false; ε Prefer FAIL. Soft-pass **off**. E7lock kept. **H2** READY-NOT-INSTALLED. **Push/pull v1** directive + fork READY-NOT-INSTALLED. Live freeze KEPT. Waiting Dave: H2 / DCM-VRP-DS / park / push-pull ACK. |

Curriculum **D–P TRUE** (sim). **Gate Q Prefer FAIL** — T5-E closed Prefer FAIL. Soft-pass **off**. No plant unfreeze without Dave unlock. No spend.

---

## 2. Frozen plants (Hardware owns)

| Item | State | Detail |
|------|-------|--------|
| **Walk / Gate E plant** | **FROZEN** | `ainex_controls_m2_145.xml` · md5 `fc94709c84f5598d4474ecfc4bb41fdc` · planform **145×86** · contact/HX untouched |
| **Companion F–Q** | **FROZEN** | `ainex_controls_m2_145_gate_f.xml` · md5 `59cc408eda07037a58f92ad27da045d6` · panel hinge ±30° · spring damp **0.05** / stiff **0.015** · lever child-of-panel · Gate K free-edge stripe (visual-only) · ticks/beads **deferred** |
| **Foot STEP / CAD** | **FROZEN** | 145×86 · ~4.5 mm CAD stack — MFG QC owns fab sheet; Hardware not changing dims |
| **M145 contact honesty** | **FROZEN** | No GEO reopen without Dave + AI cospec |
| **H0 instrument fork** | **READY (not freeze)** | `ainex_controls_m2_145_gate_f_h0.xml` · md5 `ad9a1817f015e68e68f14535311369f0` · sites + contype=0 tell only · physics-identical contact · **not** score plant-of-record |
| **H2 CAD-height A/B** | **READY-NOT-INSTALLED** | `…_gate_f_h2.xml` md5 `b6e574d60ddb1c06cdbd7e8cd256cacc` (+ optional `…_gate_f_h0_h2.xml` `90d89290…`) · vertical 16→~4.5 mm only · SOLE_OFFSET 0.026 · needs Dave unlock + AI Prefer FAIL A/B · **not** mid-T5-D1 · live freeze KEPT · see `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md` |
| **Push/pull v1 door fork** | **READY-NOT-INSTALLED** | `…_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040` · lever contype 0 (visual) · `door_panel_push_face` bit 2 · panel hinge ±30° kept · no latch · needs AI/Controls cospec + Dave ACK · live freeze KEPT · see `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` |

---

## 3. What Hardware shipped this session (all frozen now)

| Artifact | Path | Note |
|----------|------|------|
| Panel foreshortening dig | `docs/GATE_K_HARDWARE_PANEL_VISUAL.md` | Face-on thin slab looks closed; free-edge tell needed |
| Free-edge edge-stripe | companion + `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` | Visual-only green lip on +Y — enabled K lock |
| Panel soft spring | `docs/GATE_J_HARDWARE_PANEL_SPRING.md` | ≥25° under ~2 N lever |
| Panel hinge / lever contact | Gate I/G docs | Companion-only; M145 untouched |
| Force-at-a-distance dig (L) | room note | Passive panel returns toward 0; climb = residual contact, not plant ghost |
| **H0 instrument fork (T5-D1)** | `…_gate_f_h0.xml` + `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md` | Sole daylight / toe / heel / floor sites + contype=0 tell; live gate_f.md5 **KEPT** |
| **H3 AABB reconfirm** | `docs/GATE_Q_HARDWARE_H3_AABB_RECONFIRM.md` | Mesh ~135×76 vs contact 145×86×16 — dig only, no XML edit |
| **H2 CAD-height draft** | `…_gate_f_h2.xml` + `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md` | READY-NOT-INSTALLED companion fork; live gate_f.md5 **KEPT**; optional H0+H2 merge also on disk |
| **Push/pull v1 directive + fork** | `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` + `…_gate_f_push.xml` | Dave drop-knob directive; inventory; Option A push-face draft; live md5s **KEPT** |

---

## 4. Explicit non-claims / deferred

- **Not** latch / 90° / walk-through / UK handle / room-walk / Pi / Orin
- **Push/pull v1 (Dave 21:52 BST):** door task = hospital-door panel push/pull only — **no** knob torque / lever-rotate score / latch modeling; knob height & hinge params = later adjustment vars
- **Alt A** (panel range bump) **deferred** unless AI asks
- **No planted soft-XY** for Gate Q (AI criteria KEPT; Prefer FAIL structural does not reopen plant)
- **No spend / no PO / no Path A**

---

## 5. What would unblock Hardware work

| Ask | Unblock |
|-----|---------|
| Plant / geom edit (H2) | Dave freeze unlock + AI Prefer FAIL cospec — **draft pack ready** (`GATE_Q_HARDWARE_H2_DRAFT_PACK.md`); companion-first; do not install mid-T5-D1 |
| **Push/pull plant swap** | AI/Controls cospec on Option A/B/C + curriculum retarget + **Dave ACK** — draft at `GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md`; live freeze KEPT |
| Dim freeze for MFG | Hardware freezes new dims → MFG QC — **not active** |
| Dave review | Only if we hit a hard plant honesty conflict Controls cannot close controller-only |

**Right now:** Dave **push/pull v1** directive live (drop doorknob ops). Draft companion fork `…_gate_f_push.xml` **READY-NOT-INSTALLED** — needs AI/Controls cospec + Dave ACK before swap. Live freeze md5s **KEPT** (re-verified ~21:52 BST). T5-E Prefer FAIL / H2 still awaiting Dave pick (H2 / DCM-VRP-DS / park). Soft-pass off. No spend.

**Refs:** `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` · `docs/MFG_FREEZE_STATUS_MONDAY.md` · `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` · `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` · `docs/GATE_Q_HARDWARE_T5D_PLANT_LEVERS.md` · `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md` · `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md` · `docs/GATE_Q_HARDWARE_H3_AABB_RECONFIRM.md` · `docs/GATE_Q_AI_CRITERIA.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md` · `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` · `docs/GATE_K_HARDWARE_PANEL_VISUAL.md` · `docs/TELEOP_AUTONOMY_CURRICULUM.md`
