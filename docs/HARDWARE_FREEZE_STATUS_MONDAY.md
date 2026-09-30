# Hardware freeze status — Monday afternoon sync

**When:** Wed 30 Sep 2026 ~01:13 Europe/London (BST) — Dave ACK: Door v1 **Option B** **INSTALLED** as live companion; Option A `gate_f_push` **ARCHIVE** (bytes unchanged); Option C **HELD**; walk M145 KEPT; H2 still READY-NOT-INSTALLED; soft-pass off; no spend
**Audience:** Dave · Founding Hardware · room  
**Scope:** SIM-ONLY status sheet. **No Path A / thaw / greenlight / spend.**

Verified on disk this turn:
- Walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` → md5 `fc94709c84f5598d4474ecfc4bb41fdc` (**KEPT**)
- **Live companion (Door v1, Option B)** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml` → md5 `ddf084cdac71cb0998aa6a44a65594c0` (**INSTALLED**)
- Option A archive `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` → md5 `adb24309b489d56615c194e92676d040` (**ARCHIVE** — not overwritten, not live)
- Option C `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` → md5 `6a3d4a70d4797b806dcc2580f46468aa` (**READY-NOT-INSTALLED / HELD**)
- Lever-era archive `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` → md5 `59cc408eda07037a58f92ad27da045d6` (**ARCHIVE** / F–P history only — not overwritten)

Install receipt: `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`

---

## 1. Curriculum (Hardware view)

| Gate | Status | Hardware note |
|------|--------|---------------|
| **D–E** | **HOLDS / TRUE** | Locked M145 walk plant — **do not edit** |
| **F–P** | **LOCKED TRUE (lever-era history)** | Historical locks on archive companion `…_gate_f.xml` md5 `59cc408e…` — F–P rows only; **not** March door claim |
| **Q** | **Prefer FAIL / T5-E SCORED** | T5-E Prefer FAIL (`docs/GATE_Q_AI_SCORE_T5E.md`): SEQ on; skate under; maxG 0.031; ret_ok_both=false; ε Prefer FAIL. Soft-pass **off**. E7lock kept. **H2** READY-NOT-INSTALLED. **Door v1 Option B INSTALLED** (Dave ACK ~01:13 BST 30 Sep). Gate Q H2/DCM-VRP-DS/park still **separate** Dave pick. |
| **Door v1** | **INSTALLED (sim plant, Option B)** | Live companion = `…_gate_f_optb.xml` md5 `ddf084cd…`. Option A `…_gate_f_push.xml` md5 `adb24309…` = **ARCHIVE**. Option C **HELD**. No Hardware Prefer FAIL Door SCORE claim. Controls multi-try scoring pref: no FAIL report on first miss; multiple attempts; report FAIL only after repeated setbacks; report success; boss present for either outcome. |

Curriculum **D–P TRUE** (sim; F–P = lever-era). Soft-pass **off**. No plant unfreeze without Dave unlock. No spend.

---

## 2. Frozen plants (Hardware owns)

| Item | State | Detail |
|------|-------|--------|
| **Walk / Gate E plant** | **FROZEN / KEPT** | `ainex_controls_m2_145.xml` · md5 `fc94709c84f5598d4474ecfc4bb41fdc` · planform **145×86** · contact/HX untouched |
| **Live companion (Door v1)** | **INSTALLED** | `ainex_controls_m2_145_gate_f_optb.xml` · md5 `ddf084cdac71cb0998aa6a44a65594c0` · Option B push face **bit 4** · hands bit 4 · lever contype 0 (visual) · panel visual · hinge ±30° · spring damp **0.05** / stiff **0.015** · Gate K stripe visual · no latch · Dave ACK ~01:13 BST 30 Sep 2026 |
| **Option A companion archive** | **ARCHIVE** | `ainex_controls_m2_145_gate_f_push.xml` · md5 `adb24309b489d56615c194e92676d040` · prior live (Dave ACK ~22:08 BST 29 Sep) · **bytes unchanged** · **not** live · **do not overwrite** |
| **Option C companion** | **READY-NOT-INSTALLED / HELD** | `ainex_controls_m2_145_gate_f_optc.xml` · md5 `6a3d4a70d4797b806dcc2580f46468aa` · full-panel contact fork · **not installed** |
| **Lever-era companion archive** | **ARCHIVE** | `ainex_controls_m2_145_gate_f.xml` · md5 `59cc408eda07037a58f92ad27da045d6` · F–P history only · **do not overwrite** |
| **Foot STEP / CAD** | **FROZEN** | 145×86 · ~4.5 mm CAD stack — MFG QC owns fab sheet; Hardware not changing dims |
| **M145 contact honesty** | **FROZEN** | No GEO reopen without Dave + AI cospec |
| **H0 instrument fork** | **READY (not freeze)** | `ainex_controls_m2_145_gate_f_h0.xml` · md5 `ad9a1817f015e68e68f14535311369f0` · sites + contype=0 tell only · physics-identical contact · **not** score plant-of-record |
| **H2 CAD-height A/B** | **READY-NOT-INSTALLED** | `…_gate_f_h2.xml` md5 `b6e574d60ddb1c06cdbd7e8cd256cacc` (+ optional `…_gate_f_h0_h2.xml` `90d89290…`) · vertical 16→~4.5 mm only · SOLE_OFFSET 0.026 · needs Dave unlock + AI Prefer FAIL A/B · live walk/companion freeze KEPT · see `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md` |

---

## 3. What Hardware shipped this session (all frozen / installed as noted)

| Artifact | Path | Note |
|----------|------|------|
| Panel foreshortening dig | `docs/GATE_K_HARDWARE_PANEL_VISUAL.md` | Face-on thin slab looks closed; free-edge tell needed |
| Free-edge edge-stripe | companion + `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` | Visual-only green lip on +Y — enabled K lock |
| Panel soft spring | `docs/GATE_J_HARDWARE_PANEL_SPRING.md` | ≥25° under ~2 N lever |
| Panel hinge / lever contact | Gate I/G docs | Companion-only; M145 untouched |
| Force-at-a-distance dig (L) | room note | Passive panel returns toward 0; climb = residual contact, not plant ghost |
| **H0 instrument fork (T5-D1)** | `…_gate_f_h0.xml` + `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md` | Sole daylight / toe / heel / floor sites + contype=0 tell; archive gate_f.md5 **KEPT** |
| **H3 AABB reconfirm** | `docs/GATE_Q_HARDWARE_H3_AABB_RECONFIRM.md` | Mesh ~135×76 vs contact 145×86×16 — dig only, no XML edit |
| **H2 CAD-height draft** | `…_gate_f_h2.xml` + `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md` | READY-NOT-INSTALLED companion fork; optional H0+H2 merge also on disk |
| **Door v1 Option B INSTALLED** | `docs/GATE_DOOR_V1_HARDWARE_OPTION_B_DRAFT.md` + `…_gate_f_optb.xml` + `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md` | Dave ACK ~01:13 BST 30 Sep; live companion = optb md5 `ddf084cd…`; Option A `gate_f_push` archived (bytes unchanged); Option C HELD; lever-era archived |

---

## 4. Explicit non-claims / deferred

- **Not** latch / 90° / walk-through / UK handle / room-walk / Pi / Orin
- **Push/pull v1:** door task = hospital-door panel push/pull only — **no** knob torque / lever-rotate score / latch modeling; knob height & hinge params = later adjustment vars. Live plant since ~01:13 BST 30 Sep = Option B `gate_f_optb` (Option A archive)
- **Alt A** (panel range bump) **deferred** unless AI asks
- **No planted soft-XY** for Gate Q (AI criteria KEPT; Prefer FAIL structural does not reopen plant)
- **No Prefer FAIL Door SCORE** claim from Hardware (Controls owns multi-try Prefer FAIL Door scoring)
- **No spend / no PO / no Path A**

---

## 5. What would unblock Hardware work

| Ask | Unblock |
|-----|---------|
| Plant / geom edit (H2) | Dave freeze unlock + AI Prefer FAIL cospec — **draft pack ready** (`GATE_Q_HARDWARE_H2_DRAFT_PACK.md`); companion-first; do not install without Dave |
| Dim freeze for MFG | Hardware freezes new dims → MFG QC — **not active** (push plant sim-only; no STEP/fab for swap) |
| Dave review | Only if we hit a hard plant honesty conflict Controls cannot close controller-only |

**Right now:** Door v1 Option B **INSTALLED** (Dave ACK ~01:13 BST 30 Sep 2026). Live companion = `…_gate_f_optb.xml` md5 `ddf084cd…`. Option A `…_gate_f_push.xml` md5 `adb24309…` = **ARCHIVE** (bytes unchanged). Option C **HELD**. Lever-era `…_gate_f.xml` = archive. Walk M145 **KEPT**. H2 still READY-NOT-INSTALLED. Gate Q H2/DCM-VRP-DS/park = separate Dave pick. Soft-pass off. No spend. No Door SCORE claim from Hardware. Controls: multi-try Prefer FAIL Door (no FAIL on first miss; boss present for FAIL or success).

**Refs:** `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md` · `docs/GATE_DOOR_V1_HARDWARE_OPTION_B_DRAFT.md` · `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` · `docs/MFG_FREEZE_STATUS_MONDAY.md` · `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` · `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` · `docs/GATE_Q_HARDWARE_T5D_PLANT_LEVERS.md` · `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md` · `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md` · `docs/GATE_Q_HARDWARE_H3_AABB_RECONFIRM.md` · `docs/GATE_Q_AI_CRITERIA.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md` · `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` · `docs/GATE_K_HARDWARE_PANEL_VISUAL.md` · `docs/TELEOP_AUTONOMY_CURRICULUM.md`
