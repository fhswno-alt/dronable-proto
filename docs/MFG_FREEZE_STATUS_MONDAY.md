# Manufacturing freeze status — Monday morning

**When:** Wed 30 Sep 2026 ~01:13 Europe/London (BST)  
**Audience:** Dave · Founding Manufacturing Lead · room  
**Scope:** SIM-ONLY status sheet. **No Path A / thaw / greenlight.**
**Door v1 install note:** Option B companion **INSTALLED (sim-only)** as live pointer; Option A `gate_f_push` **ARCHIVE** (bytes unchanged); Option C **HELD**; no STEP/fab change; no spend.

---

## 1. Curriculum locks

| Gate | Status | One line |
|------|--------|----------|
| **D** | **HOLDS** | CSF50 / `BEST_SS_clean_walk` on locked M145 (`ainex_controls_m2_145.xml`) |
| **E** | **UNLOCKED TRUE** | Learned residual (RL04 / PPO) on **same M145 plant** — plant wins; foot planform stays |
| **F** | **LOCKED TRUE** | Lever look+approach on companion plant `ainex_controls_m2_145_gate_f.xml` — M145 contact untouched |
| **G** | **LOCKED TRUE** | G00–G03 on companion (reach + hand↔lever contact bit-2). Lock: `previews/ainex_walk/iterate/GATE_G_AI_LOCK.md`. **No** grasp-force / door-open claim. |
| **H** | **LOCKED TRUE** | H00–H03 on hinged companion (grasp hold / rotate ≥15°). Lock: `previews/ainex_walk/iterate/GATE_H_AI_LOCK.md`. **Not** full door-open / latch / UK handle height. Mfg: `docs/GATE_H_LEVER_HINGE_FAB_QC.md` (rest-pose **PASS**; hinged STEP **DEFERRED**). |
| **I** | **LOCKED TRUE** | I00–I03 on companion (lever + coupled panel ≥10°). Lock: `previews/ainex_walk/iterate/GATE_I_AI_LOCK.md`. **Not** full open / latch / walk-through. Mfg: `docs/GATE_I_PANEL_HINGE_FAB_QC.md` (lever rest-pose **PASS**; panel STEP **N/A**). |
| **J** | **LOCKED TRUE** | J00–J03 on soft-spring companion (≥25° panel + hold/close). Lock: `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md`. Plant-of-record: `docs/GATE_J_HARDWARE_PANEL_SPRING.md`. **Not** full open. **No STEP/fab change** (spring retune only). |
| **K** | **LOCKED TRUE** | K00–K03 multi-bout 3/3 XY-free + §9 + open-angle stripe. Lock: `previews/ainex_walk/iterate/GATE_K_AI_LOCK.md` · `previews/ainex_walk/iterate/GATE_L_AI_LOCK.md`. Companion md5 `59cc408eda07037a58f92ad27da045d6`. **Not** full open. **No STEP/fab change** (visual stripe only). |
| **L** | **LOCKED TRUE** | Leave/re-grasp L00–L03 2/2. Lock: `previews/ainex_walk/iterate/GATE_L_AI_LOCK.md`. Companion md5 `59cc408eda07037a58f92ad27da045d6`. **Not** full open / range bump. **No STEP/fab change.** |
| **M** | **LOCKED TRUE** | Open-hold disturb reject 2/2 (panel push 1.2 N × 0.35 s; soft-pass not used). Lock: `previews/ainex_walk/iterate/GATE_M_AI_LOCK.md` (root A; bout0 short disturb on stills). Criteria: `docs/GATE_M_AI_CRITERIA.md`. Companion md5 `59cc408eda07037a58f92ad27da045d6`. **Not** full open / latch / 90° / walk-through / range bump. **No plant/STEP/fab change.** |
| **N** | **LOCKED TRUE** | Leave+disturb compose 2/2 same bout (L+M; soft-pass not used; stills DISTURBANCE ON + leave both bouts). Lock: `previews/ainex_walk/iterate/GATE_N_AI_LOCK.md`. Criteria: `docs/GATE_N_AI_CRITERIA.md`. Companion md5 `59cc408eda07037a58f92ad27da045d6`. **Not** latch / 90° / walk-through / full open / range bump / walk→door. **No plant/STEP/fab change.** |
| **O** | **LOCKED TRUE** | Approach→N-compose 2/2 same run (soft-pass not used; stills APPROACH START + DISTURBANCE ON + LEAVE WINDOW both cycles). Lock: `previews/ainex_walk/iterate/GATE_O_AI_LOCK.md`. Criteria: `docs/GATE_O_AI_CRITERIA.md`. Companion md5 `59cc408eda07037a58f92ad27da045d6`. **Not** latch / 90° / walk-through / full open / range bump / room-walk / investor walk. **No plant/STEP/fab change.** |
| **P** | **LOCKED TRUE** | Stepped approach then N compose 2/2 (Controls iterate2; soft-pass not used; earlier Root B flush-soles / sticky-HUD FAIL resolved). Lock: `previews/ainex_walk/iterate/GATE_P_AI_LOCK.md` · `docs/GATE_Q_AI_CRITERIA.md`. Criteria: `docs/GATE_P_AI_CRITERIA.md`. Companion md5 `59cc408eda07037a58f92ad27da045d6`. **Not** latch / 90° / walk-through / room-walk / range bump. **No plant/STEP/fab change.** |
| **Q** | **OPEN (Door v1 live; Gate Q pick orthogonal)** | T5-E Prefer FAIL SCORED. Dave ACK ~01:13 BST 30 Sep: live plant = Option B `gate_f_optb` md5 `ddf084cd…`. Option A `gate_f_push` md5 `adb24309…` **ARCHIVE**. Option C **HELD**. Prefer FAIL Door SCORE next (multi-try; no one-shot FAIL ping). Gate Q next lever still Dave (H2 / DCM-VRP-DS / park) — separate. Soft-pass off; E7lock for walk. **No plant invent / no STEP/fab**. |

Vision **off** walk loop. Still **not** Pi / Orin. CoP HX marginal — not SS prove. **Door v1 Option B INSTALLED (sim-only)** (Dave ACK ~01:13 BST 30 Sep). Option A archive. Option C HELD. Lever-era F–P = history. MFG: foot freeze; lever fab parked; no spend.

---

## 2. What Manufacturing has frozen vs pre-positioned

| Item | State | Paths |
|------|-------|-------|
| **STEP foot pack** | **FROZEN** | Planform **145 × 86**; CAD stack **~4.5 mm** (3 + 1.5 plate+tread). XML **16 mm** = sim contact proxy only — **do not fab to 16 mm**. QC: `docs/M2_FAB_QC_FROZEN_145.md`. Sleeve dry-fit QC **PASS**: `docs/M2_ANK_ROLL_SLEEVE_FIT_QC.md` (~1 mm/side vs 137×78). Parts: `cad/m2_outsole/M2_outsole_145x86_*`, `M2_tread_145x86_*` (mesh-derived / not OEM). |
| **Lever demo prop** | **PARKED for v1** (Dave voice ~21:52 BST) | Was PRE-POSITIONED fixed-bar STEP QC **PASS**; hinged/panel STEP **DEFERRED**. **v1 door task = hospital push/pull only — no knob/lever torque.** Lever fab / UK £ research **not** March 2027 critical path. Paths kept on disk for later: `cad/gate_f_lever/…`, `docs/GATE_F_LEVER_FAB_QC.md`. **NOT TO ORDER / no PO.** |
| Score plant / companion | **INSTALLED (sim-only)** (Controls) | Walk M145 `fc94709c…`. Live companion: `…_m2_145_gate_f_optb.xml` md5 `ddf084cd…`. Option A `…_gate_f_push.xml` md5 `adb24309…` **ARCHIVE**. Lever-era `gate_f` `59cc408e…` **ARCHIVE**. |
| **H0 instrument fork** | **OPT-IN only** (not live train) | `…_m2_145_gate_f_h0.xml` md5 `ad9a1817f015e68e68f14535311369f0` — sites only, contype 0, contact physics identical. Doc: `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md`. **No STEP/fab.** |
| **H2 honesty draft** | **READY-NOT-INSTALLED** | `…_gate_f_h2.xml` md5 `b6e574d60ddb1c06cdbd7e8cd256cacc` (16→4.5 mm vertical); optional `…_h0_h2.xml` md5 `90d8929006ad25c4772a75aa7825a0a4`. Live freeze untouched. Doc: `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md`. **Dave unlock required** before any score swap / MFG quote. **No STEP/fab / no PO.** |
| **Push/pull door v1 (Option B)** | **INSTALLED (Dave ACK ~01:13 BST 30 Sep; sim-only)** | Live companion = `…_gate_f_optb.xml` md5 `ddf084cdac71cb0998aa6a44a65594c0` — bit-4 push face; hospital push/pull; lever visual; no latch. Option A `…_gate_f_push.xml` md5 `adb24309…` **ARCHIVE** (bytes unchanged). Option C **HELD**. Doc: `docs/GATE_DOOR_V1_HARDWARE_OPTION_B_DRAFT.md` · receipt `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`. Prefer FAIL Door SCORE = Controls multi-try (no one-shot FAIL report; no SCORE claim from this install). **INSTALLED sim-only** — still no STEP/fab for swap. No spend. |

---

## 3. What would unblock (conditions only)

| Ask | Unblock condition |
|-----|-------------------|
| **STEP foot refresh** | A **different** plant XML wins / freezes after Gate E — **not** this unlock. Plant already locked on M145 → **foot stays 145×86**. Refresh only if Controls freezes a new GEO plant **and** Dave gates a CAD turn. |
| **Lever fab** | **PARKED for v1** — Dave: no knob/lever torque for March 2027. Revisit only if task re-adds latch/knob later. |

---

## 4. Explicit for Monday

- **Waiting on Dave:** Gate Q next lever (H2 / DCM-VRP-DS / park) when he wants it — orthogonal to Door Prefer FAIL multi-try. Be present for Door Prefer FAIL / success ping (Dave preference).
- **No spend.** No cart, no sole fab, no lever order, no Path A / thaw / greenlight ask from this sheet.
- Manufacturing: foot 145×86 **FROZEN**; lever **PARKED for v1**; Door Option B plant **LIVE** (`gate_f_optb`); Option A **ARCHIVE**; Option C **HELD**. **No STEP/fab**; **no spend**. Synced ~01:13 BST Wed 30 Sep.


---

## 5. Controls Gate E checkpoint (pin)

| Item | Value |
|------|-------|
| Path | `previews/ainex_walk/iterate/learned_gate_e/ppo_gate_e_best.zip` |
| sha16 | `9ffaa1a21b607bf6` |

Controls asked to pin this on the freeze sheet. Foot planform freeze **unchanged** (145×86). Not a fab / PO signal.

**Refs:** `docs/TELEOP_AUTONOMY_CURRICULUM.md` · `docs/M2_FAB_QC_FROZEN_145.md` · `docs/GATE_F_LEVER_FAB_PREP.md` · `docs/GATE_F_LEVER_STEP_STUB.md` · `docs/GATE_F_LEVER_FAB_QC.md` · `docs/MONDAY_PERCEPTION_COMPUTE_CHECKLIST.md` · `docs/GATE_G_HARDWARE_LEVER_CONTACT.md` · `previews/ainex_walk/iterate/GATE_G_AI_LOCK.md` · `previews/ainex_walk/iterate/GATE_H_AI_LOCK.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md` · `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md` · `docs/GATE_K_HARDWARE_PANEL_VISUAL.md` · `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` · `previews/ainex_walk/iterate/GATE_K_AI_LOCK.md` · `previews/ainex_walk/iterate/GATE_L_AI_LOCK.md` · `docs/GATE_L_AI_CRITERIA.md` · `docs/GATE_M_AI_CRITERIA.md` · `previews/ainex_walk/iterate/GATE_M_AI_LOCK.md` · `docs/GATE_N_AI_CRITERIA.md` · `previews/ainex_walk/iterate/GATE_N_AI_LOCK.md` · `docs/GATE_O_AI_CRITERIA.md` · `previews/ainex_walk/iterate/GATE_O_AI_LOCK.md` · `previews/ainex_walk/iterate/GATE_P_AI_LOCK.md` · `docs/GATE_Q_AI_CRITERIA.md` · `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` · `docs/GATE_Q_AI_SCORE_R4.md` · `docs/GATE_Q_AI_COSPEC_R5.md` · `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` · `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md` · `docs/GATE_Q_HARDWARE_H2_DRAFT_PACK.md` · `docs/GATE_Q_AI_SCORE_T5D.md` · `docs/GATE_Q_HARDWARE_H3_AABB_RECONFIRM.md` · `docs/GATE_Q_AI_COSPEC_T5D.md` · `docs/GATE_Q_AI_COSPEC_T5D2.md` · `docs/GATE_Q_AI_SCORE_T5D2.md` · `docs/GATE_Q_AI_COSPEC_T5E.md` · `docs/GATE_Q_AI_SCORE_T5E.md` · `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` · `docs/GATE_DOOR_V1_CONTROLS_COSPEC.md`
