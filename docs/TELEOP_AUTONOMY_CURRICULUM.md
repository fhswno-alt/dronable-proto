# Teleop → autonomy curriculum (AiNex Path A)

**Status:** Gate D–P **LOCKED TRUE** (sim); Gate Q criteria **live**; ckpt frozen; **not** Pi/Orin; sim only.  
**Owner:** AI (criteria) + Controls (prove)  
**No NN-first on Pi / no Orin.**  
**Dave ping rule:** Gate D/E/F sim locks met — further pings only for hard falsifier, next gate unlock, or assembly greenlight ask.

## Dave bar

> **Verified single-support bipedal steps** on M2 145×86: one foot fully off the ground (clear swing), plant, alternate L/R, assist OFF.

## Locked results

| Gate | Result | Evidence |
|------|--------|----------|
| C | TRUE (lab only) | E80 QS plant-and-shift — below Dave SS bar |
| **D** | **TRUE** | **CSF50 / `BEST_SS_clean_walk`** on M2 145×86 |
| E | **TRUE** (sim learned) | RL04_locked_T75 — `LEARNED_GATE_E_NOTE`; open-loop family still FALSE |
| **F** | **TRUE** (sim) | F00–F02 — `GATE_F_AI_LOCK.md`; companion `gate_f.xml` |
| **G** | **TRUE** (sim) | G00–G03 — `GATE_G_AI_LOCK.md`; reach 0.010 m; contact 3.94 s |
| **H** | **TRUE** (sim) | H00–H03 — `GATE_H_AI_LOCK.md`; grasp 6.76 s; rotate 31.7° |
| **I** | **TRUE** (sim) | I00–I03 — `GATE_I_AI_LOCK.md`; panel 12.3° coupled; not full open |
| **J** | **TRUE** (sim) | J00–J03 — `GATE_J_AI_LOCK.md`; panel 30.0°; hold 4.40 s; not full open |
| **K** | **TRUE** (sim) | K00–K03 — `GATE_K_AI_LOCK.md`; multi-bout ±30°; not full open |
| **L** | **TRUE** (sim) | L00–L03 — `GATE_L_AI_LOCK.md`; leave/re-grasp 2/2; reconcile A |
| **M** | **TRUE** (sim) | M00–M03 — `GATE_M_AI_LOCK.md`; disturb reject 2/2; reconcile A |
| **N** | **TRUE** (sim) | N00–N03 — `GATE_N_AI_LOCK.md`; leave+disturb compose 2/2 |
| **O** | **TRUE** (sim) | O00–O03 — `GATE_O_AI_LOCK.md`; approach→N compose 2/2 |
| **P** | **TRUE** (sim) | P00–P03 — `GATE_P_AI_LOCK.md`; iterate2 stepped→N compose 2/2; Root B resolved |
| Q | criteria live **KEPT** (OPEN — Prefer FAIL **structural PARK**) | R1–R4 Prefer FAIL same plant≫clear wall; R5 vetoed; E7lock best-known; soft-pass off; `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md`; reopen only new residual family; no plant change |
| Dynamic CPG (C11/D94) | **FAIL** | Skate falsifier reconfirmed (DYN00–22) |

### Gate D metrics (CSF50, assist OFF)

| Metric | Value | Gate |
|--------|-------|------|
| tip_free | 9.0 s | ≥8 |
| dx | +0.256 m | ≥0.05 |
| SS mean bout L/R | 0.124 / 0.120 s | ≥0.12 |
| Sole clear L/R | ~0.017 m | ≥0.012 |
| Stance \|vx\| mean | 0.050 / 0.058 | ≤0.08 |
| Stance \|vx\| p95 | 0.161 / 0.177 | ≤0.18 |
| hip_corr / lead | −0.773 / 7 | alt OK |
| Video | Controls continuous 3/3 | |

Artifacts: `previews/ainex_walk/iterate/BEST_SS_clean_walk.{mp4,json}`  
Note: Hardware SS71 had `ss_verified` with skate p95 fail; CSF50 broke skate with mild retune (not residual/NN). Knife-edge basin — do not soft-widen Gate E.

## Gates (order)

| Gate | Name | Pass criteria |
|------|------|---------------|
| A | Quiet stand | ≥30 s upright, assist OFF |
| B | Disturb reject | Recover ≥5 s CP/ZMP |
| C | QS plant-and-shift (E80) | Lab milestone only |
| **D** | **Single-support step** | **LOCKED — CSF50** |
| E | Multi SS / cadence | AI criteria `GATE_E_AI_CRITERIA.md`: T≤0.75 + clean_ss + skate gates — **FALSE hard falsifier** |
| **F** | **Lever approach** | **LOCKED TRUE** — `GATE_F_AI_LOCK.md`; head_tilt −16° @ 0.4 m; vision off walk |
| **G** | **Lever reach / light contact** | **LOCKED TRUE** — `GATE_G_AI_LOCK.md`; ≤5 cm reach; light contact; no door-open |
| **H** | **Lever grasp / rotate** | **LOCKED TRUE** — `GATE_H_AI_LOCK.md`; grasp 6.76 s; rotate 31.7°; no full door-open |
| **I** | **Limited door-panel swing** | **LOCKED TRUE** — `GATE_I_AI_LOCK.md`; panel 12.3° coupled; **not** full open |
| **J** | **Controlled larger panel swing** | **LOCKED TRUE** — `GATE_J_AI_LOCK.md`; panel 30.0°; hold 4.40 s; **not** full open |
| **K** | **Multi-bout open/close reliability** | **LOCKED TRUE** — `GATE_K_AI_LOCK.md`; **not** full open |
| **L** | **Leave / re-grasp** | **LOCKED TRUE** — `GATE_L_AI_LOCK.md`; **not** range bump / latch / 90° |
| **M** | **Open-hold disturb reject** | **LOCKED TRUE** — `GATE_M_AI_LOCK.md`; **not** range bump / latch / 90° |
| **N** | **Leave + disturb compose** | **LOCKED TRUE** — `GATE_N_AI_LOCK.md`; **not** range bump / latch / 90° / walk→door |
| **O** | **Approach then N compose** | **LOCKED TRUE** — `GATE_O_AI_LOCK.md`; **not** range bump / latch / 90° / walk-through |
| **P** | **Stepped approach then N compose** | **LOCKED TRUE** — `GATE_P_AI_LOCK.md`; **not** range bump / latch / 90° / room-walk |
| Q | Stepped approach→compose→stepped retreat | **criteria live** Prefer FAIL structural (~14:22) — `GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md`; bars KEPT; **not** range bump / latch / 90° / walk-through / room-walk |

## Sensing
- CoP HX = MARGINAL — not used for SS prove.
- M2 fab freeze **145×86**. M3 μ = demo-surface only.
- Path A cart frozen until Dave assembly greenlight.

## Gate E (solidified — PASS via learned residual; open-loop FAIL)

Criteria: `docs/GATE_E_AI_CRITERIA.md` (AI). Prove note: `previews/ainex_walk/iterate/GATE_E_CONTROLS_NOTE.md`.

| # | Criterion | Threshold | Controls result |
|---|-----------|-----------|-----------------|
| 1 | Inherit Gate D | `clean_walk_ss` | Cannot hold at T≤0.75 |
| 2 | Multi SS | ≥5 bouts/side | OK when upright |
| 3 | Cadence | T ≤ **0.75 s** | CSF50 T=0.88; speed → skate |
| 4 | Bout | ≥0.10 s (Gate D ≥0.12) | Collapses with T↓ |
| 5 | Skate | mean≤0.08, p95≤0.18 | FAIL all T≤0.75 |
| 6 | Progress | tip≥8, dx≥0.15, alt | Mixed; not jointly with 3+5 |
| 7 | Video | PASS for claim | No pass — fail exemplars only |

**Hard falsifier:** open-loop CPG + plant/CoP/VIK/ZMP/foot-IK/fric A/B on M145 cannot jointly hit kit cadence (T≤0.75) and no-skate SS. Dynamic CPG skate-kill likewise FALSE. Plant planform stays 145×86; 16 mm box height not the blocking failure mode.

## Gate K (~03:24 BST)
**LOCKED TRUE** (sim multi-bout open/close reliability). Curriculum **D–K TRUE** (superseded by D–L). Not full door-open.

## Gate L (~03:36 BST)
**LOCKED TRUE** (sim leave/re-grasp 2/2). Watch conflict cleared as root **A**. Curriculum **D–L TRUE**. Not range bump / latch / 90° / walk-through.

## Gate M (~03:49 BST)
**LOCKED TRUE** (sim open-hold disturb reject 2/2). Prior HOLD cleared as root **A** (bout0 scrub miss; HUD honest). Curriculum **D–M TRUE**. Not range bump / latch / 90° / walk-through.

## Gate N (~04:00 BST)
**LOCKED TRUE** (sim leave+disturb compose 2/2). Soft-pass not used. Curriculum **D–N TRUE**. Not range bump / latch / 90° / walk-through / walk→door.

## Gate O (~04:10 BST)
**LOCKED TRUE** (sim approach→N compose 2/2). Soft-pass not used. Curriculum **D–O TRUE**. Not range bump / latch / 90° / walk-through / room-walk.

## Gate P (~04:12 BST; honesty amends ~04:56 + ~05:21; lock ~05:45 BST)
**LOCKED TRUE** (sim stepped approach→N compose 2/2). Iterate2 resolved prior Root B FAILs (dwell clear above plant rest; non-sticky FOOT-LIFT HUD; multi-frame S9 daylight; finalize = KINEMATIC XY SHIM). Soft-pass not used. Curriculum **D–P TRUE**. Not range bump / latch / 90° / walk-through / room-walk.

## Gate Q (~05:50 BST; FAIL ~06:25; iterate3n FAIL ~08:20 BST)
**Criteria live.** Pack1 + iterate3n **GATE_Q_FAIL Root B**. Iterate3n: Controls metrics PASS + claimed continuous VISUAL_PASS; **AI independent Q03 continuous VISUAL_FAIL** — approach/retreat slide-dominant (SS-count-then-slide); sparse retreat lifts in stills do not carry continuous XY; some approach FOOT-LIFT frames flush soles; N compose OK; finalize HUD honest **KINEMATIC XY SHIM**. Soft-pass not used. `GATE_Q_VIDEO_CONFLICT.md` settled. Curriculum D–P TRUE; **Q not locked**. Awaiting Controls iterate (continuous multi-step throughout both legs). Not range bump / latch / 90° / walk-through / room-walk.
