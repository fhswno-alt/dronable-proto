# Gate E — AI curriculum criteria (Path A sim)

**When:** Sun 27 Sep 2026 ~21:30 BST (criteria); **LOCKED FALSE** ~21:34 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove)  
**Plant:** `ainex_controls_m2_145.xml` (M2 145×86) — no plant change  
**Path A spend:** FROZEN — sim only  

## Verdict (locked)

| Track | Result | Evidence |
|-------|--------|----------|
| **Gate E cadence (open-loop family)** | **FALSE — hard falsifier** | Classic CPG+fixers+auth+GEO+POC — see stack below |
| **Gate E cadence (sim learned residual)** | **TRUE — UNLOCKED** ~00:58 BST 28 Sep 2026 | `LEARNED_GATE_E_NOTE.md` RL04_locked_T75; ckpt `learned_gate_e/ppo_gate_e_best.zip` |
| **Dynamic CPG skate-kill** | **FALSE** (open-loop) | C11/D94; superseded for cadence claim by learned unlock |
| Plant change | **NOT required** | Unlock on locked M145 @ k_auth=1.0 |

**Open-loop Gate E fail note (historical):** `previews/ainex_walk/iterate/GATE_E_CONTROLS_NOTE.md`  
**Learned unlock:** `previews/ainex_walk/iterate/LEARNED_GATE_E_NOTE.md` + table; videos `RL04_locked_T75.mp4` / `RL04_locked_T88.mp4`  
Gate D CSF50 unchanged. **Non-claim:** not Pi / Orin / NN-first kit gait.

### Locked falsifier stack (do not reopen for Gate E)
Open-loop CPG+VIK; linear residual; WBC; MPC; dual-T; HX auth envelope; contact/geometry; plant×outer — all HARD-FALSIFIED. Unlock class = **state-dependent PPO residual** only.

### Next lever (AI)
1. Freeze ckpt + seed; reproduce RL04 eval; continuous-video AI confirm.
2. Gate F / lever approach sensing (vision off walk loop) — Monday perception checklist.
3. **No** Pi NN deploy plan until Dave asks. Still sim-only / no spend talk.

---
## Gate E pass (all, assist OFF, freeze OFF)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit Gate D | `clean_walk_ss` TRUE on same plant |
| 2 | Multi SS | ≥ **5** alternating SS bouts each foot |
| 3 | Cadence | Mean step period ≤ **0.75 s** (≥ **1.3** steps/s) — CSF50 T=0.88 is below; must speed without skate/tip |
| 4 | Bout quality | Mean SS bout L/R ≥ **0.10 s**; peak sole clear ≥ **0.012 m** |
| 5 | Skate | Stance \|vx\| mean ≤ **0.08** and p95 ≤ **0.18** |
| 6 | Progress | tip_free ≥ **8 s**; dx ≥ **0.15 m**; hip_corr ≤ **−0.45**; foot_lead ≥ **5** |
| 7 | Continuous video | MP4 continuous articulation PASS |

**Hard falsifier:** cadence only via skate, tip, kill L/R alt, assist/freeze ON, or drop below Gate D clear-swing.

**Not Gate E:** E80 shuffle; CSF50 slow SS alone; MAD/HUD soft-pass.

## Dynamic walk (parallel — Controls)

| Claim | Pass | Fail |
|-------|------|------|
| Dynamic CPG skate-kill | tip≥8, dx≥0.05, alt OK, stx mean≤0.08 and p95≤0.18, assist OFF (C11/D94-class) | Honest tip/dx/stx/alt table — no soft-pass |

## Perception / compute (sim honesty)

| Item | Rule |
|------|------|
| Vision | **Off** walk loop (Gate F / lever only) |
| CoP HX | **MARGINAL** — not SS/cadence prove; use clearance + IMU+q CP |
| On-Pi | No NN-first gait; no Orin |
| Contact height | XML box 16 mm vs CAD ~4.5 mm is proxy — if it skews clear/cadence, treat as plant honesty issue with Hardware, not a sensing unlock |

## Monday

Mon perception/compute checklist **after** Gate E moves (pass or hard falsifier). Cart shut until assembly greenlight.

## 24h Controls artifacts

Gate E: `BEST_GATE_E_*.{mp4,json}` **or** falsifier table. Dynamic: pass clip **or** skate-fail metrics.
