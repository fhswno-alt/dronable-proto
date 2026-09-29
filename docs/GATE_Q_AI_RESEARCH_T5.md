# Gate Q AI research — T5 deeper reverse gait (Prefer FAIL) ~17:16 BST

**Owner:** Founding AI Scientist  
**Trigger:** Dave greenlit deeper walk-back research (BD/Optimus-class reverse gait → Prefer FAIL new residual). Soft-pass **off**. Bars **KEPT**.  
**Parallel to:** Controls literature → `docs/GATE_Q_AI_COSPEC_T5_PROPOSAL.md` (when ready).  
**Stance:** Companion md5 lock. Plant freeze. No Path A / spend. No reopen R*/S1–S3 twins on frozen E7lock. E7lock `9ffaa1a21b607bf6` stays rollback until a new sha16 Prefer-FAIL beats it honestly.

## Closed wall (why T5 must differ)

| Closed | Disposition | Lesson |
|--------|-------------|--------|
| R1–R5 | Prefer FAIL / veto | Scheduler twins on Gate E residual |
| S1 CSF50 / S2 ALIP / S3 REV-PHASE | Prefer FAIL on frozen ckpt | Capture / Δu_fp / phase flip ≠ reverse-native dynamics |
| **T4 reverse residual** | Prefer FAIL near-miss (`GATE_Q_AI_SCORE_T4.md`) | **clear_frac largely solved** (0.687/0.821); planted middle shortened; **skate p95 ~0.20–0.22** is the wall; bout0 `ret_ok` once |

T5 must attack **skate-honest reverse locomotion** (p95≤0.18 both bouts) while **holding** cf≥0.55 + apps + tip≥8 + plant-cam ε — not another clear_frac-only scrub.

## Literature / open-stack scan (AI) — Prefer FAIL fit

| Idea | Source flavor | Fit for Gate Q T5? |
|------|---------------|---------------------|
| ALIP-MPC + mid-swing residual Δu_fp (forward+back) | Bang/Sentis arXiv:2407.17683; thesis ALIP+RL residual | **Partial used in S2 on frozen** — OK **only inside new train** with skate-p95 primary, not S2 twin flag |
| Hierarchical ALIP NMPC + SRB (step timing + ankle) | arXiv:2509.04722 Unitree G1 | **In** as model-based **base** for reverse Vx + ankle torque authority — skate often lived in ankle/contact |
| DCM / model-based base + residual RL + oracle supervision | arXiv:2601.16109 torque residual | **In** — oracle reverse teacher (T4 opt A was OFF); Prefer FAIL if plant XY credited |
| Imitation-free constrained walking (fwd+back, footstep timing) | Walk This Way (ACM 2025/26) | **In** — footstep / timing constraints for retreat Δ without mocap; vision off |
| Cassie-style vx∈[−1,1] library | prior research T4 | **Already tried in T4 (opt B)** — keep as conditioning, not sole family |
| Teleop / BD-style capture step reverse | industrial / Atlas-class practice | **In** as **teacher trajectories** in sim only — not claim kit teleop yet |
| Plant invent / µ / STEP / FOV vision-memory | — | **Out** |

## Ranked T5 reopen candidates (Prefer FAIL; AI cospec required first)

### T5-A — Skate-primary reverse residual (preferred first)

- **What:** New PPO residual (bootstrap T4_09 or E7lock) with **reward dominated by skate mean/p95** + plant-cam ε + Root B, while **constraining** clear_frac≥0.55 (already near). Explicit tangential contact / ankle DF residual **only while clear**; planted cancel×0.70 held.  
- **Why different from T4:** T4 optimized clear/Root B and got cf; T5 inverts priority to **skate p95** without trading cf/apps.  
- **Must-holds:** soft-pass off; bars; ε; tip≥8; apps; md5; budget wall like T4.  
- **Dave:** greenlit research already — cospec before spin.

### T5-B — Model-based reverse base + residual (ALIP/DCM/ankle NMPC)

- **What:** Reduced-order reverse footstep + ankle torque prior (ALIP S2S / DCM), residual closes full-body gap; mid-swing replan OK **in new weights**.  
- **≠ S2:** Not env-flag Δu_fp on frozen E7lock — trained stack.  
- **Risk:** complexity; Prefer FAIL if collapses to plant skate.

### T5-C — Reverse teacher / constrained footstep curriculum (finish T4 opts A+E)

- **What:** BC from oracle CSF/teleop reverse + multi-seed Prefer FAIL wall; optional Walk-This-Way-style footstep timing constraints for retreat.  
- **Why:** T4 left **A** and **E** OFF — unfinished creative budget, but now framed as skate-honest curriculum.  
- **Must-holds:** same bars; teacher never credits plant XY.

### T5-D — Architecture / plant (only with Dave + Hardware)

- **What:** If T5-A–C Prefer FAIL walls die → honest plant/geom or architecture ask.  
- **Not** auto-open.

### Explicitly out

Soft-pass · skate/cf bar soften · FREEZE/cancel↑ · R*/S1–S3 twins · plant invent without Dave · vision-in-walk · Path A spend · lock from skip-video.

## Coop with Controls

1. Controls finishes `GATE_Q_AI_COSPEC_T5_PROPOSAL.md`.  
2. AI same-turn **veto / no-veto** → `docs/GATE_Q_AI_COSPEC_T5.md`.  
3. Prefer start **T5-A** unless Controls has a stronger model-based base ready (T5-B).  
4. Soft-pass off throughout. No Q03 until `ret_ok` both + continuous watch.

## Status

Curriculum D–P TRUE. Gate Q **OPEN — Prefer FAIL PARK after T4**; research reopen **GREENLIT**. Soft-pass off. No spend.
