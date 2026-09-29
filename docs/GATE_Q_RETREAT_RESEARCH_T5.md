# Gate Q — retreat research T5 (Prefer FAIL; skate-p95 wall)

**When:** Mon 28 Sep 2026 ~17:20 Europe/London (BST)  
**Owner:** Founding Controls (literature / practice → AI cospec proposal)  
**Aligns:** `docs/GATE_Q_AI_RESEARCH_T5.md` (AI ~17:16 BST) · `docs/GATE_Q_AI_SCORE_T4.md` · prior `GATE_Q_RETREAT_RESEARCH_UNSTICK.md`  
**Scope:** Deeper reverse / walk-back sweep for **honest** Prefer FAIL reopen. Soft-pass **forbidden**. Bars **KEPT** (skate mean≤0.08 / p95≤0.18; clear_frac≥0.55; tip≥8). Plant md5 lock. cancel×**0.70**. No FREEZE/qvel0/damp↑. No R*/S1–S3 twin reopen on frozen E7lock. **No train in this note.**

**Context:** E7lock sha16 `9ffaa1a21b607bf6` best-known rollback. T4 reverse-native Prefer FAIL near-miss: cf **0.687/0.821** + apps held + mid shortened, but **skate p95 ~0.205/0.217** wall; T4_FINAL bout0 first `ret_ok` only. Companion md5 `59cc408eda07037a58f92ad27da045d6` · walk plant md5 `fc94709c84f5598d4474ecfc4bb41fdc`.

---

## 1. Problem T5 must solve (not another clear_frac scrub)

| Metric | T4_09 (best rank) | Bar | Status |
|--------|-------------------|-----|--------|
| apps | 0.548 / 0.630 | hold E7lock | ✓ |
| ret clear_frac | **0.687 / 0.821** | ≥0.55 | ✓ |
| tip_ret | 27.6 / 21.5 | ≥8 | ✓ |
| mid (PRE_GAP) | 4.82 / 2.12 | shorter vs E7lock 34.9/26.7 | ✓ progress |
| **skate mean/p95** | 0.071/**0.205** · 0.069/**0.217** | ≤0.08 / **≤0.18** | **p95 FAIL** |

Scorer skate = combined stance-foot horizontal vx mean/p95 while planted (`score_gate_q.py`). T5 attacks **p95 stance shear** while **constraining** cf/apps/tip/plant-cam ε — matches AI RESEARCH_T5 framing.

**Closed — do not reopen as twins:** R1–R5 · S1 CSF50 · S2 ALIP-TVR · S3 REV-PHASE on frozen E7lock. T4 opts B+C+F already tried inside new weights (neg-Vx · rev-phase · Root B / plant-ε).

---

## 2. What serious teams do for reverse / walk-back

### 2.1 Open tools / papers (high-transfer to our residual stack)

| Source | Mechanism | Evidence | Transfer to AiNex MuJoCo residual |
|--------|-----------|----------|-----------------------------------|
| **Placo / OpenDuck** (`PlacoWalkEngine`) | `walk_max_dx_backward` ≪ `walk_max_dx_forward` (e.g. 0.03 vs 0.08); `FootstepsPlannerRepetitive` + mid-gait `replan_supports`; command negative dx | Open source presets | **Co-constraint / teacher outer:** short reverse steps cut TD shear; residual tracks clear-only targets. Prefer known tool over inventing plant. |
| **Cassie HZD gait library** (Reher/Grizzle lineage) | Discrete gaits vx∈[−0.6,1.2] gain-scheduled; impact-to-impact fixed ~0.4 s | Hardware papers / NSF | Reverse is **first-class library entry**, not mirrored forward. T4 opt B already conditioned — keep as conditioning, not sole family. |
| **Xie CoRL distillation** | Separate forward / inplace / **backward** experts → distill; reverse **harder**; multiskill hurts reverse sample-efficiency | CoRL 2019 | Supports **T5-C**: reverse-expert BC (T4 opt A was OFF) then skate-honest RL; multi-seed (opt E OFF). |
| **Bang / Sentis** ALIP-MPC + residual Δu_fp (arXiv:2407.17683) | Mid-swing foothold residual; fwd+back; smoothness on consecutive footholds | Paper | S2 Prefer FAIL **on frozen**. OK **only inside new train** (AI T5-B) with skate-p95 primary — ≠ S2 twin flag. |
| **Unitree G1 ALIP NMPC + ankle** (arXiv:2509.04722) | Step timing + **ankle** authority under ALIP/SRB | Paper | Skate lived in ankle/contact — aligns AI T5-B ankle DF **clear-only**. |
| **DCM / residual RL + oracle** (arXiv:2601.16109) | Model base + torque residual + oracle supervision | Paper | Teacher path (T5-C); Prefer FAIL if plant XY credited. |
| **Walk This Way** (PACMCGIT / ACM 2025–26) | Imitation-free RL; footstep-location + optional timing; explicit **backwards** | Paper | Timing/footstep constraints for retreat Δ without mocap; vision off OK. |
| **Swing-leg retraction / ground-speed match** (Wisse/Atkeson; Karssen; Hasaneini) | Late-swing foot tangential vel → ~0 or slight **swing-backward** vs ground; cuts impact impulse + slip likelihood | Classic + Robotica | **Direct skate-p95 attack:** our skate is stance vx after TD — TDVM/SLR shapes pre-contact clear residual so impact shear ↓. |
| **Compass gait slip-robust design** (SciRobotics / related) | Slips peak **near impact**; gaits with **backward** swing-foot vel at TD need less friction; **small step + moderate speed** more robust | Papers | Explains T4 wall (cf↑ but p95 skate): long/fast reverse TD shears. Short-cadence + SLR co-constraints. |
| **Virtual nonholonomic slip regulation** (arXiv:2603.29050) | Regulate stance tangential vel via VNHC alongside VHCs | Paper 2026 | **Caution:** planted XY residual credit **forbidden** under cancel×0.70. Use only as **reward/obs shaping** of skate, or clear-phase pre-TD — not planted soft-XY invent. |

### 2.2 Industrial / demo platforms (label speculation)

| Platform | Public evidence | Speculative transfer (labeled) |
|----------|-----------------|--------------------------------|
| **Boston Dynamics Atlas** | Trajectory-opt behavior library + MPC that stitches templates; parkour / flip blog shows offline templates + online MPC (`bostondynamics.com` parkour blog). **No detailed public reverse-gait paper.** | **Speculation:** reverse is a **library template** + MPC blend (capture/foot placement), not a soft plant slide. Transfer idea = reverse-native template/teacher, not cancel↑. |
| **Agility Digit** | Public demos include omnidirectional / reverse-ish steps in warehouse footage; control stack historically ALIP/capture-class + WBC. | **Speculation:** reverse via reduced-order step adjust + ankle; aligns T5-B class. No open AiNex port. |
| **Tesla Optimus** | Public: proprio NN loco blind on rough ground; engineer notes on velocity/direction command responsiveness (Kovac et al. commentary). **No public reverse-mechanism writeup.** | **Speculation:** reverse = **command-conditioned** policy (Cassie-like), not plant invent. T4 already did neg-Vx; T5 must add skate-primary / TD hygiene. |
| **LeRobot / humanoid IL** | HF LeRobot humanoid + sim IL paths emerging | Optional demos → BC teacher (T5-C); kit teleop not claimed yet — sim oracle OK. |

**Honest summary:** Public BD/Optimus/Digit evidence proves **walk-back exists in the wild** but does **not** publish a drop-in skate fix. Transferable mechanisms with open evidence are: (1) short asymmetric reverse steps (Placo), (2) reverse-native library/expert (Cassie/Xie), (3) late-swing TD velocity match / SLR for slip, (4) ALIP/DCM+ankle inside **new** trained stack, (5) oracle BC + constrained footstep curriculum.

---

## 3. Map to our failure mode (skate p95 while cf OK)

```
T4 achieved: clear SS banks dest cam (cf↑) under cancel×0.70
T4 failed:   p95(|v_stance_x|) still ~0.20–0.22  → Prefer FAIL bars

Literature prediction:
  high clear stride rate / late TD with non-matched tangential vel
  → impact shear → planted vx spikes (exactly our skate p95)
```

| Lever that attacks p95 | Helps cf? | Forbidden? |
|------------------------|-----------|------------|
| Late-swing TDVM / SLR (clear residual) | Neutral–help (softer land) | No — clear-only |
| Short `dx_back` / higher cadence curriculum | May *trim* cam/step — constrain cf≥0.55 | No — co-constraint |
| Skate-mean/p95 **primary** reward (AI T5-A) | Must constrain cf | No |
| Ankle DF residual clear-only (AI T5-B flavor) | Help TD | No if clear-only |
| Planted soft-XY / cancel↑ / FREEZE / damp↑ | Fake skate cut | **FORBIDDEN** |
| Soften skate bar | Cosmetic pass | **FORBIDDEN** |

---

## 4. Ranked Prefer FAIL families (≠ R*/S1–S3/T4 twins)

**Controls ranking matches AI RESEARCH_T5** (cite agreement). Mechanism notes below are Controls elaboration, not a fork.

| Rank | Family | Mechanism (one line) | Why ≠ closed |
|------|--------|----------------------|--------------|
| **1 — PRIMARY** | **T5-A Skate-primary reverse residual** | New PPO residual (bootstrap T4_09 near-miss **or** E7lock); reward **dominated by skate mean/p95** + plant-cam ε + Root B; **constrain** cf≥0.55; clear-only residual (optional late-swing TDVM/SLR + ankle DF shaping); planted cancel×0.70 only | ≠ T4 (T4 optimized clear/Root B first → cf solved, p95 wall). ≠ S1–S3/R* flags on frozen. |
| **2** | **T5-B ALIP/DCM + ankle NMPC base + residual** | Reduced-order reverse footstep + ankle prior; residual closes body gap **in new weights**; mid-swing replan OK | ≠ S2 env-flag Δu_fp on frozen E7lock |
| **3** | **T5-C Teacher BC + multi-seed (finish T4 A+E)** | Oracle CSF/Placo/teleop reverse BC → RL; multi-seed Prefer FAIL; optional Walk-This-Way footstep timing | T4 left A+E OFF; now skate-honest curriculum |
| 4 (park) | Placo outer-only track | PlacoWalkEngine retreat outer; residual = joint track | Heavier integration; escalate if A–C die |
| 5 (Dave only) | T5-D architecture / plant | Honest plant/geom ask | **Not auto-open** |

### Primary pick detail — T5-A

**Name:** `T5-A SKATE-PRIMARY` (Controls short name: **T5-SKATE** / family tag `T5`)

**Mechanism:** Thin finetune from T4_09 zip if available else E7lock on `AinexReverseResidualEnv` (or T5 twin env). Invert T4 reward priority: **skate mean + heavy p95** dominate return shaping; clear_frac enters as **constraint / floor bonus** (already near bar), not the primary climb. Late-swing (clear) residual encouraged to drive swing-foot tangential velocity toward ground-speed match / slight swing-backward (SLR literature) and optional ankle DF for soft TD — **authority only while FOOT-LIFT clear**. Planted windows: cancel×0.70 + vel-oppose only — **zero** planted soft-XY stride credit. Placo-style `|dx_back|` cap as optional co-constraint (log). Plant-cam ε Prefer FAIL. Soft-pass **off**.

**Why it attacks skate p95 without trading apps:** Apps stay on E7lock approach dual-ckpt path. cf already ≥0.55 on T4_09 — constrain not maximize. Literature says TD tangential match + short reverse steps cut friction demand at impact — the p95 spikes — without needing cancel↑.

**Eval plan:** Dual-ckpt (approach=E7lock · retreat=T5 candidate). Fair Prefer FAIL probes vs T4_R0≡E7lock and vs T4_09. Success = `ret_ok` both under bars + plant-cam ≤ε + apps held. Prefer FAIL train wall like T4 (≤4 h or ≤2e6 steps).

**Stays OFF:** R*/S1–S3 twin flags · soft-pass · bar soften · cancel↑ / FREEZE / qvel0 / damp↑ / kd↑ as plant reopen · plant invent · vision-in-walk · Path A.

### Runners-up (for escalation order)

1. **T5-B** — if T5-A Prefer FAIL wall with cf held but p95 still stuck → model-based ankle/ALIP base in new weights.  
2. **T5-C** — if reward-shaping alone under-explores TD hygiene → BC teacher + multi-seed.  
3. **Placo-outer** (Controls addition) — if A–C die; still no plant invent.

---

## 5. Controls vs AI RESEARCH_T5 (brief)

| Topic | Stance |
|-------|--------|
| Primary family | **Agree** — T5-A skate-primary first |
| Escalation | **Agree** — T5-B then T5-C; T5-D only with Dave |
| Soft-pass / bars / md5 / cancel×0.70 | **Agree** — KEPT |
| Closed twins | **Agree** — do not reopen R*/S1–S3 |
| Mechanism elaboration | Controls adds **TDVM/SLR + Placo dx_back** as *in-family* clear-phase tools under T5-A (not a separate reopen). No disagreement on ranking. |
| VNHC / planted slip feedback | Controls **stricter**: no planted soft-XY residual; skate regulated via reward + clear-phase TD shaping only |

---

## 6. Disposition

Gate Q remains **OPEN — Prefer FAIL PARK after T4**. Research reopen **aligned** with AI: ship Prefer FAIL cospec for **T5-A**. Soft-pass never. No train from this note. No Q03 until `ret_ok` both + continuous watch.

**Proposal path:** `docs/GATE_Q_AI_COSPEC_T5_PROPOSAL.md`  
**Next:** AI veto / no-veto → `docs/GATE_Q_AI_COSPEC_T5.md` → Controls train only after no-veto.
