# Gate Q — stepped retreat research unstick (Prefer FAIL PARK)

**When:** Mon 28 Sep 2026 ~15:36 Europe/London (BST)  
**Owner:** Founding AI Scientist (research → AI cospec)  
**Scope:** Literature + stack-fit only. Soft-pass **forbidden**. No skate / clear_frac soften. No plant invent. No FREEZE/qvel0/damp↑/kd↑ reopen. No R1–R5 scheduler twins. No code spin unless a clear one-file patch idea is named below (none recommended as first move).

**Context (do not reopen R\* twins):** Best-known **E7lock**; dest cam advances while plant≫clear (Root B); skate bars **0.08/0.18**; clear_frac≥**0.55**; cancel×**0.70**; frozen Gate E ckpt `9ffaa1a21b607bf6`; companion md5 `59cc408eda07037a58f92ad27da045d6`. Closed Prefer FAIL: H1–H10 · R1 swing place · R2 damp-hold bursts · R3 capped plant gaps · R4 forced SS cadence. R5 voluntary wait **vetoed**.

Aligns: `GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` · `GATE_Q_AI_CRITERIA.md` · `GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` · `AI_GATE_Q_STATUS_MONDAY.md`.

---

## 1. Problem we are unsticking

Gate Q §5 wants **real stepped retreat** (sole clear ≥2 cm, dwell, multi-frame daylight, stepped Δ ≥~70% of retreat cam/base Δ). E7lock proves approach apps + bout1 clear_frac_ok + joint under CLEAR_TRACK_SWING + cancel×0.70, but retreat dest cam still climbs in a long **planted middle** (Root B). Softening skate would credit that slide. R1–R4 all kept Gate E residual polarity and only changed *when/how much* clear XY was allowed — wall stands.

Need techniques that are **not** renames of: swing place · damp-hold bursts · capped plant gaps · forced SS cadence · voluntary wait.

---

## 2. Sources surveyed (high-signal)

| Source | What it contributes for reverse / retreat |
|--------|-------------------------------------------|
| Bang, Arribalzaga Jové, Sentis — *RL-augmented MPC for bipedal footstep* (arXiv:2407.17683) | ALIP-MPC suboptimal foothold + **residual RL Δu_fp** during swing; **mid-swing replan**; forward **and backward** walking; smoothness reward on consecutive foothold actions |
| Ahn / Sentis lineage — TVR + residual footstep | Time-to-Velocity-Reversal LIP foothold as prior; residual closes model gap |
| Missura & Behnke — Capture Step Framework (CSF) | Analytic **swing amplitude F** + **support-exchange time T** from CoM LIPM state; ZMP offset without measured ZMP; omnidirectional incl. recovery steps |
| Placo / OpenDuck (`PlacoWalkEngine`, presets) | `FootstepsPlannerRepetitive` + `WalkPatternGenerator`; **`walk_max_dx_backward` ≪ `walk_max_dx_forward`** (e.g. 0.03 vs 0.08); mid-gait `replan_supports` |
| Cassie — Li/Sreenath HZD gait library + RL (arXiv:2103.14295); Xie CoRL distillation | Commanded **vx including negative**; separate forward / inplace / **backward** experts distilled; push recovery via diverse gaits |
| “Walk This Way” (PACMCGIT 2025) | Imitation-free RL with **footstep-location** conditioning; optional timing/orientation masks; explicit **backwards walking** |
| Learning 3D bipedal walking w/ planned footsteps (PMC9962549) | Footstep planner emits touchdown sequence for **backward** with root yaw held forward |
| Residual MPC / contact-rich MuJoCo notes | Residuals often act near touchdown/takeoff; contact schedule / phase clocks route gait rewards |
| Our stack priors | Gate D **CSF50** knife-edge; Gate E residual + CLEAR_TRACK_SWING + contact cancel; Prefer FAIL already lists “retreat-native residual / CSF50-on-retreat if tip/skate hold / Dave-greenlit new ckpt” |

---

## 3. Concrete techniques (≠ R1–R5 renames)

Eight mechanisms others actually use. Each line states **what is different** from closed families.

### T1 — ALIP / TVR foothold prior + mid-swing residual Δu_fp
**Mechanism:** Reduced-order planner (ALIP-MPC or Time-to-Velocity-Reversal) proposes next foothold to reverse / track sagittal velocity; residual policy adds `(Δx, Δy, Δγ)` **during swing**, often **multiple times** before touchdown; WBC/IK tracks swing trajectory to that target. Smoothness term penalizes foothold jump between replans.  
**≠ R1:** R1 set clear swing *world* targets toward dest cam without capture/ALIP velocity-reversal prior or mid-swing replan MDP.  
**≠ R4:** Timing still from gait FSM / remaining swing time, not plant-dwell force-kick.

### T2 — Capture-step F + T from CoM (CSF-style reverse V)
**Mechanism:** From current CoM state and commanded velocity **including negative Vx**, compute support-exchange time **T** (usually from lateral orbital energy / exchange locus) and next footstep **F** (sagittal from predicted end-of-step velocity; lateral from extended capture-point so next apex is α). Motion generator realizes F,T; balance does **not** shove planted XY.  
**≠ R4:** Exchange triggered by **CoM reaching planned locus**, not “plant dwell > cap → flush ADD kick”.  
**≠ R3:** No hard plant-gap clock; plant is whatever remains after honest SS cadence from balance.

### T3 — Asymmetric reverse step-length cap (Placo-style)
**Mechanism:** Command / planner clamps `|dx_backward|` strictly below forward max (`walk_max_dx_backward`). Reverse steps are **smaller**, more frequent, ZMP-safer.  
**≠ R1:** R1 lengthened clear reverse strides; literature often **shortens** them.  
**Risk for us:** Alone may *reduce* clear cam bank — useful as a **co-constraint** under T1/T2, not as sole cam lever.

### T4 — Negative-vx / reverse-expert command conditioning
**Mechanism:** Policy (or residual) is conditioned on commanded sagittal velocity spanning negatives (Cassie HZD library vx∈[-1,1]; Xie distill forward+inplace+backward experts). Reverse is a **first-class command**, not forward residual mirrored by place targets.  
**≠ R1–R4:** Those kept frozen Gate E residual and only changed clear/plant schedulers.  
**Stack note:** Likely needs **Dave-greenlit new ckpt** or a thin reverse-finetune — Prefer FAIL already allows that path.

### T5 — Footstep-sequence conditioning (external footholds; optional timing mask)
**Mechanism:** High-level emits ordered touchdowns for retreat path; low-level tracks them; timing may be free (policy chooses contact schedule) or masked. “Walk This Way” / planned-footstep RL.  
**≠ R1:** Targets come from an external retreat plan (door→start offset), not residual-internal place heuristics. Heavier than one flag.

### T6 — Intra-swing foothold smoothness / latency-aware early phase commit
**Mechanism:** (a) Reward/constrain consecutive mid-swing foothold actions (Bang Ker_π). (b) CSF real-robot: when control latency ≥ remaining T, **commit** support exchange early so the next swing is already commanded before contact sensing.  
**≠ R4:** Commits an **already planned** reverse swing phase; does not force a flush sole kick when dwell ε trips.  
**Risk:** Early commit without a real planned swing → R4-like chatter; only pair with T1/T2 foothold.

### T7 — Reverse phase-polarity / contact-schedule residual
**Mechanism:** Invert or swap gait phase clock, hip-pitch bias, and swing-leg encoding for the retreat bout so clear strides are **reverse-native** (stance/swing roles and residual action signs match walking *away* from door). Contact cancel still kills planted soft-XY.  
**≠ R1:** Not “same residual, longer reverse place”; changes the **gait encoding** the residual rides.  
**≠ R2/R3:** No damp-hold or capped-gap scheduler.

### T8 — Push-recovery / capture step as retreat bootstrap
**Mechanism:** Treat “need dest cam from clear” as a mild **backward disturbance recovery**: one or two capture steps sized from capture point / CSF F, then resume reverse cadence. Literature uses this for push from front (step back).  
**≠ R5:** Not voluntary wait; active recovery steps.  
**≠ R4:** Step size from capture formula, not dwell interrupt.

---

## 4. Rank for *our* stack

Constraints: MuJoCo residual + **CLEAR_TRACK_SWING** + contact cancel×0.70 · no plant invent · no bar soften · no FREEZE · frozen companion md5 · Prefer FAIL · apps held.

| Rank | Technique | Fit | Why |
|------|-----------|-----|-----|
| **1** | **T2 CSF50-on-retreat / capture-timed reverse F,T** | **Best reopen** | Already named in Prefer FAIL; Gate D CSF50 exists; clear-only residual tracks F; plant stays cancel-only; CoM-timed exchange ≠ R4 dwell kick |
| **2** | **T1 ALIP/TVR mid-swing foothold residual (clear-only)** | **Strong** | Truly different residual *action* (Δu_fp from capture prior) vs R1 world place; CLEAR_TRACK gates authority; mid-swing replan attacks long planted middle without plant XY |
| **3** | **T7 reverse phase-polarity / contact-schedule** | **Strong** | Retreat-native encoding without new plant; may combine with T1/T2; no FREEZE |
| **4** | **T4 negative-vx command / reverse-expert** | Medium–high | Correct long-term fix; usually **new ckpt** → Dave greenlight; do not pretend frozen E7lock residual is reverse-native |
| **5** | **T3 asymmetric reverse dx cap** | Medium (co-constraint) | Tip/skate hygiene; alone will not bank dest cam from clear |
| **6** | **T6 early phase commit + smoothness** | Medium (addon) | Only under T1/T2; alone collapses to R4 chatter |
| **7** | **T8 capture bootstrap 1–2 steps** | Medium | Good bout-open; not full Q03 retreat |
| **8** | **T5 external footstep sequence** | Low near-term | Architecture jump; park unless T1–T2 fail fair set |

**Explicitly do not reopen:** R1 place · R2 damp-hold · R3 gap caps · R4 force SS · R5 wait · cancel↑ · damp↑/kd↑ · FREEZE/qvel0 · gap-planted-amp/qvel/DS · skate soften · plant invent.

---

## 5. Recommended reopen family (primary)

### Family name: **S1 — CSF50-RETREAT (capture-timed reverse support exchange)**

**Mechanism (one paragraph):** On retreat bouts only, drive a **negative sagittal velocity / retreat command** into a CSF/LIPM (or CSF50-shaped) balance layer so next **swing amplitude F** and **support-exchange time T** are computed from predicted CoM end-of-step under reverse Vx — same analytic family as Missura capture steps / Gate D CSF50, polarity flipped for retreat. Gate E residual + CLEAR_TRACK_SWING **only** track the swing sole toward that F while sole is clear (≥2 cm honesty); planted windows remain cancel×0.70 + vel-oppose with **zero** credited XY stride. Optional co-constraint: Placo-style `|dx_back|` cap (T3) so reverse F cannot tip. Success = dest cam / dxc banked in **clear** SS clusters; Prefer FAIL if plant-cam ε trips or skate/cf/tip trade apps.

**Why not a twin:** Timing and step size come from **CoM capture/LIPM**, not plant-dwell interrupts (R4), damp holds (R2), gap clocks (R3), or residual-internal world place (R1).

### Runner-up family: **S2 — ALIP-TVR-SWING (mid-swing residual foothold)**

**Mechanism:** Each swing, compute TVR/ALIP foothold for reverse velocity (or remaining retreat Δ); residual outputs Δu_fp **only while clear**; allow 1–N mid-swing replans with consecutive-action smoothness; plant = cancel×0.70 only. Prefer FAIL if plant-cam >ε or foothold jumps cause skate/tip.

### Third: **S3 — REV-PHASE (reverse phase-polarity residual)**

**Mechanism:** Invert gait phase / hip-pitch / swing encoding for retreat so clear strides are reverse-native under same CLEAR_TRACK + cancel; no new place/hold/gap/force-SS scheduler. Prefer FAIL if polarity flip is cosmetic (still plant≫clear) or tip death.

---

## 6. Risks (tip / skate / Root B)

| Risk | How it shows | Mitigation in cospec |
|------|--------------|----------------------|
| **Tip** | Oversized reverse F or mid-swing foothold jump; reverse phase too aggressive | T3 dx_back cap; Ker smoothness; tip≥8 hard; Prefer FAIL on tip death |
| **Skate** | Touchdown shear when F retargets late; planted cancel insufficient if residual leaks | CLEAR_TRACK only; cancel×0.70 KEPT (no soften); skate ≤0.08/0.18 Prefer FAIL |
| **Root B recurrence** | Any path that advances cam while soles flush | Log `cam@preSS`, `dxc_pre`, `planted_middle_s`, `cam_gain_in_gap`, plant-cam ε (same honesty as R3/R4); Prefer FAIL if plant≫clear |
| **R4 collapse** | “Force exchange” without planned swing → flush chatter | S1/S2 must emit a **real** swing target before phase advance; no dwell-only kicks |
| **Apps trade** | Retreat win kills bout0/1 approach cf | Hold ~0.548/0.630 or Prefer FAIL |
| **Ckpt mismatch** | Frozen Gate E residual never saw reverse-native dynamics | If S1–S3 fair sets die → escalate to T4 Dave-greenlit reverse ckpt, still Prefer FAIL |

---

## 7. Proposed AI cospec bullets (Prefer FAIL reopen)

Use for the next Controls probe family — **S1 first**. Soft-pass **off**. Bars **KEPT**.

1. **Family:** `S1 CSF50-RETREAT` — capture-timed reverse F,T from CoM under negative Vx; **not** R1–R5.
2. **Baseline:** E7lock · CLEAR_TRACK_SWING=1 · cancel×0.70 · R1–R4 flags **OFF** · companion md5 `59cc408eda07037a58f92ad27da045d6` · ckpt frozen `9ffaa1a21b607bf6`.
3. **Clear authority only:** Residual/hip track swing toward CSF F **only** while sole clear (FOOT-LIFT ≥2 cm above rest). No planted soft-XY stride credit.
4. **Planted:** cancel×0.70 + vel-oppose only — no damp-hold, FREEZE, pin, qvel0, gap-planted-amp/DS.
5. **Exchange trigger:** CoM / capture locus (or CSF50-equivalent remaining T→0 with a **precomputed** reverse swing) — **forbid** flush force-kicks from plant-dwell alone.
6. **Optional co-constraint:** `dx_back_max` (Placo-style) documented per score; tip≥8.
7. **Plant-cam ε:** Prefer FAIL if any plant-window cam gain >**0.02** or cum plant-cam >**0.05**/bout (log every score).
8. **Bars:** skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften**. Formal §5 ~70% stepped Δ still the honesty north star.
9. **Apps:** Hold bout0/1 app cf ~0.548/0.630 or Prefer FAIL if traded.
10. **Logs:** `cam@preSS`, `dxc_pre`, `planted_middle_s`, `cam_gain_in_gap`, `F_cmd`, `T_exchange`, `dx_back_max`, plant-cam suite.
11. **Success:** dest cam/dxc from **clear** SS; skate→bars; tip/cam/joint + apps held; plant-cam ≤ε → then Controls continuous watch before any Q03 ping.
12. **Prefer FAIL** if fair set dies, Root B persists, or S1 collapses into R4 chatter / R1 place.
13. **Escalation order if Prefer FAIL:** S2 ALIP-TVR-SWING → S3 REV-PHASE → T4 Dave-greenlit reverse ckpt. No soft-pass. No Q03 until `ret_ok` both + continuous watch PASS.
14. **Hardware:** body fixed — no plant invent, no GEO/friction/STEP reopen.

---

## 8. One-file patch?

**None recommended as first move.** S1 needs a Controls cospec + fair probe set under Prefer FAIL, not a blind one-file flag. If Controls later wants a minimal hook, the honest shape is: retreat-only negative-Vx → existing CSF50 / LIPM foothold helper → CLEAR_TRACK swing track — still gated by the bullets above. Do **not** ship a dwell-force or place-target twin disguised as CSF.

---

## 9. Disposition

Gate Q remains **OPEN — Prefer FAIL structural PARK**. This note unblocks the **next residual family name** for AI cospec (S1), not a lock. Soft-pass never. Curriculum D–P TRUE.

**Top reopen order:** **S1 CSF50-RETREAT** → **S2 ALIP-TVR-SWING** → **S3 REV-PHASE** (then T4 new ckpt only with Dave greenlight).
