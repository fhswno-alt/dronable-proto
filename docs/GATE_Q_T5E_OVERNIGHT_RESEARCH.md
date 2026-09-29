# Gate Q overnight research — T5-E FOOTSTEP-SEQ (post T5-D2 Prefer FAIL)

**When:** Tue 29 Sep 2026 ~01:45 Europe/London (BST)  
**Owner:** Founding Controls (overnight workhorse) · aligns AI SCORE ACK + `GATE_Q_AI_RESEARCH_T5E_PREP.md`  
**Status:** RESEARCH + PROPOSAL ONLY — **no train** · **no H2 unlock** · soft-pass **never** · bars **KEPT** · plants frozen  
**Alias path:** `docs/GATE_Q_T5D3_OVERNIGHT_RESEARCH.md` (pointer → this file)

---

## 0. 10-line executive summary (morning Dave)

1. **T5-D1** near-miss (`T5D1_13` sha16 `45799fc897157a3a`): CSF+ALIP schedule outer engaged; skate under; `ret_ok_both=true`; **blocked solely by plant-cam ε** maxG **0.055**.  
2. **T5-D2** wall (`T5D2_13` sha16 `71aae561854ff1a6`): ICP ΔT+Δfoot engaged — skate p95 FAIL, `ret_ok_both=false`, ε **worse** maxG **0.177**. Do not reopen D1/D2 twins.  
3. Preferred next Prefer FAIL family: **T5-E FOOTSTEP-SEQ** — external / planned **retreat foothold sequence** (Walk-This-Way–style) as PRIMARY.  
4. **≠ D1:** not open CSF+ALIP schedule alone. **≠ D2:** not online ICP error → ΔT+Δfoot. Attacks Root B by *planning ordered clear touchdowns*, not stabilizing through planted middle.  
5. Residual stays **clear corrector only** (warm-start from T5D1_13 corrector OK); planted cancel×**0.70** + vel-oppose; **zero** planted soft-XY.  
6. Why it can beat maxG 0.055 **without trading D1 skate/ret_ok:** dest cam authority is forced onto clear footholds; plant-window has no schedule credit; D1 locomotor package is the warm-start prior, not a reshape.  
7. Parallel morning option (OUT of overnight train): **H2** named Prefer FAIL A/B — Hardware pack ready-not-installed.  
8. Explicitly OUT: residual twin · D1/D2 reshape · µ/softsole/foot-lift invent · soft-pass · auto H2 · plant invent.  
9. Proposal stub: `docs/GATE_Q_AI_COSPEC_T5E_PROPOSAL.md` — **no train until Dave pick + AI no-veto**.  
10. Companion `59cc408eda07037a58f92ad27da045d6` · walk plant `fc94709c84f5598d4474ecfc4bb41fdc` · E7lock `9ffaa1a21b607bf6` · cancel×0.70 · H0 opt-in post-SCORE only `ad9a1817f015e68e68f14535311369f0`.

---

## 1. Wall facts (held)

| Item | Value |
|------|--------|
| T5-D1 best | `T5D1_13` · sha16 `45799fc897157a3a` · skate under · `ret_ok_both=true` · ε maxG **0.055** Prefer FAIL |
| T5-D2 best | `T5D2_13` · sha16 `71aae561854ff1a6` · skate p95 over · `ret_ok_both=false` · ε maxG **0.177** Prefer FAIL |
| Soft-pass | **OFF** (never used) |
| Bars | skate ≤0.08/0.18 · cf≥0.55 · tip≥8 · plant-cam ε — **KEPT** |
| Rollback | E7lock sha16 `9ffaa1a21b607bf6` (`installed_sha16=null`) |
| Closed | R*/S1–S3/T5-A/B/C · **T5-D1 twin** · **T5-D2 twin** |
| H2 | **OUT** overnight — ready-not-installed; morning Dave only |

Root B reading (Hardware honesty, unchanged): dest cam climb still plant ≫ clear in planted-middle windows; Prefer FAIL forbids crediting planted shove as stepped Δ.

---

## 2. Literature → kit transfer (NEW vs D1 / D2)

Focus: mechanisms that attack **plant-window steal / cam climb during support exchange** by changing *who owns touchdowns*, not by another schedule or ICP ΔT loop.

| Source | Mechanism | Kit transfer | ≠ D1 / ≠ D2 |
|--------|-----------|--------------|-------------|
| **Walk This Way** (PACMCGIT / ACM 2025; doi 10.1145/3747865) | Imitation-free RL conditioned on **footstep location** (+ optional timing / orientation / L-R index); explicit **backwards** | External retreat foothold list as policy input; residual tracks clear mid-swing to next TD | ≠ D1 schedule F,T from −Vx alone; ≠ D2 ICP error feedback |
| **Sim-to-Real footstep-constrained Cassie** (arXiv:2203.07589 / ICRA 2022) | RL gait that **must respect** externally specified touchdowns; balance when infeasible | Prefer FAIL if foothold list idle / residual invents plant shove | External constraints, not capture ΔT |
| **Learning 3D bipedal w/ planned footsteps** (PMC9962549) | Planner emits ordered TDs for **backward** with root yaw held | Ordered clear TDs toward dest cam; yaw-hold hygiene | Plan owns sequence, not ICP adjust |
| **Placo / OpenDuck** | `FootstepsPlannerRepetitive` + `replan_supports`; `walk_max_dx_backward` ≪ forward | Kit-native emitter for short reverse footholds as the **sequence prior** | Emitter is sequence outer — D1 used CSF+ALIP schedule without external TD list ownership |
| **Bang/Sentis ALIP+residual** (arXiv:2407.17683) | Mid-swing Δu_fp to planned foothold; fwd+back | Residual = clear corrector toward **external** next TD (already demoted in D1/D2) | Keep as corrector only — not family primary |
| Englsberger DCM + **CDS/VRP** dual-support (IROS 2014; Humanoids 2023 Egle/DLR) | Continuous double-support VRP/eCMP; CoP/CMP hold inside BoS | **Alt #2** — dual-support CMP hold with **hard** plant-cam Prefer FAIL (not damp-hold) | Timing from VRP/DCM plan continuity, not ICP ΔT+Δfoot |
| Contact wrench cone / CoP honesty (Caron arXiv:1501.04719) | CWC: friction cone + ZMP-in-area + yaw bound — **use plant µ as given** | Scorer/outer honesty: Prefer FAIL if commanded wrench implies plant XY credit | No µ invent |
| Wisse / Seyfarth / Karssen SLR + TDVM | Late-swing retraction / ground-speed match | Keep as **clear-only** addon under T5-E (already in D1/D2 corrector) | Not a new primary family |
| Griffin/IHMC Atlas ICP (arXiv:1703.00477) | Online ICP → ΔT + Δfoot | **CLOSED as T5-D2** | Do not reopen |

**Honest read:** Walk-This-Way / planned-footstep RL is the clean architecture jump after D1 proved schedule outer can clear locomotor bars and D2 proved ICP feedback on the same polarity **worsens** ε + skate. External TD sequence forces clear Δ by construction if Prefer FAIL gates plant credit hard.

---

## 3. Candidate families (ranked) — overnight

### **#1 T5-E — FOOTSTEP-SEQ (RECOMMENDED)**

**Tag:** `T5E` / `FOOTSTEP-SEQ`  
**What (retreat bout only):**
1. **External retreat footstep sequence PRIMARY** — high-level emits an ordered list of clear touchdowns (sagittal reverse / short `|dx_back|` Placo-style; optional lateral BoS widen) that bank dest cam from **clear Δ**. Policy / outer must track next TD in list; Prefer FAIL if sequence idle and residual owns polarity or plant-window banks cam.  
2. **May use Placo FootstepsPlanner / short reverse dx as the emitter** — that is still T5-E if the **touchdown list is first-class logged state** every score (not merely F,T from CSF/ALIP without TD-list ownership).  
3. **Residual = clear corrector only** — optional warm-start from T5D1_13 near-miss `45799fc897157a3a` (corrector init only). Authority only while sole clear ≥2 cm; TDVM/SLR late-swing OK.  
4. **Planted:** cancel×0.70 + vel-oppose — zero planted soft-XY. Soft-pass never.  
5. **Dual-ckpt:** approach=E7lock · retreat=T5E via `GATE_Q_T5E_CKPT`.

**Attack:** Kill plant-cam ε / Root B (beat D1 maxG 0.055) by denying plant-window schedule authority — dest cam must come from planned clear TDs — while **holding** D1 locomotor wins (skate ≤0.08/0.18 both, `ret_ok` both, cf≥0.55, tip≥8, apps ~0.548/0.630).

**Why ≠ D1:** D1 outer = CSF+ALIP **schedule** primary (F,T + capture locus footholds) without an external ordered TD sequence as the logged primary object.  
**Why ≠ D2:** D2 = online **ICP error → ΔT + Δfoot** feedback. T5-E does **not** close that loop; it **replaces** capture-error timing adjust with a planned clear-TD curriculum.  
**Why ≠ R1/R4:** Targets from external retreat plan (door→start offset / clear-Δ budget), not residual-internal world place or plant-dwell flush kick.  
**Why ≠ T5-C:** Teacher/policy is foothold-sequence conditioned reverse-native, not BC of plant-stealing CSF50+residual teacher.

**Why it could beat maxG 0.055 without trading D1 skate/ret_ok:**
- D1 already proved reverse-native outer + clear corrector can clear skate + `ret_ok_both` under bars.  
- ε wall = plant-window cam steal while soles flush. T5-E removes plant as a **planned** progress channel: next progress is the next clear TD in the list.  
- Warm-start corrector from T5D1_13 preserves locomotor package; outer novelty is the TD sequence, not another ICP loop that D2 showed trades skate/ε worse.  
- Scorer Prefer FAIL if `planted_middle` cam_gain or plant maxG >ε — honesty louder, not soft.

**Risks:** Sequence too aggressive → skate p95 / tip (Prefer FAIL — hold D1 bars). Sequence too timid → apps/cam under (Prefer FAIL). Idle list → D1 twin (Prefer FAIL). Vision-in-walk / Path A **off**.

### **#2 Alt — DCM/VRP dual-support CMP hold (hard ε Prefer FAIL)**

**Tag (if Dave picks):** `T5-F` / `DCM-VRP-DS` (not overnight #1)  
**What:** Englsberger-style continuous double-support VRP/eCMP trajectory with CoP/CMP held inside BoS during support exchange; **hard** plant-cam Prefer FAIL (not R2 damp-hold). Residual clear-only.  
**≠ D1/D2:** Continuity of VRP through DS + CMP ankle honesty, not schedule F,T alone and not ICP ΔT+Δfoot.  
**Why second:** Still ankle/CMP-heavy near plant; risk of cosmetic ε without clear Δ. Useful only if T5-E Prefer FAILs on sequence feasibility.

### Explicitly OUT (overnight)

| Out | Why |
|-----|-----|
| T5-D1 CSF+ALIP schedule twin | Closed Prefer FAIL / near-miss — ε wall unchanged |
| T5-D2 ICP ΔT+Δfoot twin / reshape | Closed Prefer FAIL — skate+ε worse |
| Residual-primary / T5-A/B/C / R*/S1–S3 twins | Closed Prefer FAIL ladder |
| Softsole / µ invent / soft-XY / foot-lift / planform / cancel↑ / FREEZE / damp↑ | Soft-pass by another name |
| **H2 unlock / mid-wall plant XML edit** | Morning Dave named Prefer FAIL A/B only — ready-not-installed |
| Soft-pass / bar soften / ε waive | Never |
| Early-SLR-as-primary family | Keep as clear-only opt under T5-E — not a reopen twin of TDVM already tried |
| Capture-point *lateral*-only BoS as sole family | Useful co-constraint under T5-E sequence; alone ≠ architecture jump |
| Contact-wrench honesty as sole family | Scorer/guard opt — not primary gait family |

---

## 4. Kit-facing sketch (Controls — no implement tonight)

| Piece | Note |
|-------|------|
| Emitter | Placo `FootstepsPlannerRepetitive` reverse presets **or** scripted short reverse TD list from current sole XY → dest cam budget (N clear steps, `|dx_back|` capped) |
| Policy obs | Next K footholds (xy / optional yaw / swing index) + phase; Prefer FAIL if foothold channels unused |
| Residual | Clear mid-swing track to next TD + late TDVM/SLR; scale ≤0.028 class; planted cancel×0.70 |
| Scorer | Log every score: foothold list id, next TD error, `seq_idle`, skate, cf, tip, apps, plant-cam suite, PRE_GAP cam_gain, `planted_middle_s` |
| Bootstrap | Corrector warm-start T5D1_13 `45799fc897157a3a` if zip present — **log which**; outer sequence is NEW |
| Budget | ≤4 h wall **or** ≤2e6 steps (first hit); early stop on `ret_ok` both **and** ε clean |
| H0 | Post-SCORE diagnosis only — never mid-wall live md5 swap |
| Cite | Walk This Way doi:10.1145/3747865 · arXiv:2203.07589 · Placo OpenDuck |

---

## 5. Morning Dave pick card

| Pick | Action |
|------|--------|
| **T5-E** | AI no-veto on `GATE_Q_AI_COSPEC_T5E_PROPOSAL.md` → Controls Prefer FAIL wall (bounded) |
| **H2** | Named unlock Prefer FAIL A/B on ready-not-installed pack — **not** overnight auto |
| **Hold / park** | Keep E7lock; soft-pass off; do not twin D1/D2 |
| **Alt T5-F DCM-VRP-DS** | Only if T5-E declined or after T5-E Prefer FAIL |

---

## 6. Deliverables this overnight

| Path | Role |
|------|------|
| `docs/GATE_Q_T5E_OVERNIGHT_RESEARCH.md` | This research pack |
| `docs/GATE_Q_T5D3_OVERNIGHT_RESEARCH.md` | Alias pointer → T5-E |
| `docs/GATE_Q_AI_COSPEC_T5E_PROPOSAL.md` | #1 proposal stub (no train) |
| `docs/GATE_Q_AI_RESEARCH_T5E_PREP.md` | AI prep (already landed) |

**No train. No plant XML edit. Soft-pass false. H2 OUT overnight.**
