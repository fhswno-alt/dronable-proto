# Gate Q Controls proposal — family T5-E Prefer FAIL (FOOTSTEP-SEQ outer)

**When:** Tue 29 Sep 2026 ~01:45 Europe/London (BST)  
**To:** AI Scientist · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen (proposal)  
**Dave:** Morning pick card — T5-E vs H2 named A/B vs hold. **No overnight train.**  
**Refs:** `docs/GATE_Q_T5E_OVERNIGHT_RESEARCH.md` · `docs/GATE_Q_AI_RESEARCH_T5E_PREP.md` · `docs/GATE_Q_AI_SCORE_T5D.md` · `docs/GATE_Q_AI_SCORE_T5D2.md` · Walk This Way doi:10.1145/3747865 · arXiv:2203.07589  
**Ask:** Formal **T5-E** cospec (AI → `docs/GATE_Q_AI_COSPEC_T5E.md` veto/no-veto) **after Dave morning pick**. Soft-pass never. **No train until Dave + AI no-veto.**

---

## Why T5-E (not twin, not H2, not residual reopen)

| Closed | Disposition | Lesson |
|--------|-------------|--------|
| R1–R5 · S1–S3 · T5-A/B/C | Prefer FAIL | Residual / scheduler twins ≠ reverse-native outer |
| **T5-D1** `T5D1_13` sha16 `45799fc897157a3a` | Prefer FAIL / near-miss | Schedule CSF+ALIP outer engaged; skate under; `ret_ok_both=true`; **ε maxG 0.055 only** |
| **T5-D2** `T5D2_13` sha16 `71aae561854ff1a6` | Prefer FAIL / train wall | ICP ΔT+Δfoot engaged; skate p95 FAIL; `ret_ok_both=false`; ε **worse** maxG **0.177** |

Walk This Way / planned-footstep RL: **external ordered touchdowns** condition the gait (incl. backwards). Attack Root B by *planning clear Δ*, not by ICP feedback through planted middle (D2) or open schedule alone (D1).

**≠ T5-D1 twin:** D1 = CSF+ALIP **schedule** primary (F,T + capture locus) without first-class external TD **sequence** ownership.  
**≠ T5-D2 twin:** D2 = online **ICP error → ΔT + Δfoot**. T5-E does **not** close that loop.  
**≠ R1/R4:** Targets from external retreat plan, not residual world-place or plant-dwell kick.  
**H2 OUT** of this cospec (parallel morning named unlock only — ready-not-installed). No mid-wall plant swap.

---

## Family (proposed)

**Name:** `T5-E FOOTSTEP-SEQ` (tag `T5E`)  
**What (retreat bout only):**

1. **External retreat footstep sequence PRIMARY** — emitter produces ordered clear touchdowns that bank dest cam from **clear Δ** (Placo `FootstepsPlannerRepetitive` reverse / short `|dx_back|` scripted list from sole XY → dest-cam budget). Outer owns next-TD tracking; **must log** foothold list / next TD / `seq_idle` every score. Prefer FAIL if `seq_idle` or residual owns polarity.
2. **Residual = clear-only corrector** — optional warm-start from T5D1_13 near-miss `45799fc897157a3a` (corrector init **only**; sequence outer is NEW). Authority **only while sole clear** ≥2 cm: mid-swing track to next TD + late-swing TDVM/SLR. Prefer FAIL if residual-primary collapse or D1/D2 twin without TD-list ownership.
3. **Planted windows:** cancel×**0.70** + vel-oppose only — **zero** planted soft-XY. Soft-pass **never**. No FREEZE / pin / duty / retain / full qvel0; no damp↑ / kd↑; no µ / softsole / foot-lift invent.
4. **Approach bout:** dual-ckpt approach=**E7lock** · retreat=**T5E** via `GATE_Q_T5E_CKPT`.

**Attack focus:** Kill plant-cam ε / Root B (beat T5D1_13 maxG **0.055**) via planned clear TDs while **holding** skate ≤0.08/0.18 both, `ret_ok` both, clear_frac≥0.55, tip≥8, apps ~0.548/0.630.

**Bootstrap (corrector warm-start only):**
1. T5D1_13 near-miss `ppo_t5d1_best_T5D1_13_near_miss.zip` / sha16 `45799fc897157a3a` if present
2. else T5C_s43_BC / E7lock — **log which**; Prefer FAIL if train collapses to residual-primary or D1-schedule / D2-ICP twin without foothold-sequence primary

### In-family opts

| Opt | Idea | Guard |
|-----|------|-------|
| SEQ OUTER | Ordered clear TDs; Placo reverse emitter OK | Log list / next TD / `seq_idle`; Prefer FAIL if idle |
| SHORT_DX | Placo-style `|dx_back|` cap as sequence co-constraint | Tip/skate hygiene; Prefer FAIL if alone used to waive ε |
| TDVM/SLR | Late-swing tangential → ~0 | Clear-only; Prefer FAIL if planted |
| LAT_BOS | Optional lateral BoS widen in TD list | Co-constraint under SEQ — not sole family |
| H0 | Instrument sites (contype 0) for ε / clear dwell | **Post-SCORE diagnosis only** — never mid-wall; live md5 freeze untouched |
| — | Stricter plant-cam ε / Root B scoring | Honesty louder — not bar soften |

### Explicitly OUT

| Out | Why |
|-----|-----|
| R* / S1–S3 / T5-A/B/C twins | Closed Prefer FAIL |
| T5-D1 schedule twin (no TD-list ownership) | Same ε wall |
| T5-D2 ICP ΔT+Δfoot twin / reshape | Closed Prefer FAIL — skate+ε worse |
| Residual-primary reward / BC twin | Same wall |
| µ / softsole / soft-XY / foot-lift invent / planform / cancel↑ / FREEZE / damp↑ / kd↑ | Soft-pass by another name |
| **H2** contact-height XML / mid-wall plant swap | Dave named Prefer FAIL A/B only — **not** this cospec |
| Path A / vision-in-walk / plant invent | Dave only |
| Soft-pass / bar soften / ε waive | Never |

---

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Soft-pass | **OFF** — never / Prefer FAIL hard |
| Bars | skate mean≤**0.08** p95≤**0.18** · clear_frac≥**0.55** · tip≥**8** · plant-cam ε — **KEPT** |
| Planted | cancel×**0.70** + vel-oppose only |
| Sequence primary | Retreat touchdowns from **external TD list**; residual **cannot** be sole gait authority |
| Clear authority | Residual / stride credit **only** while sole clear ≥2 cm |
| Plant-cam ε | Prefer FAIL if plant-window cam >**0.02** or cum >**0.05**/bout — **primary attack** (beat maxG 0.055) |
| Root B | Prefer FAIL if dest cam climb is plant≫clear |
| Hold D1 locomotor | Prefer FAIL if skate / `ret_ok` traded for ε cosmetics (D2 lesson) |
| Apps | Hold ~**0.548/0.630** on approach or Prefer FAIL |
| Companion / walk plant | md5s `59cc408eda07037a58f92ad27da045d6` / `fc94709c84f5598d4474ecfc4bb41fdc` — **no XML edit** |
| Sequence logging | Every score: foothold list, next TD error, `seq_idle`, skate, cf, tip, apps, plant-cam suite, PRE_GAP cam_gain, `planted_middle_s` |
| Budget | ≤**4 h** wall **or** ≤**2e6** steps (first hit); early stop on `ret_ok` both **and** ε clean |
| Dual-ckpt | approach=E7lock · retreat=T5E (`GATE_Q_T5E_CKPT`) |
| New sha16 | Install **only** after Prefer FAIL fair set beats E7lock honestly; else keep `9ffaa1a21b607bf6` |
| H0 | Opt-in **post-SCORE diagnosis only** — never mid-wall |
| Escalation if Prefer FAIL | Dave **H2** named unlock / alt DCM-VRP-DS / park — **not** residual twin; **not** auto H2; do not park without Dave |

---

## Controls stack note (for cospec — implement only after no-veto)

- Train / run: new `scripts/learned_gate_e/train_t5e_footstep_seq.py` (or documented T5E mode) — foothold-sequence outer + clear-only residual corrector  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5e_*.zip`, seq logs, `t5e_train.log`, `GATE_Q_T5E_PROGRESS.md`)  
- Scorer: `scripts/score_gate_q.py` + `GATE_Q_T5E_CKPT`; log TD list / `seq_idle` / plant-cam suite  
- Cite Walk This Way doi:10.1145/3747865 · arXiv:2203.07589 · Placo OpenDuck  
- **No train in this proposal turn** · **Do NOT edit live plant XML** · **Do NOT install a new sha16** · **H2 OUT**

---

## Baselines (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback | E7lock sha16 `9ffaa1a21b607bf6` |
| cancel | ×0.70 |
| T5-D1 note | skate under · `ret_ok_both` · ε maxG 0.055 |
| T5-D2 note | ICP engaged · skate p95 FAIL · ε maxG 0.177 |
| H2 / Path A / vision | **OFF** (H2 = parallel morning option only) |

---

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars + plant-cam ≤ε + apps held + tip≥8 + dest cam from clear Δ + sequence engaged (`seq_idle=false`) → continuous watch → AI Q03. New sha16 only after honest E7lock beat.  
- **Prefer FAIL:** budget wall; `seq_idle` / residual-primary; D1/D2 twin; Root B / ε steal; skate/`ret_ok` traded; soft bars; plant invent; H2 mid-wall; FREEZE/µ invent.  
- **Next after Prefer FAIL:** Dave **H2** named A/B **or** alt DCM-VRP-DS **or** park — soft-pass off; not residual twin; not auto H2.

---

## Spin after Dave morning pick + AI no-veto

1. AI ships `docs/GATE_Q_AI_COSPEC_T5E.md` (veto / no-veto)  
2. Controls implements T5-E under that cospec (≤4 h or ≤2e6 first-hit) — **not before**  
3. Fair Prefer FAIL vs E7lock (+ note vs T5D1_13 / T5D2_13)  
4. Same-turn Prefer FAIL score **or** `ret_ok`+ε pack after continuous watch PASS  

**No train until Dave + no-veto.** HW/MFG freeze. Soft-pass never. E7lock kept until honest beat.

**Controls recommendation:** Morning Dave pick **T5-E** as written; AI **no-veto** after pick. Aligns SCORE_T5D2 preferred next + `GATE_Q_AI_RESEARCH_T5E_PREP.md`. Parallel: H2 named A/B if Dave prefers plant honesty experiment instead.
