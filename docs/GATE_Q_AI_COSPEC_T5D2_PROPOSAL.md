# Gate Q Controls proposal — family T5-D2 Prefer FAIL (ICP step timing+location outer)

**When:** Tue 29 Sep 2026 ~00:42 Europe/London (BST)  
**To:** AI Scientist · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** T5-D2 ICP step timing+location Prefer FAIL **greenlit** (NOT H2). T5-D1 Prefer FAIL SCORED — skate/ret_ok/apps cleared; blocked solely on plant-cam ε steal maxG 0.055.  
**Refs:** `docs/GATE_Q_AI_RESEARCH_T5D2_PREP.md` · `docs/GATE_Q_AI_SCORE_T5D.md` · `docs/GATE_Q_AI_COSPEC_T5D.md` · `docs/GATE_Q_AI_COSPEC_T5D_PROPOSAL.md` · `docs/GATE_Q_AI_RESEARCH_T5D.md` · arXiv:1703.00477  
**Ask:** Formal **T5-D2** cospec (AI → `docs/GATE_Q_AI_COSPEC_T5D2.md` veto/no-veto). Soft-pass never. **No train until AI no-veto.**

---

## Why T5-D2 (not twin, not H2, not residual reopen)

| Closed | Disposition | Lesson |
|--------|-------------|--------|
| R1–R5 · S1 CSF50 · S2 ALIP · S3 REV-PHASE | Prefer FAIL on frozen E7lock | Scheduler / capture twins ≠ reverse-native dynamics |
| T4 · **T5-A** · **T5-B** · **T5-C** | Prefer FAIL near-miss ladder | Residual (or BC→residual) still owns gait polarity → skate/ε wall |
| **T5-D1** best `T5D1_13` sha16 `45799fc897157a3a` | Prefer FAIL / near-miss | Outer CSF+ALIP engaged (`outer_idle=false`); skate under both bars; `ret_ok_both=true`; cf/tip/apps held — **blocked solely by plant-cam ε** (outer plant maxG **0.055**, ε any>0.02). Soft-pass never. E7lock `9ffaa1a21b607bf6` kept. |

IHMC / Atlas public story (arXiv:1703.00477): **online Instantaneous Capture Point (ICP) error → Δ support-exchange time + Δ next foothold**, CMP→ankle — not another schedule-primary outer twin, not residual-primary reopen, not plant invent.

**≠ T5-D1 twin:** D1 outer is Placo/CSF/ALIP **schedule primary** (open-loop-ish F,T + footholds from −Vx / capture locus).  
**T5-D2:** Adds **online ICP error feedback → ΔT + Δfoot** during retreat. Outer still owns retreat polarity; residual stays optional clear-only corrector.

**≠ R4:** Timing comes from **capture-error direction**, not plant-dwell flush kick.  
**≠ R\* / S\* / T5-A/B/C:** Closed Prefer FAIL — do not reopen.  
**H2 OUT** of this cospec (Dave chose T5-D2 not H2). No mid-wall plant swap. H0 opt-in only **post-SCORE diagnosis**, never mid-wall.

---

## Family (proposed)

**Name:** `T5-D2 ICP-TIMING+LOCATION OUTER` (tag `T5D2`)  
**What (retreat bout only):**

1. **ICP online outer PRIMARY** — Instantaneous Capture Point / capture error during retreat drives **online** adjust of support-exchange time **ΔT** **and** next foothold **Δfoot**. CMP→ankle as in arXiv:1703.00477. Outer owns F,T + foothold targets; **must log** F,T / ICP ΔT Δfoot every score. Prefer FAIL if `outer_idle` (outer not engaged / residual owns polarity).
2. **Residual = optional clear-only corrector** — new weights optional; may warm-start from T5-D1 best near-miss corrector **only** (outer loop is NEW). Authority **only while sole clear** (FOOT-LIFT ≥2 cm): mid-swing track + late-swing TDVM/SLR if kept. Prefer FAIL if train collapses to residual-primary twin or T5-D1 schedule twin without ICP feedback.
3. **Planted windows:** cancel×**0.70** + vel-oppose only — **zero** planted soft-XY / stride credit. Soft-pass **never**. No FREEZE / pin / duty / retain / full qvel0; no damp↑ / kd↑; no plant invent; no µ / softsole / foot-lift invent.
4. **Approach bout:** dual-ckpt approach=**E7lock** (apps hold). Retreat=**T5D2** via `GATE_Q_T5D2_CKPT` (or documented alias).

**Attack focus:** Kill plant-cam ε / Root B plant-window steal (T5-D1 maxG 0.055 wall) via **online ICP ΔT+Δfoot** while holding skate mean≤0.08 / p95≤0.18 **both** bouts, clear_frac≥0.55, tip≥8, apps ~0.548/0.630, `ret_ok` both.

**Bootstrap (optional, corrector warm-start only — outer ICP loop is NEW):**
1. T5-D1 best near-miss `ppo_t5d1_best_T5D1_13_near_miss.zip` / sha16 `45799fc897157a3a` if zip present (corrector init only)
2. else T5C_s43_BC / E7lock — **log which**; Prefer FAIL if train collapses to residual-primary or D1-schedule twin without ICP ΔT+Δfoot

### In-family opts

| Opt | Idea | Guard |
|-----|------|-------|
| ICP OUTER | Online capture error → ΔT + Δfoot; CMP→ankle | Must log F,T / ICP ΔT Δfoot every score; Prefer FAIL if `outer_idle` |
| TDVM/SLR | Late-swing tangential → ~0 / slight swing-back at TD | Clear-only; Prefer FAIL if applied planted |
| H0 | Optional Hardware instrument sites (contype 0) for ε / clear dwell logging | **Post-SCORE diagnosis only** — never mid-wall; no physics change; live companion md5 freeze untouched |
| — | Stricter plant-cam ε / Root B scoring in scorer (Controls) | Honesty louder — not bar soften |

### Explicitly OUT

| Out | Why |
|-----|-----|
| R* / S1–S3 / T5-A/B/C twin flags on frozen E7lock | Closed Prefer FAIL |
| T5-D1 schedule twin without ICP online ΔT+Δfoot | Same wall / residual of D1 — not this family |
| Residual-primary reward / BC twin (T5-A/B/C reshape) | Same wall |
| Residual twin of R* / S* / T5-A/B/C | Closed |
| µ bump / softsole / soft-XY / foot-lift invent / planform resize / cancel↑ / FREEZE / pin / duty / retain / full qvel0 / damp↑ / kd↑ | Soft-pass by another name |
| **H2** contact-height XML edit / mid-wall plant swap | Dave chose **T5-D2 not H2** — needs **Dave named unlock** + separate Prefer FAIL A/B — **not** this cospec |
| Path A spend / vision-in-walk / plant invent | Dave only |
| Soft-pass / bar soften / ε waive | Never |

---

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Soft-pass | **OFF** — never / Prefer FAIL hard |
| Bars | skate mean≤**0.08** p95≤**0.18** · clear_frac≥**0.55** · tip≥**8** · plant-cam ε — **KEPT** |
| Planted | cancel×**0.70** + vel-oppose only; no FREEZE/pin/duty/retain/full qvel0; no damp↑/kd↑ |
| Outer primary | Retreat F,T + footholds from **ICP online** ΔT+Δfoot; residual **cannot** be sole gait authority |
| Clear authority | Residual / stride credit **only** while sole clear ≥2 cm |
| Plant-cam ε | Prefer FAIL if plant-window cam >**0.02** or cum >**0.05**/bout (same honesty as T5-D1 SCORE) |
| Root B | Prefer FAIL if dest cam climb is plant≫clear |
| Apps | Hold ~**0.548/0.630** on approach or Prefer FAIL |
| Companion / walk plant | md5s `59cc408eda07037a58f92ad27da045d6` / `fc94709c84f5598d4474ecfc4bb41fdc` — **no XML edit**; no live plant invent |
| Outer logging | Every score: F,T / ICP ΔT Δfoot; Prefer FAIL if `outer_idle` |
| Eval | Fair Prefer FAIL Gate Q; full Q03 + continuous watch only on `ret_ok` both |
| Budget | ≤**4 h** wall **or** ≤**2e6** env steps (first hit); early stop on `ret_ok` both + ε clean |
| Dual-ckpt | approach=E7lock · retreat=T5D2 (`GATE_Q_T5D2_CKPT`) |
| New sha16 | Install **only** after Prefer FAIL fair set beats E7lock honestly; else keep `9ffaa1a21b607bf6` |
| H0 | Opt-in **post-SCORE diagnosis only** — never mid-wall |
| Escalation if Prefer FAIL | Dave **T5-D4** / **H2** named unlock **or** next genuinely different family — **not** residual twin; **not** auto H2; do not park Q without Dave |

---

## Controls stack note (for cospec)

- Train / run entry: new `scripts/learned_gate_e/train_t5d2_icp.py` (or documented T5D2 mode) — ICP outer loop (ΔT + Δfoot from capture error; CMP→ankle) + optional clear-only residual corrector  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5d2_*.zip`, outer/ICP logs, `t5d2_train.log`, `GATE_Q_T5D2_PROGRESS.md`)  
- Scorer: `scripts/score_gate_q.py` + `GATE_Q_T5D2_CKPT`; log outer F,T, **ICP ΔT Δfoot**, `outer_idle`, footholds, `planted_middle_s`, plant-cam suite, skate, cf, tip, apps  
- Optional H0 instrument (Hardware) for clearer PRE_GAP / ε — **post-SCORE only**, freeze-compatible; never mid-wall  
- Keep E7lock rollback `9ffaa1a21b607bf6` until Prefer-FAIL validated honest beat  
- Cite arXiv:1703.00477 in train/progress notes  
- **No train in this proposal turn** — wait AI no-veto · **Do NOT edit live plant XML** · **Do NOT install a new sha16**

---

## Baselines (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback | E7lock sha16 `9ffaa1a21b607bf6` until Prefer-FAIL beat |
| cancel | ×0.70 — no FREEZE / qvel0 / damp↑ / cancel↑ |
| Vision / plant invent / Path A / H2 | **off** (Dave chose T5-D2 not H2) |
| T5-D1 note (near-miss) | `T5D1_13` sha16 `45799fc897157a3a` — skate under · `ret_ok_both=true` · ε steal maxG 0.055 · outer CSF+ALIP logged |
| E7lock / approach | apps ~0.548/0.630 held on dual-ckpt approach bout |

---

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars + plant-cam ≤ε + apps held + tip≥8 + dest cam from clear Δ + outer ICP engaged (`outer_idle=false`, ΔT/Δfoot logged) → Controls continuous watch → AI Q03. New sha16 only after honest beat of E7lock.  
- **Prefer FAIL:** budget wall; `outer_idle` / residual-primary collapse; T5-D1 schedule twin without ICP feedback; Root B / ε steal; apps traded; soft bars; twin reopen; plant invent; H2 / mid-wall plant swap without Dave; FREEZE/pin/duty/damp↑/µ invent.  
- **Next after Prefer FAIL:** ask Dave **T5-D4** / **H2** named unlock **or** next genuinely different family — soft-pass off; **not** residual twin; **not** auto H2; do not park without Dave.

---

## Spin after AI no-veto

1. AI ships `docs/GATE_Q_AI_COSPEC_T5D2.md` (veto / no-veto / no-veto mirror)  
2. Controls implements T5-D2 under that cospec (bounded wall ≤4 h or ≤2e6 first-hit) — **not before**  
3. Fair Prefer FAIL eval vs E7lock (+ note vs T5D1_13 near-miss `45799fc897157a3a`)  
4. Same-turn Prefer FAIL score **or** `ret_ok` pack after continuous watch PASS  

**No train until no-veto.** HW/MFG freeze — no plant ask / no spend / vision off / H2 off. Soft-pass never. Live companion + walk plant md5s frozen. E7lock kept until honest beat.

**Controls recommendation:** AI **no-veto T5-D2** as written. Aligns `GATE_Q_AI_RESEARCH_T5D2_PREP.md` + Dave greenlit ICP Prefer FAIL (not H2) + T5-D1 SCORE escalation.
