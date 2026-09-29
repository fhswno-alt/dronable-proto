# Gate Q Controls proposal — family T5-D1 Prefer FAIL (reverse-native outer PRIMARY)

**When:** Mon 28 Sep 2026 ~21:46 Europe/London (BST)  
**To:** AI Scientist · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** T5-D hybrid AGREED on voice (Controls+AI+HW+MFG) — research → one Prefer FAIL cospec; plants frozen default  
**Refs:** `docs/GATE_Q_AI_RESEARCH_T5D.md` · `docs/GATE_Q_T5D_AGREED_BREAKOUT_PLAN.md` · `docs/GATE_Q_AI_SCORE_T5C.md` · `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` · `docs/GATE_Q_HARDWARE_T5D_PLANT_LEVERS.md` · `docs/GATE_Q_RETREAT_RESEARCH_T5.md`  
**Ask:** Formal **T5-D1** cospec (AI → `docs/GATE_Q_AI_COSPEC_T5D.md` veto/no-veto). Soft-pass never. **No train until AI no-veto.**

---

## Why T5-D1 (not twin, not park, not plant invent)

| Closed | Disposition | Lesson |
|--------|-------------|--------|
| R1–R5 · S1 CSF50 · S2 ALIP · S3 REV-PHASE | Prefer FAIL on frozen E7lock | Scheduler / capture twins ≠ reverse-native dynamics |
| T4 · **T5-A** · **T5-B** · **T5-C** | Prefer FAIL near-miss ladder | Residual (or BC→residual) still owns gait polarity → skate/ε wall before `ret_ok` both |
| T5-C best | s43_BC skate under both bars, rcf~0.82/0.79, ret_ok=[T,F]; s47_03 bout0 ret_ok / bout1 skate+ε | Clear_frac moves; **both-bout package + ε** does not |

BD/Atlas/Cassie public story: **reverse-native template / ALIP-MPC / capture outer** + online stabilize — not harder residual on a forward/near-E7lock polarity (`GATE_Q_AI_RESEARCH_T5D.md`).

**≠ T5-A/B/C twin:** Those kept residual (or teacher→residual) as **primary** gait authority.  
**T5-D1:** Outer **owns** F,T + footstep targets for retreat; residual is **demoted** to clear mid-swing corrector + late-swing TDVM/SLR only.

---

## Family (proposed)

**Name:** `T5-D1 OUTER-PRIMARY+CLEAR-CORRECTOR` (tag `T5D1`)  
**What (retreat bout only):**

1. **Reverse-native outer PRIMARY** — Placo and/or CSF and/or ALIP-MPC owns support exchange timing **F,T** and next footstep targets from commanded −Vx / capture locus. Prefer Placo `walk_max_dx_backward` ≪ forward + mid-gait `replan_supports` where already in kit stack; CSF capture-step exchange from CoM + −Vx as co-primary if Placo wiring incomplete.
2. **Residual = corrector only** — new weights (optional bootstrap from best T5-C near-miss zip for mid-swing init **only**). Authority **only while sole clear** (FOOT-LIFT ≥2 cm): track clear mid-swing targets + late-swing TD velocity match / swing-leg retraction (TDVM/SLR).
3. **Planted windows:** cancel×**0.70** + vel-oppose only — **zero** planted soft-XY / stride credit. Soft-pass **never**.
4. **Approach bout:** keep dual-ckpt approach=**E7lock** (apps hold). Retreat=**T5D1** via `GATE_Q_T5D1_CKPT` (or documented alias).

**Attack focus:** Bank destination retreat cam from **clear Δ** (kill Root B / plant-cam ε) while holding skate mean≤0.08 / p95≤0.18 **both** bouts, clear_frac≥0.55, tip≥8, apps ~0.548/0.630.

**Bootstrap (optional, corrector warm-start only — outer is new):**
1. T5C_s43_BC near-miss sha16 `5e0c0726953840ed` (skate under both; ret_ok=[T,F]) if zip present
2. else T5C_s47_03 / T5B_02 / E7lock — **log** that residual is corrector-only; Prefer FAIL if train collapses to residual-primary twin

### In-family opts

| Opt | Idea | Guard |
|-----|------|-------|
| OUTER | Placo reverse schedule and/or CSF capture-step and/or ALIP-MPC foot placement as **retreat primary** | Outer must log F,T, footholds; Prefer FAIL if outer idle and residual owns polarity |
| TDVM/SLR | Late-swing tangential → ~0 / slight swing-back at TD | Clear-only; Prefer FAIL if applied planted |
| H0 | Optional Hardware instrument sites (contype 0) for ε / clear dwell logging | Diagnosis only — no physics change |
| — | Stricter plant-cam ε / Root B scoring in scorer (Controls) | Honesty louder — not bar soften |

### Explicitly OUT

| Out | Why |
|-----|-----|
| R* / S1–S3 / T5-A/B/C twin flags on frozen E7lock | Closed Prefer FAIL |
| Residual-primary reward / BC twin (T5-A/B/C reshape) | Same wall |
| µ bump / softsole / soft-XY / foot-lift invent / planform resize / cancel↑ / FREEZE | Soft-pass by another name |
| H2 contact-height XML edit | Needs **Dave named unlock** + separate AI Prefer FAIL A/B — **not** this cospec |
| Path A spend / vision-in-walk / plant invent | Dave only |

---

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Soft-pass | **OFF** — never |
| Bars | skate ≤**0.08/0.18** · clear_frac≥**0.55** · tip≥**8** — **KEPT** |
| Planted | cancel×**0.70** + vel-oppose only |
| Outer primary | Retreat F,T + footholds from reverse-native outer; residual **cannot** be sole gait authority |
| Clear authority | Residual / stride credit **only** while sole clear ≥2 cm |
| Plant-cam ε | Prefer FAIL if plant-window cam >**0.02** or cum >**0.05**/bout |
| Root B | Prefer FAIL if dest cam climb is plant≫clear (same honesty as SCORE_T5C) |
| Apps | Hold ~**0.548/0.630** on approach or Prefer FAIL |
| Companion / walk plant | md5s `59cc408eda07037a58f92ad27da045d6` / `fc94709c84f5598d4474ecfc4bb41fdc` — **no XML edit** |
| Eval | Fair Prefer FAIL Gate Q; full Q03 + continuous watch only on `ret_ok` both |
| Budget | ≤**4 h** wall **or** ≤**2e6** env steps (first hit); early stop on `ret_ok` both |
| Dual-ckpt | approach=E7lock · retreat=T5D1 |
| New sha16 | Install **only** after Prefer FAIL fair set beats E7lock honestly; else keep `9ffaa1a21b607bf6` |
| Escalation if Prefer FAIL | **T5-D2** (ICP step timing+location) or Dave **T5-D4** named plant ask — **not** residual twin; **not** auto H2 |

---

## Controls stack note (for cospec)

- Train / run entry: new `scripts/learned_gate_e/train_t5d1_outer.py` (or documented T5D1 mode) — outer loop + clear-only residual corrector  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5d1_*.zip`, outer logs, `t5d1_train.log`, `GATE_Q_T5D1_PROGRESS.md`)  
- Scorer: `scripts/score_gate_q.py` + `GATE_Q_T5D1_CKPT`; log outer F,T, footholds, `planted_middle_s`, plant-cam suite, skate, cf, tip, apps  
- Optional H0 instrument (Hardware) for clearer PRE_GAP / ε — freeze-compatible  
- Keep E7lock rollback until Prefer-FAIL validated  
- **No train in this proposal turn** — wait AI no-veto

---

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars + plant-cam ≤ε + apps held + tip≥8 + dest cam from clear Δ → Controls continuous watch → AI Q03.  
- **Prefer FAIL:** budget wall; outer idle / residual-primary collapse; Root B / ε steal; apps traded; soft bars; twin reopen; plant invent; H2 without Dave.  
- **Next after Prefer FAIL:** T5-D2 ICP outer **or** ask Dave T5-D4 / H2 named unlock — soft-pass off; do not park without Dave.

---

## Spin after AI no-veto

1. AI ships `docs/GATE_Q_AI_COSPEC_T5D.md` (veto / no-veto)  
2. Controls implements T5-D1 under that cospec (bounded wall) — **not before**  
3. Fair Prefer FAIL eval vs E7lock (+ note vs T5C_s43_BC / T5B_02)  
4. Same-turn Prefer FAIL score **or** `ret_ok` pack after continuous watch PASS  

**No train until no-veto.** HW/MFG freeze — no plant ask / no spend / vision off. Soft-pass never.

**Controls recommendation:** AI **no-veto T5-D1** as written. Aligns `GATE_Q_AI_RESEARCH_T5D.md` §5 + voice AGREED hybrid.
