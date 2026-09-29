# Gate Q AI cospec — T5-D2 ICP-ΔT+ΔFOOT (~00:42 BST Tue 29 Sep)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** GREENLIT T5-D2 ICP Prefer FAIL (not H2) ~00:42 BST  
**Prep:** `docs/GATE_Q_AI_RESEARCH_T5D2_PREP.md` · Prior SCORE: `docs/GATE_Q_AI_SCORE_T5D.md` · Prior cospec: `docs/GATE_Q_AI_COSPEC_T5D.md`

## Veto?

**No veto — fair new family.** Dave greenlit. Aligns prep + T5-D1 Prefer FAIL wall (skate + `ret_ok_both` cleared; **plant-cam ε / Root B** remains — maxG 0.055).

**≠ closed:** R*/S1–S3 · T5-A/B/C residual twins · **T5-D1 twin** (CSF+ALIP schedule primary without online ICP ΔT+Δfoot feedback) · H2 / µ / softsole / foot-lift invent.

## Why T5-D2 (not twin)

T5-D1 proved outer PRIMARY can clear skate + `ret_ok` both under bars — Prefer FAIL solely on **plant-cam ε**. D1 outer was open-loop/schedule CSF+ALIP (F,T logged; idle=false) without **online ICP error → step timing + footstep location** adjust. Atlas IHMC-class (arXiv:1703.00477) recovers by speeding exchange and expanding base of support from Instantaneous Capture Point error — that is the D2 delta.

## Family

**T5-D2** — tag `T5D2` / `ICP-ΔT+ΔFOOT`

1. **ICP / capture feedback OUTER** (retreat bout only): Estimate ICP (or CoM-proxy capture point) error online. Adjust **support-exchange time ΔT** (swing speed-up / early commit when error is in direction of motion) **and** **next foothold Δfoot** (expand BoS / sagittal reverse placement) from that error. CMP→ankle strategy OK under clear/stance rules below. Cite IHMC Atlas step timing+location.
2. **May keep T5-D1 CSF+ALIP schedule as prior** — D2 **adds** online ICP→ΔT+Δfoot feedback. Prefer FAIL if ICP loop is idle and run collapses to D1 schedule twin or residual-primary.
3. **Residual = clear corrector only** (optional warm-start from T5D1_13 near-miss `45799fc897157a3a` or T5C_s43_BC) — authority only while sole clear ≥2 cm; TDVM/SLR late-swing OK. Prefer FAIL if residual owns polarity.
4. **Planted:** cancel×**0.70** + vel-oppose only — **zero** planted soft-XY. Soft-pass **never**.
5. **Dual-ckpt:** approach=**E7lock** · retreat=**T5D2** via `GATE_Q_T5D2_CKPT`.

**Attack focus:** Kill **plant-cam ε / Root B** (T5-D1 wall: maxG 0.055; PRE_GAP cam_gain) while **holding** skate ≤0.08/0.18 both, `ret_ok` both, clear_frac≥0.55, tip≥8, apps ~0.548/0.630.

**≠ R4:** ΔT from **ICP/capture error direction**, not plant-dwell flush kick / forced SS cadence.

### In-family opts (KEPT)

| Opt | Guard |
|-----|-------|
| ICP_LOOP | Log ICP error, ΔT, Δfoot every score; Prefer FAIL if idle |
| CMP_ANK | CMP→ankle under stance honesty — not planted soft-XY |
| TDVM/SLR | Late-swing tangential → ~0; clear-only |
| H0 | Optional post-SCORE ε logging on Hardware fork — **not** mid-wall live md5 swap |
| Scorer | Stricter plant-cam ε / Root B — honesty louder |

### Explicitly OUT

R*/S1–S3/T5-A/B/C/T5-D1-as-twin · residual-primary reshape · µ/softsole/soft-XY/foot-lift invent/planform/cancel↑/FREEZE · **H2** (Dave said not H2) · Path A / vision-in-walk / plant invent.

## Baseline (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` (live freeze) |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback | E7lock `9ffaa1a21b607bf6` until Prefer-FAIL beat |
| T5-D1 near-miss note | T5D1_13 `45799fc897157a3a` — skate under · ret_ok_both · ε Prefer FAIL |
| cancel | ×0.70 — no FREEZE / qvel0 / damp↑ / cancel↑ |
| H2 / Path A / vision | **OFF** |

## Must-holds

1. Soft-pass **off**. Skate ≤0.08/0.18 · clear_frac≥0.55 · tip≥8 — **no soften**.
2. ICP loop must be active on retreat — Prefer FAIL if outer_idle or residual-primary collapse.
3. Clear authority only while sole clear ≥2 cm.
4. Plant-cam ε: Prefer FAIL if plant-window cam >**0.02** or cum >**0.05**/bout — **primary attack target** (beat T5D1_13 maxG 0.055).
5. Root B: Prefer FAIL if dest cam climb is plant≫clear.
6. Apps hold ~0.548/0.630 or Prefer FAIL.
7. Hold T5-D1 locomotor wins where possible: skate under + `ret_ok` both — Prefer FAIL if traded for ε cosmetics.
8. Logs every score: `ckpt_id`, ICP error, ΔT, Δfoot, outer F/T/footholds, skate, cf, tip, apps, plant-cam suite, `cam@preSS`, `dxc_pre`, `planted_middle_s`, PRE_GAP cam_gain.
9. Budget ≤**4 h** wall **or** ≤**2e6** steps (first hit); early stop on `ret_ok` both **and** ε clean.
10. New sha16 **only** after Prefer FAIL fair set beats E7lock honestly (note vs T5D1_13).
11. Full Q03 + continuous watch **only** on `ret_ok` both **and** ε package clean.
12. Escalation if Prefer FAIL: ask Dave (named H2 / T5-D4 / park / tool) — **not** residual twin; **not** auto H2; do not park Q without Dave.

## Spin

Controls Prefer FAIL implements after this no-veto (bounded wall). Fair Prefer FAIL vs E7lock (+ note vs T5D1_13). Same-turn Prefer FAIL score **or** `ret_ok`+ε pack after continuous watch PASS. HW/MFG freeze — no plant ask / no spend / H2 OUT.

**AI disposition:** **no-veto T5-D2** as written. Controls may spin now.
