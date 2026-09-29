# Gate Q AI cospec — S2 / Q-U2 ALIP-TVR-SWING (~15:48 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen

## Veto?

**No veto — fair new family.** Aligns Controls research `docs/GATE_Q_RETREAT_RESEARCH_UNSTICK.md` § runner-up S2 / T1 and AI research `docs/GATE_Q_AI_RESEARCH_UNSTICK.md` Q-U2-class (reverse-native outer + clear-only residual). Controls proposal `docs/GATE_Q_AI_COSPEC_S2_PROPOSAL.md` accepted as-written on must-holds.

**≠ closed families:**
| Closed | Why S2 is different |
|--------|---------------------|
| R1 | World place toward dest cam — no ALIP/TVR velocity-reversal foothold prior |
| R2 | Clear-bursts + damp-hold planted |
| R3 | Hard-capped plant gaps |
| R4 | Forced SS / dwell force-kicks without planned swing |
| R5 | Voluntary wait (vetoed) |
| Q-U1 / S1 | CSF50 capture-timed reverse **F+T** exchange — not mid-swing Δu_fp |

## Family

On **retreat bouts only**: each swing, compute a **TVR or ALIP foothold prior** for reverse sagittal Vx **or** remaining retreat Δ. Residual outputs **Δu_fp** (and CLEAR_TRACK tracks) **only while sole clear**. Allow **1–N mid-swing replans** with consecutive-action smoothness. Planted = cancel×0.70 + vel-oppose only — **zero** planted soft-XY credit. Approach/compose stay locked **E7lock** package. **GATE_Q_RET_CSF50=0** (S1 flags OFF).

## Baseline (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Ckpt | frozen `9ffaa1a21b607bf6` (no new ckpt for S2) |
| CLEAR_TRACK_SWING | 1 |
| cancel | ×0.70 (do not reopen ↑) |
| R1–R5 / H / CSF50 | **OFF** |
| FREEZE / qvel0 / damp↑ / kd↑ / gap-planted | stay abandoned |

## Must-holds

1. Soft-pass **off**. Skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften**. Formal §5 ~70% stepped Δ still the honesty north star.
2. Clear authority only while sole clear (FOOT-LIFT ≥2 cm above rest). No planted soft-XY stride credit.
3. Foothold prior = TVR/ALIP for reverse Vx or remaining retreat Δ — **not** CSF50 F,T exchange (S1) and **not** dest-cam world place without that prior (R1).
4. Mid-swing: 1–N Δu_fp replans during clear; consecutive smoothness required. Prefer FAIL on foothold-jump skate/tip death.
5. Emit a **real swing target** before any phase advance — **forbid** dwell-only force-kicks (R4 collapse).
6. Plant-cam ε: Prefer FAIL if any plant-window cam gain >**0.02** or cum >**0.05**/bout.
7. tip≥8 · apps hold ~0.548/0.630 or Prefer FAIL if traded.
8. Logs every score: `fp_prior`, `du_fp`, `n_replan`, `fp_jump`, `cam@preSS`, `dxc_pre`, `planted_middle_s`, `cam_gain_in_gap`, plant-cam suite, PRE_GAP.
9. No plant invent · no Path A · no vision-in-walk · no HW/MFG plant ask.

## Knobs (env, default OFF — Controls owns names)

- `GATE_Q_RET_ALIP_TVR` — master (0=OFF)
- `GATE_Q_RET_ALIP_TVR_VX` — reverse sagittal command (m/s)
- `GATE_Q_RET_ALIP_TVR_N_REPLAN` — mid-swing replan count (1–N)
- `GATE_Q_RET_ALIP_TVR_SMOOTH` — consecutive Δu_fp smoothness weight
- `GATE_Q_RET_ALIP_TVR_DX_CAP` — optional |foothold Δ| cap (m)

## Success / Prefer FAIL

- **Progress:** dest cam/dxc from **clear** SS under ALIP/TVR foothold residual; skate→bars; tip/cam/joint + apps held; plant-cam ≤ε; PRE_GAP middle shrinks without ε steal.
- **Prefer FAIL** if fair set dies, Root B / plant steal, flush chatter, foothold-jump skate/tip, apps traded, or S2 collapses into R1 place / R2 hold / R4 chatter / S1 F+T rename.
- Ping AI full Q03 only on `ret_ok` both + Controls continuous watch PASS.

## Escalation (if Prefer FAIL)

**S3 REV-PHASE** → **T4** Dave-greenlit reverse ckpt only after that. Soft-pass off throughout. AI cospec each step before spin.

## Spin

Controls: fair skip-video Prefer FAIL off E7lock under this cospec. HW/MFG: freeze holds — no plant ask.
