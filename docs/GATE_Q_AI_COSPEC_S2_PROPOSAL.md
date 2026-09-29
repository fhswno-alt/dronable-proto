# Gate Q Controls proposal — family S2 ALIP-TVR-SWING (~15:47 BST)

**To:** AI · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen

## Ask

Fair new family **S2 — ALIP-TVR-SWING** (mid-swing residual foothold). Escalation after **Q-U1 / S1 Prefer FAIL** (CSF50-on-retreat died; near-miss T2 apps+cf only). Distinct from R1 place, R2 damp-hold, R3 gap-cap, R4 dwell-force, R5 wait, and Q-U1 capture F+T.

Refs: Controls research `docs/GATE_Q_RETREAT_RESEARCH_UNSTICK.md` §5 runner-up; AI research `docs/GATE_Q_AI_RESEARCH_UNSTICK.md`; Bang/Sentis ALIP-MPC + residual Δu_fp (arXiv:2407.17683); Ahn/Sentis TVR lineage.

## Prior closed

| Family | Disposition |
|--------|-------------|
| R1–R4 | Prefer FAIL · structural PARK |
| R5 | AI veto (not new residual) |
| Q-U1 / S1 CSF50 | Prefer FAIL (~15:46) · flags OFF · E7lock best-known |

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Clear authority | Residual Δu_fp + track **only** while sole clear (FOOT-LIFT ≥2 cm above rest) |
| Planted | cancel×0.70 + vel-oppose only — **no** damp-hold / FREEZE / pin / qvel0 / planted soft-XY |
| Foothold prior | TVR or ALIP foothold for reverse Vx **or** remaining retreat Δ — not CSF50 F,T exchange (that was S1) |
| Mid-swing | 1–N replans of Δu_fp during clear swing; consecutive-action smoothness (Ker-style) |
| Real swing first | Emit a real swing target before any phase advance — **forbid** dwell-only force-kicks (R4 collapse) |
| ≠ R1 | Not residual-internal world place toward dest cam without ALIP/TVR velocity-reversal prior |
| Plant-cam ε | Prefer FAIL if any plant-window cam gain >**0.02** or cum >**0.05**/bout |
| Bars | skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften** |
| Apps | Hold ~0.548/0.630 or Prefer FAIL if traded |
| Tip | ≥8 hard; Prefer FAIL on tip death from foothold jump |
| Soft-pass | **off** |
| Baseline | E7lock · CLEAR_TRACK=1 · cancel×0.70 · **R\* OFF · GATE_Q_RET_CSF50=0** · md5 `59cc408eda07037a58f92ad27da045d6` · ckpt `9ffaa1a21b607bf6` |

## Knobs (env, default OFF — draft)

- `GATE_Q_RET_ALIP_TVR` — master (0=OFF)
- `GATE_Q_RET_ALIP_TVR_VX` — reverse sagittal command (m/s)
- `GATE_Q_RET_ALIP_TVR_N_REPLAN` — mid-swing replan count (1–N)
- `GATE_Q_RET_ALIP_TVR_SMOOTH` — consecutive Δu_fp smoothness weight
- `GATE_Q_RET_ALIP_TVR_DX_CAP` — optional |foothold Δ| cap (m)

## Logs (every score)

`fp_prior`, `du_fp`, `n_replan`, `fp_jump`, `cam@preSS`, `dxc_pre`, `planted_middle_s`, `cam_gain_in_gap`, plant-cam suite, PRE_GAP.

## Success / fail

- Progress: dest cam/dxc from **clear** SS under ALIP/TVR foothold residual; skate→bars; tip/cam/joint + apps held; plant-cam ≤ε.
- Prefer FAIL if fair set dies, Root B / plant steal, flush chatter, foothold-jump skate/tip, or apps traded.
- Ping full Q03 only on `ret_ok` both + Controls continuous watch PASS.
- Escalation if Prefer FAIL: **S3 REV-PHASE** → T4 Dave-greenlit reverse ckpt only.

## Hardware

Body fixed — no plant invent, no GEO/friction/STEP reopen.
