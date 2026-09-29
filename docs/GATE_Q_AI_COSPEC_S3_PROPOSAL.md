# Gate Q Controls proposal — family S3 REV-PHASE (~15:55 BST)

**To:** AI · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen

## Ask

Fair new family **S3 — REV-PHASE** (reverse phase-polarity residual). Escalation after **S2 Prefer FAIL** (ALIP-TVR mid-swing Δu_fp died; near-miss N2 apps+bout0 cf only). Distinct from R1–R5, Q-U1 CSF F+T, and S2 ALIP Δu_fp.

Refs: Controls research `docs/GATE_Q_RETREAT_RESEARCH_UNSTICK.md` §5 third / T7; AI research unstick; S2 score `docs/GATE_Q_AI_SCORE_S2.md`.

## Prior closed

| Family | Disposition |
|--------|-------------|
| R1–R4 | Prefer FAIL · structural PARK |
| R5 | AI veto |
| Q-U1 / S1 CSF50 | Prefer FAIL · flags OFF |
| S2 / Q-U2 ALIP-TVR | Prefer FAIL (~15:54) · flags OFF · E7lock best-known |

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Clear authority | Residual/hip track **only** while sole clear (FOOT-LIFT ≥2 cm above rest) |
| Planted | cancel×0.70 + vel-oppose only — **no** damp-hold / FREEZE / pin / qvel0 / planted soft-XY |
| Mechanism | Invert gait **phase clock** and/or hip-pitch / swing-leg encoding for retreat so clear strides are **reverse-native** (stance/swing roles + residual action signs match walking *away* from door) |
| ≠ S1 | Not CSF50 capture F+T exchange |
| ≠ S2 | Not ALIP/TVR foothold prior + mid-swing Δu_fp replan |
| ≠ R1–R4 | No world place / damp-hold / gap-cap / dwell force-kick |
| Cosmetic FAIL | Prefer FAIL if polarity flip is cosmetic (still plant≫clear / Root B) |
| Plant-cam ε | Prefer FAIL if any plant-window cam gain >**0.02** or cum >**0.05**/bout |
| Bars | skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften** |
| Apps | Hold ~0.548/0.630 or Prefer FAIL if traded |
| Tip | ≥8 hard |
| Soft-pass | **off** |
| Baseline | E7lock · CLEAR_TRACK=1 · cancel×0.70 · **R\* OFF · CSF50=0 · ALIP_TVR=0** · md5 `59cc408eda07037a58f92ad27da045d6` · ckpt `9ffaa1a21b607bf6` |

## Knobs (env, default OFF — draft)

- `GATE_Q_RET_REV_PHASE` — master (0=OFF)
- `GATE_Q_RET_REV_PHASE_PHI` — phase-clock invert / offset (rad or fraction)
- `GATE_Q_RET_REV_PHASE_HIP` — hip-pitch sign / bias scale
- `GATE_Q_RET_REV_PHASE_SWING` — swing-leg encoding flip strength

## Logs (every score)

`phi_polarity`, `hip_sign`, `swing_enc`, `cam@preSS`, `dxc_pre`, `planted_middle_s`, `cam_gain_in_gap`, plant-cam suite, PRE_GAP.

## Success / fail

- Progress: dest cam/dxc from **clear** reverse-native SS; skate→bars; tip/cam/joint + apps held; plant-cam ≤ε; plant≫clear shrinks.
- Prefer FAIL if fair set dies, Root B / plant steal, cosmetic flip, flush chatter, tip death, or apps traded.
- Ping full Q03 only on `ret_ok` both + Controls continuous watch PASS.
- Escalation if Prefer FAIL: **T4** Dave-greenlit reverse ckpt only (AI cospec + Dave OK). Soft-pass off.

## Hardware

Body fixed — no plant invent, no GEO/friction/STEP reopen.
