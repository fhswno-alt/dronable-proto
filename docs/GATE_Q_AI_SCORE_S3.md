# Gate Q AI score packet — S3 / Q-U2b REV-PHASE Prefer FAIL (~16:04 BST)

**From:** Controls · Soft-pass **never** · Bars **KEPT** · `ai_can_lock=false` · No Q03

## Cospec

AI no veto: `docs/GATE_Q_AI_COSPEC_S3.md`. Fair reverse phase-polarity / hip / swing encoding (clear-only). ≠ R1–R5 / S1 CSF50 F+T / S2 ALIP Δu_fp. `GATE_Q_RET_CSF50=0` · `GATE_Q_RET_ALIP_TVR=0` throughout.

## Baseline R0 (flags OFF)

≡ E7lock: apps **0.548/0.630**; bout0 rcf **0.468** skate **0.096/0.240** mid **34.88**; bout1 rcf **0.649** skate **0.098/0.221** mid **26.70**; REV `n_ticks=0`.

## Fair Prefer FAIL (12 probes)

All Prefer FAIL. No `ret_ok` both. Mechanism live (`n_ticks` ~2.5k; `phi_polarity`/`hip_sign`/`swing_enc` logged; CSF/ALIP n=0). Soft-pass NEVER.

**Near-miss S3** (PHI0.5 HIP−1 SW1): apps 0.548/0.655; bout0 cf **0.380** tip 23.2; bout1 cf **0.425** tip 14.0 — plant-cam ε steal + skate fail; plant≫clear persists.

**Near-miss H2** (HIP−1.5): apps held; bout0 cf **0.283** tip 16.5 mid 3.8; bout1 tip **6.8** + cf collapse.

Default / PHI / SWING mixes: ε steal and/or cf≪0.55 and/or cosmetic Root B / apps trade. Never skate≤0.08/0.18 while holding E7lock apps+bout1 package without ε.

## Disposition

Prefer FAIL. Flags default OFF. E7lock best-known. Soft-pass off. Escalation: **T4 Dave-greenlit reverse ckpt only**. Do not reopen R* / CSF50 / ALIP / S2 / S3.

Artifacts: `previews/ainex_walk/iterate/GATE_Q_CONTROLS_NOTE.md` · `GATE_Q_A28_RETOK_DIAG.md` · `GATE_Q_TABLE.json` · `GATE_Q_S3_*` logs · `GATE_Q_VIDEO_CONFLICT.md`.

## AI disposition (~16:04 BST)

**Agrees Prefer FAIL.** Soft-pass never. Bars KEPT. `ai_can_lock=false`. No Q03. Cospec S3 held; do not reopen S1–S3 / R*. Next only with Dave greenlight: T4 reverse ckpt (AI will cospec then).
