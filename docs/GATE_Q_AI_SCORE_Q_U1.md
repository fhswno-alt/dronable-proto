# Gate Q AI score packet — Q-U1 / S1 CSF50-on-retreat Prefer FAIL (~15:46 BST)

**From:** Controls · Soft-pass **never** · Bars **KEPT** · `ai_can_lock=false` · No Q03

## Cospec

AI no veto: `docs/GATE_Q_AI_COSPEC_Q_U1.md`. Fair CSF50-on-retreat / capture-timed reverse F+T. ≠ R1–R5.

## Baseline R0 (flags OFF)

≡ E7lock: apps **0.548/0.630**; bout1 rcf **0.649**; skate **0.096/0.240** & **0.098/0.221**; mid **34.88/26.70**; CSF `n_steps=0`.

## Fair Prefer FAIL (12 probes)

All Prefer FAIL. No `ret_ok` both. Mechanism live (`n_steps≈78`, F/T logged).

**Near-miss T2** (T_nom=0.88, T_scale=1.15): apps held; cf **0.605/0.663**; skate p95~**0.26/0.28** + plant-cam ε steal.

Most Vx/F variants hit `dx_back_max=0.03` → F_cmd capped **−0.030**.

## Disposition

Prefer FAIL. Flags default OFF. E7lock best-known. Soft-pass off. Escalation: **S2 ALIP-TVR-SWING** (AI cospec first).

Artifacts: `previews/ainex_walk/iterate/GATE_Q_CONTROLS_NOTE.md` · `GATE_Q_A28_RETOK_DIAG.md` · `GATE_Q_TABLE.json` · `GATE_Q_U1_S1_*` logs.
