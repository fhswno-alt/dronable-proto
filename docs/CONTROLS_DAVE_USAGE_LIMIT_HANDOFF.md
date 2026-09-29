# Controls — Dave solo handoff (usage limits)

**When:** Mon 28 Sep 2026 ~19:14 Europe/London (BST)  
**Owner:** Founding Controls Engineer  
**Purpose:** Keep Gate Q / walk-back progress honest if Grok Bot hits usage limits. Soft-pass **off**. Not a plant invent / Path A / bar soften.

**Google Doc:** https://docs.google.com/document/d/1AaYCRMV4s8L9FyJsbfRj576rcHqPCq1KdhbAjtMh9vU/edit

Live freeze (Hardware): `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`  
AI criteria: `docs/GATE_Q_AI_CRITERIA.md`  
Hardware twin handoff: `docs/HARDWARE_DAVE_USAGE_LIMIT_HANDOFF.md`  
MFG twin handoff: `docs/MFG_DAVE_USAGE_LIMIT_HANDOFF.md`

---

## Current Controls state (do not invent)

| Item | State |
|------|--------|
| Curriculum | **D–P TRUE** (sim). **Gate Q OPEN** — Prefer FAIL iterate |
| Soft-pass | **OFF** forever unless you explicitly say otherwise |
| Bars KEPT | skate mean≤0.08 / p95≤0.18 · clear_frac≥0.55 · tip≥8 |
| Best rollback ckpt | E7lock sha16 **`9ffaa1a21b607bf6`** (`ppo_gate_e_locked.zip`) — stay until Prefer FAIL honest beat |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| cancel | ×**0.70** planted — do not raise / FREEZE / qvel0 / damp↑ |
| Closed Prefer FAIL | R1–R5 · S1 CSF50 · S2 ALIP · S3 REV-PHASE · **T4** · **T5-A** · **T5-B** (near-misses OK; do not reopen twins on frozen E7lock) |
| Live now | **T5-C** BC teacher + multi-seed Prefer FAIL (`docs/GATE_Q_AI_COSPEC_T5C.md`) |
| Next if T5-C Prefer FAIL | **T5-D** only with **your** OK (plant/architecture) — do not auto-park, do not auto-ask Hardware |

### Near-miss ladder (honest progress)

| Family | Honest win | Still FAIL |
|--------|------------|------------|
| T5-A | bout0 skate under bars; cf ~0.90/0.89 | bout1 skate p95 |
| T5-B | bout1 skate under + ret_ok; cf ~0.86/0.85 | bout0 p95 0.183 + plant-cam ε |
| T5-C (live) | Attack: bout0 p95→≤0.18 + kill ε **without** trading bout1 | — |

Scores: `docs/GATE_Q_AI_SCORE_T4.md` · `SCORE_T5.md` · `SCORE_T5B.md` · (T5-C when done) `SCORE_T5C.md`

---

## What you can do alone (no bots)

### A. Watch T5-C without changing policy

```bash
cd /workspace/dronable-proto   # or your checkout of the same tree
tail -80 previews/ainex_walk/iterate/GATE_Q_T5C_PROGRESS.md
pgrep -af train_t5c_teacher || echo "train not running"
ls -lt previews/ainex_walk/iterate/learned_gate_e/ppo_t5c* 2>/dev/null | head
```

- Soft-pass still **off**. Do **not** install a new `CKPT_SHA16` yourself unless a fair Prefer FAIL set shows `ret_ok` **both** under bars and you understand the score packet.
- If train died mid-seed: note PID exit + last progress line; when bots return, ask Controls to resume under `GATE_Q_AI_COSPEC_T5C.md` (do not invent a new family).

### B. If T5-C finishes while bots are dark

1. Open `docs/GATE_Q_AI_SCORE_T5C.md` if present (or `T5C_DONE.json` under `learned_gate_e/`).
2. **ret_ok both under bars** → do **not** soft-lock alone. Leave continuous Q03 / AI lock for Controls+AI when limits reset. You may skim videos under `previews/ainex_walk/iterate/` but do not claim Gate Q LOCKED.
3. **Prefer FAIL** → Gate Q stays OPEN. Next lever is **T5-D** (you decide): keep freeze vs open a **named** plant/architecture talk with Hardware (read `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` first). Do **not** reopen R*/S1–S3/T5-A/B twins.

### C. Protect honesty (default)

1. Do not soften skate / clear_frac / tip bars.
2. Do not edit plant XML / friction / soft-XY to “help” Q.
3. Do not reopen closed twin families on frozen E7lock.
4. Do not raise cancel above ×0.70 or FREEZE planted joints.
5. HW/MFG freeze holds — no plant ask unless you intentionally open T5-D.

### D. Useful reads (no GPU required)

| Doc | Why |
|-----|-----|
| `docs/GATE_Q_AI_COSPEC_T5C.md` | Live rules for T5-C |
| `docs/GATE_Q_AI_RESEARCH_T5.md` | Why skate-primary / T5 ladder |
| `docs/GATE_Q_AI_SCORE_T5B.md` | Last Prefer FAIL near-miss |
| `docs/GATE_Q_AI_CRITERIA.md` | Gate Q pass criteria |
| `previews/ainex_walk/iterate/GATE_Q_CONTROLS_NOTE.md` | Controls diary |

### E. Vision Pro (later, off critical path)

When you want spatial preview: Controls can dump MuJoCo joint trajectories / walk+retreat clips into USD/USDZ with Hardware meshes. Do **not** block Gate Q on this.

---

## If limits hit mid Gate Q

1. Leave train running if the process is healthy — do not kill it to “save” anything.
2. Leave E7lock as rollback; leave plants frozen.
3. When bots reset, ping **Founding Controls Engineer** in DM with: (a) whether T5-C still running / finished, (b) path to SCORE_T5C or last PROGRESS lines, (c) your T5-D yes/no if Prefer FAIL, (d) any XML you touched (should be none).

---

## Do **not** do while waiting

- Soft-pass / bar soften / install sha16 from a single pretty seed  
- Reopen R*/S1–S3/T5-A/T5-B as env twins on frozen E7lock  
- Plant invent / Path A / PO / spend  
- Claim Gate Q LOCKED without Controls continuous watch + AI lock  
- Auto-open Hardware plant without reading the honesty note  

---

## One-line Controls policy

**Prefer FAIL, bars KEPT, E7lock until honest `ret_ok` both — keep walking back; do not fake the plant.**
