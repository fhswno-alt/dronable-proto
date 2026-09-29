# AI — Dave solo handoff (usage limits)

**When:** Mon 28 Sep 2026 ~19:15 Europe/London (BST)  
**Owner:** Founding AI Scientist  
**Purpose:** Keep Gate Q walk-back honest if Grok Bot hits usage limits. Soft-pass **off**. Not a plant invent / Path A / bar soften / soft lock.

**Google Doc:** https://docs.google.com/document/d/1CcOdvn0Rjez_LrU1JdEFL7UWZ4F7raH0M65CF4dkHmA/edit

Controls twin: `docs/CONTROLS_DAVE_USAGE_LIMIT_HANDOFF.md`  
Hardware twin: `docs/HARDWARE_DAVE_USAGE_LIMIT_HANDOFF.md`  
MFG twin: `docs/MFG_DAVE_USAGE_LIMIT_HANDOFF.md`  
Criteria (KEPT): `docs/GATE_Q_AI_CRITERIA.md`  
Sync sheet: `docs/AI_GATE_Q_STATUS_MONDAY.md`

---

## Current AI state (do not invent)

| Item | State |
|------|--------|
| Curriculum | **D–P LOCKED TRUE** (sim). **Gate Q OPEN** — Prefer FAIL iterate |
| Soft-pass | **OFF** forever unless you explicitly say otherwise |
| Bars KEPT | skate mean≤0.08 / p95≤0.18 · clear_frac≥0.55 · tip≥8 · formal §5 ~70% stepped Δ |
| Best rollback ckpt | E7lock sha16 **`9ffaa1a21b607bf6`** — stay until Prefer FAIL honest beat |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| cancel | ×**0.70** planted |
| Closed Prefer FAIL | R1–R5 · S1 · S2 · S3 · T4 · T5-A · T5-B (near-misses OK; do not reopen twins on frozen E7lock) |
| Live now | **T5-C** BC teacher + multi-seed (`docs/GATE_Q_AI_COSPEC_T5C.md`) |
| Next if T5-C Prefer FAIL | **T5-D** only with **your** OK (architecture / plant) — do not auto-park Q, do not soft-pass |
| AI lock | Only after `ret_ok` **both** + Controls continuous watch PASS + AI independent stills/Q03 — never skip-video alone |

### Near-miss ladder (honest)

| Family | Cospec | Honest win | Still FAIL |
|--------|--------|------------|------------|
| T4 | `GATE_Q_AI_COSPEC_T4.md` | cf up; mid shortened; bout0 ret_ok once | skate p95 |
| T5-A | `GATE_Q_AI_COSPEC_T5.md` | cf ~0.90/0.89; bout0 skate under bars | bout1 p95 |
| T5-B | `GATE_Q_AI_COSPEC_T5B.md` | bout1 skate under + ret_ok; cf high | bout0 p95 0.183 + ε |
| T5-C (live) | `GATE_Q_AI_COSPEC_T5C.md` | Attack bout0 p95→≤0.18 + kill ε without trading bout1 | — |

Scores: `GATE_Q_AI_SCORE_T4.md` · `SCORE_T5.md` · `SCORE_T5B.md` · (when done) `SCORE_T5C.md`

---

## What you can do alone (no bots)

### A. Watch T5-C without changing policy

```bash
cd /workspace/dronable-proto   # or your checkout
tail -80 previews/ainex_walk/iterate/GATE_Q_T5C_PROGRESS.md 2>/dev/null
ls -lt docs/GATE_Q_AI_SCORE_T5C.md 2>/dev/null
pgrep -af train_t5c || echo "train not running"
ls -lt previews/ainex_walk/iterate/learned_gate_e/ppo_t5c* 2>/dev/null | head
```

- Soft-pass still **off**. Do **not** install a new `CKPT_SHA16` yourself unless fair Prefer FAIL shows `ret_ok` **both** under bars **and** you have Controls continuous watch PASS.
- Do **not** claim Gate Q LOCKED yourself — that is AI + Controls after Q03 stills/watch.

### B. If T5-C finishes while bots are dark

1. Read `docs/GATE_Q_AI_SCORE_T5C.md` if present (or `T5C_DONE.json` under `learned_gate_e/`).
2. **`ret_ok` both under bars + ε clean + apps held** → leave continuous Q03 / AI lock for bots. You may skim videos; do **not** soft-lock.
3. **Prefer FAIL all seeds** → Gate Q stays OPEN. Next lever is **T5-D** — wait for bots or decide yourself: park vs plant/architecture conversation with Hardware (spend risk). Do **not** reopen R*/S1–S3/T5-A/B twins. Do **not** soften skate bars.
4. **Near-miss** (one bout ok) → same as Prefer FAIL for lock purposes; E7lock stays installed.

### C. Protect honesty (always)

- Soft-pass **forbidden**.
- Skate / clear_frac bars **not** softened.
- Plant md5s **frozen** unless you open a real geom ask with Hardware (T5-D).
- No Path A / spend / vision-in-walk as a walk-back cheat.
- Prefer FAIL > cosmetic metrics.

### D. Optional Vision Pro (later only)

Off critical path. When bots return: AI can package stills/FOV + Controls trajectories into a spatial pack with Hardware meshes. Do not block Gate Q for this.

---

## What to tell bots when limits reset

One short ping is enough:

1. T5-C status: spinning / Prefer FAIL / `ret_ok` both (point at `SCORE_T5C.md` if present).
2. Whether you want **T5-D** (plant/architecture) or **park** if Prefer FAIL.
3. Soft-pass still off / bars KEPT / freeze holds — unless you change that.

---

## Explicit non-claims

Not full door open / latch / 90° / walk-through / UK / Pi / Orin / spend / hinge-range bump / room-walk / investor walk script. Soft-pass forbidden. Gate Q not locked.

---

## Owner split reminder

| Role | Owns |
|------|------|
| **AI (me)** | Criteria · Prefer FAIL score · cospec veto/no-veto · lock after continuous watch |
| **Controls** | Train / eval / continuous watch · score packets |
| **Hardware** | Plant / geom / freeze md5s |
| **MFG** | Spend / PO / fab — idle until real buy ask |

If only one of us is back first: Controls can Prefer FAIL-score T5-C; AI must still cospec T5-D and lock path. Do not skip AI cospec before a new family train.
