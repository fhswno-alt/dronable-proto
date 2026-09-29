# Controls post–Gate H stub — **partially superseded**

**AI named Gate I** — `docs/GATE_I_AI_CRITERIA.md`. Score via `scripts/score_gate_i.py` (I00 now; I01+ after panel hinge).

---

# Controls post–Gate H must-prove stub

**When:** Mon 28 Sep 2026 ~01:48 Europe/London (BST)  
**Role:** Founding Controls  
**Status:** **Controls proposal for AI** — **not** a lock, **not** a named curriculum gate letter.  
**Scope:** Sim only. **No spend / Path A / thaw / greenlight.**

Curriculum today: Gate **D HOLDS** · Gate **E UNLOCKED TRUE** · Gate **F LOCKED TRUE** · Gate **G LOCKED TRUE** · Gate **H LOCKED TRUE** (sim grasp + lever rotate; companion hinge only; **explicit no full door-open**). Curriculum docs do **not** yet name a Gate I. AI owns formal criteria; Controls drafts this stub so scoring can start when AI locks a criteria doc.

---

## 1. Goal (one line)

On the **Gate F/G/H companion plant**, after H grasp+rotate, prove a **limited door-panel swing** on the demo prop (e.g. panel hinge opens on the order of **≥10–20°**) while upright — still **not** latch release / UK handle height / full **90°** open product claim. Vision **off** walk; frozen Gate E ckpt; M145 untouched.

---

## 2. Frozen inherits (do not reopen)

| Item | Locked value |
|------|----------------|
| Walk / Gate E plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (**untouched**) |
| Companion (this prove only) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Gate E ckpt | `previews/ainex_walk/iterate/learned_gate_e/ppo_gate_e_best.zip` · sha16 **`9ffaa1a21b607bf6`** |
| Head command | **`head_tilt`** (not `neck_pitch`) |
| Walk obs | `vision_in_walk_obs=false` — proprio residual unchanged |
| Cheats | assist **OFF**, freeze **OFF**, `k_auth=1.0` |
| Gate H lock (context) | `previews/ainex_walk/iterate/GATE_H_AI_LOCK.md` — grasp+rotate; **no full door-open** |
| Freeze sheet | `docs/CONTROLS_FREEZE_STATUS_MONDAY.md` |

Do **not** rescore Gate E on the companion. Do **not** modify ckpt bytes or M145 contact/HX. Lever hinge (`door_lever_hinge`) stays as Hardware shipped for H — **panel swing is a separate DOF**.

---

## 3. Proposed must-prove rows (provisional tags)

**AI renames** these when locking criteria. Tags below are **Controls placeholders only** (Q00-style = “next after H”, **not** a curriculum letter / not Gate I).

| Prov. tag | Intent |
|-----------|--------|
| **Q00** (or AI: …) | **Retain H00/H01** — reach + grasp hold ≥1 s with close cmd; tip≥8; vision off |
| **Q01** (or AI: …) | **Retain H02/H03** — lever rotate ≥15° (and optional hold ≥1 s) on companion `door_lever_hinge`; tip≥8 |
| **Q02** (or AI: …) | **Limited panel swing** — after grasp+rotate, companion **door panel** hinge opens on the order of **≥10–20°** (AI freezes exact threshold); upright tip≥8; still **not** latch release / UK handle / full 90° product |
| **Q03** (optional) | **Hold panel swing** — sustain panel angle above Q02 threshold ≥1 s without tip fail; **no** full-open claim |

**Out of this stub until AI says otherwise:** latch release, full 90° door open, UK handle-height product, vision detector in walk, Pi/Orin NN-first.

*No metrics claimed as scored here — rows are unproven until AI criteria + Hardware panel hinge + Controls score. Thresholds above are **order-of-magnitude proposals only**; AI freezes numbers.*

---

## 4. Hard falsifiers (proposed)

- Vision feature injected into Gate E / walk residual obs  
- Tip &lt; 8 (or skate fail on any stepping row AI requires)  
- Plant hop / M145 edit / Gate E rescore on companion / GEO reopen  
- “Pass” via assist, freeze, or xfrc upright cheat  
- Claiming **full door-open** / latch release / UK handle height without AI criteria  
- Cartoon panel motion that is **not** a real companion hinge DOF (or reusing lever hinge as if it were panel swing)  
- Soft-pass / invented metrics without AI formal criteria  

---

## 5. Explicit non-claims

- **Not** a locked gate — AI must publish criteria before Controls claims PASS/FAIL  
- **Not** Gate I (letter unused until AI names it)  
- **Not** full 90° door-open / latch release / UK door-handle height product  
- **Not** Pi / Orin / NN-first kit  
- **Not** Path A / spend / fab unlock / thaw / greenlight  
- **Not** GEO / sole / STEP reopen  

---

## 6. What Controls needs

| Owner | Need |
|-------|------|
| **AI** | Formal criteria doc under `docs/` (path TBD — e.g. next after `docs/GATE_H_AI_CRITERIA.md`). Name the gate letter (or “no letter”) and freeze pass thresholds for panel angle, tip/skate, hold time. |
| **Hardware** | Add a **door-panel hinge** on the companion plant **separate from** `door_lever_hinge` (honest: panel swing may need its own DOF; lever hinge alone is **not** panel open). Keep bit-2 hand–lever contact; M145 untouched. Ship a short Hardware note (path TBD). |
| **Controls** | Idle until that criteria doc + panel hinge exist; then score provisional Q00–Q03 on companion + frozen ckpt. |

---

## 7. Refs

- `docs/CONTROLS_FREEZE_STATUS_MONDAY.md` — D–H locks; idle until next AI cospec  
- `docs/GATE_H_AI_CRITERIA.md` · `docs/GATE_H_HARDWARE_LEVER_HINGE.md`  
- `previews/ainex_walk/iterate/GATE_H_AI_LOCK.md` · `GATE_H_CONTROLS_NOTE.md` · `GATE_H_TABLE.json`  
- `previews/ainex_walk/iterate/GATE_G_AI_LOCK.md` · Gate F/G notes under same iterate dir  
- Walk plant: `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
- Companion: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`  

**Honest:** may need Hardware to add `door_panel` hinge separate from lever hinge before Q02/Q03 are scoreable. Controls does **not** invent metrics or claim PASS here.
