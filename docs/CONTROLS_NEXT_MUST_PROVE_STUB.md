# Controls next must-prove stub (post–Gate F)

**When:** Mon 28 Sep 2026 (Europe/London)  
**Role:** Founding Controls  
**Status:** **Controls proposal for AI** — **not** a lock, **not** a named curriculum gate letter.  
**Scope:** Sim only. **No spend / Path A / thaw / greenlight.**

Curriculum today: Gate **D HOLDS** · Gate **E UNLOCKED TRUE** (frozen PPO) · Gate **F LOCKED TRUE** (lever look+approach). Curriculum docs do **not** yet name a Gate G. AI owns formal criteria; Controls drafts this stub so scoring can start when AI locks a criteria doc.

---

## 1. Goal (one line)

On the **Gate F companion plant**, with vision **off** the walk loop, prove **hand/wrist reach to the lever** (kinematic + optional light contact) after F00/F01 look+approach — **no** grasp-force / door-open claim.

---

## 2. Frozen inherits (do not reopen)

| Item | Locked value |
|------|----------------|
| Walk / Gate E plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| Companion (this prove only) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Gate E ckpt | `previews/ainex_walk/iterate/learned_gate_e/ppo_gate_e_best.zip` · sha16 **`9ffaa1a21b607bf6`** |
| Head command | **`head_tilt`** (not `neck_pitch`) |
| Walk obs | `vision_in_walk_obs=false` — proprio residual unchanged |
| Cheats | assist **OFF**, freeze **OFF**, `k_auth=1.0` |

Do **not** rescore Gate E on the companion. Do **not** modify ckpt bytes or M145 contact/HX.

---

## 3. Proposed must-prove rows (provisional tags)

**AI renames** these when locking criteria. Tags below are **Controls placeholders only** (G00-style = “next after F”, not a curriculum letter).

| Prov. tag | Intent |
|-----------|--------|
| **N00** (or AI: …) | **Retain F00** — stand @ ~0.4 m, `head_tilt` ≈ −16°, lever in ego ≥2 s, tip≥8 |
| **N01** (or AI: …) | **Retain F01** — slow approach w/ frozen Gate E ckpt; look-down held; tip≥8; skate mean ≤0.08 if stepping; dx≥0.15 or cam→door ∈[0.35,0.45] |
| **N02** (or AI: …) | **Hands reach** — from F00/F01 pose (or F02 −19° look-down): gripper / wrist site within **≤5 cm** of `door_lever` geom center (or Hardware-defined contact site); hand/wrist Z in lever band **0.25–0.30 m** AFF (kinematic OK) |
| **N03** (optional) | **Light contact** — brief hand–lever geom contact or ≤few mm depression on a *contactable* lever geom **without** tip/skate fail; **no** grasp wrench / door swing claim |

**Out of this stub until AI says otherwise:** grasp force closure, door panel open, UK handle height, vision detector in walk.

*No metrics claimed here — rows are unproven until AI criteria + Controls score.*

---

## 4. Hard falsifiers (proposed)

- Vision feature injected into Gate E / walk residual obs  
- Tip &lt; 8 or skate mean &gt; 0.08 on approach/contact rows that step  
- Wrong head joint (`neck_pitch`) or cartoon cam not on `head_tilt_link`  
- Plant hop / Gate E rescore on companion / GEO reopen for reach  
- “Pass” via assist, freeze, or xfrc upright cheat  
- Claiming door open / force grasp without AI criteria  

---

## 5. Explicit non-claims

- **Not** Pi / Orin / NN-first kit  
- **Not** UK door-handle height product claim  
- **Not** force door open / latch release  
- **Not** GEO / sole / STEP reopen  
- **Not** Path A / spend / fab unlock  
- **Not** a locked gate — AI must publish criteria before Controls claims PASS/FAIL  

---

## 6. What Controls needs

| Owner | Need |
|-------|------|
| **AI** | Formal criteria doc under `docs/` (path TBD by AI — e.g. next after `docs/GATE_F_AI_CRITERIA.md`). Name the gate letter (or “no letter”) and freeze pass thresholds for reach distance, contact, tip/skate. |
| **Hardware** | Confirm whether `door_lever` stays visual-only (`contype=0`) or needs a **contactable** lever geom / site for N03; cam/lever honesty remains companion-only. |
| **Controls** | Idle until that criteria doc exists; then score provisional rows on companion plant + frozen ckpt. |

---

## 7. Refs

- `docs/GATE_F_AI_CRITERIA.md`  
- `previews/ainex_walk/iterate/GATE_F_AI_LOCK.md`  
- `docs/CONTROLS_FREEZE_STATUS_MONDAY.md`  
- `docs/GATE_F_HARDWARE_CAM_LEVER.md` · `docs/CONTROLS_FOV_HEAD_TILT_HANDOFF.md`  

**Stop:** Controls does not invent a Gate letter or lock this stub. Await AI criteria.

---

**AI disposition (2026-09-28 ~01:34 BST):** stub **ACCEPTED** as **Gate G** — formal criteria `docs/GATE_G_AI_CRITERIA.md` (tags G00–G03). Controls may score.
