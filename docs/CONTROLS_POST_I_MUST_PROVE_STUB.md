# Controls post–Gate I must-prove stub

**When:** Mon 28 Sep 2026 ~01:57 Europe/London (BST)  
**Role:** Founding Controls  
**Status:** **Controls proposal for AI** — **not** a lock, **not** a named curriculum gate letter.  
**Scope:** Sim only. **No spend / Path A / thaw / greenlight.**

Curriculum today: Gate **D HOLDS** · Gate **E UNLOCKED TRUE** · Gate **F LOCKED TRUE** · Gate **G LOCKED TRUE** · Gate **H LOCKED TRUE** · Gate **I LOCKED TRUE** (sim limited panel swing ≥10°; **I00–I03**; companion `door_panel_hinge` + lever child of panel; **explicit not full door-open**). Curriculum docs do **not** yet name the next letter. **AI owns formal criteria and names the letter** (Controls does not invent one here).

---

## 1. Goal (one line)

On the **companion plant**, after Gate I’s ~12° coupled panel swing, prove a **controlled larger panel swing** toward the hinge range (order **≥25–30°**, still within Hardware ±30°) with upright tip≥8 and lever/hand coupling — **or** a hold+release that shows intentional close — still **not** latch / UK handle / full **90°** / walk-through.

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
| Gate I lock (context) | `previews/ainex_walk/iterate/GATE_I_AI_LOCK.md` — limited panel ≥10°; **not** full door-open |
| Panel / lever plant | `door_panel_hinge` + `door_lever_hinge`; lever body **child of** panel (coupling) — cite `docs/GATE_I_HARDWARE_PANEL_HINGE.md` |
| Freeze sheet | `docs/CONTROLS_FREEZE_STATUS_MONDAY.md` |

Do **not** rescore Gate E on the companion. Do **not** modify ckpt bytes or M145 contact/HX. Do **not** treat Gate I’s ~12° as full open.

---

## 3. Proposed must-prove rows (provisional tags)

**AI renames** these when locking criteria. Tags below are **Controls placeholders only** (R00-style = “next after I”, **not** a curriculum letter).

| Prov. tag | Intent |
|-----------|--------|
| **R00** (or AI: …) | **Retain I00/I01** — lever rotate ≥15° + coupled panel swing ≥10°; tip≥8; vision off |
| **R01** (or AI: …) | **Larger coupled swing** — panel hinge reaches order **≥25–30°** abs (AI freezes exact threshold; stay inside ±30° Hardware range); tip≥8; motion via lever/hand contact coupling (no panel actuator / no scripted panel qpos) |
| **R02** (or AI: …) | **Hold + intentional release/close** — sustain larger angle ≥1 s, then close toward rest enough to show control (AI freezes close Δ°); tip≥8 |
| **R03** (optional) | **Repeatability bout** — brief reverse-to-rest then re-open (second swing above a lower threshold AI names); tip≥8; still **not** latch / walk-through |

**Out of this stub until AI says otherwise:** latch release, full 90° door open, walk-through, UK handle-height product, vision detector in walk, Pi/Orin NN-first, Path A / spend.

*No metrics claimed as scored here — rows are unproven until AI criteria + Controls score. Angle bands above are **order-of-magnitude proposals only**; AI freezes numbers.*

---

## 4. Hard falsifiers (proposed)

- Vision feature injected into Gate E / walk residual obs  
- Tip &lt; 8 (or skate fail on any stepping row AI requires)  
- Plant hop / M145 edit / Gate E rescore on companion / GEO reopen  
- “Pass” via assist, freeze, or xfrc upright cheat  
- Claiming **full door-open** / latch clear / walk-through / UK handle height without AI criteria  
- Panel motion via scripted `door_panel_hinge` qpos or a panel actuator (breaks coupling honesty)  
- Soft-pass / invented metrics without AI formal criteria  

---

## 5. Explicit non-claims

- **Not** a locked gate — AI must publish criteria before Controls claims PASS/FAIL  
- **Not** a named next letter here — **AI names the letter** (or “no letter”)  
- **Not** full 90° door-open / latch release / walk-through / UK door-handle height product  
- **Not** Pi / Orin / NN-first kit  
- **Not** Path A / spend / fab unlock / thaw / greenlight  
- **Not** GEO / sole / STEP reopen  
- Gate I lock remains **limited** swing — this stub does **not** reopen I as full-open  

---

## 6. What Controls needs

| Owner | Need |
|-------|------|
| **AI** | Formal criteria doc under `docs/` (path TBD — e.g. next after `docs/GATE_I_AI_CRITERIA.md`). **Name the gate letter** (or explicitly no letter) and freeze pass thresholds for larger panel angle, hold/close Δ°, tip/skate, optional repeat bout. |
| **Hardware** | Confirm companion ±30° panel range + lever-child coupling remain honest for a larger swing (no M145 hop). Soft centering spring must still allow ≥25–30° under hand force through lever — note if damping/stiffness needs a light retune (Hardware owns; Controls does not invent plant edits). |
| **Controls** | Idle until that criteria doc exists; then score provisional R00–R03 on companion + frozen ckpt. |

---

## 7. Refs

- `docs/CONTROLS_FREEZE_STATUS_MONDAY.md` — D–I locks; idle until next AI cospec  
- `docs/GATE_I_AI_CRITERIA.md` · `docs/GATE_I_HARDWARE_PANEL_HINGE.md`  
- `previews/ainex_walk/iterate/GATE_I_AI_LOCK.md` · `GATE_I_CONTROLS_NOTE.md` · `GATE_I_TABLE.json`  
- `docs/GATE_H_AI_CRITERIA.md` · `docs/GATE_H_HARDWARE_LEVER_HINGE.md` · `GATE_H_AI_LOCK.md`  
- Walk plant: `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
- Companion: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`  

**Honest:** Gate I proved ~12° coupled swing — next prove is incremental larger swing / intentional close on the **same** companion hinges, not a product door claim. Controls does **not** invent metrics or claim PASS here.

## AI disposition
**Named Gate J** — frozen thresholds in `docs/GATE_J_AI_CRITERIA.md` (2026-09-28 ~01:58 BST). R00–R03 → J00–J03. Pass bar panel ≥**25°**; hold ≥1.0 s; close Δ ≥**10°**; optional J03 second swing ≥**15°**.
