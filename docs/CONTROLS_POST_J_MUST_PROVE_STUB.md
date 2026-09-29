# Controls post–Gate J must-prove stub

**When:** Mon 28 Sep 2026 ~02:05 Europe/London (BST)  
**Role:** Founding Controls  
**Status:** **Controls proposal for AI** — **not** a lock, **not** a named curriculum gate letter.  
**Scope:** Sim only. **No spend / Path A / thaw / greenlight.**

Curriculum today: Gate **D HOLDS** · Gate **E UNLOCKED TRUE** · Gate **F–J LOCKED TRUE** (sim). Gate **J** = larger coupled panel ≥25° on soft-spring companion (**J00–J03**); AI lock `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md`; **explicit not full door-open**. Companion `door_panel_hinge` range is **±30°** and J already hits ~**30°** — **further angle needs a Hardware range bump**; Controls will **not** pretend this plant can do 90°. Curriculum docs do **not** yet name the next letter. **AI owns formal criteria and names the letter.**

---

## 1. Goal (one line) — **primary recommendation**

On the **current ±30° companion plant**, package Gate J into a **demo-grade repeatable multi-bout open/close** script (N successful coupled swing+close bouts, tip≥8, coupling honesty) — **without** claiming full open / latch / walk-through.

---

## 1b. Alternatives (AI picks; Controls does not demand PO)

| Option | Intent | Honest constraint |
|--------|--------|-------------------|
| **Primary** | Multi-bout open/close reliability on **current ±30°** plant | Angle already at hinge stop in J; value is **repeatability / demo packaging**, not more degrees |
| **Alt A** | Hardware **range bump** (e.g. ±45° or ±60°) then larger swing prove | **Only if AI wants more angle**; Controls does **not** demand spend/PO; M145 untouched |
| **Alt B** | Leave-and-return / **re-grasp after release** (task continuity) on companion | Still ±30° unless Alt A ships; tip≥8; coupling honesty |

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
| Gate J lock (context) | `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md` — panel ≥25° coupled; ~30° at stop; **not** full door-open |
| Panel / lever plant | `door_panel_hinge` (±30°) + `door_lever_hinge`; lever **child of** panel; Gate J spring damp 0.05 / stiff 0.015 — cite `docs/GATE_I_HARDWARE_PANEL_HINGE.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md` |
| Freeze sheet | `docs/CONTROLS_FREEZE_STATUS_MONDAY.md` |

Do **not** rescore Gate E on the companion. Do **not** modify ckpt bytes or M145 contact/HX. Do **not** invent 90° on a ±30° hinge.

---

## 3. Proposed must-prove rows (provisional tags)

**AI renames** these when locking criteria. Tags below are **Controls placeholders only** (S00-style = “next after J”, **not** a curriculum letter).

### Primary path — multi-bout reliability (current ±30°)

| Prov. tag | Intent |
|-----------|--------|
| **S00** (or AI: …) | **Retain J00/J01** — lever ≥15° + coupled panel ≥25° (within ±30°); tip≥8; vision off; Gate J spring |
| **S01** (or AI: …) | **Multi-bout open/close** — N successful bouts (AI freezes N, e.g. 3–5): each bout open to ≥25° (or AI threshold), hold ≥1 s optional, close Δ ≥10° (or AI), tip≥8 throughout; coupling honesty |
| **S02** (or AI: …) | **Bout success rate** — ≥K/N bouts pass without tip fail / loss of coupling (AI freezes K/N); continuous video |
| **S03** (optional) | **Scripted demo packaging** — single continuous MP4 of N bouts suitable for room review; still **not** latch / 90° / walk-through |

### If AI selects Alt A (range bump) instead/in addition

| Prov. tag | Intent |
|-----------|--------|
| **S1A** | After Hardware ships wider `door_panel_hinge` range, prove coupled swing to AI-frozen threshold **inside new range** (still **not** 90° product / latch unless AI says so) |

### If AI selects Alt B (leave-and-return)

| Prov. tag | Intent |
|-----------|--------|
| **S1B** | Release grasp, brief leave (or stand), return + re-grasp, then one coupled swing ≥ AI threshold on companion; tip≥8 |

**Out of this stub until AI says otherwise:** latch release, full 90° door open, walk-through, UK handle-height product, vision detector in walk, Pi/Orin NN-first, Path A / spend.

*No metrics claimed as scored here — rows are unproven until AI criteria + Controls score. N/K and angle thresholds are **proposals only**; AI freezes numbers.*

---

## 4. Hard falsifiers (proposed)

- Vision feature injected into Gate E / walk residual obs  
- Tip &lt; 8 (or skate fail on any stepping row AI requires)  
- Plant hop / M145 edit / Gate E rescore on companion / GEO reopen  
- “Pass” via assist, freeze, or xfrc upright cheat  
- Claiming **full door-open** / latch clear / walk-through / UK handle height / **90°** without AI criteria + Hardware range honesty  
- Pretending ±30° plant supports &gt;30° swing (soft-pass angle)  
- Panel motion via scripted `door_panel_hinge` qpos or panel actuator  
- Soft-pass / invented metrics without AI formal criteria  

---

## 5. Explicit non-claims

- **Not** a locked gate — AI must publish criteria before Controls claims PASS/FAIL  
- **Not** a named next letter here — **AI names the letter** (or “no letter”)  
- **Not** latch release / UK door-handle height product / full **90°** open / walk-through  
- **Not** Pi / Orin / NN-first kit  
- **Not** Path A / spend / fab unlock / thaw / greenlight  
- **Not** GEO / sole / STEP reopen  
- **Not** a demand that Hardware bump range — Alt A is optional if AI wants more angle  
- Gate J lock remains **larger-but-limited** swing on ±30° — this stub does **not** reopen J as full-open  

---

## 6. What Controls needs

| Owner | Need |
|-------|------|
| **AI** | Formal criteria doc under `docs/` (path TBD — e.g. next after `docs/GATE_J_AI_CRITERIA.md`). **Name the gate letter** (or no letter). Choose **primary vs Alt A vs Alt B**. Freeze N/K bout thresholds (primary) or angle after range bump (Alt A) or leave-return timing (Alt B). |
| **Hardware** | **Primary / Alt B:** no plant change required (confirm ±30° + Gate J spring stay plant-of-record). **Alt A only:** if AI asks, ship wider `door_panel_hinge` range on companion + short note; **no M145 hop**; Controls does not invent the bump or demand PO. |
| **Controls** | Idle until that criteria doc exists; then score provisional Sxx on companion + frozen ckpt. |

---

## 7. Refs

- `docs/CONTROLS_FREEZE_STATUS_MONDAY.md` — D–J locks; idle until next AI cospec  
- `docs/GATE_J_AI_CRITERIA.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md`  
- `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md` · `GATE_J_CONTROLS_NOTE.md` · `GATE_J_TABLE.json`  
- `docs/GATE_I_AI_CRITERIA.md` · `docs/GATE_I_HARDWARE_PANEL_HINGE.md` · `GATE_I_AI_LOCK.md`  
- `docs/CONTROLS_POST_I_MUST_PROVE_STUB.md` (prior stub; Gate J named)  
- Walk plant: `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
- Companion: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (±30° panel hinge — **stop**, not 90°)  

**Honest:** Gate J already rides the ±30° stop (~30°). Next value on this plant is **repeatable multi-bout demo reliability** (primary), not inventing more degrees. More angle = Hardware range bump (Alt A) only if AI asks. Controls does **not** invent metrics or claim PASS here.

## AI disposition
**Named Gate K** — primary path only. Frozen in `docs/GATE_K_AI_CRITERIA.md` (2026-09-28 ~02:06 BST). S00–S03 → K00–K03. **N=3** consecutive bouts; **3/3** success; per-bout open ≥25° / hold ≥1.0 s / close Δ ≥10°. **Alt A** (range bump) and **Alt B** (leave/re-grasp) deferred — not Gate K.
