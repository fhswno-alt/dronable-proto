# Gate Q Hardware — PUSH/PULL v1 directive (hospital-door)

**When:** Tue 29 Sep 2026 ~21:52 Europe/London (BST) — Dave voice  
**Audience:** Dave · Founding Hardware · Controls · AI  
**Scope:** SIM-ONLY companion honesty proposal. **READY-NOT-INSTALLED.** Soft-pass **off**. **No spend / no PO.** Live freeze md5s **KEPT**.

---

## 0. Dave directive (summary)

**Drop doorknob operations** from the door task.

| Item | v1 (March 2027 prototype) | Later (adjustment vars, not required now) |
|------|---------------------------|-------------------------------------------|
| Task | Hospital-door **push/pull** only | Knob torque, knob height, latch, OEM hinge params |
| Behavior | Robot detects door **closed** and **pushes/pulls it open** in front of itself | Grasp / lever rotate / latch clear |
| Contact | Hand → **panel** (push face) | Hand → lever / knob |
| Open measure | Panel hinge open-angle (keep ±30°) | Full 90° / walk-through / UK handle height |
| Soft-pass | **OFF** | — |
| Spend | **None** | — |
| Live freeze | **KEPT** until cospec + Dave ACK of plant swap | — |

**One-liner:** Walk + open door by panel push/pull. No knob torque. No latch modeling.

---

## 1. Live freeze (MUST stay untouched)

| Plant | Path | md5 (verified this turn) |
|-------|------|--------------------------|
| Walk / M145 | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Companion F–Q | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` |

Do **not** edit either file until AI/Controls cospec + Dave ACK of a named plant swap.

---

## 2. Inventory — knob / lever assumptions (companion plant + Hardware docs)

Assumptions that encode **hand→lever / knob / latch** interaction or treat lever contact as the door-open force path. Historical Gate F–J docs remain on disk as history; this table flags what **push/pull v1 supersedes for the door task**.

### 2a. Companion plant (live `…_gate_f.xml`)

| File / element | What it assumed | Push/pull v1 disposition |
|----------------|-----------------|--------------------------|
| Header comment Gate G03 | `door_lever` contactable bit 2 for **hand–lever only**; `door_panel` visual-only | Supersede: panel push contact; lever demoted |
| `door_lever` geom | `contype=2 conaffinity=2` — hand–lever collide | **Demote** to contype 0 (visual) in draft fork |
| `door_lever_hinge` | Hinge DOF for lever rotate ≥15° (H02/I00) | **Keep** for visual/reference (or weld later if Controls asks); **not** a v1 score DOF |
| `door_lever_site` @ rest world ≈ `(0.40, 0, 0.275)` | Distance / arc / cam FOV target at **275 mm AFF** | Keep site for cam/FOV; **not** contact target |
| `door_lever_link` child of `door_panel_link` | Hand→lever force couples into panel swing (I01/J01) | Coupling path **changes** to hand→**push face**→panel |
| `door_panel` geom | Visual-only (`contype=0`) — no panel push | Keep visual; add **push face** (or Option C: make panel contactable) |
| `door_panel_hinge` ±30°, spring 0.05 / 0.015 | Open-angle under hand→lever path | **KEEP** hinge + range + spring; force path becomes push |
| Latch joint | *(none in plant)* | **None** — do not add |
| `l/r_hand_contact` bit 2 | Affinity matched to `door_lever` | Reuse bit 2 → push face (Option A) |
| Gate K free-edge stripe | Visual open-angle tell | **KEEP** (visual) |

### 2b. Hardware docs (lever / knob / latch / height)

| File | What it assumed |
|------|-----------------|
| `docs/GATE_F_HARDWARE_CAM_LEVER.md` | Lever prop @ **275 mm AFF**; cam→lever FOV bind; Gate G update: lever contactable |
| `docs/GATE_G_HARDWARE_LEVER_CONTACT.md` | **G03 hand–lever contact** bit 2; panel stays visual-only; no latch claim |
| `docs/GATE_H_HARDWARE_LEVER_HINGE.md` | **H02 lever rotate ≥15°**; `door_lever_hinge` about Z; grasp Z 0.275 |
| `docs/GATE_I_HARDWARE_PANEL_HINGE.md` | Panel hinge; **lever child-of-panel** so hand/lever drives panel ≥10° |
| `docs/GATE_J_HARDWARE_PANEL_SPRING.md` | Soft spring so **~2 N on lever body** → panel ≥25°; coupling = hand→lever |
| `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` | Free-edge tell; notes lever unchanged |
| `docs/GATE_K_HARDWARE_PANEL_VISUAL.md` | Panel foreshortening; coupling still lever-child |
| `docs/GATE_F_LEVER_FAB_PREP.md` / `_FAB_QC.md` / `_STEP_STUB.md` | Demo prop lever fab @ **275 ±5 mm AFF**; bar 60×10×8 |
| `docs/GATE_H_LEVER_HINGE_FAB_QC.md` | Re-QC hinged lever vs fixed-bar STEP |
| `docs/GATE_I_PANEL_HINGE_FAB_QC.md` | Panel hinge dims; rest lever AFF 275 |
| `docs/HARDWARE_SYSTEM_DESIGN_BRIEF.md` | March demo = room walk + **open low prop lever** 250–300 mm; teleop lever; grasp |
| `ASSUMPTIONS.md` | Lever target **0.25–0.30 m**; optional panel+lever geom @ 0.275 m |
| `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` | Panel/lever companion table; latch / 90° / walk-through = **No** |
| `docs/GATE_Q_HARDWARE_T5D_PLANT_LEVERS.md` | Named plant honesty A/B (Q retreat — not door); panel hinge out of Q scope |
| `docs/HARDWARE_FREEZE_STATUS_MONDAY.md` | Companion freeze notes lever child-of-panel; non-claim latch/UK handle |
| `cad/gate_f_lever/` + `scripts/build_gate_f_lever.py` | Fixed-bar lever STEP/STL gauge |

### 2c. Explicit non-claims already on record (still hold)

- **Not** latch / 90° / walk-through / UK full handle height / room-walk product claim  
- Sim lever = demo prop, **not** OEM door-handle kinematics  
- Push/pull v1 **adds**: not knob torque, not lever-rotate score, not latch modeling  

---

## 3. Proposed companion delta (READY-NOT-INSTALLED)

**Draft fork:** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml`  
**md5:** `adb24309b489d56615c194e92676d040` · **MuJoCo load:** OK  

### What stays

| Element | Disposition |
|---------|-------------|
| Locked M145 kinematics / feet / HX | Untouched (live + fork base) |
| `door_panel_hinge` ±30°, damp 0.05, stiff 0.015 | **KEEP** — open-angle measure |
| `door_panel` visual geom + Gate K stripe/sites | **KEEP** visual |
| `door_lever_link` / `door_lever_hinge` / `door_lever_site` | **KEEP** structure (visual / cam ref) |
| `l/r_hand_contact` + sites bit 2 | **KEEP** |
| `kit_cam` / FOV | **KEEP** |
| No latch joint | **KEEP** (none) |

### What goes contype 0 (visual)

| Element | Change |
|---------|--------|
| `door_lever` | `contype=2` → **`contype=0 conaffinity=0 group=1`** — no hand–lever contact |

### What is new (push contact)

| Element | Spec |
|---------|------|
| `door_panel_push_face` | Box `size="0.008 0.048 0.15"` @ `pos="-0.018 0.05 0.17"` on `door_panel_link` (robot-facing −X); `contype=2 conaffinity=2 condim=3`; `mass=0`; friction `1.0 0.05 0.01` |
| `door_panel_push_site` | Site on outer push face for Controls distance / closed-detect |

### Contact bit options (Controls/AI pick)

| Option | Scheme | Status |
|--------|--------|--------|
| **A (shipped conservative draft)** | Reuse **bit 2**: push face + hands; lever demoted | **In draft fork** |
| **B** | New **bit 4** for push face; leave bit 2 free for future knob A/B; hands would need dual affinity or retarget | Documented; not in draft |
| **C** | Make whole `door_panel` `contype=2` (no dedicated face) | Documented; simpler geom, less selective |

**Hardware default for draft:** Option A — minimal hand XML change; matches current hand affinity.

---

## 4. Risks

| Risk | Note |
|------|------|
| Curriculum F–J score scripts still key on `door_lever` contact / `door_lever_hinge` angle | Controls must retarget prove to **panel hinge** + **push-face contact**; Prefer FAIL / curriculum confirm asked below |
| Cam FOV still aimed at lever Z 0.275 | May still be fine for "see door"; AI may want panel-center look-down rebind later |
| Push face thin (−X); miss if hands approach from side | Option C (full panel contact) fallback |
| Lever hinge still free (visual) | Harmless with contype 0; weld/remove later if noise |
| Spring tuned for ~2 N **lever** path (Gate J) | Push force arm differs; may need soft retune **after** cospec — not this draft |
| H0/H2 forks still assume lever contact plant | Separate READY-NOT-INSTALLED; do not merge without Dave |
| Soft-pass temptation | Soft-pass **OFF** — fail closed if push does not open |

---

## 5. Prefer FAIL / curriculum confirm asks (Controls + AI)

Please confirm or Prefer FAIL before any plant swap:

1. **Door-task v1 = panel push/pull only** — drop G03 hand–lever / H02 lever-rotate / I01–J01 lever-coupled panel as **required** door proves. Panel open-angle via `door_panel_hinge` remains the open measure.  
2. **Closed detect** — acceptable signal? (panel hinge ≈0 + optional push-site distance / contact pair `hand × door_panel_push_face`.)  
3. **Contact Option A vs B vs C** — ACK Option A draft or name B/C.  
4. **Lever hinge** — keep as inert visual DOF, or weld (remove joint) in next draft?  
5. **Cam** — keep lever-height look-down, or rebind to panel center for push/pull?  
6. **Curriculum rows F–P** — which stay LOCKED TRUE on live gate_f (historical), vs which need a new push/pull companion row after swap?  
7. **March 2027 bar** — walk + open door (panel ±30° class) sufficient; knob/latch deferred. Soft-pass off.  

**No install** until: AI/Controls cospec written + Dave ACK of plant swap. Live md5s stay.

---

## 6. Soft-pass / spend / freeze

| Rule | State |
|------|-------|
| Soft-pass | **OFF** |
| Spend / PO / Path A | **None** |
| Live M145 md5 | **KEPT** `fc94709c84f5598d4474ecfc4bb41fdc` |
| Live companion md5 | **KEPT** `59cc408eda07037a58f92ad27da045d6` |
| Draft fork | READY-NOT-INSTALLED · md5 `adb24309b489d56615c194e92676d040` · load OK |

---

## 7. Refs

- Live companion header / plant: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`  
- Draft fork: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml`  
- Freeze sheet: `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`  
- Historical lever path: `docs/GATE_G_HARDWARE_LEVER_CONTACT.md` · `docs/GATE_H_HARDWARE_LEVER_HINGE.md` · `docs/GATE_I_HARDWARE_PANEL_HINGE.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md`  
- Design brief (lever language — superseded for door task by this directive): `docs/HARDWARE_SYSTEM_DESIGN_BRIEF.md`  

**One-liner for room:** Dave dropped knob ops; Hardware inventoried lever assumptions, drafted push/pull companion fork READY-NOT-INSTALLED (Option A bit-2 push face; lever visual); live freeze md5s KEPT; waiting Controls/AI cospec + Dave ACK.
