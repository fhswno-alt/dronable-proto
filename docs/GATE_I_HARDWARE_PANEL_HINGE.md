# Gate I Hardware — light door-panel hinge (companion)

**When:** Mon 28 Sep 2026 ~01:49 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Scope:** Gate **I01 panel swing ≥10°** with lever/hand coupling — add light panel hinge DOF on companion plant only. **Sim only. No spend. No M145 hop.**  
**Labels:** mesh-derived kit demo prop; not OEM door hardware; companion additive.

## Files

| Role | Path |
|------|------|
| Locked walk/score plant (**untouched**) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| **Gate F/G/H/I companion (edit here)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Gate I criteria | `docs/GATE_I_AI_CRITERIA.md` |
| Gate H lever hinge | `docs/GATE_H_HARDWARE_LEVER_HINGE.md` |
| Gate G contact honesty | `docs/GATE_G_HARDWARE_LEVER_CONTACT.md` |
| Gate F cam/lever honesty | `docs/GATE_F_HARDWARE_CAM_LEVER.md` |
| Gate J panel spring retune (owns current soft params) | `docs/GATE_J_HARDWARE_PANEL_SPRING.md` |

Companion = locked M145 + Gate F cam/door + Gate G03 hand–lever contact + Gate H02 hinged lever + **Gate I01 hinged panel**.  
**145×86 foot boxes, HX actuator clips, kit_cam, locked M145: unchanged.**

## Hinge choice (kit demo)

Door swings in the **horizontal XY plane** about **vertical world Z** (same axis family as the lever). Robot faces **+X** toward the door.

Hinge edge: `door_panel_link` origin at the **−Y** edge of the old fixed panel so the panel swings open toward **+Y**.

| Item | Value |
|------|-------|
| Parent | `door_prop` @ `(0.45, 0, 0)` |
| Moving body | `door_panel_link` @ door_prop-local `(0, -0.05, 0)` = hinge / −Y edge |
| Joint name | **`door_panel_hinge`** |
| Joint type | `hinge` |
| Axis (local = world) | **`(0, 0, 1)`** — vertical Z |
| Range | **`−0.5236 … +0.5236` rad** (±30°); Criteria need ≥10°; margin OK |
| Rest / springref | **0°** |
| Softness | Gate I ship: `damping="0.08"`, `stiffness="0.05"` (light center spring), `armature="0.0001"`. **Gate J retuned panel spring** on companion (`damping="0.05"`, `stiffness="0.015"`) for ≥25° under lever force — see `docs/GATE_J_HARDWARE_PANEL_SPRING.md` (J owns retune; this row historical). |
| Panel mass | `0.10` kg; COM at geom center `(0, 0.05, 0.17)` relative to hinge body |
| Panel geom | half-extents `0.01 × 0.05 × 0.17` at relative `(0, 0.05, 0.17)` → rest center matches old fixed panel |

### Lever as child of panel (coupling)

| Item | Value |
|------|-------|
| Lever body | `door_lever_link` **child of** `door_panel_link` @ relative `(0.01, 0.05, 0.275)` |
| Lever joint | `door_lever_hinge` — unchanged softness/range/names from Gate H |
| Rest shaft | door_prop-local `(0.01, 0, 0.275)` → world `(0.46, 0, 0.275)` |
| Rest site / grasp | world **`(0.40, 0, 0.275)`** |

**Coupling honesty:** `door_lever` is a kinematic child of `door_panel_link`, so contact forces on the lever transmit through `door_lever_link` → panel body → `door_panel_hinge`. Not an equality constraint cheat; not a scripted panel actuator. Lever hinge remains free for I00/H rotate.

### Rest pose / grasp height

| Item | Rest (q=0 both hinges) |
|------|------------------------|
| Panel hinge | door_prop-local `(0, -0.05, 0)` → world `(0.45, -0.05, 0)` |
| Panel geom center | hinge + `(0, 0.05, 0.17)` → door_prop-local `(0, 0, 0.17)` → world `(0.45, 0, 0.17)` |
| Panel world AABB | same as previous fixed `door_panel` |
| Lever shaft | world `(0.46, 0, 0.275)` |
| **`door_lever_site` world** | **`(0.40, 0, 0.275)`** |
| **Grasp center Z AFF** | **0.275 m** (unchanged from Gate F/G/H) |

## Contype scheme (G03 still works)

Unchanged from Gate G/H:

| Bit | Value | Who | Purpose |
|-----|-------|-----|---------|
| 0 | `1` | `floor`, `l_foot_contact`, `r_foot_contact` | Walk / stance |
| 1 | `2` | **`door_lever`**, `l_hand_contact`, `r_hand_contact` | Hand–lever light contact |

- `door_lever` geom: `contype="2" conaffinity="2" condim="3"` on the moving lever child.
- `door_panel`: visual-only `contype="0" conaffinity="0"` on the moving panel body (no floor/foot collision).
- Hand ↔ lever still collides on bit 2; lever/panel still do **not** collide with floor/feet.

## Names for Controls I00/I01

| Use | Name |
|-----|------|
| Panel hinge joint (angle / ≥10°) | `door_panel_hinge` |
| Panel site (arc / pose; **on moving panel**) | `door_panel_site` |
| Panel moving body | `door_panel_link` |
| Lever hinge joint (I00 retain H02/H03) | `door_lever_hinge` |
| Lever contact geom | `door_lever` |
| Lever site (arc / distance; **on moving lever**) | `door_lever_site` |
| Lever moving body | `door_lever_link` |
| Hand sites / contact | `l_hand_site`, `r_hand_site`, `l_hand_contact`, `r_hand_contact` |
| Load plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |

I00: retain H02/H03 lever rotate ≥15° (lever hinge free; panel may co-move lightly under coupling).  
I01: hand/lever contact drives `door_panel_hinge` ≥ **10°** (or measure `door_panel_site` arc) without tip/skate fail — not free-fall / not panel-only scripted actuator.

## Manufacturing / STEP note

- Sim panel hinge is a **demo prop** DOF for Gate I — **not** OEM door / latch / UK handle-height product claim.
- **Panel STEP not required** for Gate I (visual-only geom; no fab PO).
- Lever STEP stub refresh still deferred (Gate H note); out of scope this ship.
- Foot STEP / M2 145×86 sole: frozen; untouched. **No PO / no spend.**

## Explicit non-claims

- Locked **`ainex_controls_m2_145.xml`** not edited (md5 must match `fc94709c84f5598d4474ecfc4bb41fdc`).
- No foot contact / friction / HX / kit_cam / Gate E plant change.
- **Not full door open / latch clear / walk-through** — limited swing ≥10° only.
- Light centering spring must not be treated as a lock — range and soft stiffness allow ≥10° under hand force through lever. (Gate J further soft-retuned panel spring for ≥25°; see `GATE_J_HARDWARE_PANEL_SPRING.md`.)
- Not equality-constraint or scripted panel actuator; coupling is kinematic parenthood.
- Not Pi / Orin / NN-first; vision stays out of walk residual.

## Smoke

```text
md5sum mujoco/ainex_hiwonder/ainex_controls_m2_145.xml
# expect fc94709c84f5598d4474ecfc4bb41fdc

model = mujoco.MjModel.from_xml_path("mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml")
# joint door_panel_hinge: axis Z, range ±0.5236
# joint door_lever_hinge: axis Z, range ±0.5236 (unchanged)
# qpos0 → door_lever_site world ≈ (0.40, 0, 0.275)
# door_lever contype=2 conaffinity=2; door_panel contype=0
# lever is child of door_panel_link (coupling path)
```
