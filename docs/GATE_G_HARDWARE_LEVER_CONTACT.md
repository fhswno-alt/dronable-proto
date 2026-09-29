# Gate G Hardware — contactable lever + hand contact (companion)

**When:** Mon 28 Sep 2026 ~01:34 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Scope:** Gate **G03 light contact** on companion plant only. **Sim only. No spend. No STEP.**  
**Labels:** mesh-derived kit; companion additive; locked M145 untouched.

## Files

| Role | Path |
|------|------|
| Locked walk/score plant (**untouched**) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| **Gate F/G companion (edit here)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Gate G criteria | `docs/GATE_G_AI_CRITERIA.md` |
| Gate F cam/lever honesty | `docs/GATE_F_HARDWARE_CAM_LEVER.md` |

Companion = locked M145 + Gate F cam/door + **Gate G03 hand–lever contact**.  
**145×86 foot boxes, solref/solimp, friction, HX actuator clips: unchanged** on companion walk geoms and on locked M145.

## What changed (companion only)

1. **`door_lever`** — was visual-only (`contype=0`); now **contactable** on contact bit **2** (`contype="2" conaffinity="2" condim="3"`). Fixed geom (no slide joint); contact detection is enough for G03 first ship.
2. **`door_panel`** — stays visual-only (`contype="0" conaffinity="0"`).
3. **`door_lever_site`** — site at lever geom center `pos="-0.05 0 0.275"` (world ≈ `(0.40, 0, 0.275)` with `door_prop` @ `0.45 0 0`) for Controls distance checks.
4. **Hand sites + contact spheres** on gripper bodies (mass=`0`, no COM change):

| Name | Parent body | Local pos (m) | Notes |
|------|-------------|-----------------|-------|
| `l_hand_site` | `l_gripper_link` | `0.002 0.024 0` | Marker at mesh AABB distal tip (+Y) |
| `l_hand_contact` | `l_gripper_link` | `0.002 0.024 0` | Sphere `size=0.012`; bit 2 |
| `r_hand_site` | `r_gripper_link` | `0.002 -0.024 0` | Marker at mesh AABB distal tip (−Y) |
| `r_hand_contact` | `r_gripper_link` | `0.002 -0.024 0` | Sphere `size=0.012`; bit 2 |

Local tip from Hiwonder gripper STL AABB in body frame (qpos0 identity): L max-Y ≈ `0.024`; R min-Y ≈ `−0.024`; X≈`0.002` at face center. Mesh-derived, not OEM STEP.

## Contype bitmask scheme

MuJoCo collides when `(contype_a & conaffinity_b) && (contype_b & conaffinity_a)`.

| Bit | Value | Who | Purpose |
|-----|-------|-----|---------|
| 0 | `1` | `floor`, `l_foot_contact`, `r_foot_contact` | Walk / stance (unchanged) |
| 1 | `2` | `door_lever`, `l_hand_contact`, `r_hand_contact` | Hand–lever light contact only |

- Hand ↔ lever: both bit 2 → **collide** (G03).
- Hand ↔ floor/feet: bit 2 vs bit 0 → **no** collide (avoids hand–ground / hand–foot noise).
- Lever ↔ floor/feet: same → **no** collide (lever does not rest on feet or floor).
- `door_panel` / visual meshes: `contype=0` → never collide.

## Names for Controls G03

| Use | Name |
|-----|------|
| Lever geom (contact) | `door_lever` |
| Lever site (distance) | `door_lever_site` |
| Hand sites (reach ≤5 cm) | `l_hand_site`, `r_hand_site` |
| Hand contact geoms | `l_hand_contact`, `r_hand_contact` |
| Load plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |

G02 can still score kinematic reach to `door_lever` / `door_lever_site` without requiring contact pairs. G03: brief hand–lever geom contact (`l_hand_contact`/`r_hand_contact` × `door_lever`).

## Explicit non-claims

- Locked **`ainex_controls_m2_145.xml`** not edited (hash/mtime must match pre-Gate-G).
- No foot 145×86 / friction / HX change on either plant.
- No soft-slide lever (≤5 mm depression) in this ship — fixed contactable lever only.
- No STEP / cart / spend / Path A unlock.
- Not grasp-force closure; not door-open / latch claim.

## Smoke

```text
model = mujoco.MjModel.from_xml_path("mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml")
# door_lever / l_hand_contact / r_hand_contact → contype=2 conaffinity=2
# door_panel → 0; floor / feet → 1
# md5/mtime of ainex_controls_m2_145.xml unchanged
```
