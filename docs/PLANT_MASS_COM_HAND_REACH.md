# Plant mass / COM / hand reach — sim honesty one-pager

**When:** Mon 28 Sep 2026 ~01:33 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Scope:** Locked M145 plant + Gate F companion — **numbers from MuJoCo**, not invented.  
**Constraints:** No spend. No STEP refresh. No contact/HX plant hop. Gates D/E/F locked in sim; foot STEP frozen.

---

## Plants

| Role | Path | sha16 |
|------|------|-------|
| Locked score (Gate D/E) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `4f30b6e53578789f` |
| Gate F companion (hands / lever) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `8de4ab2b0084c740` |

Companion = M145 + world `door_prop`/`door_lever` + `kit_cam`/`kit_cam_site` on `head_tilt_link`. Robot link inertials **byte-identical** to locked plant.

---

## 1. Mass / COM (locked plant)

**Method:** `MjModel.body_mass` sum over robot bodies; COM = `subtree_com[body_link]` after **kinematic foot-ground** (qpos0 joint angles; free-joint Z lowered so min `l/r_foot_contact` geom bottom = 0). No settle / no dynamics.

| Item | Measured | Notes |
|------|----------|-------|
| **Robot total mass** | **2.3475 kg** | Sum of 25 kit link inertials |
| Kit envelope claim | **~2.45 kg** Standard | `docs/FACTORY_ONE_PAGER.md`, `HARDWARE_SYSTEM_DESIGN_BRIEF.md`; Starter 2.25 kg |
| Δ vs kit claim | **−0.10 kg (−4.2%)** | Within ~10% MJCF accept band (`HARDWARE_SYSTEM_DESIGN_BRIEF` §2.7); likely missing battery / harness / Pi (see `AINEX_MUJOCO_FROM_HIWONDER.md`) |
| Prior URDF note | ≈2.35 kg | Same package — matches this plant |
| **COM world Z** (foot-grounded, qpos0) | **0.2252 m** | `subtree_com[body_link]` |
| Free-joint body Z (grounded) | 0.2375 m | Foot bottoms @ 0 |
| Controls COM class | ~0.22–0.24 m | `ASSUMPTIONS.md` / brief — **in class** |

Heaviest links: `body_link` 0.743 · each `*_sho_roll_link` 0.121 · each `*_hip_roll`/`*_ank_pitch` 0.120 · each `*_el_yaw` 0.102 kg.

### Gate F companion props

| Item | Value |
|------|-------|
| `door_prop` body mass | **0.7184 kg** (MuJoCo default density × box volumes; no explicit `<inertial>`) |
| Contact | `contype="0" conaffinity="0"` (visual/marker only) |
| Weld | World child, **no joint** — not on robot free tree |
| **Robot mass on gate_f** | **2.3475 kg** — **identical** to locked M145 |
| Robot COM Z (grounded qpos0) | **0.2252 m** — identical |

**Verdict:** Companion contype-0 props do **not** change robot plant mass or COM. File-total mass rises by ~0.72 kg only because density-inferred world geom mass is counted in `body_mass[door_prop]`; that mass is **irrelevant** to walk / Gate E / hand-reach dynamics. No plant change needed.

---

## 2. Hand reach Z vs Gate F lever band

**Lever band (criteria):** **0.25–0.30 m AFF** · working center **0.275 m** (`door_lever` geom Z = **0.275 m** exact).  
**F02 lock claim:** hand Z **L ≈ 0.291 m** in band (`GATE_F_AI_LOCK.md`, `F02.json`).

### Hand frames (no dedicated hand sites)

| Frame | Kind | Role |
|-------|------|------|
| **`l_gripper_link`** | body | Left hand / gripper frame (Controls `hand_z_L`) |
| **`r_gripper_link`** | body | Right hand / gripper frame (Controls `hand_z_R`) |
| `kit_cam_site` | site | Optical only — not a hand frame |

Measured Z = `data.xpos[body_id, 2]` (body origin world Z).

### Measured hand Z AFF (gate_f plant)

| Pose | Grounding | L Z (m) | R Z (m) | vs band 0.25–0.30 |
|------|-----------|---------|---------|-------------------|
| **qpos0** (all joints 0) | foot-grounded | **0.325** | **0.325** | above band |
| Gait stand arms (`sho_roll` ±1.4, `el_pitch` 0.38) | foot-grounded | **0.143** | **0.143** | below band |
| **F02 reach** (locked JSON arms: `sho_pitch` −1.5, `sho_roll` ±0.2, `el_pitch` −0.1 + gait legs) | foot-grounded | **0.302** | **0.302** | at upper edge |
| **F02 reach** (same arms) | `COM_Z=0.225` (GateFEnv style) | **0.295** | **0.295** | **in band** |
| F02 Controls lock (dynamics hold) | Gate F prove | **0.291** L / 0.315 R | — | **PASS** (L in band) |

F02 arm setpoints from locked `previews/ainex_walk/iterate/F02.json` (`kinematic_qpos_hold`). Remeasure at GateFEnv free-joint Z reproduces L/R ≈ **0.295 m** — within a few mm of locked **0.291 L**.

### Is the F02 band plant-honest without plant change?

**Yes.** Standard gripper body frames reach **0.25–0.30 m AFF** on the locked companion with kinematic arm setpoints only. Lever @ **0.275 m** sits inside that reach. **No contact box, HX clip, inertial, or STEP change required.**

---

## 3. Caveats

1. **Mesh-derived inertials** — Hiwonder URDF per-link `mass` / `diaginertia`; **not** a weighed kit; not OEM STEP.
2. **Kit 2.45 kg** is marketing / buy-sheet envelope; sim **2.35 kg** omits battery/harness/Pi class mass until in-hand weigh.
3. **COM Z** is kinematic foot-grounded (or GateFEnv `COM_Z`), not a long settle / not IMU-fused hardware COM.
4. **Hand Z** is body-origin of `*_gripper_link`, not fingertip mesh AABB; grasp contact geometry not modeled for Gate F.
5. **`door_prop` density mass** is cosmetic for dynamics; do not treat gate_f file-total mass as robot mass.
6. Optical Z / FOV honesty lives in `docs/GATE_F_HARDWARE_CAM_LEVER.md` + `docs/MONDAY_FOV_RECONFIRM.md` — out of scope here except `kit_cam_site` Z grounded ≈ **0.380 m** (reconfirmed).

---

## 4. Bottom line

| Claim | Result |
|-------|--------|
| Locked plant mass | **2.35 kg** vs kit **~2.45 kg** (−4%) |
| COM Z foot-grounded | **0.225 m** (in 0.22–0.24 class) |
| gate_f props vs robot mass | **No change** to robot mass/COM |
| Hand frames | `l_gripper_link` / `r_gripper_link` |
| F02 band 0.25–0.30 AFF | **Plant-honest** — F02 L ~0.29 m without plant hop |

**Refs:** `docs/PLANT_TO_CAD_ASSEMBLY_HONESTY.md` · `docs/GATE_F_AI_CRITERIA.md` · `docs/AINEX_MUJOCO_FROM_HIWONDER.md` · `previews/ainex_walk/iterate/F02.json`
