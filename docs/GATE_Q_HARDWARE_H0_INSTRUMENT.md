# Gate Q — Hardware H0 instrument companion fork (T5-D1)

**When:** Mon 28 Sep 2026 ~21:52 Europe/London (BST)  
**Owner:** Founding Hardware Engineer → Controls / AI / Dave  
**Scope:** Diagnosis-only instrument sites (+ optional H1-lite visual tell) on a **companion fork**. Soft-pass **off**. Bars **KEPT**. No spend / no PO. **Live freeze NOT edited.**

Aligns: `docs/GATE_Q_HARDWARE_T5D_PLANT_LEVERS.md` · `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` · Controls T5-D Prefer FAIL train

---

## 1. Freeze line (verified this turn)

| Plant | Path | md5 | Edit? |
|-------|------|-----|-------|
| Walk / Gate E (M145) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **NO — KEPT** |
| Companion live F–Q | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **NO — KEPT** |
| **H0 instrument fork** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_h0.xml` | `ad9a1817f015e68e68f14535311369f0` | NEW (sites + contype=0 only) |

MuJoCo 3.14 load of H0 fork: **OK** (`nq=33 nv=32 nbody=29 ngeom=37 nsite=13`).  
`l_foot_contact` / `r_foot_contact` size / pos / friction / contype / conaffinity **byte-match** live gate_f.

**Not shipped:** friction bump · softsole / solref · soft-XY · foot-lift · planform resize · H2 contact-height CAD match (needs Dave unlock).

---

## 2. What H0 adds (physics-neutral)

Foot contact (unchanged): box half `0.0725 0.0430 0.008` · pos `0.030 0.0 -0.018` · sole bottom / `SOLE_OFFSET` = **0.026 m** (`pos_z − half_z`).

### Sites (on each `*_ank_roll_link` unless noted)

| Site | Body | Local pos (m) | Role |
|------|------|---------------|------|
| `l_sole_bottom_site` / `r_sole_bottom_site` | ank_roll | `0.030 0.0 -0.026` | Contact bottom center — primary plant/clear Z |
| `l_sole_toe_site` / `r_sole_toe_site` | ank_roll | `0.1025 0.0 -0.026` | +X sole edge (toe) daylight |
| `l_sole_heel_site` / `r_sole_heel_site` | ank_roll | `-0.0425 0.0 -0.026` | −X sole edge (heel) daylight |
| `plant_rest_floor_site` | **world** | `0 0 0` | Floor plane Z reference for clear logging |

### H1-lite visual (optional, included)

| Geom | Parent | Size (half) | Pos | Contact |
|------|--------|-------------|-----|---------|
| `l_sole_clear_tell` / `r_sole_clear_tell` | ank_roll | `0.0725 0.0430 0.001` | `0.030 0.0 -0.026` | `contype=0 conaffinity=0` group=1 · `edge_tell` green (Gate K family) |

Thin sole-plane tell only — watch honesty for daylight vs floor. **No** dynamics.

---

## 3. How Controls should use for PRE_GAP / ε

Recommended world-frame reads each step (after `mj_forward`):

1. **Sole daylight (m):** `site_xpos[sole_bottom].z − site_xpos[plant_rest_floor_site].z`  
   - Plant rest ≈ **0** when sole flush on floor (with `SOLE_OFFSET` 0.026 baked into kinematics).  
   - Clear credit bar (criteria): sole clear **≥ 0.02 m** above plant rest, dwell ≥150–250 ms.
2. **Toe / heel split:** same Z formula on `*_sole_toe_site` / `*_sole_heel_site` → pitch daylight / rocker honesty without inventing foot-lift geom.
3. **Plant vs clear dwell:** both `l` and `r` sole_bottom daylight ≪ 2 cm ⇒ **planted**; either ≥ 2 cm multi-frame ⇒ **clear** window for PRE_GAP / `dx_clear` vs `dx_plant`.
4. **ε / plant-cam:** do **not** treat site motion as a physics fix — sites expose Root B planted-middle signatures; Prefer FAIL bars KEPT.

Load path for diagnosis A/B (not score plant-of-record):

```text
mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_h0.xml
```

Live score / freeze companion stays `…_gate_f.xml` md5 `59cc408e…` until Dave names a different plant.

---

## 4. H3 — foot mesh AABB vs contact box (read-only dig)

See also: `docs/GATE_Q_HARDWARE_H3_AABB_RECONFIRM.md` (same numbers).

| Quantity | Contact box (XML / GEO00) | ank_roll STL AABB (mesh) |
|----------|---------------------------|---------------------------|
| Planform L×W | **145 × 86 mm** (half `0.0725 × 0.0430`) | **~135 × 76 mm** (L `0.13508`; W `0.07604` L / `0.07605` R) |
| Vertical span | **16 mm** proxy (half `0.008`) | Mesh Z extent **~34 mm** (includes ankle structure above sole) |
| Sole bottom z (ank_roll) | **−0.026** | Mesh min-Z ≈ **−0.0231** (L/R) |
| Contact `pos` | `0.030 0.0 -0.018` | — |
| CAD STEP stack | — | Fab **~4.5 mm** (not XML height) — frozen; H2 only w/ Dave unlock |

**Read:** M145 contact box is an intentional **oversize sleeve** vs mesh AABB (~+10 mm L, ~+10 mm W) with a **16 mm sim proxy** height (not CAD 4.5 mm). Box still honest as manufacturing M2 short planform; meshsole variant (`135×76`) exists for AABB-match A/B but is **not** the locked score plant. **No XML edit** from this dig.

---

## 5. Explicit non-claims

- H0/H1 do **not** fix skate / plant-cam ε / Prefer FAIL — diagnosis only.
- Live gate_f + M145 md5s **unchanged**.
- H2 contact-height CAD match **not installed** — needs Dave freeze unlock + AI Prefer FAIL A/B.
- Soft-pass **off**. No spend / no PO / no Path A.

**One-liner:** H0 companion fork ships sole daylight sites + contype=0 tell; live freeze md5s KEPT; Controls can tighten PRE_GAP logging without plant physics change.
