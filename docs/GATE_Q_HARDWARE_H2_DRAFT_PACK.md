# Gate Q Hardware — H2 CAD-height honesty draft pack

**When:** Mon 28 Sep 2026 ~22:16 Europe/London (BST)  
**Audience:** Dave · Founding Hardware · Controls / AI Prefer FAIL  
**Status:** ***READY-NOT-INSTALLED*** — companion fork(s) on disk only.  
**Scope:** SIM-ONLY draft. Soft-pass **off**. **No spend / no PO**. Live freeze **KEPT**.

---

## 1. What this is

Contact-height honesty A/B: XML **16 mm** collision proxy → CAD **~4.5 mm** plate+tread stack, mirrored from walk-plant GEO01 (`ainex_controls_m2_145_cadsole.xml`) onto the **gate_f companion** (door / lever / panel / hand / cam sites intact). Same **145×86** planform.

| Role | Path | md5 |
|------|------|-----|
| Live walk freeze (DO NOT EDIT) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Live companion freeze (DO NOT EDIT) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` |
| H0 instrument (left alone) | `…_gate_f_h0.xml` | `ad9a1817f015e68e68f14535311369f0` |
| **H2 draft (this pack)** | `…_gate_f_h2.xml` | `b6e574d60ddb1c06cdbd7e8cd256cacc` |
| **H0+H2 optional merge** | `…_gate_f_h0_h2.xml` | `90d8929006ad25c4772a75aa7825a0a4` |
| Walk GEO01 reference | `…_145_cadsole.xml` | (unchanged reference) |

Walk / companion live md5s re-verified after this draft — **unchanged**.

---

## 2. Geometry delta (feet only)

From live gate_f → H2 (both `l_foot_contact` / `r_foot_contact`):

| Field | Live gate_f | H2 draft |
|-------|-------------|---------|
| Planform (full) | 145 × 86 mm | **unchanged** |
| Half-size xy | `0.0725 0.0430` | **unchanged** |
| Half-size z | `0.008` (16 mm full) | **`0.00225`** (4.5 mm full) |
| Contact `pos` | `0.030 0.0 -0.018` | `0.030 0.0 **-0.02375**` |
| Sole bottom (body frame) | `pos_z − half_z = −0.026` | **same −0.026** |
| Friction | `1.6 0.1 0.01` | **unchanged** |
| Door / lever / panel / hand / cam | as gate_f | **intact** |

### `SOLE_OFFSET` unchanged

```
SOLE_OFFSET = 0.026  # (-pos_z) + half_z
# live:  0.018 + 0.008   = 0.026
# H2:    0.02375 + 0.00225 = 0.026
```

No script default change. No friction / softsole / soft-XY / foot-lift / planform resize.

---

## 3. Install / use policy

| Rule | Detail |
|------|--------|
| ***READY-NOT-INSTALLED*** | Forks exist for Prefer FAIL A/B after unlock — **not** score plant-of-record |
| Needs | Dave freeze unlock **+** AI Prefer FAIL A/B cospec |
| **NOT** for mid-T5-D1 | Prefer FAIL train keeps spinning on live `gate_f` (59cc…) |
| Companion-first | A/B on `…_gate_f_h2.xml` only; do **not** apply to locked M145 without Gate E re-prove |
| Soft-pass | **Off** |
| Spend | **None** — MFG may **quote** existing 145×86 / ~4.5 mm STEP on unlock; **no PO** |

Optional `…_gate_f_h0_h2.xml` = H0 sole daylight sites/tells + H2 height (sole sites remain @ z=−0.026). Useful if Controls wants instrument + honesty together after unlock.

---

## 4. Risk / honesty (non-claims)

- **Not a sure skate fix.** Tangential skate / planted-middle behavior may change *honestly* with thinner contact; Prefer FAIL may still fail.
- **May reopen contact contract** (timing, clearance heuristics, `CONTACT_Z_THR` coupling) — treat as A/B evidence, not curriculum TRUE.
- **Gate E re-prove risk** if H2 vertical is ever applied to locked M145 later — do not hop walk plant without Dave + Gate E protocol.
- Does **not** unlock soft-XY / softsole / planform resize / Path A / spend.
- Does **not** install onto live freeze while T5-D1 Prefer FAIL spins.

Refs: `docs/PLANT_CADSOLE_AB.md` · `docs/PLANT_CONTACT_HONESTY_PACK.md` GEO01 · `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md` · `docs/HARDWARE_FREEZE_STATUS_MONDAY.md`.

---

## 5. Verification (this turn)

```text
MUJOCO_GL=glfw .venv/bin/python  # MjModel.from_xml_path
  …_gate_f_h2.xml     → LOAD_OK; l/r_foot_contact half_z=0.00225 pos_z=-0.02375 friction0=1.6
  …_gate_f_h0_h2.xml  → LOAD_OK; same contact + H0 sites present
Live md5s KEPT: M145 fc94709c… · gate_f 59cc408e… · h0 ad9a1817…
H2 body == gate_f after model-name + foot size/pos normalize: True
```

Load path for Prefer FAIL A/B **after unlock only**:

```bash
MUJOCO_GL=glfw .venv/bin/python <score_or_walk> \
  --model mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_h2.xml
```

---

## 6. What Hardware is waiting on

1. Dave freeze unlock for companion contact-height A/B.  
2. AI Prefer FAIL cospec naming H2 companion-first (criteria KEPT).  
3. Until then: T5-D1 keeps Prefer FAIL on live freeze; H2 stays draft on disk.
