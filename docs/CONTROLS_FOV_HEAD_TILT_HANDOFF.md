# Controls → Hardware handoff — FOV / `head_tilt` reconfirm

**When:** Mon 28 Sep 2026 (Europe/London)  
**For:** Hardware Monday checklist #1 (FOV / `head_tilt` reconfirm)  
**From:** Founding Controls  
**Scope:** SIM-ONLY geometry + command surface. **Vision OFF** walk / PPO. **No spend.**

---

## Locked claim (Controls)

Gate F **LOCKED TRUE** on companion plant. Numbers from `previews/ainex_walk/iterate/GATE_F_TABLE.json` / `GATE_F_CONTROLS_NOTE.md` / `GATE_F_AI_LOCK.md`.

| Tag | head_tilt (cmd / mean) | Key metrics | Video |
|-----|------------------------|-------------|-------|
| **F00** | **−16°** / −16.02° | tip 9.00; ego continuous **~8.52 s**; cam_horiz **~0.398 m** ∈ **[0.35, 0.45]**; opt Z ~0.371 m | `previews/ainex_walk/iterate/F00.mp4` |
| **F01** | **−16°** / −16.02° | tip 9.00; ego ~8.52 s; **dx +0.205**; **stx 0.039**; skate **false**; frozen Gate E ckpt | `…/F01.mp4` |
| **F02** | **−19°** / −19.00° | tip 9.00; ego ~8.72 s; cam_horiz ~0.297 m; hand Z **L 0.291 m** (band **0.25–0.30** AFF, kinematic) | `…/F02.mp4` |

**Ckpt (F01):** `previews/ainex_walk/iterate/learned_gate_e/ppo_gate_e_best.zip` · sha16 `9ffaa1a21b607bf6` — **not** modified for F.

---

## Command / plant surface (do not confuse)

| Item | Locked value |
|------|----------------|
| Look-down joint | **`head_tilt`** — **not** `neck_pitch` |
| Cam site / camera | `kit_cam_site` / `kit_cam` on **`head_tilt_link`** |
| Companion plant (F only) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Walk / Gate E plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` — **do not rescore Gate E on gate_f.xml** |
| Vision in walk / PPO obs | **false** — FOV reconfirm is plant/cam geometry + head command, **not** vision-in-loop |
| Lever prop | `door_lever` @ **0.275 m** AFF (visual/marker, contype 0) |

Hardware cam note: optical Z ≈ **0.380 m** foot-grounded (mesh-derived ±15 mm). Controls F00 stand measured opt Z ~0.371 m under COM_Z stand — within honesty band.

---

## What Hardware should reconfirm vs what Controls already locked

| Already locked (Controls) | Hardware reconfirm (Monday #1) |
|---------------------------|--------------------------------|
| F00–F02 PASS on companion; `head_tilt` −16° @ ~0.4 m / −19° @ ~0.3 m | Cam local pose on `head_tilt_link` still matches FOV rebind intent |
| Lever stays in ego FOV ≥2 s at stand/approach | `kit_cam` fovy / HFOV claim vs physical kit (sim fovy 104.82° ↔ kit HFOV 120° claim) |
| Vision **not** in Gate E residual obs | Optical Z vs FOV rebind ~0.380 m when foot-grounded |
| Gate E plant contact/HX untouched by companion | Companion diff is header + materials + door + cam/site **only** — no silent contact edit |
| Assist/freeze OFF | Sign of `head_tilt` axis `(0,−1,0)`: negative depresses optical axis toward lever |

**Out of scope for this handoff:** Pi detector, UK door-handle height, sole/STEP refresh, spend / Path A, reopening Gate E falsifiers.

---

## Refs

- Criteria: `docs/GATE_F_AI_CRITERIA.md`, `docs/GATE_F_HARDWARE_CAM_LEVER.md`, `docs/FOV_REBIND_AINEX_HIWONDER.md`
- Controls freeze sheet: `docs/CONTROLS_FREEZE_STATUS_MONDAY.md`
- Artifacts: `previews/ainex_walk/iterate/GATE_F_*`, `F00.mp4`–`F02.mp4`
