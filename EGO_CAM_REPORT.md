# Egocentric kit-cam FOV — report for Mon 29 Sep 2026

**run_id:** `2fe04e48e07d`  
**When:** Sun 27 Sep 2026 ~18:58 Europe/London (BST)  
**Path A:** geometry/FOV assumed AiNex-class — **buy decision still NOT locked / not unfrozen.**  
**Walk:** v9 walk_gait acceptance untouched (`neck_pitch` held **0** during gait).

---

## Locked AI numbers (this run)

| Item | Value |
|------|-------|
| Kit HFOV | **120°** (VFOV ≈ **104.8°** @ 640×480; MJCF `fovy=104.82`) |
| Cam mount Z (stand, pitch=0) | **0.385 m** |
| Lever Z | **0.275 m** |
| Pan-tilt | `head_yaw` + **`neck_pitch`** restored (no fab shim) |
| Default approach pitch | **−16°** |
| Depression to center lever | **−12.4° @ 0.5 m → −15.4° @ 0.4 m → −20.1° @ 0.3 m** |

---

## Required stills (AI overlay)

| # | Shot | File |
|---|------|------|
| 1 | pitch=**0** @ **0.4 m** — lever low in frame | `ego_cam/01_pitch0_dy0p40.png` |
| 2 | **−16°** @ **0.4 m** — lever centered | `ego_cam/02_pitch-16_dy0p40.png` |
| 3 | **−20°** @ **0.3 m** — grasp | `ego_cam/03_pitch-20_dy0p30.png` |

Side + FOV frustum companions: `side_01_*.png`, `side_02_*.png`, `side_03_*.png`.

**Controls Monday must-prove clip:** commanded look-down holds lever in-frame on approach  
→ `ego_cam/ego_approach_lookdown.mp4` (all-intra; frames in `ego_cam/clip_frames/`).

Ref centering table: `ref_center_dy0p5_p12down.png`, `ref_center_dy0p4_p15down.png`, `ref_center_dy0p3_p20down.png`.

Machine metrics: `ego_cam/metrics.json`, `ego_cam/run_id.txt`.

---

## Finding

Level head (`neck_pitch=0`) at 0.4 m places the lever **low** in the ego frame (elev ≈ −15° vs optical axis).  
Commanded **−16°** at 0.4 m centers it (elev_from_look ≈ +1.7°).  
At grasp (0.3 m), **≈−20°** keeps lever centered. Wide **120°** HFOV keeps the prop in horizontal FOV despite door offset (+X).

**Recommendation for Monday:** restore / keep **neck pitch** (already in MJCF); use commanded approach pitch **−16°**, deepen toward **−20°** near grasp. **Do not** fab a fixed downward shim for unit one.

---

## MJCF / script touchpoints

- `mujoco/dronable_v0.xml` — `kit_cam_site`, `kit_cam`, FOV frustum geoms, `neck_pitch` + `m_neck_pitch`
- `scripts/ego_cam_preview.py` — stills + clip generator
- `scripts/walk_gait.py` — `m_neck_pitch` in `ACT_NAMES`; gait holds `neck_pitch=0` (no acceptance regress)
- `ASSUMPTIONS.md` — FOV / cam Z / neck pitch / −16° approach
