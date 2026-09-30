# Gate Door v1 — AI criteria (hospital push/pull)

**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (push-face honesty)  
**When:** Tue 29 Sep 2026 ~21:56 BST  
**Live plant (Dave ACK ~11:15 BST Wed 30 Sep — INSTALLED, Option C):** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml` md5 `6a3d4a70d4797b806dcc2580f46468aa`  
**Option B archive:** `…_gate_f_optb.xml` md5 `ddf084cdac71cb0998aa6a44a65594c0` (bytes unchanged — do not score Door v1 against it; not live)  
**Option A archive:** `…_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040` (bytes unchanged — do not score Door v1 against it; not live)  
**Archived lever-era:** `…_gate_f.xml` md5 `59cc408eda07037a58f92ad27da045d6` (history only — do not score Door v1 against it)  
**Bars:** numeric Prefer FAIL bars below are **unchanged** (not softened). Contact geom on the installed plant is hand × `door_panel` bit 2; `door_panel_push_face` is visual 0. This criteria file does **not** claim a Door SCORE. Soft-pass **OFF**. Park **OFF**.  
**Scoring rule (Dave):** multi-try Prefer FAIL Door — do **not** ping FAIL on first miss; report FAIL only after repeated setbacks; on success report + reassess. Soft-pass **OFF**.  
**Soft-pass:** **OFF** · **Spend:** none  
**Cospec:** `docs/GATE_DOOR_V1_AI_COSPEC.md` (NO-VETO) · Controls `docs/GATE_DOOR_V1_CONTROLS_COSPEC.md`

## Goal

Prove the kit can **detect door closed** and **push or pull the panel open** in front of itself (hospital-style) on the push/pull companion — **no** lever/knob torque, **no** latch.

Historical Gates **F–P LOCKED TRUE (sim)** remain **lever-era history**, not this March door claim.

## Hard thresholds (Prefer FAIL)

| # | Prove | Pass bar |
|---|-------|----------|
| 1 | Closed detect | Start with `|door_panel_hinge| ≤ 2°` (0.035 rad). Soft-pass never on “looks closed.” |
| 2 | Approach / reach | Working stand; hand within useful reach of `door_panel_push_site` (document Δ); tip≥8 if stepping |
| 3 | Push engagement | Declared contact pair `l/r_hand_contact` × `door_panel` (bit 2; `door_panel_push_face` visual 0) for ≥**0.3 s** during open attempt |
| 4 | Open | `|door_panel_hinge|` abs ≥**25°** within plant ±30°; tip≥8 |
| 5 | Hold (demo) | Hold open ≥**1.0 s** @ panel ≥**20°**; tip≥8 |
| 6 | Multi-bout | **2/2** consecutive closed→push/pull→open≥25°→hold in one continuous score run (reset OK between) |
| 7 | Plant honesty | Push fork md5 lock after install; **no** lever contact scored as open; **no** panel actuator / scripted qpos; **no** latch / 90° / walk-through claim |
| 8 | Video | Continuous MP4 both bouts; HUD `door_panel_hinge` + mark closed / contact / open |

**Optional first smoke (not March unlock alone):** single bout open ≥**10°** with honest push contact — Prefer FAIL note only; March bar stays ≥25° / 2/2.

## Hard falsifiers

- Soft-pass  
- Scoring `door_lever` contact or `door_lever_hinge` angle as door-open  
- Claiming latch / 90° / walk-through / UK height / investor room-walk  
- Installing or scoring against push fork **before** Dave ACK  
- Live md5 edit without named ACK  
- Panel scripted / panel actuator / undeclared assist / freeze / xfrc  
- Vision injected into walk loop  
- Tip&lt;8 on stepping rows when locomotion is in the prove  

## Not Door v1

Lever grasp/rotate · knob torque · latch · hinge-range bump toward 90° · Gate Q locomotion Prefer FAIL wall (orthogonal; bars KEPT) · H2 unlock · spend/PO.

## Scoring names (Controls)

- Contact: `l_hand_contact` / `r_hand_contact` × `door_panel` (bit 2). `door_panel_push_face` is visual 0 and is not the pair.  
- Sites: `door_panel_push_site`, `door_panel_site`  
- Open/closed: `door_panel_hinge`  
- **Not scored:** `door_lever` contact, `door_lever_hinge` angle, grasp-on-lever, latch  

## Escalation

Prefer FAIL on thin −X push face / side approach → Hardware **Option C** (full panel contact) new draft — not mid-score invent.  
FOV blindness → separate panel-center cam rebind honesty item post-SCORE — not mid-wall.

## Owner next

| Owner | Next |
|-------|------|
| Dave | Present for Prefer FAIL Door outcome (multi-try) |
| Controls | Multi-try Prefer FAIL Door SCORE vs this criteria |
| Hardware | Live freeze on Option C `gate_f_optc`; Option B archive; Option A archive |
| AI | Same-turn stills/watch when Controls lands SCORE |
| MFG | No PO |
