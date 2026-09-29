# Gate Door v1 — AI criteria (hospital push/pull)

**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (push-face honesty)  
**When:** Tue 29 Sep 2026 ~21:56 BST  
**Plant (post Dave ACK only):** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040`  
**Live until ACK:** companion `…_gate_f.xml` md5 `59cc408eda07037a58f92ad27da045d6` — **do not score Door v1 against live lever plant**  
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
| 3 | Push engagement | Declared contact pair `l/r_hand_contact` × `door_panel_push_face` for ≥**0.3 s** during open attempt |
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

- Contact: `l_hand_contact` / `r_hand_contact` × `door_panel_push_face`  
- Sites: `door_panel_push_site`, `door_panel_site`  
- Open/closed: `door_panel_hinge`  
- **Not scored:** `door_lever` contact, `door_lever_hinge` angle, grasp-on-lever, latch  

## Escalation

Prefer FAIL on thin −X push face / side approach → Hardware **Option C** (full panel contact) new draft — not mid-score invent.  
FOV blindness → separate panel-center cam rebind honesty item post-SCORE — not mid-wall.

## Owner next

| Owner | Next |
|-------|------|
| Dave | ACK named plant swap (or hold) |
| Controls | No Door v1 score until install ACK; then score this criteria |
| Hardware | Hold freeze; Option C only on Prefer FAIL demand |
| AI | Same-turn stills/watch when Controls lands SCORE |
| MFG | No PO |
