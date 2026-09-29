# HARDWARE SYSTEM DESIGN BRIEF — Dronable Proto (Path A)

**Product:** Cheap biped, AiNex-class (~415 mm).  
**Demo (March 2027):** Walk a room + open low prop lever at **250–300 mm** AFF (NOT full UK door height).  
**Ship-to:** SE1 4AG London.  
**Budget:** Push under **$1k**; hard ceiling **$1.5k**. Path A UK-landed all-in **~$1,050–1,100** (to **~$1.14k** if courier spikes).  
**Owner:** Founding Hardware (mechanical + BOM framing). Controls/AI interfaces stated below. Coordinated with Founding Manufacturing Lead buy sheet.  
**Status:** Path A **FROZEN** until Dave locks **Standard** Mon + live Hiwonder checkout + kit-matched MJCF/drawings. **China CM scope for unit one = NONE.**  
**Date:** Sun 27 Sep 2026 (London / BST).  
**Sources:** hiwonder.com/products/ainex; HX-35H / HX-35HM / HX-12H datasheets; Pi 5 power benches; Manufacturing buy sheet [Google Doc](https://docs.google.com/document/d/1w6tZ-UTNqlveMHpFS6vtwDNMyCJV9o0iXAH0SN3DL_Q/edit); Controls input (envelope/DoF/torque/gait/teleop/safety — Sep 2026); AI input (on-body Pi scope, perception cap, sensor must/must-not, Pi power — Sep 2026); `ASSUMPTIONS.md`.

**CRITICAL (Manufacturing):** Body/leg actuators are **Hiwonder HX serial bus only**. Do **not** list or buy Feetech STS/SMS (or any non-HX bus family) for unit one.

---

## OPEN FOUNDER DECISIONS (call required)

| ID | Decision | Why it blocks | Options / note |
|----|----------|---------------|----------------|
| **OD-1** | **Lock AiNex Standard (vs Starter)** | Hands DOF (24 vs 20); lever grasp needs Standard hands | Path A / Manufacturing preference = **Standard 24DOF + hands**; Starter only if founder forces under-$1k and accepts no hands |
| **OD-2** | **Live Hiwonder checkout** (SKU + Pi 5 4GB) | Exact cart line not locked; list **$729.99** on hiwonder.com | Monday live checkout + cart screenshot; prefer **Pi 5 4GB** |
| **OD-3** | **Under-$1k vs Path A cart** | UK-landed already **~$1.05–1.10k** (to ~$1.14k courier) | Accept Path A band, or Starter / cut UK thin-fab — founder call |
| **OD-4** | **Battery vs tether for March demo** | Runtime vs trip hazard | Hardware **recommends battery primary + bench tether available** (§6) |
| **OD-5** | **Kit-matched MJCF + fab drawings gate** | Controls blocker: no credible walk / FOV rebind / UK fab until kit-matched MJCF; exact DoF/ID map OPEN until live SKU + MJCF | Freeze fab + treat walk numbers as **control model only** until kit receipt or Hiwonder STEP + founder OK |
| **OD-6** | **Pi 5 4GB vs 8GB on cart** | Vision/voice off-board March → 4GB preferred (AI: no on-Pi LLM/NN) | Prefer **4GB** unless price delta trivial |

Inline OPEN flags also appear in §§1–6.

---

## PATH A FREEZE RULES

1. **Buy** AiNex **Standard** whole kit (24DOF + hands, Pi 5 prefer 4GB) + listed HX spares + UK eyes parts — per Manufacturing buy sheet.
2. **UK fab THIN only** (not China CM): face/eye mount, ankle bumpers, prop door+lever 250–300 mm AFF — **after** drawings match kit envelope.
3. **China CM scope for unit one = NONE.** No CM RFQs. Prior MuJoCo capsule previews failed physics — see §3.
4. **Do NOT buy:** NVIDIA Orin, 2nd Pi, Hailo, F/T ankles, depth cam, LiDAR, LeRobot open biped, custom legs, Waveshare Pico-ResTouch, TonyPi, **Feetech STS/SMS** (or any non-HX bus spare).
5. Frozen until: Dave locks Standard Mon + live checkout + kit-matched MJCF/drawings (**OD-1, OD-2, OD-5**).

**Kit envelope (Manufacturing / quote / model / fab against this):**

| Item | Spec |
|------|------|
| Size | **193 × 135 × 415 mm** |
| Mass | **2.45 kg** Standard (**2.25 kg** Starter) |
| DOF | **24** + hands (Standard); Al alloy frame |
| Compute | Pi + ROS (prefer Pi 5 4GB on cart) |
| Walk | Rated / control-model ceiling ~**21 cm/s** (*Controls:* not a shippable walk claim — see §2.6) |
| Cam | **120° / 1 MP**, 2DOF head |
| IMU | **9-axis** |
| Battery | **11.1 V 3500 mAh 5C LiPo** |
| Bus | UART serial, baud **115200**, position / temp / voltage feedback |

---

## 1. ELECTRONICS

### 1.1 Architecture (Path A)

```
[11.1 V 3500 mAh 5C LiPo] ──► kit power distribution / bus board
        │
        ├──► Hiwonder HX serial bus (HX-35H · HX-35HM · HX-12H daisy-chain)
        │         └── joint encoders + temp/voltage feedback IN SERVO
        ├──► Raspberry Pi 5 (prefer 4GB)  ← on-body ONLY (see AI split below)
        ├──► kit IMU (9-axis)
        ├──► 2DOF head + 120° / 1 MP camera (kit)
        └──► [ADD UK] eyes: bare 2.8" SPI IPS ILI9341 + MG90S ×2 (lid/aim) ≤1–2 W display budget

On-body Pi 5 4GB (AI): bus I/O, 50 Hz PD, IMU, E-stop, neck_pitch, Xbox teleop, eye LCD.
  — NOT on-Pi: LLM, NN walk, Orin, Hailo.

Off-board (AI / March): ego preview, OpenCV / lever assist, imitation train, voice —
  Wi-Fi stub latency **50–80 ms**. Perception budget ≤~2 cores / 1–1.5 GB.
Teleop: Xbox → Pi → joint Δq.
```

| Layer | Path A action | Custom vs OTS |
|-------|---------------|---------------|
| Compute | **INHERIT** kit Pi 5 4GB; **no Orin, no Hailo, no 2nd Pi** | OTS (kit) |
| Bus / drivers | **INHERIT** kit bus + **Hiwonder HX** servos only | OTS |
| Sensors | **INHERIT** kit IMU + servo encoders + head cam | OTS |
| Eyes | **ADD** bare 2.8" ILI9341 + MG90S ×2 + UK face mount | UK buy + UK fab |
| Power | **INHERIT** 11.1 V pack; optional bench tether jack | OTS |
| Harness | **INHERIT**; ADD LCD SPI + MG90S PWM leads | thin ADD |

**Do not replace** bus protocol, servo ID map, or power tree until kit is in-hand.

### 1.2 Compute *(AI input)*

| Item | Spec | Unit cost USD | Availability | Notes |
|------|------|---------------|--------------|-------|
| Kit SBC | **Raspberry Pi 5**, prefer **4GB** | Included in kit ($729.99 list) | China → UK (Hiwonder free ship >$499 to UK) | On-body: bus I/O, **50 Hz PD**, IMU, E-stop, neck_pitch, Xbox, eye LCD. **Not** on-Pi LLM / NN walk / Orin. |
| Storage | Kit 32 GB microSD | Included | — | Optional A2 upgrade later if needed |

**Do NOT buy a 2nd Pi, Orin, or Hailo for unit one.**

**On-body vs off-board (AI):**

| Where | Runs March |
|-------|------------|
| **On-body Pi 5 4GB** | HX bus I/O, 50 Hz PD, IMU read, E-stop, neck_pitch, Xbox → Δq, eye LCD |
| **Off-board** | Ego preview, OpenCV / lever assist, imitation training, voice |

**Perception cap (AI):** ≤~**2 cores / 1–1.5 GB** on any assist path; cam stream **640×480 ≤30 Hz**; stub latency **50–80 ms**; eyes display ≤**1–2 W**.

**FOV / cam Z / −16° look-down:** rebind **only after kit-matched MJCF** (AI + Controls) — do not lock depression angles from placeholder geometry.

**Power (AI-cited Pi continuous):**

| Mode | Pi 5 draw |
|------|-----------|
| Idle | **2.7–3.5 W** |
| Gait + bus I/O + IMU | **5–8 W** |
| + OpenCV assist path | **7–10 W** |
| **Keep continuous on-body** | **≤8–10 W** |

### 1.3 Power distribution

| Rail | Source | Loads |
|------|--------|-------|
| ~11.1 V (3S LiPo) | Kit 3500 mAh 5C | HX bus servos |
| 5 V | Kit regulator / Pi path | Pi, camera, MG90S (4.8–6 V) |
| 3.3 V | Pi rails | ILI9341 logic (module often 3.3/5 V tolerant) |

Optional: bench tether jack for lab only. Do not redesign BEC until measured current.

### 1.4 Motor drivers / bus — Hiwonder HX only

AiNex uses **Hiwonder HX serial bus servos** (UART **115200**, position/temp/voltage feedback). **Unit one = HX family only. No Feetech SKUs.**

| SKU | Role | Torque @ 11.1 V | Speed | Encoder / range | Unit $ | Connector |
|-----|------|-----------------|-------|-----------------|--------|-----------|
| **HX-35HM** | Hip Z (magnetic) | 35 kg·cm stall / 25 kg·cm rotation | 0.19 s/60° | 12-bit mag, 0–360° | $35.99 (kit-included; spare only if fail) | 5264-3P |
| **HX-35H** | Leg / high-load | 35 kg·cm stall / 25 kg·cm rotation | 0.18 s/60° | Pot, 0–240°; metal gear; **9–12.6 V** | **$18.99** | 5264-3P |
| **HX-12H** | Hands / light joints | 12 kg·cm locked | 0.2 s/60° | Pot, 0–240° | **$16.99** | PH2.0 3P (≠ 5264 — use Hiwonder adapter if needed) |

- Kit family on robot: **HX-35H · HX-35HM (mag encoder on hip Z) · HX-12H**.
- Kit bus board: **INHERIT**. Do not rewire to another vendor protocol.

**Eyes (UK buy — PWM micro, not bus):** MG90S ×2 for eye lid/aim (~£2.65–7 UK; ~$3–9 ea EST). Not HX; separate 5 V PWM from Pi.

### 1.5 Sensors *(AI must / must-not)*

| Sensor | Path A | Spec | Cost | Notes |
|--------|--------|------|------|-------|
| Joint encoders | **MUST INHERIT** (in HX) | Position + temp + voltage | — | Primary proprioception (AI + Controls) |
| IMU | **MUST INHERIT** kit 9-axis | Gyro + accel + compass; **100–200 Hz** target | — | Balance later in teleop→autonomy path |
| Head camera | **MUST INHERIT** | 120° HFOV, 1 MP, 2DOF head; stream **640×480 ≤30 Hz**, **50–80 ms** stub latency | — | FOV / Z / −16° rebind after kit-matched MJCF only |
| Eyes display | ADD UK | Bare **2.8" SPI IPS ILI9341** | **~$9.50** | ≤1–2 W; **not** Waveshare Pico-ResTouch |
| Ankle bumpers | ADD UK fab | Passive bumper / soft contact | thin fab | Controls safety; **not** F/T sensors |

**Do NOT buy (AI):** F/T ankles, depth camera, LiDAR, Orin, Hailo, 2nd Pi.

### 1.6 Wiring harness

- **INHERIT** HX daisy-chain (3-wire UART bus @ 115200).
- Respect **5264-3P** (HX-35*) vs **PH2.0** (HX-12H).
- **ADD:** SPI flex to ILI9341; two MG90S PWM leads into face mount; strain-relief at ankle bumpers.
- Label spare HX IDs before install. No hot-plug under load.
- E-stop path: **hard stop = kill bus** (Controls safety) — inherit kit kill / power cut; do not invent a second bus.

### 1.7 Thermal

- HX-35* stall ~3 A @ 11.1 V → hip heat (HX-35HM Al shell helps).
- Pi 5: kit heatsink/fan as supplied; trunk airflow; keep continuous on-body ≤8–10 W (AI).
- ILI9341 + MG90S: ≤1–2 W display budget; negligible vs bus.
- Abort on kit servo temp alarms.

### 1.8 ADD vs REPLACE vs FORBIDDEN

| INHERIT | ADD (unit one) | FORBIDDEN (unit one) |
|---------|----------------|----------------------|
| Pi 5 4GB, HX bus + all HX servos, IMU, cam, LiPo, Al frame | Bare 2.8" ILI9341, MG90S ×2, UK face/eye mount, ankle bumpers, prop lever; HX-35H ×2 + HX-12H ×1 spares | Orin, Hailo, 2nd Pi, F/T ankles, depth, LiDAR, Feetech STS/SMS, custom legs, LeRobot open biped, TonyPi, Waveshare Pico-ResTouch, China CM parts |

---

## 2. MOTION APPROACH

*(Controls input integrated below; Manufacturing SKUs / kit envelope unchanged.)*

### 2.1 Named stack

**Approach:** Model-based **open-loop / PD gait @ 50 Hz** + **Xbox teleop assist** → IMU balance later → learned residual / imitation **later** (not March).  
**Stack Controls prefers:** kit **WonderROS / AiNex SDK**; teleop logging compatible with **LeRobot** workflows later — **do not buy** a LeRobot open biped platform. **Not** on-Pi NN for first walk.

**Why (March 2027 / Pi / cost):** Pi sustains 50 Hz HX bus PD within on-body ≤8–10 W; kit ships IK/action tools; teleop covers lever; residual only after kit-matched physics (§3).

### 2.2 DoF map *(Controls — exact map OPEN until live SKU + MJCF)*

| Group | DoF | Actuator family (kit) |
|-------|-----|------------------------|
| Legs | **6 + 6** | HX-35H / HX-35HM (hip Z mag) |
| Waist + head | **1 + 2** | Kit HX (map OPEN) |
| Arms | **3 + 3** | Kit HX (map OPEN) |
| Hands | Standard hands | **HX-12H** |
| **Total** | **24** (Standard) | — |

**OPEN:** exact joint↔ID↔SKU map frozen only after live checkout + kit-matched MJCF (**OD-2, OD-5**). Do not invent SKUs.

### 2.3 Teleop → autonomy path *(Controls)*

1. **Bring-up:** receipt; stock kit walk; map HX IDs; Xbox → Δq (rate-limit ~1.5 rad/s, deadband ~0.08).
2. **Open-loop PD @ 50 Hz:** hips/knees **kp~40 kd~1.0**; ankles **kp~25 kd~0.6**; arms **kp~20 kd~0.4**; torque clip **0.6×stall**.
3. **IMU balance later** (not first open-loop walk gate).
4. **Teleop lever:** prop lever 250–300 mm AFF with Standard hands; neck_pitch look-down (~−16° approach / ~−20° grasp — **rebind after kit-matched MJCF**).
5. Soft-stop = **hold last q**; hard-stop = **kill bus**.
6. Autonomy stub: scripted approach + open-loop grasp; vision off-board optional (OpenCV / lever assist).
7. **Imitation later** from teleop logs — still on **this** HX body; train off-board (AI).

### 2.4 What control must learn (Hardware constraints)

| Capability | Hardware provides | Control learns |
|------------|-------------------|----------------|
| Stand / walk | HX torque, slew, friction feet, IMU | Timing, COM, foot placement |
| Turn | Hip yaw / HX-35HM on hip Z | Turning radius vs ≤21 cm/s |
| Lever open | Standard hands (HX-12H), arm reach to 250–300 mm, neck look-down | Approach, grasp via load/temp feedback |
| Fall safe | Torque clip 0.6×stall, temp alarm, soft-stop hold q, hard-stop kill bus, ankle bumpers | IMU tip detect → freeze |

### 2.5 Actuator requirements (~2.45 kg / 415 mm) *(Controls torque model @ 11.1 V)*

| Joint class | Servo | Stall | Working clip (×0.6) | Slew | Backdrivability |
|-------------|-------|-------|---------------------|------|-----------------|
| Hip / knee / ankle | HX-35H / HX-35HM | **35 kg·cm = 3.43 N·m** | **±2.1 N·m** | **0.18 s/60°** (HX-35H); HX-35HM same + mag | **POOR** — no impedance walking March |
| Hands / light | HX-12H | **12 kg·cm = 1.18 N·m** | **±0.7 N·m** | ~0.2 s/60° | **POOR** |
| Eyes (cosmetic) | MG90S | ~0.18–0.22 Nm class @ 4.8–6 V | — | ~0.1 s/60° | PWM micro; not locomotion |

**Soft stop = hold q.** Position servos only for March — **no** impedance / force walking claim.

**Contact / gait model *(Controls)*:**

| Parameter | Control model (not shippable claim) |
|-----------|--------------------------------------|
| Foot | Flat foot |
| Double support | ~**0.1 s** |
| Loop | **50 Hz PD** |
| Gait period T | **0.5–0.6 s** |
| Step length | **≤0.06 m** |
| Clearance | **0.025–0.04 m** |
| Speed ceiling | **≤21 cm/s** (kit / control model) |
| Gains | kp **40 / 25 / 20**; kd **1.0 / 0.6 / 0.4** (hip-knee / ankle / arm) |
| Mass / COM | **2.45 kg**; COM ~**0.22–0.24 m** until weighed |
| Contact | Unilateral; friction feet — no skate |

**Dynamics body must obey:** command rates respect **HX slew** — no cartoon speeds; heavy HX at joints.

### 2.6 Safety *(Controls)*

| Layer | Spec |
|-------|------|
| Torque | Clip **0.6×stall** (±2.1 N·m HX-35*; ±0.7 N·m HX-12H) |
| Soft stop | Hold last **q** |
| Hard stop | **Kill bus** |
| Mechanical | Ankle bumpers (UK fab) |

### 2.7 Honesty / rejection *(Controls)*

- **Current walk: REJECTED** as a shippable demo claim.
- Numbers above are a **control model** for bring-up and MJCF gates — **not** a proven kit walk.
- **Next prove on kit-matched MJCF** (Hardware blocker **OD-5**).
- Placeholder / capsule physics already failed (§3.3) — do not cite soft-pass MP4.

### 2.8 Lever task

- Prop lever **250–300 mm** AFF — within 415 mm biped reach; **not** UK full door handle height.
- Standard hands required (**OD-1**).
- Ankle bumpers protect kit feet on approach.

---

## 3. PHYSICAL MODELS

*(Controls input; Manufacturing envelope preferred for size/mass.)*

### 3.1 Leg dynamics (kit-matched)

| Parameter | Requirement |
|-----------|-------------|
| Size / mass | **193 × 135 × 415 mm**; **2.45 kg** Standard *(Manufacturing)* |
| COM | ~**0.22–0.24 m** height class until weighed *(Controls)* |
| DOF legs | **6+6**; HX-35H / HX-35HM on hip Z; full 24-DoF map OPEN until live SKU + MJCF |
| Hip width / foot size | **Measured kit** — not capsule guess |
| Joint limits | HX-35H 0–240°; HX-35HM 0–360°; HX-12H 0–240° mapped to robot zero |
| Ground contact | Flat foot; soft contact + friction; double support ~0.1 s; no ice skate |
| Actuator model | HX position servo + PD + **slew limit** + torque saturations 0.6×stall (§2.5); backdrivability **POOR** |
| Gait bounds | T=0.5–0.6 s; step ≤0.06 m; clearance 0.025–0.04 m; speed ≤21 cm/s |

### 3.2 Minimum sim fidelity before any MP4 is credible

Kit-matched MJCF **must** include *(Controls blocker = Hardware OD-5)*:

1. Real link lengths (caliper or Hiwonder STEP after order).
2. Correct hip width + orthogonal hip axes (kit coincident XYZ hip design).
3. Foot geom = sole; +Y forward / Z-up consistent.
4. Masses within ~10% of **2.45 kg**; COM check vs 0.22–0.24 m class.
5. Joint axes matching Standard (no pitch/roll swap); exact ID map from live kit.
6. Contact that does **not** skate (flat foot, DS ~0.1 s).
7. **HX-class** slew + torque clips (stall×0.6).
8. Cam / neck_pitch frame for −16° rebind only after this MJCF exists.

### 3.3 What failed in placeholder capsule v5-class

| Failure | Symptom | Lesson |
|---------|---------|--------|
| Wrong feet | Patch ≠ sole → tip/skate | Measure feet first |
| Skate | Low μ / bad contact | Stick-slip check in log |
| Cartoon speeds | Unbounded actuators | Cap ≤~21 cm/s + HX slew |
| Axis swap | Pitch → lateral mush | Axis unit test vs kit |
| Soft-pass MP4 | Assist / B-frames lie | CSV foot-z / COM + all-intra |

### 3.4 Explicit rejection criteria

**REJECT** if: link/hip width >5% off measured kit; mass outside ~2.2–2.6 kg without note; sustained COM speed >0.35 m/s without HX slew math; feet slide >½ foot/step; axes swapped; torque > stall; video-only proof; placeholder geometry used for any fab/CM quote; **any walk claim without kit-matched MJCF** *(Controls: current walk REJECTED)*.

**ACCEPT** only kit-matched MJCF + logged metrics (+ optional video).

---

## 4. ACTUAL PARTS

### 4.1 BUY list (unit one) — Manufacturing lock

**Buy sheet:** https://docs.google.com/document/d/1w6tZ-UTNqlveMHpFS6vtwDNMyCJV9o0iXAH0SN3DL_Q/edit

| # | Item | Spec / search | Unit USD | Qty | Notes |
|---|------|---------------|----------|-----|-------|
| 1 | **AiNex Standard** | Hiwonder AiNex Standard, **24DOF + hands**, **Pi 5 prefer 4GB** | **$729.99** list | 1 | **FLAG OPEN** — confirm live checkout Mon (**OD-2**). Envelope § above. Assembled ship. |
| 2 | **HX-35H** spare | Hiwonder HX-35H; 35 kg·cm; 9–12.6 V; metal gear; 5264-3P | **$18.99** | **2** | Kit-family spare |
| 3 | **HX-12H** spare | Hiwonder HX-12H; 12 kg·cm | **$16.99** | **1** | Kit-family spare |
| 4 | **MG90S** | Metal-gear micro PWM | ~$3–9 ea UK | **2** | Eyes only |
| 5 | **2.8" SPI IPS ILI9341** | Bare module (not Pico-ResTouch) | **~$9.50** | **1** | Eyes display |

**Kit family on robot (included, not separate buy):** HX-35H · **HX-35HM** (magnetic encoder on hip Z) · HX-12H.

### 4.2 Do NOT buy (unit one)

- NVIDIA Orin / Jetson  
- Hailo / other on-body NN accelerators  
- 2nd Raspberry Pi  
- F/T ankle sensors  
- Depth camera / LiDAR  
- LeRobot open biped / alternate full robot platform  
- Custom legs / China CM structure  
- Waveshare Pico-ResTouch  
- TonyPi  
- **Feetech STS / SMS / any non-HX bus servo SKUs**

### 4.3 UK fab bits (thin — after envelope match) — NOT China CM

| Part | Spec | Process | EST USD |
|------|------|---------|---------|
| Face / eye mount | Holds ILI9341 + MG90S ×2; mates measured head | UK FDM/SLA | 15–40 |
| Ankle bumpers | TPU over measured sole | UK FDM TPU | 10–25 / pair |
| Prop door + lever | Lever **250–300 mm** AFF | UK laser + print | 30–80 |

**China CM scope = NONE** for unit one.

### 4.4 Assembly notes (lower body)

- Path A **does not fab legs** — kit Al + HX are the lower body.
- Spares: mid-set HX before install; match kit ID map.
- Ankle bumpers: ≤2–3 mm sole raise without sim update.
- Eyes: MG90S on 5 V PWM; keep mass low on head.

---

## 5. ASSEMBLY

### 5.1 Prototype assembly (kit-first)

1. **Unbox / QC** — photo, weigh (~2.45 kg), measure 193×135×415, HX ID map, battery V, Pi boots.
2. **Bench power** — charge LiPo; stand-mount first power; IMU / cam / app OK.
3. **Stock motion** — kit walk / action groups; log current (*Controls: stock walk ≠ accepted demo claim*).
4. **Dev** — SSH; Xbox bind; 50 Hz PD bridge (Controls); E-stop = kill bus verified.
5. **Spare HX** — ID + mid-set HX-35H ×2 / HX-12H ×1 on bench before need.
6. **Measure for UK fab** — head face, ankle outline, reach to 275 mm lever; feed kit-matched MJCF.
7. **UK fab install** — bumpers → face/eye mount → ILI9341 + MG90S → prop lever.
8. **Demo** — room walk + teleop lever after MJCF gate; tether jack for lab only.

### 5.2 Jigs / fixtures

| Jig | Purpose |
|-----|---------|
| Hangar / stand | Power-on without fall |
| Footprint card | Bumper CAD |
| Lever height gauge | Fixed 275 mm AFF |
| Inline current meter | Walk power log |
| Hip-width calipers | MJCF check |

### 5.3 What Manufacturing / UK fab needs (not China CM)

**Unit one: no Chinese CM quote.** UK shop gets:

1. Face/eye mount STEP (LCD cutout + MG90S bosses) after measured head.
2. L/R ankle bumper STEP after measured foot.
3. Prop door + lever @ 250–300 mm AFF.
4. Qty 1–2; ship SE1 4AG; mass + mate ±0.5 mm QC.

**Hardware drawings Manufacturing needs:** kit-matched orthographics, mate callouts, mass budget — **after** OD-5 gate. No placeholder RFQ.

---

## 6. COST & POWER

### 6.1 BOM (Path A — Manufacturing-aligned)

| Line | Item | USD |
|------|------|-----|
| A | AiNex Standard (Pi 5 prefer 4GB) | **729.99** list |
| B | UK import VAT/duty/carrier on kit | rolled into landed |
| C | **Kit UK-landed SE1 4AG all-in** | **~$1,050–1,100** (to **~$1.14k** if courier spikes) |
| D | HX-35H ×2 | 37.98 |
| E | HX-12H ×1 | 16.99 |
| F | MG90S ×2 + ILI9341 bare | ~15–25 EST |
| G | UK thin fab (face + bumpers + prop) | 55–145 EST |
| | **Path A honest total** | **~$1.15–1.35k EST** with spares+fab |
| | **Kit cart alone** | **~$1.05–1.14k** |

**FLAG (OD-3):** Under-**$1000** is **not** honest for Standard UK-landed. Accept Path A band or drop to Starter / defer fab.

Hard ceiling **$1.5k** — buy list above stays inside if courier behaves.

### 6.2 Power draw

| Subsystem | Idle | Active walk |
|-----------|------|-------------|
| Pi 5 (AI continuous on-body ≤8–10 W) | **2.7–3.5 W** | **5–8 W** (gait+bus+IMU); **7–10 W** if +OpenCV |
| HX bus (24) | 2–5 W EST | **15–40 W EST** avg |
| Camera | 0.5–1 W | 0.5–1 W |
| IMU | <0.1 W | <0.1 W |
| Eyes (ILI9341 + MG90S) | 0.2–1 W | **≤1–2 W** |
| **System** | **~6–12 W** | **~25–50 W EST** |

Battery: 11.1 V × 3.5 Ah ≈ **38.9 Wh** → practical demo blocks **20–40 min** EST at walk power.

### 6.3 Battery vs tether

**Recommend: battery primary** for March room walk + lever; **bench tether** for lab bring-up only. Spare charged pack if Hiwonder sells replacement. Kit 3500 mAh is baseline — do not undersize.

---

## AI / CONTROLS INTERFACE

| Interface | Spec | Source |
|-----------|------|--------|
| Cam | Kit 120° / 1 MP; stub **640×480 ≤30 Hz**; 2DOF head; neck_pitch look-down — **FOV/Z/−16° rebind after kit-matched MJCF only** | AI |
| Latency | Off-board vision **50–80 ms** stub | AI |
| Perception budget | ≤~**2 cores / 1–1.5 GB** | AI |
| On-body Pi | Bus I/O, 50 Hz PD, IMU, E-stop, neck_pitch, Xbox, eye LCD — **not** LLM/NN/Orin | AI |
| Off-board | Ego preview, OpenCV/lever assist, imitation train, voice | AI |
| Control | Joint cmds **50 Hz** on HX bus @ **115200**; torque clip 0.6×stall | Controls |
| Soft / hard stop | Hold **q** / **kill bus** | Controls |
| Teleop → autonomy | Bring-up → open-loop PD → IMU balance later → teleop lever → imitation later | Controls |
| Stack | Kit SDK / WonderROS; LeRobot **logging** later — **not** buy LeRobot biped | Controls |
| Eyes | Bare 2.8" ILI9341 ≤1–2 W + MG90S ×2 | Manufacturing / AI |
| Blocker | **Kit-matched MJCF** before walk claim / fab / FOV lock | Controls + Hardware |

---

## REFERENCES

- https://www.hiwonder.com/products/ainex — envelope, $729.99 list (fetched 27 Sep 2026).
- https://www.hiwonder.com/products/hx-35h — $18.99; 35 kg·cm; 9–12.6 V; 5264-3P.
- https://www.hiwonder.com/products/hx-35hm — hip mag encoder.
- https://www.hiwonder.com/products/hx-12h — $16.99; 12 kg·cm.
- Manufacturing buy sheet: https://docs.google.com/document/d/1w6tZ-UTNqlveMHpFS6vtwDNMyCJV9o0iXAH0SN3DL_Q/edit
- Controls input (Sep 2026) — envelope/DoF/torque/gait/teleop/safety/honesty; blocker = kit-matched MJCF.
- AI input (Sep 2026) — on-body Pi scope, perception cap, sensor must/must-not, Pi power, FOV rebind gate.
- `ASSUMPTIONS.md` — PD / gait / cam depression numbers.

---

*End of Hardware System Design Brief. Feetech / non-HX bus SKUs intentionally absent for unit one. Controls walk numbers = control model only until kit-matched MJCF.*
