# AI proposals — AiNex walk failure (kit-matched MuJoCo)

**When:** Sun 27 Sep 2026 ~20:39 Europe/London (BST)  
**Context:** quiet stand PASS; ankle CoP @ 50 Hz FAIL on disturb + walk; skate + tip ~2.3 s under HX clips; clean_walk false; Path A frozen.  
**Scope:** proposals only — no training runs, no spend.

## Ranked

### 1. Capture-point / ZMP regulator (kit sensors only)
- Mechanism: 50–100 Hz on IMU + HX-35HM + joint q; modulate hip/ankle + swing XY inside ±2.1 N·m; Xbox v_des.
- Helps: closes CoP loop that open-loop CPG lacks → less skate/tip.
- Effort: 1–2 weeks. Cost: $0. Owner: Controls (AI envelope).
- Falsifier: tip <2 s or skate >0.1 m/s after CP → Path A sensing insufficient.

### 2. Teleop→autonomy curriculum
- Gates: stand → disturb reject → single step → slow multi-step → kit-speed.
- Helps: stops rushing into FAIL cadence; early hardware vs controller falsify.
- Effort: days. Cost: $0. Owner: Controls + AI criteria.
- Falsifier: fail disturb reject → March teleop-shuffle + lever only.

### 3. CoP proxy from HX bus (optional FSR later)
- Mechanism: load/temp + kinematics; MuJoCo validate vs true contact.
- Helps: detect stance unload/skate on stiff servos.
- Effort: ~1 week. Cost: $0 (or ~$10–30 FSR). Owner: AI + Controls.
- Falsifier: proxy unusable → ankles blind without add-on sensing.

### 4. Vision off walk loop
- Cam + head_tilt −16° for lever only; walk is proprioceptive.
- Helps: avoid coupling; protect Pi ≤8–10 W.
- Effort: hours. Cost: $0. Owner: AI.
- Falsifier: non-flat venue needs vision footing → architecture wrong.

### 5. Anti-proposal: no on-Pi learned gait / no Orin for tip
- Tip@2.3 s is balance/contact, not missing a network.
- Cost avoided: $400–1k+. Owner: AI / Dave.
- Falsifier: 1+2 still fail → Path A walk claim wrong; pivot or teleop-only.

## Recommendation
Pursue **1+2 now**, **3** parallel, lock **4**, keep **5** as spend guard. Path A unlock still requires Dave assembly review + greenlight.
