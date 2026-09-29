# Gate Q AI research — T5-D hybrid (Prefer FAIL; soft-pass off)

**When:** Mon 28 Sep 2026 ~21:45 Europe/London (BST)  
**Owner:** Founding AI Scientist (+ Controls kit-facing twin)  
**Trigger:** Dave live — BD-class already does clean walk-back; research *how*; don't spin residual twins after T5-A/B/C Prefer FAIL.  
**Stance:** Bars KEPT (skate ≤0.08/0.18, clear_frac≥0.55, tip≥8, plant-cam ε). Soft-pass **off**. E7lock sha16 `9ffaa1a21b607bf6` kept until honest beat. Companion md5 `59cc408eda07037a58f92ad27da045d6`. Walk plant md5 `fc94709c84f5598d4474ecfc4bb41fdc`. Plant freeze until Dave OKs a *named* honesty ask. No Path A / spend.

**Aligns:** `GATE_Q_AI_SCORE_T5C.md` · `GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md` · `GATE_Q_RETREAT_RESEARCH_UNSTICK.md` · `GATE_Q_RETREAT_RESEARCH_T5.md` · `GATE_Q_AI_RESEARCH_T5.md`

---

## 0. Closed — do not reopen as twins

| Family | Disposition |
|--------|-------------|
| R1–R5 | Prefer FAIL / veto |
| S1 CSF50 / S2 ALIP / S3 REV-PHASE on frozen E7lock | Prefer FAIL |
| **T5-A** skate-primary residual | Prefer FAIL near-miss |
| **T5-B** ALIP/DCM+ankle **new weights** | Prefer FAIL near-miss |
| **T5-C** teacher-BC + multi-seed | Prefer FAIL (`ret_ok_both=false` all seeds) |

Lesson: clear_frac and bout0 `ret_ok` move; **both-bout skate + ε package** does not under residual-on-(near-)E7lock. T5-D must change the **outer architecture** or an honesty-named plant ask — not another Prefer FAIL BC/reward twin.

---

## 1. How serious platforms reverse (public evidence)

### 1.1 Atlas (Boston Dynamics) — templates + online MPC, not plant slide

| Public fact | Source | Transfer |
|-------------|--------|----------|
| Offline behavior **templates** (traj opt) + online **MPC** that adjusts force, posture, timing | BD blog *Flipping the Script* / *Picking Up Momentum* | Reverse is a **library behavior**, not mirrored forward residual |
| MPC predicts dynamics; stitches across behavior boundaries | same | Controllers deviate from template under slip / geom — still contact-honest |
| IHMC Atlas: **step timing + footstep location** adjust from Instantaneous Capture Point (ICP); CMP → ID WBC | arXiv:1703.00477 | Capture/ICP drives *when and where* next foot lands; CoP ankle strategy + step adjust |
| Earlier Atlas: footstep planner + LQR/QP whole-body | Kuindersma et al. AR 2015 | Safe footsteps first; QP tracks under contact constraints |

**Honest label:** BD does **not** publish a skate recipe. Public story = **reverse-native template/plan + MPC/WBC with capture/foot placement**. Transfer ≠ cancel↑ / softsole.

### 1.2 Cassie / Digit-class — ALIP-MPC foot placement; reverse = commanded −vx

| Public fact | Source | Transfer |
|-------------|--------|----------|
| ALIP → MPC foot placements → virtual constraints / gait tracking | arXiv:2109.14862; github.com/UMich-BipedLab/cassie_alip_mpc | Outer is **reduced-order footstep**, not residual-on-forward-ckpt |
| MPFC / MIQP: foothold + ankle torque + CoM over horizon (~50–200 Hz) | arXiv:2309.07993; 2501.19391 | Ankle + step timing are first-class; reverse via velocity command |
| Cassie HZD libraries: vx spanning **negative** | Reher/Grizzle lineage; Xie CoRL distill fwd/inplace/**backward** experts | Reverse is a **first-class gait**, often separate expert |

**Transfer:** T5-B already put ALIP/DCM+ankle in *new weights* and Prefer FAIL'd. T5-D must make the **ALIP/Placo/CSF outer the retreat primary**, with residual only as clear mid-swing corrector — not another residual family that still owns the polarity of Gate E walk.

### 1.3 Spot / Optimus / teleop-to-policy — command-conditioned loco; IL from demos

| Public fact | Source | Transfer |
|-------------|--------|----------|
| Spot: industrial teleop + learned skills (public demos; no open reverse paper) | BD product demos | Reverse = **command / teleop**, then policy; not invent plant |
| Optimus: proprio NN loco, direction/velocity responsive (public scraps) | Tesla demos / commentary | Same: **command-conditioned** policy |
| TWIST / OmniH2O / TRILL: teleop → RL+BC whole-body | PMLR / arXiv:2309.01952 | Sim teleop/oracle reverse demos → policy; **vision-off walk** still OK if proprio/command only |
| Isaac Lab Mimic: teleop → HDF5 → BC | Isaac Lab docs | Zero-cost path if we generate **sim** reverse demos from capture/Placo outer |

**Transfer:** T5-C already BC'd CSF50+Placo teacher and Prefer FAIL'd. Next teacher must be **capture/ICP or Placo reverse outer trajectories** that already clear Root B in the teacher — not residual BC that still plant-steals.

### 1.4 Open tools we already know (highest kit transfer)

| Tool | Mechanism | Why it matters for T5-D |
|------|-----------|-------------------------|
| **Placo** `walk_max_dx_backward` ≪ forward; `replan_supports` | Short reverse steps, mid-gait replan | Asymmetric reverse outer — co-constraint + teacher |
| **CSF / Capture Step** (Missura/Behnke) | F,T from CoM + commanded −Vx | Exchange from **capture locus**, not plant-dwell kick |
| **Swing-leg retraction / TD velocity match** | Late-swing foot tangential → ~0 | Attacks skate **p95 at impact** without planted soft-XY |
| **Walk This Way** | Footstep-location conditioned RL; explicit backwards | Architecture: external footholds for retreat |

---

## 2. Map to our wall (after T5-C)

```
Best near-misses (T5-C): s43_BC skate under both bars, rcf ~0.82/0.79, ret_ok=[T,F]
                         s47_03 bout0 ret_ok; bout1 skate+ε FAIL
Never: ret_ok_both + clean plant-cam ε
Root B: dest cam still can climb from plant ≫ clear under residual polarity
```

BD-class prediction: they **do not** reverse by rewarding a forward residual harder. They run a **reverse-native plan** (template / ALIP / capture / teleop) and stabilize with MPC/WBC. Our Prefer FAIL ladder proved residual twins on (near) E7lock hit skate/ε before both-bout `ret_ok`.

---

## 3. Hardware honesty levers (AMEND — safe list only)

Read: `docs/GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md`.

| Candidate | Honesty effect | Soft-pass risk | AI recommendation |
|-----------|----------------|----------------|-------------------|
| Friction µ bump | Changes skate physics | **HIGH — OUT** | Forbidden |
| Softsole / solref / solimp | Credits planted shove | **HIGH — OUT** | Forbidden |
| Foot-lift invent / taller contact / STEP invent | Fake clear | **HIGH — OUT** | Forbidden |
| Contact-box resize / planform | Reopens M145 | **OUT** | Forbidden |
| **Stricter plant-cam ε / Root B scoring** (controls/score only) | Makes Prefer FAIL louder earlier | None | **IN** — Controls, not Hardware plant edit |
| **Document CAD vs XML sole stack** (4.5 mm CAD vs 16 mm XML proxy) as known sim gap | Honesty about plant | None if no XML edit | **IN** as note; no edit without Dave |
| Named geom ask that **reduces** planted shove authority without µ soft (e.g. future sole texture for *real* kit — not sim cheat) | Real body | Only with Dave OK | Park until Dave names Path A |

**Hardware prep (no edit):** reaffirm freeze; optionally list any *future* kit-side sole/friction facts for March demo — **not** a sim soft-pass. Freeze until Dave OKs a named ask.

---

## 4. Ranked T5-D Prefer FAIL families (architecture — ≠ T5-A/B/C)

### **T5-D1 — Reverse-native outer PRIMARY (recommended)**

- **What:** For Gate Q retreat bout only: **Placo/CSF/ALIP-MPC outer owns F,T and footstep targets** from commanded −Vx / capture. Residual (new weights, bootstrap optional) is **only** mid-swing clear tracker + late-swing TDVM/SLR. Planted cancel×0.70 KEPT. Soft-pass never.
- **Why ≠ T5-A/B/C:** Those kept residual as the gait authority (reward / ALIP base inside residual train / BC teacher on residual path). D1 **demotes residual** to corrector; outer is reverse-native plan.
- **Why BD-shaped:** Matches Atlas template+MPC and Cassie ALIP-MPC story at kit scale (Placo/CSF we already have).
- **Must-holds:** bars; ε; tip; apps; companion md5; no FREEZE/cancel↑; no plant invent.
- **Pass path:** `ret_ok` both + Controls continuous watch + AI stills/Q03.
- **Prefer FAIL if:** outer still plant≫clear or skate p95 wall returns.

### **T5-D2 — Capture/ICP step-timing + location adjust (IHMC-flavored)**

- **What:** Online adjust support-exchange time **and** next foothold from ICP / capture error during retreat; CMP→ankle; residual optional.
- **Why ≠ R4:** Timing from capture error direction, not plant-dwell flush kick.
- **Fit:** Strong if Controls can implement ICP estimate on AiNex CoM proxy without new plant.
- **Risk:** Estimate noise → chatter; Prefer FAIL if ε steal.

### **T5-D3 — Sim teleop/oracle reverse demos from D1 outer → BC+RL (only if D1 teacher already clears Root B)**

- **What:** Generate reverse trajectories from **D1 outer** (not CSF50+residual that plant-steals); BC then skate-honest RL.
- **≠ T5-C:** Teacher source must already be reverse-native outer that banks dest cam from **clear**. If teacher plant-steals, do not BC it.
- **Cost:** Zero if sim-only; Path A only if Dave opens real teleop.

### **T5-D4 — Dave+Hardware named plant ask (last resort)**

- Only after D1 (and optionally D2) Prefer FAIL wall with honest metrics.
- Ask must be **named**, not µ/softsole/foot-lift invent.
- Soft-pass never.

### Explicitly out

Soft-pass · bar soften · R*/S1–S3/T5-A/B/C twins · friction/softsole/foot-lift invent · cancel↑ · FREEZE · vision-in-walk · Path A spend without Dave · install new sha16 without Prefer FAIL win · lock without Q03.

---

## 5. AI recommendation for the single cospec (Controls consolidates)

**Ship Prefer FAIL cospec for T5-D1 first** (reverse-native Placo/CSF/ALIP outer primary + clear-only residual corrector + TDVM/SLR). Fold Hardware honesty-safe notes (no plant edit). Soft-pass never. Bars KEPT. E7lock kept as rollback. No train until AI **no-veto** on Controls proposal.

If D1 Prefer FAIL's with clear Root B still dominant → escalate **T5-D2** or Dave **T5-D4** named ask — not another residual reward twin.

---

## 6. Third-party / call list (zero spend unless Dave opens)

| Need | Why | Cost gate |
|------|-----|-----------|
| Papers: arXiv:1703.00477 (Atlas ICP step adjust); 2109.14862 / cassie_alip_mpc; 2407.17683 Bang ALIP+residual; 2509.04722 G1 ALIP+ankle; Missura CSF; Walk This Way | Architecture priors | Free |
| BD blogs: Flipping the Script; Picking Up Momentum | Template+MPC framing | Free |
| Placo / OpenDuck presets (`walk_max_dx_backward`) | Kit outer | Free / already in stack |
| Isaac Lab Mimic / teleop docs | Only if D3 after D1 teacher clears Root B | Free docs; GPU Path A only if Dave opens |
| Unitree H1/G1 public reverse demos | Optional teacher inspiration | Free watch; no claim kit teleop |

---

## 7. Coop / next

1. **Controls:** kit-facing mechanisms for T5-D1 (Placo/CSF/ALIP outer primary wiring) + fold Hardware honesty-safe candidates → `GATE_Q_AI_COSPEC_T5D_PROPOSAL.md`.
2. **AI:** same-turn veto / no-veto → `docs/GATE_Q_AI_COSPEC_T5D.md`.
3. **Hardware:** ACK freeze; optional honesty-safe candidate list (no edit).
4. **Dave:** plan landed when Controls consolidates votes — research pack **ready** (this file).

**Status:** Curriculum D–P TRUE. Gate Q **OPEN — T5-D research pack READY**. Soft-pass off. No train. No plant ask.
