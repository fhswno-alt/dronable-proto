# Gate Q T5-D breakout research — honest reverse / walk-back

**When:** Mon 28 Sep 2026 ~21:45 Europe/London (BST)  
**Scope:** Public methods → AiNex MuJoCo Gate Q. Soft-pass **never**. Bars **KEPT** (skate ≤0.08/0.18, cf≥0.55, tip≥8). Prefer FAIL. No invent.  
**Closed:** T5-A/B/C + R*/S1–S3. E7lock `9ffaa1a21b607bf6`. Plants `fc94709c…` / `59cc408e…` freeze.

---

## 1) What worked for BD-class / academic reverse (cited)

| Lineage | Public mechanism | Cite | AiNex transfer |
|---------|------------------|------|----------------|
| **Atlas (BD)** | Online **MPC** (CoM→full-body momentum) + layered offline templates; step timing/location | [BD blog](https://bostondynamics.com/blog/picking-up-momentum/); Koolen Atlas ([arXiv:1703.00477](https://arxiv.org/abs/1703.00477)); DRC WBC/capture ([CMU](https://www.cs.cmu.edu/~cga/drc/ICHR15_0025_FI.pdf)) | Reverse = **planned footholds**, not mirrored residual. No open reverse recipe — transfer architecture. |
| **Capture / DCM** | Control divergent CoM; foothold for stop/reverse Vx | Pratt capture ([CMU](https://www.cs.cmu.edu/~cga/legs/Pratt_Goswami_Humanoids2006.pdf)); Englsberger DCM ([DLR](https://elib.dlr.de/87255/1/IROS_2013_FinalSubmission_JEnglsberger.pdf)); DCM+MPC ([arXiv:1702.08742](https://arxiv.org/abs/1702.08742)) | Reverse Vx → ICP/DCM target behind stance. Needs honest CoM/contact. |
| **Cassie / Digit ALIP** | **ALIP-MPC** foot placement + virtual constraints; velocity field incl. back/lateral | Gibson ([arXiv:2109.14862](https://arxiv.org/abs/2109.14862), [code](https://github.com/UMich-BipedLab/cassie_alip_mpc)); Digit demos ALIP/WBC-class (no open reverse paper) | Reverse is a **command/library entry**, not plant slide. |
| **Raibert** | Neutral foothold + offset ∝ velocity error | Raibert 1986 | Foot **forward** of neutral decelerates/reverses — cheap outer prior. |
| **Cassie RL reverse** | Separate **backward expert** → distill; joint train hurts reverse | Xie CoRL 2019 ([PDF](https://proceedings.mlr.press/v100/xie20a/xie20a.pdf)) | Reverse-native BC teacher, not forward-mirror. |
| **Unitree H1/G1** | Command-conditioned RL (vx±); G1 **ALIP NMPC + ankle** | [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab); ([arXiv:2509.04722](https://arxiv.org/abs/2509.04722)) | Omni velocity + ankle; skate lived in contact. |
| **Optimus** | Teleop/mocap → IL; RL sim loco | [BI](https://www.businessinsider.com/tesla-job-training-optimus-robot-motion-capture-suit-2024-8); no reverse paper | Demos + reverse cmd — not plant invent. |
| **Open/cheap** | Placo `walk_max_dx_backward`≪fwd; Walk This Way footstep RL (backwards); ALIP-MPC + residual Δu_fp | [OpenDuck/Placo](https://github.com/apirrone/Open_Duck_reference_motion_generator); Walk This Way ([ACM](https://dl.acm.org/doi/10.1145/3747865)); Bang/Sentis ([arXiv:2407.17683](https://arxiv.org/abs/2407.17683)) | Best kit transfer: short `|dx_back|` + reverse outer + residual on **new** base. |

Public BD/Optimus/Digit prove walk-back exists. Open levers: reverse foothold/MPC, short `|dx_back|`, reverse-expert BC, TDVM/SLR, ALIP/DCM+ankle in new weights — not skate soft-pass.

---

## 2) Why T5-A/B/C residual-on-frozen-plant stalls

| Wall | Evidence (measured) | Meaning |
|------|---------------------|---------|
| **clear_frac ~0.85** while skate fails | T5B_02 cf **0.856/0.847**; T5C s47 **0.862/0.750** (`GATE_Q_AI_SCORE_T5B/C.md`) | Clear SS banks; dest cam not honest both bouts under skate bars. |
| **skate p95** | T5B_02 bout0 **0.183** (0.003 over 0.18); no seed `ret_ok` both | Stance vx spikes at TD (slip-near-impact). T5-A/B/C never cleared package. |
| **plant-cam ε steal** | T5B_02 maxG **0.033/0.022**; T5C steal=True | XY while soles planted → Prefer FAIL. Cancel×0.70 held; planted soft-XY forbidden. |
| **Frozen plant** | M145 16 mm / µ `1.6 0.1 0.01` locked; GEO Gate-E hard-falsified | Residual cannot invent contact; Root B planted middle remains trap (`GATE_Q_HARDWARE_PLANTED_MIDDLE_HONESTY.md`). |

**Diagnosis:** residual axis exhausted on contact-ambiguous plant. More BC/seed twins → Prefer FAIL. Breakout = plant honesty **or** reverse-native outer.

---

## 3) Ranked next (bars KEPT; soft-pass off)

| Rank | Approach | One-line |
|------|----------|----------|
| **1 A** | **Plant honesty / contact model** (Dave-named only) | Named geom/contact that kills ε-steal + bout1 p95 **without** Root B credit. Friction/softsole/foot-lift invent **out**. |
| **2 B** | **Architecture: MPC/WBC or ALIP-NMPC teacher outer + residual** | Placo/ALIP/Raibert-DCM reverse foothold as **new outer**; residual clear-only. ≠ S2/T5-B twin on frozen E7lock. Matches BD/Cassie public pattern. |
| **3 C** | **Better reverse demos / teleop** | Kit teleop or richer Placo/CSF reverse → BC→RL (Xie expert). Higher value **after** A or B. |
| **4 D** | **Genuinely different** | Footstep-conditioned policy ([arXiv:2207.12644](https://arxiv.org/abs/2207.12644) / Walk This Way); Isaac contact DR that **exposes** ε (not hides Root B); hard-gated dual approach/retreat policy. |

**Recommendation:** **A∩B hybrid cospec** — Dave-named plant-honesty lever + reverse-native MPC/Placo outer + residual → Prefer FAIL wall. No T5-A/B/C twins. Soft-pass never.

---

## 4) Third-party needs

| Need | Items |
|------|-------|
| **Papers** | arXiv:1703.00477, 2109.14862, 2407.17683, 2509.04722, 2207.12644, 1702.08742, 1903.09537; Englsberger DCM; Pratt capture; Walk This Way PACMCGIT 2025; Raibert 1986; BD *Picking Up Momentum* |
| **Datasets** | OpenDuck/Placo reverse presets; Unitree H1 Isaac ckpts; optional LeRobot IL; kit teleop reverse logs (if Path A) |
| **Isaac/MuJoCo** | Isaac Lab loco + contact DR; `cassie_alip_mpc`; Placo WalkEngine; CasADi/IPOPT ALIP NMPC; existing AiNex plant (no invent) |
| **Nvidia/APIs** | Optional Isaac GPU train; no proprietary BD/Optimus reverse API |
| **Hardware** | Dave-gated named honesty candidates only; no µ/softsole/STEP invent |
| **Compute** | Prefer FAIL ≤4 h / ≤2e6 steps class; hybrid may need CasADi CPU + PPO GPU |

---

## Disposition

T5-A/B/C Prefer FAIL **exhausted**. Next **T5-D**: **A → B → C → D**, pick **A∩B**; bars KEPT; soft-pass off; E7lock until new sha16 honestly wins. No train until Dave OK + cospec no-veto.
