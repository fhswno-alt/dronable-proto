# Controls freeze status — Monday morning

**When:** Mon 28 Sep 2026 ~14:23 Europe/London (BST) · Hardware note Tue 29 Sep ~22:08 BST: Door v1 live companion = `gate_f_push` md5 `adb24309…` (INSTALLED); lever-era `gate_f` archived
**Audience:** Dave · Founding Controls · room  
**Scope:** SIM-ONLY status sheet. **No Path A / thaw / greenlight / spend.**

---

## 1. Curriculum locks

| Gate | Status | One line |
|------|--------|----------|
| **D** | **HOLDS** | CSF50 / clean_walk_ss on locked M145 (`ainex_controls_m2_145.xml`) |
| **E** | **UNLOCKED TRUE** | Learned PPO residual (RL04) on **same M145 plant** — ckpt frozen; REPRO PASS |
| **F** | **LOCKED TRUE** | Lever look+approach on companion plant `ainex_controls_m2_145_gate_f.xml` — M145 contact/HX untouched |
| **G** | **LOCKED TRUE** | Lever reach + light contact on companion; **G00–G03**; AI lock `previews/ainex_walk/iterate/GATE_G_AI_LOCK.md` — cite `GATE_G_CONTROLS_NOTE.md` |
| **H** | **LOCKED TRUE** | Grasp hold + lever rotate on companion; **H00–H03**; AI lock `previews/ainex_walk/iterate/GATE_H_AI_LOCK.md` — companion hinge only; **no full door-open** |
| **I** | **LOCKED TRUE** | Limited panel swing on companion; **I00–I03**; AI lock `previews/ainex_walk/iterate/GATE_I_AI_LOCK.md` — cite `GATE_I_CONTROLS_NOTE.md`; **not** full door-open |
| **J** | **LOCKED TRUE** | Larger coupled panel ≥25° on soft-spring companion; **J00–J03**; AI lock `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md` — cite `GATE_J_CONTROLS_NOTE.md`; **not** full door-open |
| **K** | **LOCKED TRUE** | Multi-bout open/close 3/3 XY-free; §9 PASS; open-angle PASS (green free-edge stripe Δ screen-X ≥~25 px @ +30°); AI lock `previews/ainex_walk/iterate/GATE_K_AI_LOCK.md`; companion md5 `59cc408eda07037a58f92ad27da045d6`; **not** full door-open |
| **L** | **LOCKED TRUE** | Leave/re-grasp 2/2 XY-free; L01 leave ≥20° + contact-honest regrasp; AI re-lock after conflict root **A** cleared (`GATE_L_AI_LOCK.md`, `GATE_L_VIDEO_CONFLICT.md`); companion md5 `59cc408eda07037a58f92ad27da045d6`; **not** range bump / full open |
| **M** | **LOCKED TRUE** | Open-hold disturb reject 2/2 XY-free; panel push 1.2 N × 0.35 s declared; AI re-lock after conflict root **A** cleared (`GATE_M_AI_LOCK.md`, `GATE_M_VIDEO_CONFLICT.md`; bout0 short disturb window visible on stills); companion md5 `59cc408eda07037a58f92ad27da045d6`; soft-pass **not** used; **not** full open / latch / 90° / walk-through / UK / range bump |
| **N** | **LOCKED TRUE** | Leave+disturb compose 2/2 same bout (L+M; soft-pass **not** used); stills DISTURBANCE ON + leave windows both bouts; AI lock `previews/ainex_walk/iterate/GATE_N_AI_LOCK.md` / `.json`; companion md5 `59cc408eda07037a58f92ad27da045d6`; **not** full open / latch / 90° / walk-through / UK / range bump / walk→door |
| **O** | **LOCKED TRUE** | Approach→N-compose 2/2 same run (soft-pass **not** used); stills APPROACH START + DISTURBANCE ON + LEAVE WINDOW both cycles; AI lock `previews/ainex_walk/iterate/GATE_O_AI_LOCK.md` / `.json`; companion md5 `59cc408eda07037a58f92ad27da045d6`; **not** full open / latch / 90° / walk-through / UK / range bump / room-walk / investor walk |
| **P** | **LOCKED TRUE** | Stepped approach→N-compose 2/2 same run (Controls iterate2; soft-pass **not** used; clear above plant rest + dwell; FOOT-LIFT non-sticky; finalize KINEMATIC XY SHIM; prior Root B closed); AI lock `previews/ainex_walk/iterate/GATE_P_AI_LOCK.md` / `.json`; companion md5 `59cc408eda07037a58f92ad27da045d6`; **not** full open / latch / 90° / walk-through / UK / range bump / room-walk / investor walk |
| **Q** | **PARK Prefer FAIL** (structural; not lock TRUE; not LOCKED FALSE) | Stepped approach→N→stepped retreat; criteria KEPT (`docs/GATE_Q_AI_CRITERIA.md`); best E7lock (bout1 ret cf~0.649 clear_frac_ok + joint HIT; skate from ~27s planted middle blocks `ret_ok`); AI structural note `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md`; soft-pass **not** used; companion md5 `59cc408eda07037a58f92ad27da045d6`; **no** Q03 lock ping |

Vision **off** walk / PPO obs. Still **not** Pi / Orin. Assist / freeze **OFF**.

---

## 2. Frozen plant / stack

| Item | State | Path / value |
|------|-------|----------------|
| **Walk / Gate E plant** | **FROZEN** | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| **Door v1 live companion** | **INSTALLED** (Hardware; Dave ACK ~22:08 BST 29 Sep) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` · md5 `adb24309b489d56615c194e92676d040` — Option A push face; soft-pass off; **no Controls score claim here** · receipt `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md` |
| **Lever-era F–P archive** | **ARCHIVE** (F–P locks + Q Prefer FAIL park history; **Gate K lock plant**) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` — cam + lever + panel hinge + **Gate J spring** damp 0.05 / stiff 0.015 + Gate K visual `door_panel_free_edge_stripe` (stripe v1); **lock md5 `59cc408eda07037a58f92ad27da045d6`**; foot/HX untouched vs M145 — **not** overwritten by push install |
| **Gate E ckpt** | **FROZEN** | `previews/ainex_walk/iterate/learned_gate_e/ppo_gate_e_best.zip` · sha16 **`9ffaa1a21b607bf6`** |
| **Head command** | **LOCKED** | Joint **`head_tilt`** (not `neck_pitch`) |
| **Authority / cheats** | **OFF** | `k_auth=1.0`; assist OFF; freeze OFF; `vision_in_walk_obs=false` |

Do **not** rescore Gate E on the companion plant. Companion is for F–Q rows only — **not** a full door-open claim.

**Door v1 live pointer:** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` · md5 `adb24309b489d56615c194e92676d040` · **INSTALLED (sim-only)**. The lever-era `gate_f.xml` remains archive/history for F–P lock references.

---

## 3. Hard-falsified classes (do not reopen)

| Class | Status |
|-------|--------|
| Open-loop CPG + VIK + μ/T micro-sweep | Gate E FALSE / closed |
| Linear residual / WBC / MPC | HARD-FALSIFIED |
| Dual-T outer (shared M145) + plant×outer (GEO02a/b) | HARD-FALSIFIED |
| HX authority envelope (k→10) | HARD-FALSIFIED |
| Contact/geometry honesty with frozen outer (GEO00–03) | HARD-FALSIFIED |

Controls does **not** reopen these without an explicit Dave ask + new AI cospec.

---

## 4. Reviewable paths

| Artifact | Path |
|----------|------|
| Gate Q AI structural Prefer FAIL | `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` |
| Gate Q criteria (KEPT) | `docs/GATE_Q_AI_CRITERIA.md` |
| Gate Q Controls note / table / diag | `previews/ainex_walk/iterate/GATE_Q_CONTROLS_NOTE.md`, `GATE_Q_TABLE.json`, `GATE_Q_A28_RETOK_DIAG.md` |
| Gate Q tip E7lock / A28c | `docs/GATE_Q_AI_TIP_E7lock.md`, `docs/GATE_Q_AI_TIP_A28c.md` |
| Gate Q script | `scripts/score_gate_q.py` |
| Gate P Controls note + table | `previews/ainex_walk/iterate/GATE_P_CONTROLS_NOTE.md`, `GATE_P_TABLE.json` |
| Gate P AI lock | `previews/ainex_walk/iterate/GATE_P_AI_LOCK.md`, `GATE_P_AI_LOCK.json` |
| P stills / conflict | `previews/ainex_walk/iterate/gate_p_iterate2_stills/`, `GATE_P_VIDEO_CONFLICT.md` |
| P videos (primary) | `previews/ainex_walk/iterate/P03.mp4` |
| Gate P script | `scripts/score_gate_p.py` |
| Gate P criteria | `docs/GATE_P_AI_CRITERIA.md` |
| Gate O Controls note + table | `previews/ainex_walk/iterate/GATE_O_CONTROLS_NOTE.md`, `GATE_O_TABLE.json` |
| Gate O AI lock | `previews/ainex_walk/iterate/GATE_O_AI_LOCK.md`, `GATE_O_AI_LOCK.json` |
| O stills / rewatch | `previews/ainex_walk/iterate/gate_o_rewatch/` |
| O videos (primary) | `previews/ainex_walk/iterate/O03.mp4` |
| Gate O script | `scripts/score_gate_o.py` |
| Gate O criteria | `docs/GATE_O_AI_CRITERIA.md` |
| Gate N Controls note + table | `previews/ainex_walk/iterate/GATE_N_CONTROLS_NOTE.md`, `GATE_N_TABLE.json` |
| Gate N AI lock | `previews/ainex_walk/iterate/GATE_N_AI_LOCK.md`, `GATE_N_AI_LOCK.json` |
| N stills / rewatch | `previews/ainex_walk/iterate/gate_n_rewatch/` |
| N videos (primary) | `previews/ainex_walk/iterate/N03.mp4` |
| Gate N script | `scripts/score_gate_n.py` |
| Gate N criteria | `docs/GATE_N_AI_CRITERIA.md` |
| Gate M Controls note + table | `previews/ainex_walk/iterate/GATE_M_CONTROLS_NOTE.md`, `GATE_M_TABLE.json` |
| Gate M AI lock | `previews/ainex_walk/iterate/GATE_M_AI_LOCK.md`, `GATE_M_AI_LOCK.json` |
| M conflict / stills | `previews/ainex_walk/iterate/GATE_M_VIDEO_CONFLICT.md`, `gate_m_disturb_rewatch/` |
| M videos (primary) | `previews/ainex_walk/iterate/M03.mp4` |
| Gate M script | `scripts/score_gate_m.py` |
| Gate M criteria | `docs/GATE_M_AI_CRITERIA.md` |
| Gate K Controls note + table | `previews/ainex_walk/iterate/GATE_K_CONTROLS_NOTE.md`, `GATE_K_TABLE.json` |
| Gate K AI lock | `previews/ainex_walk/iterate/GATE_K_AI_LOCK.md`, `GATE_K_AI_LOCK.json` |
| K videos (primary) | `previews/ainex_walk/iterate/K03_dual.mp4` (kit‖oblique stripe), `K03_world.mp4` (§9) |
| K stills / stripe measure | `previews/ainex_walk/iterate/gate_k_edge_stripe_reencode/` |
| Gate K script | `scripts/score_gate_k.py` |
| Gate K HW stripe | `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` |
| Gate J Controls note + table | `previews/ainex_walk/iterate/GATE_J_CONTROLS_NOTE.md`, `GATE_J_TABLE.json` |
| Gate J AI lock | `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md`, `GATE_J_AI_LOCK.json` |
| J videos | `previews/ainex_walk/iterate/J00.mp4` … `J03.mp4` |
| Gate J script | `scripts/score_gate_j.py` |
| Gate J spring (Hardware) | `docs/GATE_J_HARDWARE_PANEL_SPRING.md` |
| Gate I Controls note + table | `previews/ainex_walk/iterate/GATE_I_CONTROLS_NOTE.md`, `GATE_I_TABLE.json` |
| Gate I AI lock | `previews/ainex_walk/iterate/GATE_I_AI_LOCK.md`, `GATE_I_AI_LOCK.json` |
| I videos | `previews/ainex_walk/iterate/I00.mp4` … `I03.mp4` |
| Gate I script | `scripts/score_gate_i.py` |
| Gate H Controls note + table | `previews/ainex_walk/iterate/GATE_H_CONTROLS_NOTE.md`, `GATE_H_TABLE.json` |
| Gate H AI lock | `previews/ainex_walk/iterate/GATE_H_AI_LOCK.md`, `GATE_H_AI_LOCK.json` |
| H videos | `previews/ainex_walk/iterate/H00.mp4` … `H03.mp4` |
| Gate H script | `scripts/score_gate_h.py` |
| Gate G Controls note + table | `previews/ainex_walk/iterate/GATE_G_CONTROLS_NOTE.md`, `GATE_G_TABLE.json` |
| Gate G AI lock | `previews/ainex_walk/iterate/GATE_G_AI_LOCK.md`, `GATE_G_AI_LOCK.json` |
| G videos | `previews/ainex_walk/iterate/G00.mp4` … `G03.mp4` |
| Gate G script | `scripts/score_gate_g.py` |
| Gate F Controls note + table | `previews/ainex_walk/iterate/GATE_F_CONTROLS_NOTE.md`, `GATE_F_TABLE.json` |
| Gate F AI lock | `previews/ainex_walk/iterate/GATE_F_AI_LOCK.md`, `GATE_F_AI_LOCK.json` |
| Gate E note + REPRO | `previews/ainex_walk/iterate/LEARNED_GATE_E_NOTE.md`, `LEARNED_GATE_E_REPRO.md` / `.json` |
| F videos | `previews/ainex_walk/iterate/F00.mp4`, `F01.mp4`, `F02.mp4` |
| Gate F script | `scripts/score_gate_f.py` |

---

## 5. Explicit for Monday (~14:23 BST)

- Gate **P LOCKED TRUE** (sim). Curriculum **D–P TRUE** (sim). Soft-pass **not** used.
- Gate **Q PARK Prefer FAIL** (structural residual-family wall; **not** lock TRUE; **not** LOCKED FALSE). Criteria **KEPT** (skate ≤0.08/0.18 + clear_frac≥0.55 — no soften). Soft-pass **off**. Cite `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md`.
- Best-known Q metrics: **E7lock** (CLEAR_TRACK_SWING=1, cancel×0.70). bout1 ret cf~0.649 + joint HIT; skate from ~27s planted middle still blocks `ret_ok`. No Q03 continuous pack / no AI lock ping.
- **Ckpt:** sha16 **`9ffaa1a21b607bf6`**. **Walk plant:** M145 frozen untouched. **Companion lock md5:** **`59cc408eda07037a58f92ad27da045d6`**.
- Controls **idle on Q** unless a genuinely new retreat-native family (not rename of abandoned cancel↑/damp↑/FREEZE/gap-planted). HW/MFG stay freeze. No plant invent. No spend.
- Explicit **non-claims:** full door-open / latch / 90° / walk-through / UK handle height / Pi / Orin / range bump / room-walk / investor walk.
- **Nothing waiting on Dave** for Q (AI already disposition-pinged).

**Refs:** `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md` · `docs/GATE_Q_AI_CRITERIA.md` · `docs/GATE_O_AI_CRITERIA.md` · `previews/ainex_walk/iterate/GATE_O_AI_LOCK.md` · `docs/GATE_N_AI_CRITERIA.md` · `previews/ainex_walk/iterate/GATE_N_AI_LOCK.md` · `docs/GATE_M_AI_CRITERIA.md` · `previews/ainex_walk/iterate/GATE_M_AI_LOCK.md` · `docs/GATE_L_AI_CRITERIA.md` · `previews/ainex_walk/iterate/GATE_L_AI_LOCK.md` · `docs/GATE_K_AI_CRITERIA.md` · `docs/GATE_K_HARDWARE_EDGE_STRIPE.md` · `previews/ainex_walk/iterate/GATE_K_AI_LOCK.md` · `GATE_K_CONTROLS_NOTE.md` · `docs/CONTROLS_POST_J_MUST_PROVE_STUB.md` · `docs/GATE_J_AI_CRITERIA.md` · `docs/GATE_J_HARDWARE_PANEL_SPRING.md` · `docs/TELEOP_AUTONOMY_CURRICULUM.md` · `docs/GATE_I_AI_CRITERIA.md` · `docs/GATE_H_AI_CRITERIA.md` · `docs/GATE_G_AI_CRITERIA.md` · `docs/GATE_F_AI_CRITERIA.md` · `docs/MONDAY_PERCEPTION_COMPUTE_CHECKLIST.md`

## Gate Q (15:20 BST): Prefer FAIL PARK after R1–R4 — E7lock best-known; R* STOP; soft-pass off; ai_can_lock=false; reopen only genuinely new residual + AI cospec first.

## Gate Q (~15:46 BST): Prefer FAIL after Q-U1/S1 CSF50-on-retreat — E7lock best-known; `GATE_Q_RET_CSF50=0`; soft-pass off; ai_can_lock=false; escalate S2 ALIP-TVR only with AI cospec.
