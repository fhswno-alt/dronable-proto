# Gate Door v1 — Controls cospec (§5 answers to Hardware push/pull pack)

**Superseded live pointer (Wed 30 Sep 2026 ~11:15 BST):** live companion = Option C `…_gate_f_optc.xml` md5 `6a3d4a70d4797b806dcc2580f46468aa` (**INSTALLED**). This cospec ACK’d Option A; `…_gate_f_push.xml` md5 `adb24309…` is **ARCHIVE** (bytes unchanged). Option B `…_gate_f_optb.xml` md5 `ddf084cd…` is **ARCHIVE** (bytes unchanged). Prefer FAIL bars unchanged. Soft-pass **OFF**. Park **OFF**. No Door SCORE claim from the Option C install. Receipt: `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`.

**When:** Tue 29 Sep 2026 ~21:55 Europe/London (BST)  
**From:** Controls · **To:** Hardware · AI · Dave  
**Refs:** `docs/GATE_Q_HARDWARE_PUSH_PULL_V1_DIRECTIVE.md` · fork `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040` (verified)  
**Status:** Controls **§5 answers READY** · awaiting **AI no-veto** + **Dave ACK** of named plant swap · **NO INSTALL** · Soft-pass **OFF** · Spend **none**

Live freeze **KEPT:**
- Walk / M145 `fc94709c84f5598d4474ecfc4bb41fdc`
- Companion F–Q `59cc408eda07037a58f92ad27da045d6`

---

## §5 answers (Prefer FAIL / curriculum confirm)

### 1. Door-task v1 = panel push/pull only — **CONFIRM**

Drop as **required** door proves: G03 hand–lever · H02 lever-rotate · I01–J01 lever-coupled panel path.  
**Open measure stays** `door_panel_hinge` angle (keep ±30° class).  
Historical F–P LOCKED TRUE = **lever-era sim history**, not March hospital-door claim.

### 2. Closed detect — **ACCEPT** (ordered)

| Priority | Signal | Prefer FAIL note |
|----------|--------|------------------|
| 1 | `|door_panel_hinge|` ≈ 0 (closed band — AI names ε°) | Primary closed claim |
| 2 | Optional: hand × `door_panel_push_face` contact pair | Contact proves push engagement, not closed alone |
| 3 | Optional: distance to `door_panel_push_site` | Approach / reach aid |

No latch. Soft-pass never on “looks closed.”

### 3. Contact Option A vs B vs C — **ACK Option A**

Reuse bit 2: `door_panel_push_face` + `l/r_hand_contact`; lever contype 0. Matches shipped draft; minimal hand XML change.  
**Escalation only after Prefer FAIL miss** (thin −X face / side approach): name **Option C** (full panel contact) in a new Hardware draft — not mid-score invent. Option B (bit 4) = later knob A/B park — not v1.

### 4. Lever hinge — **KEEP inert visual DOF**

Keep `door_lever_hinge` + site for cam/FOV reference. Contype 0 already. Weld/remove only if Prefer FAIL shows hinge noise or AI asks — not this install.

### 5. Cam — **KEEP lever-height look-down first Prefer FAIL**

Do not rebind optical axis mid-wall. If closed-detect / push engagement Prefer FAIL on FOV blindness, AI+Hardware name panel-center rebind as a **separate** honesty item after SCORE.

### 6. Curriculum rows F–P — **historical on live gate_f**

| Row | Disposition until Dave ACK swap |
|-----|----------------------------------|
| F–P LOCKED TRUE | Stay on live `…_gate_f.xml` as **lever-era history** — do **not** auto-relabel as March door PASS |
| New push/pull proves | New companion row(s) / criteria after install of `…_gate_f_push.xml` — AI owns formal criteria (`GATE_DOOR_V1_AI_CRITERIA` or amend F→Q) |
| Gate Q locomotion | Prefer FAIL bars **KEPT** (skate / clear_frac / tip / plant-cam ε); compose prereq rewrites to push/pull **after** plant ACK — orthogonal to H2 / DCM / park pick |

### 7. March 2027 bar — **CONFIRM**

Walk a space + open door via **panel push/pull** to ±30° class open-angle. Knob torque / latch / 90° / walk-through / UK height = deferred. Soft-pass **OFF**.

---

## Controls scoring posture (post-ACK only)

**Names Controls will score against (after AI no-veto criteria land):**

- Contact: `l_hand_contact` / `r_hand_contact` × `door_panel_push_face`
- Sites: `door_panel_push_site`, `door_panel_site` (open tell), free-edge stripe visual OK
- Open: `|door_panel_hinge|` toward open band (AI thresholds)
- Closed: hinge ≈0 band
- **Not scored:** `door_lever` contact, `door_lever_hinge` angle, grasp-on-lever, latch

**Hard falsifiers (Controls):** soft-pass · live md5 edit without Dave ACK · plant invent · claiming latch/90°/walk-through · scoring lever contact as door-open · installing push fork before AI no-veto + Dave ACK.

---

## Install gate (all required)

1. This Controls §5 cospec  
2. **AI no-veto** (criteria rewrite / formal ACK of Option A + closed-detect)  
3. **Dave ACK** of named swap → companion live becomes `…_gate_f_push.xml` md5 `adb24309b489d56615c194e92676d040`  
4. Until then: READY-NOT-INSTALLED · live freeze KEPT

---

## Orthogonal (unchanged)

Gate Q Prefer FAIL next lever still **Dave**: named **H2** A/B · **DCM-VRP-DS** · or **park**. Soft-pass off · E7lock frozen · H2 not auto-unlocked by this door pack.

## Owner next

| Owner | Next |
|-------|------|
| AI | No-veto on this §5 + formal push/pull criteria (or Prefer FAIL / amend asks) |
| Dave | ACK plant swap (or hold) after AI no-veto |
| Hardware | Hold freeze; Option C only if Prefer FAIL demands |
| Controls | Idle on door install until 1–3; Gate Q loco only on Dave pick |
| MFG | No PO; panel prop quote only if cospec freezes one |
