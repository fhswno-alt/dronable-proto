# Gate Door v1 — thin push-face contact-area honesty

**When:** Wed 30 Sep 2026 ~02:11 Europe/London (BST)  
**Who:** Founding Hardware Engineer  
**Scope:** DOCS-ONLY dig of the thin push face (measured while Option B was live). Soft-pass **OFF**. **No spend.** No plant XML edit. **Live pointer since ~11:15 BST 30 Sep is Option C** `…_gate_f_optc.xml` md5 `6a3d4a70…`. Option B is **ARCHIVE**. This file does not install anything and does not claim a Door SCORE.

Overnight context (not a Hardware SCORE): D-series Prefer FAIL **STUCK** (phantom PARTIAL). D-twin Prefer FAIL is spinning **control-only**. AI/Controls report intermittent contact, and contact-drop after peak, on D01/D02 against the thin −X push face. Option B bit-4 alone did not clear phantoms.

---

## Measured geom — live Option B (`gate_f_optb`)

Plant: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml`  
md5: `ddf084cdac71cb0998aa6a44a65594c0` (**INSTALLED**)

MuJoCo box `size` is **half-extents**. Face numbers below are `2 ×` the Y and Z half-sizes. Half-thickness is the X half-size.

| Geom | MuJoCo type | size (m, as stored) | contype / conaffinity | Reading |
|------|-------------|---------------------|------------------------|---------|
| `door_panel_push_face` | **box = 6** | `[0.008, 0.048, 0.15]` | **4 / 4** | Half-thickness **8 mm**. Full slab 16 × 96 × 300 mm. Contactable face ≈ **96 × 300 mm**. Bit 4. |
| `door_panel` | **box = 6** | `[0.01, 0.05, 0.17]` | **0 / 0** | Half-extents 10 × 50 × 170 mm (full 20 × 100 × 340 mm). **Visual** on optb — not a hand contact pair. |
| `l_hand_contact` | sphere | radius `0.012` | **4 / 4** | Radius **12 mm**. Pairs with bit 4 only. |
| `r_hand_contact` | sphere | radius `0.012` | **4 / 4** | Radius **12 mm**. Pairs with bit 4 only. |

On-disk attributes (optb): push face `pos="-0.018 0.05 0.17"` (robot-facing **−X** skin); panel `pos="0 0.05 0.17"`. Hands sit at `size="0.012"` with `contype="4"`.

Load facts (Hardware-verified ~02:11 BST 30 Sep 2026): optb loads, **nq=33**, **nu=24**, push-face geom type **box = 6**. This docs pass did **not** repeat a MuJoCo load — the prep environment has no MuJoCo Python package (`mujoco` resolves to the plant tree under `mujoco/`). Stored XML sizes and contact bits match the table. No XML bytes were written to confirm them.

Bit-4 pairing on optb: `l/r_hand_contact` (4) × `door_panel_push_face` (4) can contact. × `door_panel` / `door_lever` / feet = no pair. Option B changed the **bit**, not the slab size. The thin −X face is the same geom family as archived Option A.

---

## Compare — Option C (held, not live)

Plant: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optc.xml`  
md5: `6a3d4a70d4797b806dcc2580f46468aa` (**READY-NOT-INSTALLED / HELD**)

Same Hardware-verified load window: optc loads, **nq=33**, **nu=24**. Not re-loaded in this prep environment. Not installed.

| Geom | optb (LIVE) | optc (HELD) |
|------|-------------|-------------|
| `door_panel` | size `[0.01, 0.05, 0.17]`, **contype 0** (visual) | same size, **contype 2** — full panel is the contactable volume |
| `door_panel_push_face` | size `[0.008, 0.048, 0.15]`, **contype 4** | same size, **visual 0** (contype 0) — tell only; site kept |
| `l/r_hand_contact` | sphere r=0.012, **bit 4** | sphere r=0.012, **bit 2** |

Option C contact pair would be hand × `door_panel` (bit 2). The push-face geom would not earn contact. The contactable body is the panel volume (full about 20 × 100 × 340 mm), not the 8 mm half-thickness −X skin. That is a larger contactable volume than the live push face. It is still **not installed**.

---

## Honesty thesis

Intermittent contact, and contact-drop after peak, on D01/D02 is **consistent with** a thin −X face plus small hand spheres.

- Engagement depth along the push normal is the push-face half-thickness: **~8 mm**.
- Hand spheres are **r = 12 mm**, larger than that half-thickness.
- The only surface hands can pair with on live Option B is that 96 × 300 mm slab. The visual panel behind it is contype 0, so a hand that has slid off the skin, or separated by a gap the 8 mm half-size cannot cover, drops contact even while it still looks near the door.
- A peak followed by a drop fits that narrow overlap. It does not require a scorer exception.

**This is not a soft-pass excuse.** Prefer FAIL bars stay **KEPT**: closed ≤2° · contact ≥0.3 s · open ≥25° · hold ≥1 s @ ≥20° · 2/2 (`docs/GATE_DOOR_V1_AI_CRITERIA.md`). Soft-pass **OFF**. Phantom PARTIAL stays a fail mode. Bit-4 did not clear it, and this note does not waive it.

---

## Escalation if D-twin is still STUCK

Two paths. Neither is a mid-score invent. Neither spends.

| Path | What it is | Hardware state |
|------|------------|----------------|
| **Option C full-panel** | Already drafted. `door_panel` contype 2; push_face visual 0; hands bit 2. Install checklist: `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_INSTALL_READY.md` | **READY-NOT-INSTALLED / HELD** — install only on a Dave **named ACK** |
| **Control-only hold** | D-twin keeps spinning on the live Option B plant. Hardware idle. | Live stays `gate_f_optb` / `ddf084cd…` |

No third plant. No geom tweak on the live file. No bar edit between scores.

---

## Explicit non-claims

- **Not** Door LOCK.
- **Not** a plant change tonight. Live stays Option B. A / B / C / walk / lever-era XML bytes untouched.
- **Not** soft-pass. Prefer FAIL bars **KEPT**.
- **Not** an Option C install.
- **Not** a Prefer FAIL SUCCESS or FAIL claim from Hardware. Controls owns the score.
- **Not** proof that Option C will clear phantoms. It is the named larger contact volume, still held.

---

## md5 locks (unchanged this dig)

| File | md5 | State |
|------|-----|-------|
| `…_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **INSTALLED** (live) |
| `…_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **HELD** |
| `…_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **ARCHIVE** |
| `…_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** |

---

## Refs

- Option C held draft: `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_DRAFT.md`
- Option C install checklist (not executed): `docs/GATE_DOOR_V1_HARDWARE_OPTION_C_INSTALL_READY.md`
- Live Option B receipt: `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`
- AI criteria: `docs/GATE_DOOR_V1_AI_CRITERIA.md`

**One-liner:** Live optb push face is a box half-size `[0.008, 0.048, 0.15]` m (8 mm half-thickness, face ≈ 96 × 300 mm, contype 4) against hand spheres r=12 mm; panel on optb is contype 0; contact-drop after peak fits that thin −X engagement and is **not** a soft-pass; Option C (panel contype 2, push_face visual 0, hands bit 2) stays HELD; no plant change; no Door LOCK.
