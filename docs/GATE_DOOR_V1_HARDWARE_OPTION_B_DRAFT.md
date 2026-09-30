# Gate Door v1 — Hardware Option B (ARCHIVE)

**ARCHIVE** Wed 30 Sep 2026 ~11:15 Europe/London (BST) — Dave ACK installed Option C as the live companion. This pack’s file `…_gate_f_optb.xml` md5 `ddf084cdac71cb0998aa6a44a65594c0` is **KEPT**. Bytes were **not** overwritten. It is **not** live. Receipt: `docs/GATE_DOOR_V1_HARDWARE_INSTALL.md`.

**Was INSTALLED** Wed 30 Sep 2026 ~01:13 Europe/London (BST) — Dave voice ACK. That live pointer is superseded. Soft-pass **OFF**. **No spend.** No Prefer FAIL Door SCORE claim. No Door LOCK.

Draft history below is the ~00:20 BST feasibility pack. Install did **not** edit the XML (md5 unchanged).

**When (draft):** Wed 30 Sep 2026 ~00:20 Europe/London (BST)  
**Who:** Founding Hardware Engineer  
**Trigger:** Dave voice UPDATE — hold Option C; ask Option B / bit-4 feasibility  
**Scope:** SIM-ONLY companion fork. Soft-pass **OFF**. **No spend.**

---

## Live freeze (Option B demoted)

| Item | Path | md5 | State |
|------|------|-----|-------|
| **Live Door v1 (Option C)** | `…_gate_f_optc.xml` | `6a3d4a70d4797b806dcc2580f46468aa` | **INSTALLED** plant-of-record |
| Option B (this pack) | `…_gate_f_optb.xml` | `ddf084cdac71cb0998aa6a44a65594c0` | **ARCHIVE** — bytes unchanged; not live |
| Option A archive | `…_gate_f_push.xml` | `adb24309b489d56615c194e92676d040` | **ARCHIVE** — bytes unchanged; not live |
| Lever-era archive | `…_gate_f.xml` | `59cc408eda07037a58f92ad27da045d6` | **ARCHIVE** |
| Walk M145 | `…_m2_145.xml` | `fc94709c84f5598d4474ecfc4bb41fdc` | **KEPT** |

---

## Option B fork (this pack)

| Item | Value |
|------|-------|
| Path | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f_optb.xml` |
| md5 | `ddf084cdac71cb0998aa6a44a65594c0` |
| State | **ARCHIVE** (demoted ~11:15 BST 30 Sep 2026) — was INSTALLED ~01:13 BST; bytes unchanged |
| Push face | Same thin −X geom as Option A; **`contype=4 / conaffinity=4` (bit 4)** |
| Hands | `l/r_hand_contact` retargeted to **bit 4** (exclusive push-face pairing) |
| Bit 2 | **Free** for future knob/lever A/B |
| Panel / lever / feet | `door_panel` visual 0 · lever 0 · feet bit 1 **KEPT** |
| Hinge / spring | ±30° · 0.05 / 0.015 **KEPT** · no latch |

Load check: MuJoCo OK; hand×push_face pairs; hand×panel / lever / foot do **not** pair.

---

## Feasibility

**YES — bit-4 push-face contact is buildable and scoreable** under the same sustained-contact honesty gate.

- MuJoCo bitmask: bit 4 = value `4` on both push face and hands.
- Hands retarget (not dual affinity) because lever is already contype 0 on live Door v1 — no need to keep bit 2 on hands until a future knob A/B restores lever contact.
- Geometry area unchanged vs Option A (still thin −X face) — B is a **sensing / credit isolation** lever, not a bigger panel (that’s C).

---

## Scoring plan (Controls after Dave ACK install)

| Rule | Detail |
|------|--------|
| Contact pair | `l/r_hand_contact` × `door_panel_push_face` (bit-4 only) |
| Sustained gate | ≥**8** consecutive contact frames before any coupled open credit |
| Anti-phantom | Prefer FAIL if `|Δθ|` rises while contact=n (or before sustained) |
| Open / closed / hold | Bars **KEPT**: closed ≤2° · open ≥25° · hold ≥20° · soft-pass **never** |
| Continuous watch | Must show hand×push_face **before and through** open (A05/B02 lesson) |
| Not scored | Lever contact / lever hinge / latch / scripted panel qpos |

---

## Honesty caveat (Hardware)

Option B **does not by itself** stop phantom panel rise. It isolates push-face contact onto bit 4 so credit can’t be confused with future bit-2 knob paths and so the named contact geom is unambiguous. Prefer FAIL still needs **contact-before-credit** + continuous watch (lab pattern: Spot/ARMAR force/contact gates). Pair B install with those Controls gates — not soft-pass, not mid-score invent.

---

## Install semantics (executed ~01:13 BST 30 Sep 2026; demoted ~11:15 BST)

1. Point live companion at `…_gate_f_optb.xml` (Option A file bytes **not** overwritten; `gate_f_push` is **ARCHIVE**). **Done, then demoted.**  
2. Update freeze sheets + receipt. **Done** at 01:13; live pointer moved to Option C at 11:15.  
3. Controls Prefer FAIL family stays on existing bars (closed ≤2° · contact ≥0.3 s · open ≥25° · hold ≥1 s @ ≥20° · 2/2). Soft-pass **OFF**. No Door SCORE claim from this install.  
4. Option C was later **INSTALLED** (Dave ACK ~11:15 BST). This Option B file is **ARCHIVE**.

**One-liner:** Option B bit-4 fork **ARCHIVE** md5 `ddf084cd…` (bytes unchanged); live is Option C `gate_f_optc` `6a3d4a70…`; Option A `adb24309…` **ARCHIVE**; walk KEPT; soft-pass off; park off; no spend; no Door SCORE claim.
