# Next architecture — contact / geometry honesty (sim)

**When:** Sun 27 Sep 2026 ~23:24 BST  
**Why:** HX authority envelope hard-falsified (`AUTH_ENVELOPE_NOTE.md`). Saturation cleared at k=10 but T75 skate **worsened**. Open-loop + fixers + dual-T + authority all dead under locked M145 collision proxy.  
**Question:** Is Gate E blocked by **dishonest contact geometry** (16 mm collision box vs CAD ~4.5 mm; planform / CoP support) rather than gait or HX clips?  
**Sim only. No spend talk.**

## Locked falsifiers (do not reopen)
| Class | Status |
|-------|--------|
| Open-loop CPG + VIK + μ/T micro-sweep | Gate E FALSE |
| Residual / WBC / MPC / dual-T outer | HARD-FALSIFIED |
| HX-authority-upsized open-loop (k→10, μ A/B) | HARD-FALSIFIED |

## Goal
On **sim plant variants** (geometry honesty only), with assist/freeze/xfrc OFF and kit HX 1.0×:
1. Hold T88 clean_ss on each variant (or document floor break).
2. Score Gate E at T≤0.75 with GRO01 outer (and CSF50 control).
3. Unlock Gate E **or** hard-falsify contact/geometry class on M145 family.

## Plant grid (Hardware owns XML; Controls scores; AI locks)

| Tag | Plant | Intent |
|-----|-------|--------|
| GEO00 | Locked `ainex_controls_m2_145.xml` | Control (must match AUTH01 FAIL) |
| GEO01 | Existing `ainex_controls_m2_145_cadsole.xml` (~4.5 mm contact height, 145×86 planform) | Vertical honesty — now scored for **Gate E**, not residual-only |
| GEO02 | Hardware: planform A/B — e.g. **160×90** or **+10 mm** length, same 16 mm box *or* cadsole z | Support polygon honesty |
| GEO03 | Hardware: optional soft — compliance / sole mesh closer to CAD tread (still MuJoCo) | Contact shape honesty |

Keep sole bottom / `SOLE_OFFSET` consistent per `docs/PLANT_CADSOLE_AB.md`. No Path/spend framing in plant notes.

## Control / score
| Item | Spec |
|------|------|
| Outer | GRO01 primary; CSF50 control on GEO00/01 |
| Authority | k_auth=1.0 only (HX envelope closed) |
| Inner | None until a GEO row gets Gate E |
| Tags | GEO00–03 × {T88, T75}; Gate E criteria; continuous video on any claim |
| Pass | Gate E TRUE on any honesty plant |
| Hard-falsify | All GEO plants fail Gate E with T88 held (or T88 broken without Gate E win) → contact class dead; next architecture TBD (still sim) |

## Stop rule
If cadsole + planform A/B + optional soft all fail Gate E → **contact/geometry honesty HARD-FALSIFIED** for Gate E on AiNex M145 family under kit HX. Do not reopen authority or inner scrubbers. AI ships next cospec then.

If a GEO plant unlocks Gate E → **lock plant claim** (which geometry); Controls freezes that XML for further work; still no spend talk until Dave asks.

## Owner
- **Hardware:** ship GEO02/03 XML (and confirm GEO01 cadsole still valid); note exact mm deltas
- **Controls:** score table + videos
- **AI:** cospec + score/lock
- **Manufacturing:** silent unless a winning plant implies a fab note later (Dave-gated)

## Artifacts expected
`CONTACT_GEOMETRY_NOTE.md`, `CONTACT_GEOMETRY_TABLE.json`, GEO00+ under `previews/ainex_walk/iterate/`.

## Verdict LOCKED 2026-09-27 ~23:36 BST (AI score)

**Contact / geometry honesty HARD-FALSIFIED** for Gate E on AiNex M145 family under kit HX 1.0×.

Evidence: `previews/ainex_walk/iterate/CONTACT_GEOMETRY_NOTE.md`, `CONTACT_GEOMETRY_TABLE.json`.
- GEO00 T75 GRO01 matches AUTH01 FAIL (stx≃0.12, p95≃0.64, skate)
- GEO01 cadsole: broke T88 GRO01 bout (0.061≪0.10); T75 still skate — no Gate E
- GEO02a/b planform: T88 may hold; T75 still skate (GEO02a best stx hint but FAIL)
- GEO02c / GEO03: T88 broken and/or T75 FAIL
- Gate E = **0** across pack

**Sim fact locked:** dishonest 16 mm box / planform / soft solref is **not** the Gate E unlock under frozen GRO01/CSF50 + kit HX. Do not reopen k_auth or residual/WBC/MPC.

## Next architecture (sim only)
Plant-specific outer co-opt on best planform hint (GEO02a 160×90) — frozen-outer GEO score left a mild stx gap; dual-T was shared-shape on locked M145 only. Spec: `docs/PLANT_OUTER_COOPT_COSPEC.md`.
