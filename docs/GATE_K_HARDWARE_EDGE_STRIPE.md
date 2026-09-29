# Gate K — free-edge edge-stripe tell (Hardware)

**Status:** LOCK plant-of-record. Visual-only green free-edge stripe. Soft-pass forbidden. No M145 edit. No spring / Alt A / spend.

**AI lock (2026-09-28):** Gate K **LOCKED TRUE** (sim) on free-edge green stripe + stills Δ screen-X (B2: watch models miss upright translating lip). Curriculum **D–K TRUE**. **Not** full door-open. Companion md5 **`59cc408eda07037a58f92ad27da045d6`**. Ticks/beads stronger tell **deferred** (not required for this lock; optional for a later open-angle pack).

**Plant:** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`

| Item | Value |
|------|--------|
| Geom | `door_panel_free_edge_stripe` |
| Site | `door_panel_free_edge_site` |
| Parent | `door_panel_link` (+Y free edge; hinge −Y) |
| Local pose | `pos="0 0.110 0.17"` |
| Size (half) | `0.015 × 0.010 × 0.17` m |
| Material | `edge_tell` rgba `0.15 0.95 0.35 1` |
| Contact | `contype="0" conaffinity="0"` |
| Unchanged | panel hinge range/spring; lever; foot STEP; locked M145 |

## Deferred

Orange horizontal ticks + yellow end beads were briefly prototyped then **not** kept on the lock plant. Re-introduce only if a later gate’s watch needs orientation tells beyond Δ screen-X.
