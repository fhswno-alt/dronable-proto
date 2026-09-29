# Manufacturing — Dave solo handoff (usage limits)

**When:** Mon 28 Sep 2026 ~19:10 Europe/London (BST)  
**Owner:** Founding Manufacturing Lead  
**Purpose:** Keep MFG progress moving if Grok Bot hits usage limits. **Not** a greenlight / PO. Soft-pass off.

Live freeze sheet: `docs/MFG_FREEZE_STATUS_MONDAY.md`  
Factory one-pager: `docs/FACTORY_ONE_PAGER.md`  
Buy sheet (Google): https://docs.google.com/document/d/1w6tZ-UTNqlveMHpFS6vtwDNMyCJV9o0iXAH0SN3DL_Q/edit

---

## Current MFG state (do not invent)

| Item | State |
|------|--------|
| Foot STEP | **FROZEN** 145×86 · CAD stack ~4.5 mm · XML 16 mm = sim proxy only — **do not fab to 16 mm** |
| Companion plant | md5 `59cc408eda07037a58f92ad27da045d6` (locked with Controls/HW) |
| Lever prop | Fixed-bar STEP QC **PASS**; hinged STEP **DEFERRED**; panel STEP **N/A** |
| Path A / PO / China CM | **NO SPEND** until you explicitly ask after sim proves walk+fit+controls |
| Gate Q | Prefer FAIL iterate (Controls/AI) — **not** an MFG unlock by itself |

---

## What you can do alone (no bots)

### A. Keep sim honest (default)

1. Do **not** order soles, levers, kit refresh, or China CM from this sheet.
2. Do **not** invent soft-XY / friction / foot-lift / STEP dims to clear Gate Q.
3. If Controls/AI ask for plant: require a **named** geom ask + Hardware freeze note before any CAD touch.

### B. When you are ready to spend (Path A) — checklist

Only after **you** decide sim is good enough:

1. Open buy sheet + `docs/FACTORY_ONE_PAGER.md`.
2. Confirm AiNex **Standard** + HX bus only (no Feetech for unit one).
3. Live Hiwonder checkout → ship-to **SE1 4AG**.
4. Hard ceiling **$1.5k** landed; kit UK-landed ~$1.05–1.14k is the honest band.
5. On receipt: run QC outline in factory one-pager § QC (weigh, dims, Pi boot, HX map, E-stop, serials).
6. UK fab only after kit-matched drawings: face/eye mount, ankle bumpers, prop lever 250–300 mm AFF — **China CM = none** for unit one.

### C. Foot / lever fab (only if you open it)

| Ask | Files | Rule |
|-----|-------|------|
| Sole | `cad/m2_outsole/M2_outsole_145x86_*`, `M2_tread_145x86_*` · QC `docs/M2_FAB_QC_FROZEN_145.md` | Fab to **145×86 / ~4.5 mm**, not XML 16 mm |
| Sleeve fit | `docs/M2_ANK_ROLL_SLEEVE_FIT_QC.md` | Already **PASS** dry-fit |
| Lever prop | `cad/gate_f_lever/demo_prop_lever_275AFF_*` · QC `docs/GATE_F_LEVER_FAB_QC.md` | Research UK £30–80 — **NOT TO ORDER** until you ask |

### D. Vision Pro (later, off critical path)

When you want spatial preview: ask MFG/HW for USDZ of **frozen** foot + lever meshes (no fab/PO). Motion clips = Controls.

---

## If limits hit mid Gate Q

1. Leave plant frozen — Q Prefer FAIL is a **controller** problem unless Hardware freezes new dims.
2. Read `docs/MFG_FREEZE_STATUS_MONDAY.md` for curriculum D–P / Q park status.
3. Your only MFG decision is: **keep no-spend** (default) vs **explicit Path A greenlight**.
4. Ping Manufacturing Lead in DM when limits reset with: (a) any PO you placed, (b) any geom you want QC’d, (c) Path A yes/no.

---

## Do **not** do while waiting

- Soften Gate Q bars / invent plant to “look busy”
- Order Feetech / Orin / depth / China CM “for backup”
- Fab soles to the 16 mm sim contact thickness
- Assume a Prefer FAIL park = Path A unlock

---

## One-line MFG policy

**Parts stay frozen; spend stays off until you say otherwise. When bots are dark, protect the freeze and only advance Path A if you deliberately choose to.**
