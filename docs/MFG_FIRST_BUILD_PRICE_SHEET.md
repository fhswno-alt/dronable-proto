# MFG: first physical build price sheet (AiNex kit + frozen M2 foot)

**Pricing only. Nothing ordered. No PO until Dave asks, after the sim proves walk and fit.**
No cart, no quote request, no vendor contact, no sign-in was used to make this sheet.

**Prepared:** Mon 5 Oct 2026, ~10:40–10:55 BST (Europe/London) · Owner: Founding Manufacturing Lead · Ship-to assumption: SE1 4AG London

**Frozen spec priced against (confirmed by Hardware + Controls):** walk plant md5 `71b2c86d133ebc603f58b99c53e496f3`; foot contact box 145 × 86 mm (half-size 0.0725 × 0.043 × 0.008 m), pos 0.030 0 -0.018, friction 1.6; legs ±2.1 Nm, arms/head ±0.7 Nm; no added battery; kit camera only. **Not priced on purpose:** a stronger or bigger foot, stronger servos, a second camera, lidar, an extra battery, a GPU, or any door/lever parts (the door is cancelled).

> **md5 check (resolved):** `origin/main` has `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` at `71b2c86d133ebc603f58b99c53e496f3` (after the camera-pose-only merge). The `fc94709c…` hash came from a stale local checkout at `07d7135`. The contact box (145×86, pos 0.030 0 −0.018, friction 1.6) and torque limits match the spec line. No price changes.

Foot fab target is the **CAD stack**, not the sim box: outsole plate 145 × 86 × **3 mm** (pocket 137 × 78 × ~2.2 mm, 0.8 mm floor) + tread 145 × 86 × **1.5 mm** TPU, so ~4.5 mm total. **Do not fab to 16 mm** (16 mm is only the sim contact proxy). Files: `cad/m2_outsole/M2_outsole_145x86_meshAABB_notOEM.{step,stl}`, `cad/m2_outsole/M2_tread_145x86_TPU_notOEM.{step,stl}`. Qty priced: **4 outsoles + 4 treads** (1 pair + 1 spare pair; the pocket is symmetric, so one STEP covers L and R). STL volumes measured on the box: outsole 13.79 cm³, tread 18.70 cm³.

---

## Price table

All prices are exactly as shown on the source on the date seen. FX for the GBP column comes from the ECB reference rate dated 2 Oct 2026 (fetched 5 Oct 2026 via api.frankfurter.dev): **1 USD = £0.75753**, **1 EUR = £0.85033**. Card FX fees are not included.

| # | Item | Option | Price as shown | Currency | Source URL | Date seen | Note |
|---|------|--------|----------------|----------|------------|-----------|------|
| K1 | AiNex kit (Hiwonder official) | Starter Kit / Raspberry Pi 5 (2GB), 20 DOF, no hands | 729.99 | USD | https://www.hiwonder.com/products/ainex | 2026-10-05 | ≈ £552.99. **Doesn't match the plant**: the frozen plant has 24 actuators (12 legs + 12 arms/head) and Starter has 20 DOF. Listed only because it's the headline "from" price. |
| K2 | AiNex kit (Hiwonder official) | Starter Kit / Pi 5 (4GB) | 809.99 | USD | https://www.hiwonder.com/products/ainex | 2026-10-05 | 20 DOF, same mismatch |
| K3 | AiNex kit (Hiwonder official) | Starter Kit / Pi 5 (8GB) | 899.99 | USD | https://www.hiwonder.com/products/ainex | 2026-10-05 | 20 DOF, same mismatch |
| K4 | AiNex kit (Hiwonder official) | **Standard Kit / Pi 5 (2GB)**, 24 DOF + hands, SKU 21020137 | **829.99** | USD | https://www.hiwonder.com/products/ainex | 2026-10-05 | ≈ **£628.74**. **Cheapest kit that matches the 24-DOF plant.** In stock. Free standard shipping to the UK on orders over US$499 (DHL/UPS/FedEx), FOB Shenzhen, so duty and VAT are paid by us. |
| K5 | AiNex kit (Hiwonder official) | **Standard Kit / Pi 5 (4GB)**, SKU 21020117 | **909.99** | USD | https://www.hiwonder.com/products/ainex | 2026-10-05 | ≈ £689.34. Prior repo preference (OD-6: Pi 5 4GB). Free shipping to the UK. |
| K6 | AiNex kit (Hiwonder official) | Standard Kit / Pi 5 (8GB), SKU 21020118 | 999.99 | USD | https://www.hiwonder.com/products/ainex | 2026-10-05 | ≈ £757.52. Not needed (no on-Pi NN, no GPU). |
| K7 | AiNex kit (EU reseller OpenELAB) | Standard Kit / Pi 5 (4GB) | 1,219.35 | EUR (shown incl. VAT) | https://openelab.io/products/ainex-pi5 | 2026-10-05 | ≈ £1,036.85 before UK import VAT/duty. Ships free to the UK over €300. The page says UK duties and taxes are the buyer's job. It does not say whether EU VAT comes off for UK delivery. Ships from China 5–10 working days, with a holiday delay until 8 Oct. Costs more than K5. |
| K8 | AiNex kit (EU reseller OpenELAB) | Standard Kit / Pi 5 (2GB) | 1,116.85 | EUR (shown incl. VAT) | https://openelab.io/products/ainex-pi5 | 2026-10-05 | ≈ £949.69. Same caveats as K7. |
| K9 | AiNex kit (EU reseller OpenELAB) | Starter Kit / Pi 5 (2GB) | 988.65 | EUR (shown incl. VAT) | https://openelab.io/products/ainex-pi5 | 2026-10-05 | 20 DOF mismatch |
| D1 | UK import VAT on kit | 20% of (goods + shipping + duty) | 20% | — | https://www.trade-tariff.service.gov.uk/api/v2/commodities/9503007000 | 2026-10-05 | **ESTIMATE.** Shipping is free, so customs value ≈ goods price. K5 → ≈ £137.87 at 0% duty, ≈ £143.38 at 4% duty. |
| D2 | UK import duty on kit | 0% to 4% (depends on commodity code) | 0–4% | — | https://www.trade-tariff.service.gov.uk/api/v2/commodities/9503007000 · https://www.trade-tariff.service.gov.uk/api/v2/commodities/9503009990 · https://www.trade-tariff.service.gov.uk/api/v2/commodities/8479899790 | 2026-10-05 | **ESTIMATE.** UK Global Tariff third-country rates: 9503 00 70 00 "toys put up in sets" = 4.00%; 9503 00 99 90 = 0.00%; 8479 89 97 90 machines n.e.s. = 0.00%. The 35% "additional duty" applies to RU/BY only, not China. The carrier/HMRC decides the actual code. |
| D3 | Carrier clearance (disbursement) fee | DHL Express: greater of £11.00 or 2.5% of duty+VAT | 11.00 | GBP | https://ebilling.dhl.com/gpp/web/faq/gb/ | 2026-10-05 | **ESTIMATE.** Applies if Hiwonder ships by DHL. Duty+VAT here is < £440, so it's the flat £11. UPS/FedEx fees: not found (Hiwonder picks the carrier). |
| S1 | Spare servo, HX-35H (legs) | "HX-35H Servo Pack" ×2 | 18.99 each | USD | https://www.hiwonder.com/products/hx-35h | 2026-10-05 | Same servo as the kit (not stronger). 2 × $18.99 = $37.98. Prior buy list qty 2. Kit page packing list has no spare servos. |
| S2 | Spare servo, HX-12H (arms/hands) | "HX-12H Servo Pack" ×1 | 16.99 | USD | https://www.hiwonder.com/products/hx-12h | 2026-10-05 | Same servo as the kit. Prior buy list qty 1. |
| S3 | Spare servo, HX-35HM (hip Z, mag encoder) (optional) | "HX-35HM Servo Pack" ×1 | 35.99 | USD | https://www.hiwonder.com/products/hx-35hm | 2026-10-05 | Optional. Brief says "spare only if fail". Only in the high-with-option figure below. |
| F1 | Outsole ×4 | **3DPRINTUK** (London) XYZ estimator, **SLS Nylon PA12**, Natural finish, White, 145×86×3, qty 4, **Economy** 6–12 wd | 30.76 (7.69/unit) | GBP ex VAT | https://app.3dprint-uk.co.uk/portal/estimate/ | 2026-10-05 | Public dimension-only estimator. **No upload, no login.** Trade (14–25 wd) £27.60; Express (2–4 wd) £46.72; **Black** adds a colour charge: Economy £33.84. They say it's an estimate and a firm quote needs an STL upload (not done). Delivery not shown. |
| F2 | Tread ×4 | **3DPRINTUK** XYZ estimator, **SLS Flexible TPU**, Natural, White, 145×86×1.5, qty 4, **Economy** | 22.40 (5.60/unit) | GBP ex VAT | https://app.3dprint-uk.co.uk/portal/estimate/ | 2026-10-05 | Trade £19.96; Express not offered for TPU; **Black** Economy £24.64. Shore hardness of their SLS TPU vs the 95A spec isn't confirmed. Delivery not shown. |
| F3 | Outsole ×4 | **PrintBroz** (UK) instant calculator, **PETG**, Standard, 100% infill, 145×86×3, qty 4 | 5.00/item → **20.00** total | GBP, excl. shipping | https://printbroz.com/quote.html | 2026-10-05 | Public calculator, **no upload, no login**. It prices on the rectangular box volume (37.4 cm³ × £0.06 = £2.24, then lifted to the £5.00 per-item minimum). VAT status not stated. Exact quote needs an upload (not done). |
| F4 | Tread ×4 | **PrintBroz** instant calculator, **TPU**, Standard, 100% infill, 145×86×1.5, qty 4 | 5.00/item → **20.00** total | GBP, excl. shipping | https://printbroz.com/quote.html | 2026-10-05 | 18.7 cm³ × £0.08 = £1.50, lifted to the £5.00 minimum. TPU grade/shore not stated. |
| F5 | Outsole ×4 + tread ×4 | **Hark Tech** (UK) published rate card: PETG 5.52p/g, TPU 8.05p/g, + £1.15 per machine hour; floors £4.60/part, £6.90/order; TPU order min £11.50 | rate card only | GBP (not VAT registered) | https://harktech.co.uk/pricing.html | 2026-10-05 | **Firm price needs an upload (not done).** Rate-card arithmetic only: material ≈ £3.87 (4 × 17.5 g PETG) + ≈ £7.29 (4 × 22.6 g TPU) at 100% solid. If the £4.60 floor applies per copy, the floor is ≥ £18.40 + ≥ £18.40. Machine-hours unknown without slicing, and there's a +25% dense-print surcharge at ≥ 80% infill. Delivery £2.85–£5.55, free over £40. Not used in the totals. |
| C1 | Floor mat (not in kit packing list) | Arkmat 10 mm interlocking EVA, 4 × 600×600 mm tiles | 15.95 | GBP | https://www.arkmat.co.uk/10mm-interlocking-600mm-x-600mm-black-eva-mats-checker-pattern.html | 2026-10-05 | Fall protection for the stand/bench and first steps. It's soft foam, so it **isn't** the sim floor (friction 1.6). Walk QC still happens on the hard floor. Delivery not shown. |
| C2 | Zip ties for foot attach (bond + 4 rim slots per QC) | Screwfix Essentials 100 × 2.5 mm, pack of 100 | 0.99 | GBP inc VAT | https://www.screwfix.com/p/essentials-cable-ties-black-100mm-x-2-5mm-100-pack/65467 | 2026-10-05 | Kit accessory bag has screws/cables, not ties. 2.5 mm width vs the slot width still needs checking against the STEP. |
| C3 | Adhesive for tread-to-plate bond | Loctite Powerflex Super Glue Gel 3 g (rubber-flexible CA) | 3.33 (was 4.46) | GBP ex VAT | https://www.rapidonline.com/loctite-2633191-powerflex-super-glue-gel-tube-3g-84-4551 | 2026-10-05 | One tube is enough for 4 bonds. Bonding to PA12 and TPU should get a test coupon first. |
| C4 | UK plug adapter for kit charger (conditional) | Masterplug Visitor-to-UK adaptor, 3-pack (Argos) | 11.99 | GBP | https://www.argos.co.uk/product/8556295 | 2026-10-05 | The charger (12.6 V 2 A) is **in the kit**. The packing-list photo shows a flat-pin non-UK plug, so buy this only if the kit arrives without a UK plug. Price comes from the search-engine copy of the Argos page, because Argos blocks automated fetch (403). Check the price before buying. |

**In the kit per the official Standard packing list (so not priced):** robot with the 11.1 V 3500 mAh 5C LiPo fitted (spec table), 12.6 V 2 A charger (DC5.5×2.5), wireless handle, card reader, 6.3 cm ball, 3 × 3.5 cm blocks, screwdriver + accessory bag (screws and short servo cables), user manual. Kit head camera (120°, 1 MP, 2-DOF) is included and is the only camera.

**Material picks (one line each):**
- **Outsole: SLS Nylon PA12.** It's tough and isotropic, and holds the 137 × 78 pocket and the ~1 mm/side sleeve clearance better than FDM. PETG FDM (PrintBroz) is the cheap fallback for a fit-check pair.
- **Tread: TPU (95A per the frozen QC spec).** It gives flexible grip that survives impacts and bonds to the plate. Confirm the vendor's shore hardness before ordering.

**Fab QC flag (no design change made):** the outsole's 0.8 mm pocket floor is under 3DPRINTUK's stated minimum wall (1.5 mm recommended, 1.0 mm without finishing; text on the estimator page). Hardware should know before an upload or quote. Planform and stack stay frozen.

---

## Total range (GBP, estimate)

| Scenario | Lines | Total |
|----------|-------|-------|
| **Low** (cheapest credible) | K4 Standard/Pi 5 2GB landed at **0% duty** (£628.74 + VAT £125.75) + D3 £11 + S1/S2 spares landed (£41.64 + VAT £8.33) + F3+F4 PrintBroz £40.00 + C1 £15.95 + C2 £0.99 + C3 £4.00 inc VAT | **≈ £876** |
| **High** (recommended config) | K5 Standard/Pi 5 4GB landed at **4% duty** (£689.34 + £27.57 + VAT £143.38) + D3 £11 + S1/S2 spares landed at 4% (£51.97) + F1+F2 3DPRINTUK Economy **black** £58.48 ex VAT → £70.18 inc VAT + C1–C3 + C4 adapter £11.99 | **≈ £1,026** |
| High + optional HX-35HM spare (S3) | as High + £34.02 landed | ≈ £1,060 |

**Range: ≈ £876 – £1,026** (≈ £1,060 with the optional HX-35HM spare).

Assumptions:
1. Hiwonder official store, one consignment, free shipping (order > US$499), FOB Shenzhen. **Import VAT 20% and duty 0–4% are estimates.** Customs value = goods (shipping free). DHL £11 disbursement assumed; UPS/FedEx fees not found.
2. FX at ECB 2 Oct 2026 reference (USD £0.75753, EUR £0.85033). No card FX margin or bank fees.
3. Foot-part delivery charges for 3DPRINTUK and PrintBroz are **not included** (not shown without checkout). PrintBroz VAT status unknown: if 20% is added, Low rises by about £8.
4. 3DPRINTUK and PrintBroz figures are their own public estimators (dimension-based), not firm quotes. Firm quotes need an STL upload, which wasn't done.
5. Excludes eyes parts (ILI9341, MG90S) from older buy lists. They're outside this first-build scope and nothing in the frozen spec calls for them.
6. **Repo correction:** `FACTORY_ONE_PAGER.md` and `HARDWARE_SYSTEM_DESIGN_BRIEF.md` list "AiNex Standard, Pi 5 4GB, $729.99". On the live store today, **$729.99 is Starter / Pi 5 2GB**. Standard / Pi 5 4GB is **$909.99** and Standard / Pi 5 2GB is $829.99. The older "~$1,050–1,100 landed" band was built on the $729.99 figure. At today's numbers K5 lands at ≈ £827–£860 (≈ $1,092–$1,136) before spares and fab.

---

## Not found / needs login or upload

- **Exact landed tax:** the commodity code (0% vs 4%) is decided at import. UPS/FedEx clearance fees not found.
- **Spare 11.1 V 3500 mAh battery:** no public listing on Hiwonder; their FAQ says to email support. Not priced, and out of scope anyway (frozen spec: no added battery).
- **UK reseller listings:** Toys Central UK (toyscentral.uk) AiNex Standard 4GB page now returns **404** (an old search snippet showed "$1,703.35", unverifiable). Ubuy UK listing shows **no price** without interaction. Amazon UK: no AiNex listing found in search. RoboSavvy / The Pi Hut / Cool Components: no AiNex listing found.
- **OpenELAB second listing** (`/products/hiwonder-ainex-standard-pi5-2gb`): a search snippet showed different prices (€1,317.75 / €1,402.45 / €1,500.35). The page JSON didn't load, so it's not used. The UK EU-VAT treatment for OpenELAB isn't stated.
- **Firm foot-part quotes (need STL upload, not done):** 3DPRINTUK firm quote, PrintBroz exact quote, Hark Tech estimator, 3Descu (EU, Romania; upload but no account), Craftcloud (EU; upload). Xometry / Protolabs / Hubs / Shapeways need an account, so not tried.
- **Foot-part delivery charges:** 3DPRINTUK and PrintBroz don't show them before checkout.
- **Kit charger plug type for UK orders:** not stated on the product page (photo shows a non-UK flat-pin plug).

*No commit, no push. Generated from public pages only.*
