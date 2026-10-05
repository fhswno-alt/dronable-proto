# FACTORY ONE-PAGER — Dronable Proto Path A

**Date:** Sun 27 Sep 2026 (London / BST)  
**Ship-to:** SE1 4AG London  
**Demo:** March 2027 — room walk + prop lever @ **250–300 mm** AFF  
**Buy sheet:** https://docs.google.com/document/d/1w6tZ-UTNqlveMHpFS6vtwDNMyCJV9o0iXAH0SN3DL_Q/edit

---

## Path A freeze status

**FROZEN** for spend until Dave asks. UK fab still waits on **kit-matched MJCF/drawings**.  
List price checked 5 Oct 2026. The old OD-2 "confirm live checkout" price flag is **resolved** by `docs/MFG_FIRST_BUILD_PRICE_SHEET.md`. Nothing is ordered.  
**China CM scope for unit one = NONE.** No placeholder RFQs.  
Actuators = **Hiwonder HX serial bus only** (UART 115200). **No Feetech STS/SMS** for unit one.  
**Controls:** current walk **REJECTED** as shippable claim until kit-matched MJCF; ~21 cm/s = control-model ceiling.

| BUY | UK FAB (thin, after kit match) | DO NOT BUY |
|-----|--------------------------------|------------|
| AiNex **Standard**, **24 DOF** + hands, Pi 5 **2GB — $829.99** (this week's MuJoCo pick). **$729.99 is the 20-DOF Starter** and does not match the walk plant. Standard 4GB is **$909.99**, later only if voice and vision move onto the kit | Face / eye mount | Orin, Hailo, 2nd Pi |
| HX-35H ×2 — **$18.99** ea | Ankle bumpers (TPU) | F/T ankles, depth, LiDAR |
| HX-12H ×1 — **$16.99** | Prop door+lever 250–300 mm AFF | LeRobot open biped |
| MG90S ×2 (eyes); bare **2.8" SPI IPS ILI9341** ~$9.50 | | Custom legs, TonyPi |
| Kit family onboard: HX-35H · **HX-35HM** (hip Z mag) · HX-12H | | Waveshare Pico-ResTouch |
| | | **Feetech / non-HX bus** |

---

## Envelope (quote / model / fab against this)

| Dim | Value |
|-----|-------|
| Size | **193 × 135 × 415 mm** |
| Mass | **2.45 kg** Standard (Starter 2.25 kg) |
| DOF / frame | **24** + hands · Al alloy |
| Walk | ~**21 cm/s** control-model ceiling (*not* shippable until kit-matched MJCF) |
| Cam / IMU / batt | **120° / 1 MP**, 2DOF head · **9-axis** IMU · **11.1 V 3500 mAh 5C** |
| Bus | UART serial, baud **115200**, pos/temp/voltage feedback |
| Compute | Pi 5 **2GB** on-body this week (bus/PD/IMU/E-stop/Xbox/eyes) — **no** on-Pi LLM/NN/Orin. **4GB ($909.99)** only if voice and vision move onto the kit |
| Eyes | ILI9341 ≤1–2 W + MG90S ×2 |
| On-body Pi continuous | **≤8–10 W** (AI) |

---

## Buy vs fab (one line)

**Buy the biped + HX spares from Hiwonder / UK eyes parts. Fab only face, bumpers, prop in the UK. China CM = none.**

---

## Cost headline (SE1 4AG)

The **Standard** kit is **24 DOF**. On the Hiwonder page (checked 5 Oct 2026), **$729.99 is the 20-DOF Starter**, which does not match the walk plant. This week's pick is Standard Pi 5 **2GB at $829.99**. Standard Pi 5 **4GB at $909.99** is an upgrade to revisit only when voice and vision move onto the kit. The old **~$1,050–1,100** landed band was built on $729.99 and is too low for a Standard kit.

Landed totals are in `docs/MFG_FIRST_BUILD_PRICE_SHEET.md`.

| Scope | Amount |
|-------|--------|
| Kit alone, UK landed (Standard 2GB) | **about £754–£785** |
| Full first build | **about £876–£1,026** |
| Hard ceiling | **$1.5k** |

---

## QC receipt checklist (outline)

1. Photos + packing list vs order / buy sheet.  
2. Weigh ~**2.45 kg**; tape **193 × 135 × 415 mm**.  
3. Battery V / no swell; charger present.  
4. Pi 5 boots; cam; 9-axis IMU.  
5. HX ID map; spot-check **HX-35HM** on hip Z.  
6. Stock walk 30 s stand then floor; current / heat / skate notes (*stock ≠ accepted demo*).  
7. Xbox teleop smoke test; E-stop = kill bus.  
8. Log serials; open HX spares only after baseline.  
9. Caliper head + sole → kit-matched MJCF + release **UK** fab drawings (not China CM).  
10. Lever gauge @ 275 mm AFF for reach test.

---

## CM / fab scope

| Who | Scope |
|-----|-------|
| **China CM** | **NONE** (unit one) |
| **UK fab** | Face/eye mount, ankle bumpers, prop door+lever only — after kit-matched drawings |

---

## Power (March)

Battery **primary** (~39 Wh; walk EST 25–50 W → short demo blocks). Bench **tether** lab-only. Pi continuous on-body **≤8–10 W**.

---

*Detail: `HARDWARE_SYSTEM_DESIGN_BRIEF.md`*
