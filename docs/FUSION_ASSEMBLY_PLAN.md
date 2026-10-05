# Fusion / clickable assembly plan — Hiwonder AiNex Standard

**Date:** 27 Sep 2026 (Europe/London)  
**Goal:** Articulated (kinematics) review assembly so clearance / reach / interference are visible before any Path A spend. The **Standard** kit is **24 DOF**. This week's pick is Pi 5 **2GB at $829.99**. **4GB is $909.99**, later only if voice and vision move onto the kit. **$729.99 is the 20-DOF Starter** and does not match the walk plant. Kit alone, UK landed, **about £754–£785**. Full first build **about £876–£1,026**. See `docs/MFG_FIRST_BUILD_PRICE_SHEET.md`. The old ~$1.05–1.14k band used the $729.99 figure.  
**Envelope (public):** 193 × 135 × 415 mm, 2.45 kg, 24 DOF, HX bus servos (HX-35H / HX-35HM / HX-12H).  
**Constraint:** No Onshape / Fusion / SolidWorks MCP on this agent box — assembly work happens on Dave’s CAD seat or via files handed back into `/workspace/dronable-proto`.  
**Status:** Research only. **No purchase. No assembly share link claimed** (none found).

---

## Executive recommendation

| Item | Value |
|------|--------|
| **Recommended path** | **A** — official kit STEP (preferred) or official URDF meshes → Fusion (or Onshape) Revolute / As-Built joints + Motion Study |
| **Pre-spend unlock** | Ask Hiwonder for **evaluation STEP/URDF without order** (sales/support). Parallel interim: **C** (existing MuJoCo capsules). |
| **Single most important access ask** | **Hiwonder evaluation CAD package (STEP and/or `ainex_description` URDF+meshes) released without requiring a paid order number** — email `support@hiwonder.com` (and sales) for pre-purchase founder review. |
| **Estimated time to reviewable assembly** | **4–12 h** after CAD files land (Fusion joints for ~24 DOF + screenshots/Motion Study). **Blocked indefinitely** until STEP/URDF arrives. Same-day interim (C): **1–3 h** to improve capsule MJCF visuals only. |

---

## 1. Available CAD / model sources (+ URLs)

### 1.1 Official product & dimensions (public)

| Source | URL | What you get |
|--------|-----|----------------|
| Product page | https://www.hiwonder.com/products/ainex | Specs, FAQ. Checked 5 Oct 2026: Standard **24 DOF** is **$829.99** (Pi 5 2GB) and **$909.99** (Pi 5 4GB). **$729.99 is the 20-DOF Starter**, not the walk plant |
| OpenELAB mirror | https://openelab.io/products/ainex-pi5 | Same gating language for models/source |
| Wiki (Pi 5 docs) | https://wiki.hiwonder.com/projects/AiNex/en/raspberry-pi5-version/ | Tutorials; §5.3 Simulation Model & URDF |
| Wiki Appendix | https://wiki.hiwonder.com/projects/AiNex/en/raspberry-pi5-version/docs/Appendix.html | Links to Google Drive resources |

**Public envelope (confirmed):** Standard 193×135×415 mm; 24 DOF; 2.45 kg; HX-35H / HX-35HM / HX-12H; Pi + ROS1 (not ROS2).

### 1.2 Official STEP / SolidWorks / Fusion files

| Asset | Public? | How to get | Notes |
|-------|---------|------------|-------|
| **STEP (`.step` / `.stp`)** | **No** | Product FAQ: provide **order number** → `support@hiwonder.com`; they send STEP | Explicit FAQ answer: *“Where can I obtain the 3D model (URDF/SDF)… send you the **step. file**”* after order info |
| SolidWorks native | **Not found** | Same support channel if they ship SW assemblies | No public GrabCAD / TraceParts / Thingiverse AiNex STEP found |
| Fusion native | **Not found** | Would be built by us from STEP | No official Fusion package |

**Conclusion:** Prefer official STEP once accessible. **Do not claim a public STEP download URL** — none exists.

### 1.3 Official URDF / Gazebo / ROS meshes

| Asset | Public? | Location / URL | Notes |
|-------|---------|----------------|-------|
| URDF/xacro lessons (PDF) | **Yes** | Drive folder [3. Simulation Model & URDF](https://drive.google.com/drive/folders/11BTsaX1hs_hhckoduHm42VXjUTDB72FM) under [AiNex Resources](https://drive.google.com/drive/folders/1kyhah0bdW4d8omzYjot0Kq2f50l5GYcB?usp=sharing) | Lessons 1–4 PDF + video links; **not** the mesh package itself |
| Robot package layout (docs) | Described | On kit / in VM: `ros_ws/src/ainex_simulations/ainex_description/urdf/` — files `ainex.xacro`, `ainex.urdf.xacro`, `gazebo.xacro`, `materials.xacro`, `transmissions.xacro`; visuals use **mesh** files per link | Wiki §5.3 documents joint naming (e.g. `r_hip_yaw`) and structure: ~6 joints/leg, arms, head camera link |
| `AiNex_VM.zip` (~7.44 GB) | **Link is public on Drive** | [00. Virtual Machine System Image…](https://drive.google.com/drive/folders/1n0mJ1EGY587Ktq7iD7NOKdoPrxlyy8oE) | Wiki says URDF lives inside this VM. **Policy conflict:** Appendix “Source Code” folder only hosts [Important Notice](https://drive.google.com/file/d/1EnIp84ktzm05tmZpSUMbWOJ5vqcyKW78/view) stating system image & source require **order number** via support. Treat VM download as **order-gated IP** even if the folder link is open — prefer support release or evaluation ask. |
| Source Code Drive folder | **Gated** | https://drive.google.com/drive/folders/1pTMv1j5dwsnT0o_hQUruhQHUNdPgjggN | Only the Important Notice PDF; no zip without order |

**No public standalone `ainex_description` GitHub** with meshes was found. Community repos (`foundway/AinexPsi`, `deushon/hardware-modificaton-ainex`, `nesdeq/ainex-api`, `BanKiHyeon/learning-ainex`) are control/mods — **not** full official CAD.

### 1.4 Onshape / GrabCAD / third-party assemblies

| Source | Result |
|--------|--------|
| Onshape public AiNex | **None found** |
| GrabCAD / Thingiverse AiNex STEP | **None found** |
| HBRC hardware mods | Addon STLs only (e.g. GPS hat) — not full robot |

### 1.5 Local interim (this repo — **not** kit-matched)

| Path | Role |
|------|------|
| `mujoco/dronable_v0.xml` | Capsule/box MJCF, AiNex-*class* envelope (~0.41 m), walking core DOF + neck; **approximate** |
| `cad/dronable_v0_body.step`, `cad/dronable_v0_assembly.step` | CadQuery parametric STEP — importable into Fusion, **no real kit geometry** |
| `scripts/build_cad.py` | Regenerates local STEP/STL/GLB |

Controls/Hardware briefs already treat walk numbers as **control model only** until kit-matched MJCF/STEP.

---

## 2. Fusion 360 (personal / web) — STEP import & joints

### 2.1 Can Fusion import STEP and create joints?

**Yes (desktop Fusion).**

1. **Import:** Data Panel → Upload / Insert → STEP (`.stp` / `.step`). Imported assemblies arrive as **dumb geometry** (no mates/joints preserved).
2. **Prepare:** Convert bodies → **Components**; **Rigid Group** fixed subassemblies (e.g. foot plate + servo horn if separate).
3. **Joints (built-in — no paid kinematics plugin required):**
   - **As-Built Joint** — preferred when STEP already placed correctly (typical vendor dump).
   - **Joint** — if parts must be repositioned.
   - Motion types for AiNex servos: mostly **Revolute**; lock non-moving hardware with **Rigid**.
   - **Edit Joint Limits** using HX ranges (HX-35H/12H ~0–240° / 0–1000 counts; HX-35HM hip mag encoder up to 360° class).
4. **Review tools (native):** Animate joint, **Motion Study** (Assemble/Animate space), interference / section views for clearance & reach.

**URDF exporter plugins (optional, not required for founder clickable review):**

| Plugin | URL |
|--------|-----|
| fusion2urdf | https://github.com/syuntoku14/fusion2urdf |
| SpaceMaster85 fork | https://github.com/SpaceMaster85/fusion2urdf |
| fusion2urdf-ros2 | https://github.com/dheena2k2/fusion2urdf-ros2 |

Note: exporters prefer standard **Joint** over As-Built for reliable export.

### 2.2 Personal Use vs commercial / web

| License | Fit for this review |
|---------|---------------------|
| **Fusion for Personal Use** (free) | STEP import + joints generally available, but **Special Terms: non-commercial / home only**; not for company environments; limited import/export types; 10 editable docs. **Poor fit for startup founder review.** |
| **Paid Fusion seat** (monthly/annual) | Correct for company use; full translators & collaboration. |
| **Fusion web / browser** | Personal users are steered to **desktop client** for full assembly/joint work; do not rely on browser-only for 24-DOF jointing. |
| **Education** | Only if Dave qualifies separately. |

**Onshape Free** remains a valid equivalent for mates (Revolute) on imported STEP if Fusion commercial seat is delayed — still needs the STEP/meshes.

---

## 3. Practical paths ranked

### Rank 1 — **(A) Official kit CAD + Fusion joints** ← **RECOMMENDED**

**Flow:** Obtain official STEP (support) → import Fusion → As-Built Revolute joints on 24 DOF → Motion Study + reach/clearance screenshots for founder.

| Pros | Cons |
|------|------|
| Kit-true geometry; best “what do I get for a grand” | STEP **order-gated** today; needs Fusion (or Onshape) seat |
| Matches Hardware OD-5 (kit-matched drawings) | ~4–12 h joint labor after files arrive |

**Access Dave must provide:** (1) Hiwonder STEP/URDF package; (2) Autodesk account with a **commercial Fusion seat** (or Onshape account).

### Rank 2 — **(D) Other: official URDF meshes → clickable CAD / Gazebo**

**Flow:** Support (or evaluation) releases `ainex_description` (or post-order extract from robot/VM) → import meshes into Fusion/Onshape and place Revolute joints at URDF origins **or** run Gazebo/RViz from the official sim for interactive joint scrubbing.

| Pros | Cons |
|------|------|
| Official kinematics + meshes; Gazebo path is already documented in wiki | Mesh-based Fusion joints are messier than B-rep STEP; VM is huge; IP notice discourages unlicensed image use |
| Can feed MuJoCo later (URDF→MJCF tools) | Still needs Dave CAD time or local Gazebo |

Treat public `AiNex_VM.zip` as **last resort / policy-sensitive**, not the primary ask.

### Rank 3 — **(B) Onshape free public**

| Pros | Cons |
|------|------|
| Free web CAD; mates = Revolute; easy share link **once built** | **No public AiNex Onshape document exists**; still blocked on STEP/URDF input |
| Good if Fusion commercial seat is the bottleneck | Free plan limits; still not “kit CAD” until files arrive |

Use Onshape as **host** for Path A/D geometry, not as a source of AiNex CAD.

### Rank 4 — **(C) Improve existing MuJoCo MJCF visual (interim)**

**Flow:** Tighten `mujoco/dronable_v0.xml` + `scripts/build_cad.py` capsules toward published envelope / servo outline sizes; better walk/reach preview video.

| Pros | Cons |
|------|------|
| Available **now**, no purchase, no Hiwonder login | **Not kit-matched**; Hardware brief rejects using placeholder geometry for fab/CM; weak for “grand” spend conviction |
| 1–3 h | Does not replace STEP/URDF review |

**Use only as interim visual while waiting on A.**

---

## 4. Exact access Dave must provide

| Priority | Access | Why |
|----------|--------|-----|
| **P0 (blocker)** | **Hiwonder evaluation STEP and/or URDF+meshes without paid order** — email `support@hiwonder.com` (+ sales) stating pre-purchase founder mechanical review for AiNex **Standard 24DOF** | No public STEP; source/image gated by order number; without files, Fusion cannot assemble the real robot |
| **P1** | Autodesk account + **Fusion commercial seat** (desktop) — *or* Onshape Free/Pro account | Personal Use ToS unfit for company review; joints need desktop Fusion or Onshape |
| **P2** | If evaluation denied: **order number after purchase** (only then) for support STEP + source Drive link | Documented official path |
| **P3** | Optional: confirm whether sales will share **dimensioned drawings** / DOF ID map even without STEP | Unblocks MJCF axis map sooner |

**Not required from Dave for research:** this agent box has no Fusion MCP — file drop into `/workspace/dronable-proto/cad/` is enough for downstream MuJoCo rematch.

**Do not expect:** a public Fusion/Onshape “assembly link” today — **none exists**.

---

## 5. Estimated time (after access)

| Milestone | Estimate |
|-----------|----------|
| Hiwonder reply with STEP/URDF | Unknown (days typical for support) |
| Import STEP → componentize → Rigid Groups | 1–2 h |
| 24× Revolute As-Built joints + limits | 2–6 h |
| Motion Study + clearance/reach screenshots for founder | 1–2 h |
| **Total once files in hand** | **~4–12 h** |
| Interim C (capsule polish only) | **1–3 h** (no Hiwonder wait) |

---

## 6. Blockers

1. **No public official STEP** — FAQ requires order number → support.  
2. **Source code / system image policy** — order number; Important Notice on Drive.  
3. **No Onshape/Fusion/SW MCP** on agent box — human CAD seat required.  
4. **Fusion Personal Use** unsuitable for company founder review.  
5. **Local STEP/MJCF are approximate** — must not be sold as kit truth.  
6. **No existing shareable articulated Fusion/Onshape assembly URL** to give the founder today.

---

## 7. Suggested next actions (no spend)

1. Dave emails Hiwonder (`support@hiwonder.com` + sales) requesting **evaluation STEP + URDF/meshes for AiNex Standard** for pre-purchase mechanical review (cite envelope/DOF; offer NDA if they ask).  
2. In parallel, confirm **Fusion commercial** (or Onshape) account ready.  
3. Optional interim: Path **C** capsule/visual polish + founder review of existing `previews/` only, labeled **non-kit**.  
4. When STEP arrives: Path **A** jointing; drop files under `cad/vendor/ainex/` and rematch MJCF.

---

## 8. Source checklist (fetched 27 Sep 2026)

- https://www.hiwonder.com/products/ainex — FAQ STEP/order gating; specs  
- https://wiki.hiwonder.com/projects/AiNex/en/raspberry-pi5-version/ — URDF course & package paths  
- https://wiki.hiwonder.com/projects/AiNex/en/raspberry-pi5-version/docs/Appendix.html — Drive links  
- https://drive.google.com/drive/folders/1kyhah0bdW4d8omzYjot0Kq2f50l5GYcB?usp=sharing — AiNex Resources  
- https://drive.google.com/drive/folders/11BTsaX1hs_hhckoduHm42VXjUTDB72FM — Simulation Model & URDF (PDFs + VM pack link)  
- https://drive.google.com/file/d/1EnIp84ktzm05tmZpSUMbWOJ5vqcyKW78/view — Important Notice (order-gated source/image)  
- Autodesk Fusion Personal Use terms — non-commercial / <$1k revenue  
- Autodesk docs: Joint / As-Built Joint / Motion Study; STEP import workflows  
- Local: `mujoco/dronable_v0.xml`, `cad/dronable_v0_*.step`

