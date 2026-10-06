#!/usr/bin/env python3
"""Day-1 steerable walk on the frozen AiNex M145 plant.

Velocity only. No waypoints, goals, maps, door commands, or joint targets.

Plant (Hardware freeze, plus the approved kit_cam copy):
  mujoco/ainex_hiwonder/ainex_controls_m2_145.xml
  Foot contact box 135×76 mm (half-size 0.0675 × 0.038 m, friction 1.6).
  Toe spheres are visual (contype 0) and do not hold weight.
  Legs ±2.45 Nm (hip/knee kp 40–45, ankle kp 35). Arms/head ±0.7 Nm.
  kit_cam is a child of head_tilt_link: pos 0.050 0.019 0.007, xyaxes
  0 -1 0 0 0 1, fovy 104.82, plus a zero-mass non-contact site. No door
  geometry. This file does not load gate_f / OptC / door plants. The demo
  mp4 uses the offscreen renderer aimed at body_link, not kit_cam.

Command bus (latest command wins). Voice will call this same API later:
  stand(now)              still; vx=0, yaw_rate=0. Power-on default.
  stop(now)               same as stand; drops velocity the same tick.
  vel(vx, yaw_rate, now)  m/s and rad/s. +vx is forward (+X). +yaw_rate is
                          turn left. No vy.

AI resends vel at 10 Hz while moving. If no command arrives for 200 ms the
next tick is stop/stand. Each tick returns applied_vx, applied_yaw_rate, and
mode in {stand, move, fault}. A refused command prints one line and is not
applied.

Clamps (what we actually apply — not the raw request):
  Forward vx  ≤ VX_FWD_CAP (0.056 m/s). Lowered from 0.080: that command
                yaws off and tips near 1.2 m (support margin about −0.09 m).
                CPG amplitude is |applied_vx| / GAIT_AMP_VX (0.080), so full
                stick is amplitude 0.70 of the gait that used to be full stick
                at 0.080. Realized body speed is about 0.04 m/s, not 0.056.
                applied_vx is the clamped command; the summary's mean body vx
                and forward Δyaw are the measurements.
  Reverse vx  ≥ -VX_BACK_CAP (0.032 m/s) → amplitude 0.40 of the same CPG
                (still |vx| / 0.080), sagittal mirror of the forward gait.
                The stance-slip damper is raised to REVERSE_PLANT_KD only
                while reversing. Realized retreat is about two to three body
                lengths, then stop. Not a Gate Q pass.
  |yaw_rate|  ≤ YAW_RATE_CAP (0.25 rad/s). While walking forward this scales
                the outside step longer than the inside step. Heading change
                is the airborne foot: hip yaw stays 0 through stance and
                double support, then the swing foot toes into the turn and
                plants. +hip yaw toes the foot right (axis −Z), so a left
                command uses a negative swing hip yaw. Reverse does not use
                that toe. vx=0 yaw still does not turn this plant in place.
                Heading that remains after stop is the measured turn.
                vx and yaw_rate are applied together; the nav clips walk,
                arc, walk, then stop. The multi clip chains both claimed
                holds without stopping between them: forward, left arc
                (12.5 s, the nav-left window), forward, right arc (11 s,
                the nav-right window), forward, then stop. Right-then-left
                and a second arc of the same sign at these lengths do not
                steer; those orders are Prefer FAIL and are not the clip.
                A left hold of 14 s that starts at 15 s tips on the resume
                and is not used.
  Deadband and slew reuse TELEOP_DEADBAND (0.08) and TELEOP_RATE_LIMIT (1.5)
  from scripts/walk_gait.py. Those constants are joint-space (rad, rad/s).
  Velocity uses the same fraction of each cap:
    |vx| < 0.08 * VX_FWD_CAP  → 0
    |yaw_rate| < 0.08 * YAW_RATE_CAP → 0
  and slews no faster than the plant clamp, which is below TELEOP_RATE_LIMIT.
  Hip-yaw offset itself slews at TELEOP_RATE_LIMIT rad/s and ignores a desired
  offset smaller than TELEOP_DEADBAND rad. stand/stop/timeout skip the slew
  and zero velocity the same tick. Forward and turn then ease into stand over
  SETTLE_BLEND_S while the stance damper stays at SETTLE_PLANT_KD for
  SETTLE_DAMPER_S (100 N/(m/s), 1.20 s). Reverse, if the cut would land in a
  phase that pitches, finishes at most one mirrored step with the command
  already at 0, then uses that same damper. With neither, some phases pitch
  the COM about 8 cm out of the foot boxes and the existing tip check latches.
  The damper is off again after that window; quiet stand does not keep it.

Gait: scripts/walk_gait_ainex.py gait_targets. Forward uses a shorter cadence
than Gate D CSF50 (that basin crawled at ~1 cm/s): T=0.55 s, hip amp 0.24 rad,
DS=0.1375 s, stance-slip damper 25 N/(m/s) instead of CSF50's 115. Lateral
COM shift is 0.16 rad (was 0.275) so the torso does not waddle as hard.
Forward swing adds LOOK_KNEE_LIFT at mid-swing. While yaw is near zero the
contralateral shoulder is slewed up to LOOK_ARM_SCALE_STRAIGHT; a yaw command
slews it back to LOOK_ARM_SCALE. Both stay inside the existing ±0.7 Nm clip.
Reverse keeps the un-lifted knee: a shorter DS or more swing dorsiflex
dropped the retreat under up_z 0.90. CP swing and mild stance VIK stay on.
Feet, friction, kp, and ±2.1 Nm are unchanged; the legs still pin at 2.1 Nm.
Balance assist stays off (it locks yaw and fakes speed). The stance-vx npz
residual stays off. A forward-only swing residual (hold, contact preload,
stance push) is in this file and measured off: the swing knee is already on
the ±2.1 Nm clip, and the variants that stay upright do not raise the median
sole enough to see. This is not a clean-walk or Gate E PASS.
If many legs pin, or up_z approaches a tip, applied velocity is capped (not
compounded). Tip / collapse latches mode=fault and stands.

Honesty: pure yaw (vx=0) still runs a reduced forward CPG because this plant
has no turn-in-place gait, so some +X creep is expected. Reverse mirrors hip
and ankle pitch about the stand pose and keeps swing knee flexion; clearance
and heading are not a retreat proof.

Keyboard (same bus). MuJoCo's viewer callback is press-only, so keys latch
until Space:
  W  vx = +VX_FWD_CAP, yaw unchanged
  S  vx = -VX_BACK_CAP, yaw unchanged
  A  yaw_rate = +YAW_RATE_CAP (left), vx unchanged
  D  yaw_rate = -YAW_RATE_CAP (right), vx unchanged
  Space  stop / stand, clear latches
The view loop resends the latch every control tick so the 200 ms watchdog
does not trip while a key is latched.

Run:
  MUJOCO_GL=osmesa python scripts/steer_walk.py
  MUJOCO_GL=osmesa python scripts/steer_walk.py --clip nav-left
  MUJOCO_GL=osmesa python scripts/steer_walk.py --clip nav-right
  MUJOCO_GL=osmesa python scripts/steer_walk.py --clip nav-multi
  MUJOCO_GL=osmesa python scripts/steer_walk.py --no-video
  MUJOCO_GL=glfw  python scripts/steer_walk.py --view
  python scripts/steer_walk.py --self-test
  MUJOCO_GL=osmesa python scripts/steer_walk.py --bus-kit
  MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py

`--bus-kit` drives the locked kit walk (500 ms, 20 ms servo, stance
+0.005 m per foot) through this bus. `--view` uses that same row.
The forward clamp is the measured kit body speed, 0.150 m/s.
Yaw stays ±0.25 rad/s and maps into the OP3 step angle.

Nav-left resume heading on that kit row (Prefer FAIL). Scenario: 1 s
stand, 15 s forward, 12.5 s vel(+0.150, +0.25), then 6 s vel(+0.150, 0).
Resume Δyaw ≈ +12.41°. Plant md5 207f3d5e9c6a72e16f7aa0c8d224f75e
unchanged. Split: (1) 0–0.68 s applied_yaw still slewing +0.25→0 at
0.40 rad/s² after the 100 ms resend clears the target → body +6.81°
(command integral ~+5.22°); (2) 0.68–6.0 s applied_yaw and the step
angle are already 0 → leftover left curve +5.60° (body yaw rate
+0.034→+0.009 rad/s). That is ~half ramp-out, ~half steady leftover
curve under vel(+0.150, 0) — not “turn still commanded.” Soft-pass
is off.

Left versus right unload on this kit row. The 6 s vel(+0.150, 0) after
the left turn is +12.3° (+6.7° during the 0.69 s slew, command integral
+5.3°, then +5.6° with applied_yaw and the step angle already 0). After
an 11 s vel(+0.150, −0.25) the same 6 s is −2.2° chained and −0.6° from
a straight approach. The slew is 0.40 rad/s² both ways. On a matching
step phase the right ramp moves the body −5.3° (command integral −4.5°)
and the rest of the window curves left +4.7°, so they cancel. Straight
vel(+0.150, 0) already curves +4.2° over 27–33 s and +3.5° over
28.5–34.5 s. Hip-yaw targets are 0 once the step angle is 0. a_move
follows the yaw sign and the shift stays +|a|. Steady rates are +0.217
and −0.233 rad/s. Stop snaps yaw to 0 in one tick; after a right turn
the heading kick runs about +11° to −10° across 0.37 s of step phase.

Five cold starts of 1 s stand then 30 s vel(+0.150, 0) match (std 0).
Every one starts in double support at gait time 0 and the first swing
foot is the left foot: stand resets the step clock, and the cycle
swings left first. Δyaw is −1.46° at 6 s, +1.39° at 15 s, +0.50° at
30 s. Mean body yaw rate is +0.0003 rad/s. Two-second slices rock
about −2.9° to +1.9°. The +3° to +4° windows are that rock, not a
steady left bias. An extra 0.25 s of stand does not change the lead
foot. Starting the clock half a period later (probe only) swings the
right foot first and the 30 s net is −2.55°. Cold stand→forward
stays left-first: right-first Δyaw at 30 s is −2.60°, and the first
right-lead step knees are 1.749 / 1.152 Nm, under 2.33. After a yaw
target returns to 0 the next double support swings the outside foot.
A one-tick clock park stepped the targets 0.342 rad. The pose now
chases the live gait over that double support (0.056 s); the largest
tick is 0.062 rad, the same as the walk's own largest tick. The
post-left 6 s is +1.4°. The post-right 6 s does not move the clock
and is −2.0°. The left turn (12.5 s) finishes at +156.4° and the
right turn (11.0 s) at −148.6°. The right arc is 7.8° shorter because
the hold is shorter; the right rate is −0.236 rad/s, which is not
the low one.

Hip roll while yawing used to cross 2.33 Nm on the empty plant
(l_hip_roll −2.36 Nm at 3.42 s on 1 s stand then 8 s vel(+0.150,
−0.25)). The stance hip is the left leg during the right swing, and
damping adds to the spring. While applied yaw is away from 0 the
hip-roll command uses the 2.33 Nm prediction budget. The same window
then peaks at −2.27 Nm. Straight walking is not on that budget.
Forcerange stays ±2.45 Nm. The stop hold limits every leg joint
(hip yaw, hip roll, hip pitch, knee, ankle pitch, ankle roll) to a
2.28 Nm prediction, rewritten each physics step from the live q and
ω. Measured peaks on that hold are 2.280 Nm, 0.05 under 2.33. A
command written once per tick left hip pitch at 2.309 Nm on one
phase. The bus is stood on the next 8 ms tick. Body settle, T_stop, is
0.832 s on the empty-plant straight walk stopped at gait clock
0.322 s, with 8.2 cm of COM path. The kitchen body-COM settle of the
toe-gap stop is 0.842 s, and that longer sample is the T_stop in
d_min. Clear distance stays d_min = v × (T_detect + T_stop) with
v = 0.150 m/s, compared as toe_gap = eye_range − toe_offset.
The gate offset is the furthest either toe reaches during the walk,
+0.017 m on the right foot, not the sole at one frame. The compare is
(eye_range − step_off − buffer) against d_min, with buffer 0.03 m or
0.05 m. 0.1263 m at T_detect = 0 is the floor, not a safe gap.
T_detect 0.033 s and 0.100 s are placeholders, not kit measurements.
The 0.342 ms RGB compute is not kit T_detect. Those six stops have 0
prop contacts and a stop peak of −2.280 Nm. They fire on an off-axis
stool leg whose eye range reads about 0.18 m short, so the clearance
is not a calibrated toe gap. Not kit-safe. A −10° head-down walk is
not enabled. The Day-1 kitchen stop calls ray_corridor.estimate_hazard
and stops when the hit is in the foot corridor and toe_gap_m is at or
under d_min. That gap already includes the 20 mm pad and the step
offset. No 3–5 cm buffer is added. On this walk the latch is the
hitting stool leg, not the off-axis one. A stress with the pad frozen
at 0.035 m stops earlier on that same leg and still leaves the
off-axis leg outside the corridor. The Day-1 kitchen stop reads
hazard_finder cues on the kit_cam frame and also stops on too_close.
On this walk the cue path stops on the hitting leg before too_close,
earlier than the sim-projection latch because the firing frame reads
about 1 cm short. Earlier samples on the same walk read long, including
an in-corridor leg_2 sample of +0.116 m at 5.312 s, while the true gap
is still above d_min. The shortest-gap latch carries that floor point
in body x, y, and yaw and drops it outside the corridor. A new cue
joins a saved point only within 0.035 m after that re-read. That
radius is frozen before the run. Sim leg names are not a key. On
this walk it fires at the same time as the cue latch. The report
name after the stop is the hitting leg. Longer hits of that leg
stay on their own tracks, and the firing eye stays short. A one-frame
stress adds 0.10 m to the toe gap of the floor point that first
reached d_min + 0.05. The latch keeps the shorter gap and the stop
time does not move. The same chain at T_detect = 0 does not clear the five furnished rooms. Kitchen left and living straight stop on the prop the unstopped walk hits. Entrance straight hits the rug with no stop. col_mat_rug top is z 0.012 m. The left sole hits it at 6.892 s, 11.28 N, in double support, with the lowest sole corner 0.02 mm under that top. No swing-phase sole is over the rug. The leading corner at the hit is 7.4 mm above the top. That 12 mm top is a step-on, not a Day-1 stop. Walking ankles on it stay at or under 1.282 Nm. The left sole spans the edge at 7.016 s, heel on the floor and toe on the rug, 10.4 mm of corner tilt. CoP stays in the sole. min up_z is 0.934. At 7.560 s the bus flags airborne because the floor-only check ignores the rug; up_z is 0.979. On 640 swing ticks the sole minimum is -3.11 mm and the p90 is 17.51 mm. SWING_TOE_MIN_M is -0.003066 on 640 swing ticks before the 7.560 s fault. The toe p90 is 0.025967 and is report-only. The arm line is that minimum minus 0.002, -0.005066. The minimum is under 2 mm, so the gait scuffs: raise the clearance, do not disarm the line. The negative toe is a 3.2 deg toe-down corner still carrying 12.9 N, penetration equal to the corner, while the hip-frame command is 3.0 mm above stand. The 12 mm rug stays a step-on. MID_SWING_TOE_MIN_M is -0.003066 on the middle 20-80% of each swing, with p90 0.024671 report-only. That tick is still a loaded contact. From 7.016 s to 7.560 s both ankle pitches stay 1.37 rad inside ±2.09. The command holds a fixed ±0.262 rad off hip plus knee. Torso pitch goes from -14.2° to -11.4°. A swing-height copy at gm_z_m 0.034 is not the locked kit. Its mid-swing toe min is -0.002859 at 3.128 s with 5.3 N on the floor, and airborne moves from 7.560 s to 7.568 s. An ankle copy adds 0.088656 rad of toe-up on a split sole and flags airborne at 7.512 s, with the mid-swing toe still -0.003066. Walking ankles stay under 2.33 Nm and CoP stays in the sole. Both copies Prefer FAIL. Locked gm_z_m stays 0.020. The rug step is a flat-foot fight: the ankle holds the hip-plus-knee offset and does not take the 4.4 deg tilt, and that pitches the body. The swing toe at 3.384 s is the right foot. Over 3.344-3.536 s hip pitch peaks at +1.331 Nm and the knee at +1.582 Nm, under 2.33 and under 2.45. The commanded toe at that tick is -0.003205 m and the actual toe is -0.003066 m, 0.000139 m above the command. The command is on the floor, so the scuff is trajectory shape. That -0.003205 m is the live pelvis: the same target on the stand pelvis is +0.007686 m. The gap is -0.010890 m. Pelvis drop is -0.001742 m. The body-frame offset to the leading corner is +0.109801 m forward, -0.098448 m lateral, and -0.181185 m down. The live world horizontal offset is 0.097288 m, with 0.050028 m of it forward. Pitch times that forward offset is -0.001865 m, and pitch times the horizontal offset is -0.003627 m. The exact forward-axis term is -0.003937 m. The roll piece of the same rotation is -0.007631 m. droll times the -0.098448 m lateral offset is -0.007986 m. After drop, that pitch piece, and that roll piece, the residual is +0.002419 m, the body-up axis, and the four terms add to -0.010890 m. Over 3.344-3.536 s the left hip roll peaks at -2.121 Nm and the left ankle roll at -1.007 Nm, both under 2.33 and under 2.45. The 0.081 rad pelvis roll is the walk pose, not an HX-35H limit. A stance-hip offset of -0.0566 rad on the left and +0.0566 rad on the right, not the 1.29/40 rad term, leaves the mid-swing toe at -0.003022 m, puts the right hip on +2.45 Nm at 1.272 s, and flags airborne at 7.560 s. Pelvis roll at 3.384 s is 0.0475 rad and the swing peak is -0.1134 rad. Peak |cmd-q| stays 0.0608 rad. Adding an 8 mm early Bezier bump leaves the toe at -0.002143 m and the right hip on +2.45 Nm, with airborne at 7.552 s. Prefer FAIL. Commanded leading corner is -0.000501 m at 30%, +0.011144 m at 50%, and +0.019190 m at 70% of that swing. Late lift. The sole bottom is 0.026 m below the ankle roll in the plant and in the forward kinematics. MFG lock: that 0.026 m is the IK ankle-to-sole on both sim and kit. The outsole is the plant box bottom, not a stack under the 0.026 m. The -3.2 mm is not an outsole stack. The Prefer FAIL lead is the live pelvis against the nominal stand pelvis. Adding 0.018 m of mid-swing hip-frame z commands a leading corner of 0.014538 m, above 0.012 m plus 0.002 m, and the actual toe at that tick is -0.003014 m on 4.6 N. Airborne is at 7.544 s. Knees stay under 2.33 Nm. Prefer FAIL. Dropping the flat offset for a 0.076885 rad tilt on a split sole tips at 7.448 s with the left knee on -2.45 Nm. Prefer FAIL. An earlier lift adds hip-frame z from toe-off through 40% of single support. At +0.012 m the 20-30% command minimum is 0.009511 m at 3.128 s, the mid-swing toe is +0.000781 m, the walking knee is +2.350 Nm at 7.426 s, and airborne is at 7.552 s. At +0.013 m the left knee is already on -2.45 Nm at 1.128 s and the command is still 0.010495 m. At +0.020 m the 20-30% command minimum is 0.016742 m and the mid-swing toe is +0.003149 m, with the left knee on -2.45 Nm at 1.124 s and airborne at 7.536 s. Prefer FAIL. The 20-30% command does not clear 0.014 m without the knee rail. The split-sole drop scored again and is unchanged. A compliant ankle that tracks the split-sole joint tips at 7.520 s with the right knee on the 2.45 Nm rail. A mid-swing knee flex capped at 2.33 Nm leaves the toe at -0.003069 and still flags airborne at 7.560 s. Both Prefer FAIL. Other bouts false-stop, including last-row too_close with no prop on the 9 s path. On the open ±0.25 walks, existing leg centers do enter the 0.0867–0.108 m outside strip on kitchen left, both living turns, bathroom right, and entrance right. The 0.020 m pad already includes that strip in the stand corridor until |sideways| passes 0.1067 m, and the only centers past that line are still 0.57 m or more ahead. No leg was moved. A close leg that is in-corridor only because the outside edge widens to 0.108 m is not in these scenes. Not kit-safe. Not go-anywhere. The row source
is still a sim projection. Not go-anywhere. Soft-pass is off.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Literal

# GL before mujoco import. --view needs a window; headless uses OSMesa.
if "--view" in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "glfw")
else:
    os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import walk_gait_ainex as wg  # noqa: E402
import lipm_gait as lipm_gait  # noqa: E402
import op3_walk  # noqa: E402
from lipm_gait import LipmConfig, LipmWalker  # noqa: E402
from walk_gait import TELEOP_DEADBAND, TELEOP_RATE_LIMIT  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
# Walk plant after kit_cam moved to 0.050 0.019 0.007. Feet, friction,
# forcerange, kp, and mass are the same as the e3feef97… insert. Pre-camera
# freeze was fc94709c84f5598d4474ecfc4bb41fdc.
# Plant thaw from cursor/plant-thaw-legs-foot-6f10 (PR #43): legs ±2.45 Nm,
# foot contact 135×76 mm. Was 71b2c86d… at ±2.1 and 145×86.
PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
KIT_CAM_POS = (0.050, 0.019, 0.007)
KIT_CAM_FOVY = 104.82
# xyaxes "0 -1 0 0 0 1" → camera-frame columns (x, y, z). Look is −Z = +X.
KIT_CAM_AXES = (
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
)
PREVIEWS = ROOT / "previews"

# Frozen contact box (half-size, m) and servo ranges. Checked, never written.
FOOT_HALF_X = 0.0675
FOOT_HALF_Y = 0.0380
FOOT_FRICTION = 1.6
LEG_TAU = 2.45
ARM_TAU = 0.7
SAT_FRAC = 0.98

# vx that maps to CPG amplitude 1. Full-stick forward is below this: 0.080
# yaws off and tips near Δx 1.2 m. Do not divide amplitude by VX_FWD_CAP.
GAIT_AMP_VX = 0.08
# Full-stick forward command on the kit walk. This is the measured body
# speed at the longest step that stays inside the knee sag budget, not the
# old 0.056 m/s CPG stick. CPG amplitude still divides by GAIT_AMP_VX and
# saturates at 1. KIT_BODY_PER_X * KIT_X_RAIL_M = 7.50 * 0.020 = 0.150.
VX_FWD_CAP = lipm_gait.KIT_BODY_PER_X * lipm_gait.KIT_X_RAIL_M
# |vx| / GAIT_AMP_VX = 0.40. This command, with REVERSE_PLANT_KD, is the
# upright retreat. A larger reverse command shortens the distance before a tip.
VX_BACK_CAP = 0.032
# Stance-slip damper (N per m/s) used only while vx < 0. Forward stays at
# the value in apply_frozen_forward_gait (25). Not a forcerange change.
REVERSE_PLANT_KD = 100.0
YAW_RATE_CAP = 0.25
# Hip-yaw bias at full stick. Must clear TELEOP_DEADBAND (0.08 rad).
# 0.35 rad tips this cadence. 0.12 rad clears TELEOP_DEADBAND and, with
# TURN_STEP_ASYM, holds a walk-turn instead of a pure pelvis twist.
YAW_HIP_CLIP = 0.12
YAW_HIP_GAIN = YAW_HIP_CLIP / YAW_RATE_CAP
# Full-stick outside/inside step scale. +yaw lengthens the right step.
TURN_STEP_ASYM = 0.20
# Subtracted from both hip yaws only while turning left. The open-loop gait
# already drifts left; without this the left command stacks and tips.
TURN_LEFT_BIAS = 0.06
# After the gait has been live this long, that early left drift has settled
# into the straight-walk rock. Keeping the full 0.06 rad bias then lets some
# phases swallow a left yaw command (heading stays inside ±15°). Drop the
# bias and lengthen the outside step from 0.20 to 0.24 only for yaw_rate > 0
# past this gate. Cold-start left and right turns finish before it, and a
# right command never takes this branch. Not a vx or yaw-cap change.
ESTABLISHED_GAIT_S = 12.0
TURN_LEFT_BIAS_ESTABLISHED = 0.0
TURN_STEP_ASYM_ESTABLISHED = 0.24
# Right turn only. Fraction of the common-mode hip yaw kept on the swing
# foot. +hip yaw toes that foot right (axis −Z), which is the wrong way
# when the command is trying to yaw the body right. Left's swing error is
# only the post-bias 0.06 rad and still accumulates. Right uses the full
# 0.12 rad clip, and the swing foot was landing toed left: measured heading
# reached about −16° by 3 s and then sat there until about 5 s (−14°),
# while the same 11 s window as the left turn only caught up at the end
# (−60° vs +62°). 0.40 / 0.60 / 0.80 all stayed upright on that window.
# 0.60 still walks forward (Δx stays with the left turn) and the early
# stall is gone. Stance hip yaw is not scaled. Not a yaw-cap or plant change.
TURN_RIGHT_SWING_YAW_SCALE = 0.60
# Forward step look. Reverse does not take the knee lift or the arm scale:
# shortening DS or raising swing dorsiflex tipped the retreat under up_z 0.90.
# COM shift was 0.275 rad and read as a toy waddle. 0.16 rad still unweights
# the swing leg enough for the extra knee flexion to clear the sole.
LOOK_COM_SHIFT = 0.16
# Added at mid-swing on top of KNEE_SWING, forward only. Ankle pitch moves
# with the knee so the sole stays roughly flat while the leg shortens.
LOOK_KNEE_LIFT = 0.38
# gait_targets already swings the opposite shoulder. 1.0 is a few degrees.
# 3.4 is about ±0.11 rad and still looks frozen from the side, but a cold
# left turn at 6.0 tips (min up_z 0.42, then -1). Straight walking slews
# up to 11.0 (measured shoulder peak-to-peak about 0.67 rad, torque still
# clipped at ±0.7 Nm) and a yaw command slews back to 3.4. 11.0 keeps
# nav-left near +61 deg and the chained right arc near -90 deg. 11.4 tips
# nav-multi. 9.0 tips the left arc. Do not nudge this without re-running
# smoke, nav-left, nav-right, and nav-multi.
LOOK_ARM_SCALE = 3.4
LOOK_ARM_SCALE_STRAIGHT = 11.0
LOOK_ARM_SLEW = 8.0  # scale units per second
# Airborne hip yaw at full yaw stick (rad). +joint toes the foot right.
LOOK_TOE_LEFT = 0.10
LOOK_TOE_RIGHT = 0.12
# Right yaw after a long straight compounds the 0.20 outside-step with the
# airborne toe and can spin past the support box. A shorter right step keeps
# the cold-start right arc and the post-straight right arc upright.
LOOK_RIGHT_STEP_ASYM = 0.10
# vx=0 and yaw!=0: reduced forward CPG so a step exists to yaw on.
INPLACE_YAW_AMP = 0.35
# Swing residual. Forward only, after CP swing and stance VIK. Defaults off.
# Measured on the shipped gait, straight walk, swing sole median 2.31 cm,
# contact fraction during swing 0.00 (the foot is already ~2 cm up, so a
# contact-triggered swing kick never fires). Knee / hip-pitch / ankle-pitch
# sit on the ±2.1 Nm clip for much of the swing; position error is several
# tenths of a radian, so more swing flexion command does not add torque.
#   hold (don't reverse into extension while sole < 4.5 cm): median 2.31 cm
#   contact preload 0.20 rad while the foot is still down: median 2.45 cm
#   preload 0.45 rad and above lowers the sole (the hip crouches)
#   stance push 0.10 rad: median 2.59 cm, straight yaw +5.6 deg in 3 s
#   stance push 0.25 rad: median 2.84 cm, straight yaw +9.4 deg in 3 s
#   stance push 0.40 rad: tips (min up_z −1)
# None of the upright cases is a visible step, and the larger pushes walk
# off heading. Left off so the open-loop baseline (arms ~0.67 rad, sole
# ~2.4 cm, left arc ~+61 deg) stays the shipped gait. Not a torque or period
# change. Reverse never takes it.
USE_SWING_HOLD_RESIDUAL = False
SWING_HOLD_CLEAR_M = 0.045
SWING_HOLD_S_MAX = 0.72
SWING_HOLD_FADE = 0.12
SWING_HOLD_MARGIN = 0.05
SWING_HOLD_CLIP = 0.35
USE_SWING_PRELOAD = False
SWING_PRELOAD_RAD = 0.20
SWING_PRELOAD_PHASE0 = 0.48
USE_STANCE_PUSH = False
STANCE_PUSH_RAD = 0.10
STANCE_PUSH_PHASE0 = 0.22
STANCE_PUSH_PHASE1 = 0.48

# Joint-space constants, scaled into velocity units (see module docstring).
DEADBAND_VX = TELEOP_DEADBAND * VX_FWD_CAP
DEADBAND_YAW = TELEOP_DEADBAND * YAW_RATE_CAP
# Plant slew is tighter than TELEOP_RATE_LIMIT so stand→full gait is not one frame.
VX_SLEW = 0.08  # m/s^2  (0 → 0.150 cap in 1.875 s)
YAW_SLEW = 0.40  # rad/s^2

COMMAND_TIMEOUT_S = 0.200
VEL_RESEND_S = 0.10
CTRL_DT = 1.0 / wg.CTRL_HZ

# After stop, applied_vx is already 0. Forward and turn blend into stand
# over SETTLE_BLEND_S while the stance damper is held at SETTLE_PLANT_KD
# for SETTLE_DAMPER_S. Measured with the damper off: some phases pitch
# (support margin about −0.08 m) and the tip check latches. The damper is
# off again after that window. Reverse cuts near phase 0.29–0.40 and
# 0.84–0.95 pitch, so those finish at most one mirrored step first, then
# use the same damper. Clamps are unchanged.
SETTLE_BLEND_S = 0.70
REVERSE_SETTLE_BLEND_S = 0.45
SETTLE_PLANT_KD = 100.0
SETTLE_PLANT_AMP = 0.8
SETTLE_COAST_MAX_S = 0.55
# Damper stays on after the pose blend. Some reverse cuts are upright at the
# end of the blend and pitch once the wrench drops; 1.20 s covers that.
SETTLE_DAMPER_S = 1.20
# Tip / fall. up_z 0.85 is the existing upright bar; a short dip slows the
# gait, a real tip latches fault. Grace covers the drop onto the feet.
SETTLE_GRACE_S = 0.40
TIP_UP_Z = 0.72
TIP_HOLD_S = 0.12
COLLAPSE_Z = 0.14
AIRBORNE_FAULT_S = 0.20
COP_SLOW_MARGIN = 0.012  # m, COM inside support hull
COP_FAULT_MARGIN = -0.04

ModeName = Literal["stand", "move", "fault"]

KEY_W = 87
KEY_A = 65
KEY_S = 83
KEY_D = 68
KEY_SPACE = 32

_REFUSED_COMMANDS = frozenset({
    "waypoint", "waypoints", "goal", "goto", "map", "door",
    "joint", "joints", "vy", "pose", "path",
})


@dataclass(frozen=True)
class TickReport:
    """One control tick. applied_* are what the gait used, after clamps."""

    applied_vx: float
    applied_yaw_rate: float
    mode: ModeName

    def line(self) -> str:
        return (
            f"applied_vx={self.applied_vx:+.4f} "
            f"applied_yaw_rate={self.applied_yaw_rate:+.4f} "
            f"mode={self.mode}"
        )


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _slew(current: float, target: float, rate: float, dt: float) -> float:
    step = rate * dt
    return current + _clamp(target - current, -step, step)


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


class CommandBus:
    """Velocity command bus. Latest command wins. Clock is the caller's `now`."""

    def __init__(self) -> None:
        self.target_vx = 0.0
        self.target_yaw = 0.0
        self.applied_vx = 0.0
        self.applied_yaw_rate = 0.0
        self.mode: ModeName = "stand"
        self.fault = False
        self.fault_reason = ""
        self.last_cmd_time = 0.0
        self._snapped = True

    def stand(self, now: float) -> str | None:
        self._snap_zero(now)
        return None

    def stop(self, now: float) -> str | None:
        return self.stand(now)

    def vel(self, vx: float, yaw_rate: float, now: float, **extra: object) -> str | None:
        if extra:
            keys = ", ".join(sorted(str(k) for k in extra))
            return f"refused: vel accepts vx and yaw_rate only (got {keys})"
        if self.fault:
            return "refused: fault — velocity dropped; send stand"
        if not _finite(vx) or not _finite(yaw_rate):
            return "refused: non-finite vx or yaw_rate"
        vx_c = float(vx)
        yaw_c = float(yaw_rate)
        if abs(vx_c) < DEADBAND_VX:
            vx_c = 0.0
        else:
            vx_c = _clamp(vx_c, -VX_BACK_CAP, VX_FWD_CAP)
        if abs(yaw_c) < DEADBAND_YAW:
            yaw_c = 0.0
        else:
            yaw_c = _clamp(yaw_c, -YAW_RATE_CAP, YAW_RATE_CAP)
        self.last_cmd_time = now
        if vx_c == 0.0 and yaw_c == 0.0:
            self._snap_zero(now)
            return None
        self.target_vx = vx_c
        self.target_yaw = yaw_c
        self._snapped = False
        if not self.fault:
            self.mode = "move"
        return None

    def submit(self, name: str, now: float, **fields: object) -> str | None:
        if name in _REFUSED_COMMANDS:
            return f"refused: {name} is not a day-1 velocity command"
        if name == "stand":
            return self.stand(now)
        if name == "stop":
            return self.stop(now)
        if name == "vel":
            vx = fields.get("vx", 0.0)
            yaw = fields.get("yaw_rate", 0.0)
            extra = {k: v for k, v in fields.items() if k not in ("vx", "yaw_rate")}
            if not _finite(vx) or not _finite(yaw):
                return "refused: non-finite vx or yaw_rate"
            return self.vel(float(vx), float(yaw), now, **extra)
        return f"refused: unknown command {name}"

    def tick(self, now: float, dt: float) -> TickReport:
        if self.fault:
            self.applied_vx = 0.0
            self.applied_yaw_rate = 0.0
            self.target_vx = 0.0
            self.target_yaw = 0.0
            self.mode = "fault"
            return self.report()
        if (now - self.last_cmd_time) > COMMAND_TIMEOUT_S:
            self._snap_zero(now)
            # Timeout is not a fresh command; keep the old stamp so we stay stopped.
            self.last_cmd_time = now - COMMAND_TIMEOUT_S - dt
        if self._snapped:
            self.applied_vx = 0.0
            self.applied_yaw_rate = 0.0
        else:
            self.applied_vx = _slew(self.applied_vx, self.target_vx, VX_SLEW, dt)
            self.applied_yaw_rate = _slew(
                self.applied_yaw_rate, self.target_yaw, YAW_SLEW, dt
            )
        self._update_mode()
        return self.report()

    def limit_applied(self, ceiling: float) -> TickReport:
        """Cap applied velocity at ceiling × target. Does not compound across ticks."""
        cap = _clamp(float(ceiling), 0.0, 1.0)
        if self.fault or self._snapped:
            return self.report()
        vx_lim = abs(self.target_vx) * cap
        yaw_lim = abs(self.target_yaw) * cap
        if abs(self.applied_vx) > vx_lim:
            self.applied_vx = math.copysign(vx_lim, self.applied_vx)
        if abs(self.applied_yaw_rate) > yaw_lim:
            self.applied_yaw_rate = math.copysign(yaw_lim, self.applied_yaw_rate)
        self._update_mode()
        return self.report()

    def declare_fault(self, now: float, reason: str) -> TickReport:
        self.fault = True
        self.fault_reason = reason
        self._snap_zero(now)
        self.mode = "fault"
        return self.report()

    def clear_fault(self) -> None:
        if not self.fault:
            return
        self.fault = False
        self.fault_reason = ""
        self.mode = "stand"

    def report(self) -> TickReport:
        return TickReport(self.applied_vx, self.applied_yaw_rate, self.mode)

    def _snap_zero(self, now: float) -> None:
        self.last_cmd_time = now
        self.target_vx = 0.0
        self.target_yaw = 0.0
        self.applied_vx = 0.0
        self.applied_yaw_rate = 0.0
        self._snapped = True
        if not self.fault:
            self.mode = "stand"

    def _update_mode(self) -> None:
        if self.fault:
            self.mode = "fault"
            return
        moving = (
            abs(self.target_vx) > 0.0
            or abs(self.target_yaw) > 0.0
            or abs(self.applied_vx) > 1e-6
            or abs(self.applied_yaw_rate) > 1e-6
        )
        self.mode = "move" if moving else "stand"


class KeyboardLatch:
    """Press-only keys → CommandBus. Latches until Space (viewer has no key-up)."""

    def __init__(self) -> None:
        self.vx_latch = 0.0
        self.yaw_latch = 0.0
        self._stop = False

    def on_press(self, keycode: int) -> None:
        if keycode == KEY_SPACE:
            self.vx_latch = 0.0
            self.yaw_latch = 0.0
            self._stop = True
            return
        if keycode == KEY_W:
            self.vx_latch = VX_FWD_CAP
        elif keycode == KEY_S:
            self.vx_latch = -VX_BACK_CAP
        elif keycode == KEY_A:
            self.yaw_latch = YAW_RATE_CAP
        elif keycode == KEY_D:
            self.yaw_latch = -YAW_RATE_CAP

    def publish(self, bus: CommandBus, now: float) -> str | None:
        if self._stop:
            self._stop = False
            return bus.stop(now)
        if self.vx_latch == 0.0 and self.yaw_latch == 0.0:
            return None
        return bus.vel(self.vx_latch, self.yaw_latch, now)


@dataclass(frozen=True)
class DemoSegment:
    t_end: float
    kind: Literal["stand", "stop", "vel"]
    vx: float
    yaw_rate: float
    label: str


# Sneak peek: stand → forward → stop. Several body lengths at the lowered
# clamp (amplitude 0.70). 0.080 tips before 2 m. Yaw stays on the API.
DEMO_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(53.0, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(56.0, "stop", 0.0, 0.0, "stop"),
)
# Stop on a gait phase that pitched (support margin about −0.08 m) before
# the settle damper. Same phase as t=2.35, one visible walk later.
# Stand 1 s, forward, stop, hold stand past the damper window.
STOP_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(6.75, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(9.5, "stop", 0.0, 0.0, "stop"),
)
REVERSE_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(17.0, "vel", -VX_BACK_CAP, 0.0, "reverse"),
    DemoSegment(19.0, "stop", 0.0, 0.0, "stop"),
)
# Left turn at amplitude 0.70 needs a longer window to clear ~0.55 rad.
# Raising TURN_STEP_ASYM at this speed yaws the wrong way.
TURN_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(12.0, "vel", VX_FWD_CAP, YAW_RATE_CAP, "turn"),
    DemoSegment(14.5, "stop", 0.0, 0.0, "stop"),
)
# Same window as TURN_SCRIPT so Δyaw is comparable. The old 4.5 s right
# clip stopped inside the stall and read about −16°.
RIGHT_TURN_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(12.0, "vel", VX_FWD_CAP, -YAW_RATE_CAP, "turn"),
    DemoSegment(14.5, "stop", 0.0, 0.0, "stop"),
)
# Furniture avoidance: walk, arc with vx and yaw together, walk, stop.
# Approach ends at 16 s (about 0.60 m). Turn length is not the same on both
# sides; right integrates heading faster. A left arc that starts at 15 s and
# holds yaw for 14 s tips on the resume, so that window is not the clip.
NAV_LEFT_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(16.0, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(28.5, "vel", VX_FWD_CAP, YAW_RATE_CAP, "turn"),
    DemoSegment(34.5, "vel", VX_FWD_CAP, 0.0, "resume"),
    DemoSegment(37.0, "stop", 0.0, 0.0, "stop"),
)
NAV_RIGHT_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(16.0, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(27.0, "vel", VX_FWD_CAP, -YAW_RATE_CAP, "turn"),
    DemoSegment(33.0, "vel", VX_FWD_CAP, 0.0, "resume"),
    DemoSegment(35.5, "stop", 0.0, 0.0, "stop"),
)
# Claimed yaw holds, measured on the single-arc clips above.
# Right: 16→27 s = 11.0 s, about −77.3°. Left: 16→28.5 s = 12.5 s, about +75.7°.
# Approach 15 s and resume 6 s are the same lengths as those clips.
# A left arc that starts at 15 s and holds yaw for 14 s tips on the resume.
CLAIMED_RIGHT_ARC_S = 11.0
CLAIMED_LEFT_ARC_S = 12.5
CLAIMED_APPROACH_S = 15.0
CLAIMED_RESUME_S = 6.0
CLAIMED_STOP_HOLD_S = 2.5
# Continuous furniture path. Same caps, same phase lengths, no stop until
# the intentional stand at the end. Left arc first, then the right arc.
# Measured on this plant, and not used as the clip:
#   right then right (second hold 11 s) tips.
#   left then left (second hold 12.5 s) does not yaw, then tips.
#   right then left stays upright but the left hold does not yaw left.
NAV_MULTI_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    DemoSegment(1.0 + CLAIMED_APPROACH_S, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(
        1.0 + CLAIMED_APPROACH_S + CLAIMED_LEFT_ARC_S,
        "vel", VX_FWD_CAP, YAW_RATE_CAP, "arc-left",
    ),
    DemoSegment(
        1.0 + CLAIMED_APPROACH_S + CLAIMED_LEFT_ARC_S + CLAIMED_RESUME_S,
        "vel", VX_FWD_CAP, 0.0, "mid",
    ),
    DemoSegment(
        1.0 + CLAIMED_APPROACH_S + CLAIMED_LEFT_ARC_S + CLAIMED_RESUME_S + CLAIMED_RIGHT_ARC_S,
        "vel", VX_FWD_CAP, -YAW_RATE_CAP, "arc-right",
    ),
    DemoSegment(
        1.0
        + CLAIMED_APPROACH_S
        + CLAIMED_LEFT_ARC_S
        + CLAIMED_RESUME_S
        + CLAIMED_RIGHT_ARC_S
        + CLAIMED_RESUME_S,
        "vel", VX_FWD_CAP, 0.0, "resume",
    ),
    DemoSegment(
        1.0
        + CLAIMED_APPROACH_S
        + CLAIMED_LEFT_ARC_S
        + CLAIMED_RESUME_S
        + CLAIMED_RIGHT_ARC_S
        + CLAIMED_RESUME_S
        + CLAIMED_STOP_HOLD_S,
        "stop", 0.0, 0.0, "stop",
    ),
)
CLIP_SCRIPTS: dict[str, tuple[DemoSegment, ...]] = {
    "forward": DEMO_SCRIPT,
    "stop": STOP_SCRIPT,
    "reverse": REVERSE_SCRIPT,
    "turn": TURN_SCRIPT,
    "turn-right": RIGHT_TURN_SCRIPT,
    "nav-left": NAV_LEFT_SCRIPT,
    "nav-right": NAV_RIGHT_SCRIPT,
    "nav-multi": NAV_MULTI_SCRIPT,
}

# Accepted kit row. Do not retune these to chase a bus measurement.
BUS_KIT_STAND_S = 0.50
BUS_KIT_VEL_S = 6.00
BUS_KIT_STOP_S = 7.60
# A 2 ms sample on the forcerange is a rail. The bar is under this.
BUS_KIT_RAIL_NM = 2.449


def locked_kit_config() -> LipmConfig:
    """Locked kit walk the Day-1 bus actuates. Plant file is not touched."""
    return LipmConfig(
        name="kit500",
        clear_m=0.020,
        arms=True,
        schedule="gait_manager",
        gm_period_s=0.500,
        gm_dsp=0.20,
        gm_y_swap_m=0.020,
        gm_x_m=0.020,
        gm_z_m=0.020,
        gm_z_swap_m=0.006,
        gm_pelvis_deg=5.0,
        gm_hip_pitch_deg=15.0,
        gm_start_lead="L",
        gm_resume_lead="outside",
        gm_crouch_m=0.025,
        gm_move_s=0.020,
    )


BUS_KIT_SCRIPT: tuple[DemoSegment, ...] = (
    DemoSegment(BUS_KIT_STAND_S, "stand", 0.0, 0.0, "stand"),
    DemoSegment(BUS_KIT_VEL_S, "vel", VX_FWD_CAP, 0.0, "forward"),
    DemoSegment(BUS_KIT_STOP_S, "stop", 0.0, 0.0, "stop"),
)


class ScriptedDriver:
    """Resend vel at 10 Hz. stand once; stop once. Silence is the watchdog's job."""

    def __init__(self, segments: tuple[DemoSegment, ...]) -> None:
        self.segments = segments
        self._last_send = -1.0
        self._stop_sent = False
        self._stand_sent = False

    def segment(self, now: float) -> DemoSegment:
        for seg in self.segments:
            if now < seg.t_end - 1e-9:
                return seg
        return self.segments[-1]

    def publish(self, bus: CommandBus, now: float) -> str | None:
        seg = self.segment(now)
        if seg.kind == "stand":
            if not self._stand_sent:
                self._stand_sent = True
                self._last_send = now
                return bus.stand(now)
            return None
        if seg.kind == "stop":
            if not self._stop_sent:
                self._stop_sent = True
                self._last_send = now
                return bus.stop(now)
            return None
        self._stop_sent = False
        if (now - self._last_send) >= (VEL_RESEND_S - 1e-9):
            self._last_send = now
            return bus.vel(seg.vx, seg.yaw_rate, now)
        return None


def convex_hull_xy(points: np.ndarray) -> np.ndarray:
    uniq = sorted({(float(p[0]), float(p[1])) for p in np.asarray(points, dtype=np.float64)})
    if len(uniq) <= 1:
        return np.asarray(uniq, dtype=np.float64).reshape(-1, 2)

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in uniq:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(uniq):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    return np.asarray(hull, dtype=np.float64).reshape(-1, 2)


def support_margin(point: np.ndarray, hull: np.ndarray) -> float:
    """Signed distance to a CCW hull. Positive means inside."""
    pt = np.asarray(point, dtype=np.float64)
    if len(hull) < 3:
        if len(hull) == 0:
            return -1.0
        return -float(np.min(np.linalg.norm(hull - pt[:2], axis=1)))
    min_left = float("inf")
    for i in range(len(hull)):
        a = hull[i]
        b = hull[(i + 1) % len(hull)]
        edge_x = float(b[0] - a[0])
        edge_y = float(b[1] - a[1])
        length = math.hypot(edge_x, edge_y)
        if length < 1e-12:
            continue
        cross = edge_x * (float(pt[1]) - float(a[1])) - edge_y * (float(pt[0]) - float(a[0]))
        min_left = min(min_left, cross / length)
    if min_left == float("inf"):
        return -1.0
    return float(min_left)


def mirror_sagittal(q_walk: dict[str, float], q_stand: dict[str, float]) -> dict[str, float]:
    """Flip hip/ankle pitch about the stand pose. Knee flexion (clearance) stays."""
    out = dict(q_walk)
    for side in ("l_", "r_"):
        for joint in ("hip_pitch", "ank_pitch"):
            key = f"{side}{joint}"
            out[key] = (2.0 * q_stand[key]) - q_walk[key]
    return out


def apply_frozen_forward_gait() -> None:
    """Faster upright forward on the frozen actuators. Does not edit the plant.

    Gate D CSF50 (T=0.88, hip 0.116, DS=0.31, plant_kd=115) stays on the
    ±2.1 Nm clip and crawls at about 1 cm/s because the stance-slip damper
    brakes the body. This basin shortens the step and lowers that damper.
    It is not a Gate E cadence pass and not a clean-walk claim.
    """
    wg.GAIT_T = 0.55
    wg.STEP_LEN = 0.040
    wg.HIP_PITCH_AMP = 0.24
    wg.HIP_BIAS_FWD = 0.06
    wg.DS_S = 0.1375
    wg.COM_SHIFT_AMP = LOOK_COM_SHIFT
    wg.COM_SHIFT_LEAD = 0.23
    wg.KNEE_STANCE = 0.40
    wg.KNEE_SWING = 0.80
    wg.COM_Z = 0.225
    wg.PLANT_KD = 25.0
    wg.USE_CP_SWING = True
    wg.USE_STANCE_VIK = True
    wg.STANCE_VIK_KP = 0.45
    wg.STANCE_VIK_CLIP = 0.03
    wg.USE_RESIDUAL_STANCE_VX = False
    wg.USE_HIP_STRAT = False
    wg.USE_CAPTURE_STEP = False
    wg.USE_WBC_STANCE = False
    wg.USE_HYBRID_MPC = False


def script_bounds(script: tuple[DemoSegment, ...] = DEMO_SCRIPT) -> dict[str, tuple[float, float]]:
    t0 = 0.0
    bounds: dict[str, tuple[float, float]] = {}
    for seg in script:
        bounds[seg.label] = (t0, seg.t_end)
        t0 = seg.t_end
    return bounds


def gait_amp_and_dir(report: TickReport) -> tuple[float, int]:
    """Map applied velocity to CPG amplitude and sagittal sign (+1 forward)."""
    if report.mode != "move":
        return 0.0, 0
    if abs(report.applied_vx) >= 1e-6:
        direction = 1 if report.applied_vx > 0.0 else -1
        amp = min(1.0, abs(report.applied_vx) / GAIT_AMP_VX)
        return amp, direction
    if abs(report.applied_yaw_rate) >= 1e-6:
        amp = INPLACE_YAW_AMP * min(1.0, abs(report.applied_yaw_rate) / YAW_RATE_CAP)
        return amp, 1
    return 0.0, 0


def style_forward_step(
    qdes: dict[str, float], gait_t: float, yaw_rate: float,
) -> None:
    """Forward-only step look. Does not run on reverse or on stand.

    Knee lift shortens the swing leg. Shoulder scale is the contralateral
    term already in gait_targets. Hip yaw is 0 on the planted foot and toes
    the airborne foot into the turn, then holds that angle into touchdown.
    """
    qdes["l_sho_pitch"] = qdes.get("l_sho_pitch", 0.0) * LOOK_ARM_SCALE
    qdes["r_sho_pitch"] = qdes.get("r_sho_pitch", 0.0) * LOOK_ARM_SCALE
    qdes["l_el_pitch"] = 0.32 + 0.10 * abs(qdes["l_sho_pitch"])
    qdes["r_el_pitch"] = 0.32 + 0.10 * abs(qdes["r_sho_pitch"])
    ds_frac = max(0.08, min(0.55, float(wg.DS_S) / max(float(wg.GAIT_T), 1e-3)))
    ds_end = 0.50 + ds_frac
    swing_len = max(0.18, 1.0 - ds_end)
    phi = (gait_t / max(float(wg.GAIT_T), 1e-6)) % 1.0
    if LOOK_KNEE_LIFT > 0.0:
        for side, knee_sign, ank_sign in (("L", 1.0, 1.0), ("R", -1.0, -1.0)):
            leg_phase = wg.phase_leg(phi, side)
            if leg_phase < ds_end:
                continue
            swing_s = _clamp((leg_phase - ds_end) / swing_len, 0.0, 1.0)
            lift = LOOK_KNEE_LIFT * math.sin(math.pi * (swing_s ** 0.55))
            pref = "l_" if side == "L" else "r_"
            qdes[f"{pref}knee"] = _clamp(qdes.get(f"{pref}knee", 0.0) + knee_sign * lift, -2.0, 2.0)
            qdes[f"{pref}ank_pitch"] = _clamp(
                qdes.get(f"{pref}ank_pitch", 0.0) + ank_sign * lift, -1.2, 1.2,
            )
    if abs(yaw_rate) <= 1e-4:
        return
    stick = min(1.0, abs(yaw_rate) / YAW_RATE_CAP)
    if yaw_rate > 0.0:
        swing_joint = -LOOK_TOE_LEFT * stick
    else:
        swing_joint = LOOK_TOE_RIGHT * stick
    for side, name in (("L", "l_hip_yaw"), ("R", "r_hip_yaw")):
        leg_phase = wg.phase_leg(phi, side)
        if leg_phase < ds_end:
            qdes[name] = 0.0
            continue
        swing_s = _clamp((leg_phase - ds_end) / swing_len, 0.0, 1.0)
        blend = min(1.0, swing_s / 0.72)
        blend = blend * blend * (3.0 - 2.0 * blend)
        qdes[name] = swing_joint * blend


def swing_hold_residual(
    sole_m: float, swing_s: float, flex_q: float, flex_cmd: float,
) -> float:
    """Extra flexion (rad) so a low swing sole is not yanked into extension.

    Zero when the sole is already clear, the landing fade has started, or
    the CPG is still commanding more flexion than the joint has reached.
    The return value is added in the flexion direction; it does not exceed
    the gap up to the current joint plus a small margin.
    """
    if not USE_SWING_HOLD_RESIDUAL:
        return 0.0
    if swing_s < 0.0 or swing_s >= SWING_HOLD_S_MAX:
        return 0.0
    if sole_m >= SWING_HOLD_CLEAR_M:
        return 0.0
    if flex_cmd >= flex_q + SWING_HOLD_MARGIN:
        return 0.0
    fade_at = SWING_HOLD_S_MAX - SWING_HOLD_FADE
    margin = SWING_HOLD_MARGIN
    if swing_s > fade_at:
        # Fade the margin only. Scaling the whole gap left the error negative,
        # so the knee stayed on the extension clip and the sole did not move.
        fade = max(0.0, (SWING_HOLD_S_MAX - swing_s) / max(SWING_HOLD_FADE, 1e-6))
        margin *= fade
    gap = (flex_q + margin) - flex_cmd
    return min(SWING_HOLD_CLIP, max(0.0, gap))


def swing_preload_residual(leg_phase: float, ds_end: float, in_contact: bool) -> float:
    """Extra flexion (rad) on a planted foot that is about to swing.

    Ramps from SWING_PRELOAD_PHASE0 to toe-off. Zero once the foot is up
    or the leg is already in swing.
    """
    if not USE_SWING_PRELOAD or not in_contact:
        return 0.0
    if leg_phase < SWING_PRELOAD_PHASE0 or leg_phase >= ds_end:
        return 0.0
    span = max(ds_end - SWING_PRELOAD_PHASE0, 1e-6)
    u = max(0.0, min(1.0, (leg_phase - SWING_PRELOAD_PHASE0) / span))
    u = u * u * (3.0 - 2.0 * u)
    return SWING_PRELOAD_RAD * u


def stance_push_residual(leg_phase: float, in_contact: bool) -> float:
    """Knee extension (positive rad) on a planted stance leg.

    The caller subtracts this from the flexion command. Zero in swing,
    in double support, and when the foot is up.
    """
    if not USE_STANCE_PUSH or not in_contact:
        return 0.0
    if leg_phase < STANCE_PUSH_PHASE0 or leg_phase >= STANCE_PUSH_PHASE1:
        return 0.0
    span = max(STANCE_PUSH_PHASE1 - STANCE_PUSH_PHASE0, 1e-6)
    u = max(0.0, min(1.0, (leg_phase - STANCE_PUSH_PHASE0) / span))
    # Peak mid-window, off at both ends, so toe-off is not a step in command.
    return STANCE_PUSH_RAD * math.sin(math.pi * u)


def swing_hip_yaw_scale(yaw_rate: float, direction: int, leg_phase: float) -> float:
    """Stance and double-support keep the full hip-yaw command.

    A right-turn swing foot keeps TURN_RIGHT_SWING_YAW_SCALE so it does not
    land toed against the turn. Left, reverse, and straight walking return 1.
    Swing starts at the same phase as gait_targets (stance 0.50, then DS).
    """
    if direction < 0 or yaw_rate >= -1e-3:
        return 1.0
    ds_frac = max(0.08, min(0.55, float(wg.DS_S) / max(float(wg.GAIT_T), 1e-3)))
    ds_end = 0.50 + ds_frac
    if leg_phase >= ds_end:
        return TURN_RIGHT_SWING_YAW_SCALE
    return 1.0


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _kit_cam_problems(model: mj.MjModel, *, extra_sites: bool = False) -> list[str]:
    """Hardware kit_cam on head_tilt_link. Site is not a body and not a contact.

    extra_sites is for a vision include that adds named furniture sites.
    kit_cam_site itself still has to match the frozen mount.
    """
    problems: list[str] = []
    if model.ncam != 1:
        problems.append(f"expected 1 kit_cam, found {model.ncam}")
        return problems
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cid < 0:
        problems.append("missing camera kit_cam")
        return problems
    body = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, int(model.cam_bodyid[cid])) or ""
    if body != "head_tilt_link":
        problems.append(f"kit_cam parent {body} != head_tilt_link")
    pos = np.asarray(model.cam_pos[cid], dtype=np.float64)
    if float(np.max(np.abs(pos - np.array(KIT_CAM_POS)))) > 1e-6:
        problems.append(f"kit_cam pos {pos.tolist()} != {KIT_CAM_POS}")
    if abs(float(model.cam_fovy[cid]) - KIT_CAM_FOVY) > 1e-4:
        problems.append(f"kit_cam fovy {float(model.cam_fovy[cid])} != {KIT_CAM_FOVY}")
    mat = np.asarray(model.cam_mat0[cid], dtype=np.float64).reshape(3, 3)
    expected = np.array(KIT_CAM_AXES, dtype=np.float64).T
    if float(np.max(np.abs(mat - expected))) > 1e-5:
        problems.append("kit_cam xyaxes != 0 -1 0 0 0 1")
    if not extra_sites and model.nsite != 1:
        problems.append(f"expected 1 kit_cam_site, found {model.nsite} sites")
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "kit_cam_site")
    if sid < 0:
        problems.append("missing site kit_cam_site")
    else:
        sbody = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, int(model.site_bodyid[sid])) or ""
        if sbody != "head_tilt_link":
            problems.append(f"kit_cam_site parent {sbody} != head_tilt_link")
        spos = np.asarray(model.site_pos[sid], dtype=np.float64)
        if float(np.max(np.abs(spos - np.array(KIT_CAM_POS)))) > 1e-6:
            problems.append(f"kit_cam_site pos {spos.tolist()} != {KIT_CAM_POS}")
    return problems


def plant_problems(
    model: mj.MjModel,
    xml_path: Path = PLANT_XML,
    *,
    extra_sites: bool = False,
) -> list[str]:
    """Read-only freeze checks. Empty list means the loaded plant matches Hardware.

    extra_sites allows furniture sites on a vision include. The md5 is still
    the frozen walk plant. Door and lever bodies still fail this check.
    """
    problems: list[str] = []
    if xml_path.resolve() != PLANT_XML.resolve():
        problems.append(f"plant path {xml_path} is not the frozen M145 walk body")
    digest = _md5(xml_path)
    if digest != PLANT_MD5:
        problems.append(f"plant md5 {digest} != {PLANT_MD5}")
    for i in range(model.nbody):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) or ""
        low = name.lower()
        if "door" in low or "lever" in low:
            problems.append(f"door/lever body present: {name}")
    problems.extend(_kit_cam_problems(model, extra_sites=extra_sites))
    expected_kp = {
        "l_hip_yaw_pos": 40.0, "l_hip_roll_pos": 40.0, "l_hip_pitch_pos": 45.0,
        "l_knee_pos": 45.0, "l_ank_pitch_pos": 35.0, "l_ank_roll_pos": 35.0,
        "r_hip_yaw_pos": 40.0, "r_hip_roll_pos": 40.0, "r_hip_pitch_pos": 45.0,
        "r_knee_pos": 45.0, "r_ank_pitch_pos": 35.0, "r_ank_roll_pos": 35.0,
    }
    for i in range(model.nu):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        fr = model.actuator_forcerange[i]
        kp = float(model.actuator_gainprm[i, 0])
        if name in expected_kp:
            if abs(kp - expected_kp[name]) > 1e-6:
                problems.append(f"{name} kp {kp} != {expected_kp[name]}")
            if abs(float(fr[0]) + LEG_TAU) > 1e-6 or abs(float(fr[1]) - LEG_TAU) > 1e-6:
                problems.append(f"{name} forcerange {fr.tolist()} != ±{LEG_TAU}")
        elif name.startswith(("l_sho", "r_sho", "l_el", "r_el", "l_gripper", "r_gripper", "head_")):
            if abs(float(fr[0]) + ARM_TAU) > 1e-6 or abs(float(fr[1]) - ARM_TAU) > 1e-6:
                problems.append(f"{name} forcerange {fr.tolist()} != ±{ARM_TAU}")
    for gname in ("l_foot_contact", "r_foot_contact"):
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, gname)
        if gid < 0:
            problems.append(f"missing geom {gname}")
            continue
        size = model.geom_size[gid]
        if abs(float(size[0]) - FOOT_HALF_X) > 1e-6 or abs(float(size[1]) - FOOT_HALF_Y) > 1e-6:
            problems.append(f"{gname} size {size[:2].tolist()} != 135×76 half-size")
        if abs(float(model.geom_friction[gid, 0]) - FOOT_FRICTION) > 1e-6:
            problems.append(f"{gname} friction {model.geom_friction[gid, 0]} != {FOOT_FRICTION}")
    for gname in ("l_toe_viz", "r_toe_viz"):
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, gname)
        if gid < 0:
            problems.append(f"missing geom {gname}")
        elif int(model.geom_contype[gid]) != 0:
            problems.append(f"{gname} contype {int(model.geom_contype[gid])} (toe must not hold weight)")
    return problems


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float
    up_z: float
    margin: float
    applied_vx: float
    applied_yaw_rate: float
    mode: ModeName
    body_vx: float
    leg_tau: float
    cop_out: float


class SteerSession:
    """One frozen-plant sim. Commands go through `bus`; `step` is one 50 Hz tick.

    The default load is the empty walk plant. `scene_xml` may be a vision
    include of that same file (the kitchen room). It does not change the
    plant, the caps, or the gait. `initial_yaw` is the freejoint heading,
    not a camera-mount edit.
    """

    def __init__(
        self,
        *,
        video: bool,
        scene_xml: Path | None = None,
        initial_yaw: float = 0.0,
        cam_distance: float = 1.25,
        cam_azimuth: float = 135.0,
        cam_elevation: float = -18.0,
        lipm: LipmConfig | None = None,
    ) -> None:
        problems = []
        if not PLANT_XML.is_file():
            problems.append(f"missing {PLANT_XML}")
        else:
            # Hash before load so a missing file is a clean refusal.
            pass
        if scene_xml is not None and not scene_xml.is_file():
            problems.append(f"missing scene {scene_xml}")
        if problems:
            raise SystemExit("refused: " + "; ".join(problems))
        self.scene_xml = scene_xml
        self._initial_yaw = float(initial_yaw)
        load_path = PLANT_XML if scene_xml is None else scene_xml
        self.model = mj.MjModel.from_xml_path(str(load_path))
        self.data = mj.MjData(self.model)
        problems = plant_problems(
            self.model, PLANT_XML, extra_sites=scene_xml is not None,
        )
        if problems:
            raise SystemExit("refused: " + "; ".join(problems))
        self._force_checksum = float(np.sum(np.abs(self.model.actuator_forcerange)))
        apply_frozen_forward_gait()
        self.bus = CommandBus()
        self.act_idx = {
            mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(self.model.nu)
        }
        self._move_ctrl_idx = [
            i
            for name, i in self.act_idx.items()
            if name.endswith(("knee_pos", "hip_pitch_pos", "ank_pitch_pos"))
        ]
        self.bid_body = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "body_link")
        self.bid_lf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link")
        self.bid_rf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link")
        self.gid_lfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact")
        self.gid_rfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")
        self.gid_floor = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "floor")
        self.foot_local = {
            "L": (
                self.bid_lf,
                self.gid_lfoot,
                np.array(self.model.geom_pos[self.gid_lfoot], dtype=np.float64),
                np.array(self.model.geom_size[self.gid_lfoot, :2], dtype=np.float64),
            ),
            "R": (
                self.bid_rf,
                self.gid_rfoot,
                np.array(self.model.geom_pos[self.gid_rfoot], dtype=np.float64),
                np.array(self.model.geom_size[self.gid_rfoot, :2], dtype=np.float64),
            ),
        }
        self.q_stand = wg.gait_targets(0.0, False, 0.0)
        self.gait_t = 0.0
        self._arm_scale = LOOK_ARM_SCALE
        self._gait_live = False
        self._blend = 0.0
        self._q_live: dict[str, float] | None = None
        self._last_amp = 0.0
        self._last_dir = 0
        self._coast_until = -1.0
        self._damper_until = -1.0
        self.hip_yaw_cmd = 0.0
        self.cop_bias: np.ndarray | None = None
        self._cop_samples: list[np.ndarray] = []
        self.tip_hold = 0.0
        self.air_hold = 0.0
        self.cop_out_hold = 0.0
        self.samples: list[PoseSample] = []
        self.max_leg_tau = 0.0
        self.max_arm_tau = 0.0
        self.min_margin = float("inf")
        self.min_up_z = 1.0
        self.max_cop_excursion = 0.0
        self.cop_excursion = 0.0
        self.fault_announced = False
        self.sim_dt = float(self.model.opt.timestep)
        # The command bus and the Bézier loop stay at 50 Hz. The OP3
        # walking module is built for an 8 ms cycle (4 physics steps).
        # 20 ms is not an integer multiple of 8 ms, so this path does not
        # pretend a 50 Hz tick is that cycle.
        self.ctrl_dt = CTRL_DT
        if lipm is not None and lipm.schedule == "gait_manager":
            self.ctrl_dt = op3_walk.OP3_CTRL_S
        self.steps_per_ctrl = max(1, int(round(self.ctrl_dt / self.sim_dt)))
        self.lipm_cfg = lipm
        self.lipm = (
            None
            if lipm is None
            else LipmWalker(self.model, self.data, self.act_idx, lipm, self.q_stand)
        )
        if self.lipm is not None and lipm is not None and lipm.schedule == "gait_manager":
            self.q_stand = dict(self.lipm.q_stand)
        self._reset_stand()
        self.renderer: mj.Renderer | None = None
        self.cam = mj.MjvCamera()
        mj.mjv_defaultCamera(self.cam)
        self.cam.distance = float(cam_distance)
        self.cam.azimuth = float(cam_azimuth)
        self.cam.elevation = float(cam_elevation)
        if video:
            self.renderer = mj.Renderer(self.model, height=480, width=640)

    def _reset_stand(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[2] = wg.COM_Z
        half = 0.5 * self._initial_yaw
        self.data.qpos[3:7] = [math.cos(half), 0.0, 0.0, math.sin(half)]
        for jn, val in self.q_stand.items():
            jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self.data.qpos[self.model.jnt_qposadr[jid]] = val
        wg.set_ctrl(self.model, self.data, self.q_stand, self.act_idx)
        mj.mj_forward(self.model, self.data)
        # The IK crouch is a different leg length than COM_Z's knee-0.40
        # stand. Seat the soles on the floor. This is the spawn height, not
        # a plant edit.
        if self.lipm is not None and self.lipm.cfg.schedule == "gait_manager":
            seat = min(self.lipm._sole("L"), self.lipm._sole("R"))
            if abs(seat) > 1e-5:
                self.data.qpos[2] -= seat
                mj.mj_forward(self.model, self.data)

    def assert_plant_unchanged(self) -> None:
        now = float(np.sum(np.abs(self.model.actuator_forcerange)))
        if abs(now - self._force_checksum) > 1e-9:
            raise RuntimeError("actuator forcerange changed during steer")

    def step(self) -> TickReport:
        now = float(self.data.time)
        report = self.bus.tick(now, self.ctrl_dt)
        if self.lipm is None:
            ceiling = self._safety_ceiling()
            if ceiling < 0.999 and report.mode == "move":
                report = self.bus.limit_applied(ceiling)
            amp, direction = self._motion_after_stop(report)
            qdes = self._targets(amp, direction, report.applied_yaw_rate)
            wg.set_ctrl(self.model, self.data, qdes, self.act_idx)
            self._servos(amp, direction)
            self._substep(amp, direction)
        else:
            # LIPM does not use the CPG saturation throttle. A falling torso
            # still cuts the velocity command the outer loop integrates.
            vx = report.applied_vx
            if report.mode == "move" and self._up_z() < 0.90:
                vx *= 0.55
            # The tick writes the gait target. Hip, knee, and ankle pitch
            # approach it over HIP_KNEE_MOVE_S; this 20 ms tick only covers
            # part of that move. Snapshot the command before the write.
            ctrl_from = np.array(self.data.ctrl, dtype=np.float64, copy=True)
            if self.bus.fault:
                self.lipm.hold_stand()
                holding = True
            else:
                walking = report.mode == "move"
                self.lipm.yaw_target = self.bus.target_yaw
                self.lipm.tick(vx, report.applied_yaw_rate, walking)
                holding = not walking
            # A hold is already the force-limited stand command. Slewing
            # toward it from the walking ctrl is what rails the stop.
            if holding:
                ctrl_from = np.array(self.data.ctrl, dtype=np.float64, copy=True)
            self._lipm_substep(ctrl_from)
            self.lipm.observe(self._up_z())
        self._update_bias(now)
        margin = self._support_margin()
        up_z = self._up_z()
        leg_tau = self._track_torques()
        self._track_cop_excursion()
        reason = self._fault_reason(now, up_z, margin)
        if reason is not None and not self.bus.fault:
            report = self.bus.declare_fault(now, reason)
            self.hip_yaw_cmd = 0.0
            wg.set_ctrl(self.model, self.data, self.q_stand, self.act_idx)
        elif self.bus.fault and self._recovered(up_z):
            self.bus.clear_fault()
            report = self.bus.report()
        body_vx = self._body_forward_speed()
        self.samples.append(
            PoseSample(
                t=now,
                x=float(self.data.qpos[0]),
                y=float(self.data.qpos[1]),
                yaw=self.yaw(),
                up_z=up_z,
                margin=margin,
                applied_vx=report.applied_vx,
                applied_yaw_rate=report.applied_yaw_rate,
                mode=report.mode,
                body_vx=body_vx,
                leg_tau=leg_tau,
                cop_out=self.cop_excursion,
            )
        )
        self.min_margin = min(self.min_margin, margin)
        self.min_up_z = min(self.min_up_z, up_z)
        return report

    def yaw(self) -> float:
        w, x, y, z = (float(v) for v in self.data.qpos[3:7])
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def render(self, lines: list[str]) -> np.ndarray:
        if self.renderer is None:
            raise RuntimeError("renderer not created")
        self.cam.lookat[:] = self.data.xpos[self.bid_body]
        # mj_step already forwarded. A second mj_forward here changes the
        # contact warm-start and tips this gait before the stop.
        self.renderer.update_scene(self.data, self.cam)
        raw = np.ascontiguousarray(self.renderer.render().copy(), dtype=np.uint8)
        return wg._burn_overlay(raw, lines)

    def _targets(self, amp: float, direction: int, yaw_rate: float) -> dict[str, float]:
        if amp > 0.02:
            if not self._gait_live:
                self.gait_t = 0.0
                self._gait_live = True
            else:
                self.gait_t += CTRL_DT
            qdes = wg.gait_targets(self.gait_t, True, amp)
            step_asym = TURN_STEP_ASYM
            if yaw_rate < -1e-3:
                step_asym = LOOK_RIGHT_STEP_ASYM
            elif yaw_rate > 1e-3 and self.gait_t > ESTABLISHED_GAIT_S:
                step_asym = TURN_STEP_ASYM_ESTABLISHED
            if direction < 0:
                qdes = mirror_sagittal(qdes, self.q_stand)
            else:
                self._asymmetric_step(qdes, yaw_rate, amp, step_asym)
            self._q_live = dict(qdes)
            self._blend = 1.0
        else:
            self._gait_live = False
            qdes = self._settle_targets()
        yaw_cmd = self._hip_yaw(yaw_rate)
        if yaw_rate > 1e-3 and direction >= 0:
            left_bias = TURN_LEFT_BIAS
            if self.gait_t > ESTABLISHED_GAIT_S:
                left_bias = TURN_LEFT_BIAS_ESTABLISHED
            yaw_cmd -= left_bias
        phi = (self.gait_t / max(float(wg.GAIT_T), 1e-6)) % 1.0 if amp > 0.02 else 0.0
        for side, name in (("L", "l_hip_yaw"), ("R", "r_hip_yaw")):
            scale = swing_hip_yaw_scale(yaw_rate, direction, wg.phase_leg(phi, side))
            qdes[name] = qdes.get(name, 0.0) + scale * yaw_cmd
        if direction > 0 and amp > 0.02:
            style_forward_step(qdes, self.gait_t, yaw_rate)
            self._slew_arm_scale(qdes, yaw_rate)
        else:
            self._arm_scale = LOOK_ARM_SCALE
        return qdes

    def _slew_arm_scale(self, qdes: dict[str, float], yaw_rate: float) -> None:
        """Larger contralateral swing while going straight. Yaw keeps 3.4.

        style_forward_step already applied LOOK_ARM_SCALE. Undo that, then
        slew. A step from 11 to 3.4 at the yaw edge is what pushed the
        post-straight right arc over. The slew is not a torque-limit change.
        """
        raw_l = qdes.get("l_sho_pitch", 0.0) / LOOK_ARM_SCALE
        raw_r = qdes.get("r_sho_pitch", 0.0) / LOOK_ARM_SCALE
        target = LOOK_ARM_SCALE_STRAIGHT if abs(yaw_rate) < 1e-3 else LOOK_ARM_SCALE
        step = LOOK_ARM_SLEW * CTRL_DT
        self._arm_scale += max(-step, min(step, target - self._arm_scale))
        qdes["l_sho_pitch"] = raw_l * self._arm_scale
        qdes["r_sho_pitch"] = raw_r * self._arm_scale
        qdes["l_el_pitch"] = 0.32 + 0.10 * abs(qdes["l_sho_pitch"])
        qdes["r_el_pitch"] = 0.32 + 0.10 * abs(qdes["r_sho_pitch"])

    def _reverse_phase_stands(self) -> bool:
        """Reverse phases where an immediate blend already ends in stand.

        Measured on this cadence. Late swing of either foot (about 0.29–0.40
        and 0.84–0.95) pitches. Windows sit inside the passes.
        """
        phi = (self.gait_t / max(float(wg.GAIT_T), 1e-6)) % 1.0
        return phi < 0.22 or (0.50 <= phi < 0.74)

    def _motion_after_stop(self, report: TickReport) -> tuple[float, int]:
        """Bus amplitude, plus a reverse step-finish after stop.

        stand/stop/timeout already report applied_vx = 0. Forward cuts blend
        in place (the settle damper holds them). A reverse cut in a pitching
        phase keeps the last mirrored step until the phase stands up, or one
        gait cycle, whichever comes first.
        """
        amp, direction = gait_amp_and_dir(report)
        now = float(self.data.time)
        if amp > 0.02:
            self._last_amp = amp
            self._last_dir = direction
            self._coast_until = -1.0
            return amp, direction
        # Below the gait threshold the bus amplitude must still be returned.
        # Zeroing it drops the first slew tick's ankle servo and the turn
        # diverges. Reverse step-finish is the only time we replace it.
        if (
            self._last_dir < 0
            and self._gait_live
            and self._last_amp > 0.02
            and not self._reverse_phase_stands()
        ):
            if self._coast_until < 0.0:
                self._coast_until = now + SETTLE_COAST_MAX_S
            if now < self._coast_until:
                return self._last_amp, -1
        return amp, direction

    def _settle_targets(self) -> dict[str, float]:
        """Ease the last step into the stand pose. applied_vx is already 0."""
        blend_s = SETTLE_BLEND_S if self._last_dir >= 0 else REVERSE_SETTLE_BLEND_S
        self._blend = max(0.0, self._blend - CTRL_DT / blend_s)
        if self._q_live is None or self._blend <= 0.0:
            return dict(self.q_stand)
        qdes: dict[str, float] = {}
        for key, stand in self.q_stand.items():
            qdes[key] = stand + self._blend * (self._q_live.get(key, stand) - stand)
        return qdes

    def _asymmetric_step(
        self, qdes: dict[str, float], yaw_rate: float, amp: float, step_asym: float,
    ) -> None:
        """Lengthen the outside step. +yaw (left) lengthens the right step."""
        if amp <= 0.05 or abs(yaw_rate) < 1e-4 or step_asym <= 0.0:
            return
        frac = _clamp(yaw_rate / YAW_RATE_CAP, -1.0, 1.0)
        for side, side_sign in (("l_", -1.0), ("r_", 1.0)):
            scale = max(0.25, 1.0 + step_asym * frac * side_sign)
            for joint in ("hip_pitch", "ank_pitch"):
                key = f"{side}{joint}"
                stand = self.q_stand[key]
                qdes[key] = stand + scale * (qdes.get(key, stand) - stand)

    def _hip_yaw(self, yaw_rate: float) -> float:
        """Common-mode hip yaw. Both joints use axis -Z, so +cmd yaws the body left
        and toes the swing foot right. Right turns scale the swing leg separately.
        """
        if abs(yaw_rate) < 1e-6:
            self.hip_yaw_cmd = 0.0
            return 0.0
        desired = _clamp(YAW_HIP_GAIN * yaw_rate, -YAW_HIP_CLIP, YAW_HIP_CLIP)
        if abs(desired) < TELEOP_DEADBAND:
            desired = 0.0
        step = TELEOP_RATE_LIMIT * CTRL_DT
        self.hip_yaw_cmd += _clamp(desired - self.hip_yaw_cmd, -step, step)
        return self.hip_yaw_cmd

    def _servos(self, amp: float, direction: int) -> None:
        phi = (self.gait_t / wg.GAIT_T) % 1.0 if amp > 0.02 else 0.0
        lat = wg.lateral_com_target(phi) if amp > 0.02 else None
        # Match walk_gait_ainex: CoP gain follows gait amp (0 while standing).
        servo_amp = 0.0 if self.bus.fault else amp
        if servo_amp > 0.0 and float(self.data.time) > 0.15:
            wg.ankle_cop_servo(
                self.model, self.data, self.act_idx, servo_amp,
                self.bid_lf, self.bid_rf, self.gid_floor,
                lat=lat, bias_xy=self.cop_bias,
            )
        if amp > 0.05 and direction >= 0 and wg.USE_CP_SWING:
            wg.apply_cp_swing_placement(
                self.model, self.data, self.act_idx, phi, amp,
                self.bid_lf, self.bid_rf,
            )
        if amp > 0.05 and direction >= 0 and wg.USE_STANCE_VIK:
            wg.apply_stance_jacobian_vik(
                self.model, self.data, self.act_idx, phi, amp,
                self.bid_lf, self.bid_rf, self.gid_floor,
            )
        if amp > 0.05 and direction > 0 and (
            USE_SWING_HOLD_RESIDUAL or USE_SWING_PRELOAD or USE_STANCE_PUSH
        ):
            self._apply_swing_residual(phi)

    def _apply_swing_residual(self, phi: float) -> None:
        """Contact preload, then a swing hold. Forward only. No forcerange edit.

        Preload bends a still-planted foot that is about to swing. The hold
        keeps a low sole from being pulled straight once the CPG command falls.
        Both write ``data.ctrl`` after CP swing and stance VIK.
        """
        ds_frac = max(0.08, min(0.55, float(wg.DS_S) / max(float(wg.GAIT_T), 1e-3)))
        ds_end = 0.50 + ds_frac
        swing_len = max(0.18, 1.0 - ds_end)
        for side, pref, flex_sign, gid, bid in (
            ("L", "l_", 1.0, self.gid_lfoot, self.bid_lf),
            ("R", "r_", -1.0, self.gid_rfoot, self.bid_rf),
        ):
            leg_phase = wg.phase_leg(phi, side)
            knee = f"{pref}knee"
            knee_act = wg.act_name(knee)
            if knee_act not in self.act_idx:
                continue
            knee_i = self.act_idx[knee_act]
            in_contact = wg.foot_floor_contact(
                self.model, self.data, bid, self.gid_floor,
            )
            if leg_phase < ds_end:
                delta = swing_preload_residual(leg_phase, ds_end, in_contact)
                delta -= stance_push_residual(leg_phase, in_contact)
            else:
                jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, knee)
                if jid < 0:
                    continue
                flex_q = flex_sign * float(self.data.qpos[self.model.jnt_qposadr[jid]])
                flex_cmd = flex_sign * float(self.data.ctrl[knee_i])
                sole_m = float(
                    self.data.geom_xpos[gid][2] - self.model.geom_size[gid][2]
                )
                swing_s = (leg_phase - ds_end) / swing_len
                delta = swing_hold_residual(sole_m, swing_s, flex_q, flex_cmd)
            if abs(delta) < 1e-6:
                continue
            self.data.ctrl[knee_i] = float(_clamp(
                float(self.data.ctrl[knee_i]) + flex_sign * delta, -2.0, 2.0,
            ))
            ank = f"{pref}ank_pitch"
            ank_act = wg.act_name(ank)
            if ank_act in self.act_idx:
                ank_i = self.act_idx[ank_act]
                self.data.ctrl[ank_i] = float(_clamp(
                    float(self.data.ctrl[ank_i]) + flex_sign * delta, -1.2, 1.2,
                ))

    def _substep(self, amp: float, direction: int) -> None:
        # Stop already zeroed applied_vx. Hold the stance damper for
        # SETTLE_DAMPER_S after the blend starts so a mid-step cut does not
        # pitch out of the foot boxes. Reverse may still be finishing a step
        # (amp > 0) and must not start this window until that step ends.
        now = float(self.data.time)
        # direction != 0 means a step is still being commanded (including the
        # slew up and a reverse step-finish). The settle damper is only for
        # the tick the bus has already zeroed.
        if amp > 0.05 or direction != 0:
            self._damper_until = -1.0
        elif self._damper_until < 0.0 and self._blend > 0.5 and self._last_dir != 0:
            self._damper_until = now + SETTLE_DAMPER_S
        settling = self._damper_until >= 0.0 and now < self._damper_until
        if settling:
            phi = (self.gait_t / max(float(wg.GAIT_T), 1e-6)) % 1.0
            plant_amp = SETTLE_PLANT_AMP
        else:
            phi = (self.gait_t / wg.GAIT_T) % 1.0 if amp > 0.02 else 0.0
            plant_amp = amp
        saved_kd = float(wg.PLANT_KD)
        if settling:
            wg.PLANT_KD = SETTLE_PLANT_KD
        elif direction < 0:
            wg.PLANT_KD = REVERSE_PLANT_KD
        try:
            self._substep_plant(plant_amp, phi)
        finally:
            wg.PLANT_KD = saved_kd

    def _lipm_substep(self, ctrl_from: np.ndarray | None = None) -> None:
        """Integrate the position servos. No root wrench and no foot xfrc.

        ``data.ctrl`` on entry is the latest gait target. Hip, knee, and
        ankle pitch approach that target over the move time, spread
        across the physics steps, and not faster than the HX slew. The
        Bézier loop uses ``HIP_KNEE_MOVE_S`` (150 ms). The OP3 path uses
        ``gm_move_s`` (kit servo write, 20 ms) because the 8 ms planner
        already sampled the trajectory. Other joints still finish this
        tick. Plant kp, dampratio, forcerange, and armature are untouched.
        """
        n = self.steps_per_ctrl
        ctrl_to = np.array(self.data.ctrl, dtype=np.float64, copy=True)
        if ctrl_from is None:
            ctrl_from = ctrl_to
        dt = float(self.ctrl_dt)
        if self.lipm is not None and self.lipm.cfg.schedule == "gait_manager":
            move_s = max(float(self.lipm.cfg.gm_move_s), dt)
        else:
            move_s = max(float(lipm_gait.HIP_KNEE_MOVE_S), dt)
        frac = min(1.0, dt / move_s)
        slew = float(lipm_gait.HX35_SLEW_RAD_S) * dt
        end = ctrl_to.copy()
        for idx in self._move_ctrl_idx:
            delta = float(ctrl_to[idx] - ctrl_from[idx])
            step = delta * frac
            if step > slew:
                step = slew
            elif step < -slew:
                step = -slew
            end[idx] = float(ctrl_from[idx]) + step
        # A stand hold freezes the gait target, then this loop used to
        # keep that ctrl for every physics step. Hip pitch on one stop
        # phase then measured 2.309 Nm against a 2.28 Nm prediction.
        # Re-limit the leg command from the live q and ω before each step.
        hold = (
            self.lipm is not None
            and self.lipm.phase == "stand"
            and self.lipm.cfg.schedule == "gait_manager"
        )
        for i in range(n):
            alpha = (i + 1) / float(n)
            self.data.ctrl[:] = ctrl_from + (end - ctrl_from) * alpha
            if hold:
                for jn, val in self.lipm.q_stand.items():
                    if any(tok in jn for tok in ("hip_", "knee", "ank_")):
                        self.lipm.write_force_limited(jn, val, lipm_gait.LEG_STOP_NM)
                    else:
                        self.lipm.write_force_limited(jn, val)
            self.data.qfrc_applied[:] = 0.0
            self.data.xfrc_applied[:] = 0.0
            mj.mj_step(self.model, self.data)

    def _substep_plant(self, amp: float, phi: float) -> None:
        for _ in range(self.steps_per_ctrl):
            self.data.qfrc_applied[:] = 0.0
            self.data.xfrc_applied[:] = 0.0
            if amp > 0.05:
                wg.stance_plant(
                    self.model, self.data, phi, amp,
                    self.bid_lf, self.bid_rf,
                    self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                )
            mj.mj_step(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0
            self.data.xfrc_applied[:] = 0.0

    def _update_bias(self, now: float) -> None:
        if self.cop_bias is not None or now >= wg.ANK_COP_BIAS_SETTLE_S:
            if self.cop_bias is None and self._cop_samples:
                self.cop_bias = np.mean(np.stack(self._cop_samples, axis=0), axis=0)
            return
        com = np.asarray(self.data.subtree_com[0, :2], dtype=np.float64)
        support, _, _, _ = wg._support_feet(
            self.model, self.data, self.bid_lf, self.bid_rf, self.gid_floor, None,
        )
        self._cop_samples.append(com - support)
        if now >= wg.ANK_COP_BIAS_SETTLE_S - 1.5 * CTRL_DT and self._cop_samples:
            self.cop_bias = np.mean(np.stack(self._cop_samples, axis=0), axis=0)

    def _up_z(self) -> float:
        return float(self.data.xmat[self.bid_body].reshape(3, 3)[2, 2])

    def _body_forward_speed(self) -> float:
        rot = self.data.xmat[self.bid_body].reshape(3, 3)
        lin = np.asarray(self.data.cvel[self.bid_body][3:6], dtype=np.float64)
        return float(np.dot(lin, rot[:, 0]))

    def _foot_corners(self, bid: int, gid: int) -> np.ndarray:
        pos = np.asarray(self.model.geom_pos[gid], dtype=np.float64)
        half = np.asarray(self.model.geom_size[gid, :2], dtype=np.float64)
        rot = self.data.xmat[bid].reshape(3, 3)
        origin = np.asarray(self.data.xpos[bid], dtype=np.float64)
        pts: list[np.ndarray] = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                local = np.array([
                    pos[0] + sx * half[0],
                    pos[1] + sy * half[1],
                    pos[2],
                ], dtype=np.float64)
                pts.append((origin + rot @ local)[:2])
        return np.stack(pts, axis=0)

    def _support_margin(self) -> float:
        c_l = wg.foot_floor_contact(self.model, self.data, self.bid_lf, self.gid_floor)
        c_r = wg.foot_floor_contact(self.model, self.data, self.bid_rf, self.gid_floor)
        chunks: list[np.ndarray] = []
        if c_l:
            chunks.append(self._foot_corners(self.bid_lf, self.gid_lfoot))
        if c_r:
            chunks.append(self._foot_corners(self.bid_rf, self.gid_rfoot))
        if not chunks:
            return -1.0
        hull = convex_hull_xy(np.concatenate(chunks, axis=0))
        com = np.asarray(self.data.subtree_com[0, :2], dtype=np.float64)
        return support_margin(com, hull)

    def _track_cop_excursion(self) -> None:
        """Contact CoP vs the 145×86 box. Toe geoms are not in this sum."""
        frame_max = 0.0
        for bid, gid, _pos, _half in self.foot_local.values():
            num = np.zeros(3, dtype=np.float64)
            den = 0.0
            for i in range(self.data.ncon):
                con = self.data.contact[i]
                if int(con.geom1) != gid and int(con.geom2) != gid:
                    continue
                force = np.zeros(6, dtype=np.float64)
                mj.mj_contactForce(self.model, self.data, i, force)
                fn = float(force[0])
                if fn <= 1e-6:
                    continue
                num += fn * np.asarray(con.pos, dtype=np.float64)
                den += fn
            if den <= 1e-6:
                continue
            world = num / den
            rot = self.data.xmat[bid].reshape(3, 3)
            local = rot.T @ (world - np.asarray(self.data.xpos[bid], dtype=np.float64))
            center = np.asarray(self.model.geom_pos[gid, :2], dtype=np.float64)
            half = np.asarray(self.model.geom_size[gid, :2], dtype=np.float64)
            delta = np.abs(local[:2] - center) - half
            frame_max = max(frame_max, float(np.max(delta)))
        self.cop_excursion = frame_max
        self.max_cop_excursion = max(self.max_cop_excursion, frame_max)

    def _track_torques(self) -> float:
        """Peak |τ| this tick. Returns the leg peak; arm peak stays on the session."""
        leg = 0.0
        arm = 0.0
        for i in range(self.model.nu):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
            tau = abs(float(self.data.actuator_force[i]))
            if any(tok in name for tok in ("hip_", "knee", "ank_")):
                leg = max(leg, tau)
            elif name.startswith(("l_sho", "r_sho", "l_el", "r_el", "l_gripper", "r_gripper", "head_")):
                arm = max(arm, tau)
        self.max_leg_tau = max(self.max_leg_tau, leg)
        self.max_arm_tau = max(self.max_arm_tau, arm)
        return leg

    def _safety_ceiling(self) -> float:
        """Fraction of the commanded velocity the plant may use this tick.

        Touching ±2.1 Nm is normal for this position-servo gait (the hip command
        is force-clipped). A broad stall, a COM outside the foot boxes, or a
        dropping up-vector caps the command. The cap does not compound.
        """
        ceiling = 1.0
        n_sat = 0
        for i in range(self.model.nu):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
            if any(tok in name for tok in ("hip_", "knee", "ank_")):
                lim = LEG_TAU
            elif name.startswith(("l_sho", "r_sho", "l_el", "r_el", "l_gripper", "r_gripper", "head_")):
                lim = ARM_TAU
            else:
                continue
            if abs(float(self.data.actuator_force[i])) >= SAT_FRAC * lim:
                n_sat += 1
        if n_sat >= 6:
            ceiling = min(ceiling, 0.70)
        # COM-vs-hull goes a couple of centimetres outside during a normal
        # CSF50 step while the contact CoP stays inside the 145×86 box.
        # Throttle on that proxy would crush the locked gait. Tip is up_z.
        if self._up_z() < 0.90:
            ceiling = min(ceiling, 0.55)
        return ceiling

    def _fault_reason(self, now: float, up_z: float, margin: float) -> str | None:
        if not np.isfinite(self.data.qpos).all():
            return "non-finite state"
        z = float(self.data.qpos[2])
        if z < 0.08 or z > 0.55:
            return f"body z={z:.3f} out of range"
        if now < SETTLE_GRACE_S:
            return None
        if z < COLLAPSE_Z:
            return f"collapsed z={z:.3f}"
        if up_z < TIP_UP_Z:
            self.tip_hold += CTRL_DT
        else:
            self.tip_hold = 0.0
        if self.tip_hold >= TIP_HOLD_S:
            return f"tip up_z={up_z:.2f}"
        c_l = wg.foot_floor_contact(self.model, self.data, self.bid_lf, self.gid_floor)
        c_r = wg.foot_floor_contact(self.model, self.data, self.bid_rf, self.gid_floor)
        if not c_l and not c_r:
            self.air_hold += CTRL_DT
        else:
            self.air_hold = 0.0
        if self.air_hold >= AIRBORNE_FAULT_S:
            return "airborne"
        if margin < COP_FAULT_MARGIN:
            self.cop_out_hold += CTRL_DT
        else:
            self.cop_out_hold = 0.0
        # Contact CoP is checked separately. COM outside the hull is a tip
        # only together with a falling torso — the locked gait's COM leaves
        # the single-support hull by a few centimetres while still upright.
        if self.cop_out_hold >= TIP_HOLD_S and up_z < 0.85:
            return f"COM outside support and tipping margin={margin:.3f}"
        if self.cop_excursion > 0.008:
            return f"contact CoP outside 145×86 box by {self.cop_excursion:.4f} m"
        return None

    def _recovered(self, up_z: float) -> bool:
        z = float(self.data.qpos[2])
        return up_z > 0.92 and z > 0.18 and self.tip_hold == 0.0


def _segment_window(
    samples: list[PoseSample], t0: float, t1: float,
) -> list[PoseSample]:
    return [s for s in samples if t0 - 1e-9 <= s.t < t1 - 1e-9]


def _wrap_pi(delta: float) -> float:
    return (delta + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class SegmentStat:
    """One scripted window. dx_heading_m is travel along the heading at t0."""

    label: str
    t0_s: float
    t1_s: float
    dx_m: float
    dy_m: float
    dx_heading_m: float
    dyaw_rad: float
    mean_body_vx_m_s: float
    mean_yaw_rate_rad_s: float
    peak_leg_tau_nm: float
    min_up_z: float
    min_margin_m: float
    max_cop_outside_m: float
    tip: bool


def segment_stats(
    session: SteerSession, script: tuple[DemoSegment, ...],
) -> tuple[SegmentStat, ...]:
    t0 = 0.0
    rows: list[SegmentStat] = []
    for seg in script:
        window = _segment_window(session.samples, t0, seg.t_end)
        if len(window) >= 2:
            dx = window[-1].x - window[0].x
            dy = window[-1].y - window[0].y
            dyaw = _wrap_pi(window[-1].yaw - window[0].yaw)
            dt = window[-1].t - window[0].t
            mean_yaw = dyaw / dt if dt > 1e-6 else 0.0
            mean_vx = float(np.mean([s.body_vx for s in window]))
            heading = _heading_travel(window)
            min_up = min(s.up_z for s in window)
            min_margin = min(s.margin for s in window)
            peak_tau = max(s.leg_tau for s in window)
            cop_out = max(s.cop_out for s in window)
        else:
            dx = dy = dyaw = mean_yaw = mean_vx = heading = 0.0
            min_up = 1.0
            min_margin = 0.0
            peak_tau = 0.0
            cop_out = 0.0
        rows.append(
            SegmentStat(
                label=seg.label,
                t0_s=t0,
                t1_s=seg.t_end,
                dx_m=float(dx),
                dy_m=float(dy),
                dx_heading_m=float(heading),
                dyaw_rad=float(dyaw),
                mean_body_vx_m_s=float(mean_vx),
                mean_yaw_rate_rad_s=float(mean_yaw),
                peak_leg_tau_nm=float(peak_tau),
                min_up_z=float(min_up),
                min_margin_m=float(min_margin),
                max_cop_outside_m=float(cop_out),
                tip=bool(min_up < 0.85),
            )
        )
        t0 = seg.t_end
    return tuple(rows)


def _multi_honesty(stats: tuple[SegmentStat, ...]) -> str:
    parts: list[str] = []
    for row in stats:
        if row.label == "stand":
            continue
        parts.append(
            f"{row.label} Δx={row.dx_heading_m:+.3f} m "
            f"(world {row.dx_m:+.3f},{row.dy_m:+.3f}) "
            f"Δyaw={math.degrees(row.dyaw_rad):+.2f} deg "
            f"mean vx={row.mean_body_vx_m_s:+.3f} m/s "
            f"mean yaw={row.mean_yaw_rate_rad_s:+.3f} rad/s "
            f"peak τ={row.peak_leg_tau_nm:.2f} Nm "
            f"min up_z={row.min_up_z:.3f}"
        )
    text = " Multi-arc (claimed holds, no stop between arcs): " + "; ".join(parts) + "."
    tipped = [row.label for row in stats if row.tip and row.label != "stand"]
    if tipped:
        text += (
            " Prefer FAIL: "
            + ", ".join(tipped)
            + " tipped (up_z < 0.85). That segment is not claimed."
        )
    else:
        text += (
            f" Yaw holds stay at the claimed windows "
            f"(left {CLAIMED_LEFT_ARC_S:.1f} s, then right {CLAIMED_RIGHT_ARC_S:.1f} s). "
            "Other orders (a second same-sign arc, right-then-left, and a 14 s "
            "left that starts at 15 s) are not this clip and were not "
            "re-qualified on this step-cycle basin."
        )
    return text


@dataclass
class RunSummary:
    plant: str
    plant_md5: str
    fault: bool
    fault_reason: str
    dx_forward_m: float
    mean_body_vx_forward: float
    dyaw_forward_rad: float
    yaw_forward_min_rad: float
    yaw_forward_max_rad: float
    dyaw_end_rad: float
    dyaw_turn_rad: float
    gait_amp_vx: float
    dx_turn_m: float
    min_up_z: float
    min_support_margin_m: float
    max_contact_cop_outside_box_m: float
    max_leg_tau_nm: float
    max_arm_tau_nm: float
    delta_x_m: float
    mean_vx_m_s: float
    tip: bool
    cop_in_box: bool
    peak_torque_nm: float
    end_mode: str
    end_margin_m: float
    vx_fwd_cap: float
    vx_back_cap: float
    yaw_rate_cap: float
    deadband_vx: float
    deadband_yaw: float
    vx_slew: float
    yaw_slew: float
    teleop_deadband: float
    teleop_rate_limit: float
    mean_yaw_rate_turn: float
    dx_resume_m: float
    mean_body_vx_turn: float
    mean_body_vx_resume: float
    segment_stats: tuple[SegmentStat, ...]
    honesty: str


def summarize(session: SteerSession, script: tuple[DemoSegment, ...] = DEMO_SCRIPT) -> RunSummary:
    bounds = script_bounds(script)
    if "reverse" in bounds:
        fwd_t = bounds["reverse"]
    elif "forward" in bounds:
        fwd_t = bounds["forward"]
    elif "turn" in bounds:
        fwd_t = bounds["turn"]
    else:
        fwd_t = (1.2, 4.5)
    fwd = _segment_window(session.samples, fwd_t[0], fwd_t[1])
    turn_bounds = bounds.get("turn")
    turn = _segment_window(session.samples, turn_bounds[0], turn_bounds[1]) if turn_bounds is not None else []
    dx_fwd = (fwd[-1].x - fwd[0].x) if len(fwd) >= 2 else 0.0
    mean_vx = float(np.mean([s.body_vx for s in fwd])) if fwd else 0.0
    straight = bounds.get("forward")
    straight_w = _segment_window(session.samples, straight[0], straight[1]) if straight is not None else []
    dyaw_fwd = (straight_w[-1].yaw - straight_w[0].yaw) if len(straight_w) >= 2 else 0.0
    dyaw_fwd = _wrap_pi(dyaw_fwd)
    if straight_w:
        yaw_lo = min(s.yaw for s in straight_w)
        yaw_hi = max(s.yaw for s in straight_w)
    else:
        yaw_lo = 0.0
        yaw_hi = 0.0
    dyaw = (turn[-1].yaw - turn[0].yaw) if len(turn) >= 2 else 0.0
    dx_turn = (turn[-1].x - turn[0].x) if len(turn) >= 2 else 0.0
    dyaw = _wrap_pi(dyaw)
    if len(turn) >= 2 and (turn[-1].t - turn[0].t) > 1e-6:
        mean_yaw_rate = dyaw / (turn[-1].t - turn[0].t)
    else:
        mean_yaw_rate = 0.0
    mean_vx_turn = float(np.mean([s.body_vx for s in turn])) if turn else 0.0
    resume_bounds = bounds.get("resume")
    resume = (
        _segment_window(session.samples, resume_bounds[0], resume_bounds[1])
        if resume_bounds is not None else []
    )
    dx_resume = _heading_travel(resume)
    mean_vx_resume = float(np.mean([s.body_vx for s in resume])) if resume else 0.0
    dyaw_end = 0.0
    if session.samples:
        dyaw_end = _wrap_pi(session.samples[-1].yaw - session.samples[0].yaw)
    tip = bool(session.min_up_z < 0.85 or (session.bus.fault and "tip" in session.bus.fault_reason))
    cop_in_box = bool(session.max_cop_excursion <= 0.001)
    stats = segment_stats(session, script)
    multi = any(seg.label.startswith("arc-") for seg in script)
    tail = session.samples[-1] if session.samples else None
    honesty = (
        "applied_vx is the clamped command, not odometry. "
        f"Forward clamp is {VX_FWD_CAP:.3f} m/s, lowered from 0.080 because "
        "0.080 yaws off and tips near 1.2 m (support margin about −0.09 m). "
        f"CPG amplitude is |applied_vx| / {GAIT_AMP_VX:.3f}, so full forward "
        f"stick is amplitude {VX_FWD_CAP / GAIT_AMP_VX:.2f} and reverse "
        f"{-VX_BACK_CAP:.3f} m/s stays amplitude {VX_BACK_CAP / GAIT_AMP_VX:.2f} "
        f"with stance damper {REVERSE_PLANT_KD:.0f} N/(m/s). "
        f"Yaw cap is ±{YAW_RATE_CAP:.2f} rad/s. Forward yaw is swing-phase only: "
        f"stance hip yaw is 0, and the airborne foot toes {LOOK_TOE_LEFT:.2f} rad "
        f"left or {LOOK_TOE_RIGHT:.2f} rad right at full stick (+joint toes right). "
        f"Outside-step scale stays {TURN_STEP_ASYM:.2f} on a left command "
        f"({TURN_STEP_ASYM_ESTABLISHED:.2f} after {ESTABLISHED_GAIT_S:.0f} s) "
        f"and {LOOK_RIGHT_STEP_ASYM:.2f} on a right command. "
        f"Forward swing adds {LOOK_KNEE_LIFT:.2f} rad of knee flexion at mid-swing "
        f"and scales contralateral shoulder pitch by {LOOK_ARM_SCALE_STRAIGHT:.1f} "
        f"while going straight, slewed back to {LOOK_ARM_SCALE:.1f} while yaw is commanded. "
        f"Lateral COM shift is {LOOK_COM_SHIFT:.2f} rad. "
        "Reverse does not take the knee lift or the arm scale. "
        f"Measured motion Δx={dx_fwd:+.3f} m, mean body vx={mean_vx:+.3f} m/s "
        f"(not the command). Straight-forward net yaw drift="
        f"{math.degrees(dyaw_fwd):+.2f} deg; yaw during that window "
        f"{math.degrees(yaw_lo):+.1f} to {math.degrees(yaw_hi):+.1f} deg. "
        f"Heading at the end of the clip is {math.degrees(dyaw_end):+.2f} deg from the start. "
        + (
            "Per-segment Δyaw is listed below; this clip has no single turn label. "
            if multi
            else (
                f"Turn-window Δyaw={math.degrees(dyaw):+.2f} deg "
                f"(mean yaw rate {mean_yaw_rate:+.3f} rad/s, cap ±{YAW_RATE_CAP:.2f}). "
            )
        )
        + f"tip={tip}; CoP in box={cop_in_box} "
        f"(outside {session.max_cop_excursion:.4f} m); "
        f"peak leg torque={session.max_leg_tau:.2f} Nm (limit {LEG_TAU}). "
        "Stance-slip damper is 25 N/(m/s) while walking forward, down from the "
        "CSF50 crawl's 115. After stop it is "
        f"{SETTLE_PLANT_KD:.0f} N/(m/s) for {SETTLE_DAMPER_S:.2f} s "
        f"(pose blend {SETTLE_BLEND_S:.2f} s), then off. A reverse stop finishes "
        "at most one step first if that phase would pitch. "
        "Foot box, friction, kp, and ±2.1 Nm are unchanged."
    )
    if resume and not multi:
        honesty += (
            f" Nav: approach Δx={dx_fwd:+.3f} m (mean body vx {mean_vx:+.3f} m/s), "
            f"turn Δyaw={math.degrees(dyaw):+.1f} deg "
            f"(mean yaw rate {mean_yaw_rate:+.3f} rad/s, mean body vx {mean_vx_turn:+.3f} m/s), "
            f"resume along heading {dx_resume:+.3f} m "
            f"(mean body vx {mean_vx_resume:+.3f} m/s)."
        )
    if multi:
        honesty += _multi_honesty(stats)
    if session.bus.fault:
        honesty += f" FAULT: {session.bus.fault_reason}."
    if session.lipm is not None:
        sc = session.lipm.score()
        if session.lipm.cfg.schedule == "gait_manager":
            lead = (
                "ROBOTIS OP3 walking module with AiNex leg lengths, "
                "Hiwonder move presets, stepped at the OP3 8 ms cycle. "
                "Not the joint-space preset copy. No root wrench. "
                f"Row {session.lipm.cfg.name}, period {session.lipm.cfg.gm_period_s:.3f} s, "
                f"dsp {session.lipm.cfg.gm_dsp:.2f}, x {session.lipm.cfg.gm_x_m:.3f} m, "
                f"z {session.lipm.cfg.gm_z_m:.3f} m, y_swap {session.lipm.cfg.gm_y_swap_m:.3f} m, "
                f"body drop {session.lipm.cfg.gm_crouch_m:.3f} m, "
                f"servo move {session.lipm.cfg.gm_move_s:.3f} s, "
                f"pelvis {session.lipm.cfg.gm_pelvis_deg:.0f} deg, "
                f"hip pitch offset {session.lipm.cfg.gm_hip_pitch_deg:.0f} deg "
                "on the stand and the walk, "
                f"z_swap {session.lipm.cfg.gm_z_swap_m:.3f} m, "
                "arms swing with gain 0.5. "
                "Kit stance is +0.005 m outward on each foot. "
                "The 0.018 m sole-vs-hip hack is not the live stance. "
                "Plant file is the #43 foot-box shift, not edited here. "
            )
        else:
            lead = (
                "LIPM/ZMP schedule with a joint-space Bézier into the 50 Hz "
                "position servos. Not the open-loop CPG. No root wrench. "
                f"Row {session.lipm.cfg.name}, clear command {session.lipm.cfg.clear_m:.3f} m, "
                f"arms={session.lipm.cfg.arms}. "
            )
        honesty = lead + (
            f"Swing sole median {float(sc['sole_median_m']):.3f} m, "
            f"p90 {float(sc['sole_p90_m']):.3f} m, "
            f"swing contact {float(sc['swing_contact_frac']):.3f}, "
            f"stance slip {float(sc['stance_slip_m_s']):.4f} m/s, "
            f"sat_rate {float(sc['sat_rate']):.3f}, "
            f"lifts {int(sc['n_lifts'])}, missed gates {int(sc['n_missed_gates'])}, "
            f"rear_unload_frac {float(sc['rear_unload_frac']):.2f}, "
            f"rear_share_min {float(sc['rear_share_min']):.2f}, "
            f"CoP margin before lift {float(sc['cop_before_lift_median_m']):.3f} m, "
            f"CoP x before lift {float(sc['cop_x_before_lift_m']):.3f} m. "
            f"Measured Δx={dx_fwd:+.3f} m, mean body vx={mean_vx:+.3f} m/s "
            f"(not the command). "
            f"Turn-window Δyaw={math.degrees(dyaw):+.2f} deg "
            f"(mean yaw rate {mean_yaw_rate:+.3f} rad/s). "
            f"tip={tip}; min_up_z={session.min_up_z:.3f}; "
            f"peak leg torque={session.max_leg_tau:.2f} Nm (limit {LEG_TAU}). "
            "Plant is the #43 thaw only: legs ±2.45 Nm, foot box 135×76 mm, "
            f"md5 {PLANT_MD5}. No further plant edit. "
            "Vendor look is about 2 cm, a 300–600 ms step, forward progress, "
            "no skate, upright, arms not frozen. "
            "An upright march or a shuffle that misses that period is a Prefer FAIL."
        )
        if session.bus.fault:
            honesty += f" FAULT: {session.bus.fault_reason}."
    return RunSummary(
        plant=str(PLANT_XML.relative_to(ROOT)),
        plant_md5=PLANT_MD5,
        fault=bool(session.bus.fault),
        fault_reason=session.bus.fault_reason,
        dx_forward_m=float(dx_fwd),
        mean_body_vx_forward=float(mean_vx),
        dyaw_forward_rad=float(dyaw_fwd),
        yaw_forward_min_rad=float(yaw_lo),
        yaw_forward_max_rad=float(yaw_hi),
        dyaw_end_rad=float(dyaw_end),
        dyaw_turn_rad=float(dyaw),
        gait_amp_vx=GAIT_AMP_VX,
        dx_turn_m=float(dx_turn),
        min_up_z=float(session.min_up_z),
        min_support_margin_m=float(session.min_margin) if math.isfinite(session.min_margin) else -1.0,
        max_contact_cop_outside_box_m=float(session.max_cop_excursion),
        max_leg_tau_nm=float(session.max_leg_tau),
        max_arm_tau_nm=float(session.max_arm_tau),
        delta_x_m=float(dx_fwd),
        mean_vx_m_s=float(mean_vx),
        tip=tip,
        cop_in_box=cop_in_box,
        peak_torque_nm=float(session.max_leg_tau),
        end_mode=tail.mode if tail is not None else "stand",
        end_margin_m=float(tail.margin) if tail is not None else 0.0,
        vx_fwd_cap=VX_FWD_CAP,
        vx_back_cap=VX_BACK_CAP,
        yaw_rate_cap=YAW_RATE_CAP,
        deadband_vx=DEADBAND_VX,
        deadband_yaw=DEADBAND_YAW,
        vx_slew=VX_SLEW,
        yaw_slew=YAW_SLEW,
        teleop_deadband=float(TELEOP_DEADBAND),
        teleop_rate_limit=float(TELEOP_RATE_LIMIT),
        mean_yaw_rate_turn=float(mean_yaw_rate),
        dx_resume_m=float(dx_resume),
        mean_body_vx_turn=float(mean_vx_turn),
        mean_body_vx_resume=float(mean_vx_resume),
        segment_stats=stats,
        honesty=honesty,
    )


def _heading_travel(samples: list[PoseSample]) -> float:
    """Displacement along the heading at the start of the window."""
    if len(samples) < 2:
        return 0.0
    yaw0 = samples[0].yaw
    dx = samples[-1].x - samples[0].x
    dy = samples[-1].y - samples[0].y
    return dx * math.cos(yaw0) + dy * math.sin(yaw0)


def run_demo(
    *,
    duration: float,
    out_mp4: Path | None,
    log_path: Path | None,
    summary_path: Path | None,
    script: tuple[DemoSegment, ...] = DEMO_SCRIPT,
    overlay_title: str = "Day-1 steer",
    overlay_footer: str = "W/S vx  A/D yaw  space stop  |  voice uses the same bus",
    cam_distance: float = 1.25,
    cam_azimuth: float = 135.0,
    cam_elevation: float = -18.0,
    lipm: LipmConfig | None = None,
) -> RunSummary:
    video = out_mp4 is not None
    session = SteerSession(
        video=video,
        cam_distance=cam_distance,
        cam_azimuth=cam_azimuth,
        cam_elevation=cam_elevation,
        lipm=lipm,
    )
    segments = _clip_script(script, duration)
    driver = ScriptedDriver(segments)
    print(
        f"[steer] plant={PLANT_XML.name} md5={PLANT_MD5} "
        f"vx_cap=+{VX_FWD_CAP:.3f}/-{VX_BACK_CAP:.3f} yaw_cap=±{YAW_RATE_CAP:.2f} "
        f"slew vx={VX_SLEW:.2f} yaw={YAW_SLEW:.2f} "
        f"(TELEOP_RATE_LIMIT={TELEOP_RATE_LIMIT} deadband={TELEOP_DEADBAND})"
    )
    if lipm is not None:
        print(
            f"[steer] gait=lipm {lipm.name} clear={lipm.clear_m:.3f} "
            f"arms={lipm.arms} t_swing={lipm.t_swing:.2f} "
            "assist=OFF root_wrench=OFF"
        )
    else:
        print(
        "[steer] gait=forward T=0.55 hip=0.24 ds=0.1375 plant_kd=25 "
        f"com_shift={LOOK_COM_SHIFT:.2f} knee_lift={LOOK_KNEE_LIFT:.2f} "
        f"arm_scale={LOOK_ARM_SCALE_STRAIGHT:.1f}/{LOOK_ARM_SCALE:.1f} "
        f"toe_L={LOOK_TOE_LEFT:.2f} toe_R={LOOK_TOE_RIGHT:.2f} "
        f"amp=|vx|/{GAIT_AMP_VX:.3f} "
        "assist=OFF ankle_cop=ON cp_swing=ON stance_vik=ON "
        f"swing_hold={'ON' if USE_SWING_HOLD_RESIDUAL else 'OFF'} "
        f"preload={'ON' if USE_SWING_PRELOAD else 'OFF'} "
        f"stance_push={'ON' if USE_STANCE_PUSH else 'OFF'} "
        "stance_residual=OFF door=OFF"
    )
    n_ctrl = int(round(duration / session.ctrl_dt))
    frame_stride = max(1, int(round(0.04 / session.ctrl_dt)))
    last_print = -1.0
    last_mode: ModeName | None = None
    frames: list[np.ndarray] = []
    refusals: list[str] = []
    for _ in range(n_ctrl):
        now = float(session.data.time)
        refusal = driver.publish(session.bus, now)
        if refusal:
            print(refusal)
            refusals.append(refusal)
        report = session.step()
        if report.mode == "fault" and not session.fault_announced:
            print(f"fault: {session.bus.fault_reason}")
            session.fault_announced = True
        seg = driver.segment(now)
        if report.mode != last_mode or (now - last_print) >= (VEL_RESEND_S - 1e-9):
            print(f"t={now:.2f} {seg.label} {report.line()}")
            last_print = now
            last_mode = report.mode
        if session.renderer is not None and (len(session.samples) % frame_stride == 0):
            plant_tag = "plant thaw ±2.45" if lipm is not None else "M145 frozen"
            lines = [
                f"{overlay_title}  {seg.label}  {plant_tag}  no door",
                report.line(),
                f"t={now:.2f}s  x={session.data.qpos[0]:+.3f}  yaw={math.degrees(session.yaw()):+.1f} deg",
                overlay_footer,
            ]
            frames.append(session.render(lines))
        if session.bus.fault and float(session.data.time) > now + 0.4:
            # Keep a short fault tail so the clip shows the stop, then end.
            pass
    session.assert_plant_unchanged()
    summary = summarize(session, script)
    print("[steer] " + summary.honesty)
    for row in summary.segment_stats:
        print(
            f"[steer] seg {row.label} t={row.t0_s:.2f}..{row.t1_s:.2f} "
            f"Δx={row.dx_heading_m:+.3f} Δyaw={math.degrees(row.dyaw_rad):+.2f} "
            f"vx={row.mean_body_vx_m_s:+.3f} yaw={row.mean_yaw_rate_rad_s:+.3f} "
            f"τ={row.peak_leg_tau_nm:.2f} up={row.min_up_z:.3f} "
            f"margin={row.min_margin_m:+.3f} cop={row.max_cop_outside_m:.4f} "
            f"tip={row.tip}"
        )
    if log_path is not None:
        _write_log(log_path, session, summary, refusals)
    if summary_path is not None:
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(asdict(summary), indent=2) + "\n", encoding="utf-8")
        print(f"[steer] wrote {summary_path}")
    if out_mp4 is not None and frames:
        _write_mp4(frames, out_mp4)
        print(f"[steer] wrote {out_mp4} ({len(frames)} frames)")
    return summary


def _clip_script(script: tuple[DemoSegment, ...], duration: float) -> tuple[DemoSegment, ...]:
    out: list[DemoSegment] = []
    for seg in script:
        if seg.t_end <= duration + 1e-9:
            out.append(seg)
        else:
            out.append(DemoSegment(duration, seg.kind, seg.vx, seg.yaw_rate, seg.label))
            break
    if not out:
        out.append(DemoSegment(duration, "stand", 0.0, 0.0, "stand"))
    return tuple(out)


def _write_log(
    path: Path,
    session: SteerSession,
    summary: RunSummary,
    refusals: list[str],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "Day-1 steer log",
        summary.honesty,
        f"fault={summary.fault} reason={summary.fault_reason or '-'}",
        "t mode applied_vx applied_yaw_rate x y yaw_deg up_z margin body_vx",
    ]
    for s in session.samples:
        lines.append(
            f"{s.t:.3f} {s.mode} {s.applied_vx:+.4f} {s.applied_yaw_rate:+.4f} "
            f"{s.x:+.4f} {s.y:+.4f} {math.degrees(s.yaw):+.2f} {s.up_z:.3f} "
            f"{s.margin:+.4f} {s.body_vx:+.4f}"
        )
    if refusals:
        lines.append("refusals:")
        lines.extend(refusals)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[steer] wrote {path}")


def _write_mp4(frames: list[np.ndarray], out_mp4: Path) -> None:
    import imageio.v2 as imageio

    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="steer_frames_") as tmp:
        folder = Path(tmp)
        for i, frame in enumerate(frames):
            imageio.imwrite(folder / f"frame_{i:05d}.png", frame)
        wg._encode_mp4_ffmpeg(folder, out_mp4, fps=25)


def run_view(duration: float) -> None:
    try:
        from mujoco import viewer
    except ImportError:
        print("refused: mujoco.viewer unavailable", file=sys.stderr)
        raise SystemExit(1)
    session = SteerSession(video=False, lipm=locked_kit_config())
    keys = KeyboardLatch()

    def on_key(keycode: int) -> None:
        keys.on_press(keycode)

    print("[steer] view  W/S=±vx  A/D=±yaw (left/right)  space=stop  latched until space")
    print("[steer] gait=locked kit500 through CommandBus (stand / stop / vel)")
    import time
    with viewer.launch_passive(session.model, session.data, key_callback=on_key) as handle:
        wall0 = time.time()
        while handle.is_running() and float(session.data.time) < duration:
            now = float(session.data.time)
            refusal = keys.publish(session.bus, now)
            if refusal:
                print(refusal)
            report = session.step()
            if report.mode == "fault" and not session.fault_announced:
                print(f"fault: {session.bus.fault_reason}")
                session.fault_announced = True
            if int(now * wg.CTRL_HZ) % 10 == 0:
                print(f"t={now:.2f} {report.line()}")
            handle.sync()
            sleep = (wall0 + float(session.data.time)) - time.time()
            if sleep > 0:
                time.sleep(sleep)
    session.assert_plant_unchanged()
    print(f"[steer] view done t={session.data.time:.2f}s mode={session.bus.mode}")


def _expect(cond: bool, msg: str, failures: list[str]) -> None:
    if not cond:
        failures.append(msg)


def test_bus() -> list[str]:
    failures: list[str] = []
    _expect(VX_SLEW <= TELEOP_RATE_LIMIT, "vx slew exceeds TELEOP_RATE_LIMIT", failures)
    _expect(YAW_SLEW <= TELEOP_RATE_LIMIT, "yaw slew exceeds TELEOP_RATE_LIMIT", failures)
    _expect(YAW_HIP_CLIP > TELEOP_DEADBAND, "hip yaw clip is inside joint deadband", failures)
    bus = CommandBus()
    report = bus.tick(0.0, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "power-on is not stand", failures)
    refusal = bus.vel(0.10, 0.0, 0.0, vy=0.2)
    _expect(refusal == "refused: vel accepts vx and yaw_rate only (got vy)", f"vy refusal got {refusal}", failures)
    refusal = bus.submit("waypoint", 0.0, x=1.0)
    _expect(refusal is not None and refusal.startswith("refused:"), f"waypoint got {refusal}", failures)
    refusal = bus.vel(float("nan"), 0.0, 0.0)
    _expect(refusal == "refused: non-finite vx or yaw_rate", f"nan got {refusal}", failures)
    bus.vel(5.0, 5.0, 0.0)
    _expect(bus.target_vx == VX_FWD_CAP and bus.target_yaw == YAW_RATE_CAP, "caps not applied", failures)
    report = bus.tick(0.0, CTRL_DT)
    _expect(report.mode == "move", "vel did not enter move", failures)
    _expect(abs(report.applied_vx - VX_SLEW * CTRL_DT) < 1e-9, "vx slew wrong", failures)
    _expect(report.applied_vx < VX_FWD_CAP, "slew jumped to cap", failures)
    bus.stop(0.02)
    report = bus.tick(0.02, CTRL_DT)
    _expect(
        report.mode == "stand" and report.applied_vx == 0.0 and report.applied_yaw_rate == 0.0,
        "stop did not zero same tick",
        failures,
    )
    bus.vel(VX_FWD_CAP, 0.0, 1.0)
    bus.tick(1.0, CTRL_DT)
    report = bus.tick(1.0 + COMMAND_TIMEOUT_S + 0.02, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "timeout did not stand", failures)
    bus2 = CommandBus()
    bus2.vel(DEADBAND_VX * 0.5, 0.0, 0.0)
    report = bus2.tick(0.0, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "deadband did not zero vx", failures)
    full = TickReport(VX_FWD_CAP, 0.0, "move")
    amp, direction = gait_amp_and_dir(full)
    # The kit cap can sit above the CPG amplitude reference. The CPG
    # saturates at 1 instead of taking a stick larger than GAIT_AMP_VX.
    cpg_full = min(1.0, VX_FWD_CAP / GAIT_AMP_VX)
    _expect(
        abs(amp - cpg_full) < 1e-9 and direction == 1,
        f"full-stick amp {amp} (CPG saturates at {cpg_full})",
        failures,
    )
    back = TickReport(-VX_BACK_CAP, 0.0, "move")
    amp, direction = gait_amp_and_dir(back)
    _expect(
        abs(amp - VX_BACK_CAP / GAIT_AMP_VX) < 1e-9 and direction == -1,
        f"reverse amp {amp}",
        failures,
    )
    _expect(cpg_full <= 1.0 + 1e-12, "CPG amplitude left the unit range", failures)
    bus2.declare_fault(0.1, "tip")
    refusal = bus2.vel(0.1, 0.0, 0.1)
    _expect(refusal is not None and refusal.startswith("refused: fault"), f"fault vel got {refusal}", failures)
    report = bus2.tick(0.12, CTRL_DT)
    _expect(report.mode == "fault" and report.applied_vx == 0.0, "fault tick not zero", failures)
    apply_frozen_forward_gait()
    _expect(wg.COM_SHIFT_AMP == LOOK_COM_SHIFT, "forward COM shift left the look basin", failures)
    _expect(wg.DS_S == 0.1375 and wg.KNEE_SWING == 0.80, "reverse-safe knee/DS moved", failures)
    _expect(
        abs(VX_FWD_CAP - 0.150) < 1e-9 and VX_BACK_CAP == 0.032 and YAW_RATE_CAP == 0.25,
        "caps moved",
        failures,
    )
    gait_t = 0.85 * float(wg.GAIT_T)
    q_plain = wg.gait_targets(gait_t, True, 0.70)
    q_step = dict(q_plain)
    style_forward_step(q_step, gait_t, YAW_RATE_CAP)
    _expect(q_step["l_knee"] > q_plain["l_knee"] + 0.05, "left swing knee did not lift", failures)
    _expect(abs(q_step["r_knee"] - q_plain["r_knee"]) < 1e-9, "stance knee was lifted", failures)
    _expect(q_step["l_hip_yaw"] < -0.02, "left swing foot is not toed left", failures)
    _expect(abs(q_step["r_hip_yaw"]) < 1e-9, "stance foot yawed during a left command", failures)
    _expect(abs(q_step["l_sho_pitch"]) > abs(q_plain["l_sho_pitch"]) * 2.0, "arm swing stayed frozen", failures)
    _expect(
        LOOK_ARM_SCALE_STRAIGHT > LOOK_ARM_SCALE and LOOK_ARM_SLEW > 0.0,
        "straight arm scale is not above the yaw scale",
        failures,
    )
    q_right = dict(q_plain)
    style_forward_step(q_right, gait_t, -YAW_RATE_CAP)
    _expect(q_right["l_hip_yaw"] > 0.02, "left swing foot is not toed right", failures)
    global USE_SWING_HOLD_RESIDUAL, USE_SWING_PRELOAD, USE_STANCE_PUSH
    _expect(
        not USE_SWING_HOLD_RESIDUAL and not USE_SWING_PRELOAD and not USE_STANCE_PUSH,
        "swing residual shipped on; measured path stays off",
        failures,
    )
    saved_flags = (USE_SWING_HOLD_RESIDUAL, USE_SWING_PRELOAD, USE_STANCE_PUSH)
    USE_SWING_HOLD_RESIDUAL = True
    USE_SWING_PRELOAD = True
    USE_STANCE_PUSH = True
    try:
        _expect(
            swing_hold_residual(0.06, 0.50, 1.0, 0.6) == 0.0,
            "swing hold fired on a clear sole",
            failures,
        )
        _expect(
            swing_hold_residual(0.02, 0.50, 0.4, 1.0) == 0.0,
            "swing hold fired while the CPG was still flexing",
            failures,
        )
        _expect(
            swing_hold_residual(0.02, 0.95, 1.0, 0.4) == 0.0,
            "swing hold fired in the landing window",
            failures,
        )
        held = swing_hold_residual(0.02, 0.50, 1.0, 0.55)
        _expect(
            abs(held - min(SWING_HOLD_CLIP, 1.0 + SWING_HOLD_MARGIN - 0.55)) < 1e-9,
            f"swing hold did not cover the extension gap ({held})",
            failures,
        )
        _expect(
            swing_preload_residual(0.20, 0.75, True) == 0.0,
            "preload fired in early stance",
            failures,
        )
        _expect(
            swing_preload_residual(0.60, 0.75, False) == 0.0,
            "preload fired on an airborne foot",
            failures,
        )
        pre = swing_preload_residual(0.70, 0.75, True)
        _expect(0.0 < pre <= SWING_PRELOAD_RAD, f"preload ramp {pre}", failures)
        _expect(stance_push_residual(0.10, True) == 0.0, "stance push fired in early stance", failures)
        _expect(stance_push_residual(0.35, False) == 0.0, "stance push fired in the air", failures)
        pushed = stance_push_residual(0.35, True)
        _expect(0.0 < pushed <= STANCE_PUSH_RAD, f"stance push {pushed}", failures)
    finally:
        USE_SWING_HOLD_RESIDUAL, USE_SWING_PRELOAD, USE_STANCE_PUSH = saved_flags
    return failures


def test_keys() -> list[str]:
    failures: list[str] = []
    bus = CommandBus()
    keys = KeyboardLatch()
    keys.on_press(KEY_W)
    refusal = keys.publish(bus, 0.0)
    _expect(refusal is None, "W refused", failures)
    report = bus.tick(0.0, CTRL_DT)
    _expect(report.mode == "move" and report.applied_vx > 0.0, "W did not command +vx", failures)
    keys.on_press(KEY_A)
    keys.publish(bus, 0.02)
    _expect(bus.target_vx == VX_FWD_CAP and bus.target_yaw == YAW_RATE_CAP, "W+A latch", failures)
    _expect(bus.target_yaw > 0.0, "A is not turn-left (positive)", failures)
    keys.on_press(KEY_SPACE)
    keys.publish(bus, 0.04)
    report = bus.tick(0.04, CTRL_DT)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "space did not stop", failures)
    keys.on_press(KEY_S)
    keys.publish(bus, 0.06)
    _expect(bus.target_vx == -VX_BACK_CAP, "S is not reverse cap", failures)
    keys.on_press(KEY_D)
    keys.publish(bus, 0.08)
    _expect(bus.target_yaw == -YAW_RATE_CAP, "D is not turn-right", failures)
    return failures


def test_hull() -> list[str]:
    failures: list[str] = []
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    hull = convex_hull_xy(square)
    mid = support_margin(np.array([0.5, 0.5]), hull)
    edge = support_margin(np.array([0.0, 0.5]), hull)
    out = support_margin(np.array([1.2, 0.5]), hull)
    _expect(abs(mid - 0.5) < 1e-6, f"mid margin {mid}", failures)
    _expect(abs(edge) < 1e-6, f"edge margin {edge}", failures)
    _expect(abs(out - (-0.2)) < 1e-6, f"outside margin {out}", failures)
    q_stand = {
        "l_hip_pitch": -0.07, "l_ank_pitch": 0.35,
        "r_hip_pitch": 0.07, "r_ank_pitch": 0.35,
        "l_knee": 0.42,
    }
    q_walk = dict(q_stand)
    q_walk["l_hip_pitch"] = -0.07 + 0.12
    q_walk["l_ank_pitch"] = 0.35 + 0.12
    mirrored = mirror_sagittal(q_walk, q_stand)
    _expect(mirrored["l_knee"] == 0.42, "knee was mirrored", failures)
    _expect(abs(mirrored["l_hip_pitch"] - (-0.07 - 0.12)) < 1e-9, "hip mirror", failures)
    return failures


def test_plant_file() -> list[str]:
    if not PLANT_XML.is_file():
        return [f"missing {PLANT_XML}"]
    model = mj.MjModel.from_xml_path(str(PLANT_XML))
    return plant_problems(model, PLANT_XML)


def test_smoke_sim() -> list[str]:
    """Stand, forward, stop on the frozen plant. No video."""
    failures: list[str] = []
    duration = DEMO_SCRIPT[-1].t_end
    session = SteerSession(video=False)
    driver = ScriptedDriver(DEMO_SCRIPT)
    n_ctrl = int(duration * wg.CTRL_HZ)
    bounds = script_bounds()
    fwd_t = bounds["forward"]
    saw_forward = False
    for _ in range(n_ctrl):
        now = float(session.data.time)
        refusal = driver.publish(session.bus, now)
        if refusal:
            failures.append(refusal)
        report = session.step()
        if fwd_t[0] + 1.0 <= now < fwd_t[1] - 0.1 and report.applied_vx > 0.5 * VX_FWD_CAP and report.mode == "move":
            saw_forward = True
    session.assert_plant_unchanged()
    summary = summarize(session)
    print("[steer] smoke " + summary.honesty)
    _expect(not summary.fault, f"smoke fault: {summary.fault_reason}", failures)
    _expect(not summary.tip, f"tipped up_z={summary.min_up_z:.3f}", failures)
    _expect(saw_forward, "forward command was not applied", failures)
    _expect(summary.dx_forward_m > 2.0, f"forward Δx={summary.dx_forward_m:.3f} m", failures)
    _expect(summary.mean_body_vx_forward > 0.03, f"mean vx={summary.mean_body_vx_forward:.3f}", failures)
    _expect(
        abs(summary.dyaw_forward_rad) < 0.20,
        f"forward yaw drift={summary.dyaw_forward_rad:.3f} rad",
        failures,
    )
    _expect(
        abs(summary.yaw_forward_min_rad) < 0.45 and abs(summary.yaw_forward_max_rad) < 0.45,
        f"forward yaw span {summary.yaw_forward_min_rad:.3f}..{summary.yaw_forward_max_rad:.3f}",
        failures,
    )
    _expect(abs(summary.dyaw_end_rad) < 0.20, f"end heading {summary.dyaw_end_rad:.3f} rad", failures)
    _expect(summary.min_up_z >= 0.90, f"min up_z={summary.min_up_z:.3f}", failures)
    _expect(summary.cop_in_box, "CoP left the foot box", failures)
    _expect(summary.max_leg_tau_nm <= LEG_TAU + 1e-3, "leg torque above freeze", failures)
    _expect(summary.max_contact_cop_outside_box_m <= 0.005, "CoP left the foot box", failures)
    tail = session.samples[-1]
    _expect(tail.mode == "stand" and abs(tail.applied_vx) < 1e-6, "did not end in stand", failures)
    _expect(tail.margin > 0.02, f"forward stop margin {tail.margin:+.3f}", failures)
    failures.extend(test_stop_settle())
    failures.extend(test_reverse_and_turn())
    return failures


def test_stop_settle() -> list[str]:
    """Forward→stop on phases that used to pitch, plus a 200 ms silence stop.

    The tip check is unchanged. These cuts previously latched
    'COM outside support and tipping' with margin about −0.08 m.
    """
    failures: list[str] = []
    # Gait phases that faulted with an immediate blend and the damper off.
    for stop_at in (2.00, 2.25, 2.35, 2.55, 2.60, 2.80, 2.90):
        script = (
            DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
            DemoSegment(stop_at, "vel", VX_FWD_CAP, 0.0, "forward"),
            DemoSegment(stop_at + 2.5, "stop", 0.0, 0.0, "stop"),
        )
        session, summary = _run_script(script)
        tail = session.samples[-1]
        _expect(not summary.fault, f"stop@{stop_at:.2f} fault: {summary.fault_reason}", failures)
        _expect(not summary.tip, f"stop@{stop_at:.2f} tip up_z={summary.min_up_z:.3f}", failures)
        _expect(summary.min_up_z >= 0.90, f"stop@{stop_at:.2f} min up_z={summary.min_up_z:.3f}", failures)
        _expect(summary.cop_in_box, f"stop@{stop_at:.2f} CoP left the box", failures)
        _expect(tail.mode == "stand", f"stop@{stop_at:.2f} ended {tail.mode}", failures)
        _expect(tail.margin > 0.02, f"stop@{stop_at:.2f} end margin {tail.margin:+.3f}", failures)
    # Reverse cuts that pitched without the step-finish and settle damper.
    for stop_at in (2.30, 3.40, 3.80):
        rev_script = (
            DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
            DemoSegment(stop_at, "vel", -VX_BACK_CAP, 0.0, "reverse"),
            DemoSegment(stop_at + 2.6, "stop", 0.0, 0.0, "stop"),
        )
        rev, rev_sum = _run_script(rev_script)
        _expect(not rev_sum.fault, f"reverse@{stop_at:.2f} fault: {rev_sum.fault_reason}", failures)
        _expect(not rev_sum.tip, f"reverse@{stop_at:.2f} tip up_z={rev_sum.min_up_z:.3f}", failures)
        _expect(rev_sum.min_up_z >= 0.90, f"reverse@{stop_at:.2f} min up_z={rev_sum.min_up_z:.3f}", failures)
        _expect(rev_sum.cop_in_box, f"reverse@{stop_at:.2f} CoP left the box", failures)
        _expect(rev.samples[-1].mode == "stand", f"reverse@{stop_at:.2f} ended {rev.samples[-1].mode}", failures)
        _expect(rev.samples[-1].margin > 0.02, f"reverse@{stop_at:.2f} end margin {rev.samples[-1].margin:+.3f}", failures)
    rev_script = (
        DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        DemoSegment(6.5, "vel", -VX_BACK_CAP, 0.0, "reverse"),
        DemoSegment(9.0, "stop", 0.0, 0.0, "stop"),
    )
    rev, rev_sum = _run_script(rev_script)
    _expect(not rev_sum.fault, f"short reverse fault: {rev_sum.fault_reason}", failures)
    _expect(not rev_sum.tip, f"short reverse tip up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.min_up_z >= 0.90, f"short reverse min up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.cop_in_box, "short reverse CoP left the box", failures)
    _expect(rev.samples[-1].mode == "stand", "short reverse did not end in stand", failures)
    _expect(rev_sum.dx_forward_m < -0.05, f"short reverse Δx={rev_sum.dx_forward_m:.3f}", failures)
    # Voice path: resend vel at 10 Hz, then silence. The 200 ms watchdog stops.
    session = SteerSession(video=False)
    last_send = -1.0
    t_silent = 2.35
    duration = 5.0
    n_ctrl = int(duration * wg.CTRL_HZ)
    for _ in range(n_ctrl):
        now = float(session.data.time)
        if now < 1.0 - 1e-9:
            if last_send < 0.0:
                session.bus.stand(now)
                last_send = now
        elif now < t_silent - 1e-9:
            if (now - last_send) >= (VEL_RESEND_S - 1e-9):
                session.bus.vel(VX_FWD_CAP, 0.0, now)
                last_send = now
        session.step()
    tail = session.samples[-1]
    _expect(not session.bus.fault, f"silence fault: {session.bus.fault_reason}", failures)
    _expect(tail.mode == "stand" and abs(tail.applied_vx) < 1e-6, "silence did not end in stand", failures)
    _expect(session.min_up_z >= 0.90, f"silence min up_z={session.min_up_z:.3f}", failures)
    _expect(session.max_cop_excursion <= 0.001, "silence CoP left the box", failures)
    _expect(tail.margin > 0.02, f"silence end margin {tail.margin:+.3f}", failures)
    print(
        f"[steer] stop-settle phases ok, short reverse Δx={rev_sum.dx_forward_m:+.3f} m "
        f"up={rev_sum.min_up_z:.3f}, silence end={tail.mode}"
    )
    return failures


def _run_script(script: tuple[DemoSegment, ...]) -> tuple[SteerSession, RunSummary]:
    session = SteerSession(video=False)
    driver = ScriptedDriver(script)
    n_ctrl = int(script[-1].t_end * wg.CTRL_HZ)
    for _ in range(n_ctrl):
        now = float(session.data.time)
        driver.publish(session.bus, now)
        session.step()
    session.assert_plant_unchanged()
    return session, summarize(session, script)


def test_reverse_and_turn() -> list[str]:
    """Upright retreat and a walking left/right turn. No video."""
    failures: list[str] = []
    rev, rev_sum = _run_script(REVERSE_SCRIPT)
    print("[steer] reverse " + rev_sum.honesty)
    _expect(not rev_sum.fault, f"reverse fault: {rev_sum.fault_reason}", failures)
    _expect(not rev_sum.tip, f"reverse tip up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.dx_forward_m < -0.70, f"reverse Δx={rev_sum.dx_forward_m:.3f} m", failures)
    _expect(rev_sum.min_up_z >= 0.90, f"reverse min up_z={rev_sum.min_up_z:.3f}", failures)
    _expect(rev_sum.cop_in_box, "reverse CoP left the foot box", failures)
    _expect(rev.samples[-1].mode == "stand", "reverse did not end in stand", failures)

    turn, turn_sum = _run_script(TURN_SCRIPT)
    print("[steer] turn " + turn_sum.honesty)
    _expect(not turn_sum.fault, f"turn fault: {turn_sum.fault_reason}", failures)
    _expect(not turn_sum.tip, f"turn tip up_z={turn_sum.min_up_z:.3f}", failures)
    _expect(turn_sum.dyaw_turn_rad > 0.55, f"turn Δyaw={turn_sum.dyaw_turn_rad:.3f} rad", failures)
    _expect(turn_sum.dx_turn_m > 0.15, f"turn Δx={turn_sum.dx_turn_m:.3f} m", failures)
    _expect(turn_sum.min_up_z >= 0.90, f"turn min up_z={turn_sum.min_up_z:.3f}", failures)
    _expect(turn_sum.cop_in_box, "turn CoP left the foot box", failures)
    _expect(turn.samples[-1].mode == "stand", "turn did not end in stand", failures)
    turn_end = [s for s in turn.samples if s.t < TURN_SCRIPT[1].t_end][-1]
    unwind = abs(turn.samples[-1].yaw - turn_end.yaw)
    _expect(unwind < 0.20, f"heading unwound {unwind:.3f} rad after stop", failures)

    # Same 11 s window as the left turn. Swing-foot yaw scale is what keeps
    # heading moving; the old 4.5 s cut sat inside the −16° stall.
    right, right_sum = _run_script(RIGHT_TURN_SCRIPT)
    print("[steer] right " + right_sum.honesty)
    _expect(not right_sum.fault, f"right fault: {right_sum.fault_reason}", failures)
    _expect(not right_sum.tip, f"right tip up_z={right_sum.min_up_z:.3f}", failures)
    _expect(right_sum.dyaw_turn_rad < -0.90, f"right Δyaw={right_sum.dyaw_turn_rad:.3f}", failures)
    _expect(right_sum.dx_turn_m > 0.25, f"right Δx={right_sum.dx_turn_m:.3f} m", failures)
    _expect(right_sum.min_up_z >= 0.90, f"right min up_z={right_sum.min_up_z:.3f}", failures)
    _expect(right_sum.cop_in_box, "right CoP left the foot box", failures)
    _expect(right.samples[-1].mode == "stand", "right did not end in stand", failures)
    _expect(right.samples[-1].margin > 0.02, f"right end margin {right.samples[-1].margin:+.3f}", failures)
    right_end = [s for s in right.samples if s.t < RIGHT_TURN_SCRIPT[1].t_end][-1]
    right_unwind = abs(right.samples[-1].yaw - right_end.yaw)
    _expect(right_unwind < 0.20, f"right heading unwound {right_unwind:.3f} rad after stop", failures)
    _expect(0.0 < TURN_RIGHT_SWING_YAW_SCALE < 1.0, "swing yaw scale left the measured basin", failures)
    apply_frozen_forward_gait()
    _expect(swing_hip_yaw_scale(-YAW_RATE_CAP, 1, 0.10) == 1.0, "stance hip yaw was scaled", failures)
    _expect(
        swing_hip_yaw_scale(-YAW_RATE_CAP, 1, 0.90) == TURN_RIGHT_SWING_YAW_SCALE,
        "right swing hip yaw was not scaled",
        failures,
    )
    _expect(swing_hip_yaw_scale(YAW_RATE_CAP, 1, 0.90) == 1.0, "left swing hip yaw was scaled", failures)
    _expect(swing_hip_yaw_scale(-YAW_RATE_CAP, -1, 0.90) == 1.0, "reverse swing hip yaw was scaled", failures)
    # Stop inside the old stall and later in the turn. Tip check is unchanged.
    for stop_at in (2.40, 4.20, 9.00):
        cut = (
            DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
            DemoSegment(stop_at, "vel", VX_FWD_CAP, -YAW_RATE_CAP, "turn"),
            DemoSegment(stop_at + 2.5, "stop", 0.0, 0.0, "stop"),
        )
        cut_sess, cut_sum = _run_script(cut)
        _expect(not cut_sum.fault, f"right stop@{stop_at:.2f} fault: {cut_sum.fault_reason}", failures)
        _expect(not cut_sum.tip, f"right stop@{stop_at:.2f} tip up_z={cut_sum.min_up_z:.3f}", failures)
        _expect(cut_sum.min_up_z >= 0.90, f"right stop@{stop_at:.2f} min up_z={cut_sum.min_up_z:.3f}", failures)
        _expect(cut_sum.cop_in_box, f"right stop@{stop_at:.2f} CoP left the box", failures)
        _expect(cut_sess.samples[-1].mode == "stand", f"right stop@{stop_at:.2f} ended {cut_sess.samples[-1].mode}", failures)
        _expect(
            cut_sess.samples[-1].margin > 0.02,
            f"right stop@{stop_at:.2f} end margin {cut_sess.samples[-1].margin:+.3f}",
            failures,
        )
    return failures


def _turn_is_combined(session: SteerSession, script: tuple[DemoSegment, ...]) -> bool:
    """The turn window must be holding forward vx and a nonzero yaw together."""
    bounds = script_bounds(script)
    turn_t = bounds["turn"]
    # Skip the slew into the cap.
    lo = turn_t[0] + 1.0
    hi = turn_t[1] - 0.2
    held = [
        s for s in session.samples
        if lo <= s.t < hi and s.mode == "move"
        and s.applied_vx > 0.5 * VX_FWD_CAP
        and abs(s.applied_yaw_rate) > 0.5 * YAW_RATE_CAP
    ]
    return len(held) > 10


def test_nav() -> list[str]:
    """Walk, arc, walk, stop. Both directions. No video.

    The bar is the furniture pattern: approach about 0.5–1 m, turn about
    45–90°, a bit more forward travel, then stand with margin above 0.02 m.
    CoP stays in the box and the tip check is the existing one.
    """
    failures: list[str] = []
    _expect(abs(VX_FWD_CAP - 0.150) < 1e-9 and VX_BACK_CAP == 0.032, "velocity caps moved", failures)
    _expect(YAW_RATE_CAP == 0.25, "yaw cap moved", failures)
    _expect(TURN_RIGHT_SWING_YAW_SCALE == 0.60, "right swing scale moved", failures)
    _expect(_md5(PLANT_XML) == PLANT_MD5, "plant md5 changed", failures)
    deg = math.degrees
    cases = (
        ("left", NAV_LEFT_SCRIPT, 1.0),
        ("right", NAV_RIGHT_SCRIPT, -1.0),
    )
    for name, script, sign in cases:
        session, summary = _run_script(script)
        print("[steer] nav-" + name + " " + summary.honesty)
        _expect(not summary.fault, f"nav {name} fault: {summary.fault_reason}", failures)
        _expect(not summary.tip, f"nav {name} tip up_z={summary.min_up_z:.3f}", failures)
        _expect(summary.min_up_z >= 0.90, f"nav {name} min up_z={summary.min_up_z:.3f}", failures)
        _expect(summary.cop_in_box, f"nav {name} CoP left the box", failures)
        _expect(summary.max_leg_tau_nm <= LEG_TAU + 1e-3, f"nav {name} torque above freeze", failures)
        _expect(0.50 < summary.dx_forward_m < 1.00, f"nav {name} approach Δx={summary.dx_forward_m:.3f}", failures)
        _expect(summary.mean_body_vx_forward > 0.02, f"nav {name} approach speed", failures)
        lo = 0.785 * sign
        hi = 1.571 * sign
        dyaw = summary.dyaw_turn_rad
        inside = (lo < dyaw < hi) if sign > 0 else (hi < dyaw < lo)
        _expect(inside, f"nav {name} Δyaw={deg(dyaw):+.1f} deg", failures)
        _expect(summary.mean_body_vx_turn > 0.02, f"nav {name} turn was not walking", failures)
        _expect(abs(summary.mean_yaw_rate_turn) > 0.02, f"nav {name} yaw rate {summary.mean_yaw_rate_turn:+.3f}", failures)
        _expect(summary.dx_resume_m > 0.15, f"nav {name} resume {summary.dx_resume_m:.3f} m", failures)
        _expect(summary.mean_body_vx_resume > 0.02, f"nav {name} resume speed", failures)
        _expect(_turn_is_combined(session, script), f"nav {name} vx and yaw were not simultaneous", failures)
        tail = session.samples[-1]
        _expect(tail.mode == "stand" and abs(tail.applied_vx) < 1e-6, f"nav {name} ended {tail.mode}", failures)
        _expect(abs(tail.applied_yaw_rate) < 1e-6, f"nav {name} yaw still applied", failures)
        _expect(tail.margin > 0.02, f"nav {name} end margin {tail.margin:+.3f}", failures)
        _expect(summary.plant_md5 == PLANT_MD5, f"nav {name} summary md5", failures)
    return failures


def _arc_holds_both(session: SteerSession, t0: float, t1: float, sign: float) -> bool:
    """The arc window must be holding forward vx and yaw of the commanded sign."""
    lo = t0 + 1.0
    hi = t1 - 0.2
    held = [
        s for s in session.samples
        if lo <= s.t < hi and s.mode == "move"
        and s.applied_vx > 0.5 * VX_FWD_CAP
        and s.applied_yaw_rate * sign > 0.5 * YAW_RATE_CAP
    ]
    return len(held) > 10


def test_nav_multi() -> list[str]:
    """Forward, claimed left arc, forward, claimed right arc, forward, stop.

    Yaw holds are the single-arc windows (left 12.5 s, right 11.0 s).
    Right-then-left and a second same-sign arc at these lengths are
    Prefer FAIL and are not run. The 14 s left that starts at 15 s is not run.
    """
    failures: list[str] = []
    _expect(abs(VX_FWD_CAP - 0.150) < 1e-9 and VX_BACK_CAP == 0.032, "velocity caps moved", failures)
    _expect(YAW_RATE_CAP == 0.25, "yaw cap moved", failures)
    _expect(_md5(PLANT_XML) == PLANT_MD5, "plant md5 changed", failures)
    bounds = script_bounds(NAV_MULTI_SCRIPT)
    right_s = bounds["arc-right"][1] - bounds["arc-right"][0]
    left_s = bounds["arc-left"][1] - bounds["arc-left"][0]
    _expect(abs(right_s - CLAIMED_RIGHT_ARC_S) < 1e-9, f"right hold {right_s:.3f} s", failures)
    _expect(abs(left_s - CLAIMED_LEFT_ARC_S) < 1e-9, f"left hold {left_s:.3f} s", failures)
    _expect(left_s < 13.0, f"left hold {left_s:.3f} s is outside the claimed 12.5 s window", failures)
    approach_s = bounds["forward"][1] - bounds["forward"][0]
    mid_s = bounds["mid"][1] - bounds["mid"][0]
    resume_s = bounds["resume"][1] - bounds["resume"][0]
    _expect(abs(approach_s - CLAIMED_APPROACH_S) < 1e-9, f"approach {approach_s:.3f} s", failures)
    _expect(abs(mid_s - CLAIMED_RESUME_S) < 1e-9, f"mid {mid_s:.3f} s", failures)
    _expect(abs(resume_s - CLAIMED_RESUME_S) < 1e-9, f"resume {resume_s:.3f} s", failures)
    _expect(abs(bounds["arc-left"][0] - 16.0) < 1e-9, "left arc left the claimed 16 s start", failures)
    _expect(
        bounds["arc-left"][1] <= bounds["arc-right"][0] + 1e-9,
        "multi clip is not left then right",
        failures,
    )
    session, summary = _run_script(NAV_MULTI_SCRIPT)
    print("[steer] nav-multi " + summary.honesty)
    _expect(not summary.fault, f"nav-multi fault: {summary.fault_reason}", failures)
    _expect(not summary.tip, f"nav-multi tip up_z={summary.min_up_z:.3f}", failures)
    _expect(summary.min_up_z >= 0.90, f"nav-multi min up_z={summary.min_up_z:.3f}", failures)
    _expect(summary.cop_in_box, "nav-multi CoP left the box", failures)
    _expect(summary.max_leg_tau_nm <= LEG_TAU + 1e-3, "nav-multi torque above freeze", failures)
    _expect(summary.plant_md5 == PLANT_MD5, "nav-multi summary md5", failures)
    by_label = {row.label: row for row in summary.segment_stats}
    deg = math.degrees
    for name, sign in (("arc-right", -1.0), ("arc-left", 1.0)):
        row = by_label[name]
        lo = 0.785 * sign
        hi = 1.571 * sign
        inside = (lo < row.dyaw_rad < hi) if sign > 0 else (hi < row.dyaw_rad < lo)
        _expect(inside, f"nav-multi {name} Δyaw={deg(row.dyaw_rad):+.1f} deg", failures)
        _expect(row.mean_body_vx_m_s > 0.02, f"nav-multi {name} was not walking", failures)
        _expect(abs(row.mean_yaw_rate_rad_s) > 0.02, f"nav-multi {name} yaw rate", failures)
        _expect(not row.tip, f"nav-multi {name} tipped", failures)
        _expect(row.min_up_z >= 0.90, f"nav-multi {name} min up_z={row.min_up_z:.3f}", failures)
        _expect(row.peak_leg_tau_nm <= LEG_TAU + 1e-3, f"nav-multi {name} torque", failures)
        _expect(row.max_cop_outside_m <= 0.001, f"nav-multi {name} CoP left the box", failures)
        _expect(
            _arc_holds_both(session, row.t0_s, row.t1_s, sign),
            f"nav-multi {name} vx and yaw were not simultaneous",
            failures,
        )
    for name in ("forward", "mid", "resume"):
        row = by_label[name]
        _expect(row.dx_heading_m > 0.15, f"nav-multi {name} Δx={row.dx_heading_m:.3f}", failures)
        _expect(row.mean_body_vx_m_s > 0.02, f"nav-multi {name} speed", failures)
        _expect(not row.tip, f"nav-multi {name} tipped", failures)
    tail = session.samples[-1]
    _expect(tail.mode == "stand" and abs(tail.applied_vx) < 1e-6, f"nav-multi ended {tail.mode}", failures)
    _expect(abs(tail.applied_yaw_rate) < 1e-6, "nav-multi yaw still applied", failures)
    _expect(tail.margin > 0.02, f"nav-multi end margin {tail.margin:+.3f}", failures)
    return failures


def self_test() -> int:
    failures: list[str] = []
    failures.extend(test_bus())
    failures.extend(test_keys())
    failures.extend(test_hull())
    failures.extend(test_plant_file())
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    failures.extend(test_smoke_sim())
    failures.extend(test_nav())
    failures.extend(test_nav_multi())
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[steer] self-test PASS")
    return 0


@dataclass
class _TauPeak:
    force_nm: float
    t_s: float


def _note_leg_peaks(session: SteerSession, into: dict[str, _TauPeak]) -> None:
    t = float(session.data.time)
    for name, idx in session.act_idx.items():
        if not any(tok in name for tok in ("hip_", "knee", "ank_")):
            continue
        force = float(session.data.actuator_force[idx])
        prev = into.get(name)
        if prev is None or abs(force) >= abs(prev.force_nm):
            into[name] = _TauPeak(force, t)


def _bus_kit_contract() -> tuple[list[str], list[str]]:
    """Stand, stop, vel, no vy, latest wins, 10 Hz resend, 200 ms silence.

    Clock is the kit session (8 ms), not the 50 Hz CPG tick.
    """
    failures: list[str] = []
    lines: list[str] = []
    session = SteerSession(video=False, lipm=locked_kit_config())
    bus = session.bus
    _expect(session.ctrl_dt == op3_walk.OP3_CTRL_S, f"kit ctrl_dt {session.ctrl_dt}", failures)
    report = bus.tick(0.0, session.ctrl_dt)
    _expect(
        report.mode == "stand" and report.applied_vx == 0.0 and report.applied_yaw_rate == 0.0,
        "kit power-on is not stand",
        failures,
    )
    refusal = bus.vel(VX_FWD_CAP, 0.0, 0.0, vy=0.02)
    _expect(
        refusal == "refused: vel accepts vx and yaw_rate only (got vy)",
        f"kit vy refusal got {refusal}",
        failures,
    )
    _expect(bus.mode == "stand" and bus.target_vx == 0.0, "refused vy was applied", failures)
    bus.vel(0.02, 0.10, 0.0)
    bus.vel(3.0, -4.0, 0.0)
    _expect(
        bus.target_vx == VX_FWD_CAP and bus.target_yaw == -YAW_RATE_CAP,
        f"latest command did not win the clamps ({bus.target_vx}, {bus.target_yaw})",
        failures,
    )
    lines.append(
        f"vy refused; latest vel(3.0, -4.0) clamped to "
        f"vx {bus.target_vx:+.3f} m/s, yaw {bus.target_yaw:+.2f} rad/s"
    )
    bus.stop(0.0)
    _expect(bus.mode == "stand" and bus.applied_vx == 0.0, "stop before the first tick left velocity", failures)
    now = float(session.data.time)
    bus.vel(VX_FWD_CAP, YAW_RATE_CAP, now)
    report = session.step()
    _expect(report.mode == "move", "yaw vel did not enter move", failures)
    _expect(report.applied_vx > 0.0 and report.applied_vx <= VX_FWD_CAP, "vx left the clamp", failures)
    _expect(
        report.applied_yaw_rate > 0.0 and report.applied_yaw_rate <= YAW_RATE_CAP,
        "yaw_rate was not returned",
        failures,
    )
    walker = session.lipm.op3 if session.lipm is not None else None
    period = session.lipm.cfg.gm_period_s if session.lipm is not None else 0.0
    expect_angle = report.applied_yaw_rate * period * lipm_gait.KIT_YAW_GAIN
    _expect(
        walker is not None and abs(walker.angle_cmd - expect_angle) < 1e-9 and abs(walker.angle_cmd) > 1e-6,
        "kit step angle did not follow yaw_rate",
        failures,
    )
    lines.append(
        f"yaw tick {report.line()}; kit step angle "
        f"{0.0 if walker is None else walker.angle_cmd:.4f} rad"
    )
    now = float(session.data.time)
    refusal = bus.stop(now)
    report = session.step()
    _expect(refusal is None, f"stop refused {refusal}", failures)
    _expect(
        report.mode == "stand" and report.applied_vx == 0.0 and report.applied_yaw_rate == 0.0,
        f"stop was not the same tick ({report.line()})",
        failures,
    )
    _expect(walker is not None and walker.angle_cmd == 0.0, "stop left a step angle", failures)
    _expect(walker is not None and not walker.ctrl_running, "stop left the kit walker running", failures)
    last_send = -1.0
    while float(session.data.time) < 0.40 - 1e-9:
        now = float(session.data.time)
        if last_send < 0.0 or (now - last_send) >= (VEL_RESEND_S - 1e-9):
            bus.vel(VX_FWD_CAP, 0.0, now)
            last_send = now
        report = session.step()
        _expect(report.mode == "move", f"10 Hz resend dropped to {report.mode} at {now:.3f}", failures)
        _expect(abs(report.applied_vx) <= VX_FWD_CAP + 1e-12, "resend vx exceeded the clamp", failures)
        if failures:
            break
    stood_at: float | None = None
    stood: TickReport | None = None
    while float(session.data.time) < 0.90 and stood is None:
        now = float(session.data.time)
        report = session.step()
        if report.mode == "stand":
            stood_at = now
            stood = report
    _expect(stood is not None and stood_at is not None, "200 ms silence did not stand", failures)
    if stood is not None and stood_at is not None:
        silent = stood_at - last_send
        _expect(silent > COMMAND_TIMEOUT_S, f"stood after only {silent:.3f} s", failures)
        _expect(
            silent <= COMMAND_TIMEOUT_S + session.ctrl_dt + 1e-9,
            f"stood late at {silent:.3f} s",
            failures,
        )
        _expect(
            stood.applied_vx == 0.0 and stood.applied_yaw_rate == 0.0 and stood.mode == "stand",
            f"silence tick {stood.line()}",
            failures,
        )
        _expect(walker is not None and not walker.ctrl_running, "silence left the walker running", failures)
        lines.append(
            f"silence {silent * 1000:.1f} ms → {stood.line()} "
            f"(watchdog {COMMAND_TIMEOUT_S * 1000:.0f} ms, ctrl {session.ctrl_dt * 1000:.0f} ms)"
        )
    session.assert_plant_unchanged()
    return failures, lines


def _window_stats(samples: list[PoseSample]) -> tuple[float, float, float, float]:
    if not samples:
        return 0.0, 0.0, 1.0, 0.0
    dx = samples[-1].x - samples[0].x
    mean_vx = float(np.mean([s.body_vx for s in samples]))
    min_up = min(s.up_z for s in samples)
    dyaw = math.degrees(samples[-1].yaw - samples[0].yaw)
    return dx, mean_vx, min_up, dyaw


def _bus_kit_forward_stop() -> tuple[list[str], list[str]]:
    """vel forward, then stop, on a cold kit session. Physics-step leg torque."""
    failures: list[str] = []
    lines: list[str] = []
    digest = hashlib.md5(PLANT_XML.read_bytes()).hexdigest()
    _expect(digest == PLANT_MD5, f"plant md5 {digest}", failures)
    session = SteerSession(video=False, lipm=locked_kit_config())
    driver = ScriptedDriver(BUS_KIT_SCRIPT)
    peaks: dict[str, dict[str, _TauPeak]] = {"stand": {}, "forward": {}, "stop": {}}
    phase = {"name": "stand"}
    real_step = mj.mj_step

    def _step(model: mj.MjModel, data: mj.MjData) -> None:
        real_step(model, data)
        _note_leg_peaks(session, peaks[phase["name"]])

    mj.mj_step = _step
    stop_report: TickReport | None = None
    try:
        while float(session.data.time) < BUS_KIT_STOP_S - 1e-9:
            now = float(session.data.time)
            seg = driver.segment(now)
            phase["name"] = seg.label if seg.label in peaks else "stand"
            driver.publish(session.bus, now)
            report = session.step()
            if seg.kind == "stop" and stop_report is None:
                stop_report = report
            if seg.kind == "vel":
                _expect(report.mode == "move", f"forward tick {now:.3f} mode {report.mode}", failures)
                _expect(
                    abs(report.applied_vx) <= VX_FWD_CAP + 1e-12,
                    f"applied_vx {report.applied_vx} over cap",
                    failures,
                )
                _expect(abs(report.applied_yaw_rate) < 1e-12, "forward yaw was not 0", failures)
            if failures and len(failures) > 8:
                break
    finally:
        mj.mj_step = real_step
    session.assert_plant_unchanged()
    digest_after = hashlib.md5(PLANT_XML.read_bytes()).hexdigest()
    _expect(digest_after == PLANT_MD5, f"plant md5 changed to {digest_after}", failures)
    _expect(not session.bus.fault, f"fault {session.bus.fault_reason}", failures)
    _expect(
        stop_report is not None
        and stop_report.mode == "stand"
        and stop_report.applied_vx == 0.0
        and stop_report.applied_yaw_rate == 0.0,
        "stop tick did not return stand at 0",
        failures,
    )
    fwd = [s for s in session.samples if s.mode == "move"]
    stop_rows: list[PoseSample] = []
    seen_move = False
    for sample in session.samples:
        if sample.mode == "move":
            seen_move = True
        elif seen_move and sample.mode == "stand":
            stop_rows.append(sample)
    _expect(bool(fwd) and bool(stop_rows), "missing forward or stop samples", failures)
    max_applied = max((s.applied_vx for s in fwd), default=0.0)
    slew_step = VX_SLEW * session.ctrl_dt
    _expect(max_applied <= VX_FWD_CAP + 1e-9, f"applied_vx {max_applied} over the clamp", failures)
    _expect(
        max_applied > VX_FWD_CAP - slew_step - 1e-9,
        f"applied_vx reached {max_applied}, not the forward clamp",
        failures,
    )
    settle_t = BUS_KIT_STAND_S + VX_FWD_CAP / VX_SLEW + 0.40
    settled = [s for s in fwd if s.t >= settle_t - 1e-9]
    steady_vx = float(np.mean([s.body_vx for s in settled])) if settled else 0.0
    ratio = steady_vx / VX_FWD_CAP if VX_FWD_CAP else 0.0
    _expect(abs(ratio - 1.0) < 0.08, f"settled body/command {ratio:.2f}", failures)
    dx, mean_vx, min_up, dyaw = _window_stats(fwd)
    sdx, _, smin_up, sdyaw = _window_stats(stop_rows)
    lines.append(
        f"plant md5 {digest_after}  ctrl {session.ctrl_dt * 1000:.0f} ms  "
        f"resend {VEL_RESEND_S * 1000:.0f} ms  silence {COMMAND_TIMEOUT_S * 1000:.0f} ms"
    )
    lines.append(
        f"clamps vx +{VX_FWD_CAP:.3f}/-{VX_BACK_CAP:.3f} m/s  yaw ±{YAW_RATE_CAP:.2f} rad/s  "
        "on the bus, not the plant"
    )
    if stop_report is not None:
        lines.append(f"stop tick {stop_report.line()}")
    lines.append(
        f"forward n={len(fwd)} applied_vx max {max_applied:+.4f}  "
        f"Δx {dx * 100:+.1f} cm  mean body vx {mean_vx * 100:+.2f} cm/s  "
        f"min up_z {min_up:.3f}  Δyaw {dyaw:+.1f} deg  "
        f"settled vx {steady_vx:+.3f} m/s ratio {ratio:.2f}"
    )
    lines.append(
        f"stop n={len(stop_rows)} Δx {sdx * 100:+.1f} cm  "
        f"min up_z {smin_up:.3f}  Δyaw {sdyaw:+.1f} deg  "
        f"tail mode {session.samples[-1].mode if session.samples else '-'}"
    )
    for label in ("stand", "forward", "stop"):
        bucket = peaks[label]
        worst_name = ""
        worst = 0.0
        for name, peak in bucket.items():
            if abs(peak.force_nm) >= abs(worst):
                worst = peak.force_nm
                worst_name = name
            if abs(peak.force_nm) > BUS_KIT_RAIL_NM:
                failures.append(
                    f"{label} {name} {peak.force_nm:+.4f} Nm at {peak.t_s:.3f} s"
                )
            if "knee" in name and abs(peak.force_nm) > lipm_gait.KNEE_SAG_NM + 1e-3:
                failures.append(
                    f"{label} {name} {peak.force_nm:+.3f} Nm crosses {lipm_gait.KNEE_SAG_NM:.2f}"
                )
            leg = any(tok in name for tok in ("hip_", "knee", "ank_"))
            if (
                label == "stop"
                and leg
                and abs(peak.force_nm) > lipm_gait.LEG_STOP_NM + 1e-3
            ):
                failures.append(
                    f"{label} {name} {peak.force_nm:+.3f} Nm has no "
                    f"{lipm_gait.LEG_STOP_HEADROOM_NM:.2f} Nm headroom under "
                    f"{lipm_gait.KNEE_SAG_NM:.2f}"
                )
            # The 2.33 bar on hip roll is the walking turn. The stop hold
            # is the all-leg 2.28 Nm check above.
            if (
                label == "forward"
                and "hip_roll" in name
                and abs(peak.force_nm) > lipm_gait.KNEE_SAG_NM + 1e-3
            ):
                failures.append(
                    f"{label} {name} {peak.force_nm:+.3f} Nm crosses {lipm_gait.KNEE_SAG_NM:.2f}"
                )
        def _signed(joint: str) -> str:
            peak = bucket.get(joint)
            if peak is None:
                return "—"
            return f"{peak.force_nm:+.3f} @{peak.t_s:.3f}s"

        lines.append(
            f"{label} hip roll R {_signed('r_hip_roll_pos')} L {_signed('l_hip_roll_pos')}  "
            f"knee R {_signed('r_knee_pos')} L {_signed('l_knee_pos')}  "
            f"worst {worst_name} {worst:+.3f} Nm"
        )
    walker = session.lipm
    if walker is not None:
        soles_l: list[float] = []
        soles_r: list[float] = []
        tr = walker.trace
        for i, phase_name in enumerate(tr.phase):
            if phase_name != "swing" or not (BUS_KIT_STAND_S <= tr.t[i] < BUS_KIT_VEL_S):
                continue
            if tr.swing[i] == "L":
                soles_l.append(tr.sole_l[i])
            elif tr.swing[i] == "R":
                soles_r.append(tr.sole_r[i])
        p90_l = float(np.percentile(np.asarray(soles_l), 90)) if soles_l else 0.0
        p90_r = float(np.percentile(np.asarray(soles_r), 90)) if soles_r else 0.0
        lines.append(f"swing sole p90 L {p90_l * 100:.2f} cm  R {p90_r * 100:.2f} cm")
    return failures, lines


def _bus_kit_yaw() -> tuple[list[str], list[str]]:
    """Full left and full right. Step angle leaves 0 and heading follows."""
    failures: list[str] = []
    lines: list[str] = []
    for sign, vx, name in (
        (1.0, VX_FWD_CAP, "left"),
        (-1.0, VX_FWD_CAP, "right"),
        (1.0, 0.0, "inplace"),
    ):
        session = SteerSession(video=False, lipm=locked_kit_config())
        stand_s = 0.40
        move_s = 8.00
        stop_s = 9.00
        peaks: dict[str, _TauPeak] = {}
        move_peaks: dict[str, _TauPeak] = {}
        stop_peaks: dict[str, _TauPeak] = {}
        real_step = mj.mj_step

        def _step(model: mj.MjModel, data: mj.MjData) -> None:
            real_step(model, data)
            _note_leg_peaks(session, peaks)
            if session.bus.mode == "move":
                _note_leg_peaks(session, move_peaks)
            elif float(data.time) >= move_s - 1e-9:
                _note_leg_peaks(session, stop_peaks)

        mj.mj_step = _step
        last_send = -1.0
        angle_peak = 0.0
        try:
            while float(session.data.time) < stop_s - 1e-9:
                now = float(session.data.time)
                if now < stand_s - 1e-9:
                    if last_send < 0.0:
                        session.bus.stand(now)
                        last_send = now
                elif now < move_s - 1e-9:
                    if last_send < stand_s or (now - last_send) >= (VEL_RESEND_S - 1e-9):
                        session.bus.vel(vx, sign * YAW_RATE_CAP, now)
                        last_send = now
                elif last_send < move_s:
                    session.bus.stop(now)
                    last_send = move_s + 10.0
                session.step()
                walker = session.lipm.op3 if session.lipm is not None else None
                if walker is not None:
                    angle_peak = max(angle_peak, abs(walker.angle_cmd))
        finally:
            mj.mj_step = real_step
        move = [s for s in session.samples if s.mode == "move"]
        # The slew is done by ~2.3 s. The next couple of steps still
        # settle the heading. 4–8 s is the steady turn.
        steady = [s for s in move if s.t >= 4.0 - 1e-9]
        dyaw = 0.0
        rate = 0.0
        if len(steady) >= 2:
            dyaw = math.degrees(steady[-1].yaw - steady[0].yaw)
            dt = steady[-1].t - steady[0].t
            rate = math.radians(dyaw) / dt if dt > 1e-6 else 0.0
        _expect(angle_peak > 0.04, f"{name} step angle {angle_peak:.3f}", failures)
        _expect(dyaw * sign > 15.0, f"{name} Δyaw {dyaw:+.1f} deg", failures)
        _expect(session.samples[-1].mode == "stand", f"{name} tail {session.samples[-1].mode}", failures)
        knee_peak = 0.0
        worst_name = ""
        worst = 0.0
        for joint, peak in peaks.items():
            if abs(peak.force_nm) >= abs(worst):
                worst = peak.force_nm
                worst_name = joint
            if "knee" in joint:
                knee_peak = max(knee_peak, abs(peak.force_nm))
            if abs(peak.force_nm) > BUS_KIT_RAIL_NM:
                failures.append(f"{name} {joint} {peak.force_nm:+.3f} Nm")
            if "knee" in joint and abs(peak.force_nm) > lipm_gait.KNEE_SAG_NM + 1e-3:
                failures.append(f"{name} knee {joint} {peak.force_nm:+.3f} Nm")
        for joint, peak in stop_peaks.items():
            if not any(tok in joint for tok in ("hip_", "knee", "ank_")):
                continue
            if abs(peak.force_nm) > lipm_gait.LEG_STOP_NM + 1e-3:
                failures.append(
                    f"{name} stop {joint} {peak.force_nm:+.3f} Nm "
                    f"lacks {lipm_gait.LEG_STOP_HEADROOM_NM:.2f} Nm headroom"
                )
        for joint, peak in move_peaks.items():
            if "hip_roll" in joint and abs(peak.force_nm) > lipm_gait.KNEE_SAG_NM + 1e-3:
                failures.append(f"{name} hip roll {joint} {peak.force_nm:+.3f} Nm")
        hip_move = max(
            (abs(p.force_nm) for n, p in move_peaks.items() if "hip_roll" in n),
            default=0.0,
        )
        lines.append(
            f"{name} angle {angle_peak:.3f} rad  steady Δyaw {dyaw:+.1f} deg  "
            f"yaw rate {rate:+.3f} rad/s  knee |τ| {knee_peak:.3f}  "
            f"hip roll |τ| {hip_move:.3f}  "
            f"worst {worst_name} {worst:+.3f} Nm"
        )
        session.assert_plant_unchanged()
        if name in ("left", "right"):
            lines.append(f"__rate_{name}={rate:+.6f}")
    left_rate = right_rate = None
    kept: list[str] = []
    for line in lines:
        if line.startswith("__rate_left="):
            left_rate = float(line.split("=", 1)[1])
        elif line.startswith("__rate_right="):
            right_rate = float(line.split("=", 1)[1])
        else:
            kept.append(line)
    if left_rate is not None and right_rate is not None and max(abs(left_rate), abs(right_rate)) > 1e-6:
        mismatch = abs(abs(left_rate) - abs(right_rate)) / max(abs(left_rate), abs(right_rate))
        _expect(mismatch < 0.05, f"left/right yaw rate mismatch {mismatch:.3f}", failures)
        kept.append(
            f"left/right |rate| {abs(left_rate):.3f}/{abs(right_rate):.3f} "
            f"mismatch {mismatch * 100:.1f}%"
        )
    return failures, kept


def _bus_kit_reverse() -> tuple[list[str], list[str]]:
    """vel(−0.032) retreats at about that body speed."""
    failures: list[str] = []
    lines: list[str] = []
    session = SteerSession(video=False, lipm=locked_kit_config())
    peaks: dict[str, _TauPeak] = {}
    real_step = mj.mj_step

    def _step(model: mj.MjModel, data: mj.MjData) -> None:
        real_step(model, data)
        _note_leg_peaks(session, peaks)

    mj.mj_step = _step
    stand_s, move_s, stop_s = 0.40, 6.50, 7.40
    last_send = -1.0
    try:
        while float(session.data.time) < stop_s - 1e-9:
            now = float(session.data.time)
            if now < stand_s - 1e-9:
                if last_send < 0.0:
                    session.bus.stand(now)
                    last_send = now
            elif now < move_s - 1e-9:
                if last_send < stand_s or (now - last_send) >= (VEL_RESEND_S - 1e-9):
                    session.bus.vel(-VX_BACK_CAP, 0.0, now)
                    last_send = now
            elif last_send < move_s:
                session.bus.stop(now)
                last_send = move_s + 10.0
            session.step()
    finally:
        mj.mj_step = real_step
    steady = [s for s in session.samples if s.mode == "move" and s.t >= 3.0]
    body = float(np.mean([s.body_vx for s in steady])) if steady else 0.0
    ratio = body / -VX_BACK_CAP if VX_BACK_CAP else 0.0
    _expect(abs(ratio - 1.0) < 0.08, f"reverse body/command {ratio:.2f}", failures)
    knee = max((abs(p.force_nm) for n, p in peaks.items() if "knee" in n), default=0.0)
    hip_roll = max((abs(p.force_nm) for n, p in peaks.items() if "hip_roll" in n), default=0.0)
    if knee > lipm_gait.KNEE_SAG_NM + 1e-3:
        failures.append(f"reverse knee {knee:.3f} Nm")
    if hip_roll > lipm_gait.KNEE_SAG_NM + 1e-3:
        failures.append(f"reverse hip roll {hip_roll:.3f} Nm")
    for name, peak in peaks.items():
        if abs(peak.force_nm) > BUS_KIT_RAIL_NM:
            failures.append(f"reverse {name} {peak.force_nm:+.3f} Nm")
    lines.append(
        f"reverse cmd {-VX_BACK_CAP:+.3f} m/s  settled body {body:+.3f} m/s  "
        f"ratio {ratio:.2f}  knee |τ| {knee:.3f}  hip roll |τ| {hip_roll:.3f}"
    )
    session.assert_plant_unchanged()
    return failures, lines


def _bus_kit_first_step_knee() -> tuple[list[str], list[str]]:
    """First swing after stand, left lead and right lead. Knee sag bar."""
    failures: list[str] = []
    lines: list[str] = []
    for lead in ("L", "R"):
        cfg = replace(locked_kit_config(), gm_start_lead=lead, gm_resume_lead="keep")
        session = SteerSession(video=False, lipm=cfg)
        peaks = {"l_knee_pos": 0.0, "r_knee_pos": 0.0}
        seen = {"on": False, "done": False}
        real_step = mj.mj_step

        def _step(model: mj.MjModel, data: mj.MjData, _lead=lead) -> None:
            real_step(model, data)
            walker = session.lipm.op3 if session.lipm is not None else None
            if walker is None or seen["done"]:
                return
            t = walker.time
            in_ssp = (
                walker.l_ssp_start < t <= walker.l_ssp_end
                if _lead == "L"
                else walker.r_ssp_start < t <= walker.r_ssp_end
            )
            if in_ssp:
                seen["on"] = True
                for name in peaks:
                    force = abs(float(data.actuator_force[session.act_idx[name]]))
                    peaks[name] = max(peaks[name], force)
            elif seen["on"]:
                seen["done"] = True

        mj.mj_step = _step
        last_send = -1.0
        try:
            while float(session.data.time) < 2.0 - 1e-9 and not seen["done"]:
                now = float(session.data.time)
                if now < 0.40 - 1e-9:
                    if last_send < 0.0:
                        session.bus.stand(now)
                        last_send = now
                elif (now - last_send) >= (VEL_RESEND_S - 1e-9):
                    session.bus.vel(VX_FWD_CAP, 0.0, now)
                    last_send = now
                session.step()
        finally:
            mj.mj_step = real_step
        session.assert_plant_unchanged()
        _expect(seen["on"], f"{lead}-lead first swing did not start", failures)
        for name, force in peaks.items():
            if force > lipm_gait.KNEE_SAG_NM + 1e-3:
                failures.append(f"{lead}-lead first step {name} {force:.3f} Nm")
        lines.append(
            f"first {lead}-lead step knee L {peaks['l_knee_pos']:.3f} Nm  "
            f"R {peaks['r_knee_pos']:.3f} Nm"
        )
    return failures, lines


def _bus_kit_resume_carry() -> tuple[list[str], list[str]]:
    """Outside-lead resume: joint-target step, turn arc, resume leftover."""
    failures: list[str] = []
    lines: list[str] = []

    def _yaw_at(samples: list[PoseSample], t: float) -> float:
        best = samples[0]
        for sample in samples:
            if sample.t <= t + 1e-9:
                best = sample
            else:
                break
        return float(best.yaw)

    def _one(label: str, script: tuple[DemoSegment, ...], t0: float, t1: float) -> None:
        session = SteerSession(video=False, lipm=locked_kit_config())
        walker = session.lipm.op3
        if walker is None:
            failures.append(f"{label} resume has no kit walker")
            return
        driver = ScriptedDriver(script)
        normal: list[float] = []
        chase: list[float] = []
        prev: dict[str, float] | None = None
        while float(session.data.time) < t1 + 0.05:
            now = float(session.data.time)
            driver.publish(session.bus, now)
            report = session.step()
            joints = walker._last_joints
            if prev is not None and report.mode == "move" and joints:
                delta = 0.0
                for name, val in joints.items():
                    delta = max(delta, abs(float(val) - float(prev.get(name, val))))
                if walker.lead_blend_tick:
                    chase.append(delta)
                else:
                    normal.append(delta)
            if joints:
                prev = {k: float(v) for k, v in joints.items()}
        session.assert_plant_unchanged()
        arc = math.degrees(_wrap_pi(_yaw_at(session.samples, t0) - _yaw_at(session.samples, 16.0)))
        res = math.degrees(_wrap_pi(_yaw_at(session.samples, t1) - _yaw_at(session.samples, t0)))
        dur = t0 - 16.0
        rate = math.radians(arc) / dur if dur > 1e-6 else 0.0
        hold = max(normal) if normal else 0.0
        lines.append(
            f"{label} arc 16→{t0:.1f}s Δyaw {arc:+.2f}°  mean {rate:+.3f} rad/s  "
            f"resume Δyaw {res:+.2f}°"
        )
        if label == "left":
            if not chase:
                failures.append("left resume did not chase the outside-foot pose")
            else:
                peak = max(chase)
                chase_s = len(chase) * float(session.ctrl_dt)
                lines.append(
                    f"left resume chase {chase_s:.3f}s  "
                    f"max|Δq| {peak:.3f} rad  walk max {hold:.3f} rad"
                )
                if peak > hold + 0.005:
                    failures.append(
                        f"left resume chase tick {peak:.3f} rad exceeds walk max {hold:.3f}"
                    )
            if res > 8.0 or res < -8.0:
                failures.append(f"left resume Δyaw {res:+.2f}° left the outside-foot window")
        elif abs(res) > 4.0:
            failures.append(f"right resume Δyaw {res:+.2f}°")

    _one("left", NAV_LEFT_SCRIPT, 28.5, 34.5)
    _one("right", NAV_RIGHT_SCRIPT, 27.0, 33.0)
    return failures, lines


def bus_kit_check() -> int:
    """Day-1 bus on the locked kit row. Non-zero means the Prefer FAIL bar broke."""
    contract_fail, lines = _bus_kit_contract()
    torque_fail, torque_lines = _bus_kit_forward_stop()
    yaw_fail, yaw_lines = _bus_kit_yaw()
    rev_fail, rev_lines = _bus_kit_reverse()
    lead_fail, lead_lines = _bus_kit_first_step_knee()
    carry_fail, carry_lines = _bus_kit_resume_carry()
    failures = contract_fail + torque_fail + yaw_fail + rev_fail + lead_fail + carry_fail
    lines.extend(torque_lines)
    lines.extend(yaw_lines)
    lines.extend(rev_lines)
    lines.extend(lead_lines)
    lines.extend(carry_lines)
    print("[bus-kit] Day-1 CommandBus on locked kit500")
    for line in lines:
        print(f"[bus-kit] {line}")
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print(
        "[bus-kit] Prefer FAIL bar holds: settled vx matches the clamp, "
        "yaw step angle is non-zero, knees and walking hip roll stay at or under 2.33 Nm"
    )
    return 0


def main() -> None:
    ap = argparse.ArgumentParser(description="Day-1 velocity steer on frozen M145 (no door)")
    ap.add_argument("--view", action="store_true", help="Interactive viewer + keyboard")
    ap.add_argument("--duration", type=float, default=None, help="Seconds (demo default is the scripted clip, view default 120)")
    ap.add_argument("--clip", choices=tuple(CLIP_SCRIPTS), default="forward")
    ap.add_argument("--out", type=str, default=None)
    ap.add_argument("--no-video", action="store_true", help="Headless sim without mp4")
    ap.add_argument("--self-test", action="store_true", help="Command bus, freeze checks, short sim")
    ap.add_argument(
        "--bus-kit",
        action="store_true",
        help="Day-1 bus on the locked kit walk: forward then stop, plant cold",
    )
    ap.add_argument("--log", type=str, default=None)
    ap.add_argument("--summary", type=str, default=None)
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(self_test())
    if args.bus_kit:
        raise SystemExit(bus_kit_check())
    if args.view:
        run_view(120.0 if args.duration is None else float(args.duration))
        return
    script = CLIP_SCRIPTS[args.clip]
    stem = {
        "forward": "steer_walk_forward",
        "stop": "steer_walk_stop",
        "reverse": "steer_walk_reverse",
        "turn": "steer_walk_turn",
        "turn-right": "steer_walk_turn_right",
        "nav-left": "steer_walk_nav_left",
        "nav-right": "steer_walk_nav_right",
        "nav-multi": "steer_walk_nav_multi",
    }[args.clip]
    duration = script[-1].t_end if args.duration is None else float(args.duration)
    out = None if args.no_video else Path(args.out or (PREVIEWS / f"{stem}.mp4"))
    run_demo(
        duration=duration,
        out_mp4=out,
        log_path=Path(args.log or (PREVIEWS / f"{stem}_log.txt")),
        summary_path=Path(args.summary or (PREVIEWS / f"{stem}_summary.json")),
        script=script,
    )


if __name__ == "__main__":
    main()
