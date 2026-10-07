#!/usr/bin/env python3
"""Five-room yes/no search on the joined apartment. Soft-pass is off.

Reach is fixed below before any room is scored. A bout reaches only when
the settled stand COM starts outside that room's floor box in
mujoco/room_apartment.json and finishes inside the same box, upright,
with zero prop contacts, at or before the time limit, and only after a
correct per-target yes committed vel(+0.056, 0) after an in-place re-point. The box is the named
room, not the plant floor from -3 m to +3 m.

Scored spawns are the doorway xy turned ±90° from the door-facing yaw.
A geometric census logs the kit_cam doorway fraction. ±90° is the offset
inside the ±60–90° band where that fraction is ~0. kit_cam is not moved.

The question is "Is there a {room} through the doorway ahead?" yes or no.
It is not an open-set room label. Before that question, Moondream detects
a doorway. No doorway means keep searching. Search turns in place at
+0.25 rad/s with no forward velocity. Before each picture the turn stops
and the body settles, then kit_cam is captured. The answer is logged at
that heading, not at reply time. A Moondream yes while the asked room is
under 1% of the frame is refused: it is logged against that gate and
treated as no, so the search continues. A yes under 1% that still
commits forward velocity is a wrong yes. A doorway that is not actually
in the frame does not get a room question: the body turns in place at
yaw +0.25. A correct yes re-points in place toward the doorway pixel,
settles until that yaw is back at 0, then walks vel(+0.056, 0). Yaw is
not applied while walking. In-place yaw that asks a hip roll over
2.33 Nm unclamped is not a free re-point. The #71 latch stays armed on
that forward motion. A colour-free floor/wall row is ranged with the
live camera, and the stop latches only when the forward ray hits a wall.
Every real-wall ray logs the ranged gap, the sim gap, wall id,
heading, camera height, camera pitch, and the leading toe. The
0.04087 m median is the pad for wall_hall_w_2 only, and only when
the sim toe gap is under 0.40 m — the class of the 17 living
samples. wall_hall_e_1 has its own median from 43 close samples on
tip aac2baa, held out of this run. The living number is not copied
onto the east wall. A right-toe stop on that wall cannot clear
and does not end the walk. Once the left toe's forward ray on
wall_hall_e_1 is at or under 0.60 m, further in-place re-points
are dropped so the yaw-0 walk can enter the close class. No
pitch, camera-height, or toe bias is fit across the pooled
samples. Other walls and a same-named wall seen from
farther away do not get a pad. The 0.40 m bound is 2.39 times the
living latch distance. A prop, a different wall, or that far class
is a reject and is not soaked into the latch. A stop that does
commit has to land within 1 cm of its own pad or the wall stop
fails. Room reach is a separate bar. A wall or prop contact with
the latch armed fails the
bout. A stop on the speckled hall shadow by the kitchen or bathroom
doorway, with no prop in the frame, is a false stop and is not a #71 pass.

0.150 is only the d_min bound. The plant file is only hashed.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import statistics
import sys
from pathlib import Path
from typing import TypedDict

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import check_apartment_scene as apt
import hazard_finder as hf
import lipm_gait
import mono_toe_gate as gate
import ray_corridor as rc
import room_ask
import steer_walk as sw
import voice_caller as voice

ROOT = Path(__file__).resolve().parents[1]
SUMMARY_PATH = ROOT / "previews" / "voice_goto_rooms_summary.json"
SCENE_XML = ROOT / "mujoco" / "room_apartment.xml"
TIME_LIMIT_S = 53.0
STAND_S = 1.0
UPRIGHT_UP_Z = 0.90
PROP_FORCE_N = 0.5
T_DETECT_S = 0.0
# Hall floor in front of the east openings. The light shines toward -X,
# so the wall shadow lands on this checker, outside the room box.
SHADOW_X = (0.45, 1.06)
SPECKLE_STD_MIN = 8.0
SPECKLE_MEAN_MAX = 90.0
SPECKLE_P10_MAX = 55.0
SPECKLE_SPAN_MIN = 18.0
PROP_HIT_M = 0.25
PHRASES: tuple[tuple[str, str], ...] = (
    ("kitchen", "go to the kitchen"),
    ("bathroom", "go to the bathroom"),
    ("living", "go to the living room"),
    ("bedroom", "go to the bedroom"),
    ("entrance", "go to the entrance"),
)
# ±90° is inside the requested ±60–90° band. The census must show the
# doorway fraction near 0 there. ±60° and ±75° are logged and not scored,
# because a 120° kit_cam still sees the opening at those headings.
SPAWN_OFFSETS_RAD: tuple[float, ...] = (0.5 * math.pi, -0.5 * math.pi)
CENSUS_OFFSETS_DEG: tuple[int, ...] = (0, 60, -60, 75, -75, 90, -90)
SEARCH_YAW = voice.YAW_RAD_S
SEARCH_SWEEP_RAD = 2.0 * math.pi
ASK_STEP_RAD = math.radians(20.0)
# Stop, then this long at yaw 0, before the picture. 0.25 rad/s at the
# 0.40 rad/s² yaw slew takes 0.625 s to reach 0. 0.70 s covers that.
# Frozen before the bouts. Not a reach bar.
SETTLE_S = 0.70
ROOM_VISIBLE_FRAC = 0.01
HIP_BAR_NM = 2.33
# After a correct yes, walk this long, then stop, settle, and re-point.
# Frozen before the bouts. Not a reach bar.
APPROACH_LOOK_S = 1.0
# Doorway pixel inside this band of the frame centre commands yaw 0.
# Outside it, yaw is the full ±0.25 cap toward that pixel.
DOOR_DEAD_FRAC = 0.05
# Floor-edge wall range is sampled on this period during the walk.
# 0.056 m/s moves about 4.5 mm in this window.
WALL_SAMPLE_S = 0.08
# About 1 cm. Frozen before the bouts. d_min is 0.1263 m, so a few
# centimetres is a large slice of the bar. A floor-edge stop whose
# geometric range (forward minus step-off, before the hazard pad)
# misses the sim toe gap by more than this is not a wall-stop pass
# unless that measured error was added to the latch distance before
# the bouts. Not a clearance buffer on top of a standing-pose table.
RANGE_ERR_MAX_M = 0.01
# The ~4 cm wall-ray class. Used only to count repeats in the log.
WALL_BIAS_M = 0.03
# Living −90° wall_hall_w_2 on tip 66ab006. Sim gap under 0.40 m.
# Seventeen samples, each with cam_z, camera pitch, and leading toe,
# errors 0.0301–0.0543 m. The latch extra is the median of this set,
# not the peak and not the single 0.0409 m latch row. These samples
# already include the live walk bob. The settled-stand wall error is
# 0.18 mm, so the old 0.0101 m bob pad is not added on top.
SAME_WALL_ERR_M: tuple[float, ...] = (
    0.030126463895185362,
    0.030317832469307193,
    0.03202051468666248,
    0.0367884720606339,
    0.037616906496476366,
    0.038381535901046204,
    0.039638208906405126,
    0.04006368156714865,
    0.04086979572280322,
    0.047996553365196065,
    0.04936891275435906,
    0.049795950246224074,
    0.05000671234013854,
    0.051110526369427256,
    0.05353726916577645,
    0.05414385561253576,
    0.05433211195858684,
)
LATCH_EXTRA_M = float(statistics.median(SAME_WALL_ERR_M))
# Recorded 17-sample median. On tip f9d059d the right-toe living stop
# missed it by +0.0133 m. It is not the live right-toe pad, it is not
# the left-toe pad, and it does not travel to wall_hall_e_1.
LIVING_WALL = "wall_hall_w_2"
# Walk-up band the living 17 were collected in. The living latch
# distance is d_min 0.1263 m + 0.04087 m = 0.1672 m. 0.40 m is 2.39
# times that distance, so it is the approach in front of the latch.
# A ray past it stays a far reject. It is not a second clearance pad.
CLOSE_GAP_M = 0.40
# Tip 79edde5 kitchen −90°: the left-toe forward ray on wall_hall_e_1
# bottomed at 0.506 m, then the next 20° doorway re-point aimed the
# walk at the cabinet. Once that left-toe ray is at or under 0.60 m,
# further in-place re-points are dropped and the walk stays at yaw 0
# so the ray can enter the close class. Not a pad. A right toe does
# not arm it and does not stop the bout.
EAST_L_HOLD_M = 0.60
# Kitchen −90° wall_hall_e_1 on tip aac2baa. Same rule as the living
# set: same wall id, sim ray at or under 0.40 m. Forty-three samples,
# each with heading, cam_z, camera pitch, and leading-toe offset.
# Fields are err_m, heading_rad, cam_z_m, cam_pitch_rad, lead_off_m,
# ray_m. The pad is the median of the errors, not the 0.203 m stop
# and not the living 0.04087 m. This run's stop is held out of the set.
EAST_WALL = "wall_hall_e_1"
EAST_CLOSE: tuple[tuple[float, float, float, float, float, float], ...] = (
    (0.018459365711454845, -0.4606900838575969, 0.33718933789269756, -0.2671429837835369, -0.016491813459113984, 0.33490592913320616),
    (0.019870525838155018, -0.46195893615946887, 0.33744800112190904, -0.26402938625225036, -0.012644825991786368, 0.3299715110478957),
    (0.01124062789056579, -0.43427071535135814, 0.3301707897045914, -0.25295381942490985, -0.010123955800654222, 0.315424532378546),
    (0.022371324879031707, -0.4761343165000211, 0.34063553729231116, -0.25905005299972245, -0.010977319959444825, 0.312038340787725),
    (0.014172653072355712, -0.4612933593826507, 0.33717973854714234, -0.2671923372683821, -0.01651888607747099, 0.30487624685085946),
    (0.018998198212342, -0.46250215271906087, 0.3374256155723085, -0.2641164023287797, -0.012685444580628563, 0.2999100473193094),
    (0.006096072416868825, -0.4348887612765236, 0.330160175955297, -0.2529932460953037, -0.010148613032648988, 0.28577148663538565),
    (0.023313578415210978, -0.4768345281773284, 0.34065095873573525, -0.25885090649992015, -0.010919778644733808, 0.2818080682094471),
    (0.035660759960736144, -0.746090386572843, 0.33485648896777087, -0.2618130230308264, -0.01704636387668256, 0.27425972120831166),
    (0.036071440108821906, -0.7314091599820147, 0.3317063632666801, -0.25966896322938354, -0.01561049322581752, 0.27900257721470023),
    (0.050568529557177, -0.7263628537175592, 0.3380462053184142, -0.26627850535952474, -0.016255039309352903, 0.2922150871323482),
    (0.04503687948824822, -0.7283522442006857, 0.3352973155697339, -0.2581840727488311, -0.01243995440282667, 0.2821163716984157),
    (0.027595774569479314, -0.7589572132972796, 0.33010918608334866, -0.25408778341169586, -0.014156752315027728, 0.26509308151516964),
    (0.1506722347683461, -0.7756526546352759, 0.33717640441487934, -0.2672920486363157, -0.01655566964270053, 0.38381729649228774),
    (0.14898969455207245, -0.7768477128544267, 0.33743866355903174, -0.2640961546692104, -0.012668132996911288, 0.3793135738974913),
    (0.12738228017970862, -0.749222269764188, 0.3301768335657582, -0.25294125245734234, -0.010121967105166978, 0.3549950585691097),
    (0.04497046451139322, -0.7208985450729345, 0.3353585763229499, -0.26659465443992963, -0.014400297242675426, 0.2774128465796174),
    (0.05734815276887134, -0.7114021646992944, 0.3405789223798939, -0.2527152404446212, -0.007520735924884345, 0.2787574278349222),
    (0.03355800739425943, -0.7316803922536118, 0.3331088490152483, -0.24951393080036327, -0.0038629449826222914, 0.24812503447279355),
    (0.04858847054956744, -0.7732600808980788, 0.33164158284879064, -0.2730123326318353, -0.014122202117667402, 0.26037443312605485),
    (0.15977280287477152, -0.7911252235176623, 0.3406368352762272, -0.2590431501924645, -0.010973367416138751, 0.36602840881824605),
    (0.031361574454811886, -0.7471441763591538, 0.33485806509748006, -0.26173796265356525, -0.017005496145546715, 0.2444756223651281),
    (0.03983016052991295, -0.7324751512263892, 0.331677898327193, -0.2599949848092821, -0.015682675631513647, 0.24870065373761152),
    (0.048247989518562545, -0.727540082962291, 0.33801794502803356, -0.2665183138056701, -0.01633966734784161, 0.2616005499836222),
    (0.04104436193218278, -0.7295011591256627, 0.3352888602868898, -0.2581814447167103, -0.012449656336555948, 0.2515629819985977),
    (0.024918157535977842, -0.7600336858703329, 0.3301210064107261, -0.25397807379702353, -0.014127734130392429, 0.23558779659430756),
    (0.14646226056805706, -0.7766557509878589, 0.33716651931956176, -0.2673418659102439, -0.01658329482280911, 0.35357006607699026),
    (0.14554391353400592, -0.7777912036774642, 0.337415629577165, -0.26417923411511873, -0.012708381038177615, 0.34901329360502337),
    (0.12376247265784124, -0.7501185621894505, 0.3301648022024256, -0.2530107679385132, -0.010155790434849762, 0.3254086667817543),
    (0.05000255522016028, -0.7218536259628895, 0.3353584573496013, -0.2666383131569396, -0.014411483025159653, 0.24844003513458646),
    (0.05210515775543695, -0.7124457311990573, 0.34057389237431396, -0.25271394817137677, -0.007524459619047157, 0.24762418167164896),
    (0.03770138160585387, -0.7328023388525927, 0.3330854097124395, -0.24948714338283537, -0.0038667944095315708, 0.21772686074825495),
    (0.04600715811464742, -0.7743908235512676, 0.33166836539049277, -0.272962779562084, -0.014093136370115429, 0.22998706299486588),
    (0.1581449343160692, -0.7921294719740872, 0.3406569881286009, -0.2588176658778602, -0.01090441404853212, 0.33529851229523233),
    (0.029462726524616734, -1.061699202041039, 0.3348576673122178, -0.2618052220409971, -0.01704574344505056, 0.22520206093525463),
    (0.01683454355199429, -1.0470086919346588, 0.3317061854364401, -0.25967226602422905, -0.01561589177576248, 0.21938652187847438),
    (0.01029539713228686, -1.0419269940043625, 0.3380437310497457, -0.26631909655641717, -0.016272608374541156, 0.20077107470372366),
    (0.001156175575773244, -1.0439559006840442, 0.33529651083188644, -0.25819162951502245, -0.012442013996414628, 0.20710294518197478),
    (0.046379076628352306, -1.0745597017304598, 0.3301082980725061, -0.254089008125809, -0.014157488731327866, 0.2292230478241917),
    (0.15784185620779986, -1.0364333994420951, 0.3353572412084168, -0.26663515116474923, -0.014401271388128993, 0.36481984376591),
    (0.02623597153238752, -1.0269963306750818, 0.34057970020559336, -0.25270331320446876, -0.007516456413613475, 0.1795644881552401),
    (0.003559847832219243, -1.0472716893339638, 0.3331087348572291, -0.2495060459108312, -0.003860749523748706, 0.19418107280454722),
    (0.20341424894951343, -1.0887848889119358, 0.33164235823040283, -0.2729989872932855, -0.014120531915378008, 0.3550626326053704),
)
EAST_LATCH_M = float(statistics.median(row[0] for row in EAST_CLOSE))
# Left-toe close rays on wall_hall_e_1 only. The first 21 are tip
# a3c05f3, with that bout's committing stop (error 0.108 m) left out.
# The next 24 are tip 80bf2ea after the yaw-0 hold, with this bout's
# committing stop (error 0.0594 m) left out. Four samples are the same
# moment on both tips and are stored once. Right-toe samples are not
# in this list. Living wall_hall_w_2 samples are not in this list.
# Contact-box centres sit 14 mm outboard of each ankle-roll axis, so
# the ray origin flips side with the lead foot. Right has 9 samples,
# under the living set of 17, so a right stop cannot CLEAR.
TOE_OUTBOARD_M = 0.014
EAST_L_ERR_M: tuple[float, ...] = (
    0.03321512987238068,
    0.047260122455027864,
    0.06788215840253625,
    0.029441220869328988,
    0.022066927040247264,
    0.013459704433692599,
    0.01303616414467329,
    0.006908804449123562,
    0.025126806188412287,
    0.014262776219442996,
    0.003870003392395016,
    0.06758082627981898,
    0.05601018770872196,
    0.024964942802465057,
    0.01272136391048459,
    0.006389876300130076,
    0.001949075568160652,
    0.003978962925460738,
    0.02472477703146328,
    0.012370575448907656,
    0.004975797972947116,
    0.05902254006728702,
    0.04600440059019678,
    0.06506832389495176,
    0.0269606371435121,
    0.05777895156374646,
    0.0655960982836461,
    0.045424805087740217,
    0.06503350787032264,
    0.026481289163201083,
    0.054480310039754676,
    0.06926734355885605,
    0.048015141765005265,
    0.06443281216068128,
    0.030872335419293323,
    0.05944376400913556,
    0.06552327062202867,
    0.04834079119885454,
    0.06162423618398957,
    0.027692684605144735,
    0.056831318683336074,
    0.0676083041137498,
    0.04627259878243614,
    0.06307554547089292,
    0.02532959742724461,
)
EAST_R_ERR_M: tuple[float, ...] = (
    0.02727480416032496,
    0.0261903108125065,
    0.07279199541829862,
    0.07634195712865749,
    0.06134884659385631,
    0.02353490205158887,
    0.08114385348433867,
    0.17968368595785733,
    0.0864448300362029,
)
EAST_L_LATCH_M = float(statistics.median(EAST_L_ERR_M))
EAST_R_LATCH_M = float(statistics.median(EAST_R_ERR_M))
# Recorded median + MAD of the same 45. A live median stop left
# +0.0222 m. A live median+MAD stop left −0.0034 m on the kitchen
# and −0.0403 m on a left-toe living stop. That sum is not the live
# latch, and it is not applied to wall_hall_w_2.
EAST_L_MAD_M = float(statistics.median(abs(x - EAST_L_LATCH_M) for x in EAST_L_ERR_M))
EAST_L_APPLIED_M = EAST_L_LATCH_M + EAST_L_MAD_M
EAST_L_MAD_APPLIED = False


def _sorted_quantile(values: tuple[float, ...], q: float) -> float:
    ordered = sorted(values)
    pos = q * (len(ordered) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(ordered) - 1)
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


# 75th percentile of the 45. Both old committing stops are already
# out. This is the one precommitted fallback if the bout median misses.
# It is not fitted on a new stop, and it is not applied to wall_hall_w_2.
EAST_L_Q75_M = _sorted_quantile(EAST_L_ERR_M, 0.75)
# "online": median of prior close left errors on wall_hall_e_1 in this
# bout. The sample that crosses the gate is left out. That rule was
# run live on kitchen −90°: 22 priors, pad 0.05613 m, residual
# −0.0284 m. It is not the latch. "quantile" is the 75th percentile
# of the 45. Empty priors do not fall back to the median or to
# median+MAD.
EAST_L_ONLINE_PAD_M = 0.05612963080175057
EAST_L_ONLINE_N = 22
EAST_L_ONLINE_RESIDUAL_M = -0.028436946196605833
# Live kitchen −90° on the 75th percentile. Same pose as the online
# stop: ray 0.234 m, left toe, error 0.02769 m, residual −0.0318 m.
# Not a clear. Not applied to wall_hall_w_2. The approach-wide median
# and this percentile both sit near 0.056–0.059 m, above that error.
# "near" kept only a new closest left ray. On tip 05858b7 that was
# N=6, pad 0.02857 m, and the stop error was 0.06308 m, residual
# +0.0345 m. That rule is off. "near_k" is the median of the last
# K left samples whose ray is at or under the near gate. The gate
# is tighter than the 0.40 m close class. The commit is left out.
# It is not applied to wall_hall_w_2.
EAST_L_NEWMIN_PAD_M = 0.02856695273723686
EAST_L_NEWMIN_N = 6
EAST_L_NEWMIN_RESIDUAL_M = 0.03450859273365606
EAST_L_NEAR_GATE_M = 0.30
EAST_L_NEAR_K = 5
EAST_L_LIVE = "near_k"
# Bedroom −90° on tip 51ae123. wall_hall_e_0 right toe, last 5 of
# ray ≤ 0.30 m, pad 0.04391 m, stop error 0.02519 m, residual
# −0.0187 m. The median sat on the high phase and the gate crossed
# on the low phase. Not a clear. Not applied to any other wall.
E0_WALL = "wall_hall_e_0"
E0_R_FAIL_PAD_M = 0.0439108831389442
E0_R_FAIL_N = 5
E0_R_FAIL_RESIDUAL_M = -0.018717300570202544
E0_R_FAIL_ERR_M = 0.025193582568741657
# Tip 2f21a39. Same wall, right toe, last 3 of ray ≤ 0.25 m.
# The window had 2 samples when the gate crossed. Pad 0.02934 m,
# error 0.05315 m, residual +0.0238 m. Undersampled. Not the latch.
E0_R_THIN_PAD_M = 0.029336655418528076
E0_R_THIN_N = 2
E0_R_THIN_RESIDUAL_M = 0.023811270186193986
E0_R_THIN_ERR_M = 0.05314792560472206
# Tip 7bb7d31 filled that same last-5 window and took its full
# median. The stop reproduced the 0.04391 m pad and the −0.0187 m
# residual. That full-window median is not the latch.
# Live rule on this wall×toe only: fill the same window (ray ≤ 0.30 m,
# last 5, commit excluded). The pad stays undefined until 5 samples
# are in it. The pad is then the median of the 3 closest-ray samples
# in that window, not the median of all 5. Left samples are not in it.
E0_R_NEAR_GATE_M = EAST_L_NEAR_GATE_M
E0_R_NEAR_K = EAST_L_NEAR_K
E0_R_NEAR_MIN_N = EAST_L_NEAR_K
E0_R_CLOSE_N = 3
E0_R_LIVE = "e0_r_close3"
EAST_L_Q75_RESIDUAL_M = -0.03175107940399083
EAST_L_Q75_RAY_M = 0.23443045430153236
EAST_L_Q75_ERR_M = 0.02769268460514473
# Close left-toe errors on wall_hall_w_2 only. Committing stops are
# out. Right-toe samples are not in this list. East samples are not
# in this list. N=19, so this pad can CLEAR. The 0.105 m outlier stays.
W2_L_ERR_M: tuple[float, ...] = (
    0.020614517081358474,
    0.025147450046003716,
    0.02513908648854296,
    0.02716756771739376,
    0.024734052969513476,
    0.006153987390156268,
    0.0014678681927092763,
    0.007872611718017952,
    0.0014164081123713423,
    0.011606085373040753,
    0.0015006221262908181,
    0.015028955252604836,
    0.010625936072531067,
    0.008685153028399645,
    0.0018331231952742333,
    0.021709915958411347,
    0.00766451714241273,
    0.0006299854612947797,
    0.10549564179736243,
)
W2_L_LATCH_M = float(statistics.median(W2_L_ERR_M))
# Close right-toe errors on wall_hall_w_2 only. The two committing
# stops are out: the old clear at 0.04006 m and tip f9d059d at
# 0.05421 m. Left samples are not in this list. East samples are not
# in this list. The median is fuller than the recorded 0.04087 m.
W2_R_ERR_M: tuple[float, ...] = (
    0.004219883565836802,
    0.005822787015680897,
    0.0068525014782475635,
    0.0077340341254527045,
    0.010055902446165055,
    0.010319170004230827,
    0.011862733775805379,
    0.026452013695535348,
    0.02829813952413429,
    0.029279842985892757,
    0.030317832469307193,
    0.03646035483638643,
    0.0367884720606339,
    0.03710310449407078,
    0.037616906496476366,
    0.03813136897919778,
    0.038381535901046204,
    0.03871125022303812,
    0.038878755780254404,
    0.0393584045427984,
    0.03965475705095642,
    0.04638235624866521,
    0.048008362228520285,
    0.04936476497984843,
    0.04936891275435906,
    0.04943881206199996,
    0.04969792075599555,
    0.049729465533891015,
    0.04973527061689914,
    0.049795950246224074,
    0.04994283523521065,
    0.05000671234013854,
    0.05039772349096583,
    0.05052197274527637,
    0.05078093565394698,
    0.051110526369427256,
    0.0513451339470794,
    0.05340614014471562,
    0.05353726916577645,
    0.05414385561253576,
    0.05433211195858684,
    0.055055802662696524,
    0.0578047110505272,
    0.05842412481441661,
    0.061524229425781674,
    0.06245805249739411,
    0.09010144240375012,
    0.09107983338619013,
    0.18437060817683054,
)
W2_R_LATCH_M = float(statistics.median(W2_R_ERR_M))
W2_R_HELD_ERR_M = (
    0.04006368156714865,
    0.05420673512207522,
)
# Left-only additive correction, fitted on the 45 samples above.
# predicted = c0 + c_z*cam_z + c_pitch*cam_pitch + c_lead*lead_off.
# The kitchen stop on tip 80bf2ea is not in the fit. The prediction
# misses that error by 0.0252 m, so the correction is not the pad.
EAST_L_FIT_APPLIED = False
EAST_L_FIT_C0 = 1.1032860951383978
EAST_L_FIT_CZ = -2.486733769522458
EAST_L_FIT_CPITCH = 0.939171019133529
EAST_L_FIT_CLEAD = -0.5589984770891748
EAST_L_HELD_ERR_M = 0.0594168890832652
EAST_L_HELD_CAM_Z_M = 0.33485694264068067
EAST_L_HELD_PITCH_RAD = -0.2617542762292832
EAST_L_HELD_LEAD_M = -0.017012214281004516
# wall_hall_e_0 had five close samples on that tip, under the living
# set of 17, so its frozen pad stays 0. A bout that walks its own
# toe into the 0.30 m band can use the same last-5 median. It does
# not copy an east or living pad.
DOOR_Z = (0.05, 1.35)
RAY_STEP = 8
ROOM_GEOMS: dict[str, tuple[str, ...]] = {
    "kitchen": ("kitchen_", "table_", "chair_"),
    "bathroom": ("bathroom_", "sink_", "toilet_"),
    "living": ("living_", "tv_"),
    "bedroom": ("bedroom_", "nightstand_"),
    "entrance": ("entrance_", "mat_"),
}
HIP_ACTUATORS: tuple[tuple[str, str], ...] = (
    ("l_hip_yaw", "l_hip_yaw_pos"),
    ("r_hip_yaw", "r_hip_yaw_pos"),
    ("l_hip_roll", "l_hip_roll_pos"),
    ("r_hip_roll", "r_hip_roll_pos"),
)


class FloorBox(TypedDict):
    source: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


class SpawnJson(TypedDict):
    doorway_x: float
    doorway_y: float
    doorway_yaw: float
    qpos_xyz: list[float]
    yaw_rad: float
    com_xyz: list[float]
    up_z: float


class ShadowSighting(TypedDict):
    patch: str
    t: float
    u: float
    v: float
    lum_mean: float
    lum_std: float
    speckled: bool
    finder_emitted: bool
    prop_in_frame: bool
    hit_xy: list[float] | None


class AskJson(TypedDict):
    t: float
    t_capture: float
    tick: int
    heading_rad: float
    heading_at_capture: float
    applied_yaw_at_capture: float
    yaw_from_spawn_rad: float
    doorway_fraction: float
    asked_fraction: float
    visible_rooms: list[str]
    answer: str
    raw: str
    confidence: float | None
    seconds: float | None
    door_detected: bool
    door_u: float | None
    door_v: float | None
    door_frac: float | None
    door_bbox: list[float] | None
    door_seconds: float | None
    search_applied_yaw: float
    moondream_yes: bool
    frac_gate: str


class ApproachJson(TypedDict):
    t: float
    tick: int
    heading_rad: float
    commanded_vx: float
    commanded_yaw: float
    door_u: float | None
    ranged_toe_gap_m: float | None
    true_wall_gap_m: float | None
    true_wall: str
    in_corridor: bool
    cam_z_m: float
    imu_pitch_rad: float
    head_tilt_rad: float
    cam_pitch_rad: float
    geometric_m: float | None
    lead_off_m: float
    lead_side: str
    range_err_m: float | None
    residual_m: float | None


class FinderCueJson(TypedDict):
    u: float | None
    v: float | None
    too_close: bool
    bottom_clipped: bool
    span_px: int
    width_px: int
    source: str
    column: int


class WallHitJson(TypedDict):
    t: float
    contact: str
    ranged_toe_gap_m: float | None
    true_wall_gap_m: float | None
    true_wall: str
    range_err_m: float | None
    range_over_1cm: bool
    cam_z_m: float | None
    imu_pitch_rad: float | None
    head_tilt_rad: float | None
    cam_pitch_rad: float | None
    lead_off_m: float | None
    lead_side: str
    residual_m: float | None
    ranged_wall: str
    latch_path: str
    heading_rad: float
    geom_class: str
    finder: list[FinderCueJson]


class TorqueJson(TypedDict):
    joint: str
    unclamped_nm: float
    clamped_nm: float
    measured_nm: float
    approach_unclamped_nm: float | None
    approach_clamped_nm: float | None


class HipJson(TypedDict):
    joint: str
    actuator: str
    peak_nm: float
    abs_nm: float
    over_2_33: bool


class InplaceHipJson(TypedDict):
    joint: str
    unclamped_nm: float
    clamped_nm: float


class EastLeftFact(TypedDict):
    hold: bool
    presented: bool
    min_ray_m: float | None
    t: float | None
    heading_rad: float | None
    cam_z_m: float | None
    cam_pitch_rad: float | None
    lead_off_m: float | None
    lead_side: str
    online_n: int
    pad_rule: str


class ApproachBand(TypedDict):
    """Closest forward wall ray on this bout, and whether the walk held."""
    hold: bool
    hold_m: float
    closest_ray_m: float | None
    closest_wall: str
    closest_toe: str
    closest_t: float | None
    closest_heading_rad: float | None
    entered_0_30: bool


class RoomJson(TypedDict):
    room: str
    scene: str
    phrase: str
    question: str
    yaw_offset_rad: float
    voice_parse: str
    voice_line: str
    generic_caller: str
    matched_room_words: list[str]
    recogniser_label: str
    recogniser_raw: str
    recogniser_confidence: float | None
    recogniser_seconds: float | None
    commanded_vx: float | None
    commanded_yaw: float | None
    search_yaw_rate: float
    yaw_turned_rad: float
    first_yes_t: float | None
    first_yes_tick: int | None
    first_yes_heading_rad: float | None
    wrong_yes: bool
    committed: bool
    d_min_m: float
    d_min_v_mps: float
    t_detect_s: float
    t_stop_s: float
    latch_applied: bool
    latch_pass: bool
    false_stop: bool
    prop_in_frame_at_stop: bool | None
    stop_path: str
    stop_reason: str
    stop_hit_xy: list[float] | None
    stop_on_shadow: bool
    shadow_first_sightings: list[ShadowSighting]
    doorway_fraction: float
    asked_fraction: float
    visible_rooms: list[str]
    asks: list[AskJson]
    approach: list[ApproachJson]
    wall_contacts: list[WallHitJson]
    wall_range_fail: bool
    wall_stop_pass: bool
    wall_stop_clear: bool
    reach_clear: bool
    wall_stop_fail: bool
    wall_stop_residual_m: float | None
    wall_stop_class: str
    wall_stop_wall: str
    wall_stop_heading_rad: float | None
    wall_stop_cam_z_m: float | None
    wall_stop_pitch_rad: float | None
    wall_stop_lead_m: float | None
    wall_stop_lead_side: str
    wall_stop_pad_m: float | None
    e0_r_close3: list[list[float]]
    east_left: EastLeftFact
    approach_band: ApproachBand
    wall_residual_max_m: float | None
    same_wall_count: int
    surface_reject_count: int
    ray_reject_count: int
    class_reject_count: int
    frac_refuse_count: int
    wall_bias_max_m: float | None
    wall_bias_count: int
    wall_bias_repeat: bool
    search_applied_yaw_peak: float
    search_yaw_fail: bool
    latch_armed_contact: bool
    gait_limit: bool
    inplace_hip_fail: bool
    inplace_hip: list[InplaceHipJson]
    torque: list[TorqueJson]
    spawn: SpawnJson
    floor_box: FloorBox
    started_outside: bool
    finished_inside: bool
    reached: bool
    reach_block: str
    contact: str
    peak_nm: float
    peak_actuator: str
    peak_joint: str
    hip_while_yawing: list[HipJson]
    min_up_z: float
    t_end: float
    end_qpos_xyz: list[float]
    end_yaw_rad: float


class DefinitionJson(TypedDict):
    reached: str
    floor_box: str
    upright: str
    prop_contact: str
    time_limit_s: float
    voice_vx_cap: float
    voice_yaw_cap: float
    d_min: str
    d_min_m: float
    false_stop: str
    yes_no: str
    search: str
    hip_bar_nm: float
    soft_pass: bool


class _Patch(TypedDict):
    room: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


class _Stop(TypedDict):
    path: str
    reason: str
    hit_xy: tuple[float, float] | None
    on_shadow: bool
    prop_in_frame: bool
    false_stop: bool
    latch_pass: bool


PATCHES: tuple[_Patch, ...] = (
    _Patch(room="kitchen", xmin=SHADOW_X[0], xmax=SHADOW_X[1], ymin=-0.56, ymax=0.34),
    _Patch(room="bathroom", xmin=SHADOW_X[0], xmax=SHADOW_X[1], ymin=1.43, ymax=2.33),
)


def _plant_md5() -> str:
    return hashlib.md5(sw.PLANT_XML.read_bytes()).hexdigest()


def _definition() -> DefinitionJson:
    d_min_m = float(rc.d_min(T_DETECT_S))
    return DefinitionJson(
        reached=(
            "Body COM xy of body_link finishes inside that room's floor "
            "box from room_apartment.json, the bout started with that COM "
            f"outside the same box, min up_z is at least {UPRIGHT_UP_Z:.2f}, "
            "prop contacts are zero, and the bout ends at or before "
            f"{TIME_LIMIT_S:.1f} s. A start inside the box is not a reach. "
            "The plant floor from -3 m to +3 m is not the box."
        ),
        floor_box=(
            "The named room floor_box in mujoco/room_apartment.json. "
            "It is not a geom and it is not passed to a command."
        ),
        upright=f"min up_z over the bout >= {UPRIGHT_UP_Z:.2f}",
        prop_contact=(
            f"A contact with normal force >= {PROP_FORCE_N:.1f} N between "
            "a robot geom and any geom other than floor."
        ),
        time_limit_s=TIME_LIMIT_S,
        voice_vx_cap=voice.FWD_MPS,
        voice_yaw_cap=voice.YAW_RAD_S,
        d_min=(
            "d_min = 0.150 * (T_detect + T_stop). 0.150 is the hardware "
            "bound in that formula, not the voice vx cap. "
            f"T_detect = {T_DETECT_S:.3f} s. T_stop = {rc.T_STOP_S:.3f} s."
        ),
        d_min_m=d_min_m,
        false_stop=(
            "A #71 stop whose floor point lies on the speckled shadow "
            "patch in front of the kitchen or bathroom doorway, with no "
            "furniture prop in the kit_cam frame. That stop is not a "
            "#71 pass. Walls are not props."
        ),
        yes_no=(
            "Per asked room: 'Is there a {room} through the doorway "
            "ahead?' yes or no. Not an open-set room name. Moondream "
            f"yes is logged against the {ROOM_VISIBLE_FRAC:.0%} kit_cam "
            "gate. A yes under that fraction is refused and the search "
            "continues. It does not publish forward vel. A yes under "
            "that fraction that still commits forward vel is a wrong yes."
        ),
        search=(
            "Spawn yaw is the door-facing yaw plus or minus 90 degrees. "
            "After the stand, vel(0, +0.25) turns in place. No forward "
            "vel until a correct yes. Every 20 degrees the turn stops "
            f"(yaw 0) and the body settles for {SETTLE_S:.2f} s. "
            "The room question is asked only when the doorway opening "
            "is actually in kit_cam. Otherwise the body turns in place "
            "at yaw +0.25, and that applied yaw is logged on the search. "
            "The picture is still taken after yaw has settled to 0. "
            "Moondream detects a doorway pixel only on that gated frame. "
            "The logged heading is the heading at capture, not the "
            "heading when the answer returns. The search turn sign is "
            "not taken from the door bearing. A correct yes yaws in "
            "place toward the doorway pixel, settles until applied yaw "
            "is 0, then walks at +0.056 m/s with yaw 0. Yaw is not "
            "commanded while walking. It re-points the same way after "
            f"each {APPROACH_LOOK_S:.2f} s stop-settle-capture. kit_cam "
            "is not moved. A floor/wall image row is ranged with the "
            "live camera. The floor-edge latch fires only when the "
            "forward ray hits the same wall the row ranged. A prop or "
            "a different wall is logged and is not added to the latch. "
            f"latch_extra {LATCH_EXTRA_M:.5f} m is the median of "
            f"{len(SAME_WALL_ERR_M)} errors on {LIVING_WALL} with the "
            f"sim gap under {CLOSE_GAP_M:.2f} m. That median is recorded. "
            f"The live right-toe pad on {LIVING_WALL} is the median of "
            f"{len(W2_R_ERR_M)} close right samples, {W2_R_LATCH_M:.5f} m, "
            "with both committing stops left out. The left-toe pad on "
            f"that wall is the median of {len(W2_L_ERR_M)} close left "
            f"samples, {W2_L_LATCH_M:.5f} m. An east pad is not applied "
            f"to that wall. {EAST_WALL} left samples median "
            f"{EAST_L_LATCH_M:.5f} m (n={len(EAST_L_ERR_M)}). Median plus "
            f"MAD {EAST_L_APPLIED_M:.5f} m is recorded and is not the "
            "live latch. A same-bout median of every close left error, "
            f"pad {EAST_L_ONLINE_PAD_M:.5f} m, stopped at residual "
            f"{EAST_L_ONLINE_RESIDUAL_M:+.4f} m. The 75th percentile "
            f"{EAST_L_Q75_M:.5f} m stopped at residual "
            f"{EAST_L_Q75_RESIDUAL_M:+.4f} m. Neither is the latch. "
            "A new-closest median, N="
            f"{EAST_L_NEWMIN_N}, pad {EAST_L_NEWMIN_PAD_M:.5f} m, "
            f"stopped at residual {EAST_L_NEWMIN_RESIDUAL_M:+.4f} m "
            "and is not the latch. The live left latch is the median "
            f"of the last {EAST_L_NEAR_K} left samples whose ray is "
            f"at or under {EAST_L_NEAR_GATE_M:.2f} m. The sample that "
            "crosses the gate is left out. "
            "Any other wall uses that same median on its own leading "
            "toe once that ray is at or under "
            f"{EAST_L_NEAR_GATE_M:.2f} m. The commit sample is left "
            "out. An empty window keeps pad 0 and does not borrow an "
            "east or living pad. "
            f"{E0_WALL} right toe fills that same last-{E0_R_NEAR_K} "
            f"window once its own ray is at or under "
            f"{E0_R_NEAR_GATE_M:.2f} m. The pad stays undefined until "
            f"{E0_R_NEAR_MIN_N} samples are in the window. The live "
            f"pad is the median of the {E0_R_CLOSE_N} closest-ray "
            "samples in that window, not the median of the whole "
            "window. "
            f"The scored full-window miss is pad {E0_R_FAIL_PAD_M:.5f} m "
            f"(N={E0_R_FAIL_N}), residual {E0_R_FAIL_RESIDUAL_M:+.4f} m, "
            "and is not the latch. "
            f"The scored 0.25 m miss is pad {E0_R_THIN_PAD_M:.5f} m "
            f"(N={E0_R_THIN_N}), residual {E0_R_THIN_RESIDUAL_M:+.4f} m, "
            "and is not the latch. "
            "Left samples on that wall are not in the pad. "
            "East right-toe samples median "
            f"{EAST_R_LATCH_M:.5f} m (n={len(EAST_R_ERR_M)}). An unknown "
            "toe gets pad 0. Contact "
            f"boxes sit {TOE_OUTBOARD_M:.3f} m outboard of each ankle "
            "roll. The 0.40 m bound is 2.39 times the living "
            "right-toe latch distance 0.1672 m (d_min 0.1263 plus "
            "0.04087 m). "
            "A ray past 0.40 m stays a far reject. The bob pad is not "
            "stacked on any median. A stop clears only when the absolute "
            "gap between the measured error and that wall×toe pad is "
            f"at most {RANGE_ERR_MAX_M:.2f} m. A large negative residual "
            "does not clear. Right toe "
            f"has {len(EAST_R_ERR_M)} samples, under {len(SAME_WALL_ERR_M)}, "
            "so it cannot clear and it does not stop the walk. Once any "
            f"forward wall ray is at or under {EAST_L_HOLD_M:.2f} m, "
            "further in-place re-points are dropped and the walk stays "
            "at yaw 0. That is the same hold the east left toe already "
            "uses. The threshold is not widened. No pitch, camera "
            "height, or toe bias is fit across pooled samples. "
            "Room reach is a "
            "separate bar. In-place body "
            f"yaw that asks a hip roll over {HIP_BAR_NM:.2f} Nm "
            "unclamped is not a free re-point. d_min is not rebuilt. "
            "The ray uses the live camera height and the IMU pitch "
            "plus head_tilt, not a standing-pose table."
        ),
        hip_bar_nm=HIP_BAR_NM,
        soft_pass=False,
    )


def _inside(x: float, y: float, box: FloorBox) -> bool:
    return box["xmin"] <= x <= box["xmax"] and box["ymin"] <= y <= box["ymax"]


def _in_patch(x: float, y: float, patch: _Patch) -> bool:
    return patch["xmin"] <= x <= patch["xmax"] and patch["ymin"] <= y <= patch["ymax"]


def _patch_of(x: float, y: float) -> str:
    for patch in PATCHES:
        if _in_patch(x, y, patch):
            return patch["room"]
    return ""


def _yaw(data: mj.MjData, body_id: int) -> float:
    rot = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    return float(math.atan2(rot[1, 0], rot[0, 0]))


def _robot_bodies(plant: mj.MjModel) -> set[str]:
    names: set[str] = set()
    for body_id in range(plant.nbody):
        name = mj.mj_id2name(plant, mj.mjtObj.mjOBJ_BODY, body_id) or ""
        if name != "":
            names.add(name)
    return names


def _prop_geom_ids(model: mj.MjModel, robot_bodies: set[str]) -> dict[int, str]:
    found: dict[int, str] = {}
    for geom_id in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or ""
        body_id = int(model.geom_bodyid[geom_id])
        body = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or ""
        if body in robot_bodies or name == "floor" or name.startswith("wall_"):
            continue
        found[geom_id] = name
    return found


def _prop_contact(model: mj.MjModel, data: mj.MjData, robot_bodies: set[str]) -> str:
    best_name = ""
    best_force = 0.0
    for index in range(data.ncon):
        con = data.contact[index]
        names: list[str] = []
        bodies: list[str] = []
        for geom_id in (int(con.geom1), int(con.geom2)):
            names.append(mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or "")
            body_id = int(model.geom_bodyid[geom_id])
            bodies.append(mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or "")
        if "floor" in names:
            continue
        robot = [body in robot_bodies for body in bodies]
        if robot[0] == robot[1]:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, index, force)
        normal = float(force[0])
        if normal < PROP_FORCE_N:
            continue
        prop = names[0] if robot[1] else names[1]
        if normal > best_force:
            best_force = normal
            best_name = prop
    if best_name == "":
        return "none"
    return f"{best_name} {best_force:.1f} N"


def _prop_in_frame(
    seg: np.ndarray,
    prop_ids: dict[int, str],
) -> bool:
    if seg.ndim != 3 or seg.shape[2] < 2:
        raise RuntimeError(f"segmentation shape {seg.shape}")
    geom_type = int(mj.mjtObj.mjOBJ_GEOM)
    types = seg[:, :, 1]
    ids = seg[:, :, 0]
    visible = ids[types == geom_type]
    if visible.size == 0:
        # This MuJoCo build stores the geom id in channel 0 and mjOBJ_GEOM
        # in channel 1. An empty type channel is a renderer change.
        if int(np.max(types)) != geom_type and int(np.max(ids)) > 0:
            raise RuntimeError("segmentation channels are not geom id, geom type")
        return False
    for geom_id in np.unique(visible):
        if int(geom_id) in prop_ids:
            return True
    return False


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _joint_name(actuator: str) -> str:
    if actuator.endswith("_pos"):
        return actuator[: -len("_pos")]
    return actuator


def _geom_room(name: str) -> str:
    for room, prefixes in ROOM_GEOMS.items():
        if name.startswith(prefixes):
            return room
    return ""


def _room_fractions(model: mj.MjModel, seg: np.ndarray) -> dict[str, float]:
    fracs = {room: 0.0 for room in ROOM_GEOMS}
    if seg.ndim != 3 or seg.shape[2] < 2:
        raise RuntimeError(f"segmentation shape {seg.shape}")
    geom_type = int(mj.mjtObj.mjOBJ_GEOM)
    types = seg[:, :, 1]
    ids = seg[:, :, 0]
    total = float(ids.size)
    visible = types == geom_type
    for geom_id in np.unique(ids[visible]):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
        room = _geom_room(name)
        if room == "":
            continue
        fracs[room] += float(np.sum((ids == geom_id) & visible)) / total
    return fracs


def _visible_rooms(fracs: dict[str, float]) -> list[str]:
    return [room for room, frac in fracs.items() if frac >= ROOM_VISIBLE_FRAC]


def _doorway_fraction(
    model: mj.MjModel,
    data: mj.MjData,
    opening: apt.Box,
    step: int = RAY_STEP,
) -> float:
    """Fraction of kit_cam rays that pass through this doorway opening.

    The plane is the hall face of the opening. A ray counts when it meets
    that rectangle before the first geom. kit_cam is not moved.
    """
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    origin = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    rot = np.asarray(data.cam_xmat[cid], dtype=np.float64).reshape(3, 3)
    plane_x = opening["xmin"] if float(origin[0]) < opening["xmin"] else opening["xmax"]
    ymin = opening["ymin"]
    ymax = opening["ymax"]
    z0, z1 = DOOR_Z
    hit = 0
    total = 0
    geomid = np.zeros(1, dtype=np.int32)
    for v in range(0, rc.HEIGHT, step):
        for u in range(0, rc.WIDTH, step):
            total += 1
            direction = rot @ rc.pixel_direction(float(u) + 0.5, float(v) + 0.5)
            if abs(float(direction[0])) < 1e-8:
                continue
            travel = (plane_x - float(origin[0])) / float(direction[0])
            if travel <= 0.05:
                continue
            point = origin + travel * direction
            if not (ymin <= float(point[1]) <= ymax and z0 <= float(point[2]) <= z1):
                continue
            dist = mj.mj_ray(model, data, origin, direction, None, 1, -1, geomid)
            if dist < 0 or float(dist) + 1e-3 >= travel:
                hit += 1
    if total == 0:
        return 0.0
    return hit / total


def _yaw_toward_u(u: float) -> float:
    """Full ±0.25 toward a doorway pixel. Centre band commands yaw 0.

    A pixel left of centre is +yaw. kit_cam +X is image right, and +yaw
    is a left turn. The sign is the pixel, not the door bearing.
    """
    center = rc.WIDTH / 2.0
    if abs(u - center) <= DOOR_DEAD_FRAC * rc.WIDTH:
        return 0.0
    if u < center:
        return voice.YAW_RAD_S
    return -voice.YAW_RAD_S


def _door_pixels(objects: list[object]) -> list[dict[str, object]]:
    """Normalized detect boxes to pixel centre, area fraction, and bbox."""
    rows: list[dict[str, object]] = []
    for obj in objects:
        if not isinstance(obj, dict) or "x_min" not in obj:
            continue
        x0 = float(obj["x_min"]) * rc.WIDTH
        x1 = float(obj["x_max"]) * rc.WIDTH
        y0 = float(obj["y_min"]) * rc.HEIGHT
        y1 = float(obj["y_max"]) * rc.HEIGHT
        if x1 <= 0.0 or x0 >= rc.WIDTH or y1 <= 0.0 or y0 >= rc.HEIGHT:
            continue
        width = max(0.0, x1 - x0)
        height = max(0.0, y1 - y0)
        if width < 1.0 or height < 1.0:
            continue
        rows.append({
            "u": 0.5 * (x0 + x1),
            "v": 0.5 * (y0 + y1),
            "frac": (width / rc.WIDTH) * (height / rc.HEIGHT),
            "bbox": [x0, y0, x1, y1],
        })
    return rows


def _wall_ids(model: mj.MjModel) -> dict[int, str]:
    found: dict[int, str] = {}
    for geom_id in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name.startswith("wall_"):
            found[geom_id] = name
    return found


def _true_wall_gap(
    model: mj.MjModel,
    data: mj.MjData,
    session: sw.SteerSession,
    cid: int,
) -> tuple[str, float | None]:
    """Sim gap from the leading toe to the first wall along body forward."""
    toes = gate.toe_samples(session, cid)
    if not toes:
        return "", None
    lead = max(toes, key=lambda row: row.offset_m)
    fwd = gate.body_forward_xy(data, session.bid_body)
    norm = float(math.hypot(fwd[0], fwd[1]))
    if norm < 1e-9:
        return "", None
    direction = np.array([fwd[0] / norm, fwd[1] / norm, 0.0], dtype=np.float64)
    origin = np.array([lead.toe_xy[0], lead.toe_xy[1], 0.05], dtype=np.float64)
    geomid = np.zeros(1, dtype=np.int32)
    dist = mj.mj_ray(model, data, origin, direction, None, 1, -1, geomid)
    if dist < 0.0:
        return "", None
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(geomid[0])) or ""
    if not name.startswith("wall_"):
        return name, float(dist)
    return name, float(dist)


def _floor_wall_boundaries(
    seg: np.ndarray,
    floor_id: int,
    walls: dict[int, str],
) -> list[tuple[float, float, str]]:
    """Colour-free floor pixel just under a wall geom, nearest row per column.

    v grows downward. Scanning from the bottom of the frame hits the
    near wall base first. kit_cam is not moved. The row is ranged later
    with ray_corridor.estimate_hazard, the same floor ray the #70 mono
    height uses. This does not arm the refused rug-height gate.
    """
    if seg.ndim != 3 or seg.shape[2] < 2:
        raise RuntimeError(f"segmentation shape {seg.shape}")
    geom_type = int(mj.mjtObj.mjOBJ_GEOM)
    ids = seg[:, :, 0]
    types = seg[:, :, 1]
    hits: list[tuple[float, float, str]] = []
    for u in range(0, rc.WIDTH, RAY_STEP):
        for v in range(rc.HEIGHT - 2, 0, -1):
            if int(types[v, u]) != geom_type or int(types[v - 1, u]) != geom_type:
                continue
            if int(ids[v, u]) != floor_id:
                continue
            above = int(ids[v - 1, u])
            name = walls.get(above, "")
            if name == "":
                continue
            hits.append((float(u) + 0.5, float(v) + 0.5, name))
            break
    return hits


def _cam_pitch(yaw: float, pitch: float, roll: float, tilt: float, pan: float) -> float:
    """Pitch of the kit_cam look direction from IMU and head joints.

    Negative is below the horizon. This is not a standing-pose constant
    and it is not the sim camera matrix.
    """
    rot = rc.camera_rotation_from_imu(yaw, pitch, roll, tilt, pan)
    look = -rot[:, 2]
    return math.atan2(float(look[2]), float(math.hypot(look[0], look[1])))


def _range_wall(
    session: sw.SteerSession,
    model: mj.MjModel,
    renderer: mj.Renderer,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
    floor_id: int,
    walls: dict[int, str],
) -> dict[str, object]:
    """Nearest in-corridor wall base, ranged at this capture's camera.

    Height is the live kit_cam optical centre (body plus the head joints).
    The ray direction is body IMU pitch and roll plus head_tilt and
    head_pan. A fixed stand height and stand pitch are not used.
    """
    pose, body, yaw, pitch, roll, tilt, pan, step_off, _cam = _pose_bits(
        session, model, cid, jid, pan_id, reach,
    )
    toes = gate.toe_samples(session, cid)
    # Distance in front of the foot that is ahead right now. East-hall
    # and west-hall headings swap which foot that is. The latch still
    # subtracts the period high-water inside toe_gap. That margin is
    # not camera-pitch error.
    lead = None if not toes else max(toes, key=lambda row: row.offset_m)
    lead_off = 0.0 if lead is None else float(lead.offset_m)
    lead_side = "" if lead is None else str(lead.side)
    reading: dict[str, object] = {
        "u": None,
        "v": None,
        "wall": "",
        "toe_gap_m": None,
        "geometric_m": None,
        "lead_off_m": lead_off,
        "lead_side": lead_side,
        "step_off_m": float(step_off),
        "in_corridor": False,
        "cam_z_m": float(pose.position_m[2]),
        "imu_pitch_rad": float(pitch),
        "head_tilt_rad": float(tilt),
        "cam_pitch_rad": _cam_pitch(yaw, pitch, roll, tilt, pan),
    }
    seg = _render_seg(renderer, session.data)
    for u, v, name in _floor_wall_boundaries(seg, floor_id, walls):
        estimate = rc.estimate_hazard(
            u, v,
            cam=pose, body=body,
            imu_roll_rad=roll, imu_pitch_rad=pitch,
            head_tilt_rad=tilt, yaw_rate=float(session.bus.applied_yaw_rate),
            step_off_m=step_off, hazard_pad_m=rc.HAZARD_PAD_M,
            head_pan_rad=pan,
        )
        if estimate is None or not estimate.in_corridor:
            continue
        gap = float(estimate.toe_gap_m)
        geometric = float(estimate.forward_m) - lead_off
        held = reading["toe_gap_m"]
        if held is None or gap < float(held):
            reading["u"] = u
            reading["v"] = v
            reading["wall"] = name
            reading["toe_gap_m"] = gap
            reading["geometric_m"] = geometric
            reading["in_corridor"] = True
    return reading


def _same_wall(reading: dict[str, object], true_name: str) -> bool:
    ranged = str(reading.get("wall") or "")
    return ranged.startswith("wall_") and ranged == true_name


def _geom_class(wall: str, true_gap: float | None) -> str:
    if true_gap is None:
        return "unranged"
    close = float(true_gap) <= CLOSE_GAP_M
    if wall == LIVING_WALL and close:
        return "living_close"
    if wall == EAST_WALL and close:
        return "east_close"
    if wall == LIVING_WALL or wall == EAST_WALL:
        return "far_same_name"
    if close:
        return "other_wall_close"
    return "other_wall_far"


def _toe_class(klass: str, lead_side: str) -> str:
    if klass in ("east_close", "living_close") and lead_side in ("L", "R"):
        return f"{klass}_{lead_side}"
    return klass


def _toe_ready(klass: str) -> bool:
    """A wall×toe pad can CLEAR only with a set at least as large as the living 17."""
    if klass == "east_close_L":
        return len(EAST_L_ERR_M) >= len(SAME_WALL_ERR_M)
    if klass == "east_close_R":
        return len(EAST_R_ERR_M) >= len(SAME_WALL_ERR_M)
    if klass == "living_close_L":
        return len(W2_L_ERR_M) >= len(SAME_WALL_ERR_M)
    if klass == "living_close_R":
        return len(W2_R_ERR_M) >= len(SAME_WALL_ERR_M)
    if klass == "other_wall_close":
        return True
    return False


def _pad_for(wall: str, true_gap: float | None, lead_side: str = "") -> float:
    """Frozen wall×toe pad. East left is supplied by the bout, not here."""
    klass = _geom_class(wall, true_gap)
    if klass == "living_close" and lead_side == "L":
        return W2_L_LATCH_M
    if klass == "living_close" and lead_side == "R":
        return W2_R_LATCH_M
    if klass == "east_close" and lead_side == "R":
        return EAST_R_LATCH_M
    return 0.0


def _band_errs(
    seen: list[tuple[float, float]], gate_m: float, k: int,
) -> list[float]:
    """Last K errors whose ray is inside one wall's near gate."""
    chosen = [err for ray, err in seen if ray <= gate_m]
    if len(chosen) > k:
        return chosen[-k:]
    return chosen


def _near_k_errs(seen: list[tuple[float, float]]) -> list[float]:
    """Last K left errors whose ray is inside the near-stop gate."""
    return _band_errs(seen, EAST_L_NEAR_GATE_M, EAST_L_NEAR_K)


def _east_l_bout_pad(prior: list[float]) -> tuple[float, bool]:
    """East-left pad for this sample. The commit sample is not in prior."""
    if EAST_L_LIVE == "quantile":
        return EAST_L_Q75_M, True
    if prior:
        return float(statistics.median(prior)), True
    return 0.0, False


def _median_pad(prior: list[float]) -> tuple[float, int, bool]:
    if not prior:
        return 0.0, 0, False
    return float(statistics.median(prior)), len(prior), True


def _other_near_pad(seen: list[tuple[float, float]]) -> tuple[float, int, bool]:
    """Same last-K near median for a wall that is not east or living.

    The commit sample is not in seen. An empty window does not invent
    a pad and does not copy another wall. wall_hall_e_0 right toe
    does not use this window.
    """
    return _median_pad(_near_k_errs(seen))


def _e0_r_window(seen: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Last K wall_hall_e_0 right samples inside the near gate.

    The commit sample is not in seen. Left samples are not in seen.
    """
    chosen = [
        (float(ray), float(err))
        for ray, err in seen
        if float(ray) <= E0_R_NEAR_GATE_M
    ]
    if len(chosen) > E0_R_NEAR_K:
        return chosen[-E0_R_NEAR_K:]
    return chosen


def _e0_r_picked(seen: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Three closest rays inside a filled e_0 right window.

    An unfilled window returns nothing, so the latch stays dark.
    Ties break toward the smaller error, then the earlier sample.
    """
    window = _e0_r_window(seen)
    if len(window) < E0_R_NEAR_MIN_N:
        return []
    order = sorted(
        enumerate(window),
        key=lambda item: (item[1][0], item[1][1], item[0]),
    )
    return [pair for _index, pair in order[:E0_R_CLOSE_N]]


def _e0_r_near_pad(seen: list[tuple[float, float]]) -> tuple[float, int, bool]:
    """wall_hall_e_0 right toe only.

    The window is the kitchen near window. The pad is the median of
    the three closest rays in it, not the median of all five.
    """
    window = _e0_r_window(seen)
    if len(window) < E0_R_NEAR_MIN_N:
        return 0.0, len(window), False
    return _median_pad([err for _ray, err in _e0_r_picked(seen)])


def _residual_m(err: float | None, pad: float) -> float | None:
    if err is None:
        return None
    return float(err) - pad


def _within_cm(residual: float | None) -> bool:
    return residual is not None and abs(residual) <= RANGE_ERR_MAX_M


def _wall_hit(
    t: float,
    contact: str,
    ranged: float | None,
    true_gap: float | None,
    true_name: str,
    err: float | None,
    over: bool,
    reading: dict[str, object],
    latch_path: str,
    cues: tuple[hf.HazardCue, ...],
    residual: float | None,
    heading_rad: float,
    geom_class: str,
) -> WallHitJson:
    lead = reading.get("lead_off_m")
    cam_z = reading.get("cam_z_m")
    imu_pitch = reading.get("imu_pitch_rad")
    head_tilt = reading.get("head_tilt_rad")
    cam_pitch = reading.get("cam_pitch_rad")
    return WallHitJson(
        t=t,
        contact=contact,
        ranged_toe_gap_m=ranged,
        true_wall_gap_m=true_gap,
        true_wall=true_name,
        range_err_m=err,
        range_over_1cm=over,
        cam_z_m=None if cam_z is None else float(cam_z),
        imu_pitch_rad=None if imu_pitch is None else float(imu_pitch),
        head_tilt_rad=None if head_tilt is None else float(head_tilt),
        cam_pitch_rad=None if cam_pitch is None else float(cam_pitch),
        lead_off_m=None if lead is None else float(lead),
        lead_side=str(reading.get("lead_side") or ""),
        residual_m=residual,
        ranged_wall=str(reading.get("wall") or ""),
        latch_path=latch_path,
        heading_rad=heading_rad,
        geom_class=geom_class,
        finder=[_cue_json(cue) for cue in cues],
    )


def _inplace_rows(bout: _Bout) -> list[InplaceHipJson]:
    rows: list[InplaceHipJson] = []
    for joint in sorted(bout.inplace_unclamped):
        rows.append(InplaceHipJson(
            joint=joint,
            unclamped_nm=float(bout.inplace_unclamped[joint]),
            clamped_nm=float(bout.inplace_clamped.get(joint, 0.0)),
        ))
    return rows


def _cue_json(cue: hf.HazardCue) -> FinderCueJson:
    return FinderCueJson(
        u=None if cue.u is None else float(cue.u),
        v=None if cue.v is None else float(cue.v),
        too_close=bool(cue.too_close),
        bottom_clipped=bool(cue.bottom_clipped),
        span_px=int(cue.span_px),
        width_px=int(cue.width_px),
        source=str(cue.source),
        column=int(cue.column),
    )


def _nearest_prop(
    model: mj.MjModel,
    data: mj.MjData,
    prop_ids: dict[int, str],
    hit_xy: tuple[float, float],
) -> tuple[str, float]:
    best_name = ""
    best = 1e9
    for geom_id, name in prop_ids.items():
        center = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        half = np.asarray(model.geom_size[geom_id], dtype=np.float64)
        # Box geoms use half-size. Other types use the center only.
        if int(model.geom_type[geom_id]) == int(mj.mjtGeom.mjGEOM_BOX):
            dx = max(abs(hit_xy[0] - float(center[0])) - float(half[0]), 0.0)
            dy = max(abs(hit_xy[1] - float(center[1])) - float(half[1]), 0.0)
        else:
            dx = hit_xy[0] - float(center[0])
            dy = hit_xy[1] - float(center[1])
        dist = math.hypot(dx, dy)
        if dist < best:
            best = dist
            best_name = name
    return best_name, best


def _window(rgb: np.ndarray, u: float, v: float) -> np.ndarray | None:
    height = int(rgb.shape[0])
    width = int(rgb.shape[1])
    cu = int(round(u))
    cv = int(round(v))
    if cu < 12 or cv < 12 or cu >= width - 12 or cv >= height - 12:
        return None
    return rgb[cv - 12: cv + 13, cu - 12: cu + 13]


def _speckled(window: np.ndarray) -> tuple[bool, float, float]:
    lum = window.astype(np.int32).sum(axis=2) // 3
    mean = float(lum.mean())
    std = float(lum.std())
    p10 = float(np.percentile(lum, 10))
    p90 = float(np.percentile(lum, 90))
    ok = (
        std >= SPECKLE_STD_MIN
        and mean <= SPECKLE_MEAN_MAX
        and p10 <= SPECKLE_P10_MAX
        and (p90 - p10) >= SPECKLE_SPAN_MIN
    )
    return ok, mean, std


def _speckle_self_check() -> None:
    flat = np.full((25, 25, 3), 40, dtype=np.uint8)
    ok, _mean, _std = _speckled(flat)
    if ok:
        raise SystemExit("FAIL: a flat patch counted as speckled")
    checker = np.zeros((25, 25, 3), dtype=np.uint8)
    checker[::2, ::2] = 44
    checker[1::2, 1::2] = 44
    checker[::2, 1::2] = 78
    checker[1::2, ::2] = 78
    ok, mean, std = _speckled(checker)
    if not ok:
        raise SystemExit(f"FAIL: checker shadow was not speckled mean={mean:.1f} std={std:.1f}")
    bright = np.full((25, 25, 3), 210, dtype=np.uint8)
    bright[::2] = 180
    ok, _mean, _std = _speckled(bright)
    if ok:
        raise SystemExit("FAIL: a bright wall counted as the floor shadow")


def _pose_bits(
    session: sw.SteerSession,
    model: mj.MjModel,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
) -> tuple[rc.KitCamPose, rc.BodyFrame, float, float, float, float, float, float, np.ndarray]:
    data = session.data
    tilt = float(data.qpos[int(model.jnt_qposadr[jid])])
    pan = 0.0 if pan_id < 0 else float(data.qpos[int(model.jnt_qposadr[pan_id])])
    body_rot = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
    yaw, pitch, roll = rc.imu_from_body(body_rot)
    cam = np.asarray(data.cam_xpos[cid], dtype=np.float64)
    fwd = gate.body_forward_xy(data, session.bid_body)
    toes = gate.toe_samples(session, cid)
    now = float(data.time)
    reach.append((
        now,
        next(row.offset_m for row in toes if row.side == "L"),
        next(row.offset_m for row in toes if row.side == "R"),
    ))
    step_off, _side, _when = gate._high_water(reach)
    body = rc.BodyFrame(
        (float(data.qpos[0]), float(data.qpos[1])),
        (float(fwd[0]), float(fwd[1])),
    )
    pose = rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2])))
    return pose, body, yaw, pitch, roll, tilt, pan, step_off, cam


def _cue_hit(
    cue: hf.HazardCue,
    *,
    pose: rc.KitCamPose,
    body: rc.BodyFrame,
    roll: float,
    pitch: float,
    tilt: float,
    yaw_rate: float,
    step_off: float,
    pan: float,
) -> tuple[float, float] | None:
    if cue.too_close or cue.u is None or cue.v is None:
        u = float(cue.column) + 0.5
        v = float(rc.HEIGHT) - 0.5
    else:
        u = float(cue.u)
        v = float(cue.v)
    estimate = rc.estimate_hazard(
        u, v,
        cam=pose,
        body=body,
        imu_roll_rad=roll,
        imu_pitch_rad=pitch,
        head_tilt_rad=tilt,
        yaw_rate=yaw_rate,
        step_off_m=step_off,
        hazard_pad_m=rc.HAZARD_PAD_M,
        head_pan_rad=pan,
    )
    if estimate is None:
        return None
    return estimate.hit_xy_m


def _render_rgb(renderer: mj.Renderer, data: mj.MjData) -> np.ndarray:
    renderer.disable_segmentation_rendering()
    renderer.update_scene(data, camera="kit_cam")
    return np.asarray(renderer.render(), dtype=np.uint8).copy()


def _render_seg(renderer: mj.Renderer, data: mj.MjData) -> np.ndarray:
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="kit_cam")
    seg = np.asarray(renderer.render()).copy()
    renderer.disable_segmentation_rendering()
    return seg


def _shadow_candidate(
    seen: set[str],
    *,
    rgb: np.ndarray,
    cam: np.ndarray,
    rot: np.ndarray,
) -> tuple[str, float, float, float, float] | None:
    """First unseen doorway patch whose floor sample is speckled in frame."""
    for patch in PATCHES:
        if patch["room"] in seen:
            continue
        xs = (patch["xmin"], 0.5 * (patch["xmin"] + patch["xmax"]), patch["xmax"])
        ys = (patch["ymin"], 0.5 * (patch["ymin"] + patch["ymax"]), patch["ymax"])
        for x in xs:
            for y in ys:
                pix = rc.project_point(np.array([x, y, 0.0], dtype=np.float64), cam, rot)
                if pix is None:
                    continue
                window = _window(rgb, pix[0], pix[1])
                if window is None:
                    continue
                speckled, mean, std = _speckled(window)
                if speckled:
                    return patch["room"], float(pix[0]), float(pix[1]), mean, std
    return None


def _mark_finder(
    sightings: list[ShadowSighting],
    patch_name: str,
    hit: tuple[float, float],
) -> None:
    for row in sightings:
        if row["patch"] == patch_name and not row["finder_emitted"]:
            row["finder_emitted"] = True
            row["hit_xy"] = [hit[0], hit[1]]


def _latch_cues(
    cues: tuple[hf.HazardCue, ...],
    tracks: list[gate._GapTrack],
    *,
    pose: rc.KitCamPose,
    body: rc.BodyFrame,
    roll: float,
    pitch: float,
    tilt: float,
    yaw_rate: float,
    step_off: float,
    pan: float,
    now: float,
    gate_m: float,
) -> tuple[str, tuple[float, float] | None, hf.HazardCue | None]:
    """#71 shortest in-corridor gap, then an in-corridor too_close clip.

    Returns path, floor hit, and the clip. path is empty when nothing stops.
    """
    clipped: hf.HazardCue | None = None
    for cue in cues:
        if cue.too_close and clipped is None:
            clipped = cue
        if cue.u is None or cue.v is None or cue.too_close:
            continue
        estimate = rc.estimate_hazard(
            cue.u, cue.v,
            cam=pose, body=body,
            imu_roll_rad=roll, imu_pitch_rad=pitch,
            head_tilt_rad=tilt, yaw_rate=yaw_rate,
            step_off_m=step_off, hazard_pad_m=rc.HAZARD_PAD_M,
            head_pan_rad=pan,
        )
        if estimate is None or not estimate.in_corridor:
            continue
        hit = estimate.hit_xy_m
        offer = estimate.toe_gap_m
        joined: gate._GapTrack | None = None
        joined_d = gate.TRACK_JOIN_M
        for held in tracks:
            apart = math.hypot(hit[0] - held.hit_xy[0], hit[1] - held.hit_xy[1])
            if apart <= joined_d:
                joined_d = apart
                joined = held
        if joined is None:
            tracks.append(gate._GapTrack(hit, offer, estimate.sideways_m, now))
        elif offer < joined.gap_m:
            joined.hit_xy = hit
            joined.gap_m = offer
            joined.side_m = estimate.sideways_m
            joined.t_s = now
    chosen: gate._GapTrack | None = None
    for held in tracks:
        if chosen is None or held.gap_m < chosen.gap_m:
            chosen = held
    if chosen is not None and chosen.gap_m <= gate_m:
        return "shortest", chosen.hit_xy, clipped
    if clipped is not None:
        hit = _cue_hit(
            clipped, pose=pose, body=body, roll=roll, pitch=pitch,
            tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
        )
        return "too_close", hit, clipped
    return "", None, None


def _refresh_tracks(
    tracks: list[gate._GapTrack],
    *,
    cam_xy: tuple[float, float],
    body: rc.BodyFrame,
    yaw_rate: float,
    step_off: float,
) -> list[gate._GapTrack]:
    alive: list[gate._GapTrack] = []
    for held in tracks:
        gap_now, _forward, side_now, inside = gate._pose_gap(
            held.hit_xy, cam_xy, body, yaw_rate, step_off,
        )
        if not inside:
            continue
        held.gap_m = gap_now
        held.side_m = side_now
        alive.append(held)
    return alive


def _finder_patch(
    cues: tuple[hf.HazardCue, ...],
    *,
    pose: rc.KitCamPose,
    body: rc.BodyFrame,
    roll: float,
    pitch: float,
    tilt: float,
    yaw_rate: float,
    step_off: float,
    pan: float,
) -> tuple[str, tuple[float, float] | None]:
    for cue in cues:
        hit = _cue_hit(
            cue, pose=pose, body=body, roll=roll, pitch=pitch,
            tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
        )
        if hit is None:
            continue
        name = _patch_of(hit[0], hit[1])
        if name != "":
            return name, hit
    return "", None


class _Bout:
    def __init__(self) -> None:
        self.contact = "none"
        self.peak_nm = 0.0
        self.peak_actuator = ""
        self.yaw_on = False
        self.approach_yaw = False
        self.hip_peak: dict[str, float] = {}
        self.tau_unclamped: dict[str, float] = {}
        self.tau_clamped: dict[str, float] = {}
        self.approach_unclamped: dict[str, float] = {}
        self.approach_clamped: dict[str, float] = {}
        self.measured: dict[str, float] = {}
        self.gait_limit = False
        self.inplace_hip_fail = False
        self.inplace_unclamped: dict[str, float] = {}
        self.inplace_clamped: dict[str, float] = {}


def _watch_step(
    model: mj.MjModel,
    data: mj.MjData,
    bout: _Bout,
    robot_bodies: set[str],
    act_idx: dict[str, int],
) -> None:
    hit = _prop_contact(model, data, robot_bodies)
    if hit != "none" and bout.contact == "none":
        bout.contact = hit
    for name, index in act_idx.items():
        if "hip_" not in name and "knee" not in name and "ank_" not in name:
            continue
        force = float(data.actuator_force[index])
        if abs(force) > abs(bout.peak_nm):
            bout.peak_nm = force
            bout.peak_actuator = name
        joint = _joint_name(name)
        if "knee" in joint or "hip_roll" in joint:
            held = bout.measured.get(joint)
            if held is None or abs(force) >= abs(held):
                bout.measured[joint] = force
        if bout.yaw_on and name in bout.hip_peak and abs(force) > abs(bout.hip_peak[name]):
            bout.hip_peak[name] = force


def _watch_tau(session: sw.SteerSession, bout: _Bout) -> None:
    """Peak unclamped prediction beside the clamped prediction. No retune."""
    lipm = session.lipm
    if lipm is None:
        return
    turning = bout.yaw_on and (
        abs(float(session.bus.target_yaw)) > 1e-6
        or abs(float(session.bus.applied_yaw_rate)) > 1e-6
    )
    for joint, pair in lipm.tau_note.items():
        unclamped, clamped = pair
        prev = bout.tau_unclamped.get(joint)
        if prev is None or abs(unclamped) >= abs(prev):
            bout.tau_unclamped[joint] = unclamped
            bout.tau_clamped[joint] = clamped
        if turning and "hip_roll" in joint:
            held_u = bout.inplace_unclamped.get(joint)
            if held_u is None or abs(unclamped) >= abs(held_u):
                bout.inplace_unclamped[joint] = unclamped
                bout.inplace_clamped[joint] = clamped
            if abs(unclamped) > HIP_BAR_NM:
                bout.inplace_hip_fail = True
        if not bout.approach_yaw:
            continue
        held = bout.approach_unclamped.get(joint)
        if held is None or abs(unclamped) >= abs(held):
            bout.approach_unclamped[joint] = unclamped
            bout.approach_clamped[joint] = clamped
        # The write budget is 2.28 Nm on a stop and 2.33 Nm on a yawed
        # hip roll. A larger unclamped prediction sitting on that write
        # is the clamp, not a measured peak.
        if (
            abs(unclamped) > abs(clamped) + 0.02
            and abs(clamped) >= lipm_gait.LEG_STOP_NM - 0.02
        ):
            bout.gait_limit = True


def _hip_rows(bout: _Bout) -> list[HipJson]:
    rows: list[HipJson] = []
    for joint, actuator in HIP_ACTUATORS:
        peak = float(bout.hip_peak.get(actuator, 0.0))
        rows.append(HipJson(
            joint=joint,
            actuator=actuator,
            peak_nm=peak,
            abs_nm=abs(peak),
            over_2_33=abs(peak) > HIP_BAR_NM,
        ))
    return rows


def _score_stop(
    path: str,
    reason: str,
    hit: tuple[float, float] | None,
    *,
    model: mj.MjModel,
    data: mj.MjData,
    prop_ids: dict[int, str],
    prop_in_frame: bool,
) -> _Stop:
    on_shadow = False if hit is None else _patch_of(hit[0], hit[1]) != ""
    near = ""
    dist = 1e9
    if hit is not None:
        near, dist = _nearest_prop(model, data, prop_ids, hit)
    false_stop = path in ("shortest", "too_close") and on_shadow and not prop_in_frame
    latch_pass = (
        path in ("shortest", "too_close")
        and prop_in_frame
        and not on_shadow
        and dist <= PROP_HIT_M
    )
    if false_stop:
        reason = (
            f"{reason} false_stop shadow patch, no prop in frame"
        )
    elif path in ("shortest", "too_close"):
        reason = (
            f"{reason} prop_in_frame={prop_in_frame} nearest={near} "
            f"dist={dist:.3f} on_shadow={on_shadow}"
        )
    return _Stop(
        path=path,
        reason=reason,
        hit_xy=hit,
        on_shadow=on_shadow,
        prop_in_frame=prop_in_frame,
        false_stop=false_stop,
        latch_pass=latch_pass,
    )


def _empty_stop() -> _Stop:
    return _Stop(
        path="",
        reason="",
        hit_xy=None,
        on_shadow=False,
        prop_in_frame=False,
        false_stop=False,
        latch_pass=False,
    )


def _sense(
    session: sw.SteerSession,
    model: mj.MjModel,
    renderer: mj.Renderer,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
    tracks: list[gate._GapTrack],
    phase_floor: hf.SamePhaseFloor,
    gate_m: float,
    sightings: list[ShadowSighting],
    seen: set[str],
    prop_ids: dict[int, str],
) -> tuple[str, tuple[float, float] | None, np.ndarray, tuple[hf.HazardCue, ...]]:
    pose, body, yaw, pitch, roll, tilt, pan, step_off, cam = _pose_bits(
        session, model, cid, jid, pan_id, reach,
    )
    yaw_rate = float(session.bus.applied_yaw_rate)
    tracks[:] = _refresh_tracks(
        tracks, cam_xy=(float(cam[0]), float(cam[1])),
        body=body, yaw_rate=yaw_rate, step_off=step_off,
    )
    data = session.data
    rgb = _render_rgb(renderer, data)
    raw = hf.find_hazard_cues(rgb)
    cues = hf.corridor_gate_cues(
        raw, cam=pose, body=body,
        imu_roll_rad=roll, imu_pitch_rad=pitch,
        head_tilt_rad=tilt, yaw_rate=yaw_rate,
        step_off_m=step_off, head_pan_rad=pan,
    )
    if session.lipm is not None and not phase_floor.has_prior(session.lipm.stance):
        cues = hf.confirm_leg_columns(cues, rgb)
    if session.lipm is not None:
        cues = phase_floor.apply(
            cues,
            phase=session.lipm.phase,
            stance=session.lipm.stance,
            body_xy=(float(data.qpos[0]), float(data.qpos[1])),
            cam=pose, body=body,
            imu_roll_rad=roll, imu_pitch_rad=pitch,
            head_tilt_rad=tilt, yaw_rate=yaw_rate,
            step_off_m=step_off, head_pan_rad=pan,
        )
    rot = rc.camera_rotation_from_imu(yaw, pitch, roll, tilt, pan)
    candidate = _shadow_candidate(seen, rgb=rgb, cam=cam, rot=rot)
    finder_name, finder_hit = _finder_patch(
        cues, pose=pose, body=body, roll=roll, pitch=pitch,
        tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
    )
    if candidate is not None:
        seg = _render_seg(renderer, data)
        name, u, v, mean, std = candidate
        hit_list = None
        if finder_name == name and finder_hit is not None:
            hit_list = [finder_hit[0], finder_hit[1]]
        sightings.append(ShadowSighting(
            patch=name,
            t=float(data.time),
            u=u,
            v=v,
            lum_mean=mean,
            lum_std=std,
            speckled=True,
            finder_emitted=finder_name == name,
            prop_in_frame=_prop_in_frame(seg, prop_ids),
            hit_xy=hit_list,
        ))
        seen.add(name)
    elif finder_name != "" and finder_hit is not None:
        _mark_finder(sightings, finder_name, finder_hit)
    path, hit, _clip = _latch_cues(
        cues, tracks, pose=pose, body=body, roll=roll, pitch=pitch,
        tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
        now=float(data.time), gate_m=gate_m,
    )
    return path, hit, rgb, cues


def _note_shadow(
    session: sw.SteerSession,
    model: mj.MjModel,
    renderer: mj.Renderer,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
    phase_floor: hf.SamePhaseFloor,
    sightings: list[ShadowSighting],
    seen: set[str],
    prop_ids: dict[int, str],
    rgb: np.ndarray,
) -> None:
    pose, body, yaw, pitch, roll, tilt, pan, step_off, cam = _pose_bits(
        session, model, cid, jid, pan_id, reach,
    )
    yaw_rate = float(session.bus.applied_yaw_rate)
    raw_cues = hf.corridor_gate_cues(
        hf.find_hazard_cues(rgb),
        cam=pose, body=body,
        imu_roll_rad=roll, imu_pitch_rad=pitch,
        head_tilt_rad=tilt, yaw_rate=yaw_rate,
        step_off_m=step_off, head_pan_rad=pan,
    )
    if session.lipm is not None and not phase_floor.has_prior(session.lipm.stance):
        raw_cues = hf.confirm_leg_columns(raw_cues, rgb)
    finder_name, finder_hit = _finder_patch(
        raw_cues, pose=pose, body=body, roll=roll, pitch=pitch,
        tilt=tilt, yaw_rate=yaw_rate, step_off=step_off, pan=pan,
    )
    rot = rc.camera_rotation_from_imu(yaw, pitch, roll, tilt, pan)
    candidate = _shadow_candidate(seen, rgb=rgb, cam=cam, rot=rot)
    if candidate is not None:
        seg = _render_seg(renderer, session.data)
        name, u, v, mean, std = candidate
        hit_list = None
        if finder_name == name and finder_hit is not None:
            hit_list = [finder_hit[0], finder_hit[1]]
        sightings.append(ShadowSighting(
            patch=name,
            t=float(session.data.time),
            u=u,
            v=v,
            lum_mean=mean,
            lum_std=std,
            speckled=True,
            finder_emitted=finder_name == name,
            prop_in_frame=_prop_in_frame(seg, prop_ids),
            hit_xy=hit_list,
        ))
        seen.add(name)
    elif finder_name != "" and finder_hit is not None:
        _mark_finder(sightings, finder_name, finder_hit)


def _look(
    session: sw.SteerSession,
    model: mj.MjModel,
    renderer: mj.Renderer,
    opening: apt.Box,
    room_name: str,
    question: str,
    asker: room_ask.RoomAsk | None,
    ask_error: str,
    *,
    tick: int,
    yaw_spawn: float,
    cid: int,
    jid: int,
    pan_id: int,
    reach: list[tuple[float, float, float]],
    phase_floor: hf.SamePhaseFloor,
    sightings: list[ShadowSighting],
    seen: set[str],
    prop_ids: dict[int, str],
) -> tuple[AskJson, dict[str, float]]:
    applied = float(session.bus.applied_yaw_rate)
    target = float(session.bus.target_yaw)
    if abs(applied) > 1e-6 or abs(target) > 1e-6:
        raise SystemExit(
            f"FAIL: question while yaw is commanded applied={applied:+.4f} "
            f"target={target:+.4f}"
        )
    heading_at_capture = _yaw(session.data, session.bid_body)
    t_capture = float(session.data.time)
    rgb = _render_rgb(renderer, session.data)
    _note_shadow(
        session, model, renderer, cid, jid, pan_id, reach, phase_floor,
        sightings, seen, prop_ids, rgb,
    )
    seg = _render_seg(renderer, session.data)
    fracs = _room_fractions(model, seg)
    door = _doorway_fraction(model, session.data, opening)
    door_detected = False
    door_u: float | None = None
    door_v: float | None = None
    door_frac: float | None = None
    door_bbox: list[float] | None = None
    door_seconds: float | None = None
    if asker is not None and door <= 0.0:
        # The opening is not in the frame. A detect box here is not a
        # doorway, and the room question is not asked.
        answer = "no_door"
        raw = ""
        confidence = None
        seconds = None
    elif asker is None:
        answer = "not loaded"
        raw = ask_error
        confidence = None
        seconds = None
    else:
        detected = asker.detect_doorway(rgb)
        door_seconds = float(detected["seconds"])
        boxes = _door_pixels(list(detected["objects"]))  # type: ignore[arg-type]
        if not boxes:
            answer = "no_door"
            raw = ""
            confidence = None
            seconds = door_seconds
        else:
            best = max(boxes, key=lambda item: float(item["frac"]))
            door_detected = True
            door_u = float(best["u"])
            door_v = float(best["v"])
            door_frac = float(best["frac"])
            door_bbox = [float(v) for v in best["bbox"]]  # type: ignore[union-attr]
            asked = asker.ask_yes_no(rgb, question)
            parsed = asked.get("answer")
            answer = "undecided" if parsed is None else str(parsed)
            raw = str(asked.get("raw", ""))
            conf = asked.get("confidence")
            confidence = None if conf is None else float(conf)
            sec = asked.get("seconds")
            seconds = None if sec is None else float(sec)
    if abs(_wrap(_yaw(session.data, session.bid_body) - heading_at_capture)) > 1e-6:
        raise SystemExit("FAIL: heading moved during the question")
    if abs(float(session.data.time) - t_capture) > 1e-9:
        raise SystemExit("FAIL: sim time advanced during the question")
    row = AskJson(
        t=t_capture,
        t_capture=t_capture,
        tick=tick,
        heading_rad=heading_at_capture,
        heading_at_capture=heading_at_capture,
        applied_yaw_at_capture=applied,
        yaw_from_spawn_rad=_wrap(heading_at_capture - yaw_spawn),
        doorway_fraction=door,
        asked_fraction=float(fracs.get(room_name, 0.0)),
        visible_rooms=_visible_rooms(fracs),
        answer=answer,
        raw=raw,
        confidence=confidence,
        seconds=seconds,
        door_detected=door_detected,
        door_u=door_u,
        door_v=door_v,
        door_frac=door_frac,
        door_bbox=door_bbox,
        door_seconds=door_seconds,
        search_applied_yaw=0.0,
        moondream_yes=False,
        frac_gate="na",
    )
    return row, fracs


def _place(session: sw.SteerSession, x: float, y: float, yaw: float) -> None:
    session.data.qpos[0] = float(x)
    session.data.qpos[1] = float(y)
    half = 0.5 * float(yaw)
    session.data.qpos[3] = math.cos(half)
    session.data.qpos[4] = 0.0
    session.data.qpos[5] = 0.0
    session.data.qpos[6] = math.sin(half)
    session.data.qvel[:] = 0.0
    mj.mj_forward(session.model, session.data)


def _fov_census(spec: dict[str, apt.RoomSpec]) -> list[dict[str, object]]:
    """Doorway fraction before any model call. kit_cam is not moved."""
    session = sw.SteerSession(
        video=False,
        scene_xml=SCENE_XML,
        initial_yaw=0.0,
        lipm=sw.locked_kit_config(),
    )
    model = session.model
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    rows: list[dict[str, object]] = []
    try:
        for name in ROOM_GEOMS:
            room = spec[name]
            door = room["doorways"][0]
            opening = door["opening"]
            spawn = door["spawn"]
            box = room["floor_box"]
            for deg in CENSUS_OFFSETS_DEG:
                yaw = float(spawn["yaw"]) + math.radians(deg)
                _place(session, float(spawn["x"]), float(spawn["y"]), yaw)
                seg = _render_seg(renderer, session.data)
                fracs = _room_fractions(model, seg)
                com = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64)
                outside = not (
                    box["xmin"] <= float(com[0]) <= box["xmax"]
                    and box["ymin"] <= float(com[1]) <= box["ymax"]
                )
                rows.append({
                    "room": name,
                    "offset_deg": deg,
                    "doorway_fraction": _doorway_fraction(model, session.data, opening),
                    "asked_fraction": float(fracs.get(name, 0.0)),
                    "visible_rooms": _visible_rooms(fracs),
                    "started_outside": outside,
                })
    finally:
        renderer.close()
    return rows


def _assert_census(rows: list[dict[str, object]]) -> None:
    by = {(str(row["room"]), int(row["offset_deg"])): row for row in rows}
    facing = by[("kitchen", 0)]
    left = by[("kitchen", 90)]
    right = by[("kitchen", -90)]
    if float(facing["doorway_fraction"]) < 0.20:
        raise SystemExit(
            f"FAIL: facing kitchen doorway fraction {facing['doorway_fraction']}"
        )
    for row in (left, right):
        if float(row["doorway_fraction"]) > 0.02:
            raise SystemExit(
                f"FAIL: kitchen offset {row['offset_deg']} still sees the doorway "
                f"({row['doorway_fraction']})"
            )
    if float(facing["asked_fraction"]) < ROOM_VISIBLE_FRAC:
        raise SystemExit("FAIL: facing kitchen furniture is under the visibility bar")
    if float(left["asked_fraction"]) >= ROOM_VISIBLE_FRAC:
        raise SystemExit("FAIL: kitchen at +90 deg is already in frame")
    for row in rows:
        if int(row["offset_deg"]) in (90, -90) and not bool(row["started_outside"]):
            raise SystemExit(f"FAIL: {row['room']} ±90 spawn is inside its box")
    if abs(HIP_BAR_NM - 2.33) > 1e-12 or abs(HIP_BAR_NM - lipm_gait.KNEE_SAG_NM) > 1e-12:
        raise SystemExit(f"FAIL: hip bar {HIP_BAR_NM} is not 2.33")


def _run_room(
    room_name: str,
    phrase: str,
    yaw_offset: float,
    spec_room: apt.RoomSpec,
    asker: room_ask.RoomAsk | None,
    ask_error: str,
    robot_bodies: set[str],
    d_min_m: float,
    latch_extra_m: float,
) -> RoomJson:
    if abs(latch_extra_m - LATCH_EXTRA_M) > 1e-12:
        raise SystemExit(f"FAIL: bout latch {latch_extra_m} is not the median")
    door = spec_room["doorways"][0]
    spawn_xy = door["spawn"]
    opening = door["opening"]
    face_yaw = float(spawn_xy["yaw"])
    box = FloorBox(
        source="room_apartment.json",
        xmin=spec_room["floor_box"]["xmin"],
        xmax=spec_room["floor_box"]["xmax"],
        ymin=spec_room["floor_box"]["ymin"],
        ymax=spec_room["floor_box"]["ymax"],
    )
    session = sw.SteerSession(
        video=False,
        scene_xml=SCENE_XML,
        initial_yaw=face_yaw + yaw_offset,
        lipm=sw.locked_kit_config(),
    )
    session.data.qpos[0] = float(spawn_xy["x"])
    session.data.qpos[1] = float(spawn_xy["y"])
    mj.mj_forward(session.model, session.data)
    model = session.model
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    pan_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_pan")
    if cid < 0 or jid < 0:
        raise SystemExit(f"FAIL {room_name}: kit_cam or head_tilt missing")
    prop_ids = _prop_geom_ids(model, robot_bodies)
    act_idx = {
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or "": i
        for i in range(model.nu)
    }
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    bout = _Bout()
    for _joint, actuator in HIP_ACTUATORS:
        if actuator not in act_idx:
            raise SystemExit(f"FAIL: missing actuator {actuator}")
        bout.hip_peak[actuator] = 0.0
    question = room_ask.yes_no_prompt(room_name)
    real = mj.mj_step

    def hook(step_model: mj.MjModel, step_data: mj.MjData) -> None:
        real(step_model, step_data)
        _watch_step(step_model, step_data, bout, robot_bodies, act_idx)
        _watch_tau(session, bout)

    mj.mj_step = hook
    reach: list[tuple[float, float, float]] = []
    tracks: list[gate._GapTrack] = []
    phase_floor = hf.SamePhaseFloor()
    sightings: list[ShadowSighting] = []
    seen: set[str] = set()
    asks: list[AskJson] = []
    approach: list[ApproachJson] = []
    wall_contacts: list[WallHitJson] = []
    wall_range_fail = False
    wall_stop_pass = False
    wall_stop_clear = False
    wall_stop_fail = False
    wall_stop_residual: float | None = None
    wall_stop_class = ""
    wall_stop_wall = ""
    wall_stop_heading: float | None = None
    wall_stop_cam_z: float | None = None
    wall_stop_pitch: float | None = None
    wall_stop_lead: float | None = None
    wall_stop_lead_side = ""
    wall_stop_pad: float | None = None
    east_l_hold = False
    walk_hold = False
    closest_ray: float | None = None
    closest_wall = ""
    closest_toe = ""
    closest_t: float | None = None
    closest_heading: float | None = None
    east_l_seen: list[tuple[float, float]] = []
    east_l_pad_n = 0
    other_near: dict[tuple[str, str], list[tuple[float, float]]] = {}
    other_near_n = 0
    e0_r_seen: list[tuple[float, float]] = []
    e0_close3: list[list[float]] = []
    near_rule = EAST_L_LIVE
    east_l_min_ray: float | None = None
    east_l_min_t: float | None = None
    east_l_min_heading: float | None = None
    east_l_min_cam_z: float | None = None
    east_l_min_pitch: float | None = None
    east_l_min_lead: float | None = None
    wall_bias_max: float | None = None
    wall_bias_count = 0
    wall_residual_max: float | None = None
    same_wall_count = 0
    surface_reject_count = 0
    class_reject_count = 0
    gate_m = d_min_m
    # Per wall. Living and east close classes add their own medians.
    wall_gate_m = d_min_m + latch_extra_m
    floor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    walls = _wall_ids(model)
    if floor_id < 0 or not walls:
        raise SystemExit("FAIL: floor or wall geoms missing")
    generic = voice.parse_phrase(phrase)
    words = sorted(set(voice.normalize_phrase(phrase).split()) & voice._ROOM_WORDS)
    stop = _empty_stop()
    commanded_vx: float | None = None
    commanded_yaw: float | None = None
    latch_applied = False
    committed = False
    search_applied_peak = 0.0
    ray_reject_count = 0
    wrong_yes = False
    block = ""
    ticks = 0
    yaw_turned = 0.0
    first_yes_t: float | None = None
    first_yes_tick: int | None = None
    first_yes_heading: float | None = None
    stand_door = 0.0
    stand_asked = 0.0
    stand_visible: list[str] = []
    try:
        while float(session.data.time) < STAND_S - 1e-9:
            session.bus.stand(float(session.data.time))
            bout.yaw_on = False
            session.step()
            ticks += 1
        com = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64)
        yaw_spawn = _yaw(session.data, session.bid_body)
        spawn = SpawnJson(
            doorway_x=float(spawn_xy["x"]),
            doorway_y=float(spawn_xy["y"]),
            doorway_yaw=face_yaw,
            qpos_xyz=[
                float(session.data.qpos[0]),
                float(session.data.qpos[1]),
                float(session.data.qpos[2]),
            ],
            yaw_rad=yaw_spawn,
            com_xyz=[float(com[0]), float(com[1]), float(com[2])],
            up_z=float(session.data.xmat[session.bid_body].reshape(3, 3)[2, 2]),
        )
        started_outside = not _inside(float(com[0]), float(com[1]), box)
        if not started_outside:
            block = "settled COM is inside the named room box"

        def consider(row: AskJson) -> str:
            nonlocal block, wrong_yes, committed, first_yes_t, first_yes_tick, first_yes_heading
            asks.append(row)
            moondream_yes = row["answer"] == "yes"
            row["moondream_yes"] = moondream_yes
            if not moondream_yes:
                row["frac_gate"] = "na"
                return row["answer"]
            if row["asked_fraction"] < ROOM_VISIBLE_FRAC:
                # Logged against the gate and treated as no. A later
                # picture under 1% does not cancel a yes that already
                # cleared the gate, and it does not publish a new commit.
                row["frac_gate"] = "refuse"
                return "no"
            row["frac_gate"] = "pass"
            if first_yes_t is None:
                first_yes_t = row["t"]
                first_yes_tick = row["tick"]
                first_yes_heading = row["heading_at_capture"]
            committed = True
            return "commit"

        if asker is None:
            block = "recogniser did not load, so no vel was published"
            look, _fracs = _look(
                session, model, renderer, opening, room_name, question,
                None, ask_error, tick=ticks, yaw_spawn=yaw_spawn,
                cid=cid, jid=jid, pan_id=pan_id, reach=reach,
                phase_floor=phase_floor, sightings=sightings, seen=seen,
                prop_ids=prop_ids,
            )
            asks.append(look)
            stand_door = look["doorway_fraction"]
            stand_asked = look["asked_fraction"]
            stand_visible = look["visible_rooms"]
        elif block == "":
            look, _fracs = _look(
                session, model, renderer, opening, room_name, question,
                asker, ask_error, tick=ticks, yaw_spawn=yaw_spawn,
                cid=cid, jid=jid, pan_id=pan_id, reach=reach,
                phase_floor=phase_floor, sightings=sightings, seen=seen,
                prop_ids=prop_ids,
            )
            stand_door = look["doorway_fraction"]
            stand_asked = look["asked_fraction"]
            stand_visible = look["visible_rooms"]
            decision = consider(look)
            prev_yaw = yaw_spawn
            last_capture = yaw_spawn
            next_ask = ASK_STEP_RAD

            def _search_fault() -> str:
                nonlocal block, stop, ticks, yaw_turned, prev_yaw
                if bout.contact != "none":
                    block = f"prop contact during search {bout.contact}"
                    stop = _Stop(
                        path="contact",
                        reason=bout.contact,
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
                    return "contact"
                com_now = np.asarray(
                    session.data.subtree_com[session.bid_body], dtype=np.float64,
                )
                if _inside(float(com_now[0]), float(com_now[1]), box):
                    block = (
                        "COM entered the named box during the in-place search, "
                        "before a go-to"
                    )
                    stop = _Stop(
                        path="search_inside",
                        reason=block,
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
                    return "inside"
                return ""

            sweep_base = yaw_turned
            approach_yaw = 0.0
            search_applied_peak = 0.0
            ray_reject_count = 0
            last_reject_t = -1.0
            last_reject_name = ""
            last_surface_t = -1.0
            last_surface_name = ""
            last_class_t = -1.0
            last_class_name = ""
            if decision == "commit":
                if look["door_u"] is not None:
                    approach_yaw = _yaw_toward_u(float(look["door_u"]))
                need_search = False
            else:
                need_search = decision in ("no", "undecided", "no_door")

            def _time_stop(reason: str) -> None:
                nonlocal block, stop
                block = reason
                stop = _Stop(
                    path="time",
                    reason=reason,
                    hit_xy=None,
                    on_shadow=False,
                    prop_in_frame=False,
                    false_stop=False,
                    latch_pass=False,
                )

            def _settle_body(during_search: bool) -> bool:
                nonlocal ticks, yaw_turned, prev_yaw, stop, block
                settle_end = float(session.data.time) + SETTLE_S
                while float(session.data.time) < settle_end - 1e-9:
                    if float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                        return False
                    now = float(session.data.time)
                    session.bus.vel(0.0, 0.0, now)
                    bout.yaw_on = True
                    bout.approach_yaw = False
                    session.step()
                    ticks += 1
                    heading = _yaw(session.data, session.bid_body)
                    yaw_turned += _wrap(heading - prev_yaw)
                    prev_yaw = heading
                    if during_search:
                        if _search_fault() != "":
                            return False
                        continue
                    if bout.contact != "none":
                        _log_range(bout.contact, "", ())
                        stop = _Stop(
                            path="contact",
                            reason=bout.contact,
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
                        return False
                    com_now = np.asarray(
                        session.data.subtree_com[session.bid_body], dtype=np.float64,
                    )
                    if _inside(float(com_now[0]), float(com_now[1]), box):
                        stop = _Stop(
                            path="inside",
                            reason="COM finished inside the named room box",
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
                        return False
                return float(session.data.time) < TIME_LIMIT_S - 1e-9

            def _capture() -> AskJson:
                return _look(
                    session, model, renderer, opening, room_name, question,
                    asker, ask_error, tick=ticks, yaw_spawn=yaw_spawn,
                    cid=cid, jid=jid, pan_id=pan_id, reach=reach,
                    phase_floor=phase_floor, sightings=sightings, seen=seen,
                    prop_ids=prop_ids,
                )[0]

            def _turn_toward(rate: float) -> bool:
                """In-place yaw toward the doorway, then settle. The walk stays at yaw 0."""
                nonlocal ticks, yaw_turned, prev_yaw, stop, block
                start = yaw_turned
                while (
                    abs(yaw_turned - start) < ASK_STEP_RAD
                    and float(session.data.time) < TIME_LIMIT_S - 1e-9
                ):
                    now = float(session.data.time)
                    session.bus.vel(0.0, rate, now)
                    bout.yaw_on = True
                    bout.approach_yaw = False
                    session.step()
                    ticks += 1
                    heading = _yaw(session.data, session.bid_body)
                    yaw_turned += _wrap(heading - prev_yaw)
                    prev_yaw = heading
                    if bout.contact != "none":
                        _log_range(bout.contact, "", ())
                        stop = _Stop(
                            path="contact",
                            reason=bout.contact,
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
                        return False
                if float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                    _time_stop(f"time limit {TIME_LIMIT_S:.1f} s")
                    return False
                return _settle_body(False)

            def _log_range(kind: str, latch_path: str, cues: tuple[hf.HazardCue, ...]) -> None:
                reading = _range_wall(
                    session, model, renderer, cid, jid, pan_id, reach, floor_id, walls,
                )
                true_name, true_gap = _true_wall_gap(model, session.data, session, cid)
                ranged_raw = reading["geometric_m"]
                ranged = None if ranged_raw is None else float(ranged_raw)
                same = _same_wall(reading, true_name)
                err = None
                if same and ranged is not None and true_gap is not None:
                    err = abs(ranged - float(true_gap))
                side = str(reading.get("lead_side") or "")
                base = _geom_class(true_name, true_gap) if same else "reject"
                klass = _toe_class(base, side) if same else "reject"
                if same and base == "east_close" and side == "L":
                    pad, defined = _east_l_bout_pad(_near_k_errs(east_l_seen))
                    residual = _residual_m(err, pad) if defined else None
                else:
                    pad = _pad_for(true_name, true_gap, side) if same else 0.0
                    e0_open = True
                    if same and true_name == E0_WALL and side == "R":
                        bout_pad, _bout_n, bout_defined = _e0_r_near_pad(e0_r_seen)
                        e0_open = bout_defined
                        if bout_defined:
                            pad = bout_pad
                    elif same and base == "other_wall_close" and side in ("L", "R"):
                        bout_pad, _bout_n, bout_defined = _other_near_pad(
                            other_near.get((true_name, side), []),
                        )
                        if bout_defined:
                            pad = bout_pad
                    residual = _residual_m(err, pad) if same and e0_open else None
                wall_contacts.append(_wall_hit(
                    float(session.data.time),
                    kind,
                    ranged,
                    true_gap,
                    true_name,
                    err,
                    False,
                    reading,
                    latch_path,
                    cues,
                    residual,
                    _yaw(session.data, session.bid_body),
                    klass,
                ))

            while (
                stop["path"] == ""
                and not wrong_yes
                and float(session.data.time) < TIME_LIMIT_S - 1e-9
                and (need_search or committed)
            ):
                if need_search:
                    turned_before = yaw_turned
                    segment_peak = 0.0
                    while (
                        yaw_turned - sweep_base < next_ask
                        and yaw_turned - sweep_base < SEARCH_SWEEP_RAD
                        and float(session.data.time) < TIME_LIMIT_S - 1e-9
                    ):
                        now = float(session.data.time)
                        session.bus.vel(0.0, SEARCH_YAW, now)
                        bout.yaw_on = True
                        bout.approach_yaw = False
                        session.step()
                        ticks += 1
                        applied_now = float(session.bus.applied_yaw_rate)
                        if abs(applied_now) >= abs(segment_peak):
                            segment_peak = applied_now
                        if abs(applied_now) >= abs(search_applied_peak):
                            search_applied_peak = applied_now
                        heading = _yaw(session.data, session.bid_body)
                        yaw_turned += _wrap(heading - prev_yaw)
                        prev_yaw = heading
                        if _search_fault() != "":
                            break
                    if stop["path"] != "":
                        break
                    if float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                        _time_stop("no yes before the time limit")
                        break
                    if yaw_turned - turned_before < 1e-3:
                        break
                    if yaw_turned - sweep_base >= SEARCH_SWEEP_RAD - 1e-6:
                        need_search = False
                        if not committed:
                            block = (
                                f"turned {yaw_turned - sweep_base:.3f} rad at yaw "
                                f"{SEARCH_YAW:+.2f} with no yes"
                            )
                            stop = _Stop(
                                path="no_yes",
                                reason=block,
                                hit_xy=None,
                                on_shadow=False,
                                prop_in_frame=False,
                                false_stop=False,
                                latch_pass=False,
                            )
                        break
                    if not _settle_body(True):
                        if stop["path"] == "" and float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                            _time_stop("no yes before the time limit")
                        break
                    if abs(_wrap(_yaw(session.data, session.bid_body) - last_capture)) < 0.5 * ASK_STEP_RAD:
                        break
                    look = _capture()
                    look["search_applied_yaw"] = segment_peak
                    next_ask = (yaw_turned - sweep_base) + ASK_STEP_RAD
                    last_capture = look["heading_at_capture"]
                    decision = consider(look)
                    if decision == "commit":
                        if look["door_u"] is not None:
                            approach_yaw = _yaw_toward_u(float(look["door_u"]))
                        else:
                            approach_yaw = 0.0
                        need_search = False
                    elif decision == "wrong":
                        break
                    continue

                if abs(approach_yaw) > 1e-6 and not _turn_toward(approach_yaw):
                    break
                if stop["path"] != "":
                    break
                commanded_vx = voice.FWD_MPS
                commanded_yaw = 0.0
                latch_applied = True
                look_end = float(session.data.time) + APPROACH_LOOK_S
                last_wall_t = -1.0
                last_cues: tuple[hf.HazardCue, ...] = ()
                repoint = True
                while float(session.data.time) < look_end - 1e-9:
                    now = float(session.data.time)
                    if now >= TIME_LIMIT_S - 1e-9:
                        break
                    path, hit, _frame, cues = _sense(
                        session, model, renderer, cid, jid, pan_id, reach, tracks,
                        phase_floor, gate_m, sightings, seen, prop_ids,
                    )
                    last_cues = cues
                    if now - last_wall_t >= WALL_SAMPLE_S - 1e-9:
                        last_wall_t = now
                        reading = _range_wall(
                            session, model, renderer, cid, jid, pan_id, reach,
                            floor_id, walls,
                        )
                        true_name, true_gap = _true_wall_gap(
                            model, session.data, session, cid,
                        )
                        ranged_g = (
                            None if reading["geometric_m"] is None
                            else float(reading["geometric_m"])
                        )
                        is_wall = str(true_name).startswith("wall_")
                        same = _same_wall(reading, true_name)
                        err = None
                        if same and ranged_g is not None and true_gap is not None:
                            err = abs(ranged_g - float(true_gap))
                        side = str(reading.get("lead_side") or "")
                        base = _geom_class(true_name, true_gap) if same else "reject"
                        klass = _toe_class(base, side) if same else "reject"
                        pad_defined = True
                        near_rule = EAST_L_LIVE
                        if same and base == "east_close" and side == "L":
                            near_errs = _near_k_errs(east_l_seen)
                            east_l_pad_n = len(near_errs)
                            pad, pad_defined = _east_l_bout_pad(near_errs)
                            residual = _residual_m(err, pad) if pad_defined else None
                        else:
                            pad = _pad_for(true_name, true_gap, side) if same else 0.0
                            if same and true_name == E0_WALL and side == "R":
                                bout_pad, bout_n, bout_defined = _e0_r_near_pad(e0_r_seen)
                                other_near_n = bout_n
                                near_rule = E0_R_LIVE
                                pad_defined = bout_defined
                                if bout_defined:
                                    pad = bout_pad
                            elif same and base == "other_wall_close" and side in ("L", "R"):
                                bout_pad, bout_n, bout_defined = _other_near_pad(
                                    other_near.get((true_name, side), []),
                                )
                                other_near_n = bout_n
                                near_rule = EAST_L_LIVE
                                if bout_defined:
                                    pad = bout_pad
                                    pad_defined = True
                            residual = _residual_m(err, pad) if same and pad_defined else None
                        heading = _yaw(session.data, session.bid_body)
                        approach.append(ApproachJson(
                            t=now,
                            tick=ticks,
                            heading_rad=heading,
                            commanded_vx=voice.FWD_MPS,
                            commanded_yaw=0.0,
                            door_u=None if look["door_u"] is None else float(look["door_u"]),
                            ranged_toe_gap_m=(
                                None if reading["toe_gap_m"] is None
                                else float(reading["toe_gap_m"])
                            ),
                            true_wall_gap_m=true_gap,
                            true_wall=true_name,
                            in_corridor=bool(reading["in_corridor"]),
                            cam_z_m=float(reading["cam_z_m"]),
                            imu_pitch_rad=float(reading["imu_pitch_rad"]),
                            head_tilt_rad=float(reading["head_tilt_rad"]),
                            cam_pitch_rad=float(reading["cam_pitch_rad"]),
                            geometric_m=ranged_g,
                            lead_off_m=float(reading["lead_off_m"]),
                            lead_side=str(reading["lead_side"]),
                            range_err_m=err,
                            residual_m=residual,
                        ))
                        if (
                            true_name == EAST_WALL
                            and side == "L"
                            and true_gap is not None
                            and (
                                east_l_min_ray is None
                                or float(true_gap) < east_l_min_ray
                            )
                        ):
                            east_l_min_ray = float(true_gap)
                            east_l_min_t = now
                            east_l_min_heading = heading
                            east_l_min_cam_z = float(reading["cam_z_m"])
                            east_l_min_pitch = float(reading["cam_pitch_rad"])
                            east_l_min_lead = float(reading["lead_off_m"])
                        if (
                            str(true_name).startswith("wall_")
                            and true_gap is not None
                            and (
                                closest_ray is None
                                or float(true_gap) < closest_ray
                            )
                        ):
                            closest_ray = float(true_gap)
                            closest_wall = true_name
                            closest_toe = side
                            closest_t = now
                            closest_heading = heading
                        if (
                            str(true_name).startswith("wall_")
                            and true_gap is not None
                            and float(true_gap) <= EAST_L_HOLD_M
                        ):
                            walk_hold = True
                            approach_yaw = 0.0
                        if (
                            true_name == EAST_WALL
                            and side == "L"
                            and true_gap is not None
                            and float(true_gap) <= EAST_L_HOLD_M
                        ):
                            east_l_hold = True
                            approach_yaw = 0.0
                        gap = reading["toe_gap_m"]
                        close_class = base in (
                            "living_close", "east_close", "other_wall_close",
                        )
                        if same and close_class:
                            same_wall_count += 1
                            if err is not None:
                                if wall_bias_max is None or err > wall_bias_max:
                                    wall_bias_max = err
                                if err > WALL_BIAS_M:
                                    wall_bias_count += 1
                            wall_contacts.append(_wall_hit(
                                now, "none", ranged_g, true_gap, true_name, err,
                                False, reading, "wall_ray", cues, residual,
                                heading, klass,
                            ))
                            gate = d_min_m + pad
                            # Right toe on wall_hall_e_1 has 9 samples.
                            # It cannot CLEAR, and it must not consume
                            # the walk before a left-toe close hit.
                            # East left with no prior close sample has
                            # no pad yet, so it does not latch.
                            crossed = (
                                gap is not None
                                and float(gap) <= gate
                                and klass != "east_close_R"
                                and pad_defined
                            )
                            if crossed:
                                within = _within_cm(residual)
                                ready = _toe_ready(klass)
                                clear = within and ready
                                wall_stop_clear = clear
                                wall_stop_fail = not within
                                wall_stop_pass = clear
                                wall_range_fail = not within
                                wall_stop_residual = residual
                                wall_stop_class = klass
                                wall_stop_wall = true_name
                                wall_stop_heading = heading
                                wall_stop_cam_z = float(reading["cam_z_m"])
                                wall_stop_pitch = float(reading["cam_pitch_rad"])
                                wall_stop_lead = float(reading["lead_off_m"])
                                wall_stop_lead_side = str(reading["lead_side"])
                                wall_stop_pad = pad
                                if residual is not None and (
                                    wall_residual_max is None
                                    or residual > wall_residual_max
                                ):
                                    wall_residual_max = residual
                                wall_contacts.append(_wall_hit(
                                    now, "floor_edge", ranged_g, true_gap,
                                    true_name, err, not clear, reading,
                                    "floor_edge", cues, residual, heading, klass,
                                ))
                                session.bus.stop(now)
                                bout.approach_yaw = False
                                bout.yaw_on = abs(session.bus.applied_yaw_rate) > 1e-6
                                session.step()
                                ticks += 1
                                if near_rule == E0_R_LIVE:
                                    e0_close3 = [
                                        [float(ray), float(err)]
                                        for ray, err in _e0_r_picked(e0_r_seen)
                                    ]
                                stop = _Stop(
                                    path="floor_edge",
                                    reason=(
                                        f"floor-edge wall {true_name} "
                                        f"class={klass} "
                                        f"toe_gap={float(gap):.3f} "
                                        f"ray={true_gap} "
                                        f"pad={pad:.5f} gate={gate:.3f} "
                                        f"online_n={other_near_n if klass == 'other_wall_close' else east_l_pad_n} "
                                        f"rule={near_rule} "
                                        f"residual={residual} "
                                        f"heading={heading:+.3f} "
                                        f"cam_z={float(reading['cam_z_m']):.3f} "
                                        f"cam_pitch={float(reading['cam_pitch_rad']):+.4f} "
                                        f"lead={float(reading['lead_off_m']):+.4f} "
                                        f"lead_side={reading['lead_side']}"
                                        + (
                                            " close3="
                                            + ",".join(
                                                f"{ray:.5f}/{err:.5f}"
                                                for ray, err in _e0_r_picked(e0_r_seen)
                                            )
                                            if near_rule == E0_R_LIVE
                                            else ""
                                        )
                                    ),
                                    hit_xy=None,
                                    on_shadow=False,
                                    prop_in_frame=False,
                                    false_stop=False,
                                    latch_pass=False,
                                )
                                repoint = False
                                break
                            if (
                                EAST_L_LIVE == "near_k"
                                and true_name == EAST_WALL
                                and side == "L"
                                and err is not None
                                and true_gap is not None
                                and float(true_gap) <= EAST_L_NEAR_GATE_M
                            ):
                                east_l_seen.append((float(true_gap), float(err)))
                            if (
                                EAST_L_LIVE == "near_k"
                                and true_name == E0_WALL
                                and side == "R"
                                and err is not None
                                and true_gap is not None
                                and float(true_gap) <= E0_R_NEAR_GATE_M
                            ):
                                e0_r_seen.append((float(true_gap), float(err)))
                            elif (
                                EAST_L_LIVE == "near_k"
                                and base == "other_wall_close"
                                and side in ("L", "R")
                                and err is not None
                                and true_gap is not None
                                and float(true_gap) <= EAST_L_NEAR_GATE_M
                            ):
                                other_near.setdefault((true_name, side), []).append(
                                    (float(true_gap), float(err)),
                                )
                        elif same:
                            class_reject_count += 1
                            if (
                                true_name != last_class_name
                                or now - last_class_t >= 0.5
                            ):
                                last_class_name = true_name
                                last_class_t = now
                                wall_contacts.append(_wall_hit(
                                    now, "none", ranged_g, true_gap, true_name,
                                    err, False, reading, "class_reject", cues,
                                    residual, heading, klass,
                                ))
                        elif is_wall:
                            surface_reject_count += 1
                            if (
                                true_name != last_surface_name
                                or now - last_surface_t >= 0.5
                            ):
                                last_surface_name = true_name
                                last_surface_t = now
                                miss = None
                                if ranged_g is not None and true_gap is not None:
                                    miss = abs(ranged_g - float(true_gap))
                                wall_contacts.append(_wall_hit(
                                    now, "none", ranged_g, true_gap, true_name,
                                    miss, False, reading, "surface_reject",
                                    cues, None, heading, "surface_reject",
                                ))
                        elif gap is not None and float(gap) <= d_min_m:
                            ray_reject_count += 1
                            if (
                                true_name != last_reject_name
                                or now - last_reject_t >= 0.5
                            ):
                                last_reject_name = true_name
                                last_reject_t = now
                                miss = None
                                if ranged_g is not None and true_gap is not None:
                                    miss = abs(ranged_g - float(true_gap))
                                wall_contacts.append(_wall_hit(
                                    now, "none", ranged_g, true_gap, true_name,
                                    miss, False, reading, "ray_reject", cues,
                                    None, heading, "ray_reject",
                                ))
                    if path != "":
                        seg = _render_seg(renderer, session.data)
                        prop = _prop_in_frame(seg, prop_ids)
                        stop = _score_stop(
                            path, path, hit,
                            model=model, data=session.data, prop_ids=prop_ids,
                            prop_in_frame=prop,
                        )
                        session.bus.stop(now)
                        bout.approach_yaw = False
                        bout.yaw_on = abs(session.bus.applied_yaw_rate) > 1e-6
                        session.step()
                        ticks += 1
                        repoint = False
                        break
                    if bout.contact != "none":
                        _log_range(bout.contact, "", last_cues)
                        stop = _Stop(
                            path="contact",
                            reason=bout.contact,
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
                        repoint = False
                        break
                    com_now = np.asarray(
                        session.data.subtree_com[session.bid_body], dtype=np.float64,
                    )
                    if _inside(float(com_now[0]), float(com_now[1]), box):
                        session.bus.stop(now)
                        bout.approach_yaw = False
                        bout.yaw_on = abs(session.bus.applied_yaw_rate) > 1e-6
                        session.step()
                        ticks += 1
                        stop = _Stop(
                            path="inside",
                            reason="COM finished inside the named room box",
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
                        repoint = False
                        break
                    bout.approach_yaw = False
                    bout.yaw_on = (
                        abs(session.bus.target_yaw) > 1e-6
                        or abs(session.bus.applied_yaw_rate) > 1e-6
                    )
                    session.bus.vel(voice.FWD_MPS, 0.0, now)
                    session.step()
                    ticks += 1
                    if bout.contact != "none":
                        _log_range(bout.contact, "", last_cues)
                        stop = _Stop(
                            path="contact",
                            reason=bout.contact,
                            hit_xy=None,
                            on_shadow=False,
                            prop_in_frame=False,
                            false_stop=False,
                            latch_pass=False,
                        )
                        repoint = False
                        break
                if stop["path"] != "":
                    break
                if float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                    _time_stop(f"time limit {TIME_LIMIT_S:.1f} s")
                    break
                if not repoint:
                    break
                if not _settle_body(False):
                    if stop["path"] == "" and float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                        _time_stop(f"time limit {TIME_LIMIT_S:.1f} s")
                    break
                look = _capture()
                last_capture = look["heading_at_capture"]
                decision = consider(look)
                if walk_hold:
                    approach_yaw = 0.0
                    need_search = False
                elif decision == "commit":
                    if look["door_u"] is not None:
                        approach_yaw = _yaw_toward_u(float(look["door_u"]))
                    else:
                        approach_yaw = 0.0
                    need_search = False
                elif decision == "wrong":
                    break
                else:
                    need_search = True
                    approach_yaw = 0.0
                    sweep_base = yaw_turned
                    next_ask = ASK_STEP_RAD

            if not committed and not wrong_yes and block == "" and stop["path"] == "":
                if float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                    _time_stop("no yes before the time limit")
                else:
                    block = (
                        f"turned {yaw_turned:.3f} rad at yaw {SEARCH_YAW:+.2f} "
                        "with no yes"
                    )
                    stop = _Stop(
                        path="no_yes",
                        reason=block,
                        hit_xy=None,
                        on_shadow=False,
                        prop_in_frame=False,
                        false_stop=False,
                        latch_pass=False,
                    )
            if wrong_yes and stop["path"] == "":
                stop = _Stop(
                    path="wrong_yes",
                    reason=block,
                    hit_xy=None,
                    on_shadow=False,
                    prop_in_frame=False,
                    false_stop=False,
                    latch_pass=False,
                )
            if stop["path"] == "" and float(session.data.time) >= TIME_LIMIT_S - 1e-9:
                _time_stop(f"time limit {TIME_LIMIT_S:.1f} s")
        if bout.peak_actuator == "":
            raise SystemExit(f"FAIL {room_name}: no leg actuator force")
        for row in asks:
            under = (
                row["answer"] == "yes"
                and float(row["asked_fraction"]) < ROOM_VISIBLE_FRAC
            )
            if under and row["frac_gate"] != "refuse":
                wrong_yes = True
                if block == "":
                    block = (
                        f"wrong yes slipped the frac gate; {room_name} covers "
                        f"{row['asked_fraction']:.3f} of kit_cam"
                    )
            if under and committed and row["frac_gate"] == "pass":
                wrong_yes = True
                if block == "":
                    block = (
                        f"wrong yes committed under the frac gate; {room_name} "
                        f"covers {row['asked_fraction']:.3f} of kit_cam"
                    )
        com_end = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64)
        finished_inside = _inside(float(com_end[0]), float(com_end[1]), box)
        min_up = float(session.min_up_z)
        t_end = float(session.data.time)
        upright = min_up >= UPRIGHT_UP_Z
        clean = bout.contact == "none"
        in_time = t_end <= TIME_LIMIT_S + 1e-6
        reached = (
            started_outside
            and finished_inside
            and upright
            and clean
            and in_time
            and committed
            and not wrong_yes
        )
        if reached:
            block = ""
        elif block == "" and not finished_inside:
            block = f"COM did not finish inside the named box ({stop['path']})"
        elif block == "" and not upright:
            block = f"min up_z {min_up:.3f} is under {UPRIGHT_UP_Z:.2f}"
        elif block == "" and not clean:
            block = f"prop contact {bout.contact}"
        label = "not asked"
        raw_answer = ""
        confidence: float | None = None
        seconds: float | None = None
        if asks:
            chosen = None
            for row in asks:
                if row["answer"] == "yes" and row["frac_gate"] == "pass":
                    chosen = row
                    break
            if chosen is None:
                for row in reversed(asks):
                    if row["frac_gate"] != "refuse":
                        chosen = row
                        break
            if chosen is None:
                chosen = asks[-1]
                label = "no"
            else:
                label = chosen["answer"]
            raw_answer = chosen["raw"]
            confidence = chosen["confidence"]
            seconds = chosen["seconds"]
        voice_line = generic.line
        voice_parse = generic.kind
        if commanded_vx is not None and commanded_yaw is not None:
            voice_line = voice.bus_text("vel", commanded_vx, commanded_yaw)
            voice_parse = "vel"
        hit_list = None if stop["hit_xy"] is None else [stop["hit_xy"][0], stop["hit_xy"][1]]
        prop_at_stop: bool | None = None
        if stop["path"] in ("shortest", "too_close"):
            prop_at_stop = stop["prop_in_frame"]
        return RoomJson(
            room=room_name,
            scene=SCENE_XML.name,
            phrase=phrase,
            question=question,
            yaw_offset_rad=yaw_offset,
            voice_parse=voice_parse,
            voice_line=voice_line,
            generic_caller=generic.line,
            matched_room_words=words,
            recogniser_label=label,
            recogniser_raw=raw_answer,
            recogniser_confidence=confidence,
            recogniser_seconds=seconds,
            commanded_vx=commanded_vx,
            commanded_yaw=commanded_yaw,
            search_yaw_rate=SEARCH_YAW,
            yaw_turned_rad=yaw_turned,
            first_yes_t=first_yes_t,
            first_yes_tick=first_yes_tick,
            first_yes_heading_rad=first_yes_heading,
            wrong_yes=wrong_yes,
            committed=committed,
            d_min_m=d_min_m,
            d_min_v_mps=float(rc.V_MPS),
            t_detect_s=T_DETECT_S,
            t_stop_s=float(rc.T_STOP_S),
            latch_applied=latch_applied,
            latch_pass=stop["latch_pass"],
            false_stop=stop["false_stop"],
            prop_in_frame_at_stop=prop_at_stop,
            stop_path=stop["path"],
            stop_reason=stop["reason"],
            stop_hit_xy=hit_list,
            stop_on_shadow=stop["on_shadow"],
            shadow_first_sightings=sightings,
            doorway_fraction=stand_door,
            asked_fraction=stand_asked,
            visible_rooms=stand_visible,
            asks=asks,
            approach=approach,
            wall_contacts=wall_contacts,
            wall_range_fail=wall_range_fail,
            wall_stop_pass=wall_stop_pass,
            wall_stop_clear=wall_stop_clear,
            reach_clear=reached,
            wall_stop_fail=wall_stop_fail,
            wall_stop_residual_m=wall_stop_residual,
            wall_stop_class=wall_stop_class,
            wall_stop_wall=wall_stop_wall,
            wall_stop_heading_rad=wall_stop_heading,
            wall_stop_cam_z_m=wall_stop_cam_z,
            wall_stop_pitch_rad=wall_stop_pitch,
            wall_stop_lead_m=wall_stop_lead,
            wall_stop_lead_side=wall_stop_lead_side,
            wall_stop_pad_m=wall_stop_pad,
            e0_r_close3=e0_close3,
            east_left=EastLeftFact(
                hold=east_l_hold,
                presented=(
                    east_l_min_ray is not None
                    and east_l_min_ray <= CLOSE_GAP_M
                ),
                min_ray_m=east_l_min_ray,
                t=east_l_min_t,
                heading_rad=east_l_min_heading,
                cam_z_m=east_l_min_cam_z,
                cam_pitch_rad=east_l_min_pitch,
                lead_off_m=east_l_min_lead,
                lead_side="L" if east_l_min_ray is not None else "",
                online_n=east_l_pad_n,
                pad_rule=EAST_L_LIVE,
            ),
            approach_band=ApproachBand(
                hold=walk_hold,
                hold_m=EAST_L_HOLD_M,
                closest_ray_m=closest_ray,
                closest_wall=closest_wall,
                closest_toe=closest_toe,
                closest_t=closest_t,
                closest_heading_rad=closest_heading,
                entered_0_30=(
                    closest_ray is not None and closest_ray <= EAST_L_NEAR_GATE_M
                ),
            ),
            wall_residual_max_m=wall_residual_max,
            same_wall_count=same_wall_count,
            surface_reject_count=surface_reject_count,
            ray_reject_count=ray_reject_count,
            class_reject_count=class_reject_count,
            frac_refuse_count=sum(1 for row in asks if row["frac_gate"] == "refuse"),
            wall_bias_max_m=wall_bias_max,
            wall_bias_count=wall_bias_count,
            wall_bias_repeat=wall_bias_count >= 2,
            search_applied_yaw_peak=search_applied_peak,
            search_yaw_fail=(
                any(row["answer"] == "no_door" for row in asks)
                and abs(search_applied_peak) < 0.24
            ),
            latch_armed_contact=bool(latch_applied and bout.contact != "none"),
            gait_limit=bout.gait_limit,
            inplace_hip_fail=bout.inplace_hip_fail,
            inplace_hip=_inplace_rows(bout),
            torque=_torque_rows(bout),
            spawn=spawn,
            floor_box=box,
            started_outside=started_outside,
            finished_inside=finished_inside,
            reached=reached,
            reach_block=block,
            contact=bout.contact,
            peak_nm=bout.peak_nm,
            peak_actuator=bout.peak_actuator,
            peak_joint=_joint_name(bout.peak_actuator),
            hip_while_yawing=_hip_rows(bout),
            min_up_z=min_up,
            t_end=t_end,
            end_qpos_xyz=[
                float(session.data.qpos[0]),
                float(session.data.qpos[1]),
                float(session.data.qpos[2]),
            ],
            end_yaw_rad=_yaw(session.data, session.bid_body),
        )
    finally:
        mj.mj_step = real
        renderer.close()


def _torque_rows(bout: _Bout) -> list[TorqueJson]:
    names = sorted(set(bout.tau_unclamped) | set(bout.measured) | set(bout.approach_unclamped))
    rows: list[TorqueJson] = []
    for joint in names:
        if "knee" not in joint and "hip_roll" not in joint:
            continue
        approach_u = bout.approach_unclamped.get(joint)
        approach_c = bout.approach_clamped.get(joint)
        rows.append(TorqueJson(
            joint=joint,
            unclamped_nm=float(bout.tau_unclamped.get(joint, 0.0)),
            clamped_nm=float(bout.tau_clamped.get(joint, 0.0)),
            measured_nm=float(bout.measured.get(joint, 0.0)),
            approach_unclamped_nm=None if approach_u is None else float(approach_u),
            approach_clamped_nm=None if approach_c is None else float(approach_c),
        ))
    return rows


def _place_pitched(session: sw.SteerSession, x: float, y: float, yaw: float, pitch: float) -> None:
    """Root pose at this yaw and IMU pitch. Legs stay at the stand qpos."""
    _place(session, x, y, yaw)
    rot = rc.body_rotation(yaw, pitch, 0.0)
    quat = np.zeros(4, dtype=np.float64)
    mj.mju_mat2Quat(quat, np.ascontiguousarray(rot.reshape(9)))
    session.data.qpos[3:7] = quat
    session.data.qvel[:] = 0.0
    mj.mj_forward(session.model, session.data)


def _range_probe() -> dict[str, object]:
    """Live-pose wall range before any room score. Not a standing table.

    Samples a settled stand, a short walk, and root pitches of a few
    milliradians plus 0.02 rad.     The settled-stand error is logged. It is not added to latch_extra.
    The bout pad is the living close-wall median, and only on that wall.
    """
    session = sw.SteerSession(
        video=False,
        scene_xml=SCENE_XML,
        initial_yaw=0.0,
        lipm=sw.locked_kit_config(),
    )
    model = session.model
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
    pan_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "head_pan")
    floor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    walls = _wall_ids(model)
    reach: list[tuple[float, float, float]] = []
    samples: list[dict[str, object]] = []

    def take(tag: str) -> None:
        reading = _range_wall(
            session, model, renderer, cid, jid, pan_id, reach, floor_id, walls,
        )
        true_name, true_gap = _true_wall_gap(model, session.data, session, cid)
        ranged = reading["geometric_m"]
        err = None
        if ranged is not None and true_gap is not None:
            err = abs(float(ranged) - float(true_gap))
        samples.append({
            "tag": tag,
            "ranged_m": None if ranged is None else float(ranged),
            "toe_gap_m": None if reading["toe_gap_m"] is None else float(reading["toe_gap_m"]),
            "true_wall_gap_m": true_gap,
            "true_wall": true_name,
            "range_err_m": err,
            "cam_z_m": float(reading["cam_z_m"]),
            "imu_pitch_rad": float(reading["imu_pitch_rad"]),
            "head_tilt_rad": float(reading["head_tilt_rad"]),
            "cam_pitch_rad": float(reading["cam_pitch_rad"]),
            "lead_off_m": float(reading["lead_off_m"]),
            "step_off_m": float(reading["step_off_m"]),
        })

    try:
        _place(session, 0.40, -1.05, 0.0)
        while float(session.data.time) < STAND_S - 1e-9:
            session.bus.stand(float(session.data.time))
            session.step()
        take("stand")
        walk_end = float(session.data.time) + 1.2
        next_take = float(session.data.time)
        while float(session.data.time) < walk_end - 1e-9:
            now = float(session.data.time)
            session.bus.vel(voice.FWD_MPS, 0.0, now)
            session.step()
            if now + 1e-9 >= next_take:
                take("walk")
                next_take = now + 0.4
        for pitch in (0.005, -0.005, 0.02, -0.02):
            _place_pitched(session, 0.40, -1.05, 0.0, pitch)
            take(f"pitch {pitch:+.3f}")
    finally:
        renderer.close()
    errs = [
        float(row["range_err_m"])
        for row in samples
        if row["range_err_m"] is not None and str(row["true_wall"]).startswith("wall_")
    ]
    worst = None if not errs else max(errs)
    stand_err = next(
        (row["range_err_m"] for row in samples if row["tag"] == "stand"),
        None,
    )
    # The old bob pad was this walk worst when it exceeded 1 cm.
    # It is not the bout latch. Stacking it on the same-wall median
    # would count the live bob twice.
    bob = 0.0 if worst is None or worst <= RANGE_ERR_MAX_M else float(worst)
    return {
        "where": "x=0.40 y=-1.05 yaw=0 facing wall_hall_e",
        "range_err_max_m": RANGE_ERR_MAX_M,
        "worst_abs_err_m": worst,
        "stand_err_m": stand_err,
        "bob_pad_m": bob,
        "bob_pad_stacked": False,
        "uses_live_cam_height_and_imu_pitch": True,
        "standing_table_not_used": True,
        "samples": samples,
    }


def _write(payload: dict[str, object]) -> None:
    SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
    SUMMARY_PATH.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _load_asker() -> tuple[room_ask.RoomAsk | None, str]:
    try:
        return room_ask.RoomAsk(), ""
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"


def main() -> int:
    _speckle_self_check()
    before = _plant_md5()
    if before != sw.PLANT_MD5 or before != voice.PLANT_MD5:
        raise SystemExit(f"FAIL: plant md5 {before}")
    if abs(voice.FWD_MPS - 0.056) > 1e-12:
        raise SystemExit(f"FAIL: voice forward cap moved to {voice.FWD_MPS}")
    if abs(voice.YAW_RAD_S - 0.25) > 1e-12:
        raise SystemExit(f"FAIL: voice yaw cap moved to {voice.YAW_RAD_S}")
    if abs(SEARCH_YAW - 0.25) > 1e-12:
        raise SystemExit(f"FAIL: search yaw moved to {SEARCH_YAW}")
    if abs(RANGE_ERR_MAX_M - 0.01) > 1e-12:
        raise SystemExit(f"FAIL: range error bar moved to {RANGE_ERR_MAX_M}")
    if len(SAME_WALL_ERR_M) != 17:
        raise SystemExit(f"FAIL: same-wall set has {len(SAME_WALL_ERR_M)} samples")
    if abs(LATCH_EXTRA_M - 0.04086979572280322) > 1e-12:
        raise SystemExit(f"FAIL: latch median moved to {LATCH_EXTRA_M}")
    if len(EAST_CLOSE) != 43:
        raise SystemExit(f"FAIL: east close set has {len(EAST_CLOSE)} samples")
    if any(row[5] > CLOSE_GAP_M + 1e-12 for row in EAST_CLOSE):
        raise SystemExit("FAIL: east close set includes a ray past 0.40 m")
    if abs(EAST_LATCH_M - 0.03983016052991295) > 1e-12:
        raise SystemExit(f"FAIL: east median moved to {EAST_LATCH_M}")
    if abs(EAST_LATCH_M - LATCH_EXTRA_M) < 1e-4:
        raise SystemExit("FAIL: east median copied the living median")
    if len(EAST_L_ERR_M) != 45:
        raise SystemExit(f"FAIL: east left set has {len(EAST_L_ERR_M)} samples")
    if abs(EAST_L_LATCH_M - 0.045424805087740217) > 1e-12:
        raise SystemExit(f"FAIL: east left median moved to {EAST_L_LATCH_M}")
    if abs(EAST_L_MAD_M - 0.019643518807211546) > 1e-12:
        raise SystemExit(f"FAIL: east left MAD moved to {EAST_L_MAD_M}")
    if abs(EAST_L_APPLIED_M - 0.06506832389495176) > 1e-12:
        raise SystemExit(f"FAIL: east left applied pad moved to {EAST_L_APPLIED_M}")
    if EAST_L_MAD_APPLIED:
        raise SystemExit("FAIL: east left median+MAD is on the live latch")
    if EAST_L_LIVE != "near_k":
        raise SystemExit(f"FAIL: east left live rule {EAST_L_LIVE}")
    if abs(EAST_L_NEAR_GATE_M - 0.30) > 1e-12:
        raise SystemExit(f"FAIL: east left near gate moved to {EAST_L_NEAR_GATE_M}")
    if EAST_L_NEAR_GATE_M >= CLOSE_GAP_M:
        raise SystemExit("FAIL: east left near gate is the fat close class")
    if EAST_L_NEAR_K != 5:
        raise SystemExit(f"FAIL: east left near K moved to {EAST_L_NEAR_K}")
    if EAST_L_NEWMIN_N != 6 or abs(EAST_L_NEWMIN_PAD_M - 0.02856695273723686) > 1e-12:
        raise SystemExit("FAIL: the new-closest failure moved")
    if abs(EAST_L_NEWMIN_RESIDUAL_M) <= RANGE_ERR_MAX_M:
        raise SystemExit("FAIL: the new-closest residual was marked clear")
    near_demo = _near_k_errs([
        (0.35, 0.09),
        (0.29, 0.01),
        (0.28, 0.02),
        (0.27, 0.03),
        (0.26, 0.04),
        (0.25, 0.05),
        (0.24, 0.06),
    ])
    if near_demo != [0.02, 0.03, 0.04, 0.05, 0.06]:
        raise SystemExit(f"FAIL: near-k set moved to {near_demo}")
    if abs(E0_R_NEAR_GATE_M - EAST_L_NEAR_GATE_M) > 1e-12:
        raise SystemExit(f"FAIL: e0 right near gate moved to {E0_R_NEAR_GATE_M}")
    if E0_R_NEAR_K != EAST_L_NEAR_K or E0_R_NEAR_K != 5:
        raise SystemExit(f"FAIL: e0 right near K moved to {E0_R_NEAR_K}")
    if E0_R_NEAR_MIN_N != 5:
        raise SystemExit(f"FAIL: e0 right min N moved to {E0_R_NEAR_MIN_N}")
    if E0_R_LIVE != "e0_r_close3":
        raise SystemExit(f"FAIL: e0 right live rule {E0_R_LIVE}")
    if E0_R_CLOSE_N != 3:
        raise SystemExit(f"FAIL: e0 right close count moved to {E0_R_CLOSE_N}")
    if E0_R_FAIL_N != 5 or abs(E0_R_FAIL_PAD_M - 0.0439108831389442) > 1e-12:
        raise SystemExit("FAIL: the e0 right 0.30 m miss moved")
    if abs(E0_R_FAIL_RESIDUAL_M) <= RANGE_ERR_MAX_M:
        raise SystemExit("FAIL: the e0 right overshoot was marked clear")
    if abs(E0_R_FAIL_ERR_M - (E0_R_FAIL_PAD_M + E0_R_FAIL_RESIDUAL_M)) > 1e-12:
        raise SystemExit("FAIL: e0 right error does not match the residual")
    if E0_R_THIN_N != 2 or abs(E0_R_THIN_PAD_M - 0.029336655418528076) > 1e-12:
        raise SystemExit("FAIL: the e0 right thin miss moved")
    if abs(E0_R_THIN_RESIDUAL_M) <= RANGE_ERR_MAX_M:
        raise SystemExit("FAIL: the e0 right thin residual was marked clear")
    if abs(E0_R_THIN_ERR_M - (E0_R_THIN_PAD_M + E0_R_THIN_RESIDUAL_M)) > 1e-12:
        raise SystemExit("FAIL: e0 right thin error does not match the residual")
    e0_short_pad, e0_short_n, e0_short_on = _e0_r_near_pad([
        (0.28, 0.054),
        (0.26, 0.044),
    ])
    if e0_short_on or e0_short_n != 2 or e0_short_pad != 0.0:
        raise SystemExit(
            f"FAIL: short e0 window latched {e0_short_pad} n={e0_short_n}"
        )
    e0_demo_pad, e0_demo_n, e0_demo_on = _e0_r_near_pad([
        (0.28, 0.054),
        (0.26, 0.044),
        (0.24, 0.033),
        (0.23, 0.026),
        (0.22, 0.025),
    ])
    if not e0_demo_on or e0_demo_n != 3 or abs(e0_demo_pad - 0.026) > 1e-12:
        raise SystemExit(f"FAIL: e0 right near pad moved to {e0_demo_pad} n={e0_demo_n}")
    if abs(e0_demo_pad - 0.033) < 1e-12:
        raise SystemExit("FAIL: e0 right pad used the full last-5 median")
    e0_demo_pick = _e0_r_picked([
        (0.28, 0.054),
        (0.26, 0.044),
        (0.24, 0.033),
        (0.23, 0.026),
        (0.22, 0.025),
    ])
    if e0_demo_pick != [(0.22, 0.025), (0.23, 0.026), (0.24, 0.033)]:
        raise SystemExit(f"FAIL: e0 right close3 moved to {e0_demo_pick}")
    if abs(e0_demo_pad - E0_R_FAIL_PAD_M) < 1e-3:
        raise SystemExit("FAIL: e0 right near pad copied the 0.04391 miss")
    if abs(e0_demo_pad - E0_R_THIN_PAD_M) < 1e-3:
        raise SystemExit("FAIL: e0 right near pad copied the thin miss")
    e0_order_pad, e0_order_n, e0_order_on = _e0_r_near_pad([
        (0.21, 0.040),
        (0.30, 0.080),
        (0.29, 0.070),
        (0.28, 0.060),
        (0.27, 0.050),
    ])
    e0_order_pick = _e0_r_picked([
        (0.21, 0.040),
        (0.30, 0.080),
        (0.29, 0.070),
        (0.28, 0.060),
        (0.27, 0.050),
    ])
    if (
        not e0_order_on
        or e0_order_n != 3
        or abs(e0_order_pad - 0.050) > 1e-12
        or e0_order_pick != [(0.21, 0.040), (0.27, 0.050), (0.28, 0.060)]
    ):
        raise SystemExit(
            f"FAIL: e0 close3 followed recency {e0_order_pad} {e0_order_pick}"
        )
    e0_cap_pad, e0_cap_n, e0_cap_on = _e0_r_near_pad([
        (0.20, 0.010),
        (0.28, 0.054),
        (0.26, 0.044),
        (0.24, 0.033),
        (0.23, 0.026),
        (0.22, 0.025),
    ])
    if not e0_cap_on or e0_cap_n != 3 or abs(e0_cap_pad - e0_demo_pad) > 1e-12:
        raise SystemExit(f"FAIL: e0 right cap moved to {e0_cap_pad} n={e0_cap_n}")
    if abs(e0_cap_pad - 0.025) < 1e-12:
        raise SystemExit("FAIL: e0 close3 reached outside the last-5 window")
    e0_far_pad, e0_far_n, e0_far_on = _e0_r_near_pad([
        (0.31, 0.090),
        (0.29, 0.050),
        (0.28, 0.048),
        (0.27, 0.046),
        (0.26, 0.044),
    ])
    if e0_far_on or e0_far_n != 4:
        raise SystemExit(f"FAIL: ray past 0.30 filled the e0 window n={e0_far_n}")
    if abs(EAST_L_ONLINE_RESIDUAL_M) <= RANGE_ERR_MAX_M:
        raise SystemExit("FAIL: the failed online residual was marked clear")
    if EAST_L_ONLINE_N != 22:
        raise SystemExit("FAIL: online prior count moved")
    if abs(EAST_L_ONLINE_PAD_M - 0.05612963080175057) > 1e-12:
        raise SystemExit("FAIL: online pad moved")
    if abs(EAST_L_Q75_RESIDUAL_M - -0.03175107940399083) > 1e-12:
        raise SystemExit("FAIL: east left q75 residual moved")
    if abs(EAST_L_Q75_ERR_M - (EAST_L_Q75_M + EAST_L_Q75_RESIDUAL_M)) > 1e-12:
        raise SystemExit("FAIL: east left q75 error does not match the residual")
    if abs(EAST_L_Q75_RESIDUAL_M) <= RANGE_ERR_MAX_M:
        raise SystemExit("FAIL: the q75 residual was marked clear")
    if abs(EAST_L_Q75_M - 0.05944376400913556) > 1e-12:
        raise SystemExit(f"FAIL: east left q75 moved to {EAST_L_Q75_M}")
    if len(W2_L_ERR_M) != 19:
        raise SystemExit(f"FAIL: living left set has {len(W2_L_ERR_M)} samples")
    if abs(W2_L_LATCH_M - 0.010625936072531067) > 1e-12:
        raise SystemExit(f"FAIL: living left median moved to {W2_L_LATCH_M}")
    if abs(W2_L_LATCH_M - EAST_L_APPLIED_M) < 1e-3 or abs(W2_L_LATCH_M - EAST_L_LATCH_M) < 1e-3:
        raise SystemExit("FAIL: living left pad copied an east pad")
    if abs(W2_L_LATCH_M - LATCH_EXTRA_M) < 1e-3:
        raise SystemExit("FAIL: living left pad copied the right-toe pad")
    if EAST_L_FIT_APPLIED:
        raise SystemExit("FAIL: east left additive correction was applied")
    fit_pred = (
        EAST_L_FIT_C0
        + EAST_L_FIT_CZ * EAST_L_HELD_CAM_Z_M
        + EAST_L_FIT_CPITCH * EAST_L_HELD_PITCH_RAD
        + EAST_L_FIT_CLEAD * EAST_L_HELD_LEAD_M
    )
    fit_resid = EAST_L_HELD_ERR_M - fit_pred
    if abs(fit_resid - 0.02515308966231159) > 1e-9:
        raise SystemExit(f"FAIL: east left fit residual moved to {fit_resid}")
    if abs((EAST_L_HELD_ERR_M - EAST_L_LATCH_M) - 0.013992083995524984) > 1e-12:
        raise SystemExit("FAIL: east left held-out residual moved")
    if abs(EAST_R_LATCH_M - 0.07279199541829862) > 1e-12:
        raise SystemExit(f"FAIL: east right median moved to {EAST_R_LATCH_M}")
    if len(EAST_R_ERR_M) != 9:
        raise SystemExit("FAIL: east right set changed size")
    if _pad_for(EAST_WALL, 0.20) != 0.0:
        raise SystemExit("FAIL: east unknown toe took the pooled pad")
    if _pad_for(EAST_WALL, 0.20, "L") != 0.0:
        raise SystemExit("FAIL: east left frozen pad is on the live path")
    if _pad_for(EAST_WALL, 0.20, "R") != EAST_R_LATCH_M:
        raise SystemExit("FAIL: east right pad is not the right median")
    if _east_l_bout_pad([]) != (0.0, False):
        raise SystemExit("FAIL: empty east left bout took a frozen pad")
    if _east_l_bout_pad([0.0265, 0.0294, 0.0309]) != (0.0294, True):
        raise SystemExit("FAIL: east left near median moved")
    if _east_l_bout_pad([0.05613]) == (EAST_L_Q75_M, True):
        raise SystemExit("FAIL: east left near pad fell back to the 75th percentile")
    if abs(EAST_L_LATCH_M - LATCH_EXTRA_M) < 1e-3 or abs(EAST_R_LATCH_M - LATCH_EXTRA_M) < 1e-3:
        raise SystemExit("FAIL: a toe median copied the living pad")
    if _pad_for(EAST_WALL, 1.0, "L") != 0.0:
        raise SystemExit("FAIL: far east ray took a pad")
    if _pad_for("wall_hall_e_0", 0.20, "L") != 0.0:
        raise SystemExit("FAIL: an unmeasured close wall took a frozen pad")
    if _pad_for("wall_hall_e_0", 0.20, "R") != 0.0:
        raise SystemExit("FAIL: east-0 right borrowed a frozen pad")
    if _pad_for("wall_hall_e_2", 0.20, "L") != 0.0 or _pad_for("wall_hall_w_1", 0.20, "R") != 0.0:
        raise SystemExit("FAIL: another wall borrowed a frozen pad")
    other_pad, other_n, other_defined = _other_near_pad([
        (0.35, 0.09),
        (0.29, 0.01),
        (0.28, 0.03),
        (0.22, 0.05),
    ])
    if not other_defined or other_n != 3 or abs(other_pad - 0.03) > 1e-12:
        raise SystemExit(f"FAIL: other-wall near pad moved to {other_pad} n={other_n}")
    if _other_near_pad([]) != (0.0, 0, False):
        raise SystemExit("FAIL: empty other-wall window invented a pad")
    if _other_near_pad([(0.31, 0.04)]) != (0.0, 0, False):
        raise SystemExit("FAIL: a ray past the near gate entered the other-wall pad")
    if abs(_other_near_pad([(0.29, EAST_L_LATCH_M), (0.28, W2_R_LATCH_M)])[0] - statistics.median([EAST_L_LATCH_M, W2_R_LATCH_M])) > 1e-12:
        raise SystemExit("FAIL: other-wall near pad is not its own median")
    if _pad_for(LIVING_WALL, 0.20, "L") != W2_L_LATCH_M:
        raise SystemExit("FAIL: living left pad is not the left median")
    if len(W2_R_ERR_M) != 49:
        raise SystemExit(f"FAIL: living right set has {len(W2_R_ERR_M)} samples")
    if abs(W2_R_LATCH_M - 0.04936891275435906) > 1e-12:
        raise SystemExit(f"FAIL: living right median moved to {W2_R_LATCH_M}")
    if W2_R_LATCH_M <= LATCH_EXTRA_M:
        raise SystemExit("FAIL: living right rebuild is not fuller than 0.04087 m")
    if any(abs(err - held) < 1e-6 for err in W2_R_ERR_M for held in W2_R_HELD_ERR_M):
        raise SystemExit("FAIL: a living right commit is in the pad")
    if abs(W2_R_LATCH_M - EAST_L_LATCH_M) < 1e-3 or abs(W2_R_LATCH_M - EAST_L_Q75_M) < 1e-3:
        raise SystemExit("FAIL: living right pad copied an east pad")
    if abs(W2_R_LATCH_M - W2_L_LATCH_M) < 1e-3:
        raise SystemExit("FAIL: living right pad copied the left pad")
    if _pad_for(LIVING_WALL, 0.20, "R") != W2_R_LATCH_M:
        raise SystemExit("FAIL: living right pad is not the rebuilt median")
    if _pad_for(LIVING_WALL, 0.20) != 0.0:
        raise SystemExit("FAIL: living unknown toe borrowed a pad")
    if _pad_for(LIVING_WALL, 0.20, "L") == EAST_L_APPLIED_M:
        raise SystemExit("FAIL: living left took the east MAD pad")
    if _within_cm(-0.040270638405520004):
        raise SystemExit("FAIL: a 4.0 cm living miss would clear")
    if not _within_cm(-0.0008061141556545692):
        raise SystemExit("FAIL: the living right residual would not clear")
    if _within_cm(0.02218349902600958):
        raise SystemExit("FAIL: the kitchen median residual would clear")
    if not _within_cm(-0.0034440877109621904):
        raise SystemExit("FAIL: a 0.34 cm miss would not clear")
    if _toe_ready("east_close_R"):
        raise SystemExit("FAIL: east right can clear under the living sample bar")
    if not _toe_ready("east_close_L"):
        raise SystemExit("FAIL: east left pad is not ready")
    if not _toe_ready("living_close_L") or not _toe_ready("living_close_R"):
        raise SystemExit("FAIL: a living toe pad is not ready")
    if _toe_ready("living_close"):
        raise SystemExit("FAIL: living close with no toe can clear")
    if abs(EAST_L_HOLD_M - 0.60) > 1e-12:
        raise SystemExit(f"FAIL: walk hold moved to {EAST_L_HOLD_M}")
    if _yaw_toward_u(100.0) != voice.YAW_RAD_S or _yaw_toward_u(540.0) != -voice.YAW_RAD_S:
        raise SystemExit("FAIL: doorway pixel yaw sign moved")
    if _yaw_toward_u(rc.WIDTH / 2.0) != 0.0:
        raise SystemExit("FAIL: centred doorway pixel commands yaw")
    if abs(rc.V_MPS - 0.150) > 1e-12:
        raise SystemExit(f"FAIL: d_min speed bound moved to {rc.V_MPS}")
    if not SCENE_XML.is_file():
        raise SystemExit(f"FAIL: missing {SCENE_XML}")
    spec = apt._load_spec(apt.SPEC_JSON)
    census = _fov_census(spec["rooms"])
    _assert_census(census)
    for row in census:
        print(
            f"census {row['room']} off={row['offset_deg']:+d} "
            f"door={row['doorway_fraction']:.3f} "
            f"asked={row['asked_fraction']:.3f} "
            f"visible={row['visible_rooms']} outside={row['started_outside']}",
            flush=True,
        )
    definition = _definition()
    probe = _range_probe()
    stand_err = probe["stand_err_m"]
    bob_needed = stand_err is not None and float(stand_err) > RANGE_ERR_MAX_M
    # Median alone. The stand error is under 1 cm, so the bob pad is
    # not required and is not stacked on the median.
    latch_extra = LATCH_EXTRA_M
    print(
        f"range probe worst={probe['worst_abs_err_m']} "
        f"stand_err={stand_err} bob_needed={bob_needed} "
        f"latch_extra=median {latch_extra:.6f} n={len(SAME_WALL_ERR_M)} "
        f"bob_stacked=false bar={RANGE_ERR_MAX_M:.3f}",
        flush=True,
    )
    for sample in probe["samples"]:  # type: ignore[union-attr]
        print(
            f"  {sample['tag']} ranged={sample['ranged_m']} "
            f"true={sample['true_wall_gap_m']} err={sample['range_err_m']} "
            f"cam_z={sample['cam_z_m']:.3f} "
            f"imu_pitch={sample['imu_pitch_rad']:+.4f} "
            f"cam_pitch={sample['cam_pitch_rad']:+.4f} "
            f"lead={sample['lead_off_m']:+.3f} "
            f"step={sample['step_off_m']:+.3f} "
            f"wall={sample['true_wall']}",
            flush=True,
        )
    want = sys.argv[1] if len(sys.argv) > 1 else ""
    if want == "--wall-range":
        if _plant_md5() != before:
            raise SystemExit("FAIL: plant file changed during the range probe")
        return 0
    header: dict[str, object] = {
        "prefer_fail": True,
        "soft_pass": False,
        "go_anywhere": False,
        "kit_safe": False,
        "plant_md5": before,
        "definition": definition,
        "scene": SCENE_XML.name,
        "joined_scene": True,
        "doorway_spawn": True,
        "kit_cam_moved": False,
        "recogniser": {
            "model": room_ask.MODEL_ID,
            "revision": room_ask.MODEL_REV,
            "prompt": room_ask.YES_NO_PROMPT,
            "open_set_prompt_not_used": room_ask.PROMPT,
            "room_visible_frac": ROOM_VISIBLE_FRAC,
            "ask_step_rad": ASK_STEP_RAD,
            "settle_s": SETTLE_S,
            "door_object": room_ask.DOOR_OBJECT,
            "approach_look_s": APPROACH_LOOK_S,
            "range_err_max_m": RANGE_ERR_MAX_M,
            "search_yaw": SEARCH_YAW,
            "search_sweep_rad": SEARCH_SWEEP_RAD,
            "spawn_offsets_rad": list(SPAWN_OFFSETS_RAD),
        },
        "doorway_census": census,
        "shadow_patches": list(PATCHES),
        "range_probe": probe,
        "latch_extra_m": latch_extra,
        "latch_extra_stat": "median",
        "latch_sample_count": len(SAME_WALL_ERR_M),
        "latch_band_min_m": min(SAME_WALL_ERR_M),
        "latch_band_max_m": max(SAME_WALL_ERR_M),
        "latch_wall": LIVING_WALL,
        "east_wall": EAST_WALL,
        "east_latch_m": EAST_LATCH_M,
        "east_sample_count": len(EAST_CLOSE),
        "east_l_latch_m": EAST_L_LATCH_M,
        "east_l_mad_m": EAST_L_MAD_M,
        "east_l_applied_m": EAST_L_APPLIED_M,
        "east_l_mad_applied": EAST_L_MAD_APPLIED,
        "east_l_q75_m": EAST_L_Q75_M,
        "east_l_q75_applied": False,
        "east_l_newmin_pad_m": EAST_L_NEWMIN_PAD_M,
        "east_l_newmin_n": EAST_L_NEWMIN_N,
        "east_l_newmin_residual_m": EAST_L_NEWMIN_RESIDUAL_M,
        "east_l_near_gate_m": EAST_L_NEAR_GATE_M,
        "east_l_near_k": EAST_L_NEAR_K,
        "east_l_online_pad_m": EAST_L_ONLINE_PAD_M,
        "east_l_online_n": EAST_L_ONLINE_N,
        "east_l_online_residual_m": EAST_L_ONLINE_RESIDUAL_M,
        "east_l_q75_residual_m": EAST_L_Q75_RESIDUAL_M,
        "east_l_q75_ray_m": EAST_L_Q75_RAY_M,
        "east_l_q75_err_m": EAST_L_Q75_ERR_M,
        "east_l_pad_rule": EAST_L_LIVE,
        "other_wall_pad_rule": EAST_L_LIVE,
        "other_wall_near_gate_m": EAST_L_NEAR_GATE_M,
        "other_wall_near_k": EAST_L_NEAR_K,
        "e0_r_fail_pad_m": E0_R_FAIL_PAD_M,
        "e0_r_fail_n": E0_R_FAIL_N,
        "e0_r_fail_residual_m": E0_R_FAIL_RESIDUAL_M,
        "e0_r_thin_pad_m": E0_R_THIN_PAD_M,
        "e0_r_thin_n": E0_R_THIN_N,
        "e0_r_thin_residual_m": E0_R_THIN_RESIDUAL_M,
        "e0_r_near_gate_m": E0_R_NEAR_GATE_M,
        "e0_r_near_k": E0_R_NEAR_K,
        "e0_r_near_min_n": E0_R_NEAR_MIN_N,
        "e0_r_close_n": E0_R_CLOSE_N,
        "east_l_sample_count": len(EAST_L_ERR_M),
        "w2_l_latch_m": W2_L_LATCH_M,
        "w2_l_sample_count": len(W2_L_ERR_M),
        "w2_r_latch_m": W2_R_LATCH_M,
        "w2_r_sample_count": len(W2_R_ERR_M),
        "w2_r_recorded_m": LATCH_EXTRA_M,
        "clear_rule": "abs(measured_error-pad)<=0.01",
        "east_l_fit_applied": EAST_L_FIT_APPLIED,
        "east_l_held_err_m": EAST_L_HELD_ERR_M,
        "east_l_held_residual_m": EAST_L_HELD_ERR_M - EAST_L_LATCH_M,
        "east_l_fit_residual_m": fit_resid,
        "east_r_latch_m": EAST_R_LATCH_M,
        "east_r_sample_count": len(EAST_R_ERR_M),
        "toe_outboard_m": TOE_OUTBOARD_M,
        "east_source": "aac2baa kitchen -90 wall_hall_e_1 ray<=0.40 held out",
        "east_l_hold_m": EAST_L_HOLD_M,
        "east_r_can_clear": False,
        "bias_fit": False,
        "close_gap_m": CLOSE_GAP_M,
        "living_latch_distance_m": float(definition["d_min_m"]) + LATCH_EXTRA_M,
        "close_gap_over_living_latch": (
            CLOSE_GAP_M / (float(definition["d_min_m"]) + LATCH_EXTRA_M)
        ),
        "per_wall_pad_m": {
            "wall_hall_w_2×L": W2_L_LATCH_M,
            "wall_hall_w_2×R": W2_R_LATCH_M,
            "wall_hall_e_1×L": None,
            "wall_hall_e_1×R": EAST_R_LATCH_M,
        },
        "bob_pad_stacked": False,
        "bob_pad_needed": bob_needed,
        "rooms": [],
    }
    _write(header)
    sw.apply_frozen_forward_gait()
    plant = mj.MjModel.from_xml_path(str(sw.PLANT_XML))
    robot_bodies = _robot_bodies(plant)
    asker, ask_error = _load_asker()
    if ask_error:
        print(f"recogniser not loaded: {ask_error}", flush=True)
    rows: list[RoomJson] = []
    expected = len(PHRASES) * len(SPAWN_OFFSETS_RAD)
    try:
        for name, phrase in PHRASES:
            if want != "" and want != name:
                continue
            for offset in SPAWN_OFFSETS_RAD:
                row = _run_room(
                    name, phrase, offset, spec["rooms"][name], asker, ask_error,
                    robot_bodies, float(definition["d_min_m"]), latch_extra,
                )
                rows.append(row)
                spawn = row["spawn"]
                hip_over = [item["joint"] for item in row["hip_while_yawing"] if item["over_2_33"]]
                hips = ",".join(
                    f"{item['joint']} {item['unclamped_nm']:+.2f}/{item['clamped_nm']:+.2f}"
                    for item in row["inplace_hip"]
                )
                door_u = None
                door_frac = None
                for ask in reversed(row["asks"]):
                    if ask["door_u"] is not None:
                        door_u = ask["door_u"]
                        door_frac = ask["door_frac"]
                        break
                print(
                    f"{row['room']} off={row['yaw_offset_rad']:+.3f}  "
                    f"door={row['doorway_fraction']:.3f}  "
                    f"label={row['recogniser_label']!r}  "
                    f"frac_refuse={row['frac_refuse_count']}  "
                    f"door_u={door_u} door_px={door_frac}  "
                    f"wrong_yes={row['wrong_yes']}  "
                    f"yes_t={row['first_yes_t']} yes_tick={row['first_yes_tick']}  "
                    f"yes_heading={row['first_yes_heading_rad']}  "
                    f"yaw_turned={row['yaw_turned_rad']:+.3f}  "
                    f"spawn_com={spawn['com_xyz'][0]:+.3f},{spawn['com_xyz'][1]:+.3f}  "
                    f"vx={row['commanded_vx']} yaw={row['commanded_yaw']}  "
                    f"search_yaw={row['search_applied_yaw_peak']:+.3f}  "
                    f"search_fail={row['search_yaw_fail']}  "
                    f"ray_reject={row['ray_reject_count']}  "
                    f"d_min={row['d_min_m']:.4f}  stop={row['stop_path']}  "
                    f"false_stop={row['false_stop']}  "
                    f"prop_in_frame={row['prop_in_frame_at_stop']}  "
                    f"latch_pass={row['latch_pass']}  "
                    f"reached={row['reached']}  reach_clear={row['reach_clear']}  "
                    f"contact={row['contact']}  "
                    f"wall_stop_clear={row['wall_stop_clear']}  "
                    f"wall_stop_fail={row['wall_stop_fail']}  "
                    f"stop_wall={row['wall_stop_wall']} "
                    f"stop_class={row['wall_stop_class']} "
                    f"stop_residual={row['wall_stop_residual_m']} "
                    f"hdg={row['wall_stop_heading_rad']} "
                    f"z={row['wall_stop_cam_z_m']} "
                    f"pitch={row['wall_stop_pitch_rad']} "
                    f"lead={row['wall_stop_lead_m']} "
                    f"toe={row['wall_stop_lead_side']}  "
                    f"walk_hold={row['approach_band']['hold']} "
                    f"closest_ray={row['approach_band']['closest_ray_m']} "
                    f"closest_wall={row['approach_band']['closest_wall']} "
                    f"closest_toe={row['approach_band']['closest_toe']} "
                    f"band={row['approach_band']['entered_0_30']} "
                    f"east_l_hold={row['east_left']['hold']} "
                    f"east_l_ray={row['east_left']['min_ray_m']} "
                    f"east_l_presented={row['east_left']['presented']} "
                    f"east_l_hdg={row['east_left']['heading_rad']} "
                    f"east_l_z={row['east_left']['cam_z_m']} "
                    f"east_l_pitch={row['east_left']['cam_pitch_rad']} "
                    f"east_l_lead={row['east_left']['lead_off_m']}  "
                    f"class_reject={row['class_reject_count']}  "
                    f"same_wall={row['same_wall_count']} "
                    f"surface_reject={row['surface_reject_count']}  "
                    f"wall_bias={row['wall_bias_max_m']} "
                    f"n={row['wall_bias_count']} repeat={row['wall_bias_repeat']}  "
                    f"latch_contact={row['latch_armed_contact']}  "
                    f"gait_limit={row['gait_limit']}  "
                    f"inplace_hip={row['inplace_hip_fail']} {hips}  "
                    f"peak={row['peak_nm']:+.3f} {row['peak_joint']}  "
                    f"hip_over={hip_over}  "
                    f"shadows={len(row['shadow_first_sightings'])}",
                    flush=True,
                )
                header["rooms"] = rows
                _write(header)
    finally:
        if asker is not None:
            asker.close()
    if _plant_md5() != before:
        raise SystemExit("FAIL: plant file changed during the measure")
    if definition != _definition():
        raise SystemExit("FAIL: reach definition changed after the measure")
    reached_all = len(rows) == expected and all(row["reached"] for row in rows)
    false_any = any(row["false_stop"] for row in rows)
    wrong_any = any(row["wrong_yes"] for row in rows)
    contact_any = any(row["latch_armed_contact"] for row in rows)
    gait_any = any(row["gait_limit"] for row in rows)
    search_any = any(row["search_yaw_fail"] for row in rows)
    inplace_any = any(row["inplace_hip_fail"] for row in rows)
    bias_repeat_any = any(row["wall_bias_repeat"] for row in rows)
    wall_stop_any = any(row["wall_stop_clear"] for row in rows)
    stop_fail_any = any(row["wall_stop_fail"] for row in rows)
    residuals = [
        float(row["wall_stop_residual_m"])
        for row in rows
        if row["wall_stop_residual_m"] is not None
    ]
    header["rooms"] = rows
    header["reached_all"] = reached_all
    header["reach_clear_count"] = sum(1 for row in rows if row["reach_clear"])
    header["false_stop_any"] = false_any
    header["wrong_yes_any"] = wrong_any
    header["latch_contact_any"] = contact_any
    header["wall_range_fail_any"] = stop_fail_any
    header["wall_stop_pass_any"] = wall_stop_any
    header["wall_stop_clear_count"] = sum(1 for row in rows if row["wall_stop_clear"])
    header["wall_stop_fail_any"] = stop_fail_any
    header["class_reject_count"] = sum(int(row["class_reject_count"]) for row in rows)
    header["latch_wall"] = LIVING_WALL
    header["close_gap_m"] = CLOSE_GAP_M
    header["wall_residual_max_m"] = None if not residuals else max(residuals)
    header["gait_limit_any"] = gait_any
    header["search_yaw_fail_any"] = search_any
    header["inplace_hip_fail_any"] = inplace_any
    header["wall_bias_repeat_any"] = bias_repeat_any
    header["frac_refuse_count"] = sum(int(row["frac_refuse_count"]) for row in rows)
    header["ray_reject_count"] = sum(int(row["ray_reject_count"]) for row in rows)
    header["surface_reject_count"] = sum(int(row["surface_reject_count"]) for row in rows)
    # Reach and the wall stop are separate bars. A far or other-wall
    # ray is a class reject and is not a wall-stop fail. A stop that
    # commits and misses its own pad is a wall-stop fail.
    header["prefer_fail"] = (
        (not reached_all) or false_any or wrong_any or contact_any
        or stop_fail_any or gait_any or search_any or inplace_any or bob_needed
    )
    header["go_anywhere"] = (
        reached_all and not false_any and not wrong_any
        and not contact_any and not stop_fail_any and not gait_any and not search_any
        and not inplace_any and not bob_needed
    )
    header["kit_safe"] = False
    header["soft_pass"] = False
    _write(header)
    if header["go_anywhere"]:
        print("Prefer PASS  soft-pass=off  reached_all=true", flush=True)
        return 0
    print(
        f"Prefer FAIL  soft-pass=off  reached_all={str(reached_all).lower()}  "
        f"false_stop_any={str(false_any).lower()}  "
        f"wrong_yes_any={str(wrong_any).lower()}  "
        f"latch_contact_any={str(contact_any).lower()}  "
        f"reach_clear={header['reach_clear_count']}/{len(rows)}  "
        f"wall_stop_clear={header['wall_stop_clear_count']}/{len(rows)}  "
        f"wall_stop_fail_any={str(stop_fail_any).lower()}  "
        f"stop_residual_max={header['wall_residual_max_m']}  "
        f"class_reject={header['class_reject_count']}  "
        f"gait_limit_any={str(gait_any).lower()}  "
        f"search_yaw_fail_any={str(search_any).lower()}  "
        f"inplace_hip_fail_any={str(inplace_any).lower()}  "
        f"wall_bias_repeat_any={str(bias_repeat_any).lower()}  "
        f"frac_refuse={header['frac_refuse_count']}  "
        f"ray_reject={header['ray_reject_count']}  "
        f"surface_reject={header['surface_reject_count']}",
        flush=True,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
