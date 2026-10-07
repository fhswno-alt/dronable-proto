# Walk stepping bars

Soft-pass is off. A row is CLEAR only when the existing jerk, declared-stance
ZMP/CoM, and unclamped-torque bars pass and the stepping bars pass.
Plant file md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is checked before and after every bout.
Mass, friction, latency, and the entrance mat are applied on the loaded model.
The plant file is not written.

## Period

`op3_walk.update_time` places the left single-support window and the right
single-support window inside one `period`. `gait_manager_traj` uses the same
split. Period is one left-plus-right cycle, not one step. Each foot swings
once per period.

A no-slip walk at speed `vx` puts the next plant of that same foot `vx·T`
further along x. That is the per-swing foot travel, and it is the stride bar
(±20% on the airborne x change of each swing that starts while the bus is in
`vel`). `vx·T/2` is the spacing between consecutive opposite footfalls, the
stance-to-stance step. At 0.056 m/s and 3.60 s that spacing is 0.1008 m.
The ~0.10 m figure is that stance-to-stance step. It is not the distance one
swing foot travels.

`kit_bus_step` does not command either distance. `x_amp = min(0.020, vx/7.50)`
and the swing sine runs about ±`x_amp` in the hip frame, so the commanded
foot travel is about `2·x_amp` (14.9 mm at 0.056 m/s). The bar stays on `vx·T`.

## Bars

Swing events are runs of ticks where that foot has zero contacts with the
floor (or the entrance mat, when that geom is ground). The walker's declared
phase is not the detector. Declared-versus-actual mismatch is reported.

- Step fraction: forward x travelled with zero floor contacts, divided by the
  forward x of both feet. Bar ≥ 90%.
- Stance slip: path length of the loaded stance foot while the other foot is
  airborne. Bar ≤ 2 mm on the worst move-window step.
- Sole clearance: lowest of the eight contact-box corners (half-length 67.5 mm)
  over 20–80% of each actual swing. Bar ≥ 8 mm. A miss reports the honest max
  of those per-step minima.
- Airborne advance of each move-window swing within ±20% of `vx·T`.
- Single support, from contact: stance contact count ≥ 3 on every tick.
- Declared-stance and actual-stance contact CoP and CoM margins ≥ 0, outside
  fraction 0. Jerk strictly below the kit baseline. Unclamped ask ≤ 2.33 Nm.
- Final 1 s: both feet have at least 3 contacts on every tick, trunk pitch and
  roll stay within 5° of the stand median, and `up_z` stays at least 0.90.
  The pose line says whether that window returns to the stand joints or
  freezes in a lean.

Fore/aft separation, CoP percentiles, edge dwell, and sole tilt are reported.
They do not add a second cutoff.

| Tip | Row | Cell | Verdict | Step frac | Slip mm | Clear min mm | Clear honest mm | Air min m | vx·T m | vx·T/2 m | Sep move mm | SS min contacts | CoP min mm | Edge <5 mm | Tilt deg | Stop | Declared ZMP mm | Actual ZMP mm | Ask Nm | Mismatch | n |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| d7b06e7 | shape0-3.57 | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.200 | 0.100 | 14.2 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.054 | 0.186 | 1800 |
| d7b06e7 | shape0-3.60 | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.2 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.042 | 0.186 | 1800 |
| d7b06e7 | shape1-3.60 | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.3 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.311 | 0.186 | 1800 |
| d7b06e7 | slow-6.40 | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.256 | 0.128 | 9.8 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.001 | 0.200 | 1800 |
| d7b06e7 | shape0-3.57 | rug | Prefer FAIL | 0.001 | — | — | — | — | 0.200 | 0.100 | 13.3 | 6 | +31.25 | 0.000 | 0.14 | freezes in a lean | +0.00 | +8.89 | 2.051 | 0.186 | 1800 |
| d7b06e7 | shape0-3.57 | cycles-5 | Prefer FAIL | 0.000 | — | — | — | — | 0.200 | 0.100 | 14.2 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.054 | 0.038 | 8800 |
| d7b06e7 | shape0-3.60 | rug | Prefer FAIL | 0.001 | — | — | — | — | 0.202 | 0.101 | 13.3 | 6 | +31.60 | 0.000 | 0.14 | freezes in a lean | +0.00 | +8.89 | 2.040 | 0.186 | 1800 |
| d7b06e7 | shape0-3.60 | cycles-5 | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.2 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.042 | 0.038 | 8800 |
| d7b06e7 | shape0-3.57 | mass-0.95 | Prefer FAIL | 0.000 | — | — | — | — | 0.200 | 0.100 | 14.4 | — | — | — | — | freezes in a lean | +0.00 | +9.26 | 2.047 | 0.186 | 1800 |
| d7b06e7 | shape0-3.57 | friction-1.2 | Prefer FAIL | 0.155 | 0.05 | 0.000 | 0.012 | 0.001 | 0.200 | 0.100 | 16.4 | 4 | +25.69 | 0.000 | 0.16 | freezes in a lean | +0.00 | +9.73 | 2.070 | 0.172 | 1800 |
| d7b06e7 | shape0-3.57 | latency--1 | Prefer FAIL | 0.000 | — | — | — | — | 0.200 | 0.100 | 14.2 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.054 | 0.186 | 1800 |
| d7b06e7 | shape0-3.60 | mass-0.95 | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.4 | — | — | — | — | freezes in a lean | +0.00 | +9.26 | 2.065 | 0.186 | 1800 |
| d7b06e7 | shape0-3.60 | friction-1.2 | Prefer FAIL | 0.170 | 0.04 | 0.000 | 0.015 | 0.001 | 0.202 | 0.101 | 16.5 | 4 | +25.42 | 0.000 | 0.16 | freezes in a lean | +0.00 | +9.73 | 2.051 | 0.172 | 1800 |
| d7b06e7 | shape0-3.60 | latency--1 | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.2 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.042 | 0.186 | 1800 |
| d7b06e7 | shape1-3.60 | mass-0.95 | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.5 | — | — | — | — | freezes in a lean | +0.00 | +9.26 | 2.300 | 0.186 | 1800 |
| d7b06e7 | shape1-3.60 | friction-1.2 | Prefer FAIL | 0.175 | 0.04 | 0.000 | 0.003 | 0.001 | 0.202 | 0.101 | 16.6 | 4 | +25.98 | 0.000 | 0.16 | freezes in a lean | +0.00 | +9.73 | 2.295 | 0.174 | 1800 |
| d7b06e7 | shape1-3.60 | latency--1 | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.3 | — | — | — | — | freezes in a lean | +0.00 | +8.89 | 2.311 | 0.186 | 1800 |
| ac81435 | voice-3.60 | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.202 | 0.101 | 14.1 | — | — | — | — | freezes in a lean | -46.58 | +3.59 | 2.313 | 0.193 | 1800 |
| ac81435 | voice-3.70 | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.207 | 0.104 | 14.2 | — | — | — | — | freezes in a lean | -49.05 | +8.89 | 2.262 | 0.202 | 1800 |
| ac81435 | slow | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.256 | 0.128 | 10.0 | — | — | — | — | freezes in a lean | -48.83 | +8.89 | 1.996 | 0.212 | 1800 |
| 58ce1d8 | slow | nominal | Prefer FAIL | 0.000 | — | — | — | — | 0.256 | 0.128 | 10.1 | — | — | — | — | freezes in a lean | -48.83 | +0.36 | 1.979 | 0.212 | 1800 |

Worst perturbation cell on `d7b06e7` `d7b06e79757a6394784ab3582e9c34c1b8c2ab2b`: `shape1-3.60` / `latency--1` is Prefer FAIL.

Nominal rows, from the sim state:

- `d7b06e7` `shape0-3.57`: contact advance 0.080 m, airborne advance 0.000 m, move separation 14.2 mm, declared/actual mismatch 0.186, final 1 s pitch -14.56 deg against stand +1.61 deg (off by 16.17 deg), roll off 0.33 deg, up_z 0.968, contacts L/R 4/4, joint error 15.5 deg. freezes in a lean.
- `d7b06e7` `shape0-3.60`: contact advance 0.080 m, airborne advance 0.000 m, move separation 14.2 mm, declared/actual mismatch 0.186, final 1 s pitch -14.56 deg against stand +1.61 deg (off by 16.17 deg), roll off 0.33 deg, up_z 0.968, contacts L/R 4/4, joint error 15.5 deg. freezes in a lean.
- `d7b06e7` `shape1-3.60`: contact advance 0.081 m, airborne advance 0.000 m, move separation 14.3 mm, declared/actual mismatch 0.186, final 1 s pitch -14.56 deg against stand +1.61 deg (off by 16.17 deg), roll off 0.31 deg, up_z 0.968, contacts L/R 4/4, joint error 15.5 deg. freezes in a lean.
- `d7b06e7` `slow-6.40`: contact advance 0.034 m, airborne advance 0.000 m, move separation 9.8 mm, declared/actual mismatch 0.200, final 1 s pitch -14.52 deg against stand +1.61 deg (off by 16.13 deg), roll off 0.23 deg, up_z 0.968, contacts L/R 4/4, joint error 15.5 deg. freezes in a lean.
- `ac81435` `voice-3.60`: contact advance 0.080 m, airborne advance 0.000 m, move separation 14.1 mm, declared/actual mismatch 0.193, final 1 s pitch -14.63 deg against stand +1.61 deg (off by 16.56 deg), roll off 0.30 deg, up_z 0.966, contacts L/R 2/2, joint error 18.1 deg. freezes in a lean.
- `ac81435` `voice-3.70`: contact advance 0.080 m, airborne advance 0.000 m, move separation 14.2 mm, declared/actual mismatch 0.202, final 1 s pitch -14.94 deg against stand +1.61 deg (off by 16.55 deg), roll off 0.38 deg, up_z 0.966, contacts L/R 3/4, joint error 18.1 deg. freezes in a lean.
- `ac81435` `slow`: contact advance 0.035 m, airborne advance 0.000 m, move separation 10.0 mm, declared/actual mismatch 0.212, final 1 s pitch -14.59 deg against stand +1.61 deg (off by 16.20 deg), roll off 0.43 deg, up_z 0.968, contacts L/R 3/4, joint error 15.7 deg. freezes in a lean.
- `58ce1d8` `slow`: contact advance 0.045 m, airborne advance 0.000 m, move separation 10.1 mm, declared/actual mismatch 0.212, final 1 s pitch -14.59 deg against stand -4.40 deg (off by 10.19 deg), roll off 0.41 deg, up_z 0.968, contacts L/R 3/4, joint error 15.7 deg. freezes in a lean.

`Sep move mm` is the peak `|x_L − x_R|` while the bus is in `vel`.
With no swing, that peak is the fore/aft gap of the two feet as they move.

58ce1d8 has no vx 0.056 preview row. The slow row is the one that tip published.

Controls marked the ac81435 voice rows and the slow row CLEAR, and marked
the d7b06e7 declared-stance cells CLEAR. A step fraction of 0, with both feet
keeping floor contact and the forward motion happening in contact, is a skate.
The #102 margin and torque bars already Prefer-FAIL'd ac81435 and 58ce1d8.
They did not say the feet were skating. On d7b06e7 those older bars pass:
declared contact CoP sits on the polygon edge, outside fraction is 0, unclamped
ask stays ≤ 2.33 Nm, and jerk is under the kit baseline. The stepping bars
are what fail that tip. The stop freezes pitched forward of the stand.
μ 1.2 is the only #103 cell with airborne ticks. Those runs last a few
control ticks, the lowest sole corner stays under 0.02 mm, and the airborne
advance is a few millimetres against `vx·T` of about 0.20 m. Mass −5% and
latency −1 stay at step fraction 0. Latency here is the #102 command clock,
one tick early on the bus phrase. It does not create a step.

## Retro voice tips

Soft-pass is off. These bouts are the suite's own scripts on each tip.
The plant file is not written. A tip whose plant md5 is not
`207f3d5e9c6a72e16f7aa0c8d224f75e` is reported as predating that plant.

On these tips `locked_kit_config` sets `gm_period_s` to 0.500 s.
`op3_walk.update_time` still places both single-support windows inside
that period, so period is one left-plus-right cycle. Each foot swings
once per period. The per-swing foot travel is `vx·T`. `vx·T/2` is the
stance-to-stance spacing. Voice bouts command `voice.FWD_MPS` 0.056 m/s,
so `vx·T` is 0.028 m and `vx·T/2` is 0.014 m. The day-1 kit script
commands `VX_FWD_CAP` 0.150 m/s, so `vx·T` is 0.075 m and `vx·T/2` is
0.0375 m. The ~0.10 m figure belongs to the 3.60 s preview rows, not
to this 0.500 s walker.

Forward travel is along the trunk heading, not world +x. Apartment
bouts turn. A straight kit walk has yaw near 0, so the two axes match.
A bout is STEPS when at least one swing starts in bus mode `move` and
airborne advance is at least half the forward travel. It is SKATES
when the feet stay down or contact advance is the majority. The 90%
step-fraction bar stays in the fail list either way.
Swing events are runs of ticks with zero contacts on the floor geom,
and on `mat_rug` when that geom is in the scene.

| Tip | PR | Bout | Verdict | Earlier CLEAR | Step frac | Air ticks | Air m | vx·T m | Contact m | Slip mm | Clear mm | Sep mm | Mismatch | Pitch vs stand | Plant |
| --- | ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 79edde5 | 82 | kitchen-m90 | STEPS | — | 0.926 | 3040 | 1.212 | 0.028 | 0.097 | 3.44 | 2.08 | 17.0 | 0.333 | -16.32 vs stand -0.00 (16.32 off) | frozen |
| 79edde5 | 82 | living-m90 | STEPS | living −90 wall-stop CLEAR | 0.927 | 2750 | 1.199 | 0.028 | 0.094 | 3.44 | 2.08 | 17.0 | 0.320 | +15.57 vs stand -0.00 (15.58 off) | frozen |
| 79edde5 | 82 | entrance-m90 | STEPS | — | 0.927 | 3831 | 1.560 | 0.028 | 0.123 | 3.44 | 2.07 | 17.0 | 0.329 | +14.57 vs stand -0.00 (15.62 off) | frozen |
| e2da4f3 | 88 | kitchen-m90 | STEPS | kitchen −90 wall-stop CLEAR | 0.930 | 2483 | 1.235 | 0.028 | 0.093 | 3.44 | 2.08 | 17.0 | 0.306 | -13.18 vs stand -0.00 (16.50 off) | frozen |
| e2da4f3 | 88 | living-m90 | STEPS | living −90 wall-stop CLEAR | 0.927 | 2750 | 1.199 | 0.028 | 0.094 | 3.44 | 2.08 | 17.0 | 0.320 | +15.57 vs stand -0.00 (15.58 off) | frozen |
| e2da4f3 | 88 | entrance-m90 | STEPS | — | 0.927 | 3831 | 1.560 | 0.028 | 0.123 | 3.44 | 2.07 | 17.0 | 0.329 | +14.57 vs stand -0.00 (15.62 off) | frozen |
| 51ae123 | 90 | kitchen-m90 | STEPS | kitchen −90 wall-stop CLEAR | 0.930 | 2483 | 1.235 | 0.028 | 0.093 | 3.44 | 2.08 | 17.0 | 0.306 | -13.18 vs stand -0.00 (16.50 off) | frozen |
| 51ae123 | 90 | living-m90 | STEPS | living −90 wall-stop CLEAR | 0.929 | 2413 | 1.177 | 0.028 | 0.090 | 3.39 | 2.08 | 17.0 | 0.307 | +15.39 vs stand -0.00 (16.06 off) | frozen |
| 51ae123 | 90 | entrance-m90 | STEPS | entrance −90 wall-stop CLEAR | 0.928 | 2808 | 1.252 | 0.028 | 0.097 | 3.44 | 2.07 | 17.0 | 0.321 | +16.32 vs stand -0.00 (16.32 off) | frozen |
| f8c9edb | 98 | kitchen-m90 | STEPS | kitchen −90 wall-stop CLEAR | 0.930 | 2564 | 1.278 | 0.028 | 0.096 | 3.44 | 2.08 | 17.0 | 0.309 | -14.07 vs stand -0.00 (16.05 off) | frozen |
| f8c9edb | 98 | living-m90 | STEPS | living −90 wall-stop CLEAR | 0.929 | 2415 | 1.177 | 0.028 | 0.090 | 3.44 | 2.08 | 17.0 | 0.307 | +15.39 vs stand -0.00 (16.05 off) | frozen |
| f8c9edb | 98 | entrance-m90 | STEPS | entrance −90 wall-stop CLEAR | 0.928 | 2808 | 1.252 | 0.028 | 0.097 | 3.44 | 2.07 | 17.0 | 0.321 | +16.32 vs stand -0.00 (16.32 off) | frozen |
| 743b79a | 99 | kitchen-m90 | STEPS | kitchen −90 wall-stop CLEAR | 0.930 | 2483 | 1.235 | 0.028 | 0.093 | 3.44 | 2.08 | 17.0 | 0.306 | -13.18 vs stand -0.00 (16.50 off) | frozen |
| 743b79a | 99 | living-m90 | STEPS | living −90 wall-stop CLEAR | 0.929 | 2415 | 1.177 | 0.028 | 0.090 | 3.44 | 2.08 | 17.0 | 0.307 | +15.39 vs stand -0.00 (16.05 off) | frozen |
| 743b79a | 99 | entrance-m90 | STEPS | — | 0.928 | 2808 | 1.252 | 0.028 | 0.097 | 3.44 | 2.07 | 17.0 | 0.321 | +16.32 vs stand -0.00 (16.32 off) | frozen |
| 69da170 | 70 | kit | STEPS | Track 1 toe-clearance CLEAR, MID_SWING_TOE_MIN_M −0.003066 | 0.961 | 585 | 1.392 | 0.075 | 0.057 | 3.19 | 2.02 | 42.8 | 0.389 | -14.98 vs stand -4.40 (10.59 off) | frozen |

- `79edde5` `kitchen-m90` STEPS: airborne ticks 3040, airborne advance 1.212 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0101 m), contact advance 0.097 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.90 mm), fore/aft separation 17.0 mm, mismatch 0.333, final pitch -16.32 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_e_1` clear False residual -0.0016417252995797604.
- `79edde5` `living-m90` STEPS: airborne ticks 2750, airborne advance 1.199 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.094 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.90 mm), fore/aft separation 17.0 mm, mismatch 0.320, final pitch +15.57 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_2` clear True residual -0.0008061141556545692.
- `79edde5` `entrance-m90` STEPS: airborne ticks 3831, airborne advance 1.560 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0103 m), contact advance 0.123 m, worst slip 3.44 mm, lowest-corner clearance 2.07 mm (honest max 7.92 mm), fore/aft separation 17.0 mm, mismatch 0.329, final pitch +14.57 deg against stand -0.00 deg, freezes in a lean. Suite stop `time` wall `` clear False residual None.
- `e2da4f3` `kitchen-m90` STEPS: airborne ticks 2483, airborne advance 1.235 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.093 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.88 mm), fore/aft separation 17.0 mm, mismatch 0.306, final pitch -13.18 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_e_1` clear True residual 0.008490527484481536.
- `e2da4f3` `living-m90` STEPS: airborne ticks 2750, airborne advance 1.199 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.094 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.90 mm), fore/aft separation 17.0 mm, mismatch 0.320, final pitch +15.57 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_2` clear True residual -0.009305231187210411.
- `e2da4f3` `entrance-m90` STEPS: airborne ticks 3831, airborne advance 1.560 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0103 m), contact advance 0.123 m, worst slip 3.44 mm, lowest-corner clearance 2.07 mm (honest max 7.92 mm), fore/aft separation 17.0 mm, mismatch 0.329, final pitch +14.57 deg against stand -0.00 deg, freezes in a lean. Suite stop `time` wall `` clear False residual None.
- `51ae123` `kitchen-m90` STEPS: airborne ticks 2483, airborne advance 1.235 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.093 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.88 mm), fore/aft separation 17.0 mm, mismatch 0.306, final pitch -13.18 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_e_1` clear True residual 0.008490527484481536.
- `51ae123` `living-m90` STEPS: airborne ticks 2413, airborne advance 1.177 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.090 m, worst slip 3.39 mm, lowest-corner clearance 2.08 mm (honest max 7.88 mm), fore/aft separation 17.0 mm, mismatch 0.307, final pitch +15.39 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_2` clear True residual 0.0005739224808515853.
- `51ae123` `entrance-m90` STEPS: airborne ticks 2808, airborne advance 1.252 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.097 m, worst slip 3.44 mm, lowest-corner clearance 2.07 mm (honest max 7.92 mm), fore/aft separation 17.0 mm, mismatch 0.321, final pitch +16.32 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_1` clear True residual 0.0030894237059098106.
- `f8c9edb` `kitchen-m90` STEPS: airborne ticks 2564, airborne advance 1.278 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.096 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.88 mm), fore/aft separation 17.0 mm, mismatch 0.309, final pitch -14.07 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_e_1` clear True residual -0.005542217279546707.
- `f8c9edb` `living-m90` STEPS: airborne ticks 2415, airborne advance 1.177 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.090 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.88 mm), fore/aft separation 17.0 mm, mismatch 0.307, final pitch +15.39 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_2` clear True residual -0.005356256706489343.
- `f8c9edb` `entrance-m90` STEPS: airborne ticks 2808, airborne advance 1.252 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.097 m, worst slip 3.44 mm, lowest-corner clearance 2.07 mm (honest max 7.92 mm), fore/aft separation 17.0 mm, mismatch 0.321, final pitch +16.32 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_1` clear True residual 0.0030894237059098106.
- `743b79a` `kitchen-m90` STEPS: airborne ticks 2483, airborne advance 1.235 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.093 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.88 mm), fore/aft separation 17.0 mm, mismatch 0.306, final pitch -13.18 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_e_1` clear True residual 0.008490527484481536.
- `743b79a` `living-m90` STEPS: airborne ticks 2415, airborne advance 1.177 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.090 m, worst slip 3.44 mm, lowest-corner clearance 2.08 mm (honest max 7.88 mm), fore/aft separation 17.0 mm, mismatch 0.307, final pitch +15.39 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_2` clear True residual -0.005356256706489343.
- `743b79a` `entrance-m90` STEPS: airborne ticks 2808, airborne advance 1.252 m against vx·T 0.028 m (per-swing min -0.0016 m, median 0.0107 m), contact advance 0.097 m, worst slip 3.44 mm, lowest-corner clearance 2.07 mm (honest max 7.92 mm), fore/aft separation 17.0 mm, mismatch 0.321, final pitch +16.32 deg against stand -0.00 deg, freezes in a lean. Suite stop `floor_edge` wall `wall_hall_w_1` clear True residual 0.0030894237059098106.
- `69da170` `kit` STEPS: airborne ticks 585, airborne advance 1.392 m against vx·T 0.075 m (per-swing min 0.0039 m, median 0.0772 m), contact advance 0.057 m, worst slip 3.19 mm, lowest-corner clearance 2.02 mm (honest max 8.07 mm), fore/aft separation 42.8 mm, mismatch 0.389, final pitch -14.98 deg against stand -4.40 deg, freezes in a lean.

No earlier CLEAR on this sample reopens as a skate.

Earlier CLEARs whose bouts step: #82 living-m90, #88 kitchen-m90, #88 living-m90, #90 kitchen-m90, #90 living-m90, #90 entrance-m90, #98 kitchen-m90, #98 living-m90, #98 entrance-m90, #99 kitchen-m90, #99 living-m90, #70 kit. Those feet leave the floor. The wall-stop residual and the Track 1 toe lock stay what they were. They are not the #103 skate (step fraction 0, zero airborne ticks).

Every bout is STEPS and every bout still misses the hard stepping bars, so none of these rows is a stepping CLEAR. Voice stance slip is 3.39–3.44 mm (kit 3.19 mm) against a 2 mm bar. Lowest-corner clearance over 20–80% of swing is 2.02–2.08 mm against an 8 mm bar. The honest max of those per-step minima is 7.88–7.92 mm on the voice rows and 8.07 mm on the kit. Voice per-swing airborne travel has a median near 10.7 mm and a minimum near −1.6 mm, against vx·T of 0.028 m. The kit's shortest swing is 3.9 mm and its median is 77.2 mm, against vx·T of 0.075 m. Single-support contact count bottoms at 2. Declared-versus-actual phase mismatch is 0.31–0.33 on the voice rows and 0.389 on the kit. Peak fore/aft separation while the bus is in `move` is 17.0 mm on every voice bout and 42.8 mm on the kit.

The wall suite returns on the stop tick, so the last second is still the approach: both feet leave the floor (contacts 0/0) and the trunk sits about 16° off stand. Kitchen faces +x, so the final pitch is negative (−13.18° to −16.32°). Living and entrance face −x, so the same forward lean reads positive (+14.57° to +16.32°). Stand pitch on these voice tips is about 0°. The kit stand pitch is −4.40°, the final pitch is −14.98°, and that pose freezes in a lean with both feet down (contacts 4/4).

This pass, wall residuals: #82 kitchen-m90 residual -0.0016 m wall clear False; #82 living-m90 residual -0.0008 m wall clear True; #82 entrance-m90 stop `time` wall clear False; #88 kitchen-m90 residual +0.0085 m wall clear True; #88 living-m90 residual -0.0093 m wall clear True; #88 entrance-m90 stop `time` wall clear False; #90 kitchen-m90 residual +0.0085 m wall clear True; #90 living-m90 residual +0.0006 m wall clear True; #90 entrance-m90 residual +0.0031 m wall clear True; #98 kitchen-m90 residual -0.0055 m wall clear True; #98 living-m90 residual -0.0054 m wall clear True; #98 entrance-m90 residual +0.0031 m wall clear True; #99 kitchen-m90 residual +0.0085 m wall clear True; #99 living-m90 residual -0.0054 m wall clear True; #99 entrance-m90 residual +0.0031 m wall clear True. Published wall CLEARs that this pass repeats: #82 living −0.0008 (here −0.0008); #88 kitchen +0.0085 and living −0.0093; #90 kitchen +0.0085 and entrance +0.0031; #98 kitchen −0.0055 and living −0.0054; #99 kitchen +0.0085 and living −0.0054. #90 living on this heading-frame pass is +0.0006. The published figure is −0.0054, and an earlier run of the same script landed on −0.0054. Moondream answers move the residual by a few millimetres. The latch still CLEARs. #98 entrance published +0.0062 and this pass CLEARs at +0.0031, the same stop time as #90. #99 published an entrance miss of +0.1254. This pass CLEARs that bout at +0.0031, same t=39.664 s as #90. #82 kitchen stops on the east wall with residual −0.0016 and wall clear false (right toe). #82 and #88 entrance hit the 53 s time limit. Those three bouts were not published wall CLEARs. They still STEP.

The #70 lock `MID_SWING_TOE_MIN_M` = −0.003066 is the entrance-straight loaded scuff that tip recorded. It is not this day-1 kit bout. The kit script (`scripts/steer_walk.py` `_bus_kit_forward_stop`, `BUS_KIT_SCRIPT`, `locked_kit_config`) steps: fraction 0.961, 585 airborne ticks, lowest corner +2.02 mm. That does not reopen the toe lock as a skate, and it does not meet the 8 mm sole bar.

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` held before and after every bout. None of these tips predates that plant.

## Tiebreak d6e8b5e

Soft-pass is off. The gait is tip `d6e8b5ebb801250814552fc728886a11dd0350c2` and the metrics are the #102 scorer. Controls' step-honesty function is not called. The plant file is not written.

A tick is airborne only when three tests agree on that foot: zero `mjData` contacts between its group-0 contact box and the floor, all eight corners of that box above the floor plane, and summed `mj_contactForce` normal equal to 0. The mesh is not sampled. HW's mesh sole sits about 2.9 mm above this box bottom.

This tip has one scored bout: stand 0.40 s, walk 22.00 s, stop 8.00 s, period 20.0 s, dsp 0.35, swing height 18 mm, hip-frame cap 21 mm, bus vx 0.056 m/s. The cadence grid (T 0.5–2.0 s, x_amp = vx·T/4) is not on this commit.

`voice-20` STEPS / Prefer FAIL. Plant `207f3d5e9c6a72e16f7aa0c8d224f75e` before and `207f3d5e9c6a72e16f7aa0c8d224f75e` after.

| Side | Lift s | Touch s | Dur s | Clear 20–80 mm | Peak 20–80 mm | Peak air mm | Air m | Stance dx m | Slip mm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| L | 5.224 | 10.288 | 5.064 | 8.41 | 10.67 | 10.67 | 0.0763 | -0.0000 | 0.27 |
| R | 15.240 | 20.296 | 5.056 | 8.60 | 10.80 | 10.80 | 0.0764 | -0.0000 | 0.26 |

The three tests never disagree. Contact count, corner height, and normal force pick the same airborne ticks on both feet. Contact-box bottom local z is -26.00 mm. Floor plane z is 0. The mesh is not in the score.

Commanded hip-frame `x_amp` peaked at 21.00 mm and `z_flat` was on. Two swings, left then right. Row step fraction 0.794 (airborne 0.1536 m, contact 0.0399 m). Whole-bout phase mismatch 0.094. Stance contact count minimum 4. Worst stance slip 0.27 mm. Lowest corner over 20–80% of the airborne window 8.41 mm (honest max of those minima 8.60 mm).

Single-support CoP margin min 17.52 mm, p5 19.28 mm, p50 20.37 mm, p95 21.48 mm. Edge dwell under 5 mm is 0.000. Sole tilt max +1.00 deg, fraction over 1 deg 0.0024.

Declared-phase contact ZMP minimum 0.002 mm, outside 0.000. Declared CoM minimum 15.10 mm, outside 0.000. Actual-stance ZMP minimum 7.23 mm, CoM minimum 15.10 mm, outside 0.000 and 0.000.

CoM jerk 88.853 / 2.987 m/s³. Joint jerk vector 779.752 / 50.612 rad/s³, largest hinge r_ank_pitch 616.809 / 21.822. Unclamped peak r_hip_roll 2.2567 Nm at 2.832 s, headroom 0.0733 Nm. Leg asks: r_hip_roll 2.2567 Nm at 2.832 s, headroom 0.0733 Nm; l_knee 2.2556 Nm at 5.248 s, headroom 0.0744 Nm; r_knee 2.2147 Nm at 15.208 s, headroom 0.1153 Nm; l_hip_roll 2.1076 Nm at 2.832 s, headroom 0.2224 Nm; r_ank_roll 1.9795 Nm at 2.816 s, headroom 0.3505 Nm; l_hip_pitch 1.7934 Nm at 1.832 s, headroom 0.5366 Nm.

Stop returns to the stand pose. Final trunk pitch +1.67 deg against stand +1.61 deg (off +0.06 deg). Final 1 s contacts L/R 4/4, min up_z 1.000.

Bus `vx·T` at 0.056 m/s and 20 s is 1.12 m. Both airborne advances are 76 mm, so the ±20% stride bar fails. Step fraction 0.794 is under 0.90 because 0.040 m of forward travel happens in contact. Clearance, slip, stance contacts, declared and actual margins, jerk, unclamped ask, and the stop pose pass. The hinge-speed bar is in the fail list when a leg hinge exceeds 5.82 rad/s. The row is STEPS and Prefer FAIL. The cadence grid is not on this commit, so there is no second row.

Against the posted row: unclamped right hip roll 2.2567 Nm at 2.832 s matches. Declared CoM minimum +15.10 mm matches the posted +15.11 mm. Declared whole-bout ZMP minimum +0.002 mm matches the posted contact-CoP minimum. Declared single-support ZMP minima are +6.34 mm (ss_L) and +6.36 mm (ss_R). Stance contacts stay at 4. The right-foot peak clearance is 10.80 mm. The airborne-window minima are 8.41 mm and 8.60 mm, above the posted 8.08 mm, because this scorer's swing is the contact-off interval (5.06 s) and the posted window is the longer clocked single support. Stance-slip path is 0.27 mm, and the whole-bout step fraction is 0.794 with phase mismatch 0.094. Those three are this scorer's definitions. The feet do leave the floor: 633 and 632 ticks with zero contacts, zero force, and every box corner above the plane.

Fail reasons: step fraction 0.794 is under 0.90 (airborne forward 0.1536 m, contact forward 0.0399 m); airborne advance 0.0763 m on L at 5.224 s is outside ±20% of vx·T (1.1200 m). vx·T/2 is the stance-to-stance spacing (0.5600 m), not this travel.

Hinge speed and trunk vx:
Period T 20.000 s, commanded vx 0.0560 m/s, actual trunk vx 0.0060 m/s (forward 0.1321 m over 22.000 s along the trunk heading), ratio 0.107.
Peak |qvel| bar 5.82 rad/s passes. The plant has no velocity cap.

| Joint | Peak rad/s | t s | Stage | Headroom rad/s |
| --- | ---: | ---: | --- | ---: |
| r_ank_roll | 0.6251 | 2.840 | walk | 5.1949 |
| l_ank_roll | 0.5975 | 2.840 | walk | 5.2225 |
| l_knee | 0.5904 | 10.520 | walk | 5.2296 |
| r_hip_roll | 0.5894 | 2.840 | walk | 5.2306 |
| l_hip_roll | 0.5696 | 2.840 | walk | 5.2504 |
| r_knee | 0.5691 | 15.144 | walk | 5.2509 |
| l_ank_pitch | 0.4264 | 23.816 | stop | 5.3936 |
| r_ank_pitch | 0.3766 | 24.472 | stop | 5.4434 |
| l_hip_pitch | 0.3029 | 5.224 | walk | 5.5171 |
| r_hip_pitch | 0.3011 | 15.184 | walk | 5.5189 |
| r_hip_yaw | 0.0516 | 23.520 | stop | 5.7684 |
| l_hip_yaw | 0.0405 | 23.520 | stop | 5.7795 |

## Hinge speed

Soft-pass is off. Peak `|qvel|` on each of the 12 leg hinges must be
≤ 5.82 rad/s (HX-35H no-load, 0.18 s/60°). The plant has no velocity
cap. Headroom is 5.82 − peak. A joint over the bar is a row fail.
Preview rows take the stage from `preview_stage`. The #90 kitchen
walker has no preview stage, so its stage column is the gait phase.

The speed line is period T, commanded vx, actual trunk vx, and the
ratio. Actual trunk vx is the trunk origin's heading-frame forward
displacement over the bus `move` window, divided by that window's
duration. The ratio is reported. It is not a separate cutoff.

`d6e8b5e` `voice-20` STEPS, Prefer FAIL. Hinge-speed bar passes. Plant `207f3d5e9c6a72e16f7aa0c8d224f75e` before and `207f3d5e9c6a72e16f7aa0c8d224f75e` after.
Period T 20.000 s, commanded vx 0.0560 m/s, actual trunk vx 0.0060 m/s (forward 0.1321 m over 22.000 s), ratio 0.107.

| Joint | Peak rad/s | t s | Stage | Headroom rad/s |
| --- | ---: | ---: | --- | ---: |
| r_ank_roll | 0.6251 | 2.840 | walk | 5.1949 |
| l_ank_roll | 0.5975 | 2.840 | walk | 5.2225 |
| l_knee | 0.5904 | 10.520 | walk | 5.2296 |
| r_hip_roll | 0.5894 | 2.840 | walk | 5.2306 |
| l_hip_roll | 0.5696 | 2.840 | walk | 5.2504 |
| r_knee | 0.5691 | 15.144 | walk | 5.2509 |
| l_ank_pitch | 0.4264 | 23.816 | stop | 5.3936 |
| r_ank_pitch | 0.3766 | 24.472 | stop | 5.4434 |
| l_hip_pitch | 0.3029 | 5.224 | walk | 5.5171 |
| r_hip_pitch | 0.3011 | 15.184 | walk | 5.5189 |
| r_hip_yaw | 0.0516 | 23.520 | stop | 5.7684 |
| l_hip_yaw | 0.0405 | 23.520 | stop | 5.7795 |

`51ae123` `kitchen-m90` STEPS, other bars not re-judged. Hinge-speed bar passes. Plant `207f3d5e9c6a72e16f7aa0c8d224f75e` before and `207f3d5e9c6a72e16f7aa0c8d224f75e` after.
Period T 0.500 s, commanded vx 0.0560 m/s, actual trunk vx 0.0240 m/s (forward 0.5316 m over 22.128 s), ratio 0.429.

| Joint | Peak rad/s | t s | Stage | Headroom rad/s |
| --- | ---: | ---: | --- | ---: |
| r_knee | 3.6088 | 14.432 | stand | 2.2112 |
| l_hip_roll | 2.6229 | 2.312 | ds | 3.1971 |
| l_knee | 2.6216 | 9.408 | ds | 3.1984 |
| r_hip_roll | 2.6183 | 9.968 | ds | 3.2017 |
| r_hip_pitch | 1.8124 | 35.968 | stand | 4.0076 |
| l_ank_pitch | 1.8105 | 8.208 | ss_L | 4.0095 |
| r_ank_pitch | 1.7696 | 12.456 | ss_R | 4.0504 |
| l_ank_roll | 1.7505 | 1.576 | ss_R | 4.0695 |
| r_ank_roll | 1.7397 | 1.840 | ss_L | 4.0803 |
| l_hip_pitch | 1.4463 | 18.808 | ss_L | 4.3737 |
| r_hip_yaw | 0.5870 | 1.680 | ss_R | 5.2330 |
| l_hip_yaw | 0.5529 | 1.672 | ss_R | 5.2671 |
