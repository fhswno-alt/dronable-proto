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
  fraction 0. Jerk strictly below the kit baseline. Torque pass is the
  signed pre-clamp ask ≤ 2.33 Nm on every leg joint on every tick. The sum
  stays a column. A signed pass that fails the sum is flagged
  `passes signed, fails sum`.
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

CoM jerk 88.853 / 2.987 m/s³. Joint jerk vector 779.752 / 50.612 rad/s³, largest hinge r_ank_pitch 616.809 / 21.822. Sum column r_hip_roll 2.2567 Nm at 2.832 s, headroom 0.0733 Nm. Leg asks: r_hip_roll 2.2567 Nm at 2.832 s, headroom 0.0733 Nm; l_knee 2.2556 Nm at 5.248 s, headroom 0.0744 Nm; r_knee 2.2147 Nm at 15.208 s, headroom 0.1153 Nm; l_hip_roll 2.1076 Nm at 2.832 s, headroom 0.2224 Nm; r_ank_roll 1.9795 Nm at 2.816 s, headroom 0.3505 Nm; l_hip_pitch 1.7934 Nm at 1.832 s, headroom 0.5366 Nm.

Stop returns to the stand pose. Final trunk pitch +1.67 deg against stand +1.61 deg (off +0.06 deg). Final 1 s contacts L/R 4/4, min up_z 1.000.

Bus `vx·T` at 0.056 m/s and 20 s is 1.12 m. Both airborne advances are 76 mm, so the ±20% stride bar fails. Step fraction 0.794 is under 0.90 because 0.040 m of forward travel happens in contact. The signed pre-clamp ask must stay ≤ 2.33 Nm on every leg joint. A zero clamp-active fraction is not that pass. The sum stays a column, and a signed pass that fails the sum is marked `passes signed, fails sum`. The hinge-speed bar, the speed-torque line, and the clamp-active bar are in the fail list when they fail. The cadence grid is not on this commit, so there is no second row.

Against the posted row: unclamped right hip roll 2.2567 Nm at 2.832 s matches. Declared CoM minimum +15.10 mm matches the posted +15.11 mm. Declared whole-bout ZMP minimum +0.002 mm matches the posted contact-CoP minimum. Declared single-support ZMP minima are +6.34 mm (ss_L) and +6.36 mm (ss_R). Stance contacts stay at 4. The right-foot peak clearance is 10.80 mm. The airborne-window minima are 8.41 mm and 8.60 mm, above the posted 8.08 mm, because this scorer's swing is the contact-off interval (5.06 s) and the posted window is the longer clocked single support. Stance-slip path is 0.27 mm, and the whole-bout step fraction is 0.794 with phase mismatch 0.094. Those three are this scorer's definitions. The feet do leave the floor: 633 and 632 ticks with zero contacts, zero force, and every box corner above the plane.

Fail reasons: step fraction 0.794 is under 0.90 (airborne forward 0.1536 m, contact forward 0.0399 m); airborne advance 0.0763 m on L at 5.224 s is outside ±20% of vx·T (1.1200 m). vx·T/2 is the stance-to-stance spacing (0.5600 m), not this travel.

Hinge speed and trunk vx:
DC-motor model, not datasheet (Hiwonder HX-35H page values)
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

Torque pass is the signed pre-clamp ask `|kp·(ctrl−q) − kv·q̇|`
≤ 2.33 Nm on every leg joint on every tick. kv is
`−model.actuator_biasprm[i, 2]`. The sum `|kp·(q_des−q)| + |kv·ω|`
stays a column. A joint or a bout that passes the signed ask and
fails the sum is marked `passes signed, fails sum`. That note is
not a fail by itself, and it is not dropped.

Controls' posted unclamped numbers are the sum. On `d6e8b5e`,
r_hip_roll 2.2567 Nm at 2.832 s is that sum. The q_des signed torque
on the same write is +0.2545 Nm.

`tau_signed_nm` is `|kp·(q_des−q) − kv·ω|` of the unclamped target.
`tau_sum_nm` is the sum. `tau_ask_nm` is the signed pre-clamp ask
of the written ctrl.

The speed-torque line is a hard bar. The label is
`DC-motor model, not datasheet (Hiwonder HX-35H page values)`.
no_load_speed 5.82 rad/s, stall_torque 3.43 Nm, voltage 11.1 V.
Every leg write must satisfy `|qvel| ≤ 5.82·(1 − |signed ask|/3.43)`.
Voltage is recorded and does not scale the line.

Clamp-active fraction is the share of control ticks where
`|signed ask| ≥ 2.45` Nm. That fraction must be 0 on every leg
joint. A zero fraction is not a torque pass: the 2.33 Nm signed
ask is still required. The applied-force table is a separate
measurement.

`d6e8b5e` `voice-20` STEPS, Prefer FAIL. Hinge-speed bar passes. Signed ask ≤ 2.33 passes. Sum column passes. Speed-torque passes. Clamp-active passes. Ctrl-clip passes: 0/3801 control ticks, largest |writer command| 1.174 rad, largest |data.ctrl| entering mj_step 1.174 rad. Plant `207f3d5e9c6a72e16f7aa0c8d224f75e` before and `207f3d5e9c6a72e16f7aa0c8d224f75e` after.
Fail reasons: step fraction 0.794 is under 0.90 (airborne forward 0.1536 m, contact forward 0.0399 m); airborne advance 0.0763 m on L at 5.224 s is outside ±20% of vx·T (1.1200 m). vx·T/2 is the stance-to-stance spacing (0.5600 m), not this travel.

kv is `−model.actuator_biasprm[i, 2]`, read per actuator. Legs use dampratio=1.

| Joint | kp | kv |
| --- | ---: | ---: |
| l_hip_yaw | 40.0 | 1.291860 |
| l_hip_roll | 40.0 | 1.702690 |
| l_hip_pitch | 45.0 | 1.810179 |
| l_knee | 45.0 | 1.457342 |
| l_ank_pitch | 35.0 | 1.198019 |
| l_ank_roll | 35.0 | 1.187611 |
| r_hip_yaw | 40.0 | 1.291860 |
| r_hip_roll | 40.0 | 1.702689 |
| r_hip_pitch | 45.0 | 1.810178 |
| r_knee | 45.0 | 1.457341 |
| r_ank_pitch | 35.0 | 1.198019 |
| r_ank_roll | 35.0 | 1.187611 |

Line `|qvel| ≤ 5.82·(1 − |τ|/3.43)`. Voltage 11.1 V is recorded and is not a scale. Writes checked: 49456. The line passes. Worst margin 3.6184 rad/s on l_knee at 24.072 s stage stop (|τ| 1.2447 Nm, |qvel| 0.0896 rad/s, limit 3.7080 rad/s).

DC-motor model, not datasheet (Hiwonder HX-35H page values)

| Joint | Largest \|qvel\| at pre-clamp \|ask\|≥2 Nm | Largest pre-clamp \|ask\| at \|qvel\|≥4 rad/s |
| --- | --- | --- |
| l_hip_yaw | none | none |
| l_hip_roll | none | none |
| l_hip_pitch | none | none |
| l_knee | none | none |
| l_ank_pitch | none | none |
| l_ank_roll | none | none |
| r_hip_yaw | none | none |
| r_hip_roll | none | none |
| r_hip_pitch | none | none |
| r_knee | none | none |
| r_ank_pitch | none | none |
| r_ank_roll | none | none |

| Joint | Peak signed \|τ\| Nm | t s | Peak sum Nm | t s |
| --- | ---: | ---: | ---: | ---: |
| l_hip_yaw | 0.5564 | 23.576 | 0.5564 | 23.576 |
| l_hip_roll | 0.8093 | 15.296 | 2.1076 | 2.840 |
| l_hip_pitch | 1.0807 | 1.800 | 1.7934 | 1.840 |
| l_knee | 1.2456 | 24.096 | 2.2556 | 5.256 |
| l_ank_pitch | 0.7184 | 20.296 | 1.1585 | 23.488 |
| l_ank_roll | 0.4764 | 20.896 | 1.6756 | 2.832 |
| r_hip_yaw | 0.4111 | 2.400 | 0.4111 | 2.400 |
| r_hip_roll | 0.8079 | 5.296 | 2.2567 | 2.840 |
| r_hip_pitch | 0.9065 | 1.680 | 1.4080 | 15.160 |
| r_knee | 0.9832 | 1.640 | 2.2147 | 15.216 |
| r_ank_pitch | 0.7200 | 10.288 | 1.5032 | 24.464 |
| r_ank_roll | 0.6120 | 2.744 | 1.9795 | 2.824 |

Peak signed pre-clamp ask `|kp·(ctrl−q) − kv·q̇|`:

| Joint | Peak |ask| Nm | t s |
| --- | ---: | ---: |
| l_hip_yaw | 0.5564 | 23.576 |
| l_hip_roll | 0.8093 | 15.296 |
| l_hip_pitch | 1.0807 | 1.800 |
| l_knee | 1.2456 | 24.096 |
| l_ank_pitch | 0.7184 | 20.296 |
| l_ank_roll | 0.4764 | 20.896 |
| r_hip_yaw | 0.4111 | 2.400 |
| r_hip_roll | 0.8079 | 5.296 |
| r_hip_pitch | 0.9065 | 1.680 |
| r_knee | 0.9832 | 1.640 |
| r_ank_pitch | 0.7200 | 10.288 |
| r_ank_roll | 0.6120 | 2.744 |

Torque pass, signed pre-clamp ask ≤ 2.33 Nm. The ask passes. The sum column passes.

| Joint | Peak \|ask\| Nm | Ask ticks over 2.33 | Peak sum Nm | Sum ticks over 2.33 | Note |
| --- | ---: | ---: | ---: | ---: | --- |
| l_hip_yaw | 0.5564 | 0/3801 | 0.5564 | 0/3801 |  |
| l_hip_roll | 0.8093 | 0/3801 | 2.1076 | 0/3801 |  |
| l_hip_pitch | 1.0807 | 0/3801 | 1.7934 | 0/3801 |  |
| l_knee | 1.2456 | 0/3801 | 2.2556 | 0/3801 |  |
| l_ank_pitch | 0.7184 | 0/3801 | 1.1585 | 0/3801 |  |
| l_ank_roll | 0.4764 | 0/3801 | 1.6756 | 0/3801 |  |
| r_hip_yaw | 0.4111 | 0/3801 | 0.4111 | 0/3801 |  |
| r_hip_roll | 0.8079 | 0/3801 | 2.2567 | 0/3801 |  |
| r_hip_pitch | 0.9065 | 0/3801 | 1.4080 | 0/3801 |  |
| r_knee | 0.9832 | 0/3801 | 2.2147 | 0/3801 |  |
| r_ank_pitch | 0.7200 | 0/3801 | 1.5032 | 0/3801 |  |
| r_ank_roll | 0.6120 | 0/3801 | 1.9795 | 0/3801 |  |

Clamp-active fraction, `|signed ask| ≥ 2.45` Nm. Hard bar passes.

| Joint | Peak |ask| Nm | Clamped ticks | Fraction |
| --- | ---: | ---: | ---: |
| l_hip_yaw | 0.5564 | 0/3801 | 0.0000 |
| l_hip_roll | 0.8093 | 0/3801 | 0.0000 |
| l_hip_pitch | 1.0807 | 0/3801 | 0.0000 |
| l_knee | 1.2456 | 0/3801 | 0.0000 |
| l_ank_pitch | 0.7184 | 0/3801 | 0.0000 |
| l_ank_roll | 0.4764 | 0/3801 | 0.0000 |
| r_hip_yaw | 0.4111 | 0/3801 | 0.0000 |
| r_hip_roll | 0.8079 | 0/3801 | 0.0000 |
| r_hip_pitch | 0.9065 | 0/3801 | 0.0000 |
| r_knee | 0.9832 | 0/3801 | 0.0000 |
| r_ank_pitch | 0.7200 | 0/3801 | 0.0000 |
| r_ank_roll | 0.6120 | 0/3801 | 0.0000 |

Applied `actuator_force` on the rail. This measurement is not the clamp bar:

| Joint | Rail Nm | Peak applied Nm | Clamped ticks | Fraction |
| --- | ---: | ---: | ---: | ---: |
| l_hip_yaw | -2.45 to 2.45 | 0.5578 | 0/3801 | 0.000 |
| l_hip_roll | -2.45 to 2.45 | 0.8111 | 0/3801 | 0.000 |
| l_hip_pitch | -2.45 to 2.45 | 0.9204 | 0/3801 | 0.000 |
| l_knee | -2.45 to 2.45 | 1.2669 | 0/3801 | 0.000 |
| l_ank_pitch | -2.45 to 2.45 | 0.8022 | 0/3801 | 0.000 |
| l_ank_roll | -2.45 to 2.45 | 0.4708 | 0/3801 | 0.000 |
| r_hip_yaw | -2.45 to 2.45 | 0.4110 | 0/3801 | 0.000 |
| r_hip_roll | -2.45 to 2.45 | 0.8080 | 0/3801 | 0.000 |
| r_hip_pitch | -2.45 to 2.45 | 0.8968 | 0/3801 | 0.000 |
| r_knee | -2.45 to 2.45 | 0.8478 | 0/3801 | 0.000 |
| r_ank_pitch | -2.45 to 2.45 | 0.8010 | 0/3801 | 0.000 |
| r_ank_roll | -2.45 to 2.45 | 0.5057 | 0/3801 | 0.000 |

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

`51ae123` `kitchen-m90` STEPS, Prefer FAIL. Hinge-speed bar passes. Signed ask ≤ 2.33 fails. Sum column fails. Speed-torque fails. Clamp-active fails. Ctrl-clip passes: 0/4583 control ticks, largest |writer command| 1.317 rad, largest |data.ctrl| entering mj_step 1.307 rad. Plant `207f3d5e9c6a72e16f7aa0c8d224f75e` before and `207f3d5e9c6a72e16f7aa0c8d224f75e` after.
Fail reasons: worst stance slip 3.44 mm is over 2 mm; sole clearance min 2.08 mm is under 8 mm; honest max of the per-step minima is 7.88 mm; airborne advance -0.0016 m on L at 16.760 s is outside ±20% of vx·T (0.0280 m). vx·T/2 is the stance-to-stance spacing (0.0140 m), not this travel; stance contact count min 2 is under 3; stop does not end upright: final 1 s min up_z 0.957, contacts L/R 0/0, trunk pitch off the stand by 16.50 deg, roll off by 8.07 deg; actual-stance ZMP margin -4.50785144216012e-05 outside 0.0006545930613135501; actual-stance CoM margin -0.017595349834986795 outside 0.11673576260091643; speed-torque margin -4.5545 rad/s on l_knee at 13.416 s stage ds (|τ| 6.1105 Nm, |qvel| 0.0061 rad/s, limit -4.5483 rad/s); clamp-active fraction 0.0041 on l_hip_pitch is not 0 (19/4583 ticks with |signed ask| ≥ 2.45 Nm); clamp-active fraction 0.2540 on l_knee is not 0 (1164/4583 ticks with |signed ask| ≥ 2.45 Nm); clamp-active fraction 0.0041 on l_ank_pitch is not 0 (19/4583 ticks with |signed ask| ≥ 2.45 Nm); clamp-active fraction 0.0041 on r_hip_pitch is not 0 (19/4583 ticks with |signed ask| ≥ 2.45 Nm); clamp-active fraction 0.2717 on r_knee is not 0 (1245/4583 ticks with |signed ask| ≥ 2.45 Nm); clamp-active fraction 0.0041 on r_ank_pitch is not 0 (19/4583 ticks with |signed ask| ≥ 2.45 Nm); signed pre-clamp ask 2.3833 Nm on l_hip_roll, 16/4583 ticks over 2.33; signed pre-clamp ask 2.9328 Nm on l_hip_pitch, 50/4583 ticks over 2.33; signed pre-clamp ask 6.1105 Nm on l_knee, 1234/4583 ticks over 2.33; signed pre-clamp ask 2.5290 Nm on l_ank_pitch, 19/4583 ticks over 2.33; signed pre-clamp ask 2.4194 Nm on r_hip_roll, 16/4583 ticks over 2.33; signed pre-clamp ask 2.9385 Nm on r_hip_pitch, 19/4583 ticks over 2.33; signed pre-clamp ask 6.1034 Nm on r_knee, 1324/4583 ticks over 2.33; signed pre-clamp ask 2.5290 Nm on r_ank_pitch, 19/4583 ticks over 2.33.

Line `|qvel| ≤ 5.82·(1 − |τ|/3.43)`. Voltage 11.1 V is recorded and is not a scale. Writes checked: 141300. The line fails. Worst margin -4.5545 rad/s on l_knee at 13.416 s stage ds (|τ| 6.1105 Nm, |qvel| 0.0061 rad/s, limit -4.5483 rad/s).

DC-motor model, not datasheet (Hiwonder HX-35H page values)

| Joint | Largest \|qvel\| at pre-clamp \|ask\|≥2 Nm | Largest pre-clamp \|ask\| at \|qvel\|≥4 rad/s |
| --- | --- | --- |
| l_hip_yaw | none | none |
| l_hip_roll | |qvel| 2.0518 rad/s, |τ| 2.2800 Nm at 3.096 s stand | none |
| l_hip_pitch | |qvel| 1.2093 rad/s, |τ| 2.1597 Nm at 4.672 s ss_L | none |
| l_knee | |qvel| 2.5403 rad/s, |τ| 2.2800 Nm at 4.800 s stand | none |
| l_ank_pitch | |qvel| 1.8105 rad/s, |τ| 2.2800 Nm at 8.208 s stand | none |
| l_ank_roll | |qvel| 0.0274 rad/s, |τ| 2.2673 Nm at 8.920 s ds | none |
| r_hip_yaw | none | none |
| r_hip_roll | |qvel| 2.6157 rad/s, |τ| 2.2800 Nm at 17.208 s stand | none |
| r_hip_pitch | |qvel| 1.2460 rad/s, |τ| 2.2293 Nm at 8.216 s stand | none |
| r_knee | |qvel| 2.8032 rad/s, |τ| 2.2800 Nm at 14.422 s stand | none |
| r_ank_pitch | |qvel| 0.3016 rad/s, |τ| 2.0864 Nm at 1.008 s ds | none |
| r_ank_roll | |qvel| 1.6073 rad/s, |τ| 2.2800 Nm at 3.096 s stand | none |

| Joint | Peak signed \|τ\| Nm | t s | Peak sum Nm | t s |
| --- | ---: | ---: | ---: | ---: |
| l_hip_yaw | 0.6967 | 9.304 | 1.5621 | 9.584 |
| l_hip_roll | 4.7889 | 3.104 | 9.2997 | 10.224 |
| l_hip_pitch | 2.9328 | 1.008 | 6.7303 | 18.800 |
| l_knee | 6.1105 | 13.424 | 10.2100 | 19.984 |
| l_ank_pitch | 3.1540 | 14.424 | 5.3247 | 6.504 |
| l_ank_roll | 4.8021 | 17.200 | 7.0705 | 15.664 |
| r_hip_yaw | 0.6624 | 2.152 | 1.7573 | 1.672 |
| r_hip_roll | 5.7488 | 11.016 | 9.2875 | 17.200 |
| r_hip_pitch | 4.4546 | 12.720 | 7.2206 | 35.968 |
| r_knee | 6.1034 | 5.512 | 10.8373 | 14.432 |
| r_ank_pitch | 2.5290 | 1.008 | 5.2431 | 14.160 |
| r_ank_roll | 4.7939 | 10.224 | 6.9000 | 9.200 |

Peak signed pre-clamp ask `|kp·(ctrl−q) − kv·q̇|`:

| Joint | Peak |ask| Nm | t s |
| --- | ---: | ---: |
| l_hip_yaw | 0.6967 | 9.304 |
| l_hip_roll | 2.3833 | 3.816 |
| l_hip_pitch | 2.9328 | 1.008 |
| l_knee | 6.1105 | 13.424 |
| l_ank_pitch | 2.5290 | 1.008 |
| l_ank_roll | 2.2695 | 17.928 |
| r_hip_yaw | 0.6624 | 2.152 |
| r_hip_roll | 2.4194 | 17.928 |
| r_hip_pitch | 2.9385 | 19.624 |
| r_knee | 6.1034 | 5.512 |
| r_ank_pitch | 2.5290 | 1.008 |
| r_ank_roll | 2.2800 | 3.104 |

Torque pass, signed pre-clamp ask ≤ 2.33 Nm. The ask fails. The sum column fails.

| Joint | Peak \|ask\| Nm | Ask ticks over 2.33 | Peak sum Nm | Sum ticks over 2.33 | Note |
| --- | ---: | ---: | ---: | ---: | --- |
| l_hip_yaw | 0.6967 | 0/4583 | 1.5621 | 0/4583 |  |
| l_hip_roll | 2.3833 | 16/4583 | 9.2997 | 1857/4583 |  |
| l_hip_pitch | 2.9328 | 50/4583 | 6.7303 | 1455/4583 |  |
| l_knee | 6.1105 | 1234/4583 | 10.2100 | 2343/4583 |  |
| l_ank_pitch | 2.5290 | 19/4583 | 5.3247 | 1342/4583 |  |
| l_ank_roll | 2.2695 | 0/4583 | 7.0705 | 1328/4583 | passes signed, fails sum |
| r_hip_yaw | 0.6624 | 0/4583 | 1.7573 | 0/4583 |  |
| r_hip_roll | 2.4194 | 16/4583 | 9.2875 | 1975/4583 |  |
| r_hip_pitch | 2.9385 | 19/4583 | 7.2206 | 1622/4583 |  |
| r_knee | 6.1034 | 1324/4583 | 10.8373 | 2513/4583 |  |
| r_ank_pitch | 2.5290 | 19/4583 | 5.2431 | 1412/4583 |  |
| r_ank_roll | 2.2800 | 0/4583 | 6.9000 | 1320/4583 | passes signed, fails sum |

Clamp-active fraction, `|signed ask| ≥ 2.45` Nm. Hard bar fails.

| Joint | Peak |ask| Nm | Clamped ticks | Fraction |
| --- | ---: | ---: | ---: |
| l_hip_yaw | 0.6967 | 0/4583 | 0.0000 |
| l_hip_roll | 2.3833 | 0/4583 | 0.0000 |
| l_hip_pitch | 2.9328 | 19/4583 | 0.0041 |
| l_knee | 6.1105 | 1164/4583 | 0.2540 |
| l_ank_pitch | 2.5290 | 19/4583 | 0.0041 |
| l_ank_roll | 2.2695 | 0/4583 | 0.0000 |
| r_hip_yaw | 0.6624 | 0/4583 | 0.0000 |
| r_hip_roll | 2.4194 | 0/4583 | 0.0000 |
| r_hip_pitch | 2.9385 | 19/4583 | 0.0041 |
| r_knee | 6.1034 | 1245/4583 | 0.2717 |
| r_ank_pitch | 2.5290 | 19/4583 | 0.0041 |
| r_ank_roll | 2.2800 | 0/4583 | 0.0000 |

Applied `actuator_force` on the rail. This measurement is not the clamp bar:

| Joint | Rail Nm | Peak applied Nm | Clamped ticks | Fraction |
| --- | ---: | ---: | ---: | ---: |
| l_hip_yaw | -2.45 to 2.45 | 0.6088 | 0/4583 | 0.000 |
| l_hip_roll | -2.45 to 2.45 | 2.2800 | 0/4583 | 0.000 |
| l_hip_pitch | -2.45 to 2.45 | 1.2761 | 0/4583 | 0.000 |
| l_knee | -2.45 to 2.45 | 2.2800 | 0/4583 | 0.000 |
| l_ank_pitch | -2.45 to 2.45 | 2.2800 | 0/4583 | 0.000 |
| l_ank_roll | -2.45 to 2.45 | 1.8071 | 0/4583 | 0.000 |
| r_hip_yaw | -2.45 to 2.45 | 0.5445 | 0/4583 | 0.000 |
| r_hip_roll | -2.45 to 2.45 | 2.2800 | 0/4583 | 0.000 |
| r_hip_pitch | -2.45 to 2.45 | 2.2800 | 0/4583 | 0.000 |
| r_knee | -2.45 to 2.45 | 2.2800 | 0/4583 | 0.000 |
| r_ank_pitch | -2.45 to 2.45 | 1.2602 | 0/4583 | 0.000 |
| r_ank_roll | -2.45 to 2.45 | 1.7556 | 0/4583 | 0.000 |

Largest sum on `r_knee` is 10.8373 Nm at 14.432 s, stage stand. Signed |τ| on that tick is 1.5977 Nm. Signed pre-clamp ask is 1.5977 Nm. |qvel| at the signed write is 3.1507 rad/s. |qvel| at the sum write is 3.3992 rad/s. stop from a straight walk. The previous second was moving forward, then the bus snapped to stand and the gait holds the stand pose.
Bus mode stand, applied vx 0.0000 m/s, applied yaw 0.0000 rad/s, fault False. That mode runs 14.424–15.120 s. The previous 1 s peaked at |vx| 0.0560 m/s and |yaw| 0.0000 rad/s. Contacts L/R 2/0.
Applied actuator force -0.0190 Nm. Signed pre-clamp ask 1.5977 Nm. Forcerange -2.45 to 2.45 Nm. Applied force on the rail: False. Signed-ask clamp (|ask| ≥ 2.45): False.

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

### Torque signal audit

Torque pass is `|kp·(ctrl−q) − kv·q̇|` ≤ 2.33 Nm on every leg joint
on every tick. kv is `−actuator_biasprm[i, 2]`. The sum column is
`|kp·(q_des−q)| + |kv·ω|`. The #102 scorer's own unclamped-ask
numbers so far are that sum. The signed column below is
`kp·(q_des−q) − kv·ω` on the same posted write, not the pre-clamp ask.

A stored row whose peak sum is ≤ 2.33 and whose over-tick count is 0
passes the sum column. Those older files did not log the pre-clamp
ask, so the signed-ask column says `not logged` and the note is
empty. None of them is `passes signed, fails sum`: the stored sum
does not fail, and a signed pass is not invented.

| Row | Posted Nm | Joint | t s | Sum Nm | q_des signed Nm | Signed ask ≤ 2.33 | Sum ≤ 2.33 | Note |
| --- | ---: | --- | ---: | ---: | ---: | --- | --- | --- |
| d6e8b5e voice-20 | 2.2567 | r_hip_roll | 2.832 | 2.2567 | +0.2545 | pass | pass |  |
| d7b06e7 shape1-3.60 | 2.311 | r_hip_roll | 8.176 | 2.3106 | +0.2587 | not logged | pass |  |
| d7b06e7 shape0-3.60 | 2.042 | r_knee | 9.256 | 2.0415 | +0.7653 | not logged | pass |  |
| d7b06e7 shape0-3.57 | 2.054 | r_knee | 9.248 | 2.0535 | +0.7655 | not logged | pass |  |
| ac81435 voice-3.60 | 2.3135 | r_hip_roll | 8.176 | 2.3135 | +0.2607 | not logged | pass |  |
| ac81435 voice-3.70 | 2.2619 | r_hip_roll | 8.352 | 2.2619 | +0.2534 | not logged | pass |  |
| 58ce1d8 slow | 1.979 | r_hip_roll | 2.816 | 1.9792 | +0.2486 | not logged | pass |  |

`d6e8b5e` is the exception: the logged pre-clamp series passes
≤ 2.33 on every leg joint, and the sum column also passes (peak
2.2567 Nm, 0 ticks over). There is no `passes signed, fails sum` note.
Controls' posted 2.2567 is the sum. The q_des signed value on that
write is +0.2545 Nm.

PRs #82, #88, #90, #98, and #99 did not claim a torque CLEAR. Their
CLEARs are wall-stop CLEARs. #82 and #88 post in-place hip-roll
unclamped asks around ±4 Nm and treat that as over ±2.33 Nm. #90
says hip roll stays unclamped. `51ae123` kitchen is the representative
bout. Its signed-ask pass, sum column, and any `passes signed, fails sum`
joint notes are in the kitchen table above. The 10.17 Nm / 3.61 rad/s
r_knee sample is the sum, not the pre-clamp ask.

## Signed ID a5a9183

Independent score of Controls tip `a5a9183461105016211bebbf110051c2bb8f9df2` (PR #103, `cursor/declared-stance-sync-16df`). The gait is that tip's `voice056` path. This branch only scores. Soft-pass is off. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` before and after every cell. All 12 leg `forcerange` and `actuatorfrcrange` values are ±2.45 Nm. Joint armature is 0.01. Every leg position actuator has `ctrlrange` ±2.09. The loaded model reports `actuator_ctrllimited` = 1 on all 12 leg actuators in every cell. No gait edit and no plant edit.

Bout is the cadence grid: stand 0.25 s, walk `1.00 + 2.05·T`, stop 2.40 s. dsp 0.25, swing height 8 mm, crouch 25 mm, hip pitch 15°, arm 1.00 s, preview shape 0, preview r 1e-4, y_swap 0. `x_amp = vx·T/4`. Sway amplitude is `sway_zmp_amp` with a 16 mm CoM target and a 43 mm cap: T 0.50 s → 43.00 mm, T 0.55 s → 43.00 mm, T 0.60 s → 42.94 mm, T 0.65 s → 37.46 mm, T 0.70 s → 33.54 mm, T 0.75 s → 30.59 mm, T 0.80 s → 28.31 mm, T 1.00 s → 22.85 mm, T 1.20 s → 20.23 mm.

The 2.33 Nm pass is the signed ask on the ctrl the plant uses. MuJoCo clips ctrl to `ctrlrange` before the force when `actuator_ctrllimited` is set, so `ctrl_plant = clip(data.ctrl, ±2.09)` and the ask is `|kp·(ctrl_plant−q) − kv·q̇|` at every physics substep. q and q̇ are the state that entered that `mj_step`. kv is `−actuator_biasprm[i, 2]`. The control-tick value is the max |ask| across that tick's four 2 ms substeps. A tick fails when that value is above 2.33 Nm.

The goal ask stays `|kp·(stored ctrl−q) − kv·q̇|` at the writer, before the hip, knee, and ankle slew. On this gait the knee stores `q_des` already inside ±2.09, so the goal peak equals the q_des signed peak. Those peaks are 3.810–6.210 Nm, every one a knee, with 22–47 over-bar q_des writes. That matches Controls (3.81–6.21 Nm, 22–47 writes). T 0.60 s / vx 0.032 m/s still peaks at 5.820 Nm on the left knee at 1.328 s. The goal ask is the Controls comparison. The torque pass is the plant ask. The writer and the slew both store a ctrl inside ±2.09, so the plant clip leaves every sample unchanged. The plant ask is lower than the goal ask because the slew reaches the goal over several ticks.

A control tick is clipped when any leg writer command before the ctrlrange clip, or any leg `data.ctrl` entering `mj_step`, has `|ctrl_raw| > 2.09`. Exactly ±2.09 is inside the range. The raw command is the writer output before that clip. When that command does not reproduce the stored ctrl, the clip fraction is the string `raw ctrl unavailable` and that string is a fail. Every write reproduced the stored ctrl, so the fraction is the share of clipped control ticks. It is 0 on all 54 rows. The largest |writer command| is 1.159 rad and the largest |data.ctrl| at `mj_step` is 1.158 rad. Any clipped tick is a fail. The clip bar passes all 54.

The sum `|kp·(q_des−q)| + |kv·ω|` is a column. Sum peaks stay 8.014–11.809 Nm, so the sum fails every row. Clamp-active fraction is the share of control ticks with plant `|ask| ≥ 2.45`. The DC-motor line is `|qvel| ≤ 5.82·(1 − |plant ask|/3.43)` on every plant-ask sample, labelled `DC-motor model, not datasheet (Hiwonder HX-35H page values)`. Peak `|qvel|` ≤ 5.82 is a separate bar. Stepping bars, declared-stance ZMP and CoM margins, whole-bout jerk under the #102 baseline, and the final-1 s stop (both feet ≥ 3 contacts, trunk within 5° of the stand median, up_z ≥ 0.90) are the rest of the CLEAR set. Trunk speed ratio is a report.

MuJoCo is 3.14.0. `mjtEnableBit.mjENBL_INVDISCRETE` is present (value 8). The module-level name `mujoco.mjENBL_INVDISCRETE` is absent, so the scorer uses the enum. The flag is set on a binary copy of the model, together with `mjENBL_FWDINV`. The live model's enableflags stay 0. The plant file is not written. Under implicitfast the forward step solves `(M + dt·D)·qacc = f`. The qacc that belongs to that discrete step is `(qvel_next − qvel) / dt` on the pre-step state. `mj_forward` still runs on the copy so the pre-step actuator and passive forces are the ones in the residual. `mj_compareFwdInv` runs on the discrete qacc, then `mj_inverse` runs on that same qacc. The inverse column is `data_copy.qfrc_inverse` only:

```python
def _qfrc_inverse_column(data_copy: mj.MjData) -> np.ndarray:
    """Inverse column for every dof. This reads ``data_copy.qfrc_inverse`` only."""
    return np.array(data_copy.qfrc_inverse, dtype=np.float64, copy=True)
```

```python
"id": _qfrc_inverse_column(copy),
```

That array is what the knee column reads.

The plant timestep is 0.002 s. The integrator is implicitfast on every cell. The solver is the MuJoCo default: Newton, 100 iterations, tolerance 1e-8, Jacobian auto. Cone is pyramidal, impratio is 1, and noslip iterations are 0. The option element does not override cone, impratio, or noslip.

The match residual is `|qfrc_inverse − qfrc_actuator|` on each of the 12 leg joints and each of the 6 root dofs. Exactly 1e-3 Nm stays inside the Prefer FAIL target. Exactly 1e-2 Nm stays inside the bucket. A tick over 1e-3 Nm is a Prefer FAIL count and can still be bucketed when it is at or under 1e-2 Nm. A tick is bucketed only when every one of those residuals is ≤ 1e-2 Nm. Counts over each threshold are both in the table. On the example cell, T 0.60 s / vx 0.032 m/s, the leg match peaks at 1.476e-3 Nm. Controls' nominal on MuJoCo 3.14 with this flag is 1.48e-3 Nm. The same cell's root match peaks at 9.32e-3 Nm. Across the 54 cells the leg match peaks at 3.56e-3 Nm and the root match peaks at 0.0173 Nm. Ticks over 1e-3 Nm are 0–7 per row. Ticks over 1e-2 Nm are 0–3 per row, and on those ticks the leg match stays under 1e-2 Nm while the root match is the one over 1e-2 Nm. Three rows have no tick over 1e-3 Nm (T 0.50 s at 0.016 m/s, T 0.70 s at 0.024 m/s, T 0.65 s at 0.048 m/s).

The passive-inclusive residual `|qfrc_inverse − (qfrc_actuator + qfrc_passive)|` stays a comparison column. It still equals `|qfrc_passive|`, 0.080 Nm per rad/s on every row, and its largest sample is 0.270 Nm. That column is what used to fail 569–762 ticks per row. It does not decide the bucket. The band `0.05 + 0.002·(0.08 + kv_i)·|q̈_i|` and its 0.15 Nm cap stay in the table for comparison. They do not decide the bucket, and they do not relax the signed applied bar. The signed bar still sees every control tick. It fails 42 rows and passes 12. Those 12 still fail the sum. The maximum band on a row is 0.319–0.775 Nm. Ticks counted by band > 0.15 Nm are 59–119 per row. Ticks counted by residual > band are 91–153 per row. The overlap is 31–60 ticks per row. These counts match the previous score. `solver_fwdinv[0]` and `solver_fwdinv[1]` peak at 0.0186.

A joint with `|applied| ≥ 2.45` Nm is `clamped, unclassifiable`. Applied means the signed plant ask or `qfrc_actuator` on that joint. Exactly 2.45 is clamped. The realised-motion inverse returns the clamp, so the sample is not sorted into controller, unsourced-armature, or physics. The armature-stripped value `qfrc_inverse − 0.01·qacc` is reported on the sample and is not a pass. On the example cell the right knee is on the rail for 10 substeps in the stop. At 2.688 s the inverse is +2.450 Nm and the armature-stripped value is +1.302 Nm. At 2.690 s the same rail reads +2.450 Nm and strips to +1.435 Nm. Controls cited r_knee +2.450 at 2.69 s in the stop, armature-stripped 1.302 Nm. That 1.302 Nm sample is the 2.688 s substep, labelled `clamped, unclassifiable`.

On an over-bar sample that is not clamped, a tick whose match residual is over 1e-2 Nm is `residual over 1e-2, not bucketed`. No over-bar sample landed in that class: the ticks over 1e-2 Nm did not carry an unclamped over-bar joint. The remaining samples are the three buckets. Of 376 over-bar substeps, 325 are `clamped, unclassifiable` and 51 are `unsourced-armature candidate`. Controller and physics counts are 0. The 51 sit on 28 rows, and on each of those rows the armature share of the bucketed samples is 1. One of them is T 0.80 s / vx 0.032 m/s, right knee at 3.044 s: plant ask +2.446 Nm, inverse +2.446 Nm, armature-stripped +1.879 Nm. The ask is over 2.33 Nm, so the row does not pass. Stripping armature is what puts the inverse under 2.33 Nm, which is why the class is an unsourced-armature candidate.

The implicitfast offset term is `0.002·(0.08 + kv_i)·q̈_i`, signed. At the sample with the largest |raw| residual, |offset| is 0.0003–0.091 Nm while |raw| is 0.169–0.270 Nm. Subtracting the offset makes the absolute residual larger on all 54 rows.

The knee column is a column bug wherever it is exactly ±2.450 Nm. That is the actuator rail. Nominal peak is +2.450 Nm on the right knee (41 of 54 rows, 325 samples). The other 13 rows are 1.846–2.379 Nm. The closest of those to the old 2.045 Nm is 2.023 Nm, on T 0.55 s at vx 0.056 m/s. The closest to Controls' 0.934 Nm is 1.846 Nm. None of the nominal peaks is 0.934 or 1.087. The knee value on ticks that meet the passive-inclusive 1e-3 Nm gate is 0.678 Nm at 0.168 s, the same stand sample on every row.

Worst case is the same rail. Mass ×0.95 peaks at +2.450 Nm (largest unsaturated sample +2.363 Nm, T 1.20 s / vx 0.016 m/s). Mass ×1.05 peaks at +2.450 Nm (largest unsaturated sample +2.408 Nm, T 1.20 s / vx 0.016 m/s). Seeds 0–9 (σ 0.002 rad on the twelve leg hinges, PCG64) all peak at +2.450 Nm. The largest sample is +2.450001 Nm, seed 1, T 0.75 s, vx 0.056 m/s, right knee at 2.926 s. The entrance rug (`room_entrance.xml`, `mat_rug` placed at runtime, plant file not written) peaks at +2.450 Nm (largest unsaturated sample +2.443 Nm, T 1.00 s / vx 0.056 m/s). That is still the ±2.450 column bug. It is not Controls' 0.934 / 1.087, and it is not the old 2.045.

On the same perturbations the leg match peaks at 4.40e-3 Nm (seed 8). Controls' worst seed is 5.2e-3 Nm. The root match on seeds peaks at 2.57e-2 Nm (seed 4), which is over the 1e-2 Nm bucket. Mass ×0.95 leg match peaks at 3.89e-3 Nm, mass ×1.05 at 3.95e-3 Nm, and the rug at 6.51e-3 Nm. The rug root match peaks at 0.0371 Nm.

The pass stays the real plant. Armature stays 0.01. Signed applied is ≤ 2.33 Nm on every leg joint on every tick, and the limiter fraction is 0. A row that clears 2.33 only after subtracting armature is an unsourced-armature candidate. It is not a pass. Limiter-active ticks are a fail on their own. A control tick is limiter-active when the `write_clipped` band binds on a non-sagittal leg joint, or `write_force_limited` changes the predicted-force command, or the hip, knee, and ankle pitch slew in `_lipm_substep` shortens the move. The Clip column is still the plant ctrlrange test, which stays 0. The fraction is 0.000–0.149. Three rows are at 0 (T 1.20 s at 0.016, 0.024, and 0.032 m/s). They still fail other bars.

The planned-motion check is separate and does not change the pass. `τ_req` is plain `mj_inverse` on the walker's own `q_des`, with `q̇` and `q̈` the backward differences at 8 ms. Contacts are off. The root force `qfrc_inverse[0:3]` is applied through the planned ZMP, so gravity is not added twice. The ZMP point is the body-link subtree CoM x, the preview's instantaneous ZMP y, and z = 0. In double support the wrench is split three ways. The first follows ZMP position along the foot-to-foot line. The second is the min-norm ankle-torque split. The third, called QP in the table, minimises the largest |τ| over the 12 leg joints after `armature·q̈_ref` is removed. Its variables are the local forces at the four bottom corners of each 135×76 mm sole. Those forces sum to the planned root force and produce zero moment about the planned ZMP. Each corner stays in the friction cone of μ 1.2: the normal force is non-negative, and the tangential magnitude is at most 1.2 times the normal. Each foot's centre of pressure stays inside that rectangle shrunk on every side by `step_bars.EDGE_DWELL_M`. The scorer prints that inset as 0.005 m. It is the same inner margin the #102 scorer uses for edge dwell: a CoP closer to the sole edge than this is not clearly inside. The margin to the sole edge is logged on every feasible double-support tick, for each foot that carries weight. A tick is flagged when a reported CoP sits on the shrunk edge, within 0.1 mm of 0.005 m. The flag does not change the pass. The program is the linear epigraph of that maximum. It is solved first on the containing pyramid `|fx| ≤ μ fz` and `|fy| ≤ μ fz`. When that solution leaves the disk, a second solve puts every corner back on the disk. Among disk optima, within 1e-7 Nm of the minimum, the reported wrench minimises the sum of tangential magnitudes. The wall number is the disk minimum. In single support there is one wrench, so the QP column copies it and the spread is 0. The spread column remains the absolute difference of the foot-line and min-norm torques. A root step larger than 5 mm, or a root z step larger than 2 mm, is a foothold relabel. Root velocity is held across that tick. Joint differences stay the raw reference. Those holds are 0–10 per row. The largest root step is 0.122 m, on T 1.20 s at vx 0.056 m/s.

A planned-motion wall is a double-support tick whose QP minimum is above 2.33 Nm without armature. Seven ticks on five rows meet it. The largest is T 0.75 s / vx 0.016 m/s, stop, left hip roll at 3.176 s: −3.112 Nm without armature and −3.928 Nm with it. That row has two such ticks. T 0.80 s / vx 0.024 m/s has the left hip roll at 3.288 s in the stop, −2.625 Nm without armature. T 0.80 s / vx 0.016 m/s has the left hip roll at 3.296 s in the stop, −2.614 Nm without armature. T 0.55 s / vx 0.032 m/s has two ticks; the larger is the right hip pitch at 1.888 s in the walk, +2.467 Nm without armature. T 0.50 s / vx 0.040 m/s has the left hip pitch at 1.480 s in the walk, −2.438 Nm without armature. The example cell's largest double-support QP value is 2.082 Nm, right knee at 1.944 s in the walk. Eighty-nine double-support ticks are infeasible and 23 are unsolved, out of 27,642. Those ticks are counted and left blank in a torque column. Of the 27,530 feasible ticks, 14,334 have a foot on the shrunk edge (17,465 of 55,060 loaded feet). No loaded foot's margin is below 0.005 m by more than 0.1 mm. The per-cell median margin is 0.005–0.011 m. On the example cell, 260 of 489 feasible ticks are flagged, the median margin is 0.0085 m, and the minimum is 0.005 m. The foot-line split and the min-norm split stay in the table. On 66 worst-tick joints those two splits disagree about 2.33 Nm. The largest spread is 4.477 Nm, T 0.70 s / vx 0.024 m/s, stop, left knee: linear +0.351 Nm and min-norm −4.125 Nm.

Single support above 2.33 Nm without armature is required single-foot torque. The grid's largest is T 0.55 s / vx 0.040 m/s, right hip pitch at 1.296 s, −29.875 Nm with armature and −29.868 Nm without. The example cell's same tick is the right hip pitch at 1.296 s, −25.250 Nm on all three columns and −25.255 Nm without armature. Eight of that cell's 116 single-support ticks are above 2.33 Nm without armature. Across the grid, 295 of 7,290 single-support ticks are.

The plant is a position servo, so the double-support force split is whatever the floor contacts produce. After each physics step the scorer sums the world force on each foot, the contact frame transposed times `mj_contactForce`, with world +z upward, and averages the four 2 ms substeps of the control tick. A tick enters the comparison when the plan is double support, the QP solved, and both realised feet have world-z force above 1 N. The vertical share is the right foot's world-z force divided by the sum of the two feet. On the example cell, 479 ticks are compared. The absolute share difference has median 0.049 and maximum 0.382. The Euclidean norm of the six-force difference has median 6.57 N and maximum 48.1 N. That norm also carries the gap between the planned root wrench and the plant's contact total. Across the 54 cells the per-cell share-difference median is 0.027–0.090 and the per-cell maximum is 0.318–0.652. The per-cell force-gap median is 5.64–8.40 N and the per-cell maximum is 12.2–72.2 N. The pass stays signed applied ≤ 2.33 Nm on the real plant, with armature 0.01 and limiter fraction 0.

`τ` is printed with and without `armature·q̈_ref` for every leg joint, worst tick of the start, the walk, and the stop. The table below is the example cell. The QP columns are the third split on that same tick. The phase median, over ticks, of the largest |τ| of the foot-line and min-norm splits is 0.757 Nm in the start, 1.212 Nm in the walk, and 0.802 Nm in the stop. Across the grid those medians are 0.636–0.763 Nm, 1.149–1.265 Nm, and 0.559–0.818 Nm. The phase median of the QP's largest |τ| without armature is 0.535 Nm in the start, 0.997 Nm in the walk, and 0.496 Nm in the stop on the example cell, and 0.535–0.536 Nm, 0.857–1.134 Nm, and 0.491–0.529 Nm across the grid. The cell's root residual after the wrench is removed peaks at 30.5 Nm (3.7–36.1 Nm across the grid). The stop right knee on the example is −5.329 Nm with armature and −0.183 Nm without on the foot-line split, and −5.362 Nm and −0.216 Nm on the min-norm split, spread 0.033 Nm. The QP on that tick is −4.689 Nm with armature and +0.457 Nm without. The foot-line and min-norm samples clear 2.33 only without armature, so the tick stays an unsourced-armature candidate, and the row stays Prefer FAIL.

The planned τ for T 0.60 s / vx 0.032 m/s, worst tick of each phase. QP is the disk minimax on that same tick, with each foot's CoP inside the sole shrunk by the printed 0.005 m inset. In single support the three torque columns match and the spread is 0. The spread is the absolute difference of the foot-line and min-norm torques. The no-armature columns subtract `dof_armature·q̈_ref` with the plant armature 0.01.

| Phase | Joint | t s | Support | Linear Nm | Linear, no armature | Min-norm Nm | Min-norm, no armature | QP Nm | QP, no armature | Spread Nm |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| start | l_hip_yaw | 0.312 | ds | -0.021 | -0.021 | -0.008 | -0.008 | -0.539 | -0.539 | 0.013 |
| start | l_hip_roll | 1.224 | ds | +0.033 | +0.023 | -0.261 | -0.271 | -0.486 | -0.496 | 0.294 |
| start | l_hip_pitch | 0.264 | ds | +0.469 | +0.468 | -0.013 | -0.013 | +0.022 | +0.022 | 0.481 |
| start | l_knee | 0.288 | ds | -0.093 | -0.089 | -0.562 | -0.558 | -0.541 | -0.538 | 0.469 |
| start | l_ank_pitch | 0.400 | ds | -0.117 | -0.128 | +0.332 | +0.322 | +0.237 | +0.226 | 0.450 |
| start | l_ank_roll | 0.344 | ds | -0.184 | -0.157 | -0.035 | -0.009 | +0.042 | +0.068 | 0.148 |
| start | r_hip_yaw | 0.312 | ds | -0.022 | -0.022 | -0.008 | -0.008 | +0.469 | +0.469 | 0.014 |
| start | r_hip_roll | 1.208 | ds | +0.302 | +0.292 | +0.773 | +0.763 | +0.460 | +0.450 | 0.471 |
| start | r_hip_pitch | 1.240 | ds | -1.159 | -1.162 | -0.403 | -0.406 | -0.470 | -0.474 | 0.756 |
| start | r_knee | 1.240 | ds | -0.050 | -0.042 | +1.020 | +1.028 | +0.487 | +0.495 | 1.070 |
| start | r_ank_pitch | 1.240 | ds | +0.317 | +0.319 | -0.680 | -0.679 | -0.497 | -0.495 | 0.997 |
| start | r_ank_roll | 1.240 | ds | +0.282 | +0.272 | +0.046 | +0.035 | +0.506 | +0.495 | 0.236 |
| walk | l_hip_yaw | 1.288 | ss | -0.733 | -0.733 | -0.733 | -0.733 | -0.733 | -0.733 | 0.000 |
| walk | l_hip_roll | 1.704 | ss | -1.401 | -1.375 | -1.401 | -1.375 | -1.401 | -1.375 | 0.000 |
| walk | l_hip_pitch | 2.464 | ss | +9.736 | +9.714 | +9.736 | +9.714 | +9.736 | +9.714 | 0.000 |
| walk | l_knee | 2.472 | ss | -9.536 | -9.505 | -9.536 | -9.505 | -9.536 | -9.505 | 0.000 |
| walk | l_ank_pitch | 2.472 | ss | +3.618 | +3.652 | +3.618 | +3.652 | +3.618 | +3.652 | 0.000 |
| walk | l_ank_roll | 1.944 | ds | -1.317 | -1.308 | -0.035 | -0.026 | -2.091 | -2.082 | 1.282 |
| walk | r_hip_yaw | 1.288 | ss | +3.399 | +3.399 | +3.399 | +3.399 | +3.399 | +3.399 | 0.000 |
| walk | r_hip_roll | 2.072 | ss | +1.279 | +1.279 | +1.279 | +1.279 | +1.279 | +1.279 | 0.000 |
| walk | r_hip_pitch | 1.296 | ss | -25.250 | -25.255 | -25.250 | -25.255 | -25.250 | -25.255 | 0.000 |
| walk | r_knee | 1.288 | ss | +13.864 | +13.875 | +13.864 | +13.875 | +13.864 | +13.875 | 0.000 |
| walk | r_ank_pitch | 2.080 | ss | -3.612 | -3.640 | -3.612 | -3.640 | -3.612 | -3.640 | 0.000 |
| walk | r_ank_roll | 2.312 | ds | +1.288 | +1.266 | +0.040 | +0.018 | +2.084 | +2.061 | 1.248 |
| stop | l_hip_yaw | 2.704 | ds | -0.566 | -0.566 | -0.135 | -0.135 | +0.347 | +0.347 | 0.431 |
| stop | l_hip_roll | 2.704 | ds | -2.647 | -2.436 | -2.116 | -1.904 | -1.077 | -0.866 | 0.532 |
| stop | l_hip_pitch | 2.920 | ds | +2.547 | +2.548 | +1.726 | +1.726 | +0.532 | +0.532 | 0.822 |
| stop | l_knee | 2.584 | ss | -1.588 | -1.592 | -1.588 | -1.592 | -1.588 | -1.592 | 0.000 |
| stop | l_ank_pitch | 2.912 | ds | -1.185 | -0.432 | -0.278 | +0.474 | -0.467 | +0.285 | 0.906 |
| stop | l_ank_roll | 2.696 | ds | -0.552 | -0.408 | -0.117 | +0.027 | -0.756 | -0.612 | 0.435 |
| stop | r_hip_yaw | 2.920 | ds | -0.014 | -0.014 | -0.087 | -0.087 | -0.039 | -0.039 | 0.073 |
| stop | r_hip_roll | 2.696 | ds | +0.466 | +0.260 | +0.807 | +0.600 | +0.819 | +0.612 | 0.341 |
| stop | r_hip_pitch | 2.696 | ds | +2.643 | +0.223 | +2.967 | +0.547 | +2.943 | +0.523 | 0.324 |
| stop | r_knee | 2.696 | ds | -5.329 | -0.183 | -5.362 | -0.216 | -4.689 | +0.457 | 0.033 |
| stop | r_ank_pitch | 2.504 | ss | +2.636 | +0.021 | +2.636 | +0.021 | +2.636 | +0.021 | 0.000 |
| stop | r_ank_roll | 2.672 | ss | -0.393 | -0.012 | -0.393 | -0.012 | -0.393 | -0.012 | 0.000 |

Armature 0.025 is a load-time sensitivity, not a plant edit. The scorer checks the plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`, loads that XML through `mujoco.MjSpec`, sets armature 0.025 on the twelve leg joints, and compiles. `dampratio=1` recomputes `actuator_biasprm` kv at that compile. `model.dof_armature` is not written after compile. The file is not written. The label is `sensitivity, HW cited bound 0.0012–0.045 (STS3215 SysID 0.022–0.026), not plant`. It is informational while a row is still an intermediate result. It is a hard gate before that row is called locked. A row that is CLEAR at 0.01 and not CLEAR at 0.025 is `transfer risk`. The pass on the plant stays armature 0.01.

The scorer prints kv at 0.01 and 0.025 side by side and stops if any leg matches. Every joint moved:

| Joint | kv at 0.01 | kv at 0.025 |
| --- | ---: | ---: |
| l_hip_yaw | 1.291860 | 2.017152 |
| l_hip_roll | 1.702690 | 2.301989 |
| l_hip_pitch | 1.810179 | 2.444739 |
| l_knee | 1.457342 | 2.196326 |
| l_ank_pitch | 1.198019 | 1.880226 |
| l_ank_roll | 1.187611 | 1.873612 |
| r_hip_yaw | 1.291860 | 2.017152 |
| r_hip_roll | 1.702689 | 2.301988 |
| r_hip_pitch | 1.810178 | 2.444738 |
| r_knee | 1.457341 | 2.196325 |
| r_ank_pitch | 1.198019 | 1.880226 |
| r_ank_roll | 1.187611 | 1.873612 |

The bout is the best row of the committed grid and that bout's stop, on the full bar set, at both armatures. The best row is T 0.50 s / vx 0.016 m/s: seven fail reasons, the signed ask passes, and the match residual stays under 1e-3 Nm. The 0.01 rescore matches that committed cell. The stop is the final 1 s of the same bout.

At 0.01 the row is Prefer FAIL STEPS. Signed ask peaks at 1.850 Nm on the left knee, with the note `passes signed, fails sum`. The stop is not upright: min up_z 0.998, contacts 4/4, trunk pitch 5.18° off the stand, roll 0.20° off. It freezes in a lean. Match residual peaks at 2.91e-4 Nm, 0 ticks over 1e-3. Limiter fraction 0.125 (73/585). Double-support QP wall ticks are 0 of 508 feasible (3 infeasible). The largest stop QP value without armature is +1.294 Nm, left hip roll at 2.488 s. Single support above 2.33 Nm without armature is required single-foot torque: right hip pitch at 2.016 s, −9.010 Nm with armature and −8.983 Nm without. CoP flags 129 of 508 feasible ticks, minimum margin 0.005 m, median 0.011 m. Realised versus QP: 496 ticks, share-difference median 0.027 and maximum 0.504, force-gap median 5.64 N and maximum 33.0 N.

At 0.025 the same row is Prefer FAIL STEPS. It is not `transfer risk`: the row does not clear at 0.01. It is not locked. The signed ask now fails, 3.694 Nm on the left knee at 2.288 s in the stop (6/585 ticks) and 2.353 Nm on the right knee (1/585). The clamp bar fails, 4/585 ticks on the left knee. The DC-motor line fails on that same stop sample, margin −0.963 rad/s. Limiter fraction 0.181 (106/585). The stop pitch off the stand is 6.59°, roll 0.08°, min up_z 0.996, contacts 4/4, still a lean. Match residual peaks at 4.15e-4 Nm, leg match 5.95e-5 Nm, 0 ticks over 1e-3. The knee inverse sits on the rail, −2.450 Nm at 2.288 s, and strips to −0.041 Nm. Eight over-bar samples are clamped and eight are unsourced-armature candidates. Double-support QP wall ticks stay 0, of 506 feasible (5 infeasible). CoP flags 153 of 506, minimum margin 0.005 m, median 0.0085 m. Realised versus QP: 492 ticks, share-difference median 0.034 and maximum 0.615, force-gap median 6.74 N and maximum 32.1 N. Phase medians of the largest |τ| without armature on the QP are 0.534 Nm in the start, 0.924 Nm in the walk, and 0.522 Nm in the stop. The foot-line and min-norm medians are 0.770 / 1.323 / 0.648 Nm.

Single support above 2.33 Nm without armature is still required single-foot torque. The largest is the right hip roll at 2.176 s in the walk, −6.291 Nm with armature and +10.442 Nm without. The largest with-armature sample is the left hip roll at 2.184 s, +45.140 Nm with armature and +7.083 Nm without. The three columns match and the spread is 0. Neither tick is a double-support wall.

The planned τ for the stop at armature 0.025, worst tick of each joint. QP is the disk minimax on that same tick, with each foot's CoP inside the sole shrunk by 0.005 m. In single support the three torque columns match. The no-armature columns subtract `dof_armature·q̈_ref` with the compiled armature 0.025.

| Phase | Joint | t s | Support | Linear Nm | Linear, no armature | Min-norm Nm | Min-norm, no armature | QP Nm | QP, no armature | Spread Nm |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| stop | l_hip_yaw | 2.544 | ds | +0.183 | +0.183 | +0.094 | +0.094 | +0.412 | +0.412 | 0.089 |
| stop | l_hip_roll | 2.544 | ds | +2.008 | +0.726 | +2.553 | +1.271 | +2.471 | +1.189 | 0.544 |
| stop | l_hip_pitch | 2.280 | ss | +7.285 | +0.238 | +7.285 | +0.238 | +7.285 | +0.238 | 0.000 |
| stop | l_knee | 2.280 | ss | -15.339 | -0.698 | -15.339 | -0.698 | -15.339 | -0.698 | 0.000 |
| stop | l_ank_pitch | 2.288 | ss | -11.595 | -0.345 | -11.595 | -0.345 | -11.595 | -0.345 | 0.000 |
| stop | l_ank_roll | 2.544 | ds | +1.124 | -0.157 | +1.288 | +0.006 | +1.328 | +0.046 | 0.164 |
| stop | r_hip_yaw | 2.544 | ds | +0.633 | +0.633 | +0.178 | +0.178 | -0.044 | -0.044 | 0.455 |
| stop | r_hip_roll | 2.544 | ds | +3.334 | +2.030 | +2.762 | +1.458 | +2.493 | +1.189 | 0.572 |
| stop | r_hip_pitch | 2.304 | ds | -2.280 | -2.221 | -0.669 | -0.610 | -1.398 | -1.338 | 1.611 |
| stop | r_knee | 2.304 | ds | +0.390 | +0.099 | +2.460 | +2.169 | +1.629 | +1.338 | 2.070 |
| stop | r_ank_pitch | 2.536 | ds | -1.013 | +0.253 | -1.775 | -0.509 | -1.805 | -0.539 | 0.762 |
| stop | r_ank_roll | 2.544 | ds | +1.243 | -0.061 | +1.292 | -0.012 | +1.618 | +0.314 | 0.049 |

The stop left knee clears 2.33 Nm only after the 0.025 armature term is removed. That sample is an unsourced-armature candidate on the planned columns. The plant ask on the same stop is 3.694 Nm, so the row stays Prefer FAIL. Full table: `previews/walk_a5a9183_armature_sensitivity.json`.

Planned-ID check on voice056 at T 1.00 s / vx 0.016 m/s, plant armature 0.01. The plant md5 stayed `207f3d5e9c6a72e16f7aa0c8d224f75e`. A hip-roll torque is not a discard. The only inconsistency flags are a root residual over 1e-2 Nm, or the planned CoM or the planned ZMP outside the support polygon of the feet that are down.

The six root rows are printed for all 711 control ticks. `qfrc` is `mj_inverse` on the reference. `resid` is that column minus the generalized force of the planned ZMP wrench. The three force rows of `resid` are at most 4e-15 N on every tick, so the wrench carries the inverse force. The three moment rows are not at the noise floor. Their peaks are 7.46 Nm, 27.28 Nm, and 2.43 Nm. All 711 ticks exceed 1e-2 Nm. The 27.28 Nm sample is ty at 1.320 s, paired with the previous tick at 1.312 s, where the inverse force in x is ±136 N. That is a qacc spike, not the stance. On walk single support with |fx| under 40 N the median peak residual is 0.33 Nm. The root flag is set.

The y trace is the second walk cycle, 2.248 s through 3.240 s. Right stance in that cycle runs 2.504 s to 2.872 s. Preview CoM y, the placed body-link CoM y, the planned ZMP y, and the live CoM y are all negative together on every tick of that stance. The sign is the right foot. It is not flipped. The planned ZMP hold of −0.0229 m is the T 1.00 s sway amplitude already listed above, 22.85 mm, not the 43 mm box centre. At the zero pose the hip-roll axis is at body y ±0.029 and the sole centre is at ±0.043, so the hip sits 14 mm inboard of the box centre. At 2.592 s the placed hip-roll axis is at −0.0587 m and the stance box centre is at −0.0478 m, so the hip is 11 mm outboard of the box. The sole has moved about 25 mm inboard relative to the hip. That is the preview offset on top of the outboard sole, and it is the same sign as the stance.

| t s | preview CoM y | placed CoM y | planned ZMP y | box centre y | hip-roll axis y | live CoM y | live CoP y | bare r_hip_roll Nm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2.504 | -0.0208 | -0.0153 | -0.0205 | -0.0478 | -0.0486 | -0.0095 | +0.0130 | +0.351 |
| 2.592 | -0.0314 | -0.0232 | -0.0229 | -0.0478 | -0.0587 | -0.0217 | -0.0495 | +1.491 |
| 2.688 | -0.0254 | -0.0189 | -0.0229 | -0.0478 | -0.0531 | -0.0250 | -0.0614 | +0.851 |
| 2.816 | -0.0116 | -0.0088 | -0.0229 | -0.0478 | -0.0401 | -0.0097 | -0.0217 | +0.419 |

The planned ZMP stays inside the stance sole for the whole bout (0 ticks outside). The placed CoM leaves that sole on 27 ticks, 23 in the walk and 4 in the stop. The support flag is set. The first of those walk ticks in this cycle is 2.816 s, margin −0.2 mm. The stop samples at 3.440–3.464 s put the placed CoM at about +0.023 m while the right box is still at −0.048 m.

+2.75 Nm does not occur. The largest walk right-stance bare hip-roll torque is +1.491 Nm at 2.592 s, not a 2.75. The sample-mid of that stance, 2.688 s, is +0.851 Nm. The split at 2.592 s is gravity −0.053 Nm, M·q̈ +0.137 Nm of which the bare inertial part is −0.012 Nm, velocity −0.012 Nm, and contact +1.568 Nm. The swing leg does not appear in `qfrc_bias` at this joint: those three swing differences are 0. It does change the root force. Applying the swing-free root force at the same planned ZMP removes 0.251 Nm, so the swing leg is 0.251 Nm of the contact term. The sum closes. A vertical force anywhere on that sole, plus gravity, spans −0.74 Nm to +1.13 Nm. The contact term +1.57 Nm sits above that span.

The point on z = 0 that minimises the root residual at 2.592 s is (0.012 m, −0.068 m). It is inside the sole, and the root residual there is 0.039 Nm. Bare hip-roll torque at that point is +0.464 Nm, inside the vertical span. The planned ZMP y is −0.023 m, 45 mm inboard of that point, and the root residual of the planned wrench is 1.07 Nm. The +1.03 Nm between +1.491 and +0.464 is that moment. Both points are on the sole. The flag is the root residual, not the torque.

At the sample-mid, 2.688 s, the same split is gravity −0.038, inertia +0.026, velocity +0.051, contact +0.840, swing wrench +0.165. Bare torque is +0.851 Nm. The best point is 4 mm from the planned ZMP and the bare torque there is +0.760 Nm. The vertical span is −0.84 Nm to +1.01 Nm.

The stop tick at 3.448 s reaches bare −4.941 Nm. The placed CoM is outside the right sole by 36 mm. That tick is flagged for the CoM and for the root residual. The torque is not the flag.

On this same bout the live signed ask on r_hip_roll peaks at 0.823 Nm, 0 ticks over 2.33. The DC-motor line fails on r_knee at 3.456 s in the stop, margin −1.697 rad/s. The stop is a lean: min up_z 0.994, contacts 4/4, pitch 8.14° off the stand, roll 0.29° off. The row is Prefer FAIL STEPS. Full tick list: `previews/walk_a5a9183_plan_y.json`.

That rollout is pinned in `previews/run_manifest_a5a9183_voice056.json`. Walker tip `a5a9183461105016211bebbf110051c2bb8f9df2`. Scorer `d4a38cc`. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. MuJoCo `3.14.0`. Seed is unset. Planned-ID feedforward is off. There is no knee q̈ cap. Armature 0.01 is the plant XML at compile. Ctrl is the position target after the 20 ms sagittal slew and the ±2.09 clip. The stop is the preview blend: finish the airborne swing, hold 0.20 s in double support, return the ZMP over 1.50 s, and smootherstep the joints to the stand over 2.0 s, with the sole-pitch null capped at ±0.12 rad. Duration is stand 0.25 s, walk 3.05 s, stop 2.40 s. Both feet stay loaded and the roll offset is 0.29°, so this bout does not tip.

The compiled plant is in that file under `xml_md5`, `compiled_md5`, `dof_armature`, `kv`, `kp`, `forcerange`, `actfrcrange`, `ctrlrange`, `mujoco_version`, `tip_sha`, `scorer_sha`, `seed`, `args`, and `cmd`, with `perturbation` `none`. `compiled_md5` is `03ed33386178ab8d05db76a7307f1c1d`. It is the md5 of 253 little-endian float64 values, 2024 bytes. The order is the twelve leg joints' `dof_armature`, `kv` (−actuator_biasprm[:, 2]), `kp` (actuator_gainprm[:, 0]), `forcerange`, `actfrcrange`, and `ctrlrange` as lo, hi; then every `body_mass`; then every `body_inertia` as ix, iy, iz; then `geom_friction` of floor, r_foot_contact, and l_foot_contact as slide, torsion, roll; then every `dof_damping`; then `opt.timestep`; then `opt.integrator` as a float64 of the mjtIntegrator value (`mjINT_IMPLICITFAST` is 3). Joint order is the compiled model's joint ids: r_hip_yaw, r_hip_roll, r_hip_pitch, r_knee, r_ank_pitch, r_ank_roll, l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee, l_ank_pitch, l_ank_roll. Body order is ascending body id, world first. Dof order is ascending dof id: root dofs 0–5, then the hinges. Each leg value is read from that joint's `_pos` actuator. The model is `MjModel.from_xml_path` on the tip plant at armature 0.01. An MjSpec compile that sets those twelve armatures to 0.01 and does not write `dof_armature` afterwards has the same `compiled_md5`. PR #103 at `6bae07e` has no scripts/ manifest under other names.

The armature 0.025 reference is `previews/compiled_md5_armature_0.025.json`, perturbation `armature_0.025`. `compiled_md5` is `f4fb2b70b5312a851e16316a38a88283`. It uses the same 253 little-endian float64 values, 2024 bytes, and the same joint, body, geom, and dof order. The model is `MjSpec.from_file` with those twelve leg armatures set to 0.025 before compile. `model.dof_armature` is not written afterwards. The XML file is not written. Against baseline `03ed33386178ab8d05db76a7307f1c1d`, only `dof_armature` and `kv` change. `compile_plant` on PR #103 at `6bae07e` hashes the same at 0.01 and at 0.025.

Both hashes are keyed by the exact `mujoco.__version__` in `previews/compiled_refs.json`. On this build that string is `3.14.0`. `armature_0.01` is `03ed33386178ab8d05db76a7307f1c1d` and `armature_0.025` is `f4fb2b70b5312a851e16316a38a88283`. `check_compiled` in `scripts/compiled_plant_md5.py` compares `compiled_md5` first. On a mismatch it compares the 253 values, one compiled_order field at a time, at rtol 1e-9 with atol 0, and names the field and the index inside that field.

PR #103 entry point `scripts/run_live_row.py` at `6bae07e` ran both configs with those same arguments. Armature 0.01 goes through `MjSpec` before kv. The baseline sets feedforward 0 and knee q̈ cap none. The other row sets feedforward 1 and knee q̈ cap 40. On the baseline the live r_hip_roll ask peaks at +5.645 Nm at 3.208 s, 30 ticks over 2.33, and the bout tips: min up_z 0.849, pitch −30.0°, fault `COM outside support and tipping margin=-0.068`. Signed, sum, DC, clamp, margins, and stepping all fail. Ctrl-clip is 0. The run stops at 562 ticks. The feedforward row aborts at 0.256 s in the start, planned root residual over 0.01 Nm, so it has no live r_hip_roll and no tip call. Plant md5 held. `6bae07e` shadows the ask-over list with the loop variable `over`, and the unpatched baseline raises TypeError after the rollout. The baseline JSON renamed that variable to `tick_over` in the checkout. The gait was not edited. Files: `previews/run_live_row_voice056_pair.json`, `previews/run_live_row_a5a9183_baseline.json`, `previews/run_live_row_ff_cap.json`.

SHA `d4a38cc`. Logs from SHA `424b3d0`. Prior goal-ask score SHA `697d53b`.
