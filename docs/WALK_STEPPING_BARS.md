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

`d6e8b5e` `voice-20` STEPS, Prefer FAIL. Hinge-speed bar passes. Signed ask ≤ 2.33 passes. Sum column passes. Speed-torque passes. Clamp-active passes. Plant `207f3d5e9c6a72e16f7aa0c8d224f75e` before and `207f3d5e9c6a72e16f7aa0c8d224f75e` after.
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

`51ae123` `kitchen-m90` STEPS, Prefer FAIL. Hinge-speed bar passes. Signed ask ≤ 2.33 fails. Sum column fails. Speed-torque fails. Clamp-active fails. Plant `207f3d5e9c6a72e16f7aa0c8d224f75e` before and `207f3d5e9c6a72e16f7aa0c8d224f75e` after.
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

q̈ is `data.qacc` from the `mj_step` that applied this ctrl, not a finite difference of qvel. Each control tick is four 2 ms substeps. Inverse dynamics runs on every substep whose plant |ask| exceeds 2.33, at the q and q̇ that entered that `mj_step`, with the plant-clipped ctrl and the contacts of that state. `qfrc_inverse` is the ID torque. The split is armature·q̈ = `0.01·qacc` (it matches `dof_armature·qacc`), impact/contact = `−qfrc_constraint`, link inertia = `(M·qacc − armature·qacc) + (qfrc_bias(q,v) − qfrc_bias(q,0))`, gravity = `qfrc_bias` at zero velocity. Those terms plus `−qfrc_passive` reproduce `qfrc_inverse` (max residual 9e-16 Nm). The hand estimate is `m·g·L·sin(|q|/2)` with m 2.2 kg and L 0.093 m, labelled hand estimate. The over-bar split has three buckets. Each test uses the absolute value of the signed torque, and exactly 2.33 stays on the low side of each greater-than test. A substep with `|qfrc_inverse| ≤ 2.33` and `|signed ask| > 2.33` is a controller fail. A substep with `|qfrc_inverse| > 2.33` and `|qfrc_inverse − armature·q̈| ≤ 2.33` is an unsourced-armature candidate. Armature is 0.01 and has no Hiwonder source. That bucket is not a physics candidate. A substep with `|qfrc_inverse − armature·q̈| > 2.33` is a physics candidate, named by the largest of impact/contact, link inertia, and gravity.

Plant peaks are 1.846–4.064 Nm, every one a knee. Twelve rows pass the plant signed ask and fail the sum: T 0.50 s and T 0.55 s at every vx from 0.016 to 0.056 m/s. Their plant peaks are 1.846–2.023 Nm and their sum peaks are 10.557–10.949 Nm. The note on each of those rows is `passes signed, fails sum`. They have no over-bar substep, so all three class fractions and all three worst ticks are empty. The note stays visible. It is not a fail reason. Those rows stay Prefer FAIL on the other bars.

The other 42 rows fail the plant signed ask. Those failures are 174 control ticks and 376 substeps. All 376 are unsourced-armature candidates. The controller-fail count is 0 and the physics-candidate count is 0. On every one of the 376, `|qfrc_inverse|` is above 2.33 and `|qfrc_inverse − armature·q̈|` is between 0.093 and 2.091 Nm. Removing the 0.01 armature term brings the inverse torque back to 2.33 or under. The largest of the three remaining terms is impact/contact on 354 substeps, link inertia on 16, and gravity on 6. That name is the remainder. It does not put the substep in the physics bucket. 325 of the 376 ID torques sit on the ±2.45 Nm actuator rail. The other 51 lie between 2.331 and 2.446 Nm. The worst unsourced-armature tick on each of those 42 rows, ranked by `|armature·q̈|`, is a right knee, in touchdown or in the stop. `|armature·q̈|` on that tick is 0.825–2.357 Nm. Controller-fail and physics worst ticks are empty on every row.

Every row is Prefer FAIL. qvel ≤ 5.82 passes all 54. The clip bar passes all 54. The DC line and the clamp pass on 13 rows and fail on 41. The 13 are the twelve `passes signed, fails sum` rows plus T 1.20 s / 0.016 m/s, whose plant peak is 2.379 Nm (one tick over 2.33, under 2.45, inside the DC line). Two rows are SKATES (T 1.20 s at 0.048 and 0.056 m/s, step fraction 0.482 and 0.436). The other 52 are STEPS, and every step fraction is under 0.90 (max 0.780). Slip is 0.58–1.79 mm, under 2 mm. Lowest-corner clearance over 20–80% of swing is under 8 mm on every scored swing. Declared-stance ZMP margin is negative on every row. No stop is upright on the final-1 s bar: the closest trunk pitch is 4.96° off the stand (T 0.55 s, vx 0.056 m/s) with a left-foot contact count of 2, and the largest pitch excursion is 8.96°.

On the example row (T 0.60 s, vx 0.032 m/s) the goal peak is still +5.820 Nm on the left knee at 1.328 s. The plant peak is +3.854 Nm on the right knee at 2.680 s, in the stop. The ctrl at that step is −0.887 rad, inside ±2.09. Five of 610 control ticks are over 2.33, across 10 substeps, and all 10 are unsourced-armature candidates. On the plant-peak substep, `kp·e` is +6.102 Nm, `kv·q̇` is +2.249 Nm, and ID is +2.450 Nm. Armature·q̈ is +1.611 Nm, so `qfrc_inverse − armature·q̈` is +0.839 Nm. Impact/contact is +0.827 Nm, link inertia −0.015 Nm, gravity −0.097 Nm. CoP relative to the right ankle is −56.5 mm forward and −22.5 mm lateral. Ctrl second difference is +0.062 rad. The hand estimate is 0.982 Nm. The worst unsourced-armature tick on this row, ranked by `|armature·q̈|`, is the right knee at 2.496 s in the stop: signed ask −3.147 Nm, ID −2.450 Nm, armature·q̈ −2.171 Nm, remainder −0.279 Nm, impact/contact −0.048 Nm, link inertia −0.096 Nm, gravity −0.107 Nm. The DC-line margin on that row is −2.851 rad/s on the right knee at 2.688 s (`|τ|` 3.633 Nm, `|qvel|` 2.506 rad/s, limit −0.345 rad/s). Body-forward ratio 1.018.

Per-substep logs, including CoP, ctrl second difference, the hand estimate, the four ID terms, and the three bucket worst ticks, are in `previews/walk_a5a9183_signed_id.json`. Plant is the plant-ask peak. Goal is the writer ask peak, the Controls comparison. Clip is the ctrl-clip fraction (`|ctrl_raw| > 2.09`). Ticks is the number of control ticks over 2.33. Sub is the number of over-bar substeps. Ctrl, Arm, and Phys are the fractions of Sub in the controller-fail, unsourced-armature, and physics buckets. Note is `passes signed, fails sum` when the row passes the plant ask and fails the sum. Arm joint, phase, Arm Nm, ID Nm, and Rest Nm are the worst unsourced-armature tick, ranked by `|armature·q̈|`. Rest Nm is `qfrc_inverse − armature·q̈`. Controller-fail and physics worst ticks are empty on every row.

| T s | vx | Plant Nm | Joint | Goal Nm | Clip | Ticks | Sub | Sum Nm | Note | Ctrl | Arm | Phys | Arm joint | Phase | Arm Nm | ID Nm | Rest Nm |
| ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | --- | ---: | ---: | ---: |
| 0.80 | 0.016 | 3.383 | r_knee | 4.560 | 0 | 4 | 8 | 11.10 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.463 | +2.45 | +0.987 |
| 0.80 | 0.024 | 3.259 | r_knee | 4.613 | 0 | 4 | 6 | 10.83 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.297 | +2.42 | +0.126 |
| 0.80 | 0.032 | 3.412 | r_knee | 4.830 | 0 | 4 | 8 | 10.93 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.450 | +2.42 | +0.973 |
| 0.80 | 0.040 | 3.703 | r_knee | 4.934 | 0 | 4 | 9 | 11.17 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.355 | +2.45 | +1.095 |
| 0.80 | 0.048 | 3.726 | r_knee | 5.031 | 0 | 4 | 11 | 10.80 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.312 | +2.45 | +1.138 |
| 0.80 | 0.056 | 3.923 | r_knee | 5.113 | 0 | 4 | 13 | 11.05 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.289 | +2.45 | +1.161 |
| 0.75 | 0.016 | 3.025 | r_knee | 4.857 | 0 | 4 | 5 | 10.75 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.331 | +2.45 | +0.119 |
| 0.75 | 0.024 | 2.802 | r_knee | 4.906 | 0 | 3 | 3 | 10.75 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.334 | +2.45 | +0.116 |
| 0.75 | 0.032 | 2.853 | r_knee | 4.956 | 0 | 3 | 4 | 10.77 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.337 | +2.45 | +0.113 |
| 0.75 | 0.040 | 3.072 | r_knee | 5.032 | 0 | 4 | 5 | 10.52 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.343 | +2.45 | +0.107 |
| 0.75 | 0.048 | 3.434 | r_knee | 5.138 | 0 | 4 | 7 | 10.56 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.346 | +2.45 | +0.104 |
| 0.75 | 0.056 | 3.862 | r_knee | 5.226 | 0 | 4 | 12 | 10.86 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.338 | +2.45 | +1.112 |
| 0.70 | 0.016 | 3.678 | r_knee | 5.055 | 0 | 4 | 8 | 11.50 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.575 | +2.45 | +0.875 |
| 0.70 | 0.024 | 3.408 | r_knee | 5.099 | 0 | 3 | 6 | 11.24 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.335 | +2.45 | +0.115 |
| 0.70 | 0.032 | 3.470 | r_knee | 5.145 | 0 | 4 | 8 | 11.23 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.340 | +2.45 | +0.110 |
| 0.70 | 0.040 | 3.706 | r_knee | 5.190 | 0 | 4 | 8 | 11.43 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.565 | +2.45 | +0.885 |
| 0.70 | 0.048 | 3.919 | r_knee | 5.234 | 0 | 4 | 10 | 11.74 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.517 | +2.45 | +0.933 |
| 0.70 | 0.056 | 4.064 | r_knee | 5.338 | 0 | 4 | 11 | 11.81 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.455 | +2.45 | +0.995 |
| 0.65 | 0.016 | 3.428 | r_knee | 5.248 | 0 | 4 | 7 | 11.27 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.346 | +2.45 | +0.104 |
| 0.65 | 0.024 | 3.575 | r_knee | 5.293 | 0 | 3 | 7 | 11.39 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.347 | +2.45 | +0.103 |
| 0.65 | 0.032 | 3.389 | r_knee | 5.339 | 0 | 3 | 6 | 11.26 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.354 | +2.45 | +0.096 |
| 0.65 | 0.040 | 3.568 | r_knee | 5.387 | 0 | 3 | 7 | 11.28 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.357 | +2.45 | +0.093 |
| 0.65 | 0.048 | 3.991 | r_knee | 5.435 | 0 | 4 | 11 | 11.58 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.583 | +2.45 | +0.867 |
| 0.65 | 0.056 | 4.029 | r_knee | 5.476 | 0 | 4 | 11 | 11.56 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +1.519 | +2.45 | +0.931 |
| 0.60 | 0.016 | 3.514 | r_knee | 5.737 | 0 | 5 | 10 | 11.26 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.347 | +2.45 | +0.103 |
| 0.60 | 0.024 | 3.846 | r_knee | 5.778 | 0 | 5 | 10 | 11.47 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | -2.167 | -2.45 | -0.283 |
| 0.60 | 0.032 | 3.854 | r_knee | 5.820 | 0 | 5 | 10 | 11.37 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | -2.171 | -2.45 | -0.279 |
| 0.60 | 0.040 | 3.843 | r_knee | 5.860 | 0 | 5 | 10 | 11.21 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | -2.174 | -2.45 | -0.276 |
| 0.60 | 0.048 | 3.806 | r_knee | 5.904 | 0 | 5 | 10 | 11.29 | — | 0.00 | 1.00 | 0.00 | r_knee | touchdown | +2.274 | +2.45 | +0.176 |
| 0.60 | 0.056 | 3.971 | r_knee | 5.954 | 0 | 5 | 10 | 11.57 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | -2.156 | -2.45 | -0.294 |
| 0.55 | 0.016 | 1.846 | l_knee | 5.932 | 0 | 0 | 0 | 10.56 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.55 | 0.024 | 1.881 | l_knee | 5.969 | 0 | 0 | 0 | 10.60 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.55 | 0.032 | 1.917 | l_knee | 6.006 | 0 | 0 | 0 | 10.64 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.55 | 0.040 | 1.944 | l_knee | 6.041 | 0 | 0 | 0 | 10.69 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.55 | 0.048 | 1.982 | l_knee | 6.084 | 0 | 0 | 0 | 10.75 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.55 | 0.056 | 2.023 | l_knee | 6.131 | 0 | 0 | 0 | 10.82 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.50 | 0.016 | 1.850 | l_knee | 6.012 | 0 | 0 | 0 | 10.69 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.50 | 0.024 | 1.883 | l_knee | 6.048 | 0 | 0 | 0 | 10.72 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.50 | 0.032 | 1.915 | l_knee | 6.085 | 0 | 0 | 0 | 10.77 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.50 | 0.040 | 1.938 | l_knee | 6.117 | 0 | 0 | 0 | 10.81 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.50 | 0.048 | 1.974 | l_knee | 6.161 | 0 | 0 | 0 | 10.88 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 0.50 | 0.056 | 2.012 | l_knee | 6.210 | 0 | 0 | 0 | 10.95 | passes signed, fails sum | — | — | — | — | — | — | — | — |
| 1.00 | 0.016 | 3.306 | r_knee | 4.121 | 0 | 4 | 8 | 10.50 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.266 | +2.45 | +1.184 |
| 1.00 | 0.024 | 3.346 | r_knee | 4.257 | 0 | 4 | 8 | 10.73 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.264 | +2.45 | +1.186 |
| 1.00 | 0.032 | 3.507 | r_knee | 4.382 | 0 | 4 | 9 | 11.00 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.234 | +2.45 | +1.216 |
| 1.00 | 0.040 | 3.486 | r_knee | 4.513 | 0 | 5 | 12 | 10.57 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.181 | +2.45 | +1.269 |
| 1.00 | 0.048 | 3.636 | r_knee | 4.645 | 0 | 5 | 13 | 10.38 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.136 | +2.45 | +1.314 |
| 1.00 | 0.056 | 3.606 | r_knee | 4.784 | 0 | 5 | 14 | 9.53 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.139 | +2.45 | +1.311 |
| 1.20 | 0.016 | 2.379 | r_knee | 3.810 | 0 | 1 | 1 | 8.01 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.038 | +2.38 | +1.340 |
| 1.20 | 0.024 | 2.568 | r_knee | 3.941 | 0 | 2 | 2 | 8.28 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.130 | +2.45 | +1.320 |
| 1.20 | 0.032 | 2.879 | r_knee | 4.105 | 0 | 4 | 6 | 9.21 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +1.103 | +2.45 | +1.347 |
| 1.20 | 0.040 | 3.202 | r_knee | 4.342 | 0 | 5 | 9 | 9.93 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +0.980 | +2.45 | +1.470 |
| 1.20 | 0.048 | 3.491 | r_knee | 4.526 | 0 | 6 | 16 | 10.04 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +0.876 | +2.45 | +1.574 |
| 1.20 | 0.056 | 3.460 | r_knee | 4.739 | 0 | 9 | 27 | 9.20 | — | 0.00 | 1.00 | 0.00 | r_knee | stop | +0.825 | +2.45 | +1.625 |

Logs from SHA `424b3d0`. Prior goal-ask score SHA `697d53b`.
