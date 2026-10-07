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

