# Find kitchen (Prefer FAIL)

The steer cue is the warm white slab on the merged kitchen, not the old flat-yellow HSV test and not the dark wood band. Arrival bars are unchanged: cue fraction ≥ 0.50 and torso-to-kitchen ≤ 0.25 m. Both are required. This walk meets both on the settled stop frame. That is arrival. Not go-anywhere.

`scripts/find_kitchen.py` is a last-mile steer on the frozen walk plant. The phrase path paints `kit_cam` into the explore map and commands from `query_kitchen_like_yellow()` and `frontier_cells()`. The yellow log stores the warm-white slab. It is not SLAM and not an arrival. If that log has no slab, it sends no vel.

## Merged kitchen, this run

On main after `99e06df`. The cabinet run is white plaster and light oak. The stove faces +X. `mujoco/room_kitchen.xml` includes the frozen plant (md5 `71b2c86d133ebc603f58b99c53e496f3`). `kit_cam` stays at `0.050 0.019 0.007`. The command is half-cap `vel(+0.028, yaw)` inside yaw ±0.25. `vx = 0` yaw is not sent. The world-x budget is 1.32 m. Once the slab fills a fifth of the frame, yaw follows the live centroid inside a 0.04 bias band. Phrase "go to the kitchen", after stand:

| | Warm white slab (steer) | Old HSV yellow (reported, not the steer) |
|--|--|--|
| Stand | **0.046**, bias **−0.094**, logged | **0.073** |
| Map peak | **0.751** at bearing **−0.050 rad** | |
| Walked stop | **0.793** | **0.009** |
| y = 0 pose inside the gap bar, gap **0.250 m** | **0.770** | **0.021** |
| Bathroom stand | **0.0007** | **0.0205** |
| Empty plant | **0.000** | **0.000** |
| Living / bedroom / entrance stand | **0.0000 / 0.0007 / 0.0000** | **0.146 / 0.002 / 0.213** |

The slab is luminance ≥ 185, saturation ≤ 48, and red at least 8 above blue. That stays under the 0.015 log bar in the bathroom, living room, bedroom, entrance, and empty plant. The cue was not widened.

| | |
|--|--|
| Stop | `stop_kind` arrival. Both bars on the settled frame. |
| End x | **+1.270 m** (Δx **+1.268 m**) |
| End y, yaw | +0.062 m, +0.187 rad |
| min up_z | **0.980** (bar 0.90, not crossed) |
| Remaining | **0.230 m** |
| Commands | 15 forward slices and 177 walk-yaw slices, every one `vx = +0.028`, yaw inside ±0.25. `vx = 0` was not sent. |
| Map | Slab was logged. `map_command_source` is `map`. **385** queries. |
| Arrival | **true** |

0.230 m is inside the 0.25 m gap. Settled slab is 0.793, above 0.50. The 1.32 m budget did not fire. The bars were not lowered.

## Previous 1.10 m budget stop

Squash `99e06df` on main. Same cue, budget 1.10 m. Phrase "go to the kitchen" stopped Prefer FAIL `budget`: remaining **0.384 m**, settled slab **0.496**, min up_z **0.956**, Δx **+1.115 m**, arrival **false**. That stop is not this walk.

## Previous colored-box run

The tables below are the flat yellow backsplash on the old box kitchen. They are not this mesh run.

The scene is `mujoco/room_kitchen.xml`, which includes `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`). The finder does not edit that file. `kit_cam` stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. Feet stay 145×86. Leg actuators stay ±2.1 Nm. No second camera, no lidar, no door.

## What the pixels were (colored boxes)

From the stand pose the backsplash is the signal that separates the kitchen from the empty checkerboard. Measured on `kit_cam` at 640×480:

| Frame | Yellow fraction | Blob bias |
|-------|-----------------|-----------|
| Room, stand | ~0.039 | ~−0.014 (center) |
| Room, base yaw −18° | ~0.051 | ~−0.35 (left) |
| Room, base yaw +18° | ~0.046 | ~+0.31 (right) |
| Empty plant, or kitchen behind the camera | 0 | no blob |

The wood counter's lit face does not separate from the blue cabinet, and the kettle renders brown, so those colors are not the blob. The blue cabinet is not the blob either: the empty-floor sky already passes a loose blue test.

A named-body projection of `kitchen`, `table`, and `chair` checks that the yellow centroid sits on the kitchen. That check can only refuse. It is not a waypoint, and it is not passed to the map query. The phrase `vel` comes from the logged ray and the frontiers.

## Phrase path

The bus is `CommandBus` in `scripts/steer_walk.py`: `stand`, `stop`, and `vel(vx, yaw_rate)` only. Resend is 10 Hz. 200 ms of silence stands. The bus caps stay +0.056 / −0.032 m/s and yaw ±0.25. The phrase sends half the forward cap. The bus clamps again. Full-cap finder bursts crossed `up_z` 0.90 near +0.39 m, so this last mile stays at `vel(+0.028, yaw)`.

The stand frame is integrated into an `ExploreMap`. The next command is `last_mile_from_map()`, which calls `query_kitchen_like_yellow()` and `frontier_cells()`. If yellow was not logged, the phrase sends no vel. Until the slab fills a fifth of the frame, yaw aims at the frontier nearest that camera ray, or at the ray when no frontier sits in the cone. After that, yaw is a tighter trim on the live centroid and vx stays at 0.028. That aim is not a waypoint and not a counter cell. The live fraction is still an arrival bar. A lost blob does not replace the map command.

`vx = 0` yaw does not change heading on this plant, so it is not sent.

The `up_z` Prefer FAIL bar stays **0.90**. It is not lowered. A full-cap slice run on the same bar stopped at end x +0.394 m when `up_z` hit 0.894. Shorter slices with a stand between them were tried and did not get past that distance:

| Trial | Result |
|-------|--------|
| 0.25 s vel, 0.40 s stand | Δx −0.052 m at the time limit, min up_z 0.999. The velocity slew takes 0.70 s to reach a cap, so the slice never walks and the stand drifts backward. |
| 0.25 s vel, 0.15 s stand | Δx −0.029 m, min up_z 0.999. Same stall. |
| 0.20 s vel, 0.30 s stand | Kitchen left the frame at Δx +0.082 m (yaw +0.79 rad), min up_z 0.956. |

Half the forward cap, with no stand between slices, is the duty that stayed upright. At a 0.60 m world-x budget that duty stopped on the budget: end x +0.610 m, Δx +0.608 m, min up_z 0.989, remaining 0.705 m, yellow fraction 0.131, arrival false.

The world-x budget on this draft is **1.32 m**. That is still a stop, not a counter pose. The `up_z` bar, the half-cap command, and both arrival bars are unchanged. The previous merged stop used 1.10 m and fired first.

Without a reacquire, an earlier half-cap walk stopped at end x +0.938 m when the yellow fraction fell to 0.004 with the kitchen body still in frame (remaining 0.377 m, min up_z 0.981). `vx = 0` yaw does not change heading on this plant, so a yaw-only reacquire was not used. A later blob-only draft kept half-cap `vel(+0.028, yaw trim)` for at most two tries of 2.0 s. The phrase path in this draft does not do that. It keeps steering from the map log until a stop bar.

Baseline on main, before the fade recenter, "go to the kitchen", room scene, after stand:

| | |
|--|--|
| Stop | Prefer FAIL, `stop_kind` blob. Two soft walk-yaw tries. Yellow came back above 0.015 and the normal half-cap loop resumed, then the blob was lost again. The two-try budget blocked a third try. No further `vel`. |
| End x | **+1.024 m** (start x +0.002 m, Δx **+1.022 m**) |
| End y, yaw | +0.052 m, +0.229 rad |
| min up_z | **0.980** (bar 0.90, not crossed) |
| Remaining | **0.291 m** from the torso to the kitchen geom (near face x = 1.315 m) |
| Final blob | yellow fraction **0.000** on the settled frame. Kitchen body still in frame. |
| Reacquires | 2. Every phrase command was `vx = +0.028` (yaw trim up to +0.250). `vx = 0` was not sent. |
| Arrival | **false** |

111 centered slices and 50 walking yaw trims were sent at `vx = +0.028`, including the slices after yellow briefly returned. The mid still is the first frame after Δx crossed 0.55 m (yellow 0.105, bias −0.078). That bias is inside the 0.08 forward-only band, so the walk kept going straight while the backsplash later fell to 0.000. 0.291 m is still outside the 0.25 m gap.

The blob-only draft, before this map query, stopped at end x **+1.038 m** (Δx **+1.036 m**), remaining **0.277 m**, settled yellow **0.006**, min up_z **0.980**. That was Prefer FAIL on a lost blob.

This phrase queries the map. Measured "go to the kitchen", room scene, after stand:

| | |
|--|--|
| Stop | Prefer FAIL, `stop_kind` close. The torso gap is inside 0.25 m and the backsplash does not fill half the frame. Not arrival. |
| Map | Yellow was logged (max fraction **0.148**, bearing **−0.181 rad**). `map_command_source` is `map`. **315** queries. An empty map returns no command; the empty plant still sends no vel. |
| End x | **+1.076 m** (Δx **+1.074 m**) |
| End y, yaw | +0.003 m, −0.049 rad |
| min up_z | **0.979** (bar 0.90, not crossed) |
| Remaining | **0.239 m** |
| Final blob | yellow fraction **0.000** on the settled frame. The logged ray is not a counter outline. |
| Commands | 48 forward slices and 109 yaw slices, every one `vx = +0.028`, yaw inside ±0.25. `vx = 0` was not sent. |
| Arrival | **false** |

0.239 m meets the gap bar alone. Settled yellow is 0, not 0.50. Both bars are required. That is the Prefer FAIL.

## What would count as arrival

Both bars, on the settled stop frame:

1. Warm-white slab fraction ≥ 0.50. Half the frame is "most of the frame". Stand is 0.046, and this stop is 0.793.
2. Torso-to-kitchen horizontal gap ≤ 0.25 m. That is contact range for this torso, not a room crossing. This stop is 0.230 m.

Either bar alone is not arrival. A stop inside 0.25 m with the grain still small is Prefer FAIL (`close`), not arrival. The gap is read from the kitchen geom boxes so the summary can state the remaining distance. It does not choose left versus right and it is not a waypoint.

## 1.2 s hop (self-test)

`--self-test` still runs one short hop. The sequence is twelve resends of `vel(+0.056, 0)` from t=0.60 s to t=1.70 s, then stop. Measured end x **+0.035 m**, mode stand. Stopping there is not arrival.

If the blob is off to one side, that self-test uses one `vx = 0` yaw hold (1.0 s, or sooner if the blob reaches the center). Forward comes next only when the new frame is a clear center. A base yaw of −18° puts the slab on the left, bias about −0.44. The self-test sends ten resends of `vel(0, +0.25)` and then stops with no forward. The phrase path does not use that `vx = 0` hold.

## Prefer FAIL

No further `vel` is sent when:

- the phrase is bathroom, or any other room
- the loaded scene is the empty walk plant
- the explore map has no cabinet-grain cue yet
- `up_z` drops below 0.90
- the 1.32 m budget is reached, or the slice cap is exhausted
- the torso is within 0.25 m and the cabinet grain does not fill half the frame (`close`)
- the stop frame does not meet both arrival bars

```bash
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the bathroom"
MUJOCO_GL=osmesa python scripts/find_kitchen.py --scene plant "go to the kitchen"
python scripts/find_kitchen.py --self-test
```

Stills from the room phrase: `previews/find_kitchen_before.png`, `previews/find_kitchen_mid.png`, `previews/find_kitchen_after.png`, and `previews/find_kitchen_prefer_fail.png` (empty plant). `scripts/steer_walk.py` with no scene argument still loads the empty plant.
