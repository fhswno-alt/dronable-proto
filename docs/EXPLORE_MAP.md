# Explore / map (Prefer FAIL)

`scripts/explore_map.py` walks the frozen plant and builds a partial map from `kit_cam`. It is not a room finder, not SLAM, and not go-anywhere. Finders stay last mile. They can ask this map two questions later: "have I seen kitchen-like yellow?" and "which cells are frontiers?"

The room scenes no longer use flat primary-color boxes (`docs/ROOM_ASSETS.md`). The kitchen has no yellow backsplash, and the entrance mat is hessian, not a saturated violet plane. `scripts/explore_map.py` was not retuned. The yellow fractions, the soft-XY probe, and the mat-as-feature description below were measured on the previous box scenes.

Voice still refuses "explore", "build a map", and "go anywhere" (`scripts/voice_caller.py`). This script is not wired to that caller.

## What is frozen

The walk plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `71b2c86d133ebc603f58b99c53e496f3`. This slice does not edit that file, the gait, the tip check, `CommandBus`, or the `kit_cam` mount. The camera stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. One camera. No lidar. No second camera. No GPU and no paid API.

Commands are only `stand`, `stop`, and `vel(vx, yaw_rate)` on `CommandBus` in `scripts/steer_walk.py`. Resend is 10 Hz. 200 ms of silence stands. Caps stay **+0.056 / −0.032** m/s and yaw **±0.25** rad/s. `vx = 0` yaw does not change heading on this plant, and this script does not send it.

The empty-plant walk starts with the claimed nav-multi prefix, not a 0.40 s frontier replan. Stand is **1.0 s** so the left arc starts at t = 16 s. Then `vel(+0.056, 0)` for 15 s and `vel(+0.056, +0.25)` for 12.5 s. A 0.6 s stand starts that left arc 0.4 s early, and the same 12.5 s hold then only reached about **+35 deg**. Half-cap `vel(+0.028, +0.25)` is upright, and a short left command on it is swallowed.

After that prefix the dense rim chooses the next held window. On this run the rim was still left, so the walk holds `vel(+0.056, 0)` for **4 s** and then `vel(+0.056, +0.25)` for **8 s**. The claimed 6 s mid is the wrong gap: a left command there is swallowed (about **+8 deg**) and then crosses `up_z` 0.90. Extending the first left hold through about 21 s also tips. The 4 s gap is the one where the 8 s left window yaws (about **+30 deg**) and stays up.

The right leg is one `vel(+0.056, −0.25)` hold for **34 s**. It is not a second right after yaw returns to 0. From the pose after the second left, the dense rim is still left of the heading. Another left window is not this hold. About **38 s** on the right hold crosses `up_z` 0.90. 34 s stayed at **0.939**.

Furnished scenes do not get that chain. A full-cap approach in the kitchen crosses `up_z` 0.90 near **+0.39 m**. An 11 s `vel(+0.056, −0.25)` reached `up_z` **0.899** in the kitchen and **0.889** in the bathroom. A cold left hold stayed near **+5 to +11 deg** and then leaned. Those windows are not the room schedule. The room walk is the same right command for **8 s** (`vel(+0.056, −0.25)`). That hold stayed at `up_z` **0.919** (kitchen) and **0.934** (bathroom). It is not the claimed 11 s window and it is not a new cap.

## What the map is

The grid is 0.10 m cells in the world frame, from x −0.80..2.60 m and y −1.80..1.80 m. A cell is painted only from `kit_cam` rays that meet the floor plane between 0.30 m and 2.60 m. After each view, an unknown cell with three or four orthogonal free neighbors is filled. A frontier rim cell has one free neighbor and stays unknown.

| Paint | Pixel test |
|-------|------------|
| Free | Low saturation (checker floor). Saturation under 12. |
| Feature | Saturated and not sky, and the ray meets the floor. On the previous box entrance the violet mat was this case. The current mat is hessian, and this script was not retuned. |
| Not a cell | Elevated color. The ray misses the floor inside 2.60 m. Stored as a bearing only. |

The kitchen backsplash is the elevated case. The yellow test is the same pixel rule as `scripts/find_kitchen.py` (red and green high, blue low). At the stand pose that blob is about 0.039 of the frame and the ray does not meet the floor, so `ground_cell_ij` stays empty. The bearing is the camera ray. It is not a waypoint and it is not passed to `choose_velocity`.

Sky is the empty-plant blue around rgb (70, 100, 140). Cyan tile fails that test (not enough red) and is counted as other chromatic color. Other chromatic pixels are not given a room name.

Frontier cells are unknown cells in the 8-neighborhood of a free cell or a walked cell, between 0.40 m and 2.70 m from the body. The empty-plant walk does not replan every view. At the end of a held window it weights that dense rim into left, right, and a narrow ahead bin. Yellow is not an input. `choose_velocity()` still aims at the nearest cell inside a ±1.20 rad cone, with a left tie-break, and the finder does not call these explore windows. If the lower center of the frame is saturated inside 0.80 m, `choose_velocity()` drops frontiers straight ahead.

`find_kitchen.py` is not edited. It still paints with the main fan, 0.30–1.80 m and no hole fill, and it still calls `frontier_cells()` on the main ring: 4-connected, 0.40–1.60 m. `last_mile_from_map()` does not read a soft XY. The 2.60 m fan, the hole fill, and the 8-connected rim are the explore demo.

The explore demo can freeze a soft XY 1.50 m along the first yellow bearing. A separate probe walks half-cap toward that guess for up to 20 s. That probe is not the kitchen path. Yellow ≥ 0.50 is not the success test and arrival is not claimed. The measured probe is Prefer FAIL against find-kitchen (remaining 0.982 m vs 0.239 m).

The trail is the sim freejoint. In this sim that is odometry. It is not a visual pose.

Room XML is vision input only. The script does not read `kitchen`, `bathroom`, `living`, `bedroom`, or `entrance` body names, and it does not score success against those poses. The rooms on main are separate files (`room_kitchen.xml` and the four scenes from the merged room PR). They are not one building. An explore run loads one of them.

## What would count as a real land later

All of these, together:

1. One continuous space, not a separate XML per room.
2. Metric occupied cells for elevated furniture, from depth or parallax. A floor-plane guess of a backsplash is not that.
3. A pose that is not the simulator freejoint.
4. The last-mile finder in `scripts/find_kitchen.py` now queries `query_kitchen_like_yellow()` and `frontier_cells()` and steers from that. It still has no pre-placed waypoint. A query is not this land by itself.
5. Arrival still belongs to the finder (backsplash fills half the frame and the torso is within 0.25 m). This loop does not claim it.

`vx = 0` yaw still does not turn. A turn-in-place gait would be a different controls change. It is not this slice.

## Tonight's limit

The empty plant walks the claimed left prefix, then the frontier windows above: 4 s forward, 8 s left, and one 34 s right. Furnished scenes only take the 8 s right-first window, because the claimed yaw windows cross `up_z` 0.90 there. The paint is still a fan from one camera, out to 2.60 m with one-cell holes filled. A yellow bearing is logged when the backsplash is in frame, and a soft XY is frozen along it. Frontiers are the 8-connected edge of the paint. White and gray furniture can fail the saturation test and never become a cell. That is a partial feature map of whatever was in view. It is not go-anywhere.

The empty plant is the honest miss: floor cells, no yellow, no room label.

```bash
MUJOCO_GL=osmesa python scripts/explore_map.py --self-test
MUJOCO_GL=osmesa python scripts/explore_map.py --demo
```

`--demo` walks the empty plant on the claimed left prefix plus the frontier windows, and the kitchen and bathroom on the 8 s right-first window. It then runs a 20 s half-cap probe toward the frozen soft XY in the kitchen. Each run stops and settles 1.2 s. It writes `previews/explore_map_summary.json`, kit_cam and map stills, `previews/explore_map_kitchen.mp4`, and `previews/explore_map_kitchen_soft.mp4`. Exit status is non-zero if the plant md5 changes, if `up_z` drops below 0.90, or if a `vx = 0` yaw is sent. A passing exit still prints Prefer FAIL for go-anywhere and for arrival.

## Measured

Plant md5 after the demo: `71b2c86d133ebc603f58b99c53e496f3`. No tip, no fault, end mode stand on every run. `vx = 0` yaw sends: 0. Explore commands were `vx = +0.056`. The soft-XY probe was `vx = +0.028`. Arrival was not claimed. Both arrival bars stayed where they are: yellow ≥ 0.50 and torso-to-kitchen ≤ 0.25 m.

The empty plant followed the claimed left prefix, then the frontier windows. Claimed left arc **+75.5 deg** on `yaw_rate +0.25`. The dense rim at that pose was left **31.3** against right **0.0**, so the next window was the 4 s forward gap and then an 8 s left. That second left tracked **+29.5 deg**. At the next pose the rim was still left (**30.4** against **0.0**). The walk then held one right command for 34 s, **−177.7 deg**, on the commanded side. It did not insert a yaw-0 gap and then a second right.

Δx below is the settled pose minus the pose after the stand. Δyaw is the same pair. min up_z is the whole run, including settle.

| Scene | Command | Δx | Δy | Δyaw settled | Arc Δyaw | min up_z | Tip / fault |
|-------|---------|----|----|--------------|----------|----------|-------------|
| Empty plant | claimed left, frontier left, one 34 s right | +1.899 m | +1.287 m | −66.5 deg | left +75.5, frontier-left +29.5, right −177.7 | 0.939 | no |
| Kitchen | 8 s right | +0.337 m | −0.209 m | −55.8 deg | −58.2 | 0.919 | no |
| Bathroom | 8 s right | +0.423 m | −0.236 m | −50.6 deg | −56.4 | 0.934 | no |
| Kitchen soft XY | 20 s half-cap toward the frozen point | +0.332 m | −0.025 m | +2.4 deg | +2.1 | 0.986 | no |

Open-loop on this same dense fan, the claimed chain plus the 15 s forward, was **+1.736 / +1.746 m**, settled **+78.4 deg**, min up_z **0.949**. This frontier walk stays above up_z 0.90. The minimum is lower than that open-loop run. It is not a tip.

What the map held at the end, next to that open-loop chain. The older 1.80 m fan is in parentheses.

| Scene | Free cells (start → end) | Open-loop end | Floor-feature | Walked (open-loop) | Frontiers (start → end) | Open-loop end |
|-------|--------------------------|---------------|---------------|--------------------|-------------------------|---------------|
| Empty plant | 404 → 883 | 821 (766) | 0 | 57 (52) | 248 → 41 | 39 (11) |
| Kitchen | 293 → 450 | 450 (366) | 47 → 305 | 7 (7) | 131 → 55 | 55 (26) |
| Bathroom | 404 → 794 | 794 (531) | 0 | 10 (10) | 248 → 61 | 61 (28) |
| Kitchen soft XY | 293 → 315 | — | 47 → 288 | 8 | 131 → 50 | — |

The empty-plant stand fan is 404 free cells. The median frontier gap at that stand is **0.007 rad**. After the frontier walk the rim is still inside the query: **41** frontiers, median gap **0.023 rad**, against the open-loop chain's **39**. Walked cells are **57** against **52**. Free cells are **883** against **821**. The extra free cells are the 8 s left window, taken because the dense rim was on the left. The extra walked cells are the 34 s right hold. That is a larger floor fan than the open-loop chain. It is not a house and not go-anywhere.

Kitchen and bathroom stay on the 8 s right window. Their cell counts match the open-loop chain. They are not a frontier search.

Kitchen yellow on the right-first window peaked at fraction **0.071**, bearing **+0.303 rad**, elevation **+0.526 rad**. That ray does not meet the floor inside 2.60 m, so there is no yellow cell. The 305 floor-feature cells are saturated pixels whose rays meet the floor plane. The longer range is why that count is above main's 150. They are not a counter outline and they were not the aim of the walk. Settled yellow is **0.000**. Torso-to-kitchen remaining is **0.976 m**.

Bathroom other-chromatic pixels peaked at **0.128** of a frame. They are not labeled as a room. None of them passed the yellow test.

The soft XY is frozen at the stand log, at **(1.501, 0.036) m**, 1.50 m along the bearing in that first frame. The 20 s probe walks half-cap toward it and does not read yellow as success.

| | This probe | Main find-kitchen |
|--|------------|-------------------|
| Remaining | **0.982 m** | **0.239 m** |
| End yellow | **0.066** | **0.000** |
| min up_z | **0.986** | **0.979** |
| Arrival claimed | no | no |

Remaining is worse than main. End yellow is above 0 and below 0.50. The gap bar and the yellow bar are not both met. This is Prefer FAIL on arrival. It is not the close-range yellow chase.

This is not go-anywhere. The finder may query the yellow log and the frontiers. Arrival stays on the finder's bars.
