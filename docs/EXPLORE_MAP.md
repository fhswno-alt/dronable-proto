# Explore / map (Prefer FAIL)

`scripts/explore_map.py` walks the frozen plant and builds a partial map from `kit_cam`. It is not a room finder, not SLAM, and not go-anywhere. Finders stay last mile. They can ask this map two questions later: "have I seen kitchen-like yellow?" and "which cells are frontiers?"

Voice still refuses "explore", "build a map", and "go anywhere" (`scripts/voice_caller.py`). This script is not wired to that caller.

## What is frozen

The walk plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. This slice does not edit that file, the gait, the tip check, `CommandBus`, or the `kit_cam` mount. The camera stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. One camera. No lidar. No second camera. No GPU and no paid API.

Commands are only `stand`, `stop`, and `vel(vx, yaw_rate)` on `CommandBus` in `scripts/steer_walk.py`. Resend is 10 Hz. 200 ms of silence stands. Caps stay **+0.150 / −0.032** m/s and yaw **±0.25** rad/s. This script does not raise them. The walk uses the kit gait. `vel(0, yaw)` does change heading on this row. This script does not send it.

The empty-plant walk is the remeasured nav-multi chain, not a frontier replan. Stand is **1.0 s**. Then `vel(+0.150, 0)` for 15 s, `vel(+0.150, +0.25)` for 12.5 s, forward 6 s, `vel(+0.150, −0.25)` for 11 s, and forward 6 s. Prefer FAIL: approach heading **+2.210 m**, left **+145.9 deg**, chained right **−138.6 deg**, min up_z **0.932**, no tip, no fault. The stale **+0.598 m / +75.7 deg** envelope is not this path.

The 4 s forward gap, the 8 s second left, and the 34 s right hold were not remeasured on this tip. `next_frontier_phase` can still build them. `QUALIFIED_FRONTIER_CHAIN` is false, so a walk does not append them. Their old Δyaw figures are not claimed.

Furnished scenes use the remeasured 8 s right, `vel(+0.150, −0.25)`, after a 1 s stand. Kitchen and bathroom produced the same trace: heading **+0.540 m**, Δyaw **−94.6 deg**, min up_z **0.929** on the stop, no tip, no fault. Furniture did not change that result. The 11 s furnished window was not remeasured. The 8 s window is not a new cap.

## What the map is

The grid is 0.10 m cells in the world frame, from x −0.80..2.60 m and y −1.80..1.80 m. A cell is painted only from `kit_cam` rays that meet the floor plane between 0.30 m and 2.60 m. After each view, an unknown cell with three or four orthogonal free neighbors is filled. A frontier rim cell has one free neighbor and stays unknown.

| Paint | Pixel test |
|-------|------------|
| Free | Low saturation (checker floor). Saturation under 12. |
| Feature | Saturated and not sky, and the ray meets the floor. The entrance mat is this case. |
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

`vel(0, yaw)` does turn on this kit row. This explore schedule does not send it.

## Tonight's limit

The empty plant walks the remeasured nav-multi chain at the Day1 forward clamp. The 4 s / 8 s / 34 s frontier holds are refused as claims. Furnished scenes take the remeasured 8 s right. The paint is still a fan from one camera, out to 2.60 m with one-cell holes filled. A yellow bearing is logged when the backsplash is in frame, and a soft XY is frozen along it. Frontiers are the 8-connected edge of the paint. White and gray furniture can fail the saturation test and never become a cell. That is a partial feature map of whatever was in view. It is not go-anywhere. It is not arrival. It is not a demo-ready human walk.

The empty plant is the honest miss: floor cells, no yellow, no room label.

```bash
MUJOCO_GL=osmesa python scripts/explore_map.py --self-test
MUJOCO_GL=osmesa python scripts/explore_map.py --demo
```

`--demo` walks the empty plant on the remeasured nav-multi chain, and the kitchen and bathroom on the 8 s right. It then runs a 20 s half-cap probe toward the frozen soft XY in the kitchen. That probe's old Δx is not a current claim. Each run stops and settles 1.2 s. It writes `previews/explore_map_summary.json`, kit_cam and map stills, `previews/explore_map_kitchen.mp4`, and `previews/explore_map_kitchen_soft.mp4`. Exit status is non-zero if the plant md5 changes, if `up_z` drops below 0.90, or if a `vx = 0` yaw is sent. A passing exit still prints Prefer FAIL for go-anywhere and for arrival.

## Measured

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Kit gait. Video off. Commands at the Day1 clamp `vx = +0.150`. `vx = 0` yaw sends: 0. Half-cap inside this script is `+0.075` m/s. Arrival was not claimed. The soft-XY probe was not remeasured, so its old **+0.332 m / min up_z 0.986** row is not claimed.

Heading Δx is travel along the heading at the start of the window. The table is the fresh remeasure. The previous-basin rows (+75.5 deg, kitchen −58.2 deg, the 34 s right) are withdrawn.

| Scene | Command | Heading Δx | Δyaw | min up_z | Tip / fault |
|-------|---------|------------|------|----------|-------------|
| Empty plant, approach 15 s | `vel(+0.150, 0)` | +2.210 m | +1.4 deg | 0.934 run | no |
| Empty plant, left 12.5 s | `vel(+0.150, +0.25)` | +0.422 m | +145.9 deg | 0.950 window | no |
| Empty plant, chained right 11 s | `vel(+0.150, −0.25)` | +0.465 m | −138.6 deg | 0.932 run (stop) | no |
| Kitchen and bathroom, 8 s right | `vel(+0.150, −0.25)` | +0.540 m | −94.6 deg | 0.929 | no |
| 4 s gap, 8 s second left, 34 s right | not walked | refused | refused | not remeasured | — |

Cell counts from the previous frontier walk (883 free, 41 frontiers, 34 s right, soft-XY remaining 0.982 m) were not remeasured on this tip. They are not the current claim. The map is still a floor fan plus a yellow bearing. Yellow ≥ 0.50 and torso-to-kitchen ≤ 0.25 m are still the arrival bars, and this loop does not claim either one.

This is not go-anywhere. The finder may query the yellow log and the frontiers. Arrival stays on the finder's bars.
