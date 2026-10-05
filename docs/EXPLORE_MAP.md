# Explore / map (Prefer FAIL)

`scripts/explore_map.py` walks the frozen plant and builds a partial map from `kit_cam`. It is not a room finder, not SLAM, and not go-anywhere. Finders stay last mile. They can ask this map two questions later: "have I seen kitchen-like yellow?" and "which cells are frontiers?"

Voice still refuses "explore", "build a map", and "go anywhere" (`scripts/voice_caller.py`). This script is not wired to that caller.

## What is frozen

The walk plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `71b2c86d133ebc603f58b99c53e496f3`. This slice does not edit that file, the gait, the tip check, `CommandBus`, or the `kit_cam` mount. The camera stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. One camera. No lidar. No second camera. No GPU and no paid API.

Commands are only `stand`, `stop`, and `vel(vx, yaw_rate)` on `CommandBus` in `scripts/steer_walk.py`. Resend is 10 Hz. 200 ms of silence stands. Caps stay **+0.056 / −0.032** m/s and yaw **±0.25** rad/s. `vx = 0` yaw does not change heading on this plant, and this script does not send it.

The empty-plant walk is the claimed nav-multi chain, not a 0.40 s frontier replan. Stand is **1.0 s** so the left arc starts at t = 16 s. Then `vel(+0.056, 0)` for 15 s, `vel(+0.056, +0.25)` for 12.5 s, `vel(+0.056, 0)` for 6 s, `vel(+0.056, −0.25)` for 11 s, and `vel(+0.056, 0)` for 6 s. A 0.6 s stand starts that left arc 0.4 s early, and the same 12.5 s hold then only reached about **+35 deg**. Half-cap `vel(+0.028, +0.25)` is upright, and a short left command on it is swallowed.

Furnished scenes do not get that chain. A full-cap approach in the kitchen crosses `up_z` 0.90 near **+0.39 m**. An 11 s `vel(+0.056, −0.25)` reached `up_z` **0.899** in the kitchen and **0.889** in the bathroom. A cold left hold stayed near **+5 to +11 deg** and then leaned. Those windows are not the room schedule. The room walk is the same right command for **8 s** (`vel(+0.056, −0.25)`). That hold stayed at `up_z` **0.919** (kitchen) and **0.934** (bathroom). It is not the claimed 11 s window and it is not a new cap.

## What the map is

The grid is 0.10 m cells in the world frame, from x −0.80..2.60 m and y −1.80..1.80 m. A cell is painted only from `kit_cam` rays that meet the floor plane between 0.30 m and 2.60 m. After each view, an unknown cell with three or four orthogonal free neighbors is filled. A frontier rim cell has one free neighbor and stays unknown.

| Paint | Pixel test |
|-------|------------|
| Free | Low saturation (checker floor). Saturation under 12. |
| Feature | Saturated and not sky, and the ray meets the floor. The entrance mat is this case. |
| Not a cell | Elevated color. The ray misses the floor inside 2.60 m. Stored as a bearing only. |

The kitchen backsplash is the elevated case. The yellow test is the same pixel rule as `scripts/find_kitchen.py` (red and green high, blue low). At the stand pose that blob is about 0.039 of the frame and the ray does not meet the floor, so `ground_cell_ij` stays empty. The bearing is the camera ray. It is not a waypoint and it is not passed to `choose_velocity`.

Sky is the empty-plant blue around rgb (70, 100, 140). Cyan tile fails that test (not enough red) and is counted as other chromatic color. Other chromatic pixels are not given a room name.

Frontier cells are unknown cells in the 8-neighborhood of a free cell or a walked cell, between 0.40 m and 2.70 m from the body, inside a ±1.20 rad cone of the current heading. The next `vel` aims at the nearest of those cells. A tie breaks to the left of the heading. Yellow is not an input. If the lower center of the frame is saturated inside 0.80 m, frontiers straight ahead are dropped.

The first time kitchen-like yellow clears the log threshold, a soft XY is frozen 1.50 m along that camera bearing (or on the floor cell, if the ray met the floor). That point is not a waypoint. A separate kitchen probe walks half-cap `vel(+0.028, yaw)` toward it for up to 20 s and stops on a tip, a plant fault, or on reaching within 0.40 m of that guess. Yellow ≥ 0.50 is not the success test and arrival is not claimed.

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

The empty plant walks the claimed chain, then one more claimed forward window: `vel(+0.056, 0)` for 15 s. A second left hold does not yaw and then tips. A second right hold tips. Those are not this schedule. Furnished scenes only take the 8 s right-first window, because the claimed yaw windows cross `up_z` 0.90 there. The paint is still a fan from one camera, now out to 2.60 m with one-cell holes filled. A yellow bearing is logged when the backsplash is in frame, and a soft XY is frozen along it. Frontiers are the 8-connected edge of the paint. White and gray furniture can fail the saturation test and never become a cell. That is a partial feature map of whatever was in view. It is not go-anywhere.

The empty plant is the honest miss: floor cells, no yellow, no room label.

```bash
MUJOCO_GL=osmesa python scripts/explore_map.py --self-test
MUJOCO_GL=osmesa python scripts/explore_map.py --demo
```

`--demo` walks the empty plant on the claimed chain, and the kitchen and bathroom on the 8 s right-first window, then stops and settles 1.2 s. It writes `previews/explore_map_summary.json`, kit_cam and map stills, and `previews/explore_map_kitchen.mp4`. Exit status is non-zero if the plant md5 changes, if `up_z` drops below 0.90, or if a `vx = 0` yaw is sent. A passing exit still prints Prefer FAIL for go-anywhere.

## Measured

The table below is the previous main run (the claimed chain, 1.80 m fan). This draft's demo replaces it. The plant md5 on that run was `71b2c86d133ebc603f58b99c53e496f3`. No tip, no fault, end mode stand on every run. `vx = 0` yaw sends: 0. Every moving command was `vx = +0.056`.

The empty plant followed the claimed chain. Left arc **+75.5 deg** on `yaw_rate +0.25`. Chained right arc **−54.5 deg** on `yaw_rate −0.25`. Both are on the commanded side (the track bar used here is 20 deg; the swallowed run was about −12 deg on a left command). The resume then drifts **+30.8 deg**, which is the same drift the nav-multi clip already reports. It is not a second left command.

Δx below is the settled pose minus the pose after the stand. Δyaw is the same pair. min up_z is the whole run, including settle.

| Scene | Command | Δx | Δy | Δyaw settled | Arc Δyaw | min up_z | Tip / fault |
|-------|---------|----|----|--------------|----------|----------|-------------|
| Empty plant | claimed chain | +1.569 m | +1.022 m | +52.7 deg | left +75.5, right −54.5 | 0.954 | no |
| Kitchen | 8 s right | +0.337 m | −0.209 m | −55.8 deg | −58.2 | 0.919 | no |
| Bathroom | 8 s right | +0.423 m | −0.236 m | −50.6 deg | −56.4 | 0.934 | no |

The previous 12 s `vel(+0.028, yaw)` run moved **+0.156 / +0.185 / +0.190 m** and settled about **−12 deg** while commanding left. This run is longer on every scene, and each yaw arc moves the heading to the commanded side.

What the map held at the end:

| Scene | Free cells (start → end) | Floor-feature cells | Walked cells | Frontiers (start → end) | Kitchen-like yellow |
|-------|--------------------------|---------------------|--------------|-------------------------|---------------------|
| Empty plant | 266 → 766 | 0 | 41 | 55 → 11 | no (fraction 0) |
| Kitchen | 221 → 366 | 25 → 150 | 7 | 45 → 26 | yes, max fraction 0.071 |
| Bathroom | 266 → 531 | 0 | 10 | 55 → 28 | no (fraction 0) |

The empty plant is floor and sky. Frontiers are the rim of the floor paint. Walked cells went from 4–5 on the short run to 41 here because the body actually traveled. That is still not a house and not go-anywhere.

Kitchen yellow peaked at fraction **0.071**, bearing **+0.303 rad**, elevation **+0.526 rad**. That ray does not meet the floor inside 1.80 m, so there is no yellow cell. The 150 floor-feature cells are saturated pixels whose rays do meet the floor plane. They are not a counter outline and they were not the aim of the walk. The walk command is the right-first window, not the yellow bearing.

Bathroom other-chromatic pixels peaked at **0.128** of a frame. They are not labeled as a room. None of them passed the yellow test. The map is another floor fan, swung to the right.

This is not go-anywhere. The finder may query the yellow log and the frontiers. Arrival stays on the finder's bars.
