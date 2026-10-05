# Explore / map (Prefer FAIL)

`scripts/explore_map.py` walks the frozen plant and builds a partial map from `kit_cam`. It is not a room finder, not SLAM, and not go-anywhere. Finders stay last mile. They can ask this map two questions later: "have I seen kitchen-like yellow?" and "which cells are frontiers?"

Voice still refuses "explore", "build a map", and "go anywhere" (`scripts/voice_caller.py`). This script is not wired to that caller.

## What is frozen

The walk plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `71b2c86d133ebc603f58b99c53e496f3`. This slice does not edit that file, the gait, the tip check, `CommandBus`, or the `kit_cam` mount. The camera stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. One camera. No lidar. No second camera. No GPU and no paid API.

Commands are only `stand`, `stop`, and `vel(vx, yaw_rate)` on `CommandBus` in `scripts/steer_walk.py`. Resend is 10 Hz. 200 ms of silence stands. Caps stay **+0.056 / −0.032** m/s and yaw **±0.25** rad/s. Every moving command is soft walk-yaw: `vel(+0.028, yaw)` with yaw inside the cap. `vx = 0` yaw does not change heading on this plant, and this script does not send it.

## What the map is

The grid is 0.10 m cells in the world frame, from x −0.80..2.60 m and y −1.80..1.80 m. A cell is painted only from `kit_cam` rays that meet the floor plane between 0.30 m and 1.80 m:

| Paint | Pixel test |
|-------|------------|
| Free | Low saturation (checker floor). Saturation under 12. |
| Feature | Saturated and not sky, and the ray meets the floor. The entrance mat is this case. |
| Not a cell | Elevated color. The ray misses the floor inside 1.80 m. Stored as a bearing only. |

The kitchen backsplash is the elevated case. The yellow test is the same pixel rule as `scripts/find_kitchen.py` (red and green high, blue low). At the stand pose that blob is about 0.039 of the frame and the ray does not meet the floor, so `ground_cell_ij` stays empty. The bearing is the camera ray. It is not a waypoint and it is not passed to `choose_velocity`.

Sky is the empty-plant blue around rgb (70, 100, 140). Cyan tile fails that test (not enough red) and is counted as other chromatic color. Other chromatic pixels are not given a room name.

Frontier cells are unknown cells next to a free cell or a walked cell, between 0.40 m and 1.60 m from the body, inside a ±1.20 rad cone of the current heading. The next `vel` aims at the nearest of those cells. A tie breaks to the left of the heading. Yellow is not an input. If the lower center of the frame is saturated inside 0.80 m, frontiers straight ahead are dropped.

The trail is the sim freejoint. In this sim that is odometry. It is not a visual pose.

Room XML is vision input only. The script does not read `kitchen`, `bathroom`, `living`, `bedroom`, or `entrance` body names, and it does not score success against those poses. The rooms on main are separate files (`room_kitchen.xml` and the four scenes from the merged room PR). They are not one building. An explore run loads one of them.

## What would count as a real land later

All of these, together:

1. One continuous space, not a separate XML per room.
2. Metric occupied cells for elevated furniture, from depth or parallax. A floor-plane guess of a backsplash is not that.
3. A pose that is not the simulator freejoint.
4. A last-mile finder that queries `query_kitchen_like_yellow()` and `frontier_cells()` and then steers, without a pre-placed waypoint.
5. Arrival still belongs to the finder (backsplash fills half the frame and the torso is within 0.25 m). This loop does not claim it.

`vx = 0` yaw still does not turn. A turn-in-place gait would be a different controls change. It is not this slice.

## Tonight's limit

A short soft walk paints a fan of floor in front of `kit_cam`, shifts that fan a little as the body moves, and logs a yellow bearing when the backsplash is already in the opening frame. Frontiers are the edge of the paint. White and gray furniture can fail the saturation test and never become a cell. That is a partial feature map of whatever was in view. It is not go-anywhere.

The empty plant is the honest miss: floor cells, no yellow, no room label.

```bash
MUJOCO_GL=osmesa python scripts/explore_map.py --self-test
MUJOCO_GL=osmesa python scripts/explore_map.py --demo
```

`--demo` walks the empty plant, the kitchen scene, and the bathroom scene for 12 s each at `vel(+0.028, yaw)`, then stops. It writes `previews/explore_map_summary.json`, kit_cam and map stills, and `previews/explore_map_kitchen.mp4`. The summary is the measurement. This document's measured table is filled from that file. Exit status is non-zero if the plant md5 changes, if `up_z` drops below 0.90, or if a `vx = 0` yaw is sent. A passing exit still prints Prefer FAIL for go-anywhere.
