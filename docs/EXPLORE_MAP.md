# Explore / map (Prefer FAIL)

`scripts/explore_map.py` walks the frozen plant and builds a partial map from `kit_cam`. It is not a room finder, not SLAM, and not go-anywhere. Finders stay last mile. They can ask this map two questions later: "have I seen kitchen-like yellow?" and "which cells are frontiers?"

Voice still refuses "explore", "build a map", and "go anywhere" (`scripts/voice_caller.py`). This script is not wired to that caller.

## What is frozen

The walk plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. This slice does not edit that file, the gait, the tip check, `CommandBus`, or the `kit_cam` mount. The camera stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. One camera. No lidar. No second camera. No GPU and no paid API.

Commands are only `stand`, `stop`, and `vel(vx, yaw_rate)` on `CommandBus` in `scripts/steer_walk.py`. Resend is 10 Hz. 200 ms of silence stands. Caps stay **+0.150 / −0.032** m/s and yaw **±0.25** rad/s. This script does not raise them. The walk uses the kit gait. `vel(0, yaw)` does change heading on this row. This script does not send it.

The empty-plant walk is the nav-multi chain on this CommandBus, not a frontier replan. Stand is **1.0 s**. Then `vel(+0.150, 0)` for 15 s, `vel(+0.150, +0.25)` for 12.5 s, forward 6 s, `vel(+0.150, −0.25)` for 11 s, and forward 6 s. The live phase close is in the measured table below. The stale **+0.598 m / +75.7 deg** envelope is not this path.

The 4 s forward gap, the 8 s second left, and the 34 s right hold stay off. `QUALIFIED_FRONTIER_CHAIN` is false. The plant run appended exactly those five vel windows and no frontier hold.

Furnished scenes use 8 s of `vel(+0.150, −0.25)` after a 1 s stand. Kitchen and bathroom produced the same body trace. Furniture did not change it. The 11 s furnished window was not this schedule. The 8 s window is not a new cap.

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

The explore demo can freeze a soft XY 1.50 m along the first yellow bearing. A separate probe walks half the forward cap (`+0.075` m/s) toward that guess for up to 20 s. That probe is not the kitchen path. Yellow ≥ 0.50 is not the success test and arrival is not claimed. On this tip the probe stopped at 13.8 s because the guess was inside 0.40 m. Remaining was **0.159 m** and yellow was **0**. The gap bar alone is not arrival.

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

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Kit gait. Video off. This table is `run_explore` / `run_soft_goal` on the CommandBus, not the voice clip. Commands at the Day1 clamp `vx = +0.150` except the soft probe at `+0.075`. `vx = 0` yaw sends: 0. `QUALIFIED_FRONTIER_CHAIN` stayed false. No tip. No fault. End mode stand. Arrival was not claimed.

Heading Δx is travel along the heading at the start of that phase. The voice clip's left arc is **+145.9 deg** on its summarize window. The explore phase close on the same command is **+145.1 deg**. The stale **+75.7 deg** row stays withdrawn.

| Phase | Command | Heading Δx | Δyaw | min up_z | Tip / fault |
|-------|---------|------------|------|----------|-------------|
| Empty plant, approach 15 s | `vel(+0.150, 0)` | +2.210 m | +1.4 deg | 0.950 | no |
| Empty plant, left 12.5 s | `vel(+0.150, +0.25)` | +0.420 m | +145.1 deg | 0.950 | no |
| Empty plant, mid 6 s | `vel(+0.150, 0)` | +0.944 m | +12.3 deg | 0.950 | no |
| Empty plant, right 11 s | `vel(+0.150, −0.25)` | +0.466 m | −138.6 deg | 0.950 | no |
| Empty plant, resume 6 s | `vel(+0.150, 0)` | +0.958 m | −2.5 deg | 0.950 | no |
| Empty plant, whole run | stand then that chain then stop | world +2.676, +3.389 m | settled +5.5 deg | **0.932** | no |
| Kitchen and bathroom, 8 s right | `vel(+0.150, −0.25)` | +0.540 m | −94.6 deg | **0.929** | no |
| Soft XY | `vel(+0.075, yaw trim)` for 13.8 s | +1.099 m | +0.2 deg | 0.934 | no |
| 4 s gap, 8 s second left, 34 s right | not appended | refused | refused | not this walk | — |

The empty-plant map went from **336** free cells and **261** frontiers to **955** free, **0** frontiers, **66** walked. Zero frontiers means the floor fan no longer has an unknown rim inside the query. It is not a house and not go-anywhere. Yellow stayed **0**.

Kitchen after the 8 s right: free **258 → 511**, frontiers **176 → 50**, walked **20**, yellow **0**, torso-to-kitchen **0.692 m**. Bathroom body motion matched the kitchen. Its paint was free **336 → 871**, frontiers **261 → 50**, walked **20**. Settled heading after the stop was **−114.4 deg** on both. That extra yaw is the stop settle, not a second command.

The soft probe stopped because the frozen guess was inside **0.40 m**, not because both arrival bars were met. Remaining **0.159 m** is inside the 0.25 m gap. End yellow is **0**. Both bars are required. This is Prefer FAIL on arrival. The previous probe row (remaining 0.982 m, Δx +0.332 m) is withdrawn.

This is not go-anywhere. The finder may query the yellow log and the frontiers. Arrival stays on the finder's bars.
