# Explore / map (Prefer FAIL)

`scripts/explore_map.py` walks the frozen plant and builds a partial map from `kit_cam`. It is not a room finder, not SLAM, and not go-anywhere. Finders stay last mile. They can ask this map two questions later: "have I seen kitchen-like yellow?" and "which cells are frontiers?"

Voice still refuses "explore", "build a map", and "go anywhere" (`scripts/voice_caller.py`). This script is not wired to that caller.

## What is frozen

The walk plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. This slice does not edit that file, the gait, the tip check, `CommandBus`, or the `kit_cam` mount. The camera stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. One camera. No lidar. No second camera. No GPU and no paid API.

Commands are only `stand`, `stop`, and `vel(vx, yaw_rate)` on `CommandBus` in `scripts/steer_walk.py`. Resend is 10 Hz. 200 ms of silence stands. Caps stay **+0.150 / −0.032** m/s and yaw **±0.25** rad/s. This script does not raise them. The walk uses the kit gait. `vel(0, yaw)` does change heading on this row. This script does not send it.

The empty-plant walk is the nav-multi chain on this CommandBus, not a frontier replan. Stand is **1.0 s**. Then `vel(+0.150, 0)` for 15 s, `vel(+0.150, +0.25)` for 12.5 s, forward 6 s, `vel(+0.150, −0.25)` for 11 s, and forward 6 s. Between those windows this tip stops, asks one scene question, and then sends the next vel. The phase close with those stops is in Stop-look-ask. The uninterrupted close, with no question, stays in Measured. The stale **+0.598 m / +75.7 deg** envelope is not this path.

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

The empty plant walks the remeasured nav-multi chain at the Day1 forward clamp, with one scene question at each vel stop. The 4 s / 8 s / 34 s frontier holds are refused as claims. Furnished scenes take the remeasured 8 s right. The paint is still a fan from one camera, out to 2.60 m with one-cell holes filled. A yellow bearing is logged when the backsplash is in frame, and a soft XY is frozen along it. Frontiers are the 8-connected edge of the paint. White and gray furniture can fail the saturation test and never become a cell. That is a partial feature map of whatever was in view. It is not go-anywhere. It is not arrival. It is not a demo-ready human walk. A room word from the stop question is a label in memory. It is not a waypoint and it does not satisfy arrival.

The empty plant is the honest miss: floor cells, no yellow, no room label.

```bash
MUJOCO_GL=osmesa python scripts/explore_map.py --self-test
MUJOCO_GL=osmesa python scripts/explore_map.py --demo
```

`--demo` loads the pinned Moondream2, asks once on a stand frame of each scene, then walks the empty plant on the nav-multi chain and the kitchen and bathroom on the 8 s right. Each vel window ends in stop, one `kit_cam` frame, and one question. The sim stays paused while the model answers. The 0.40 s map paint does not ask. It then runs a 20 s half-cap probe toward the frozen soft XY in the kitchen, with no scene question. That probe's old Δx is not a current claim. Each run stops and settles 1.2 s. It writes `previews/explore_map_summary.json`, kit_cam and map stills, `previews/explore_map_kitchen.mp4`, and `previews/explore_map_kitchen_soft.mp4`. Exit status is non-zero if the plant md5 changes, if `up_z` drops below 0.90, or if a `vx = 0` yaw is sent. A passing exit still prints Prefer FAIL for go-anywhere and for arrival.

## Measured

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Kit gait. Video off. This table is the uninterrupted `run_explore` / `run_soft_goal` on the CommandBus, with no scene question. The stop-look-ask close is the next section. Commands at the Day1 clamp `vx = +0.150` except the soft probe at `+0.075`. `vx = 0` yaw sends: 0. `QUALIFIED_FRONTIER_CHAIN` stayed false. No tip. No fault. End mode stand. Arrival was not claimed.

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

## Stop-look-ask

At the end of each vel window the schedule sends `stop`, waits **0.20 s**, grabs one `kit_cam` frame, and asks Moondream2 once. The 0.40 s map paint does not ask. The sim clock stays still while the model runs. The answer is remembered. `none` and a hedge keep the next scheduled vel. The first time a room word appears, it is only stored. The same word on a later stop inserts `vel(−0.032, 0)` for **2.0 s**, then the scheduled window still runs. A second reverse in a row is not inserted. The walk-budget stop asks once and the decision is stop. A room word is not a waypoint, not a house map, and not a scripted room.

Hub main for `vikhyatk/moondream2` is still `5d6c926f44e26b07957b0dd315bbedcb4c17a5fe`. The card license is `apache-2.0`. The weights file sha256 is `70a7d94c0c8349eb58ed2d9e636ef2d0916960f321ecabeac6354b8ba3d7403f`. Prompt: "Which room is this? Answer with one word: kitchen, bathroom, living, bedroom, entrance, or none." CPU seconds below are this agent VM (torch CPU, 4 threads), not a laptop score. `QUALIFIED_FRONTIER_CHAIN` stayed false. Plant md5 after the runs was still `207f3d5e9c6a72e16f7aa0c8d224f75e`.

There is no corridor scene on this branch, so corridor is not scored. The entrance file is a visual door frame (no hinge). The XML file name is not the expected answer.

Stand, **1.0 s**, one `kit_cam` frame, one question. min up_z **0.934**. No fault.

| Label | Scene | Answer | Raw | Conf | CPU s |
|-------|-------|--------|-----|------|-------|
| empty | plant | none | None | 0.639 | 5.55 |
| door | entrance | entrance | entrance | 0.894 | 5.69 |
| corridor | no scene on this branch | — | — | — | — |
| kitchen | kitchen | living | living | 0.730 | 5.31 |
| bathroom | bathroom | living | living | 0.752 | 4.65 |
| living | living | living | living | 0.732 | 4.90 |
| bedroom | bedroom | living | living | 0.470 | 4.80 |

Kitchen, bathroom, living, and bedroom all returned **living**. The door frame returned **entrance**. The empty stand returned **none**. That is Prefer FAIL on separating the furnished rooms.

Empty-plant walk with a question at every vel stop, including the budget stop. Both passes walked the same phase close. No backtrack was inserted. min up_z **0.934**. No fault. End mode stand. World at the stop **+3.030 m, +3.109 m**, yaw **+7.34 deg**. After the 1.2 s settle, **+3.028 m, +3.108 m**, yaw **+7.37 deg**. `vx = 0` yaw sends: 0.

| Phase | Command | Heading Δx | Δyaw | min up_z |
|-------|---------|------------|------|----------|
| approach 15 s | `vel(+0.150, 0)` | +2.210 m | +1.39 deg | 0.950 |
| left 12.5 s, after the first stop | `vel(+0.150, +0.25)` | +0.184 m | +153.05 deg | 0.950 |
| mid 6 s | `vel(+0.150, 0)` | +0.785 m | −0.58 deg | 0.951 |
| right 11 s | `vel(+0.150, −0.25)` | +0.295 m | −132.23 deg | 0.950 |
| resume 6 s | `vel(+0.150, 0)` | +0.785 m | −2.16 deg | 0.951 |

The approach matches the uninterrupted row. The later windows do not. Each **0.20 s** stop breaks the gait, so the left arc is **+153.05 deg** here and **+145.1 deg** on the uninterrupted close.

Questions on the remeasure, same poses as the phase table:

| t | Answer | Raw | Conf | CPU s | Next |
|---|--------|-----|------|-------|------|
| 16.20 s | none | None | 0.493 | 5.66 | scheduled vel |
| 28.90 s | none | None | 0.676 | 4.60 | scheduled vel |
| 35.10 s | none | None | 0.679 | 4.13 | scheduled vel |
| 46.30 s | living | living | 0.386 | 3.94 | scheduled vel |
| 52.50 s | none | None | 0.517 | 4.16 | stop |

An earlier pass at those same four stops, before the budget stop asked, answered **none** every time (conf 0.472, 0.680, 0.677, 0.359; 4.72 s, 4.67 s, 4.68 s, 4.67 s). The arc-right frame then said **none** and on the remeasure said **living**. The body trace did not change. The label is not stable on the empty plant. The single **living** was the first copy of that word, so the resume stayed `vel(+0.150, 0)`. A second **living** would have inserted the 2 s reverse. That reverse did not run.

CPU time on these asks ran from **3.94 s** to **5.69 s**. This is not go-anywhere and not arrival. Yellow ≥ 0.50 and torso-to-kitchen ≤ 0.25 m are still both required, and this loop did not meet them. The furnished 8 s right was not re-walked; those scenes are the stand rows above.

## Living collapse (Prefer FAIL)

The frozen prompt stays. Labels stay `kitchen`, `bathroom`, `living`, `bedroom`, `entrance`, and `none`. Hub pin stays `5d6c926f44e26b07957b0dd315bbedcb4c17a5fe`, apache-2.0. The same 55 fresh stills from the Moondream score were rescored on these pixels, plus the six current-plant stand frames from the table above, saved and scored as files. The frozen prompt reproduced that score: fresh living-collapse **6/44**, named hits **47**, empty plant **none**. The current-plant stand still answers kitchen, bathroom, and bedroom as **living** (**3/3**).

Two other wordings were scored with the same pin and the same parser. Neither is wired.

The furniture prompt told the model not to guess living, defined each label by visible furniture, and still required one frozen word. On the 61 frames, living-collapse went from **6/44** to **36/44** on the fresh stills, named hits from **47** to **19**, and the empty plant from **none** to **living**. The current-plant trio stayed **3/3** living, and the empty stand also became **living**. CPU median **3.96 s** (range 1.65–5.23) against **3.84 s** (range 1.56–5.52) for the frozen prompt on the same pass.

The shorter choice-order prompt was scored on the 36-frame collapse subset (iTHOR kitchen, bedroom, and bathroom; Places bathroom and bedroom; both kit sets). Living-collapse on that subset went from **7** to **8**. Named hits went from **26** to **24**. Both empty frames went from **none** to **bedroom**. The current-plant kitchen left living and landed on **bedroom**. Bathroom and bedroom stayed **living**. CPU median **4.21 s** (range 1.83–4.61).

OWLv2 `google/owlv2-base-patch16-ensemble` (`cfd3195ba4ea9592eec887ded089f4c08eff231d`, apache-2.0) was the tie-break only when the frozen prompt said living or hedged. Cutoff **0.10** was the processor default, set before the scores. Objects map only onto the frozen labels: bed to bedroom, toilet or bathtub to bathroom, stove, oven, or refrigerator to kitchen, sofa or television to living. A door is logged and is not a room. Two rooms at the cutoff return none. The tie-break is not wired.

| Set | Frozen prompt | After tie-break |
|-----|---------------|-----------------|
| Fresh living-collapse | 6/44 | 1/44 |
| Fresh named hits | 47 | 42 |
| Empty plant, secondary alone | none | none |
| Current-plant kitchen / bathroom / bedroom | living / living / living | none / none / living |
| True labels on that trio | 0/3 | 0/3 |

The five fresh hit losses are living rooms whose sofa or television fired together with a bed, a toilet, or a stove, so the tie-break returned none. The current-plant bedroom stayed living on a television score of **0.312** with the bed at **0.027**. The collapse count dropped because answers became none, not because kitchen, bathroom, and bedroom separated. Empty stays none. This is Prefer FAIL. The primary model stays Moondream2 with the frozen prompt.

## Primary model swap (Prefer FAIL)

The frozen prompt and the frozen labels stay. The question is still one word among kitchen, bathroom, living, bedroom, entrance, and none. A hedge stays undecided. This pass puts a different model in front of that question on the same 55 fresh stills and the same six current-plant stand files. The prompt is unchanged. A miss stays a miss.

Moondream2 on those pixels, frozen prompt: fresh living-collapse **6/44**, named hits **47**, empty plant **none**, current-plant kitchen / bathroom / bedroom **living / living / living** (true **0/3**). CPU median **3.84 s** (range 1.56–5.52).

Three candidates. None is wired.

OWLv2 `google/owlv2-base-patch16-ensemble` (`cfd3195ba4ea9592eec887ded089f4c08eff231d`, apache-2.0) is the answer on every frame. Cutoff **0.10** and the object map are the tie-break rule above. A door is logged and is not a room. Two rooms at the cutoff return none. The vote is an object count. The model was not asked the which-room sentence. The per-frame vote is the tie-break secondary column, read here as the primary label. It was not run a second time.

SmolVLM-256M `HuggingFaceTB/SmolVLM-256M-Instruct` revision `7e3e67edbbed1bf9888184d9df282b700a323964`, apache-2.0, weights sha256 `74dea5904032e5ae99a2e0eef5179e6ac0f1dedc3ab0c7c2a5d4d387c843203e`. Greedy decode (`do_sample` false), 24 new tokens, the same parser as `scripts/room_ask.py`.

SmolVLM-500M `HuggingFaceTB/SmolVLM-500M-Instruct` revision `a7da5b986cb59b408707209984f360a5f4ad7e47`, apache-2.0, weights sha256 `d05b567eeaf534e83d375551f068ed57b5f52d37c657197f644af5ef9db091a2`. Same decode and the same parser.

| Primary | Fresh living-collapse | Fresh named hits | Empty | Current-plant kitchen / bathroom / bedroom | True | CPU median s |
|---------|----------------------|------------------|-------|--------------------------------------------|------|--------------|
| Moondream2 frozen | 6/44 | 47 | none | living / living / living | 0/3 | 3.84 (1.56–5.52) |
| OWLv2 ensemble object vote | 3/44 | 19 | none | none / none / living | 0/3 | object vote, untuned this pass |
| SmolVLM-256M | 7/44 | 30 | bathroom | bathroom / kitchen / kitchen | 0/3 | 2.21 (1.89–2.69) |
| SmolVLM-500M | 7/44 | 34 | kitchen | kitchen / kitchen / kitchen | 1/3 | 2.44 (1.96–2.99) |

The OWLv2 living-collapse count is **3/44** because non-living frames became none. Named hits fell from **47** to **19**. Empty stays none. The current-plant trio is none / none / living, true **0/3**. Bedroom stayed living on television **0.312** with the bed at **0.027**. Kitchen fired bathtub **0.138** and television **0.123** together, so the vote is none. Bathroom toilet **0.055** stayed under **0.10**.

SmolVLM-256M living-collapse is **7/44**. Named hits **30**. The old kit_cam plant frame and the current-plant empty stand both answered **Bathroom.** The current-plant trio is bathroom / kitchen / kitchen, true **0/3**. The door stand answered **Entrance.** CPU median **2.21 s** (range 1.89–2.69). The four old kit_cam furnished frames in the fresh set answered kitchen, bathroom, living, and bedroom. Those frames are the earlier plant. They are not the current-plant trio.

SmolVLM-500M living-collapse is **7/44**. Named hits **34**. Both empty frames answered **Kitchen.** The door stand answered **Kitchen.** The current-plant kitchen, bathroom, living, and bedroom stands all answered **Kitchen.** The trio is kitchen / kitchen / kitchen, true **1/3**. CPU median **2.44 s** (range 1.96–2.99).

Qwen2-VL-2B stayed unloaded. On torch 2.14.1+cpu, torchvision 0.29.1 raises `operator torchvision::nms does not exist`. That wheel stayed off this machine. The #46 selection-set SmolVLM figure (iTHOR named 9/20, prompt with no none word) is a different still set.

No candidate kept empty as none and gave kitchen, bathroom, and bedroom different true labels. Named hits stayed below **47**. This is Prefer FAIL. The primary stays Moondream2 with the frozen prompt. This is not go-anywhere. Plant md5 stayed `207f3d5e9c6a72e16f7aa0c8d224f75e`.

## Post-stop settle (separate envelope)

`ASK_STOP_S` stays **0.20 s**. A 1.0 s stand before the ask was measured with `ask_hold_s` on the empty plant, frozen prompt, same five vel windows. No backtrack was inserted. min up_z **0.934**. No fault. End mode stand. World at the stop **+2.944 m, +3.134 m**, yaw **+9.34 deg**. After the 1.2 s settle, **+2.944 m, +3.134 m**, yaw **+9.35 deg**. Plant md5 stayed `207f3d5e9c6a72e16f7aa0c8d224f75e`.

| Phase | 0.20 s interrupt | 1.0 s stand | No-ask unload |
|-------|------------------|-------------|----------------|
| approach 15 s | +2.210 m / +1.39 deg | +2.210 m / +1.39 deg | +2.210 m / +1.4 deg |
| left 12.5 s | +0.184 m / +153.05 deg | +0.154 m / +154.45 deg | +0.420 m / +145.1 deg |
| mid 6 s | +0.785 m / −0.58 deg | +0.784 m / −0.89 deg | +0.944 m / +12.3 deg |
| right 11 s | +0.295 m / −132.23 deg | +0.308 m / −131.62 deg | +0.466 m / −138.6 deg |
| resume 6 s | +0.785 m / −2.16 deg | +0.785 m / −2.04 deg | +0.958 m / −2.5 deg |

The no-ask column is the uninterrupted Measured table. It is not replaced by either ask row. The 1.0 s left arc is **+154.45 deg**. That does not close the no-ask left of **+145.1 deg** or the no-ask right of **−138.6 deg**. The five answers were none, none, none, living, none. The stand did not remove the living label. CPU on those five asks was 5.40, 4.77, 4.43, 3.98, and 4.46 s.
