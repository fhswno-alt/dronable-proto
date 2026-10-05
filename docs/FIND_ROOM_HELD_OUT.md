# Held-out room scores (Prefer FAIL)

`scripts/find_room.py` is unchanged. Cutoff **0.06**, margin **0.15**, bonus **0.03**, cap **0.06**. One still is one frame, so the frozen window is that frame and a commit still needs a lead of 0.15. Nothing here edits the plant, steps a gait, or publishes vel. This is not an arrival.

The five MuJoCo kit_cam rooms were not rendered again and were not used to change the object map.

## Install

`ai2thor` **5.0.0** installed. The CloudRendering build downloaded, then Unity aborted:

`vkEnumeratePhysicalDevices: Invalid instance [VUID-vkEnumeratePhysicalDevices-instance-parameter]`, signal 6, `returncode=-6`. No Vulkan physical device.

The Linux64 build rendered on `DISPLAY=:1`. Camera height **0.38 m**, field of view **104.82°**, 640×480, level. The still stands on a reachable floor point about 1.5 m from the object centroid and looks at that centroid.

`prior` **1.0.3** loaded `procthor-10k`. Places365 `val_256.tar` (525158400 bytes) is a public file. No paid call.

## iTHOR (primary)

Five scenes per class: kitchens `FloorPlan1`–`5`, living `FloorPlan201`–`205`, bedrooms `FloorPlan301`–`305`, bathrooms `FloorPlan401`–`405`. Stills are in `previews/held_out_thor/`. Scores are in `previews/find_room_thor.json`.

| Scene | Hits | Commit | Lead | Committed false |
|--|--|--|--|--|
| Kitchen | 0/5 | none | closest **0.034** | none |
| Living | 0/5 | none | closest **0.145** | none |
| Bedroom | 0/5 | none | closest **0.072** | none |
| Bathroom | 0/5 | none | closest **0.128** | none |

Every pose stays undecided. Kitchen's closest lead is 0.034 on `FloorPlan4`, and the leader is kitchen; the top box is TV at 0.154. Living's closest lead is 0.145 on `FloorPlan204`, and the leader is bedroom (bed 0.180). That is under 0.15, so it does not commit. Bedroom's closest lead is 0.072 on `FloorPlan305`, leader entrance (shoe rack 0.072). Bathroom's closest lead is 0.128 on `FloorPlan404`, leader bathroom (bathroom sink 0.194).

iTHOR has no entrance scene.

## ProcTHOR entrance

`procthor-10k` has only Bedroom (14464), Bathroom (10901), LivingRoom (10763), and Kitchen (9032) across train, val, and test. Door assets are interior `Doorway_*` and `Doorframe_*`. There is no hallway room and no exterior door. Those interiors were not relabeled as entrance.

## Places365 (secondary, eye height)

Not the 0.38 m table. Five validation photos per class, 256 px, from `val_256`. Stills are in `previews/held_out_places/`. Scores are in `previews/find_room_places.json`.

| Scene | Hits | Commit | Lead | Committed false |
|--|--|--|--|--|
| Kitchen | 2/5 | kitchen | **0.155** | none |
| Bathroom | 3/5 | bathroom | **0.164** | none |
| Bedroom | 3/5 | bedroom | **0.205** | none |
| Living | 1/5 | living | **0.200** | none |
| Entrance hall | 0/5 | none | closest **0.139** | none |
| Corridor | 0/5 | none | closest **0.090** | none |

Entrance hall's closest lead is 0.139, leader entrance (door 0.109). Corridor's closest lead is 0.090, leader entrance (door 0.090). Neither commits. No wrong room commits on these thirty photos.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam on the MuJoCo plant stays on `head_tilt_link` at `0.050 0.019 0.007`.
