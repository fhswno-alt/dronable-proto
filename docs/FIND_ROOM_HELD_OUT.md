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

## Margin off, same stills

The object map is unchanged. Cutoff 0.06 stays. Margin off means the highest room at that cutoff is the guess, even when the lead is under 0.15. A tie, or a frame with nothing at 0.06, is not a guess. Cutoff off is the room whose best object score is highest, including scores under 0.06. A five-way uniform guess is right about 4 times in 20.

| Source | Scene | Margin 0.15 | Margin off top-1 | Cutoff off top-1 |
|--|--|--|--|--|
| iTHOR | Kitchen | 0/5 | 2/5 | 2/5 |
| iTHOR | Living | 0/5 | 2/5 | 2/5 |
| iTHOR | Bedroom | 0/5 | 1/5 | 1/5 |
| iTHOR | Bathroom | 0/5 | 2/5 | 3/5 |
| Places365 | Kitchen | 2/5 | 5/5 | 5/5 |
| Places365 | Bathroom | 3/5 | 4/5 | 4/5 |
| Places365 | Bedroom | 3/5 | 5/5 | 5/5 |
| Places365 | Living | 1/5 | 3/5 | 3/5 |
| Places365 | Entrance hall | 0/5 | 1/5 | 4/5 |
| Places365 | Corridor | 0/5 | 1/5 | 5/5 |

On the 0.38 m stills the margin-off top-1 is **7/20** and the cutoff-off top-1 is **8/20**. Ten frames never clear 0.06, so the margin is not what keeps them undecided. Prefer FAIL: **OWL-ViT base-p32 is the limit** on iTHOR.

On the eye-height kitchen, bathroom, bedroom, and living photos the margin-off top-1 is **17/20** and the 0.15 margin commits **9/20**. Prefer FAIL on that secondary set: **margin too strict**.

## Other local models, same stills

Labels, cutoff, and margin stayed frozen for the detectors. The VLMs were asked `which room is this? kitchen/bathroom/living/bedroom/entrance`. Scores are in `previews/find_room_swap.json`.

| Model | iTHOR margin 0.15 | iTHOR top-1 or named | Places top-1 or named |
|--|--|--|--|
| OWL-ViT base-patch32 | 0/20 | 7/20 margin off | 17/20 on the four rooms |
| OWLv2 base-patch16 | 1/20 | 10/20 margin off | 18/20 on the four rooms |
| Grounding DINO tiny | 2/20 | 4/20 margin off | 10/20 on the four rooms |
| SmolVLM-256M | — | 9/20 named | 11/30 named |
| Moondream2 | — | 20/20 named | 26/30 named |
| Qwen2-VL-2B | blocked | blocked | blocked |

OWLv2's one iTHOR commit is a bedroom hit. It also commits entrance on kitchen `FloorPlan2`. Its kitchen margin-off top-1 is 0/5. Grounding DINO's two commits are bedroom hits, and four other iTHOR frames commit the wrong room. SmolVLM names 9/20 iTHOR rooms, with 3 wrong names. Moondream2 names all 20 iTHOR rooms and 26/30 Places photos. That is a direct room question, not the object map, and it is not wired into the walk.

Qwen2-VL-2B did not score. `Qwen2VLVideoProcessor requires the Torchvision library.` `torchvision==0.29.1` on torch 2.14.1+cpu then raised `RuntimeError: operator torchvision::nms does not exist`. That wheel was removed.

The iTHOR and Places stills in this file are the set that selected Moondream2. They are not its final score. The fresh score is in `docs/FIND_ROOM_MOONDREAM.md`.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam on the MuJoCo plant stays on `head_tilt_link` at `0.050 0.019 0.007`.
