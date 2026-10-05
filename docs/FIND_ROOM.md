# Open-vocab room scores (Prefer FAIL)

`scripts/find_room.py` reads the frozen kit_cam. It does not edit `scripts/find_kitchen.py`. The kitchen Prefer PASS walk on main stays the slab cue.

The model is local OWL-ViT `google/owlvit-base-patch32` at Hub revision `cbc355fb364588351c5d51c7f74465e8e7ec6f72`. No paid API. Objects vote for a room. One score cutoff, **0.06**, applies to every scene. It is unchanged. The commit margin stays **0.15**. Room-name labels are not the query.

This measurement places the stand pose at x = 0.0, 0.3, 0.6, 0.9, and 1.2 m, with y = 0 and yaw = 0, then reads kit_cam. It does not step the gait and it does not publish vel. It does not claim arrival.

Object map: fridge, oven, stove, kitchen sink, and cupboard vote kitchen. Toilet, bathtub, and bathroom sink vote bathroom. Bed, pillow, nightstand, wardrobe, lamp, and rug vote bedroom. Sofa, TV, and couch vote living. Door and hallway vote entrance.

A room's frame vote is the sum of its object scores, and only when at least one of those objects clears 0.06. The path vote is that sum over the last **3** poses. The pose commits when the leader clears the runner-up by **0.15**. A smaller gap stays undecided. On the bedroom scene, an undecided straight pose logs a re-look and yaws the stand to +0.55, +0.90, −0.75, and −1.05 rad, toward the nightstand and then the dresser. The first commit ends that look. A pose that still leads by less than 0.15 stays undecided.

## Detection quality

| Scene | Hits | Straight | Re-looks | Commit | Committed false | Bearing std | Max step |
|--|--|--|--|--|--|--|--|
| Kitchen | 5/5 | 5 | 0 | kitchen, lead **0.181** | none | **0.233** | **0.307** |
| Bathroom | 5/5 | 5 | 0 | bathroom, lead **0.559** | none | **0.048** | **0.052** |
| Living | 4/5 | 4 | 0 | living, lead **0.275** | none | **0.008** | **0.016** |
| Bedroom | 3/5 | 2 | 3 | bedroom, lead **0.218** | none | **0.326** | **0.788** |
| Entrance | 0/5 | 0 | 0 | bedroom on 4 poses | wardrobe 0.150, rug 0.093, nightstand 0.072 | — | — |
| Empty plant | 0/5 | 0 | 0 | none | none | — | — |

Kitchen's mean-bearing step stays **0.307** rad. The empty plant peaks at **0.010**. Bathroom does not commit the kitchen objects or the nightstand. The bedroom sum does commit on the entrance, and the living room loses the pose at x = 0.0 m.

### Kitchen

Unchanged from the previous tip. Cupboard and fridge carry the vote. All five poses commit. The smallest lead is 0.181. The new bedroom prompts stay under 0.06 on this scene (wardrobe peaks at 0.053).

### Bathroom

Toilet, bathtub, and bathroom sink still lead on every pose. The smallest lead is 0.559. Nightstand now clears 0.06 as well (peak 0.101), with cupboard 0.149, kitchen sink 0.087, and fridge 0.074. None of them commits.

### Living

Four poses commit. At x = 0.0 m the lead falls to 0.086, under 0.15, so that pose stays undecided. Rug 0.164, nightstand 0.114, and pillow 0.079 are what shrink the lead. Couch still has the highest single score, 0.240.

### Bedroom

Three poses commit. The bathtub does not win any of them.

| x | Straight | After re-look |
|--|--|--|
| 0.0 | undecided, lead 0.058 | still undecided after all four yaws |
| 0.3 | undecided, lead 0.117 | commits at yaw −1.05, lead 0.218, bed 0.080 and nightstand 0.078 |
| 0.6 | undecided, lead 0.117 | still undecided after all four yaws |
| 0.9 | commits, lead 0.271, rug 0.073 | no re-look |
| 1.2 | commits, lead 0.242 | no re-look. Raw top is the bathtub at 0.092, and it does not win |

The yaws toward the nightstand (+0.55 and +0.90) drop the bed under 0.06 and do not raise the nightstand or the lamp over 0.06. The pose stays a re-look, not a forced commit.

### Entrance

Door remains the highest single score (0.112 to 0.164). Wardrobe scores 0.101 to 0.150 on that same view, and with rug and nightstand the bedroom sum leads by as much as 0.442. Four poses commit bedroom. x = 0.0 m leads by 0.102 and stays undecided. This is a wrong commit. The margin was not lowered to undo it.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam stays on `head_tilt_link` at `0.050 0.019 0.007`.

Per-pose scores are in `previews/find_room_detect.json`.
