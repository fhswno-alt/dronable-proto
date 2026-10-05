# Open-vocab room scores (Prefer FAIL)

`scripts/find_room.py` reads the frozen kit_cam. It does not edit `scripts/find_kitchen.py`. The kitchen Prefer PASS walk on main stays the slab cue.

The model is local OWL-ViT `google/owlvit-base-patch32` at Hub revision `cbc355fb364588351c5d51c7f74465e8e7ec6f72`. No paid API. Objects vote for a room. One score cutoff, **0.06**, applies to every scene. It is unchanged. The commit margin stays **0.15**. Room-name labels are not the query.

This measurement places the stand pose at x = 0.0, 0.3, 0.6, 0.9, and 1.2 m, with y = 0 and yaw = 0, then reads kit_cam. It does not step the gait and it does not publish vel. It does not claim arrival.

A frame scores a room as its best object, plus **0.03** for each other object at or above 0.06, and those extras stop at **0.06**. Extra labels do not add their full scores. The path sums that frame score over the last **3** poses. The pose commits when the leader clears the runner-up by 0.15. A smaller gap stays undecided.

Object map: fridge, oven, stove, kitchen sink, and cupboard vote kitchen. Toilet, bathtub, and bathroom sink vote bathroom. Bed, pillow, nightstand, and lamp vote bedroom. Sofa, TV, and couch vote living. Door, hallway, doormat, shoe rack, and coat hooks vote entrance. Wardrobe and rug are not prompts.

On the bedroom scene, an undecided straight pose logs a re-look and yaws the stand to +0.55, +0.90, −0.75, and −1.05 rad. The first commit ends that look. A pose that still leads by less than 0.15 stays undecided.

## Detection quality

| Scene | Hits | Commit | Lead | Committed false | Bearing std | Max step |
|--|--|--|--|--|--|--|
| Kitchen | 4/5 | kitchen | **0.155** | none | **0.213** | **0.307** |
| Bathroom | 5/5 | bathroom | **0.354** | none | **0.048** | **0.052** |
| Living | 5/5 | living | **0.156** | none | **0.029** | **0.056** |
| Bedroom | 0/5 | none | closest **0.122** | none | — | — |
| Entrance | 4/5 | door | **0.167** | none | **0.005** | **0.007** |
| Empty plant | 0/5 | none | — | none | — | — |

The entrance no longer commits bedroom. The empty plant peaks at **0.011**. Bathroom stays 5/5. Bedroom falls from 3/5 to 0/5. Kitchen falls from 5/5 to 4/5.

### Entrance

Door commits at x = 0.3, 0.6, 0.9, and 1.2 m. The smallest lead on those poses is 0.167. At x = 0.0 m the door is still the top object at 0.158, and the lead over nightstand 0.070 is 0.088, so the pose stays undecided. Nightstand is the only wrong-room object at the cutoff (peak 0.072). Doormat peaks at 0.045 here. Shoe rack peaks at 0.021 and coat hooks at 0.030, both under 0.06. Neither wardrobe nor rug is in the vote.

### Kitchen

Four poses commit. The mean-bearing step on those poses is 0.307 rad. At x = 0.0 m the only kitchen object at the cutoff is the cupboard at 0.106, so the lead is 0.106 and the pose stays undecided. The later pose at x = 1.2 m still commits, lead 0.155, from the fridge and cupboard on the earlier frame.

### Bathroom

Toilet, bathtub, and bathroom sink lead on every pose. The smallest lead is 0.354. Cupboard 0.149, nightstand 0.101, kitchen sink 0.087, and fridge 0.074 clear 0.06 and do not commit.

### Living

Couch, sofa, and TV commit on every pose. The smallest lead is 0.156. Nightstand 0.141, pillow 0.119, bed 0.108, and doormat 0.065 clear 0.06 and do not commit.

### Bedroom

No pose commits, including after the re-look. The closest lead is 0.122, at x = 1.2 m, and the leader there is the bathroom, not the bed. The bathtub peaks at 0.123 on a yaw of −0.75 and does not commit. At the straight pose x = 1.2 m the bathtub is the raw top at 0.092 and the lead is 0.018, so it does not win. Bed peaks at 0.102. The yaws toward the nightstand do not bring nightstand or lamp over a committing lead.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam stays on `head_tilt_link` at `0.050 0.019 0.007`.

Per-pose scores are in `previews/find_room_detect.json`.
