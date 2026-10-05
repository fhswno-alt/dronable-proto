# Open-vocab room scores (Prefer FAIL)

`scripts/find_room.py` reads the frozen kit_cam. It does not edit `scripts/find_kitchen.py`. The kitchen Prefer PASS walk on main stays the slab cue.

The model is local OWL-ViT `google/owlvit-base-patch32` at Hub revision `cbc355fb364588351c5d51c7f74465e8e7ec6f72`. No paid API. Objects vote for a room. One score cutoff, **0.06**, applies to every scene. It is unchanged. The commit margin stays **0.15**. Room-name labels are not the query.

This measurement places the stand pose at x = 0.0, 0.3, 0.6, 0.9, and 1.2 m, with y = 0 and yaw = 0, then reads kit_cam. It does not step the gait and it does not publish vel. It does not claim arrival.

A frame scores a room as its best object, plus **0.03** for each other object at or above 0.06, and those extras stop at **0.06**. Extra labels do not add their full scores. The path sums that frame score over the last **3** poses. The pose commits when the leader clears the runner-up by 0.15. A smaller gap stays undecided.

Object map: fridge, oven, stove, kitchen sink, and cupboard vote kitchen. Toilet, bathtub, and bathroom sink vote bathroom. Bed, pillow, mattress, and upholstered bed vote bedroom. Sofa, TV, and couch vote living. Door, hallway, doormat, shoe rack, and coat hooks vote entrance. Wardrobe and rug are not prompts. Headboard is not a prompt. Nightstand is not a prompt.

On the bedroom scene, an undecided straight pose logs a re-look and yaws the stand to +0.55, +0.90, −0.75, and −1.05 rad. The first commit ends that look. A pose that still leads by less than 0.15 stays undecided.

## Detection quality

| Scene | Hits | Commit | Lead | Committed false | Bearing std | Max step |
|--|--|--|--|--|--|--|
| Kitchen | 4/5 | kitchen | **0.155** | none | **0.213** | **0.307** |
| Bathroom | 5/5 | bathroom | **0.354** | none | **0.048** | **0.052** |
| Living | 5/5 | living | **0.221** | none | **0.029** | **0.056** |
| Bedroom | 3/5 | bedroom | **0.158** | none | **0.014** | **0.034** |
| Entrance | 5/5 | door | **0.158** | none | **0.005** | **0.007** |
| Empty plant | 0/5 | none | — | none | — | — |

The entrance commits the door on every pose and does not commit bedroom. The empty plant peaks at **0.011**. Bathroom stays 5/5. Kitchen stays 4/5. Living stays 5/5. Bedroom commits 3/5.

### Entrance

Door commits at x = 0.0, 0.3, 0.6, 0.9, and 1.2 m. The smallest lead is 0.158, at x = 0.0 m, where the door at 0.158 is the only object at the cutoff. No bedroom object clears 0.06. Mattress peaks at 0.033 and the upholstered bed at 0.040. Pillow peaks at 0.021 and bed at 0.032. Doormat peaks at 0.046 here. Hallway peaks at 0.030, coat hooks at 0.015, and the shoe rack at 0.012, all under 0.06. Neither wardrobe nor rug is in the vote.

### Kitchen

Four poses commit. The mean-bearing step on those poses is 0.307 rad. At x = 0.0 m the only kitchen object at the cutoff is the cupboard at 0.106, so the lead is 0.106 and the pose stays undecided. The pose at x = 1.2 m commits, lead 0.155, from the fridge and cupboard on the earlier frame. No bedroom object clears 0.06 on this scene.

### Bathroom

Toilet, bathtub, and bathroom sink lead on every pose. The smallest lead is 0.354. Cupboard 0.149, kitchen sink 0.087, and fridge 0.074 clear 0.06 and do not commit.

### Living

Couch, sofa, and TV commit on every pose. The smallest lead is 0.221. Pillow 0.119, bed 0.108, cupboard 0.089, upholstered bed 0.075, fridge 0.065, and doormat 0.065 clear 0.06 and do not commit. Mattress stays under the cutoff (peak 0.046).

### Bedroom

Three straight poses commit, at x = 0.6, 0.9, and 1.2 m. The smallest lead is 0.158, at x = 1.2 m. Mattress is the strongest bedroom object (peak 0.125 at x = 0.0 m). At x = 1.2 m mattress is 0.095, the upholstered bed is 0.089, and bed is 0.074, against bathtub 0.092. The bathtub does not win. At x = 0.9 m nothing on that frame clears 0.06. The commit is the window of the two earlier frames, lead 0.166 over living.

x = 0.0 m and x = 0.3 m stay undecided. The straight leads are 0.056 and 0.127. The re-look does not commit. The closest re-look lead is 0.140, at x = 0.3 m and yaw −1.05. Pillow stays under 0.06 on this scene (peak 0.026).

Headboard, duvet, and bedside lamp are not prompts. On the same path, headboard cleared 0.06 on the entrance (peak 0.072) and on the kitchen (0.062), and that kitchen hit dropped the x = 1.2 m commit. Duvet peaked at 0.018 on the bed. Bedside lamp peaked at 0.013. A comforter query peaked at 0.060 and did not carry a committing lead. Those labels stayed off the map.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam stays on `head_tilt_link` at `0.050 0.019 0.007`.

Per-pose scores are in `previews/find_room_detect.json`.
