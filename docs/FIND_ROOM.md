# Open-vocab room scores (Prefer FAIL)

`scripts/find_room.py` reads the frozen kit_cam. It does not edit `scripts/find_kitchen.py`. The kitchen Prefer PASS walk on main stays the slab cue.

The model is local OWL-ViT `google/owlvit-base-patch32` at Hub revision `cbc355fb364588351c5d51c7f74465e8e7ec6f72`. No paid API. Objects vote for a room. One score cutoff, **0.06**, applies to every scene. It is unchanged. Room-name labels are not the query.

This measurement places the stand pose at x = 0.0, 0.3, 0.6, 0.9, and 1.2 m, with y = 0 and yaw = 0, then reads kit_cam. It does not step the gait and it does not publish vel. It does not claim arrival.

Object map: fridge, oven, stove, kitchen sink, and cupboard vote kitchen. Toilet, bathtub, and bathroom sink vote bathroom. Bed votes bedroom. Sofa, TV, and couch vote living. Door and hallway vote entrance.

A room's frame vote is the sum of its object scores, and only when at least one of those objects clears 0.06. The path vote is that sum over the last **3** poses. The pose commits when the leader clears the runner-up by **0.15**. A smaller gap stays undecided. Bearing is the score-weighted mean of the committed room's boxes that clear 0.06, over the same three poses.

## Detection quality

| Scene | Hits | Commit | Best object | Committed false | Raw objects ≥ 0.06 that lose the vote | Bearing std | Max step |
|--|--|--|--|--|--|--|--|
| Kitchen | 5/5 | kitchen, lead **0.181** | fridge **0.125** | none | none | **0.233** | **0.307** |
| Bathroom | 5/5 | bathroom, lead **0.559** | toilet **0.399** | none | cupboard 0.149, kitchen sink 0.087, fridge 0.074 | **0.048** | **0.052** |
| Living | 5/5 | living, lead **0.369** | couch **0.245** | none | bed 0.108, cupboard 0.089, fridge 0.065 | **0.029** | **0.056** |
| Bedroom | 0/5 | none, closest lead **0.121** | bed **0.102** | none | couch 0.099, bathtub 0.092, sofa 0.082, cupboard 0.078 | — | — |
| Entrance | 5/5 | entrance, lead **0.188** | door **0.164** | none | none | **0.005** | **0.007** |
| Empty plant | 0/5 | none | **0.010** (hallway) | none | none | — | — |

The empty plant stays under 0.06. Entrance and living still hit 5/5.

### Kitchen

Cupboard and fridge both clear 0.06, and both vote kitchen. The raw fridge box at x = 0.6 m sits at bearing −0.891. The three-pose mean at that pose is −0.118. The mean then moves through −0.224 to −0.398 as the window slides. The largest step of the mean is 0.307 rad, down from the raw box step of 1.072. All five poses commit kitchen. The smallest lead is 0.181.

### Bathroom

Toilet, bathtub, and bathroom sink sum ahead of the kitchen objects on every pose. The smallest lead is 0.559. Toilet still has the highest single score, 0.399. Cupboard 0.149, kitchen sink 0.087, and fridge 0.074 still clear 0.06, and none of them commits. The mean bearing stays between +0.290 and +0.410 because the bathtub box is in the same sum. Its largest step is 0.052 rad.

### Living

Couch, sofa, and TV keep the living vote ahead on every pose. The smallest lead is 0.369. Bed 0.108, cupboard 0.089, and fridge 0.065 clear 0.06 and do not commit. The mean bearing stays near −0.20 to −0.12. Its largest step is 0.056 rad.

### Bedroom

No pose commits. The largest lead in the window is 0.121, under 0.15, so the pose stays undecided. At x = 1.2 m the raw top object is the bathtub at 0.092 and the window lead is 0.016, so the bathtub does not win. Bed peaks at 0.102 and does not clear the margin against couch and sofa.

### Entrance

Door wins all five poses. Hallway stays under 0.06. The smallest lead is 0.188. The mean bearing stays within about ±0.01. Its largest step is 0.007 rad.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam stays on `head_tilt_link` at `0.050 0.019 0.007`.

Per-pose scores are in `previews/find_room_detect.json`.
