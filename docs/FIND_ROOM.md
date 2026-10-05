# Open-vocab room scores (Prefer FAIL)

`scripts/find_room.py` reads the frozen kit_cam. It does not edit `scripts/find_kitchen.py`. The kitchen Prefer PASS walk on main stays the slab cue.

The model is local OWL-ViT `google/owlvit-base-patch32` at Hub revision `cbc355fb364588351c5d51c7f74465e8e7ec6f72`. No paid API. Objects vote for a room. One score cutoff, **0.06**, applies to every scene. Room-name labels are not the query.

This measurement places the stand pose at x = 0.0, 0.3, 0.6, 0.9, and 1.2 m, with y = 0 and yaw = 0, then reads kit_cam. It does not step the gait and it does not publish vel. It does not claim arrival. The earlier gait log in `previews/find_room_summary.json` is not this result.

Object map: fridge, oven, stove, kitchen sink, and cupboard vote kitchen. Toilet, bathtub, and bathroom sink vote bathroom. Bed votes bedroom. Sofa, TV, and couch vote living. Door and hallway vote entrance.

A room hits when its own object is the unique top score and that score is at least 0.06. A wrong-room object at or above 0.06 is a false positive even when it does not win. Bearing is the kit_cam angle of the winning box on the poses that hit, in radians. The empty plant has no true room, so any selection would be a false positive.

## Detection quality

| Scene | Hits | Object | Best score | False objects (≥ 0.06) | Bearing std | Max step |
|--|--|--|--|--|--|--|
| Kitchen | 3/5 | fridge, also cupboard | fridge **0.125** | none | **0.509** | **1.072** |
| Bathroom | 5/5 | toilet | **0.399** | cupboard 0.149, kitchen sink 0.087, fridge 0.074 | **0.024** | **0.042** |
| Living | 5/5 | couch | **0.245** | bed 0.108, cupboard 0.089, fridge 0.065 | **0.125** | **0.360** |
| Bedroom | 2/5 | bed | **0.102** | couch 0.099, bathtub 0.092, sofa 0.082, cupboard 0.078 | **0.011** | **0.023** |
| Entrance | 5/5 | door | **0.164** | none | **0.008** | **0.020** |
| Empty plant | 0/5 | none | **0.010** (hallway) | none | — | — |

The empty plant stays under 0.06. The cutoff stays one number. Raising it would drop the bedroom bed (0.094 at x = 0.3 m) and the kitchen cupboard (0.094 at x = 0.3 m).

### Kitchen

Cupboard fires at x = 0.0 (0.106, bearing +0.194) and x = 0.3 (0.094, bearing +0.181). Fridge fires at x = 0.6 (0.125, bearing −0.891). Both vote kitchen, and no wrong-room object clears 0.06. The box jumps from the cupboard to the fridge, so the bearing step is 1.072 rad. At x = 0.9 and x = 1.2 every object is under 0.06 (peaks 0.041 and 0.005).

### Bathroom

Toilet wins all five poses (0.367, 0.335, 0.327, 0.399, 0.317). Bearing stays near −0.03 to −0.10. Cupboard, kitchen sink, and fridge also clear 0.06 and vote kitchen. They do not outscore the toilet.

### Living

Couch wins all five poses (0.240, 0.225, 0.245, 0.201, 0.128). Bed, cupboard, and fridge clear 0.06 on some poses and do not win. The bearing stays near −0.3 to −0.4 until x = 1.2, where it steps to −0.045.

### Bedroom

Bed wins at x = 0.0 (0.102) and x = 0.3 (0.094), with couch, sofa, cupboard, and bathtub also at or above 0.06. Those two bearings are +0.022 and +0.044. At x = 0.6 and x = 0.9 nothing clears 0.06. At x = 1.2 the bathtub wins (0.092, bathroom). The bed is still 0.074, so the wrong room is the selection.

### Entrance

Door wins all five poses (0.158, 0.151, 0.130, 0.112, 0.164). Hallway peaks at 0.028, under the cutoff. No wrong-room object clears 0.06. Bearing stays within about ±0.01.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam stays on `head_tilt_link` at `0.050 0.019 0.007`.

The same object map and the one cutoff can attach to a walk after Controls advances the thawed plant. This draft does not do that.

Per-pose scores are in `previews/find_room_detect.json`.
