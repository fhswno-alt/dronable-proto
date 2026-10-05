# Open-vocab room steer (Prefer FAIL)

`scripts/find_room.py` is a separate phrase path from the kitchen slab finder. It does not edit `scripts/find_kitchen.py`. The kitchen Prefer PASS walk on main stays the slab cue.

The model is local OWL-ViT `google/owlvit-base-patch32` at Hub revision `cbc355fb364588351c5d51c7f74465e8e7ec6f72`. No paid API. The same five text labels and the library score cutoff 0.1 are used on every scene. A phrase sends vel only when that room is the unique top label. The command is half-cap `vel(+0.028, yaw)` inside yaw ±0.25. `vx = 0` yaw is not sent.

Arrival still needs both bars on the stop frame: cue fraction ≥ 0.50 and torso-to-room ≤ 0.25 m. A box that already covers half the stand frame is not an approach cue, so that stop is not arrival.

## Stand labels

| Scene | Top label | Kitchen score | Bathroom score |
|--|--|--|--|
| Kitchen | living room 0.356 | 0.095 | 0.099 |
| Bathroom | bathroom 0.308 | 0.092 | 0.308 |
| Living | living room 0.404 | 0.041 | 0.050 |
| Bedroom | living room 0.373 | 0.048 | 0.084 |
| Entrance | living room 0.411 | 0.070 | 0.101 |
| Empty plant | none (all under 0.1) | 0.001 | 0.003 |

"A kitchen" is not selected on the kitchen stand. "A living room" is the top label on the kitchen, living room, bedroom, and entrance, so that phrase is not a room-specific cue and was not walked. "A bathroom" is the top label only on the bathroom stand.

## Kitchen phrase

Prefer FAIL `no_cue`. The model selected "a living room" (score 0.356, box fraction 0.987). No vel.

| | |
|--|--|
| Remaining | **1.209 m** |
| Settled kitchen cue | **not selected** (living-room box 0.985) |
| min up_z | **0.998** |
| Arrival | **false** |
| Stop | `no_cue` |

## Bathroom phrase

The stand box covers the frame (fraction 0.990), so the cue bar is already met from across the room and does not count as arrival. The walk then lost the bathroom label. Prefer FAIL `lost`. Not arrival.

| | |
|--|--|
| Remaining | **0.818 m** |
| Settled cue | **0.981** (bathroom box; stand was already 0.990) |
| min up_z | **0.939** |
| Arrival | **false** |
| Stop | `lost` (Δx **+1.131 m**, end y −0.265 m, yaw −1.100 rad) |
| Commands | every one `vx = +0.028`, yaw from 0 to +0.121. `vx = 0` was not sent. |

0.818 m misses the 0.25 m gap. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`.
