# Open-vocab room steer (Prefer FAIL)

`scripts/find_room.py` is a separate phrase path from the kitchen slab finder. It does not edit `scripts/find_kitchen.py`. The kitchen Prefer PASS walk on main stays the slab cue.

The model is local OWL-ViT `google/owlvit-base-patch32` at Hub revision `cbc355fb364588351c5d51c7f74465e8e7ec6f72`. No paid API. Objects vote for a room. One score cutoff, **0.06**, applies to every scene. Room-name labels are not the query. A phrase sends half-cap `vel(+0.028, yaw)` only when that room's object is the unique top hit. Up to four slices keep the last good box if the hit drops. `vx = 0` yaw is not sent.

Arrival still needs both bars on the stop frame: cue fraction ≥ 0.50 and torso-to-room ≤ 0.25 m. A box that already covers half the stand frame is not an approach cue.

Object map: fridge, oven, stove, kitchen sink, and cupboard vote kitchen. Toilet, bathtub, and bathroom sink vote bathroom. Bed votes bedroom. Sofa, TV, and couch vote living. Door and hallway vote entrance. Fridge, oven, stove, and the kitchen sink score under the door on the kitchen stand. Cupboard is the kitchen hit that clears 0.06 (0.072 against the door at 0.049). The empty plant peaks at 0.004.

## Kitchen phrase

The stand selects kitchen via cupboard (score 0.072, box fraction 0.066). The hit then drops. Prefer FAIL `lost`. Not arrival.

| | |
|--|--|
| Remaining | **1.000 m** |
| Settled cue | **0** (kitchen not selected on the stop frame) |
| min up_z | **0.994** |
| Arrival | **false** |
| Stop | `lost` (Δx **+0.258 m**) |
| Commands | every one `vx = +0.028`, yaw from −0.134 to 0. `vx = 0` was not sent. |

## Bathroom phrase

The stand selects bathroom via toilet (score 0.329, box fraction 0.016). The walk continues until the torso leans through the up_z bar. Prefer FAIL `tip`. Not arrival.

| | |
|--|--|
| Remaining | **0.756 m** |
| Settled cue | **0** |
| min up_z | **0.877** |
| Arrival | **false** |
| Stop | `tip` (Δx **+1.513 m**, end y −0.414 m) |
| Commands | every one `vx = +0.028`, yaw inside ±0.25. `vx = 0` was not sent. |

## Other rooms, stand only

The same cutoff selects the room's own object on the living room (couch), bedroom (bed), and entrance (door). Those phrases were not walked. The empty plant stays under 0.06.

The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`.
