# Held-out room scores (Prefer FAIL blocked)

`scripts/find_room_held_out.py` scores images with the frozen recogniser in `scripts/find_room.py`. It does not edit the object map. The cutoff stays **0.06**. The margin stays **0.15**. The bonus stays **0.03**, capped at **0.06**. The vote is still one frame's best object plus that capped bonus. It does not step the gait and it does not publish vel. It does not claim arrival.

This run did not score a room. There is no held-out image to score.

## Assets in this tree

| Asset | Count | Held-out |
|--|--|--|
| Tuned room XML (`room_kitchen`, `room_bathroom`, `room_living`, `room_bedroom`, `room_entrance`) | 5 | no |
| Stills of those scenes and the empty plant (`kit_cam_room*`, `find_room_*`, `find_kitchen_*`, `explore_map_*`) | 41 | no |
| Walk and voice stills, plus the assembly render | 15 | no |
| Furniture albedo PNGs under `mujoco/assets/rooms` | 40 | no |
| Robot textures under `mujoco/ainex_hiwonder` | 14 | no |
| OBJ meshes not referenced by the five room XMLs | 9 | no |
| Images under `previews/held_out/` | 0 | — |
| JPEG or WebP photographs | 0 | missing |

The nine unplaced meshes are `bedroom/bed.obj` (Gothic Bed 01), `bedroom/nightstand.obj` (Classic Nightstand 01), and the kitchen cabinet, upper, table, chair, kettle, microwave, and pot. They are files, not a room. Composing them into a new XML would be a new labelled layout, so they were not scored.

`cursor/photoreal-room-assets-bdcd` (`0cdaf58`) is an earlier furniture pass of these same five shells. It is not a sixth home. A small yaw or x nudge of the tuned scenes would be the same rooms again. Neither was scored.

## Gap

Dave or Hardware needs to supply one of these before a held-out number exists:

- An alternate room layout that is not `room_kitchen.xml`, `room_bathroom.xml`, `room_living.xml`, `room_bedroom.xml`, or `room_entrance.xml`, and not a small pose change of those meshes.
- Real-home photographs taken near kit_cam height, about 0.38 m, looking level. Put the files in `previews/held_out/`. An optional `manifest.json` may set `"room"` per file. A file with no room stays undecided. An empty frame stays undecided.

```bash
python3 scripts/find_room_held_out.py
```

The inventory and the blocker string are in `previews/find_room_held_out.json`.

## Not a walk

Gait is off. Vel is off. Arrival is not claimed. The plant file is not in the diff. md5 stays `71b2c86d133ebc603f58b99c53e496f3`. Kit_cam stays on `head_tilt_link` at `0.050 0.019 0.007`.
