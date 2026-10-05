# Hallway front test scene

Test scene for Moondream2 / find_room entrance. **Prefer FAIL.**

This is a still of a hallway with a front leaf at the end. It is not an arrival test. It is not a go-anywhere claim. Nothing here is wired to Moondream, and `find_room` labels are unchanged. "Go to the entrance", "hallway", and "front door" stay refused.

## What it is

`mujoco/room_hallway_front.xml` includes the frozen walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`) and adds static geometry. It does not edit the plant. `kit_cam` is still the only camera in the file, still on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. On the quiet stand the camera world height is about **0.376 m**.

The robot stands in the hall looking along +X. The hall is narrow (inner width about 1.44 m) and closed (side walls, a back wall, a wall behind the robot, a ceiling). A wool runner leads to a static leaf in an oak casing. A plant and two pictures sit on the side walls. A sconce and a ceiling lamp are in the hall. Named bodies: `front`, `runner`, `plant`, `frame`, `picture`. `sconce` and `lamp` are present and are not pass bars.

This is not `mujoco/room_entrance.xml`. That scene is a square room: castle-door panel, hessian mat, boots, and a console. This scene does not use that panel, those boots, or that console. The five room XMLs are not edited. `scripts/render_kit_cam_room.py --all` still renders only those five.

The leaf and the casing are visual. There is no hinge, no lever, no latch, and no knob torque. Dave door v1 push/pull is a different plant and is not in this scene.

## Prefer FAIL

- No Moondream score was run. A still is not a recognition result.
- No finder, no map, no arrival bar, no go-to.
- The camera stays at about 0.38 m and looks level. The still is the lower half of a hall: runner, baseboards, plant, the bottom of the leaf. It is not a standing-eye photo of an entry. Moving `kit_cam` is out of scope.
- Poly Haven has no flat painted entry-door mesh. The leaf is the CC0 rough-pine door albedo on a generated slab (0.90 × 2.03 × 0.04 m). It reads as a plank leaf, not a flush painted front door, and not the castle panel in `room_entrance`.
- The casing and the baseboards are oak veneer on boxes.
- The pictures are scaled 1.55 so the near frame clears the 36 px bar. At scan size it was 31 px wide. That scale is a gap against the real frame.
- There is no coat. This set has no CC0 coat mesh.
- The frame glass was not exported. Leaf cards use a black-background cutout from the diffuse JPG, not the original mask. Edges can fringe.
- From this camera the sconce is about 24×34 px and the ceiling lamp about 19×51 px. They are in the side overview. They are not required in the kit_cam pass bar.
- Soft-pass is off. The render script exits non-zero if the plant md5 changes, `kit_cam` moves, the room XML adds a mechanism tag, the still is still the empty checkerboard, or `front`, `runner`, `plant`, `frame`, or `picture` fall under 36 px.

Licenses and source pages are in `docs/ROOM_ASSETS.md`.

## Stills

```bash
MUJOCO_GL=osmesa python scripts/render_kit_cam_hallway.py
```

That writes `previews/kit_cam_hallway_front.png` from `kit_cam` and `previews/hallway_front_side.png` from a free camera inside the hall, behind the robot, looking toward the leaf. The free camera is not a camera in the XML.
