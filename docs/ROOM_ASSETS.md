# Room assets

The five `kit_cam` rooms use open meshes and albedo textures under `mujoco/assets/rooms/`. They replace the flat colored boxes. The walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` is not edited. Room XML only includes it and adds static bodies. Furniture has no joints.

Every downloaded model and texture below is **CC0 1.0**. No paid asset. Poly Haven asks for credit when the API is used to fetch files; the assets themselves need no permission. Pages: [polyhaven.com/license](https://polyhaven.com/license).

## What is in the repo

Poly Haven models were downloaded as glTF 2k, rotated from Y-up to Z-up (`x' = x`, `y' = -z`, `z' = y`), and written as one OBJ per unique albedo with the origin on the floor. MuJoCo 3.14's classic renderer loads PNG and rejects JPEG, and it uses the `<texture type="2d">` albedo on the material. Normal, roughness, and metallic maps are not in the repo. Albedo maps are 1024 px PNG. Unique furniture materials use the baked UVs (`texrepeat` left at 1 1). Floor and wall planes tile.

The entrance panel file is named `panel.obj` so the loaded MuJoCo name does not contain `door`. The source model is a castle door mesh used as a static visual frame: no hinge, no lever, no latch.

`entrance/mat.obj` is a generated thin box (0.62 × 0.40 × 0.012 m) with its own UVs. The weave is the Poly Haven hessian texture, not a second furniture mesh.

## Models (Poly Haven, CC0)

| Repo file | Poly Haven asset | Author | Page |
|-----------|------------------|--------|------|
| `kitchen/upper.obj` | Modern Wooden Cabinet (base cabinets, two copies) | Patrik Pangerl | https://polyhaven.com/a/modern_wooden_cabinet |
| `kitchen/cabinet.obj` | Drawer Cabinet | Ulan Cabanilla | https://polyhaven.com/a/drawer_cabinet |
| `kitchen/stove.obj` | Electric Stove (cooktop and oven) | Kuutti Siitonen | https://polyhaven.com/a/electric_stove |
| `kitchen/kettle.obj` | Vintage Electric Kettle | SV Garip | https://polyhaven.com/a/vintage_electric_kettle |
| `kitchen/microwave.obj` | Vintage Microwave | Adam Nekola | https://polyhaven.com/a/vintage_microwave |
| `kitchen/pot.obj` | Pot Enamel 01 | Kuutti Siitonen | https://polyhaven.com/a/pot_enamel_01 |
| `kitchen/basin.obj` | loafbrr Sink_A basin and faucets | loafbrr | see below |
| `kitchen/table.obj` | Dining Table | Aron Łyczek | https://polyhaven.com/a/dining_table |
| `kitchen/chair.obj` | Dining Chair 02 | James Ray Cock | https://polyhaven.com/a/dining_chair_02 |
| `living/sofa.obj` | Sofa 03 | Fran Calvente | https://polyhaven.com/a/sofa_03 |
| `living/coffee.obj` | Gothic Coffee Table | Ulan Cabanilla | https://polyhaven.com/a/gothic_coffee_table |
| `living/tv.obj` | Television 01 | Gabriel Radić | https://polyhaven.com/a/Television_01 |
| `bedroom/bed.obj` | Gothic Bed 01 | Kirill Sannikov | https://polyhaven.com/a/GothicBed_01 |
| `bedroom/nightstand.obj` | Classic Nightstand 01 | Kirill Sannikov | https://polyhaven.com/a/ClassicNightstand_01 |
| `bedroom/lamp.obj` | Desk Lamp Arm 01 | Kuutti Siitonen (model and texture), Yann Kervran (rigging) | https://polyhaven.com/a/desk_lamp_arm_01 |
| `bathroom/vanity.obj` | Painted Wooden Cabinet | Kirill Sannikov | https://polyhaven.com/a/painted_wooden_cabinet |
| `bathroom/mirror.obj` | Ornate Mirror 01 | James Ray Cock | https://polyhaven.com/a/ornate_mirror_01 |
| `entrance/panel.obj` | Large Castle Door (static frame only) | Tina | https://polyhaven.com/a/large_castle_door |
| `entrance/console.obj` | Classic Console 01 | Kirill Sannikov | https://polyhaven.com/a/ClassicConsole_01 |

Each OBJ has a matching PNG albedo next to it.

## Textures (Poly Haven, CC0)

| Repo file | Poly Haven asset | Author | Page |
|-----------|------------------|--------|------|
| `textures/wood_floor.png` | Wood Floor | Dimitrios Savva | https://polyhaven.com/a/wood_floor |
| `textures/beige_wall.png` | Beige Wall 001 | Dimitrios Savva (photography), Rico Cilliers (processing) | https://polyhaven.com/a/beige_wall_001 |
| `textures/interior_tiles.png` | Interior Tiles | Charlotte Baglioni | https://polyhaven.com/a/interior_tiles |
| `textures/white_plaster.png` | White Plaster 02 | Rob Tuytel | https://polyhaven.com/a/white_plaster_02 |
| `textures/hessian_mat.png` | Hessian 380 | colormass (photography), Rico Cilliers (processing) | https://polyhaven.com/a/hessian_380 |
| `textures/marble.png` | Marble 01 | Rob Tuytel | https://polyhaven.com/a/marble_01 |

## Bathroom fixtures (loafbrr, CC0)

Poly Haven has no toilet or sink mesh. The sanitary pieces are from loafbrr's **Toilets** pack, licensed **CC0**:

- https://opengameart.org/content/toilets
- https://loafbrr.itch.io/

Attribution is not required. The pack README asks for credit anyway: loafbrr.

| Repo file | Pack mesh | Notes |
|-----------|-----------|--------|
| `bathroom/toilet_bowl.obj` | Toilet_Round_A | Shared frame with the other toilet parts. Assembled height about 0.74 m |
| `bathroom/toilet_tank.obj` | Toilet_Round_A flush box | Same frame as the bowl, so the cistern sits behind the bowl |
| `bathroom/toilet_seat.obj` | seat and cover | Same frame. Lies on the bowl |
| `bathroom/toilet_handle.obj` | flusher | Same frame, flush-box albedo. The pack has no separate flusher atlas |
| `bathroom/sink.obj` | Sink_A plus both faucets | Pedestal basin on the vanity. Replaces the earlier Sink_C trough, which read as a wooden block from `kit_cam` |
| `bathroom/bathtub.obj` | bath | Stylized tub, about 1.08 × 0.72 × 0.58 m. See below |
| `bathroom/bathtub_water.obj` | bath_water | Water surface in the same frame as the shell |
| `entrance/boots.obj` | Rubber Boots | Pair of wellies beside the mat. Poly Haven, not the bathroom pack |

The four toilet files are one round toilet split by material. They were re-exported from the pack glTF in a shared Z-up frame (Y-up to Z-up, then one floor-center for the whole fixture). An earlier export centered each part on its own, so the tank lay flat and the seat stood on edge. The sink file is Sink_A from the same CC0 pack, not a new download and not a paid model.

The glTF did not embed images. UVs were kept and the pack's diffuse PNGs were assigned. These are textured game meshes, not photogrammetry. They are still shaped fixtures, not flat rgba boxes.

Poly Haven has no kitchen sink. `kitchen/basin.obj` is the same CC0 Sink_A mesh, with the pedestal faces removed so the bowl and faucets sit on the counter. It is a bathroom basin, not an undermount kitchen sink. That style gap is a Prefer FAIL for a photoreal kitchen sink. The faucet is what makes the counter read as a sink from `kit_cam`.

The drawer cabinet file is still in the repo. The kitchen scene does not place it. From `kit_cam` it read as a wire baker's rack, not as base cabinets. The run is two copies of the modern wooden cabinet, the basin, and the electric stove.

Poly Haven has no bathtub. The tub is from Isa Lousberg's **Tiny Treats — Bubbly Bathroom** set, licensed **CC0**:

- https://opengameart.org/content/tiny-treats-bubbly-bathroom-set

Only the bath shell and the water surface are in the repo, with the set's gradient atlas. It is stylized low-poly game art, not a scan. That style gap against the Poly Haven vanity is still a Prefer FAIL for photoreal kit_cam, and it is the CC0 mesh that reads as a tub.

## Entrance boots (Poly Haven, CC0)

| Repo file | Poly Haven asset | Author | Page |
|-----------|------------------|--------|------|
| `entrance/boots.obj` | Rubber Boots | L | https://polyhaven.com/a/rubber_boots |

There is no CC0 coat in this set. The shoes cue is the boots, and the hall table is Classic Console 01. Poly Haven has no flat interior door; the leaf is still the castle-door mesh, scaled down and set in static wood trim. Still no hinge, no lever, and no latch.

## Scene names

Body and site names stay `kitchen`, `table`, `chair`, `bathroom`, `sink`, `toilet`, `living`, `tv`, `bedroom`, `nightstand`, `entrance`, `mat`. Later passes add `bathtub`, `shoes`, `coffee`, and `console`. Each room adds a floor texture, side walls, a back wall, and a ceiling as extra worldbody geoms, using the CC0 textures above. The plant's checker plane is still in the included file. Furniture stays static. The entrance panel is still a visual frame: no hinge, no lever, no latch.
