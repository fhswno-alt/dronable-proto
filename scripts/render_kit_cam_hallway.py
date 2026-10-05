#!/usr/bin/env python3
"""Stand the frozen plant in the hallway-front test scene and save stills.

This is not one of the five kit_cam rooms. It does not edit the plant,
the gait, CommandBus, or find_room labels. It does not add a second
camera to the XML. The side still is a free-camera overview for a person
reading the scene. Prefer FAIL: the still is not a Moondream score, not
an arrival, and not a go-anywhere claim.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import render_kit_cam_room as rooms  # noqa: E402
import steer_walk  # noqa: E402

ROOT = rooms.ROOT
SCENE = rooms.RoomScene(
    name="hallway_front",
    xml_name="room_hallway_front.xml",
    png_name="kit_cam_hallway_front.png",
    bodies=("front", "runner", "plant", "frame", "picture"),
    geoms=(
        ("front", "front_leaf"),
        ("runner", "runner_rug"),
        ("plant", "plant_pot"),
        ("frame", "frame_art"),
        ("picture", "picture_art"),
    ),
)
SIDE_PNG = ROOT / "previews" / "hallway_front_side.png"
# Printed, not required. A small sconce can miss the 36 px bar and still
# be a real mesh in the hall. The required bodies are the ones above.
EXTRA_BODIES = ("sconce", "lamp")


def _side_view(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    """Free camera from the side. Not a camera in the XML."""
    camera = mj.MjvCamera()
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    # Inside the hall, behind the robot, looking toward the leaf.
    camera.lookat[:] = np.array([2.0, 0.0, 0.70], dtype=np.float64)
    camera.distance = 2.6
    camera.azimuth = 170.0
    camera.elevation = -24.0
    renderer = mj.Renderer(model, height=540, width=960)
    try:
        renderer.update_scene(data, camera=camera)
        frame = renderer.render()
    finally:
        renderer.close()
    image = np.asarray(frame, dtype=np.uint8)
    if image.shape != (540, 960, 3):
        raise SystemExit(f"FAIL: side frame shape {image.shape}")
    return image


def main() -> int:
    if not steer_walk.PLANT_XML.is_file():
        print(f"FAIL: missing {steer_walk.PLANT_XML}")
        return 1
    before = rooms._md5(steer_walk.PLANT_XML)
    if before != steer_walk.PLANT_MD5:
        print(f"FAIL: plant md5 {before} != {steer_walk.PLANT_MD5}")
        return 1

    plant = mj.MjModel.from_xml_path(str(steer_walk.PLANT_XML))
    targets, com_z = rooms._stand_targets()
    plant_data = mj.MjData(plant)
    rooms._hold_stand(plant, plant_data, targets, com_z)
    empty_img = rooms._render(plant, plant_data)

    status = rooms._render_scene(SCENE, plant, empty_img, before, targets, com_z)
    if rooms._md5(steer_walk.PLANT_XML) != before:
        print("FAIL: plant file changed while rendering the hallway")
        return 1
    if status != 0:
        return status

    room = mj.MjModel.from_xml_path(str(SCENE.xml_path))
    room_data = mj.MjData(room)
    rooms._hold_stand(room, room_data, targets, com_z)
    for body_name in EXTRA_BODIES:
        box_w, box_h = rooms._body_box_pixels(room, room_data, body_name)
        print(f"  {body_name} in frame {box_w:.0f}x{box_h:.0f} px (not a pass bar)")
    side = _side_view(room, room_data)
    rooms._save_png(side, SIDE_PNG)
    print(f"side overview out={SIDE_PNG.relative_to(ROOT)}")
    if rooms._md5(steer_walk.PLANT_XML) != before:
        print("FAIL: plant file changed while rendering the hallway")
        return 1
    print(
        "hallway_front stills only. Prefer FAIL for Moondream2 / find_room "
        "entrance: not scored, not an arrival, not go-anywhere."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
