#!/usr/bin/env python3
"""Final Moondream2 score on stills that were not used to pick the model.

Selection stills stay in previews/held_out_thor and previews/held_out_places.
This script does not read them. Fresh iTHOR plans are FloorPlan6-10 and the
matching living, bedroom, and bathroom blocks. Fresh Places ids are the next
five validation files per class after the selection files.

Kit_cam entrance is not rendered. Hardware has not supplied a hallway.
The empty plant is the empty frame. No gait and no vel. The object map in
find_room.py is not edited.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import find_room as fr
import find_room_thor as thor
import room_ask

ROOT = fr.ROOT
FRESH_DIR = fr.PREVIEWS / "moondream_fresh"
SUMMARY = fr.PREVIEWS / "find_room_moondream.json"

# Picked before any fresh frame was shown to the model.
FRESH_ITHOR = {
    "kitchen": [f"FloorPlan{i}" for i in range(6, 11)],
    "living": [f"FloorPlan{i}" for i in range(206, 211)],
    "bedroom": [f"FloorPlan{i}" for i in range(306, 311)],
    "bathroom": [f"FloorPlan{i}" for i in range(406, 411)],
}
SELECTION_ITHOR = {scene for scenes in thor.SCENES.values() for scene in scenes}
FRESH_PLACES = {
    "kitchen": [
        "Places365_val_00001691.jpg",
        "Places365_val_00001940.jpg",
        "Places365_val_00002112.jpg",
        "Places365_val_00002295.jpg",
        "Places365_val_00002312.jpg",
    ],
    "bathroom": [
        "Places365_val_00002198.jpg",
        "Places365_val_00002239.jpg",
        "Places365_val_00002398.jpg",
        "Places365_val_00002575.jpg",
        "Places365_val_00002580.jpg",
    ],
    "bedroom": [
        "Places365_val_00001822.jpg",
        "Places365_val_00003614.jpg",
        "Places365_val_00003929.jpg",
        "Places365_val_00004018.jpg",
        "Places365_val_00004922.jpg",
    ],
    "living": [
        "Places365_val_00005291.jpg",
        "Places365_val_00005689.jpg",
        "Places365_val_00005899.jpg",
        "Places365_val_00005909.jpg",
        "Places365_val_00006199.jpg",
    ],
    "entrance_hall": [
        "Places365_val_00001503.jpg",
        "Places365_val_00001730.jpg",
        "Places365_val_00002003.jpg",
        "Places365_val_00002170.jpg",
        "Places365_val_00002424.jpg",
    ],
    "corridor": [
        "Places365_val_00002272.jpg",
        "Places365_val_00002774.jpg",
        "Places365_val_00003168.jpg",
        "Places365_val_00004729.jpg",
        "Places365_val_00004785.jpg",
    ],
}
SELECTION_PLACES = {
    "Places365_val_00000266.jpg",
    "Places365_val_00000390.jpg",
    "Places365_val_00000861.jpg",
    "Places365_val_00001370.jpg",
    "Places365_val_00001600.jpg",
    "Places365_val_00000172.jpg",
    "Places365_val_00000463.jpg",
    "Places365_val_00000885.jpg",
    "Places365_val_00001356.jpg",
    "Places365_val_00001738.jpg",
    "Places365_val_00000244.jpg",
    "Places365_val_00000320.jpg",
    "Places365_val_00000713.jpg",
    "Places365_val_00001436.jpg",
    "Places365_val_00001617.jpg",
    "Places365_val_00000785.jpg",
    "Places365_val_00001876.jpg",
    "Places365_val_00003477.jpg",
    "Places365_val_00004801.jpg",
    "Places365_val_00005205.jpg",
    "Places365_val_00000075.jpg",
    "Places365_val_00000422.jpg",
    "Places365_val_00001024.jpg",
    "Places365_val_00001111.jpg",
    "Places365_val_00001192.jpg",
    "Places365_val_00000006.jpg",
    "Places365_val_00001078.jpg",
    "Places365_val_00001510.jpg",
    "Places365_val_00001709.jpg",
    "Places365_val_00002244.jpg",
}
PLACES_EXPECTED = {
    "kitchen": "kitchen",
    "bathroom": "bathroom",
    "bedroom": "bedroom",
    "living": "living",
    "entrance_hall": "entrance",
    "corridor": "entrance",
}
# A corridor may be named entrance or left unknown. A furnished room name is wrong.
CORRIDOR_OK = {None, "none", "entrance"}
KIT_SCENES = ("kitchen", "bathroom", "living", "bedroom", "plant")


def _guard() -> None:
    if fr.SCORE_MIN != 0.06 or fr.VOTE_MARGIN != 0.15:
        raise RuntimeError("owl cutoff or margin moved")
    if fr._md5(fr.PLANT_XML) != "71b2c86d133ebc603f58b99c53e496f3":
        raise RuntimeError("plant md5 changed")
    fresh_plans = {scene for scenes in FRESH_ITHOR.values() for scene in scenes}
    if fresh_plans & SELECTION_ITHOR:
        raise RuntimeError("fresh iTHOR overlaps selection")
    fresh_files = {name for names in FRESH_PLACES.values() for name in names}
    if fresh_files & SELECTION_PLACES:
        raise RuntimeError("fresh Places overlaps selection")
    if "entrance" in KIT_SCENES:
        raise RuntimeError("kit_cam entrance is not a scored hallway")


def _cpu() -> str:
    name = "unknown"
    count = 0
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("model name") and name == "unknown":
            name = line.split(":", 1)[1].strip()
        if line.startswith("processor"):
            count += 1
    return f"{count}x {name}"


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    weight = index - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _ithor_rows() -> list[dict[str, str]]:
    rows = []
    out = FRESH_DIR / "ithor"
    for room, scenes in FRESH_ITHOR.items():
        for scene in scenes:
            path = out / f"{room}_{scene}.png"
            rows.append((path, {
                "source": "ithor_fresh",
                "group": room,
                "expected": room,
                "scene": scene,
                "file": str(path.relative_to(ROOT)),
            }))
    return rows


def _render_ithor() -> list[dict[str, str]]:
    planned = _ithor_rows()
    if all(path.is_file() for path, _row in planned):
        print("[moon] fresh iTHOR stills already on disk", flush=True)
        return [row for _path, row in planned]
    from PIL import Image
    from ai2thor.controller import Controller

    out = FRESH_DIR / "ithor"
    out.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, str]] = []
    controller = Controller(
        platform="Linux64",
        scene="FloorPlan6",
        quality="Low",
        width=thor.WIDTH,
        height=thor.HEIGHT,
        fieldOfView=thor.FOV_DEG,
        server_timeout=90,
        server_start_timeout=180,
    )
    try:
        for room, scenes in FRESH_ITHOR.items():
            for scene in scenes:
                event = controller.reset(scene=scene, fieldOfView=thor.FOV_DEG)
                spot, yaw = thor._aim(controller, event)
                shot = controller.step(
                    action="AddThirdPartyCamera",
                    position={"x": float(spot["x"]), "y": thor.CAM_Y_M, "z": float(spot["z"])},
                    rotation={"x": 0.0, "y": yaw, "z": 0.0},
                    fieldOfView=thor.FOV_DEG,
                )
                if not shot.metadata.get("lastActionSuccess") or not shot.third_party_camera_frames:
                    raise RuntimeError(f"{scene} camera failed: {shot.metadata.get('errorMessage')}")
                path = out / f"{room}_{scene}.png"
                Image.fromarray(shot.third_party_camera_frames[-1]).save(path)
                rows.append({
                    "source": "ithor_fresh",
                    "group": room,
                    "expected": room,
                    "scene": scene,
                    "file": str(path.relative_to(ROOT)),
                })
                print(f"[moon] rendered {scene}", flush=True)
    finally:
        controller.stop()
    return rows


def _extract_places() -> list[dict[str, str]]:
    import tarfile

    out = FRESH_DIR / "places"
    out.mkdir(parents=True, exist_ok=True)
    wanted = {name: group for group, names in FRESH_PLACES.items() for name in names}
    tar_path = Path("/tmp/places_val_256.tar")
    missing = [name for name in wanted if not (out / f"{wanted[name]}_{name}").is_file()]
    if missing:
        if not tar_path.is_file():
            raise RuntimeError("Places val_256 tar is not at /tmp/places_val_256.tar")
        with tarfile.open(tar_path) as archive:
            for member in archive:
                base = Path(member.name).name
                if base not in wanted:
                    continue
                dest = out / f"{wanted[base]}_{base}"
                if dest.is_file():
                    continue
                extracted = archive.extractfile(member)
                if extracted is None:
                    continue
                dest.write_bytes(extracted.read())
    rows = []
    for group, names in FRESH_PLACES.items():
        for name in names:
            path = out / f"{group}_{name}"
            if not path.is_file():
                raise RuntimeError(f"missing fresh photo {path}")
            rows.append({
                "source": "places_fresh",
                "group": group,
                "expected": PLACES_EXPECTED[group],
                "scene": name,
                "file": str(path.relative_to(ROOT)),
            })
    return rows


def _render_kit() -> list[dict[str, str]]:
    from PIL import Image

    out = FRESH_DIR / "kit"
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for scene in KIT_SCENES:
        xml = fr.SCENE_XML[scene]
        session = fr.steer_walk.SteerSession(video=False, scene_xml=xml)
        cam = fr.fk.KitCam(session.model)
        try:
            fr._set_scripted_pose(session, 0.0, 0.0)
            frame = cam.grab(session.data)
        finally:
            cam.close()
        path = out / f"{scene}.png"
        Image.fromarray(frame).save(path)
        expected = "none" if scene == "plant" else scene
        rows.append({
            "source": "kit_cam",
            "group": scene,
            "expected": expected,
            "scene": scene,
            "file": str(path.relative_to(ROOT)),
        })
        print(f"[moon] kit still {scene}", flush=True)
    return rows


def _judge(row: dict[str, object]) -> None:
    named = row["room"]
    expected = str(row["expected"])
    group = str(row["group"])
    if group == "corridor":
        row["hit"] = named == "entrance"
        row["ok_unknown"] = named in (None, "none")
        row["wrong"] = named not in CORRIDOR_OK
        return
    if expected == "none":
        row["hit"] = False
        row["ok_unknown"] = named in (None, "none")
        row["wrong"] = named not in (None, "none")
        return
    row["hit"] = named == expected
    row["ok_unknown"] = named in (None, "none")
    row["wrong"] = named not in (None, expected, "none")


def _summarize(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    order: list[tuple[str, str]] = []
    for row in rows:
        key = (str(row["source"]), str(row["group"]))
        if key not in order:
            order.append(key)
    classes = []
    for source, group in order:
        group_rows = [row for row in rows if row["source"] == source and row["group"] == group]
        classes.append({
            "source": source,
            "group": group,
            "scenes": len(group_rows),
            "hits": sum(1 for row in group_rows if row["hit"]),
            "unknown": sum(1 for row in group_rows if row["ok_unknown"]),
            "wrong": sum(1 for row in group_rows if row["wrong"]),
        })
    return classes


def score() -> dict[str, object]:
    _guard()
    import numpy as np
    from PIL import Image

    specs = _render_ithor() + _extract_places() + _render_kit()
    asker = room_ask.RoomAsk()
    rows: list[dict[str, object]] = []
    try:
        blank = np.zeros((64, 64, 3), dtype=np.uint8)
        smoke = asker.ask(blank)
        if "room" not in smoke or "raw" not in smoke:
            raise RuntimeError(f"smoke ask missing fields {smoke}")
        print(f"[moon] smoke blank room={smoke['room']} raw={smoke['raw']!r}", flush=True)
        for spec in specs:
            frame = np.asarray(Image.open(ROOT / spec["file"]).convert("RGB"))
            answer = asker.ask(frame)
            row = {**spec, **answer}
            _judge(row)
            rows.append(row)
            print(
                f"[moon] {spec['source']:12} {spec['group']:14} {spec['scene']:28} "
                f"room {answer['room'] or 'undecided':9} {answer['seconds']:.2f}s "
                f"raw {answer['raw']!r}",
                flush=True,
            )
    finally:
        asker.close()
    seconds = [float(row["seconds"]) for row in rows]
    return {
        "status": "scored",
        "model": room_ask.MODEL_ID,
        "model_rev": room_ask.MODEL_REV,
        "model_sha": room_ask.MODEL_SHA,
        "weights_sha256": room_ask.WEIGHTS_SHA256,
        "tag_rev": room_ask.TAG_REV,
        "tag_sha": room_ask.TAG_SHA,
        "tag_load_error": "AttributeError: 'HfMoondream' object has no attribute 'all_tied_weights_keys'",
        "licence": "apache-2.0",
        "licence_card": "license: apache-2.0",
        "prompt": room_ask.PROMPT,
        "selection_sha": room_ask.SELECTION_SHA,
        "selection_note": "FloorPlan1-5 / 201-205 / 301-305 / 401-405 and the first five Places files per class picked the model. They are not in this score.",
        "kit_cam_entrance": "untested until Hardware supplies a hallway",
        "cpu": _cpu(),
        "cpu_note": "Agent VM CPU. Not Dave's M4 laptop.",
        "seconds_p50": _percentile(seconds, 0.50),
        "seconds_p95": _percentile(seconds, 0.95),
        "seconds_n": len(seconds),
        "gait": False,
        "vel": False,
        "arrival": False,
        "owl_map_changed": False,
        "plant_md5": fr._md5(fr.PLANT_XML),
        "classes": _summarize(rows),
        "scenes": rows,
    }


def _print(payload: dict[str, object]) -> None:
    print(
        f"[moon] {payload['model']} @ {payload['model_rev']} {payload['model_sha']} "
        f"licence {payload['licence']} p50 {payload['seconds_p50']:.2f}s "
        f"p95 {payload['seconds_p95']:.2f}s n {payload['seconds_n']} "
        f"cpu {payload['cpu']}"
    )
    print("| Source | Scene | Named hit | Unknown | Wrong room |")
    print("|--|--|--|--|--|")
    for row in payload["classes"]:
        n = row["scenes"]
        print(
            f"| {row['source']} | {row['group']} | {row['hits']}/{n} | "
            f"{row['unknown']}/{n} | {row['wrong']}/{n} |"
        )


def self_test() -> int:
    _guard()
    body = Path(__file__).read_text().split("def self_test", 1)[0]
    failures = []
    if "bus.vel" in body or "session.step" in body:
        failures.append("gait or vel")
    if room_ask.self_test() != 0:
        failures.append("parse")
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[moon] self-test PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    started = time.perf_counter()
    payload = score()
    payload["wall_seconds"] = time.perf_counter() - started
    SUMMARY.write_text(json.dumps(payload, indent=2) + "\n")
    if fr.ARTIFACTS.is_dir():
        (fr.ARTIFACTS / SUMMARY.name).write_text(SUMMARY.read_text())
    _print(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
