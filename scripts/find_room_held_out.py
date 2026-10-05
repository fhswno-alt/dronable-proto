#!/usr/bin/env python3
"""Held-out scores for the frozen kit_cam room recogniser.

This script does not edit the object map, the cutoff, the margin, or the
bonus. It does not load a gait and it does not publish vel. It does not
edit the plant. The five tuned room XMLs are not a held-out set.

Drop new images under previews/held_out/. An optional manifest.json in that
folder may name each file's room. A file with no room stays undecided.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import find_room as fr

ROOT = fr.ROOT
PREVIEWS = fr.PREVIEWS
HELD_OUT_DIR = PREVIEWS / "held_out"
SUMMARY = PREVIEWS / "find_room_held_out.json"
ARTIFACTS = fr.ARTIFACTS
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".webp"}
TUNED_XML = {
    "room_kitchen.xml": "kitchen",
    "room_bathroom.xml": "bathroom",
    "room_living.xml": "living",
    "room_bedroom.xml": "bedroom",
    "room_entrance.xml": "entrance",
}
# Stills rendered from the tuned scenes or from the empty plant. Not held-out.
TUNED_STILL_PREFIXES = (
    "kit_cam_room",
    "find_room_",
    "find_kitchen_",
    "explore_map_",
)


def _frozen() -> None:
    if fr.SCORE_MIN != 0.06 or fr.VOTE_MARGIN != 0.15:
        raise RuntimeError("cutoff or margin moved")
    if fr.OBJECT_BONUS != 0.03 or fr.OBJECT_BONUS_CAP != 0.06:
        raise RuntimeError("bonus or cap moved")
    if fr.VOTE_FRAMES != 3:
        raise RuntimeError("vote window moved")


def _plant_md5() -> str:
    digest = fr._md5(fr.PLANT_XML)
    if digest != fr.steer_walk.PLANT_MD5:
        raise RuntimeError("plant md5 changed")
    return digest


def _mesh_files(xml: Path) -> set[str]:
    text = xml.read_text()
    names: set[str] = set()
    for token in text.replace('"', " ").replace("'", " ").split():
        if token.endswith(".obj"):
            names.add(Path(token).name)
    return names


def _tuned_meshes() -> set[str]:
    used: set[str] = set()
    for name in TUNED_XML:
        used |= _mesh_files(ROOT / "mujoco" / name)
    return used


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def inventory() -> dict[str, object]:
    """Classify what is already in the repo. Does not score it."""
    tuned_xml = []
    for name, room in TUNED_XML.items():
        path = ROOT / "mujoco" / name
        tuned_xml.append({"file": _rel(path), "room": room, "exists": path.is_file()})
    used = _tuned_meshes()
    unplaced = []
    for path in sorted((ROOT / "mujoco" / "assets" / "rooms").rglob("*.obj")):
        if path.name not in used:
            unplaced.append(_rel(path))
    tuned_stills = []
    other_images = []
    texture_images = []
    robot_images = []
    held_out = []
    rooms = ROOT / "mujoco" / "assets" / "rooms"
    robot = ROOT / "mujoco" / "ainex_hiwonder"
    for folder in (PREVIEWS, ROOT / "docs", rooms, robot):
        if not folder.is_dir():
            continue
        for path in sorted(folder.rglob("*")):
            if not path.is_file() or path.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            rel = _rel(path)
            if HELD_OUT_DIR in path.parents or path.parent == HELD_OUT_DIR:
                held_out.append(rel)
                continue
            if rooms in path.parents or path.parent == rooms:
                texture_images.append(rel)
                continue
            if robot in path.parents or path.parent == robot:
                robot_images.append(rel)
                continue
            if path.parent == PREVIEWS and path.name.startswith(TUNED_STILL_PREFIXES):
                tuned_stills.append(rel)
                continue
            other_images.append(rel)
    return {
        "tuned_room_xml": tuned_xml,
        "tuned_stills": tuned_stills,
        "other_images": other_images,
        "texture_images": texture_images,
        "robot_images": robot_images,
        "unplaced_meshes": unplaced,
        "held_out_images": held_out,
        "real_home_photos": [
            rel for rel in held_out if Path(rel).suffix.lower() in {".jpg", ".jpeg", ".webp"}
        ],
    }


def _manifest_rooms() -> dict[str, str | None]:
    path = HELD_OUT_DIR / "manifest.json"
    if not path.is_file():
        return {}
    payload = json.loads(path.read_text())
    rooms: dict[str, str | None] = {}
    for row in payload.get("images", []):
        rooms[str(row["file"])] = row.get("room")
    return rooms


def _table_row(vote: fr.Vote, expected: str | None) -> dict[str, object]:
    """Unknown truth stays undecided. A model commit is not a hit."""
    if expected is None:
        return {
            "expected": None,
            "commit": None,
            "hit": False,
            "false_commit": None,
            "model_room": vote.room,
            "lead": vote.gap,
            "undecided": True,
        }
    false_commit = vote.room if vote.room not in (None, expected) else None
    return {
        "expected": expected,
        "commit": vote.room,
        "hit": vote.room == expected,
        "false_commit": false_commit,
        "model_room": vote.room,
        "lead": vote.gap,
        "undecided": vote.room is None,
    }


def _load_rgb(path: Path):
    import numpy as np
    from PIL import Image

    image = Image.open(path).convert("RGB")
    return np.asarray(image)


def score_held_out(detector: fr.RoomDetector | None = None) -> dict[str, object]:
    """Score only images under previews/held_out. One frame, frozen vote."""
    _frozen()
    found = inventory()
    images = [ROOT / rel for rel in found["held_out_images"]]
    rooms = _manifest_rooms()
    rows = []
    if images:
        if detector is None:
            detector = fr.RoomDetector()
        for path in images:
            seen = detector.read(_load_rgb(path))
            vote = fr.decide_window([fr._frame(seen.object_scores or {}, seen.object_bias or {})])
            expected = rooms.get(path.name)
            row = _table_row(vote, expected)
            row["file"] = _rel(path)
            row["raw_object"] = seen.object_name
            row["raw_score"] = seen.score
            rows.append(row)
    blocked = not rows
    payload = {
        "status": "blocked" if blocked else "scored",
        "model": fr.MODEL_ID,
        "model_rev": fr.MODEL_REV,
        "score_min": fr.SCORE_MIN,
        "vote_margin": fr.VOTE_MARGIN,
        "object_bonus": fr.OBJECT_BONUS,
        "object_bonus_cap": fr.OBJECT_BONUS_CAP,
        "vote_frames": fr.VOTE_FRAMES,
        "gait": False,
        "vel": False,
        "arrival": False,
        "plant_md5": _plant_md5(),
        "plant_in_diff": False,
        "labels_changed": False,
        "held_out_dir": _rel(HELD_OUT_DIR),
        "scenes": rows,
        "inventory": found,
        "blocker": None
        if not blocked
        else (
            "No held-out room image. The five kit_cam XMLs and their stills "
            "are the tuned set. Unplaced OBJ files are not layouts. "
            "Supply alternate furniture layouts that are not those five scenes, "
            "or real-home photos at about 0.38 m looking level, under "
            "previews/held_out/."
        ),
    }
    return payload


def _write(payload: dict[str, object]) -> None:
    SUMMARY.parent.mkdir(parents=True, exist_ok=True)
    SUMMARY.write_text(json.dumps(payload, indent=2) + "\n")
    if ARTIFACTS.is_dir():
        (ARTIFACTS / SUMMARY.name).write_text(SUMMARY.read_text())


def _print(payload: dict[str, object]) -> None:
    inv = payload["inventory"]
    print(
        f"[held-out] status {payload['status']} cutoff {payload['score_min']} "
        f"margin {payload['vote_margin']} plant {payload['plant_md5']} "
        f"gait {payload['gait']}"
    )
    print(
        f"[held-out] tuned_xml {len(inv['tuned_room_xml'])} "
        f"tuned_stills {len(inv['tuned_stills'])} "
        f"other_images {len(inv['other_images'])} "
        f"textures {len(inv['texture_images'])} "
        f"robot_images {len(inv['robot_images'])} "
        f"unplaced_meshes {len(inv['unplaced_meshes'])} "
        f"held_out_images {len(inv['held_out_images'])}"
    )
    if payload["status"] == "blocked":
        print(f"[held-out] Prefer FAIL blocked: {payload['blocker']}")
        return
    print("| Scene | Hits | Commit | Lead | Committed false |")
    print("|--|--|--|--|--|")
    for row in payload["scenes"]:
        lead = row["lead"]
        lead_s = "—" if lead is None else f"{float(lead):.3f}"
        commit = row["commit"] if row["commit"] else "undecided"
        false_commit = row["false_commit"] if row["false_commit"] else "none"
        hit = "1/1" if row["hit"] else "0/1"
        print(f"| {row['file']} | {hit} | {commit} | {lead_s} | {false_commit} |")


def self_test() -> int:
    failures: list[str] = []

    def expect(cond: bool, message: str) -> None:
        if not cond:
            failures.append(message)

    _frozen()
    expect(_plant_md5() == "71b2c86d133ebc603f58b99c53e496f3", "plant md5")
    body = Path(__file__).read_text().split("def self_test", 1)[0]
    expect("bus.vel" not in body and ".step(" not in body, "gait or vel call")
    found = inventory()
    tuned_names = {Path(row["file"]).name for row in found["tuned_room_xml"]}
    expect(tuned_names == set(TUNED_XML), f"tuned xml set {tuned_names}")
    expect(all(row["exists"] for row in found["tuned_room_xml"]), "a tuned xml is missing")
    expect("previews/kit_cam_room.png" in found["tuned_stills"], "kitchen still not tuned")
    expect(all(rel.startswith("previews/held_out/") for rel in found["held_out_images"]), "held-out path")
    expect(any(rel.endswith("bedroom/bed.obj") for rel in found["unplaced_meshes"]), "unplaced gothic bed")
    expect(all(not rel.endswith("platform_bed.obj") for rel in found["unplaced_meshes"]), "placed bed marked unplaced")
    strong = {name: 0.0 for name, _room in fr.OBJECTS}
    strong["door"] = 0.20
    vote = fr.decide_window([fr._frame(strong)])
    unknown = _table_row(vote, None)
    expect(unknown["undecided"] and unknown["commit"] is None, f"unknown committed {unknown}")
    expect(unknown["model_room"] == "entrance", f"model room {unknown}")
    if not found["held_out_images"]:
        payload = score_held_out(detector=None)
        expect(payload["status"] == "blocked", str(payload["status"]))
        expect(payload["scenes"] == [], "blocked run scored a scene")
        expect(payload["score_min"] == 0.06 and payload["vote_margin"] == 0.15, "reported bars")
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[held-out] self-test PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    payload = score_held_out()
    _write(payload)
    _print(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
