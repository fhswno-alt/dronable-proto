#!/usr/bin/env python3
"""Score frozen kit_cam room votes on AI2-THOR stills.

Camera height is 0.38 m and the field of view is 104.82 degrees, matching
kit_cam. The object map, cutoff, margin, and bonus are the ones in
find_room.py. This script does not edit the plant, does not step a gait,
and does not publish vel. The five MuJoCo room XMLs are not loaded.

CloudRendering aborts on this machine (no Vulkan physical device). The
Linux64 build is the one that renders.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import find_room as fr

ROOT = fr.ROOT
PREVIEWS = fr.PREVIEWS
STILL_DIR = PREVIEWS / "held_out_thor"
SUMMARY = PREVIEWS / "find_room_thor.json"
CAM_Y_M = 0.38
FOV_DEG = 104.82
WIDTH = 640
HEIGHT = 480
# Five iTHOR scenes per class. Not the MuJoCo kit_cam rooms.
SCENES = {
    "kitchen": [f"FloorPlan{i}" for i in range(1, 6)],
    "living": [f"FloorPlan{i}" for i in range(201, 206)],
    "bedroom": [f"FloorPlan{i}" for i in range(301, 306)],
    "bathroom": [f"FloorPlan{i}" for i in range(401, 406)],
}


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


def _yaw_toward(src: dict, dst: dict) -> float:
    dx = float(dst["x"]) - float(src["x"])
    dz = float(dst["z"]) - float(src["z"])
    return math.degrees(math.atan2(dx, dz)) % 360.0


def _aim(controller, event) -> tuple[dict, float]:
    """Stand the still camera on the floor mesh, looking at the object centroid."""
    objects = [obj for obj in event.metadata["objects"] if obj.get("position")]
    centroid = {
        "x": sum(obj["position"]["x"] for obj in objects) / len(objects),
        "z": sum(obj["position"]["z"] for obj in objects) / len(objects),
    }
    reached = controller.step(action="GetReachablePositions").metadata.get("actionReturn") or []

    def apart(point) -> float:
        return math.hypot(point["x"] - centroid["x"], point["z"] - centroid["z"])

    band = [point for point in reached if 1.0 <= apart(point) <= 2.2]
    pool = band or [point for point in reached if apart(point) >= 0.6] or reached
    if pool:
        spot = min(pool, key=lambda point: abs(apart(point) - 1.5))
    else:
        spot = event.metadata["agent"]["position"]
    return spot, _yaw_toward(spot, centroid)


def render_stills() -> list[dict[str, object]]:
    """One level still per scene. Requires the Linux64 THOR build and a display."""
    from PIL import Image
    from ai2thor.controller import Controller

    STILL_DIR.mkdir(parents=True, exist_ok=True)
    controller = Controller(
        platform="Linux64",
        scene="FloorPlan1",
        quality="Low",
        width=WIDTH,
        height=HEIGHT,
        fieldOfView=FOV_DEG,
        server_timeout=90,
        server_start_timeout=180,
    )
    rows: list[dict[str, object]] = []
    try:
        for room, scenes in SCENES.items():
            for scene in scenes:
                event = controller.reset(scene=scene, fieldOfView=FOV_DEG)
                spot, yaw = _aim(controller, event)
                shot = controller.step(
                    action="AddThirdPartyCamera",
                    position={"x": float(spot["x"]), "y": CAM_Y_M, "z": float(spot["z"])},
                    rotation={"x": 0.0, "y": yaw, "z": 0.0},
                    fieldOfView=FOV_DEG,
                )
                if not shot.metadata.get("lastActionSuccess") or not shot.third_party_camera_frames:
                    raise RuntimeError(
                        f"{scene} camera failed: {shot.metadata.get('errorMessage')}"
                    )
                frame = shot.third_party_camera_frames[-1]
                path = STILL_DIR / f"{room}_{scene}.png"
                Image.fromarray(frame).save(path)
                rows.append({
                    "room": room,
                    "scene": scene,
                    "file": str(path.relative_to(ROOT)),
                    "camera_y_m": CAM_Y_M,
                    "fov_deg": FOV_DEG,
                    "yaw_deg": yaw,
                    "camera_x_m": float(spot["x"]),
                    "camera_z_m": float(spot["z"]),
                })
                print(f"[thor] rendered {room} {scene} yaw {yaw:.1f}", flush=True)
    finally:
        controller.stop()
    return rows


def _vote_row(seen: fr.Detection, expected: str) -> dict[str, object]:
    frame = fr._frame(seen.object_scores or {}, seen.object_bias or {})
    vote = fr.decide_window([frame])
    ranked = sorted(fr._frame_votes(seen.object_scores or {}).items(), key=lambda item: item[1], reverse=True)
    false_commit = vote.room if vote.room not in (None, expected) else None
    return {
        "leader": ranked[0][0],
        "commit": vote.room,
        "hit": vote.room == expected,
        "false_commit": false_commit,
        "lead": vote.gap,
        "vote": vote.vote,
        "runner": vote.runner,
        "raw_object": seen.object_name,
        "raw_score": seen.score,
        "object_scores": seen.object_scores,
    }


def score_stills(rows: list[dict[str, object]]) -> dict[str, object]:
    _frozen()
    detector = fr.RoomDetector()
    import numpy as np
    from PIL import Image

    for row in rows:
        frame = np.asarray(Image.open(ROOT / row["file"]).convert("RGB"))
        seen = detector.read(frame)
        row.update(_vote_row(seen, str(row["room"])))
        mark = "HIT" if row["hit"] else ("FALSE " + str(row["false_commit"]) if row["false_commit"] else "und")
        print(
            f"[thor] {row['room']:9} {row['scene']:12} {mark:16} "
            f"lead {float(row['lead']):.3f} raw {row['raw_object']} {float(row['raw_score'] or 0):.3f}",
            flush=True,
        )
    classes = []
    for room in SCENES:
        group = [row for row in rows if row["room"] == room]
        hits = [row for row in group if row["hit"]]
        false_rows = [row for row in group if row["false_commit"]]
        leads = [float(row["lead"]) for row in hits]
        closest = max(group, key=lambda row: float(row["lead"]))
        classes.append({
            "room": room,
            "hits": len(hits),
            "scenes": len(group),
            "commit": room if hits else None,
            "min_lead": min(leads) if leads else None,
            "closest_lead": float(closest["lead"]),
            "closest_scene": closest["scene"],
            "closest_leader": closest.get("leader"),
            "closest_raw": closest.get("raw_object"),
            "closest_raw_score": closest.get("raw_score"),
            "false_commits": [
                {"scene": row["scene"], "room": row["false_commit"], "lead": row["lead"]}
                for row in false_rows
            ],
            "undecided": sum(1 for row in group if row["commit"] is None),
        })
    return {
        "status": "scored",
        "source": "ai2thor-ithor",
        "platform": "Linux64",
        "cloud_rendering": "aborted: vkEnumeratePhysicalDevices Invalid instance, returncode=-6, no Vulkan physical device",
        "ai2thor": "5.0.0",
        "commit_id": "f0825767cd50d69f666c7f282e54abfe58f1e917",
        "camera_y_m": CAM_Y_M,
        "fov_deg": FOV_DEG,
        "width": WIDTH,
        "height": HEIGHT,
        "model": fr.MODEL_ID,
        "model_rev": fr.MODEL_REV,
        "score_min": fr.SCORE_MIN,
        "vote_margin": fr.VOTE_MARGIN,
        "object_bonus": fr.OBJECT_BONUS,
        "object_bonus_cap": fr.OBJECT_BONUS_CAP,
        "vote_frames": 1,
        "vote_frames_note": "one still per scene, so the frozen window is that one frame",
        "gait": False,
        "vel": False,
        "arrival": False,
        "plant_md5": _plant_md5(),
        "classes": classes,
        "scenes": rows,
    }


def _print(payload: dict[str, object]) -> None:
    print(
        f"[thor] cutoff {payload['score_min']} margin {payload['vote_margin']} "
        f"camera_y {payload['camera_y_m']} fov {payload['fov_deg']} "
        f"plant {payload['plant_md5']}"
    )
    print("| Scene | Hits | Commit | Lead | Committed false |")
    print("|--|--|--|--|--|")
    for row in payload["classes"]:
        lead = row["min_lead"] if row["min_lead"] is not None else row.get("closest_lead")
        lead_s = "—" if lead is None else f"{float(lead):.3f}"
        commit = row["commit"] or "none"
        false = row["false_commits"]
        false_s = "none" if not false else ", ".join(
            f"{item['scene']}->{item['room']}" for item in false
        )
        print(
            f"| {row['room']} | {row['hits']}/{row['scenes']} | {commit} | {lead_s} | {false_s} |"
        )


def self_test() -> int:
    _frozen()
    failures = []
    if _plant_md5() != "71b2c86d133ebc603f58b99c53e496f3":
        failures.append("plant md5")
    body = Path(__file__).read_text().split("def self_test", 1)[0]
    if "bus.vel" in body or "session.step" in body:
        failures.append("gait or vel")
    names = {scene for scenes in SCENES.values() for scene in scenes}
    if len(names) != 20:
        failures.append(f"scene count {len(names)}")
    if any(name.startswith("room_") for name in names):
        failures.append("mujoco room in the thor list")
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[thor] self-test PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--score-only", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.score_only:
        rows = []
        for room, scenes in SCENES.items():
            for scene in scenes:
                path = STILL_DIR / f"{room}_{scene}.png"
                if not path.is_file():
                    print(f"FAIL missing still {path}")
                    return 1
                rows.append({
                    "room": room,
                    "scene": scene,
                    "file": str(path.relative_to(ROOT)),
                    "camera_y_m": CAM_Y_M,
                    "fov_deg": FOV_DEG,
                })
    else:
        rows = render_stills()
    payload = score_stills(rows)
    SUMMARY.write_text(json.dumps(payload, indent=2) + "\n")
    if fr.ARTIFACTS.is_dir():
        (fr.ARTIFACTS / SUMMARY.name).write_text(SUMMARY.read_text())
    _print(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
