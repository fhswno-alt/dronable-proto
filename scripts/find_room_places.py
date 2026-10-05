#!/usr/bin/env python3
"""Secondary held-out scores on Places365 validation photos.

These are eye-height photographs, not 0.38 m kit_cam stills. They are not
the iTHOR table. The object map, cutoff, margin, and bonus stay frozen.
Five photos per class, one frame each, so a commit still needs a lead of 0.15.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import find_room as fr

ROOT = fr.ROOT
STILL_DIR = fr.PREVIEWS / "held_out_places"
SUMMARY = fr.PREVIEWS / "find_room_places.json"
# Filename prefix, expected room, Places365 class id.
GROUPS = (
    ("kitchen", "kitchen", 203),
    ("bathroom", "bathroom", 45),
    ("bedroom", "bedroom", 52),
    ("living", "living", 215),
    ("entrance_hall", "entrance", 134),
    ("corridor", "entrance", 106),
)


def _frozen() -> None:
    if fr.SCORE_MIN != 0.06 or fr.VOTE_MARGIN != 0.15:
        raise RuntimeError("cutoff or margin moved")
    if fr.OBJECT_BONUS != 0.03 or fr.OBJECT_BONUS_CAP != 0.06:
        raise RuntimeError("bonus or cap moved")


def score() -> dict[str, object]:
    _frozen()
    import numpy as np
    from PIL import Image

    detector = fr.RoomDetector()
    rows = []
    for prefix, expected, class_id in GROUPS:
        paths = sorted(STILL_DIR.glob(f"{prefix}_Places365_val_*.jpg"))
        if len(paths) < 5:
            raise RuntimeError(f"{prefix} has {len(paths)} photos, need 5")
        for path in paths[:5]:
            frame = np.asarray(Image.open(path).convert("RGB"))
            seen = detector.read(frame)
            vote = fr.decide_window([fr._frame(seen.object_scores or {}, seen.object_bias or {})])
            ranked = sorted(
                fr._frame_votes(seen.object_scores or {}).items(),
                key=lambda item: item[1],
                reverse=True,
            )
            false_commit = vote.room if vote.room not in (None, expected) else None
            row = {
                "group": prefix,
                "expected": expected,
                "places_class_id": class_id,
                "file": str(path.relative_to(ROOT)),
                "commit": vote.room,
                "hit": vote.room == expected,
                "false_commit": false_commit,
                "lead": vote.gap,
                "leader": ranked[0][0],
                "raw_object": seen.object_name,
                "raw_score": seen.score,
            }
            rows.append(row)
            mark = "HIT" if row["hit"] else ("FALSE " + str(false_commit) if false_commit else "und")
            print(
                f"[places] {prefix:14} {path.name} {mark:16} lead {vote.gap:.3f} "
                f"raw {seen.object_name} {float(seen.score or 0):.3f}",
                flush=True,
            )
    classes = []
    for prefix, expected, class_id in GROUPS:
        group = [row for row in rows if row["group"] == prefix]
        hits = [row for row in group if row["hit"]]
        false_rows = [row for row in group if row["false_commit"]]
        leads = [float(row["lead"]) for row in hits]
        closest = max(group, key=lambda row: float(row["lead"]))
        classes.append({
            "group": prefix,
            "expected": expected,
            "places_class_id": class_id,
            "hits": len(hits),
            "scenes": len(group),
            "commit": expected if hits else None,
            "min_lead": min(leads) if leads else None,
            "closest_lead": float(closest["lead"]),
            "closest_leader": closest["leader"],
            "closest_raw": closest["raw_object"],
            "closest_raw_score": closest["raw_score"],
            "false_commits": [
                {"file": row["file"], "room": row["false_commit"], "lead": row["lead"]}
                for row in false_rows
            ],
            "undecided": sum(1 for row in group if row["commit"] is None),
        })
    digest = fr._md5(fr.PLANT_XML)
    if digest != fr.steer_walk.PLANT_MD5:
        raise RuntimeError("plant md5 changed")
    return {
        "status": "scored",
        "source": "places365-val-256",
        "camera": "eye-height photograph, not 0.38 m kit_cam",
        "note": "Secondary to the iTHOR table. Five validation photos per class.",
        "score_min": fr.SCORE_MIN,
        "vote_margin": fr.VOTE_MARGIN,
        "object_bonus": fr.OBJECT_BONUS,
        "object_bonus_cap": fr.OBJECT_BONUS_CAP,
        "vote_frames": 1,
        "gait": False,
        "vel": False,
        "arrival": False,
        "plant_md5": digest,
        "model": fr.MODEL_ID,
        "model_rev": fr.MODEL_REV,
        "classes": classes,
        "scenes": rows,
    }


def _print(payload: dict[str, object]) -> None:
    print("[places] eye-height secondary. Not the 0.38 m THOR table.")
    print("| Scene | Hits | Commit | Lead | Committed false |")
    print("|--|--|--|--|--|")
    for row in payload["classes"]:
        lead = row["min_lead"] if row["min_lead"] is not None else row["closest_lead"]
        false = row["false_commits"]
        false_s = "none" if not false else str(len(false))
        commit = row["commit"] or "none"
        print(
            f"| {row['group']} | {row['hits']}/{row['scenes']} | {commit} | {lead:.3f} | {false_s} |"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        _frozen()
        body = Path(__file__).read_text().split("def main", 1)[0]
        if "bus.vel" in body or "session.step" in body:
            print("FAIL gait or vel")
            return 1
        print("[places] self-test PASS")
        return 0
    payload = score()
    SUMMARY.write_text(json.dumps(payload, indent=2) + "\n")
    if fr.ARTIFACTS.is_dir():
        (fr.ARTIFACTS / SUMMARY.name).write_text(SUMMARY.read_text())
    _print(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
