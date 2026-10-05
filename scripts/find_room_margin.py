#!/usr/bin/env python3
"""Margin diagnosis on the held-out stills already rendered.

The object map, cutoff, bonus, and cap stay the values in find_room.py.
This script does not write those constants. It only asks two extra questions
of the same boxes:

- margin off: the room with the highest cutoff vote, even when the lead is
  under 0.15. A room with nothing at 0.06 still votes 0 and is not a guess.
- cutoff off: the room whose best object score is highest, including scores
  under 0.06.

A tie is not a correct top-1. The plant file is not edited.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import find_room as fr
import find_room_places as places
import find_room_thor as thor

ROOT = fr.ROOT
SUMMARY = fr.PREVIEWS / "find_room_margin.json"


def _frozen() -> None:
    if fr.SCORE_MIN != 0.06 or fr.VOTE_MARGIN != 0.15:
        raise RuntimeError("cutoff or margin moved")
    if fr.OBJECT_BONUS != 0.03 or fr.OBJECT_BONUS_CAP != 0.06:
        raise RuntimeError("bonus or cap moved")


def _rooms() -> list[str]:
    return [room for room in fr.STAND_TOP if room != "plant"]


def _margin_off(scores: dict[str, float]) -> tuple[str | None, float, float]:
    """Highest room at the cutoff. None when nothing clears 0.06 or the top ties."""
    votes = fr._frame_votes(scores)
    ranked = sorted(votes.items(), key=lambda item: item[1], reverse=True)
    winner, win_vote = ranked[0]
    _runner, run_vote = ranked[1]
    if win_vote < fr.SCORE_MIN or win_vote == run_vote:
        return None, win_vote, win_vote - run_vote
    return winner, win_vote, win_vote - run_vote


def _cutoff_off(scores: dict[str, float]) -> tuple[str | None, float, str | None]:
    """Highest room by its best object, with the cutoff ignored. Ties abstain."""
    best = {room: 0.0 for room in _rooms()}
    name_of = {room: None for room in _rooms()}
    for name, room in fr.OBJECTS:
        score = float(scores.get(name, 0.0))
        if score > best[room]:
            best[room] = score
            name_of[room] = name
    ranked = sorted(best.items(), key=lambda item: item[1], reverse=True)
    winner, win_score = ranked[0]
    if win_score <= 0.0 or win_score == ranked[1][1]:
        return None, win_score, name_of[winner]
    return winner, win_score, name_of[winner]


def _rows() -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for room, scenes in thor.SCENES.items():
        for scene in scenes:
            path = thor.STILL_DIR / f"{room}_{scene}.png"
            rows.append({
                "source": "ithor",
                "group": room,
                "expected": room,
                "scene": scene,
                "file": str(path.relative_to(ROOT)),
            })
    for prefix, expected, _class_id in places.GROUPS:
        paths = sorted(places.STILL_DIR.glob(f"{prefix}_Places365_val_*.jpg"))
        for path in paths[:5]:
            rows.append({
                "source": "places365",
                "group": prefix,
                "expected": expected,
                "scene": path.name,
                "file": str(path.relative_to(ROOT)),
            })
    return rows


def _summarize(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    order: list[tuple[str, str]] = []
    for row in rows:
        key = (str(row["source"]), str(row["group"]))
        if key not in order:
            order.append(key)
    classes = []
    for source, group in order:
        group_rows = [row for row in rows if row["source"] == source and row["group"] == group]
        n = len(group_rows)
        margin_hits = sum(1 for row in group_rows if row["margin_hit"])
        off_hits = sum(1 for row in group_rows if row["margin_off_hit"])
        off_calls = sum(1 for row in group_rows if row["margin_off_room"] is not None)
        off_wrong = sum(1 for row in group_rows if row["margin_off_wrong"])
        cut_hits = sum(1 for row in group_rows if row["cutoff_off_hit"])
        cut_wrong = sum(1 for row in group_rows if row["cutoff_off_wrong"])
        classes.append({
            "source": source,
            "group": group,
            "scenes": n,
            "margin_015_hits": margin_hits,
            "margin_off_top1": off_hits,
            "margin_off_calls": off_calls,
            "margin_off_wrong": off_wrong,
            "cutoff_off_top1": cut_hits,
            "cutoff_off_wrong": cut_wrong,
        })
    return classes


def score() -> dict[str, object]:
    _frozen()
    import numpy as np
    from PIL import Image

    detector = fr.RoomDetector()
    rows: list[dict[str, object]] = []
    for spec in _rows():
        path = ROOT / spec["file"]
        if not path.is_file():
            raise RuntimeError(f"missing still {path}")
        frame = np.asarray(Image.open(path).convert("RGB"))
        seen = detector.read(frame)
        scores = seen.object_scores or {}
        vote = fr.decide_window([fr._frame(scores, seen.object_bias or {})])
        off_room, off_vote, off_gap = _margin_off(scores)
        cut_room, cut_score, cut_object = _cutoff_off(scores)
        expected = spec["expected"]
        row = {
            **spec,
            "margin_room": vote.room,
            "margin_hit": vote.room == expected,
            "margin_gap": vote.gap,
            "margin_off_room": off_room,
            "margin_off_vote": off_vote,
            "margin_off_gap": off_gap,
            "margin_off_hit": off_room == expected,
            "margin_off_wrong": off_room not in (None, expected),
            "cutoff_off_room": cut_room,
            "cutoff_off_score": cut_score,
            "cutoff_off_object": cut_object,
            "cutoff_off_hit": cut_room == expected,
            "cutoff_off_wrong": cut_room not in (None, expected),
        }
        rows.append(row)
        print(
            f"[margin] {spec['source']:9} {spec['group']:14} {spec['scene']:28} "
            f"m015 {vote.room or 'none':9} off {off_room or 'none':9} "
            f"cut {cut_room or 'none':9} {cut_object or ''} {cut_score:.3f}",
            flush=True,
        )
    digest = fr._md5(fr.PLANT_XML)
    if digest != fr.steer_walk.PLANT_MD5:
        raise RuntimeError("plant md5 changed")
    classes = _summarize(rows)
    ithor = [row for row in classes if row["source"] == "ithor"]
    ithor_n = sum(int(row["scenes"]) for row in ithor)
    ithor_off = sum(int(row["margin_off_top1"]) for row in ithor)
    ithor_cut = sum(int(row["cutoff_off_top1"]) for row in ithor)
    ithor_m = sum(int(row["margin_015_hits"]) for row in ithor)
    # Five room classes. A uniform guess is right about one time in five.
    chance = 1.0 / len(_rooms())
    off_rate = ithor_off / ithor_n
    # Primary call is the 0.38 m set. Mostly-right means at least 60% top-1.
    ithor_call = "margin too strict" if off_rate >= 0.60 and ithor_m == 0 else "OWL-ViT base-p32 is the limit"
    core_names = {"kitchen", "bathroom", "bedroom", "living"}
    core = [row for row in classes if row["source"] == "places365" and row["group"] in core_names]
    core_n = sum(int(row["scenes"]) for row in core)
    core_off = sum(int(row["margin_off_top1"]) for row in core)
    core_m = sum(int(row["margin_015_hits"]) for row in core)
    places_call = (
        "margin too strict"
        if core_n and (core_off / core_n) >= 0.60 and core_m < core_off
        else "OWL-ViT base-p32 is the limit"
    )
    return {
        "status": "scored",
        "call": ithor_call,
        "ithor_call": ithor_call,
        "places_call": places_call,
        "places_core_margin_015_hits": core_m,
        "places_core_margin_off_top1": core_off,
        "places_core_scenes": core_n,
        "chance_top1": chance,
        "ithor_margin_015_hits": ithor_m,
        "ithor_margin_off_top1": ithor_off,
        "ithor_cutoff_off_top1": ithor_cut,
        "ithor_scenes": ithor_n,
        "score_min": fr.SCORE_MIN,
        "vote_margin": fr.VOTE_MARGIN,
        "object_bonus": fr.OBJECT_BONUS,
        "object_bonus_cap": fr.OBJECT_BONUS_CAP,
        "labels_changed": False,
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
    print(
        f"[margin] ithor {payload['ithor_call']} places {payload['places_call']} "
        f"ithor margin-off {payload['ithor_margin_off_top1']}/{payload['ithor_scenes']} "
        f"cutoff-off {payload['ithor_cutoff_off_top1']}/{payload['ithor_scenes']} "
        f"margin-0.15 {payload['ithor_margin_015_hits']}/{payload['ithor_scenes']}"
    )
    print("| Source | Scene | Margin 0.15 | Margin off top-1 | Cutoff off top-1 | Wrong margin-off |")
    print("|--|--|--|--|--|--|")
    for row in payload["classes"]:
        n = row["scenes"]
        print(
            f"| {row['source']} | {row['group']} | {row['margin_015_hits']}/{n} | "
            f"{row['margin_off_top1']}/{n} | {row['cutoff_off_top1']}/{n} | "
            f"{row['margin_off_wrong']}/{n} |"
        )


def self_test() -> int:
    _frozen()
    body = Path(__file__).read_text().split("def self_test", 1)[0]
    failures = []
    if "bus.vel" in body or "session.step" in body:
        failures.append("gait or vel")
    if fr._md5(fr.PLANT_XML) != "71b2c86d133ebc603f58b99c53e496f3":
        failures.append("plant md5")
    kitchen = {name: 0.0 for name, _room in fr.OBJECTS}
    kitchen["cupboard"] = 0.10
    kitchen["fridge"] = 0.07
    kitchen["door"] = 0.07
    room, _vote, gap = _margin_off(kitchen)
    if room != "kitchen" or gap >= fr.VOTE_MARGIN:
        failures.append(f"margin-off kitchen {room} gap {gap}")
    vote = fr.decide_window([fr._frame(kitchen)])
    if vote.room is not None:
        failures.append("margin 0.15 committed a thin kitchen lead")
    tied = {name: 0.0 for name, _room in fr.OBJECTS}
    tied["toilet"] = 0.04
    tied["bed"] = 0.04
    cut_room, _score, _name = _cutoff_off(tied)
    if cut_room is not None:
        failures.append(f"cutoff-off broke a tie {cut_room}")
    low = {name: 0.0 for name, _room in fr.OBJECTS}
    low["bed"] = 0.05
    low["toilet"] = 0.02
    cut_room, _score, _name = _cutoff_off(low)
    off_room, _vote, _gap = _margin_off(low)
    if cut_room != "bedroom" or off_room is not None:
        failures.append(f"cutoff-off bed {cut_room} margin-off {off_room}")
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[margin] self-test PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    payload = score()
    SUMMARY.write_text(json.dumps(payload, indent=2) + "\n")
    if fr.ARTIFACTS.is_dir():
        (fr.ARTIFACTS / SUMMARY.name).write_text(SUMMARY.read_text())
    _print(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
