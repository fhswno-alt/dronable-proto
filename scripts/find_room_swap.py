#!/usr/bin/env python3
"""Same held-out stills, other local models. Labels stay frozen.

Detectors use the object map, cutoff 0.06, and margin 0.15 from find_room.py.
A room VLM is asked which of the five rooms the photo is. No paid API.
A candidate that does not install is recorded and skipped.
"""

from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path

import find_room as fr
import find_room_margin as margin

ROOT = fr.ROOT
SUMMARY = fr.PREVIEWS / "find_room_swap.json"
ROOMS = ("kitchen", "bathroom", "living", "bedroom", "entrance")
VLM_PROMPT = "which room is this? kitchen/bathroom/living/bedroom/entrance"


def _frozen() -> None:
    if fr.SCORE_MIN != 0.06 or fr.VOTE_MARGIN != 0.15:
        raise RuntimeError("cutoff or margin moved")
    if tuple(fr.OBJECTS) != (
        ("fridge", "kitchen"),
        ("oven", "kitchen"),
        ("stove", "kitchen"),
        ("kitchen sink", "kitchen"),
        ("cupboard", "kitchen"),
        ("toilet", "bathroom"),
        ("bathtub", "bathroom"),
        ("bathroom sink", "bathroom"),
        ("bed", "bedroom"),
        ("pillow", "bedroom"),
        ("mattress", "bedroom"),
        ("upholstered bed", "bedroom"),
        ("sofa", "living"),
        ("TV", "living"),
        ("couch", "living"),
        ("door", "entrance"),
        ("hallway", "entrance"),
        ("doormat", "entrance"),
        ("shoe rack", "entrance"),
        ("coat hooks", "entrance"),
    ):
        raise RuntimeError("object map moved")


def _empty_scores() -> dict[str, float]:
    return {name: 0.0 for name, _room in fr.OBJECTS}


def _judge(scores: dict[str, float], expected: str) -> dict[str, object]:
    vote = fr.decide_window([fr._frame(scores)])
    off_room, off_vote, off_gap = margin._margin_off(scores)
    return {
        "margin_room": vote.room,
        "margin_hit": vote.room == expected,
        "margin_gap": vote.gap,
        "margin_off_room": off_room,
        "margin_off_hit": off_room == expected,
        "margin_off_wrong": off_room not in (None, expected),
        "margin_off_vote": off_vote,
        "margin_off_gap": off_gap,
    }


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
        classes.append({
            "source": source,
            "group": group,
            "scenes": n,
            "margin_015_hits": sum(1 for row in group_rows if row.get("margin_hit")),
            "margin_off_top1": sum(1 for row in group_rows if row.get("margin_off_hit")),
            "margin_off_wrong": sum(1 for row in group_rows if row.get("margin_off_wrong")),
            "named_hits": sum(1 for row in group_rows if row.get("named_hit")),
            "named_wrong": sum(1 for row in group_rows if row.get("named_wrong")),
        })
    return classes


class OwlV2Reader:
    model_id = "google/owlv2-base-patch16"

    def __init__(self) -> None:
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor

        torch.set_num_threads(4)
        self.torch = torch
        self.processor = Owlv2Processor.from_pretrained(self.model_id)
        self.model = Owlv2ForObjectDetection.from_pretrained(self.model_id)
        self.model.eval()
        self.texts = [name for name, _room in fr.OBJECTS]

    def scores(self, frame) -> dict[str, float]:
        inputs = self.processor(text=[self.texts], images=frame, return_tensors="pt")
        with self.torch.no_grad():
            outputs = self.model(**inputs)
        probs = self.torch.sigmoid(outputs.logits[0])
        best = probs.max(dim=0).values
        found = _empty_scores()
        for index, (name, _room) in enumerate(fr.OBJECTS):
            found[name] = float(best[index])
        return found

    def close(self) -> None:
        del self.model
        del self.processor


class GroundingDinoReader:
    model_id = "IDEA-Research/grounding-dino-tiny"

    def __init__(self) -> None:
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        torch.set_num_threads(4)
        self.torch = torch
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id)
        self.model.eval()
        self.text = ". ".join(name for name, _room in fr.OBJECTS) + "."

    def scores(self, frame) -> dict[str, float]:
        # One forward, then the max token score inside each phrase. The
        # decoded label string glues every phrase together at a low text
        # threshold, so it is not a per-object score.
        inputs = self.processor(images=frame, text=self.text, return_tensors="pt")
        with self.torch.no_grad():
            outputs = self.model(**inputs)
        probs = self.torch.sigmoid(outputs.logits[0])
        ids = inputs["input_ids"][0].tolist()
        dot = self.processor.tokenizer.convert_tokens_to_ids(".")
        special = set(self.processor.tokenizer.all_special_ids)
        spans: list[tuple[int, int]] = []
        start = None
        for index, token_id in enumerate(ids):
            if token_id == dot or token_id in special:
                if start is not None:
                    spans.append((start, index))
                    start = None
                continue
            if start is None:
                start = index
        if len(spans) != len(fr.OBJECTS):
            raise RuntimeError(f"phrase spans {len(spans)} != {len(fr.OBJECTS)}")
        found = _empty_scores()
        for index, (name, _room) in enumerate(fr.OBJECTS):
            begin, end = spans[index]
            found[name] = float(probs[:, begin:end].max())
        return found

    def close(self) -> None:
        del self.model
        del self.processor


def _named_room(text: str) -> str | None:
    lowered = text.lower()
    hits = []
    for room in ROOMS:
        token = "living room" if room == "living" else room
        if token in lowered or (room == "living" and "living" in lowered):
            hits.append(room)
    if len(hits) == 1:
        return hits[0]
    return None


class MoondreamReader:
    model_id = "vikhyatk/moondream2"

    def __init__(self) -> None:
        import torch
        from transformers import AutoModelForCausalLM

        torch.set_num_threads(4)
        self.model = AutoModelForCausalLM.from_pretrained(self.model_id, trust_remote_code=True)
        self.model.eval()

    def answer(self, frame) -> str:
        from PIL import Image

        image = Image.fromarray(frame)
        result = self.model.query(image, VLM_PROMPT)
        if isinstance(result, dict):
            return str(result.get("answer", result))
        return str(result)

    def close(self) -> None:
        del self.model


class QwenReader:
    """Recorded blocker. A mismatched torchvision wheel breaks torch, so it is not installed."""

    model_id = "Qwen/Qwen2-VL-2B-Instruct"

    def __init__(self) -> None:
        try:
            from transformers import AutoProcessor

            AutoProcessor.from_pretrained(self.model_id)
        except Exception as exc:
            raise RuntimeError(
                f"{type(exc).__name__}: {exc}\n"
                "torchvision==0.29.1 was installed and then removed. On torch 2.14.1+cpu it raised "
                "RuntimeError: operator torchvision::nms does not exist. The model was not scored."
            ) from exc
        raise RuntimeError("Qwen2-VL processor loaded; scoring was not reached")


class SmolVlmReader:
    model_id = "HuggingFaceTB/SmolVLM-256M-Instruct"

    def __init__(self) -> None:
        import torch
        from transformers import AutoModelForImageTextToText, AutoProcessor

        torch.set_num_threads(4)
        self.torch = torch
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = AutoModelForImageTextToText.from_pretrained(self.model_id)
        self.model.eval()

    def answer(self, frame) -> str:
        from PIL import Image

        image = Image.fromarray(frame)
        messages = [{
            "role": "user",
            "content": [
                {"type": "image"},
                {"type": "text", "text": VLM_PROMPT},
            ],
        }]
        prompt = self.processor.apply_chat_template(messages, add_generation_prompt=True)
        inputs = self.processor(text=prompt, images=[image], return_tensors="pt")
        with self.torch.no_grad():
            generated = self.model.generate(**inputs, max_new_tokens=16)
        trimmed = generated[:, inputs["input_ids"].shape[-1] :]
        return self.processor.batch_decode(trimmed, skip_special_tokens=True)[0]

    def close(self) -> None:
        del self.model
        del self.processor


def _score_detector(reader, specs: list[dict[str, str]]) -> list[dict[str, object]]:
    import numpy as np
    from PIL import Image

    rows = []
    for spec in specs:
        frame = np.asarray(Image.open(ROOT / spec["file"]).convert("RGB"))
        scores = reader.scores(frame)
        row = {**spec, **_judge(scores, spec["expected"])}
        rows.append(row)
        print(
            f"[swap] {spec['source']:9} {spec['group']:14} {spec['scene']:28} "
            f"m015 {row['margin_room'] or 'none':9} off {row['margin_off_room'] or 'none'}",
            flush=True,
        )
    return rows


def _score_vlm(reader, specs: list[dict[str, str]]) -> list[dict[str, object]]:
    import numpy as np
    from PIL import Image

    rows = []
    for spec in specs:
        frame = np.asarray(Image.open(ROOT / spec["file"]).convert("RGB"))
        text = reader.answer(frame)
        named = _named_room(text)
        expected = spec["expected"]
        row = {
            **spec,
            "answer": text.strip(),
            "named_room": named,
            "named_hit": named == expected,
            "named_wrong": named not in (None, expected),
        }
        rows.append(row)
        print(
            f"[swap] {spec['source']:9} {spec['group']:14} {spec['scene']:28} "
            f"named {named or 'none':9} {text.strip()!r}",
            flush=True,
        )
    return rows


def _run_one(kind: str, factory, specs: list[dict[str, str]]) -> dict[str, object]:
    _frozen()
    try:
        reader = factory()
    except Exception:
        error = traceback.format_exc()
        print(f"[swap] {kind} install/load failed\n{error}", flush=True)
        return {"model": kind, "status": "blocked", "error": error}
    try:
        if kind in ("smolvlm", "moondream"):
            rows = _score_vlm(reader, specs)
        else:
            rows = _score_detector(reader, specs)
    except Exception:
        error = traceback.format_exc()
        print(f"[swap] {kind} score failed\n{error}", flush=True)
        return {"model": kind, "status": "blocked", "error": error}
    finally:
        close = getattr(reader, "close", None)
        if close is not None:
            close()
    classes = _summarize(rows)
    ithor = [row for row in classes if row["source"] == "ithor"]
    return {
        "model": kind,
        "model_id": getattr(factory, "model_id", kind),
        "status": "scored",
        "classes": classes,
        "scenes": rows,
        "ithor_margin_015_hits": sum(int(row["margin_015_hits"]) for row in ithor),
        "ithor_margin_off_top1": sum(int(row["margin_off_top1"]) for row in ithor),
        "ithor_named_hits": sum(int(row["named_hits"]) for row in ithor),
        "ithor_scenes": sum(int(row["scenes"]) for row in ithor),
    }


def score(only: list[str] | None = None) -> dict[str, object]:
    _frozen()
    specs = margin._rows()
    missing = [spec["file"] for spec in specs if not (ROOT / spec["file"]).is_file()]
    if missing:
        raise RuntimeError(f"missing stills {missing[:3]}")
    wanted = only or ["owlv2", "grounding-dino", "smolvlm", "moondream", "qwen2-vl"]
    factories = {
        "owlv2": OwlV2Reader,
        "grounding-dino": GroundingDinoReader,
        "smolvlm": SmolVlmReader,
        "moondream": MoondreamReader,
        "qwen2-vl": QwenReader,
    }
    models = []
    for name in wanted:
        print(f"[swap] start {name}", flush=True)
        models.append(_run_one(name, factories[name], specs))
    digest = fr._md5(fr.PLANT_XML)
    if digest != fr.steer_walk.PLANT_MD5:
        raise RuntimeError("plant md5 changed")
    return {
        "status": "scored",
        "labels_changed": False,
        "score_min": fr.SCORE_MIN,
        "vote_margin": fr.VOTE_MARGIN,
        "prompt": VLM_PROMPT,
        "gait": False,
        "vel": False,
        "arrival": False,
        "plant_md5": digest,
        "models": models,
    }


def _print(payload: dict[str, object]) -> None:
    for model in payload["models"]:
        if model["status"] != "scored":
            first = model["error"].strip().splitlines()[-1]
            print(f"[swap] {model['model']} BLOCKED {first}")
            continue
        print(
            f"[swap] {model['model']} ithor margin {model['ithor_margin_015_hits']}/"
            f"{model['ithor_scenes']} top1 {model['ithor_margin_off_top1']}/{model['ithor_scenes']} "
            f"named {model['ithor_named_hits']}/{model['ithor_scenes']}"
        )
        print("| Model | Source | Scene | Margin 0.15 | Margin off top-1 | Named room |")
        print("|--|--|--|--|--|--|")
        for row in model["classes"]:
            n = row["scenes"]
            named = "—" if model["model"] not in ("smolvlm", "moondream") else f"{row['named_hits']}/{n}"
            print(
                f"| {model['model']} | {row['source']} | {row['group']} | "
                f"{row['margin_015_hits']}/{n} | {row['margin_off_top1']}/{n} | {named} |"
            )


def self_test() -> int:
    _frozen()
    failures = []
    if _named_room("kitchen") != "kitchen":
        failures.append("parse kitchen")
    if _named_room("a bedroom and a kitchen") is not None:
        failures.append("two rooms should abstain")
    if _named_room("hallway") is not None:
        failures.append("hallway is not a room answer")
    body = Path(__file__).read_text().split("def self_test", 1)[0]
    if "bus.vel" in body or "session.step" in body:
        failures.append("gait or vel")
    if fr._md5(fr.PLANT_XML) != "71b2c86d133ebc603f58b99c53e496f3":
        failures.append("plant md5")
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[swap] self-test PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--only", default="")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    only = [name for name in args.only.split(",") if name] or None
    payload = score(only)
    if SUMMARY.is_file() and only:
        previous = json.loads(SUMMARY.read_text())
        by_name = {model["model"]: model for model in previous.get("models", [])}
        for model in payload["models"]:
            by_name[model["model"]] = model
        order = ("owlv2", "grounding-dino", "smolvlm", "moondream", "qwen2-vl")
        payload["models"] = [by_name[name] for name in order if name in by_name]
        payload["plant_md5"] = fr._md5(fr.PLANT_XML)
    SUMMARY.write_text(json.dumps(payload, indent=2) + "\n")
    if fr.ARTIFACTS.is_dir():
        (fr.ARTIFACTS / SUMMARY.name).write_text(SUMMARY.read_text())
    _print(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
