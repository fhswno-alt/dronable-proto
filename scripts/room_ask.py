#!/usr/bin/env python3
"""Local Moondream2 scene question. Not an object map and not a walk.

The Hub card for the pinned revision says `license: apache-2.0`.
The prompt and the parse rules below are frozen before the fresh score.
A hedge stays undecided. The word none is a decided unknown, not a room.
"""

from __future__ import annotations

import time

MODEL_ID = "vikhyatk/moondream2"
# Hub main snapshot. Same weights as the card's tag 2025-06-21.
# That tag's custom class does not load on transformers 5.18.0:
# AttributeError: 'HfMoondream' object has no attribute 'all_tied_weights_keys'.
MODEL_REV = "5d6c926f44e26b07957b0dd315bbedcb4c17a5fe"
MODEL_SHA = "5d6c926f44e26b07957b0dd315bbedcb4c17a5fe"
WEIGHTS_SHA256 = "70a7d94c0c8349eb58ed2d9e636ef2d0916960f321ecabeac6354b8ba3d7403f"
TAG_REV = "2025-06-21"
TAG_SHA = "9a7d4024050840e001defacec2b00727e89149e6"
# Selection 20/20 used this same unpinned snapshot.
SELECTION_SHA = "5d6c926f44e26b07957b0dd315bbedcb4c17a5fe"

PROMPT = (
    "Which room is this? Answer with one word: "
    "kitchen, bathroom, living, bedroom, entrance, or none."
)
ROOMS = ("kitchen", "bathroom", "living", "bedroom", "entrance")
NONE_WORDS = ("none", "unknown", "unsure", "undecided")
# Doorway measure. Not the open-set prompt above. The hall fills most of
# a 0.70 m doorway frame, so "entrance" stays a fair open-set answer.
YES_NO_PROMPT = "Is there a {name} through the doorway ahead? Answer yes or no."
YES_WORDS = ("yes", "yeah")
NO_WORDS = ("no", "nope")


def parse_answer(raw: str) -> str | None:
    """One allowed word, or None when the model hedges.

    Lowercase, then treat "living room" as living. Split on spaces and
    on / , ;. One room word and no none-word returns that room. Only a
    none-word returns "none". Two room words, a room word plus a none-word,
    or no allowed word returns None.
    """
    text = raw.strip().lower()
    for mark in ".!?:;\"'":
        text = text.replace(mark, " ")
    text = text.replace("living room", "living")
    for sep in ("/", ",", ";"):
        text = text.replace(sep, " ")
    tokens = [token for token in text.split() if token]
    rooms = [token for token in tokens if token in ROOMS]
    nones = [token for token in tokens if token in NONE_WORDS]
    if len(rooms) == 1 and not nones:
        return rooms[0]
    if nones and not rooms:
        return "none"
    return None


def yes_no_prompt(room: str) -> str:
    """Per-target question. Living is asked as 'living room'."""
    name = "living room" if room == "living" else room
    return YES_NO_PROMPT.format(name=name)


def parse_yes_no(raw: str) -> str | None:
    """'yes' or 'no' when that is the only decision. A hedge is None."""
    text = raw.strip().lower()
    for mark in ".!?:;\"'":
        text = text.replace(mark, " ")
    tokens = [token for token in text.split() if token]
    yes = [token for token in tokens if token in YES_WORDS]
    no = [token for token in tokens if token in NO_WORDS]
    if yes and not no:
        return "yes"
    if no and not yes:
        return "no"
    return None


class RoomAsk:
    """One pinned Moondream2. query() is the only call. No paid API."""

    def __init__(self) -> None:
        import torch
        from transformers import AutoModelForCausalLM

        torch.set_num_threads(4)
        self.torch = torch
        self.model = AutoModelForCausalLM.from_pretrained(
            MODEL_ID,
            revision=MODEL_REV,
            trust_remote_code=True,
        )
        self.model.eval()

    def _query(self, frame, prompt: str) -> tuple[str, float | None, float]:
        from PIL import Image

        image = Image.fromarray(frame)
        inner = getattr(self.model, "model", None)
        captured: list[float] = []
        original = getattr(inner, "_prefill_prompt", None)

        def _wrapped(*args, **kwargs):
            out = original(*args, **kwargs)
            logits = out[0]
            probs = self.torch.softmax(logits.detach().float().reshape(-1), dim=-1)
            captured.append(float(probs.max()))
            return out

        if original is not None:
            inner._prefill_prompt = _wrapped
        started = time.perf_counter()
        try:
            result = self.model.query(image, prompt)
        finally:
            if original is not None:
                inner._prefill_prompt = original
            elapsed = time.perf_counter() - started
        raw = str(result.get("answer", result) if isinstance(result, dict) else result)
        confidence = captured[-1] if captured else None
        return raw.strip(), confidence, elapsed

    def ask(self, frame) -> dict[str, object]:
        raw, confidence, elapsed = self._query(frame, PROMPT)
        room = parse_answer(raw)
        # First answer token from the prompt prefill. A hedge keeps the
        # probability for the log and still returns no room.
        return {
            "room": room,
            "confidence": confidence,
            "raw": raw,
            "seconds": elapsed,
        }

    def ask_yes_no(self, frame, prompt: str) -> dict[str, object]:
        raw, confidence, elapsed = self._query(frame, prompt)
        return {
            "answer": parse_yes_no(raw),
            "confidence": confidence,
            "raw": raw,
            "seconds": elapsed,
        }

    def close(self) -> None:
        del self.model


def self_test() -> int:
    failures = []
    cases = {
        "kitchen": "kitchen",
        "Kitchen.": "kitchen",
        "living room": "living",
        "none": "none",
        "unknown": "none",
        "hallway": None,
        "kitchen/bathroom": None,
        "none of the kitchen": None,
        "3rd room": None,
        "": None,
    }
    for raw, expected in cases.items():
        got = parse_answer(raw)
        if got != expected:
            failures.append(f"{raw!r} -> {got!r}, expected {expected!r}")
    if "none" not in PROMPT:
        failures.append("prompt has no none")
    yes_no_cases = {
        "yes": "yes",
        "No.": "no",
        "yeah": "yes",
        "nope": "no",
        "yes no": None,
        "maybe": None,
        "there is no kitchen": "no",
        "": None,
    }
    for raw, expected in yes_no_cases.items():
        got = parse_yes_no(raw)
        if got != expected:
            failures.append(f"yes/no {raw!r} -> {got!r}, expected {expected!r}")
    kitchen_q = yes_no_prompt("kitchen")
    if kitchen_q != "Is there a kitchen through the doorway ahead? Answer yes or no.":
        failures.append(f"kitchen question moved: {kitchen_q}")
    if "living room" not in yes_no_prompt("living"):
        failures.append("living question dropped living room")
    if MODEL_SHA != "5d6c926f44e26b07957b0dd315bbedcb4c17a5fe":
        failures.append("revision pin moved")
    if not WEIGHTS_SHA256.startswith("70a7d94c"):
        failures.append("weights pin moved")
    if failures:
        for message in failures:
            print(f"FAIL {message}")
        return 1
    print("[ask] self-test PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(self_test())
