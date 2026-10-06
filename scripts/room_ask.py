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

    def ask(self, frame) -> dict[str, object]:
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
            result = self.model.query(image, PROMPT)
        finally:
            if original is not None:
                inner._prefill_prompt = original
            elapsed = time.perf_counter() - started
        raw = str(result.get("answer", result) if isinstance(result, dict) else result)
        room = parse_answer(raw)
        # First answer token from the prompt prefill. A hedge keeps the
        # probability for the log and still returns no room.
        confidence = captured[-1] if captured else None
        return {
            "room": room,
            "confidence": confidence,
            "raw": raw.strip(),
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
