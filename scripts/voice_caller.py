#!/usr/bin/env python3
"""Day-1 voice caller on the CommandBus in scripts/steer_walk.py.

A short English phrase becomes stand, stop, or vel(vx, yaw_rate). Latest
command wins. vel is resent at 10 Hz. Silence longer than 200 ms already
stands inside the bus. This file does not write that clock and does not
change the clamps.

Forward sends the bus forward cap, +0.08 m/s, into the gait. Controls'
sneak peek on this plant realized about +0.074 m/s and moved +0.64 m
upright. This caller does not claim faster than that. It is not a clean walk.
The gait is not a room crossing.

Back up is refused in one line: not a walk-back. The bus reverse cap is
0.032 m/s and their probe tipped. This caller does not send reverse.

Turn left and turn right send ±0.25 rad/s. That is a hip bias, not a verified spin.

Go to the kitchen, the bathroom, or any other room is refused in one
line: the camera can see, but there is no room and no map. No path is
invented. The floor in the clip is empty.

kit_cam is the plant camera on head_tilt_link. The clip renders that
camera. This file does not edit the plant or the gait.

Run:
  python scripts/voice_caller.py "walk forward"
  python scripts/voice_caller.py "back up"
  python scripts/voice_caller.py "go to the kitchen"
  python scripts/voice_caller.py --self-test
  MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal, Protocol

os.environ.setdefault("MUJOCO_GL", "osmesa")

ROOT = Path(__file__).resolve().parents[1]
PREVIEWS = ROOT / "previews"

# Controls' clamps on the merged bus. The self-test fails if steer_walk drifts.
FWD_MPS = 0.08
BACK_CAP_MPS = 0.032
YAW_RAD_S = 0.25
RESEND_S = 0.10
TIMEOUT_S = 0.200
PLANT_MD5 = "e3feef973d7ae1fb09748d13fdbcb4ed"
SNEAK_VX_MPS = 0.074
SNEAK_DX_M = 0.64

ROOM_LINE = "the camera can see, but there is no room and no map"
BACK_LINE = "not a walk-back"
VY_LINE = "refused: no vy"
DOOR_LINE = "refused: door is not a day-1 velocity command"
MAP_LINE = "refused: no map"
JOINT_LINE = "refused: joint targets are not a day-1 velocity command"
UNKNOWN_LINE = "refused: not a stand, stop, or vel phrase"
EMPTY_LINE = "refused: empty phrase"

Kind = Literal["stand", "stop", "vel", "refuse"]

_POLITE_PREFIXES = (
    "would you ",
    "could you ",
    "can you ",
    "let us ",
    "lets ",
    "please ",
)
_ROOM_WORDS = frozenset({
    "kitchen", "bathroom", "bedroom", "room", "rooms",
    "hallway", "hall", "office", "garage", "toilet", "bath",
    "lobby", "corridor", "stairs", "stair", "stairway",
    "dining", "living", "nursery", "closet", "pantry",
    "basement", "attic", "porch", "yard", "garden",
    "restroom", "washroom", "lounge", "den", "study",
    "laundry", "balcony", "patio", "foyer", "cellar",
    "lab", "laboratory", "classroom", "clinic", "ward",
    "library", "cafeteria", "gym",
})
_DOOR_WORDS = frozenset({"door", "doors", "doorway"})
_MAP_WORDS = frozenset({"map", "maps", "waypoint", "waypoints", "goal", "goals"})
_JOINT_WORDS = frozenset({
    "joint", "joints", "knee", "knees", "ankle", "ankles",
    "pose", "torque", "torques", "hip", "hips",
})
_VY_PHRASES = frozenset({
    "strafe", "strafe left", "strafe right",
    "sidestep", "sidestep left", "sidestep right",
    "side step", "side step left", "side step right",
    "sideways", "lateral", "vy",
    "move left", "move right",
    "step left", "step right",
    "slide left", "slide right",
})
_FORWARD_PHRASES = frozenset({
    "walk forward", "walk forwards",
    "go forward", "go forwards",
    "move forward", "move forwards",
    "forward", "forwards",
    "walk", "walk ahead", "go ahead", "ahead",
})
_BACK_PHRASES = frozenset({
    "back up", "backup", "back",
    "walk back", "go back", "move back",
    "reverse",
    "walk backward", "walk backwards",
    "go backward", "go backwards",
    "backward", "backwards",
})
_LEFT_PHRASES = frozenset({"turn left", "left", "yaw left", "go left"})
_RIGHT_PHRASES = frozenset({"turn right", "right", "yaw right", "go right"})
_STOP_PHRASES = frozenset({
    "stop", "halt", "freeze",
    "stop walking", "stop moving",
    "dont move", "do not move",
})
_STAND_PHRASES = frozenset({
    "stand", "stand up", "stand still",
    "stay", "stay still", "hold still",
})
_PLACE_BARE = frozenset({"navigate", "goto", "go to", "take me", "waypoint"})
_PLACE_PREFIXES = (
    "navigate to ",
    "take me to ",
    "bring me to ",
    "walk into ",
    "go into ",
    "walk to ",
    "head to ",
    "come to ",
    "move to ",
    "path to ",
    "go to ",
    "goto ",
    "navigate ",
    "take me ",
    "bring me ",
)
_DIRECTION_TAIL = {
    "left": "left",
    "the left": "left",
    "right": "right",
    "the right": "right",
    "forward": "forward",
    "forwards": "forward",
    "ahead": "forward",
    "the front": "forward",
    "back": "back",
    "backward": "back",
    "backwards": "back",
}


@dataclass(frozen=True)
class VoiceCommand:
    kind: Kind
    vx: float
    yaw_rate: float
    line: str


class CommandSink(Protocol):
    def stand(self, now: float) -> str | None: ...

    def stop(self, now: float) -> str | None: ...

    def vel(self, vx: float, yaw_rate: float, now: float) -> str | None: ...


class BusTick(Protocol):
    applied_vx: float
    applied_yaw_rate: float
    mode: str

    def line(self) -> str: ...


class ClockedBus(CommandSink, Protocol):
    def tick(self, now: float, dt: float) -> BusTick: ...


class SampleView(Protocol):
    t: float
    x: float
    body_vx: float
    applied_vx: float
    yaw: float
    mode: str
    up_z: float


def normalize_phrase(phrase: str) -> str:
    text = phrase.strip().lower()
    text = text.replace("\u2019", "").replace("'", "")
    text = re.sub(r"[^a-z0-9\s]+", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    changed = True
    while changed:
        changed = False
        for prefix in _POLITE_PREFIXES:
            if text.startswith(prefix):
                text = text[len(prefix):].strip()
                changed = True
        if text.endswith(" please"):
            text = text[: -len(" please")].strip()
            changed = True
    return text


def _refuse(line: str) -> VoiceCommand:
    return VoiceCommand("refuse", 0.0, 0.0, line)


def _forward() -> VoiceCommand:
    return VoiceCommand(
        "vel",
        FWD_MPS,
        0.0,
        (
            f"vel vx={FWD_MPS:+.3f} yaw_rate={0.0:+.3f} "
            "(command into the gait; sneak peek realized about "
            f"{SNEAK_VX_MPS:+.3f} m/s and {SNEAK_DX_M:+.2f} m upright; "
            "not a clean walk)"
        ),
    )


def _yaw(yaw_rate: float) -> VoiceCommand:
    return VoiceCommand(
        "vel",
        0.0,
        yaw_rate,
        (
            f"vel vx={0.0:+.3f} yaw_rate={yaw_rate:+.3f} "
            "(hip bias, not a verified spin)"
        ),
    )


def _stop() -> VoiceCommand:
    return VoiceCommand("stop", 0.0, 0.0, "stop")


def _stand() -> VoiceCommand:
    return VoiceCommand("stand", 0.0, 0.0, "stand")


def _from_direction(name: str) -> VoiceCommand:
    if name == "left":
        return _yaw(YAW_RAD_S)
    if name == "right":
        return _yaw(-YAW_RAD_S)
    if name == "forward":
        return _forward()
    if name == "back":
        return _refuse(BACK_LINE)
    return _refuse(UNKNOWN_LINE)


def _place_command(text: str) -> VoiceCommand | None:
    if text in _PLACE_BARE:
        return _refuse(ROOM_LINE)
    for prefix in _PLACE_PREFIXES:
        if not text.startswith(prefix):
            continue
        direction = _DIRECTION_TAIL.get(text[len(prefix):].strip())
        if direction is None:
            return _refuse(ROOM_LINE)
        return _from_direction(direction)
    return None


def parse_phrase(phrase: str) -> VoiceCommand:
    text = normalize_phrase(phrase)
    if not text:
        return _refuse(EMPTY_LINE)
    tokens = set(text.split())
    if text in _BACK_PHRASES:
        return _refuse(BACK_LINE)
    if tokens & _ROOM_WORDS:
        return _refuse(ROOM_LINE)
    if tokens & _DOOR_WORDS:
        return _refuse(DOOR_LINE)
    if tokens & _MAP_WORDS:
        return _refuse(MAP_LINE)
    if tokens & _JOINT_WORDS:
        return _refuse(JOINT_LINE)
    if text in _VY_PHRASES:
        return _refuse(VY_LINE)
    if text in _FORWARD_PHRASES:
        return _forward()
    if text in _LEFT_PHRASES:
        return _yaw(YAW_RAD_S)
    if text in _RIGHT_PHRASES:
        return _yaw(-YAW_RAD_S)
    if text in _STOP_PHRASES:
        return _stop()
    if text in _STAND_PHRASES:
        return _stand()
    placed = _place_command(text)
    if placed is not None:
        return placed
    return _refuse(UNKNOWN_LINE)


class VoiceCaller:
    """Publish stand, stop, or forward/yaw vel. Never sends reverse."""

    def __init__(self, bus: CommandSink, *, resend_s: float = RESEND_S) -> None:
        if not math.isfinite(resend_s) or resend_s <= 0.0 or resend_s >= TIMEOUT_S:
            raise ValueError("vel resend must be shorter than the 200 ms bus timeout")
        self.bus = bus
        self.resend_s = float(resend_s)
        self._command: VoiceCommand | None = None
        self._last_send = -1.0e9
        self._oneshot_sent = False

    def hear(self, phrase: str, now: float) -> str:
        command = parse_phrase(phrase)
        if command.kind == "refuse":
            return command.line
        if command.vx < 0.0:
            return BACK_LINE
        self._command = command
        self._oneshot_sent = False
        self._last_send = -1.0e9
        refusal = self.publish(now)
        if refusal:
            return refusal
        return command.line

    def publish(self, now: float) -> str | None:
        command = self._command
        if command is None:
            return None
        if command.kind == "stand":
            if self._oneshot_sent:
                return None
            self._oneshot_sent = True
            self._last_send = now
            return self.bus.stand(now)
        if command.kind == "stop":
            if self._oneshot_sent:
                return None
            self._oneshot_sent = True
            self._last_send = now
            return self.bus.stop(now)
        if command.kind != "vel" or command.vx < 0.0:
            return BACK_LINE if command.vx < 0.0 else None
        if command.vx > FWD_MPS or abs(command.yaw_rate) > YAW_RAD_S:
            return "refused: above the day-1 cap"
        if (now - self._last_send) < (self.resend_s - 1e-9):
            return None
        self._last_send = now
        return self.bus.vel(command.vx, command.yaw_rate, now)


def drive(
    caller: VoiceCaller,
    bus: ClockedBus,
    dt: float,
    phrase: str,
    t0: float,
    hold: float,
) -> float:
    command = parse_phrase(phrase)
    print(caller.hear(phrase, t0))
    steps = max(1, int(round(hold / dt)))
    report: BusTick | None = None
    for i in range(steps):
        t = t0 + i * dt
        if i > 0:
            caller.publish(t)
        report = bus.tick(t, dt)
    if command.kind != "refuse" and report is not None:
        print(report.line() + "  (bus command, not a clean-walk claim)")
    return t0 + steps * dt


@dataclass(frozen=True)
class PhraseCue:
    t_end: float
    phrase: str | None
    label: str


# Short typed steer. Forward is a few seconds, not a room crossing.
CLIP_CUES: tuple[PhraseCue, ...] = (
    PhraseCue(1.0, None, "quiet"),
    PhraseCue(5.0, "walk forward", "forward"),
    PhraseCue(8.0, "turn left", "turn"),
    PhraseCue(9.0, "stop", "stop"),
)
CLIP_END = CLIP_CUES[-1].t_end
STILL_TIMES = (
    (3.5, "forward", "voice_caller_kit_cam_forward.png"),
    (6.5, "turn", "voice_caller_kit_cam_turn.png"),
    (8.5, "stop", "voice_caller_kit_cam_stop.png"),
)


def _cue_index(now: float, cues: tuple[PhraseCue, ...] = CLIP_CUES) -> int:
    for index, cue in enumerate(cues):
        if now < cue.t_end - 1e-9:
            return index
    return len(cues) - 1


def steer_load_error() -> str | None:
    scripts = str(Path(__file__).resolve().parent)
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    try:
        import steer_walk as steer
    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
        print(f"[voice] CommandBus unavailable: {message}", file=sys.stderr)
        return message
    del steer
    return None


def _cap_problems() -> list[str]:
    import steer_walk as steer

    problems: list[str] = []
    if steer.VX_FWD_CAP != FWD_MPS:
        problems.append(f"VX_FWD_CAP {steer.VX_FWD_CAP} != {FWD_MPS}")
    if steer.VX_BACK_CAP != BACK_CAP_MPS:
        problems.append(f"VX_BACK_CAP {steer.VX_BACK_CAP} != {BACK_CAP_MPS}")
    if steer.YAW_RATE_CAP != YAW_RAD_S:
        problems.append(f"YAW_RATE_CAP {steer.YAW_RATE_CAP} != {YAW_RAD_S}")
    if steer.VEL_RESEND_S != RESEND_S:
        problems.append(f"VEL_RESEND_S {steer.VEL_RESEND_S} != {RESEND_S}")
    if abs(steer.COMMAND_TIMEOUT_S - TIMEOUT_S) > 1e-12:
        problems.append(f"COMMAND_TIMEOUT_S {steer.COMMAND_TIMEOUT_S} != {TIMEOUT_S}")
    if steer.PLANT_MD5 != PLANT_MD5:
        problems.append(f"PLANT_MD5 {steer.PLANT_MD5} != {PLANT_MD5}")
    if not (0.0 < steer.VEL_RESEND_S < steer.COMMAND_TIMEOUT_S):
        problems.append("vel resend is not inside the 200 ms watchdog")
    return problems


def _expect(cond: bool, msg: str, failures: list[str]) -> None:
    if not cond:
        failures.append(msg)


def test_phrases() -> list[str]:
    failures: list[str] = []
    doc = __doc__ or ""
    _expect("not a walk-back" in doc, "module doc omits the reverse refusal", failures)
    _expect("not a verified spin" in doc, "module doc omits the yaw limit", failures)
    _expect("not a clean walk" in doc, "module doc calls it a clean walk", failures)
    _expect("no room and no map" in doc, "module doc invents a map", failures)
    _expect(
        set(VoiceCommand.__dataclass_fields__) == {"kind", "vx", "yaw_rate", "line"},
        "voice command grew a path or joint field",
        failures,
    )
    table: tuple[tuple[str, Kind, float, float], ...] = (
        ("walk forward", "vel", FWD_MPS, 0.0),
        ("Walk Forward!", "vel", FWD_MPS, 0.0),
        ("please walk forward", "vel", FWD_MPS, 0.0),
        ("turn left", "vel", 0.0, YAW_RAD_S),
        ("turn right", "vel", 0.0, -YAW_RAD_S),
        ("stop", "stop", 0.0, 0.0),
        ("stand", "stand", 0.0, 0.0),
        ("go to the left", "vel", 0.0, YAW_RAD_S),
    )
    for phrase, kind, vx, yaw in table:
        command = parse_phrase(phrase)
        _expect(command.kind == kind, f"{phrase!r} kind {command.kind}", failures)
        _expect(
            command.vx == vx and command.yaw_rate == yaw,
            f"{phrase!r} maps to vx={command.vx} yaw={command.yaw_rate}",
            failures,
        )
        _expect(command.vx <= FWD_MPS and command.vx >= 0.0, f"{phrase!r} sent reverse or over-speed", failures)
    forward = parse_phrase("walk forward")
    _expect("not a clean walk" in forward.line, "forward line claims a clean walk", failures)
    _expect("+0.074" in forward.line and "+0.64" in forward.line, "forward line omits the sneak peek", failures)
    _expect(forward.vx == FWD_MPS, "forward is not the bus cap", failures)
    left = parse_phrase("turn left")
    _expect("hip bias" in left.line and "not a verified spin" in left.line, "left line claims a spin", failures)
    for phrase in ("back up", "walk back", "reverse", "go back", "Back up."):
        command = parse_phrase(phrase)
        _expect(command.kind == "refuse" and command.line == BACK_LINE, f"{phrase!r} -> {command.line!r}", failures)
        _expect(command.vx == 0.0 and command.yaw_rate == 0.0, f"{phrase!r} carried a velocity", failures)
        _expect("\n" not in command.line, f"{phrase!r} is not one line", failures)
    for phrase in (
        "go to the kitchen",
        "go to the bathroom",
        "Go to the bedroom.",
        "walk to the living room",
        "take me to the kitchen",
    ):
        command = parse_phrase(phrase)
        _expect(command.kind == "refuse" and command.line == ROOM_LINE, f"{phrase!r} -> {command.line!r}", failures)
        _expect(command.vx == 0.0 and command.yaw_rate == 0.0, f"{phrase!r} carried a velocity", failures)
    phrases = [cue.phrase for cue in CLIP_CUES if cue.phrase is not None]
    _expect(phrases == ["walk forward", "turn left", "stop"], f"clip phrases {phrases}", failures)
    return failures


class _CountBus:
    def __init__(self, inner: ClockedBus) -> None:
        self.inner = inner
        self.vel_n = 0
        self.stop_n = 0
        self.stand_n = 0
        self.vxs: list[float] = []

    def stand(self, now: float) -> str | None:
        self.stand_n += 1
        return self.inner.stand(now)

    def stop(self, now: float) -> str | None:
        self.stop_n += 1
        return self.inner.stop(now)

    def vel(self, vx: float, yaw_rate: float, now: float) -> str | None:
        self.vel_n += 1
        self.vxs.append(vx)
        return self.inner.vel(vx, yaw_rate, now)

    def tick(self, now: float, dt: float) -> BusTick:
        return self.inner.tick(now, dt)


def test_bus() -> list[str]:
    """Phrase → CommandBus. No plant step and no walk claim."""
    import steer_walk as steer

    failures = _cap_problems()
    if failures:
        return failures
    dt = steer.CTRL_DT
    fwd_cap = steer.VX_FWD_CAP
    yaw_cap = steer.YAW_RATE_CAP
    timeout = steer.COMMAND_TIMEOUT_S

    bus = steer.CommandBus()
    count = _CountBus(bus)
    caller = VoiceCaller(count, resend_s=steer.VEL_RESEND_S)
    line = caller.hear("go to the kitchen", 0.0)
    _expect(line == ROOM_LINE, f"kitchen line {line!r}", failures)
    _expect(count.vel_n == 0, "kitchen phrase called vel", failures)
    report = bus.tick(0.0, dt)
    _expect(report.mode == "stand" and report.applied_vx == 0.0, "kitchen phrase moved the bus", failures)

    back = caller.hear("back up", 0.0)
    caller.hear("reverse", 0.05)
    _expect(back == BACK_LINE, f"back up line {back!r}", failures)
    _expect(count.vel_n == 0 and bus.target_vx == 0.0, "reverse was sent", failures)

    caller.hear("walk forward", 0.0)
    _expect(bus.target_vx == fwd_cap and bus.target_yaw == 0.0, "walk forward target", failures)
    _expect(count.vel_n == 1 and count.vxs == [fwd_cap], "forward vel was not the cap", failures)
    for i in range(1, 18):
        t = i * dt
        if i == 4:
            _expect(bus.last_cmd_time == 0.0, "stamp moved inside the resend gap", failures)
            _expect(count.vel_n == 1, "vel resent before 100 ms", failures)
        caller.publish(t)
        bus.tick(t, dt)
    _expect(count.vel_n == 4, f"10 Hz resend count {count.vel_n}", failures)
    _expect(all(vx >= 0.0 and vx <= fwd_cap for vx in count.vxs), "a resend left the forward cap", failures)
    _expect(bus.mode == "move" and bus.applied_vx <= fwd_cap + 1e-12, "applied vx above cap", failures)

    last = float(bus.last_cmd_time)
    i = 18
    while i * dt <= last + timeout + dt + 1e-12:
        bus.tick(i * dt, dt)
        i += 1
    _expect(
        bus.mode == "stand" and bus.applied_vx == 0.0 and bus.applied_yaw_rate == 0.0,
        "silence did not stand",
        failures,
    )
    _expect(count.vel_n == 4, "watchdog path sent another vel", failures)

    fresh = steer.CommandBus()
    holder = VoiceCaller(_CountBus(fresh), resend_s=steer.VEL_RESEND_S)
    holder.hear("walk forward", 0.0)
    steps = int(round(1.2 / dt))
    saturated = fresh.tick(0.0, dt)
    for i in range(1, steps):
        t = i * dt
        holder.publish(t)
        saturated = fresh.tick(t, dt)
    _expect(
        abs(saturated.applied_vx - fwd_cap) < 1e-6 and saturated.applied_vx <= fwd_cap + 1e-12,
        f"command did not sit on +0.08: {saturated.line()}",
        failures,
    )

    latest = steer.CommandBus()
    latest_count = _CountBus(latest)
    latest_caller = VoiceCaller(latest_count, resend_s=steer.VEL_RESEND_S)
    latest_caller.hear("walk forward", 0.0)
    latest_caller.hear("turn left", 0.4)
    _expect(latest.target_vx == 0.0 and latest.target_yaw == yaw_cap, "turn left did not replace forward", failures)
    latest_caller.hear("turn right", 0.5)
    _expect(latest.target_yaw == -yaw_cap and latest.target_vx == 0.0, "turn right sign", failures)
    latest_caller.hear("back up", 0.55)
    _expect(latest.target_vx == 0.0 and latest.target_yaw == -yaw_cap, "back up replaced the yaw command", failures)
    latest_caller.hear("stop", 0.6)
    stopped = latest.tick(0.6, dt)
    _expect(
        stopped.mode == "stand" and stopped.applied_vx == 0.0 and stopped.applied_yaw_rate == 0.0,
        "stop did not zero the same tick",
        failures,
    )
    vel_at_stop = latest_count.vel_n
    stamp = float(latest.last_cmd_time)
    for i in range(1, 8):
        latest_caller.publish(0.6 + i * dt)
        stopped = latest.tick(0.6 + i * dt, dt)
    _expect(latest.last_cmd_time == stamp, "stop was resent inside the watchdog window", failures)
    for i in range(8, 20):
        latest_caller.publish(0.6 + i * dt)
        stopped = latest.tick(0.6 + i * dt, dt)
    _expect(latest_count.vel_n == vel_at_stop and latest_count.stop_n == 1, "stop did not stick", failures)
    _expect(stopped.mode == "stand", "post-stop tick left stand", failures)
    _expect(all(vx >= 0.0 for vx in latest_count.vxs), "a negative vx was sent", failures)
    return failures


def self_test() -> int:
    failures = test_phrases()
    if steer_load_error() is not None:
        if failures:
            for msg in failures:
                print(f"FAIL {msg}")
            return 1
        print("[voice] self-test PASS (mapping only; CommandBus not loaded; no walk claim)")
        return 0
    failures.extend(test_bus())
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[voice] self-test PASS (phrase → stand/stop/vel; no reverse; no walk claim)")
    return 0


@dataclass(frozen=True)
class ClipResult:
    plant_md5: str
    mujoco_gl: str
    phrases: tuple[str, ...]
    fault: bool
    fault_reason: str
    command_vx_cap: float
    mean_applied_vx_settled: float | None
    mean_body_vx_settled: float | None
    dx_forward_m: float | None
    dyaw_turn_deg: float | None
    min_up_z: float | None
    end_mode: str
    stills: tuple[str, ...]
    mp4: str | None
    honesty: str


def _between(samples: Sequence[SampleView], t0: float, t1: float) -> list[SampleView]:
    return [sample for sample in samples if t0 - 1e-9 <= sample.t < t1 - 1e-9]


def _mean(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return float(sum(values) / len(values))


def _fmt(value: float | None, spec: str) -> str:
    if value is None:
        return "n/a"
    return format(value, spec)


def _honesty(result: ClipResult) -> str:
    text = (
        "kit_cam during typed phrases walk forward, then turn left, then stop. "
        f"Forward command into the gait is {result.command_vx_cap:+.3f} m/s. "
        "Mean applied_vx after slew is "
        f"{_fmt(result.mean_applied_vx_settled, '+.3f')} m/s. "
        "Mean body vx in that window is "
        f"{_fmt(result.mean_body_vx_settled, '+.3f')} m/s. "
        f"Forward Δx={_fmt(result.dx_forward_m, '+.3f')} m. "
        f"Controls' sneak peek realized about {SNEAK_VX_MPS:+.3f} m/s and "
        f"{SNEAK_DX_M:+.2f} m upright. This clip does not claim faster than that, "
        "and it is not a clean walk. "
        f"Turn left sent yaw_rate {YAW_RAD_S:+.2f} rad/s, a hip bias, "
        "not a verified spin. Measured heading change "
        f"{_fmt(result.dyaw_turn_deg, '+.1f')} deg. "
        "Reverse was not sent. A back-up phrase is refused: not a walk-back. "
        "The camera can see, but there is no room and no map. "
        "The floor is empty. This gait is not a room crossing. "
        "Not autonomous navigation."
    )
    if result.fault:
        text += f" FAULT: {result.fault_reason}."
    return text


def _overlay(phrase: str, label: str, report_line: str, now: float, x: float) -> list[str]:
    if label == "forward":
        note = "cmd +0.080  sneak peek ~0.074  not a clean walk"
    elif label == "turn":
        note = "yaw +0.25 hip bias, not a verified spin"
    else:
        note = "stand/stop  |  camera sees, no room, no map"
    return [
        f'kit_cam "{phrase}"  empty floor',
        report_line,
        note,
        f"t={now:.2f}s  x={x:+.3f}",
    ]


def _save_png(frame: object, path: Path) -> None:
    from PIL import Image
    import numpy as np

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.asarray(frame)).save(path)


def run_clip(out_mp4: Path, summary_path: Path) -> int:
    """Render kit_cam while the typed phrases drive the existing steer."""
    if steer_load_error() is not None:
        print("[voice] clip skipped: MuJoCo/CommandBus did not load", file=sys.stderr)
        return 2
    import numpy as np

    import steer_walk as steer

    problems = _cap_problems()
    if problems:
        print("[voice] refused: " + "; ".join(problems), file=sys.stderr)
        return 2
    try:
        session = steer.SteerSession(video=True)
    except SystemExit:
        raise
    except Exception as exc:
        print(f"[voice] kit_cam renderer failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    if session.renderer is None:
        print("[voice] kit_cam renderer was not created", file=sys.stderr)
        return 2

    caller = VoiceCaller(session.bus, resend_s=steer.VEL_RESEND_S)
    dt = steer.CTRL_DT
    n_ctrl = int(round(CLIP_END / dt))
    frames: list[np.ndarray] = []
    stills: list[str] = []
    saved: set[str] = set()
    active = -1
    last_print = -1.0
    fault_announced = False
    print(
        f"[voice] kit_cam phrases: walk forward, turn left, stop  "
        f"md5={steer.PLANT_MD5} gl={os.environ.get('MUJOCO_GL', '')}"
    )
    for _ in range(n_ctrl):
        now = float(session.data.time)
        index = _cue_index(now)
        cue = CLIP_CUES[index]
        if index != active:
            active = index
            if cue.phrase is None:
                print(f"t={now:.2f} quiet")
            else:
                print(f"t={now:.2f} {caller.hear(cue.phrase, now)}")
        else:
            refusal = caller.publish(now)
            if refusal:
                print(refusal)
        report = session.step()
        if report.mode == "fault" and not fault_announced:
            print(f"fault: {session.bus.fault_reason}")
            fault_announced = True
        if (now - last_print) >= (steer.VEL_RESEND_S - 1e-9):
            print(f"t={now:.2f} {cue.label} {report.line()}")
            last_print = now
        if len(session.samples) % 2 != 0:
            continue
        shown = cue.phrase if cue.phrase is not None else "(quiet)"
        lines = _overlay(shown, cue.label, report.line(), now, float(session.data.qpos[0]))
        # No mj_forward here. A second forward tips this gait.
        session.renderer.update_scene(session.data, camera="kit_cam")
        raw = np.ascontiguousarray(session.renderer.render().copy(), dtype=np.uint8)
        frame = steer.wg._burn_overlay(raw, lines)
        frames.append(frame)
        for still_t, still_label, name in STILL_TIMES:
            if name in saved or cue.label != still_label or now + 1e-9 < still_t:
                continue
            path = PREVIEWS / name
            _save_png(frame, path)
            saved.add(name)
            stills.append(str(path.relative_to(ROOT)))
            print(f"[voice] wrote {path}")
    session.assert_plant_unchanged()
    mp4_path: Path | None = None
    render_error = ""
    if frames:
        try:
            steer._write_mp4(frames, out_mp4)
            mp4_path = out_mp4
        except Exception as exc:
            render_error = f"{type(exc).__name__}: {exc}"
            print(f"[voice] mp4 failed: {render_error}", file=sys.stderr)
    result = _measure(session.samples, session.bus.fault, session.bus.fault_reason, stills, mp4_path)
    if render_error:
        result = ClipResult(
            plant_md5=result.plant_md5,
            mujoco_gl=result.mujoco_gl,
            phrases=result.phrases,
            fault=result.fault,
            fault_reason=result.fault_reason,
            command_vx_cap=result.command_vx_cap,
            mean_applied_vx_settled=result.mean_applied_vx_settled,
            mean_body_vx_settled=result.mean_body_vx_settled,
            dx_forward_m=result.dx_forward_m,
            dyaw_turn_deg=result.dyaw_turn_deg,
            min_up_z=result.min_up_z,
            end_mode=result.end_mode,
            stills=result.stills,
            mp4=None,
            honesty=result.honesty + f" RENDER: {render_error}.",
        )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(asdict(result), indent=2) + "\n", encoding="utf-8")
    print("[voice] " + result.honesty)
    print(f"[voice] wrote {summary_path}")
    if mp4_path is not None and render_error == "":
        print(f"[voice] wrote {mp4_path} ({len(frames)} frames)")
        return 0 if stills else 2
    print("[voice] kit_cam clip was not written", file=sys.stderr)
    return 2


def _measure(
    samples: Sequence[SampleView],
    fault: bool,
    fault_reason: str,
    stills: list[str],
    mp4: Path | None,
) -> ClipResult:
    quiet_end = CLIP_CUES[0].t_end
    forward_end = CLIP_CUES[1].t_end
    turn_end = CLIP_CUES[2].t_end
    settled = _between(samples, quiet_end + 1.0, forward_end)
    forward = _between(samples, quiet_end, forward_end)
    turn = _between(samples, forward_end, turn_end)
    dyaw: float | None = None
    if len(turn) >= 2:
        delta = turn[-1].yaw - turn[0].yaw
        dyaw = math.degrees((delta + math.pi) % (2.0 * math.pi) - math.pi)
    dx = (forward[-1].x - forward[0].x) if len(forward) >= 2 else None
    end = samples[-1] if samples else None
    ups = [sample.up_z for sample in samples]
    draft = ClipResult(
        plant_md5=PLANT_MD5,
        mujoco_gl=os.environ.get("MUJOCO_GL", ""),
        phrases=tuple(cue.phrase for cue in CLIP_CUES if cue.phrase is not None),
        fault=fault,
        fault_reason=fault_reason,
        command_vx_cap=FWD_MPS,
        mean_applied_vx_settled=_mean([sample.applied_vx for sample in settled]),
        mean_body_vx_settled=_mean([sample.body_vx for sample in settled]),
        dx_forward_m=dx,
        dyaw_turn_deg=dyaw,
        min_up_z=min(ups) if ups else None,
        end_mode=end.mode if end is not None else "",
        stills=tuple(stills),
        mp4=str(mp4.relative_to(ROOT)) if mp4 is not None else None,
        honesty="",
    )
    return ClipResult(
        plant_md5=draft.plant_md5,
        mujoco_gl=draft.mujoco_gl,
        phrases=draft.phrases,
        fault=draft.fault,
        fault_reason=draft.fault_reason,
        command_vx_cap=draft.command_vx_cap,
        mean_applied_vx_settled=draft.mean_applied_vx_settled,
        mean_body_vx_settled=draft.mean_body_vx_settled,
        dx_forward_m=draft.dx_forward_m,
        dyaw_turn_deg=draft.dyaw_turn_deg,
        min_up_z=draft.min_up_z,
        end_mode=draft.end_mode,
        stills=draft.stills,
        mp4=draft.mp4,
        honesty=_honesty(draft),
    )


def _run_phrases(phrases: list[str], hold: float) -> int:
    if steer_load_error() is not None:
        for phrase in phrases:
            print(parse_phrase(phrase).line)
        print("[voice] CommandBus not loaded; phrases were not sent", file=sys.stderr)
        return 0
    import steer_walk as steer

    problems = _cap_problems()
    if problems:
        print("[voice] refused: " + "; ".join(problems), file=sys.stderr)
        return 2
    bus = steer.CommandBus()
    caller = VoiceCaller(bus, resend_s=steer.VEL_RESEND_S)
    t = 0.0
    for phrase in phrases:
        t = drive(caller, bus, steer.CTRL_DT, phrase, t, hold)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Day-1 voice caller: a phrase becomes stand, stop, or vel on CommandBus",
    )
    parser.add_argument("phrases", nargs="*", help="Phrases, in order. Latest command wins.")
    parser.add_argument("--self-test", action="store_true", help="Map phrases on the bus. No walk claim.")
    parser.add_argument(
        "--clip",
        action="store_true",
        help="Render kit_cam for walk forward, turn left, then stop",
    )
    parser.add_argument("--out", default=str(PREVIEWS / "voice_caller_kit_cam.mp4"))
    parser.add_argument("--summary", default=str(PREVIEWS / "voice_caller_day1_summary.json"))
    parser.add_argument("--hold", type=float, default=1.2, help="Seconds to hold each phrase on the bus clock")
    args = parser.parse_args(argv)
    if args.hold <= 0.0 or not math.isfinite(args.hold):
        print("refused: --hold must be a positive number of seconds", file=sys.stderr)
        return 2
    if args.self_test:
        return self_test()
    if args.clip:
        if args.phrases:
            print("refused: --clip plays walk forward, turn left, stop", file=sys.stderr)
            return 2
        return run_clip(Path(args.out), Path(args.summary))
    if args.phrases:
        return _run_phrases(list(args.phrases), float(args.hold))
    if not sys.stdin.isatty():
        piped = [line.strip() for line in sys.stdin if line.strip()]
        if piped:
            return _run_phrases(piped, float(args.hold))
    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
