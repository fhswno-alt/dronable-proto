#!/usr/bin/env python3
"""Voice phrases on the existing CommandBus. No second controller.

A typed phrase — the text a speech front-end would hand over — becomes
stand, stop, or vel(vx, yaw_rate) on scripts/steer_walk.py CommandBus.
This file does not edit the plant, the gait, the tip check, or the bus.
It does not move kit_cam. It does not add waypoints, a map, or strafe.

Caps stay the bus caps: vx +0.056 / −0.032 m/s, yaw ±0.25 rad/s.
vel is resent at 10 Hz. 200 ms with no command stands inside the bus.

Walk forward publishes vel(+0.056, 0). A turn publishes walk-yaw at the
forward cap, vel(+0.056, ±0.25). vx=0 with a yaw command does not change
heading here, so "turn in place" is refused and is not published.

The clip is Controls' claimed nav-left window, spoken as phrases:
stand, walk forward, turn left, walk forward, stop. Same times, same
vel(+0.056, 0) and vel(+0.056, +0.25). It is not a second arc and not
a room crossing. Half-cap vel(+0.028, yaw) is the finder trim, not this arc.

Locked-kit resume on the same window shape (Prefer FAIL), commanded as
vel(+0.150, …) on --bus-kit, not by the phrases in this file. Scenario:
1 s stand, 15 s forward, 12.5 s vel(+0.150, +0.25), then 6 s
vel(+0.150, 0). Resume Δyaw ≈ +12.41°. Plant md5
207f3d5e9c6a72e16f7aa0c8d224f75e unchanged. Split: (1) 0–0.68 s
applied_yaw still slewing +0.25→0 at 0.40 rad/s² after the 100 ms
resend clears the target → body +6.81° (command integral ~+5.22°);
(2) 0.68–6.0 s applied_yaw and the step angle are already 0 → leftover
left curve +5.60° (body yaw rate +0.034→+0.009 rad/s). That is ~half
ramp-out, ~half steady leftover curve under vel(+0.150, 0) — not
“turn still commanded.” Soft-pass is off.

The same 6 s vel(+0.150, 0) after an 11 s right turn is −2.2° chained
and −0.6° from a straight approach. The slew is 0.40 rad/s² both ways.
On a matching step phase the right ramp is −5.3° of body yaw (command
integral −4.5°) and the rest of the window curves left +4.7° with
applied_yaw and the step angle already 0, so they cancel. Straight
vel(+0.150, 0) already curves +4.2° over 27–33 s and +3.5° over
28.5–34.5 s. Hip-yaw targets are 0 in that stretch. Stop snaps yaw to 0
in one tick; after a right turn the heading kick runs about +11° to
−10° across 0.37 s of step phase. Five cold straight walks net +0.50°
over 30 s and the cold start still swings the left foot first. After
a turn the straight resume swings the outside foot, and the pose
chases the live gait over that double support instead of stepping
0.342 rad in one tick. Post-left 6 s is +1.4°, post-right stays
−2.0°. The left turn finishes at +156.4° in 12.5 s; the right turn
finishes at −148.6° in 11.0 s (7.8° shorter because the hold is
shorter). Hip roll while yawing stays at or under 2.33 Nm. First right-lead step knees after stand are 1.749 / 1.152 Nm,
under 2.33. This file still publishes the 0.056 phrases.

Go to the kitchen, the bathroom, anywhere, SLAM, a map, a waypoint, or
a strafe is refused. Kitchen and bathroom finders are other scripts.
Tonight is voice → bus motion only.

Run:
  python scripts/voice_caller.py "turn left"
  python scripts/voice_caller.py --self-test
  MUJOCO_GL=osmesa python scripts/voice_caller.py --measure
  MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys
from collections.abc import Sequence
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Literal, Protocol

import numpy as np

os.environ.setdefault("MUJOCO_GL", "osmesa")

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

ROOT = Path(__file__).resolve().parents[1]
PREVIEWS = ROOT / "previews"
PLANT_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"

# Locked to Controls' bus. self-test refuses if steer_walk drifts.
FWD_MPS = 0.056
BACK_MPS = 0.032
YAW_RAD_S = 0.25
# Half the forward cap. The finder uses this as a trim. It is inside the
# caps, and it is not the demo turn: heading barely accumulates.
SOFT_VX = 0.028
RESEND_S = 0.10
TIMEOUT_S = 0.200
PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
KIT_CAM_POS = (0.050, 0.019, 0.007)
KIT_CAM_POS_TEXT = "0.050 0.019 0.007"

ROOM_LINE = (
    "refused: voice does not go to a room; kitchen and bathroom finders "
    "are separate, and this caller has no map"
)
MAP_LINE = "refused: no map, no SLAM, and no go-anywhere"
VY_LINE = "refused: no vy and no strafe on this bus"
DOOR_LINE = "refused: door is not a day-1 velocity command"
JOINT_LINE = "refused: joint targets are not a day-1 velocity command"
IN_PLACE_LINE = (
    "refused: vx=0 yaw does not change heading on this plant; "
    "say turn left or turn right"
)
TOO_FAST_LINE = "refused: forward cap is +0.056 m/s; 0.080 tips"
UNKNOWN_LINE = "refused: not a stand, stop, or vel phrase"
EMPTY_LINE = "refused: empty phrase"
CAP_LINE = "refused: above the day-1 cap"

Kind = Literal["stand", "stop", "vel", "refuse"]

_POLITE_PREFIXES = (
    "would you ",
    "could you ",
    "can you ",
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
})
_DOOR_WORDS = frozenset({"door", "doors", "doorway"})
_MAP_WORDS = frozenset({
    "map", "maps", "waypoint", "waypoints", "slam", "explore",
})
_JOINT_WORDS = frozenset({
    "joint", "joints", "knee", "knees", "ankle", "ankles",
    "pose", "torque", "torques", "hip", "hips",
})
_STAND_PHRASES = frozenset({"stand", "stand up", "stand still"})
_STOP_PHRASES = frozenset({
    "stop", "halt", "freeze", "stop walking", "stop moving",
})
_FORWARD_PHRASES = frozenset({
    "walk forward", "walk forwards",
    "go forward", "go forwards",
    "move forward", "move forwards",
    "forward", "forwards", "walk",
})
_BACK_PHRASES = frozenset({
    "back up", "backup", "reverse",
    "walk backward", "walk backwards",
    "go backward", "go backwards",
    "backward", "backwards",
})
_LEFT_PHRASES = frozenset({
    "turn left", "go left", "yaw left", "left",
})
_RIGHT_PHRASES = frozenset({
    "turn right", "go right", "yaw right", "right",
})
_IN_PLACE_PHRASES = frozenset({
    "turn left in place", "turn right in place",
    "turn in place", "spin", "spin left", "spin right",
    "turn around", "yaw in place",
})
_VY_PHRASES = frozenset({
    "strafe", "strafe left", "strafe right",
    "sidestep", "sidestep left", "sidestep right",
    "side step", "side step left", "side step right",
    "move left", "move right", "walk left", "walk right",
    "slide left", "slide right", "sideways",
})
_TOO_FAST_PHRASES = frozenset({
    "walk faster", "go faster", "faster", "speed up",
    "full speed", "sprint",
})
_ANYWHERE_PHRASES = frozenset({
    "go anywhere", "go everywhere", "go somewhere",
})


@dataclass(frozen=True)
class VoiceCommand:
    kind: Kind
    vx: float
    yaw_rate: float
    line: str


@dataclass(frozen=True)
class Cue:
    """Phrase held until t_end. The next cue replaces it."""

    t_end: float
    phrase: str
    label: str


# Claimed nav-left window. Phrase times match NAV_LEFT_SCRIPT t_end.
# stand → walk forward → turn left → walk forward → stop.
# Not the 14 s left that starts at 15 s. Not a second arc.
DEMO_CUES: tuple[Cue, ...] = (
    Cue(1.0, "stand", "stand"),
    Cue(16.0, "walk forward", "walk"),
    Cue(28.5, "turn left", "turn"),
    Cue(34.5, "walk forward", "resume"),
    Cue(37.0, "stop", "stop"),
)

STILLS: tuple[tuple[float, str, str], ...] = (
    (0.80, "stand", "voice_commands_stand.png"),
    (9.25, "walk", "voice_commands_walk.png"),
    (22.90, "turn", "voice_commands_turn.png"),
    (31.80, "resume", "voice_commands_resume.png"),
    (36.60, "stop", "voice_commands_stop.png"),
)


def bus_text(kind: str, vx: float, yaw_rate: float) -> str:
    if kind == "vel":
        return f"vel({vx:+.3f}, {yaw_rate:+.3f})"
    return kind


def normalize_phrase(phrase: str) -> str:
    text = phrase.strip().lower().replace("\u2019", "").replace("'", "")
    text = re.sub(r"[^a-z0-9.\s]+", " ", text)
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


def _vel(vx: float, yaw_rate: float) -> VoiceCommand:
    return VoiceCommand("vel", vx, yaw_rate, bus_text("vel", vx, yaw_rate))


def parse_phrase(phrase: str) -> VoiceCommand:
    """Map one phrase to a bus command. A miss does not invent a path."""
    text = normalize_phrase(phrase)
    if not text:
        return _refuse(EMPTY_LINE)
    if "0.080" in text or "0.08" in text or text in _TOO_FAST_PHRASES:
        return _refuse(TOO_FAST_LINE)
    if text in _IN_PLACE_PHRASES:
        return _refuse(IN_PLACE_LINE)
    if text in _VY_PHRASES:
        return _refuse(VY_LINE)
    if text in _ANYWHERE_PHRASES:
        return _refuse(MAP_LINE)
    if text in _BACK_PHRASES:
        return _vel(-BACK_MPS, 0.0)
    if text in _FORWARD_PHRASES:
        return _vel(FWD_MPS, 0.0)
    if text in _LEFT_PHRASES:
        return _vel(FWD_MPS, YAW_RAD_S)
    if text in _RIGHT_PHRASES:
        return _vel(FWD_MPS, -YAW_RAD_S)
    if text in _STOP_PHRASES:
        return VoiceCommand("stop", 0.0, 0.0, "stop")
    if text in _STAND_PHRASES:
        return VoiceCommand("stand", 0.0, 0.0, "stand")
    tokens = set(text.split())
    if tokens & _ROOM_WORDS or text.startswith("go to "):
        return _refuse(ROOM_LINE)
    if tokens & _DOOR_WORDS:
        return _refuse(DOOR_LINE)
    if tokens & _MAP_WORDS:
        return _refuse(MAP_LINE)
    if tokens & _JOINT_WORDS:
        return _refuse(JOINT_LINE)
    return _refuse(UNKNOWN_LINE)


class CommandSink(Protocol):
    """The three methods this caller is allowed to use on CommandBus."""

    def stand(self, now: float) -> str | None: ...

    def stop(self, now: float) -> str | None: ...

    def vel(self, vx: float, yaw_rate: float, now: float) -> str | None: ...


class _Sample(Protocol):
    t: float
    x: float
    y: float
    yaw: float
    up_z: float
    margin: float
    mode: str
    body_vx: float


class VoiceCaller:
    """Publish the latest phrase. vel is resent until the phrase changes."""

    def __init__(self, bus: CommandSink, *, resend_s: float = RESEND_S) -> None:
        if not math.isfinite(resend_s) or resend_s <= 0.0 or resend_s >= TIMEOUT_S:
            raise ValueError("vel resend must be shorter than the 200 ms bus timeout")
        self.bus = bus
        self.resend_s = float(resend_s)
        self.command: VoiceCommand | None = None
        self._last_send = -1.0e9
        self._oneshot_sent = False

    def hear(self, phrase: str, now: float) -> str:
        command = parse_phrase(phrase)
        if command.kind == "refuse":
            return command.line
        self.command = command
        self._oneshot_sent = False
        self._last_send = -1.0e9
        refusal = self.publish(now)
        if refusal:
            return refusal
        return command.line

    def publish(self, now: float) -> str | None:
        command = self.command
        if command is None or command.kind == "refuse":
            return None
        if command.kind == "stand":
            return self._once_stand(now)
        if command.kind == "stop":
            return self._once_stop(now)
        if command.vx > FWD_MPS or command.vx < -BACK_MPS:
            return CAP_LINE
        if abs(command.yaw_rate) > YAW_RAD_S:
            return CAP_LINE
        if command.yaw_rate != 0.0 and command.vx == 0.0:
            return IN_PLACE_LINE
        if (now - self._last_send) < (self.resend_s - 1e-9):
            return None
        self._last_send = now
        return self.bus.vel(command.vx, command.yaw_rate, now)

    def _once_stand(self, now: float) -> str | None:
        if self._oneshot_sent:
            return None
        self._oneshot_sent = True
        self._last_send = now
        return self.bus.stand(now)

    def _once_stop(self, now: float) -> str | None:
        if self._oneshot_sent:
            return None
        self._oneshot_sent = True
        self._last_send = now
        return self.bus.stop(now)


def _expect(cond: bool, msg: str, failures: list[str]) -> None:
    if not cond:
        failures.append(msg)


def _plant_md5() -> str:
    return hashlib.md5(PLANT_XML.read_bytes()).hexdigest()


def test_phrases() -> list[str]:
    failures: list[str] = []
    stand = parse_phrase("please stand")
    _expect(stand.kind == "stand" and stand.line == "stand", f"stand {stand}", failures)
    stop = parse_phrase("Stop!")
    _expect(stop.kind == "stop" and stop.line == "stop", f"stop {stop}", failures)
    walk = parse_phrase("walk forward")
    _expect(
        walk.kind == "vel" and walk.vx == FWD_MPS and walk.yaw_rate == 0.0,
        f"walk {walk}",
        failures,
    )
    _expect(walk.line == "vel(+0.056, +0.000)", f"walk line {walk.line}", failures)
    back = parse_phrase("back up")
    _expect(back.vx == -BACK_MPS and back.yaw_rate == 0.0, f"back {back}", failures)
    left = parse_phrase("turn left")
    _expect(
        left.kind == "vel" and left.vx == FWD_MPS and left.yaw_rate == YAW_RAD_S,
        f"left {left}",
        failures,
    )
    _expect(left.line == "vel(+0.056, +0.250)", f"left line {left.line}", failures)
    right = parse_phrase("turn right")
    _expect(
        right.vx == FWD_MPS and right.yaw_rate == -YAW_RAD_S and right.vx != 0.0,
        f"right {right}",
        failures,
    )
    _expect(right.line == "vel(+0.056, -0.250)", f"right line {right.line}", failures)
    for phrase in (
        "turn left in place",
        "spin",
        "turn around",
        "yaw in place",
    ):
        got = parse_phrase(phrase)
        _expect(got.kind == "refuse" and got.line == IN_PLACE_LINE, f"{phrase} {got}", failures)
    for phrase in ("go to the kitchen", "bathroom", "walk to the bedroom"):
        got = parse_phrase(phrase)
        _expect(got.line == ROOM_LINE, f"{phrase} {got.line}", failures)
    for phrase in ("go anywhere", "explore", "build a map", "waypoint"):
        got = parse_phrase(phrase)
        _expect(got.line == MAP_LINE, f"{phrase} {got.line}", failures)
    for phrase in ("strafe left", "move left", "walk right"):
        got = parse_phrase(phrase)
        _expect(got.line == VY_LINE, f"{phrase} {got.line}", failures)
    _expect(parse_phrase("open the door").line == DOOR_LINE, "door", failures)
    _expect(parse_phrase("bend the knee").line == JOINT_LINE, "joint", failures)
    _expect(parse_phrase("walk faster").line == TOO_FAST_LINE, "faster", failures)
    _expect(parse_phrase("vx 0.080").line == TOO_FAST_LINE, "0.080", failures)
    _expect(parse_phrase("").line == EMPTY_LINE, "empty", failures)
    _expect(parse_phrase("dance").line == UNKNOWN_LINE, "unknown", failures)
    phrases = [cue.phrase for cue in DEMO_CUES]
    _expect(
        phrases == ["stand", "walk forward", "turn left", "walk forward", "stop"],
        f"demo phrases {phrases}",
        failures,
    )
    return failures


def test_bus() -> list[str]:
    import steer_walk as steer

    failures: list[str] = []
    _expect(steer.VX_FWD_CAP == FWD_MPS, f"fwd cap {steer.VX_FWD_CAP}", failures)
    _expect(steer.VX_BACK_CAP == BACK_MPS, f"back cap {steer.VX_BACK_CAP}", failures)
    _expect(steer.YAW_RATE_CAP == YAW_RAD_S, f"yaw cap {steer.YAW_RATE_CAP}", failures)
    _expect(abs(SOFT_VX - 0.5 * steer.VX_FWD_CAP) < 1e-12, "soft vx is not half cap", failures)
    _expect(SOFT_VX < FWD_MPS, "soft vx is not under the forward cap", failures)
    _expect(steer.COMMAND_TIMEOUT_S == TIMEOUT_S, f"timeout {steer.COMMAND_TIMEOUT_S}", failures)
    _expect(steer.VEL_RESEND_S == RESEND_S, f"resend {steer.VEL_RESEND_S}", failures)
    _expect(steer.PLANT_MD5 == PLANT_MD5, f"steer md5 {steer.PLANT_MD5}", failures)
    _expect(_plant_md5() == PLANT_MD5, f"file md5 {_plant_md5()}", failures)
    xml = PLANT_XML.read_text(encoding="utf-8")
    _expect(
        f'<camera name="kit_cam" pos="{KIT_CAM_POS_TEXT}"' in xml,
        "kit_cam pose text drifted",
        failures,
    )

    bus = steer.CommandBus()
    caller = VoiceCaller(bus, resend_s=steer.VEL_RESEND_S)
    heard = caller.hear("walk forward", 0.0)
    _expect(heard == "vel(+0.056, +0.000)", f"hear walk {heard}", failures)
    dt = float(steer.CTRL_DT)
    t = 0.0
    report = bus.tick(t, dt)
    while t < 1.0 - 1e-9:
        caller.publish(t)
        report = bus.tick(t, dt)
        t += dt
    _expect(report.mode == "move", f"resend mode {report.mode}", failures)
    _expect(abs(report.applied_vx - FWD_MPS) < 1e-3, f"applied vx {report.applied_vx}", failures)
    _expect(abs(report.applied_yaw_rate) < 1e-6, "walk yaw leaked", failures)

    silent_from = t
    while t < silent_from + 0.30:
        report = bus.tick(t, dt)
        t += dt
    _expect(report.mode == "stand", f"silence mode {report.mode}", failures)
    _expect(abs(report.applied_vx) < 1e-6, f"silence vx {report.applied_vx}", failures)
    _expect(abs(report.applied_yaw_rate) < 1e-6, "silence yaw", failures)

    refused = caller.hear("go to the kitchen", t)
    _expect(refused == ROOM_LINE, f"kitchen during walk {refused}", failures)
    # A refused phrase must not replace the vel. Silence already stood this bus.
    _expect(caller.command is not None and caller.command.kind == "vel", "refuse cleared vel", failures)

    fresh = steer.CommandBus()
    quiet = VoiceCaller(fresh, resend_s=steer.VEL_RESEND_S)
    quiet.hear("go to the kitchen", 0.0)
    held = fresh.tick(0.0, dt)
    _expect(held.mode == "stand" and abs(held.applied_vx) < 1e-6, "kitchen published vel", failures)

    turn_bus = steer.CommandBus()
    turn = VoiceCaller(turn_bus, resend_s=steer.VEL_RESEND_S)
    turn_line = turn.hear("turn left", 0.0)
    _expect(turn_line == "vel(+0.056, +0.250)", f"turn hear {turn_line}", failures)
    _expect(abs(turn_bus.target_vx - FWD_MPS) < 1e-12, f"target vx {turn_bus.target_vx}", failures)
    _expect(abs(turn_bus.target_yaw - YAW_RAD_S) < 1e-12, f"target yaw {turn_bus.target_yaw}", failures)
    script = steer.NAV_LEFT_SCRIPT
    cue_ends = [cue.t_end for cue in DEMO_CUES]
    seg_ends = [seg.t_end for seg in script]
    _expect(cue_ends == seg_ends, f"cue times {cue_ends} != nav-left {seg_ends}", failures)
    for cue, seg in zip(DEMO_CUES, script, strict=True):
        cmd = parse_phrase(cue.phrase)
        _expect(cmd.kind == seg.kind, f"{cue.label} kind {cmd.kind} != {seg.kind}", failures)
        if seg.kind == "vel":
            _expect(
                cmd.vx == seg.vx and cmd.yaw_rate == seg.yaw_rate,
                f"{cue.label} {cmd.line} != vel({seg.vx:+.3f}, {seg.yaw_rate:+.3f})",
                failures,
            )

    place = VoiceCaller(steer.CommandBus(), resend_s=steer.VEL_RESEND_S)
    place.command = VoiceCommand("vel", 0.0, YAW_RAD_S, "vel(+0.000, +0.250)")
    blocked = place.publish(0.0)
    _expect(blocked == IN_PLACE_LINE, f"zero-vx yaw {blocked}", failures)

    over = VoiceCaller(steer.CommandBus(), resend_s=steer.VEL_RESEND_S)
    over.command = VoiceCommand("vel", 0.080, 0.0, "vel(+0.080, +0.000)")
    blocked = over.publish(0.0)
    _expect(blocked == CAP_LINE, f"0.080 publish {blocked}", failures)
    return failures


def self_test() -> int:
    failures = test_phrases() + test_bus()
    if failures:
        for msg in failures:
            print(f"[voice] FAIL {msg}", file=sys.stderr)
        print(f"[voice] self-test {len(failures)} failure(s)", file=sys.stderr)
        return 1
    print("[voice] self-test passed")
    print(f"[voice] plant md5 {PLANT_MD5}")
    return 0


@dataclass(frozen=True)
class Snapshot:
    t: float
    x: float
    y: float
    yaw_deg: float
    up_z: float
    margin: float
    mode: str
    applied_vx: float
    applied_yaw_rate: float
    phrase: str
    bus: str


@dataclass(frozen=True)
class DemoResult:
    plant_md5: str
    kit_cam_pos: tuple[float, float, float]
    fault: bool
    fault_reason: str
    tip: bool
    min_up_z: float
    end_mode: str
    end_margin_m: float
    approach_m: float
    walk_dyaw_deg: float
    heading_deg: float
    mean_yaw_rate_turn: float
    mean_body_vx_walk: float
    mean_body_vx_turn: float
    stop_dyaw_deg: float
    resume_m: float
    end_heading_deg: float
    peak_tau_nm: float
    cop_in_box: bool
    matches_claimed_nav_left: bool
    turn_bus: str
    walk_bus: str
    stills: tuple[str, ...]
    mp4: str | None
    snapshots: tuple[Snapshot, ...]
    honesty: str


def _wrap_rad(dyaw: float) -> float:
    return (dyaw + math.pi) % (2.0 * math.pi) - math.pi


def _cue_at(now: float, cues: Sequence[Cue]) -> Cue:
    for cue in cues:
        if now < cue.t_end - 1e-9:
            return cue
    return cues[-1]


def _window(samples: Sequence[_Sample], t0: float, t1: float) -> list[_Sample]:
    return [sample for sample in samples if t0 - 1e-9 <= sample.t < t1 - 1e-9]


def _heading_travel(samples: Sequence[_Sample]) -> float:
    if len(samples) < 2:
        return 0.0
    yaw0 = samples[0].yaw
    dx = samples[-1].x - samples[0].x
    dy = samples[-1].y - samples[0].y
    return dx * math.cos(yaw0) + dy * math.sin(yaw0)


def _dyaw_deg(samples: Sequence[_Sample]) -> float:
    if len(samples) < 2:
        return 0.0
    return math.degrees(_wrap_rad(samples[-1].yaw - samples[0].yaw))


def _mean_yaw_rate(samples: Sequence[_Sample]) -> float:
    if len(samples) < 2:
        return 0.0
    dt = samples[-1].t - samples[0].t
    if dt <= 1e-9:
        return 0.0
    return _wrap_rad(samples[-1].yaw - samples[0].yaw) / dt


def _mean_body_vx(samples: Sequence[_Sample]) -> float:
    if not samples:
        return 0.0
    return sum(sample.body_vx for sample in samples) / float(len(samples))


def _bounds(label: str, cues: Sequence[Cue]) -> tuple[float, float]:
    t0 = 0.0
    for cue in cues:
        if cue.label == label:
            return (t0, cue.t_end)
        t0 = cue.t_end
    raise KeyError(label)


def burn_caption(frame: np.ndarray, lines: Sequence[str]) -> np.ndarray:
    """Dark band so the phrase and the bus command stay readable."""
    from PIL import Image, ImageDraw, ImageFont

    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 18)
    band_h = 22 * len(lines) + 12
    draw.rectangle((0, 0, image.width, band_h), fill=(0, 0, 0))
    y = 6
    for line in lines:
        draw.text((8, y), line, fill=(255, 230, 80), font=font)
        y += 22
    burned = np.asarray(image, dtype=np.uint8)
    return np.ascontiguousarray(burned)


def _save_png(frame: np.ndarray, path: Path) -> None:
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(frame).save(path)


def _render_body(session: object, lines: list[str]) -> np.ndarray:
    import steer_walk as steer

    if not isinstance(session, steer.SteerSession):
        raise RuntimeError("voice clip requires SteerSession")
    if session.renderer is None:
        raise RuntimeError("renderer not created")
    session.cam.lookat[:] = session.data.xpos[session.bid_body]
    # No mj_forward. A second forward changes the contact warm-start.
    session.renderer.update_scene(session.data, session.cam)
    raw = np.ascontiguousarray(session.renderer.render().copy(), dtype=np.uint8)
    return burn_caption(raw, lines)


def _kit_cam_tuple(model: object) -> tuple[float, float, float]:
    import steer_walk as steer

    cid = steer.mj.mj_name2id(model, steer.mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cid < 0:
        raise RuntimeError("kit_cam missing")
    pos = getattr(model, "cam_pos")[cid]
    return (float(pos[0]), float(pos[1]), float(pos[2]))


def run_demo(*, video: bool, out_mp4: Path | None) -> DemoResult:
    import steer_walk as steer

    problems = test_bus()
    if problems:
        raise SystemExit("refused: " + "; ".join(problems))
    session = steer.SteerSession(video=video)
    cam_pos = _kit_cam_tuple(session.model)
    if any(abs(got - want) > 1e-6 for got, want in zip(cam_pos, KIT_CAM_POS, strict=True)):
        raise SystemExit(f"refused: kit_cam pos {cam_pos} != {KIT_CAM_POS}")
    if _plant_md5() != PLANT_MD5:
        raise SystemExit(f"refused: plant md5 {_plant_md5()}")

    caller = VoiceCaller(session.bus, resend_s=steer.VEL_RESEND_S)
    dt = float(steer.CTRL_DT)
    n_ctrl = int(round(DEMO_CUES[-1].t_end / dt))
    frames: list[np.ndarray] = []
    stills: list[str] = []
    saved: set[str] = set()
    snapshots: list[Snapshot] = []
    active = ""
    last_print = -1.0
    fault_announced = False
    for _ in range(n_ctrl):
        now = float(session.data.time)
        cue = _cue_at(now, DEMO_CUES)
        if cue.label != active:
            active = cue.label
            print(f"t={now:.2f} {cue.phrase!r} -> {caller.hear(cue.phrase, now)}")
        else:
            refusal = caller.publish(now)
            if refusal:
                print(refusal)
        report = session.step()
        if report.mode == "fault" and not fault_announced:
            print(f"fault: {session.bus.fault_reason}")
            fault_announced = True
        command = caller.command
        phrase = cue.phrase
        published = command.line if command is not None else "(none)"
        latest = session.samples[-1]
        if (now - last_print) >= 0.999:
            print(
                f"t={now:.2f} {cue.label} voice={phrase!r} bus={published} "
                f"{report.line()} x={latest.x:+.3f} "
                f"yaw={math.degrees(latest.yaw):+.1f} up={latest.up_z:.3f}"
            )
            last_print = now
            snapshots.append(
                Snapshot(
                    t=now,
                    x=latest.x,
                    y=latest.y,
                    yaw_deg=math.degrees(latest.yaw),
                    up_z=latest.up_z,
                    margin=latest.margin,
                    mode=report.mode,
                    applied_vx=float(report.applied_vx),
                    applied_yaw_rate=float(report.applied_yaw_rate),
                    phrase=phrase,
                    bus=published,
                )
            )
        if not video or session.renderer is None or len(session.samples) % 2 != 0:
            continue
        lines = [
            f'"{phrase}"  →  {published}',
            report.line(),
            (
                f"t={now:.2f}s  x={latest.x:+.3f} m  "
                f"heading={math.degrees(latest.yaw):+.1f} deg"
            ),
            "CommandBus only  |  no map  |  not go-anywhere",
        ]
        frame = _render_body(session, lines)
        frames.append(frame)
        for still_t, still_label, name in STILLS:
            if name in saved or cue.label != still_label or now + 1e-9 < still_t:
                continue
            path = PREVIEWS / name
            _save_png(frame, path)
            saved.add(name)
            stills.append(str(path.relative_to(ROOT)))
            print(f"[voice] wrote {path}")
    session.assert_plant_unchanged()
    if _plant_md5() != PLANT_MD5:
        raise SystemExit("refused: plant md5 changed during the clip")

    mp4_path: Path | None = None
    if video and frames and out_mp4 is not None:
        steer._write_mp4(frames, out_mp4)
        mp4_path = out_mp4
        print(f"[voice] wrote {out_mp4} ({len(frames)} frames)")

    import demo_8pm_motion as pack

    summary = steer.summarize(session, steer.NAV_LEFT_SCRIPT)
    matched = pack._claimed_match(summary)
    stop = _window(session.samples, *_bounds("stop", DEMO_CUES))
    result = DemoResult(
        plant_md5=_plant_md5(),
        kit_cam_pos=cam_pos,
        fault=bool(summary.fault),
        fault_reason=summary.fault_reason,
        tip=bool(summary.tip),
        min_up_z=float(summary.min_up_z),
        end_mode=summary.end_mode,
        end_margin_m=float(summary.end_margin_m),
        approach_m=float(summary.dx_forward_m),
        walk_dyaw_deg=math.degrees(summary.dyaw_forward_rad),
        heading_deg=math.degrees(summary.dyaw_turn_rad),
        mean_yaw_rate_turn=float(summary.mean_yaw_rate_turn),
        mean_body_vx_walk=float(summary.mean_body_vx_forward),
        mean_body_vx_turn=float(summary.mean_body_vx_turn),
        stop_dyaw_deg=_dyaw_deg(stop),
        resume_m=float(summary.dx_resume_m),
        end_heading_deg=math.degrees(summary.dyaw_end_rad),
        peak_tau_nm=float(summary.peak_torque_nm),
        cop_in_box=bool(summary.cop_in_box),
        matches_claimed_nav_left=bool(matched),
        turn_bus=bus_text("vel", FWD_MPS, YAW_RAD_S),
        walk_bus=bus_text("vel", FWD_MPS, 0.0),
        stills=tuple(stills),
        mp4=str(mp4_path.relative_to(ROOT)) if mp4_path is not None else None,
        snapshots=tuple(snapshots),
        honesty="",
    )
    text = _honesty(result)
    result = replace(result, honesty=text)
    print("[voice] " + text)
    return result


def _honesty(result: DemoResult) -> str:
    fault = (
        f"fault ({result.fault_reason})"
        if result.fault
        else "no fault"
    )
    tip = "tipped (up_z < 0.85)" if result.tip else "did not tip"
    match = (
        "matches Controls' claimed nav-left envelope"
        if result.matches_claimed_nav_left
        else "does not match Controls' claimed nav-left envelope"
    )
    pos = result.kit_cam_pos
    return (
        f"Voice clip on plant md5 {result.plant_md5}. "
        f"kit_cam stayed at {pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f}. "
        f"Phrases were stand, walk forward as {result.walk_bus}, "
        f"turn left as {result.turn_bus}, walk forward again, then stop. "
        f"Same windows as the claimed nav-left arc. "
        f"Approach Δx {result.approach_m:+.3f} m "
        f"(walk heading change {result.walk_dyaw_deg:+.1f} deg, "
        f"mean body vx {result.mean_body_vx_walk:+.3f} m/s). "
        f"Left arc Δyaw {result.heading_deg:+.1f} deg, "
        f"mean yaw rate {result.mean_yaw_rate_turn:+.3f} rad/s "
        f"(cap ±{YAW_RAD_S:.2f}; the command is not the heading rate), "
        f"mean body vx {result.mean_body_vx_turn:+.3f} m/s. "
        f"Resume along the new heading {result.resume_m:+.3f} m. "
        f"End heading {result.end_heading_deg:+.1f} deg. "
        f"Heading change during the stop hold {result.stop_dyaw_deg:+.1f} deg. "
        f"min up_z {result.min_up_z:.3f}. Peak leg torque {result.peak_tau_nm:.2f} Nm. "
        f"CoP in box {result.cop_in_box}. End mode {result.end_mode}. "
        f"End support margin {result.end_margin_m:+.3f} m. "
        f"{fault}; {tip}. This clip {match}. "
        "Claimed on that envelope: approach +0.598 m, left arc +75.7 deg, "
        "resume +0.317 m, min up_z 0.954. "
        "vx=0 yaw was not sent. Not a kitchen trip, not a map, not go-anywhere. "
        "Prefer FAIL, not this clip: a 14 s left hold from t=15 s tips on the resume; "
        "a second arc is not published."
    )


def _write_summary(result: DemoResult, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(result), indent=2) + "\n", encoding="utf-8")
    print(f"[voice] wrote {path}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Voice phrases on CommandBus")
    parser.add_argument("phrase", nargs="?", default=None, help="One typed phrase")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--measure",
        action="store_true",
        help="Run the stand/walk/turn/stop steer and write the summary, no video",
    )
    parser.add_argument(
        "--clip",
        action="store_true",
        help="Same steer, third-person mp4 with the phrase and bus command on frame",
    )
    parser.add_argument("--out", default=str(PREVIEWS / "voice_commands_demo.mp4"))
    parser.add_argument(
        "--summary",
        default=str(PREVIEWS / "voice_commands_summary.json"),
    )
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.measure or args.clip:
        result = run_demo(video=bool(args.clip), out_mp4=Path(args.out) if args.clip else None)
        _write_summary(result, Path(args.summary))
        if args.clip and result.mp4 is None:
            return 2
        if result.fault or result.tip or not result.matches_claimed_nav_left:
            return 2
        return 0
    if args.phrase is None:
        parser.print_help()
        return 2
    command = parse_phrase(args.phrase)
    print(command.line)
    return 0 if command.kind != "refuse" else 1


if __name__ == "__main__":
    raise SystemExit(main())
