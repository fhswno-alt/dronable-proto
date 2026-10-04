#!/usr/bin/env python3
"""Day-1 voice caller on the CommandBus in scripts/steer_walk.py.

A short English phrase becomes stand, stop, or vel(vx, yaw_rate). Latest
command wins. vel is resent at 10 Hz. Silence longer than 200 ms already
stands inside the bus. This file does not write that clock and does not
change the clamps or the CommandBus.

Forward sends +0.056 m/s, the bus forward cap, and resends it at 10 Hz.
This caller does not send 0.080. 0.080 yaws and tips near 1.2 m. Controls'
clip at +0.056 moved about +2.16 m, mean body speed about +0.039 m/s,
then stood, end margin +0.063 m. The command 0.056 is not the odometry.
This caller does not claim faster or farther than that clip.
The gait is not a room crossing. The tip predicate is still COM outside
support together with up_z < 0.85. It was not loosened.

Back up sends the bus reverse clamp, -0.032 m/s, and resends it at 10 Hz.
Controls' merged clip moved about -0.97 m, mean body speed about -0.053 m/s,
upright, no tip. This caller does not send a more negative vx. This caller
does not claim faster than that clip.

Turn right while walking sends yaw_rate -0.25 rad/s together with forward
vx +0.056 m/s. Turn left while walking sends yaw_rate +0.25 rad/s with the
same forward vx. Neither sends 0.080, and neither sends a yaw past ±0.25.
Yaw with zero forward is only a turn in place. That hold is a hip bias,
not a spin. Left while walking on the same 11 s window is about +62 deg.
Their right-turn ceiling on that window is about -83 deg, still moving
forward about +0.46 m, end stand, margin about +0.063 m. A 4.5 s window
used to stall near -17 deg. That short window is obsolete. This caller
does not claim more than their 11 s clip.

Go to the kitchen, the bathroom, or any other room is refused in one
line: the camera can see, but there is no room and no map. No path is
invented. The floor in the clip is empty.

kit_cam is the plant camera on head_tilt_link at pos 0.050 0.019 0.007,
same aim and fovy. Plant md5 is 71b2c86d133ebc603f58b99c53e496f3. The
clip renders that camera for back up, for walk forward then stop, and
for turn right while walking for about 11 s then stop. This file does
not move the camera, and does not edit the plant, the gait, the tip
check, or the CommandBus.

Run:
  python scripts/voice_caller.py "walk forward"
  python scripts/voice_caller.py "back up"
  python scripts/voice_caller.py "go to the kitchen"
  python scripts/voice_caller.py --self-test
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

os.environ.setdefault("MUJOCO_GL", "osmesa")

ROOT = Path(__file__).resolve().parents[1]
PREVIEWS = ROOT / "previews"

# Controls' clamps on the merged bus. The self-test fails if steer_walk drifts.
FWD_MPS = 0.056
BACK_CAP_MPS = 0.032
YAW_RAD_S = 0.25
RESEND_S = 0.10
TIMEOUT_S = 0.200
PLANT_MD5 = "71b2c86d133ebc603f58b99c53e496f3"
KIT_CAM_POS = (0.050, 0.019, 0.007)
PLANT_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
FWD_CLIP_DX_M = 2.16
FWD_CLIP_VX_MPS = 0.039
FWD_CLIP_MARGIN_M = 0.063
REVERSE_VX_MPS = -0.053
REVERSE_DX_M = -0.97
WALK_LEFT_DEG = 62.0
WALK_RIGHT_DEG = -83.0
WALK_RIGHT_DX_M = 0.46
WALK_RIGHT_MARGIN_M = 0.063
RIGHT_TURN_S = 11.0

ROOM_LINE = "the camera can see, but there is no room and no map"
CLAMP_LINE = "refused: reverse is only the bus clamp"
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
_LEFT_IN_PLACE = frozenset({
    "turn left in place", "left in place", "yaw left in place",
})
_RIGHT_IN_PLACE = frozenset({
    "turn right in place", "right in place", "yaw right in place",
})
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
    "the back": "back",
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
    margin: float


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
            "(command into the gait; the command 0.056 is not the odometry; "
            "Controls' clip moved about "
            f"{FWD_CLIP_DX_M:+.2f} m, mean body speed about {FWD_CLIP_VX_MPS:+.3f} m/s, "
            f"then stood, end margin {FWD_CLIP_MARGIN_M:+.3f} m; "
            "0.080 yaws and tips near 1.2 m; "
            "this caller does not claim faster or farther)"
        ),
    )


def _back() -> VoiceCommand:
    return VoiceCommand(
        "vel",
        -BACK_CAP_MPS,
        0.0,
        (
            f"vel vx={-BACK_CAP_MPS:+.3f} yaw_rate={0.0:+.3f} "
            "(bus reverse clamp; Controls' clip moved about "
            f"{REVERSE_DX_M:+.2f} m, mean body speed about {REVERSE_VX_MPS:+.3f} m/s, "
            "upright, no tip; this caller does not claim faster)"
        ),
    )


def _walk_turn(yaw_rate: float) -> VoiceCommand:
    return VoiceCommand(
        "vel",
        FWD_MPS,
        yaw_rate,
        (
            f"vel vx={FWD_MPS:+.3f} yaw_rate={yaw_rate:+.3f} "
            f"(walking turn inside ±{YAW_RAD_S:.2f} rad/s with forward "
            f"vx {FWD_MPS:+.3f}; not a spin; left while walking about "
            f"{WALK_LEFT_DEG:+.0f} deg; right {RIGHT_TURN_S:.0f} s window about "
            f"{WALK_RIGHT_DEG:+.0f} deg, still moving forward about "
            f"{WALK_RIGHT_DX_M:+.2f} m, end stand, margin about "
            f"{WALK_RIGHT_MARGIN_M:+.3f} m; the 4.5 s window is obsolete; "
            "this caller does not claim more than that clip; does not send 0.080)"
        ),
    )


def _yaw_in_place(yaw_rate: float) -> VoiceCommand:
    return VoiceCommand(
        "vel",
        0.0,
        yaw_rate,
        (
            f"vel vx={0.0:+.3f} yaw_rate={yaw_rate:+.3f} "
            f"(turn in place; hip bias inside ±{YAW_RAD_S:.2f} rad/s; not a spin)"
        ),
    )


def _stop() -> VoiceCommand:
    return VoiceCommand("stop", 0.0, 0.0, "stop")


def _stand() -> VoiceCommand:
    return VoiceCommand("stand", 0.0, 0.0, "stand")


def _from_direction(name: str) -> VoiceCommand:
    if name == "left":
        return _walk_turn(YAW_RAD_S)
    if name == "right":
        return _walk_turn(-YAW_RAD_S)
    if name == "forward":
        return _forward()
    if name == "back":
        return _back()
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
        return _back()
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
    if text in _LEFT_IN_PLACE:
        return _yaw_in_place(YAW_RAD_S)
    if text in _RIGHT_IN_PLACE:
        return _yaw_in_place(-YAW_RAD_S)
    if text in _LEFT_PHRASES:
        return _walk_turn(YAW_RAD_S)
    if text in _RIGHT_PHRASES:
        return _walk_turn(-YAW_RAD_S)
    if text in _STOP_PHRASES:
        return _stop()
    if text in _STAND_PHRASES:
        return _stand()
    placed = _place_command(text)
    if placed is not None:
        return placed
    return _refuse(UNKNOWN_LINE)


class VoiceCaller:
    """Publish stand, stop, or vel. Reverse is only the bus clamp."""

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
        if command.vx < -BACK_CAP_MPS:
            return CLAMP_LINE
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
        if command.kind != "vel":
            return None
        if command.vx < -BACK_CAP_MPS or (command.vx < 0.0 and command.vx != -BACK_CAP_MPS):
            return CLAMP_LINE
        if command.vx > FWD_MPS or abs(command.yaw_rate) > YAW_RAD_S:
            return "refused: above the day-1 cap"
        if command.yaw_rate != 0.0 and command.vx == 0.0 and "turn in place" not in command.line:
            return "refused: yaw with zero forward is only a turn in place"
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


# Separate steers. The stop hold runs past the 1.20 s settle damper.
# This file does not edit that damper or the tip check.
BACKUP_CUES: tuple[PhraseCue, ...] = (
    PhraseCue(1.0, None, "quiet"),
    PhraseCue(5.0, "back up", "backup"),
    PhraseCue(6.0, "stop", "stop"),
)
# ~0.039 m/s body speed needs well past 26 s to clear 1 m. The stop hold
# runs past the 1.20 s settle damper. Command is +0.056, not body speed.
FORWARD_CUES: tuple[PhraseCue, ...] = (
    PhraseCue(1.0, None, "quiet"),
    PhraseCue(36.0, "walk forward", "forward"),
    PhraseCue(38.5, "stop", "stop"),
)
# Same 11 s command window as Controls' right-turn clip, then the settle.
RIGHT_CUES: tuple[PhraseCue, ...] = (
    PhraseCue(1.0, None, "quiet"),
    PhraseCue(1.0 + RIGHT_TURN_S, "turn right", "turn-right"),
    PhraseCue(1.0 + RIGHT_TURN_S + 2.5, "stop", "stop"),
)
BACKUP_STILLS = ((3.0, "backup", "voice_caller_kit_cam_backup.png"),)
FORWARD_STILLS = (
    (12.0, "forward", "voice_caller_kit_cam_forward.png"),
    (38.2, "stop", "voice_caller_kit_cam_stop.png"),
)
RIGHT_STILLS = (
    (11.0, "turn-right", "voice_caller_kit_cam_turn_right.png"),
    (14.2, "stop", "voice_caller_kit_cam_turn_right_stop.png"),
)


def _cue_index(now: float, cues: tuple[PhraseCue, ...]) -> int:
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


def _plant_file_md5() -> str:
    return hashlib.md5(PLANT_XML.read_bytes()).hexdigest()


def _shipped_plant_problems() -> list[str]:
    """The freeze already names this plant. Do not rewrite it."""
    import steer_walk as steer

    problems: list[str] = []
    if not PLANT_XML.is_file():
        problems.append(f"missing {PLANT_XML}")
        return problems
    digest = _plant_file_md5()
    if digest != PLANT_MD5:
        problems.append(f"plant file md5 {digest} != {PLANT_MD5}")
    if steer.PLANT_MD5 != PLANT_MD5:
        problems.append(f"PLANT_MD5 {steer.PLANT_MD5} != {PLANT_MD5}")
    if tuple(float(v) for v in steer.KIT_CAM_POS) != KIT_CAM_POS:
        problems.append(f"KIT_CAM_POS {steer.KIT_CAM_POS} != {KIT_CAM_POS}")
    return problems


def _cap_problems() -> list[str]:
    import steer_walk as steer

    problems = _shipped_plant_problems()
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
    if tuple(float(v) for v in steer.KIT_CAM_POS) != KIT_CAM_POS:
        problems.append(f"KIT_CAM_POS {steer.KIT_CAM_POS} != {KIT_CAM_POS}")
    if not (0.0 < steer.VEL_RESEND_S < steer.COMMAND_TIMEOUT_S):
        problems.append("vel resend is not inside the 200 ms watchdog")
    return problems


def _expect(cond: bool, msg: str, failures: list[str]) -> None:
    if not cond:
        failures.append(msg)


def test_phrases() -> list[str]:
    failures: list[str] = []
    doc = __doc__ or ""
    _expect("+2.16" in doc and "+0.039" in doc and "+0.063" in doc, "module doc omits the 0.056 clip", failures)
    _expect("does not send 0.080" in doc, "module doc still sends 0.080", failures)
    _expect("not the odometry" in doc, "module doc treats 0.056 as body speed", failures)
    _expect("up_z < 0.85" in doc and "was not loosened" in doc, "module doc loosens the tip check", failures)
    _expect("-0.032" in doc, "module doc omits the reverse clamp", failures)
    _expect("does not claim faster" in doc, "module doc claims a faster reverse", failures)
    _expect("-0.053" in doc and "-0.97" in doc, "module doc omits the reverse clip", failures)
    _expect("+62" in doc and "-83" in doc, "module doc omits the walking-turn holds", failures)
    _expect("11 s" in doc and "4.5" in doc and "obsolete" in doc, "module doc keeps the short right window", failures)
    _expect("+0.46" in doc and "end stand" in doc, "module doc omits the right-turn ceiling", failures)
    _expect("not a spin" in doc and "spin in place" not in doc, "module doc calls the turn a spin", failures)
    _expect("turn in place" in doc, "module doc drops the in-place phrase", failures)
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
        ("turn left", "vel", FWD_MPS, YAW_RAD_S),
        ("turn right", "vel", FWD_MPS, -YAW_RAD_S),
        ("turn left in place", "vel", 0.0, YAW_RAD_S),
        ("turn right in place", "vel", 0.0, -YAW_RAD_S),
        ("stop", "stop", 0.0, 0.0),
        ("stand", "stand", 0.0, 0.0),
        ("go to the left", "vel", FWD_MPS, YAW_RAD_S),
    )
    for phrase, kind, vx, yaw in table:
        command = parse_phrase(phrase)
        _expect(command.kind == kind, f"{phrase!r} kind {command.kind}", failures)
        _expect(
            command.vx == vx and command.yaw_rate == yaw,
            f"{phrase!r} maps to vx={command.vx} yaw={command.yaw_rate}",
            failures,
        )
        _expect(command.vx <= FWD_MPS and command.vx >= -BACK_CAP_MPS, f"{phrase!r} left the bus clamp", failures)
    forward = parse_phrase("walk forward")
    _expect("not the odometry" in forward.line, "forward line treats 0.056 as body speed", failures)
    _expect("+2.16" in forward.line and "+0.039" in forward.line and "+0.063" in forward.line, "forward line omits the 0.056 clip", failures)
    _expect("0.080" in forward.line and "1.2" in forward.line, "forward line omits the 0.080 tip", failures)
    _expect("does not claim faster or farther" in forward.line, "forward line claims a longer walk", failures)
    _expect(forward.vx == FWD_MPS, "forward is not the bus cap", failures)
    left = parse_phrase("turn left")
    right = parse_phrase("turn right")
    _expect(left.vx == FWD_MPS and left.yaw_rate == YAW_RAD_S, "left is not a walking turn", failures)
    _expect("not a spin" in left.line and "spin in place" not in left.line, "left line claims a spin", failures)
    _expect("+62" in left.line and "-83" in left.line and "+0.46" in left.line, "left line omits the walking holds", failures)
    _expect(right.vx == FWD_MPS and right.yaw_rate == -YAW_RAD_S, "right is not a walking turn", failures)
    _expect("not a spin" in right.line and "does not send 0.080" in right.line, "right line claims a spin or sends 0.080", failures)
    _expect("4.5 s window is obsolete" in right.line, "right line keeps the short window", failures)
    for phrase in ("turn right in place", "right in place", "yaw right in place"):
        command = parse_phrase(phrase)
        _expect(command.vx == 0.0 and command.yaw_rate == -YAW_RAD_S, f"{phrase!r} is not in place", failures)
        _expect("turn in place" in command.line and "spin in place" not in command.line, f"{phrase!r} calls it a spin", failures)
    for phrase in ("back up", "walk back", "reverse", "go back", "Back up.", "go to the back"):
        command = parse_phrase(phrase)
        _expect(command.kind == "vel", f"{phrase!r} kind {command.kind}", failures)
        _expect(command.vx == -BACK_CAP_MPS and command.yaw_rate == 0.0, f"{phrase!r} vx={command.vx}", failures)
        _expect("does not claim faster" in command.line, f"{phrase!r} claims a faster reverse", failures)
        _expect("-0.053" in command.line and "-0.97" in command.line, f"{phrase!r} omits the reverse clip", failures)
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
    backup_phrases = [cue.phrase for cue in BACKUP_CUES if cue.phrase is not None]
    forward_phrases = [cue.phrase for cue in FORWARD_CUES if cue.phrase is not None]
    _expect(backup_phrases == ["back up", "stop"], f"backup phrases {backup_phrases}", failures)
    _expect(forward_phrases == ["walk forward", "stop"], f"forward phrases {forward_phrases}", failures)
    right_phrases = [cue.phrase for cue in RIGHT_CUES if cue.phrase is not None]
    _expect(right_phrases == ["turn right", "stop"], f"right phrases {right_phrases}", failures)
    _expect(
        abs((RIGHT_CUES[1].t_end - RIGHT_CUES[0].t_end) - RIGHT_TURN_S) < 1e-9,
        "right window is not 11 s",
        failures,
    )
    return failures


class _CountBus:
    def __init__(self, inner: ClockedBus) -> None:
        self.inner = inner
        self.vel_n = 0
        self.stop_n = 0
        self.stand_n = 0
        self.vxs: list[float] = []
        self.yaws: list[float] = []

    def stand(self, now: float) -> str | None:
        self.stand_n += 1
        return self.inner.stand(now)

    def stop(self, now: float) -> str | None:
        self.stop_n += 1
        return self.inner.stop(now)

    def vel(self, vx: float, yaw_rate: float, now: float) -> str | None:
        self.vel_n += 1
        self.vxs.append(vx)
        self.yaws.append(yaw_rate)
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
        f"command did not sit on +0.056: {saturated.line()}",
        failures,
    )

    latest = steer.CommandBus()
    latest_count = _CountBus(latest)
    latest_caller = VoiceCaller(latest_count, resend_s=steer.VEL_RESEND_S)
    latest_caller.hear("walk forward", 0.0)
    latest_caller.hear("turn left", 0.4)
    _expect(
        latest.target_vx == fwd_cap and latest.target_yaw == yaw_cap,
        "turn left did not replace forward with a walking turn",
        failures,
    )
    latest_caller.hear("turn right", 0.5)
    _expect(
        latest.target_yaw == -yaw_cap and latest.target_vx == fwd_cap,
        "turn right is not yaw -0.25 with forward +0.056",
        failures,
    )
    backup_line = latest_caller.hear("back up", 0.55)
    _expect(backup_line.startswith("vel vx=-0.032"), f"back up line {backup_line!r}", failures)
    _expect(
        latest.target_vx == -steer.VX_BACK_CAP and latest.target_yaw == 0.0,
        "back up did not replace the yaw command with the reverse clamp",
        failures,
    )
    kitchen = latest_caller.hear("go to the kitchen", 0.57)
    _expect(kitchen == ROOM_LINE, f"kitchen line {kitchen!r}", failures)
    _expect(
        latest.target_vx == -steer.VX_BACK_CAP and latest.target_yaw == 0.0,
        "kitchen replaced the reverse command",
        failures,
    )
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
    _expect(
        all(-steer.VX_BACK_CAP - 1e-12 <= vx <= fwd_cap for vx in latest_count.vxs),
        "a vel left the bus clamps",
        failures,
    )

    rev = steer.CommandBus()
    rev_count = _CountBus(rev)
    rev_caller = VoiceCaller(rev_count, resend_s=steer.VEL_RESEND_S)
    rev_caller.hear("back up", 0.0)
    _expect(rev.target_vx == -steer.VX_BACK_CAP and rev_count.vxs == [-steer.VX_BACK_CAP], "reverse clamp", failures)
    for i in range(1, 18):
        t = i * dt
        if i == 4:
            _expect(rev_count.vel_n == 1, "reverse resent before 100 ms", failures)
        rev_caller.publish(t)
        rev.tick(t, dt)
    _expect(rev_count.vel_n == 4, f"reverse 10 Hz resend count {rev_count.vel_n}", failures)
    _expect(all(vx == -steer.VX_BACK_CAP for vx in rev_count.vxs), "a reverse resend was not the clamp", failures)
    _expect(rev.applied_vx >= -steer.VX_BACK_CAP - 1e-12, "applied vx more negative than the clamp", failures)

    held = steer.CommandBus()
    held_caller = VoiceCaller(_CountBus(held), resend_s=steer.VEL_RESEND_S)
    held_caller.hear("reverse", 0.0)
    held_report = held.tick(0.0, dt)
    for i in range(1, steps):
        t = i * dt
        held_caller.publish(t)
        held_report = held.tick(t, dt)
    _expect(
        abs(held_report.applied_vx - (-steer.VX_BACK_CAP)) < 1e-6
        and held_report.applied_vx >= -steer.VX_BACK_CAP - 1e-12,
        f"reverse command did not sit on -0.032: {held_report.line()}",
        failures,
    )

    guard = steer.CommandBus()
    guard_caller = VoiceCaller(guard)
    guard_caller._command = VoiceCommand("vel", -steer.VX_BACK_CAP - 0.001, 0.0, "nope")
    guard_caller._last_send = -1.0e9
    refusal = guard_caller.publish(0.0)
    _expect(refusal == CLAMP_LINE, f"more-negative vx refusal {refusal!r}", failures)
    _expect(guard.target_vx == 0.0, "a more negative vx was applied", failures)
    fast = steer.CommandBus()
    fast_caller = VoiceCaller(fast)
    fast_caller._command = VoiceCommand("vel", 0.08, 0.0, "nope")
    fast_caller._last_send = -1.0e9
    fast_refusal = fast_caller.publish(0.0)
    _expect(fast_refusal == "refused: above the day-1 cap", f"0.080 refusal {fast_refusal!r}", failures)
    _expect(fast.target_vx == 0.0, "0.080 was sent", failures)
    wide = steer.CommandBus()
    wide_caller = VoiceCaller(wide)
    wide_caller._command = VoiceCommand("vel", FWD_MPS, -0.30, "nope")
    wide_caller._last_send = -1.0e9
    wide_refusal = wide_caller.publish(0.0)
    _expect(wide_refusal == "refused: above the day-1 cap", f"yaw past cap {wide_refusal!r}", failures)
    _expect(wide.target_yaw == 0.0 and wide.target_vx == 0.0, "a yaw past ±0.25 was sent", failures)
    bare = steer.CommandBus()
    bare_caller = VoiceCaller(bare)
    bare_caller._command = VoiceCommand("vel", 0.0, -YAW_RAD_S, "vel yaw only")
    bare_caller._last_send = -1.0e9
    bare_refusal = bare_caller.publish(0.0)
    _expect(
        bare_refusal == "refused: yaw with zero forward is only a turn in place",
        f"zero-forward yaw {bare_refusal!r}",
        failures,
    )
    _expect(bare.target_yaw == 0.0, "yaw with zero forward was sent", failures)
    parked = steer.CommandBus()
    parked_count = _CountBus(parked)
    parked_caller = VoiceCaller(parked_count, resend_s=steer.VEL_RESEND_S)
    parked_caller.hear("turn right in place", 0.0)
    _expect(parked.target_vx == 0.0 and parked.target_yaw == -yaw_cap, "in-place right target", failures)
    walking = steer.CommandBus()
    walking_count = _CountBus(walking)
    walking_caller = VoiceCaller(walking_count, resend_s=steer.VEL_RESEND_S)
    walking_caller.hear("turn right", 0.0)
    _expect(
        walking.target_vx == fwd_cap and walking.target_yaw == -yaw_cap,
        "walking right target",
        failures,
    )
    for i in range(1, 18):
        t = i * dt
        walking_caller.publish(t)
        walking.tick(t, dt)
    _expect(walking_count.vel_n == 4, f"right-turn 10 Hz resend count {walking_count.vel_n}", failures)
    _expect(
        all(vx == fwd_cap and yaw == -yaw_cap for vx, yaw in zip(walking_count.vxs, walking_count.yaws, strict=True)),
        "a right-turn resend left +0.056 or -0.25",
        failures,
    )
    return failures


def test_tip_predicate() -> list[str]:
    """The COM-outside-support check is the one already on main."""
    import inspect

    import steer_walk as steer

    failures: list[str] = []
    src = inspect.getsource(steer.SteerSession._fault_reason)
    _expect(steer.TIP_UP_Z == 0.72, f"TIP_UP_Z {steer.TIP_UP_Z}", failures)
    _expect(abs(steer.TIP_HOLD_S - 0.12) < 1e-12, f"TIP_HOLD_S {steer.TIP_HOLD_S}", failures)
    _expect(steer.COP_FAULT_MARGIN == -0.04, f"COP_FAULT_MARGIN {steer.COP_FAULT_MARGIN}", failures)
    _expect("COM outside support and tipping" in src, "COM-outside-support line missing", failures)
    _expect("up_z < 0.85" in src, "torso bar in the tip predicate was loosened", failures)
    _expect("margin < COP_FAULT_MARGIN" in src, "margin gate missing", failures)
    _expect("self.cop_out_hold >= TIP_HOLD_S and up_z < 0.85" in src, "tip conjunction was rewritten", failures)
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
    failures.extend(test_tip_predicate())
    failures.extend(test_bus())
    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[voice] self-test PASS (phrase → stand/stop/vel; reverse at the bus clamp; no walk claim)")
    return 0


@dataclass(frozen=True)
class ClipResult:
    plant_md5: str
    kit_cam_pos: tuple[float, float, float]
    mujoco_gl: str
    phrases: tuple[str, ...]
    fault: bool
    fault_reason: str
    command_vx_back: float
    command_vx_fwd: float
    mean_applied_vx_backup: float | None
    mean_body_vx_backup: float | None
    dx_backup_m: float | None
    mean_applied_vx_forward: float | None
    mean_body_vx_forward: float | None
    dx_forward_m: float | None
    min_up_z_backup: float | None
    min_up_z_forward: float | None
    end_mode_backup: str
    end_mode_forward: str
    end_margin_forward: float | None
    dyaw_right_deg: float | None
    dx_right_m: float | None
    mean_body_vx_right: float | None
    mean_applied_vx_right: float | None
    min_up_z_right: float | None
    end_mode_right: str
    end_margin_right: float | None
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
    stop_pose = "The forward stop frame is level. " if result.end_mode_forward == "stand" else ""
    text = (
        "kit_cam for back up, for walk forward then stop, and for an 11 s right turn. "
        "These are three separate steers. "
        f"Camera pos {result.kit_cam_pos[0]:.3f} {result.kit_cam_pos[1]:.3f} "
        f"{result.kit_cam_pos[2]:.3f}. The camera was not moved. "
        "The frame is still largely the inside of the head. "
        "Empty floor shows through the opening. "
        f"Back up command is {result.command_vx_back:+.3f} m/s. "
        "Mean applied_vx in the reverse window is "
        f"{_fmt(result.mean_applied_vx_backup, '+.3f')} m/s. "
        "Mean body vx in that window is "
        f"{_fmt(result.mean_body_vx_backup, '+.3f')} m/s. "
        f"Reverse Δx={_fmt(result.dx_backup_m, '+.3f')} m. "
        f"min up_z during back up is {_fmt(result.min_up_z_backup, '.3f')}. "
        f"End mode after back up is {result.end_mode_backup}. "
        f"Controls' merged reverse clip moved about {REVERSE_DX_M:+.2f} m, "
        f"mean body speed about {REVERSE_VX_MPS:+.3f} m/s, upright, no tip. "
        "This clip does not claim faster than that clip. "
        f"Forward command into the gait is {result.command_vx_fwd:+.3f} m/s. "
        "Mean applied_vx in the forward window is "
        f"{_fmt(result.mean_applied_vx_forward, '+.3f')} m/s. "
        "Mean body vx in that window is "
        f"{_fmt(result.mean_body_vx_forward, '+.3f')} m/s. "
        f"Forward Δx={_fmt(result.dx_forward_m, '+.3f')} m. "
        "min up_z while the forward command was applied is "
        f"{_fmt(result.min_up_z_forward, '.3f')}. "
        f"{_end_mode_sentence(result.end_mode_forward, 'Forward end mode')} "
        f"{stop_pose}"
        f"Forward end support margin is {_fmt(result.end_margin_forward, '+.3f')} m. "
        f"The command {FWD_MPS:.3f} is not the odometry. "
        f"Controls' clip at that cap moved about {FWD_CLIP_DX_M:+.2f} m, "
        f"mean body speed about {FWD_CLIP_VX_MPS:+.3f} m/s, then stood, "
        f"end margin {FWD_CLIP_MARGIN_M:+.3f} m. "
        "This clip does not claim faster or farther than that. "
        "0.080 yaws and tips near 1.2 m. This caller does not send 0.080. "
        f"Turn right while walking publishes vx {FWD_MPS:+.3f} m/s and "
        f"yaw_rate {-YAW_RAD_S:+.2f} rad/s for {RIGHT_TURN_S:.0f} s, then stop. "
        f"Heading in that window is {_fmt(result.dyaw_right_deg, '+.1f')} deg. "
        f"Forward Δx in that window is {_fmt(result.dx_right_m, '+.3f')} m. "
        f"Mean body vx in that window is {_fmt(result.mean_body_vx_right, '+.3f')} m/s. "
        f"min up_z during the right turn is {_fmt(result.min_up_z_right, '.3f')}. "
        f"{_end_mode_sentence(result.end_mode_right, 'Right-turn end mode')} "
        f"Right-turn end support margin is {_fmt(result.end_margin_right, '+.3f')} m. "
        f"Controls' {RIGHT_TURN_S:.0f} s window is the ceiling: right heading about "
        f"{WALK_RIGHT_DEG:+.0f} deg, still moving forward about {WALK_RIGHT_DX_M:+.2f} m, "
        f"end stand, margin about {WALK_RIGHT_MARGIN_M:+.3f} m. "
        "This clip does not claim more than that. "
        "The 4.5 s window that stalled near -17 deg is obsolete. "
        f"Left while walking on the same window is about {WALK_LEFT_DEG:+.0f} deg. "
        f"Both stay inside ±{YAW_RAD_S:.2f} rad/s. That is a hip bias, not a spin. "
        "Go to the kitchen or the bathroom stays refused: "
        "the camera can see, but there is no room and no map. "
        "The floor is empty. This gait is not a room crossing. "
        "Not autonomous navigation."
    )
    if result.fault:
        text += f" FAULT: {result.fault_reason}."
    return text


def _end_mode_sentence(mode: str, what: str = "End mode") -> str:
    if mode == "fault":
        return f"{what} is fault."
    if mode == "stand":
        return f"{what} is stand."
    if mode == "":
        return f"{what} is missing."
    return f"{what} is {mode}."


def _overlay(phrase: str, label: str, report_line: str, now: float, x: float) -> list[str]:
    if label == "backup":
        note = "cmd -0.032  clip ~-0.053 m/s ~-0.97 m  do not claim faster"
    elif label == "forward":
        note = "cmd +0.056  clip ~+0.039 m/s ~+2.16 m  do not claim farther"
    elif label == "turn-right":
        note = "cmd +0.056 yaw -0.25  11s ceiling ~-83 deg ~+0.46 m  do not claim more"
    else:
        note = "stop  |  ceiling end stand margin +0.063  do not claim more"
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
    cid = steer.mj.mj_name2id(session.model, steer.mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cid < 0:
        print("[voice] kit_cam is missing", file=sys.stderr)
        return 2
    cam_pos = tuple(float(v) for v in session.model.cam_pos[cid])
    if any(abs(got - want) > 1e-6 for got, want in zip(cam_pos, KIT_CAM_POS, strict=True)):
        print(f"[voice] refused: kit_cam pos {cam_pos} != {KIT_CAM_POS}", file=sys.stderr)
        return 2

    frames: list[np.ndarray] = []
    stills: list[str] = []
    print(
        f"[voice] kit_cam separate steers: back up | walk forward then stop | turn right 11s  "
        f"pos={cam_pos[0]:.3f} {cam_pos[1]:.3f} {cam_pos[2]:.3f} "
        f"md5={steer.PLANT_MD5} gl={os.environ.get('MUJOCO_GL', '')}"
    )
    backup_samples = _drive_render(session, steer, BACKUP_CUES, frames, stills, BACKUP_STILLS)
    forward = steer.SteerSession(video=True)
    if forward.renderer is None:
        print("[voice] kit_cam renderer was not created for the forward steer", file=sys.stderr)
        return 2
    forward_samples = _drive_render(forward, steer, FORWARD_CUES, frames, stills, FORWARD_STILLS)
    right = steer.SteerSession(video=True)
    if right.renderer is None:
        print("[voice] kit_cam renderer was not created for the right turn", file=sys.stderr)
        return 2
    right_samples = _drive_render(right, steer, RIGHT_CUES, frames, stills, RIGHT_STILLS)
    mp4_path: Path | None = None
    render_error = ""
    if frames:
        try:
            steer._write_mp4(frames, out_mp4)
            mp4_path = out_mp4
        except Exception as exc:
            render_error = f"{type(exc).__name__}: {exc}"
            print(f"[voice] mp4 failed: {render_error}", file=sys.stderr)
    result = _measure(
        backup_samples,
        session.bus.fault,
        session.bus.fault_reason,
        forward_samples,
        forward.bus.fault,
        forward.bus.fault_reason,
        right_samples,
        right.bus.fault,
        right.bus.fault_reason,
        stills,
        mp4_path,
        cam_pos,
    )
    if render_error:
        result = replace(
            result,
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


def _dx(samples: Sequence[SampleView]) -> float | None:
    if len(samples) < 2:
        return None
    return float(samples[-1].x - samples[0].x)


def _window(
    samples: Sequence[SampleView],
    cues: tuple[PhraseCue, ...],
    settle_after: float,
) -> tuple[list[SampleView], list[SampleView]]:
    quiet_end = cues[0].t_end
    move_end = cues[1].t_end
    return (
        _between(samples, quiet_end, move_end),
        _between(samples, quiet_end + settle_after, move_end),
    )


class _Qpos(Protocol):
    def __getitem__(self, index: int) -> float: ...


class _SimData(Protocol):
    time: float
    qpos: _Qpos


class _KitRenderer(Protocol):
    def update_scene(self, data: _SimData, camera: str) -> None: ...

    def render(self) -> object: ...


class _FaultBus(ClockedBus, Protocol):
    fault_reason: str


class _ClipSession(Protocol):
    bus: _FaultBus
    data: _SimData
    samples: list[SampleView]
    renderer: _KitRenderer | None

    def step(self) -> BusTick: ...

    def assert_plant_unchanged(self) -> None: ...


class _BurnGait(Protocol):
    def _burn_overlay(self, raw: object, lines: list[str]) -> object: ...


class _SteerApi(Protocol):
    CTRL_DT: float
    VEL_RESEND_S: float
    wg: _BurnGait


def _drive_render(
    session: _ClipSession,
    steer: _SteerApi,
    cues: tuple[PhraseCue, ...],
    frames: list[object],
    stills: list[str],
    still_times: tuple[tuple[float, str, str], ...],
) -> list[SampleView]:
    import numpy as np

    caller = VoiceCaller(session.bus, resend_s=steer.VEL_RESEND_S)
    dt = float(steer.CTRL_DT)
    n_ctrl = int(round(cues[-1].t_end / dt))
    saved: set[str] = set()
    active = -1
    last_print = -1.0
    fault_announced = False
    for _ in range(n_ctrl):
        now = float(session.data.time)
        index = _cue_index(now, cues)
        cue = cues[index]
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
        if (now - last_print) >= (float(steer.VEL_RESEND_S) - 1e-9):
            print(f"t={now:.2f} {cue.label} {report.line()}")
            last_print = now
        if len(session.samples) % 2 != 0:
            continue
        shown = cue.phrase if cue.phrase is not None else "(quiet)"
        lines = _overlay(shown, cue.label, report.line(), now, float(session.data.qpos[0]))
        # No mj_forward here. A second forward tips this gait.
        if session.renderer is None:
            continue
        session.renderer.update_scene(session.data, camera="kit_cam")
        raw = np.ascontiguousarray(session.renderer.render().copy(), dtype=np.uint8)
        frame = steer.wg._burn_overlay(raw, lines)
        frames.append(frame)
        for still_t, still_label, name in still_times:
            if name in saved or cue.label != still_label or now + 1e-9 < still_t:
                continue
            path = PREVIEWS / name
            _save_png(frame, path)
            saved.add(name)
            stills.append(str(path.relative_to(ROOT)))
            print(f"[voice] wrote {path}")
    session.assert_plant_unchanged()
    return list(session.samples)


def _wrap_rad(dyaw: float) -> float:
    return (dyaw + math.pi) % (2.0 * math.pi) - math.pi


def _dyaw_deg(samples: Sequence[SampleView]) -> float | None:
    if len(samples) < 2:
        return None
    return math.degrees(_wrap_rad(samples[-1].yaw - samples[0].yaw))


def _measure(
    backup_samples: Sequence[SampleView],
    backup_fault: bool,
    backup_reason: str,
    forward_samples: Sequence[SampleView],
    forward_fault: bool,
    forward_reason: str,
    right_samples: Sequence[SampleView],
    right_fault: bool,
    right_reason: str,
    stills: list[str],
    mp4: Path | None,
    cam_pos: tuple[float, float, float],
) -> ClipResult:
    backup, backup_settled = _window(backup_samples, BACKUP_CUES, 0.5)
    forward, forward_settled = _window(forward_samples, FORWARD_CUES, 1.5)
    right, right_settled = _window(right_samples, RIGHT_CUES, 1.5)
    backup_end = backup_samples[-1] if backup_samples else None
    forward_end = forward_samples[-1] if forward_samples else None
    right_end = right_samples[-1] if right_samples else None
    backup_ups = [sample.up_z for sample in backup]
    forward_ups = [sample.up_z for sample in forward]
    right_ups = [sample.up_z for sample in right]
    phrases = tuple(
        cue.phrase
        for cues in (BACKUP_CUES, FORWARD_CUES, RIGHT_CUES)
        for cue in cues
        if cue.phrase is not None
    )
    reasons = [reason for reason in (backup_reason, forward_reason, right_reason) if reason]
    draft = ClipResult(
        plant_md5=PLANT_MD5,
        kit_cam_pos=cam_pos,
        mujoco_gl=os.environ.get("MUJOCO_GL", ""),
        phrases=phrases,
        fault=backup_fault or forward_fault or right_fault,
        fault_reason="; ".join(reasons),
        command_vx_back=-BACK_CAP_MPS,
        command_vx_fwd=FWD_MPS,
        mean_applied_vx_backup=_mean([sample.applied_vx for sample in backup_settled]),
        mean_body_vx_backup=_mean([sample.body_vx for sample in backup_settled]),
        dx_backup_m=_dx(backup),
        mean_applied_vx_forward=_mean([sample.applied_vx for sample in forward_settled]),
        mean_body_vx_forward=_mean([sample.body_vx for sample in forward_settled]),
        dx_forward_m=_dx(forward),
        min_up_z_backup=min(backup_ups) if backup_ups else None,
        min_up_z_forward=min(forward_ups) if forward_ups else None,
        end_mode_backup=backup_end.mode if backup_end is not None else "",
        end_mode_forward=forward_end.mode if forward_end is not None else "",
        end_margin_forward=float(forward_end.margin) if forward_end is not None else None,
        dyaw_right_deg=_dyaw_deg(right),
        dx_right_m=_dx(right),
        mean_body_vx_right=_mean([sample.body_vx for sample in right_settled]),
        mean_applied_vx_right=_mean([sample.applied_vx for sample in right_settled]),
        min_up_z_right=min(right_ups) if right_ups else None,
        end_mode_right=right_end.mode if right_end is not None else "",
        end_margin_right=float(right_end.margin) if right_end is not None else None,
        stills=tuple(stills),
        mp4=str(mp4.relative_to(ROOT)) if mp4 is not None else None,
        honesty="",
    )
    return replace(draft, honesty=_honesty(draft))


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
        help="Render kit_cam for back up, walk forward, and an 11 s right turn",
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
            print("refused: --clip plays back up, walk forward, turn right, stop", file=sys.stderr)
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
