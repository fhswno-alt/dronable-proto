#!/usr/bin/env python3
"""Cold-stand measure of voice go-to for five rooms. Prefer FAIL.

Each phrase is parsed by scripts/voice_caller.py. A room phrase is
refused, so CommandBus is not given stand, stop, or vel. There is no
room recogniser on this tip. Moondream is not loaded. Zero spend.

The pose is the documented quiet stand in scripts/render_kit_cam_room.py.
One kit_cam frame is passed through the #71 finder: hazard cues, the
in-corridor too_close gate, and the first-sighting low-chroma confirm.
That cue is a hazard pixel. It is not a room label and not a distance
to a named room. No map query, no waypoint, and no find_kitchen path.

The plant file is only hashed. Soft-pass is off. Exit status is 1.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TypedDict

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import hazard_finder as hf
import mono_toe_gate as gate
import ray_corridor as rc
import render_kit_cam_room as rooms
import steer_walk as sw
import voice_caller as voice

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = sw.PLANT_XML
SUMMARY_PATH = ROOT / "previews" / "voice_goto_rooms_summary.json"
RECOGNISER_ABSENT = "absent"
STEP_OFF_M = 0.0
LEG_TOKS = ("hip_", "knee", "ank_")
PHRASES: tuple[tuple[str, str], ...] = (
    ("kitchen", "go to the kitchen"),
    ("bathroom", "go to the bathroom"),
    ("living", "go to the living room"),
    ("bedroom", "go to the bedroom"),
    ("entrance", "go to the entrance"),
)
GAP = (
    "Voice refuses a room phrase before CommandBus. This tip has no "
    "kit_cam room label (room_ask.py and find_room.py are absent; "
    "Moondream is not loaded). The #71 finder can report a hazard cue "
    "on the stand frame and does not choose vx or yaw."
)


class CueJson(TypedDict):
    too_close: bool
    bottom_clipped: bool
    column: int
    contact_row: int
    span_px: int
    width_px: int
    u: float | None
    v: float | None
    toe_gap_m: float | None
    eye_range_m: float | None
    sideways_m: float | None
    in_corridor: bool | None


class ContactJson(TypedDict):
    robot_geom: str
    prop_geom: str
    force_n: float


class RoomJson(TypedDict):
    room: str
    phrase: str
    voice_parse: str
    voice_line: str
    matched_room_words: list[str]
    bus_vel_count: int
    bus_stand_count: int
    bus_stop_count: int
    recogniser_label: str
    cue_count: int
    cue: CueJson | None
    remaining: str
    stop: str
    contact: ContactJson | None
    peak_nm: float
    peak_actuator: str
    min_up_z: float


class SummaryJson(TypedDict):
    prefer_fail: bool
    soft_pass: bool
    go_anywhere: bool
    kit_safe: bool
    plant_md5: str
    plant_file_changed: bool
    recogniser: str
    map_waypoints: bool
    room_script: bool
    vel_published: bool
    step_off_m: float
    step_off_note: str
    gap: str
    rooms: list[RoomJson]


@dataclass
class _BusCount:
    vel_count: int = 0
    stand_count: int = 0
    stop_count: int = 0

    def stand(self, now: float) -> str | None:
        del now
        self.stand_count += 1
        return None

    def stop(self, now: float) -> str | None:
        del now
        self.stop_count += 1
        return None

    def vel(self, vx: float, yaw_rate: float, now: float) -> str | None:
        del vx, yaw_rate, now
        self.vel_count += 1
        return None


@dataclass(frozen=True)
class _StandTrace:
    peak_nm: float
    peak_actuator: str
    min_up_z: float
    contact: ContactJson | None


def _plant_md5() -> str:
    return hashlib.md5(PLANT_XML.read_bytes()).hexdigest()


def _recogniser_files_present() -> bool:
    return (ROOT / "scripts" / "room_ask.py").is_file() or (
        ROOT / "scripts" / "find_room.py"
    ).is_file()


def _matched_room_words(phrase: str) -> list[str]:
    text = voice.normalize_phrase(phrase)
    hit = sorted(set(text.split()) & voice._ROOM_WORDS)
    return hit


def _parse_row(room: str, phrase: str) -> tuple[voice.VoiceCommand, _BusCount, list[str]]:
    command = voice.parse_phrase(phrase)
    bus = _BusCount()
    caller = voice.VoiceCaller(bus)
    heard = caller.hear(phrase, 0.0)
    if heard != command.line:
        raise SystemExit(f"FAIL {room}: hear line {heard!r} != parse {command.line!r}")
    if command.kind == "refuse" and caller.command is not None:
        raise SystemExit(f"FAIL {room}: a refusal replaced the bus command")
    return command, bus, _matched_room_words(phrase)


def _is_wall(name: str) -> bool:
    return name.startswith("col_room_wall") or name == "col_entrance_sill"


def _prop_contact(model: mj.MjModel, data: mj.MjData) -> ContactJson | None:
    hit = gate._prop_hit(model, data)
    if hit is None:
        return None
    robot, prop, force_n = hit
    if _is_wall(prop):
        return None
    return ContactJson(robot_geom=robot, prop_geom=prop, force_n=force_n)


def _settle(
    model: mj.MjModel,
    data: mj.MjData,
    targets: dict[str, float],
    com_z: float,
) -> _StandTrace:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[2] = com_z
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    act_idx = {
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or "": i
        for i in range(model.nu)
    }
    for joint_name, value in targets.items():
        joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise SystemExit(f"FAIL: stand joint missing: {joint_name}")
        data.qpos[model.jnt_qposadr[joint_id]] = value
    wg_set = sw.wg.set_ctrl
    wg_set(model, data, targets, act_idx)
    mj.mj_forward(model, data)
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if body_id < 0:
        raise SystemExit("FAIL: missing body_link")
    peak_abs = -1.0
    peak_signed = 0.0
    peak_name = ""
    min_up_z = 1.0
    contact: ContactJson | None = None
    steps = int(round(rooms.STAND_SETTLE_S / float(model.opt.timestep)))
    for _ in range(steps):
        mj.mj_step(model, data)
        up_z = float(data.xmat[body_id].reshape(3, 3)[2, 2])
        if up_z < min_up_z:
            min_up_z = up_z
        for name, index in act_idx.items():
            if not any(tok in name for tok in LEG_TOKS):
                continue
            force = float(data.actuator_force[index])
            if abs(force) > peak_abs:
                peak_abs = abs(force)
                peak_signed = force
                peak_name = name
        hit = _prop_contact(model, data)
        if hit is not None and (contact is None or hit["force_n"] > contact["force_n"]):
            contact = hit
    if peak_name == "":
        raise SystemExit("FAIL: no leg actuator force during the stand")
    return _StandTrace(peak_signed, peak_name, min_up_z, contact)


def _joint_q(model: mj.MjModel, data: mj.MjData, name: str) -> float:
    joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        return 0.0
    return float(data.qpos[int(model.jnt_qposadr[joint_id])])


def _finder_cues(
    model: mj.MjModel,
    data: mj.MjData,
    rgb: np.ndarray,
) -> tuple[hf.HazardCue, ...]:
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if body_id < 0 or cam_id < 0:
        raise SystemExit("FAIL: stand frame missing body_link or kit_cam")
    rot = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    yaw, pitch, roll = rc.imu_from_body(rot)
    del yaw
    cam = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
    fwd = gate.body_forward_xy(data, body_id)
    body = rc.BodyFrame(
        (float(data.qpos[0]), float(data.qpos[1])),
        (float(fwd[0]), float(fwd[1])),
    )
    pose = rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2])))
    tilt = _joint_q(model, data, "head_tilt")
    pan = _joint_q(model, data, "head_pan")
    cues = hf.corridor_gate_cues(
        hf.find_hazard_cues(rgb),
        cam=pose,
        body=body,
        imu_roll_rad=roll,
        imu_pitch_rad=pitch,
        head_tilt_rad=tilt,
        yaw_rate=0.0,
        step_off_m=STEP_OFF_M,
        head_pan_rad=pan,
    )
    # No prior floor point on a cold stand, so the first-sighting gate applies.
    return hf.confirm_leg_columns(cues, rgb)


def _cue_json(
    model: mj.MjModel,
    data: mj.MjData,
    cue: hf.HazardCue,
) -> CueJson:
    toe_gap: float | None = None
    eye: float | None = None
    side: float | None = None
    inside: bool | None = None
    if cue.u is not None and cue.v is not None and not cue.too_close:
        body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
        cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
        rot = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
        _yaw, pitch, roll = rc.imu_from_body(rot)
        cam = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
        fwd = gate.body_forward_xy(data, body_id)
        estimate = rc.estimate_hazard(
            cue.u,
            cue.v,
            cam=rc.KitCamPose((float(cam[0]), float(cam[1]), float(cam[2]))),
            body=rc.BodyFrame(
                (float(data.qpos[0]), float(data.qpos[1])),
                (float(fwd[0]), float(fwd[1])),
            ),
            imu_roll_rad=roll,
            imu_pitch_rad=pitch,
            head_tilt_rad=_joint_q(model, data, "head_tilt"),
            yaw_rate=0.0,
            step_off_m=STEP_OFF_M,
            hazard_pad_m=rc.HAZARD_PAD_M,
            head_pan_rad=_joint_q(model, data, "head_pan"),
        )
        if estimate is not None:
            toe_gap = estimate.toe_gap_m
            eye = estimate.eye_range
            side = estimate.sideways_m
            inside = estimate.in_corridor
    return CueJson(
        too_close=cue.too_close,
        bottom_clipped=cue.bottom_clipped,
        column=cue.column,
        contact_row=cue.contact_row,
        span_px=cue.span_px,
        width_px=cue.width_px,
        u=cue.u,
        v=cue.v,
        toe_gap_m=toe_gap,
        eye_range_m=eye,
        sideways_m=side,
        in_corridor=inside,
    )


def _remaining(cue: CueJson | None) -> str:
    if cue is None:
        return "no cue"
    if cue["too_close"]:
        return "too_close flag, no ray, latch not called"
    if cue["toe_gap_m"] is None:
        return "cue without a floor ray"
    return (
        f"hazard toe_gap {cue['toe_gap_m']:+.3f} m "
        f"(not a room distance; step_off {STEP_OFF_M:.3f})"
    )


def _measure_room(
    scene: rooms.RoomScene,
    targets: dict[str, float],
    com_z: float,
) -> RoomJson:
    phrase = ""
    for name, text in PHRASES:
        if name == scene.name:
            phrase = text
            break
    if phrase == "":
        raise SystemExit(f"FAIL: no phrase for {scene.name}")
    command, bus, words = _parse_row(scene.name, phrase)
    if not scene.xml_path.is_file():
        raise SystemExit(f"FAIL: missing {scene.xml_path}")
    model = mj.MjModel.from_xml_path(str(scene.xml_path))
    data = mj.MjData(model)
    trace = _settle(model, data, targets, com_z)
    renderer = mj.Renderer(model, height=rc.HEIGHT, width=rc.WIDTH)
    try:
        renderer.update_scene(data, camera="kit_cam")
        rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
    finally:
        renderer.close()
    cues = _finder_cues(model, data, rgb)
    primary = _cue_json(model, data, cues[0]) if cues else None
    return RoomJson(
        room=scene.name,
        phrase=phrase,
        voice_parse=command.kind,
        voice_line=command.line,
        matched_room_words=words,
        bus_vel_count=bus.vel_count,
        bus_stand_count=bus.stand_count,
        bus_stop_count=bus.stop_count,
        recogniser_label=RECOGNISER_ABSENT,
        cue_count=len(cues),
        cue=primary,
        remaining=_remaining(primary),
        stop="not commanded",
        contact=trace.contact,
        peak_nm=trace.peak_nm,
        peak_actuator=trace.peak_actuator,
        min_up_z=trace.min_up_z,
    )


def _summary(rows: list[RoomJson], before: str) -> SummaryJson:
    vel = any(row["bus_vel_count"] != 0 for row in rows)
    return SummaryJson(
        prefer_fail=True,
        soft_pass=False,
        go_anywhere=False,
        kit_safe=False,
        plant_md5=before,
        plant_file_changed=_plant_md5() != before,
        recogniser=RECOGNISER_ABSENT,
        map_waypoints=False,
        room_script=False,
        vel_published=vel,
        step_off_m=STEP_OFF_M,
        step_off_note=(
            "0 because no vel was published and the walk high-water was "
            "not taken. The kitchen-walk +0.017 m is not applied."
        ),
        gap=GAP,
        rooms=rows,
    )


def _print_row(row: RoomJson) -> None:
    contact = "none"
    if row["contact"] is not None:
        contact = (
            f"{row['contact']['prop_geom']} {row['contact']['force_n']:.1f} N"
        )
    cue = "none"
    if row["cue"] is not None:
        cue = (
            f"col={row['cue']['column']} too_close={row['cue']['too_close']} "
            f"span={row['cue']['span_px']} w={row['cue']['width_px']}"
        )
    print(
        f"{row['room']}  parse={row['voice_parse']}  "
        f"words={','.join(row['matched_room_words']) or '-'}  "
        f"label={row['recogniser_label']}  cues={row['cue_count']}  {cue}  "
        f"remaining={row['remaining']}  stop={row['stop']}  "
        f"contact={contact}  peak={row['peak_nm']:+.3f} Nm "
        f"{row['peak_actuator']}  min_up_z={row['min_up_z']:.3f}  "
        f"vel={row['bus_vel_count']}",
        flush=True,
    )


def main() -> int:
    if _recogniser_files_present():
        raise SystemExit(
            "FAIL: a room recogniser file is on this tip; this measure "
            "assumes it is absent and will not call it"
        )
    before = _plant_md5()
    if before != sw.PLANT_MD5 or before != voice.PLANT_MD5:
        raise SystemExit(f"FAIL: plant md5 {before}")
    if abs(rc.HAZARD_PAD_M - 0.020) > 1e-12:
        raise SystemExit(f"FAIL: hazard pad moved to {rc.HAZARD_PAD_M}")
    sw.apply_frozen_forward_gait()
    targets, com_z = rooms._stand_targets()
    rows: list[RoomJson] = []
    for name in rooms.ROOM_ORDER:
        row = _measure_room(rooms.ROOMS[name], targets, com_z)
        rows.append(row)
        _print_row(row)
    after = _plant_md5()
    if after != before:
        raise SystemExit("FAIL: plant file changed during the measure")
    summary = _summary(rows, before)
    SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
    SUMMARY_PATH.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(
        "Prefer FAIL  soft-pass=off  go-anywhere=false  "
        f"plant={before}  recogniser=absent  vel_published=false",
        flush=True,
    )
    print(GAP, flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
