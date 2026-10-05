#!/usr/bin/env python3
"""8 PM Controls motion demo on the frozen walk plant.

One sequence, empty plant, CommandBus only:

  stand → vel(+vx_fwd, 0) → vel(+vx_fwd, +yaw_rate) → vel(+vx_fwd, 0) → stop

The windows are the nav-left arc (approach 15 s, left yaw hold 12.5 s,
resume 6 s, stop hold 2.5 s). Caps stay vx_fwd 0.056, vx_back 0.032,
yaw_rate ±0.25. Soft-pass is off. The plant file is not opened for write.
Numbers will not match the previous +0.598 m / +75.7 deg basin.

A second clip is a short reverse (stand → vel(-vx_back, 0) for 5.5 s →
stop). It is not chained onto the turn. If that snippet tips, the JSON
says so and the clip is not kept.

A third clip is a side close-up of a few forward steps
(`previews/demo_step_cycle.mp4`). Other yaw orders are not re-qualified
here.

  MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
  MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py --no-video
  python scripts/demo_8pm_motion.py --check
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path
from typing import TypedDict

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import steer_walk as sw  # noqa: E402

# Existing self-test upright bar. Not lowered for this pack.
UPRIGHT_UP_Z = 0.90
# Published on main in previews/steer_walk_nav_left_summary.json.
CLAIMED_NAV_LEFT_DX_M = 0.598029205326821
CLAIMED_NAV_LEFT_DYAW_RAD = 1.3220177875448265
CLAIMED_NAV_LEFT_RESUME_M = 0.31672178880181834
CLAIMED_NAV_LEFT_MIN_UP_Z = 0.9541741213512263

# Short retreat from test_stop_settle. Separate clip, not after the turn.
SHORT_REVERSE_SCRIPT: tuple[sw.DemoSegment, ...] = (
    sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
    sw.DemoSegment(6.5, "vel", -sw.VX_BACK_CAP, 0.0, "reverse"),
    sw.DemoSegment(9.0, "stop", 0.0, 0.0, "stop"),
)

PREFER_FAIL_NOT_THIS_CLIP = (
    "This clip is the same stand → forward → left → resume → stop windows. "
    "It does not match the previous basin's +0.598 m / +75.7 deg envelope. "
    "Other orders (a 14 s left that starts at 15 s, a second same-sign arc, "
    "right-then-left) are not this clip and were not re-qualified here. "
    "Tip and up_z bars are unchanged. Caps are unchanged. Soft-pass is off."
)

# Side view of a few forward steps. Same bus and caps. Not the nav arc.
STEP_CLOSEUP_SCRIPT: tuple[sw.DemoSegment, ...] = (
    sw.DemoSegment(0.6, "stand", 0.0, 0.0, "stand"),
    sw.DemoSegment(6.2, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    sw.DemoSegment(7.6, "stop", 0.0, 0.0, "stop"),
)

OVERLAY_TITLE = "8PM motion"
OVERLAY_FOOTER = "CommandBus  stand | stop | vel(vx, yaw_rate)  |  voice uses this bus"


class BusCommand(TypedDict):
    t0_s: float
    t1_s: float
    api: str
    call: str
    vx: float
    yaw_rate: float
    label: str


def bus_commands(script: tuple[sw.DemoSegment, ...]) -> list[BusCommand]:
    """One row per CommandBus window. vel is resent at 10 Hz inside the window."""
    t0 = 0.0
    rows: list[BusCommand] = []
    for seg in script:
        if seg.kind == "stand":
            call = "stand"
        elif seg.kind == "stop":
            call = "stop"
        else:
            call = f"vel({seg.vx:.3f}, {seg.yaw_rate:+.3f})"
        rows.append(
            BusCommand(
                t0_s=t0,
                t1_s=seg.t_end,
                api=seg.kind,
                call=call,
                vx=seg.vx,
                yaw_rate=seg.yaw_rate,
                label=seg.label,
            )
        )
        t0 = seg.t_end
    return rows


def _upright(summary: sw.RunSummary) -> bool:
    return (
        (not summary.fault)
        and (not summary.tip)
        and summary.min_up_z >= UPRIGHT_UP_Z
        and summary.end_mode == "stand"
        and summary.cop_in_box
        and summary.max_leg_tau_nm <= sw.LEG_TAU + 1e-3
    )


def _close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol


def assert_freeze() -> None:
    digest = sw._md5(sw.PLANT_XML)
    if digest != sw.PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {sw.PLANT_MD5}")
    if sw.PLANT_MD5 != "17dc4ff37491c8e61900fd83b5d31f0c":
        raise SystemExit("PLANT_MD5 constant left the thawed plant digest")
    if sw.VX_FWD_CAP != 0.056 or sw.VX_BACK_CAP != 0.032 or sw.YAW_RATE_CAP != 0.25:
        raise SystemExit(
            f"caps moved: vx_fwd={sw.VX_FWD_CAP} vx_back={sw.VX_BACK_CAP} "
            f"yaw={sw.YAW_RATE_CAP}"
        )
    script = sw.NAV_LEFT_SCRIPT
    bounds = sw.script_bounds(script)
    turn_s = bounds["turn"][1] - bounds["turn"][0]
    approach_s = bounds["forward"][1] - bounds["forward"][0]
    resume_s = bounds["resume"][1] - bounds["resume"][0]
    if abs(turn_s - sw.CLAIMED_LEFT_ARC_S) > 1e-9:
        raise SystemExit(f"left arc {turn_s:.3f} s left the claimed 12.5 s window")
    if abs(approach_s - sw.CLAIMED_APPROACH_S) > 1e-9:
        raise SystemExit(f"approach {approach_s:.3f} s left the claimed 15 s window")
    if abs(resume_s - sw.CLAIMED_RESUME_S) > 1e-9:
        raise SystemExit(f"resume {resume_s:.3f} s left the claimed 6 s window")
    turn = script[2]
    if turn.kind != "vel" or turn.vx != sw.VX_FWD_CAP or turn.yaw_rate != sw.YAW_RATE_CAP:
        raise SystemExit("turn window is not vel(+vx_fwd, +yaw_rate) at the caps")
    forward = script[1]
    resume = script[3]
    if forward.kind != "vel" or forward.yaw_rate != 0.0 or forward.vx != sw.VX_FWD_CAP:
        raise SystemExit("forward window is not vel(+vx_fwd, 0)")
    if resume.kind != "vel" or resume.yaw_rate != 0.0 or resume.vx != sw.VX_FWD_CAP:
        raise SystemExit("resume window is not vel(+vx_fwd, 0)")
    if script[0].kind != "stand" or script[-1].kind != "stop":
        raise SystemExit("sequence is not stand … stop")


def _still_times(script: tuple[sw.DemoSegment, ...]) -> list[tuple[str, float]]:
    t0 = 0.0
    shots: list[tuple[str, float]] = []
    for seg in script:
        span = seg.t_end - t0
        if seg.kind == "stand":
            t = seg.t_end - min(0.2, 0.5 * span)
        elif seg.kind == "stop":
            t = seg.t_end - min(0.4, 0.5 * span)
        else:
            t = t0 + 0.55 * span
        shots.append((seg.label, max(0.0, t)))
        t0 = seg.t_end
    return shots


def _extract_stills(mp4: Path, script: tuple[sw.DemoSegment, ...], prefix: str) -> list[str]:
    written: list[str] = []
    for label, t in _still_times(script):
        out = sw.PREVIEWS / f"{prefix}_{label}.png"
        cmd = [
            "ffmpeg", "-y",
            "-ss", f"{t:.3f}",
            "-i", str(mp4),
            "-frames:v", "1",
            str(out),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            raise RuntimeError(f"ffmpeg still failed for {label}: {proc.stderr[-500:]}")
        written.append(str(out.relative_to(sw.ROOT)))
    return written


def _segment_rows(summary: sw.RunSummary) -> list[dict[str, float | str | bool]]:
    rows: list[dict[str, float | str | bool]] = []
    for row in summary.segment_stats:
        item = asdict(row)
        item["dyaw_deg"] = math.degrees(row.dyaw_rad)
        rows.append(item)
    return rows


def _pack(
    summary: sw.RunSummary,
    script: tuple[sw.DemoSegment, ...],
    *,
    clip: str,
    claimed_match: bool | None,
) -> dict[str, object]:
    upright = _upright(summary)
    world_dx = float(sum(row.dx_m for row in summary.segment_stats))
    world_dy = float(sum(row.dy_m for row in summary.segment_stats))
    return {
        "clip": clip,
        "plant": str(sw.PLANT_XML.relative_to(sw.ROOT)),
        "plant_md5": sw._md5(sw.PLANT_XML),
        "soft_pass": False,
        "caps_used": {
            "vx_fwd": sw.VX_FWD_CAP,
            "vx_back": sw.VX_BACK_CAP,
            "yaw_rate": sw.YAW_RATE_CAP,
        },
        "commanded_at_cap": True,
        "ceilings_raised": False,
        "bus": "CommandBus.stand | stop | vel(vx, yaw_rate)",
        "vel_resend_hz": 10.0,
        "watchdog_s": sw.COMMAND_TIMEOUT_S,
        "bus_commands": bus_commands(script),
        "delta_x_m": summary.dx_forward_m,
        "delta_yaw_rad": summary.dyaw_turn_rad if summary.dyaw_turn_rad != 0.0 else summary.dyaw_end_rad,
        "delta_x_world_m": world_dx,
        "delta_y_world_m": world_dy,
        "delta_yaw_end_rad": summary.dyaw_end_rad,
        "delta_yaw_end_deg": math.degrees(summary.dyaw_end_rad),
        "delta_x_resume_m": summary.dx_resume_m,
        "min_up_z": summary.min_up_z,
        "peak_tau_nm": summary.peak_torque_nm,
        "end_mode": summary.end_mode,
        "upright": upright,
        "tip": summary.tip,
        "fault": summary.fault,
        "fault_reason": summary.fault_reason,
        "cop_in_box": summary.cop_in_box,
        "end_margin_m": summary.end_margin_m,
        "upright_bar_up_z": UPRIGHT_UP_Z,
        "tip_bar_up_z": sw.TIP_UP_Z,
        "segments": _segment_rows(summary),
        "matches_claimed_nav_left": claimed_match,
        "honesty": summary.honesty,
    }


def _claimed_match(summary: sw.RunSummary) -> bool:
    return (
        _close(summary.dx_forward_m, CLAIMED_NAV_LEFT_DX_M, 1e-4)
        and _close(summary.dyaw_turn_rad, CLAIMED_NAV_LEFT_DYAW_RAD, 1e-4)
        and _close(summary.dx_resume_m, CLAIMED_NAV_LEFT_RESUME_M, 1e-4)
        and _close(summary.min_up_z, CLAIMED_NAV_LEFT_MIN_UP_Z, 1e-4)
        and summary.end_mode == "stand"
        and (not summary.tip)
        and summary.cop_in_box
    )


def _run(
    script: tuple[sw.DemoSegment, ...],
    stem: str,
    *,
    video: bool,
    cam_distance: float = 1.25,
    cam_azimuth: float = 135.0,
    cam_elevation: float = -18.0,
) -> sw.RunSummary:
    duration = script[-1].t_end
    out = sw.PREVIEWS / f"{stem}.mp4" if video else None
    return sw.run_demo(
        duration=duration,
        out_mp4=out,
        log_path=sw.PREVIEWS / f"{stem}_log.txt",
        summary_path=None,
        script=script,
        overlay_title=OVERLAY_TITLE,
        overlay_footer=OVERLAY_FOOTER,
        cam_distance=cam_distance,
        cam_azimuth=cam_azimuth,
        cam_elevation=cam_elevation,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="8PM CommandBus motion demo on frozen M145")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--no-reverse", action="store_true", help="Skip the short reverse snippet")
    ap.add_argument("--check", action="store_true", help="Freeze, caps, and claimed window only")
    args = ap.parse_args()
    assert_freeze()
    if args.check:
        print(
            f"[8pm] check ok plant={sw.PLANT_MD5} "
            f"caps vx=+{sw.VX_FWD_CAP}/-{sw.VX_BACK_CAP} yaw=±{sw.YAW_RATE_CAP} "
            "script=nav-left claimed windows soft-pass=off"
        )
        return

    video = not args.no_video
    primary = _run(sw.NAV_LEFT_SCRIPT, "demo_8pm_motion", video=video)
    match = _claimed_match(primary)
    primary_pack = _pack(primary, sw.NAV_LEFT_SCRIPT, clip="stand-forward-left-resume-stop", claimed_match=match)
    stills: list[str] = []
    motion_mp4 = sw.PREVIEWS / "demo_8pm_motion.mp4"
    if video and motion_mp4.is_file():
        stills = _extract_stills(motion_mp4, sw.NAV_LEFT_SCRIPT, "demo_8pm")

    reverse_pack: dict[str, object] | None = None
    if not args.no_reverse:
        reverse = _run(SHORT_REVERSE_SCRIPT, "demo_8pm_reverse", video=video)
        reverse_pack = _pack(
            reverse,
            SHORT_REVERSE_SCRIPT,
            clip="stand-reverse-stop",
            claimed_match=None,
        )
        reverse_mp4 = sw.PREVIEWS / "demo_8pm_reverse.mp4"
        if not bool(reverse_pack["upright"]):
            reverse_pack["kept_as_demo_clip"] = False
            reverse_pack["prefer_fail"] = (
                "Short reverse tipped or left the upright bar. "
                "Clip is not part of the show. Bars were not lowered."
            )
            if reverse_mp4.is_file():
                reverse_mp4.unlink()
        else:
            reverse_pack["kept_as_demo_clip"] = True
            if video and reverse_mp4.is_file():
                stills.extend(_extract_stills(reverse_mp4, SHORT_REVERSE_SCRIPT, "demo_8pm_reverse"))

    payload: dict[str, object] = {
        "title": "8PM Controls motion demo",
        "audience": "Dave / Controls",
        "scene": "empty frozen walk plant (no kitchen, no door, no map)",
        "voice": "same CommandBus: stand | stop | vel(vx, yaw_rate)",
        "plant": primary_pack["plant"],
        "plant_md5": primary_pack["plant_md5"],
        "plant_md5_expected": sw.PLANT_MD5,
        "plant_file_edited": False,
        "soft_pass": False,
        "caps_used": primary_pack["caps_used"],
        "bus_commands": primary_pack["bus_commands"],
        "delta_x_m": primary_pack["delta_x_m"],
        "delta_yaw_rad": primary_pack["delta_yaw_rad"],
        "delta_yaw_deg": math.degrees(float(primary_pack["delta_yaw_rad"])),
        "delta_x_resume_m": primary_pack["delta_x_resume_m"],
        "delta_x_world_m": primary_pack["delta_x_world_m"],
        "delta_y_world_m": primary_pack["delta_y_world_m"],
        "delta_yaw_end_rad": primary_pack["delta_yaw_end_rad"],
        "min_up_z": primary_pack["min_up_z"],
        "peak_tau_nm": primary_pack["peak_tau_nm"],
        "end_mode": primary_pack["end_mode"],
        "upright": primary_pack["upright"],
        "tip": primary_pack["tip"],
        "cop_in_box": primary_pack["cop_in_box"],
        "matches_claimed_nav_left": match,
        "claimed_nav_left": {
            "source": "previews/steer_walk_nav_left_summary.json on main",
            "delta_x_m": CLAIMED_NAV_LEFT_DX_M,
            "delta_yaw_rad": CLAIMED_NAV_LEFT_DYAW_RAD,
            "delta_yaw_deg": math.degrees(CLAIMED_NAV_LEFT_DYAW_RAD),
            "delta_x_resume_m": CLAIMED_NAV_LEFT_RESUME_M,
            "min_up_z": CLAIMED_NAV_LEFT_MIN_UP_Z,
        },
        "stills": stills,
        "video": "previews/demo_8pm_motion.mp4" if video else None,
        "sequence": primary_pack,
        "reverse_snippet": reverse_pack,
        "prefer_fail_not_this_clip": PREFER_FAIL_NOT_THIS_CLIP,
    }
    print(
        f"[8pm] upright={payload['upright']} tip={payload['tip']} "
        f"Δx={float(payload['delta_x_m']):+.3f} m "
        f"Δyaw={float(payload['delta_yaw_deg']):+.2f} deg "
        f"min_up_z={float(payload['min_up_z']):.3f} "
        f"peak_τ={float(payload['peak_tau_nm']):.2f} "
        f"end={payload['end_mode']} "
        f"claimed_match={match} "
        f"md5={payload['plant_md5']}"
    )
    if reverse_pack is not None:
        print(
            f"[8pm] reverse upright={reverse_pack['upright']} "
            f"Δx={float(reverse_pack['delta_x_m']):+.3f} m "
            f"min_up_z={float(reverse_pack['min_up_z']):.3f} "
            f"kept={reverse_pack.get('kept_as_demo_clip')}"
        )

    close = _run(
        STEP_CLOSEUP_SCRIPT,
        "demo_step_cycle",
        video=video,
        cam_distance=0.82,
        cam_azimuth=78.0,
        cam_elevation=-8.0,
    )
    close_pack = _pack(
        close,
        STEP_CLOSEUP_SCRIPT,
        clip="stand-forward-stop-side",
        claimed_match=None,
    )
    payload["step_cycle"] = close_pack
    if not bool(close_pack["upright"]):
        close_pack["prefer_fail"] = (
            "Forward close-up left the upright bar. Not a success clip. "
            "Bars were not lowered."
        )
        close_mp4 = sw.PREVIEWS / "demo_step_cycle.mp4"
        if close_mp4.is_file():
            close_mp4.unlink()
    print(
        f"[8pm] step-cycle upright={close_pack['upright']} "
        f"Δx={float(close_pack['delta_x_m']):+.3f} m "
        f"min_up_z={float(close_pack['min_up_z']):.3f}"
    )
    out_json = sw.PREVIEWS / "demo_8pm_motion_summary.json"
    out_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[8pm] wrote {out_json}")
    if not bool(payload["upright"]):
        raise SystemExit(
            "Prefer FAIL: primary sequence is not upright. "
            "Summary written. Bars were not lowered."
        )


if __name__ == "__main__":
    main()
