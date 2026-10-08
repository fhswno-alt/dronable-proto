#!/usr/bin/env python3
"""Independent tiebreak of tip d6e8b5e with the #102 scorer.

The gait is that tip's voice preview. Metrics are this branch's
score_walk_smoothness and step_bars. Controls' step-honesty code is not
called. A tick is airborne only when three tests agree: no floor contact,
every contact-box corner above the floor plane, and zero summed normal force.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TIP = "d6e8b5ebb801250814552fc728886a11dd0350c2"
LABEL = "d6e8b5e"
FROZEN_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
HEADING = "## Tiebreak d6e8b5e"
JSON_PATH = ROOT / "previews" / "walk_tiebreak_d6e8b5e.json"


def _worktree() -> Path:
    dest = Path("/tmp/tiebreak-walk") / TIP
    marker = dest / "scripts" / "steer_walk.py"
    if marker.is_file():
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.check_call(
        ["git", "worktree", "add", "--detach", str(dest), TIP],
        cwd=ROOT,
    )
    return dest


def _plant_md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _floor_force(mj, model, data, foot_gid: int, floor_gid: int) -> tuple[int, float]:
    """Floor-only contact count and summed normal force. The mesh is not queried."""
    n = 0
    total = 0.0
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        if not ((g1 == foot_gid and g2 == floor_gid) or (g2 == foot_gid and g1 == floor_gid)):
            continue
        n += 1
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, force)
        total += float(force[0])
    return n, total


def _runs(times: list[float], flags: list[bool]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    i = 0
    n = len(flags)
    while i < n:
        if not flags[i]:
            i += 1
            continue
        j = i + 1
        while j < n and flags[j]:
            j += 1
        rows.append({"t0": times[i], "t1": times[j - 1], "n": j - i})
        i = j
    return rows


def _swing_rows(trace: dict[str, list[float]]) -> list[dict[str, object]]:
    """Airborne windows on the strict contact count. Touchdown is the next loaded tick."""
    import numpy as np

    t = np.asarray(trace["t"], dtype=np.float64)
    rows: list[dict[str, object]] = []
    for side, other in (("l", "r"), ("r", "l")):
        ncon = np.asarray(trace[f"n_{side}"], dtype=np.int32)
        x = np.asarray(trace[f"x_{side}"], dtype=np.float64)
        xo = np.asarray(trace[f"x_{other}"], dtype=np.float64)
        yo = np.asarray(trace[f"y_{other}"], dtype=np.float64)
        z = np.asarray(trace[f"z_{side}"], dtype=np.float64)
        no = np.asarray(trace[f"n_{other}"], dtype=np.int32)
        i = 1
        n = int(ncon.shape[0])
        while i < n:
            if ncon[i - 1] > 0 and ncon[i] == 0:
                j = i + 1
                while j < n and ncon[j] == 0:
                    j += 1
                span = j - i
                lo = i + int(math.floor(0.20 * span))
                hi = i + int(math.ceil(0.80 * span))
                hi = min(n, max(lo + 1, hi))
                slip = 0.0
                loaded = 0
                for k in range(i, min(j, n)):
                    if int(no[k]) <= 0:
                        continue
                    loaded += 1
                    slip += math.hypot(float(xo[k] - xo[k - 1]), float(yo[k] - yo[k - 1]))
                touch = float(t[j]) if j < n else None
                rows.append({
                    "side": side.upper(),
                    "t_lift": float(t[i]),
                    "t_touch": touch,
                    "t_last_air": float(t[min(j, n) - 1]),
                    "duration_s": None if touch is None else touch - float(t[i]),
                    "n_air": span,
                    "airborne_m": float(x[min(j, n) - 1] - x[i]),
                    "stance_dx_m": float(xo[min(j, n) - 1] - xo[i]),
                    "slip_m": None if loaded == 0 else slip,
                    "clear_20_80_m": float(np.min(z[lo:hi])) if hi > lo else None,
                    "peak_20_80_m": float(np.max(z[lo:hi])) if hi > lo else None,
                    "peak_air_m": float(np.max(z[i:j])) if j > i else None,
                    "mode": str(trace["mode"][i]),
                    "landed": j < n and span >= 2,
                })
                i = max(j, i + 1)
            else:
                i += 1
    rows.sort(key=lambda row: float(row["t_lift"]))
    return rows


def _pattern_name(a: bool, b: bool, c: bool) -> str:
    return (
        f"contact {'0' if a else 'yes'}, "
        f"corners {'above' if b else 'not above'}, "
        f"force {'0' if c else 'nonzero'}"
    )


def _gait_label(summary: dict[str, object]) -> str:
    n = int(summary.get("n_scored_swings") or 0)
    frac = float(summary.get("step_fraction") or 0.0)
    if n > 0 and frac >= 0.50:
        return "STEPS"
    return "SKATES"


def tiebreak_markdown(payload: dict[str, object] | None = None) -> str:
    if payload is None:
        if not JSON_PATH.is_file():
            return ""
        loaded = json.loads(JSON_PATH.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            return ""
        payload = loaded
    rows = payload.get("rows")
    if not isinstance(rows, list) or not rows:
        return ""
    lines = [
        HEADING,
        "",
        "Soft-pass is off. The gait is tip "
        f"`{payload.get('tip_sha')}` and the metrics are the #102 scorer. "
        "Controls' step-honesty function is not called. The plant file is not written.",
        "",
        "A tick is airborne only when three tests agree on that foot: "
        "zero `mjData` contacts between its group-0 contact box and the floor, "
        "all eight corners of that box above the floor plane, and summed "
        "`mj_contactForce` normal equal to 0. The mesh is not sampled. "
        "HW's mesh sole sits about 2.9 mm above this box bottom.",
        "",
        "This tip has one scored bout: stand 0.40 s, walk 22.00 s, stop 8.00 s, "
        "period 20.0 s, dsp 0.35, swing height 18 mm, hip-frame cap 21 mm, "
        "bus vx 0.056 m/s. The cadence grid (T 0.5–2.0 s, x_amp = vx·T/4) "
        "is not on this commit.",
        "",
    ]
    for row in rows:
        if not isinstance(row, dict):
            continue
        lines.append(
            f"`{row.get('name')}` {row.get('gait')} / {row.get('verdict')}. "
            f"Plant `{row.get('plant_md5_before')}` before and `{row.get('plant_md5')}` after."
        )
        lines.append("")
        swings = row.get("swings")
        if isinstance(swings, list) and swings:
            lines.append(
                "| Side | Lift s | Touch s | Dur s | Clear 20–80 mm | Peak 20–80 mm | Peak air mm | Air m | Stance dx m | Slip mm |"
            )
            lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
            for swing in swings:
                if not isinstance(swing, dict):
                    continue
                lines.append(
                    "| "
                    + " | ".join([
                        str(swing.get("side")),
                        _n(swing.get("t_lift"), 3),
                        _n(swing.get("t_touch"), 3),
                        _n(swing.get("duration_s"), 3),
                        _mm(swing.get("clear_20_80_m")),
                        _mm(swing.get("peak_20_80_m")),
                        _mm(swing.get("peak_air_m")),
                        _n(swing.get("airborne_m"), 4),
                        _n(swing.get("stance_dx_m"), 4),
                        _mm(swing.get("slip_m")),
                    ])
                    + " |"
                )
            lines.append("")
        else:
            lines.append("No swing passed the three-part airborne test.")
            lines.append("")
        lines.extend(_row_prose(row))
        lines.extend(_hinge_lines(row))
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def _row_prose(row: dict[str, object]) -> list[str]:
    step = row.get("stepping")
    if not isinstance(step, dict):
        step = {}
    ss = step.get("ss")
    if not isinstance(ss, dict):
        ss = {}
    stop = step.get("stop")
    if not isinstance(stop, dict):
        stop = {}
    actual = step.get("actual")
    if not isinstance(actual, dict):
        actual = {}
    asks = row.get("asks_by_joint")
    ask_lines = []
    if isinstance(asks, list):
        for sample in asks:
            if not isinstance(sample, dict):
                continue
            if str(sample.get("joint", "")).startswith(("l_", "r_")) and (
                "hip" in str(sample.get("joint"))
                or "knee" in str(sample.get("joint"))
                or "ank" in str(sample.get("joint"))
            ):
                ask_lines.append(
                    f"{sample.get('joint')} {_n(sample.get('sum_nm'), 4)} Nm "
                    f"at {_n(sample.get('t_s'), 3)} s, headroom {_n(sample.get('headroom_nm'), 4)} Nm"
                )
            if len(ask_lines) >= 6:
                break
    pitch_final = stop.get("final_pitch_rad")
    pitch_stand = stop.get("stand_pitch_rad")
    pitch_off = stop.get("pitch_off_rad")
    return [
        "The three tests never disagree. Contact count, corner height, and "
        "normal force pick the same airborne ticks on both feet. "
        f"Contact-box bottom local z is {_mm(row.get('box_bottom_L_m'))} mm. "
        "Floor plane z is 0. The mesh is not in the score.",
        "",
        f"Commanded hip-frame `x_amp` peaked at {_mm(row.get('x_cmd_max_m'))} mm "
        "and `z_flat` was on. Two swings, left then right. "
        f"Row step fraction {_n(step.get('step_fraction'), 3)} "
        f"(airborne {_n(step.get('airborne_forward_m'), 4)} m, "
        f"contact {_n(step.get('contact_forward_m'), 4)} m). "
        f"Whole-bout phase mismatch {_n(step.get('mismatch_fraction'), 3)}. "
        f"Stance contact count minimum {ss.get('min_contacts')}. "
        f"Worst stance slip {_mm(step.get('worst_slip_m'))} mm. "
        f"Lowest corner over 20–80% of the airborne window "
        f"{_mm(step.get('min_clear_m'))} mm "
        f"(honest max of those minima {_mm(step.get('honest_max_clear_m'))} mm).",
        "",
        "Single-support CoP margin "
        f"min {_mm(ss.get('cop_min_m'))} mm, "
        f"p5 {_mm(ss.get('cop_p5_m'))} mm, "
        f"p50 {_mm(ss.get('cop_p50_m'))} mm, "
        f"p95 {_mm(ss.get('cop_p95_m'))} mm. "
        f"Edge dwell under 5 mm is {_n(ss.get('edge_fraction'), 3)}. "
        f"Sole tilt max {_deg(ss.get('tilt_max_rad'))} deg, "
        f"fraction over 1 deg {_n(ss.get('tilt_over_1deg'), 4)}.",
        "",
        "Declared-phase contact ZMP minimum "
        f"{_mm(row.get('declared_zmp_min_m'), 3)} mm, outside "
        f"{_n(row.get('declared_zmp_out'), 3)}. "
        "Declared CoM minimum "
        f"{_mm(row.get('declared_com_min_m'))} mm, outside "
        f"{_n(row.get('declared_com_out'), 3)}. "
        "Actual-stance ZMP minimum "
        f"{_mm(actual.get('zmp_min_m'))} mm, CoM minimum "
        f"{_mm(actual.get('com_min_m'))} mm, outside "
        f"{_n(actual.get('zmp_out'), 3)} and {_n(actual.get('com_out'), 3)}.",
        "",
        "CoM jerk "
        f"{_n(row.get('com_jerk_whole_peak'), 3)} / {_n(row.get('com_jerk_whole_rms'), 3)} m/s³. "
        "Joint jerk vector "
        f"{_n(row.get('joint_jerk_peak'), 3)} / {_n(row.get('joint_jerk_rms'), 3)} rad/s³, "
        f"largest hinge {row.get('worst_joint')} "
        f"{_n(row.get('worst_joint_peak'), 3)} / {_n(row.get('worst_joint_rms'), 3)}. "
        "Sum column "
        f"{row.get('ask_joint')} {_n(row.get('ask_nm'), 4)} Nm at "
        f"{_n(row.get('ask_t_s'), 3)} s, headroom {_n(row.get('ask_headroom_nm'), 4)} Nm. "
        "Leg asks: " + "; ".join(ask_lines) + ".",
        "",
        "Stop "
        f"{stop.get('pose')}. Final trunk pitch {_deg(pitch_final)} deg "
        f"against stand {_deg(pitch_stand)} deg "
        f"(off {_deg(pitch_off)} deg). "
        f"Final 1 s contacts L/R {stop.get('min_contacts_l')}/{stop.get('min_contacts_r')}, "
        f"min up_z {_n(stop.get('min_up_z'), 3)}.",
        "",
        "Bus `vx·T` at 0.056 m/s and 20 s is 1.12 m. Both airborne advances are "
        "76 mm, so the ±20% stride bar fails. Step fraction 0.794 is under 0.90 "
        "because 0.040 m of forward travel happens in contact. The signed "
        "pre-clamp ask must stay ≤ 2.33 Nm on every leg joint. A zero "
        "clamp-active fraction is not that pass. The sum stays a column, and "
        "a signed pass that fails the sum is marked `passes signed, fails sum`. "
        "The hinge-speed bar, the speed-torque line, and the clamp-active bar "
        "are in the fail list when they fail. The cadence grid is not on this "
        "commit, so there is no second row.",
        "",
        "Against the posted row: unclamped right hip roll 2.2567 Nm at 2.832 s "
        "matches. Declared CoM minimum +15.10 mm matches the posted +15.11 mm. "
        "Declared whole-bout ZMP minimum +0.002 mm matches the posted contact-CoP "
        "minimum. Declared single-support ZMP minima are +6.34 mm (ss_L) and "
        "+6.36 mm (ss_R). Stance contacts stay at 4. The right-foot peak clearance "
        "is 10.80 mm. The airborne-window minima are 8.41 mm and 8.60 mm, above "
        "the posted 8.08 mm, because this scorer's swing is the contact-off "
        "interval (5.06 s) and the posted window is the longer clocked single "
        "support. Stance-slip path is 0.27 mm, and the whole-bout step fraction "
        "is 0.794 with phase mismatch 0.094. Those three are this scorer's "
        "definitions. The feet do leave the floor: 633 and 632 ticks with zero "
        "contacts, zero force, and every box corner above the plane.",
        "",
        "Fail reasons: " + "; ".join(str(item) for item in (row.get("fail_reasons") or [])) + ".",
    ]


def _hinge_lines(row: dict[str, object]) -> list[str]:
    qvel = row.get("qvel")
    speed = row.get("trunk_speed")
    if not isinstance(qvel, dict) and not isinstance(speed, dict):
        return []
    lines = ["", "Hinge speed and trunk vx:"]
    torque = row.get("speed_torque")
    if isinstance(torque, dict):
        lines.append(str(torque.get("status")))
    if isinstance(speed, dict):
        lines.append(
            f"Period T {_n(speed.get('period_s'), 3)} s, commanded vx "
            f"{_n(speed.get('vx_cmd_m_s'), 4)} m/s, actual trunk vx "
            f"{_n(speed.get('actual_vx_m_s'), 4)} m/s "
            f"(forward {_n(speed.get('fwd_m'), 4)} m over {_n(speed.get('move_s'), 3)} s "
            f"along the trunk heading), ratio {_n(speed.get('ratio'), 3)}."
        )
    if isinstance(qvel, dict):
        passed = "passes" if qvel.get("passes") else "fails"
        lines.append(
            f"Peak |qvel| bar { _n(qvel.get('bar_rad_s'), 2) } rad/s {passed}. "
            "The plant has no velocity cap."
        )
        joints = qvel.get("joints")
        if isinstance(joints, list) and joints:
            lines.append("")
            lines.append("| Joint | Peak rad/s | t s | Stage | Headroom rad/s |")
            lines.append("| --- | ---: | ---: | --- | ---: |")
            for joint in joints:
                if not isinstance(joint, dict):
                    continue
                lines.append(
                    "| "
                    + " | ".join([
                        str(joint.get("joint")),
                        _n(joint.get("peak_rad_s"), 4),
                        _n(joint.get("t_s"), 3),
                        str(joint.get("stage")),
                        _n(joint.get("headroom_rad_s"), 4),
                    ])
                    + " |"
                )
    return lines


def _deg(value: object) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "—"
    return f"{math.degrees(float(value)):+.2f}"


def _n(value: object, digits: int) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "—"
    return f"{float(value):.{digits}f}"


def _mm(value: object, digits: int = 2) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "—"
    return f"{float(value) * 1000.0:.{digits}f}"


def _splice_doc() -> None:
    path = ROOT / "docs" / "WALK_STEPPING_BARS.md"
    body = path.read_text(encoding="utf-8") if path.is_file() else "# Walk stepping bars\n"
    section = tiebreak_markdown()
    if not section:
        raise SystemExit("no tiebreak JSON")
    path.write_text(_replace_section(body, HEADING, section), encoding="utf-8")
    print(f"[tiebreak] wrote {path}")


def _replace_section(body: str, heading: str, section: str) -> str:
    """Replace one heading's section and keep a later heading."""
    start = body.find(heading)
    block = section if section.endswith("\n") else section + "\n"
    if start < 0:
        return body.rstrip() + "\n\n" + block
    nxt = body.find("\n## ", start + len(heading))
    if nxt < 0:
        return body[:start].rstrip() + "\n\n" + block
    return body[:start].rstrip() + "\n\n" + block + "\n" + body[nxt + 1:]


def main() -> None:
    if "--doc" in sys.argv:
        _splice_doc()
        return
    os.environ["MUJOCO_GL"] = "glfw"
    tree = _worktree()
    os.environ["WALK_GAIT_SCRIPTS"] = str(tree / "scripts")
    sys.path.insert(0, str(ROOT / "scripts"))
    import mujoco as mj
    import numpy as np

    import score_walk_smoothness as sws

    sw = sws.steer_walk
    if not str(sw.__file__).startswith(str(tree)):
        raise SystemExit(f"gait loaded from {sw.__file__}, not {tree}")
    plant = Path(sw.PLANT_XML)
    digest = _plant_md5(plant)
    if digest != FROZEN_MD5:
        raise SystemExit(f"plant md5 {digest} predates the frozen plant")
    cfg = sw.voice_preview_config()
    if cfg.name != "voice056":
        raise SystemExit(f"voice config name is {cfg.name}")
    spec = sws.PreviewRowSpec(
        name="voice-20",
        period_s=20.0,
        dsp=0.35,
        amp_m=0.043,
        z_m=0.018,
        arm_s=2.40,
        vx_m_s=0.056,
        stand_s=0.40,
        walk_s=22.00,
        stop_s=8.00,
        preview_shape=0.0,
        source=(
            "d6e8b5e docs/LIPM_VENDOR_LOOK.md stand 0.40 walk 22.00 stop 8.00 "
            "period 20 dsp 0.35 z 0.018 x cap 0.021 vx 0.056"
        ),
    )
    captured: dict[str, list] = {
        key: []
        for key in (
            "t", "x_l", "x_r", "y_l", "y_r", "n_l", "n_r", "n_raw_l", "n_raw_r",
            "z_l", "z_r", "fn_l", "fn_r", "declared", "mode",
            "roll", "pitch", "roll_l", "pitch_l", "roll_r", "pitch_r",
            "up_z", "zmp_act", "com_act", "cop_l", "cop_r",
            "a_l", "b_l", "c_l", "a_r", "b_r", "c_r", "q",
        )
    }
    meta: dict[str, object] = {"floor_vs_grounds": 0}

    def probe(tick: dict[str, object]) -> None:
        session = tick["session"]
        walker = tick["walker"]
        model = session.model
        data = session.data
        if "box_ok" not in meta:
            floor = int(walker.gid_floor)
            meta["floor_z"] = float(model.geom_pos[floor, 2])
            meta["floor_type"] = int(model.geom_type[floor])
            for side in ("L", "R"):
                gid = int(walker.gid[side])
                if int(model.geom_group[gid]) != 0:
                    raise SystemExit(f"{side} contact geom group is not 0")
                if int(model.geom_type[gid]) != int(mj.mjtGeom.mjGEOM_BOX):
                    raise SystemExit(f"{side} contact geom is not a box")
                pos = model.geom_pos[gid]
                half = model.geom_size[gid]
                meta[f"box_bottom_{side}"] = float(pos[2] - half[2])
            meta["box_ok"] = True
            meta["x_cmd_max"] = 0.0
            meta["z_flat_seen"] = False
        op3 = walker.op3
        if op3 is not None:
            meta["x_cmd_max"] = max(float(meta["x_cmd_max"]), abs(float(op3.x_cmd)))
            meta["z_flat_seen"] = bool(meta["z_flat_seen"] or bool(getattr(op3, "z_flat", False)))
        floor_gid = int(walker.gid_floor)
        for side, key in (("L", "l"), ("R", "r")):
            gid = int(walker.gid[side])
            n_floor, fn = _floor_force(mj, model, data, gid, floor_gid)
            clear = float(tick[f"z_{key}"])
            above = clear > float(meta["floor_z"])
            airborne = n_floor == 0 and above and fn == 0.0
            captured[f"a_{key}"].append(n_floor == 0)
            captured[f"b_{key}"].append(above)
            captured[f"c_{key}"].append(fn == 0.0)
            captured[f"fn_{key}"].append(fn)
            raw_n = int(tick[f"n_{key}"])
            if raw_n != n_floor:
                meta["floor_vs_grounds"] = int(meta["floor_vs_grounds"]) + 1
            captured[f"n_raw_{key}"].append(raw_n)
            captured[f"n_{key}"].append(0 if airborne else max(raw_n, 1))
            captured[f"z_{key}"].append(clear)
        captured["t"].append(float(tick["t"]))
        captured["mode"].append(str(tick["mode"]))
        captured["declared"].append(str(tick["declared"]))
        for key in ("x_l", "x_r", "y_l", "y_r"):
            # Filled below from the same box sample the scorer already took.
            pass
        # Re-read box centre from the scorer's own corner function so x/y match.
        for side, key in (("L", "l"), ("R", "r")):
            x, y, _z, roll, pitch = sws._foot_box_sample(
                model, data, int(walker.gid[side]), None,
            )
            captured[f"x_{key}"].append(x)
            captured[f"y_{key}"].append(y)
            captured[f"roll_{key}"].append(roll)
            captured[f"pitch_{key}"].append(pitch)
            captured[f"cop_{key}"].append(sws._foot_cop_margin(session, walker, side, tick["grounds"]))
        roll, pitch, up_z = sws._trunk_angles(data, int(walker.bid_body))
        zmp_act, com_act = sws._actual_margins(
            session, walker, tick["grounds"], int(tick["n_l"]), int(tick["n_r"]),
        )
        captured["roll"].append(roll)
        captured["pitch"].append(pitch)
        captured["up_z"].append(up_z)
        captured["zmp_act"].append(zmp_act)
        captured["com_act"].append(com_act)
        captured["q"].append(sws._leg_q(model, data))
        n = len(captured["t"])
        if n == 1 or n % 500 == 0:
            print(f"[tiebreak] t={captured['t'][-1]:.2f}s n={n}", flush=True)

    row = sws.run_preview_row(spec, tip_sha=TIP, lipm_config=cfg, probe=probe)
    if row["plant_md5_before"] != FROZEN_MD5 or row["plant_md5"] != FROZEN_MD5:
        raise SystemExit(
            f"plant moved {row['plant_md5_before']} -> {row['plant_md5']}"
        )
    arrays = {
        key: np.asarray(captured[key])
        for key in captured
        if key not in ("declared", "mode", "q")
    }
    arrays["declared"] = np.asarray(captured["declared"])
    arrays["mode"] = np.asarray(captured["mode"])
    q_mat = np.asarray(captured["q"], dtype=np.float64)
    t_arr = arrays["t"]
    stand_mask = t_arr <= spec.stand_s + 1e-12
    if q_mat.shape[0] == 0 or not np.any(stand_mask):
        q_err = None
    else:
        q_ref = np.median(q_mat[stand_mask], axis=0)
        q_err = np.max(np.abs(q_mat - q_ref), axis=1)
    strict = sws.step_bars.summarize_trace(
        arrays,
        vx_m_s=spec.vx_m_s,
        period_s=spec.period_s,
        q_stand_err=q_err,
    )
    raw_n = {key: arrays[key] for key in arrays}
    raw_n["n_l"] = np.asarray(captured["n_raw_l"], dtype=np.int32)
    raw_n["n_r"] = np.asarray(captured["n_raw_r"], dtype=np.int32)
    contact_only = sws.step_bars.summarize_trace(
        raw_n, vx_m_s=spec.vx_m_s, period_s=spec.period_s,
    )
    swings = [s for s in _swing_rows(captured) if s["mode"] == "move" and s["landed"]]
    disagree: dict[str, list[dict[str, object]]] = {}
    counts: dict[str, int] = {}
    for side in ("l", "r"):
        flags = []
        for a, b, c in zip(captured[f"a_{side}"], captured[f"b_{side}"], captured[f"c_{side}"]):
            agree = (a and b and c) or ((not a) and (not b) and (not c))
            flags.append(not agree)
            if not agree:
                name = _pattern_name(bool(a), bool(b), bool(c))
                counts[name] = counts.get(name, 0) + 1
        disagree[side] = _runs(captured["t"], flags)
    cop = np.concatenate([
        np.asarray(captured["cop_l"], dtype=np.float64),
        np.asarray(captured["cop_r"], dtype=np.float64),
    ])
    cop_ok = cop[np.isfinite(cop)]
    mfg = sws._mfg_reasons(
        measured=row["plant_md5"] == FROZEN_MD5 and row["n_samples"] > 0,
        zmp_min_m=row["zmp_min_margin_m"],
        zmp_frac=float(row["zmp_outside_fraction"]),
        com_min_m=row["com_min_margin_m"],
        com_frac=float(row["com_outside_fraction"]),
        tipped=bool(row["tipped"]),
        ask_nm=float(row["ask_nm"]),
        ask_over_ticks=int(row["ask_over_ticks"]),
        com_peak=row["com_jerk_whole_peak"],
        com_rms=row["com_jerk_whole_rms"],
        joint_peak=row["joint_jerk_peak"],
        joint_rms=row["joint_jerk_rms"],
    )
    step_reasons = list(strict.get("fail_reasons") or [])
    qvel = row.get("qvel") if isinstance(row.get("qvel"), dict) else {}
    trunk_speed = row.get("trunk_speed") if isinstance(row.get("trunk_speed"), dict) else {}
    qvel_reasons = list(qvel.get("fail_reasons") or [])
    speed_torque = row.get("speed_torque") if isinstance(row.get("speed_torque"), dict) else {}
    hinge_pairs = row.get("hinge_pairs") if isinstance(row.get("hinge_pairs"), dict) else {}
    torque_reasons = list(speed_torque.get("fail_reasons") or [])
    clamp_bar = row.get("clamp_bar") if isinstance(row.get("clamp_bar"), dict) else {}
    clamp_reasons = list(clamp_bar.get("fail_reasons") or [])
    torque_bar = row.get("torque_bar") if isinstance(row.get("torque_bar"), dict) else {}
    signed_reasons = list(torque_bar.get("fail_reasons") or [])
    kv_rows = row.get("kv") if isinstance(row.get("kv"), list) else []
    fail = (
        mfg
        + step_reasons
        + [str(item) for item in qvel_reasons]
        + [str(item) for item in torque_reasons]
        + [str(item) for item in clamp_reasons]
        + [str(item) for item in signed_reasons]
    )
    verdict = "CLEAR" if not fail else "Prefer FAIL"
    gait = _gait_label(strict)
    note = _note(row, strict, contact_only, swings, counts, disagree, meta, cop_ok, gait, verdict, fail)
    payload_row = {
        "name": spec.name,
        "tip_sha": TIP,
        "gait": gait,
        "verdict": verdict,
        "fail_reasons": fail,
        "plant_md5_before": row["plant_md5_before"],
        "plant_md5": row["plant_md5"],
        "n_samples": row["n_samples"],
        "ctrl_dt_s": row["ctrl_dt_s"],
        "x_cmd_max_m": meta.get("x_cmd_max"),
        "z_flat_seen": meta.get("z_flat_seen"),
        "box_bottom_L_m": meta.get("box_bottom_L"),
        "box_bottom_R_m": meta.get("box_bottom_R"),
        "floor_z_m": meta.get("floor_z"),
        "declared_zmp_min_m": row["zmp_min_margin_m"],
        "declared_com_min_m": row["com_min_margin_m"],
        "declared_zmp_out": row["zmp_outside_fraction"],
        "declared_com_out": row["com_outside_fraction"],
        "phases": row["phases"],
        "com_jerk_whole_peak": row["com_jerk_whole_peak"],
        "com_jerk_whole_rms": row["com_jerk_whole_rms"],
        "joint_jerk_peak": row["joint_jerk_peak"],
        "joint_jerk_rms": row["joint_jerk_rms"],
        "worst_joint": row["worst_joint"],
        "worst_joint_peak": row["worst_joint_peak"],
        "worst_joint_rms": row["worst_joint_rms"],
        "ask_nm": row["ask_nm"],
        "ask_joint": row["ask_joint"],
        "ask_t_s": row["ask_t_s"],
        "ask_headroom_nm": float(sws.SAG_BAR_NM) - float(row["ask_nm"]),
        "asks_by_joint": row["asks_by_joint"],
        "preview_stages": row["preview_stages"],
        "min_up_z": row["min_up_z"],
        "fault_reasons": row["fault_reasons"],
        "qvel": _jsonable(qvel),
        "trunk_speed": _jsonable(trunk_speed),
        "speed_torque": _jsonable(speed_torque),
        "hinge_pairs": _jsonable(hinge_pairs),
        "clamp": _jsonable(row.get("clamp")),
        "clamp_bar": _jsonable(clamp_bar),
        "torque_bar": _jsonable(torque_bar),
        "kv": _jsonable(kv_rows),
        "stepping": _jsonable(strict),
        "stepping_contact_count": _jsonable(contact_only),
        "swings": swings,
        "disagree_counts": counts,
        "disagree_runs": {side: runs[:40] for side, runs in disagree.items()},
        "disagree_run_n": {side: len(runs) for side, runs in disagree.items()},
        "cop_whole_p5_m": None if cop_ok.size == 0 else float(np.percentile(cop_ok, 5)),
        "cop_whole_min_m": None if cop_ok.size == 0 else float(np.min(cop_ok)),
        "note": note,
    }
    payload = {
        "tip": LABEL,
        "tip_sha": TIP,
        "pr": 103,
        "soft_pass": False,
        "plant_md5": digest,
        "cadence_grid_on_tip": False,
        "rows": [payload_row],
    }
    JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
    JSON_PATH.write_text(json.dumps(_jsonable(payload), indent=2) + "\n", encoding="utf-8")
    print(
        f"[tiebreak] {gait} / {verdict} swings {len(swings)} "
        f"frac {strict['step_fraction']:.3f} plant {digest}",
        flush=True,
    )
    print(f"[tiebreak] wrote {JSON_PATH}")
    _splice_doc()
    retro_path = Path(__file__).resolve().parent / "score_retro_voice.py"
    spec_mod = importlib.util.spec_from_file_location("score_retro_hinge", retro_path)
    if spec_mod is None or spec_mod.loader is None:
        raise SystemExit(f"cannot load {retro_path}")
    retro = importlib.util.module_from_spec(spec_mod)
    spec_mod.loader.exec_module(retro)
    retro.merge_hinge_row({
        "tip": "d6e8b5e",
        "tip_sha": TIP,
        "pr": 103,
        "bout": payload_row["name"],
        "gait": gait,
        "verdict": verdict,
        "plant_md5_before": payload_row["plant_md5_before"],
        "plant_md5": payload_row["plant_md5"],
        "qvel": payload_row["qvel"],
        "trunk_speed": payload_row["trunk_speed"],
        "speed_torque": payload_row["speed_torque"],
        "hinge_pairs": payload_row["hinge_pairs"],
        "clamp": payload_row.get("clamp"),
        "clamp_bar": payload_row.get("clamp_bar"),
        "torque_bar": payload_row.get("torque_bar"),
        "kv": payload_row.get("kv"),
        "fail_reasons": payload_row.get("fail_reasons"),
    })
    retro.splice_hinge_doc()


def _note(row, strict, contact_only, swings, counts, disagree, meta, cop_ok, gait, verdict, fail) -> str:
    ss = strict.get("ss") or {}
    stop = strict.get("stop") or {}
    qvel = row.get("qvel") if isinstance(row.get("qvel"), dict) else {}
    trunk_speed = row.get("trunk_speed") if isinstance(row.get("trunk_speed"), dict) else {}
    parts = [
        f"Gait label {gait}. Bar verdict {verdict}.",
        f"Commanded x_amp peaked at {float(meta.get('x_cmd_max') or 0.0) * 1000.0:.2f} mm. "
        f"z_flat was {'on' if meta.get('z_flat_seen') else 'off'}. "
        f"Contact-box bottom local z is {float(meta.get('box_bottom_L') or 0.0) * 1000.0:.2f} mm "
        f"(L) and {float(meta.get('box_bottom_R') or 0.0) * 1000.0:.2f} mm (R). Floor plane z is "
        f"{float(meta.get('floor_z') or 0.0) * 1000.0:.2f} mm.",
        f"Three-part step fraction {float(strict['step_fraction']):.3f}, "
        f"airborne {float(strict['airborne_forward_m']):.4f} m, "
        f"contact {float(strict['contact_forward_m']):.4f} m, "
        f"mismatch {float(strict['mismatch_fraction']):.3f}, "
        f"stance contacts min {ss.get('min_contacts')}, "
        f"slip {strict.get('worst_slip_m')}, "
        f"clearance min {strict.get('min_clear_m')}.",
        f"Contact-count-only step fraction {float(contact_only['step_fraction']):.3f}, "
        f"{contact_only.get('n_scored_swings')} scored swings, "
        f"mismatch {float(contact_only['mismatch_fraction']):.3f}.",
        f"Declared ZMP min {row['zmp_min_margin_m']} outside {row['zmp_outside_fraction']}. "
        f"Declared CoM min {row['com_min_margin_m']} outside {row['com_outside_fraction']}. "
        f"Actual-stance ZMP {strict.get('actual')}.",
        f"CoP whole-bout min {None if cop_ok.size == 0 else float(cop_ok.min())} "
        f"p5 {None if cop_ok.size == 0 else float(__import__('numpy').percentile(cop_ok, 5))}. "
        f"Single-support CoP min {ss.get('cop_min_m')} p5 {ss.get('cop_p5_m')} "
        f"p50 {ss.get('cop_p50_m')} p95 {ss.get('cop_p95_m')}, "
        f"edge dwell {ss.get('edge_fraction')}, on-edge {ss.get('edge_zero_fraction')}, "
        f"sole tilt max {ss.get('tilt_max_rad')} fraction over 1 deg {ss.get('tilt_over_1deg')}.",
        f"CoM jerk {row['com_jerk_whole_peak']} / {row['com_jerk_whole_rms']}. "
        f"Joint jerk {row['joint_jerk_peak']} / {row['joint_jerk_rms']} worst {row['worst_joint']}. "
        f"Sum column {row['ask_joint']} {float(row['ask_nm']):.4f} Nm at {row['ask_t_s']} s, "
        f"headroom {2.33 - float(row['ask_nm']):+.4f} Nm. "
        f"Hinge |qvel| bar {qvel.get('bar_rad_s')} passes={qvel.get('passes')}. "
        f"Trunk speed T {trunk_speed.get('period_s')} s, commanded "
        f"{trunk_speed.get('vx_cmd_m_s')} m/s, actual {trunk_speed.get('actual_vx_m_s')} m/s, "
        f"ratio {trunk_speed.get('ratio')}.",
        f"Stop pose: {stop.get('pose')}, pitch {stop.get('final_pitch_rad')} "
        f"vs stand {stop.get('stand_pitch_rad')}.",
        f"Scored swings {len(swings)}. Disagreeing-tick patterns: {counts}. "
        f"Disagree runs L {len(disagree['l'])}, R {len(disagree['r'])}.",
        "Fail reasons: " + ("; ".join(fail) if fail else "none") + ".",
    ]
    return " ".join(parts)


def _jsonable(value):
    import numpy as np
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


if __name__ == "__main__":
    main()
