#!/usr/bin/env python3
"""Egocentric kit-cam FOV preview — AI-locked numbers for Mon 29 Sep review.

Locked:
  cam_z ≈ 385 mm, lever_z = 275 mm, HFOV = 120° (VFOV≈104.8° @ 640×480)
  neck_pitch restored (2DOF pan-tilt); default approach pitch −16°
  depression to center lever: ~12°@0.5m → ~15°@0.4m → ~20°@0.3m

Required stills:
  1) pitch=0 @ 0.4 m   (lever low in frame)
  2) −16° @ 0.4 m      (lever centered)
  3) −20° @ 0.3 m      (grasp)

Plus continuous ego clip with commanded look-down holding lever in-frame.

Does NOT unfreeze Path A. Does NOT re-run / regress walk_gait v9 acceptance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "mujoco" / "dronable_v0.xml"
OUT = ROOT / "previews" / "ego_cam"

HFOV_DEG = 120.0
VFOV_DEG = 104.82  # MJCF kit_cam fovy; 640×480
LEVER_Z = 0.275
CAM_Z_TARGET = 0.385
APPROACH_DEFAULT_DEG = -16.0


def _jid(model, name):
    return mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)


def _stand(model, data, pelvis_y: float, neck_pitch_rad: float = 0.0) -> None:
    mj.mj_resetDataKeyframe(model, data, 0)
    data.qpos[0] = 0.0
    data.qpos[1] = pelvis_y
    data.qvel[:] = 0.0
    j = _jid(model, "neck_pitch")
    # qpos address for hinge
    adr = model.jnt_qposadr[j]
    data.qpos[adr] = neck_pitch_rad
    # also set ctrl so position actuator holds
    aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "m_neck_pitch")
    if aid >= 0:
        data.ctrl[aid] = neck_pitch_rad
    mj.mj_forward(model, data)


def _cam_world(model, data) -> np.ndarray:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "kit_cam_site")
    return data.site_xpos[sid].copy()


def _lever_world(model, data) -> np.ndarray:
    gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "door_lever")
    return data.geom_xpos[gid].copy()


def _cam_forward(model, data) -> np.ndarray:
    """Optical axis (+look) from kit_cam site xmat: site frame +Y? site default.
    Camera looks along -Z of camera body; camera is on head_face with xyaxes
    so look = +Y of parent when pitch=0. Use camera xmat column.
    """
    cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    # camera xmat: columns are cam axes in world; look dir = -Z_cam = -col2
    R = data.cam_xmat[cid].reshape(3, 3)
    return -R[:, 2].copy()


def elev_vs_look(cam, lever, look) -> tuple[float, float, float]:
    """Return (dy_along_look_approx, elev_deg_from_look, dy_world_y)."""
    d = lever - cam
    dy = float(d[1])
    # elevation from level +Y (world): atan2(dz, dy)
    elev_level = math.degrees(math.atan2(float(d[2]), max(dy, 1e-9)))
    # angle between look and vector-to-lever (signed about camera right ≈ +X)
    v = d / (np.linalg.norm(d) + 1e-12)
    # pitch of look vs level +Y in YZ
    look_pitch = math.degrees(math.atan2(float(look[2]), max(float(look[1]), 1e-9)))
    elev_from_look = elev_level - look_pitch
    return dy, elev_level, elev_from_look


def pelvis_y_for_dy(model, data, target_dy: float, neck_pitch_rad: float = 0.0) -> float:
    _stand(model, data, 0.0, neck_pitch_rad)
    cam0 = _cam_world(model, data)
    lev = _lever_world(model, data)
    return float(lev[1] - target_dy - cam0[1])


def _burn(img, lines):
    from PIL import Image, ImageDraw, ImageFont

    out = Image.fromarray(img)
    draw = ImageDraw.Draw(out)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 13)
        font_sm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11)
    except Exception:
        font = ImageFont.load_default()
        font_sm = font
    y = 4
    for i, line in enumerate(lines):
        f = font if i == 0 else font_sm
        draw.text((5, y + 1), line, fill=(0, 0, 0), font=f)
        draw.text((4, y), line, fill=(255, 235, 90), font=f)
        y += 15 if i == 0 else 13
    return np.asarray(out, dtype=np.uint8)


def _save(img, path: Path):
    from PIL import Image
    Image.fromarray(img).save(path)


def _render_ego(model, data, renderer) -> np.ndarray:
    renderer.update_scene(data, camera="kit_cam")
    return renderer.render()


def _render_side(model, data, renderer, lookat, distance=1.35) -> np.ndarray:
    cam = mj.MjvCamera()
    mj.mjv_defaultCamera(cam)
    cam.type = mj.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = 0.0
    cam.elevation = -8.0
    renderer.update_scene(data, camera=cam)
    return renderer.render()


def _encode_mp4(frames: list[Path], out: Path, fps: int = 12) -> Path:
    cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-framerate", str(fps),
        "-i", str(frames[0].parent / "frame_%03d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-g", "1", "-bf", "0", "-crf", "18",
        str(out),
    ]
    try:
        subprocess.run(cmd, check=True)
        return out
    except (FileNotFoundError, subprocess.CalledProcessError):
        import imageio.v2 as imageio
        from PIL import Image
        imgs = [np.asarray(Image.open(p)) for p in frames]
        gif = out.with_suffix(".gif")
        imageio.mimsave(gif, imgs, fps=fps)
        return gif


def commanded_pitch_for_dy(dy: float, cam_z: float = CAM_Z_TARGET, lever_z: float = LEVER_Z) -> float:
    """Pitch (deg, negative=down) to center lever at given cam-lever dy."""
    return -math.degrees(math.atan2(cam_z - lever_z, max(dy, 1e-6)))


def run(out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    run_id = hashlib.sha256(f"ego_cam|{time.time():.3f}|120fov".encode()).hexdigest()[:12]
    model = mj.MjModel.from_xml_path(str(XML))
    data = mj.MjData(model)

    # Sanity: DOFs
    assert _jid(model, "neck_pitch") >= 0, "neck_pitch missing"
    assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam") >= 0

    shots = [
        # label, dy_m, pitch_deg, note
        ("01_pitch0_dy0p40", 0.40, 0.0, "pitch=0 @ 0.4 m — lever low in frame"),
        ("02_pitch-16_dy0p40", 0.40, -16.0, "−16° @ 0.4 m — lever centered (default approach)"),
        ("03_pitch-20_dy0p30", 0.30, -20.0, "−20° @ 0.3 m — grasp"),
    ]

    rows = []
    with mj.Renderer(model, height=480, width=640) as renderer:
        # Mount height check (level)
        _stand(model, data, 0.0, 0.0)
        cam0 = _cam_world(model, data)
        mount_z = float(cam0[2])

        for label, dy_t, pitch_deg, note in shots:
            pitch_rad = math.radians(pitch_deg)
            py = pelvis_y_for_dy(model, data, dy_t, pitch_rad)
            _stand(model, data, py, pitch_rad)
            cam = _cam_world(model, data)
            lev = _lever_world(model, data)
            look = _cam_forward(model, data)
            dy, elev_lvl, elev_from_look = elev_vs_look(cam, lev, look)
            look_pitch = math.degrees(math.atan2(float(look[2]), max(float(look[1]), 1e-9)))

            lines = [
                f"{note}",
                f"kit_cam HFOV={HFOV_DEG:.0f}° VFOV≈{VFOV_DEG:.1f}°  640×480  run={run_id}",
                f"cam_z={cam[2]:.3f}m  lever_z={lev[2]:.3f}m  dy={dy:.3f}m  neck_pitch={pitch_deg:+.1f}°",
                f"elev_level={elev_lvl:+.1f}°  look_pitch={look_pitch:+.1f}°  elev_from_look={elev_from_look:+.1f}°",
            ]
            img = _burn(_render_ego(model, data, renderer), lines)
            path = out_dir / f"{label}.png"
            _save(img, path)

            # matching side FOV view
            mid_y = 0.5 * (float(cam[1]) + float(lev[1]))
            side = _burn(
                _render_side(model, data, renderer, np.array([0.0, mid_y, 0.25]), 1.4),
                [
                    f"SIDE+FOV  {note}",
                    f"dy={dy:.3f}m pitch={pitch_deg:+.1f}° cam_z={cam[2]:.3f} run={run_id}",
                    "cyan = kit FOV frustum (pitches with neck)",
                ],
            )
            side_path = out_dir / f"side_{label}.png"
            _save(side, side_path)

            rows.append({
                "label": label,
                "note": note,
                "dy_m": round(dy, 4),
                "neck_pitch_deg": pitch_deg,
                "cam_z_m": round(float(cam[2]), 4),
                "elev_level_deg": round(elev_lvl, 2),
                "look_pitch_deg": round(look_pitch, 2),
                "elev_from_look_deg": round(elev_from_look, 2),
                "ego_png": path.name,
                "side_png": side_path.name,
            })
            print(f"  {label}: dy={dy:.3f} pitch={pitch_deg:+.1f} elev_from_look={elev_from_look:+.1f} → {path.name}")

        # Continuous approach clip: dy 0.70 → 0.28 with commanded look-down
        clip_dir = out_dir / "clip_frames"
        clip_dir.mkdir(exist_ok=True)
        n = 48
        dy0, dy1 = 0.70, 0.28
        frames = []
        for i in range(n):
            u = i / (n - 1)
            dy_t = dy0 + (dy1 - dy0) * u
            # schedule: ease toward default −16°, then deepen to ~−20° near grasp
            pitch_cmd = commanded_pitch_for_dy(dy_t)  # centers lever
            # blend: start from 0 at far, track center command (≈−12..−20)
            # hold lever in-frame: use full centering command (Controls must-prove)
            pitch_rad = math.radians(pitch_cmd)
            py = pelvis_y_for_dy(model, data, dy_t, pitch_rad)
            _stand(model, data, py, pitch_rad)
            cam = _cam_world(model, data)
            lev = _lever_world(model, data)
            look = _cam_forward(model, data)
            dy, elev_lvl, elev_from_look = elev_vs_look(cam, lev, look)
            lines = [
                f"ego approach COMMANDED look-down  {i+1}/{n}  run={run_id}",
                f"dy={dy:.3f}m  neck_pitch={pitch_cmd:+.1f}°  elev_from_look={elev_from_look:+.1f}°",
                f"HFOV={HFOV_DEG:.0f}° cam_z={cam[2]:.3f}m — lever held in-frame",
            ]
            img = _burn(_render_ego(model, data, renderer), lines)
            fp = clip_dir / f"frame_{i:03d}.png"
            _save(img, fp)
            frames.append(fp)
        clip_path = _encode_mp4(frames, out_dir / "ego_approach_lookdown.mp4", fps=12)
        print(f"  clip → {clip_path}")

        # Extra reference: depression table stills at 0.5 / 0.4 / 0.3 with centering pitch
        for dy_t in (0.50, 0.40, 0.30):
            pdeg = commanded_pitch_for_dy(dy_t)
            pitch_rad = math.radians(pdeg)
            py = pelvis_y_for_dy(model, data, dy_t, pitch_rad)
            _stand(model, data, py, pitch_rad)
            cam = _cam_world(model, data)
            lev = _lever_world(model, data)
            look = _cam_forward(model, data)
            dy, elev_lvl, elev_from_look = elev_vs_look(cam, lev, look)
            tag = f"ref_center_dy{dy_t:.2f}_pitch{pdeg:+.0f}".replace(".", "p").replace("+", "")
            # fix tag
            tag = f"ref_center_dy{str(dy_t).replace('.', 'p')}_p{abs(pdeg):.0f}down"
            lines = [
                f"REF center lever  dy={dy_t:.2f}m  pitch={pdeg:+.1f}°  (AI depression table)",
                f"cam_z={cam[2]:.3f} elev_from_look={elev_from_look:+.1f}° run={run_id}",
            ]
            _save(_burn(_render_ego(model, data, renderer), lines), out_dir / f"{tag}.png")

    summary = {
        "run_id": run_id,
        "hfov_deg": HFOV_DEG,
        "vfov_deg": VFOV_DEG,
        "mount_z_stand_m": round(mount_z, 4),
        "mount_z_target_m": CAM_Z_TARGET,
        "lever_z_m": LEVER_Z,
        "approach_default_pitch_deg": APPROACH_DEFAULT_DEG,
        "depression_table_deg": {
            "0.50m": round(commanded_pitch_for_dy(0.50), 1),
            "0.40m": round(commanded_pitch_for_dy(0.40), 1),
            "0.30m": round(commanded_pitch_for_dy(0.30), 1),
        },
        "required_stills": rows,
        "clip": clip_path.name,
        "out_dir": str(out_dir),
    }
    (out_dir / "metrics.json").write_text(json.dumps(summary, indent=2))
    (out_dir / "run_id.txt").write_text(run_id + "\n")
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    print(f"Loading {XML}")
    s = run(args.out)
    print("SUMMARY", {k: v for k, v in s.items() if k != "required_stills"})
    for r in s["required_stills"]:
        print(r)


if __name__ == "__main__":
    main()
