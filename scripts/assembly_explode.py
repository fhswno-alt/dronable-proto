#!/usr/bin/env python3
"""Exploded assembly animation of the frozen AiNex walk plant.

Loads mujoco/ainex_hiwonder/ainex_controls_m2_145.xml, checks its md5, and
renders an offscreen build: every kinematic group starts pushed outward,
then seats one group at a time. Body offsets are applied only on the loaded
MjModel / MjData. The plant file, its meshes, and the controller are not
written.

The free joint on body_link is posed with MjData.qpos. Child bodies move
with MjModel.body_pos. Foot contact pads move with MjModel.geom_pos.
"""
from __future__ import annotations

import hashlib
import math
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

Vec3 = npt.NDArray[np.float64]

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
PRICE_SHEET = ROOT / "docs" / "MFG_FIRST_BUILD_PRICE_SHEET.md"
OUT_DIR = ROOT / "docs" / "previews" / "assembly"

EXPECTED_MD5 = "71b2c86d133ebc603f58b99c53e496f3"

WIDTH = 1280
HEIGHT = 720
FPS = 30
INTRO_S = 0.90
MOVE_S = 1.05
SETTLE_S = 0.40
HOLD_S = 2.00
SPIN_S = 4.50
SPIN_DEG = 360.0

LOOKAT = np.array([0.02, 0.0, 0.21], dtype=np.float64)
DISTANCE = 1.50
AZIMUTH0 = 128.0
ELEVATION = -17.0

# World-frame offsets away from the assembled centre. Plant +Y is the left
# side (l_* bodies). Magnitudes are chosen so groups separate without
# dropping the 145x86 pads through the floor.
EXPLODE_OFFSET_M: dict[str, Vec3] = {
    "torso": np.array([0.00, 0.00, 0.18], dtype=np.float64),
    "hips_l": np.array([0.00, 0.13, -0.11], dtype=np.float64),
    "hips_r": np.array([0.00, -0.13, -0.11], dtype=np.float64),
    "left_leg": np.array([0.02, 0.24, -0.03], dtype=np.float64),
    "right_leg": np.array([0.02, -0.24, -0.03], dtype=np.float64),
    "foot_l": np.array([0.18, 0.12, -0.02], dtype=np.float64),
    "foot_r": np.array([0.18, -0.12, -0.02], dtype=np.float64),
    "arm_l": np.array([0.00, 0.18, 0.10], dtype=np.float64),
    "arm_r": np.array([0.00, -0.18, 0.10], dtype=np.float64),
    "head": np.array([-0.02, 0.00, 0.32], dtype=np.float64),
}

GROUP_ORDER: tuple[str, ...] = (
    "torso",
    "hips",
    "left_leg",
    "right_leg",
    "feet",
    "arms",
    "head",
)

GROUP_TITLE: dict[str, str] = {
    "torso": "Torso",
    "hips": "Hips",
    "left_leg": "Left leg",
    "right_leg": "Right leg",
    "feet": "Feet",
    "arms": "Arms",
    "head": "Head",
}

FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
MAX_MP4_BYTES = 15 * 1024 * 1024
MAX_GIF_BYTES = 8 * 1024 * 1024


def _bootstrap_gl() -> str:
    """Pick egl, then osmesa, and re-exec so MuJoCo imports on that backend."""
    locked = os.environ.get("ASSEMBLY_GL_LOCKED", "")
    if locked in {"egl", "osmesa"}:
        os.environ["MUJOCO_GL"] = locked
        return locked
    probe = (
        "import mujoco as mj\n"
        "m = mj.MjModel.from_xml_string("
        "'<mujoco><worldbody><geom type=\"sphere\" size=\"0.1\"/></worldbody></mujoco>')\n"
        "d = mj.MjData(m)\n"
        "r = mj.Renderer(m, height=64, width=64)\n"
        "r.update_scene(d)\n"
        "r.render()\n"
        "r.close()\n"
    )
    for backend in ("egl", "osmesa"):
        env = os.environ.copy()
        env["MUJOCO_GL"] = backend
        env["ASSEMBLY_GL_LOCKED"] = backend
        result = subprocess.run(
            [sys.executable, "-c", probe],
            cwd="/tmp",
            env=env,
            capture_output=True,
            check=False,
        )
        if result.returncode == 0:
            os.environ["MUJOCO_GL"] = backend
            os.environ["ASSEMBLY_GL_LOCKED"] = backend
            os.execv(sys.executable, [sys.executable, *sys.argv])
        print(
            f"MUJOCO_GL={backend} unavailable ({result.returncode})",
            file=sys.stderr,
        )
    raise SystemExit(
        "FAIL: offscreen rendering unavailable. Tried MUJOCO_GL=egl, then osmesa."
    )


if __name__ == "__main__":
    _bootstrap_gl()

import mujoco as mj  # noqa: E402


@dataclass(frozen=True)
class AssemblyGroup:
    key: str
    title: str
    body_ids: tuple[int, ...]
    geom_ids: tuple[int, ...]
    servo_count: int
    subtitle: str


@dataclass(frozen=True)
class FrameCue:
    phase: str
    group_key: str | None
    ease: float
    spin_deg: float


def plant_md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def assert_plant_md5(path: Path, when: str) -> str:
    if not path.is_file():
        raise SystemExit(f"FAIL: plant file missing at {when}: {path}")
    digest = plant_md5(path)
    if digest != EXPECTED_MD5:
        raise SystemExit(
            f"FAIL: plant md5 at {when} is {digest}, expected {EXPECTED_MD5}. "
            f"Refusing to continue. File: {path}"
        )
    return digest


def smootherstep(t: float) -> float:
    clamped = min(1.0, max(0.0, t))
    return clamped * clamped * clamped * (clamped * (clamped * 6.0 - 15.0) + 10.0)


def _body_name(model: mj.MjModel, body_id: int) -> str:
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id)
    if not name:
        raise SystemExit(f"FAIL: body {body_id} has no name")
    return name


def _geom_name(model: mj.MjModel, geom_id: int) -> str:
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id)
    if not name:
        raise SystemExit(f"FAIL: geom {geom_id} has no name")
    return name


def classify_body(name: str) -> str:
    """Map a plant body name onto one assembly group. Unknown names fail."""
    if name == "body_link":
        return "torso"
    if name.startswith("head_"):
        return "head"
    if "_hip_yaw_" in name or "_hip_roll_" in name:
        return "hips"
    if name.startswith("l_") and (
        "_hip_pitch_" in name or "_knee_" in name or "_ank_" in name
    ):
        return "left_leg"
    if name.startswith("r_") and (
        "_hip_pitch_" in name or "_knee_" in name or "_ank_" in name
    ):
        return "right_leg"
    if "_sho_" in name or "_el_" in name or "_gripper_" in name:
        return "arms"
    raise SystemExit(f"FAIL: body {name} is not in the kinematic assembly grouping")


def _servo_counts(model: mj.MjModel, bodies_by_group: dict[str, list[int]]) -> dict[str, int]:
    counts = {key: 0 for key in GROUP_ORDER}
    seen_joints: set[int] = set()
    for actuator_id in range(model.nu):
        trn_type = int(model.actuator_trntype[actuator_id])
        if trn_type != int(mj.mjtTrn.mjTRN_JOINT):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            raise SystemExit(f"FAIL: actuator {name} is not a joint servo")
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id in seen_joints:
            raise SystemExit(f"FAIL: joint {joint_id} has more than one actuator")
        seen_joints.add(joint_id)
        if int(model.jnt_type[joint_id]) != int(mj.mjtJoint.mjJNT_HINGE):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            raise SystemExit(f"FAIL: actuator {name} does not drive a hinge")
        body_id = int(model.jnt_bodyid[joint_id])
        body_name = _body_name(model, body_id)
        group_key = classify_body(body_name)
        counts[group_key] += 1
    if sum(counts.values()) != model.nu:
        raise SystemExit(
            f"FAIL: servo counts {counts} do not sum to nu={model.nu}"
        )
    return counts


def _pad_subtitle(model: mj.MjModel) -> str:
    measured: list[tuple[int, int]] = []
    for name in ("l_foot_contact", "r_foot_contact"):
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise SystemExit(f"FAIL: missing foot pad geom {name}")
        size = model.geom_size[geom_id]
        length_mm = int(round(float(size[0]) * 2000.0))
        width_mm = int(round(float(size[1]) * 2000.0))
        measured.append((length_mm, width_mm))
    if measured[0] != measured[1]:
        raise SystemExit(f"FAIL: left/right foot pads differ: {measured}")
    length_mm, width_mm = measured[0]
    return f"{length_mm}×{width_mm} mm pads"


def build_groups(model: mj.MjModel) -> dict[str, AssemblyGroup]:
    bodies_by_group: dict[str, list[int]] = {key: [] for key in GROUP_ORDER}
    for body_id in range(1, model.nbody):
        name = _body_name(model, body_id)
        bodies_by_group[classify_body(name)].append(body_id)
    for key, body_ids in bodies_by_group.items():
        if key == "feet":
            continue
        if not body_ids:
            raise SystemExit(f"FAIL: assembly group {key} has no bodies")

    foot_ids: list[int] = []
    for name in ("l_foot_contact", "r_foot_contact"):
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise SystemExit(f"FAIL: missing geom {name}")
        foot_ids.append(geom_id)
    cam_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    if cam_id < 0:
        raise SystemExit("FAIL: camera kit_cam is missing from the plant")
    cam_body = _body_name(model, int(model.cam_bodyid[cam_id]))
    if classify_body(cam_body) != "head":
        raise SystemExit(f"FAIL: kit_cam sits on {cam_body}, not the head group")

    counts = _servo_counts(model, bodies_by_group)
    pad_line = _pad_subtitle(model)
    subtitles = {key: "" for key in GROUP_ORDER}
    subtitles["feet"] = pad_line
    subtitles["head"] = "kit_cam"
    groups: dict[str, AssemblyGroup] = {}
    for key in GROUP_ORDER:
        groups[key] = AssemblyGroup(
            key=key,
            title=GROUP_TITLE[key],
            body_ids=tuple(bodies_by_group[key]),
            geom_ids=tuple(foot_ids) if key == "feet" else (),
            servo_count=counts[key],
            subtitle=subtitles[key],
        )
    return groups


def _offset_key(group_key: str, name: str) -> str:
    if group_key == "hips":
        if name.startswith("l_"):
            return "hips_l"
        if name.startswith("r_"):
            return "hips_r"
        raise SystemExit(f"FAIL: hip body {name} is not left or right")
    if group_key == "arms":
        if name.startswith("l_"):
            return "arm_l"
        if name.startswith("r_"):
            return "arm_r"
        raise SystemExit(f"FAIL: arm body {name} is not left or right")
    return group_key


def build_offsets(
    model: mj.MjModel,
    groups: dict[str, AssemblyGroup],
) -> tuple[dict[int, Vec3], dict[int, Vec3]]:
    body_delta: dict[int, Vec3] = {
        body_id: np.zeros(3, dtype=np.float64) for body_id in range(model.nbody)
    }
    geom_delta: dict[int, Vec3] = {
        geom_id: np.zeros(3, dtype=np.float64) for geom_id in range(model.ngeom)
    }
    for key in GROUP_ORDER:
        group = groups[key]
        for body_id in group.body_ids:
            name = _body_name(model, body_id)
            spec = _offset_key(key, name)
            body_delta[body_id] = EXPLODE_OFFSET_M[spec].copy()
        for geom_id in group.geom_ids:
            name = _geom_name(model, geom_id)
            spec = "foot_l" if name.startswith("l_") else "foot_r"
            geom_delta[geom_id] = EXPLODE_OFFSET_M[spec].copy()
    for body_id in range(1, model.nbody):
        if float(np.linalg.norm(body_delta[body_id])) < 1e-8:
            name = _body_name(model, body_id)
            raise SystemExit(f"FAIL: {name} has no explode offset")
    return body_delta, geom_delta


def apply_offsets(
    model: mj.MjModel,
    data: mj.MjData,
    *,
    home_qpos: Vec3,
    home_body_pos: npt.NDArray[np.float64],
    home_geom_pos: npt.NDArray[np.float64],
    home_xmat: npt.NDArray[np.float64],
    root_id: int,
    body_delta: dict[int, Vec3],
    geom_delta: dict[int, Vec3],
    progress: dict[str, float],
    groups: dict[str, AssemblyGroup],
) -> None:
    """Pose the loaded model. progress 1 = exploded, 0 = assembled."""
    scaled_body: dict[int, Vec3] = {
        body_id: np.zeros(3, dtype=np.float64) for body_id in range(model.nbody)
    }
    scaled_geom: dict[int, Vec3] = {
        geom_id: np.zeros(3, dtype=np.float64) for geom_id in range(model.ngeom)
    }
    for key, group in groups.items():
        amount = progress[key]
        for body_id in group.body_ids:
            scaled_body[body_id] = body_delta[body_id] * amount
        for geom_id in group.geom_ids:
            scaled_geom[geom_id] = geom_delta[geom_id] * amount

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[:7] = home_qpos
    data.qpos[:3] = home_qpos[:3] + scaled_body[root_id]
    for body_id in range(model.nbody):
        if body_id == root_id:
            model.body_pos[body_id] = home_body_pos[body_id]
            continue
        parent_id = int(model.body_parentid[body_id])
        parent_rot = home_xmat[parent_id].reshape(3, 3)
        local = parent_rot.T @ (scaled_body[body_id] - scaled_body[parent_id])
        model.body_pos[body_id] = home_body_pos[body_id] + local
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        body_rot = home_xmat[body_id].reshape(3, 3)
        local = body_rot.T @ (scaled_geom[geom_id] - scaled_body[body_id])
        model.geom_pos[geom_id] = home_geom_pos[geom_id] + local
    mj.mj_forward(model, data)


def assert_offsets_match_world(
    model: mj.MjModel,
    data: mj.MjData,
    *,
    home_xpos: npt.NDArray[np.float64],
    home_geom_xpos: npt.NDArray[np.float64],
    body_delta: dict[int, Vec3],
    geom_delta: dict[int, Vec3],
    apply_kwargs: dict[str, object],
) -> None:
    """Fail if an in-memory offset does not land on the expected world pose."""
    groups = apply_kwargs["groups"]
    if not isinstance(groups, dict):
        raise SystemExit("FAIL: internal groups missing from offset check")
    home_progress = {key: 0.0 for key in GROUP_ORDER}
    apply_offsets(model, data, progress=home_progress, body_delta=body_delta, geom_delta=geom_delta, **apply_kwargs)  # type: ignore[arg-type]
    if not np.allclose(data.xpos, home_xpos, atol=1e-5):
        raise SystemExit("FAIL: assembled pose drifted from the loaded plant")
    full_progress = {key: 1.0 for key in GROUP_ORDER}
    apply_offsets(model, data, progress=full_progress, body_delta=body_delta, geom_delta=geom_delta, **apply_kwargs)  # type: ignore[arg-type]
    for body_id in range(1, model.nbody):
        expected = home_xpos[body_id] + body_delta[body_id]
        if not np.allclose(data.xpos[body_id], expected, atol=1e-4):
            name = _body_name(model, body_id)
            raise SystemExit(
                f"FAIL: {name} exploded pose {data.xpos[body_id]} != {expected}"
            )
    for geom_id, delta in geom_delta.items():
        if float(np.linalg.norm(delta)) < 1e-8:
            continue
        expected = home_geom_xpos[geom_id] + delta
        if not np.allclose(data.geom_xpos[geom_id], expected, atol=1e-4):
            name = _geom_name(model, geom_id)
            raise SystemExit(
                f"FAIL: {name} exploded pose {data.geom_xpos[geom_id]} != {expected}"
            )
    apply_offsets(model, data, progress=home_progress, body_delta=body_delta, geom_delta=geom_delta, **apply_kwargs)  # type: ignore[arg-type]


def price_caption(path: Path) -> tuple[str, ...]:
    """Kit and foot cost lines. Cells are copied from the price sheet."""
    if not path.is_file():
        raise SystemExit(f"FAIL: price sheet missing: {path}")
    raw = path.read_text(encoding="utf-8")
    found: dict[str, str] = {}
    for line in raw.splitlines():
        stripped = line.strip()
        if not stripped.startswith("|"):
            continue
        cells = [cell.strip().replace("**", "") for cell in stripped.strip("|").split("|")]
        if not cells or cells[0] not in {"K4", "F1", "F2"}:
            continue
        if cells[0] in found:
            continue
        if len(cells) < 5:
            raise SystemExit(f"FAIL: price row {cells[0]} is short: {stripped}")
        ident, item, option, price, currency = cells[:5]
        for token in (ident, item, option, price, currency):
            if token not in raw.replace("**", ""):
                raise SystemExit(f"FAIL: price token {token!r} is not in the sheet")
        if ident == "K4":
            found[ident] = f"{ident} | {option} | {price} {currency}"
        else:
            found[ident] = f"{ident} | {item} | {price} {currency}"
    missing = [key for key in ("K4", "F1", "F2") if key not in found]
    if missing:
        raise SystemExit(f"FAIL: price sheet has no rows {missing}")
    return (found["K4"], found["F1"], found["F2"])


def label_for(group: AssemblyGroup | None, phase: str) -> tuple[str, str]:
    if phase == "exploded":
        return "Exploded view", ""
    if phase in {"hold", "spin"} or group is None:
        return "Assembled", ""
    if group.servo_count > 0:
        word = "servo" if group.servo_count == 1 else "servos"
        title = f"{group.title}: {group.servo_count} {word}"
    else:
        title = group.title
    return title, group.subtitle


def _font(path: str, size: int) -> ImageFont.ImageFont:
    from PIL import ImageFont

    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def project_point(point: Vec3, azimuth_deg: float, fovy_deg: float) -> tuple[float, float] | None:
    """Pixel of a world point for the steady free camera. None if off-screen."""
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(ELEVATION)
    horizontal = DISTANCE * math.cos(elevation)
    camera_pos = LOOKAT + np.array(
        [
            -horizontal * math.cos(azimuth),
            -horizontal * math.sin(azimuth),
            -DISTANCE * math.sin(elevation),
        ],
        dtype=np.float64,
    )
    forward = LOOKAT - camera_pos
    forward = forward / float(np.linalg.norm(forward))
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(forward, world_up)
    right_norm = float(np.linalg.norm(right))
    if right_norm < 1e-8:
        return None
    right = right / right_norm
    up = np.cross(right, forward)
    relative = point - camera_pos
    depth = float(np.dot(relative, forward))
    if depth <= 0.05:
        return None
    focal = (HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    pixel_x = (WIDTH / 2.0) + focal * float(np.dot(relative, right)) / depth
    pixel_y = (HEIGHT / 2.0) - focal * float(np.dot(relative, up)) / depth
    if pixel_x < 12 or pixel_y < 12 or pixel_x > WIDTH - 12 or pixel_y > HEIGHT - 12:
        return None
    return pixel_x, pixel_y


def _draw_cam_marker(
    draw: ImageDraw.ImageDraw,
    pixel: tuple[float, float],
    font: ImageFont.ImageFont,
) -> None:
    """Ring at the plant's kit_cam site. The head mesh does not contain a camera body."""
    center_x, center_y = pixel
    radius = 9.0
    draw.ellipse(
        [center_x - radius, center_y - radius, center_x + radius, center_y + radius],
        outline=(255, 248, 240, 255),
        width=3,
    )
    draw.ellipse(
        [center_x - 3.0, center_y - 3.0, center_x + 3.0, center_y + 3.0],
        fill=(214, 64, 48, 255),
    )
    label = "kit_cam"
    box = draw.textbbox((0, 0), label, font=font)
    text_w = box[2] - box[0]
    text_x = center_x + radius + 8.0
    text_y = center_y - 12.0
    if text_x + text_w > WIDTH - 20:
        text_x = center_x - radius - 10.0 - text_w
    draw.text((text_x + 1, text_y + 1), label, font=font, fill=(0, 0, 0, 190))
    draw.text((text_x, text_y), label, font=font, fill=(255, 255, 255, 255))


def overlay_frame(
    frame: npt.NDArray[np.uint8],
    title: str,
    subtitle: str,
    caption: tuple[str, ...],
    cam_pixel: tuple[float, float] | None,
) -> npt.NDArray[np.uint8]:
    from PIL import Image, ImageDraw

    base = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    title_font = _font(FONT_BOLD, 40)
    sub_font = _font(FONT_REGULAR, 26)
    cap_font = _font(FONT_REGULAR, 16)
    pad_x = 18
    pad_y = 12
    title_box = draw.textbbox((0, 0), title, font=title_font)
    title_w = title_box[2] - title_box[0]
    title_h = title_box[3] - title_box[1]
    sub_w = 0
    sub_h = 0
    if subtitle:
        sub_box = draw.textbbox((0, 0), subtitle, font=sub_font)
        sub_w = sub_box[2] - sub_box[0]
        sub_h = sub_box[3] - sub_box[1]
    block_w = max(title_w, sub_w) + pad_x * 2
    block_h = title_h + (sub_h + 6 if subtitle else 0) + pad_y * 2
    origin_x = 32
    origin_y = 28
    draw.rounded_rectangle(
        [origin_x, origin_y, origin_x + block_w, origin_y + block_h],
        radius=10,
        fill=(22, 24, 28, 186),
    )
    draw.text((origin_x + pad_x, origin_y + pad_y - 4), title, font=title_font, fill=(255, 255, 255, 255))
    if subtitle:
        draw.text(
            (origin_x + pad_x, origin_y + pad_y + title_h + 2),
            subtitle,
            font=sub_font,
            fill=(232, 236, 240, 255),
        )
    cap_widths: list[int] = []
    cap_heights: list[int] = []
    for line in caption:
        box = draw.textbbox((0, 0), line, font=cap_font)
        cap_widths.append(box[2] - box[0])
        cap_heights.append(box[3] - box[1])
    cap_w = max(cap_widths) + pad_x * 2
    line_gap = 4
    cap_h = sum(cap_heights) + line_gap * (len(caption) - 1) + pad_y * 2
    cap_x = 32
    cap_y = HEIGHT - cap_h - 28
    draw.rounded_rectangle(
        [cap_x, cap_y, cap_x + cap_w, cap_y + cap_h],
        radius=8,
        fill=(22, 24, 28, 170),
    )
    cursor_y = cap_y + pad_y - 2
    for line, line_h in zip(caption, cap_heights):
        draw.text((cap_x + pad_x, cursor_y), line, font=cap_font, fill=(236, 238, 241, 255))
        cursor_y += line_h + line_gap
    if cam_pixel is not None:
        _draw_cam_marker(draw, cam_pixel, cap_font)
    composed = Image.alpha_composite(base, layer).convert("RGB")
    out = np.asarray(composed, dtype=np.uint8)
    return out


def neutral_studio(model: mj.MjModel) -> None:
    """Flat gray sky and floor. In memory only; the XML is not saved."""
    if model.ntex < 1:
        raise SystemExit("FAIL: plant has no texture to recolor for the backdrop")
    sky_end = int(model.tex_adr[1]) if model.ntex > 1 else int(model.tex_data.shape[0])
    model.tex_data[0:sky_end] = 228
    floor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    if floor_id < 0:
        raise SystemExit("FAIL: plant has no floor geom")
    model.geom_matid[floor_id] = -1
    model.geom_rgba[floor_id] = np.array([0.86, 0.86, 0.87, 1.0], dtype=np.float32)
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = [0.50, 0.50, 0.51]
    model.vis.headlight.diffuse[:] = [0.58, 0.58, 0.57]
    model.vis.global_.ipd = 0.0
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT


def frame_cues() -> list[FrameCue]:
    cues: list[FrameCue] = []
    intro_n = int(round(INTRO_S * FPS))
    cues.extend(FrameCue("exploded", None, 1.0, 0.0) for _ in range(intro_n))
    for key in GROUP_ORDER:
        move_n = int(round(MOVE_S * FPS))
        for index in range(move_n):
            ease = smootherstep((index + 1) / move_n)
            cues.append(FrameCue("in", key, ease, 0.0))
        settle_n = int(round(SETTLE_S * FPS))
        cues.extend(FrameCue("in", key, 1.0, 0.0) for _ in range(settle_n))
    hold_n = int(round(HOLD_S * FPS))
    cues.extend(FrameCue("hold", None, 1.0, 0.0) for _ in range(hold_n))
    spin_n = int(round(SPIN_S * FPS))
    for index in range(spin_n):
        spin = SPIN_DEG * ((index + 1) / spin_n)
        cues.append(FrameCue("spin", None, 1.0, spin))
    return cues


def progress_for(cue: FrameCue) -> dict[str, float]:
    """1 = still exploded, 0 = seated."""
    if cue.phase == "exploded":
        return {key: 1.0 for key in GROUP_ORDER}
    if cue.phase in {"hold", "spin"}:
        return {key: 0.0 for key in GROUP_ORDER}
    progress = {key: 1.0 for key in GROUP_ORDER}
    for key in GROUP_ORDER:
        if key == cue.group_key:
            progress[key] = 1.0 - cue.ease
            break
        progress[key] = 0.0
    return progress


def render_animation(
    model: mj.MjModel,
    data: mj.MjData,
    groups: dict[str, AssemblyGroup],
    apply_kwargs: dict[str, object],
    body_delta: dict[int, Vec3],
    geom_delta: dict[int, Vec3],
    caption: tuple[str, ...],
    out_dir: Path,
) -> tuple[Path, Path, Path, Path]:
    import imageio.v2 as imageio

    out_dir.mkdir(parents=True, exist_ok=True)
    cues = frame_cues()
    mp4_path = out_dir / "assembly.mp4"
    gif_path = out_dir / "assembly.gif"
    exploded_path = out_dir / "exploded.png"
    assembled_path = out_dir / "assembled.png"
    renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
    camera = mj.MjvCamera()
    mj.mjv_defaultCamera(camera)
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = LOOKAT
    camera.distance = DISTANCE
    camera.elevation = ELEVATION
    camera.azimuth = AZIMUTH0
    writer = imageio.get_writer(
        mp4_path,
        fps=FPS,
        codec="libx264",
        quality=8,
        macro_block_size=1,
        ffmpeg_params=["-movflags", "+faststart"],
    )
    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "kit_cam_site")
    if site_id < 0:
        raise SystemExit("FAIL: site kit_cam_site is missing")
    fovy_deg = float(model.vis.global_.fovy)
    wrote_exploded = False
    wrote_assembled = False
    try:
        for index, cue in enumerate(cues):
            apply_offsets(
                model,
                data,
                progress=progress_for(cue),
                body_delta=body_delta,
                geom_delta=geom_delta,
                **apply_kwargs,  # type: ignore[arg-type]
            )
            camera.azimuth = AZIMUTH0 + cue.spin_deg
            renderer.update_scene(data, camera=camera)
            rgb = np.asarray(renderer.render(), dtype=np.uint8)
            group = groups[cue.group_key] if cue.group_key is not None else None
            title, subtitle = label_for(group, cue.phase)
            cam_pixel = project_point(
                np.array(data.site_xpos[site_id], dtype=np.float64),
                camera.azimuth,
                fovy_deg,
            )
            painted = overlay_frame(rgb, title, subtitle, caption, cam_pixel)
            writer.append_data(painted)
            if cue.phase == "exploded" and not wrote_exploded:
                _save_png(painted, exploded_path)
                wrote_exploded = True
            if cue.phase == "hold" and not wrote_assembled:
                _save_png(painted, assembled_path)
                wrote_assembled = True
            if index % FPS == 0:
                print(f"  frame {index + 1}/{len(cues)}", flush=True)
    finally:
        writer.close()
        renderer.close()
    if not exploded_path.is_file() or not assembled_path.is_file():
        raise SystemExit("FAIL: stills were not written")
    _shrink_mp4_if_needed(mp4_path)
    _write_gif(mp4_path, gif_path)
    return mp4_path, gif_path, exploded_path, assembled_path


def _save_png(frame: npt.NDArray[np.uint8], path: Path) -> None:
    from PIL import Image

    Image.fromarray(frame).save(path)


def _shrink_mp4_if_needed(path: Path) -> None:
    import imageio_ffmpeg

    size = path.stat().st_size
    if size <= MAX_MP4_BYTES:
        return
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    temporary = path.with_suffix(".tmp.mp4")
    subprocess.run(
        [
            ffmpeg, "-y", "-i", str(path),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28",
            "-movflags", "+faststart",
            str(temporary),
        ],
        check=True,
        capture_output=True,
    )
    temporary.replace(path)
    if path.stat().st_size > MAX_MP4_BYTES:
        raise SystemExit(
            f"FAIL: {path} is {path.stat().st_size} bytes, over {MAX_MP4_BYTES}"
        )


def _write_gif(mp4_path: Path, gif_path: Path) -> None:
    """Short sped-up gif of the whole assembly. Shrink until it fits."""
    import imageio_ffmpeg

    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    attempts: tuple[tuple[float, int, int], ...] = (
        (0.50, 10, 640),
        (0.45, 8, 560),
        (0.40, 8, 480),
    )
    for speed, fps, scale in attempts:
        filtergraph = (
            f"setpts={speed}*PTS,fps={fps},scale={scale}:-1:flags=lanczos,"
            "split[s0][s1];"
            "[s0]palettegen=stats_mode=diff:max_colors=96[p];"
            "[s1][p]paletteuse=dither=bayer:bayer_scale=3"
        )
        subprocess.run(
            [
                ffmpeg, "-y", "-i", str(mp4_path),
                "-filter_complex", filtergraph,
                "-loop", "0",
                str(gif_path),
            ],
            check=True,
            capture_output=True,
        )
        if gif_path.stat().st_size <= MAX_GIF_BYTES:
            return
    raise SystemExit(
        f"FAIL: {gif_path} is {gif_path.stat().st_size} bytes, over {MAX_GIF_BYTES}"
    )


def _describe(model: mj.MjModel, groups: dict[str, AssemblyGroup]) -> None:
    print(f"MUJOCO_GL={os.environ.get('MUJOCO_GL', '')}")
    for key in GROUP_ORDER:
        group = groups[key]
        names = [_body_name(model, body_id) for body_id in group.body_ids]
        geoms = [_geom_name(model, geom_id) for geom_id in group.geom_ids]
        print(
            f"  {key}: servos={group.servo_count} bodies={names} geoms={geoms}",
            flush=True,
        )


def main() -> None:
    digest_before = assert_plant_md5(PLANT_XML, "start")
    print(f"plant md5 {digest_before} OK", flush=True)
    caption = price_caption(PRICE_SHEET)
    model = mj.MjModel.from_xml_path(str(PLANT_XML))
    data = mj.MjData(model)
    mj.mj_resetData(model, data)
    mj.mj_forward(model, data)
    groups = build_groups(model)
    _describe(model, groups)
    body_delta, geom_delta = build_offsets(model, groups)
    root_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if root_id < 0:
        raise SystemExit("FAIL: body_link missing")
    if int(model.jnt_type[0]) != int(mj.mjtJoint.mjJNT_FREE):
        raise SystemExit("FAIL: expected a free joint on the root body")
    apply_kwargs: dict[str, object] = {
        "home_qpos": np.array(data.qpos[:7], dtype=np.float64).copy(),
        "home_body_pos": np.array(model.body_pos, dtype=np.float64).copy(),
        "home_geom_pos": np.array(model.geom_pos, dtype=np.float64).copy(),
        "home_xmat": np.array(data.xmat, dtype=np.float64).copy(),
        "root_id": root_id,
        "groups": groups,
    }
    home_xpos = np.array(data.xpos, dtype=np.float64).copy()
    home_geom_xpos = np.array(data.geom_xpos, dtype=np.float64).copy()
    assert_offsets_match_world(
        model,
        data,
        home_xpos=home_xpos,
        home_geom_xpos=home_geom_xpos,
        body_delta=body_delta,
        geom_delta=geom_delta,
        apply_kwargs=apply_kwargs,
    )
    neutral_studio(model)
    paths = render_animation(
        model,
        data,
        groups,
        apply_kwargs,
        body_delta,
        geom_delta,
        caption,
        OUT_DIR,
    )
    digest_after = assert_plant_md5(PLANT_XML, "end")
    print(f"plant md5 after render {digest_after} OK", flush=True)
    for path in paths:
        print(f"wrote {path} ({path.stat().st_size} bytes)", flush=True)


if __name__ == "__main__":
    main()
