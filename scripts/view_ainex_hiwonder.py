#!/usr/bin/env python3
"""Load kit-matched AiNex MJCF (headless check or mujoco.viewer)."""
from pathlib import Path
import sys

XML = Path(__file__).resolve().parents[1] / "mujoco/ainex_hiwonder/ainex.xml"

def main():
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    nh = sum(1 for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE)
    print(f"loaded {XML}")
    print(f"  nq={m.nq} nv={m.nv} nu={m.nu} nhinge={nh} nmesh={m.nmesh} mass={m.body_mass.sum():.3f} kg")
    if "--view" in sys.argv:
        from mujoco import viewer
        viewer.launch(m, d)
    else:
        for _ in range(50):
            mujoco.mj_step(m, d)
        print("  smoke step OK")

if __name__ == "__main__":
    main()
