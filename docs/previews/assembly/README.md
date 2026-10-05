Regenerate with `python3 scripts/assembly_explode.py` (tries MUJOCO_GL=egl, then osmesa).
Writes assembly.mp4 (30 fps, 1280x720), assembly.gif, exploded.png, and assembled.png in this folder.
The script asserts mujoco/ainex_hiwonder/ainex_controls_m2_145.xml md5 71b2c86d133ebc603f58b99c53e496f3 and exits if the file differs.
Offsets are applied only in memory (body_pos, geom_pos, and the free-joint qpos for body_link). The plant XML and meshes are not written.
Plant md5 asserted for this render: 71b2c86d133ebc603f58b99c53e496f3.
