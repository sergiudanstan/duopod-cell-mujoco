"""Export the vendor DuoPod parts to STL in the robot frame at the CAD pose (local only, _cad/ is git-ignored).

Usage: uv run --with trimesh python tools/export_meshes.py [path/to/duopod.glb]
CAD -> world: x = y_cad, y = 0.065 - x_cad, z = ZM + z_cad (same mapping as duopod-cell-pro).
"""
import sys
from pathlib import Path

import numpy as np
import trimesh

from duopod_mj import geometry as g
from duopod_mj.model import CAD_PART, MESH_DIR

src = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / "code/duopod-cell-pro/_cad/duopod.glb")
scene = trimesh.load(src)
MESH_DIR.mkdir(parents=True, exist_ok=True)
C = np.array([[0, 1, 0, 0], [-1, 0, 0, 0.065], [0, 0, 1, g.ZM], [0, 0, 0, 1]], float)
by_node: dict[str, list] = {}
for name in scene.graph.nodes_geometry:
    T, geom = scene.graph[name]
    key = name.rsplit("_", 1)[0]
    if key not in CAD_PART:
        continue
    m = scene.geometry[geom].copy()
    m.apply_transform(C @ T)
    by_node.setdefault(key, []).append(m)
for key, ms in by_node.items():
    body = CAD_PART[key]
    mesh = trimesh.util.concatenate(ms)
    out = MESH_DIR / f"{body}__{key.replace(':', '_')}.stl"
    mesh.export(out)
    print(out, len(mesh.faces))
