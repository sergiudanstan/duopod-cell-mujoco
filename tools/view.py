"""Interactive MuJoCo viewer of the running cell (real time, loops the 12-carton run).

macOS: uv run mjpython tools/view.py [--procedural]      Linux/Windows: uv run python tools/view.py
Drag to orbit, right-drag to pan, scroll to zoom; double-click a body to select it; Ctrl+right-drag to push it.
"""
import argparse
import time

import mujoco.viewer

from duopod_mj import geometry as g
from duopod_mj import sim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--procedural", action="store_true")
    a = ap.parse_args()
    cell = sim.Cell(g.Params(), use_mesh=not a.procedural)
    with mujoco.viewer.launch_passive(cell.m, cell.d) as v:
        v.cam.lookat[:] = (0.0, -0.05, 0.45)
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 4.0, 140, -20
        while v.is_running():
            start_wall, start_sim = time.perf_counter(), cell.d.time

            def frame(c):
                if not v.is_running():
                    raise KeyboardInterrupt
                v.sync()
                lag = (c.d.time - start_sim) - (time.perf_counter() - start_wall)
                if lag > 0:
                    time.sleep(lag)

            try:
                cell.run(on_frame=frame, frame_dt=1 / 60)
            except KeyboardInterrupt:
                break
            cell.reset()


if __name__ == "__main__":
    main()
