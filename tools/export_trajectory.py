"""Run the cell in MuJoCo and export the result for the photoreal web scene (duopod-cell-pro, "MuJoCo physics" mode).

Usage: uv run python tools/export_trajectory.py [out.json]
Default output: ~/code/duopod-cell-pro/physics/duopod_physics.json
Per frame (60 Hz): actual TCP (x, z), robot state, vacuum, blade / clamp travel, each carton's pose
(centre x, y, z + quaternion w, x, y, z, or 0 when off the belts), measured arm-side torques (theta convention).
"""
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import mujoco

from duopod_mj import geometry as g
from duopod_mj import sim

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "code/duopod-cell-pro/physics/duopod_physics.json"
FPS = 60


def main():
    p = g.Params()
    cell = sim.Cell(p, use_mesh=False)
    m = cell.m
    qb = m.jnt_qposadr[m.joint("blade").id]
    qc = m.jnt_qposadr[m.joint("clamp").id]
    frames = dict(t=[], tcp=[], state=[], k=[], vac=[], blade=[], clamp=[], tau=[], cartons=[[] for _ in range(p.n_cartons)])
    pick_t: dict[int, float] = {}
    r = lambda v, n=4: round(float(v), n)  # noqa: E731

    def frame(c):
        d = c.d
        tcp = c.tcp()
        frames["t"].append(r(d.time, 4))
        frames["tcp"].append([r(tcp[0], 5), r(tcp[2], 5)])
        frames["state"].append(c.state)
        k = c.held if c.held is not None else (c.next_pick if c.state in ("down", "grip", "backPick") else -1)
        frames["k"].append(-1 if k is None else int(k))
        frames["vac"].append(1 if c.vac else 0)
        frames["blade"].append(r(d.qpos[qb], 4))
        frames["clamp"].append(r(d.qpos[qc], 4))
        tq = d.sensordata[[c.sensor["tqL"], c.sensor["tqR"]]]
        frames["tau"].append([r(tq[0], 2), r(-tq[1], 2)])     # arm side, theta convention [left, right]
        if c.held is not None and c.held not in pick_t:
            pick_t[c.held] = r(d.time, 4)
        for i in range(p.n_cartons):
            if c.status[i] in ("parked", "gone"):
                frames["cartons"][i].append(0)
            else:
                a = c.carton_qadr[i]
                frames["cartons"][i].append([r(v, 4) for v in d.qpos[a:a + 7]])

    t0 = time.perf_counter()
    cell.run(on_frame=frame, frame_dt=1 / FPS)
    data = dict(
        meta=dict(engine=f"MuJoCo {mujoco.__version__}", generated=datetime.now(timezone.utc).isoformat(timespec="seconds"),
                  fps=FPS, timestep=m.opt.timestep, control_hz=1.0 / (m.opt.timestep * sim.CTRL_EVERY),
                  ratio=p.ratio, eta=p.eta, belt_a_w=g.BELT_A["w"], carton=g.CARTON, travel=p.travel, smart=p.smart,
                  wall_s=round(time.perf_counter() - t0, 2)),
        duration=r(cell.d.time, 4),
        picks={str(k): v for k, v in pick_t.items()},
        placed=[[r(t, 4), int(k)] for t, k in cell.placed],
        **frames,
    )
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, separators=(",", ":")))
    print(OUT, f"{OUT.stat().st_size / 1e6:.2f} MB", f"{len(frames['t'])} frames", f"{len(cell.placed)} placed in {cell.d.time:.2f} s")


if __name__ == "__main__":
    main()
