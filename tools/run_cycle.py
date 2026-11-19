"""Headless run of the cell with a short report. Usage: uv run python tools/run_cycle.py [--perturb]"""
import time, math, numpy as np, sys
from duopod_mj import sim, geometry as g
p = g.Params(perturb="--perturb" in sys.argv)
t0 = time.time()
c = sim.run_cycle(p)
L = c.log
wall = time.time() - t0
print(f"sim {c.d.time:.2f}s wall {wall:.1f}s RTF {c.d.time/wall:.2f}")
print("placed", len(c.placed), [round(t,2) for t,_ in c.placed])
print("status", c.status)
ev = [e for e in L.events if e[1] not in ("escapement open","escapement closed")]
print("events", [(round(t,2),s) for t,s in ev[:24]])
tr = np.array(L.tcp_ref); ta = np.array(L.tcp)[:, [0,2]]
err = np.linalg.norm(tr - ta, axis=1)
print(f"track err max {err.max()*1000:.2f} mm rms {np.sqrt((err**2).mean())*1000:.2f}")
print(f"eq err max {max(L.eq_err)*1000:.3f} mm, tilt max {max(abs(x) for x in L.tilt)*180/math.pi:.3f} deg")
print(f"slip max {max(L.slip)*1000 if L.slip else 0:.2f} mm, cc contacts {len(L.cc_contact)}")
tau = np.array(L.tau); ana = np.array(L.tau_ana)
print("tau peak meas", np.abs(tau).max(0).round(1), "ana", np.abs(ana).max(0).round(1), "rms diff", np.sqrt(((tau-ana)**2).mean(0)).round(2))
for k in range(p.n_cartons):
    pos = c.carton_pos(k); print(k, c.status[k], np.round(pos,3))
