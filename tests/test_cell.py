"""Physics checks of the DuoPod cell in MuJoCo. Run: uv run pytest -q   (and --perturb: must fail on the collision)."""
import math

import mujoco
import numpy as np
import pytest

from duopod_mj import geometry as g
from duopod_mj import sim

FLANGE_TOP = g.MOUNT_Z - g.DATASHEET["below"]


def test_mjcf_matches_analytic_kinematics():
    """Joint values from the analytic IK close every loop of the MJCF and put the TCP site where fk2 says (600 x 400 grid)."""
    cell = sim.Cell(g.Params(), use_mesh=False)
    worst_tcp, worst_loop = 0.0, 0.0
    for x in np.linspace(-0.3, 0.3, 7):
        for zf in np.linspace(FLANGE_TOP, FLANGE_TOP - 0.4, 5):
            tcp = (float(x), float(zf - g.TOOL_LEN))
            cell.set_robot(tcp)
            mujoco.mj_forward(cell.m, cell.d)
            site = cell.d.site("tcp").xpos
            worst_tcp = max(worst_tcp, math.hypot(site[0] - tcp[0], site[2] - tcp[1]))
            eq = [i for i in range(cell.d.nefc) if cell.d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
            worst_loop = max(worst_loop, float(np.abs(cell.d.efc_pos[eq]).max()))
    assert worst_tcp < 0.5e-3, f"TCP off by {worst_tcp * 1e3:.3f} mm"
    assert worst_loop < 0.5e-3, f"loop closure off by {worst_loop * 1e3:.3f} mm"


MOVING = ["rod1a", "rod1b", "tri0", "tri1", "tri2", "upperL", "upperR"]
SUPPORT = ["mountplate", "carrierL", "carrierR"]


def test_support_clears_robot_over_datasheet_range():
    """Rod pair 1, the triangle and the upper arms stay >= 10 mm from the adapter plate and the carriers
    over the whole 1000 x 400 datasheet range (MuJoCo geometry distance, real poses with closed loops)."""
    cell = sim.Cell(g.Params(), use_mesh=False)
    m, d = cell.m, cell.d
    ids = [(m.geom(a).id, m.geom(b).id, a, b) for a in MOVING for b in SUPPORT]
    fromto = np.zeros(6)
    worst = (1.0, "")
    for x in np.linspace(-0.5, 0.5, 21):
        for zf in np.linspace(FLANGE_TOP, FLANGE_TOP - 0.4, 9):
            tcp = (float(x), float(zf - g.TOOL_LEN))
            if g.ik2(tcp) is None:
                continue
            cell.set_robot(tcp)
            mujoco.mj_forward(m, d)
            for g1, g2, a, b in ids:
                dist = mujoco.mj_geomDistance(m, d, g1, g2, 0.5, fromto)
                if dist < worst[0]:
                    worst = (dist, f"{a} <-> {b} at x {x:.2f}, flange z {zf:.3f}")
    print(f"\nsupport clearance over the datasheet range: {worst[0] * 1e3:.1f} mm ({worst[1]})")
    assert worst[0] >= 0.010, f"{worst[0] * 1e3:.1f} mm: {worst[1]}"


def test_closed_loops_and_level_plate(cycle):
    L = cycle.log
    assert max(L.eq_err) < 0.5e-3, f"loop error {max(L.eq_err) * 1e3:.3f} mm"
    tilt = max(abs(v) for v in L.tilt)
    assert math.degrees(tilt) < 0.2, f"plate tilt {math.degrees(tilt):.3f} deg"


def test_tracking(cycle):
    tr, ta = np.array(cycle.log.tcp_ref), np.array(cycle.log.tcp)[:, [0, 2]]
    err = np.linalg.norm(tr - ta, axis=1)
    assert err.max() < 5e-3, f"TCP tracking error {err.max() * 1e3:.2f} mm"


def test_all_cartons_transferred_upright_between_guides(cycle):
    p = cycle.p
    assert len(cycle.placed) == p.n_cartons, f"{len(cycle.placed)} of {p.n_cartons} placed"
    assert len(cycle.place_check) == p.n_cartons
    for k, (x, y, tilt) in cycle.place_check.items():
        assert tilt < 5.0, f"carton {k} tilted {tilt:.1f} deg"
        side = g.BELT_B["w"] / 2 - g.CARTON["x"] / 2 - abs(x - g.BELT_B["x"])
        assert side > 0.0, f"carton {k} not between the guides of belt B (x {x:.3f})"
    assert min(cycle.min_z) > g.CARTON["h"] / 2 - 0.01, "a carton fell off a belt"


def test_no_carton_collision_while_held(cycle):
    """The held carton never touches another carton (escapement interlock). Fails under --perturb."""
    hits = cycle.log.cc_contact
    assert not hits, f"{len(hits)} contact samples, first at {hits[0][0]:.2f} s between cartons {hits[0][1]} and {hits[0][2]}, depth {hits[0][3] * 1e3:.1f} mm"


def test_held_carton_clears_the_infeed_guides(cycle):
    """The carton is lifted out of the 220 mm guides without rubbing: lateral gap >= 5 mm on both sides."""
    gaps = cycle.log.guide_gap
    assert gaps and min(gaps) >= 0.005, f"min lateral gap {min(gaps) * 1e3:.1f} mm"


def test_vacuum_holds(cycle):
    slip = max(cycle.log.slip) if cycle.log.slip else 0.0
    assert slip < 5e-3, f"carton moved {slip * 1e3:.2f} mm relative to the pad in transit"


def test_torques_match_analytic_model_and_motor(cycle):
    L = cycle.log
    tau, ana = np.array(L.tau), np.array(L.tau_ana)
    st, pf = np.array(L.state), np.array(L.pad_force)
    free = (pf < 1e-6) & np.isin(st, ["transfer", "back", "backPick", "down"])
    d = tau[free] - ana[free]
    rel = np.sqrt((d ** 2).mean(0)) / np.sqrt((tau[free] ** 2).mean(0))
    print(f"\nfree motion: rms diff {np.sqrt((d ** 2).mean(0)).round(2)} Nm, relative {(100 * rel).round(1)} %, "
          f"peak measured {np.abs(tau[free]).max(0).round(1)} vs analytic {np.abs(ana[free]).max(0).round(1)} Nm")
    assert (rel < 0.05).all(), f"analytic model differs by {(100 * rel).round(1)} % RMS from MuJoCo"
    i, eta = cycle.p.ratio, cycle.p.eta
    motor = np.abs(tau) / (i * eta)
    rms = np.sqrt((motor ** 2).mean(0))
    assert motor.max() <= 0.85 * g.MOTOR["Mmax"], f"motor peak {motor.max():.2f} Nm"
    assert (rms <= g.MOTOR["M0"]).all(), f"motor RMS {rms.round(2)} Nm"


def test_cadence_and_realtime(cycle):
    t = [tt for tt, _ in cycle.placed]
    period = (t[-1] - t[2]) / (len(t) - 3)
    print(f"\nperiod {period:.3f} s -> {60 / period:.1f} /min, real-time factor {cycle.d.time / cycle.wall:.2f}")
    assert 60 / period >= 25
    assert cycle.d.time / cycle.wall > 1.0


def test_deterministic():
    a, b = sim.Cell(g.Params(), use_mesh=False), sim.Cell(g.Params(), use_mesh=False)
    a.run(t_end=2.0)
    b.run(t_end=2.0)
    assert np.array_equal(a.d.qpos, b.d.qpos)
