"""Run the DuoPod cell in MuJoCo: cell controller (sensors -> state machine -> servo targets), belt drive, logging."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from . import dynamics as dyn
from . import geometry as g
from . import model as mdl

KP, KD = 40000.0, 600.0          # arm-side servo gains (Nm/rad, Nm s/rad): ~25 Hz with the reflected motor inertia
CTRL_EVERY = 2                   # control at 1 kHz (timestep 0.5 ms)
BELT_MU, BELT_C = 0.35, 250.0    # belt friction drive: force pulls the carton speed to the belt speed, capped at mu*N
SPAWN_FRONT = g.BELT_A["y0"] + g.CARTON["y"] + 0.05
PRELOAD_FRONTS = [g.STOP_Y - 0.001, g.BLADE_Y - 0.0015, g.BLADE_Y - g.CARTON["y"] - 0.45, g.BLADE_Y - g.CARTON["y"] - 0.95]


@dataclass
class Log:
    t: list = field(default_factory=list)
    tcp_ref: list = field(default_factory=list)
    tcp: list = field(default_factory=list)
    tau: list = field(default_factory=list)        # measured actuator torque, joint side [mL, mR]
    tau_ana: list = field(default_factory=list)    # analytic model (+ armature) for the same reference, joint side
    qdd: list = field(default_factory=list)
    qd: list = field(default_factory=list)
    eq_err: list = field(default_factory=list)     # max loop-closure error, m
    tilt: list = field(default_factory=list)       # plate pitch, rad
    state: list = field(default_factory=list)
    slip: list = field(default_factory=list)       # held carton offset change relative to the pad, m
    cc_contact: list = field(default_factory=list)  # (t, k1, k2, depth) carton-carton contacts while one is held
    pad_force: list = field(default_factory=list)   # normal contact force on the pad, N (0 = free motion)
    events: list = field(default_factory=list)


class Cell:
    def __init__(self, p: g.Params | None = None, use_mesh: bool | None = None):
        self.p = p or g.Params()
        self.m = mdl.load(self.p, use_mesh)
        self.d = mujoco.MjData(self.m)
        self.moves = g.build_moves(self.p)
        m = self.m
        self.jid = {n: m.joint(n).id for n in ["mL", "eL", "pL", "mR", "eR", "tri", "r2", "r1"]}
        self.qadr = {n: m.jnt_qposadr[i] for n, i in self.jid.items()}
        self.dadr = {n: m.jnt_dofadr[i] for n, i in self.jid.items()}
        self.carton_body = [m.body(f"carton{k}").id for k in range(self.p.n_cartons)]
        self.carton_qadr = [m.jnt_qposadr[m.joint(f"carton{k}").id] for k in range(self.p.n_cartons)]
        self.carton_dadr = [m.jnt_dofadr[m.joint(f"carton{k}").id] for k in range(self.p.n_cartons)]
        self.carton_geom = {m.geom(f"carton{k}").id: k for k in range(self.p.n_cartons)}
        self.pad_geom = m.geom("pad").id
        self.gripper_body = m.body("gripper").id
        self.plate_body = m.body("plate").id
        self.sensor = {n: m.sensor(n).adr[0] for n in ["eye_stop", "eye_zone", "eye_head", "eye_gate", "eye_clamp", "eye_b0", "eye_b1", "eye_b2", "tqL", "tqR"]}
        self.th_ref0 = [math.pi - dyn._ang(dyn.REF_A[0], dyn.REF_E[0]), dyn._ang(dyn.REF_A[1], dyn.REF_E[1])]
        self.armature = g.MOTOR["J"] * self.p.ratio ** 2
        self.log = Log()
        self.reset()

    # ------------------------------------------------------------ pose helpers
    def q_for_tcp(self, tcp) -> dict[str, float]:
        """Joint values (all hinges about +y) for a TCP: q = -(child angle change - parent angle change)."""
        th = g.ik2(tcp)
        A = [g.motor_axis(s) for s in g.ARMS]
        El = g.elbows_from_theta(th)
        Jt = [g.plate_joint(tcp, s) for s in g.ARMS]
        dU = [dyn._wrap(dyn._ang(A[i], El[i]) - dyn._ang(dyn.REF_A[i], dyn.REF_E[i])) for i in range(2)]
        dL = [dyn._wrap(dyn._ang(El[i], Jt[i]) - dyn._ang(dyn.REF_E[i], dyn.REF_J[i])) for i in range(2)]
        return {"mL": -dU[0], "eL": -(dL[0] - dU[0]), "pL": dL[0], "mR": -dU[1], "eR": -(dL[1] - dU[1]),
                "tri": dU[1], "r2": -dL[1], "r1": -dU[1]}

    def theta_to_q(self, th):     # motor joints: left q = dtheta, right q = -dtheta
        return [th[0] - self.th_ref0[0], -(th[1] - self.th_ref0[1])]

    def set_robot(self, tcp):
        for n, v in self.q_for_tcp(tcp).items():
            self.d.qpos[self.qadr[n]] = v
            self.d.qvel[self.dadr[n]] = 0.0

    def place_carton(self, k: int, x: float, y: float, z: float = 0.0):
        a, b = self.carton_qadr[k], self.carton_dadr[k]
        self.d.qpos[a:a + 7] = [x, y, z + g.CARTON["h"] / 2 + 0.0005, 1, 0, 0, 0]
        self.d.qvel[b:b + 6] = 0.0

    def park(self, k: int):
        self.place_carton(k, 2.4 + 0.3 * k, 1.5, -0.85)
        self.status[k] = "parked" if self.status[k] == "new" else "gone"

    def carton_pos(self, k):
        a = self.carton_qadr[k]
        return self.d.qpos[a:a + 3]

    # ------------------------------------------------------------ reset
    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        p = self.p
        self.status = ["new"] * p.n_cartons
        self.spawn_due = [0.0 if k < len(PRELOAD_FRONTS) else (k - len(PRELOAD_FRONTS) + 1) * p.t_in for k in range(p.n_cartons)]
        for k in range(p.n_cartons):
            if k < len(PRELOAD_FRONTS):
                self.place_carton(k, g.BELT_A["x"], PRELOAD_FRONTS[k] - g.CARTON["y"] / 2)
                self.status[k] = "infeed"
            else:
                self.park(k)
        self.set_robot(p.ready)
        mujoco.mj_forward(self.m, self.d)
        # robot state machine
        self.state, self.t0, self.move, self.clock = "wait", 0.0, None, 0.0
        self.paused = False
        self.next_pick, self.held = 0, None
        self.eye_on_since: dict[str, float | None] = {}
        self.blade_open, self.blade_open_t, self.head_clear_since, self.gate_seen = False, 0.0, None, False
        self.held_offset = None
        self.placed = []
        self.place_check: dict[int, tuple] = {}   # carton -> (x, y, tilt_deg) 0.3 s after release
        self.min_z = [1e9] * p.n_cartons
        self.tcp_ref = p.ready
        self.vac = 0.0
        self.d.ctrl[self.m.actuator("blade").id] = 0.0
        self.d.ctrl[self.m.actuator("clamp").id] = 0.0

    # ------------------------------------------------------------ sensors
    def eye(self, name: str) -> bool:
        v = self.d.sensordata[self.sensor[name]]
        return 0.0 <= v < 0.4

    def eye_stable(self, name: str, t: float, hold: float) -> bool:
        on = self.eye(name)
        since = self.eye_on_since.get(name)
        if on and since is None:
            self.eye_on_since[name] = t
        elif not on:
            self.eye_on_since[name] = None
        since = self.eye_on_since.get(name)
        return on and since is not None and t - since >= hold - 1e-9

    def tcp(self):
        return self.d.site("tcp").xpos

    # ------------------------------------------------------------ belts: friction drive on cartons resting on a belt
    def belt_forces(self):
        xf = self.d.xfrc_applied
        for k, bid in enumerate(self.carton_body):
            xf[bid, :] = 0.0
            if self.status[k] in ("parked", "gone", "held"):
                continue
            x, y, z = self.carton_pos(k)
            if z > g.CARTON["h"] / 2 + 0.006:
                continue
            for belt in (g.BELT_A, g.BELT_B):
                if abs(x - belt["x"]) < belt["w"] / 2 + 0.03 and belt["y0"] < y < belt["y1"]:
                    v = self.d.qvel[self.carton_dadr[k]:self.carton_dadr[k] + 6]
                    N = g.CARTON["m"] * g.G
                    cap = BELT_MU * N
                    xf[bid, 1] = max(-cap, min(cap, BELT_C * (belt["v"] - v[1])))
                    xf[bid, 0] = max(-cap, min(cap, -BELT_C * v[0]))
                    xf[bid, 5] = -2.0 * v[5]
                    break

    # ------------------------------------------------------------ infeed spawn and outfeed removal
    def manage_cartons(self, t: float):
        for k in range(self.p.n_cartons):
            if self.status[k] == "parked" and t >= self.spawn_due[k]:
                yc = SPAWN_FRONT - g.CARTON["y"] / 2
                free = all(self.status[j] not in ("infeed",) or abs(self.carton_pos(j)[1] - yc) > g.CARTON["y"] + 0.02
                           for j in range(self.p.n_cartons) if j != k)
                if free:
                    self.place_carton(k, g.BELT_A["x"], yc)
                    self.status[k] = "infeed"
            if self.status[k] in ("infeed", "held", "placed"):
                self.min_z[k] = min(self.min_z[k], float(self.carton_pos(k)[2]))
            if self.status[k] == "placed" and k not in self.place_check:
                tp = next(tt for tt, kk in self.placed if kk == k)
                if t >= tp + 0.3:
                    a = self.carton_qadr[k]
                    q = self.d.qpos[a + 3:a + 7]
                    tilt = math.degrees(2 * math.asin(min(1.0, math.hypot(q[1], q[2]))))
                    self.place_check[k] = (float(self.d.qpos[a]), float(self.d.qpos[a + 1]), tilt)
            if self.status[k] == "placed" and self.carton_pos(k)[1] > g.BELT_B["y1"] - 0.15:
                self.park(k)

    # ------------------------------------------------------------ escapement (sensor based, same interlock as the twin)
    def escapement(self, t: float):
        p, tcp = self.p, self.tcp()
        column_clear = abs(tcp[0] - g.BELT_A["x"]) > (g.CARTON["x"] + g.PAD_X) / 2 + 0.02 or tcp[2] >= p.z_clear
        perturbed = p.perturb and self.state in ("grip", "holdB", "transfer") and t - self.t0 >= 0.0
        if perturbed:
            column_clear = True                    # interlocks removed: release as soon as the vacuum is on
        bl, cl = self.m.actuator("blade").id, self.m.actuator("clamp").id
        if not self.blade_open:
            zone_free = (not self.eye("eye_stop") and not self.eye("eye_zone")) or perturbed
            if self.eye_stable("eye_head", t, 0.05) and zone_free and column_clear:
                self.blade_open, self.blade_open_t, self.head_clear_since, self.gate_seen = True, t, None, False
                # clamp the carton behind the head only if one is standing there (queued); a moving one is left alone
                self.d.ctrl[bl] = 0.13
                self.d.ctrl[cl] = 0.03 if self.eye_stable("eye_clamp", t, 0.05) else 0.0
                self.log.events.append((t, "escapement open"))
        else:
            # close only after the released carton has covered the gate eye (just downstream of the blade) and left it
            if self.eye("eye_gate"):
                self.gate_seen, self.head_clear_since = True, None
            elif self.gate_seen:
                if self.head_clear_since is None:
                    self.head_clear_since = t
                elif t - self.head_clear_since > 0.03:
                    self.blade_open, self.gate_seen = False, False
                    self.d.ctrl[bl], self.d.ctrl[cl] = 0.0, 0.0
                    self.log.events.append((t, "escapement closed"))

    # ------------------------------------------------------------ robot state machine
    def start(self, state: str, t: float, move: str | None = None):
        self.state, self.t0, self.move, self.clock, self.paused = state, t, move, 0.0, False
        self.log.events.append((t, state))

    def held_carton(self):
        """Carton under / on the pad: the one at the stopper, nearest to the TCP."""
        tcp = self.tcp()
        best, bd = None, 1e9
        for k in range(self.p.n_cartons):
            if self.status[k] != "infeed":
                continue
            c = self.carton_pos(k)
            dd = math.hypot(c[0] - tcp[0], c[1] - tcp[1])
            if dd < bd:
                best, bd = k, dd
        return best if bd < 0.05 else None

    def robot(self, t: float, dt: float):
        p, mv = self.p, self.moves
        st = self.state
        if st == "wait":
            if self.next_pick >= p.n_cartons:
                return
            if self.eye_stable("eye_stop", t, p.settle_t):
                self.start("down", t, "down")
        elif self.move:
            M = mv[self.move]
            if self.move == "backPick" and not self.paused and self.clock >= M.descend_at - 1e-9 and not self.eye_stable("eye_stop", t, p.settle_t):
                self.paused = True                  # wait above the pick point, zero speed, until the photo-eye confirms
                self.log.events.append((t, "smart return paused"))
            if self.paused and self.eye_stable("eye_stop", t, p.settle_t):
                self.paused = False
            if not self.paused:
                self.clock += dt
            if self.clock >= M.T - 1e-9:
                self.arrive(t)
        else:
            if st == "grip" and t - self.t0 >= p.grip_t:
                self.held = self.held_carton()
                if self.held is None:
                    self.log.events.append((t, "no carton under the pad"))
                    self.vac = 0.0
                    self.start("wait", t)
                    return
                self.status[self.held] = "held"
                self.held_offset = np.array(self.carton_pos(self.held)) - np.array(self.tcp())
                self.start("holdB", t)
            elif st == "holdB":
                if not (self.eye("eye_b0") or self.eye("eye_b1") or self.eye("eye_b2")):
                    self.start("transfer", t, "transfer")
            elif st == "release" and t - self.t0 >= p.release_t:
                self.status[self.held] = "placed"
                self.placed.append((t, self.held))
                self.held, self.held_offset = None, None
                self.next_pick += 1
                in_transit = self.blade_open or self.eye("eye_zone") or self.eye("eye_stop")
                if p.smart and in_transit and self.next_pick < p.n_cartons:
                    self.start("backPick", t, "backPick")
                else:
                    self.start("back", t, "back")

    def arrive(self, t: float):
        mv = self.move
        if mv in ("down", "backPick"):
            self.vac = 1.0
            self.start("grip", t)
        elif mv == "transfer":
            self.vac = 0.0
            self.start("release", t)
        elif mv == "back":
            self.start("wait", t)

    def ref_tcp(self, extra: float = 0.0):
        if self.move:
            return self.moves[self.move].at(self.clock + (0.0 if self.paused else extra))
        if self.state in ("grip", "holdB"):
            return self.p.pick
        if self.state == "release":
            return self.p.place
        return self.p.ready

    # ------------------------------------------------------------ servo control (PD with velocity reference + model feed-forward)
    def servo(self, holding: bool):
        fn = lambda tt: self.ref_tcp(tt)  # noqa: E731
        tau_th, th, thd, thdd = dyn.joint_torques(fn, 0.0, holding, g.CARTON["m"])
        q_ref = self.theta_to_q(th)
        qd_ref = [thd[0], -thd[1]]
        qdd_ref = [thdd[0], -thdd[1]]
        ff = [tau_th[0] + self.armature * qdd_ref[0], -tau_th[1] + self.armature * qdd_ref[1]]
        out = []
        for i, n in enumerate(("mL", "mR")):
            q, qd = self.d.qpos[self.qadr[n]], self.d.qvel[self.dadr[n]]
            out.append(KP * (q_ref[i] - q) + KD * (qd_ref[i] - qd) + ff[i])
        lim = g.MOTOR["Mmax"] * self.p.ratio * self.p.eta
        self.d.ctrl[0] = max(-lim, min(lim, out[0]))
        self.d.ctrl[1] = max(-lim, min(lim, out[1]))
        return ff, qdd_ref, qd_ref

    # ------------------------------------------------------------ main loop
    def run(self, t_end: float | None = None, on_frame=None, frame_dt: float = 1 / 30, log_every: int = 10):
        m, d, p = self.m, self.d, self.p
        dt = m.opt.timestep * CTRL_EVERY
        next_frame, step = 0.0, 0
        t_limit = t_end if t_end is not None else 60.0
        done_t = None
        eq_rows = None
        while d.time < t_limit:
            t = d.time
            if step % CTRL_EVERY == 0:
                self.manage_cartons(t)
                self.escapement(t)
                self.robot(t, dt)
                holding = self.held is not None and self.state in ("holdB", "transfer")
                ff, qdd_ref, qd_ref = self.servo(holding)
                d.ctrl[m.actuator("vacuum").id] = self.vac
                if t_end is None and done_t is None and self.next_pick >= p.n_cartons and self.state == "back":
                    done_t = t
                if done_t is not None and t > done_t + 1.5:
                    break
            self.belt_forces()
            mujoco.mj_step(m, d)
            step += 1
            if step % log_every == 0:
                self._log(ff, qdd_ref, qd_ref)
            if on_frame is not None and d.time >= next_frame:
                on_frame(self)
                next_frame += frame_dt
        return self.log

    def _log(self, ff, qdd_ref, qd_ref):
        d, L = self.d, self.log
        L.t.append(d.time)
        L.tcp_ref.append(self.ref_tcp())
        L.tcp.append(tuple(self.tcp()))
        L.tau.append((d.sensordata[self.sensor["tqL"]], d.sensordata[self.sensor["tqR"]]))
        L.tau_ana.append(tuple(ff))
        L.qdd.append(tuple(qdd_ref))
        L.qd.append(tuple(qd_ref))
        L.state.append(self.state)
        eq = [i for i in range(d.nefc) if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
        L.eq_err.append(float(np.max(np.abs(d.efc_pos[eq]))) if eq else 0.0)
        R = d.xmat[self.plate_body].reshape(3, 3)
        L.tilt.append(math.atan2(R[0, 2], R[0, 0]))
        fsum, buf = 0.0, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if self.pad_geom in (c.geom1, c.geom2):
                mujoco.mj_contactForce(self.m, d, i, buf)
                fsum += abs(buf[0])
        L.pad_force.append(fsum)
        if self.held is not None and self.held_offset is not None:
            off = np.array(self.carton_pos(self.held)) - np.array(self.tcp())
            L.slip.append(float(np.linalg.norm(off - self.held_offset)))
        if self.held is not None:
            for i in range(d.ncon):
                c = d.contact[i]
                k1, k2 = self.carton_geom.get(c.geom1), self.carton_geom.get(c.geom2)
                if k1 is not None and k2 is not None and self.held in (k1, k2) and c.dist < 0.0:
                    L.cc_contact.append((d.time, k1, k2, -c.dist))


def run_cycle(p: g.Params | None = None, use_mesh: bool | None = False, **kw):
    cell = Cell(p, use_mesh)
    cell.run(**kw)
    return cell
