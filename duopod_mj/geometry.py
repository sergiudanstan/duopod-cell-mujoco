"""Geometry, kinematics and motion profiles of the DuoPod transfer cell.

Port of duopod-cell-pro/src/js/10-logic.js and 11-dynamics.js (same constants, same conventions).
World frame: metres, z up, belt top z = 0. The robot works in the plane y = 0 (x across the belts).
Joint centres were fitted on the bearing bores of the vendor STEP (Autonox 3D-model_A_00913).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------- robot: Autonox DuoPod RVS2-1000-3kg (A_00913)
B = 0.125            # half distance between the motor axes
L1 = 0.450           # upper arm
L2 = 0.930           # lower arm
E = 0.090            # half distance of the plate joints
HJ = 0.039           # plate joints above the flange (TCP plane of the datasheet)
MOUNT_ABOVE = 0.170  # mounting face above the motor axes
O1 = (0.04153, 0.11641)    # triangle top pivot - elbow (= head rod pivot - motor axis), right arm
O2 = (-0.11434, 0.05052)   # triangle lower pivot - elbow (= plate rod pivot - plate joint)
REF_ELBOW = (0.56495, 0.09458)  # CAD pose, relative to the motor-axis line
REF_FLANGE_Z = -0.744           # CAD pose flange, relative to the motor-axis line
MOUNT_Z = 1.330                 # mounting face above the belt top
ZM = MOUNT_Z - MOUNT_ABOVE      # motor-axis height
DATASHEET = dict(width=1.000, height=0.400, below=0.914, arc_depth=0.510)

# vacuum gripper (flange -> foam face) and payload
TOOL_LEN, PAD_X, PAD_Y, TOOL_MASS = 0.110, 0.160, 0.220, 1.2
G = 9.81

# mass properties from the STEP (tools/cad/mass.py in duopod-cell-pro): mass kg, CoM (x, z) in the CAD pose
# relative to the motor-axis line (right arm values; the left arm is mirrored), planar inertia about the CoM kg m^2
BODY_REF = {
    "upper": dict(m=7.1507, com=(0.32956, 0.05604), J=0.16993),
    "lower": dict(m=4.4467, com=(0.32795, -0.30531), J=0.36254),
    "plate": dict(m=2.5100, com=(-0.00148, -0.70643), J=0.01372),
    "tri": dict(m=1.2555, com=(0.55761, 0.15112), J=0.00583),
    "rod1": dict(m=2 * 0.2139, com=(0.39450, 0.17188), J=2 * 0.0034),
    "rod2": dict(m=2 * 0.4540, com=(0.20243, -0.25510), J=2 * 0.03139),
}
# recommended motor (Autonox page): Beckhoff AM8053-0N21, winding N, 400 V (Beckhoff Infosys AM805x)
MOTOR = dict(name="Beckhoff AM8053-0N21", M0=11.40, Mn=2.70, nN=7000.0, Mmax=53.14, nMax=9000.0, J=5.93e-4)
# vacuum on corrugated board: 30 kPa effective, 70 % of the pad sealed
VAC = dict(dp=30e3, sealed=0.70, mu=0.5, sf_req=2.0)

# ---------------------------------------------------------------- cell
CARTON = dict(x=0.180, y=0.250, h=0.120, m=3.0)
BELT_A = dict(x=0.20, w=0.22, v=0.40, y0=-1.75, y1=0.40)    # infeed, flows +y; guides 220 mm apart (20 mm play per side) keep
#   the carton centred under the gripper - with real friction a 300 mm gap lets it drift sideways
BELT_B = dict(x=-0.20, w=0.30, v=0.40, y0=-0.55, y1=1.75)   # outfeed, continuous
GUIDE_T, GUIDE_TOP = 0.010, 0.060
STOP_Y = CARTON["y"] / 2                     # carton front edge at the end stop -> carton centred on y = 0
BLADE_Y = STOP_Y - CARTON["y"] - 0.033       # escapement blade: queue head front edge stops here


@dataclass
class Params:
    travel: float = 0.280      # TCP height for transfers
    scale: float = 1.0         # motion time scale
    smart: bool = True         # straight back to the next carton when the photo-eye is confirmed
    ratio: float = 40.0        # gearbox ratio (assumption: not published)
    eta: float = 0.90
    grip_t: float = 0.12
    release_t: float = 0.10
    settle_t: float = 0.05
    place_gap: float = 0.030
    n_cartons: int = 12
    t_in: float = 1.25         # one carton every t_in s from upstream
    perturb: bool = False      # escapement released on vacuum ON (interlock removed) - tests must fail

    z_pick: float = field(init=False)
    z_place: float = field(init=False)
    z_clear: float = field(init=False)

    def __post_init__(self) -> None:
        self.z_pick = CARTON["h"] + 0.002    # pad stops 2 mm above the carton: adhesion acts across the foam gap
        self.z_place = CARTON["h"] + 0.002   # carton released 2 mm above the belt
        self.z_clear = 2 * CARTON["h"] + 0.020

    @property
    def pick(self) -> tuple[float, float]:
        return (BELT_A["x"], self.z_pick)

    @property
    def place(self) -> tuple[float, float]:
        return (BELT_B["x"], self.z_place)

    @property
    def ready(self) -> tuple[float, float]:
        return (BELT_A["x"], self.travel)


# ---------------------------------------------------------------- planar kinematics (x, z), world frame, TCP = foam face
ARMS = (-1, 1)   # left arm (x < 0), right arm


def motor_axis(s: int) -> tuple[float, float]:
    return (s * B, ZM)


def plate_joint(tcp, s: int) -> tuple[float, float]:
    return (tcp[0] + s * E, tcp[1] + TOOL_LEN + HJ)


def elbow_for(tcp, s: int):
    ax, az = motor_axis(s)
    px, pz = plate_joint(tcp, s)
    dx, dz = px - ax, pz - az
    d = math.hypot(dx, dz)
    if d > L1 + L2 or d < abs(L1 - L2) or d < 1e-9:
        return None
    a = (L1 * L1 - L2 * L2 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, L1 * L1 - a * a))
    mx, mz = ax + a * dx / d, az + a * dz / d
    e1 = (mx + h * dz / d, mz - h * dx / d)
    e2 = (mx - h * dz / d, mz + h * dx / d)
    return e1 if s * e1[0] > s * e2[0] else e2


def ik2(tcp):
    """Upper-arm angles above the outward horizontal (rad), [left, right]; None if out of reach."""
    th = []
    for s in ARMS:
        el = elbow_for(tcp, s)
        if el is None:
            return None
        ax, az = motor_axis(s)
        th.append(math.atan2(el[1] - az, s * (el[0] - ax)))
    return th


def elbows_from_theta(th):
    out = []
    for i, s in enumerate(ARMS):
        ax, az = motor_axis(s)
        out.append((ax + s * L1 * math.cos(th[i]), az + L1 * math.sin(th[i])))
    return out


def fk2(th):
    el, er = elbows_from_theta(th)
    c1, c2 = (el[0] + E, el[1]), (er[0] - E, er[1])
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dz)
    if d > 2 * L2 or d < 1e-9:
        return None
    h = math.sqrt(L2 * L2 - d * d / 4)
    mx, mz = (c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2
    p1, p2 = (mx + h * dz / d, mz - h * dx / d), (mx - h * dz / d, mz + h * dx / d)
    pc = p1 if p1[1] < p2[1] else p2
    return (pc[0], pc[1] - HJ - TOOL_LEN)


# ---------------------------------------------------------------- motion: overlapped single-axis minimum-jerk pieces
def ease(s: float) -> float:
    return s * s * s * (10 + s * (6 * s - 15))


class Move:
    """x and z each run through (t0, t1, from, to) pieces; between pieces an axis holds its value."""

    def __init__(self, T, x0, z0, xs, zs, **extra):
        self.T, self.x0, self.z0, self.xs, self.zs = T, x0, z0, xs, zs
        self.__dict__.update(extra)

    @staticmethod
    def _axis(v0, pieces, t):
        v = v0
        for a, b, f, to in pieces:
            if t <= a:
                return v
            v = to if t >= b else f + (to - f) * ease((t - a) / (b - a))
            if t < b:
                return v
        return v

    def at(self, t: float) -> tuple[float, float]:
        t = min(max(t, 0.0), self.T)
        return (self._axis(self.x0, self.xs, t), self._axis(self.z0, self.zs, t))


def build_moves(p: Params) -> dict[str, Move]:
    k, tr = p.scale, p.travel
    T = lambda v: v * k  # noqa: E731
    pick, place, ready = p.pick, p.place, p.ready
    return {
        "down": Move(T(0.24), ready[0], tr, [], [(0, T(0.24), tr, p.z_pick)]),
        "transfer": Move(T(0.62), pick[0], p.z_pick, [(T(0.16), T(0.46), pick[0], place[0])],
                         [(0, T(0.30), p.z_pick, tr), (T(0.32), T(0.62), tr, p.z_place)], z_down=T(0.32)),
        "back": Move(T(0.52), place[0], p.z_place, [(T(0.12), T(0.52), place[0], ready[0])], [(0, T(0.26), p.z_place, tr)]),
        # smart return: x arrives before the descent starts, so the move can pause at descend_at (zero speed)
        # until the photo-eye confirms the next carton - the descent never starts on a prediction
        "backPick": Move(T(0.68), place[0], p.z_place, [(T(0.12), T(0.44), place[0], pick[0])],
                         [(0, T(0.26), p.z_place, tr), (T(0.44), T(0.68), tr, p.z_pick)], descend_at=T(0.44)),
    }
