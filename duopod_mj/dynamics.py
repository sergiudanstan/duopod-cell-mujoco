"""Analytic rigid-body dynamics of the mechanism (port of duopod-cell-pro/src/js/11-dynamics.js).

Torques by virtual work: tau_j = sum_b m_b (a_b + g z) . dp_b/dtheta_j + J_b phi''_b dphi_b/dtheta_j,
theta = upper-arm angle above the outward horizontal, [left, right]. Used as feed-forward for the servos and
compared in the tests with the torques MuJoCo actually needs.
"""
from __future__ import annotations

import math

from . import geometry as g

ZM = g.ZM
REF_A = [(-g.B, ZM), (g.B, ZM)]
REF_E = [(-g.REF_ELBOW[0], ZM + g.REF_ELBOW[1]), (g.REF_ELBOW[0], ZM + g.REF_ELBOW[1])]
REF_J = [(-g.E, ZM - 0.705), (g.E, ZM - 0.705)]
REF_F = (0.0, ZM + g.REF_FLANGE_Z)


def _ang(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _rot(v, a):
    c, s = math.cos(a), math.sin(a)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)


def _moved(ref, piv_ref, piv_new, d):
    r = _rot((ref[0] - piv_ref[0], ref[1] - piv_ref[1]), d)
    return (piv_new[0] + r[0], piv_new[1] + r[1])


def _W(c):
    return (c[0], ZM + c[1])


def body_states(tcp, holding: bool, payload: float):
    th = g.ik2(tcp)
    if th is None:
        return None
    A = [g.motor_axis(s) for s in g.ARMS]
    El = g.elbows_from_theta(th)
    Jt = [g.plate_joint(tcp, s) for s in g.ARMS]
    F = (tcp[0], tcp[1] + g.TOOL_LEN)
    out = []
    R = g.BODY_REF
    for i in range(2):
        side = (lambda c, i=i: _W((-c[0], c[1]) if i == 0 else c))
        dU = _wrap(_ang(A[i], El[i]) - _ang(REF_A[i], REF_E[i]))
        out.append((R["upper"]["m"], R["upper"]["J"], _moved(side(R["upper"]["com"]), REF_A[i], A[i], dU), dU))
        dL = _wrap(_ang(El[i], Jt[i]) - _ang(REF_E[i], REF_J[i]))
        out.append((R["lower"]["m"], R["lower"]["J"], _moved(side(R["lower"]["com"]), REF_E[i], El[i], dL), dL))
    dF = (F[0] - REF_F[0], F[1] - REF_F[1])
    pc = _W(R["plate"]["com"])
    out.append((R["plate"]["m"], 0.0, (pc[0] + dF[0], pc[1] + dF[1]), 0.0))
    out.append((g.TOOL_MASS, 0.0, (F[0], F[1] - 0.055), 0.0))
    if holding:
        out.append((payload, 0.0, (tcp[0], tcp[1] - g.CARTON["h"] / 2), 0.0))
    dE = (El[1][0] - REF_E[1][0], El[1][1] - REF_E[1][1])
    tc = _W(R["tri"]["com"])
    out.append((R["tri"]["m"], 0.0, (tc[0] + dE[0], tc[1] + dE[1]), 0.0))
    Bp = (REF_A[1][0] + g.O1[0], REF_A[1][1] + g.O1[1])
    dU1 = _wrap(_ang(A[1], El[1]) - _ang(REF_A[1], REF_E[1]))
    out.append((R["rod1"]["m"], R["rod1"]["J"], _moved(_W(R["rod1"]["com"]), Bp, Bp, dU1), dU1))
    low_ref = (REF_E[1][0] + g.O2[0], REF_E[1][1] + g.O2[1])
    low_new = (El[1][0] + g.O2[0], El[1][1] + g.O2[1])
    dL1 = _wrap(_ang(El[1], Jt[1]) - _ang(REF_E[1], REF_J[1]))
    out.append((R["rod2"]["m"], R["rod2"]["J"], _moved(_W(R["rod2"]["com"]), low_ref, low_new, dL1), dL1))
    return out


def joint_torques(tcp_fn, t: float, holding: bool, payload: float, h: float = 2e-3):
    """Arm-side torques [left, right] (theta convention) and theta, theta', theta'' at time t of a TCP trajectory."""
    pts = [tcp_fn(t - h), tcp_fn(t), tcp_fn(t + h)]
    st = [body_states(p, holding, payload) for p in pts]
    th = [g.ik2(p) for p in pts]
    thd = [(th[2][j] - th[0][j]) / (2 * h) for j in range(2)]
    thdd = [(th[2][j] - 2 * th[1][j] + th[0][j]) / (h * h) for j in range(2)]
    d, tau = 1e-6, [0.0, 0.0]
    for j in range(2):
        a, b = list(th[1]), list(th[1])
        a[j] += d
        b[j] -= d
        sa, sb = body_states(g.fk2(a), holding, payload), body_states(g.fk2(b), holding, payload)
        for k, (m, J, p, phi) in enumerate(st[1]):
            acc = ((st[2][k][2][0] - 2 * p[0] + st[0][k][2][0]) / (h * h), (st[2][k][2][1] - 2 * p[1] + st[0][k][2][1]) / (h * h))
            phidd = (st[2][k][3] - 2 * phi + st[0][k][3]) / (h * h)
            dp = ((sa[k][2][0] - sb[k][2][0]) / (2 * d), (sa[k][2][1] - sb[k][2][1]) / (2 * d))
            dphi = (sa[k][3] - sb[k][3]) / (2 * d)
            tau[j] += m * (acc[0] * dp[0] + (acc[1] + g.G) * dp[1]) + J * phidd * dphi
    return tau, th[1], thd, thdd
