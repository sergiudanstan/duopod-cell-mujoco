"""MJCF of the DuoPod transfer cell.

Robot: planar closed-chain mechanism (5-bar + two orientation parallelograms) built in the CAD reference pose,
all hinges about +y, loops closed with <connect> equalities. Every body frame is axis-aligned at qpos = 0, so the
vendor meshes (local only) can be attached with a pure translation.
"""
from __future__ import annotations

from pathlib import Path

from . import geometry as g

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "_cad" / "meshes"

ZM = g.ZM
# reference joint points in world (x, z): CAD pose
A_L, A_R = (-g.B, ZM), (g.B, ZM)
E_L, E_R = (-g.REF_ELBOW[0], ZM + g.REF_ELBOW[1]), (g.REF_ELBOW[0], ZM + g.REF_ELBOW[1])
J_L, J_R = (-g.E, ZM - 0.705), (g.E, ZM - 0.705)
F_REF = (0.0, ZM + g.REF_FLANGE_Z)
ROD1_BASE = (g.B + g.O1[0], ZM + g.O1[1])
TRI_TOP = (E_R[0] + g.O1[0], E_R[1] + g.O1[1])
TRI_LOW = (E_R[0] + g.O2[0], E_R[1] + g.O2[1])
PLATE_PIVOT = (J_R[0] + g.O2[0], J_R[1] + g.O2[1])

# body origins in world at the reference pose (used to place local meshes)
BODY_ORIGIN = {
    "upperL": A_L, "upperR": A_R, "lowerL": E_L, "lowerR": E_R, "plate": J_L,
    "tri": E_R, "rod1": ROD1_BASE, "rod2": TRI_LOW,
}
CAD_PART = {  # vendor GLB node -> body (local meshes, never published)
    "MT_WST00065198:1": "head", "MT_WST00055121:1": "head", "MT_WST00055121:2": "head",
    "MT_WST00061011:1": "upperL", "MT_WST00061012:1": "upperR", "MT_WST00061013:1": "lowerL", "MT_WST00061013:2": "lowerR",
    "MT_WST00065199:1": "plate", "MT_WST00060925:1": "tri",
    "MT_WST00061015:1": "rod1", "MT_WST00061015:2": "rod1", "MT_WST00061014:1": "rod2", "MT_WST00061014:2": "rod2",
}


def v3(x, z, y=0.0):
    return f"{x:.6f} {y:.6f} {z:.6f}"


def rel(p, o):
    return (p[0] - o[0], p[1] - o[1])


def inertial(name, mirror=False, origin=(0, 0)):
    r = g.BODY_REF[name]
    cx, cz = r["com"]
    if mirror:
        cx = -cx
    com = rel((cx, ZM + cz), origin)
    J = r["J"]
    return f'<inertial pos="{v3(*com)}" mass="{r["m"]}" diaginertia="{J / 2:.6f} {J:.6f} {J / 2:.6f}"/>'


def capsule(a, b, r, mat, name=None, y=0.0):
    nm = f' name="{name}"' if name else ""
    return f'<geom{nm} type="capsule" fromto="{v3(a[0], a[1], y)} {v3(b[0], b[1], y)}" size="{r}" material="{mat}" class="link"/>'


def mesh_geoms(body: str, parts: dict[str, list[str]]) -> str:
    o = BODY_ORIGIN.get(body, (0.0, 0.0))
    out = []
    for i, f in enumerate(parts.get(body, [])):
        out.append(f'<geom type="mesh" mesh="{Path(f).stem}" pos="{v3(-o[0], -o[1])}" class="cadvis" material="{"red" if body == "plate" else "cad"}"/>')
    return "\n".join(out)


def build_xml(p: g.Params | None = None, use_mesh: bool | None = None) -> str:
    p = p or g.Params()
    meshes = sorted(MESH_DIR.glob("*.stl")) if MESH_DIR.exists() else []
    use_mesh = bool(meshes) if use_mesh is None else (use_mesh and bool(meshes))
    parts: dict[str, list[str]] = {}
    mesh_assets = ""
    if use_mesh:
        for f in meshes:
            body = f.stem.split("__")[0]
            parts.setdefault(body, []).append(str(f))
        mesh_assets = "\n".join(f'<mesh name="{f.stem}" file="{f}"/>' for f in meshes)
    proc = "" if use_mesh else ""   # procedural link geoms are always present (invisible group when meshes are shown)
    del proc
    link_group = 3 if use_mesh else 0

    ratio, eta = p.ratio, p.eta
    tq_max = g.MOTOR["Mmax"] * ratio * eta
    armature = g.MOTOR["J"] * ratio * ratio

    # ---------------- cell: belts, guides, end stop, frame
    A, Bb = g.BELT_A, g.BELT_B
    cell = []
    for nm, b in (("A", A), ("B", Bb)):
        L, yc = b["y1"] - b["y0"], (b["y0"] + b["y1"]) / 2
        cell.append(f'<geom name="belt{nm}" type="box" pos="{b["x"]} {yc} -0.004" size="{b["w"] / 2 + 0.02} {L / 2} 0.004" material="belt" class="env" priority="1" friction="0.02 0.005 0.0001"/>')
        for s in (-1, 1):
            xg = b["x"] + s * (b["w"] / 2 + g.GUIDE_T / 2)
            cell.append(f'<geom name="guide{nm}{"m" if s < 0 else "p"}" type="box" pos="{xg} {yc} {(g.GUIDE_TOP + 0.01) / 2}" size="{g.GUIDE_T / 2} {L / 2} {(g.GUIDE_TOP - 0.01) / 2}" material="hdpe" class="env" priority="1" friction="0.15 0.005 0.0001"/>')
            xs = b["x"] + s * (b["w"] / 2 + 0.035)
            cell.append(f'<geom type="box" pos="{xs} {yc} -0.055" size="0.015 {L / 2 + 0.03} 0.055" material="alu" class="vis"/>')
        for y in (b["y0"], b["y1"]):
            cell.append(f'<geom type="cylinder" fromto="{b["x"] - b["w"] / 2 - 0.02} {y} -0.04 {b["x"] + b["w"] / 2 + 0.02} {y} -0.04" size="0.04" material="steel" class="vis"/>')
        for k in range(int(L // 0.9) + 1):
            y = b["y0"] + 0.12 + k * min(0.9, (L - 0.24) / max(1, int(L // 0.9)))
            for s in (-1, 1):
                cell.append(f'<geom type="box" pos="{b["x"] + s * (b["w"] / 2 + 0.035)} {y} {(-0.11 - 0.85) / 2}" size="0.02 0.02 {(0.85 - 0.11) / 2}" material="alu" class="vis"/>')
    cell.append(f'<geom name="stopper" type="box" pos="{A["x"]} {g.STOP_Y + 0.006} 0.037" size="{A["w"] / 2} 0.006 0.033" material="steel" class="env" priority="1" friction="0.2 0.005 0.0001"/>')
    # frame: 4 posts, top beams, carriers, mounting plate; acrylic guards (visual only)
    PX, PY, TZ = 0.80, 0.72, g.MOUNT_Z + 0.07
    for sx in (-1, 1):
        for sy in (-1, 1):
            cell.append(f'<geom type="box" pos="{sx * PX} {sy * PY} {(TZ - 0.85) / 2}" size="0.04 0.04 {(TZ + 0.85) / 2}" material="alu" class="vis"/>')
        cell.append(f'<geom type="box" pos="{sx * PX} 0 {TZ}" size="0.04 {PY} 0.04" material="alu" class="vis"/>')
        cell.append(f'<geom type="box" pos="{sx * 0.30} 0 {TZ}" size="0.04 {PY} 0.04" material="alu" class="vis"/>')
        cell.append(f'<geom type="box" pos="{sx * (PX + 0.05)} 0 0.25" size="0.003 {PY - 0.03} 0.95" material="acrylic" class="vis"/>')
    for sy in (-1, 1):
        cell.append(f'<geom type="box" pos="0 {sy * PY} {TZ}" size="{PX + 0.04} 0.04 0.04" material="alu" class="vis"/>')
        for sx in (-1, 1):
            cell.append(f'<geom type="box" pos="{sx * 0.61} {sy * (PY + 0.05)} 0.25" size="0.19 0.003 0.95" material="acrylic" class="vis"/>')
        cell.append(f'<geom type="box" pos="0 {sy * (PY + 0.05)} 0.82" size="0.42 0.003 0.38" material="acrylic" class="vis"/>')
    cell.append(f'<geom type="box" pos="0 0 {g.MOUNT_Z + 0.015}" size="0.35 0.21 0.015" material="alu" class="vis"/>')
    # robot head (fixed), gearboxes and motors (motors are not in the vendor CAD)
    head = [
        f'<geom type="box" pos="0 0 {ZM + 0.155}" size="0.215 0.145 0.015" material="alu" class="vis"/>',
        f'<geom type="box" pos="0 0.13 {ZM + 0.02}" size="0.215 0.015 0.12" material="alu" class="vis"/>',
        f'<geom type="box" pos="0 -0.13 {ZM + 0.02}" size="0.215 0.015 0.12" material="alu" class="vis"/>',
    ] if not use_mesh else [mesh_geoms("head", parts)]
    for s in (-1, 1):
        head.append(f'<geom type="cylinder" fromto="{s * g.B} 0.145 {ZM} {s * g.B} 0.20 {ZM}" size="0.075" material="steel" class="vis"/>')
        head.append(f'<geom type="box" pos="{s * g.B} 0.31 {ZM}" size="0.05 0.09 0.05" material="black" class="vis"/>')

    # ---------------- cartons (free bodies), parked on the floor until they enter the infeed
    cartons = []
    for k in range(p.n_cartons):
        cartons.append(f'''<body name="carton{k}" pos="{2.4 + 0.3 * k} 1.5 {-0.85 + 0.0601}">
      <freejoint name="carton{k}"/>
      <geom name="carton{k}" type="box" size="{g.CARTON["x"] / 2} {g.CARTON["y"] / 2} {g.CARTON["h"] / 2}" mass="{g.CARTON["m"]}" material="carton" class="carton"/>
      <geom type="box" pos="0 0 {g.CARTON["h"] / 2 + 0.0006}" size="0.024 {g.CARTON["y"] / 2 + 0.001} 0.0006" material="tape" class="vis"/>
    </body>''')

    # ---------------- escapement: blade and side clamp on slide joints (outer side of belt A)
    so = 1 if A["x"] > 0 else -1
    blade_root = A["x"] + so * (A["w"] / 2 + 0.10)
    clamp_y = g.BLADE_Y - g.CARTON["y"] - g.CARTON["y"] / 2
    esc = f'''<body name="blade" pos="{blade_root} {g.BLADE_Y - 0.0045} 0.035">
      <joint name="blade" type="slide" axis="{so} 0 0" range="0 0.13" damping="50"/>
      <geom name="blade" type="box" pos="{-so * 0.10} 0 0" size="0.10 0.004 0.02" material="yellow" class="env" mass="0.3" priority="1" friction="0.2 0.005 0.0001"/>
    </body>
    <body name="clamp" pos="{A["x"] + so * (A["w"] / 2 + 0.005)} {clamp_y} 0.035">
      <joint name="clamp" type="slide" axis="{-so} 0 0" range="0 0.03" damping="30"/>
      <geom name="clamp" type="box" size="0.004 0.06 0.018" material="rubber" class="env" mass="0.2" priority="1" friction="1.0 0.005 0.0001"/>
    </body>'''

    # ---------------- robot (CAD reference pose, axis-aligned body frames)
    lk = f' group="{link_group}"' if use_mesh else ""

    def lgeoms(body, geoms):
        procedural = "\n".join(x.replace('class="link"', f'class="link"{lk}') for x in geoms)
        return procedural + ("\n" + mesh_geoms(body, parts) if use_mesh else "")

    ue_l, ue_r = rel(E_L, A_L), rel(E_R, A_R)
    lj_l, lj_r = rel(J_L, E_L), rel(J_R, E_R)
    robot = f'''
  <body name="upperL" pos="{v3(*A_L)}">
    <joint name="mL" axis="0 1 0" armature="{armature:.4f}" range="-2.2 2.2"/>
    {inertial("upper", mirror=True, origin=A_L)}
    {lgeoms("upperL", [capsule((0, 0), ue_l, 0.042, "carbon")])}
    <body name="lowerL" pos="{v3(*ue_l)}">
      <joint name="eL" axis="0 1 0"/>
      {inertial("lower", mirror=True, origin=E_L)}
      {lgeoms("lowerL", [capsule((0, 0), lj_l, 0.031, "carbon")])}
      <body name="plate" pos="{v3(*lj_l)}">
        <joint name="pL" axis="0 1 0"/>
        {inertial("plate", origin=J_L)}
        {lgeoms("plate", [f'<geom type="box" pos="{v3(g.E, -0.022)}" size="0.13 0.06 0.022" material="red" class="link"/>'])}
        <body name="gripper" pos="{v3(g.E, -g.HJ)}">
          <inertial pos="0 0 -0.055" mass="{g.TOOL_MASS}" diaginertia="0.004 0.004 0.004"/>
          <geom type="box" pos="0 0 -0.04" size="0.05 0.032 0.03" material="alu" class="vis"/>
          <geom name="pad" type="box" pos="0 0 {-g.TOOL_LEN + 0.016}" size="{g.PAD_X / 2} {g.PAD_Y / 2} 0.016" material="foam" class="pad"/>
          <site name="tcp" pos="0 0 {-g.TOOL_LEN}" size="0.006"/>
        </body>
      </body>
    </body>
  </body>
  <body name="upperR" pos="{v3(*A_R)}">
    <joint name="mR" axis="0 1 0" armature="{armature:.4f}" range="-2.2 2.2"/>
    {inertial("upper", origin=A_R)}
    {lgeoms("upperR", [capsule((0, 0), ue_r, 0.042, "carbon")])}
    <body name="lowerR" pos="{v3(*ue_r)}">
      <joint name="eR" axis="0 1 0"/>
      {inertial("lower", origin=E_R)}
      {lgeoms("lowerR", [capsule((0, 0), lj_r, 0.031, "carbon")])}
    </body>
    <body name="tri" pos="{v3(*ue_r)}">
      <joint name="tri" axis="0 1 0"/>
      {inertial("tri", origin=E_R)}
      {lgeoms("tri", [capsule((0, 0), g.O1, 0.02, "alu", y=0.085), capsule((0, 0), g.O2, 0.02, "alu", y=0.085), capsule(g.O1, g.O2, 0.02, "alu", y=0.085)])}
      <body name="rod2" pos="{v3(*g.O2)}">
        <joint name="r2" axis="0 1 0"/>
        {inertial("rod2", origin=TRI_LOW)}
        {lgeoms("rod2", [capsule((0, 0), rel(PLATE_PIVOT, TRI_LOW), 0.007, "carbon", y=0.071), capsule((0, 0), rel(PLATE_PIVOT, TRI_LOW), 0.007, "carbon", y=-0.071)])}
      </body>
    </body>
  </body>
  <body name="rod1" pos="{v3(*ROD1_BASE)}">
    <joint name="r1" axis="0 1 0"/>
    {inertial("rod1", origin=ROD1_BASE)}
    {lgeoms("rod1", [capsule((0, 0), rel(TRI_TOP, ROD1_BASE), 0.0075, "carbon", y=0.1015), capsule((0, 0), rel(TRI_TOP, ROD1_BASE), 0.0075, "carbon", y=-0.1015)])}
  </body>'''

    # photo-eyes: rangefinders across the belts (site z axis points across, toward -so for belt A)
    eyes = []
    eye_a = A["x"] + so * (A["w"] / 2 + 0.02)
    # site z axis = rotation of +z: point along -so*x  -> quat for rotating +z to -x is (cos45, 0, -sin45, 0) etc.
    quat_to_minus_x = "0.7071068 0 -0.7071068 0"
    quat_to_plus_x = "0.7071068 0 0.7071068 0"
    qa = quat_to_minus_x if so > 0 else quat_to_plus_x
    for nm, y in (("eye_stop", g.STOP_Y - 0.04), ("eye_zone", g.BLADE_Y + 0.06), ("eye_head", g.BLADE_Y - 0.05), ("eye_gate", g.BLADE_Y + 0.015), ("eye_clamp", g.BLADE_Y - g.CARTON["y"] - g.CARTON["y"] / 2)):
        eyes.append(f'<site name="{nm}" pos="{eye_a} {y} 0.09" quat="{qa}" size="0.008" rgba="0.1 0.4 0.9 1"/>')
    sb = -1 if Bb["x"] < 0 else 1
    eye_b = Bb["x"] + sb * (Bb["w"] / 2 + 0.02)
    qb = quat_to_minus_x if sb > 0 else quat_to_plus_x
    for k, y in enumerate((-g.CARTON["y"] / 2 - p.place_gap, 0.0, g.CARTON["y"] / 2 + p.place_gap)):
        eyes.append(f'<site name="eye_b{k}" pos="{eye_b} {y} 0.09" quat="{qb}" size="0.008" rgba="0.1 0.4 0.9 1"/>')
    sensors = "\n".join(f'<rangefinder name="{s}" site="{s}" cutoff="0.6"/>' for s in ["eye_stop", "eye_zone", "eye_head", "eye_gate", "eye_clamp", "eye_b0", "eye_b1", "eye_b2"])

    lowR_end = rel(J_R, E_R)
    rod1_end = rel(TRI_TOP, ROD1_BASE)
    rod2_end = rel(PLATE_PIVOT, TRI_LOW)
    vac_force = g.VAC["dp"] * g.PAD_X * g.PAD_Y * g.VAC["sealed"]

    return f'''<mujoco model="duopod_cell">
  <compiler angle="radian" autolimits="true" meshdir="{MESH_DIR}"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="3">
    <flag multiccd="enable"/>
  </option>
  <size memory="64M"/>
  <visual>
    <global offwidth="1920" offheight="1080" azimuth="135" elevation="-22"/>
    <quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5" specular="0.1 0.1 0.1"/>
    <map znear="0.01" zfar="40"/>
  </visual>
  <default>
    <geom solref="0.004 1" solimp="0.95 0.99 0.001"/>
    <default class="env"><geom contype="1" conaffinity="1" group="0"/></default>
    <default class="vis"><geom contype="0" conaffinity="0" group="1" mass="0"/></default>
    <default class="link"><geom contype="0" conaffinity="0" group="0" mass="0"/></default>
    <default class="cadvis"><geom contype="0" conaffinity="0" group="1" mass="0"/></default>
    <default class="carton"><geom contype="1" conaffinity="1" friction="0.6 0.005 0.0001" condim="4"/></default>
    <default class="pad"><geom contype="1" conaffinity="1" priority="2" friction="1.0 0.01 0.0001" condim="4" margin="0.003" gap="0.003" solref="0.01 1" mass="0"/></default>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.92 0.93 0.95" rgb2="0.62 0.65 0.70" width="256" height="256"/>
    <texture name="floor" type="2d" builtin="checker" rgb1="0.56 0.57 0.58" rgb2="0.52 0.53 0.54" width="512" height="512"/>
    <material name="floor" texture="floor" texrepeat="12 12" reflectance="0.05"/>
    <material name="alu" rgba="0.78 0.80 0.82 1" specular="0.6" shininess="0.6"/>
    <material name="steel" rgba="0.82 0.84 0.86 1" specular="0.8" shininess="0.8"/>
    <material name="carbon" rgba="0.08 0.09 0.10 1" specular="0.5" shininess="0.7"/>
    <material name="red" rgba="0.70 0.10 0.09 1" specular="0.5" shininess="0.6"/>
    <material name="cad" rgba="0.70 0.72 0.75 1" specular="0.6" shininess="0.6"/>
    <material name="black" rgba="0.08 0.08 0.09 1" specular="0.3"/>
    <material name="belt" rgba="0.18 0.30 0.23 1" specular="0.1"/>
    <material name="hdpe" rgba="0.94 0.94 0.91 1"/>
    <material name="carton" rgba="0.76 0.60 0.42 1" specular="0.05"/>
    <material name="tape" rgba="0.85 0.78 0.66 1"/>
    <material name="foam" rgba="0.17 0.23 0.29 1"/>
    <material name="yellow" rgba="0.95 0.76 0.0 1"/>
    <material name="rubber" rgba="0.08 0.08 0.08 1"/>
    <material name="acrylic" rgba="0.85 0.92 0.97 0.12" specular="0.9" shininess="0.9"/>
    {mesh_assets}
  </asset>
  <worldbody>
    <light pos="1.8 -1.5 4" dir="-0.4 0.3 -1" diffuse="0.8 0.8 0.78" castshadow="true"/>
    <light pos="-2 2 3" dir="0.5 -0.5 -1" diffuse="0.3 0.32 0.35" castshadow="false"/>
    <geom name="floor" type="plane" pos="0 0 -0.85" size="8 8 0.1" material="floor" class="env" priority="1" friction="0.8 0.005 0.0001"/>
    <camera name="iso" pos="2.3 -2.6 1.6" xyaxes="0.75 0.66 0 -0.25 0.28 0.93"/>
    <camera name="front" pos="0 3.4 0.65" xyaxes="-1 0 0 0 -0.02 1"/>
    <camera name="pick" pos="0.75 -0.75 0.55" xyaxes="0.71 0.71 0 -0.25 0.25 0.94"/>
    <camera name="robot" pos="-0.75 0.95 1.25" xyaxes="-0.79 -0.61 0 0.08 -0.1 0.99"/>
    {"".join(cell)}
    {"".join(head)}
    {"".join(eyes)}
    {esc}
    {robot}
    {"".join(cartons)}
  </worldbody>
  <equality>
    <connect name="loopR" body1="lowerR" body2="plate" anchor="{v3(*lowR_end)}" solref="0.002 1"/>
    <connect name="loop1" body1="rod1" body2="tri" anchor="{v3(*rod1_end)}" solref="0.002 1"/>
    <connect name="loop2" body1="rod2" body2="plate" anchor="{v3(*rod2_end)}" solref="0.002 1"/>
  </equality>
  <contact>
    <exclude body1="blade" body2="clamp"/>
    <exclude body1="blade" body2="world"/>
    <exclude body1="clamp" body2="world"/>
  </contact>
  <actuator>
    <motor name="mL" joint="mL" ctrlrange="{-tq_max:.1f} {tq_max:.1f}"/>
    <motor name="mR" joint="mR" ctrlrange="{-tq_max:.1f} {tq_max:.1f}"/>
    <position name="blade" joint="blade" kp="4000" kv="120" ctrlrange="0 0.13" forcerange="-400 400"/>
    <position name="clamp" joint="clamp" kp="3000" kv="80" ctrlrange="0 0.03" forcerange="-180 180"/>
    <adhesion name="vacuum" body="gripper" ctrlrange="0 1" gain="{vac_force:.1f}"/>
  </actuator>
  <sensor>
    {sensors}
    <actuatorfrc name="tqL" actuator="mL"/>
    <actuatorfrc name="tqR" actuator="mR"/>
    <framepos name="tcp" objtype="site" objname="tcp"/>
  </sensor>
</mujoco>'''


def load(p: g.Params | None = None, use_mesh: bool | None = None):
    import mujoco
    return mujoco.MjModel.from_xml_string(build_xml(p, use_mesh))
