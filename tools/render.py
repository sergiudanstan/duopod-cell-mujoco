"""Offscreen film (MP4) and stills (PNG) of the MuJoCo cell.

Usage: uv run python tools/render.py [--procedural] [--seconds N]
Writes ~/Downloads/duopod_cell_mujoco/duopod_cell_mujoco[_procedural].mp4 and stills in .../shots/.
"""
import argparse
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from duopod_mj import geometry as g
from duopod_mj import sim

OUT = Path.home() / "Downloads" / "duopod_cell_mujoco"
W, H, FPS = 1600, 900, 30
# free cameras: lookat (x, y, z), distance, azimuth, elevation
VIEWS = {
    "iso": ((0.0, -0.05, 0.45), 4.0, 140, -20),
    "front": ((0.0, 0.0, 0.62), 3.3, 90, -6),
    "pick": ((0.18, 0.0, 0.16), 1.15, 150, -24),
    "robot": ((0.0, 0.05, 1.02), 1.7, 115, -14),
    "top": ((0.0, 0.0, 0.0), 3.6, 90, -88),
}


def camera(name):
    c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    (c.lookat[0], c.lookat[1], c.lookat[2]), c.distance, c.azimuth, c.elevation = VIEWS[name]
    return c


def font(size):
    for f in ("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Helvetica.ttc"):
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()


F1, F2 = font(26), font(20)
LABEL = {"wait": "wait for carton", "down": "descend", "grip": "vacuum on", "holdB": "belt B drop zone check",
         "transfer": "transfer A -> B", "release": "vacuum off", "back": "return", "backPick": "straight to next carton"}


def overlay(img, cell, view):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im, "RGBA")
    d.rounded_rectangle((16, 16, 640, 150), 12, fill=(255, 255, 255, 215))
    d.text((32, 26), "DuoPod transfer cell · MuJoCo physics", font=F1, fill=(20, 20, 20, 255))
    tq = cell.d.sensordata[[cell.sensor["tqL"], cell.sensor["tqR"]]] / (cell.p.ratio * cell.p.eta)
    d.text((32, 64), f"t {cell.d.time:5.2f} s   robot: {LABEL.get(cell.state, cell.state)}", font=F2, fill=(40, 40, 40, 255))
    d.text((32, 92), f"placed {len(cell.placed):2d}/{cell.p.n_cartons}   vacuum {'ON ' if cell.vac else 'off'}   "
                     f"motor torque M1 {tq[1]:+5.2f}  M2 {tq[0]:+5.2f} Nm", font=F2, fill=(40, 40, 40, 255))
    d.text((32, 120), f"camera: {view}   escapement {'open' if cell.blade_open else 'closed'}", font=F2, fill=(90, 90, 90, 255))
    return np.asarray(im)


def view_at(t, dur):
    if 4.6 <= t < 7.6:
        return "pick"
    if 9.5 <= t < 12.5:
        return "front"
    if 14.0 <= t < 16.5:
        return "robot"
    return "iso"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--procedural", action="store_true", help="hide the local vendor meshes")
    ap.add_argument("--seconds", type=float, default=None)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "shots").mkdir(exist_ok=True)
    cell = sim.Cell(g.Params(), use_mesh=not a.procedural)
    tag = "_procedural" if a.procedural or not any(cell.m.geom_type == mujoco.mjtGeom.mjGEOM_MESH) else ""
    r = mujoco.Renderer(cell.m, H, W)
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    opt.sitegroup[:] = 0                                       # hide sensor sites
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0          # hide photo-eye rays
    mp4 = OUT / f"duopod_cell_mujoco{tag}.mp4"
    writer = imageio.get_writer(mp4, fps=FPS, codec="libx264", quality=8, macro_block_size=1, pixelformat="yuv420p")
    stills = {1.5: "iso", 3.40: "pick", 3.9: "pick", 5.0: "front", 6.0: "robot", 7.0: "top"}
    done = set()

    def frame(c):
        t = c.d.time
        v = view_at(t, None)
        r.update_scene(c.d, camera(v), opt)
        img = r.render()
        writer.append_data(overlay(img, c, v))
        for ts, sv in stills.items():
            if ts not in done and t >= ts:
                done.add(ts)
                r.update_scene(c.d, camera(sv), opt)
                path = OUT / "shots" / f"{sv}_{ts:.2f}{tag}.png"
                Image.fromarray(overlay(r.render(), c, sv)).save(path)
                print(path)

    cell.run(t_end=a.seconds, on_frame=frame, frame_dt=1 / FPS)
    writer.close()
    print(mp4)
    print(f"placed {len(cell.placed)} cartons in {cell.d.time:.2f} s")


if __name__ == "__main__":
    main()
