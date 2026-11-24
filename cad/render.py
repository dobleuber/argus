"""Images of the mount system (on a headless server: MUJOCO_GL=osmesa python render.py).

Uses MuJoCo to assemble the real SO-101 arm on the exported parts (stl/), on the desk and on the backpack.
"""

import os
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sim"))
import scene  # noqa: E402

# Dimensions that match params.scad (mm)
ARM_BASE = np.array([-100, -260, 0])
BAR_Y, BASE_Z0, BASE_H = 39, -2.4, 16
SHOULDER_H = 1450  # mm, shoulder height (same as sim/backpack.py)
OUT = HERE.parent / "media"


def add_stl(spec, body, name, pos_mm=(0, 0, 0), rgba=(0.8, 0.8, 0.8, 1), quat=(1, 0, 0, 0)):
    mesh = spec.add_mesh()
    mesh.name, mesh.file, mesh.scale = f"m_{name}_{len(spec.meshes)}", str(HERE / "stl" / f"{name}.stl"), [1e-3] * 3
    g = body.add_geom()
    g.type, g.meshname = mujoco.mjtGeom.mjGEOM_MESH, mesh.name
    g.pos, g.rgba, g.quat = list(np.asarray(pos_mm) / 1000), list(rgba), list(quat)
    g.contype = g.conaffinity = 0
    return g


def mounted_cradle(spec, body, base_mm):
    add_stl(spec, body, "cradle", base_mm, (0.85, 0.85, 0.88, 1))
    for s in (-1, 1):
        add_stl(spec, body, "clamp_bar", np.add(base_mm, [0, s * BAR_Y, BASE_Z0 + BASE_H]), (0.3, 0.3, 0.32, 1))


def arm_spec(base_m=None):
    spec = mujoco.MjSpec.from_file(str(scene.ensure_model())) if base_m is not None else mujoco.MjSpec()
    if base_m is not None:
        spec.body("base").pos = list(base_m)
    sky = spec.add_texture()
    sky.name, sky.type, sky.builtin = "sky", mujoco.mjtTexture.mjTEXTURE_SKYBOX, mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1, sky.rgb2, sky.width, sky.height = [0.95, 0.95, 0.97], [0.75, 0.78, 0.82], 256, 1536
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1600, 1200
    spec.visual.headlight.ambient = [0.4, 0.4, 0.4]
    w = spec.worldbody
    li = w.add_light()
    li.pos, li.dir, li.type = [0.5, -1, 3], [-0.1, 0.3, -1], mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    return spec, w


def render(spec, views, q, path, title):
    m = spec.compile()
    d = mujoco.MjData(m)
    if q is not None:
        d.qpos[:5] = q
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 600, 600)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
    tiles = []
    for (lookat, dist, az, el, label) in views:
        cam = mujoco.MjvCamera()
        cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, dist, az, el
        r.update_scene(d, camera=cam)
        im = Image.fromarray(r.render())
        ImageDraw.Draw(im).text((12, 10), label, fill=(20, 20, 20), font=font)
        tiles.append(np.array(im))
    r.close()
    img = Image.fromarray(np.concatenate(tiles, 1))
    ImageDraw.Draw(img).text((12, 570), title, fill=(20, 20, 20), font=font)
    img.save(path)
    print("Image:", path)


def table():
    spec, w = arm_spec([0, 0, 0])
    # Desk: the top face of the desk base rests at world z = 0
    rail_z = BASE_Z0 - 4 - 9  # bottom face of the rail relative to the arm base (FLOOR=4, RAIL_H=9)
    mount_top = -(-rail_z) / 1000
    off = np.array([0, 0, 0])
    rx = np.array([-22.4 + 87 / 2 - 35 - 5, 0, rail_z])  # receiver centred under the cradle
    g = w.add_geom()
    g.type, g.size, g.pos, g.rgba = mujoco.mjtGeom.mjGEOM_BOX, [0.35, 0.3, 0.012], [0.12, 0, (rail_z - 8 - 8 - 12) / 1000], [0.75, 0.6, 0.45, 1]
    add_stl(spec, w, "desk_base", rx + off, (0.6, 0.7, 0.85, 1))
    add_stl(spec, w, "receiver", rx, (0.85, 0.85, 0.88, 1))
    add_stl(spec, w, "jaw", rx, (0.3, 0.3, 0.32, 1))
    mounted_cradle(spec, w, [0, 0, 0])
    views = [((0.05, 0, 0.03), 0.55, 135, -25, "On the desk"), ((0.05, 0, 0.0), 0.35, 90, -8, "From the side")]
    render(spec, views, [0, -1.0, 1.0, 0.6, -1.57], OUT / "mount_desk.png",
           "Cradle with rail + receiver with knob + base for 2 C-clamps")


def backpack():
    shoulder = np.array([0, 0, SHOULDER_H])
    spec, w = arm_spec((ARM_BASE + shoulder) / 1000)
    # Simple person (for scale) and transparent backpack
    def box(pos, size, rgba, gtype=mujoco.mjtGeom.mjGEOM_BOX):
        g = w.add_geom()
        g.type, g.size, g.pos, g.rgba = gtype, size, list(np.asarray(pos) / 1000), rgba
        g.contype = g.conaffinity = 0
    box(shoulder + [0, 0, -270], [0.11, 0.19, 0.28], [0.2, 0.45, 0.35, 1])
    box(shoulder + [20, 0, 200], [0.1, 0, 0], [0.35, 0.37, 0.42, 1], mujoco.mjtGeom.mjGEOM_SPHERE)
    box(shoulder + [10, 0, 70], [0.05, 0.04, 0], [0.35, 0.37, 0.42, 1], mujoco.mjtGeom.mjGEOM_CAPSULE)
    box(shoulder + [-200, 0, -200], [0.085, 0.165, 0.23], [0.75, 0.35, 0.15, 0.25])
    for name, rgba in [("mast_foot", (0.4, 0.42, 0.48, 1)), ("head_fixed", (0.4, 0.42, 0.48, 1)),
                       ("head_moving", (0.6, 0.7, 0.85, 1))]:
        add_stl(spec, w, name, shoulder, rgba)
    # Aluminium tube (between the same points as in params.scad)
    a, b = np.array([-154, -100, -60]) + shoulder, np.array([-154, -225, -45]) + shoulder
    g = w.add_geom()
    g.type, g.size, g.rgba = mujoco.mjtGeom.mjGEOM_BOX, [0.01, 0.01, 0], [0.75, 0.76, 0.78, 1]
    g.fromto = list(np.r_[a, b] / 1000)
    # Back plate and electronics box
    box(shoulder + [-136.5, 0, -40 - 190], [0.0015, 0.125, 0.19], [0.92, 0.92, 0.92, 0.9])
    box(shoulder + [-138 - 25, 0, -330], [0.025, 0.0725, 0.049], [0.3, 0.4, 0.25, 1])
    mounted_cradle(spec, w, ARM_BASE + shoulder)
    c = (shoulder + [-120, -120, -150]) / 1000
    views = [(c, 1.45, 30, -15, "On the backpack (right shoulder)"), (c, 1.45, 300, -15, "From behind")]
    render(spec, views, [0.2, -0.9, 0.9, -0.7, -1.57], OUT / "mount_backpack.png",
           "Plate (laptop pocket) + foot + 20x20 tube + head with hinge + receiver")


HINGE = np.array([-148.9, -260, -31.4])  # hinge axis (params.scad)


def head_detail():
    """The head on its own: closed and folded backwards (as when the arm gets snagged)."""
    tiles = []
    for fold in (0, 60):
        spec, w = arm_spec()
        li = w.add_light()
        li.pos, li.dir, li.type = [0.5, -1, 3], [-0.1, 0.3, -1], mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        spec.visual.global_.offwidth, spec.visual.global_.offheight = 1600, 1200
        for name, rgba in [("mast_foot", (0.4, 0.42, 0.48, 1)), ("head_fixed", (0.4, 0.42, 0.48, 1))]:
            add_stl(spec, w, name, (0, 0, 0), rgba)
        # Moving part (and what it carries) rotated about the hinge
        th = -np.radians(fold)
        q = [np.cos(th / 2), 0, np.sin(th / 2), 0]
        Ry = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])
        def rot(p):
            return HINGE + Ry @ (np.asarray(p, float) - HINGE)
        add_stl(spec, w, "head_moving", rot([0, 0, 0]), (0.6, 0.7, 0.85, 1), q)
        add_stl(spec, w, "cradle", rot(ARM_BASE), (0.85, 0.85, 0.88, 1), q)
        for sgn in (-1, 1):
            add_stl(spec, w, "clamp_bar", rot(ARM_BASE + [0, sgn * BAR_Y, BASE_Z0 + BASE_H]), (0.3, 0.3, 0.32, 1), q)
        a, b = np.array([-154, -100, -60]), np.array([-154, -225, -45])
        g = w.add_geom()
        g.type, g.size, g.rgba = mujoco.mjtGeom.mjGEOM_BOX, [0.01, 0.01, 0], [0.75, 0.76, 0.78, 1]
        g.fromto = list(np.r_[a, b] / 1000)
        m = spec.compile()
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        r = mujoco.Renderer(m, 600, 600)
        cam = mujoco.MjvCamera()
        cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = np.array([-120, -220, -20]) / 1000, 0.42, 250, -20
        r.update_scene(d, camera=cam)
        im = Image.fromarray(r.render())
        r.close()  # with two viewers open at once, osmesa returns garbage
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
        ImageDraw.Draw(im).text((12, 10), "Closed (magnets)" if fold == 0 else "Folded 60° backwards",
                                fill=(20, 20, 20), font=font)
        tiles.append(np.array(im))
    Image.fromarray(np.concatenate(tiles, 1)).save(OUT / "mount_hinge.png")
    print("Image:", OUT / "mount_hinge.png")


if __name__ == "__main__":
    os.environ.setdefault("MUJOCO_GL", "egl")
    table()
    backpack()
    head_detail()
