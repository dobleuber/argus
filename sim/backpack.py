"""Backpack scene: the SO-101 arm mounted over the right shoulder of a walking person.

- The wearer is a "mocap" body (we move it with a realistic gait pattern: bobs up and down
  ~2 cm at 1.8 steps/s, sways sideways, turns the hips a little).
- The backpack + mount is a free body "welded" to the wearer: that way the arm receives the real
  accelerations of walking (it does not teleport).
- The mount is the one in cad/: plate on the back, 20x20 tube, and a head with a hinge that
  folds backwards if the arm gets snagged (magnets: ~7 N·m closed, almost nothing once open).
- The IMU sits at the arm base, after the hinge (like a BNO055 on the cradle).
- There are people standing on the sidewalk (with skin-coloured faces, which the detector looks for).

Wearer axes: x = forward, y = left, z = up; origin = centre of the shoulders.
"""

import os
from dataclasses import dataclass

import mujoco
import numpy as np

import scene
from scene import B, CAP, P, S, _geom, add_person

SHOULDER_H = 1.45  # m, shoulder height (person of ~1.75 m)
# Arm base relative to the centre of the shoulders: behind, at the edge of the right shoulder, at
# shoulder height (further in, it ends up less than 10 cm from the head)
MOUNT = np.array([-0.10, -0.26, 0.0])
HEAD = np.array([0.02, 0.0, 0.2])  # centre of the wearer's head
HEAD_R = 0.10
STEP_HZ = 1.8  # steps per second
WALK_SPEED = 1.2  # m/s
# From the mount in cad/params.scad (mm -> m), relative to the arm base:
HINGE_REL = np.array([-0.0489, 0.0, -0.0314])   # hinge axis (HINGE - ARM_BASE)
TUBE_FROM = np.array([-0.054, 0.160, -0.060])   # FOOT_AT - ARM_BASE
TUBE_TO = np.array([-0.054, 0.035, -0.045])     # HEAD_AT - ARM_BASE
FOLD_TORQUE = 7.0  # N·m held by the closed magnets
MAGNET_DECAY = 0.05  # rad: once open ~3°, the magnet force drops to a third
FITS = {"tight": [0.01, 1.0], "loose": [0.05, 0.3]}  # how the backpack fits (weld: time constant, damping)
# People on the sidewalk: (name, x, y). Those on the left are on the other side of the head.
BYSTANDERS = [("alice", 6.0, -1.7), ("bob", 10.0, 1.8), ("carol", 16.0, -0.9)]


def mount_xml():
    """so101.xml with the arm base inside "fold" (hinge) inside "mount" (the backpack)."""
    src = scene.ensure_model()
    out = src.with_name("so101_mount.xml")
    xml = src.read_text()
    a = xml.index("<!-- Link base -->")
    b = xml.rindex("</worldbody>")
    base = xml[a:b]
    hx, hy, hz = HINGE_REL
    base = base.replace('<body name="base" pos="0 0 0"', f'<body name="base" pos="{-hx} {-hy} {-hz}"', 1)
    wrapped = (xml[:a] + '<body name="mount">\n<freejoint name="mount"/>\n'
               f'<body name="fold" pos="{hx} {hy} {hz}">\n'
               # Washer friction on the pin (~0.8 N·m): it folds without slamming. The closed stop
               # is rigid (face to face): a soft one makes the base wobble while walking.
               '<joint name="fold" type="hinge" axis="0 1 0" range="-1.6 0" damping="0.5" frictionloss="0.8" '
               'armature="0.002" solreflimit="0.01 1" solimplimit="0.95 0.99 0.001"/>\n'
               + base + "</body>\n</body>\n" + xml[b:])
    if not out.exists() or out.read_text() != wrapped:
        # Atomic write: several processes may build the scene at the same time
        tmp = out.with_suffix(f".{os.getpid()}.tmp")
        tmp.write_text(wrapped)
        os.replace(tmp, out)
    return out


def build(volts="12V", fit="tight", rigid=False, branch=None) -> mujoco.MjModel:
    spec = build_spec(volts, fit, rigid)
    if branch:
        # Branch (or low door frame) at height `branch` (m), on the right side only: it does not touch the head
        g = spec.worldbody.add_geom()
        g.name, g.type, g.size = "branch", mujoco.mjtGeom.mjGEOM_CAPSULE, [0.015, 0, 0]
        g.fromto = [7.0, -0.7, branch, 7.0, -0.13, branch]
        g.rgba = [0.4, 0.28, 0.15, 1]
    return spec.compile()


def magnets(m, d, torque=FOLD_TORQUE):
    """Torque of the hinge magnets: maximum when closed, drops quickly as it opens."""
    j = m.joint("fold")
    th = d.qpos[j.qposadr[0]]  # 0 = closed, negative = folded backwards
    # (with a soft stop the angle can go a little past 0: there the magnets are in contact)
    d.qfrc_applied[j.dofadr[0]] = torque * np.exp(min(th, 0.0) / MAGNET_DECAY)


def build_spec(volts="12V", fit="tight", rigid=False) -> mujoco.MjSpec:
    """The uncompiled scene (to add objects to it)."""
    spec = mujoco.MjSpec.from_file(str(mount_xml()))
    scene.configure_arm(spec, volts)
    w = spec.worldbody
    sky = spec.add_texture()
    sky.name, sky.type, sky.builtin = "sky", mujoco.mjtTexture.mjTEXTURE_SKYBOX, mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1, sky.rgb2, sky.width, sky.height = [0.55, 0.7, 0.9], [0.2, 0.3, 0.5], 256, 1536
    light = w.add_light()
    light.pos, light.dir, light.type = [5, -2, 6], [0.3, 0.3, -1], mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    light.castshadow = True

    # Sidewalk, street and buildings (cool colours: not mistaken for skin)
    _geom(w, "floor", P, (0, 0, 0.05), (0, 0, 0), (0.33, 0.35, 0.38, 1))
    _geom(w, "sidewalk", B, (14, 1.5, 0.005), (8, 0, 0.005), (0.6, 0.62, 0.66, 1))
    for i, x in enumerate(np.arange(-2, 22, 4.0)):
        _geom(w, None, B, (1.8, 0.8, 1.6 + 0.6 * (i % 3)), (x, 3.6, 1.6 + 0.6 * (i % 3)), (0.45, 0.55, 0.68, 1))
    for x, y in [(4, -2.6), (13, -2.6)]:
        _geom(w, None, CAP, (0.06, 1.5), (x, y, 1.5), (0.3, 0.3, 0.32, 1))  # posts

    for name, x, y in BYSTANDERS:
        add_person(w, name, [x, y, 0.0], face_z=1.62)  # facing -x: towards the approaching wearer

    # Wearer (mocap): hooded head (not skin-coloured, so the arm does not "see" itself), torso,
    # arms and legs. Legs and arms are separate mocap bodies to animate the gait.
    u = w.add_body()
    u.name, u.mocap, u.pos = "wearer", True, [0, 0, SHOULDER_H]
    nc = dict(contype=0, conaffinity=0)
    _geom(u, "wearer_head", S, (HEAD_R,), HEAD, (0.35, 0.37, 0.42, 1), **nc)
    _geom(u, None, CAP, (0.05, 0.04), (0.01, 0, 0.07), (0.35, 0.37, 0.42, 1), **nc)
    _geom(u, None, B, (0.11, 0.19, 0.28), (0, 0, -0.27), (0.2, 0.45, 0.35, 1), **nc)
    for side in ("left", "right"):
        for limb, top, length, r in [("leg", -0.55, 0.85, 0.07), ("arm", -0.02, 0.55, 0.045)]:
            y = (0.09 if limb == "leg" else 0.23) * (1 if side == "left" else -1)
            lb = w.add_body()
            lb.name, lb.mocap, lb.pos = f"{side}_{limb}", True, [0, y, SHOULDER_H + top]
            _geom(lb, None, CAP, (r, length / 2), (0, 0, -length / 2), (0.2, 0.2, 0.28, 1) if limb == "leg"
                  else (0.2, 0.45, 0.35, 1), **nc)

    # Backpack + mount: free body welded to the wearer. Its origin is the arm base.
    mb = spec.body("mount")
    mb.pos = list(MOUNT + [0, 0, SHOULDER_H])
    to_user = -MOUNT  # from the mount to the centre of the shoulders
    _geom(mb, "backpack", B, (0.08, 0.16, 0.22), to_user + [-0.2, 0, -0.18], (0.75, 0.35, 0.15, 1),
          mass=2.5, **nc)
    _geom(mb, "tube", B, (0.01, 0.01, 0), rgba=(0.75, 0.76, 0.78, 1), mass=0.045, fromto=list(np.r_[TUBE_FROM, TUBE_TO]), **nc)
    _geom(mb, "foot", B, (0.015, 0.035, 0.035), TUBE_FROM, (0.4, 0.42, 0.48, 1), mass=0.072, **nc)
    _geom(mb, "head_fixed", B, (0.04, 0.035, 0.012), HINGE_REL + [0.03, 0, -0.012], (0.4, 0.42, 0.48, 1), mass=0.124, **nc)
    fb = spec.body("fold")  # moving part of the head + receiver + cradle (what folds with the arm)
    _geom(fb, "head_moving", B, (0.055, 0.035, 0.012), [0.065, 0, 0.012], (0.6, 0.7, 0.85, 1), mass=0.26, **nc)
    s = fb.add_site()
    s.name, s.pos = "mount_imu", list(-HINGE_REL + [0, 0, -0.01])
    gyro = spec.add_sensor()
    gyro.name, gyro.type, gyro.objtype, gyro.objname = ("mount_gyro", mujoco.mjtSensor.mjSENS_GYRO,
                                                       mujoco.mjtObj.mjOBJ_SITE, "mount_imu")
    eq = spec.add_equality()
    eq.name = "backpack_fit"
    eq.type, eq.objtype, eq.name1, eq.name2 = (mujoco.mjtEq.mjEQ_WELD, mujoco.mjtObj.mjOBJ_BODY,
                                               "mount", "wearer")
    eq.solref = FITS[fit]
    if rigid:  # for comparison: mount without a hinge (the joint is locked)
        lock = spec.add_equality()
        lock.name, lock.type, lock.objtype, lock.name1 = "fold_lock", mujoco.mjtEq.mjEQ_JOINT, mujoco.mjtObj.mjOBJ_JOINT, "fold"

    # Camera that follows the wearer (from the side and ahead), without the gait sway
    cb = w.add_body()
    cb.name, cb.mocap, cb.pos = "follow_cam", True, [0, 0, 0]
    cam = cb.add_camera()
    cam.name, cam.pos = "follow", [-0.95, -0.85, 2.0]  # over the shoulder, from behind
    cam.mode, cam.targetbody = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY, "cam_target"
    tb = w.add_body()  # point it looks at: a little ahead of the wearer, without the sway
    tb.name, tb.mocap, tb.pos = "cam_target", True, [0.6, -0.25, 1.5]
    return spec


def _yaw_quat(yaw):
    return [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]


def _euler_quat(roll, pitch, yaw):
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.array([roll, pitch, yaw]), "xyz")
    return q


@dataclass
class Walk:
    """Route: standing, walks, stops. speed(t) in m/s with smooth starts."""

    start: float = 2.0
    stop: float = 15.5
    speed_max: float = WALK_SPEED

    def speed(self, t):
        ramp = 0.8
        if t < self.start or t > self.stop + ramp:
            return 0.0
        if t < self.start + ramp:
            return self.speed_max * (t - self.start) / ramp
        if t > self.stop:
            return self.speed_max * (1 - (t - self.stop) / ramp)
        return self.speed_max


class Wearer:
    """Moves the wearer (and their arms and legs) with a gait pattern."""

    def __init__(self, model, data, walk=None):
        self.m, self.d = model, data
        self.walk = walk or Walk()
        self.x, self.phase = 0.0, 0.0
        ids = {n: model.body(n).mocapid[0] for n in
               ("wearer", "left_leg", "right_leg", "left_arm", "right_arm", "follow_cam", "cam_target")}
        self.ids = ids
        self.limb0 = {n: data.mocap_pos[i].copy() for n, i in ids.items()}

    def update(self, t, dt):
        v = self.walk.speed(t)
        k = v / WALK_SPEED  # gait intensity
        self.x += v * dt
        self.phase += 2 * np.pi * STEP_HZ * dt * (k > 0.01)
        ph = self.phase
        d, ids = self.d, self.ids
        # Centre of the shoulders: goes up and down twice per cycle (once per step), sways once
        pos = np.array([self.x, 0.02 * k * np.sin(ph / 2), SHOULDER_H + 0.02 * k * np.cos(ph)])
        d.mocap_pos[ids["wearer"]] = pos
        d.mocap_quat[ids["wearer"]] = _euler_quat(0.035 * k * np.sin(ph / 2), 0.04 * k + 0.02 * k * np.sin(ph),
                                                  0.05 * k * np.sin(ph / 2))
        for side, sgn in (("left", 1), ("right", -1)):
            for limb, amp in (("leg", 0.35), ("arm", -0.3)):
                n = f"{side}_{limb}"
                p0 = self.limb0[n]
                d.mocap_pos[ids[n]] = [self.x + p0[0], p0[1], p0[2] + (pos[2] - SHOULDER_H)]
                d.mocap_quat[ids[n]] = _euler_quat(0, sgn * amp * k * np.sin(ph / 2), 0)
        d.mocap_pos[ids["follow_cam"]] = [self.x, 0, 0]
        d.mocap_pos[ids["cam_target"]] = [self.x + 0.6, -0.25, 1.5]

    def head_center(self):
        i = self.ids["wearer"]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.d.mocap_quat[i])
        return self.d.mocap_pos[i] + R.reshape(3, 3) @ HEAD
