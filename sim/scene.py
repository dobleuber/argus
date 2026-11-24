"""Desk scene: SO-101 arm + desk + objects + a "person" who moves.

The arm model is the official one from MuJoCo Menagerie (robotstudio_so101, Apache 2.0). It is not
copied into the repo: it is downloaded once into third_party/ at a fixed version (MENAGERIE_COMMIT).

Axes: x = forward of the arm, y = left, z = up. The arm base is at (0, 0, 0) on the desk.
"""

import subprocess
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
THIRD = ROOT / "third_party"
SO101_DIR = THIRD / "so101"
MENAGERIE_URL = "https://github.com/google-deepmind/mujoco_menagerie.git"
MENAGERIE_COMMIT = "cda5236f98383f6fbaa4820c6b18dba892cd087f"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
TABLE_Z = 0.0  # height of the desk top (the arm base sits on it)
FLOOR_Z = -0.75

# Feetech STS3215: stall torque by version
STALL_TORQUE = {"7.4V": 1.91, "12V": 2.94}  # N·m (19.5 and 30 kg·cm)
GRIP_TORQUE = 0.8  # N·m: gripper limit (LeRobot limits it the same way in the servo)

# Skin colour of the "person" (the camera detector looks for it)
SKIN = (0.92, 0.72, 0.58, 1)


def ensure_model() -> Path:
    """Download the SO-101 model (only the folder needed) if it is not there."""
    xml = SO101_DIR / "so101.xml"
    if xml.exists():
        return xml
    repo = THIRD / "menagerie"
    repo.mkdir(parents=True, exist_ok=True)

    def git(*args):
        subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)

    git("init", "-q")
    git("remote", "add", "origin", MENAGERIE_URL)
    git("sparse-checkout", "set", "robotstudio_so101")
    git("fetch", "-q", "--depth", "1", "--filter=blob:none", "origin", MENAGERIE_COMMIT)
    git("checkout", "-q", "FETCH_HEAD")
    (repo / "robotstudio_so101").rename(SO101_DIR)
    return xml


def _geom(body, name, gtype, size, pos=(0, 0, 0), rgba=(0.8, 0.8, 0.8, 1), **kw):
    g = body.add_geom()
    if name:
        g.name = name
    g.type = gtype
    g.size = list(size) + [0] * (3 - len(size))
    g.pos = list(pos)
    g.rgba = list(rgba)
    for k, v in kw.items():
        setattr(g, k, v)
    return g


def configure_arm(spec, volts):
    """Arm settings shared by all scenes."""
    spec.option.timestep = 0.005
    # Maximum servo torque for the chosen version
    for a in spec.actuators:
        a.forcerange = [-STALL_TORQUE[volts], STALL_TORQUE[volts]]
    # Gripper: limited torque (~10 N of grip). With full torque it crushes objects and lets them go.
    spec.actuator(GRIPPER).forcerange = [-GRIP_TORQUE, GRIP_TORQUE]
    # The convex meshes of the jaws fill the gap where the object goes: only the collision boxes
    # and spheres that come with the model are used.
    for g in spec.geoms:
        if g.classname.name == "collision_gripper_mesh":
            g.contype = g.conaffinity = 0
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1280, 720


B, S, C, P, CAP = (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CYLINDER,
                   mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_CAPSULE)


def add_person(parent, name, pos, face_z=0.42, mocap=False):
    """Simple person: torso, skin-coloured face (what the detector looks for), hair, eyes and mouth.

    Faces -x. face_z = height of the face above `pos`.
    """
    b = parent.add_body()
    b.name, b.pos, b.mocap = name, list(pos), mocap
    nc = dict(contype=0, conaffinity=0)
    _geom(b, None, B, (0.09, 0.17, 0.2), (0.05, 0, face_z - 0.3), (0.25, 0.4, 0.55, 1), **nc)
    if face_z > 1.0:  # standing: legs
        for y in (-0.08, 0.08):
            _geom(b, None, CAP, (0.06, (face_z - 0.55) / 2), (0.05, y, (face_z - 0.55) / 2), (0.2, 0.2, 0.25, 1), **nc)
    _geom(b, f"{name}_face" if name != "person" else "face", S, (0.085,), (0, 0, face_z), SKIN, **nc)
    _geom(b, None, S, (0.088,), (0.025, 0, face_z + 0.025), (0.2, 0.13, 0.08, 1), **nc)
    for y in (-0.03, 0.03):
        _geom(b, None, S, (0.011,), (-0.078, y, face_z + 0.015), (0.05, 0.05, 0.05, 1), **nc)
    _geom(b, None, B, (0.005, 0.025, 0.006), (-0.08, 0, face_z - 0.035), (0.6, 0.25, 0.25, 1), **nc)
    return b


def build(volts="12V", objects=True, person=True) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(ensure_model()))
    configure_arm(spec, volts)
    w = spec.worldbody
    light = w.add_light()
    light.pos, light.dir, light.type = [0.3, -0.3, 2.0], [0, 0.2, -1], mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    _geom(w, "floor", P, (0, 0, 0.05), (0, 0, FLOOR_Z), (0.25, 0.27, 0.3, 1))
    # Desk 90 x 100 cm (light grey so as not to confuse the skin detector)
    _geom(w, "table", B, (0.45, 0.5, 0.015), (0.3, 0, TABLE_Z - 0.015), (0.82, 0.83, 0.85, 1))
    for sx in (-0.13, 0.73):
        for sy in (-0.45, 0.45):
            _geom(w, None, B, (0.02, 0.02, (TABLE_Z - FLOOR_Z) / 2 - 0.015),
                  (sx, sy, (TABLE_Z + FLOOR_Z) / 2 - 0.015), (0.4, 0.4, 0.42, 1))
    # Back wall
    _geom(w, "wall", B, (0.02, 2.0, 1.2), (2.2, 0, FLOOR_Z + 1.2), (0.55, 0.62, 0.7, 1))

    if objects:
        # Green 4 x 4 x 6 cm cube (30 g), red ball and blue can
        for name, gtype, size, pos, rgba, mass in [
            ("cube", B, (0.02, 0.02, 0.03), (0.24, 0.0, 0.03), (0.1, 0.75, 0.2, 1), 0.03),
            ("ball", S, (0.022,), (0.22, 0.16, 0.022), (0.85, 0.1, 0.1, 1), 0.04),
            ("can", C, (0.02, 0.04), (0.2, -0.16, 0.04), (0.15, 0.3, 0.85, 1), 0.06),
        ]:
            b = w.add_body()
            b.name, b.pos = name, list(pos)
            b.add_freejoint()
            _geom(b, name, gtype, size, rgba=rgba, mass=mass, condim=4, friction=[1.0, 0.03, 0.003],
                  solref=[0.01, 1])

    if person:
        # Person sitting across the desk ("mocap" body: we move it ourselves)
        add_person(w, "person", [0.95, 0.0, 0.0], mocap=True)

    # Fixed external camera for the videos
    target = w.add_body()
    target.name, target.pos = "view_target", [0.5, 0.05, 0.12]
    cam = w.add_camera()
    cam.name, cam.pos = "side", [-0.35, -1.25, 0.6]
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "view_target"
    return spec.compile()


def arm_ids(model):
    """Indices in qpos/qvel/ctrl of the 5 arm joints and of the gripper."""
    j = [model.joint(n).id for n in ARM_JOINTS]
    qadr = np.array([model.jnt_qposadr[i] for i in j])
    dadr = np.array([model.jnt_dofadr[i] for i in j])
    act = np.array([model.actuator(n).id for n in ARM_JOINTS])
    g = model.joint(GRIPPER).id
    return qadr, dadr, act, model.jnt_qposadr[g], model.actuator(GRIPPER).id
