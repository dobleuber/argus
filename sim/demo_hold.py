"""Demo 2: the arm grasps an object, shows it to the person, holds it and puts it back.

Usage:
  python demo_hold.py                          # 30 g cube, print results
  python demo_hold.py --mass 0.2               # the same cube weighing 200 g
  python demo_hold.py --video hold.mp4         # record a video
  python demo_hold.py --volts 7.4V             # 7.4 V servos (less torque)

The sequence is scripted (inverse kinematics); the object position is taken from the
simulation. On the real robot it would come from the camera (or from imitation learning).
"""

import argparse

import mujoco
import numpy as np

import scene
from control import CONTROL_HZ, REST, Arm, reach_velocity

OPEN, CLOSED = 1.2, -0.17  # gripper (rad); "closed" squeezes against the object
DOWN = np.array([0.0, 0.0, -1.0])
# On the way back, up high, the gripper is tilted ~20°: vertical, the wrist hits its limit
TILTED = np.array([0.35, 0.0, -1.0]) / np.linalg.norm([0.35, 0.0, -1.0])
FORWARD = np.array([1.0, 0.0, 0.0])


def plan(obj0):
    """Steps: (name, gripper target, gripper direction, gripper opening, seconds)."""
    # The fixed jaw sits 2 cm to the side (+y) of the grasp point: shift 6 mm so that it comes
    # down outside the cube; on closing, the moving jaw pushes the cube against it.
    side = np.array([0, 0.006, 0])
    above = obj0 + side + [0, 0, 0.07]
    grasp = obj0 + side + [0, 0, -0.005]
    show = np.array([0.30, 0.0, 0.24])
    return [
        ("approach", above, DOWN, OPEN, 2.0),
        ("lower", grasp, DOWN, OPEN, 1.5),
        ("grasp", grasp, DOWN, CLOSED, 1.0),
        ("lift", above, DOWN, CLOSED, 1.5),
        ("show", show, FORWARD, CLOSED, 2.0),
        ("hold", show, FORWARD, CLOSED, 4.0),
        ("return", above, TILTED, CLOSED, 2.0),
        ("put down", grasp + [0, 0, 0.012], DOWN, CLOSED, 1.5),
        ("release", grasp + [0, 0, 0.012], DOWN, OPEN, 0.8),
        ("retreat", above, DOWN, OPEN, 1.2),
    ]


def run(obj="cube", mass=None, volts="12V", video=None):
    m = scene.build(volts)
    body = m.body(obj).id
    if mass:
        m.body_inertia[body] *= mass / m.body_mass[body]  # same shape, heavier
        m.body_mass[body] = mass
    d = mujoco.MjData(m)
    mujoco.mj_setConst(m, d)  # recompute what depends on the mass (otherwise the contact blows up)
    arm = Arm(m, d)
    arm.set_pose(REST)
    arm.grip_cmd = OPEN
    d.mocap_pos[0] = [0.95, 0.0, 0.0]
    mujoco.mj_forward(m, d)
    rec = Recorder(m) if video else None
    obj0 = d.xpos[body].copy()

    log = {}
    for name, target, approach, grip, seconds in plan(obj0):
        arm.grip_cmd = grip
        tq, errs = [], []
        for _ in range(int(seconds * CONTROL_HZ)):
            dq, err = reach_velocity(arm, target, approach)
            arm.move(dq)
            arm.step()
            tq.append(np.abs(d.actuator_force[arm.act]))
            errs.append(err)
            if rec:
                rec.frame(d, name)
        log[name] = dict(torque=np.array(tq), err=errs[-1], obj_z=float(d.xpos[body][2]),
                         obj_to_grip=float(np.linalg.norm(d.xpos[body] - arm.site_pose()[0])))
    if rec:
        rec.save(video)

    held = log["hold"]
    return {
        "mass_g": 1000 * float(m.body_mass[body]),
        # Held = the object is still in the gripper (within 3 cm) and above the desk
        "held": held["obj_to_grip"] < 0.03 and held["obj_z"] > 0.1,
        "hold_torque": held["torque"][-25:].mean(0),  # mean over the last second
        "max_torque": max(float(v["torque"].max()) for v in log.values()),
        "placed_back": log["retreat"]["obj_z"] < 0.05,
        "reach_err_mm": {k: 1000 * v["err"] for k, v in log.items()},
    }


class Recorder:
    def __init__(self, m):
        from PIL import ImageFont

        self.r = mujoco.Renderer(m, 480, 640)
        self.cam_r = mujoco.Renderer(m, 168, 224)
        self.frames = []
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)

    def frame(self, d, step):
        from PIL import Image, ImageDraw

        self.r.update_scene(d, camera="side")
        big = Image.fromarray(self.r.render())
        self.cam_r.update_scene(d, camera="wrist_cam")
        big.paste(Image.fromarray(self.cam_r.render()), (8, 8))
        dr = ImageDraw.Draw(big)
        dr.rectangle([8, 8, 232, 176], outline=(255, 255, 255), width=2)
        dr.text((12, 180), "arm camera", fill=(255, 255, 255), font=self.font)
        dr.text((470, 10), step, fill=(255, 230, 90), font=self.font)
        self.frames.append(np.array(big))

    def save(self, path):
        import imageio.v2 as iio

        iio.mimsave(path, self.frames, fps=CONTROL_HZ, macro_block_size=1)
        print("Video saved to", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="cube", choices=["cube", "ball", "can"])
    ap.add_argument("--mass", type=float, help="kg (default: the object's own mass)")
    ap.add_argument("--volts", default="12V", choices=list(scene.STALL_TORQUE))
    ap.add_argument("--video")
    args = ap.parse_args()
    out = run(args.object, args.mass, args.volts, args.video)
    names = ["base", "shoulder", "elbow", "wrist", "wrist roll"]
    print(f"Object: {args.object}, {out['mass_g']:.0f} g, servos {args.volts}")
    print("Held it:", "yes" if out["held"] else "NO", "| Put it back on the desk:", "yes" if out["placed_back"] else "no")
    print("Torque while holding (N·m):", ", ".join(f"{n} {t:.2f}" for n, t in zip(names, out["hold_torque"])))
    print(f"Max torque: {out['max_torque']:.2f} N·m (limit {scene.STALL_TORQUE[args.volts]} N·m)")


if __name__ == "__main__":
    main()
