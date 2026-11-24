"""Demo 1: the arm searches for a person with its camera, greets them and follows them with its gaze.

Usage:
  python demo_track.py                       # simulate and print results
  python demo_track.py --video track.mp4     # also record a video (external view + camera)
  MUJOCO_GL=osmesa python demo_track.py ...  # on a headless server

The arm only uses the image from its wrist camera (skin detector) and its own joint angles.
The true position of the person is used only to measure how well it tracks them.
"""

import argparse

import mujoco
import numpy as np

import scene
from control import CONTROL_HZ, REST, Arm, detect_face, look_velocity

DET_W, DET_H = 160, 120  # detection resolution (the real robot would use something similar)


def person_path(t):
    """Path of the person: (x, y, z) of the body."""
    if t < 4:  # sitting in front
        return 0.95, 0.0, 0.0
    if t < 10:  # moves from side to side
        return 0.95, 0.28 * np.sin((t - 4) * 2 * np.pi / 6), 0.0
    if t < 13:  # comes closer and stands up a little
        k = (t - 10) / 3
        return 0.95 - 0.2 * np.sin(np.pi * k), 0.0, 0.12 * np.sin(np.pi * k)
    if t < 14:  # walks off to one side...
        return 0.95 + 0.6 * (t - 13), -0.6 * (t - 13), 0.0
    if t < 17:  # ...and leaves the room
        return 0.95, 0.0, -5.0
    if t < 20:  # comes back from the other side
        return 0.95, 0.7 - 0.7 * (t - 17) / 3, 0.0
    return 0.95, 0.0, 0.0


def run(seconds=23.0, video=None, volts="12V"):
    m = scene.build(volts)
    d = mujoco.MjData(m)
    arm = Arm(m, d)
    arm.set_pose(REST)
    arm.grip_cmd = 0.0
    det = mujoco.Renderer(m, DET_H, DET_W)
    face = m.geom("face").id
    rec = Recorder(m) if video else None

    state, lost_for, t_seen, nod_until = "search", 0.0, None, -1.0
    stats = dict(frames=0, seen=0, err=[], torque=[], states=[])
    n = int(seconds * CONTROL_HZ)
    for k in range(n):
        t = k / CONTROL_HZ
        d.mocap_pos[0] = person_path(t)
        mujoco.mj_forward(m, d)

        det.update_scene(d, camera="wrist_cam")
        img = det.render()
        hit = detect_face(img)
        if hit:
            lost_for = 0.0
            if state == "search":
                state, nod_until = "greet", t + 1.2
        else:
            lost_for += 1 / CONTROL_HZ
            if lost_for > 0.8:
                state = "search"

        if state == "search":
            # Sweep: turn the base from side to side, looking straight ahead
            target = REST.copy()
            target[0] = 1.2 * np.sin(2 * np.pi * t / 6)
            dq = 2.0 * (target - d.qpos[arm.qadr])
        else:
            dq = look_velocity(arm, *(hit[:2] if hit else (0.0, 0.0)))
            if state == "greet":
                # Nod the "head" (wrist) on finding someone
                dq[3] += 1.5 * np.cos(2 * np.pi * 1.6 * t)
                if t > nod_until:
                    state = "track"
        arm.move(dq)
        arm.step()

        # Metrics: angle between the gaze and the true face
        cpos, Rc = arm.cam_frame()
        to_face = d.geom_xpos[face] - cpos
        ang = np.degrees(np.arccos(np.clip(-Rc[:, 2] @ to_face / np.linalg.norm(to_face), -1, 1)))
        stats["frames"] += 1
        stats["seen"] += bool(hit)
        if state == "track":
            stats["err"].append(ang)
        stats["torque"].append(np.abs(d.actuator_force[arm.act]))
        stats["states"].append(state)
        if rec:
            rec.frame(d, img, state, hit, t)

    if rec:
        rec.save(video)
    err = np.array(stats["err"]) if stats["err"] else np.array([np.nan])
    return {
        "visible_pct": 100 * stats["seen"] / stats["frames"],
        "track_err_median_deg": float(np.median(err)),
        "track_err_p90_deg": float(np.percentile(err, 90)),
        "max_torque": float(np.max(stats["torque"])),
        # Torque while tracking (99th percentile): what the servo does most of the time
        "p99_torque_follow": float(np.percentile(
            [tq.max() for tq, s in zip(stats["torque"], stats["states"]) if s == "track"], 99)),
        "torque": np.array(stats["torque"]),
        "states": stats["states"],
    }


class Recorder:
    """Video: large external view + what the arm camera sees in a corner."""

    LABEL = {"search": "searching...", "greet": "hello!", "track": "tracking"}

    def __init__(self, m):
        from PIL import ImageFont

        self.r = mujoco.Renderer(m, 480, 640)
        self.frames = []
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)

    def frame(self, d, cam_img, state, hit, t):
        from PIL import Image, ImageDraw

        self.r.update_scene(d, camera="side")
        big = Image.fromarray(self.r.render())
        small = Image.fromarray(cam_img).resize((224, 168))
        dr = ImageDraw.Draw(small)
        if hit:
            u, v, size = hit
            cx, cy = (u + 1) / 2 * 224, (v + 1) / 2 * 168
            rad = max(8, 120 * np.sqrt(size))
            dr.rectangle([cx - rad, cy - rad, cx + rad, cy + rad], outline=(60, 230, 90), width=3)
        big.paste(small, (8, 8))
        dr = ImageDraw.Draw(big)
        dr.rectangle([8, 8, 232, 176], outline=(255, 255, 255), width=2)
        dr.text((12, 180), "arm camera", fill=(255, 255, 255), font=self.font)
        dr.text((470, 10), self.LABEL[state], fill=(255, 230, 90), font=self.font)
        dr.text((470, 36), f"{t:4.1f} s", fill=(230, 230, 230), font=self.font)
        self.frames.append(np.array(big))

    def save(self, path):
        import imageio.v2 as iio

        iio.mimsave(path, self.frames, fps=CONTROL_HZ, macro_block_size=1)
        print("Video saved to", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video")
    ap.add_argument("--seconds", type=float, default=23.0)
    ap.add_argument("--volts", default="12V", choices=list(scene.STALL_TORQUE))
    args = ap.parse_args()
    out = run(args.seconds, args.video, args.volts)
    states = out.pop("states")
    out.pop("torque")
    print(f"Face visible: {out['visible_pct']:.0f} % of the time (the person leaves the view on purpose)")
    print(f"Gaze error while tracking: median {out['track_err_median_deg']:.1f}°, p90 {out['track_err_p90_deg']:.1f}°")
    print(f"Torque: max {out['max_torque']:.2f} N·m, while tracking (p99) {out['p99_torque_follow']:.2f} N·m "
          f"(limit {scene.STALL_TORQUE[args.volts]} N·m)")
    print("Greetings:", sum(1 for i, s in enumerate(states) if s == "greet" and (i == 0 or states[i - 1] != "greet")))


if __name__ == "__main__":
    main()
