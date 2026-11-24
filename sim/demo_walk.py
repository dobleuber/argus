"""Demo 3: the arm on the backpack, over the right shoulder, while the person walks.

Usage:
  python demo_walk.py                     # look: tracks the people on the sidewalk (with and without IMU)
  python demo_walk.py --video walk.mp4    # with video
  python demo_walk.py --hold              # hold a box while walking (masses and voltages)
  python demo_walk.py --hold --mass 0.2 --video hold_walk.mp4

Safety (always on): the base does not turn towards the head (PAN_MIN) and, if any part of the arm
comes within SAFE_DIST of the head, the arm retreats to the rest posture.
"""

import argparse

import mujoco
import numpy as np

import backpack as bp
import scene
from control import CONTROL_HZ, REST, Arm, TargetMotion, detect_face, look_velocity, ray_world, reach_velocity

DET_W, DET_H = 160, 120
PAN_MIN, PAN_MAX = -0.15, 1.9  # rad: negative = towards the head (the arm is on the right shoulder)
SAFE_DIST = 0.10  # m, minimum distance between the arm and the head


class Safety:
    """Distance from the arm to the wearer's head (using the arm's collision shapes)."""

    def __init__(self, m, d, wearer):
        self.m, self.d, self.wearer = m, d, wearer
        arm_bodies = {m.body(n).id for n in ("upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1")}
        self.geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in arm_bodies and m.geom_group[g] == 3
                      and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH]
        self.head = m.geom("wearer_head").id
        self.events = 0

    def head_distance(self):
        """Minimum distance (m) between the arm's collision shapes and the head."""
        ft = np.zeros(6)
        return min(mujoco.mj_geomDistance(self.m, self.d, g, self.head, 1.0, ft) for g in self.geoms)

    def filter(self, arm, dq):
        """Return the safe velocity: if it is too close, retreat to REST."""
        if self.head_distance() < SAFE_DIST:
            self.events += 1
            return 2.0 * (REST - self.d.qpos[arm.qadr])
        return dq


def _walker(m, d, wearer, t0=0.0, fold_torque=bp.FOLD_TORQUE):
    """Move the wearer at every physics step (otherwise the mount gets jerked) and apply the
    hinge magnets."""

    def f():
        wearer.update(d.time - t0, m.opt.timestep)
        bp.magnets(m, d, fold_torque)

    return f


def base_omega(m, d):
    """Angular velocity of the base (mount IMU), in the world frame."""
    s = m.site("mount_imu").id
    return d.site_xmat[s].reshape(3, 3) @ d.sensor("mount_gyro").data


def mount_loads(m, d):
    """Force (N) and moment (N·m) that the mount exerts on the arm base."""
    mujoco.mj_rnePostConstraint(m, d)
    b = m.body("base").id
    tau_c, f = d.cfrc_int[b][:3], d.cfrc_int[b][3:]
    c = d.subtree_com[m.body_rootid[b]]
    tau_p = tau_c + np.cross(c - d.xpos[b], f)
    return float(np.linalg.norm(f)), float(np.linalg.norm(tau_p))


def run_look(imu=True, video=None, volts="12V", seconds=19.0, predict=True, fit="tight"):
    m = bp.build(volts, fit)
    d = mujoco.MjData(m)
    arm = Arm(m, d)
    arm.set_pose(REST)
    arm.grip_cmd = 0.0
    wearer = bp.Wearer(m, d)
    safety = Safety(m, d, wearer)
    det = mujoco.Renderer(m, DET_H, DET_W)
    faces = [m.geom(f"{n}_face").id for n, _, _ in bp.BYSTANDERS]
    rec = Recorder(m, "Looking while walking" + (" (with IMU)" if imu else " (no IMU)")) if video else None

    walk = _walker(m, d, wearer)
    walk()
    mujoco.mj_forward(m, d)
    state, lost_for, nod_until = "search", 0.0, -1
    motion = TargetMotion(1 / CONTROL_HZ)
    st = dict(err=[], err_t=[], torque=[], head=[], force=[], moment=[], states=[], fold=[], shake=[])
    for k in range(int(seconds * CONTROL_HZ)):
        t = k / CONTROL_HZ
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
        w_base = base_omega(m, d) if imu else None
        w_target = motion.update(ray_world(arm, *hit[:2]) if hit else None) if predict else None
        if state == "search":
            target = REST.copy()
            target[0] = 0.75 + 0.9 * np.sin(2 * np.pi * t / 6)  # sweeps forward and to the right
            dq = 2.0 * (target - d.qpos[arm.qadr])
        else:
            dq = look_velocity(arm, *(hit[:2] if hit else (0.0, 0.0)), base_omega=w_base, target_omega=w_target)
            if state == "greet":
                dq[3] += 1.5 * np.cos(2 * np.pi * 1.6 * t)
                if t > nod_until:
                    state = "track"
        dq = safety.filter(arm, dq)
        arm.move(dq)
        arm.q_cmd[0] = np.clip(arm.q_cmd[0], PAN_MIN, PAN_MAX)
        arm.step(walk)

        cpos, Rc = arm.cam_frame()
        if state == "track" and hit:
            # Error against the face closest to the gaze direction
            view = -Rc[:, 2]
            angs = [np.degrees(np.arccos(np.clip(view @ (d.geom_xpos[f] - cpos) / np.linalg.norm(d.geom_xpos[f] - cpos), -1, 1)))
                    for f in faces]
            st["err"].append(min(angs))
            st["err_t"].append((t, min(angs), int(np.argmin(angs))))
        st["torque"].append(np.abs(d.actuator_force[arm.act]).max())
        st["head"].append(safety.head_distance())
        f, mo = mount_loads(m, d)
        st["force"].append(f)
        st["moment"].append(mo)
        st["states"].append(state)
        st["fold"].append(-d.qpos[m.joint("fold").qposadr[0]])
        st["shake"].append(np.linalg.norm(base_omega(m, d)))
        if rec:
            rec.frame(d, img, state, hit, t, wearer)
    if rec:
        rec.save(video)
    err = np.array(st["err"]) if st["err"] else np.array([np.nan])
    states = st["states"]
    return {
        "err_median": float(np.median(err)), "err_p90": float(np.percentile(err, 90)),
        "greetings": sum(1 for i, s in enumerate(states) if s == "greet" and (i == 0 or states[i - 1] != "greet")),
        "max_torque": float(np.max(st["torque"])), "p99_torque": float(np.percentile(st["torque"], 99)),
        "min_head": float(np.min(st["head"])), "safety_events": safety.events,
        "max_force": float(np.max(st["force"])), "max_moment": float(np.max(st["moment"])),
        "max_fold_deg": float(np.degrees(np.max(st["fold"]))),
        "shake_rms": float(np.sqrt(np.mean(np.square(st["shake"])))),  # rotation of the base (rad/s)
        "err_series": st["err_t"], "states": states,
    }


# Gripper relative to the base: 28 cm forward, 8 to the right and 12 up (at chest and face
# height, as if holding a phone). With 5 joints the gripper has to point where it is
# (it cannot shift sideways while pointing straight ahead).
def _wearer_load(m, d, wearer, ext_body=None, ext_force=None):
    """Force (N) and moment (N·m, about the centre of the shoulders) that the wearer has to exert
    to carry everything hanging from the backpack: mass x acceleration - weight - external forces.
    (It does not include the rotational inertia of each part, which is small.)"""
    mujoco.mj_rnePostConstraint(m, d)
    W = d.mocap_pos[wearer.ids["wearer"]]
    g = m.opt.gravity
    F, M = np.zeros(3), np.zeros(3)
    acc = np.zeros(6)
    root = m.body("mount").id
    for b in range(m.nbody):
        if m.body_rootid[b] != root:
            continue
        mujoco.mj_objectAcceleration(m, d, mujoco.mjtObj.mjOBJ_BODY, b, acc, 0)
        f = m.body_mass[b] * (acc[3:] - g)
        F += f
        M += np.cross(d.xipos[b] - W, f)
    if ext_body is not None:
        F -= ext_force
        M -= np.cross(d.xipos[ext_body] - W, ext_force)
    return float(np.linalg.norm(F)), float(np.linalg.norm(M))


def run_snag(rigid=False, seconds=11.0, video=None, height=1.78):
    """While walking, the arm hits a branch at `height` m on the right side (it does not touch the head).

    rigid=True: mount without a hinge, for comparison.
    """
    m = bp.build(rigid=rigid, branch=height)
    d = mujoco.MjData(m)
    arm = Arm(m, d)
    arm.set_pose(REST)
    wearer = bp.Wearer(m, d)
    safety = Safety(m, d, wearer)
    walk = _walker(m, d, wearer)
    walk()
    mujoco.mj_forward(m, d)
    fold_q = m.joint("fold").qposadr[0]
    branch = m.geom("branch").id
    rec = Recorder(m, f"Branch at {height:.2f} m: " + ("rigid mount" if rigid else "with hinge")) if video else None
    st = dict(fold=[], load=[], moment=[], torque=[], head=[], hit=[], crush=[])
    for k in range(int(seconds * CONTROL_HZ)):
        t = k / CONTROL_HZ
        arm.move(2.0 * (REST - d.qpos[arm.qadr]))
        arm.step(walk)
        f, mo = _wearer_load(m, d, wearer)
        touching, crush = False, 0.0
        cf = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if branch in (c.geom1, c.geom2):
                touching = True
                mujoco.mj_contactForce(m, d, i, cf)
                crush += abs(cf[0])
        st["crush"].append(crush)
        st["fold"].append(-d.qpos[fold_q])
        st["load"].append(f)
        st["moment"].append(mo)
        st["torque"].append(np.abs(d.actuator_force[arm.act]).max())
        st["head"].append(safety.head_distance())
        st["hit"].append(touching)
        if rec:
            rec.frame(d, None, "rest", None, t, wearer)
    if rec:
        rec.save(video)
    hit = np.array(st["hit"])
    idx = np.nonzero(hit)[0]
    win = slice(max(0, idx[0] - 5), idx[-1] + 15) if len(idx) else slice(0, 0)
    walking = slice(25, idx[0] - 10) if len(idx) else slice(25, None)
    return {
        "contact_s": float(hit.sum() / CONTROL_HZ),
        "max_crush": float(np.max(st["crush"])),  # N: force of the branch against the arm
        "max_fold_deg": float(np.degrees(np.max(st["fold"]))),
        "fold_after_deg": float(np.degrees(st["fold"][-1])),
        "peak_moment": float(np.max(st["moment"][win])) if len(idx) else 0.0,
        "walking_moment": float(np.max(st["moment"][walking])),
        "peak_force": float(np.max(st["load"][win])) if len(idx) else 0.0,
        "walking_force": float(np.max(st["load"][walking])),
        "peak_servo_torque": float(np.max(st["torque"][win])) if len(idx) else 0.0,
        "min_head": float(np.min(st["head"])),
    }


HOLD_TARGET = np.array([0.28, -0.08, 0.12])
HOLD_APPROACH = np.array([HOLD_TARGET[0], HOLD_TARGET[1], 0.0]) / np.linalg.norm(HOLD_TARGET[:2])
HOLD_SEED = np.array([0.2, -0.8, 0.66, 0.14, -1.56])  # starting point of the inverse kinematics


# Grasped box: position and orientation in the gripper frame, and jaw angle
GRASP_REL = np.array([0.00314, 0.01202, 0.00375])
GRASP_QUAT = np.array([0.49974, 0.49891, 0.50479, 0.49652])
GRASP_JAW = 0.311


def _quat_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q / np.linalg.norm(q))
    return R.reshape(3, 3)


def _to_world(m, d, p_local, v_local=None):
    b = m.body("base").id
    R = d.xmat[b].reshape(3, 3)
    return d.xpos[b] + R @ p_local, (None if v_local is None else R @ v_local)


def _hold_pose(m, d, arm):
    """Holding posture (inverse kinematics without physics, with the wearer standing still)."""
    q = HOLD_SEED.copy()
    for _ in range(400):
        arm.set_pose(q)
        tgt, app = _to_world(m, d, HOLD_TARGET, HOLD_APPROACH)
        dq, err = reach_velocity(arm, tgt, app)
        q = np.clip(q + dq * 0.02, arm.lo + 0.02, arm.hi - 0.02)
    return q, err


def run_hold(mass=0.2, volts="12V", video=None, seconds=17.0):
    m = _add_box(volts, mass)
    d = mujoco.MjData(m)
    arm = Arm(m, d)
    wearer = bp.Wearer(m, d)
    q_hold, ik_err = _hold_pose(m, d, arm)
    arm.set_pose(q_hold)
    # Start with the box already grasped, in the position where it ends up after closing the gripper
    # (measured in a grasp simulation with 200 g). Closing the gripper in simulation on light boxes
    # is fragile: the squeeze pops them out.
    box = m.body("box").id
    jq = m.jnt_qposadr[m.body_jntadr[box]]
    p, R = arm.site_pose()
    Rb = R @ _quat_mat(GRASP_QUAT)
    qb = np.zeros(4)
    mujoco.mju_mat2Quat(qb, Rb.flatten())
    d.qpos[jq : jq + 3] = p + R @ GRASP_REL
    d.qpos[jq + 3 : jq + 7] = qb
    d.qpos[arm.gq] = GRASP_JAW
    arm.grip_cmd = -0.17
    mujoco.mj_forward(m, d)
    for _ in range(int(0.5 * CONTROL_HZ)):  # settle in place before walking
        arm.step()
    t0 = d.time
    walk = _walker(m, d, wearer, t0)
    rec = Recorder(m, f"Holding {mass * 1000:.0f} g while walking ({volts})") if video else None
    st = dict(torque=[], force=[], moment=[], acc=[])
    prev_v = None
    R0 = arm.site_pose()[1]
    rel0 = R0.T @ (d.xpos[box] - arm.site_pose()[0])  # position of the box in the gripper at the start
    for k in range(int(seconds * CONTROL_HZ)):
        t = k / CONTROL_HZ
        arm.move(2.0 * (q_hold - arm.q_cmd))
        arm.step(walk)
        st["torque"].append(np.abs(d.actuator_force[arm.act]))
        f, mo = mount_loads(m, d)
        st["force"].append(f)
        st["moment"].append(mo)
        v = d.qvel[m.jnt_dofadr[m.body_jntadr[box]] : m.jnt_dofadr[m.body_jntadr[box]] + 3].copy()
        if prev_v is not None:
            st["acc"].append(np.linalg.norm(v - prev_v) * CONTROL_HZ)
        prev_v = v
        if rec:
            rec.frame(d, None, "hold", None, t, wearer)
    if rec:
        rec.save(video)
    T = np.array(st["torque"])
    p, R = arm.site_pose()
    slip = float(np.linalg.norm(R.T @ (d.xpos[box] - p) - rel0))  # how far it slipped in the gripper
    held = slip < 0.02 and d.xpos[box][2] > 1.0
    return {
        "mass_g": mass * 1000, "volts": volts, "held": bool(held), "slip_mm": slip * 1000, "ik_err_mm": ik_err * 1000,
        "p99_torque": float(np.percentile(T.max(1), 99)), "max_torque": float(T.max()),
        "max_force": float(np.max(st["force"])), "max_moment": float(np.max(st["moment"])),
        "p99_box_acc": float(np.percentile(st["acc"], 99)),
    }


def _add_box(volts, mass):
    """The backpack scene with a 4 x 4 x 6 cm box to hold."""
    spec = bp.build_spec(volts)
    b = spec.worldbody.add_body()
    b.name, b.pos = "box", [0, 0, 3]
    b.add_freejoint()
    scene._geom(b, "box", scene.B, (0.02, 0.02, 0.03), rgba=(0.1, 0.75, 0.2, 1), mass=mass, condim=4,
                friction=[1.0, 0.03, 0.003], solref=[0.01, 1])
    return spec.compile()


class Recorder:
    LABEL = {"search": "searching...", "greet": "hello!", "track": "tracking", "hold": "holding",
             "rest": "at rest"}

    def __init__(self, m, title):
        from PIL import ImageFont

        self.r = mujoco.Renderer(m, 480, 640)
        self.cam_r = mujoco.Renderer(m, 168, 224)
        self.frames, self.title = [], title
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)

    def frame(self, d, cam_img, state, hit, t, wearer):
        from PIL import Image, ImageDraw

        self.r.update_scene(d, camera="follow")
        big = Image.fromarray(self.r.render())
        if cam_img is None:
            self.cam_r.update_scene(d, camera="wrist_cam")
            cam_img = self.cam_r.render()
        small = Image.fromarray(cam_img).resize((224, 168))
        if hit:
            dr = ImageDraw.Draw(small)
            u, v, size = hit
            cx, cy = (u + 1) / 2 * 224, (v + 1) / 2 * 168
            rad = max(6, 120 * np.sqrt(size))
            dr.rectangle([cx - rad, cy - rad, cx + rad, cy + rad], outline=(60, 230, 90), width=3)
        big.paste(small, (8, 8))
        dr = ImageDraw.Draw(big)
        dr.rectangle([8, 8, 232, 176], outline=(255, 255, 255), width=2)
        dr.text((12, 180), "arm camera", fill=(255, 255, 255), font=self.font)
        dr.text((250, 10), self.title, fill=(255, 255, 255), font=self.font)
        speed = wearer.walk.speed(t)
        dr.text((250, 36), f"{self.LABEL[state]}   {t:4.1f} s   walking {speed * 3.6:.1f} km/h"
                if speed > 0.05 else f"{self.LABEL[state]}   {t:4.1f} s   standing", fill=(255, 230, 90), font=self.font)
        self.frames.append(np.array(big))

    def save(self, path):
        import imageio.v2 as iio

        iio.mimsave(path, self.frames, fps=CONTROL_HZ, macro_block_size=1)
        print("Video saved to", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", action="store_true", help="hold a box while walking")
    ap.add_argument("--mass", type=float, help="kg (with --hold)")
    ap.add_argument("--volts", default="12V", choices=list(scene.STALL_TORQUE))
    ap.add_argument("--video")
    args = ap.parse_args()
    if args.hold:
        cases = ([(args.mass, args.volts)] if args.mass else
                 [(mm, v) for v in ("7.4V", "12V") for mm in (0.1, 0.2, 0.3)])
        for mass, volts in cases:
            o = run_hold(mass, volts, args.video if args.mass else None)
            print(f"{o['volts']:5s} {o['mass_g']:4.0f} g  held={'yes' if o['held'] else 'NO'} (slipped {o['slip_mm']:.0f} mm)  "
                  f"torque p99 {o['p99_torque']:.2f} / max {o['max_torque']:.2f} N·m (limit {scene.STALL_TORQUE[volts]})  "
                  f"mount: {o['max_force']:.0f} N, {o['max_moment']:.2f} N·m  box shaking p99 {o['p99_box_acc']:.1f} m/s²")
        return
    for imu in (True, False):
        o = run_look(imu, args.video if imu else None, args.volts)
        o.pop("err_series"), o.pop("states")
        print(f"{'With IMU' if imu else 'No IMU'}: gaze error median {o['err_median']:.1f}°, p90 {o['err_p90']:.1f}° | "
              f"greetings {o['greetings']} | max torque {o['max_torque']:.2f} N·m | "
              f"min distance to the head {o['min_head'] * 100:.0f} cm (retreats: {o['safety_events']}) | "
              f"mount: {o['max_force']:.0f} N, {o['max_moment']:.2f} N·m")


if __name__ == "__main__":
    main()
