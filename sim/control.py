"""Arm control: inverse kinematics, camera tracking and face detector.

Everything the tracking uses, the real robot would have too: the wrist camera image and the servo
angles. It does not use the "true" position of the person.
"""

import mujoco
import numpy as np

from scene import SKIN, arm_ids

CONTROL_HZ = 25
MAX_SPEED = 2.0  # rad/s per joint (the STS3215 reaches ~4.7 unloaded)
MAX_ACCEL = 10.0  # rad/s² (smooth starts and stops: fewer torque spikes)
REST = np.array([0.0, -0.9, 0.9, -0.7, -1.57])  # comfortable posture (camera upright, looking forward)


class Arm:
    """Keeps the servo targets and moves them smoothly (at limited speed)."""

    def __init__(self, model, data):
        self.m, self.d = model, data
        self.qadr, self.dadr, self.act, self.gq, self.gact = arm_ids(model)
        self.lo, self.hi = model.jnt_range[[model.jnt_qposadr.tolist().index(a) for a in self.qadr]].T
        self.q_cmd = data.qpos[self.qadr].copy()
        self.dq = np.zeros(len(self.qadr))
        self.grip_cmd = float(data.qpos[self.gq])
        self.substeps = int(round(1 / (CONTROL_HZ * model.opt.timestep)))
        self.site = model.site("gripperframe").id
        self.gripper_body = model.body("gripper").id
        self.cam = model.camera("wrist_cam").id

    def set_pose(self, q):
        self.d.qpos[self.qadr] = q
        self.q_cmd = np.array(q, float)
        self.dq = np.zeros(len(self.qadr))
        self.d.ctrl[self.act] = q
        mujoco.mj_forward(self.m, self.d)

    def move(self, dq):
        """Apply a velocity (rad/s) for one control step."""
        dq = np.clip(dq, -MAX_SPEED, MAX_SPEED)
        dv = MAX_ACCEL / CONTROL_HZ
        dq = np.clip(dq, self.dq - dv, self.dq + dv)
        self.dq = dq
        self.q_cmd = np.clip(self.q_cmd + dq / CONTROL_HZ, self.lo + 0.02, self.hi - 0.02)

    def step(self, on_substep=None):
        # Interpolate the target within the control step (like the servo's velocity profile)
        start = self.d.ctrl[self.act].copy()
        self.d.ctrl[self.gact] = self.grip_cmd
        for i in range(self.substeps):
            self.d.ctrl[self.act] = start + (self.q_cmd - start) * (i + 1) / self.substeps
            mujoco.mj_step(self.m, self.d)
            if on_substep:
                on_substep()

    # ------------------------------------------------------------- kinematics
    def jac(self, body=None, point=None):
        jp = np.zeros((3, self.m.nv))
        jr = np.zeros((3, self.m.nv))
        if point is None:
            mujoco.mj_jacSite(self.m, self.d, jp, jr, self.site)
        else:
            mujoco.mj_jac(self.m, self.d, jp, jr, point, body)
        return jp[:, self.dadr], jr[:, self.dadr]

    def site_pose(self):
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def cam_frame(self):
        return self.d.cam_xpos[self.cam].copy(), self.d.cam_xmat[self.cam].reshape(3, 3).copy()


def dls(J, err, lam=0.05):
    """Damped least squares: dq such that J dq ≈ err."""
    return J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(J.shape[0]), err)


def nullspace(J, lam=0.05):
    Jp = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(J.shape[0]))
    return np.eye(J.shape[1]) - Jp @ J


def reach_velocity(arm, target, approach=None, gain=4.0, posture_gain=0.5):
    """Joint velocity that brings the gripper to `target` (and its x axis towards `approach`)."""
    p, R = arm.site_pose()
    jp, jr = arm.jac()
    err = gain * (np.asarray(target) - p)
    J = jp
    if approach is not None:
        a = R[:, 0]
        rot_err = np.cross(a, approach / np.linalg.norm(approach))  # turn the x axis towards approach
        err = np.concatenate([err, gain * rot_err])
        J = np.vstack([jp, jr])
    dq = dls(J, err)
    q = arm.d.qpos[arm.qadr]
    dq += nullspace(J) @ (posture_gain * (REST - q))
    return dq, float(np.linalg.norm(np.asarray(target) - p))


# ------------------------------------------------------------------ vision
def detect_face(img):
    """Look for skin in the image (like a simple face detector). Returns (u, v, size) or None.

    u, v in [-1, 1] (right and down positive); size = fraction of the image covered.
    """
    f = img.astype(np.float32) / 255.0
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    tr, tg, tb = SKIN[:3]
    mask = (r > 0.25) & (np.abs(g / (r + 1e-6) - tg / tr) < 0.1) & (np.abs(b / (r + 1e-6) - tb / tr) < 0.12)
    n = int(mask.sum())
    if n < 12:
        return None
    ys, xs = np.nonzero(mask)
    H, W = mask.shape
    return (xs.mean() / W * 2 - 1, ys.mean() / H * 2 - 1, n / mask.size)


def ray_world(arm, u, v, fovy_deg=61.0, aspect=4 / 3):
    """Direction (world frame) of the point seen at (u, v) in the image."""
    _, Rc = arm.cam_frame()
    th = np.tan(np.radians(fovy_deg) / 2)
    r = np.array([u * th * aspect, -v * th, -1.0])
    return Rc @ (r / np.linalg.norm(r))


class TargetMotion:
    """Estimate the angular velocity of the seen target (rad/s, world) from consecutive detections.

    Used to lead the target when it (or oneself) moves: without this, tracking always lags a
    little behind.
    """

    def __init__(self, dt, smooth=0.3):
        self.dt, self.smooth = dt, smooth
        self.prev, self.omega = None, np.zeros(3)

    def update(self, ray):
        if ray is None:
            self.prev, self.omega = None, np.zeros(3)
            return self.omega
        if self.prev is not None:
            w = np.cross(self.prev, ray) / self.dt
            self.omega = (1 - self.smooth) * self.omega + self.smooth * w
        self.prev = ray
        return self.omega


def look_velocity(arm, u, v, gain=2.5, posture_gain=0.6, fovy_deg=61.0, aspect=4 / 3, base_omega=None,
                  target_omega=None):
    """Joint velocity that centres a point seen at (u, v) in the image, with the camera upright.

    Rotates the camera about its own axes (image-based visual servoing) and keeps the horizon
    level (no roll). The leftover posture returns to REST. If the base moves (arm on the
    backpack), base_omega = angular velocity of the base measured by the IMU (rad/s, world frame).
    """
    _, Rc = arm.cam_frame()  # columns: x right, y up, z backwards (MuJoCo)
    half_v = np.radians(fovy_deg) / 2
    half_h = np.arctan(np.tan(half_v) * aspect)
    # Angles to the target in the camera frame
    yaw_err = -np.arctan(u * np.tan(half_h))  # target to the right -> turn right (-y)
    pitch_err = -np.arctan(v * np.tan(half_v))  # target below -> look down (-x)
    roll_err = -Rc[2, 0]  # camera x axis should stay horizontal
    w_cam = gain * np.array([pitch_err, yaw_err, roll_err])
    w = Rc @ w_cam
    if target_omega is not None:
        w = w + target_omega  # lead the target's motion
    if base_omega is not None:
        # The base rotates (IMU): the servos cancel that rotation so the camera does not shake
        w = w - base_omega
    _, jr = arm.jac()
    dq = dls(jr, w)
    q = arm.d.qpos[arm.qadr]
    dq += nullspace(jr) @ (posture_gain * (REST - q))
    return dq
