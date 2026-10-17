"""Smooth, checked motion for the RoArm-M3, built on T:1041 / T:102 streaming.

Why not the firmware's own moves: T:104 starts its interpolation from the
measured (gravity-sagged) position, so the target drops when a move starts,
and T:1041 silently clamps joints at their limits. Here we plan the path,
check it against the limits, and stream small steps on a minimum-jerk curve.

The servos only run proportional control, so the arm settles a few mm short
of where it's sent. Correcting that afterwards (re-aiming from where it
stopped) fails for the joints that hold the arm up: reversing against
gravity + friction doesn't unstick them. What works is approaching from the
same direction every time, which lands within ~0.2 mm, and aiming ahead by
the learned offset (LandingOffset).
"""
import math
import time

import numpy as np

from . import kinematics as K

RATE_USB = 50     # Hz, how often a new target is sent over USB
RATE_WIFI = 20    # Hz over WiFi: each HTTP command takes ~20-45 ms, sometimes 150


def rate_for(arm):
    return RATE_WIFI if "." in arm.target else RATE_USB


def min_jerk(u):
    """0 -> 1 as u goes 0 -> 1, with zero speed and acceleration at both ends."""
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def stream(send, a, b, duration, rate):
    """Call send(point) along a min-jerk blend from a to b. Each point comes from
    the clock, so a slow command makes the next step catch up instead of lagging."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    t0 = time.perf_counter()
    while True:
        tick = time.perf_counter()
        u = min(1.0, (tick - t0) / duration)
        send(a + (b - a) * min_jerk(u))
        if u >= 1.0:
            return
        time.sleep(max(0.0, 1 / rate - (time.perf_counter() - tick)))


def check_line(a, b, pitch, margin=0.05, samples=21):
    """Joint-limit problems anywhere on the straight line from a to b (mm)."""
    problems = []
    for s in np.linspace(0, 1, samples):
        p = np.asarray(a, float) + (np.asarray(b, float) - a) * s
        problems += [f"at x={p[0]:.0f} y={p[1]:.0f} z={p[2]:.0f}: {i}" for i in K.limit_problems(*p, pitch, margin)]
    return problems


def go_to_pose(arm, xyz, pitch, roll, grip, rate, joint_speed=0.3):
    """Move to a tip pose by blending joint angles, which can't pass a limit.

    Refuses if the blend would swing the tip more than 10 mm below both the
    start and the goal.
    """
    j = K.ik(*xyz, pitch)
    if j is None or K.limit_problems(*xyz, pitch):
        raise ValueError(f"pose {xyz} at pitch {pitch:.2f} is out of reach or at a joint limit")
    q0 = arm.state().astype(float)
    q1 = np.array([j["base"], j["shoulder"], j["elbow"], j["wrist"], roll, grip])
    lowest = min(K.fk(*(q0 + (q1 - q0) * min_jerk(u))[:4])[2] for u in np.linspace(0, 1, 51))
    if lowest < min(K.fk(*q0[:4])[2], xyz[2]) - 10:
        raise ValueError(f"getting there from here would swing the tip down to z={lowest:.0f} mm; "
                         "move the arm up first (e.g. {\"T\":100})")
    duration = max(2.0, np.max(np.abs(q1 - q0)) / joint_speed)
    stream(lambda q: arm.move(q, spd=0, acc=0), q0, q1, duration, rate)


def move_tcp(arm, a, b, pitch, roll, grip, rate, speed=15.0, end_pitch=None):
    """Smooth straight-line tip move from a to b (mm) at `speed` mm/s on average.

    The gripper keeps `pitch`, or blends to `end_pitch` if given.
    """
    end_pitch = pitch if end_pitch is None else end_pitch
    duration = max(1.0, np.linalg.norm(np.asarray(b, float) - a) / speed)
    stream(lambda v: arm.move_tcp(*v[:3], v[3], roll, grip),
           np.append(a, pitch), np.append(b, end_pitch), duration, rate)


def joints_for(xyz, pitch, roll, grip, gravity=None):
    """Joint command for a tip pose, with gravity feedforward if a model is given."""
    j = K.ik(*xyz, pitch)
    if j is None:
        raise ValueError(f"{np.round(xyz, 1)} is out of reach")
    q = np.array([j["base"], j["shoulder"], j["elbow"], j["wrist"], roll, grip])
    if gravity is not None:
        q[1:4] += gravity.predict(q[1:4])
    return q


def move_tcp_joints(arm, a, b, pitch, roll, grip, rate, speed=15.0, gravity=None):
    """Like move_tcp, but solves the joints here and streams them (T:102), so a
    gravity model can be added. Checks the line against the joint limits first."""
    problems = check_line(a, b, pitch)
    if problems:
        raise ValueError(problems[0])
    duration = max(1.0, np.linalg.norm(np.asarray(b, float) - a) / speed)
    stream(lambda p: arm.move(joints_for(p, pitch, roll, grip, gravity), spd=0, acc=0), a, b, duration, rate)


def measured(arm):
    fb = arm.feedback()
    return np.array([fb["x"], fb["y"], fb["z"]]), fb["tit"]


class GravityModel:
    """Predicts how far the shoulder, elbow and wrist sag below their targets (rad).

    Gravity's torque on a joint is proportional to the horizontal lever arm of the
    weight beyond it, so each joint's sag is fitted as a linear function of the
    horizontal distances from that joint to the elbow / wrist / tip. Add predict()
    to the joint commands (feedforward) to cancel the sag. Fitted from poses
    reached from above, so use it for poses reached from above.
    """

    def __init__(self, coef, residual=None):
        self.coef = [np.asarray(c, float) for c in coef]
        self.residual = residual or [0.0, 0.0, 0.0]

    @staticmethod
    def features(sew):
        elbow, wrist, tip = K.points(*sew)
        r = lambda p: p[0] / 100.0  # horizontal distance from the shoulder, per 100 mm
        return [np.array([1.0, r(elbow), r(wrist)]),
                np.array([1.0, r(wrist) - r(elbow)]),
                np.array([1.0, r(tip) - r(wrist)])]

    def predict(self, sew):
        """Sag (rad) for shoulder, elbow, wrist at commanded angles `sew`."""
        return np.array([c @ f for c, f in zip(self.coef, self.features(sew))])

    @classmethod
    def fit(cls, commanded, sag, ridge=1e-3):
        """Least squares (lightly regularised) from commanded angles -> measured sag."""
        feats = [cls.features(q) for q in commanded]
        coef, residual = [], []
        for j in range(3):
            A, y = np.array([f[j] for f in feats]), np.asarray(sag)[:, j]
            c = np.linalg.solve(A.T @ A + ridge * np.eye(A.shape[1]), A.T @ y)
            coef.append(c)
            residual.append(float(np.sqrt(np.mean((A @ c - y) ** 2))))
        return cls(coef, residual)

    def to_dict(self):
        return {"coef": [c.tolist() for c in self.coef], "residual_rad": self.residual}

    @classmethod
    def from_dict(cls, d):
        return cls(d["coef"], d.get("residual_rad"))


COUNT = 2 * math.pi / 4096  # one encoder step (rad)


class Compensation:
    """Gravity (depends on pose) + Coulomb friction (depends on direction of travel).

    For each of base, shoulder, elbow, wrist, the shortfall (commanded minus
    measured, rad) is modelled as
        gravity lever-arm terms  +  direction * friction
    where direction is +1 / -1 for the way the joint last moved. Friction makes a
    joint stop short in its direction of travel; adding the predicted shortfall to
    the command makes it land on target whichever way it arrives.
    """

    NAMES = ("base", "shoulder", "elbow", "wrist")

    def __init__(self, coef, residual=None, load_friction=False):
        self.coef = {n: np.asarray(coef[n], float) for n in self.NAMES}
        self.residual = residual or {n: 0.0 for n in self.NAMES}
        self.load_friction = load_friction

    @staticmethod
    def features(q4, s4, load_friction=False):
        """With load_friction, friction also scales with the joint's gravity lever
        arm (gear friction grows with load)."""
        _, s, e, w = q4
        elbow, wrist, tip = K.points(s, e, w)
        r = lambda p: p[0] / 100.0
        re, rw, rt = r(elbow), r(wrist), r(tip)
        f = {"base": [1.0, s4[0]],
             "shoulder": [1.0, re, rw, s4[1]],
             "elbow": [1.0, rw - re, s4[2]],
             "wrist": [1.0, rt - rw, s4[3]]}
        if load_friction:
            f["shoulder"] += [s4[1] * re, s4[1] * rw]
            f["elbow"] += [s4[2] * (rw - re)]
            f["wrist"] += [s4[3] * (rt - rw)]
        return {n: np.array(v) for n, v in f.items()}

    def predict(self, q4, s4):
        f = self.features(q4, s4, self.load_friction)
        return np.array([self.coef[n] @ f[n] for n in self.NAMES])

    def friction_counts(self):
        """Friction term (counts) at zero lever arm; with load_friction it also grows with load."""
        idx = {"base": 1, "shoulder": 3, "elbow": 2, "wrist": 2}
        return {n: round(float(self.coef[n][idx[n]] / COUNT), 2) for n in self.NAMES}

    @classmethod
    def fit(cls, commanded, directions, shortfall, ridge=1e-3, load_friction=False):
        feats = [cls.features(q, s, load_friction) for q, s in zip(commanded, directions)]
        coef, residual = {}, {}
        for j, n in enumerate(cls.NAMES):
            A, y = np.array([f[n] for f in feats]), np.asarray(shortfall)[:, j]
            c = np.linalg.solve(A.T @ A + ridge * np.eye(A.shape[1]), A.T @ y)
            coef[n], residual[n] = c, float(np.sqrt(np.mean((A @ c - y) ** 2)))
        return cls(coef, residual, load_friction)

    def to_dict(self):
        return {"coef": {n: c.tolist() for n, c in self.coef.items()}, "residual_rad": self.residual,
                "load_friction": self.load_friction}

    @classmethod
    def from_dict(cls, d):
        return cls(d["coef"], d.get("residual_rad"), d.get("load_friction", False))


class Mover:
    """Straight-line tip moves with gravity + friction feedforward.

    Remembers which way each joint last moved (that picks the sign of its
    friction term). When a joint reverses, the term is blended over the first
    20 % of the move instead of jumping.
    """

    def __init__(self, arm, comp, pitch, roll, grip, rate):
        self.arm, self.comp, self.pitch, self.roll, self.grip, self.rate = arm, comp, pitch, roll, grip, rate
        self.dir = np.zeros(4)

    def command(self, p, s4):
        q = joints_for(p, self.pitch, self.roll, self.grip)
        if self.comp is not None:
            q[:4] += self.comp.predict(q[:4], s4)
        return q

    def move(self, a, b, speed=15.0, log=None):
        """log: optional list; each streamed step appends (time, x, y, z) of the target."""
        problems = check_line(a, b, self.pitch)
        if problems:
            raise ValueError(problems[0])
        a, b = np.asarray(a, float), np.asarray(b, float)
        d = joints_for(b, self.pitch, self.roll, self.grip)[:4] - joints_for(a, self.pitch, self.roll, self.grip)[:4]
        old, new = self.dir.copy(), np.where(np.abs(d) > 0.5 * COUNT, np.sign(d), self.dir)
        duration = max(1.0, np.linalg.norm(b - a) / speed)
        t0 = time.perf_counter()
        while True:
            tick = time.perf_counter()
            u = min(1.0, (tick - t0) / duration)
            s4 = old + (new - old) * min(1.0, u / 0.2)
            p = a + (b - a) * min_jerk(u)
            self.arm.move(self.command(p, s4), spd=0, acc=0)
            if log is not None:
                log.append((tick, *p))
            if u >= 1.0:
                break
            time.sleep(max(0.0, 1 / self.rate - (time.perf_counter() - tick)))
        self.dir = new


class LandingOffset:
    """Learns how far the arm lands from where it's sent, so you can aim ahead.

    Only valid for the same approach direction, the same area of the workspace
    and the same load: the offset comes from gravity and friction, which change
    with all three.
    """

    def __init__(self, max_mm=10.0, max_deg=5.0):
        self.errors, self.max_mm, self.max_rad = [], max_mm, math.radians(max_deg)

    def add(self, aim, aim_pitch, pos, tit):
        """Record one landing: what was sent vs what the arm measured."""
        self.errors.append(np.append(np.asarray(aim, float) - pos, aim_pitch - tit))

    def aim(self, target, pitch):
        """Where to send the arm so it lands on `target` at `pitch`."""
        if not self.errors:
            return np.asarray(target, float), pitch
        e = np.mean(self.errors, axis=0)
        return (np.asarray(target, float) + np.clip(e[:3], -self.max_mm, self.max_mm),
                pitch + float(np.clip(e[3], -self.max_rad, self.max_rad)))
