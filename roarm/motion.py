"""Smooth, checked motion for the RoArm-M3, built on T:1041 / T:102 streaming.

Why not the firmware's own moves: T:104 starts its interpolation from the
measured (gravity-sagged) position, so the target drops when a move starts,
and T:1041 silently clamps joints at their limits. Here we plan the path,
check it against the limits, and stream small steps on a minimum-jerk curve.

The servos only run proportional control, so the arm settles a few mm short
of where it's sent. settle() measures that once the arm is still and re-aims
by the difference.
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

    Refuses if the blend would swing the tip more than 10 mm below the goal.
    """
    j = K.ik(*xyz, pitch)
    if j is None or K.limit_problems(*xyz, pitch):
        raise ValueError(f"pose {xyz} at pitch {pitch:.2f} is out of reach or at a joint limit")
    q0 = arm.state().astype(float)
    q1 = np.array([j["base"], j["shoulder"], j["elbow"], j["wrist"], roll, grip])
    lowest = min(K.fk(*(q0 + (q1 - q0) * min_jerk(u))[:4])[2] for u in np.linspace(0, 1, 51))
    if lowest < xyz[2] - 10:
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


def measured(arm):
    fb = arm.feedback()
    return np.array([fb["x"], fb["y"], fb["z"]]), fb["tit"]


def settle(arm, target, pitch, aim, aim_pitch, roll, grip, rate,
           rounds=2, wait=0.4, max_fix_mm=10.0, max_fix_deg=5.0):
    """Re-aim until the measured tip matches `target` (or `rounds` run out).

    `aim`/`aim_pitch` are what was last sent. Each round: wait for the arm to
    go still, measure, and shift the aim by the remaining error. Corrections
    are capped and limit-checked. Returns the new aim, aim pitch, and the
    measured position and pitch after the last round.
    """
    target, aim = np.asarray(target, float), np.asarray(aim, float)
    for _ in range(rounds):
        time.sleep(wait)
        pos, tit = measured(arm)
        err, err_p = target - pos, pitch - tit
        if np.linalg.norm(err) < 0.3 and abs(err_p) < math.radians(0.2):
            return aim, aim_pitch, pos, tit
        new_aim = target + np.clip(aim + err - target, -max_fix_mm, max_fix_mm)
        new_pitch = pitch + float(np.clip(aim_pitch + err_p - pitch, -math.radians(max_fix_deg), math.radians(max_fix_deg)))
        if K.limit_problems(*new_aim, new_pitch, margin=0.02):
            break
        stream(lambda v: arm.move_tcp(*v[:3], v[3], roll, grip),
               np.append(aim, aim_pitch), np.append(new_aim, new_pitch), 0.3, rate)
        aim, aim_pitch = new_aim, new_pitch
    time.sleep(wait)
    pos, tit = measured(arm)
    return aim, aim_pitch, pos, tit
