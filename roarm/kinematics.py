"""The RoArm-M3 firmware's own kinematics, copied into Python.

Lets you check a path before sending it: T:1041 silently clamps any joint that
would pass its limit, which shows up as the gripper tilting or the tip drifting.
Source: RoArm-M3_module.h / RoArm-M3_config.h in Waveshare's RoArm-M3_example.
"""
import math

import numpy as np

L2A, L2B = 236.82, 30.00          # upper arm
L3A, L3B = 144.49, 0.0            # forearm
L4A, L4B = 171.67, 13.69          # wrist to gripper tip
L2, T2 = math.hypot(L2A, L2B), math.atan2(L2B, L2A)
L3, T3 = math.hypot(L3A, L3B), math.atan2(L3B, L3A)
LE, TE = math.hypot(L4A, L4B), math.atan2(L4B, L4A)

# Firmware clamps (radians): base, shoulder, elbow, wrist, roll.
LIMITS = {"base": (-math.pi, math.pi), "shoulder": (-math.pi / 2, math.pi / 2),
          "elbow": (0.0, math.pi), "wrist": (-math.pi / 2, math.pi / 2), "roll": (-math.pi, math.pi)}


def ik(x, y, z, pitch):
    """Joint angles T:1041 computes for a tip position (mm) and gripper pitch (rad).

    Returns None if the point is out of reach. Doesn't apply the clamps.
    """
    a = TE + (pitch - math.pi)
    dx, dz = -LE * math.cos(a), -LE * math.sin(a)
    dist = math.hypot(x, y)
    if dist - dx <= 1e-6:
        bx = by = 0.0
    else:
        bx, by = x * (dist - dx) / dist, y * (dist - dx) / dist
    r, base = math.hypot(bx, by), math.atan2(by, bx)
    zz = z + dz
    l2c = r * r + zz * zz
    lc = math.sqrt(l2c)
    try:
        psi = math.acos((L2 * L2 + l2c - L3 * L3) / (2 * L2 * lc)) + T2
        omega = math.acos((L3 * L3 + l2c - L2 * L2) / (2 * lc * L3))
    except ValueError:
        return None
    shoulder = math.pi / 2 - math.atan2(zz, r) - psi
    elbow = psi + omega - T3
    wrist = (math.pi / 2 - shoulder - elbow) + pitch
    return {"base": base, "shoulder": shoulder, "elbow": elbow, "wrist": wrist}


def fk(base, shoulder, elbow, wrist):
    """Tip x, y, z (mm) and pitch (rad) for given joint angles (same as T:105)."""
    ra, za = L2 * math.cos(math.pi / 2 - (shoulder + T2)), L2 * math.sin(math.pi / 2 - (shoulder + T2))
    rb, zb = L3 * math.cos(math.pi / 2 - (elbow + shoulder + T3)), L3 * math.sin(math.pi / 2 - (elbow + shoulder + T3))
    a = math.pi / 2 - (elbow + shoulder + wrist + TE)
    rc, zc = LE * math.cos(a), LE * math.sin(a)
    r = ra + rb + rc
    return r * math.cos(base), r * math.sin(base), za + zb + zc, elbow + shoulder + wrist - math.pi / 2


def points(shoulder, elbow, wrist):
    """(radial, z) of the elbow, wrist and tip in the arm's vertical plane,
    measured from the shoulder pivot (mm). Used for gravity lever arms."""
    elbow_pt = np.array([L2 * math.cos(math.pi / 2 - (shoulder + T2)), L2 * math.sin(math.pi / 2 - (shoulder + T2))])
    a = math.pi / 2 - (elbow + shoulder + T3)
    wrist_pt = elbow_pt + [L3 * math.cos(a), L3 * math.sin(a)]
    b = math.pi / 2 - (elbow + shoulder + wrist + TE)
    tip = wrist_pt + [LE * math.cos(b), LE * math.sin(b)]
    return elbow_pt, wrist_pt, tip


def limit_problems(x, y, z, pitch, margin=0.05):
    """Joints that would be clamped (within `margin` rad of a limit) at this target."""
    joints = ik(x, y, z, pitch)
    if joints is None:
        return ["out of reach"]
    return [f"{name} {q:+.2f} rad (limit {LIMITS[name][0]:+.2f}..{LIMITS[name][1]:+.2f})"
            for name, q in joints.items()
            if not LIMITS[name][0] + margin <= q <= LIMITS[name][1] - margin]
