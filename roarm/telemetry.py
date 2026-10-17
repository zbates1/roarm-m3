"""Parse the roarm-m3 firmware's T:118 telemetry stream.

Each line: @<ms>,<pos b,s,e,w>,<spd b,s,e,w>,<load s,e,w>,<mA s,e> in raw servo
counts (4096 per turn). ~110 lines per second over USB.
"""
import math

import numpy as np

from . import kinematics as K

STEP = 2 * math.pi / 4096


def counts_to_rad(pos):
    """Raw servo counts (base, shoulder, elbow, wrist) -> joint angles, as the firmware does."""
    b, s, e, w = (np.asarray(pos, float)[..., k] * STEP for k in range(4))
    return np.stack([-b + math.pi, s - math.pi, e - math.pi / 2, w - math.pi], axis=-1)


def parse(lines):
    """[(host time, "@..."), ...] -> dict of arrays: host_t, fw_ms, joints (rad), tip (mm),
    pitch (rad), speed (counts/s), load, current (mA)."""
    rows, t_host = [], []
    for t, line in lines:
        try:
            vals = [int(x) for x in line[1:].split(",")]
        except ValueError:
            continue
        if len(vals) == 14:
            rows.append(vals)
            t_host.append(t)
    if not rows:
        return None
    a = np.array(rows)
    joints = counts_to_rad(a[:, 1:5])
    fk = np.array([K.fk(*q) for q in joints])
    return {"host_t": np.array(t_host), "fw_ms": a[:, 0], "joints": joints, "tip": fk[:, :3], "pitch": fk[:, 3],
            "speed": a[:, 5:9], "load": a[:, 9:12], "current_ma": a[:, 12:14]}
