"""Trace a 20 mm square with the gripper tip, dipping 20 mm down at each corner.

1. Moves to a ready pose with the gripper pointing straight down, where every
   joint has room to spare (it blends joint angles, so it can't hit a limit).
2. At each corner: down 20 mm, pause 2 s, up 20 mm. Corners go left, forward
   (away from the base), right, then back. The gripper keeps its angle.
3. At the bottom of each dip it prints commanded vs measured height and angle,
   so you can see how much the arm sags.

Every point is checked against the firmware's joint limits first; T:1041
silently clamps a joint at its limit, which shows up as the gripper tilting.

Smoothness: the firmware's own smooth move (T:104) starts from the measured,
gravity-sagged position, so the target drops when a move starts. This script
plans the path itself and streams small steps along a minimum-jerk curve.

    uv run python tmp_scripts/square_dips.py --dry-run          # check the path, don't move
    uv run python tmp_scripts/square_dips.py                    # USB (smoothest)
    uv run python tmp_scripts/square_dips.py --arm 192.168.1.203  # WiFi
    Ctrl+C stops it; the arm holds wherever it is.
"""
import argparse
import math
import time

import numpy as np

from roarm import kinematics as K
from roarm.arm import RoArm

READY = np.array([180.0, 0.0, -40.0])  # mm. z=0 is the shoulder pivot; the table is ~126 mm below it
PITCH = math.pi / 2                     # gripper pointing straight down
SIDE = 20.0       # mm, side of the square
DIP = 20.0        # mm, how far down at each corner
PAUSE = 2.0       # s, at the bottom of each dip
SPEED = 15.0      # mm/s average for each move (peaks at ~1.9x on the min-jerk curve)
RATE_USB = 50     # Hz, how often a new target is sent over USB
RATE_WIFI = 20    # Hz over WiFi: each HTTP command takes ~20-45 ms, sometimes 150
LEFT = +1         # +y is the arm's own left (your right, if you face it). -1 flips it.


def min_jerk(u):
    """0 -> 1 as u goes 0 -> 1, with zero speed and acceleration at both ends."""
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def stream(send, a, b, duration, rate):
    """Call send(point) along a min-jerk blend from a to b. Each point comes from
    the clock, so a slow command makes the next step catch up instead of lagging."""
    t0 = time.perf_counter()
    while True:
        tick = time.perf_counter()
        u = min(1.0, (tick - t0) / duration)
        send(a + (b - a) * min_jerk(u))
        if u >= 1.0:
            return
        time.sleep(max(0.0, 1 / rate - (time.perf_counter() - tick)))


def plan():
    """The square around READY: ("move", xyz) and ("pause", seconds) steps."""
    down = np.array([0, 0, -DIP])
    steps = []
    for i, (fwd, left) in enumerate([(0, 0), (0, SIDE), (SIDE, SIDE), (SIDE, 0)]):
        corner = READY + [fwd, LEFT * left, 0]
        if i > 0:
            steps.append(("move", corner))
        steps += [("move", corner + down), ("pause", PAUSE), ("move", corner)]
    steps.append(("move", READY.copy()))
    return steps


def check(steps):
    """Every point along every move, against the joint limits. Returns problems."""
    here, problems = READY, []
    for kind, value in steps:
        if kind == "move":
            for s in np.linspace(0, 1, 21):
                p = here + (value - here) * s
                for issue in K.limit_problems(*p, PITCH):
                    problems.append(f"at x={p[0]:.0f} y={p[1]:.0f} z={p[2]:.0f}: {issue}")
            here = value
    return problems


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm", default=None, help='"COM14" or "192.168.1.203"; default finds it on USB')
    p.add_argument("--dry-run", action="store_true", help="check and print the path without moving")
    args = p.parse_args()

    steps = plan()
    problems = check(steps)
    ready = K.ik(*READY, PITCH)
    print(f"Ready pose: x={READY[0]:.0f} y={READY[1]:.0f} z={READY[2]:.0f} mm, gripper straight down; "
          + " ".join(f"{k}={v:+.2f}" for k, v in ready.items()))
    if problems:
        raise SystemExit("Path would hit a joint limit:\n  " + "\n  ".join(problems[:5]))
    print("Path check: every point stays clear of the joint limits.")

    arm = RoArm(args.arm)
    fb = arm.feedback()
    if args.dry_run:
        for kind, value in steps:
            print(f"  {kind:5s} " + (f"to x={value[0]:.0f} y={value[1]:.0f} z={value[2]:.0f}" if kind == "move" else f"{value:.1f} s"))
        arm.close()
        return
    if fb["v"] < 1000:
        raise SystemExit("Supply is under 10 V. Turn on the 12 V switch first.")
    if not all(fb[f"torswitch{j}"] for j in "BSERTG"):
        raise SystemExit("Some motors are off (the arm is limp). Fix that before running this.")

    rate = RATE_WIFI if "." in arm.target else RATE_USB
    roll, grip = fb["r"], fb["g"]
    q_now = arm.state().astype(float)
    q_ready = np.array([ready["base"], ready["shoulder"], ready["elbow"], ready["wrist"], roll, grip])
    approach = max(2.0, np.max(np.abs(q_ready - q_now)) / 0.3)  # ~0.3 rad/s for the biggest joint
    lowest = min(K.fk(*(q_now + (q_ready - q_now) * min_jerk(u))[:4])[2] for u in np.linspace(0, 1, 51))
    if lowest < READY[2] - 10:
        raise SystemExit(f"Getting to the ready pose from here would swing the tip down to z={lowest:.0f} mm. "
                         "Move the arm up first (e.g. {\"T\":100} in the web app).")

    input(f"The arm will move to the ready pose ({approach:.0f} s), then do the square. "
          f"Clear the area, then press Enter ({rate} steps/s over {arm.target})...")
    try:
        stream(lambda q: arm.move(q, spd=0, acc=0), q_now, q_ready, approach, rate)
        time.sleep(0.5)
        here, dip = READY.copy(), 0
        for kind, value in steps:
            if kind == "pause":
                dip += 1
                time.sleep(0.5)
                m = arm.feedback()
                print(f"Dip {dip}: z commanded {here[2]:.1f}, measured {m['z']:.1f} mm "
                      f"(off by {m['z'] - here[2]:+.1f}); gripper {math.degrees(PITCH):.1f} deg, "
                      f"measured {math.degrees(m['tit']):.1f}")
                time.sleep(max(0.0, value - 0.5))
            else:
                duration = max(1.0, np.linalg.norm(value - here) / SPEED)
                stream(lambda xyz: arm.move_tcp(*xyz, PITCH, roll, grip), here, value, duration, rate)
                here = value
        print("Done. The arm stays at the ready pose.")
    except KeyboardInterrupt:
        print("Stopped; the arm holds where it is.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
