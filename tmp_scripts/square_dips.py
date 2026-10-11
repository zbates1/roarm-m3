"""Trace a 20 mm square with the gripper tip, dipping 20 mm down at each corner.

At each corner: down 20 mm, pause 2 s, up 20 mm. The corners go left, then
forward (away from the base), then right, then back to the start.

Smoothness: the firmware's own smooth move (T:104) starts from the *measured*
position, which sags a little under gravity, so the target drops at the start
of every move. Instead, this script plans the path itself and streams small
steps (T:1041) along a minimum-jerk curve: speed and acceleration both start
and end at zero. It reads the arm's position once at the start and never again.

    uv run python tmp_scripts/square_dips.py            # USB, found automatically
    uv run python tmp_scripts/square_dips.py --dry-run  # print the path, don't move
    uv run python tmp_scripts/square_dips.py --arm 192.168.1.203   # over WiFi (less smooth)
    Ctrl+C stops it; the arm holds wherever it is.
"""
import argparse
import time

import numpy as np

from roarm.arm import RoArm

SIDE = 20.0       # mm, side of the square
DIP = 20.0        # mm, how far down at each corner
PAUSE = 2.0       # s, at the bottom of each dip
MOVE_TIME = 1.5   # s, for each 20 mm move
RATE = 50         # Hz, how often a new target is sent
LEFT = +1         # +y is the arm's own left (your right, if you face it). -1 flips it.


def min_jerk(n):
    """n points from 0 to 1 with zero speed and acceleration at both ends."""
    s = np.linspace(0, 1, n)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def plan(start):
    """The path as a list of ("move", xyz) and ("pause", seconds)."""
    down = np.array([0, 0, -DIP])
    corners = [(0, 0), (0, SIDE), (SIDE, SIDE), (SIDE, 0)]  # (forward, left) in mm
    steps = []
    for i, (fwd, left) in enumerate(corners):
        corner = start + [fwd, LEFT * left, 0]
        if i > 0:
            steps.append(("move", corner))
        steps += [("move", corner + down), ("pause", PAUSE), ("move", corner)]
    steps.append(("move", start.copy()))
    return steps


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm", default=None, help='"COM14" or "192.168.1.203"; default finds it on USB')
    p.add_argument("--dry-run", action="store_true", help="print the path without moving")
    args = p.parse_args()

    arm = RoArm(args.arm)
    fb = arm.feedback()
    start = np.array([fb["x"], fb["y"], fb["z"]])
    pitch, roll, grip = fb["tit"], fb["r"], fb["g"]
    print(f"Start: x={start[0]:.1f} y={start[1]:.1f} z={start[2]:.1f} mm, supply {fb['v'] / 100:.1f} V")

    steps = plan(start)
    if args.dry_run:
        for kind, value in steps:
            print(f"  {kind:5s} " + (f"to x={value[0]:.1f} y={value[1]:.1f} z={value[2]:.1f}" if kind == "move" else f"{value:.1f} s"))
        arm.close()
        return

    if fb["v"] < 1000:
        raise SystemExit("Supply is under 10 V. Turn on the 12 V switch first.")
    if not all(fb[f"torswitch{j}"] for j in "BSERTG"):
        raise SystemExit("Some motors are off (the arm is limp). Fix that before running this.")

    # Hold the current position and check that nothing jumps before moving.
    joints_before = arm.state()
    arm.move_tcp(*start, pitch, roll, grip)
    time.sleep(0.5)
    jump = np.max(np.abs(arm.state() - joints_before))
    if jump > 0.1:
        arm.move(joints_before, spd=300, acc=10)
        raise SystemExit(f"A joint moved {jump:.2f} rad just holding still. Stopped; please tell Claude.")

    input("Clear the area, then press Enter to start...")
    here = start.copy()
    try:
        for kind, value in steps:
            if kind == "pause":
                time.sleep(value)
                continue
            n = max(2, int(MOVE_TIME * RATE))
            t0 = time.perf_counter()
            for i, s in enumerate(min_jerk(n)):
                arm.move_tcp(*(here + (value - here) * s), pitch, roll, grip)
                time.sleep(max(0.0, t0 + (i + 1) / RATE - time.perf_counter()))
            here = value
        print("Done.")
    except KeyboardInterrupt:
        print("Stopped; the arm holds where it is.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
