"""Trace a 20 mm square with the gripper tip, dipping 20 mm down at each corner.

1. Moves to a ready pose with the gripper pointing straight down, where every
   joint has room to spare (it blends joint angles, so it can't hit a limit).
2. At each corner: down 20 mm, pause 2 s, up 20 mm. Corners go left, forward
   (away from the base), right, then back. The gripper keeps its angle.
3. At the bottom of each dip it prints commanded vs measured height and angle.

Every point is checked against the joint limits first, and moves are streamed
as small min-jerk steps (see roarm/motion.py for why).

    uv run python tmp_scripts/square_dips.py --dry-run            # check the path, don't move
    uv run python tmp_scripts/square_dips.py                      # USB (smoothest)
    uv run python tmp_scripts/square_dips.py --arm 192.168.1.203  # WiFi
    Ctrl+C stops it; the arm holds wherever it is.
"""
import argparse
import math
import time

import numpy as np

from roarm import kinematics as K
from roarm import motion as M
from roarm.arm import RoArm

READY = np.array([180.0, 0.0, -40.0])  # mm. z=0 is the shoulder pivot; the table is ~126 mm below it
PITCH = math.pi / 2                     # gripper pointing straight down
SIDE = 20.0       # mm, side of the square
DIP = 20.0        # mm, how far down at each corner
PAUSE = 2.0       # s, at the bottom of each dip
SPEED = 15.0      # mm/s average for each move (peaks at ~1.9x on the min-jerk curve)
LEFT = +1         # +y is the arm's own left (your right, if you face it). -1 flips it.


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


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm", default=None, help='"COM14" or "192.168.1.203"; default finds it on USB')
    p.add_argument("--dry-run", action="store_true", help="check and print the path without moving")
    args = p.parse_args()

    steps, here, problems = plan(), READY, []
    for kind, value in steps:
        if kind == "move":
            problems += M.check_line(here, value, PITCH)
            here = value
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

    rate, roll, grip = M.rate_for(arm), fb["r"], fb["g"]
    input(f"The arm will move to the ready pose, then do the square. "
          f"Clear the area, then press Enter ({rate} steps/s over {arm.target})...")
    try:
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        time.sleep(0.5)
        here, dip = READY.copy(), 0
        for kind, value in steps:
            if kind == "pause":
                dip += 1
                time.sleep(0.5)
                pos, tit = M.measured(arm)
                print(f"Dip {dip}: z commanded {here[2]:.1f}, measured {pos[2]:.1f} mm "
                      f"(off by {pos[2] - here[2]:+.1f}); gripper {math.degrees(PITCH):.1f} deg, "
                      f"measured {math.degrees(tit):.1f}")
                time.sleep(max(0.0, value - 0.5))
            else:
                M.move_tcp(arm, here, value, PITCH, roll, grip, rate, speed=SPEED)
                here = value
        print("Done. The arm stays at the ready pose.")
    except (KeyboardInterrupt, ValueError) as e:
        print("Stopped; the arm holds where it is." if isinstance(e, KeyboardInterrupt) else f"Stopped: {e}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
