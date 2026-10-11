"""Try different servo gains and measure whether the arm reaches its targets.

Each servo runs its own position loop. The firmware sets P=16, I=0 at every
startup. Proportional-only control can't remove a steady error against
gravity: higher P shrinks it, and an integral term should remove it. This
tries a few settings on the joints that hold the arm up (shoulder, elbow,
wrist) and, for each one, measures:

- landing: how far the tip settles from the dip spot after arriving from
  above, at 0.4 s, 1.5 s and 3 s (integral action should shrink it over time)
- step up: a 3 mm upward target change from rest, which didn't move the
  arm at all with the stock gains
- jitter: how much the reading moves while holding still (oscillation)

If the jitter gets large it resets the gains (T:109) and stops. It always
resets them at the end, and a restart resets them too.

    uv run python tmp_scripts/gain_test.py --dry-run
    uv run python tmp_scripts/gain_test.py           # ~2-3 min, USB
    Ctrl+C stops it, resets the gains, and the arm holds where it is.
"""
import argparse
import math
import time

import numpy as np

from roarm import motion as M
from roarm.arm import RoArm

READY = np.array([180.0, 0.0, -40.0])  # mm, above the dip spot (table is ~126 mm below z=0)
PITCH = math.pi / 2                     # gripper pointing straight down
DIP = 20.0                              # mm
STEP_UP = 3.0                           # mm, the reversal test
LOADED_JOINTS = (2, 3, 4)               # shoulder, elbow, wrist
CONFIGS = [(16, 0), (32, 0), (16, 2), (16, 8), (16, 32), (32, 8)]  # (P, I); (16, 0) is stock
JITTER_LIMIT = 0.5                      # mm std of height while holding; above this = oscillating


def heights(arm, n):
    return np.array([M.measured(arm)[0][2] for _ in range(n)])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm", default=None, help='"COM14" or "192.168.1.203"; default finds it on USB')
    p.add_argument("--dry-run", action="store_true", help="check the path without moving")
    args = p.parse_args()

    bottom = READY - [0, 0, DIP]
    if M.check_line(READY, bottom, PITCH):
        raise SystemExit("Dip path would hit a joint limit.")
    print(f"Dip spot x={bottom[0]:.0f} y={bottom[1]:.0f} z={bottom[2]:.0f} mm. Gains to try on shoulder, "
          f"elbow, wrist (P, I): {CONFIGS}")

    arm = RoArm(args.arm)
    fb = arm.feedback()
    if args.dry_run:
        arm.close()
        return
    if "." in arm.target:
        raise SystemExit("Run this over USB: WiFi readings are too slow to measure jitter.")
    if fb["v"] < 1000:
        raise SystemExit("Supply is under 10 V. Turn on the 12 V switch first.")
    if not all(fb[f"torswitch{j}"] for j in "BSERTG"):
        raise SystemExit("Some motors are off (the arm is limp). Fix that before running this.")

    rate, roll, grip = M.rate_for(arm), fb["r"], fb["g"]
    input("Clear the area, keep a hand near the power switch, then press Enter...")
    rows = []
    try:
        arm.reset_pid()
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        for P, I in CONFIGS:
            for j in LOADED_JOINTS:
                arm.set_pid(j, P, I)
            time.sleep(1.0)
            if heights(arm, 15).std() > JITTER_LIMIT:
                raise RuntimeError(f"P={P} I={I}: the arm is oscillating while holding still")

            landing = []
            for _ in range(2):
                M.move_tcp(arm, READY, bottom, PITCH, roll, grip, rate)
                t0, errs = time.perf_counter(), []
                for t in (0.4, 1.5, 3.0):
                    time.sleep(max(0.0, t0 + t - time.perf_counter()))
                    errs.append(M.measured(arm)[0][2] - bottom[2])
                landing.append(errs)
                jitter = heights(arm, 15).std()
                if jitter > JITTER_LIMIT:
                    raise RuntimeError(f"P={P} I={I}: the arm is oscillating at the dip spot")
                M.move_tcp(arm, bottom, READY, PITCH, roll, grip, rate)

            M.move_tcp(arm, READY, bottom, PITCH, roll, grip, rate)
            time.sleep(1.5)
            before = M.measured(arm)[0][2]
            M.move_tcp(arm, bottom, bottom + [0, 0, STEP_UP], PITCH, roll, grip, rate, speed=5.0)
            time.sleep(1.5)
            followed = M.measured(arm)[0][2] - before
            M.move_tcp(arm, bottom + [0, 0, STEP_UP], READY, PITCH, roll, grip, rate)

            landing = np.mean(landing, axis=0)
            rows.append((P, I, *landing, jitter, followed))
            print(f"P={P:2d} I={I:2d}: landing {landing[0]:+.2f} / {landing[1]:+.2f} / {landing[2]:+.2f} mm "
                  f"(0.4 / 1.5 / 3 s)   step up {followed:+.2f} of {STEP_UP:.0f} mm   jitter {jitter:.2f} mm")
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print("Stopped." if isinstance(e, KeyboardInterrupt) else f"Stopped: {e}")
    finally:
        arm.reset_pid()
        time.sleep(0.2)
        arm.close()
        print("Gains reset to P=16, I=0.")

    if rows:
        print("\n  P   I   landing 0.4 s   1.5 s    3 s    step up    jitter")
        for P, I, a, b, c, jit, up in rows:
            print(f" {P:2d}  {I:2d}     {a:+6.2f}    {b:+6.2f}  {c:+6.2f}    {up:+5.2f}     {jit:.2f}")


if __name__ == "__main__":
    main()
