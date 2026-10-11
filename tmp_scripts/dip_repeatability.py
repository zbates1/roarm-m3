"""Dip at one spot again and again, and measure how consistently the arm lands.

Each trial: move 40 mm to the side and back (so the arm really repositions),
dip 20 mm straight down, measure at the bottom, hold, come back up.
Trials alternate:
- plain:  dip to the target.
- offset: dip to the target shifted by the landing offset learned so far
          (motion.LandingOffset), approached from above just like plain.
Then a lift-step test at the dip spot: raise the target in small steps and
lower it again, to see how far the target must move before the arm follows.

The numbers come from the arm's own joint sensors: they show how far the
joints settle from their targets and how consistent that is. They can't see
flex, backlash outside the sensors, or calibration; a ruler or camera can.

    uv run python tmp_scripts/dip_repeatability.py --dry-run
    uv run python tmp_scripts/dip_repeatability.py              # 10 + 10 dips + lift test, ~5 min
    uv run python tmp_scripts/dip_repeatability.py --trials 5
    Ctrl+C stops it; the arm holds wherever it is.
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
ASIDE = np.array([0.0, 40.0, 0.0])      # where the arm goes between dips (arm's left)
HOLD = 1.0                              # s at the bottom after measuring
LIFT_STEPS = [1, 2, 3, 5, 8, 12]        # mm above the dip bottom, for the lift-step test


def summary(name, rows, bottom):
    pos = np.array([p for p, _ in rows])
    pitch = np.degrees(np.array([t for _, t in rows])) - 90
    off = pos - bottom
    print(f"\n{name} ({len(rows)} dips)")
    print(f"  height: off by {off[:, 2].mean():+.2f} mm on average; spread {off[:, 2].std():.2f} mm (std), "
          f"{np.ptp(off[:, 2]):.2f} mm (max-min)")
    print(f"  sideways: off by x {off[:, 0].mean():+.2f}, y {off[:, 1].mean():+.2f} mm on average; "
          f"worst distance from the average landing point {np.linalg.norm(off - off.mean(0), axis=1).max():.2f} mm")
    print(f"  gripper: {pitch.mean():+.2f} deg past vertical on average; spread {np.ptp(pitch):.2f} deg")


def lift_test(arm, bottom, roll, grip, rate):
    print("\nLift-step test at the dip spot (target and measured height, both relative to the dip bottom):")
    def report(up):
        time.sleep(0.6)
        pos, _ = M.measured(arm)
        print(f"  target {up:+5.1f} mm   measured {pos[2] - bottom[2]:+6.2f} mm   (short by {up - (pos[2] - bottom[2]):.2f})")

    M.move_tcp(arm, READY, bottom, PITCH, roll, grip, rate)
    report(0)
    here = bottom
    for up in LIFT_STEPS + LIFT_STEPS[-2::-1] + [0]:
        goal = bottom + [0, 0, up]
        M.move_tcp(arm, here, goal, PITCH, roll, grip, rate, speed=5.0)
        here = goal
        report(up)
    M.move_tcp(arm, here, READY, PITCH, roll, grip, rate)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm", default=None, help='"COM14" or "192.168.1.203"; default finds it on USB')
    p.add_argument("--trials", type=int, default=10, help="dips per mode")
    p.add_argument("--dry-run", action="store_true", help="check the path without moving")
    args = p.parse_args()

    bottom = READY - [0, 0, DIP]
    problems = M.check_line(READY, bottom, PITCH) + M.check_line(READY, READY + ASIDE, PITCH)
    if problems:
        raise SystemExit("Path would hit a joint limit:\n  " + "\n  ".join(problems[:5]))
    print(f"Dip spot: x={bottom[0]:.0f} y={bottom[1]:.0f} z={bottom[2]:.0f} mm, gripper straight down. "
          f"Path check passed. {args.trials} plain + {args.trials} offset dips, then a lift-step test.")

    arm = RoArm(args.arm)
    fb = arm.feedback()
    if args.dry_run:
        arm.close()
        return
    if fb["v"] < 1000:
        raise SystemExit("Supply is under 10 V. Turn on the 12 V switch first.")
    if not all(fb[f"torswitch{j}"] for j in "BSERTG"):
        raise SystemExit("Some motors are off (the arm is limp). Fix that before running this.")

    rate, roll, grip = M.rate_for(arm), fb["r"], fb["g"]
    input("Clear the area (the arm swings 40 mm to its left between dips), then press Enter...")
    results, offset = {"plain": [], "offset": []}, M.LandingOffset()
    try:
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        for i in range(2 * args.trials):
            mode = "plain" if i % 2 == 0 else "offset"
            aim, aim_pitch = (bottom, PITCH) if mode == "plain" else offset.aim(bottom, PITCH)
            if M.check_line(READY, aim, aim_pitch):
                raise ValueError(f"offset aim {np.round(aim, 1)} would hit a joint limit")
            M.move_tcp(arm, READY, READY + ASIDE, PITCH, roll, grip, rate)
            M.move_tcp(arm, READY + ASIDE, READY, PITCH, roll, grip, rate)
            M.move_tcp(arm, READY, aim, PITCH, roll, grip, rate, end_pitch=aim_pitch)
            time.sleep(0.4)
            pos, tit = M.measured(arm)
            offset.add(aim, aim_pitch, pos, tit)
            results[mode].append((pos, tit))
            off = pos - bottom
            print(f"{i + 1:2d} {mode:6s}  height {off[2]:+.2f} mm   x {off[0]:+.2f}   y {off[1]:+.2f}   "
                  f"gripper {math.degrees(tit) - 90:+.2f} deg")
            time.sleep(HOLD)
            M.move_tcp(arm, aim, READY, aim_pitch, roll, grip, rate, end_pitch=PITCH)
        lift_test(arm, bottom, roll, grip, rate)
    except (KeyboardInterrupt, ValueError) as e:
        print("Stopped; the arm holds where it is." if isinstance(e, KeyboardInterrupt) else f"Stopped: {e}")
    finally:
        arm.close()
    for mode, rows in results.items():
        if rows:
            summary(mode.capitalize(), rows, bottom)
    if offset.errors:
        e = np.mean(offset.errors, axis=0)
        print(f"\nLearned landing offset: x {e[0]:+.2f}, y {e[1]:+.2f}, z {e[2]:+.2f} mm, "
              f"gripper {math.degrees(e[3]):+.2f} deg (sent minus measured)")


if __name__ == "__main__":
    main()
