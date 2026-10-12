"""Demo for filming: the same moves with stiff gains (vibrates) and then fixed.

Acts, each the same pattern (~12 s): from the ready pose (gripper down), down
25 mm, pause, back up, 40 mm to the arm's left and back; twice.
  1. stiff  - P=80 on base/shoulder/elbow/wrist, plain streamed targets. This is
              the setting that moved jerkily and vibrated. Kept short on purpose.
  2. stock  - (only with --with-stock) P=16, no compensation: smooth, but it
              stops short when rising.
  3. fixed  - P=16 + gravity + friction feedforward from roarm/tuning.json:
              smooth and lands on target.
After each act it prints the measured landing error (joint sensors) for captions.

Press Enter to start (start recording first); acts follow with a countdown.
Ctrl+C stops at once and resets the gains. Stops on any firmware warning.

    uv run python tmp_scripts/demo_vibration.py
    uv run python tmp_scripts/demo_vibration.py --with-stock
"""
import argparse
import math
import time

import numpy as np

from roarm import kinematics as K
from roarm import motion as M
from roarm import tuning
from roarm.arm import RoArm

READY = np.array([180.0, 0.0, -40.0])
PITCH = math.pi / 2
DOWN = READY - [0, 0, 25.0]
ASIDE = READY + [0, 40.0, 0]
PAUSE_BETWEEN = 5      # s of stillness between acts (room to cut or narrate)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--with-stock", action="store_true", help="add the uncompensated stock act in the middle")
    args = ap.parse_args()
    for a, b in ((READY, DOWN), (READY, ASIDE)):
        if M.check_line(a, b, PITCH):
            raise SystemExit("demo path would hit a joint limit")
    comp = tuning.load_compensation()
    if comp is None:
        raise SystemExit("No friction compensation in roarm/tuning.json yet (run friction_tune.py).")

    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)
    stock = {n: 16 for n in tuning.JOINT_IDS}

    def check():
        f = arm.feedback()
        if f["v"] < 1050 or not all(f[f"torswitch{j}"] for j in "BSERTG") or arm.link.warnings:
            raise RuntimeError(f"safety stop: {f['v'] / 100:.2f} V, warnings {arm.link.warnings}")
        return np.array(K.fk(f["b"], f["s"], f["e"], f["t"])[:3])

    def plain(a, b):
        M.move_tcp_joints(arm, a, b, PITCH, roll, grip, rate)

    def pattern(move):
        errs = {}
        for _ in range(2):
            move(READY, DOWN)
            time.sleep(1.0)
            errs.setdefault("down", []).append(np.linalg.norm(check() - DOWN))
            move(DOWN, READY)
            time.sleep(0.6)
            errs.setdefault("up", []).append(np.linalg.norm(check() - READY))
            move(READY, ASIDE)
            move(ASIDE, READY)
            time.sleep(0.6)
        return {k: float(np.mean(v)) for k, v in errs.items()}

    def countdown(title):
        print(f"\n{title}")
        for n in (3, 2, 1):
            print(f"  {n}...", flush=True)
            time.sleep(1.0)

    acts = [("stiff", "Act 1: stiff gains (P=80), the setting that vibrated")]
    if args.with_stock:
        acts.append(("stock", "Act 2: stock gains (P=16), no compensation"))
    acts.append(("fixed", f"Act {len(acts) + 1}: stock gains + gravity & friction compensation"))

    try:
        arm.reset_pid()
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        input("Start recording, then press Enter...")
        for i, (kind, title) in enumerate(acts):
            if i:
                print(f"\n(holding still {PAUSE_BETWEEN} s)")
                time.sleep(PAUSE_BETWEEN)
            countdown(title)
            if kind == "stiff":
                tuning.apply_gains(arm, {n: 80 for n in tuning.JOINT_IDS}, allow_high=True)
                errs = pattern(plain)
                arm.reset_pid()
            elif kind == "stock":
                errs = pattern(plain)
            else:
                mover = M.Mover(arm, comp, PITCH, roll, grip, rate)
                M.stream(lambda q: arm.move(q, spd=0, acc=0), M.joints_for(READY, PITCH, roll, grip),
                         mover.command(READY, mover.dir), 0.5, rate)
                errs = pattern(mover.move)
            print(f"  {title.split(':')[0]}: lands {errs['down']:.1f} mm off at the bottom, "
                  f"{errs['up']:.1f} mm off after rising")
        print("\nDone.")
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"\nStopped: {e}")
    finally:
        arm.reset_pid()
        arm.close()
        print("Gains reset to stock (P=16).")


if __name__ == "__main__":
    main()
