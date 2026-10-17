"""Supervised smoothness test: you watch each setting and rate it.

P=80 looked fine in the at-rest checks but moved jerkily and vibrated, because
every streamed step (50 per second) was sent with the servos' acceleration
limit off (acc=0), so each step became a full-acceleration kick. This runs the
same short motion for each combination of
    P   (stiffness) on base/shoulder/elbow/wrist: 16 (stock), 24, 32, 40
    acc (the servos' own acceleration limit per step): 0 = off (as before),
        20 = the firmware's default for its own moves, 8 = gentler
and asks you to rate each one. P only goes up, so if a setting looks rough you
can stop before anything stiffer runs.

Each trial (~8 s): from the ready pose, down 20 mm, up 20 mm, 30 mm to the
arm's left and back. Also records, from the joint sensors, the landing error,
how far short the rise stops, and the error after the sideways return.

    uv run python tmp_scripts/smooth_test.py
    Type 1-5 after each trial (1 = rough/vibrating, 5 = smooth), or s to stop.
"""
import json
import math
import time
from pathlib import Path

import numpy as np

from roarm import experiments
from roarm import kinematics as K
from roarm import motion as M
from roarm import tuning
from roarm.arm import RoArm

READY = np.array([180.0, 0.0, -40.0])
PITCH = math.pi / 2
P_LEVELS = [16, 24, 32, 40]
ACC_LEVELS = [0, 20, 8]
SPEED = 15.0     # mm/s average, same as the other tests
OUT = experiments.day_dir()  # experiments/<today>/


def main():
    for a, b in ((READY, READY - [0, 0, 20]), (READY, READY + [0, 30, 0])):
        if M.check_line(a, b, PITCH):
            raise SystemExit("test path would hit a joint limit")
    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)

    def tip():
        f = arm.feedback()
        if f["v"] < 1050 or not all(f[f"torswitch{j}"] for j in "BSERTG") or arm.link.warnings:
            raise RuntimeError(f"safety stop: {f['v'] / 100:.2f} V, warnings {arm.link.warnings}")
        loads = max(abs(f[k]) for k in ("tB", "tS", "tE", "tT"))
        return np.array(K.fk(f["b"], f["s"], f["e"], f["t"])[:3]), loads

    def line(a, b, acc):
        duration = max(1.0, np.linalg.norm(b - a) / SPEED)
        M.stream(lambda p: arm.move(M.joints_for(p, PITCH, roll, grip), spd=0, acc=acc), a, b, duration, rate)

    def ask(prompt):
        while True:
            a = input(prompt).strip().lower()
            if a == "s" or a in {"1", "2", "3", "4", "5"}:
                return a

    rows, p_now = [], 16
    print("Watch the arm, especially on the way down. Starting at stock settings.")
    try:
        arm.reset_pid()
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        stop = False
        for P in P_LEVELS:
            if P > p_now:
                tuning.apply_gains(arm, {n: P for n in tuning.JOINT_IDS}, start=p_now, step=8, allow_high=True)
                p_now = P
            for acc in ACC_LEVELS:
                print(f"\nP={P}, acc={acc} {'(off)' if acc == 0 else ''} ...", flush=True)
                bottom, aside = READY - [0, 0, 20], READY + [0, 30, 0]
                line(READY, bottom, acc)
                time.sleep(0.8)
                down, l1 = tip()
                line(bottom, READY, acc)
                time.sleep(0.8)
                up, l2 = tip()
                line(READY, aside, acc)
                line(aside, READY, acc)
                time.sleep(0.8)
                back, l3 = tip()
                r = {"P": P, "acc": acc, "down_err": (down - bottom).round(2).tolist(),
                     "rise_short_mm": round(float(READY[2] - up[2]), 2),
                     "return_err": (back - READY).round(2).tolist(), "max_load": int(max(l1, l2, l3))}
                print(f"  landing off {np.linalg.norm(down - bottom):.2f} mm, rise stopped {r['rise_short_mm']:.2f} mm short, "
                      f"sideways return off {np.linalg.norm(back - READY):.2f} mm, max load {r['max_load']}")
                a = ask("  How did that look? 1 = rough/vibrating ... 5 = smooth, s = stop: ")
                if a == "s":
                    stop = True
                    break
                r["rating"] = int(a)
                rows.append(r)
            if stop:
                break
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"\nStopped: {e}")
    finally:
        arm.reset_pid()
        arm.close()
        print("Gains reset to stock (P=16).")

    if rows:
        print("\n  P  acc  rating  landing off  rise short  return off  max load")
        for r in rows:
            print(f" {r['P']:2d}  {r['acc']:3d}     {r['rating']}      {np.linalg.norm(r['down_err']):5.2f}       "
                  f"{r['rise_short_mm']:5.2f}       {np.linalg.norm(r['return_err']):5.2f}      {r['max_load']}")
        OUT.mkdir(exist_ok=True)
        path = OUT / time.strftime("smooth_%Y%m%d_%H%M%S.json")
        path.write_text(json.dumps(rows, indent=1))
        print(f"Saved {path}")


if __name__ == "__main__":
    main()
