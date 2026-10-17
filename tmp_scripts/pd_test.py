"""Supervised P x D smoothness test (needs the roarm-m3 firmware with T:116/T:117).

Higher P (stiffness) made motion rough; the servos already run D=32 (damping).
This tries more damping at each stiffness and you rate how it looks. The same
motion as smooth_test.py: from the ready pose, down 20 mm, up 20 mm, 30 mm to the
arm's left and back. Gains go to base, shoulder (both servos), elbow, wrist.

Writes are not saved on the servos; the script puts every servo back to P=16,
D=32 at the end (or on Ctrl+C / a safety stop) and reads them back to confirm.

    uv run python tmp_scripts/pd_test.py
    uv run python tmp_scripts/pd_test.py --p 16 32 48 --d 32 64 128
    Type 1-5 after each trial (1 = rough/vibrating, 5 = smooth), or s to stop.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from roarm import kinematics as K
from roarm import motion as M
from roarm.arm import RoArm

READY = np.array([180.0, 0.0, -40.0])
PITCH = math.pi / 2
SPEED = 15.0
IDS = [11, 12, 13, 14, 15]          # base, shoulder x2, elbow, wrist
STOCK = {"P": 16, "D": 32}
REG = {"P": 21, "D": 22}
OUT = Path(__file__).resolve().parent / "results"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--p", type=int, nargs="+", default=[16, 32, 48])
    ap.add_argument("--d", type=int, nargs="+", default=[32, 64, 128])
    args = ap.parse_args()
    for a, b in ((READY, READY - [0, 0, 20]), (READY, READY + [0, 30, 0])):
        if M.check_line(a, b, PITCH):
            raise SystemExit("test path would hit a joint limit")

    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    if arm.read_reg(11, 22) is None:
        raise SystemExit("No reply to T:116: flash the roarm-m3 firmware first (firmware/README.md).")
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)

    def set_gain(name, value):
        for i in IDS:
            arm.write_reg(i, REG[name], value)

    def ramp_p(frm, to):
        """Step P up by 8 at a time (a jump from 16 to 80 once tripped the overcurrent warning)."""
        for p in (list(range(frm + 8, to, 8)) + [to]) if to > frm else [to]:
            set_gain("P", p)
            time.sleep(0.25)

    def tip():
        f = arm.feedback()
        if f["v"] < 1050 or not all(f[f"torswitch{j}"] for j in "BSERTG") or arm.link.warnings:
            raise RuntimeError(f"safety stop: {f['v'] / 100:.2f} V, warnings {arm.link.warnings}")
        return np.array(K.fk(f["b"], f["s"], f["e"], f["t"])[:3])

    def line(a, b):
        duration = max(1.0, np.linalg.norm(b - a) / SPEED)
        M.stream(lambda p: arm.move(M.joints_for(p, PITCH, roll, grip), spd=0, acc=0), a, b, duration, rate)

    def ask():
        while True:
            a = input("  How did that look? 1 = rough/vibrating ... 5 = smooth, s = stop: ").strip().lower()
            if a == "s" or a in {"1", "2", "3", "4", "5"}:
                return a

    rows, p_now = [], STOCK["P"]
    try:
        set_gain("P", STOCK["P"])
        set_gain("D", STOCK["D"])
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        print("Watch the arm, especially on the way down. Starting at stock P=16, D=32.")
        stop = False
        for P in args.p:
            ramp_p(p_now, P)
            p_now = P
            for D in args.d:
                set_gain("D", D)
                time.sleep(0.5)
                print(f"\nP={P}, D={D} ...", flush=True)
                bottom, aside = READY - [0, 0, 20], READY + [0, 30, 0]
                line(READY, bottom)
                time.sleep(0.8)
                down = tip()
                line(bottom, READY)
                time.sleep(0.8)
                up = tip()
                line(READY, aside)
                line(aside, READY)
                time.sleep(0.8)
                r = {"P": P, "D": D, "landing_mm": round(float(np.linalg.norm(down - bottom)), 2),
                     "rise_short_mm": round(float(READY[2] - up[2]), 2)}
                print(f"  landing off {r['landing_mm']:.2f} mm, rise stopped {r['rise_short_mm']:.2f} mm short")
                a = ask()
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
        try:
            set_gain("P", STOCK["P"])
            set_gain("D", STOCK["D"])
            back = {i: (arm.read_reg(i, 21), arm.read_reg(i, 22)) for i in IDS}
            print(f"Gains restored (servo: (P, D)): {back}")
        finally:
            arm.close()

    if rows:
        print("\n  P    D  rating  landing off  rise short")
        for r in rows:
            print(f" {r['P']:2d}  {r['D']:3d}     {r['rating']}       {r['landing_mm']:5.2f}       {r['rise_short_mm']:5.2f}")
        OUT.mkdir(exist_ok=True)
        path = OUT / time.strftime("pd_%Y%m%d_%H%M%S.json")
        path.write_text(json.dumps(rows, indent=1))
        print(f"Saved {path}")


if __name__ == "__main__":
    main()
