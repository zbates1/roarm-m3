"""Does the shoulder trim stop the two shoulder servos fighting? (roarm-m3 firmware). No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR. Stock gains, plain commands.
Conditions: the power-on trim, trim 0 (Waveshare's behaviour), the power-on trim
again (drift check). For each:
  poses  - 4 reaches, arrive from above: mismatch, each servo's error, loads A/B
  band   - shoulder-only steps of +-16/32 counts at the ready pose: how far the
           resting point depends on direction (friction band, counts)
  rise   - 20 mm straight up at x=190: how far short (vertical) and inward it stops
Results: experiments/<date>/<time>_shoulder_trim/.

    uv run python tmp_scripts/shoulder_trim_test.py
"""
import json
import math
import time

import numpy as np

from roarm import experiments
from roarm import kinematics as K
from roarm import motion as M
from roarm.arm import RoArm

import shoulder_share as ss

PITCH = math.pi / 2
READY = np.array([180.0, 0.0, -40.0])
RISE = (np.array([190.0, 0.0, -60.0]), np.array([190.0, 0.0, -40.0]))
STEPS = [0, 16, 32, 16, 0, -16, -32, -16, 0]
COUNT = 2 * math.pi / 4096


def main():
    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    boot_trim = arm.shoulder_trim()
    run = experiments.new_run("shoulder_trim")
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)
    here = None
    results = []
    print(f"Power-on trim: {boot_trim}. Results -> {run}", flush=True)

    def go(target):
        nonlocal here
        target = np.asarray(target, float)
        above = target + [0, 0, 15]
        if here is None:
            M.go_to_pose(arm, above, PITCH, roll, grip, rate)
        else:
            top = np.array([*here[:2], max(here[2], above[2])])
            for a, b in ((here, top), (top, np.array([*above[:2], top[2]])), (np.array([*above[:2], top[2]]), above)):
                if np.linalg.norm(b - a) > 0.01:
                    M.move_tcp_joints(arm, a, b, PITCH, roll, grip, rate)
        M.move_tcp_joints(arm, above, target, PITCH, roll, grip, rate)
        here = target

    try:
        arm.reset_pid()
        for label, trim in (("power-on trim", boot_trim), ("trim 0", 0), ("power-on trim again", boot_trim)):
            arm.shoulder_trim(trim)
            row = {"condition": label, "trim": trim, "poses": [], "band": None, "rise": None}
            print(f"\n== {label} (trim {trim}) ==", flush=True)
            for pose in ss.POSES:
                go(pose)
                time.sleep(1.5)
                r = ss.sample(arm)
                row["poses"].append({"x": float(pose[0]), **r})
                print(f"  x={pose[0]:.0f}: mismatch {r['mismatch']:+3d}  err A/B {r['err_A']:+3d}/{r['err_B']:+3d}"
                      f"  load A/B {r['A']['load']:+4d}/{r['B']['load']:+4d}", flush=True)
            # shoulder-only steps at the ready pose
            go(READY)
            time.sleep(1.0)
            q0 = M.joints_for(READY, PITCH, roll, grip)
            meas = []
            for d in STEPS:
                q = q0.copy()
                q[1] += d * COUNT
                arm.move(q, spd=0, acc=0)
                time.sleep(0.8)
                meas.append(round((arm.state()[1] - q0[1]) / COUNT, 1))
            arm.move(q0, spd=0, acc=0)
            row["band"] = {"commanded": STEPS, "measured": meas, "band_counts": round(meas[4] - meas[8], 1),
                           "sag_counts": round(-meas[0], 1)}
            print(f"  shoulder steps {STEPS} -> {meas}; band {row['band']['band_counts']} counts", flush=True)
            # rise
            go(RISE[0])
            time.sleep(0.8)
            M.move_tcp_joints(arm, RISE[0], RISE[1], PITCH, roll, grip, rate)
            time.sleep(1.0)
            s = arm.state()
            tip = np.array(K.fk(*s[:4])[:3])
            err = tip - RISE[1]
            row["rise"] = {"vertical_mm": round(float(err[2]), 2), "radial_mm": round(float(err[0]), 2)}
            here = RISE[1]
            print(f"  rise: stops {err[2]:+.2f} mm vertical, {err[0]:+.2f} mm radial", flush=True)
            results.append(row)
            if arm.link.warnings:
                raise RuntimeError(f"firmware warning: {arm.link.warnings[-1]}")
        arm.shoulder_trim(boot_trim)
        go(np.array([180.0, 0.0, -25.0]))
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e}", flush=True)
        results.append({"stopped": str(e)})
    finally:
        try:
            arm.shoulder_trim(boot_trim)
        finally:
            arm.close()
        (run / "data.json").write_text(json.dumps({"boot_trim": boot_trim, "results": results}, indent=1))
        print(f"Saved {run / 'data.json'}", flush=True)


if __name__ == "__main__":
    main()
