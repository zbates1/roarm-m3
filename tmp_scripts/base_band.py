"""Where does the base stop, depending on its angle, reach and history? (roarm-m3 firmware, USB). No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR (x 175-220, |y| <= 31, z -30).
Stock gains, plain commands, only the base moves.

The friction refit (experiments/2026-10-17/223652_friction_refit/) found the
base's friction band centred ~5 counts differently on the -y and +y sides. If
something pulls the base towards one angle (cable drag, a tilted table), the
centre moves smoothly with angle; a tilt also grows with reach, cable drag
doesn't. If the centre depends on which way the sweep is going, it's hysteresis.

1. Sweep: gripper down at z=-30, reach 180 then 220 mm, base -8 -> +8 deg and
   back in 4 deg steps. At each angle, arrive from 40 counts below and above
   (twice each) and read the base servo's goal and present position.
2. Approach length: at reach 200, base 0, arrive from 10/20/40/80 counts away on
   each side. A short move may not fully break free of friction.
--fine: only a 1 deg map, reach 180, base -10 -> +10 deg and back, once from each
   side per pass (|y| <= 31 mm).

Error = measured - commanded, in base joint counts (1 count = 0.088 deg; at
200 mm reach about 0.31 mm sideways). Band = from above - from below; centre =
their mean. Results: experiments/<date>/<time>_base_band/.

    uv run python tmp_scripts/base_band.py [--fine]
"""
import json
import math
import time

import numpy as np

from roarm import experiments
from roarm import motion as M
from roarm.arm import RoArm

import shoulder_share as ss

PITCH = math.pi / 2
Z = -30.0
COUNT = M.COUNT
REACHES = (180.0, 220.0)
ANGLES = [-8, -4, 0, 4, 8, 4, 0, -4, -8]   # deg, up then down
DELTA = 40                                  # counts
SIDES = (-1, 1, 1, -1)                      # -1 = arrive from below (moving +), +1 = from above
LENGTHS = (10, 20, 40, 80)
LOAD_STOP = 850


def main():
    import argparse

    global REACHES, ANGLES, SIDES
    ap = argparse.ArgumentParser()
    ap.add_argument("--fine", action="store_true")
    fine = ap.parse_args().fine
    if fine:
        REACHES, SIDES = (180.0,), (-1, 1)
        ANGLES = list(range(-10, 11)) + list(range(9, -11, -1))
    arm = RoArm()
    if "." in arm.target or arm.read_reg(11, 22) is None:
        raise SystemExit("Needs USB and the roarm-m3 firmware (T:116).")
    run = experiments.new_run("base_band_fine" if fine else "base_band")
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)
    send = lambda q: arm.move(q, spd=0, acc=0)
    log = {"delta_counts": DELTA, "sweep": [], "lengths": []}
    print(f"Results -> {run}", flush=True)
    q_now = None

    def base_to(theta, duration):
        nonlocal q_now
        q = q_now.copy()
        q[0] = theta
        M.stream(send, q_now, q, duration, rate)
        q_now = q

    def land(theta, side, delta):
        """Arrive at theta from delta counts away on one side; return the error (joint counts)."""
        base_to(theta + side * delta * COUNT, 0.5)
        time.sleep(0.3)
        base_to(theta, 0.5)
        time.sleep(0.8)
        goal, present = arm.read_reg(11, 42, 2), arm.read_reg(11, 56, 2)
        load = ss.signed_load(arm.read_reg(11, 60, 2))
        meas = float(arm.state()[0])
        if arm.link.warnings:
            raise RuntimeError(f"firmware warning: {arm.link.warnings[-1]}")
        if load is not None and abs(load) > LOAD_STOP:
            raise RuntimeError(f"base load {load}")
        # base servo counts run opposite to the joint angle
        return {"side": side, "delta": delta, "goal": goal, "present": present, "load": load,
                "err_counts": -(present - goal), "err_state_counts": round((meas - theta) / COUNT, 1)}

    def show(rows):
        lo = [r["err_counts"] for r in rows if r["side"] == -1]
        hi = [r["err_counts"] for r in rows if r["side"] == 1]
        return np.mean(lo), np.mean(hi), np.mean(hi) - np.mean(lo), 0.5 * (np.mean(hi) + np.mean(lo))

    try:
        arm.reset_pid()
        here = np.array([REACHES[0], 0.0, Z + 15])
        M.go_to_pose(arm, here, PITCH, roll, grip, rate)
        for r in REACHES:
            pose = np.array([r, 0.0, Z])
            M.move_tcp_joints(arm, here, pose, PITCH, roll, grip, rate)
            here = pose
            q_now = M.joints_for(pose, PITCH, roll, grip)
            print(f"\nReach {r:.0f} mm: angle  pass  from below / from above (counts)  band  centre", flush=True)
            for k, deg in enumerate(ANGLES):
                theta = math.radians(deg)
                base_to(theta, 0.8)
                rows = [land(theta, s, DELTA) for s in SIDES]
                lo, hi, band, centre = show(rows)
                pas = "up" if k <= len(ANGLES) // 2 else "down"
                log["sweep"].append({"reach": r, "deg": deg, "pass": pas, "rows": rows,
                                     "from_below": lo, "from_above": hi, "band": band, "centre": centre})
                print(f"  {deg:+3d}  {pas:4s}  {lo:+6.1f} / {hi:+6.1f}   {band:5.1f}  {centre:+5.1f}", flush=True)
            base_to(0.0, 1.0)
        # approach length at reach 200
        pose = np.array([200.0, 0.0, Z])
        lengths = () if fine else LENGTHS
        if lengths:
            M.move_tcp_joints(arm, here, pose, PITCH, roll, grip, rate)
            here = pose
            q_now = M.joints_for(pose, PITCH, roll, grip)
            print("\nReach 200 mm, base 0: approach (counts)  from below / from above  band  centre", flush=True)
        for d in lengths:
            rows = [land(0.0, s, d) for s in SIDES]
            lo, hi, band, centre = show(rows)
            log["lengths"].append({"delta": d, "rows": rows, "from_below": lo, "from_above": hi, "band": band, "centre": centre})
            print(f"  {d:3d}   {lo:+6.1f} / {hi:+6.1f}   {band:5.1f}  {centre:+5.1f}", flush=True)
        base_to(0.0, 0.8)
        # centre vs angle: straight-line fit per reach
        log["fit"] = {}
        for r in REACHES:
            pts = [(s["deg"], s["centre"]) for s in log["sweep"] if s["reach"] == r]
            slope, icpt = np.polyfit(*zip(*pts), 1)
            up = {s["deg"]: s["centre"] for s in log["sweep"] if s["reach"] == r and s["pass"] == "up"}
            dn = {s["deg"]: s["centre"] for s in log["sweep"] if s["reach"] == r and s["pass"] == "down"}
            hyst = float(np.mean([up[d] - dn[d] for d in dn if d in up]))
            log["fit"][str(int(r))] = {"centre_at_0": round(float(icpt), 2), "slope_per_deg": round(float(slope), 3),
                                       "up_minus_down": round(hyst, 2)}
            print(f"Reach {r:.0f}: centre = {icpt:+.1f} {slope:+.2f}*deg counts; up pass - down pass {hyst:+.1f} counts",
                  flush=True)
        M.move_tcp_joints(arm, here, np.array([180.0, 0.0, -25.0]), PITCH, roll, grip, rate)
        log["finished"] = "ok"
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e}", flush=True)
        log["finished"] = f"stopped: {e}"
    finally:
        arm.reset_pid()
        arm.close()
        (run / "data.json").write_text(json.dumps(log, indent=1, default=float))
        print(f"Saved {run / 'data.json'}", flush=True)


if __name__ == "__main__":
    main()
