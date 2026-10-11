"""Watch the tip during a dip and rise: commanded path vs what the encoders see.

For stock and then tuned control, at the dip spot: descend 20 mm, hold, rise
20 mm, hold, reading the joints as fast as USB allows (~20 per second) the whole
time. The gap between measured and commanded tip is split into radial (+ away
from the base, - toward it), sideways and vertical, so you can see whether the
"slides toward the base while rising" shows up in the joint sensors.

If it does: it's joints lagging their targets (fixable in control).
If the sensors show a straight path while you see the tip move: it's flex or
play past the sensors (needs a camera or gauge to measure).

    uv run python tmp_scripts/track_test.py --dry-run
    uv run python tmp_scripts/track_test.py              # ~1 min, USB
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from roarm import kinematics as K
from roarm import motion as M
from roarm import tuning
from roarm.arm import RoArm

TOP = np.array([190.0, 0.0, -40.0])
BOTTOM = TOP - [0, 0, 20.0]
PITCH = math.pi / 2
MOVE_TIME = 1.5      # s for each 20 mm move (same as the dip tests)
HOLD = 1.0           # s of readings after each move
DIPS = 2             # per mode
OUT = Path(__file__).resolve().parent / "results"


def components(cmd, meas):
    """Measured minus commanded, as (radial, sideways, vertical) mm."""
    d = np.asarray(meas) - cmd
    r_hat = np.array([cmd[0], cmd[1]]) / np.hypot(cmd[0], cmd[1])
    t_hat = np.array([-r_hat[1], r_hat[0]])
    return float(d[:2] @ r_hat), float(d[:2] @ t_hat), float(d[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    if M.check_line(TOP, BOTTOM, PITCH):
        raise SystemExit("path would hit a joint limit")
    gains, gravity = tuning.load()
    print(f"Dip at x={TOP[0]:.0f}, z {TOP[2]:.0f} -> {BOTTOM[2]:.0f} mm; stock, then tuned {gains}")
    arm = RoArm(args.arm)
    if args.dry_run:
        arm.close()
        return
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    fb = arm.feedback()
    roll, grip, trace = fb["r"], fb["g"], []

    def tip():
        f = arm.feedback()
        if f["v"] < 1050 or getattr(arm.link, "warnings", None):
            raise RuntimeError(f"safety stop: {f['v'] / 100:.2f} V, {arm.link.warnings}")
        return np.array(K.fk(f["b"], f["s"], f["e"], f["t"])[:3])

    def send(mode, p):
        if mode == "stock":
            arm.move_tcp(*p, PITCH, roll, grip)
        else:
            arm.move(M.joints_for(p, PITCH, roll, grip, gravity), spd=0, acc=0)

    def run(mode, phase, a, b):
        """Stream a -> b while reading the tip after every command, then hold."""
        t0 = time.perf_counter()
        while True:
            t = time.perf_counter() - t0
            u = min(1.0, t / MOVE_TIME)
            cmd = a + (b - a) * M.min_jerk(u)
            send(mode, cmd)
            trace.append({"mode": mode, "phase": phase, "t": t, "cmd": cmd.tolist(), "meas": tip().tolist()})
            if u >= 1.0 and t > MOVE_TIME + HOLD:
                return

    try:
        arm.reset_pid()
        M.go_to_pose(arm, TOP, PITCH, roll, grip, M.rate_for(arm))
        for mode in ("stock", "tuned"):
            if mode == "tuned":
                M.stream(lambda q: arm.move(q, spd=0, acc=0), M.joints_for(TOP, PITCH, roll, grip),
                         M.joints_for(TOP, PITCH, roll, grip, gravity), 0.5, M.rate_for(arm))
                tuning.apply_gains(arm, gains)
            for i in range(DIPS):
                run(mode, "down", TOP, BOTTOM)
                run(mode, "up", BOTTOM, TOP)
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e}")
    finally:
        arm.reset_pid()
        arm.close()

    print("\nMeasured minus commanded tip (mm). radial: + away from base, - toward it")
    print(f"{'mode':6s} {'move':5s}  {'radial during':>21s}  {'radial at rest':>14s}  {'sideways at rest':>16s}  {'vertical at rest':>16s}")
    summary = {}
    for mode in ("stock", "tuned"):
        for phase in ("down", "up"):
            rows = [r for r in trace if r["mode"] == mode and r["phase"] == phase]
            if not rows:
                continue
            comp = np.array([components(np.array(r["cmd"]), r["meas"]) for r in rows])
            moving = np.array([r["t"] <= MOVE_TIME for r in rows])
            rest = comp[~moving] if (~moving).any() else comp[-1:]
            s = {"radial_min": float(comp[moving, 0].min()), "radial_max": float(comp[moving, 0].max()),
                 "rest": rest.mean(0).round(2).tolist()}
            summary[f"{mode}_{phase}"] = s
            print(f"{mode:6s} {phase:5s}  {s['radial_min']:+6.2f} .. {s['radial_max']:+6.2f}      "
                  f"{s['rest'][0]:+8.2f}        {s['rest'][1]:+8.2f}          {s['rest'][2]:+8.2f}")
    OUT.mkdir(exist_ok=True)
    path = OUT / time.strftime("track_%Y%m%d_%H%M%S.json")
    path.write_text(json.dumps({"summary": summary, "trace": trace}, indent=1))
    print(f"\nSaved the full trace to {path}")


if __name__ == "__main__":
    main()
