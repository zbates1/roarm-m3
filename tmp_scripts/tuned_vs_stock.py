"""End-to-end check: stock control vs tuned (higher P + gravity feedforward).

Dips onto the four corners of a 20 mm square around the dip spot, alternating
the two modes, so drift over the run affects both the same way:
  stock: P=16 and the firmware's own IK (T:1041), as in the earlier tests
  tuned: gains + gravity model from roarm/tuning.json, joints solved here (T:102)
Each dip: swing 40 mm aside and back at the top, descend 20 mm onto the corner,
measure, rise again. Reports how far the tip lands from each corner, from the
arm's joint sensors.

No prompts: assumes the space around the ready pose (x=180, z=-40) is clear.

    uv run python tmp_scripts/tuned_vs_stock.py              # 3 rounds, ~4 min
    uv run python tmp_scripts/tuned_vs_stock.py --rounds 5
"""
import argparse
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
DIP = 20.0
ASIDE = np.array([0.0, 40.0, 0.0])
CORNERS = [READY + [0, 0, -DIP] + [f, l, 0] for f, l in ((0, 0), (0, 20), (20, 20), (20, 0))]
OUT = experiments.day_dir()  # experiments/<today>/


def tip(arm):
    fb = arm.feedback()
    if fb["v"] < 1050 or not all(fb[f"torswitch{j}"] for j in "BSERTG") or getattr(arm.link, "warnings", None):
        raise RuntimeError(f"safety stop: supply {fb['v'] / 100:.2f} V, warnings {getattr(arm.link, 'warnings', None)}")
    return np.array(K.fk(fb["b"], fb["s"], fb["e"], fb["t"])[:3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--rounds", type=int, default=3)
    args = ap.parse_args()

    gains, gravity = tuning.load()
    for c in CORNERS:
        top = c + [0, 0, DIP]
        if M.check_line(top, c, PITCH) or M.check_line(top, top + ASIDE, PITCH):
            raise SystemExit(f"corner {c} path would hit a joint limit")
    arm = RoArm(args.arm)
    rate = M.rate_for(arm)
    fb = arm.feedback()
    roll, grip = fb["r"], fb["g"]
    print(f"Tuned gains {gains}; comparing {args.rounds} rounds x 4 corners x 2 modes", flush=True)

    def move(mode, a, b, label):
        if mode == "stock":
            M.move_tcp(arm, a, b, PITCH, roll, grip, rate)
        else:
            M.move_tcp_joints(arm, a, b, PITCH, roll, grip, rate, gravity=gravity)
        try:
            tip(arm)
        except RuntimeError as e:
            raise RuntimeError(f"after '{label}' ({mode}): {e}") from None

    def switch(to, at):
        """Change modes while holding at `at`: blend the feedforward offset in or
        out over 0.5 s at stock gains, and ramp the gains rather than jumping."""
        plain = M.joints_for(at, PITCH, roll, grip)
        ff = M.joints_for(at, PITCH, roll, grip, gravity)
        if to == "tuned":
            M.stream(lambda q: arm.move(q, spd=0, acc=0), plain, ff, 0.5, rate)
            tuning.apply_gains(arm, gains)
        else:
            arm.reset_pid()
            M.stream(lambda q: arm.move(q, spd=0, acc=0), ff, plain, 0.5, rate)
        time.sleep(0.3)
        try:
            tip(arm)
        except RuntimeError as e:
            raise RuntimeError(f"switching to {to}: {e}") from None

    results = {"stock": [], "tuned": []}
    try:
        arm.reset_pid()
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        here, current = READY.copy(), "stock"
        for r in range(args.rounds):
            for k, corner in enumerate(CORNERS):
                for mode in (("stock", "tuned") if (r + k) % 2 == 0 else ("tuned", "stock")):
                    top = corner + [0, 0, DIP]
                    move(current, here, top, "to the corner's top")
                    if mode != current:
                        switch(mode, top)
                        current = mode
                    move(mode, top, top + ASIDE, "aside")
                    move(mode, top + ASIDE, top, "back")
                    move(mode, top, corner, "down")
                    time.sleep(0.6)
                    err = tip(arm) - corner
                    results[mode].append({"corner": k, "err": err.tolist()})
                    print(f"round {r + 1} corner {k + 1} {mode:5s}: off by {np.linalg.norm(err):.2f} mm "
                          f"(x {err[0]:+.2f} y {err[1]:+.2f} z {err[2]:+.2f})", flush=True)
                    move(mode, corner, top, "up")
                    here = top
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e}")
    finally:
        arm.reset_pid()
        arm.close()

    summary = {}
    for mode, rows in results.items():
        if not rows:
            continue
        e = np.array([row["err"] for row in rows])
        per_corner = [np.array([row["err"] for row in rows if row["corner"] == k]) for k in range(4)]
        spread = np.mean([np.linalg.norm(c - c.mean(0), axis=1).max() for c in per_corner if len(c) > 1] or [0])
        summary[mode] = {"mean_dist_mm": float(np.linalg.norm(e, axis=1).mean()), "mean_z_mm": float(e[:, 2].mean()),
                         "mean_xy_mm": float(np.linalg.norm(e[:, :2], axis=1).mean()), "repeat_spread_mm": float(spread)}
        s = summary[mode]
        print(f"\n{mode}: {len(rows)} dips, off by {s['mean_dist_mm']:.2f} mm on average "
              f"(height {s['mean_z_mm']:+.2f}, sideways {s['mean_xy_mm']:.2f}); "
              f"repeat spread {s['repeat_spread_mm']:.2f} mm")
    OUT.mkdir(exist_ok=True)
    path = OUT / time.strftime("tuned_vs_stock_%Y%m%d_%H%M%S.json")
    path.write_text(json.dumps({"gains": gains, "results": results, "summary": summary}, indent=1))
    print(f"Saved {path}")


if __name__ == "__main__":
    main()
