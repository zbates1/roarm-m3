"""Fit and test gravity + friction compensation at stock gains (P=16). No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR (x 150-245, y +-35, z -70..0 mm).
Stock gains because higher P made motion rough (smooth_test.py ratings).

1. Data: plain commands (no feedforward). Land on 12 grid poses twice, once
   arriving from above-left and once from below-right (25 mm approaches, so each
   joint fully breaks free of friction), so every joint arrives from opposite
   directions. Record each joint's shortfall.
2. Fit three models from that data:
   gravity-only (GravityModel, from the arrivals from above, as before)
   gravity + friction (Compensation: lever-arm terms + direction * friction)
   gravity + load-scaled friction (friction also grows with the lever arm)
3. Check on 5 other poses, arriving from above and from below, all three.
4. Rise test: 20 mm straight up at x=190, the move that stops ~13 mm short.

Saves the better friction model (and stock gains) to roarm/tuning.json only if it
beats gravity-only on arrivals from below without getting worse from
above. Stops on firmware warnings, a load over 85 %, supply outside 10.5-13 V,
or a motor turning off. Results: tmp_scripts/results/friction_<time>.json.

    uv run python tmp_scripts/friction_tune.py      # ~7 min; Ctrl+C stops it
"""
import json
import math
import time
import traceback
from pathlib import Path

import numpy as np

from roarm import kinematics as K
from roarm import motion as M
from roarm import tuning
from roarm.arm import RoArm

PITCH = math.pi / 2
COUNT = M.COUNT
START = np.array([190.0, 0.0, -20.0])
GRID = [(x, 0.0, z) for x in (160, 185, 210, 235) for z in (-45, -30, -15)]
CHECK = [(175, 25, -45), (225, -25, -30), (200, 30, -20), (170, -20, -40), (215, 15, -35)]
# 25 mm approaches: at stock gains the shoulder's friction band is ~9 mm at the tip, so a
# shorter approach may arrive before the joint has fully broken free of friction.
APPROACH = {"above": np.array([0.0, 10.0, 25.0]), "below": np.array([0.0, -10.0, -25.0])}
RISE = (np.array([190.0, 0.0, -60.0]), np.array([190.0, 0.0, -40.0]))
OUT = Path(__file__).resolve().parent / "results"


class Abort(Exception):
    pass


def say(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


class Rig:
    def __init__(self, arm, gravity):
        fb = self.check(arm.feedback(), arm)
        self.arm, self.rate, self.roll, self.grip = arm, M.rate_for(arm), fb["r"], fb["g"]
        self.gravity, self.mode, self.here = gravity, "plain", None
        self.comps = {}
        self.mover = M.Mover(arm, None, PITCH, self.roll, self.grip, self.rate)

    @staticmethod
    def check(fb, arm):
        if not 1050 <= fb["v"] <= 1300:
            raise Abort(f"supply {fb['v'] / 100:.2f} V")
        if not all(fb[f"torswitch{j}"] for j in "BSERTG"):
            raise Abort("a motor turned off")
        loads = {k: fb[k] for k in ("tB", "tS", "tE", "tT", "tR", "tG")}
        if max(abs(v) for v in loads.values()) > 850:
            raise Abort(f"high load {loads}")
        if getattr(arm.link, "warnings", None):
            raise Abort(f"arm warning: {arm.link.warnings[-1]}")
        return fb

    def measure(self):
        fb = self.check(self.arm.feedback(), self.arm)
        q = np.array([fb["b"], fb["s"], fb["e"], fb["t"]])
        return q, np.array(K.fk(*q)[:3])

    def command(self, p, mode=None):
        mode = mode or self.mode
        if mode in self.comps:
            q = M.joints_for(p, PITCH, self.roll, self.grip)
            q[:4] += self.comps[mode].predict(q[:4], self.mover.dir)
            return q
        return M.joints_for(p, PITCH, self.roll, self.grip, self.gravity if mode == "gravity" else None)

    def set_mode(self, mode):
        """Blend from the old mode's command to the new one's, holding still."""
        if mode != self.mode:
            M.stream(lambda q: self.arm.move(q, spd=0, acc=0),
                     self.command(self.here), self.command(self.here, mode), 0.5, self.rate)
            self.mode = mode
            self.mover.comp = self.comps.get(mode)
            time.sleep(0.3)
            self.measure()

    def move(self, a, b):
        if self.mode in self.comps:
            self.mover.move(a, b)
        else:
            M.move_tcp_joints(self.arm, a, b, PITCH, self.roll, self.grip, self.rate,
                              gravity=self.gravity if self.mode == "gravity" else None)
        self.here = np.asarray(b, float)

    def land(self, target, approach):
        """Clear path to target + approach offset, then the final approach onto target.
        Returns plain commanded joints, each joint's direction on arrival, measured
        joints and measured tip."""
        target = np.asarray(target, float)
        start = target + APPROACH[approach]
        z_safe = max(self.here[2], start[2], target[2]) + 5
        path = [self.here.copy(), np.array([*self.here[:2], z_safe]), np.array([*start[:2], z_safe]), start]
        for a, b in zip(path, path[1:]):
            if np.linalg.norm(b - a) > 0.01:
                self.move(a, b)
        self.move(start, target)
        q_from = M.joints_for(start, PITCH, self.roll, self.grip)[:4]
        q_to = M.joints_for(target, PITCH, self.roll, self.grip)[:4]
        time.sleep(0.8)
        q, tip = self.measure()
        dirs = np.where(np.abs(q_to - q_from) > 0.5 * COUNT, np.sign(q_to - q_from), 0.0)
        return q_to, dirs, q, tip


def main():
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    args = ap.parse_args()
    gains = {n: 16 for n in tuning.JOINT_IDS}   # stock: higher P made motion rough
    OUT.mkdir(exist_ok=True)
    path = OUT / time.strftime("friction_%Y%m%d_%H%M%S.json")
    log = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "gains": gains}
    arm = RoArm(args.arm)
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    rig = None
    try:
        arm.reset_pid()
        rig = Rig(arm, None)
        M.go_to_pose(arm, START, PITCH, rig.roll, rig.grip, rig.rate)
        rig.here = START.copy()
        rig.measure()

        say("Data: landing on each grid pose from above-left and from below-right (plain commands)")
        Q, D, E, A = [], [], [], []
        for pose in GRID:
            for approach in ("above", "below"):
                q_cmd, dirs, q, _ = rig.land(pose, approach)
                Q.append(q_cmd)
                D.append(dirs)
                E.append(q_cmd - q)
                A.append(approach)
                say(f"  {pose} from {approach:5s}: shortfall (counts) base {E[-1][0] / COUNT:+5.1f}  "
                    f"shoulder {E[-1][1] / COUNT:+5.1f}  elbow {E[-1][2] / COUNT:+5.1f}  wrist {E[-1][3] / COUNT:+5.1f}")
        Q, D, E = np.array(Q), np.array(D), np.array(E)
        comp = M.Compensation.fit(Q, D, E)
        comp_load = M.Compensation.fit(Q, D, E, load_friction=True)
        above = np.array([a == "above" for a in A])
        gravity = M.GravityModel.fit(Q[above][:, 1:4], E[above][:, 1:4])
        log["data"] = {"commanded": Q.tolist(), "directions": D.tolist(), "shortfall": E.tolist(), "approach": A}
        log["model"] = comp.to_dict()
        log["model_load"] = comp_load.to_dict()
        log["gravity_model"] = gravity.to_dict()
        say(f"Friction per joint (counts): {comp.friction_counts()}")
        say(f"Fit residual (counts): { {n: round(r / COUNT, 2) for n, r in comp.residual.items()} }")
        say(f"Load-scaled friction fit residual (counts): { {n: round(r / COUNT, 2) for n, r in comp_load.residual.items()} }")
        rig.gravity, rig.comps = gravity, {"comp": comp, "compL": comp_load}

        modes = ["gravity", "comp", "compL"]
        say("Check on other poses: gravity-only vs +friction vs +load-scaled friction, from above and below")
        rows = []
        for i, pose in enumerate(CHECK):
            for approach in ("above", "below"):
                row = {"pose": pose, "approach": approach}
                for mode in modes[i % 3:] + modes[:i % 3]:
                    rig.set_mode(mode)
                    _, _, _, tip = rig.land(pose, approach)
                    err = tip - pose
                    row[mode] = {"err_mm": err.round(2).tolist(), "dist_mm": round(float(np.linalg.norm(err)), 2)}
                say(f"  {pose} from {approach:5s}: gravity-only {row['gravity']['dist_mm']:.2f}  +friction "
                    f"{row['comp']['dist_mm']:.2f}  +load-scaled {row['compL']['dist_mm']:.2f} mm "
                    f"(heights {row['gravity']['err_mm'][2]:+.2f} / {row['comp']['err_mm'][2]:+.2f} / {row['compL']['err_mm'][2]:+.2f})")
                rows.append(row)
        log["check"] = rows

        say("Rise test: 20 mm straight up at x=190")
        rise = []
        for mode in modes + modes:
            rig.set_mode(mode)
            rig.land(RISE[0], "above")
            rig.move(RISE[0], RISE[1])
            time.sleep(1.0)
            _, tip = rig.measure()
            err = tip - RISE[1]
            r_hat = RISE[1][:2] / np.linalg.norm(RISE[1][:2])
            rise.append({"mode": mode, "err_mm": err.round(2).tolist(), "radial_mm": round(float(err[:2] @ r_hat), 2)})
            say(f"  {mode:7s}: ends {np.linalg.norm(err):.2f} mm off (height {err[2]:+.2f}, radial {rise[-1]['radial_mm']:+.2f})")
        log["rise"] = rise

        summary = {}
        for mode in modes:
            for approach in ("above", "below"):
                d = [r[mode]["dist_mm"] for r in rows if r["approach"] == approach]
                summary[f"{mode}_{approach}"] = round(float(np.mean(d)), 2)
            summary[f"{mode}_rise"] = round(float(np.mean([np.linalg.norm(r["err_mm"]) for r in rise if r["mode"] == mode])), 2)
        log["summary"] = summary
        say(f"Average distance off (mm): {summary}")
        score = {m: summary[f"{m}_above"] + summary[f"{m}_below"] + summary[f"{m}_rise"] for m in ("comp", "compL")}
        best = min(score, key=score.get)
        better = (summary[f"{best}_below"] < summary["gravity_below"] and summary[f"{best}_above"] <= summary["gravity_above"] + 0.2)
        log["best"] = best
        if better:
            tuning.save(gains, gravity, source=path.name, compensation=rig.comps[best])
            say(f"Best: {'load-scaled friction' if best == 'compL' else 'constant friction'}; saved it (with stock gains) to {tuning.PATH}")
        else:
            say("Gravity + friction didn't clearly beat gravity-only; tuning.json left unchanged")
        log["saved"] = better
        rig.set_mode("plain")
        rig.move(rig.here, np.array([180.0, 0.0, -25.0]))
        log["finished"] = "ok"
    except Abort as e:
        say(f"STOPPED for safety: {e}")
        log["finished"] = f"stopped: {e}"
    except Exception as e:
        say(f"STOPPED on error: {e!r}")
        log["finished"] = f"error: {e!r}"
        log["traceback"] = traceback.format_exc()
    finally:
        try:
            arm.reset_pid()
        except Exception as e:
            log["cleanup_error"] = repr(e)
        arm.close()
        path.write_text(json.dumps(log, indent=1))
        say(f"Gains reset to P=16. Results: {path}")


if __name__ == "__main__":
    main()
