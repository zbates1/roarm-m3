"""Autonomous control experiments for the RoArm-M3. No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR. All motion stays within
about 60 mm of the ready pose (x=180, z=-40 mm, gripper straight down).

Phases:
  A. per-joint step test at stock gains (P=16): step the base, shoulder, elbow
     and wrist a few encoder counts up and down; measures sag and friction band
  B. per-joint P sweep: raise each joint's P until it oscillates, keep the
     highest value below that
  C. motion check of the tuned gains after real moves; back off if they ring
  D. stock vs tuned: landing error from above, 3 mm step up, jitter
  E. gravity feedforward: measure joint sag over a grid of poses (tuned gains,
     always approached from above), fit a per-joint model, check it on other poses
  F. per-joint step test again at the tuned gains

Stops (resets gains, arm holds) on: firmware overcurrent/voltage warnings, a load
over 85 %, supply outside 10.5-13 V, a motor turning off. Always resets the
gains to P=16, I=0 at the end. Writes tmp_scripts/results/autotune_<time>.json.

    uv run python tmp_scripts/autotune.py          # USB, ~10-15 min
"""
import json
import math
import sys
import time
import traceback
from pathlib import Path

import numpy as np

from roarm import kinematics as K
from roarm import motion as M
from roarm.arm import RoArm

COUNT = 2 * math.pi / 4096                 # one encoder step (rad)
READY = np.array([180.0, 0.0, -40.0])
PITCH = math.pi / 2
JOINTS = {"base": (1, 0), "shoulder": (2, 1), "elbow": (3, 2), "wrist": (4, 3)}  # name: (T:108 id, state index)
STEPS = [0, 2, 4, 8, 16, 32, 16, 8, 4, 2, 0, -2, -4, -8, -16, -32, -16, -8, -4, -2, 0]  # counts
P_VALUES = [16, 24, 32, 40, 48]
OSC_STD, OSC_PTP = 0.8, 3                  # counts; holding still above either = oscillating
GRID = [(x, 0.0, z) for x in (160, 185, 210, 235) for z in (-65, -40, -15)]
CHECK = [(175, 25, -55), (225, -25, -30), (200, 30, -20), (170, -20, -60), (215, 15, -50)]
ABOVE = 15.0                               # mm: every pose is reached by descending this far
OUT = Path(__file__).resolve().parent / "results"


class Abort(Exception):
    pass


def say(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


class Lab:
    def __init__(self, arm):
        self.arm, self.rate = arm, M.rate_for(arm)
        fb = self.check(arm.feedback())
        self.roll, self.grip = fb["r"], fb["g"]
        self.q_cmd = np.array([fb[k] for k in "bsetrg"])
        self.pos = None          # last commanded tip position (mm)
        self.gravity = None      # M.GravityModel, applied by ik6 when set
        self.gains = {n: 16 for n in JOINTS}

    # --- safety + measurement ---------------------------------------------
    def check(self, fb):
        if not 1050 <= fb["v"] <= 1300:
            raise Abort(f"supply {fb['v'] / 100:.2f} V")
        if not all(fb[f"torswitch{j}"] for j in "BSERTG"):
            raise Abort("a motor turned off")
        loads = {k: fb[k] for k in ("tB", "tS", "tE", "tT", "tR", "tG")}
        if max(abs(v) for v in loads.values()) > 850:
            raise Abort(f"high load {loads}")
        if getattr(self.arm.link, "warnings", None):
            raise Abort(f"arm warning: {self.arm.link.warnings[-1]}")
        return fb

    def joints(self):
        fb = self.check(self.arm.feedback())
        return np.array([fb[k] for k in "bsetrg"])

    def samples(self, n=15):
        return np.array([self.joints() for _ in range(n)])

    # --- motion -------------------------------------------------------------
    def send(self, q, duration=0.3):
        q = np.asarray(q, float)
        M.stream(lambda v: self.arm.move(v, spd=0, acc=0), self.q_cmd, q, duration, self.rate)
        self.q_cmd = q

    def ik6(self, xyz):
        j = K.ik(*xyz, PITCH)
        q = np.array([j["base"], j["shoulder"], j["elbow"], j["wrist"], self.roll, self.grip])
        if self.gravity is not None:
            q[1:4] += self.gravity.predict(q[1:4])
        return q

    def line(self, a, b, speed=15.0):
        """Straight tip line a -> b, streamed as joint commands (so feedforward applies)."""
        if M.check_line(a, b, PITCH):
            raise Abort(f"line {np.round(a)} -> {np.round(b)} would hit a joint limit")
        a, b = np.asarray(a, float), np.asarray(b, float)
        duration = max(0.6, np.linalg.norm(b - a) / speed)
        M.stream(lambda p: self.arm.move(self.ik6(p), spd=0, acc=0), a, b, duration, self.rate)
        self.q_cmd, self.pos = self.ik6(b), b

    def go(self, target):
        """Up ABOVE mm, across, then down onto target, so every pose is reached from above."""
        target = np.asarray(target, float)
        up = np.array([0, 0, ABOVE])
        if self.pos is None:
            M.go_to_pose(self.arm, target + up, PITCH, self.roll, self.grip, self.rate)
            self.q_cmd, self.pos = self.ik6(target + up), target + up
        else:
            if self.pos[2] < target[2] + ABOVE:
                self.line(self.pos, self.pos + [0, 0, target[2] + ABOVE - self.pos[2]])
            self.line(self.pos, target + up)
        self.line(self.pos, target)

    def set_gain(self, name, p):
        self.arm.set_pid(JOINTS[name][0], p, 0)
        self.gains[name] = p

    def set_gains(self, gains):
        for name, p in gains.items():
            self.set_gain(name, p)
        time.sleep(0.8)

    # --- experiments --------------------------------------------------------
    def step_test(self, label):
        say(f"Step test ({label}): {self.gains}")
        self.go(READY)
        q0, out = self.q_cmd.copy(), {}
        for name, (_, i) in JOINTS.items():
            self.send(q0, 0.8)
            time.sleep(0.8)
            meas = []
            for d in STEPS:
                q = q0.copy()
                q[i] += d * COUNT
                self.send(q, 0.25)
                time.sleep(0.8)
                meas.append((self.joints()[i] - q0[i]) / COUNT)
            self.send(q0, 0.25)
            meas = np.array(meas)
            first_move = lambda run, ref: next((STEPS[k] for k in run if abs(meas[k] - meas[ref]) >= 1), None)
            out[name] = {
                "commanded": STEPS, "measured": meas.round(2).tolist(),
                "sag_counts": round(-meas[0], 2),
                "band_counts": round(meas[10] - meas[20], 2),
                "first_move_up": first_move(range(1, 6), 0),
                "first_move_down": first_move(range(11, 16), 10),
                "lag_at_+32": round(32 - meas[5], 2), "lag_at_-32": round(-32 - meas[15], 2),
            }
            r = out[name]
            say(f"  {name:8s} sag {r['sag_counts']:+.1f}  band {r['band_counts']:.1f}  "
                f"first move +{r['first_move_up']} / {r['first_move_down']}  "
                f"lag at +32: {r['lag_at_+32']:+.1f}, at -32: {r['lag_at_-32']:+.1f}  (counts)")
        return out

    def p_sweep(self):
        say("P sweep, one joint at a time")
        self.go(READY)
        q0, out, chosen = self.q_cmd.copy(), {}, {}
        for name, (_, i) in JOINTS.items():
            rows, best = [], 16
            for p in P_VALUES:
                self.set_gain(name, p)
                time.sleep(0.8)
                for d in (8, -8):
                    q = q0.copy()
                    q[i] += d * COUNT
                    self.send(q, 0.2)
                    time.sleep(0.5)
                    self.send(q0, 0.2)
                    time.sleep(0.3)
                s = self.samples(15)[:, [v[1] for v in JOINTS.values()]] / COUNT
                std, ptp = s.std(0).max(), np.ptp(s, 0).max()
                lag = q0[i] / COUNT - s[:, list(JOINTS).index(name)].mean()
                ok = std <= OSC_STD and ptp <= OSC_PTP
                rows.append({"P": p, "std": round(float(std), 2), "ptp": round(float(ptp), 2),
                             "lag_counts": round(float(lag), 2), "stable": bool(ok)})
                say(f"  {name:8s} P={p:2d}: holding std {std:.2f}, max-min {ptp:.1f}, lag {lag:+.1f} counts"
                    + ("" if ok else "  <- oscillating"))
                if not ok:
                    break
                best = p
            chosen[name] = best
            self.set_gain(name, 16)
            time.sleep(0.5)
            out[name] = rows
        return out, chosen

    def ringing(self):
        """Worst holding std / max-min (counts) after a few real moves."""
        worst = (0.0, 0.0)
        for target in (READY + [25, 0, 0], READY + [0, 25, -20], READY):
            self.go(target)
            time.sleep(0.3)
            s = self.samples(15)[:, [v[1] for v in JOINTS.values()]] / COUNT
            worst = (max(worst[0], float(s.std(0).max())), max(worst[1], float(np.ptp(s, 0).max())))
        return worst

    def motion_check(self, gains):
        tries = []
        for _ in range(4):
            self.set_gains(gains)
            std, ptp = self.ringing()
            ok = std <= OSC_STD and ptp <= OSC_PTP
            tries.append({"gains": dict(gains), "std": round(std, 2), "ptp": round(ptp, 2), "stable": ok})
            say(f"Motion check {gains}: worst std {std:.2f}, max-min {ptp:.1f}" + ("" if ok else "  <- ringing, backing off"))
            if ok:
                return gains, tries
            top = max(gains, key=gains.get)
            if gains[top] <= 16:
                break
            gains = {**gains, top: P_VALUES[max(0, P_VALUES.index(gains[top]) - 1)]}
        return {n: 16 for n in JOINTS}, tries

    def landing(self, label, dips=3):
        bottom = READY - [0, 0, 20]
        errs, ups, jit = [], [], []
        for _ in range(dips):
            self.go(bottom)
            t0 = time.perf_counter()
            row = []
            for t in (0.4, 1.5):
                time.sleep(max(0.0, t0 + t - time.perf_counter()))
                q = self.joints()
                row.append(np.array(K.fk(*q[:4])[:3]) - bottom)
            errs.append(row)
            jit.append(float(np.ptp(self.samples(10)[:, 1:4] / COUNT, 0).max()))
            before = K.fk(*self.joints()[:4])[2]
            self.line(bottom, bottom + [0, 0, 3.0], speed=5.0)
            time.sleep(1.5)
            ups.append(K.fk(*self.joints()[:4])[2] - before)
        e = np.array(errs)
        res = {"z_err_0.4s": round(float(e[:, 0, 2].mean()), 2), "z_err_1.5s": round(float(e[:, 1, 2].mean()), 2),
               "xyz_err_1.5s": round(float(np.linalg.norm(e[:, 1], axis=1).mean()), 2),
               "z_spread": round(float(np.ptp(e[:, 1, 2])), 2), "step_up_followed": round(float(np.mean(ups)), 2),
               "jitter_counts": round(max(jit), 2)}
        say(f"Landing ({label}): height off {res['z_err_1.5s']:+.2f} mm (spread {res['z_spread']:.2f}), "
            f"distance off {res['xyz_err_1.5s']:.2f} mm, 3 mm step up moved {res['step_up_followed']:+.2f}, "
            f"jitter {res['jitter_counts']:.1f} counts")
        return res

    def gravity_fit(self):
        say("Gravity model: measuring sag over the grid")
        self.gravity, X, Y = None, [], []
        for pose in GRID:
            self.go(pose)
            time.sleep(0.8)
            q = self.joints()
            X.append(self.q_cmd[1:4].copy())
            Y.append(self.q_cmd[1:4] - q[1:4])
            say(f"  at {pose}: sag shoulder {Y[-1][0] / COUNT:+.1f}, elbow {Y[-1][1] / COUNT:+.1f}, "
                f"wrist {Y[-1][2] / COUNT:+.1f} counts")
        model = M.GravityModel.fit(np.array(X), np.array(Y))
        say(f"  fit residual (counts): {np.round(np.array(model.residual) / COUNT, 2).tolist()}")
        return model, {"poses": GRID, "commanded": np.array(X).tolist(), "sag": np.array(Y).tolist()}

    def gravity_check(self, model):
        say("Gravity model: checking on poses it wasn't fitted on")
        rows = []
        for pose in CHECK:
            row = {"pose": pose}
            for mode in ("plain", "feedforward"):
                self.gravity = model if mode == "feedforward" else None
                self.go(pose)
                time.sleep(0.8)
                q = self.joints()
                err = np.array(K.fk(*q[:4])[:3]) - pose
                row[mode] = {"err_mm": err.round(2).tolist(), "dist_mm": round(float(np.linalg.norm(err)), 2)}
            self.gravity = None
            say(f"  at {pose}: plain {row['plain']['dist_mm']:.2f} mm off (z {row['plain']['err_mm'][2]:+.2f}), "
                f"feedforward {row['feedforward']['dist_mm']:.2f} mm off (z {row['feedforward']['err_mm'][2]:+.2f})")
            rows.append(row)
        return rows


def main():
    import argparse

    global P_VALUES
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default=None)
    ap.add_argument("--p", type=int, nargs="+", default=P_VALUES, help="P values for the sweep, ascending")
    ap.add_argument("--skip-stock-steps", action="store_true", help="skip phase A")
    args = ap.parse_args()
    P_VALUES = args.p

    OUT.mkdir(exist_ok=True)
    path = OUT / time.strftime("autotune_%Y%m%d_%H%M%S.json")
    log = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "steps": STEPS, "p_values": P_VALUES}
    arm = RoArm(args.arm)
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    lab = None
    try:
        arm.reset_pid()
        lab = Lab(arm)
        say(f"Connected on {arm.target}; supply {arm.feedback()['v'] / 100:.2f} V")
        if not args.skip_stock_steps:
            log["A_steps_stock"] = lab.step_test("stock")
        log["B_sweep"], chosen = lab.p_sweep()
        say(f"Highest stable P per joint: {chosen}")
        tuned, log["C_motion_check"] = lab.motion_check(chosen)
        log["tuned_gains"] = tuned
        lab.set_gains({n: 16 for n in JOINTS})
        log["D_landing_stock"] = lab.landing("stock")
        lab.set_gains(tuned)
        log["D_landing_tuned"] = lab.landing("tuned")
        model, log["E_gravity_data"] = lab.gravity_fit()
        log["E_gravity_model"] = model.to_dict()
        log["E_gravity_check"] = lab.gravity_check(model)
        log["F_steps_tuned"] = lab.step_test("tuned")
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
            time.sleep(0.5)
            if lab is not None and log.get("finished") == "ok":
                lab.gravity = None
                lab.go(READY)
        except Exception as e:
            log["cleanup_error"] = repr(e)
        arm.close()
        path.write_text(json.dumps(log, indent=1))
        say(f"Gains reset to P=16. Results: {path}")


if __name__ == "__main__":
    main()
