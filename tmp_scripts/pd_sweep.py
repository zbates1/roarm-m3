"""Automated P x D sweep with high-rate telemetry (roarm-m3 firmware, USB). No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR (x 180, y 0..30, z -60..-40 mm).

Each trial sets P (stiffness) and D (damping) on the base, shoulder (both
servos), elbow and wrist, then runs: down 20 mm, up 20 mm, 30 mm to the arm's
left, back. The T:118 stream (~95 samples/s) records every joint the whole time.
Per move it reports:
  roughness  - RMS of each joint's wiggle around its own 120 ms moving average
               (encoder counts), and how often the servo's speed changes sign
  path       - measured minus commanded tip, split into radial (- = toward the
               base), sideways and vertical, during the move and once settled
Calibration: P=16/24/32 at the stock D=32 were rated by eye earlier (P=16 smooth,
24 so-so, 32 rough), so the roughness numbers can be checked against them.

A trial whose first move is far rougher than stock stops early (less wear). Any
firmware warning stops everything. Gains go back to P=16, D=32 at the end and
are read back. Results: experiments/<date>/<time>_pd_sweep/.

    uv run python tmp_scripts/pd_sweep.py
"""
import json
import math
import time

import numpy as np

from roarm import experiments
from roarm import kinematics as K
from roarm import motion as M
from roarm.arm import RoArm

READY = np.array([180.0, 0.0, -40.0])
PITCH = math.pi / 2
MOVES = [("down", READY, READY - [0, 0, 20.0], 0.8), ("up", READY - [0, 0, 20.0], READY, 0.8),
         ("aside", READY, READY + [0, 30.0, 0], 0.5), ("back", READY + [0, 30.0, 0], READY, 0.8)]
SPEED = 15.0           # mm/s average
IDS = [11, 12, 13, 14, 15]
JOINT_NAMES = ["base", "shoulder", "elbow", "wrist"]
# (P, D): stock first, the eye-rated calibration points, more damping, then stock again (drift check)
TRIALS = [(16, 32), (24, 32), (24, 64), (24, 128), (32, 32), (32, 64), (32, 128), (48, 64), (48, 128), (16, 32)]
RATED = {(16, 32): "smooth (4-5)", (24, 32): "so-so (2-3)", (32, 32): "rough (1-2)"}
COUNT = 2 * math.pi / 4096
SMOOTH_WIN = 11        # samples (~120 ms at ~95/s)
ABORT_FACTOR = 4.0     # stop a trial if its first move is this many times rougher than stock


def stream_logged(arm, a, b, roll, grip, rate, log):
    """Stream a straight line as joint commands at `rate` Hz, logging (time, commanded tip)."""
    duration = max(1.0, np.linalg.norm(b - a) / SPEED)
    t0 = time.perf_counter()
    while True:
        tick = time.perf_counter()
        u = min(1.0, (tick - t0) / duration)
        p = a + (b - a) * M.min_jerk(u)
        arm.move(M.joints_for(p, PITCH, roll, grip), spd=0, acc=0)
        log.append((tick, *p))
        if u >= 1.0:
            return
        time.sleep(max(0.0, 1 / rate - (time.perf_counter() - tick)))


def components(cmd, meas):
    """meas - cmd as (radial, sideways, vertical); radial < 0 = toward the base."""
    d = meas - cmd
    r_hat = cmd[:, :2] / np.linalg.norm(cmd[:, :2], axis=1, keepdims=True)
    t_hat = np.stack([-r_hat[:, 1], r_hat[:, 0]], axis=1)
    return np.stack([(d[:, :2] * r_hat).sum(1), (d[:, :2] * t_hat).sum(1), d[:, 2]], axis=1)


def roughness(joints):
    """Per joint: RMS (counts) of the wiggle around a centered moving average."""
    k = np.ones(SMOOTH_WIN) / SMOOTH_WIN
    out = []
    for j in range(joints.shape[1]):
        x = joints[:, j] / COUNT
        if len(x) <= SMOOTH_WIN:
            out.append(float("nan"))
            continue
        sm = np.convolve(x, k, mode="valid")
        h = SMOOTH_WIN // 2
        out.append(float(np.sqrt(np.mean((x[h:len(x) - h] - sm) ** 2))))
    return out


def sign_flips(speed, deadband=20):
    """Per joint: how many times the servo-reported speed changes sign (ignoring |v| < deadband)."""
    out = []
    for j in range(speed.shape[1]):
        s = np.sign(np.where(np.abs(speed[:, j]) < deadband, 0, speed[:, j]))
        s = s[s != 0]
        out.append(int(np.sum(s[1:] != s[:-1])))
    return out


def analyse(tel, cmd_log, windows):
    """Split the trial into moves and compute roughness + path numbers for each."""
    cmd = np.array(cmd_log)
    res = {}
    for name, (t_start, t_end, t_hold_end) in windows.items():
        sel = (tel["host_t"] >= t_start) & (tel["host_t"] <= t_hold_end)
        moving = (tel["host_t"] >= t_start) & (tel["host_t"] <= t_end)
        rest = (tel["host_t"] >= t_hold_end - 0.3) & (tel["host_t"] <= t_hold_end)
        if sel.sum() < SMOOTH_WIN + 2:
            continue
        p_cmd = np.stack([np.interp(tel["host_t"], cmd[:, 0], cmd[:, k]) for k in (1, 2, 3)], axis=1)
        comp = components(p_cmd, tel["tip"])
        rough = roughness(tel["joints"][sel])
        res[name] = {
            "roughness_counts": dict(zip(JOINT_NAMES, np.round(rough, 3).tolist())),
            "roughness_max": round(float(np.nanmax(rough)), 3),
            "speed_sign_flips": dict(zip(JOINT_NAMES, sign_flips(tel["speed"][moving]))),
            "radial_during_min_mm": round(float(comp[moving, 0].min()), 2),
            "radial_during_max_mm": round(float(comp[moving, 0].max()), 2),
            "sideways_during_maxabs_mm": round(float(np.abs(comp[moving, 1]).max()), 2),
            "rest_radial_mm": round(float(comp[rest, 0].mean()), 2) if rest.any() else None,
            # how far this move itself shifted the tip toward (-) / away from (+) the base
            "radial_shift_mm": round(float(comp[rest, 0].mean() - comp[sel, 0][0]), 2) if rest.any() else None,
            "rest_sideways_mm": round(float(comp[rest, 1].mean()), 2) if rest.any() else None,
            "rest_vertical_mm": round(float(comp[rest, 2].mean()), 2) if rest.any() else None,
            "max_load": dict(zip(JOINT_NAMES[1:], np.abs(tel["load"][sel]).max(0).astype(int).tolist())),
            "samples": int(sel.sum()),
        }
    return res


def main():
    import sys

    global TRIALS
    if "--quick" in sys.argv:  # just the stock trial, to check the pipeline
        TRIALS = TRIALS[:1]
    for _, a, b, _ in MOVES:
        if M.check_line(a, b, PITCH):
            raise SystemExit("test path would hit a joint limit")
    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    if arm.read_reg(11, 22) is None:
        raise SystemExit("No reply to T:116: flash the roarm-m3 firmware first (firmware/README.md).")
    run = experiments.new_run("pd_sweep")
    (run / "trials").mkdir()
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)
    p_now, summary, stock_rough = 16, [], None
    print(f"Results -> {run}", flush=True)

    def set_reg(addr, value):
        for i in IDS:
            arm.write_reg(i, addr, value)

    def set_p(to):
        nonlocal p_now
        for p in (list(range(p_now + 8, to, 8)) + [to]) if to > p_now else [to]:
            set_reg(21, p)
            time.sleep(0.25)
        p_now = to

    def check():
        f = arm.feedback()
        if f["v"] < 1050 or not all(f[f"torswitch{j}"] for j in "BSERTG") or arm.link.warnings:
            raise RuntimeError(f"safety stop: {f['v'] / 100:.2f} V, warnings {arm.link.warnings}")

    try:
        set_reg(21, 16)
        set_reg(22, 32)
        M.go_to_pose(arm, READY, PITCH, roll, grip, rate)
        for n, (P, D) in enumerate(TRIALS):
            set_p(P)
            set_reg(22, D)
            time.sleep(0.6)
            check()
            cmd_log, windows, aborted = [], {}, None
            arm.telemetry_start()
            time.sleep(0.3)
            for name, a, b, hold in MOVES:
                t_start = time.perf_counter()
                stream_logged(arm, a, b, roll, grip, rate, cmd_log)
                t_end = time.perf_counter()
                time.sleep(hold)
                windows[name] = (t_start, t_end, time.perf_counter())
                if name == "down" and stock_rough is not None:
                    peek = arm.link.telemetry[-int((t_end - t_start + hold) * 95):]
                    from roarm import telemetry
                    t = telemetry.parse(peek)
                    if t is not None and np.nanmax(roughness(t["joints"])) > ABORT_FACTOR * stock_rough:
                        aborted = "first move far rougher than stock; rest of trial skipped"
                        stream_logged(arm, b, READY, roll, grip, rate, cmd_log)
                        time.sleep(0.5)
                        break
            tel = arm.telemetry_stop()
            check()
            res = analyse(tel, cmd_log, windows) if tel is not None else {}
            if (P, D) == (16, 32) and stock_rough is None and "down" in res:
                stock_rough = max(res["down"]["roughness_max"], 0.05)
            np.savez_compressed(run / "trials" / f"{n:02d}_P{P}_D{D}.npz",
                                cmd=np.array(cmd_log), **{k: v for k, v in (tel or {}).items()})
            row = {"trial": n, "P": P, "D": D, "rated_by_eye": RATED.get((P, D)), "aborted": aborted, "moves": res}
            summary.append(row)
            r = [res[m]["roughness_max"] for m in ("down", "up", "aside", "back") if m in res]
            inward = [res[m]["radial_during_min_mm"] for m in ("up", "aside", "back") if m in res]
            print(f"P={P:2d} D={D:3d}: roughness {np.round(r, 2).tolist()} counts  "
                  f"most inward during up/aside/back {np.round(inward, 2).tolist()} mm"
                  + (f"  [{aborted}]" if aborted else ""), flush=True)
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e}", flush=True)
        summary.append({"stopped": str(e)})
    finally:
        try:
            set_p(16)
            set_reg(22, 32)
            gains = {i: (arm.read_reg(i, 21), arm.read_reg(i, 22)) for i in IDS}
            print(f"Gains restored (servo: (P, D)): {gains}", flush=True)
        finally:
            arm.close()
        (run / "summary.json").write_text(json.dumps(summary, indent=1))
        print(f"Saved {run / 'summary.json'}", flush=True)


if __name__ == "__main__":
    main()
