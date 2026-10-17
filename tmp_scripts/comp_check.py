"""Does gravity + friction compensation stop the inward drift? Telemetry check. No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR. Stock gains (P=16, D=32).
Same moves and analysis as pd_sweep.py (down 20, up 20, 30 mm left, back), run
alternately with plain commands and with the compensation in roarm/tuning.json.
Results: experiments/<date>/<time>_comp_check/.

    uv run python tmp_scripts/comp_check.py
"""
import json
import time

import numpy as np

from roarm import experiments
from roarm import motion as M
from roarm import tuning
from roarm.arm import RoArm

import pd_sweep as ps

ROUNDS = 2


def main():
    comp = tuning.load_compensation()
    if comp is None:
        raise SystemExit("No compensation in roarm/tuning.json (run friction_tune.py).")
    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    run = experiments.new_run("comp_check")
    (run / "trials").mkdir()
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)
    mover = M.Mover(arm, comp, ps.PITCH, roll, grip, rate)
    summary = []
    print(f"Results -> {run}", flush=True)
    try:
        arm.reset_pid()
        M.go_to_pose(arm, ps.READY, ps.PITCH, roll, grip, rate)
        mode = "plain"
        for n in range(2 * ROUNDS):
            want = "plain" if n % 2 == 0 else "compensated"
            if want != mode:  # blend the feedforward in or out while holding still
                plain_q = M.joints_for(ps.READY, ps.PITCH, roll, grip)
                comp_q = mover.command(ps.READY, mover.dir)
                a, b = (plain_q, comp_q) if want == "compensated" else (comp_q, plain_q)
                M.stream(lambda q: arm.move(q, spd=0, acc=0), a, b, 0.5, rate)
                mode = want
                time.sleep(0.5)
            cmd_log, windows = [], {}
            arm.telemetry_start()
            time.sleep(0.3)
            for name, a, b, hold in ps.MOVES:
                t_start = time.perf_counter()
                if mode == "plain":
                    ps.stream_logged(arm, a, b, roll, grip, rate, cmd_log)
                else:
                    mover.move(a, b, speed=ps.SPEED, log=cmd_log)
                t_end = time.perf_counter()
                time.sleep(hold)
                windows[name] = (t_start, t_end, time.perf_counter())
            tel = arm.telemetry_stop()
            if arm.link.warnings:
                raise RuntimeError(f"firmware warning: {arm.link.warnings[-1]}")
            res = ps.analyse(tel, cmd_log, windows)
            np.savez_compressed(run / "trials" / f"{n:02d}_{mode}.npz", cmd=np.array(cmd_log), **tel)
            summary.append({"trial": n, "mode": mode, "moves": res})
            print(f"{mode:12s}: radial shift up/aside/back "
                  f"{[res[m]['radial_shift_mm'] for m in ('up', 'aside', 'back')]} mm | at rest after up: radial "
                  f"{res['up']['rest_radial_mm']} vertical {res['up']['rest_vertical_mm']} mm | after back: radial "
                  f"{res['back']['rest_radial_mm']} sideways {res['back']['rest_sideways_mm']} vertical {res['back']['rest_vertical_mm']}",
                  flush=True)
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e}", flush=True)
        summary.append({"stopped": str(e)})
    finally:
        arm.reset_pid()
        arm.close()
        (run / "summary.json").write_text(json.dumps(summary, indent=1))
        print(f"Saved {run / 'summary.json'}", flush=True)


if __name__ == "__main__":
    main()
