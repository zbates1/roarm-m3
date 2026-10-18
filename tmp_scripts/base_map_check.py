"""Does the per-angle base map help? (roarm-m3 firmware, USB). No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR. Stock gains.
Builds the base map from the latest *_base_band_fine run (each angle's average
landing error from below and from above, T:105 readings, both passes), adds it to
the saved compensation, then lands on friction_tune.py's check poses plus two
more inside +-10 deg, from above and from below, with the saved compensation
("comp", fitted base term) and with the map ("map"), alternating which goes first.
If the map is better on average and not worse from either side, the compensation
with the map is saved to roarm/tuning.json.
Results: experiments/<date>/<time>_base_map_check/.

    uv run python tmp_scripts/base_map_check.py
"""
import glob
import json
import math
import time
from pathlib import Path

import numpy as np

from roarm import experiments
from roarm import motion as M
from roarm import tuning
from roarm.arm import RoArm

import friction_tune as ft

POSES = ft.CHECK + [(190, 10, -30), (185, -8, -35)]


def base_map_from(run):
    d = json.loads((run / "data.json").read_text())
    rows = {}
    for s in d["sweep"]:
        for r in s["rows"]:
            rows.setdefault(s["deg"], {-1: [], 1: []})[r["side"]].append(r["err_state_counts"])
    deg = sorted(rows)
    below = [float(np.mean(rows[g][-1])) for g in deg]
    above = [float(np.mean(rows[g][1])) for g in deg]
    return {"deg": deg, "centre": [round((a + b) / 2, 2) for a, b in zip(above, below)],
            "half": [round((a - b) / 2, 2) for a, b in zip(above, below)],
            "source": run.relative_to(experiments.ROOT.parent).as_posix()}


def sideways_mm(pose, err):
    th = math.atan2(pose[1], pose[0])
    return float(err[0] * -math.sin(th) + err[1] * math.cos(th))


def main():
    fine = sorted(glob.glob(str(experiments.ROOT / "*" / "*_base_band_fine")))
    if not fine:
        raise SystemExit("No base map yet (run base_band.py --fine).")
    saved = tuning.load_compensation()
    bmap = base_map_from(Path(fine[-1]))
    plain = M.Compensation(saved.coef, saved.residual, saved.load_friction)
    mapped = M.Compensation(saved.coef, saved.residual, saved.load_friction, bmap)
    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB.")
    run = experiments.new_run("base_map_check")
    log = {"map": bmap, "check": []}
    print(f"Base map from {bmap['source']}. Results -> {run}", flush=True)
    try:
        arm.reset_pid()
        rig = ft.Rig(arm, None)
        rig.comps = {"comp": plain, "map": mapped}
        M.go_to_pose(arm, ft.START, ft.PITCH, rig.roll, rig.grip, rig.rate)
        rig.here = ft.START.copy()
        print("pose               base   from   comp: dist / sideways   map: dist / sideways (mm)", flush=True)
        n = 0
        for pose in POSES:
            for approach in ("above", "below"):
                row = {"pose": pose, "approach": approach}
                for mode in (("comp", "map") if n % 2 == 0 else ("map", "comp")):
                    rig.set_mode(mode)
                    _, _, _, tip = rig.land(pose, approach)
                    err = tip - np.array(pose, float)
                    row[mode] = {"err_mm": err.round(2).tolist(), "dist_mm": round(float(np.linalg.norm(err)), 2),
                                 "sideways_mm": round(sideways_mm(pose, err), 2)}
                n += 1
                log["check"].append(row)
                deg = math.degrees(math.atan2(pose[1], pose[0]))
                print(f"{str(pose):18s} {deg:+5.1f}  {approach:5s}  {row['comp']['dist_mm']:5.2f} / {row['comp']['sideways_mm']:+5.2f}"
                      f"          {row['map']['dist_mm']:5.2f} / {row['map']['sideways_mm']:+5.2f}", flush=True)
        summary = {}
        for mode in ("comp", "map"):
            for approach in ("above", "below"):
                rows = [r[mode] for r in log["check"] if r["approach"] == approach]
                summary[f"{mode}_{approach}"] = round(float(np.mean([r["dist_mm"] for r in rows])), 2)
                summary[f"{mode}_{approach}_sideways"] = round(float(np.mean([abs(r["sideways_mm"]) for r in rows])), 2)
        log["summary"] = summary
        print(f"Average (mm): {summary}", flush=True)
        better = (summary["map_above"] + summary["map_below"] < summary["comp_above"] + summary["comp_below"]
                  and summary["map_above"] <= summary["comp_above"] + 0.2 and summary["map_below"] <= summary["comp_below"] + 0.2)
        if better:
            tuning.save_compensation(mapped, source=run.relative_to(experiments.ROOT.parent).as_posix())
            print(f"The map helps; saved the compensation with it to {tuning.PATH}", flush=True)
        else:
            print("The map didn't clearly help; tuning.json left unchanged", flush=True)
        log["saved"] = better
        rig.set_mode("plain")
        rig.move(rig.here, np.array([180.0, 0.0, -25.0]))
        log["finished"] = "ok"
    except (ft.Abort, KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e!r}", flush=True)
        log["finished"] = f"stopped: {e!r}"
    finally:
        try:
            arm.reset_pid()
        finally:
            arm.close()
        (run / "data.json").write_text(json.dumps(log, indent=1))
        print(f"Saved {run / 'data.json'}", flush=True)


if __name__ == "__main__":
    main()
