"""How do the two shoulder servos share the load? (roarm-m3 firmware, USB). No prompts.

ASSUMES THE SPACE AROUND THE READY POSE IS CLEAR (x 150-240, z -20..-5 mm).

The shoulder joint is driven by two servos from opposite sides: A (ID 12, the
"driving" one) and B (ID 13, "driven"). Their targets are mirrored about 2047, so
goal_A + goal_B = 4094. If both see the joint at the same angle, their present
positions also sum to 4094. The mismatch (present_A + present_B - 4094) says how
far apart their ideas of the joint angle are (counts; 1 count = 0.088 deg).

1. Poses at 4 reaches (gripper down, z=-20, approached from above): settle,
   then read goal / present / load / current / temperature of A and B 3 times.
   Constant mismatch across reach = a calibration offset; changing = mechanical.
2. Heat: hold the farthest pose for 60 s, reading every ~3 s. Stops if either
   servo reaches 60 C (their own limit is 70 C).

Load and current are each servo's own reading; joint-frame "share" uses the
fact that the servos are mirrored, so equal shares show up as opposite-sign loads.
Results: experiments/<date>/<time>_shoulder_share/.

    uv run python tmp_scripts/shoulder_share.py
"""
import json
import math
import time

import numpy as np

from roarm import experiments
from roarm import motion as M
from roarm.arm import RoArm

PITCH = math.pi / 2
POSES = [np.array([x, 0.0, -20.0]) for x in (150.0, 180.0, 210.0, 240.0)]
HOLD_S, TEMP_STOP = 60, 60


def signed_load(v):
    return -(v - 1024) if v is not None and v >= 1024 else v


def sample(arm):
    out = {}
    for name, sid in (("A", 12), ("B", 13)):
        out[name] = {"goal": arm.read_reg(sid, 42, 2), "present": arm.read_reg(sid, 56, 2),
                     "load": signed_load(arm.read_reg(sid, 60, 2)), "current_raw": arm.read_reg(sid, 69, 2),
                     "temp_c": arm.read_reg(sid, 63)}
    a, b = out["A"], out["B"]
    out["mismatch"] = a["present"] + b["present"] - 4094
    out["err_A"] = a["present"] - a["goal"]
    out["err_B"] = b["present"] - b["goal"]
    # joint frame: B is mirrored, so its push in the joint's direction is -load_B
    jA, jB = a["load"], -b["load"]
    out["share_A"] = round(jA / (jA + jB), 3) if (jA + jB) else None
    return out


def main():
    arm = RoArm()
    if "." in arm.target or arm.read_reg(12, 22) is None:
        raise SystemExit("Needs USB and the roarm-m3 firmware (T:116).")
    run = experiments.new_run("shoulder_share")
    fb = arm.feedback()
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)
    log = {"poses": [], "hold": []}
    print(f"Results -> {run}", flush=True)
    try:
        arm.reset_pid()
        M.go_to_pose(arm, POSES[0] + [0, 0, 15], PITCH, roll, grip, rate)
        here = POSES[0] + [0, 0, 15]
        print("  x    shoulder  goal A/B     present A/B  mismatch  err A/B   load A/B   share A  mA-raw A/B  temp A/B", flush=True)
        for pose in POSES:
            above = pose + [0, 0, 15]
            M.move_tcp_joints(arm, here, above, PITCH, roll, grip, rate)
            M.move_tcp_joints(arm, above, pose, PITCH, roll, grip, rate)
            here = pose
            time.sleep(1.5)
            reads = [sample(arm) for _ in range(3)]
            s = arm.state()
            log["poses"].append({"pose": pose.tolist(), "shoulder_rad": float(s[1]), "reads": reads})
            r = reads[-1]
            print(f"{pose[0]:5.0f}  {s[1]:+.3f}   {r['A']['goal']}/{r['B']['goal']}   {r['A']['present']}/{r['B']['present']}"
                  f"   {r['mismatch']:+4d}    {r['err_A']:+3d}/{r['err_B']:+3d}  {r['A']['load']:+4d}/{r['B']['load']:+4d}"
                  f"    {r['share_A']}   {r['A']['current_raw']}/{r['B']['current_raw']}   {r['A']['temp_c']}/{r['B']['temp_c']}",
                  flush=True)
            if arm.link.warnings:
                raise RuntimeError(f"firmware warning: {arm.link.warnings[-1]}")
        print(f"\nHolding at x={POSES[-1][0]:.0f} for {HOLD_S} s:", flush=True)
        t0 = time.time()
        while time.time() - t0 < HOLD_S:
            r = sample(arm)
            r["t"] = round(time.time() - t0, 1)
            log["hold"].append(r)
            print(f"  t={r['t']:5.1f}s  temp A/B {r['A']['temp_c']}/{r['B']['temp_c']} C  load A/B {r['A']['load']:+4d}/{r['B']['load']:+4d}"
                  f"  mismatch {r['mismatch']:+d}", flush=True)
            if max(r["A"]["temp_c"] or 0, r["B"]["temp_c"] or 0) >= TEMP_STOP:
                raise RuntimeError(f"temperature reached {TEMP_STOP} C")
            if arm.link.warnings:
                raise RuntimeError(f"firmware warning: {arm.link.warnings[-1]}")
            time.sleep(max(0.0, 3.0 - (time.time() - t0 - r["t"])))
        M.move_tcp_joints(arm, here, np.array([180.0, 0.0, -25.0]), PITCH, roll, grip, rate)
        log["finished"] = "ok"
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e}", flush=True)
        log["finished"] = f"stopped: {e}"
    finally:
        arm.close()
        (run / "data.json").write_text(json.dumps(log, indent=1))
        print(f"Saved {run / 'data.json'}", flush=True)


if __name__ == "__main__":
    main()
