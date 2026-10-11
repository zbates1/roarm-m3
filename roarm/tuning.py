"""Tuned servo gains + gravity feedforward found by tmp_scripts/autotune.py.

    from roarm import tuning
    gains, gravity = tuning.load()
    tuning.apply_gains(arm, gains)       # lasts until the arm restarts
    motion.move_tcp_joints(arm, a, b, pitch, roll, grip, rate, gravity=gravity)

Gains aren't saved on the servos (every startup resets them to P=16), so apply
them after connecting. The gravity model was fitted with these gains, for poses
reached from above, gripper pointing down, around x=160-235, z=-65..-15 mm.
Higher gains also mean the arm pushes harder if something blocks it.
"""
import json
import time
from pathlib import Path

from .motion import GravityModel

PATH = Path(__file__).with_name("tuning.json")
JOINT_IDS = {"base": 1, "shoulder": 2, "elbow": 3, "wrist": 4}


def load(path=PATH):
    d = json.loads(Path(path).read_text())
    return d["gains"], GravityModel.from_dict(d["gravity"])


def save(gains, gravity, source, path=PATH):
    Path(path).write_text(json.dumps({
        "gains": gains, "gravity": gravity.to_dict(), "source": source,
        "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
    }, indent=1))


def apply_gains(arm, gains):
    for name, p in gains.items():
        arm.set_pid(JOINT_IDS[name], p, 0)
    time.sleep(0.5)
