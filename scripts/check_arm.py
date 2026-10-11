"""Find the arm and print its joint angles. Doesn't move anything.

    python scripts/check_arm.py                 # USB, found automatically
    python scripts/check_arm.py 192.168.1.203   # WiFi
"""
import sys
import time

from roarm.arm import JOINTS, RoArm

arm = RoArm(sys.argv[1] if len(sys.argv) > 1 else None)
print(f"Arm on {arm.target}")
for name, q in zip(JOINTS, arm.state()):
    print(f"  {name:9s} {q:+.3f} rad")

t0, n = time.perf_counter(), 20
for _ in range(n):
    arm.state()
print(f"State reads: {n / (time.perf_counter() - t0):.0f} per second")
arm.close()
