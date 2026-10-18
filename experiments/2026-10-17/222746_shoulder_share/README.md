# How the two shoulder servos share the load

`tmp_scripts/shoulder_share.py`, stock gains. A = ID 12 ("driving"), B = ID 13 ("driven"), mirrored about 2047 (goal_A + goal_B = 4094). Mismatch = present_A + present_B − 4094: how far apart their ideas of the joint angle are (1 count = 0.088°).

| reach x (mm) | shoulder (rad) | mismatch | A off its goal | B off its goal | load A / B | temp A / B (°C) |
|---|---|---|---|---|---|---|
| 150 | 0.089 | +34 | +17 | +17 | +80 / +80 | 46 / 32 |
| 180 | 0.195 | +30 | +13 | +17 | +64 / +80 | 46 / 32 |
| 210 | 0.305 | +29 | +13 | +16 | +64 / +76 | 46 / 32 |
| 240 | 0.420 | +28 | +14 | +14 | +68 / +68 | 45 / 32 |

60 s hold at x = 240: nothing changed (temps 45/32, loads +68/+68, mismatch +28). Currents stayed tiny (raw 2–3), so A isn't actively heating there; it's warm from earlier work. An earlier spot reading at the ready pose had A +188 / B −24 with mismatch +40, which is A carrying the gravity load alone.

## Findings

- **The two servos disagree about the joint angle by a nearly constant ~30 counts (2.6°)**: a calibration offset, not wear.
- **Cause, in the firmware's power-on routine** (`RoArm-M3_module.h` ~L258–279): B is made limp, A moves the shoulder alone (and only has to get within 120 counts of its goal), then B's current position is set as its middle. B's zero therefore includes A's gravity sag at that moment, which depends on the elbow pose at power-on (the elbow is straightened afterwards).
- **Consequence:** arriving from above, each servo sits ~15 counts past its own goal in opposite joint directions and they push against each other (equal loads), with friction holding the joint. In other states A carries the load alone. That built-in fight squeezes the gears together and plausibly explains why the shoulder has the widest friction band (31 counts) and the worst-fitting compensation model.

Next: firmware trim for B (= A's error at calibration), compared against trim 0.
