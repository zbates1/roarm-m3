# Shoulder trim: stop the two shoulder servos fighting

Firmware change (commit 1195a48): at power-on, right after B's middle is set, the firmware reads how far A is from its goal (A's gravity sag) and adds that to every later B goal (`shoulderDrivenTrim`, readable/settable with T:119). This power-on measured **41**.

`tmp_scripts/shoulder_trim_test.py`, stock gains, plain commands; power-on trim, trim 0 (Waveshare's behaviour), power-on trim again.

| | trim 41 | trim 0 | trim 41 again |
|---|---|---|---|
| error A / B (counts) | +3..4 / +1..4 | +13..21 / +16..20 | +3..4 / +2..4 |
| load A / B | 24–28 / 0–28 | 64–96 / 76–92 | 24–28 / 20–28 |
| shoulder sag at ready pose (counts) | **2** | 15 | **2** |
| shoulder friction band (counts) | 23 | 28 | 26 |
| 20 mm rise stops short (vertical / radial) | **−10.7 / −7.1 mm** | −15.8 / −7.8 mm | **−10.7 / −7.1 mm** |

`../223625_shoulder_trim_sweep/`: trim 35–56 with the arm held at the ready pose. B pushes least at 41 (load 0–20); above that it pushes the other way (−28 → −56). A's +5 stays the same throughout (it's A's own gravity sag), so the power-on measurement is right.

## Findings

- The trim removes the fight (loads ~3× lower, errors ~5× lower), cuts the shoulder's sag from 15 to 2 counts, and the plain-command rise shortfall from 15.8 to 10.7 mm.
- The friction band shrinks a little (28 → 23–26). Most of the shoulder's band is real gear friction, not the fight.
- The compensation in `roarm/tuning.json` was fitted with the servos fighting, so it has to be refit with the trim on.
