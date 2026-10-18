# Compensation refit with the shoulder trim on

`tmp_scripts/friction_tune.py`, stock gains (P 16, D 32), shoulder trim 41 (measured at power-on). Same procedure as the 2026-10-11 v2 fit: land on 12 grid poses (x 160–235, z −45..−15, y = 0) from above-left and below-right with 25 mm approaches, fit gravity-only and gravity + friction models, then check on 5 other poses and a 20 mm rise. The old compensation was fitted while the two shoulder servos were fighting (`../223312_shoulder_trim/`).

## Fit

| | base | shoulder | elbow | wrist |
|---|---|---|---|---|
| friction (counts, half the band) | 9.0 | 13.1 | 9.3 | 5.1 |
| fit residual, old (counts) | 0.97 | 3.41 | 1.26 | 1.00 |
| **fit residual, now (counts)** | 0.79 | **2.13** | 0.78 | 0.99 |
| with load-scaled friction, now | 0.79 | 1.89 | 0.72 | 0.99 |

## Check (average distance from target, mm)

| | from above | from below | 20 mm rise |
|---|---|---|---|
| gravity only, now | 3.55 | 13.18 | 13.34 |
| gravity + friction, old (2026-10-11, no trim) | 1.75 | 1.68 | 1.32 |
| gravity + friction, now | 1.20 | 2.05 | 1.05 |
| **gravity + load-scaled friction, now (saved)** | **1.15** | 1.97 | 1.13 |

Saved to `roarm/tuning.json` (stock gains + load-scaled model).

## Findings

- The shoulder model fits much better with the fight gone (residual 3.4 → 1.9–2.1 counts), and the from-above and rise results improve.
- The from-below errors are almost all **sideways** (heights within ±0.9 mm), so they come from the base. Gravity-only mode applies no base correction, so its tangential errors give the raw base band at each check pose:

| check pose | base angle | from above / below (counts) | band | centre |
|---|---|---|---|---|
| grid, y = 0 (24 landings) | 0° | +13.1 / −5.0 | 18.1 | +4.0 |
| (225, −25, −30) | −6.3° | +15.1 / −7.7 | 22.8 | +3.7 |
| (170, −20, −40) | −6.7° | +13.3 / −6.4 | 19.7 | +3.4 |
| (215, 15, −35) | +4.0° | +8.6 / −13.0 | 21.6 | −2.2 |
| (175, 25, −45) | +8.1° | +9.5 / −12.0 | 21.5 | −1.2 |
| (200, 30, −20) | +8.5° | +9.9 / −11.8 | 21.7 | −0.9 |

  The base band is wider off-axis (20–23 counts vs 18), and its centre moves ~5 counts between the −y and +y sides. The model's single base offset (−4 counts, fitted at y = 0) is right for y ≤ 0 and overcorrects on the +y side, which is where the 2–3 mm from-below errors are. Something pulls the base towards one side depending on its angle (cable drag, a tilted table, the base gears). Next: base-only sweep across angles and reaches to find out which.

Files: `friction_20261017_223652.json` (all landings, models, checks), `run.log`.
