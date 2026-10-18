# Does the per-angle base map help?

`tmp_scripts/base_map_check.py`, stock gains, shoulder trim 41. The base map from `../225625_base_band_fine/` (T:105 readings, both passes averaged) was added to the saved compensation (`../223652_friction_refit/`). The script landed on friction_tune's 5 check poses plus 2 more inside ±10°, from above and from below (25 mm approaches at 15 mm/s), once with the fitted base term ("comp") and once with the map ("map"), alternating which went first.

| pose | base | from | comp: dist / sideways (mm) | map: dist / sideways (mm) |
|---|---|---|---|---|
| (175, 25, −45) | +8.1° | above | 0.69 / −0.68 | 1.02 / +0.95 |
| | | below | 2.50 / −2.29 | 1.44 / −0.14 |
| (225, −25, −30) | −6.3° | above | 1.74 / +1.09 | 1.51 / +1.09 |
| | | below | 1.46 / −1.34 | 0.41 / +0.05 |
| (200, 30, −20) | +8.5° | above | 1.17 / −0.33 | 1.18 / +0.91 |
| | | below | 2.92 / −2.80 | 1.02 / +0.29 |
| (170, −20, −40) | −6.7° | above | 0.92 / +0.88 | 1.04 / +0.88 |
| | | below | 1.06 / −0.96 | 0.48 / −0.17 |
| (215, 15, −35) | +4.0° | above | 1.46 / −1.46 | 1.26 / +1.19 |
| | | below | 2.90 / −2.77 | 1.50 / −0.13 |
| (190, 10, −30) | +3.0° | above | 1.57 / −1.54 | 0.36 / +0.21 |
| | | below | 2.61 / −2.40 | 1.01 / −0.37 |
| (185, −8, −35) | −2.5° | above | 2.38 / +2.31 | 1.04 / +0.90 |
| | | below | 1.26 / −0.80 | 1.04 / −0.23 |
| **average** | | above | 1.42 / 1.18 | **1.06 / 0.88** |
| | | below | 2.10 / 1.91 | **0.99 / 0.20** |

Saved: `roarm/tuning.json` now has the compensation with the base map (`compensation_source` points here).

## Findings

- With the map, arrivals from below land within 0.2 mm sideways on average (was 1.9 mm), and the average distance off drops to ~1 mm from both sides.
- From above, a consistent **+0.9 mm sideways** remains. These approaches are slow and the other joints move at the same time; the map was built from quick base-only moves, and the base stops ~3 counts shorter in slow moves.

Files: `data.json` (the map, every landing), `run.log`.
