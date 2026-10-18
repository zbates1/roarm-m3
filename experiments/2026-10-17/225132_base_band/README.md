# Where does the base stop, depending on angle, reach and history?

`tmp_scripts/base_band.py`, stock gains, plain commands, base only. Gripper down at z = −30, reach 180 then 220 mm; base −8° → +8° → −8° in 4° steps. At each angle the base arrives from 40 counts below and above, twice each (0.5 s min-jerk moves), and the base servo's goal and present position are read from its registers. Error = measured − commanded, in base counts (1 count = 0.088°, ~0.3 mm sideways at 200 mm). Band = from above − from below; centre = their mean.

Prompted by `../223652_friction_refit/`: the base's band centre was ~5 counts different on the −y and +y sides.

| base angle | centre, 180 mm (up / down pass) | centre, 220 mm (up / down pass) | band |
|---|---|---|---|
| −8° | +1.0 / +0.8 | +1.0 / +1.0 | 16–16.5 |
| −4° | +2.5 / +3.5 | +2.8 / +3.5 | 22–23 |
| 0° | +1.8 / +2.5 | +1.8 / +2.5 | 14–17 |
| +4° | −4.5 / −3.5 | −4.5 / −4.2 | 19–21 |
| +8° | −2.5 | −2.5 | 21–23 |

Approach length (reach 200, base 0°):

| approach (counts) | from below / above | band |
|---|---|---|
| 10 | −2.5 / +2.0 | 4.5 |
| 20 | −4.5 / +8.0 | 12.5 |
| 40 | −6.0 / +9.0 | 15.0 |
| 80 | −6.0 / +9.5 | 15.5 |

## Findings

- **Reach makes no difference** (180 vs 220 mm agree within 0.5 counts), so it isn't the arm's weight on a tilted table.
- **No hysteresis**: the up and down passes agree within 0.4–0.6 counts on average.
- **The centre depends on the base angle, but not in a straight line**: it jumps ~6–7 counts between 0° and +4° (~2 mm at the tip). The band width also changes (14–23 counts). This is a repeatable property of the base drive at each angle (gear mesh or the cable bundle pushing on it), so a calibrated per-angle map of the base would correct it. That's how CNC controllers handle position-dependent errors (pitch error compensation). It's not a per-spot offset: it holds at any reach and from either side.
- Moves of 20 counts or more break the base fully free; a 10-count move doesn't.
- 4° steps are too coarse to tell whether the map is smooth: `../225625_base_band_fine/` maps it in 1° steps.
- Register and T:105 readings agree within 1–1.5 counts; base loads stayed under 8 %.

Files: `data.json` (every landing: goal, present, load, error), `run.log`.
