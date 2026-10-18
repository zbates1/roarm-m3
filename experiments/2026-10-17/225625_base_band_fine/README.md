# Base map in 1° steps

`tmp_scripts/base_band.py --fine`: same as `../225132_base_band/` but reach 180 only, base −10° → +10° → −10° in 1° steps, one landing from each side per angle per pass (40-count approaches, 0.5 s moves). Counts from the base servo's registers (measured − commanded; 1 count ≈ 0.3 mm sideways at 200 mm).

| deg | −10 | −9 | −8 | −7 | −6 | −5 | −4 | −3 | −2 | −1 | 0 | +1 | +2 | +3 | +4 | +5 | +6 | +7 | +8 | +9 | +10 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| centre, up | +0.5 | 0 | 0 | +1.0 | +1.5 | +1.0 | +3.5 | +2.5 | +3.5 | +3.0 | +1.5 | −1.0 | −3.0 | −3.5 | −5.5 | −4.5 | −2.5 | −2.5 | −2.5 | −1.5 | −1.0 |
| centre, down | +2.5 | +2.5 | +1.5 | +1.5 | +1.5 | +2.5 | +4.0 | +3.5 | +4.0 | +3.5 | +2.5 | +0.5 | −1.5 | −3.5 | −4.5 | −3.0 | −2.5 | −2.0 | −2.0 | −1.0 | — |
| band, up | 19 | 18 | 18 | 18 | 21 | 22 | 25 | 23 | 19 | 18 | 15 | 16 | 18 | 19 | 21 | 21 | 23 | 23 | 21 | 21 | 22 |

## Findings

- **The centre is a smooth, repeatable curve of the base angle**: up to about +4 around −3°, through zero near +1°, down to about −5 at +4°, back to about −1 at +10°. It looks like a ripple roughly 14° long. That would fit a slightly eccentric gear inside the base servo, but this test can't prove the cause.
- The two passes agree within ~1 count from −8° to +10°. At −9° and −10° the down pass reads ~2 counts higher (the base had just come from +10°).
- The band width also changes with angle (15–25 counts), smallest near 0° and widest near −4° and +6°.
- The map's T:105 values (both passes averaged) went into `motion.Compensation.base_map`. `../230219_base_map_check/` tests it.

Files: `data.json`, `run.log`.
