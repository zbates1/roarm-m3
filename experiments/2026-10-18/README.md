# 2026-10-18

| Run | What | Result |
|---|---|---|
| `205741_benchmark/` | First up-over-down benchmark | Superseded: compensated mode started without the base's direction known (dip 1 3 mm off). Lead-in added to the script. Not in the history. |
| `210226_benchmark/` | Up-over-down benchmark, baseline | Compensated: bottoms 0.88 mean / 1.37 worst mm, tops 1.00 / 2.36, repeat 1.11, path worst 4.74, inward worst −2.84 (one stuck rise; others within ±0.9). Plain: 3.71 / 5.07, tops 13.7. Compensated is ~30 % rougher. |

The benchmark (`scripts/benchmark.py`) is the standard test from now on; every run adds a row to [`../benchmark.md`](../benchmark.md).
