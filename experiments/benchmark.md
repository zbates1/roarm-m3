# Up-over-down benchmark history

`scripts/benchmark.py`: 4 spots 20 mm apart (x 190, y -30..+30), 20 mm dips, 15 mm/s, there and back twice. mm unless noted, measured minus commanded tip from the encoders. Lower is better; inward: closer to 0 is better.

| when | mode | change | bottom mean / worst | top mean / worst | repeat | path worst | inward worst | rough | code | run |
|---|---|---|---|---|---|---|---|---|---|---|
| 2026-10-18 21:05 | plain | baseline: shoulder trim + refit + base map | 3.71 / 5.07 | 13.69 / 14.41 | 3.86 | 15.82 | -6.27 | 10.5 | 146d2ce+changes | [2026-10-18/210226_benchmark](2026-10-18/210226_benchmark/) |
| 2026-10-18 21:05 | comp | baseline: shoulder trim + refit + base map | 0.88 / 1.37 | 1.00 / 2.36 | 1.11 | 4.74 | -2.84 | 13.7 | 146d2ce+changes | [2026-10-18/210226_benchmark](2026-10-18/210226_benchmark/) |
