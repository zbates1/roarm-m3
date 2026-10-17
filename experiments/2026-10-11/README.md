# 2026-10-11

Moved here from `tmp_scripts/results/` on 2026-10-17. Everything ran on the stock firmware.

| Folder | What | Result |
|---|---|---|
| `autotune/` | Unattended step tests, P sweep, gravity feedforward (runs 1–3) | Run 1 stopped on an over-strict safety check (no motion). Friction bands at stock P 16 (counts): base 18, shoulder 31, elbow 18, wrist 10; at P 80: 5 / 8 / 4 / 3. I ≥ 2 oscillates. **P 80 "stable" was only checked at rest. Watched in motion it vibrated hard; don't use it.** |
| `tuned_vs_stock/` | P 80 + gravity feedforward vs stock, 24 dips | 3.83 → 0.72 mm average landing error. Run 1 tripped the 2 A overcurrent warning when gains jumped 16 → 80; run 2 ramped the gains. |
| `track_test/` | Commanded vs measured path during dip and rise | Rising stalls short: stock 14.6 mm low and 9 mm toward the base; P 80 + feedforward 2.9 / 1.3 mm. |
| `smooth_test/` | Eye-rated P × servo acceleration | P 16 smooth (4–5), 24 so-so, 32 rough, 40 rough; acceleration limits didn't help. |
| `friction/` | Gravity + friction feedforward at stock gains (v1 and v2) | v2 (25 mm approaches): 1.75 / 1.68 / 1.32 mm (from above / from below / after a rise) vs gravity-only 4.3 / 15.2 / 15.2. Saved to `roarm/tuning.json`. |
