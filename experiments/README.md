# Experiments

One folder per day; inside, one folder per run (`<HHMMSS>_<name>/`), or per experiment type for older days. Each day has a README with what ran and what came out. Scripts write here through `roarm.experiments` (`new_run(name)`, `day_dir()`).

| Day | Highlights |
|---|---|
| [2026-10-11](2026-10-11/README.md) | Friction bands measured; P 80 accurate but vibrates in motion; gravity + friction feedforward at stock gains ~1.3–1.8 mm |
| [2026-10-17](2026-10-17/README.md) | Firmware backup, rebuild and additions (register access, telemetry); more D makes motion rougher; compensation stops the inward drift on rising; shoulder servos no longer fight (firmware trim); refitted compensation ~1–2 mm; per-angle base map brings landings to ~1 mm |
