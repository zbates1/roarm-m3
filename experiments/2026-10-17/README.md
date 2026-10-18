# 2026-10-17

| Run | What | Result |
|---|---|---|
| (no folder) | Firmware backup + rebuild | Full 16 MB flash backed up and verified (`firmware/README.md`). The arm's program was byte-identical to Waveshare's 20260701 build; rebuilt it from source with matching toolchain, then added T:116/117 (servo registers) and T:118 (telemetry). |
| (no folder) | Servo register dump | All seven servos: P 16, **D 32**, I 0, dead zones 1, min startup force 16, model 777 (STS3215-compatible), firmware 3.9. Details in `firmware/README.md`. |
| `143345_pd_sweep/` | Pipeline check | One stock trial to check the telemetry analysis. |
| `143419_pd_sweep/` | P × D sweep, automated | More D makes motion rougher at every P; higher P reduces drift but is rough. Keep stock gains. |
| `143713_comp_check/` | Compensation vs plain | At stock gains, compensation cuts the inward pull on rising from −7 mm to under 1 mm and the vertical shortfall from 14.6 mm to ~1 mm. Remaining: ~1.9 mm sideways (base). |
| `222746_shoulder_share/` | Both shoulder servos: position, load, temperature | The two servos disagree about the joint angle by ~30 counts and push against each other. Cause: the power-on calibration bakes A's gravity sag into B's zero. |
| (firmware, commit 1195a48) | Shoulder trim | At power-on the firmware measures A's sag and adds it to B's goals (T:119 reads/sets it). |
| `223312_shoulder_trim/`, `223625_shoulder_trim_sweep/` | Trim vs no trim; trim 35–56 | The fight is gone (errors 15–20 → 3–4 counts, loads ~3× lower), shoulder sag 15 → 2 counts. The power-on value (41) is the best one. |
| `223652_friction_refit/` | Compensation refit with the trim | Shoulder residual 3.4 → 1.9 counts; checks 1.15 / 1.97 / 1.13 mm (above / below / rise). Saved. The base's band centre moves ~5 counts between the −y and +y sides: the next thing to fix. |
| `224651_comp_check/` | Compensation vs plain, refitted | Sideways after returning from the left 1.9 → 0.8 mm, vertical after rising ~1.1 → 0.1–0.6 mm; inward shift on rising still ~1 mm. |

`make_figures.py` regenerates the figures.
