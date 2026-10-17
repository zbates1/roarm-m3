# 2026-10-17

| Run | What | Result |
|---|---|---|
| (no folder) | Firmware backup + rebuild | Full 16 MB flash backed up and verified (`firmware/README.md`). The arm's program was byte-identical to Waveshare's 20260701 build; rebuilt it from source with matching toolchain, then added T:116/117 (servo registers) and T:118 (telemetry). |
| (no folder) | Servo register dump | All seven servos: P 16, **D 32**, I 0, dead zones 1, min startup force 16, model 777 (STS3215-compatible), firmware 3.9. Details in `firmware/README.md`. |
| `143345_pd_sweep/` | Pipeline check | One stock trial to check the telemetry analysis. |
| `143419_pd_sweep/` | P × D sweep, automated | More D makes motion rougher at every P; higher P reduces drift but is rough. Keep stock gains. |
| `143713_comp_check/` | Compensation vs plain | At stock gains, compensation cuts the inward pull on rising from −7 mm to under 1 mm and the vertical shortfall from 14.6 mm to ~1 mm. Remaining: ~1.9 mm sideways (base). |

`make_figures.py` regenerates the figures.
