"""Where experiment results go: experiments/<YYYY-MM-DD>/...

    run = experiments.new_run("pd_sweep")   # experiments/2026-10-17/143012_pd_sweep/
    day = experiments.day_dir()             # experiments/2026-10-17/
"""
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / "experiments"


def day_dir():
    d = ROOT / time.strftime("%Y-%m-%d")
    d.mkdir(parents=True, exist_ok=True)
    return d


def new_run(name):
    d = day_dir() / f"{time.strftime('%H%M%S')}_{name}"
    d.mkdir(parents=True, exist_ok=False)
    return d
