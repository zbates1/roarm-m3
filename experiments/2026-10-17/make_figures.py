"""Figures for 2026-10-17: the P x D sweep and the compensation checks.

    uv run python experiments/2026-10-17/make_figures.py
"""
import glob
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DAY = Path(__file__).resolve().parent
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3de"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]   # validated: categorical slots 1-3, light surface
MARKERS = ["o", "s", "^"]
plt.rcParams.update({"font.size": 11, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.titlecolor": INK, "axes.titlesize": 12, "axes.titleweight": "bold",
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE})


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def hp_std(x, win=11):
    sm = np.convolve(x, np.ones(win) / win, mode="valid")
    h = win // 2
    return float(np.std(x[h:len(x) - h] - sm))


def sweep_figure():
    run = Path(sorted(glob.glob(str(DAY / "*_pd_sweep")))[-1])
    summary = [r for r in json.loads((run / "summary.json").read_text()) if "moves" in r]
    rows = []
    for r in summary:
        z = np.load(run / "trials" / f"{r['trial']:02d}_P{r['P']}_D{r['D']}.npz")
        t, t0 = z["host_t"], z["cmd"][0, 0]
        sel = (t >= t0) & (t <= t0 + 1.4)          # the way down
        wiggle = sum(hp_std(z["load"][sel, j].astype(float)) for j in (1, 2))   # elbow + wrist
        rows.append((r["P"], r["D"], wiggle, r["moves"]["up"]["radial_shift_mm"], r.get("rated_by_eye")))
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    panels = [(2, "Load wiggle on the way down (elbow + wrist)", "load wiggle (servo load units, std)"),
              (3, "Inward pull during the 20 mm rise", "radial shift (mm, - = toward base)")]
    for ax, (k, title, ylabel) in zip(axes, panels):
        for c, D in enumerate((32, 64, 128)):
            pts = sorted({P: np.mean([row[k] for row in rows if row[0] == P and row[1] == D])
                          for P, d, *_ in rows if d == D}.items())
            x, y = zip(*pts)
            ax.plot(x, y, color=SERIES[c], linewidth=2, marker=MARKERS[c], markersize=8,
                    markeredgecolor=SURFACE, markeredgewidth=1.5, label=f"D = {D}")
            above = k == 3 and D == 32                  # keep the D = 32 end label off the other lines
            ax.annotate(f"D = {D}", (x[-1], y[-1]), xytext=(-6, 12) if above else (8, 0), textcoords="offset points",
                        va="center", ha="right" if above else "left", color=INK2, fontsize=10)
        ax.set_title(title, loc="left")
        ax.set_xlabel("P (servo stiffness)")
        ax.set_ylabel(ylabel)
        ax.set_xticks([16, 24, 32, 48])
        ax.set_xlim(12, 56)
        style(ax)
    seen = set()
    for P, D, w, _, eye in rows:
        if eye and D == 32 and P not in seen:      # the two stock trials share one label
            seen.add(P)
            axes[0].annotate("rated " + eye.split(" ")[0], (P, w), xytext=(0, 12), textcoords="offset points",
                             ha="center", color=INK2, fontsize=9)
    axes[0].legend(frameon=False, loc="upper left")
    fig.suptitle("P x D sweep, plain commands, stock D = 32 vs more damping", x=0.01, ha="left",
                 color=INK, fontsize=13, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = run / "pd_sweep.png"
    fig.savefig(out, dpi=150)
    return out


def comp_figure(run):
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True)
    for c, mode in enumerate(("plain", "compensated")):
        z = np.load(sorted(run.glob(f"trials/*_{mode}.npz"))[0])
        cmd, t = z["cmd"], z["host_t"]
        p_cmd = np.stack([np.interp(t, cmd[:, 0], cmd[:, k]) for k in (1, 2, 3)], axis=1)
        d = z["tip"] - p_cmd
        r_hat = p_cmd[:, :2] / np.linalg.norm(p_cmd[:, :2], axis=1, keepdims=True)
        radial, vertical = (d[:, :2] * r_hat).sum(1), d[:, 2]
        tt = t - cmd[0, 0]
        for ax, y in zip(axes, (radial, vertical)):
            ax.plot(tt, y, color=SERIES[c], linewidth=2, label=mode)
            ax.annotate(mode, (tt[-1], y[-1]), xytext=(8, 0), textcoords="offset points",
                        va="center", color=INK2, fontsize=10)
    # move phases from the plain trial's command timing (same schedule for both)
    z = np.load(sorted(run.glob("trials/*_plain.npz"))[0])
    tc = z["cmd"][:, 0] - z["cmd"][0, 0]
    gaps = np.where(np.diff(tc) > 0.3)[0]
    starts = np.r_[tc[0], tc[gaps + 1]]
    for ax in axes:
        for s, name in zip(starts, ("down", "up", "left", "back")):
            ax.axvline(s, color=GRID, linewidth=1)
            ax.text(s + 0.05, 1.0, name, transform=ax.get_xaxis_transform(), va="top", color=INK2, fontsize=9)
        ax.axhline(0, color=INK2, linewidth=0.8)
        style(ax)
    axes[0].set_title("Toward (-) / away from (+) the base: measured minus commanded tip", loc="left")
    axes[0].set_ylabel("radial (mm)")
    axes[1].set_title("Below (-) / above (+) the commanded height", loc="left")
    axes[1].set_ylabel("vertical (mm)")
    axes[1].set_xlabel("time since the first move started (s)")
    axes[0].legend(frameon=False, loc="lower left")
    fig.suptitle("Stock gains: plain commands vs gravity + friction compensation", x=0.01, ha="left",
                 color=INK, fontsize=13, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 0.97, 0.95))
    out = run / "comp_check.png"
    fig.savefig(out, dpi=150)
    return out


if __name__ == "__main__":
    runs = [Path(r) for r in sorted(glob.glob(str(DAY / "*_comp_check")))]
    for f in (sweep_figure(), *map(comp_figure, runs)):
        print("wrote", f)
