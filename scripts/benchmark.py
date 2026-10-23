"""Up-over-down benchmark: the standard test of how well the arm places the tip.

ASSUMES THE SPACE IS CLEAR: x 185-195, y -30..+30, z -40..-10 mm (gripper down).
Needs USB and the roarm-m3 firmware (T:118 telemetry). No prompts.

Four spots in a row, 20 mm apart sideways (x = 190, y = -30, -10, +10, +30; base
about -9..+9 deg), like four wells to dip into. From the top of spot 1 the tip goes
down 20 mm, holds, up 20 mm, over to the next spot, down again, along the row and
back: 1 2 3 4 3 2 1, then 2 3 4 3 2 1 again (--rounds 2). Every move is a straight
line at 15 mm/s; the joints are recorded the whole time (~100 samples/s). Each
mode starts with a lead-in from spot 2 over to spot 1 (not scored), so the first
dip arrives the way all the others do, with every joint's direction known.

Scores (measured minus commanded tip, from the encoders, mm):
  bottom  error at rest at the bottom of each dip: mean / worst
  top     error at rest after each rise: mean / worst
  repeat  worst spread of one spot's bottom position over its visits
          (largest distance from that spot's average)
  path    worst distance from the commanded line while moving
  inward  how far each rise pulls the tip toward the base (worst; - = inward)
  rough   elbow + wrist load wiggle while moving (servo load units; more = more
          jerky, as rated by eye in the 2026-10-17 P x D sweep)

Modes: comp = the gravity + friction compensation in roarm/tuning.json (default);
plain = the same moves without it. Results go to
experiments/<date>/<time>_benchmark/ and every run adds a row per mode to
experiments/benchmark.md, so changes can be compared over time.

    uv run python scripts/benchmark.py
    uv run python scripts/benchmark.py --mode both --label "base map"
"""
import argparse
import json
import math
import subprocess
import time

import numpy as np

from roarm import experiments
from roarm import motion as M
from roarm import tuning
from roarm.arm import RoArm

PITCH = math.pi / 2
X, Z_DOWN, Z_UP = 190.0, -40.0, -20.0
ROW = (-30.0, -10.0, 10.0, 30.0)
SPEED = 15.0                                # mm/s average (min-jerk)
HOLD_DOWN, HOLD_UP, HOLD_OVER = 1.0, 0.5, 0.3
REST = 0.3                                  # s: the end of each hold counts as "at rest"
HISTORY = experiments.ROOT / "benchmark.md"
SMOOTH_WIN = 11                             # samples (~110 ms) for the load wiggle
SCORES = ["bottom_mean", "bottom_worst", "top_mean", "top_worst", "repeat_worst", "path_worst",
          "inward_worst", "rough"]


def at(i, z):
    return np.array([X, ROW[i], z])


def order(rounds):
    return [0] + [1, 2, 3, 2, 1, 0] * rounds


def check(arm):
    fb = arm.feedback()
    if not 1050 <= fb["v"] <= 1300:
        raise RuntimeError(f"supply {fb['v'] / 100:.2f} V")
    if not all(fb[f"torswitch{j}"] for j in "BSERTG"):
        raise RuntimeError("a motor turned off")
    if arm.link.warnings:
        raise RuntimeError(f"firmware warning: {arm.link.warnings[-1]}")
    return fb


def components(cmd, meas):
    """meas - cmd as (radial, sideways, vertical); radial < 0 = toward the base."""
    d = meas - cmd
    r_hat = cmd[:, :2] / np.linalg.norm(cmd[:, :2], axis=1, keepdims=True)
    t_hat = np.stack([-r_hat[:, 1], r_hat[:, 0]], axis=1)
    return np.stack([(d[:, :2] * r_hat).sum(1), (d[:, :2] * t_hat).sum(1), d[:, 2]], axis=1)


def load_wiggle(load):
    """Elbow + wrist load: std of what's left after a centred moving average."""
    out = 0.0
    for j in (1, 2):
        x = load[:, j].astype(float)
        if len(x) <= SMOOTH_WIN:
            return float("nan")
        sm = np.convolve(x, np.ones(SMOOTH_WIN) / SMOOTH_WIN, mode="valid")
        h = SMOOTH_WIN // 2
        out += float(np.std(x[h:len(x) - h] - sm))
    return out


def run_mode(arm, mover, rounds):
    """One benchmark pass; returns telemetry, commanded path [(t, x, y, z)] and move windows."""
    seq = order(rounds)
    log, moves = [], []

    def move(kind, spot, a, b, hold):
        t0 = time.perf_counter()
        mover.move(a, b, speed=SPEED, log=log)
        t1 = time.perf_counter()
        time.sleep(hold)
        moves.append({"kind": kind, "spot": spot, "t": (t0, t1, time.perf_counter())})

    arm.telemetry_start()
    try:
        time.sleep(0.3)
        for i, s in enumerate(seq):
            if i:
                move("over", s, at(seq[i - 1], Z_UP), at(s, Z_UP), HOLD_OVER)
            move("down", s, at(s, Z_UP), at(s, Z_DOWN), HOLD_DOWN)
            move("up", s, at(s, Z_DOWN), at(s, Z_UP), HOLD_UP)
            if arm.link.warnings:
                raise RuntimeError(f"firmware warning: {arm.link.warnings[-1]}")
    finally:
        tel = arm.telemetry_stop()
    return tel, np.array(log), moves


def score(tel, cmd, moves):
    t = tel["host_t"]
    p_cmd = np.stack([np.interp(t, cmd[:, 0], cmd[:, k]) for k in (1, 2, 3)], axis=1)
    comp = components(p_cmd, tel["tip"])
    dist = np.linalg.norm(tel["tip"] - p_cmd, axis=1)
    dips, tops, paths, inward, rough = [], [], [], [], []
    for m in moves:
        t0, t1, t2 = m["t"]
        moving = (t >= t0) & (t <= t1)
        rest = (t >= t2 - REST) & (t <= t2)
        if not rest.any() or moving.sum() <= SMOOTH_WIN:
            continue
        e = comp[rest].mean(0)
        paths.append(float(dist[moving].max()))
        rough.append(load_wiggle(tel["load"][moving]))
        row = {"spot": m["spot"] + 1, "radial": round(float(e[0]), 2), "sideways": round(float(e[1]), 2),
               "vertical": round(float(e[2]), 2), "dist": round(float(np.linalg.norm(e)), 2)}
        if m["kind"] == "down":
            row["tip"] = tel["tip"][rest].mean(0).round(2).tolist()
            row["t"] = round(float(t2 - moves[0]["t"][0]), 2)
            dips.append(row)
        elif m["kind"] == "up":
            before = (t >= t0 - REST) & (t < t0)
            if before.any():
                row["inward"] = round(float(comp[rest, 0].mean() - comp[before, 0].mean()), 2)
                inward.append(row["inward"])
            tops.append(row)
    spread = []
    for s in sorted({d["spot"] for d in dips}):
        pts = np.array([d["tip"] for d in dips if d["spot"] == s])
        spread.append(float(np.linalg.norm(pts - pts.mean(0), axis=1).max()) if len(pts) > 1 else 0.0)
    b, tp = [d["dist"] for d in dips], [d["dist"] for d in tops]
    summary = {"bottom_mean": np.mean(b), "bottom_worst": np.max(b), "top_mean": np.mean(tp), "top_worst": np.max(tp),
               "repeat_worst": max(spread), "path_worst": max(paths), "path_mean": np.mean(paths),
               "inward_worst": min(inward), "inward_mean": np.mean(inward), "rough": np.nanmean(rough)}
    for name in ("radial", "sideways", "vertical"):
        summary[f"bottom_{name}_mean"] = np.mean([d[name] for d in dips])
    summary = {k: round(float(v), 2) for k, v in summary.items()}
    return summary, {"dips": dips, "tops": tops, "repeat_by_spot": [round(x, 2) for x in spread]}


def figure(run, results):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    surface, ink, ink2, grid = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3de"
    colors = {"plain": "#2a78d6", "comp": "#eb6834"}   # validated categorical slots 1-2 (light surface)
    names = {"plain": "plain", "comp": "compensated"}
    plt.rcParams.update({"font.size": 11, "axes.edgecolor": grid, "axes.labelcolor": ink2, "xtick.color": ink2,
                         "ytick.color": ink2, "axes.titlecolor": ink, "axes.titlesize": 12,
                         "axes.titleweight": "bold", "figure.facecolor": surface, "axes.facecolor": surface,
                         "savefig.facecolor": surface})
    fig, axes = plt.subplots(2, 1, figsize=(11, 7.6), gridspec_kw={"height_ratios": [1.3, 1]})
    ax = axes[0]
    first = next(iter(results.values()))
    ax.plot(first["cmd"][:, 2], first["cmd"][:, 3], color=ink2, linewidth=1, linestyle=(0, (4, 3)), label="commanded")
    for mode, r in results.items():
        tip = r["tel"]["tip"]
        ax.plot(tip[:, 1], tip[:, 2], color=colors[mode], linewidth=1.5, alpha=0.9, label=names[mode])
    for i, y in enumerate(ROW):
        ax.text(y, Z_UP + 4, f"spot {i + 1}", ha="center", color=ink2, fontsize=10)
    ax.set_title("Side view along the row: tip path (measured from the encoders)", loc="left")
    ax.set_xlabel("sideways position y (mm)")
    ax.set_ylabel("height z (mm)")
    low = min(float(r["tel"]["tip"][:, 2].min()) for r in results.values())
    ax.set_ylim(min(low, Z_DOWN) - 3, Z_UP + 8)
    ax = axes[1]
    for mode, r in results.items():
        d = r["detail"]["dips"]
        x = np.arange(1, len(d) + 1)
        y = [p["dist"] for p in d]
        ax.plot(x, y, color=colors[mode], linewidth=2, marker="o", markersize=8, markeredgecolor=surface,
                markeredgewidth=1.5, label=names[mode])
        ax.annotate(f"{names[mode]}: mean {r['summary']['bottom_mean']:.2f} mm", (x[-1], y[-1]), xytext=(10, 0),
                    textcoords="offset points", va="center", color=ink2, fontsize=10)
    d = first["detail"]["dips"]
    ax.set_xticks(np.arange(1, len(d) + 1), [f"{i + 1}\nspot {p['spot']}" for i, p in enumerate(d)], fontsize=9)
    ax.set_xlim(0.5, len(d) + 3.5)
    ax.set_ylim(bottom=0)
    ax.set_title("Error at the bottom of each dip, at rest", loc="left")
    ax.set_ylabel("distance off (mm)")
    for a in axes:
        a.grid(True, color=grid, linewidth=0.8)
        a.set_axisbelow(True)
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    axes[0].legend(frameon=False, loc="lower center", ncol=len(results) + 1)
    fig.suptitle("Up-over-down benchmark: 4 spots 20 mm apart, 20 mm dips", x=0.01, ha="left", color=ink,
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(run / "benchmark.png", dpi=150)
    plt.close(fig)


def git_version():
    root = experiments.ROOT.parent
    try:
        head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=root, capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--", "roarm", "firmware", "scripts"], cwd=root,
                               capture_output=True, text=True).stdout.strip()
        return head + ("+changes" if dirty else "")
    except OSError:
        return "?"


def record(run, results, label, meta):
    stamp = time.strftime("%Y-%m-%d %H:%M")
    rel = run.relative_to(experiments.ROOT).as_posix()
    header = ("# Up-over-down benchmark history\n\n"
              "`scripts/benchmark.py`: 4 spots 20 mm apart (x 190, y -30..+30), 20 mm dips, 15 mm/s, there and back "
              "twice. mm unless noted, measured minus commanded tip from the encoders. Lower is better; inward: "
              "closer to 0 is better.\n\n"
              "| when | mode | change | bottom mean / worst | top mean / worst | repeat | path worst | inward worst "
              "| rough | code | run |\n|---|---|---|---|---|---|---|---|---|---|---|\n")
    lines = []
    for mode, r in results.items():
        s = r["summary"]
        lines.append(f"| {stamp} | {mode} | {label or ''} | {s['bottom_mean']:.2f} / {s['bottom_worst']:.2f} | "
                     f"{s['top_mean']:.2f} / {s['top_worst']:.2f} | {s['repeat_worst']:.2f} | {s['path_worst']:.2f} | "
                     f"{s['inward_worst']:+.2f} | {s['rough']:.1f} | {meta['code']} | [{rel}]({rel}/) |\n")
    text = HISTORY.read_text(encoding="utf-8") if HISTORY.exists() else header
    HISTORY.write_text(text + "".join(lines), encoding="utf-8")
    table = "".join(lines).replace(f"| [{rel}]({rel}/) |", "|")
    (run / "README.md").write_text(
        f"# Up-over-down benchmark, {stamp}\n\n{label or ''}\n\n![benchmark](benchmark.png)\n\n"
        f"Code {meta['code']}, compensation from `{meta['compensation_source']}`, shoulder trim {meta['shoulder_trim']}.\n\n"
        "| when | mode | change | bottom mean / worst | top mean / worst | repeat | path worst | inward worst | rough | code |\n"
        "|---|---|---|---|---|---|---|---|---|---|\n" + table +
        "\nFiles: `data.json` (scores, every dip), `trials/<mode>.npz` (telemetry + commanded path).\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("comp", "plain", "both"), default="comp")
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--label", default="", help="what changed since the last run (goes in the history)")
    args = ap.parse_args()
    modes = ["plain", "comp"] if args.mode == "both" else [args.mode]
    for i in range(len(ROW) - 1):
        for a, b in ((at(i, Z_UP), at(i + 1, Z_UP)), (at(i, Z_UP), at(i, Z_DOWN))):
            if M.check_line(a, b, PITCH):
                raise SystemExit("benchmark path would hit a joint limit")
    comp = tuning.load_compensation() if "comp" in modes else None
    if "comp" in modes and comp is None:
        raise SystemExit("No compensation in roarm/tuning.json.")
    arm = RoArm()
    if "." in arm.target:
        raise SystemExit("Run over USB (telemetry).")
    run = experiments.new_run("benchmark")
    (run / "trials").mkdir()
    tuned = json.loads(tuning.PATH.read_text())
    meta = {"code": git_version(), "label": args.label, "rounds": args.rounds, "shoulder_trim": arm.shoulder_trim(),
            "compensation_source": tuned.get("compensation_source", tuned.get("source")),
            "spots": [at(i, Z_UP).tolist() for i in range(len(ROW))], "z_down": Z_DOWN, "speed_mm_s": SPEED}
    fb = check(arm)
    roll, grip, rate = fb["r"], fb["g"], M.rate_for(arm)
    send = lambda q: arm.move(q, spd=0, acc=0)
    plain = M.Mover(arm, None, PITCH, roll, grip, rate)
    results, prev = {}, None
    print(f"Benchmark ({', '.join(modes)}). Results -> {run}", flush=True)
    try:
        arm.reset_pid()
        above = at(1, Z_UP + 10)   # lead-in: above spot 2, down, over to spot 1
        for mode in modes:
            mover = M.Mover(arm, comp if mode == "comp" else None, PITCH, roll, grip, rate)
            if prev is None:
                M.go_to_pose(arm, above, PITCH, roll, grip, rate)
                prev = plain
            else:
                prev.move(at(0, Z_UP), above)
            M.stream(send, prev.command(above, prev.dir), mover.command(above, mover.dir), 0.5, rate)
            time.sleep(0.3)
            mover.move(above, at(1, Z_UP))
            mover.move(at(1, Z_UP), at(0, Z_UP))
            time.sleep(0.5)
            check(arm)
            tel, cmd, moves = run_mode(arm, mover, args.rounds)
            check(arm)
            summary, detail = score(tel, cmd, moves)
            results[mode] = {"tel": tel, "cmd": cmd, "summary": summary, "detail": detail}
            np.savez_compressed(run / "trials" / f"{mode}.npz", cmd=cmd, **tel)
            print(f"{mode:5s}: " + "  ".join(f"{k} {summary[k]:+.2f}" if k == "inward_worst" else f"{k} {summary[k]:.2f}"
                                             for k in SCORES), flush=True)
            prev = mover
        prev.move(at(0, Z_UP), np.array([180.0, 0.0, -25.0]))
        meta["finished"] = "ok"
    except (KeyboardInterrupt, RuntimeError, ValueError) as e:
        print(f"Stopped: {e!r}", flush=True)
        meta["finished"] = f"stopped: {e!r}"
    finally:
        try:
            arm.reset_pid()
        finally:
            arm.close()
        (run / "data.json").write_text(json.dumps(
            {"meta": meta, **{m: {"summary": r["summary"], **r["detail"]} for m, r in results.items()}}, indent=1))
        if results and meta.get("finished") == "ok":
            figure(run, results)
            record(run, results, args.label, meta)
            print(f"Added to {HISTORY}", flush=True)
        print(f"Saved {run}", flush=True)


if __name__ == "__main__":
    main()
