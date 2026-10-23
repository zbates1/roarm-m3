"""Rack that holds screen-printed electrodes (SPEs) upright for the arm to pick.

Writes, next to this file:
  spe_rack.stl        the rack: 4 x 3 = 12 slots, 10 x 0.5 mm, 10 mm deep
  spe_slot_test.stl   a small coupon with the slot at 5 clearances (print first)
  spe_rack.json       slot centres in the rack's frame, for pick scripts
  spe_rack.png        preview

    UV_CACHE_DIR=D:/.cache/uv uv run --with manifold3d --with trimesh python hardware/spe_rack/make_rack.py

Rack frame: origin at the chamfered corner, on the bottom; x across the strips'
faces (the way the jaws close), y along their 10 mm width, z up.

Spacing comes from the gripper and the arm's measured error, not from a fixed
number: see SLOT_PITCH_X / SLOT_PITCH_Y below and README.md.
"""
import json
import math
from pathlib import Path

import numpy as np
import trimesh
from manifold3d import Manifold

HERE = Path(__file__).resolve().parent

# Electrode (DropSens-style strip; change to match yours)
SPE_T, SPE_W, SPE_L = 0.5, 10.0, 34.0
# Slot, as asked: 10 x 0.5 mm. CLEARANCE is added to both; print spe_slot_test.stl to choose it.
SLOT_T, SLOT_W, CLEARANCE = 0.5, 10.0, 0.0
SLOT_DEPTH, FLOOR = 10.0, 3.0
LEAD_IN = 1.5          # funnel at the top of each slot: 1.5 mm deep, opening 1.5 mm wider per side
FACE_RELIEF = 0.0      # > 0: widen the middle of each slot by this much per side, so only the
EDGE_GRIP = 1.5        # strip's edges (EDGE_GRIP mm at each end) touch the rack, not its printed face

# Gripper (Waveshare doesn't publish these: measure the jaw tips and change them)
JAW_T = 6.0            # jaw thickness at the tip (x)
JAW_W = 20.0           # jaw width (y, along the strip's width)

# How far the jaws can be from where they're meant to be, per side (mm)
ERROR = {
    "arm, last 5 mm of the descent (benchmark 2026-10-18, worst, encoders)": 2.0,
    "not seen by the encoders (link flex, geometry)": 1.0,
    "where the rack sits (taught by touching it, or found by camera)": 1.0,
    "strip leaning in its slot": 0.5,
}
E = sum(ERROR.values())

# Across the faces: each gap between strips holds one jaw, with E of clearance on both sides.
SLOT_PITCH_X = math.ceil(SPE_T + JAW_T + 2 * E)
# Along the width: the jaw overhangs the strip's edges; it must stay E clear of the next row's strip,
# or it would pinch two strips at once.
SLOT_PITCH_Y = math.ceil(SPE_W + (JAW_W - SPE_W) / 2 + E)
COLS, ROWS = 4, 3
MARGIN = 6.0           # solid wall around the outer slots
CORNER = 6.0           # chamfer on one vertical corner: which way round the rack goes


def box(x0, y0, z0, x1, y1, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def slot_cut(cx, cy, t, w, top, depth):
    """Slot with its lead-in funnel, centred at (cx, cy), opening at z = top."""
    cut = box(cx - t / 2, cy - w / 2, top - depth, cx + t / 2, cy + w / 2, top + 1)
    if LEAD_IN > 0:
        lip = box(cx - t / 2 - LEAD_IN, cy - w / 2 - LEAD_IN, top, cx + t / 2 + LEAD_IN, cy + w / 2 + LEAD_IN, top + 1)
        neck = box(cx - t / 2, cy - w / 2, top - LEAD_IN - 0.01, cx + t / 2, cy + w / 2, top - LEAD_IN)
        cut += Manifold.batch_hull([lip, neck])
    if FACE_RELIEF > 0:
        r = t / 2 + FACE_RELIEF
        cut += box(cx - r, cy - w / 2 + EDGE_GRIP, top - depth, cx + r, cy + w / 2 - EDGE_GRIP, top + 1)
    return cut


def corner_cut(size, height):
    """Remove a right-angled triangle (legs `size`) from the corner at the origin."""
    s = size * math.sqrt(2)
    return Manifold.cube([s, s, height + 2], center=True).rotate([0, 0, 45]).translate([0, 0, height / 2])


def to_trimesh(m):
    mesh = m.to_mesh()
    t = trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)
    assert t.is_watertight, "mesh isn't watertight"
    return t


def rack():
    t, w = SLOT_T + CLEARANCE, SLOT_W + CLEARANCE
    span_x = (COLS - 1) * SLOT_PITCH_X + t + 2 * MARGIN
    span_y = (ROWS - 1) * SLOT_PITCH_Y + w + 2 * MARGIN
    side = math.ceil(max(span_x, span_y))       # square, grid centred
    height = FLOOR + SLOT_DEPTH
    x0 = (side - (COLS - 1) * SLOT_PITCH_X) / 2
    y0 = (side - (ROWS - 1) * SLOT_PITCH_Y) / 2
    centres = [(round(x0 + i * SLOT_PITCH_X, 3), round(y0 + j * SLOT_PITCH_Y, 3)) for j in range(ROWS) for i in range(COLS)]
    solid = box(0, 0, 0, side, side, height) - corner_cut(CORNER, height)
    for cx, cy in centres:
        solid -= slot_cut(cx, cy, t, w, height, SLOT_DEPTH)
    info = {
        "units": "mm",
        "frame": "origin at the chamfered corner, on the bottom; x across the strips' faces (jaw closing direction), "
                 "y along their width, z up",
        "size": [side, side, height], "top_z": height,
        "slot": {"t": t, "w": w, "depth": SLOT_DEPTH, "lead_in": LEAD_IN, "face_relief": FACE_RELIEF},
        "pitch": [SLOT_PITCH_X, SLOT_PITCH_Y],
        "slots": [{"n": k + 1, "x": cx, "y": cy} for k, (cx, cy) in enumerate(centres)],
        "strip_above_rack_mm": SPE_L - SLOT_DEPTH,
        "gripper_assumed": {"jaw_t": JAW_T, "jaw_w": JAW_W},
        "error_budget_per_side": ERROR, "error_total_per_side": E,
    }
    return solid, info


def slot_test():
    """Coupon: the slot at clearance 0, 0.1, 0.2, 0.3, 0.4 mm, smallest at the chamfered end."""
    clearances = [0.0, 0.1, 0.2, 0.3, 0.4]
    pitch, height = 6.0, FLOOR + SLOT_DEPTH
    length = (len(clearances) - 1) * pitch + 2 * 8.0
    width = SLOT_W + 2 * 5.0
    solid = box(0, 0, 0, length, width, height) - corner_cut(4.0, height)
    for k, c in enumerate(clearances):
        solid -= slot_cut(8.0 + k * pitch, width / 2, SLOT_T + c, SLOT_W + c, height, SLOT_DEPTH)
    return solid, clearances


def preview(solid, info, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    surface, ink, ink2, grid = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3de"
    jaw, clear = "#eb6834", "#2a78d6"             # validated categorical slots 2 and 1
    fig = plt.figure(figsize=(12, 5.4), facecolor=surface)
    ax = fig.add_subplot(1, 2, 1, facecolor=surface)
    s, top = info["size"], info["top_z"]
    row_y = info["slots"][0]["y"]
    # rotate 90 deg about x (z' = y, y' = -z), then slice horizontally at the row
    for loop in solid.rotate([90, 0, 0]).slice(row_y).to_polygons():
        loop = np.asarray(loop)
        ax.fill(loop[:, 0], -loop[:, 1], color=grid, edgecolor=ink2, linewidth=1)
    for sl in info["slots"][:COLS]:
        ax.add_patch(plt.Rectangle((sl["x"] - SPE_T / 2, top - SLOT_DEPTH), SPE_T, SPE_L, color=ink))
    # the jaws around strip 2, each E clear of the strip and of its neighbours
    cx = info["slots"][1]["x"]
    z0, z1 = top + 4, top - SLOT_DEPTH + SPE_L + 2
    for sign in (-1, 1):
        inner = cx + sign * (SPE_T / 2 + E)
        x_left = inner if sign > 0 else inner - JAW_T
        ax.add_patch(plt.Rectangle((x_left, z0), JAW_T, z1 - z0, color=jaw, alpha=0.85, linewidth=0))
        for xa, xb in ((cx + sign * SPE_T / 2, inner), (inner + sign * JAW_T, inner + sign * (JAW_T + E))):
            ax.add_patch(plt.Rectangle((min(xa, xb), z0), abs(xb - xa), z1 - z0, color=clear, alpha=0.18, linewidth=0))
    ax.text(0, z1 + 2.5, f"orange: jaws ({JAW_T:g} mm thick, assumed) around strip 2;  blue: {E:g} mm clearance",
            color=ink2, fontsize=9)
    ax.set_xlim(-3, s[0] + 3)
    ax.set_ylim(-3, z1 + 6)
    ax.set_aspect("equal")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.tick_params(colors=ink2)
    ax.set_xlabel("x (mm)", color=ink2)
    ax.set_ylabel("z (mm)", color=ink2)
    ax.set_title("Section through a row, strips standing in the slots", loc="left", color=ink, fontweight="bold")
    ax2 = fig.add_subplot(1, 2, 2, facecolor=surface)
    ax2.add_patch(plt.Polygon([(CORNER, 0), (s[0], 0), (s[0], s[1]), (0, s[1]), (0, CORNER)], fill=False,
                              edgecolor=ink2, linewidth=1.5))
    t, w = info["slot"]["t"], info["slot"]["w"]
    for sl in info["slots"]:
        ax2.add_patch(plt.Rectangle((sl["x"] - t / 2, sl["y"] - w / 2), t, w, color=ink))
        ax2.add_patch(plt.Rectangle((sl["x"] - t / 2 - LEAD_IN, sl["y"] - w / 2 - LEAD_IN), t + 2 * LEAD_IN,
                                    w + 2 * LEAD_IN, fill=False, edgecolor=grid, linewidth=1))
        ax2.text(sl["x"] + 1.8, sl["y"] + w / 2 - 1, str(sl["n"]), color=ink2, fontsize=9)
    p = info["slots"]
    ax2.annotate("", (p[1]["x"], p[0]["y"] - w / 2 - 3), (p[0]["x"], p[0]["y"] - w / 2 - 3),
                 arrowprops={"arrowstyle": "<->", "color": ink2})
    ax2.text((p[0]["x"] + p[1]["x"]) / 2, p[0]["y"] - w / 2 - 5.5, f"{SLOT_PITCH_X} mm", ha="center", color=ink2, fontsize=9)
    ax2.annotate("", (p[0]["x"] - 3, p[COLS]["y"]), (p[0]["x"] - 3, p[0]["y"]), arrowprops={"arrowstyle": "<->", "color": ink2})
    ax2.text(p[0]["x"] - 4, (p[0]["y"] + p[COLS]["y"]) / 2, f"{SLOT_PITCH_Y} mm", ha="right", va="center", color=ink2,
             fontsize=9, rotation=90)
    ax2.set_xlim(-8, s[0] + 4); ax2.set_ylim(-8, s[1] + 4)
    ax2.set_aspect("equal")
    for side in ("top", "right"):
        ax2.spines[side].set_visible(False)
    ax2.tick_params(colors=ink2)
    ax2.set_xlabel("x: jaws close this way (mm)", color=ink2)
    ax2.set_ylabel("y (mm)", color=ink2)
    ax2.set_title(f"Top view: {len(p)} slots {t:g} x {w:g} mm, {s[0]:g} x {s[1]:g} x {s[2]:g} mm", loc="left",
                  color=ink, fontweight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=surface)
    plt.close(fig)


def main():
    solid, info = rack()
    mesh = to_trimesh(solid)
    mesh.export(HERE / "spe_rack.stl")
    (HERE / "spe_rack.json").write_text(json.dumps(info, indent=1))
    coupon, clearances = slot_test()
    to_trimesh(coupon).export(HERE / "spe_slot_test.stl")
    preview(solid, info, HERE / "spe_rack.png")
    print(f"rack {info['size']} mm, {len(info['slots'])} slots, pitch {SLOT_PITCH_X} x {SLOT_PITCH_Y} mm "
          f"(error budget {E:g} mm per side), volume {mesh.volume / 1000:.1f} cm^3")
    print(f"slot test coupon: clearances {clearances} mm, smallest at the chamfered end")


if __name__ == "__main__":
    main()
