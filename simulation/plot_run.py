#!/usr/bin/env python3
"""Generate a top-down buoy detection map for a sim_tests run.

Usage:
    python3 simulation/plot_run.py                        # latest run
    python3 simulation/plot_run.py simulation/sim_tests/run_3
    python3 simulation/plot_run.py --run 3
"""
import argparse
import csv
import json
import math
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_DEFAULT = os.path.join(REPO, "simulation/gazebo/worlds/robotx_uav_course.sdf")
SIM_TESTS_DIR = os.path.join(REPO, "simulation/sim_tests")

# ------------------------------------------------------------------ colours
COLOUR_MAP = {
    "red":   "#E05555",
    "green": "#44BB55",
    "blue":  "#4488EE",
    "light": "#AAAAAA",
}
BG = "#2C1B0E"
FG = "#E8DCC8"
GRID = "#4A3020"


# ------------------------------------------------------------------ helpers
def parse_world(world_path):
    """Return list of {name, color, east, north} from SDF."""
    with open(world_path, encoding="utf-8") as f:
        text = f.read()
    buoys = []
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.DOTALL):
        name, body = m.group(1), m.group(2)
        color = (
            "green" if "green" in name else
            "red"   if "red"   in name else
            "light" if "light_buoy" in name else None
        )
        if color is None:
            continue
        pose = re.search(r"<pose>\s*([-\d.eE]+)\s+([-\d.eE]+)", body)
        if not pose:
            continue
        buoys.append({"name": name, "color": color,
                      "east": float(pose.group(1)), "north": float(pose.group(2))})
    return buoys


def load_detections(csv_path):
    """Return list of {color, est_east_m, est_north_m, confidence}."""
    rows = []
    if not os.path.isfile(csv_path):
        return rows
    with open(csv_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    "color":      r["color"],
                    "east":       float(r["est_east_m"]),
                    "north":      float(r["est_north_m"]),
                    "confidence": float(r["confidence"]),
                })
            except (KeyError, ValueError):
                pass
    return rows


def match_detections(detections, buoys, match_radius=3.0):
    """Return {buoy_name: [(east, north, conf), ...]}."""
    matched = {b["name"]: [] for b in buoys}
    for d in detections:
        best, best_dist = None, match_radius
        for b in buoys:
            if b["color"] != d["color"]:
                continue
            dist = math.hypot(d["east"] - b["east"], d["north"] - b["north"])
            if dist < best_dist:
                best_dist, best = dist, b
        if best:
            matched[best["name"]].append((d["east"], d["north"], d["confidence"]))
    return matched


def find_latest_run():
    if not os.path.isdir(SIM_TESTS_DIR):
        return None
    best_n, best_dir = -1, None
    for name in os.listdir(SIM_TESTS_DIR):
        m = re.match(r"run_(\d+)$", name)
        if m:
            n = int(m.group(1))
            if n > best_n:
                best_n, best_dir = n, os.path.join(SIM_TESTS_DIR, name)
    return best_dir


# ------------------------------------------------------------------ main plot
def make_map(run_dir, world_path, out_path=None):
    run_name = os.path.basename(run_dir)
    csv_path  = os.path.join(run_dir, "detections.csv")
    json_path = os.path.join(run_dir, "summary.json")
    out_path  = out_path or os.path.join(run_dir, "map.png")

    buoys      = parse_world(world_path)
    detections = load_detections(csv_path)
    matched    = match_detections(detections, buoys)

    summary = {}
    if os.path.isfile(json_path):
        with open(json_path, encoding="utf-8") as f:
            summary = json.load(f)

    duration   = summary.get("duration_s", 0)
    dur_ok     = summary.get("duration_ok", True)
    mean_err   = summary.get("mean_error_m")
    n_det_total = summary.get("n_detections", len(detections))
    run_n      = summary.get("run", run_name.replace("run_", ""))

    # ---- figure setup
    fig, ax = plt.subplots(figsize=(11, 7))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    for spine in ax.spines.values():
        spine.set_edgecolor(GRID)

    # ---- field bounds: pad around buoy positions
    all_east  = [b["east"]  for b in buoys] + [d["east"]  for d in detections]
    all_north = [b["north"] for b in buoys] + [d["north"] for d in detections]
    e_min, e_max = min(all_east) - 8,  max(all_east) + 8
    n_min, n_max = min(all_north) - 6, max(all_north) + 6

    ax.set_xlim(e_min, e_max)
    ax.set_ylim(n_min, n_max)
    ax.set_aspect("equal")
    ax.tick_params(colors=FG, labelsize=8)
    ax.xaxis.label.set_color(FG)
    ax.yaxis.label.set_color(FG)
    ax.set_xlabel("East (m)", color=FG, fontsize=9)
    ax.set_ylabel("North (m)", color=FG, fontsize=9)

    # ---- faint grid
    ax.grid(True, color=GRID, linewidth=0.5, linestyle="--", alpha=0.6)

    # ---- centreline (the drone's path along E at N=0)
    ax.axhline(0, color=GRID, linewidth=0.8, linestyle=":", alpha=0.5)

    # ---- gate markers (vertical lines at each gate East position)
    for gate_east in [10, 25, 40]:
        ax.axvline(gate_east, color=GRID, linewidth=0.6, linestyle=":", alpha=0.4)

    # ---- compass labels
    ax.text(e_min + 0.5, n_max - 0.5, "W", color=FG, fontsize=10, va="top", ha="left")
    ax.text(e_max - 0.5, n_max - 0.5, "E", color=FG, fontsize=10, va="top", ha="right")
    ax.text((e_min + e_max) / 2, n_max - 0.5, "N", color=FG,
            fontsize=9, va="top", ha="center", alpha=0.6)
    ax.text((e_min + e_max) / 2, n_min + 0.3, "S", color=FG,
            fontsize=9, va="bottom", ha="center", alpha=0.6)

    # ---- ground truth: hollow circles
    for b in buoys:
        c = COLOUR_MAP.get(b["color"], "#888888")
        ax.plot(b["east"], b["north"],
                "o", markersize=22, markerfacecolor="none",
                markeredgecolor=c, markeredgewidth=2.0, alpha=0.55, zorder=2)
        ax.text(b["east"] + 0.5, b["north"],
                f"GT\n{b['color']}", color=c, fontsize=6.5,
                va="center", ha="left", alpha=0.7)

    # ---- detections: all raw points as small dots, then mean as solid circle
    colour_counts = {}
    for b in buoys:
        pts = matched[b["name"]]
        if not pts:
            continue
        c = COLOUR_MAP.get(b["color"], "#888888")
        colour_counts[b["color"]] = colour_counts.get(b["color"], 0) + 1

        east_vals  = [p[0] for p in pts]
        north_vals = [p[1] for p in pts]
        mean_e = sum(east_vals) / len(east_vals)
        mean_n = sum(north_vals) / len(north_vals)

        # individual detections as faint small dots
        ax.scatter(east_vals, north_vals,
                   s=18, c=c, alpha=0.25, zorder=3, linewidths=0)

        # mean detected position as solid filled circle
        ax.plot(mean_e, mean_n,
                "o", markersize=16, color=c,
                markeredgecolor="white", markeredgewidth=0.8,
                alpha=0.92, zorder=5)
        ax.text(mean_e + 0.5, mean_n,
                b["color"], color="white", fontsize=8,
                va="center", ha="left", fontweight="bold",
                zorder=6)

        # error line from GT to detected mean
        gt_e, gt_n = b["east"], b["north"]
        err_m = math.hypot(mean_e - gt_e, mean_n - gt_n)
        ax.annotate("", xy=(mean_e, mean_n), xytext=(gt_e, gt_n),
                    arrowprops=dict(arrowstyle="-", color=c,
                                   lw=1.2, linestyle="dashed", alpha=0.5),
                    zorder=4)
        # error label at midpoint
        mx, mn = (mean_e + gt_e) / 2, (mean_n + gt_n) / 2
        ax.text(mx, mn, f"{err_m:.2f}m", color=c, fontsize=6,
                ha="center", va="center", alpha=0.8,
                bbox=dict(facecolor=BG, edgecolor="none", pad=1))

    # unmatched detections (no GT buoy within match radius)
    unmatched_dets = []
    matched_pts = set()
    for b in buoys:
        for pt in matched[b["name"]]:
            matched_pts.add((round(pt[0], 6), round(pt[1], 6)))
    for d in detections:
        key = (round(d["east"], 6), round(d["north"], 6))
        if key not in matched_pts:
            unmatched_dets.append(d)
    if unmatched_dets:
        ax.scatter([d["east"] for d in unmatched_dets],
                   [d["north"] for d in unmatched_dets],
                   s=12, c="#888888", alpha=0.3, zorder=3,
                   label=f"unmatched ({len(unmatched_dets)})")

    # ---- title
    status_str = "" if dur_ok else "  [DURATION TOO SHORT]"
    ax.set_title(f"Buoy Detection Map  --  {run_name}{status_str}",
                 color=FG, fontsize=13, pad=10, fontweight="bold")

    # ---- legend patches
    legend_items = []
    for color_name, count in sorted(colour_counts.items()):
        c = COLOUR_MAP.get(color_name, "#888888")
        legend_items.append(
            mpatches.Patch(facecolor=c, edgecolor="white",
                           linewidth=0.5, label=f"{color_name} x{count}"))
    gt_patch = mpatches.Patch(facecolor="none", edgecolor=FG,
                               linewidth=1.5, linestyle="--",
                               label="ground truth (hollow)")
    legend_items.append(gt_patch)

    leg = ax.legend(handles=legend_items, loc="lower left",
                    facecolor=BG, edgecolor=GRID, labelcolor=FG,
                    fontsize=8.5, framealpha=0.85,
                    bbox_to_anchor=(0.0, -0.18), ncol=len(legend_items))

    # ---- runtime + accuracy text below legend
    err_str = f"  mean err={mean_err:.2f}m" if mean_err is not None else ""
    info_line = f"t={duration:.1f}s{err_str}   detections={n_det_total}"
    fig.text(0.01, 0.01, info_line, color=FG, fontsize=9,
             va="bottom", ha="left")

    plt.tight_layout(rect=[0, 0.06, 1, 1])
    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"[plot_run] saved: {out_path}")
    return out_path


# ------------------------------------------------------------------ CLI
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", nargs="?", default=None,
                    help="Path to a sim_tests/run_N directory (default: latest run).")
    ap.add_argument("--run", type=int, default=None,
                    help="Run number to plot (alternative to positional arg).")
    ap.add_argument("--world", default=WORLD_DEFAULT,
                    help="Path to world SDF for ground truth.")
    ap.add_argument("--out", default=None,
                    help="Output PNG path (default: <run_dir>/map.png).")
    args = ap.parse_args()

    if args.run is not None:
        run_dir = os.path.join(SIM_TESTS_DIR, f"run_{args.run}")
    elif args.run_dir:
        run_dir = os.path.abspath(args.run_dir)
    else:
        run_dir = find_latest_run()

    if not run_dir or not os.path.isdir(run_dir):
        print(f"[plot_run] ERROR: run directory not found: {run_dir}", file=sys.stderr)
        sys.exit(1)

    make_map(run_dir, args.world, out_path=args.out)


if __name__ == "__main__":
    main()
