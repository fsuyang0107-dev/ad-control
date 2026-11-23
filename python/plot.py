#!/usr/bin/env python3
"""Plot adc_sim CSV output.

Usage:
    python3 python/plot.py --out docs/figures/x.png results/a.csv [results/b.csv ...]

Produces a 3-panel figure:
    1) top view: reference path vs vehicle path
    2) lateral error e_y over time
    3) speed tracking: v_ref vs v
"""

import argparse
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path):
    """Read an adc_sim CSV into {column: [float, ...]}."""
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        cols = list(reader.fieldnames or [])
        data = {k: [] for k in cols}
        for row in reader:
            for k in cols:
                data[k].append(float(row[k]))
    return data


def main():
    ap = argparse.ArgumentParser(description="Plot adc_sim CSV output")
    ap.add_argument("csv", nargs="+", help="one or more CSV files")
    ap.add_argument("--out", required=True, help="output PNG path")
    ap.add_argument("--title", default="", help="figure title")
    ap.add_argument("--labels", default="", help="comma separated legend labels")
    args = ap.parse_args()

    names = [os.path.splitext(os.path.basename(p))[0] for p in args.csv]
    if args.labels:
        labels = [x.strip() for x in args.labels.split(",")]
        if len(labels) != len(names):
            raise SystemExit("--labels count must match number of CSV files")
        names = labels
    runs = [(n, load(p)) for n, p in zip(names, args.csv)]
    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.4))

    # ---- panel 1: top view ----
    ax = axes[0]
    ref = runs[0][1]
    styles = ["-", "--", "-.", ":"]
    ax.plot(ref["ref_x"], ref["ref_y"], "--", color="0.55", lw=1.6, label="reference")
    for i, (name, d) in enumerate(runs):
        ax.plot(d["x"], d["y"], ls=styles[i % len(styles)], lw=1.8, label=name)
        ax.plot(d["x"][0], d["y"][0], "o", ms=6, mfc="white", mec="0.2")
        ax.plot(d["x"][-1], d["y"][-1], "s", ms=6, mfc="white", mec="0.2")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Top view (vertical scale exaggerated)")
    ax.set_aspect("auto")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper left")

    # ---- panel 2: lateral error ----
    ax = axes[1]
    ax.axhspan(-0.05, 0.05, color="0.92", zorder=0, label="+/- 5 cm")
    ax.axhline(0.0, color="0.4", lw=0.8, zorder=1)
    for i, (name, d) in enumerate(runs):
        ax.plot(d["t"], d["e_y"], ls=styles[i % len(styles)], lw=1.6, label=name, zorder=2)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("lateral error $e_y$ [m]")
    ax.set_title("Lateral tracking error")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="center right")

    # ---- panel 3: speed ----
    ax = axes[2]
    ax.plot(ref["t"], ref["v_ref"], "--", color="0.55", lw=1.4, label="v_ref")
    for i, (name, d) in enumerate(runs):
        ax.plot(d["t"], d["v"], ls=styles[i % len(styles)], lw=1.6, label=name)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("speed [m/s]")
    ax.set_title("Speed tracking")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")

    if args.title:
        fig.suptitle(args.title, fontsize=13)
    fig.tight_layout()

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    fig.savefig(args.out, dpi=130, bbox_inches="tight")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
