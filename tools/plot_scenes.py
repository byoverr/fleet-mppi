"""Draw the CostCritic inputs exported with --export-scenes."""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon

from report_costcritic import DATA, ROOT


def plot(source, target):
    scenes = defaultdict(lambda: defaultdict(list))
    with source.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            key = row["kind"], int(row["id"])
            scenes[int(row["scene"])][key].append((float(row["x"]), float(row["y"])))

    wall, path, outline = "#414851", "#247a8c", "#503e2f"
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.15), sharey=True, layout="constrained")
    for scene, (ax, title) in enumerate(zip(axes, ("Коридор с коробкой", "Вдоль стены"))):
        entries = scenes[scene]
        cells = [[(x + dx, y + dy) for dx, dy in
                  ((-.05, -.05), (.05, -.05), (.05, .05), (-.05, .05))]
                 for x, y in entries["cell", 0]]
        ax.add_collection(PolyCollection(cells, facecolors=wall, edgecolors="none", antialiaseds=False))
        ax.add_patch(Polygon(entries["footprint", 0], facecolor="#f0e6da", edgecolor=outline))
        for i in range(32):
            xs, ys = zip(*entries["path", i])
            ax.plot(xs, ys, color=path, lw=.8, alpha=.62)
        ax.set(aspect="equal", xlim=(9.5, 20.5), ylim=(11.4, 18.8), title=title,
               xlabel="x, м", xticks=(10, 12, 14, 16, 18, 20), yticks=(12, 14, 16, 18))
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("y, м")
    fig.legend(handles=[Patch(color=wall, label="Клетки препятствий"),
                        Patch(facecolor="#f0e6da", edgecolor=outline, label="Корпус в начальной позе"),
                        Line2D([], [], color=path, label="Траектории центра")],
               loc="outside lower center", ncols=3, frameon=False)
    fig.savefig(target, dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DATA / "scenes.csv")
    parser.add_argument("--output", type=Path, default=ROOT / "docs/scenes.png")
    args = parser.parse_args()
    plot(args.input, args.output)
