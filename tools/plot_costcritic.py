"""Summarize the pinned Nav2 benchmark and render the comparison used in README."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "nav2/current"
BATCHES = (32, 125, 2000)
SCENES = ("Коридор", "У стены")


def median_cpu(file):
    rows = json.loads((DATA / file).read_text())["benchmarks"]
    result = {}
    for row in rows:
        if row.get("aggregate_name") != "median":
            continue
        fields = dict(p.split(":", 1) for p in row["run_name"].split("/") if ":" in p)
        key = (int(fields["scene"]), int(fields["K"]), int(fields["mask"]))
        assert key not in result and fields["threads"] == "1"
        assert row["time_unit"] == "us" and row["repetitions"] == 5
        result[key] = row["cpu_time"] / 1000
    return result


def main():
    upstream = median_cpu("upstream.json")
    paired = median_cpu("paired.json")
    assert len(upstream) == 6 and len(paired) == 12
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8), layout="constrained", sharey=True)
    print("| Сцена | Траекторий | Штатный Nav2, мс CPU | Фильтр выкл., мс CPU | Фильтр вкл., мс CPU |")
    print("|---|---:|---:|---:|---:|")
    for scene, (title, ax) in enumerate(zip(SCENES, axes, strict=True)):
        for k in BATCHES:
            times = [upstream[scene, k, 0], paired[scene, k, 0], paired[scene, k, 1]]
            print(f"| {title} | {k} | " + " | ".join(f"{t:.3f}".replace(".", ",") for t in times) + " |")
        for label, source, mask, style in [
            ("Штатный Nav2", upstream, 0, "o-"),
            ("Опытная сборка, фильтр выкл.", paired, 0, "x--"),
            ("Опытная сборка, фильтр вкл.", paired, 1, "s-"),
        ]:
            ax.plot(range(len(BATCHES)), [source[scene,k,mask] for k in BATCHES], style, label=label)
        ax.set(title=title, xlabel="Траекторий за вызов", xticks=range(len(BATCHES)), xticklabels=BATCHES, yscale="log")
        ax.grid(axis="y", alpha=.2)
    axes[0].set_ylabel("CPU-время вызова, мс (лог. шкала)")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="outside lower center", ncols=3, frameon=False, fontsize=9)
    fig.savefig(ROOT / "docs/costcritic.png", dpi=175)
    plt.close(fig)


if __name__ == "__main__":
    main()
