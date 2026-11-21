"""Plot process CPU time and wall time from archived single-critic benchmarks."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "nav2/results"
VARIANTS = [
    ("Nav2 main · 1 поток", "bench_main", 1, 0),
    ("PR #6228 · 1 поток", "bench_pr", 1, 0),
    ("PR #6228 · 8 потоков", "bench_pr", 8, 0),
    ("OR-таблица и маски · 1 поток", "bench_box", 1, 1),
]

def measurement(source, threads, mask, scene):
    rows = json.loads((RESULTS / f"{source}.json").read_text())["benchmarks"]
    prefix = f"BM_CostCritic/threads:{threads}/scene:{scene}/bitmask:{mask}/"
    row, = [r for r in rows if r["name"].startswith(prefix) and r["aggregate_name"] == "median"]
    assert row["time_unit"] == "us" and row["repetitions"] == 3
    return row["real_time"] / 1000, row["cpu_time"] / 1000

def main():
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.5), sharey=True, layout="constrained")
    for scene, (ax, title) in enumerate(zip(axes, ["Коридор", "У стены"], strict=True)):
        for i, (label, source, threads, mask) in enumerate(VARIANTS):
            wall, cpu = measurement(source, threads, mask, scene)
            print(f"{title:8}  {label:30}  wall {wall:5.2f} ms  CPU {cpu:5.2f} ms")
            for shift, value, color, key in [(-.18, wall, "#8b9aa2", "Время до ответа"),
                                            (.18, cpu, "#263f50", "CPU-время процесса")]:
                ax.barh(i+shift, value, height=.32, color=color, label=key if i == 0 else None)
                ax.text(value+.17, i+shift, f"{value:.2f}", va="center", fontsize=8)
        ax.set(title=title, xlabel="Миллисекунды на вызов", yticks=range(len(VARIANTS)),
               yticklabels=[v[0] for v in VARIANTS], xlim=(0, 28 if scene else 6.6))
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].invert_yaxis()
    fig.legend(*axes[0].get_legend_handles_labels(), loc="outside lower center", ncols=2, frameon=False)
    fig.savefig(ROOT / "docs/costcritic.png", dpi=175)
    plt.close(fig)

if __name__ == "__main__":
    main()
