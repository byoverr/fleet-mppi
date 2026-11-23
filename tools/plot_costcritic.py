"""Summarize the pinned Nav2 benchmark and render the comparison used in README."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from report_costcritic import BATCHES, DATA, ROOT, SCENES, load_series, print_table


def main():
    upstream, paired = load_series(DATA)
    print_table(upstream, paired)
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8), layout="constrained", sharey=True)
    for scene, (title, ax) in enumerate(zip(SCENES, axes, strict=True)):
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
