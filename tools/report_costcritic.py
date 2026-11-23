"""Recalculate the comparison table from Google Benchmark repetitions."""

import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "nav2/current"
BATCHES = (32, 125, 2000)
SCENES = ("Коридор", "У стены")


def median_cpu(path, masks):
    runs = defaultdict(dict)
    for row in json.loads(path.read_text(encoding="utf-8"))["benchmarks"]:
        if row["run_type"] != "iteration":
            continue
        fields = dict(p.split(":", 1) for p in row["run_name"].split("/") if ":" in p)
        key = (int(fields["scene"]), int(fields["K"]), int(fields["mask"]))
        index = row["repetition_index"]
        if fields["threads"] != "1" or row["time_unit"] != "us" or row["repetitions"] != 5:
            raise ValueError(f"{path}: unexpected settings for {row['run_name']}")
        if index in runs[key]:
            raise ValueError(f"{path}: repeated index {index} for {key}")
        runs[key][index] = row["cpu_time"] / 1000

    expected = {(scene, k, mask) for scene in range(len(SCENES)) for k in BATCHES for mask in masks}
    if runs.keys() != expected:
        raise ValueError(f"{path}: benchmark configurations differ from the experiment")
    for key, values in runs.items():
        if values.keys() != set(range(5)):
            raise ValueError(f"{path}: incomplete repetitions for {key}")
    return {key: statistics.median(values.values()) for key, values in runs.items()}


def load_series(directory):
    return median_cpu(directory / "upstream.json", (0,)), median_cpu(directory / "paired.json", (0, 1))


def print_table(upstream, paired):
    print("| Сцена | Траекторий | Штатный Nav2, мс CPU | Фильтр выкл., мс CPU | Фильтр вкл., мс CPU |")
    print("|---|---:|---:|---:|---:|")
    for scene, title in enumerate(SCENES):
        for k in BATCHES:
            times = upstream[scene, k, 0], paired[scene, k, 0], paired[scene, k, 1]
            cells = " | ".join(f"{value:.3f}".replace(".", ",") for value in times)
            print(f"| {title} | {k} | {cells} |")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, nargs="?", default=DATA, help="directory with upstream.json and paired.json")
    print_table(*load_series(parser.parse_args().directory))
