"""Upper bound for choosing the faster fixed mode per archived footprint/scene."""
from collections import defaultdict
import json
from pathlib import Path
import re

source = Path(__file__).resolve().parents[1] / "nav2/results/footprint_sweep.json"
pairs = defaultdict(dict)
for row in json.loads(source.read_text())["benchmarks"]:
    if row["aggregate_name"] != "median":
        continue
    assert row["time_unit"] == "us"
    match = re.search(r"/robot:(\d+)/scene:(\d+)/bitmask:([01])/", row["name"])
    robot, scene, mask = map(int, match.groups())
    modes = pairs[robot, scene]
    assert mask not in modes
    modes[mask] = row["cpu_time"]
assert len(pairs) == 12 and all(set(modes) == {0, 1} for modes in pairs.values())
fixed = [sum(modes[m] for modes in pairs.values()) for m in (0, 1)]
oracle = sum(min(modes.values()) for modes in pairs.values())
wins = sum(modes[1] < modes[0] for modes in pairs.values())
print(f"Masks use less CPU in {wins}/{len(pairs)} archived cases.")
print(f"Equal case weights, zero switch cost, mean CPU microseconds per call:")
print(f"exact {fixed[0]/len(pairs):.2f}, mask {fixed[1]/len(pairs):.2f}, oracle {oracle/len(pairs):.2f}")
print(f"Oracle saving vs better fixed mode: {100*(1-oracle/min(fixed)):.3f}%")
