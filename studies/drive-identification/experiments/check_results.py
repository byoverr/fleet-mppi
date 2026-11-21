"""Check the report's deterministic outcomes against archived and repeated runs."""
from collections import Counter
from pathlib import Path
import itertools
import json
from statistics import median

DATA = Path(__file__).resolve().parents[1] / "results"

def rows(name):
    return [json.loads(s) for s in (DATA / name).read_text().splitlines() if s.strip()]

main = rows("closed_loop.jsonl")
cases = {(r["scene"], r["seed"], r["model"], r["K"]) for r in main}
expected = set(itertools.product(("blocked", "bypass"), range(301,305), ("nominal","arx","oe"), (100,400)))
assert cases == expected and len(main) == len(expected)
print("Closed-loop successes / numbers of cases, by scene and model")
for sc, model in itertools.product(("blocked","bypass"), ("nominal","arx","oe")):
    group = [r for r in main if (r["scene"],r["model"]) == (sc,model)]
    print(sc, model, sum(r["success"] for r in group), "/", len(group))
old = {r["job"]: r for r in rows("historical_432.jsonl")}
repeated = rows("repeat_432.jsonl")
assert len(old) == len(repeated) == 432
assert set(old) == {r["job"] for r in repeated}
for field in ("status","success","collisions","srv_calls","min_gap","sim_time"):
    changes = sum(old[r["job"]][field] != r[field] for r in repeated)
    print(field, "changed on repeat", changes, "of",len(old))
    assert changes == 0, field
bench = (DATA / "nav2-summary.txt").read_text().splitlines()
measurements = {}
for config, record in zip(bench[::2], bench[1::2],strict=True):
    run, wall, build = config.split()
    d = dict(x.split("=",1) for x in record.split())
    assert (d["robots"],d["workers"],d["batch"]) == ("8","1","2000")
    key=(int(run),int(wall),build)
    assert key not in measurements
    measurements[key]=float(d["robots@20Hz"])
assert len(measurements)==12
for wall in (0,1):
    values=[median(measurements[r,wall,build] for r in (1,2,3)) for build in ("ws_main","ws")]
    print("Nav2 wall share",wall,"median equivalents",values,"ratio",round(values[1]/values[0],2))
