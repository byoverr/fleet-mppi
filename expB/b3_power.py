"""B3: more clutter scenes (new seed) for the four key variants, for statistical power."""
import json, sys
from concurrent.futures import ProcessPoolExecutor
import b2_clutter as B

B.VARIANTS = {k: B.VARIANTS[k] for k in ("gauss K400", "gauss K16", "knots K16", "pursuit K16 wo6")}
B.VARIANTS["pursuit K16"] = dict(samples=16, ancillary="pursuit")

def main():
    out, workers, n = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    scenes = B.make_scenes(n, seed=11)
    jobs = [(i, sc, lab, s) for i, sc in enumerate(scenes) for lab in B.VARIANTS for s in B.SEEDS]
    rows = []
    with ProcessPoolExecutor(workers) as ex:
        for k, r in enumerate(ex.map(B.run, jobs, chunksize=1)):
            rows.append(r)
            if k % 50 == 0: print(k, "/", len(jobs), flush=True)
    json.dump(dict(scenes=scenes, rows=rows), open(out, "w"))

if __name__ == "__main__":
    main()
