import sys
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.dirname(__import__("os").path.abspath(__file__))))
from concurrent.futures import ProcessPoolExecutor
from mpp.runner import EpisodeConfig
from netsim import NetConfig, LinkConfig, run_net_episode
PL = {"none": None, "mid": (0.4, 0.25, 1.0, 0.8), "strong": (0.8, 0.5, 1.0, 0.7)}
def job(a):
    sc, pl, pmk, ev, d = a
    P = PL[pl]
    pm = P if pmk == "model" else None
    r = run_net_episode(EpisodeConfig(scenario=sc, local_planner="mppi", seed=1),
                        NetConfig(strategy="S3", link=LinkConfig(delay=d), server_every=ev, plant=P, planner_model=pm))
    return (sc, pl, pmk, ev, d, r["success"], r["time"], r["collisions"], r["min_gap"], r["srv_calls"], r["safety_stops"])
jobs = [(sc, pl, pmk, ev, d) for sc in ("bypass", "blocked", "open") for pl in ("mid", "strong")
        for pmk in ("nominal", "model") for ev in (1, 4) for d in (0.0, 0.15)]
if __name__ == "__main__":
    with ProcessPoolExecutor(8) as ex:
        for r in ex.map(job, jobs):
            print(*r, sep="\t", flush=True)
