"""Build binary scene files for bench.cpp: map (uint8 cost: 0 free / 254 lethal / 255 unknown),
Euclidean distance-to-lethal (m, cell centres, scipy EDT), poses (N,3) float32, footprint polygon.
Scenes: real rollouts (TurtleBot 12-gon), real rollouts with geometry scaled 3x (AMR 0.7x0.5 rect),
and two synthetic MPPI-like wall scenes (AMR 0.7x0.5 @0.05 m, big 3.9x1.5 @0.1 m)."""
import sys, struct, numpy as np
from scipy.ndimage import distance_transform_edt

LAB = sys.argv[1] if len(sys.argv) > 1 else "../data"
OUT = sys.argv[2] if len(sys.argv) > 2 else "scenes"
import os; os.makedirs(OUT, exist_ok=True)


def write(name, cost, res, origin, poses, foot, K, T):
    lethal = cost >= 254
    dist = (distance_transform_edt(~lethal) * res).astype(np.float32)
    ny, nx = cost.shape
    poses = np.ascontiguousarray(poses.reshape(-1, 3), dtype=np.float32)
    foot = np.asarray(foot, dtype=np.float64)
    with open(f"{OUT}/{name}.bin", "wb") as f:
        f.write(struct.pack("<iiddd iii i", nx, ny, res, origin[0], origin[1], len(poses) // (K * T), K, T, len(foot)))
        f.write(cost.astype(np.uint8).tobytes()); f.write(dist.tobytes())
        f.write(poses.tobytes()); f.write(foot.tobytes())
    print(f"{name}: map {nx}x{ny} @{res} m, poses {len(poses)} (K={K},T={T}), lethal {lethal.mean():.3f}")


def rect(L, W):
    return [(L / 2, W / 2), (-L / 2, W / 2), (-L / 2, -W / 2), (L / 2, -W / 2)]


ngon = [(0.105 * np.cos(a), 0.105 * np.sin(a)) for a in np.arange(12) * 2 * np.pi / 12]

for sc in ["bypass", "blocked", "open"]:
    d = np.load(f"{LAB}/rollouts_{sc}.npz")
    tr, L, org, res = d["traj"], d["lethal"], d["origin"], float(d["res"])
    C, K, T, _ = tr.shape
    cost = np.where(L, 254, 0).astype(np.uint8)
    write(f"real_{sc}_tb", cost, res, org, tr, ngon, K, T)
    # same geometry and rollouts scaled 3x, resampled at the same 0.05 m: a TB-sized robot becomes ~0.63 m
    s = 3
    cost3 = np.repeat(np.repeat(cost, s, 0), s, 1)
    tr3 = tr.copy(); tr3[..., :2] *= s
    write(f"real_{sc}_amr3x", cost3, res, org * s, tr3, rect(0.7, 0.5), K, T)


def wall_scene(name, res, size, wall_y, band, d0rng, v0, wstd, foot, seed, C=5, K=2000, T=56, step=2):
    rng = np.random.default_rng(seed)
    n = int(round(size / res))
    cost = np.zeros((n, n), np.uint8)
    wy = int(wall_y / res)
    cost[wy:wy + 2, :] = 254                                  # long wall (2 cells thick)
    for _ in range(14):                                        # small posts in front of the wall
        px, py = rng.uniform(1, size - 1), rng.uniform(*band)
        cost[int(py / res):int(py / res) + 2, int(px / res):int(px / res) + 2] = 254
    dt = 0.05
    out = np.zeros((C, K, T // step, 3), np.float32)
    for c in range(C):
        x0, y0 = rng.uniform(1.5, size / 2), wall_y - rng.uniform(*d0rng)
        th0, w0 = rng.normal(0, 0.1), rng.normal(0, 0.1)
        v = v0 + 0.2 * rng.standard_normal((K, T))             # Nav2 default vx_std 0.2
        w = w0 + wstd * rng.standard_normal((K, T))
        th = th0 + np.cumsum(w * dt, 1)
        x = x0 + np.cumsum(v * np.cos(th) * dt, 1)
        y = y0 + np.cumsum(v * np.sin(th) * dt, 1)
        out[c] = np.stack([x, y, th], -1)[:, step - 1::step]
    write(name, cost, res, (0.0, 0.0), out, foot, K, T // step)


# typical AMR 0.7x0.5 m at 0.05 m, 10x10 m local costmap, driving 0.3-0.5 m from a wall (circumscribed r = 0.43)
wall_scene("wall_amr", 0.05, 10.0, 6.0, (5.3, 5.9), (0.30, 0.50), 0.5, 0.4, rect(0.7, 0.5), 1)
# PR #6228-like: 0.1 m map, 3.9x1.5 m rectangle, 20x20 m, 0.9-1.6 m from the wall (circumscribed r = 2.09)
wall_scene("wall_big", 0.10, 20.0, 12.0, (10.5, 11.8), (0.9, 1.6), 1.0, 0.2, rect(3.9, 1.5), 2)
