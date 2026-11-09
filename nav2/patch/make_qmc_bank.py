"""Write noise banks for the noise_bank_file option: ROWS rows of standard-normal float32, each row
2 * T values (wz for T steps, then vx for T steps), row-major.

  uv run --with scipy python make_qmc_bank.py OUT_DIR [rows] [T] [seed]

Writes sobol_SEED.bin (scrambled Sobol, Owen scrambling), halton_SEED.bin (scrambled Halton) and
gauss_SEED.bin (pseudo-random normal, the control that goes through the same loading path).
Uniform points are mapped through the inverse normal CDF, as in Halton-MPPI (eq. 11).
"""

import sys
from pathlib import Path

import numpy as np
from scipy.stats import norm, qmc

out = Path(sys.argv[1])
rows = int(sys.argv[2]) if len(sys.argv) > 2 else 4096
T = int(sys.argv[3]) if len(sys.argv) > 3 else 56
seed = int(sys.argv[4]) if len(sys.argv) > 4 else 1
d = 2 * T
out.mkdir(parents=True, exist_ok=True)
eps = 0.5 / rows


def save(name, z):
    z = z.astype(np.float32)
    z.tofile(out / name)
    print(f"{name}: {z.shape}, mean {z.mean():+.4f}, std {z.std():.4f}, "
          f"|col mean| max {np.abs(z.mean(0)).max():.4f}, col std range "
          f"{z.std(0).min():.3f}-{z.std(0).max():.3f}")


u = qmc.Sobol(d, scramble=True, seed=seed).random(rows)
save(f"sobol_{seed}.bin", norm.ppf(np.clip(u, eps, 1 - eps)))
u = qmc.Halton(d, scramble=True, seed=seed).random(rows)
save(f"halton_{seed}.bin", norm.ppf(np.clip(u, eps, 1 - eps)))
save(f"gauss_{seed}.bin", np.random.default_rng(seed).standard_normal((rows, d)))
