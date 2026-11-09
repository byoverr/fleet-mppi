"""Compare the C++ Sobol generator (sobol_bank.hpp, via sobol_check.cpp) with SciPy:
unscrambled integer points, the inverse normal CDF, and the block-mean error of scrambled banks
against SciPy banks (make_qmc_bank.py) and pseudo-random ones.

  uv run --with scipy python sobol_compare.py CPP_OUT_DIR SCIPY_BANK_DIR
"""
import sys, warnings, numpy as np
from scipy.stats import qmc, norm
warnings.simplefilter("ignore")
D = sys.argv[1]
cpp = np.fromfile(f"{D}/plain_u32.bin", dtype=np.uint32).reshape(4096, 112).astype(np.int64)
ref = (qmc.Sobol(112, scramble=False, bits=30).random(4096) * 2**30).astype(np.int64)
print("unscrambled points identical to SciPy:", np.array_equal(cpp, ref), " mismatching entries:", int((cpp != ref).sum()))
q = np.fromfile(f"{D}/invnorm.bin", dtype=np.float64)
p = np.r_[np.arange(1, 100000) / 100000.0, 0.5 / 2**30]
err = np.abs(q - norm.ppf(p))
print(f"invNormal max abs error vs scipy norm.ppf: {err.max():.2e} (tail point {q[-1]:.6f} vs {norm.ppf(p[-1]):.6f})")
rng = np.random.default_rng(0)
for kind, pat in (("C++ sobol", f"{D}/cpp_sobol_%d.bin"), ("SciPy sobol", sys.argv[2] + "/sobol_%d.bin"), ("gauss", sys.argv[2] + "/gauss_%d.bin")):
    row = []
    for K in (125, 32):
        vals = []
        for s in range(1, 6):
            z = np.fromfile(pat % s, dtype=np.float32).reshape(4096, 112)
            st = rng.integers(0, 4096 - K + 1, 400)
            vals.append(np.stack([z[r:r + K].mean(0) for r in st]).std(0).mean() * np.sqrt(K))
        row.append(f"K={K}: {np.mean(vals):.2f}")
    z = np.fromfile(pat % 1, dtype=np.float32).reshape(4096, 112)
    print(f"{kind:12s} block-mean error vs MC  {'  '.join(row)}   std {z.std():.4f}  |col mean| max {np.abs(z.mean(0)).max():.4f}")
