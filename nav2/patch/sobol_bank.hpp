// Scrambled Sobol points mapped to standard normal noise, for the MPPI noise bank (lab option).
//
// Points follow SciPy's scipy.stats.qmc.Sobol: Joe-Kuo direction numbers (sobol_joe_kuo.hpp),
// 30 bits, Gray-code order (point i = point i-1 XOR v[lowest zero bit of i-1]), and with
// scrambling a random linear matrix scramble (lower-triangular, unit diagonal, per dimension)
// plus a random digital shift. Uniform u = (x + 0.5) / 2^30 is mapped through the inverse normal
// CDF (Halley iterations on erfc, so accurate to double rounding).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "nav2_mppi_controller/tools/sobol_joe_kuo.hpp"

namespace mppi::sobol
{
constexpr int kBits = 30;

// v[d * kBits + k], k = 0 is the most significant bit.
inline std::vector<uint32_t> directionNumbers(int dims)
{
  if (dims > kMaxDim) {
    throw std::runtime_error("sobol: at most " + std::to_string(kMaxDim) + " dimensions");
  }
  std::vector<uint32_t> v(static_cast<size_t>(dims) * kBits);
  for (int k = 0; k < kBits; ++k) {
    v[k] = 1u << (kBits - 1 - k);
  }
  for (int d = 1; d < dims; ++d) {
    const uint32_t p = kPoly[d];
    const int s = 31 - __builtin_clz(p);  // degree of the primitive polynomial
    uint32_t m[kBits];
    for (int k = 0; k < s; ++k) {
      m[k] = kMinit[d][k];
    }
    for (int k = s; k < kBits; ++k) {
      uint32_t mk = m[k - s] ^ (m[k - s] << s);
      for (int j = 1; j < s; ++j) {
        if ((p >> (s - j)) & 1u) {
          mk ^= m[k - j] << j;
        }
      }
      m[k] = mk;
    }
    for (int k = 0; k < kBits; ++k) {
      v[static_cast<size_t>(d) * kBits + k] = m[k] << (kBits - 1 - k);
    }
  }
  return v;
}

// Integer points (rows x dims), unscrambled or scrambled with rng.
template<class Rng>
std::vector<uint32_t> points(int rows, int dims, bool scramble, Rng & rng)
{
  std::vector<uint32_t> v = directionNumbers(dims);
  std::vector<uint32_t> x(dims, 0u);
  if (scramble) {
    std::uniform_int_distribution<uint32_t> bit(0u, 1u), word(0u, (1u << kBits) - 1u);
    for (int d = 0; d < dims; ++d) {
      // Row r of L (MSB-first bit rows): ones at random columns c < r, one on the diagonal.
      uint32_t mask[kBits];
      for (int r = 0; r < kBits; ++r) {
        mask[r] = 1u << (kBits - 1 - r);
        for (int c = 0; c < r; ++c) {
          if (bit(rng)) {mask[r] |= 1u << (kBits - 1 - c);}
        }
      }
      for (int k = 0; k < kBits; ++k) {
        const uint32_t old = v[static_cast<size_t>(d) * kBits + k];
        uint32_t out = 0;
        for (int r = 0; r < kBits; ++r) {
          out |= static_cast<uint32_t>(__builtin_parity(old & mask[r])) << (kBits - 1 - r);
        }
        v[static_cast<size_t>(d) * kBits + k] = out;
      }
      x[d] = word(rng);  // digital shift = the first point
    }
  }
  std::vector<uint32_t> pts(static_cast<size_t>(rows) * dims);
  for (int i = 0; i < rows; ++i) {
    if (i > 0) {
      const int c = __builtin_ctz(~static_cast<uint32_t>(i - 1));  // lowest zero bit of i - 1
      for (int d = 0; d < dims; ++d) {
        x[d] ^= v[static_cast<size_t>(d) * kBits + c];
      }
    }
    for (int d = 0; d < dims; ++d) {
      pts[static_cast<size_t>(i) * dims + d] = x[d];
    }
  }
  return pts;
}

// Inverse of the standard normal CDF.
inline double invNormal(double p)
{
  const double q = std::min(p, 1.0 - p);
  const double t = std::sqrt(-2.0 * std::log(q));
  // Rough start (error ~1e-3), then Halley steps on Phi(x) - p.
  double x = t - (2.515517 + 0.802853 * t + 0.010328 * t * t) /
    (1.0 + 1.432788 * t + 0.189269 * t * t + 0.001308 * t * t * t);
  if (p < 0.5) {x = -x;}
  for (int it = 0; it < 4; ++it) {
    const double e = 0.5 * std::erfc(-x / std::sqrt(2.0)) - p;
    const double u = e * std::sqrt(2.0 * M_PI) * std::exp(0.5 * x * x);
    x -= u / (1.0 + 0.5 * x * u);
  }
  return x;
}

// rows x dims standard normal noise from (scrambled) Sobol points.
template<class Rng>
Eigen::ArrayXXf normalBank(int rows, int dims, bool scramble, Rng & rng)
{
  const std::vector<uint32_t> pts = points(rows, dims, scramble, rng);
  Eigen::ArrayXXf z(rows, dims);
  constexpr double kScale = 1.0 / static_cast<double>(1u << kBits);
  for (int i = 0; i < rows; ++i) {
    for (int d = 0; d < dims; ++d) {
      z(i, d) = static_cast<float>(invNormal(
          (pts[static_cast<size_t>(i) * dims + d] + 0.5) * kScale));
    }
  }
  return z;
}
}  // namespace mppi::sobol
