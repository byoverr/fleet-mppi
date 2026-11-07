// Copyright (c) 2026 Sergey Shchetinkin
// Licensed under the Apache License, Version 2.0
// mppi::utils::sincosArray against Eigen's .sin() / .cos(): bit-identical over a sweep of
// arguments, huge ones, special values and odd lengths (padded tail).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <vector>
#include <Eigen/Core>
#include "nav2_mppi_controller/tools/sincos_packet.hpp"

static size_t compare(const Eigen::ArrayXf & x)
{
  Eigen::ArrayXf s = x.sin(), c = x.cos();
  std::vector<float> s2(x.size()), c2(x.size());
  mppi::utils::sincosArray(x.data(), s2.data(), c2.data(), x.size());
  size_t d = 0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    d += std::memcmp(&s[i], &s2[i], 4) != 0 && !(std::isnan(s[i]) && std::isnan(s2[i]));
    d += std::memcmp(&c[i], &c2[i], 4) != 0 && !(std::isnan(c[i]) && std::isnan(c2[i]));
  }
  return d;
}

int main()
{
  std::mt19937 g(1);
  size_t total = 0, n = 0;
  for (float range : {4.0f, 100.0f, 2.0e4f, 1.0e5f, 1.0e7f}) {
    std::uniform_real_distribution<float> u(-range, range);
    Eigen::ArrayXf x(4'000'000);
    for (auto & v : x) {v = u(g);}
    const size_t d = compare(x);
    std::printf("uniform +-%g: %zu mismatches of %ld x 2\n", range, d, (long)x.size());
    total += d; n += x.size();
  }
  const float inf = std::numeric_limits<float>::infinity();
  Eigen::ArrayXf sp(12);
  sp << 0.f, -0.f, inf, -inf, std::nanf(""), 1e-40f, -1e-40f, 18838.f, 71476.0625f, 117435.992f,
    3.14159265f, -1.57079633f;
  size_t d = compare(sp);
  std::printf("special values: %zu mismatches of 12 x 2\n", d);
  total += d;
  for (int len : {1, 3, 5, 7, 2001, 257}) {  // tails through the padded packet
    Eigen::ArrayXf x = Eigen::ArrayXf::Random(len) * 3.0f;
    d = compare(x);
    total += d;
    std::printf("length %d: %zu mismatches\n", len, d);
  }
  std::printf("TOTAL mismatches=%zu\n", total);
  return total == 0 ? 0 : 1;
}
