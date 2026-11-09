// Standalone check of sobol_bank.hpp against SciPy: writes unscrambled integer points, a scrambled
// normal bank and the inverse normal CDF on a grid. Usage: sobol_check OUT_DIR
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include "nav2_mppi_controller/tools/sobol_bank.hpp"

int main(int argc, char ** argv)
{
  const std::string out = argc > 1 ? argv[1] : ".";
  std::default_random_engine rng(1);
  const auto pts = mppi::sobol::points(4096, 112, false, rng);
  std::ofstream(out + "/plain_u32.bin", std::ios::binary).write(
    reinterpret_cast<const char *>(pts.data()), pts.size() * sizeof(uint32_t));
  for (int s = 1; s <= 5; ++s) {
    std::default_random_engine g(s);
    const Eigen::ArrayXXf z = mppi::sobol::normalBank(4096, 112, true, g);
    const Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> zr = z;
    std::ofstream(out + "/cpp_sobol_" + std::to_string(s) + ".bin", std::ios::binary).write(
      reinterpret_cast<const char *>(zr.data()), zr.size() * sizeof(float));
  }
  std::vector<double> q;
  for (int i = 1; i < 100000; ++i) {q.push_back(mppi::sobol::invNormal(i / 100000.0));}
  q.push_back(mppi::sobol::invNormal(0.5 / (1 << 30)));
  std::ofstream(out + "/invnorm.bin", std::ios::binary).write(
    reinterpret_cast<const char *>(q.data()), q.size() * sizeof(double));
  std::printf("ok\n");
}
