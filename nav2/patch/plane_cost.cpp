// Cost of FootprintBitmask::buildPlane and of one mayCollide call, for the overhead question.
#include <chrono>
#include <cstdio>
#include <random>
#include "nav2_mppi_controller/tools/footprint_bitmask.hpp"
int main()
{
  for (int cells : {60, 300, 600}) {
    nav2_costmap_2d::Costmap2D cm(cells, cells, 0.1, 0.0, 0.0, 0);
    std::mt19937 rng(1);
    for (int i = 0; i < cells * cells / 50; ++i) {cm.setCost(rng() % cells, rng() % cells, 254);}
    std::vector<geometry_msgs::msg::Point> fp(4);
    fp[0].x = 2.33; fp[0].y = 0.75; fp[1].x = -1.53; fp[1].y = 0.75;
    fp[2].x = -1.53; fp[2].y = -0.75; fp[3].x = 2.33; fp[3].y = -0.75;
    mppi::FootprintBitmask m;
    auto a = std::chrono::steady_clock::now();
    m.configure(fp, 0.1, 128, 2);
    auto b = std::chrono::steady_clock::now();
    const int reps = 2000;
    for (int r = 0; r < reps; ++r) {m.buildPlane(cm, false);}
    auto c = std::chrono::steady_clock::now();
    long hits = 0;
    std::uniform_real_distribution<double> u(0.0, cells * 0.1), th(-3.2, 3.2);
    const int q = 200000;
    for (int i = 0; i < q; ++i) {hits += m.mayCollide(u(rng), u(rng), th(rng));}
    auto d = std::chrono::steady_clock::now();
    auto us = [](auto x, auto y) {return std::chrono::duration<double, std::micro>(y - x).count();};
    std::printf("map %dx%d: build masks %.1f ms, buildPlane %.1f us, mayCollide %.1f ns (hit %.0f%%)\n",
      cells, cells, us(a, b) / 1000, us(b, c) / reps, 1000 * us(c, d) / q, 100.0 * hits / q);
  }
}
