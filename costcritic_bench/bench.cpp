// CostCritic collision-check microbenchmark: Nav2-like exact footprint checks vs
// per-heading-bin conservative bitmask filter (+ exact fallback).
// Build: clang++ -std=c++17 -O3 -mcpu=native -o bench bench.cpp
// Usage: ./bench scenes/<scene>.bin [reps]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

using u64 = uint64_t;
struct P2 { double x, y; };
static const int MAXV = 32;

// ---------------------------------------------------------------- scene
struct Scene {
  int nx, ny, C, K, T, nv;
  double res, ox, oy, circ, insc;
  std::vector<uint8_t> cost, ccost;  // cost: 0/254/255; ccost: Nav2-like inflated center cost
  std::vector<float> dist, poses;
  std::vector<P2> foot;
};

static Scene load(const char* path) {
  Scene s;
  FILE* f = fopen(path, "rb");
  if (!f) { perror(path); exit(1); }
  auto rd = [&](void* p, size_t n) { if (fread(p, 1, n, f) != n) { fprintf(stderr, "short read\n"); exit(1); } };
  rd(&s.nx, 4); rd(&s.ny, 4); rd(&s.res, 8); rd(&s.ox, 8); rd(&s.oy, 8);
  rd(&s.C, 4); rd(&s.K, 4); rd(&s.T, 4); rd(&s.nv, 4);
  size_t N = (size_t)s.nx * s.ny, NP = (size_t)s.C * s.K * s.T * 3;
  s.cost.resize(N); s.dist.resize(N); s.poses.resize(NP); s.foot.resize(s.nv);
  rd(s.cost.data(), N); rd(s.dist.data(), N * 4); rd(s.poses.data(), NP * 4); rd(s.foot.data(), s.nv * 16);
  fclose(f);
  s.circ = 0; s.insc = 1e9;
  for (int i = 0; i < s.nv; i++) {
    P2 a = s.foot[i], b = s.foot[(i + 1) % s.nv];
    s.circ = std::max(s.circ, std::hypot(a.x, a.y));
    double ex = b.x - a.x, ey = b.y - a.y, t = std::clamp(-(a.x * ex + a.y * ey) / (ex * ex + ey * ey), 0.0, 1.0);
    s.insc = std::min(s.insc, std::hypot(a.x + t * ex, a.y + t * ey));
  }
  // Nav2 gate: center cost >= circumscribed cost  <=>  dist(cell, lethal) <= circumscribed radius.
  s.ccost.resize(N);
  for (size_t i = 0; i < N; i++)
    s.ccost[i] = s.cost[i] >= 254 ? s.cost[i] : s.dist[i] <= s.insc ? 253 : s.dist[i] <= s.circ ? 128 : 0;
  return s;
}

// ---------------------------------------------------------------- shared rasterization
// Exactly nav2_util::LineIterator (Bresenham variant), calls f(x,y) per cell, stops if f returns true.
template <class F> static inline bool line(int x0, int y0, int x1, int y1, F&& f) {
  int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  int xi1 = x1 >= x0 ? 1 : -1, xi2 = xi1, yi1 = y1 >= y0 ? 1 : -1, yi2 = yi1, den, num, add, npix;
  if (dx >= dy) { xi1 = 0; yi2 = 0; den = dx; num = dx / 2; add = dy; npix = dx; }
  else          { xi2 = 0; yi1 = 0; den = dy; num = dy / 2; add = dx; npix = dy; }
  int x = x0, y = y0;
  for (int i = 0; i <= npix; i++) {
    if (f(x, y)) return true;
    num += add;
    if (num >= den) { num -= den; x += xi1; y += yi1; }
    x += xi2; y += yi2;
  }
  return false;
}

// Cells read by the exact check. Perimeter: same edge order/direction as
// FootprintCollisionChecker::footprintCost (0->1,...,n-2->n-1, then 0->n-1).
// FULL additionally: scanline fill of cells whose centre lies inside the (continuous) polygon.
template <bool FULL, class F>
static inline bool raster(int n, const int* cx, const int* cy, const double* fx, const double* fy, F&& f) {
  for (int i = 0; i + 1 < n; i++)
    if (line(cx[i], cy[i], cx[i + 1], cy[i + 1], f)) return true;
  if (line(cx[0], cy[0], cx[n - 1], cy[n - 1], f)) return true;
  if (FULL) {
    double ymin = fy[0], ymax = fy[0];
    for (int i = 1; i < n; i++) { ymin = std::min(ymin, fy[i]); ymax = std::max(ymax, fy[i]); }
    double xs[MAXV];
    for (int j = (int)std::floor(ymin); j <= (int)std::floor(ymax); j++) {
      double yc = j + 0.5; int m = 0;
      for (int a = 0, b = n - 1; a < n; b = a++)
        if ((fy[a] <= yc) != (fy[b] <= yc)) xs[m++] = fx[a] + (yc - fy[a]) * (fx[b] - fx[a]) / (fy[b] - fy[a]);
      std::sort(xs, xs + m);
      for (int k = 0; k + 1 < m; k += 2)
        for (int i = (int)std::ceil(xs[k] - 0.5); i <= (int)std::floor(xs[k + 1] - 0.5); i++)
          if (f(i, j)) return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------- exact reference checks
struct Map { int nx, ny; double res, ox, oy; const uint8_t* cost; const P2* foot; int nv; };

// Lean reference: stack arrays, inlined; collision iff a read cell is LETHAL(254) or UNKNOWN(255)
// (= Nav2 with track_unknown false), or a vertex is off the map.
template <bool FULL> static inline bool ref_check(const Map& m, double x, double y, double th) {
  double c = std::cos(th), s = std::sin(th), fx[MAXV], fy[MAXV];
  int cx[MAXV], cy[MAXV];
  for (int i = 0; i < m.nv; i++) {
    double wx = x + m.foot[i].x * c - m.foot[i].y * s, wy = y + m.foot[i].x * s + m.foot[i].y * c;
    if (wx < m.ox || wy < m.oy) return true;  // Costmap2D::worldToMap
    fx[i] = (wx - m.ox) / m.res; fy[i] = (wy - m.oy) / m.res;
    cx[i] = (int)fx[i]; cy[i] = (int)fy[i];
    if (cx[i] >= m.nx || cy[i] >= m.ny) return true;
  }
  const uint8_t* cost = m.cost; int nx = m.nx;
  return raster<FULL>(m.nv, cx, cy, fx, fy, [&](int i, int j) { return cost[j * nx + i] >= 254; });
}

// Closer to Nav2's code shape: out-of-line call, heap-allocated oriented footprint each call.
__attribute__((noinline)) static bool ref_perim_nav2like(const Map& m, double x, double y, double th) {
  double c = std::cos(th), s = std::sin(th);
  std::vector<P2> o; o.reserve(m.nv);
  for (int i = 0; i < m.nv; i++)
    o.push_back({x + m.foot[i].x * c - m.foot[i].y * s, y + m.foot[i].x * s + m.foot[i].y * c});
  int cx[MAXV], cy[MAXV];
  for (int i = 0; i < m.nv; i++) {
    if (o[i].x < m.ox || o[i].y < m.oy) return true;
    cx[i] = (int)((o[i].x - m.ox) / m.res); cy[i] = (int)((o[i].y - m.oy) / m.res);
    if (cx[i] >= m.nx || cy[i] >= m.ny) return true;
  }
  const uint8_t* cost = m.cost; int nx = m.nx;
  return raster<false>(m.nv, cx, cy, nullptr, nullptr, [&](int i, int j) { return cost[j * nx + i] >= 254; });
}

// Faithful to the Nav2 call chain as I read it: CostCritic::inCollision ->
// collision_checker_.footprintCostAtPose(x, y, th, costmap_ros_->getRobotFootprint())
//  - getRobotFootprint() returns std::vector<geometry_msgs::msg::Point> BY VALUE (heap copy per call)
//  - footprintCostAtPose builds a new oriented std::vector<Point> (reserve + push_back)
//  - footprintCost/lineCost call Costmap2D::worldToMap / getCost, defined out-of-line in costmap_2d.cpp
//  - lineCost returns early on LETHAL only; max-cost otherwise (255 -> collision at the end).
struct Pt3 { double x, y, z; };
struct Costmap { int nx, ny; double res, ox, oy; const uint8_t* cost; };
static std::vector<Pt3> g_padded_footprint;
__attribute__((noinline)) static std::vector<Pt3> getRobotFootprint() { return g_padded_footprint; }
__attribute__((noinline)) static bool cm_worldToMap(const Costmap* c, double wx, double wy, unsigned& mx, unsigned& my) {
  if (wx < c->ox || wy < c->oy) return false;
  mx = (unsigned)((wx - c->ox) / c->res); my = (unsigned)((wy - c->oy) / c->res);
  return mx < (unsigned)c->nx && my < (unsigned)c->ny;
}
__attribute__((noinline)) static uint8_t cm_getCost(const Costmap* c, unsigned mx, unsigned my) { return c->cost[my * c->nx + mx]; }
static double lineCost(const Costmap* cm, int x0, int x1, int y0, int y1) {
  double lc = 0;
  line(x0, y0, x1, y1, [&](int x, int y) {
    double pc = cm_getCost(cm, x, y);
    if (pc == 254.0) { lc = pc; return true; }
    if (lc < pc) lc = pc;
    return false;
  });
  return lc;
}
__attribute__((noinline)) static double footprintCostAtPose(const Costmap* cm, double x, double y, double th, const std::vector<Pt3>& fp) {
  double c = std::cos(th), s = std::sin(th);
  std::vector<Pt3> o; o.reserve(fp.size());
  for (auto& p : fp) o.push_back({x + (p.x * c - p.y * s), y + (p.x * s + p.y * c), 0.0});
  unsigned x0, y0, x1 = 0, y1 = 0;
  if (!cm_worldToMap(cm, o[0].x, o[0].y, x0, y0)) return 254;
  unsigned xs = x0, ys = y0; double cost = 0;
  for (size_t i = 0; i + 1 < o.size(); i++) {
    if (!cm_worldToMap(cm, o[i + 1].x, o[i + 1].y, x1, y1)) return 254;
    cost = std::max(lineCost(cm, x0, x1, y0, y1), cost);
    x0 = x1; y0 = y1;
    if (cost == 254) return cost;
  }
  return std::max(lineCost(cm, xs, x1, ys, y1), cost);
}
static inline bool ref_perim_nav2faithful(const Costmap* cm, double x, double y, double th) {
  return footprintCostAtPose(cm, x, y, th, getRobotFootprint()) >= 254;
}

// Faithful to the Nav2 call chain as I read it: CostCritic::inCollision ->
// collision_checker_.footprintCostAtPose(x, y, th, costmap_ros_->getRobotFootprint())
//  - getRobotFootprint() returns std::vector<geometry_msgs::msg::Point> BY VALUE (heap copy per call)
//  - footprintCostAtPose builds a new oriented std::vector<Point> (reserve + push_back)
//  - footprintCost/lineCost call Costmap2D::worldToMap / getCost, defined out-of-line in costmap_2d.cpp
//  - lineCost returns early on LETHAL only; max-cost otherwise (255 -> collision at the end).
struct Pt3 { double x, y, z; };
struct Costmap { int nx, ny; double res, ox, oy; const uint8_t* cost; };
static std::vector<Pt3> g_padded_footprint;
__attribute__((noinline)) static std::vector<Pt3> getRobotFootprint() { return g_padded_footprint; }
__attribute__((noinline)) static bool cm_worldToMap(const Costmap* c, double wx, double wy, unsigned& mx, unsigned& my) {
  if (wx < c->ox || wy < c->oy) return false;
  mx = (unsigned)((wx - c->ox) / c->res); my = (unsigned)((wy - c->oy) / c->res);
  return mx < (unsigned)c->nx && my < (unsigned)c->ny;
}
__attribute__((noinline)) static uint8_t cm_getCost(const Costmap* c, unsigned mx, unsigned my) { return c->cost[my * c->nx + mx]; }
static double lineCost(const Costmap* cm, int x0, int x1, int y0, int y1) {
  double lc = 0;
  line(x0, y0, x1, y1, [&](int x, int y) {
    double pc = cm_getCost(cm, x, y);
    if (pc == 254.0) { lc = pc; return true; }
    if (lc < pc) lc = pc;
    return false;
  });
  return lc;
}
__attribute__((noinline)) static double footprintCostAtPose(const Costmap* cm, double x, double y, double th, const std::vector<Pt3>& fp) {
  double c = std::cos(th), s = std::sin(th);
  std::vector<Pt3> o; o.reserve(fp.size());
  for (auto& p : fp) o.push_back({x + (p.x * c - p.y * s), y + (p.x * s + p.y * c), 0.0});
  unsigned x0, y0, x1 = 0, y1 = 0;
  if (!cm_worldToMap(cm, o[0].x, o[0].y, x0, y0)) return 254;
  unsigned xs = x0, ys = y0; double cost = 0;
  for (size_t i = 0; i + 1 < o.size(); i++) {
    if (!cm_worldToMap(cm, o[i + 1].x, o[i + 1].y, x1, y1)) return 254;
    cost = std::max(lineCost(cm, x0, x1, y0, y1), cost);
    x0 = x1; y0 = y1;
    if (cost == 254) return cost;
  }
  return std::max(lineCost(cm, xs, x1, ys, y1), cost);
}
static inline bool ref_perim_nav2faithful(const Costmap* cm, double x, double y, double th) {
  return footprintCostAtPose(cm, x, y, th, getRobotFootprint()) >= 254;
}

// ---------------------------------------------------------------- bitmask filter
static const int PX = 64;  // horizontal padding = one word (keeps packing word-aligned)

struct Plane {  // lethal|unknown cells, 1 bit/cell, row-major, padded with 1s (off-map = blocked)
  int W, rows, PY; std::vector<u64> w;
  void init(int nx, int ny, int py) { PY = py; W = (nx + 2 * PX + 63) / 64 + 1; rows = ny + 2 * py; w.assign((size_t)W * rows, ~0ull); }
};

static void build_plane_scalar(Plane& p, const uint8_t* cost, int nx, int ny) {
  for (int j = 0; j < ny; j++) {
    u64* row = &p.w[(size_t)(j + p.PY) * p.W + 1];
    const uint8_t* c = cost + (size_t)j * nx;
    int i = 0;
    for (; i + 64 <= nx; i += 64) {
      u64 b = 0;
      for (int k = 0; k < 64; k++) b |= (u64)(c[i + k] >= 254) << k;
      row[i >> 6] = b;
    }
    if (i < nx) { u64 b = ~0ull; for (int k = 0; k < nx - i; k++) if (c[i + k] < 254) b &= ~(1ull << k); row[i >> 6] = b; }
  }
}

#if defined(__ARM_NEON)
static inline u64 movemask64(const uint8_t* p, uint8x16_t thr, uint8x16_t bw) {
  uint8x16_t m0 = vandq_u8(vcgeq_u8(vld1q_u8(p), thr), bw), m1 = vandq_u8(vcgeq_u8(vld1q_u8(p + 16), thr), bw);
  uint8x16_t m2 = vandq_u8(vcgeq_u8(vld1q_u8(p + 32), thr), bw), m3 = vandq_u8(vcgeq_u8(vld1q_u8(p + 48), thr), bw);
  uint8x16_t s = vpaddq_u8(vpaddq_u8(m0, m1), vpaddq_u8(m2, m3));
  s = vpaddq_u8(s, s);
  return vgetq_lane_u64(vreinterpretq_u64_u8(s), 0);
}
static void build_plane_neon(Plane& p, const uint8_t* cost, int nx, int ny) {
  const uint8x16_t thr = vdupq_n_u8(254);
  const uint8_t bwv[16] = {1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128};
  const uint8x16_t bw = vld1q_u8(bwv);
  for (int j = 0; j < ny; j++) {
    u64* row = &p.w[(size_t)(j + p.PY) * p.W + 1];
    const uint8_t* c = cost + (size_t)j * nx;
    int i = 0;
    for (; i + 64 <= nx; i += 64) row[i >> 6] = movemask64(c + i, thr, bw);
    if (i < nx) { u64 b = ~0ull; for (int k = 0; k < nx - i; k++) if (c[i + k] < 254) b &= ~(1ull << k); row[i >> 6] = b; }
  }
}
#endif

struct Masks {  // per heading bin: dense rows dy = dymin..dymax, each (x0, 64-bit pattern)
  int nb, R; double inv_binw;
  std::vector<int> start, dymin;
  std::vector<int32_t> dyoff, x0;  // dyoff = dy*W (filled when bound to a plane)
  std::vector<int> dy;
  std::vector<u64> bits;
  size_t bytes() const { return bits.size() * 16 + start.size() * 8; }  // 8B bits + 4B dyoff + 4B x0 per row
};

// Conservative mask: union of exact cell sets over dense samples of heading in the bin (with eps margin)
// and sub-cell offset in [0,1)^2, then dilated by `dil` cells. Validated empirically (containment test).
template <bool FULL> static Masks build_masks(const std::vector<P2>& foot, double res, int nb, int dil) {
  Masks M; M.nb = nb; double binw = 2 * M_PI / nb; M.inv_binw = 1 / binw;
  double rmax = 0; for (auto& p : foot) rmax = std::max(rmax, std::hypot(p.x, p.y) / res);
  int Rc = (int)std::ceil(rmax) + dil + 2, S = 2 * Rc + 1; M.R = Rc;
  int nth = (int)std::ceil(binw * rmax / 0.1) + 2;  // <= 0.1 cell arc between heading samples
  const int NO = 11;                                 // sub-cell offsets step 0.1
  int n = foot.size();
  M.start.push_back(0);
  for (int b = 0; b < nb; b++) {
    std::vector<uint8_t> occ(S * S, 0), dl(S * S, 0);
    for (int t = 0; t < nth; t++) {
      double th = b * binw - 1e-6 + t * (binw + 2e-6) / (nth - 1), c = std::cos(th), s = std::sin(th);
      for (int ou = 0; ou < NO; ou++) for (int ov = 0; ov < NO; ov++) {
        double u = std::min(ou / 10.0, 0.999999), v = std::min(ov / 10.0, 0.999999), fx[MAXV], fy[MAXV];
        int cx[MAXV], cy[MAXV];
        for (int i = 0; i < n; i++) {
          fx[i] = u + (foot[i].x * c - foot[i].y * s) / res; fy[i] = v + (foot[i].x * s + foot[i].y * c) / res;
          cx[i] = (int)std::floor(fx[i]); cy[i] = (int)std::floor(fy[i]);
        }
        raster<FULL>(n, cx, cy, fx, fy, [&](int i, int j) { occ[(j + Rc) * S + i + Rc] = 1; return false; });
      }
    }
    for (int j = 0; j < S; j++) for (int i = 0; i < S; i++) if (occ[j * S + i])
      for (int a = -dil; a <= dil; a++) for (int c2 = -dil; c2 <= dil; c2++) dl[(j + a) * S + i + c2] = 1;
    int jmin = S, jmax = -1;
    for (int j = 0; j < S; j++) for (int i = 0; i < S; i++) if (dl[j * S + i]) { jmin = std::min(jmin, j); jmax = std::max(jmax, j); }
    M.dymin.push_back(jmin - Rc);
    for (int j = jmin; j <= jmax; j++) {
      int i0 = S, i1 = -1;
      for (int i = 0; i < S; i++) if (dl[j * S + i]) { i0 = std::min(i0, i); i1 = std::max(i1, i); }
      if (i1 < i0) i0 = i1 = Rc;
      if (i1 - i0 >= 64) { fprintf(stderr, "mask row wider than 64 cells\n"); exit(1); }
      u64 bits = 0; for (int i = i0; i <= i1; i++) if (dl[j * S + i]) bits |= 1ull << (i - i0);
      M.dy.push_back(j - Rc); M.x0.push_back(i0 - Rc); M.bits.push_back(bits);
    }
    M.start.push_back(M.bits.size());
  }
  return M;
}
static void bind(Masks& M, const Plane& P) { M.dyoff.resize(M.dy.size()); for (size_t r = 0; r < M.dy.size(); r++) M.dyoff[r] = M.dy[r] * P.W; }

static inline int bin_of(const Masks& M, double th) { int b = (int)std::floor(th * M.inv_binw) % M.nb; return b < 0 ? b + M.nb : b; }

struct Ctx { Map m; const Plane* P; };
static long g_fallback = 0;

// Returns true (=collision) exactly as ref_check<FULL>; the mask only certifies "free".
template <bool FULL> static inline bool bm_check(const Ctx& k, const Masks& M, double x, double y, double th) {
  const Map& m = k.m; const Plane& P = *k.P;
  double fx = (x - m.ox) / m.res, fy = (y - m.oy) / m.res;
  if (!(fx >= M.R - PX && fx < m.nx + PX - M.R - 1 && fy >= M.R - P.PY && fy < m.ny + P.PY - M.R - 1)) {
    g_fallback++; return ref_check<FULL>(m, x, y, th);  // far outside the padded plane
  }
  int ax = (int)std::floor(fx), ay = (int)std::floor(fy), b = bin_of(M, th);
  const u64* base = P.w.data() + (size_t)(ay + P.PY) * P.W;
  int col0 = ax + PX;
  u64 acc = 0;
  for (int r = M.start[b], e = M.start[b + 1]; r < e; r++) {
    int col = col0 + M.x0[r], sh = col & 63;
    const u64* w = base + M.dyoff[r] + (col >> 6);
    acc |= ((w[0] >> sh) | ((w[1] << 1) << (63 - sh))) & M.bits[r];
  }
  if (!acc) return false;
  g_fallback++;
  return ref_check<FULL>(m, x, y, th);
}

// Containment self-test: every cell the exact check would read must be set in the mask.
template <bool FULL> static long violations(const Masks& M, const Map& m, double x, double y, double th) {
  double fxp = (x - m.ox) / m.res, fyp = (y - m.oy) / m.res;
  int ax = (int)std::floor(fxp), ay = (int)std::floor(fyp), b = bin_of(M, th);
  double c = std::cos(th), s = std::sin(th), fx[MAXV], fy[MAXV]; int cx[MAXV], cy[MAXV];
  for (int i = 0; i < m.nv; i++) {
    double wx = x + m.foot[i].x * c - m.foot[i].y * s, wy = y + m.foot[i].x * s + m.foot[i].y * c;
    fx[i] = (wx - m.ox) / m.res; fy[i] = (wy - m.oy) / m.res;
    cx[i] = (int)std::floor(fx[i]); cy[i] = (int)std::floor(fy[i]);
  }
  long v = 0; int nrow = M.start[b + 1] - M.start[b];
  raster<FULL>(m.nv, cx, cy, fx, fy, [&](int i, int j) {
    int r = j - ay - M.dymin[b], dx;
    if (r < 0 || r >= nrow) { v++; return false; }
    dx = i - ax - M.x0[M.start[b] + r];
    if (dx < 0 || dx >= 64 || !((M.bits[M.start[b] + r] >> dx) & 1)) v++;
    return false;
  });
  return v;
}

// ---------------------------------------------------------------- timing helpers
using clk = std::chrono::steady_clock;
template <class F> static void timeit(int reps, F&& f, double& tmin, double& tmed) {
  std::vector<double> t;
  f();  // warm-up
  for (int r = 0; r < reps; r++) { auto a = clk::now(); f(); t.push_back(std::chrono::duration<double, std::nano>(clk::now() - a).count()); }
  std::sort(t.begin(), t.end()); tmin = t[0]; tmed = t[t.size() / 2];
}
static volatile long g_sink;

// Nav2 CostCritic loop semantics: per trajectory, per point: center lookup (float worldToMap);
// cost 0 -> skip; cost >= circumscribed cost -> footprint check; collision -> stop this trajectory.
template <class Check> static long pipeline(const Scene& s, Check&& chk) {
  long ncol = 0; const float* p = s.poses.data();
  float ox = s.ox, oy = s.oy, res = s.res;
  for (int tr = 0; tr < s.C * s.K; tr++, p += 3 * s.T) {
    for (int t = 0; t < s.T; t++) {
      float x = p[3 * t], y = p[3 * t + 1], th = p[3 * t + 2], fx = (x - ox) / res, fy = (y - oy) / res;
      uint8_t c = (fx < 0 || fy < 0 || fx >= s.nx || fy >= s.ny) ? 255 : s.ccost[(int)fy * s.nx + (int)fx];
      if (c == 0 || c < 128) continue;
      if (chk(x, y, th)) { ncol++; break; }
    }
  }
  return ncol;
}

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s scene.bin [reps]\n", argv[0]); return 1; }
  int reps = argc > 2 ? atoi(argv[2]) : 9;
  Scene s = load(argv[1]);
  Map m{s.nx, s.ny, s.res, s.ox, s.oy, s.cost.data(), s.foot.data(), s.nv};
  size_t NP = (size_t)s.C * s.K * s.T;
  const char* name = strrchr(argv[1], '/') ? strrchr(argv[1], '/') + 1 : argv[1];
  printf("# scene %s map %dx%d res %.3f footprint %d verts circ %.3f m insc %.3f m poses %zu reps %d\n",
         name, s.nx, s.ny, s.res, s.nv, s.circ, s.insc, NP, reps);

  // ---- gating + exact decisions (all poses, no early break)
  std::vector<int> gidx;
  std::vector<uint8_t> dp(NP, 0), df(NP, 0);
  for (size_t i = 0; i < NP; i++) {
    const float* q = &s.poses[3 * i];
    float fx = (q[0] - (float)s.ox) / (float)s.res, fy = (q[1] - (float)s.oy) / (float)s.res;
    uint8_t c = (fx < 0 || fy < 0 || fx >= s.nx || fy >= s.ny) ? 255 : s.ccost[(int)fy * s.nx + (int)fx];
    if (c >= 128) { gidx.push_back(i); dp[i] = ref_check<false>(m, q[0], q[1], q[2]); df[i] = ref_check<true>(m, q[0], q[1], q[2]); }
  }
  size_t G = gidx.size(), cp = 0, cf = 0, hid = 0, trp = 0, trf = 0, trhid = 0;
  for (int i : gidx) { cp += dp[i]; cf += df[i]; hid += (!dp[i] && df[i]); }
  for (size_t tr = 0; tr < (size_t)s.C * s.K; tr++) {
    bool a = 0, b = 0; for (int t = 0; t < s.T; t++) { a |= dp[tr * s.T + t]; b |= df[tr * s.T + t]; }
    trp += a; trf += b; trhid += (!a && b);
  }
  printf("gated %.4f of poses (%zu); colliding among gated: perim %.4f full %.4f\n", (double)G / NP, G, (double)cp / G, (double)cf / G);
  printf("#5329 effect: poses perim-free but full-colliding %zu (%.4f of gated); trajectories %zu of %d (perim-colliding %zu, full-colliding %zu)\n",
         hid, (double)hid / G, trhid, s.C * s.K, trp, trf);

  // ---- reference timings: footprint check only, on the gated pose list
  std::vector<float> gp; for (int i : gidx) gp.insert(gp.end(), &s.poses[3 * i], &s.poses[3 * i] + 3);
  auto over_gated = [&](auto&& chk) { return [&, chk]() { long n = 0; for (size_t i = 0; i < G; i++) n += chk(gp[3 * i], gp[3 * i + 1], gp[3 * i + 2]); g_sink = n; }; };
  double mn, md;
  printf("\n%-24s %10s %10s\n", "footprint check (gated)", "ns_min", "ns_median");
  timeit(reps, over_gated([&](double x, double y, double t) { return ref_perim_nav2like(m, x, y, t); }), mn, md);
  printf("%-24s %10.1f %10.1f\n", "ref_perim_nav2like", mn / G, md / G);
  timeit(reps, over_gated([&](double x, double y, double t) { return ref_check<false>(m, x, y, t); }), mn, md);
  double ref_p = md / G; printf("%-24s %10.1f %10.1f\n", "ref_perim", mn / G, md / G);
  timeit(reps, over_gated([&](double x, double y, double t) { return ref_check<true>(m, x, y, t); }), mn, md);
  double ref_f = md / G; printf("%-24s %10.1f %10.1f\n", "ref_full", mn / G, md / G);

  // pipeline projections (Nav2 defaults K=2000, T=56, trajectory_point_step=2 -> 28 points/trajectory)
  auto proj = [&](double ns_total) { return ns_total / NP * 2000 * 28 / 1e6; };  // ms per cycle
  double pg, pp, pf, tmp;
  timeit(reps, [&] { g_sink = pipeline(s, [](float, float, float) { return false; }); }, tmp, pg);
  timeit(reps, [&] { g_sink = pipeline(s, [&](float x, float y, float t) { return ref_check<false>(m, x, y, t); }); }, tmp, pp);
  timeit(reps, [&] { g_sink = pipeline(s, [&](float x, float y, float t) { return ref_check<true>(m, x, y, t); }); }, tmp, pf);

  // ---- bitmask sweep
  printf("\n%5s %4s %9s %8s %8s %8s %8s %8s %9s %9s %8s %8s\n", "bins", "kind", "mask_KB", "viol", "mismatch", "fallbk", "falsealm",
         "ns_med", "x_vs_ref", "x_nav2lk", "cyc_ms", "ref_ms");
  double nav2lk; { double a; timeit(reps, over_gated([&](double x, double y, double t) { return ref_perim_nav2like(m, x, y, t); }), a, nav2lk); nav2lk /= G; }
  std::mt19937_64 rng(42);
  for (int nb : {32, 64, 128, 256}) {
    Masks Mp = build_masks<false>(s.foot, s.res, nb, 1), Mf = build_masks<true>(s.foot, s.res, nb, 1);
    Plane P; P.init(s.nx, s.ny, std::max(Mp.R, Mf.R) + 1);
#if defined(__ARM_NEON)
    build_plane_neon(P, s.cost.data(), s.nx, s.ny);
#else
    build_plane_scalar(P, s.cost.data(), s.nx, s.ny);
#endif
    bind(Mp, P); bind(Mf, P);
    Ctx k{m, &P};
    for (int full = 0; full < 2; full++) {
      const Masks& M = full ? Mf : Mp;
      // containment: all gated dataset poses + 200k random poses (any heading, incl. |theta| > 2pi)
      long viol = 0;
      std::uniform_real_distribution<double> ux(s.ox, s.ox + s.nx * s.res), uy(s.oy, s.oy + s.ny * s.res), ut(-4 * M_PI, 4 * M_PI);
      for (size_t i = 0; i < G; i++) viol += full ? violations<true>(M, m, gp[3 * i], gp[3 * i + 1], gp[3 * i + 2]) : violations<false>(M, m, gp[3 * i], gp[3 * i + 1], gp[3 * i + 2]);
      for (int i = 0; i < 200000; i++) { double x = ux(rng), y = uy(rng), t = ut(rng); viol += full ? violations<true>(M, m, x, y, t) : violations<false>(M, m, x, y, t); }
      // decisions + fallback stats
      long mism = 0, falarm = 0; g_fallback = 0;
      for (size_t i = 0; i < G; i++) {
        bool d = full ? bm_check<true>(k, M, gp[3 * i], gp[3 * i + 1], gp[3 * i + 2]) : bm_check<false>(k, M, gp[3 * i], gp[3 * i + 1], gp[3 * i + 2]);
        bool r = full ? df[gidx[i]] : dp[gidx[i]];
        mism += d != r;
      }
      long fb = g_fallback; falarm = fb - (long)(full ? cf : cp);
      double ns;
      if (full) timeit(reps, over_gated([&](double x, double y, double t) { return bm_check<true>(k, Mf, x, y, t); }), mn, ns);
      else      timeit(reps, over_gated([&](double x, double y, double t) { return bm_check<false>(k, Mp, x, y, t); }), mn, ns);
      ns /= G;
      double cyc;
      if (full) timeit(reps, [&] { g_sink = pipeline(s, [&](float x, float y, float t) { return bm_check<true>(k, Mf, x, y, t); }); }, tmp, cyc);
      else      timeit(reps, [&] { g_sink = pipeline(s, [&](float x, float y, float t) { return bm_check<false>(k, Mp, x, y, t); }); }, tmp, cyc);
      printf("%5d %4s %9.1f %8ld %8ld %8.4f %8.4f %8.1f %9.2f %9.2f %8.3f %8.3f\n", nb, full ? "full" : "per", M.bytes() / 1024.0, viol, mism,
             (double)fb / G, (double)falarm / G, ns, (full ? ref_f : ref_p) / ns, nav2lk / ns, proj(cyc), proj(full ? pf : pp));
    }
  }
  printf("pipeline projection (ms/cycle, K=2000 x 28 pts, median): gate-only %.3f  ref_perim %.3f  ref_full %.3f\n", proj(pg), proj(pp), proj(pf));

  // ---- bit-plane build cost for a 10x10 m local costmap at 0.05 m (200x200), padding 64/16
  {
    int n = 200; std::vector<uint8_t> c(n * n); std::mt19937 r2(1);
    for (auto& v : c) v = (r2() % 10 == 0) ? 254 : (r2() % 50 == 0 ? 255 : r2() % 253);
    Plane A, B; A.init(n, n, 16); B.init(n, n, 16);
    double a1, a2, b1, b2;
    timeit(201, [&] { build_plane_scalar(A, c.data(), n, n); g_sink = A.w[100]; }, a1, a2);
#if defined(__ARM_NEON)
    timeit(201, [&] { build_plane_neon(B, c.data(), n, n); g_sink = B.w[100]; }, b1, b2);
    printf("bit-plane build 200x200: scalar min %.2f us median %.2f us; NEON min %.2f us median %.2f us; identical=%d; plane %zu B\n",
           a1 / 1e3, a2 / 1e3, b1 / 1e3, b2 / 1e3, (int)(A.w == B.w), A.w.size() * 8);
#else
    printf("bit-plane build 200x200: scalar min %.2f us median %.2f us\n", a1 / 1e3, a2 / 1e3);
#endif
  }
  return 0;
}
