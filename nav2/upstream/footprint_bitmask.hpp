// Copyright (c) 2026 Sergey Shchetinkin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__FOOTPRINT_BITMASK_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__FOOTPRINT_BITMASK_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#if defined(__ARM_NEON) && !defined(MPPI_BITMASK_SCALAR)
#include <arm_neon.h>
#endif

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mppi
{

/**
 * @brief Exact-decision filter in front of FootprintCollisionChecker::footprintCostAtPose.
 *
 * The collision decision of CostCritic depends only on whether any cell read by the perimeter
 * check is LETHAL (or NO_INFORMATION when unknown space is not tracked), or a footprint vertex
 * is off the map. The costmap is packed into one bit plane of such cells (64 cells per word,
 * off-map padding set). For every heading bin and sub-cell offset bin a mask is precomputed that
 * contains every cell the perimeter check can read for any pose in that bin (vertex cells are
 * bounded exactly, every Bresenham line between them is enumerated). A pose whose mask ANDs to
 * zero is certainly collision-free; otherwise the exact check decides. Decisions are therefore
 * identical to the exact check.
 *
 * In front of the masks, the bounding box of each mask is tested in O(1): a row sparse table of
 * the bit plane (level l row j is the OR of rows [j, j + 2^l)) covers the box rows with two
 * overlapping ranges. An empty box certifies the pose free without reading the mask rows.
 */
class FootprintBitmask
{
public:
  bool ready() const {return ready_;}

  /** Rebuild the masks if the footprint, resolution or bin counts changed. */
  void configure(
    const std::vector<geometry_msgs::msg::Point> & footprint, double resolution,
    int heading_bins, int subcell_bins)
  {
    std::vector<double> key;
    for (const auto & p : footprint) {key.push_back(p.x); key.push_back(p.y);}
    key.push_back(resolution); key.push_back(heading_bins); key.push_back(subcell_bins);
    if (key == key_) {
      return;
    }
    key_ = key;
    ready_ = build(footprint, resolution, heading_bins, subcell_bins);
  }

  /** Pack the costmap into the bit plane; call once per control cycle. */
  void buildPlane(const nav2_costmap_2d::Costmap2D & costmap, bool tracking_unknown)
  {
    if (!ready_) {
      return;
    }
    nx_ = static_cast<int>(costmap.getSizeInCellsX());
    ny_ = static_cast<int>(costmap.getSizeInCellsY());
    ox_ = costmap.getOriginX();
    oy_ = costmap.getOriginY();
    res_ = costmap.getResolution();
    py_ = r_ + 2;
    pad_x_ = 64 * ((r_ + 2 + 63) / 64);  // whole words of off-map padding per side
    words_ = (nx_ + 2 * pad_x_ + 63) / 64 + 1;
    plane_.assign(static_cast<size_t>(words_) * (ny_ + 2 * py_), ~0ull);
    // LETHAL (254) always blocks, NO_INFORMATION (255) only when unknown space is not tracked
    const unsigned char * cost = costmap.getCharMap();
    for (int j = 0; j < ny_; ++j) {
      uint64_t * row = &plane_[static_cast<size_t>(j + py_) * words_ + pad_x_ / 64];
      const unsigned char * c = cost + static_cast<size_t>(j) * nx_;
      int i = 0;
      for (; i + 64 <= nx_; i += 64) {
        row[i >> 6] = pack64(c + i, tracking_unknown);
      }
      if (i < nx_) {
        uint64_t b = ~0ull;
        for (int k = 0; k < nx_ - i; ++k) {
          if (!blocked(c[i + k], tracking_unknown)) {b &= ~(1ull << k);}
        }
        row[i >> 6] = b;
      }
    }
    buildTable();
  }

  /** False means the pose is certainly free; true means the exact check must decide. */
  bool mayCollide(double x, double y, double theta) const
  {
    const double fx = (x - ox_) / res_, fy = (y - oy_) / res_;
    if (!(fx >= r_ - pad_x_ && fx < nx_ + pad_x_ - r_ - 1 && fy >= r_ - py_ &&
      fy < ny_ + py_ - r_ - 1))
    {
      return true;
    }
    const int ax = static_cast<int>(std::floor(fx)), ay = static_cast<int>(std::floor(fy));
    int b = static_cast<int>(std::floor(theta * inv_binw_)) % nb_;
    if (b < 0) {b += nb_;}
    const int su = std::min(s_ - 1, static_cast<int>((fx - ax) * s_));
    const int sv = std::min(s_ - 1, static_cast<int>((fy - ay) * s_));
    const int k = (b * s_ + su) * s_ + sv;
    const int col0 = ax + pad_x_, row0 = ay + py_;
    // box check: two overlapping row ranges of the sparse table
    const uint64_t * lv = levels_[tl_[k]].data();
    const uint64_t * ra = lv + static_cast<size_t>(row0 + by0_[k]) * words_;
    const uint64_t * rb = lv + static_cast<size_t>(row0 + ty1_[k]) * words_;
    uint64_t box = 0;
    for (int r = tstart_[k], e = tstart_[k + 1]; r < e; ++r) {
      const int col = col0 + tx0_[r], sh = col & 63, wi = col >> 6;
      const uint64_t va = (ra[wi] >> sh) | ((ra[wi + 1] << 1) << (63 - sh));
      const uint64_t vb = (rb[wi] >> sh) | ((rb[wi + 1] << 1) << (63 - sh));
      box |= (va | vb) & tbits_[r];
    }
    if (box == 0) {
      return false;
    }
    const uint64_t * base = plane_.data() + static_cast<size_t>(row0) * words_;
    uint64_t acc = 0;
    for (int r = start_[k], e = start_[k + 1]; r < e; ++r) {
      const int col = col0 + x0_[r], sh = col & 63;
      const uint64_t * w = base + dy_[r] * words_ + (col >> 6);
      acc |= ((w[0] >> sh) | ((w[1] << 1) << (63 - sh))) & bits_[r];
    }
    return acc != 0;
  }

private:
  void buildTable()
  {
    const size_t n = static_cast<size_t>(ny_ + 2 * py_) * words_;
    levels_.resize(tmax_ + 1);
    levels_[0] = plane_;
    for (int l = 1; l <= tmax_; ++l) {
      const size_t off = static_cast<size_t>(1 << (l - 1)) * words_;
      const std::vector<uint64_t> & a = levels_[l - 1];
      std::vector<uint64_t> & o = levels_[l];
      o.resize(n);
      const size_t m = n > off ? n - off : 0;
      for (size_t q = 0; q < m; ++q) {o[q] = a[q] | a[q + off];}
      for (size_t q = m; q < n; ++q) {o[q] = ~0ull;}
    }
  }

  static constexpr double kEps = 1e-6;  // cells; absorbs floating point differences

  static bool blocked(unsigned char c, bool tracking_unknown)
  {
    return c == nav2_costmap_2d::LETHAL_OBSTACLE ||
           (c == nav2_costmap_2d::NO_INFORMATION && !tracking_unknown);
  }

  static uint64_t pack64(const unsigned char * c, bool tracking_unknown)
  {
#if defined(__ARM_NEON) && !defined(MPPI_BITMASK_SCALAR)
    const uint8x16_t lethal = vdupq_n_u8(nav2_costmap_2d::LETHAL_OBSTACLE);
    const uint8x16_t unknown = vdupq_n_u8(nav2_costmap_2d::NO_INFORMATION);
    const uint8_t bw_v[16] = {1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128};
    const uint8x16_t bw = vld1q_u8(bw_v);
    uint8x16_t m[4];
    for (int q = 0; q < 4; ++q) {
      const uint8x16_t v = vld1q_u8(c + 16 * q);
      uint8x16_t hit = vceqq_u8(v, lethal);
      if (!tracking_unknown) {hit = vorrq_u8(hit, vceqq_u8(v, unknown));}
      m[q] = vandq_u8(hit, bw);
    }
    uint8x16_t s = vpaddq_u8(vpaddq_u8(m[0], m[1]), vpaddq_u8(m[2], m[3]));
    s = vpaddq_u8(s, s);
    return vgetq_lane_u64(vreinterpretq_u64_u8(s), 0);
#else
    uint64_t b = 0;
    for (int k = 0; k < 64; ++k) {
      b |= static_cast<uint64_t>(blocked(c[k], tracking_unknown)) << k;
    }
    return b;
#endif
  }

  // Same cell sequence as nav2_util::LineIterator (used by FootprintCollisionChecker::lineCost)
  template<class F>
  static void line(int x0, int y0, int x1, int y1, F && f)
  {
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int xi1 = x1 >= x0 ? 1 : -1, xi2 = xi1, yi1 = y1 >= y0 ? 1 : -1, yi2 = yi1;
    int den, num, add, npix;
    if (dx >= dy) {
      xi1 = 0; yi2 = 0; den = dx; num = dx / 2; add = dy; npix = dx;
    } else {
      xi2 = 0; yi1 = 0; den = dy; num = dy / 2; add = dx; npix = dy;
    }
    int x = x0, y = y0;
    for (int i = 0; i <= npix; ++i) {
      f(x, y);
      num += add;
      if (num >= den) {num -= den; x += xi1; y += yi1;}
      x += xi2; y += yi2;
    }
  }

  // Exact range of a point rotated by every angle in [t0, t1] (endpoints and axis crossings)
  static void rotRange(
    double px, double py, double t0, double t1,
    double & xlo, double & xhi, double & ylo, double & yhi)
  {
    const double r = std::hypot(px, py), ph = std::atan2(py, px);
    xlo = ylo = 1e300; xhi = yhi = -1e300;
    auto upd = [&](double a) {
        const double X = r * std::cos(a), Y = r * std::sin(a);
        xlo = std::min(xlo, X); xhi = std::max(xhi, X);
        ylo = std::min(ylo, Y); yhi = std::max(yhi, Y);
      };
    upd(t0 + ph); upd(t1 + ph);
    for (int k = static_cast<int>(std::ceil((t0 + ph) / (M_PI / 2)));
      k <= static_cast<int>(std::floor((t1 + ph) / (M_PI / 2))); ++k)
    {
      upd(k * M_PI / 2);
    }
  }

  bool build(
    const std::vector<geometry_msgs::msg::Point> & footprint, double res, int nb, int s)
  {
    const int n = static_cast<int>(footprint.size());
    if (n < 3 || nb < 1 || s < 1) {
      return false;
    }
    std::vector<double> fx(n), fy(n);
    double rmax = 0.0;
    for (int i = 0; i < n; ++i) {
      fx[i] = footprint[i].x / res; fy[i] = footprint[i].y / res;
      rmax = std::max(rmax, std::hypot(fx[i], fy[i]));
    }
    nb_ = nb; s_ = s; inv_binw_ = nb / (2.0 * M_PI);
    r_ = static_cast<int>(std::ceil(rmax)) + 4;
    const int w = 2 * r_ + 1;
    const double binw = 2.0 * M_PI / nb;
    start_.assign(1, 0); dy_.clear(); x0_.clear(); bits_.clear();
    by0_.clear(); tl_.clear(); ty1_.clear(); tx0_.clear(); tbits_.clear(); tstart_.assign(1, 0);
    tmax_ = 0;
    std::vector<uint8_t> occ(static_cast<size_t>(w) * w);
    std::vector<int> lx(n), hx(n), ly(n), hy(n);
    for (int b = 0; b < nb; ++b) {
      for (int su = 0; su < s; ++su) {
        for (int sv = 0; sv < s; ++sv) {
          const double t0 = b * binw - 1e-6, t1 = (b + 1) * binw + 1e-6;
          const double u0 = static_cast<double>(su) / s - kEps, u1 = static_cast<double>(su + 1) / s + kEps;
          const double v0 = static_cast<double>(sv) / s - kEps, v1 = static_cast<double>(sv + 1) / s + kEps;
          std::fill(occ.begin(), occ.end(), 0);
          bool outside = false;
          auto mark = [&](int i, int j) {
              if (std::abs(i) >= r_ || std::abs(j) >= r_) {outside = true; return;}
              occ[static_cast<size_t>(j + r_) * w + i + r_] = 1;
            };
          for (int i = 0; i < n; ++i) {
            double xl, xh, yl, yh;
            rotRange(fx[i], fy[i], t0, t1, xl, xh, yl, yh);
            lx[i] = static_cast<int>(std::floor(u0 + xl - kEps));
            hx[i] = static_cast<int>(std::floor(u1 + xh + kEps));
            ly[i] = static_cast<int>(std::floor(v0 + yl - kEps));
            hy[i] = static_cast<int>(std::floor(v1 + yh + kEps));
          }
          // edge order and direction as in FootprintCollisionChecker::footprintCost
          auto edge = [&](int a, int c) {
              for (int ax = lx[a]; ax <= hx[a]; ++ax) {
                for (int ay = ly[a]; ay <= hy[a]; ++ay) {
                  for (int cx = lx[c]; cx <= hx[c]; ++cx) {
                    for (int cy = ly[c]; cy <= hy[c]; ++cy) {
                      line(ax, ay, cx, cy, mark);
                    }
                  }
                }
              }
            };
          for (int i = 0; i + 1 < n; ++i) {edge(i, i + 1);}
          edge(0, n - 1);
          if (outside || !emit(occ, w)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  // Store one mask as dense rows (row offset, first column, 64-bit pattern)
  bool emit(const std::vector<uint8_t> & occ, int w)
  {
    int jmin = w, jmax = -1, imin = w, imax = -1;
    for (int j = 0; j < w; ++j) {
      for (int i = 0; i < w; ++i) {
        if (occ[static_cast<size_t>(j) * w + i]) {jmin = std::min(jmin, j); jmax = std::max(jmax, j);}
      }
    }
    for (int j = jmin; j <= jmax; ++j) {
      int i0 = w, i1 = -1;
      for (int i = 0; i < w; ++i) {
        if (occ[static_cast<size_t>(j) * w + i]) {i0 = std::min(i0, i); i1 = std::max(i1, i);}
      }
      // rows wider than 64 cells are split into several 64-cell segments
      for (int c0 = i0; c0 <= i1; c0 += 64) {
        uint64_t bits = 0;
        for (int i = c0; i <= std::min(i1, c0 + 63); ++i) {
          if (occ[static_cast<size_t>(j) * w + i]) {bits |= 1ull << (i - c0);}
        }
        if (bits) {
          dy_.push_back(j - r_); x0_.push_back(c0 - r_); bits_.push_back(bits);
        }
      }
      if (i1 >= 0) {imin = std::min(imin, i0); imax = std::max(imax, i1);}
    }
    start_.push_back(static_cast<int>(bits_.size()));
    if (jmax < 0) {
      return false;
    }
    // bounding box rows [jmin, jmax] as two ranges of length 2^l, columns as 64-cell segments
    int l = 0;
    while ((2 << l) <= jmax - jmin + 1) {++l;}
    tl_.push_back(l);
    tmax_ = std::max(tmax_, l);
    by0_.push_back(jmin - r_);
    ty1_.push_back(jmax - (1 << l) + 1 - r_);
    for (int c0 = imin; c0 <= imax; c0 += 64) {
      const int len = std::min(64, imax - c0 + 1);
      tx0_.push_back(c0 - r_);
      tbits_.push_back(len == 64 ? ~0ull : ((1ull << len) - 1));
    }
    tstart_.push_back(static_cast<int>(tbits_.size()));
    return true;
  }

  std::vector<double> key_;
  bool ready_{false};
  int nb_{1}, s_{1}, r_{0};
  double inv_binw_{0.0};
  std::vector<int> start_, dy_, x0_;
  std::vector<uint64_t> bits_;
  // bounding box of each mask for the sparse table check
  std::vector<int> by0_, tl_, ty1_, tx0_, tstart_;
  std::vector<uint64_t> tbits_;
  int tmax_{0};
  std::vector<std::vector<uint64_t>> levels_;
  // bit plane of the current costmap
  int nx_{0}, ny_{0}, py_{0}, pad_x_{64}, words_{0};
  double ox_{0.0}, oy_{0.0}, res_{1.0};
  std::vector<uint64_t> plane_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__FOOTPRINT_BITMASK_HPP_
