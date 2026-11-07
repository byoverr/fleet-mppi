// Copyright (c) 2026 Sergey Shchetinkin
// Licensed under the Apache License, Version 2.0
//
// sin and cos of a packet in one call, bit-identical to Eigen 3.4's psin_float / pcos_float.
// Eigen computes both polynomials inside psincos_float<ComputeSine> and then picks one; the
// argument reduction does not depend on ComputeSine except for the "huge argument" threshold.
// Here the reduction and both polynomials are computed once. Packets with an argument at or
// above the smaller threshold fall back to Eigen's own psin and pcos, so the result is the
// same for every input. The arithmetic below is copied from
// Eigen/src/Core/arch/Default/GenericPacketMathFunctions.h (3.4.0) and must stay in sync.

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__SINCOS_PACKET_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__SINCOS_PACKET_HPP_

#include <Eigen/Core>

#if EIGEN_WORLD_VERSION != 3 || EIGEN_MAJOR_VERSION != 4
#error "sincos_packet.hpp is a copy of Eigen 3.4 psincos_float; check it against this Eigen"
#endif

namespace mppi::utils
{

template<typename Packet>
#if EIGEN_GNUC_AT_LEAST(4, 4) && EIGEN_COMP_GNUC_STRICT
__attribute__((optimize("-fno-unsafe-math-optimizations")))
#endif
inline void psincosBoth(const Packet & _x, Packet & s, Packet & c)
{
  using namespace Eigen::internal;  // NOLINT
  typedef typename unpacket_traits<Packet>::integer_packet PacketI;

  const Packet cst_2oPI = pset1<Packet>(0.636619746685028076171875f);
  const Packet cst_rounding_magic = pset1<Packet>(12582912);
  const PacketI csti_1 = pset1<PacketI>(1);
  const Packet cst_sign_mask = pset1frombits<Packet>(0x80000000u);

#if defined(EIGEN_HAS_SINGLE_INSTRUCTION_MADD)
  const float huge_th = 71476.0625f;  // min of the sin (117435.992) and cos thresholds
#else
  const float huge_th = 18838.f;  // min of 25966 (sin) and 18838 (cos)
#endif
  if (predux_any(pcmp_le(pset1<Packet>(huge_th), pabs(_x)))) {
    s = psin(_x);
    c = pcos(_x);
    return;
  }

  Packet x = pabs(_x);
  Packet y = pmul(x, cst_2oPI);
  Packet y_round = padd(y, cst_rounding_magic);
  EIGEN_OPTIMIZATION_BARRIER(y_round)
  PacketI y_int = preinterpret<PacketI>(y_round);
  y = psub(y_round, cst_rounding_magic);

#if defined(EIGEN_HAS_SINGLE_INSTRUCTION_MADD)
  x = pmadd(y, pset1<Packet>(-1.57079601287841796875f), x);
  x = pmadd(y, pset1<Packet>(-3.1391647326017846353352069854736328125e-07f), x);
  x = pmadd(y, pset1<Packet>(-5.390302529957764765544681040410068817436695098876953125e-15f), x);
#else
  x = pmadd(y, pset1<Packet>(-1.5703125), x);
  EIGEN_OPTIMIZATION_BARRIER(x)
  x = pmadd(y, pset1<Packet>(-0.000483989715576171875), x);
  EIGEN_OPTIMIZATION_BARRIER(x)
  x = pmadd(y, pset1<Packet>(1.62865035235881805419921875e-07), x);
  x = pmadd(y, pset1<Packet>(5.5644315544167710640977020375430583953857421875e-11), x);
#endif

  Packet sign_s = pxor(_x, preinterpret<Packet>(plogical_shift_left<30>(y_int)));
  Packet sign_c = preinterpret<Packet>(plogical_shift_left<30>(padd(y_int, csti_1)));
  sign_s = pand(sign_s, cst_sign_mask);
  sign_c = pand(sign_c, cst_sign_mask);

  Packet poly_mask = preinterpret<Packet>(pcmp_eq(pand(y_int, csti_1), pzero(y_int)));

  Packet x2 = pmul(x, x);

  Packet y1 = pset1<Packet>(2.4372266125283204019069671630859375e-05f);
  y1 = pmadd(y1, x2, pset1<Packet>(-0.00138865201734006404876708984375f));
  y1 = pmadd(y1, x2, pset1<Packet>(0.041666619479656219482421875f));
  y1 = pmadd(y1, x2, pset1<Packet>(-0.5f));
  y1 = pmadd(y1, x2, pset1<Packet>(1.f));

  Packet y2 = pset1<Packet>(-0.0001959234114083702898469196984621021329076029360294342041015625f);
  y2 = pmadd(y2, x2,
      pset1<Packet>(0.0083326873655616851693794799871284340042620897293090820312500000f));
  y2 = pmadd(y2, x2,
      pset1<Packet>(-0.1666666203982298255503735617821803316473960876464843750000000000f));
  y2 = pmul(y2, x2);
  y2 = pmadd(y2, x, x);

  s = pxor(pselect(poly_mask, y2, y1), sign_s);
  c = pxor(pselect(poly_mask, y1, y2), sign_c);
}

/// sin and cos of n floats. Every element goes through the packet path (the tail through a
/// padded packet), as in Eigen's .sin() / .cos() over an array whose size is a multiple of
/// the packet size.
inline void sincosArray(const float * in, float * s_out, float * c_out, Eigen::Index n)
{
  using namespace Eigen::internal;  // NOLINT
  typedef typename packet_traits<float>::type Packet;
  constexpr int PS = unpacket_traits<Packet>::size;
  Eigen::Index i = 0;
  for (; i + PS <= n; i += PS) {
    Packet s, c;
    psincosBoth(ploadu<Packet>(in + i), s, c);
    pstoreu(s_out + i, s);
    pstoreu(c_out + i, c);
  }
  if (i < n) {
    EIGEN_ALIGN_TO_BOUNDARY(sizeof(Packet)) float buf[PS] = {};
    EIGEN_ALIGN_TO_BOUNDARY(sizeof(Packet)) float sb[PS];
    EIGEN_ALIGN_TO_BOUNDARY(sizeof(Packet)) float cb[PS];
    for (Eigen::Index k = i; k < n; ++k) {
      buf[k - i] = in[k];
    }
    Packet s, c;
    psincosBoth(pload<Packet>(buf), s, c);
    pstore(sb, s);
    pstore(cb, c);
    for (Eigen::Index k = i; k < n; ++k) {
      s_out[k] = sb[k - i]; c_out[k] = cb[k - i];
    }
  }
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__SINCOS_PACKET_HPP_
