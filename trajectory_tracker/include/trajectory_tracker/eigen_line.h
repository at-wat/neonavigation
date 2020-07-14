/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * Copyright (c) 2014-2018, the neonavigation authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TRAJECTORY_TRACKER_EIGEN_LINE_H
#define TRAJECTORY_TRACKER_EIGEN_LINE_H

#include <cmath>

#include <Eigen/Core>

namespace trajectory_tracker
{
inline float curv3p(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
  float ret;
  ret = 2 * (a[0] * b[1] + b[0] * c[1] + c[0] * a[1] -
             a[0] * c[1] - b[0] * a[1] - c[0] * b[1]);
  ret /= std::sqrt((b - a).squaredNorm() * (b - c).squaredNorm() * (c - a).squaredNorm());

  return ret;
}

inline float cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
  return a[0] * b[1] - a[1] * b[0];
}

inline float lineDistance(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
  return cross2((b - a), (c - a)) / (b - a).norm();
}

// lineStripDistanceSigned returns signed distance from the line strip.
// If c is behind a, negative value will be returned. [ (c)  (a)---(b) ]
// Otherwise, positive value will be returned. [ (a)---(b)  (c) ] [ (a)--(c)--(b) ]
inline float lineStripDistanceSigned(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
  if ((b - a).dot(c - a) <= 0)
    return -(c - a).norm();
  if ((a - b).dot(c - b) <= 0)
    return (c - b).norm();
  return std::abs(lineDistance(a, b, c));
}

inline float lineStripDistance(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
  return std::abs(lineStripDistanceSigned(a, b, c));
}

inline Eigen::Vector2d projection2d(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
  const float r = (b - a).dot(c - a) / (b - a).squaredNorm();
  return b * r + a * (1.0 - r);
}
}  // namespace trajectory_tracker

#endif  // TRAJECTORY_TRACKER_EIGEN_LINE_H
