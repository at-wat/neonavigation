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

#ifndef TRAJECTORY_TRACKER_BASIC_CONTROL_H
#define TRAJECTORY_TRACKER_BASIC_CONTROL_H

#include <cmath>

namespace trajectory_tracker
{
inline float timeOptimalControl(const float angle, const float acc)
{
  return -std::copysign(1.0, angle) * std::sqrt(std::abs(2 * angle * acc));
}

inline float clip(const float v, const float max)
{
  if (v > max)
    return max;
  else if (v < -max)
    return -max;

  return v;
}

inline float angleNormalized(float ang)
{
  while (ang < -M_PI)
    ang += 2.0 * M_PI;
  while (ang > M_PI)
    ang -= 2.0 * M_PI;
  return ang;
}

class VelAccLimitter
{
private:
  float val_prev_;

public:
  inline VelAccLimitter()
    : val_prev_(0)
  {
  }
  inline float increment(
      const float v, const float vel, const float acc, const float dt)
  {
    return set(val_prev_ + v, vel, acc, dt);
  }
  inline float set(
      float v, const float vel, const float acc, const float dt)
  {
    v = clip(v, vel);

    if (v > val_prev_ + dt * acc)
      v = val_prev_ + dt * acc;
    else if (v < val_prev_ - dt * acc)
      v = val_prev_ - dt * acc;

    if (!std::isfinite(v))
      v = 0;

    val_prev_ = v;
    return v;
  }
  inline float get() const
  {
    return val_prev_;
  }
  inline void clear()
  {
    val_prev_ = 0;
  }
};
}  // namespace trajectory_tracker

#endif  // TRAJECTORY_TRACKER_BASIC_CONTROL_H
