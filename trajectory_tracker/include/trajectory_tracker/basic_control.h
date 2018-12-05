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
inline float timeoptimalControl(const float angle, const float acc, const float dt)
{
  return -std::copysign(1.0, angle) * std::sqrt(std::abs(2 * angle * acc));
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
      const float val, const float vel, const float acc, const float dt)
  {
    return set(val_prev_ + val, vel, acc, dt);
  }
  inline float set(
      float val, const float vel, const float acc, const float dt)
  {
    if (val > vel)
      val = vel;
    else if (val < -vel)
      val = -vel;

    if (val > val_prev_ + dt * acc)
      val = val_prev_ + dt * acc;
    else if (val < val_prev_ - dt * acc)
      val = val_prev_ - dt * acc;

    if (!std::isfinite(val))
      val = 0;

    val_prev_ = val;
    return val;
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
