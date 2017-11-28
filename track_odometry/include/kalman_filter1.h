/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#ifndef KALMAN_FILTER1_H
#define KALMAN_FILTER1_H

#include <limits>

class KalmanFilter1
{
public:
  float x_;
  float sigma_;

  void set(const float x0 = 0.0,
           const float sigma0 = std::numeric_limits<float>::infinity())
  {
    x_ = x0;
    sigma_ = sigma0;
  }
  KalmanFilter1(const float x0 = 0.0,
                const float sigma0 = std::numeric_limits<float>::infinity())
  {
    set(x0, sigma0);
  }
  void predict(const float x_plus, const float sigma_plus)
  {
    x_ += x_plus;
    sigma_ += sigma_plus;
  }
  void measure(const float x_in, const float sigma_in)
  {
    if (std::isinf(sigma_in))
      return;
    if (std::isinf(sigma_))
    {
      if (std::isinf(x_in))
        x_ = 0;
      else
        x_ = x_in;
      sigma_ = sigma_in;
      return;
    }
    float kt = sigma_ * sigma_ / (sigma_ * sigma_ + sigma_in * sigma_in);
    x_ = x_ + kt * (x_in - x_);
    sigma_ = (1.0 - kt) * sigma_;
  }
};

#endif  // KALMAN_FILTER1_H
