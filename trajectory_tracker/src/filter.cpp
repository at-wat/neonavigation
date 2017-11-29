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

#include <filter.h>

double Filter::in(const double i)
{
  x_ = k_[0] * i + k_[1] * x_;
  return k_[2] * i + k_[3] * x_;
}

Filter::Filter(const Type type, const double tc, const double out0)
{
  time_const_ = tc;
  switch (type)
  {
    case FILTER_LPF:
      k_[3] = -1 / (1.0 + 2 * time_const_);
      k_[2] = -k_[3];
      k_[1] = (1.0 - 2 * time_const_) * k_[3];
      k_[0] = -k_[1] - 1.0;
      x_ = (1 - k_[2]) * out0 / k_[3];
      break;
    case FILTER_HPF:
      k_[3] = -1 / (1.0 + 2 * time_const_);
      k_[2] = -k_[3] * 2 * time_const_;
      k_[1] = (1.0 - 2 * time_const_) * k_[3];
      k_[0] = 2 * time_const_ * (-k_[1] + 1.0);
      x_ = (1 - k_[2]) * out0 / k_[3];
      break;
  }
}
