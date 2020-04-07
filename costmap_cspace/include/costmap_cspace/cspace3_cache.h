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

#ifndef COSTMAP_CSPACE_CSPACE3_CACHE_H
#define COSTMAP_CSPACE_CSPACE3_CACHE_H

#include <memory>

#include <ros/ros.h>

namespace costmap_cspace
{
class CSpace3Cache
{
protected:
  std::unique_ptr<char[]> c_;
  int size_[3];
  int center_[3];
  int stride_[3];
  size_t array_size_;

public:
  CSpace3Cache()
    : c_(nullptr)
    , array_size_(0)
  {
    size_[0] = size_[1] = size_[2] = 0;
    center_[0] = center_[1] = center_[2] = 0;
    stride_[0] = stride_[1] = stride_[2] = 0;
  }
  void reset(const int& x, const int& y, const int& yaw)
  {
    size_[0] = x * 2 + 1;
    size_[1] = y * 2 + 1;
    size_[2] = yaw;
    center_[0] = x;
    center_[1] = y;
    center_[2] = 0;
    array_size_ = size_[0] * size_[1] * size_[2];
    c_.reset(new char[array_size_]);
    memset(c_.get(), 0, array_size_ * sizeof(char));
    stride_[0] = 1;
    stride_[1] = size_[0];
    stride_[2] = size_[0] * size_[1];
  }

  char& e(const int& x, const int& y, const int& yaw)
  {
    const size_t addr = yaw * stride_[2] + (y + center_[1]) * stride_[1] + (x + center_[0]);
    ROS_ASSERT(addr < array_size_);

    return c_[addr];
  }
  const char& e(const int& x, const int& y, const int& yaw) const
  {
    const size_t addr = yaw * stride_[2] + (y + center_[1]) * stride_[1] + (x + center_[0]);
    ROS_ASSERT(addr < array_size_);

    return c_[addr];
  }
  void getSize(int& x, int& y, int& a) const
  {
    x = size_[0];
    y = size_[1];
    a = size_[2];
  }
  void getCenter(int& x, int& y, int& a) const
  {
    x = center_[0];
    y = center_[1];
    a = center_[2];
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_CSPACE3_CACHE_H
