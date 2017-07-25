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

#ifndef CSPACE3_CACHE_H
#define CSPACE3_CACHE_H

namespace costmap_cspace
{
class CSpace3Cache
{
protected:
  std::unique_ptr<char[]> c;
  int size[3];
  int center[3];

public:
  void reset(const int &x, const int &y, const int &yaw)
  {
    size[0] = x * 2 + 1;
    size[1] = y * 2 + 1;
    size[2] = yaw;
    center[0] = x;
    center[1] = y;
    center[2] = 0;
    c.reset(new char[size[0] * size[1] * size[2]]);
  }

  char &e(const int &x, const int &y, const int &yaw)
  {
    return c[(yaw * size[1] + (y + center[1])) * size[0] + x + center[0]];
  }
  const char &e(const int &x, const int &y, const int &yaw) const
  {
    return c[(yaw * size[1] + (y + center[1])) * size[0] + x + center[0]];
  }
  void getSize(int &x, int &y, int &a) const
  {
    x = size[0];
    y = size[1];
    a = size[2];
  }
};
}  // namespace costmap_cspace

#endif  // CSPACE3_CACHE_H
