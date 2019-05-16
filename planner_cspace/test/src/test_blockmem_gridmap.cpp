/*
 * Copyright (c) 2017, the neonavigation authors
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

#include <cstddef>
#include <limits>

#include <gtest/gtest.h>

#include <planner_cspace/blockmem_gridmap.h>

TEST(BlockmemGridmap, ResetClear)
{
  BlockMemGridmap<float, 3, 3, 0x20> gm;

  for (int s = 4; s <= 6; s += 2)
  {
    gm.reset(CyclicVecInt<3, 3>(s, s, s));
    gm.clear(0.0);

    CyclicVecInt<3, 3> i;
    for (i[0] = 0; i[0] < s; ++i[0])
    {
      for (i[1] = 0; i[1] < s; ++i[1])
      {
        for (i[2] = 0; i[2] < s; ++i[2])
        {
          ASSERT_EQ(gm[i], 0.0);
        }
      }
    }

    gm.clear(3.0);
    for (i[0] = 0; i[0] < s; ++i[0])
    {
      for (i[1] = 0; i[1] < s; ++i[1])
      {
        for (i[2] = 0; i[2] < s; ++i[2])
        {
          ASSERT_EQ(gm[i], 3.0);
        }
      }
    }
  }
}

TEST(BlockmemGridmap, WriteRead)
{
  BlockMemGridmap<float, 3, 3, 0x20> gm;

  const int s = 4;
  gm.reset(CyclicVecInt<3, 3>(s, s, s));
  gm.clear(0.0);

  CyclicVecInt<3, 3> i;
  for (i[0] = 0; i[0] < s; ++i[0])
  {
    for (i[1] = 0; i[1] < s; ++i[1])
    {
      for (i[2] = 0; i[2] < s; ++i[2])
      {
        gm[i] = i[2] * 100 + i[1] * 10 + i[0];
      }
    }
  }

  for (i[0] = 0; i[0] < s; ++i[0])
  {
    for (i[1] = 0; i[1] < s; ++i[1])
    {
      for (i[2] = 0; i[2] < s; ++i[2])
      {
        ASSERT_EQ(gm[i], i[2] * 100 + i[1] * 10 + i[0]);
      }
    }
  }
}

TEST(BlockmemGridmap, OuterBoundery)
{
  BlockMemGridmap<float, 3, 2, 0x20> gm;

  const int s = 0x30;
  gm.reset(CyclicVecInt<3, 2>(s, s, s));
  gm.clear(1.0);

  CyclicVecInt<3, 2> i;
  const int outer = 0x10;
  for (i[0] = -outer; i[0] < s + outer; ++i[0])
  {
    for (i[1] = -outer; i[1] < s + outer; ++i[1])
    {
      for (i[2] = -outer; i[2] < s + outer; ++i[2])
      {
        if (i[0] >= 0 && i[1] >= 0 && i[2] >= 0 &&
            i[0] < s && i[1] < s && i[2] < s)
        {
          ASSERT_TRUE(gm.validate(i));
        }
        else
        {
          ASSERT_FALSE(gm.validate(i));
        }
        // Confirm at least not dead
        gm[i] = 1.0;
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
