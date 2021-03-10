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

namespace planner_cspace
{
template <int BLOCK_WIDTH>
class BlockMemGridmapHelper : public BlockMemGridmap<int, 1, 1, BLOCK_WIDTH, false>
{
public:
  size_t getBlockBit() const
  {
    return this->block_bit_;
  }
};
TEST(BlockmemGridmap, BlockWidth)
{
  ASSERT_EQ(4u, BlockMemGridmapHelper<0x10>().getBlockBit());
  ASSERT_EQ(8u, BlockMemGridmapHelper<0x100>().getBlockBit());
  ASSERT_EQ(12u, BlockMemGridmapHelper<0x1000>().getBlockBit());
  ASSERT_EQ(16u, BlockMemGridmapHelper<0x10000>().getBlockBit());
}

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

TEST(BlockmemGridmap, ClearAndCopyPartially)
{
  const CyclicVecInt<3, 3> base_size(17, 8, 3);
  BlockMemGridmap<float, 3, 3, 0x20> gm_base;
  gm_base.reset(base_size);
  for (CyclicVecInt<3, 3> p(0, 0, 0); p[0] < base_size[0]; ++p[0])
  {
    for (p[1] = 0; p[1] < base_size[1]; ++p[1])
    {
      for (p[2] = 0; p[2] < base_size[2]; ++p[2])
      {
        gm_base[p] = p[0] * base_size[1] * base_size[2] + p[1] * base_size[2] + p[2];
      }
    }
  }

  BlockMemGridmap<float, 3, 3, 0x20> gm;
  gm = gm_base;

  const CyclicVecInt<3, 3> copy_min_pos(3, 5, 0);
  const CyclicVecInt<3, 3> copy_max_pos(6, 7, 2);
  gm.clear_partially(-1, copy_min_pos, copy_max_pos);

  for (CyclicVecInt<3, 3> p(0, 0, 0); p[0] < base_size[0]; ++p[0])
  {
    for (p[1] = 0; p[1] < base_size[1]; ++p[1])
    {
      for (p[2] = 0; p[2] < base_size[2]; ++p[2])
      {
        if ((copy_min_pos[0] <= p[0]) && (p[0] < copy_max_pos[0]) &&
            (copy_min_pos[1] <= p[1]) && (p[1] < copy_max_pos[1]) &&
            (copy_min_pos[2] <= p[2]) && (p[2] < copy_max_pos[2]))
        {
          EXPECT_EQ(gm[p], -1) << p[0] << "," << p[1] << "," << p[2];
        }
        else
        {
          EXPECT_EQ(gm[p], gm_base[p]) << p[0] << "," << p[1] << "," << p[2];
        }
      }
    }
  }

  BlockMemGridmap<float, 3, 3, 0x20> gm_update;
  gm_update.reset(base_size);
  for (CyclicVecInt<3, 3> p(0, 0, 0); p[0] < base_size[0]; ++p[0])
  {
    for (p[1] = 0; p[1] < base_size[1]; ++p[1])
    {
      for (p[2] = 0; p[2] < base_size[2]; ++p[2])
      {
        gm_update[p] = p[0] * base_size[1] * base_size[2] + p[1] * base_size[2] + p[2] * -1;
      }
    }
  }

  gm = gm_base;
  gm.copy_partially(gm_update, copy_min_pos, copy_max_pos);

  for (CyclicVecInt<3, 3> p(0, 0, 0); p[0] < base_size[0]; ++p[0])
  {
    for (p[1] = 0; p[1] < base_size[1]; ++p[1])
    {
      for (p[2] = 0; p[2] < base_size[2]; ++p[2])
      {
        if ((copy_min_pos[0] <= p[0]) && (p[0] < copy_max_pos[0]) &&
            (copy_min_pos[1] <= p[1]) && (p[1] < copy_max_pos[1]) &&
            (copy_min_pos[2] <= p[2]) && (p[2] < copy_max_pos[2]))
        {
          ASSERT_EQ(gm[p], gm_update[p]);
        }
        else
        {
          ASSERT_EQ(gm[p], gm_base[p]);
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

TEST(BlockmemGridmap, OuterBoundary)
{
  BlockMemGridmap<float, 3, 2, 0x20, true> gm;

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
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
