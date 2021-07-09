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
#include <memory>

#include <gtest/gtest.h>

#include <planner_cspace/blockmem_gridmap.h>

namespace planner_cspace
{
TEST(BlockmemGridmap, SpacialAccessPerformance)
{
  constexpr int size[3] = {0x420, 0x420, 0x28};
  constexpr int pad[3] = {0x204, 0x204, 0x10};
  constexpr int range = 0x10;
  constexpr int repeat = 4;

  planner_cspace::BlockMemGridmap<float, 3, 2, 0x20> gm;
  planner_cspace::BlockMemGridmap<float, 3, 2, 0x20> gm_ret;

  using Vec = CyclicVecInt<3, 2>;
  using ThreeDimArrayFloat = std::array<std::array<std::array<float, size[0]>, size[1]>, size[2]>;

  // Allocate raw grid in heap memory to avoid stack overflow
  std::shared_ptr<ThreeDimArrayFloat> array_ptr(new ThreeDimArrayFloat);
  std::shared_ptr<ThreeDimArrayFloat> array_ret_ptr(new ThreeDimArrayFloat);
  ThreeDimArrayFloat& array = *array_ptr;
  ThreeDimArrayFloat& array_ret = *array_ret_ptr;

  gm.reset(Vec(size[0], size[1], size[2]));
  gm_ret.reset(Vec(size[0], size[1], size[2]));

  Vec i;
  // Generate dataset.
  for (i[0] = 0; i[0] < size[0]; ++i[0])
  {
    for (i[1] = 0; i[1] < size[1]; ++i[1])
    {
      for (i[2] = 0; i[2] < size[2]; ++i[2])
      {
        gm[i] = i[2] * 0x100 + i[1] * 0x10 + i[0];
        array[i[2]][i[1]][i[0]] = gm[i];
      }
    }
  }
  // Check dataset.
  for (i[0] = pad[0]; i[0] < size[0] - pad[0]; ++i[0])
  {
    for (i[1] = pad[1]; i[1] < size[1] - pad[1]; ++i[1])
    {
      for (i[2] = pad[2]; i[2] < size[2] - pad[2]; ++i[2])
      {
        ASSERT_EQ(gm[i], array[i[2]][i[1]][i[0]]);
      }
    }
  }
  boost::chrono::duration<float> d0;
  boost::chrono::duration<float> d1;

  for (int r = 0; r < repeat; ++r)
  {
    // Performance test for BlockMemGridmap.
    const auto ts0 = boost::chrono::high_resolution_clock::now();
    for (i[0] = pad[0]; i[0] < size[0] - pad[0]; ++i[0])
    {
      for (i[1] = pad[1]; i[1] < size[1] - pad[1]; ++i[1])
      {
        for (i[2] = pad[2]; i[2] < size[2] - pad[2]; ++i[2])
        {
          Vec j;
          gm_ret[i] = 0;

          for (j[0] = -range; j[0] <= range; ++j[0])
          {
            for (j[1] = -range; j[1] <= range; ++j[1])
            {
              for (j[2] = -range; j[2] <= range; ++j[2])
              {
                const Vec ij = i + j;
                gm_ret[i] += gm[ij];
                gm[ij]++;
              }
            }
          }
        }
      }
      std::cerr << ".";
    }
    std::cerr << std::endl;
    const auto te0 = boost::chrono::high_resolution_clock::now();
    d0 += boost::chrono::duration<float>(te0 - ts0);

    // Performance test for 3D Array.
    const auto ts1 = boost::chrono::high_resolution_clock::now();
    for (i[0] = pad[0]; i[0] < size[0] - pad[0]; ++i[0])
    {
      for (i[1] = pad[1]; i[1] < size[1] - pad[1]; ++i[1])
      {
        for (i[2] = pad[2]; i[2] < size[2] - pad[2]; ++i[2])
        {
          Vec j;
          array_ret[i[2]][i[1]][i[0]] = 0;

          for (j[0] = -range; j[0] <= range; ++j[0])
          {
            for (j[1] = -range; j[1] <= range; ++j[1])
            {
              for (j[2] = -range; j[2] <= range; ++j[2])
              {
                const Vec ij = i + j;
                array_ret[i[2]][i[1]][i[0]] += array[ij[2]][ij[1]][ij[0]];
                array[ij[2]][ij[1]][ij[0]]++;
              }
            }
          }
        }
      }
      std::cerr << ".";
    }
    std::cerr << std::endl;
    const auto te1 = boost::chrono::high_resolution_clock::now();
    d1 += boost::chrono::duration<float>(te1 - ts1);
  }
  std::cout << "BlockMemGridmap<3, 2>: " << d0.count() << std::endl;
  std::cout << "Array[][][]: " << d1.count() << std::endl;

  // Check result.
  for (i[0] = pad[0]; i[0] < size[0] - pad[0]; ++i[0])
  {
    for (i[1] = pad[1]; i[1] < size[1] - pad[1]; ++i[1])
    {
      for (i[2] = pad[2]; i[2] < size[2] - pad[2]; ++i[2])
      {
        ASSERT_EQ(gm_ret[i], array_ret[i[2]][i[1]][i[0]]);
      }
    }
  }

  // Compare performance.
  std::cout << "Improvement ratio: " << d1.count() / d0.count() << std::endl;
  ASSERT_LT(d0, d1);
}
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
