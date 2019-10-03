/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <map>

#include <gtest/gtest.h>

#include <planner_cspace/planner_3d/costmap_bbf.h>
#include <planner_cspace/bbf.h>

namespace planner_cspace
{
namespace planner_3d
{
TEST(CostmapBBF, ForEach)
{
  CostmapBBF cm;
  const int w = 10, h = 5;
  cm.reset(CostmapBBF::Vec(w, h, 0));
  bool called[w][h];
  for (int i = 0; i < w; i++)
    for (int j = 0; j < h; j++)
      called[i][j] = false;

  const auto cb = [&called](const CostmapBBF::Vec& p, bbf::BinaryBayesFilter& /* bbf */)
  {
    ASSERT_LT(p[0], w);
    ASSERT_LT(p[1], h);
    ASSERT_GE(p[0], 0);
    ASSERT_GE(p[1], 0);
    ASSERT_EQ(0, p[2]);
    called[p[0]][p[1]] = true;
  };
  cm.forEach(cb);

  int cnt_called = 0;
  for (int i = 0; i < w; i++)
    for (int j = 0; j < h; j++)
      if (called[i][j])
        cnt_called++;

  ASSERT_EQ(w * h, cnt_called);
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
