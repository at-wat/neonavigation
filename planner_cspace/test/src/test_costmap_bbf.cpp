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
#include <cmath>

#include <gtest/gtest.h>

#include <planner_cspace/planner_3d/costmap_bbf.h>
#include <planner_cspace/bbf.h>

namespace planner_cspace
{
namespace planner_3d
{
TEST(CostmapBBF, ForEach)
{
  CostmapBBF bbf;
  const int w = 10, h = 5;
  bbf.reset(CostmapBBF::Vec(w, h, 1));
  bool called[w][h];
  for (int i = 0; i < w; i++)
    for (int j = 0; j < h; j++)
      called[i][j] = false;

  const auto cb = [&called, w, h](const CostmapBBF::Vec& p, bbf::BinaryBayesFilter& /* bbf */)
  {
    ASSERT_LT(p[0], w);
    ASSERT_LT(p[1], h);
    ASSERT_GE(p[0], 0);
    ASSERT_GE(p[1], 0);
    ASSERT_EQ(0, p[2]);
    called[p[0]][p[1]] = true;
  };
  bbf.forEach(cb);

  int cnt_called = 0;
  for (int i = 0; i < w; i++)
    for (int j = 0; j < h; j++)
      if (called[i][j])
        cnt_called++;

  ASSERT_EQ(w * h, cnt_called);
}
TEST(CostmapBBF, Update)
{
  const float odds_hit = bbf::probabilityToOdds(0.8);
  const float odds_miss = bbf::probabilityToOdds(0.3);
  const char costmap0[3][3] =
      {
          {10, 100, 100},
          {100, 100, 100},
          {100, 100, 100},
      };
  const char costmap1[3][3] =
      {
          {0, -1, 0},
          {100, 0, 0},
          {0, 0, 100},
      };
  const float expected_odds[3][3] =
      {
          {bbf::MIN_ODDS, bbf::MIN_ODDS * odds_hit, bbf::MIN_ODDS * odds_hit * odds_miss},
          {bbf::MIN_ODDS * odds_hit * odds_hit, bbf::MIN_ODDS * odds_hit * odds_miss, bbf::MIN_ODDS},
          {bbf::MIN_ODDS * odds_hit * odds_miss, bbf::MIN_ODDS, bbf::MIN_ODDS},
      };
  const char expected_cost[3][3] =
      {
          {0, 26, 8},
          {68, 8, 0},
          {8, 0, 0},
      };
  BlockMemGridmap<char, 3, 2> cm;

  CostmapBBF bbf;
  bbf.reset(CostmapBBF::Vec(3, 3, 1));
  bbf.clear();

  cm.reset(CostmapBBF::Vec(3, 3, 1));
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      cm[CostmapBBF::Vec(i, j, 0)] = costmap0[i][j];

  bbf.remember(&cm, CostmapBBF::Vec(0, 0, 0), odds_hit, odds_miss, 1, 2);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      cm[CostmapBBF::Vec(i, j, 0)] = costmap1[i][j];

  bbf.remember(&cm, CostmapBBF::Vec(0, 0, 0), odds_hit, odds_miss, 1, 2);

  const auto cb = [expected_odds](const CostmapBBF::Vec& p, bbf::BinaryBayesFilter& b)
  {
    EXPECT_FLOAT_EQ(expected_odds[p[0]][p[1]], b.get()) << "at " << p[0] << ", " << p[1];
  };
  bbf.forEach(cb);
  bbf.updateCostmap();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_EQ(expected_cost[i][j], bbf.getCost(CostmapBBF::Vec(i, j, 0))) << "at " << i << ", " << j;

  bbf.clear();
  const auto cb_cleared = [](const CostmapBBF::Vec& p, bbf::BinaryBayesFilter& b)
  {
    EXPECT_FLOAT_EQ(bbf::MIN_ODDS, b.get()) << "at " << p[0] << ", " << p[1];
  };
  bbf.forEach(cb_cleared);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_EQ(0, bbf.getCost(CostmapBBF::Vec(i, j, 0))) << "at " << i << ", " << j;
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
