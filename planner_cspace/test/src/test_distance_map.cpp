/*
 * Copyright (c) 2021, the neonavigation authors
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

#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <omp.h>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/distance_map.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>

namespace planner_cspace
{
namespace planner_3d
{
using Vec3 = CyclicVecInt<3, 2>;
using Astar = GridAstar<3, 2>;

namespace
{
std::string xyStr(const float x, const float y)
{
  return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

void debugOutput(
    const DistanceMap& dm,
    const Astar::Gridmap<char, 0x80>& cm_rough,
    const Astar::Vec& s, const Astar::Vec& e)
{
  for (int y = 0; y < cm_rough.size()[1]; y++)
  {
    for (int x = 0; x < cm_rough.size()[0]; x++)
    {
      const Astar::Vec pos(x, y, 0);
      const float d = dm[pos];

      const char type = (pos == s ? 's' : (pos == e ? 'e' : ' '));
      if (d == std::numeric_limits<float>::max())
      {
        fprintf(stderr, "xxx%c ", type);
        continue;
      }
      else if (cm_rough[pos] == 100)
      {
        fprintf(stderr, "***%c ", type);
        continue;
      }
      fprintf(stderr, "%3.1f%c ", d, type);
    }
    fprintf(stderr, "\n");
  }
}
}  // namespace

class DistanceMapTest : public ::testing::Test
{
protected:
  const int w_ = 10;
  const int h_ = 10;
  const int angle_ = 10;

  const Astar::Vec s_;
  const Astar::Vec e_;

  const Astar::Vecf ec_;
  const float tolerance_ = 0.4;

  Astar::Gridmap<char, 0x80> cm_rough_;
  CostmapBBF bbf_costmap_;

  DistanceMap dm_;

  DistanceMapTest()
    : s_(2, 2, 0)
    , e_(8, 8, 0)
    , ec_(0.5f, 0.5f, 0.2f)
    , dm_(cm_rough_, bbf_costmap_)
  {
    const int range = 4;
    const int local_range = 10;
    CostCoeff cc;
    cc.weight_costmap_ = 1.0f;
    omp_set_num_threads(2);
    dm_.setParams(cc, 64 * 2);

    costmap_cspace_msgs::MapMetaData3D map_info;
    map_info.width = w_;
    map_info.height = h_;
    map_info.angle = angle_;
    map_info.linear_resolution = 1.0;
    map_info.angular_resolution = M_PI * 2 / angle_;

    const Astar::Vec size3d(w_, h_, angle_);
    const Astar::Vec size2d(w_, h_, 1);
    Astar::Gridmap<char, 0x40> cm;
    Astar::Gridmap<char, 0x80> cm_hyst;
    GridAstarModel3D::Ptr model(
        new GridAstarModel3D(
            map_info,
            ec_,
            local_range,
            dm_.gridmap(), cm, cm_hyst, cm_rough_,
            cc, range));
    cm.reset(size3d);
    cm_hyst.reset(size3d);
    cm_rough_.reset(size2d);
    bbf_costmap_.reset(size2d);

    const DistanceMap::Params dmp =
        {
            .euclid_cost = ec_,
            .range = range,
            .local_range = local_range,
            .longcut_range = 10,
            .size = size2d,
            .resolution = map_info.linear_resolution,
        };
    dm_.init(model, dmp);
  }

  void setupCostmap()
  {
    /*
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][S][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [X][X][X][ ][X][X][X][X][X][X]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][G][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
     */
    cm_rough_.clear(0);
    for (int x = 0; x < w_; x++)
    {
      cm_rough_[Astar::Vec(x, 5, 0)] = 100;
    }
    cm_rough_[Astar::Vec(3, 5, 0)] = 0;
  }

  void validateDistance(
      const Astar::Vec p,
      const float d,
      const std::string& msg) const
  {
    if (p[1] < 5)
    {
      ASSERT_NEAR(
          2.7 + ec_[0] * (p - Astar::Vec(3, 5, 0)).norm(),
          d, tolerance_)
          << msg;
    }
    else if (p[1] > 5)
    {
      ASSERT_NEAR(
          ec_[0] * (p - e_).norm(),
          d, tolerance_)
          << msg;
    }
    else if (p == Astar::Vec(3, 5, 0))
    {
      ASSERT_NEAR(2.7, d, tolerance_) << msg;
    }
  }

  bool validate(const std::string& msg) const
  {
    for (int y = 0; y < h_; y++)
    {
      for (int x = 0; x < w_; x++)
      {
        const Astar::Vec pos(x, y, 0);
        validateDistance(pos, dm_[pos], msg + " failed at " + xyStr(x, y));
        if (::testing::Test::HasFatalFailure())
          return false;
      }
    }
    return true;
  }
};

class DistanceMapTestWithParam
  : public DistanceMapTest,
    public ::testing::WithParamInterface<std::vector<Vec3>>
{
};

INSTANTIATE_TEST_CASE_P(
    UpdateWithTemporaryObstacles,
    DistanceMapTestWithParam,
    ::testing::Values(
        std::vector<Vec3>(
            {
                Vec3(7, 7, 0),
            }),  // NOLINT(whitespace/braces)
        std::vector<Vec3>(
            {
                Vec3(4, 2, 0),
                Vec3(7, 2, 0),
            }),  // NOLINT(whitespace/braces)
        std::vector<Vec3>(
            {
                // goal is unreachable
                Vec3(3, 5, 0),
            }),  // NOLINT(whitespace/braces)
        std::vector<Vec3>()));

TEST_F(DistanceMapTest, Create)
{
  setupCostmap();

  dm_.create(s_, e_);
  debugOutput(dm_, cm_rough_, s_, e_);

  if (!validate("create"))
  {
    debugOutput(dm_, cm_rough_, s_, e_);
    return;
  }
}

TEST_P(DistanceMapTestWithParam, Update)
{
  const std::vector<Vec3> obstacles = GetParam();

  // Slide update area to check all combination of updated area
  for (int from_y = 0; from_y < h_; from_y++)
  {
    for (int from_x = 0; from_x < w_; from_x++)
    {
      const std::string from(xyStr(from_x, from_y));
      for (int to_y = from_y; to_y < h_; to_y++)
      {
        for (int to_x = from_x; to_x < w_; to_x++)
        {
          const std::string to(xyStr(to_x, to_y));

          // Add obstacles to the costmap and create distance map
          setupCostmap();
          for (const Vec3& obstacle : obstacles)
          {
            if (from_x < obstacle[0] && obstacle[0] < to_x &&
                from_y < obstacle[1] && obstacle[1] < to_y)
            {
              cm_rough_[obstacle] = 100;
            }
          }
          dm_.create(s_, e_);

          // Reset costmap
          setupCostmap();

          // Update multiple times to check idempotence
          for (int i = 0; i < 5; i++)
          {
            dm_.update(
                s_, e_,
                DistanceMap::Rect(
                    Vec3(from_x, from_y, 0), Vec3(to_x, to_y, 0)));
            if (!validate("update " + from + "-" + to))
            {
              debugOutput(dm_, cm_rough_, s_, e_);
              return;
            }
          }
        }
      }
    }
  }
}

class DistanceMapTestLongMap : public ::testing::Test
{
protected:
  const int w_ = 20;
  const int h_ = 3;
  const int angle_ = 10;

  const int range_ = 2;
  const int local_range_ = 2;
  const int longcut_range_ = 2;

  const Astar::Vecf ec_;

  const int search_range_ = 4;

  Astar::Gridmap<char, 0x80> cm_rough_;
  CostmapBBF bbf_costmap_;

  DistanceMap dm_;

  DistanceMapTestLongMap()
    : ec_(0.5f, 0.5f, 0.2f)
    , dm_(cm_rough_, bbf_costmap_)
  {
    CostCoeff cc;
    cc.weight_costmap_ = 1.0f;
    omp_set_num_threads(1);
    dm_.setParams(cc, 1);

    costmap_cspace_msgs::MapMetaData3D map_info;
    map_info.width = w_;
    map_info.height = h_;
    map_info.angle = angle_;
    map_info.linear_resolution = 1.0;
    map_info.angular_resolution = M_PI * 2 / angle_;

    const Astar::Vec size3d(w_, h_, angle_);
    const Astar::Vec size2d(w_, h_, 1);
    Astar::Gridmap<char, 0x40> cm;
    Astar::Gridmap<char, 0x80> cm_hyst;
    GridAstarModel3D::Ptr model(
        new GridAstarModel3D(
            map_info,
            ec_,
            local_range_,
            dm_.gridmap(), cm, cm_hyst, cm_rough_,
            cc, range_));
    cm.reset(size3d);
    cm_hyst.reset(size3d);
    cm_rough_.reset(size2d);
    bbf_costmap_.reset(size2d);
    cm_rough_.clear(0);

    const DistanceMap::Params dmp =
        {
            .euclid_cost = ec_,
            .range = range_,
            .local_range = local_range_,
            .longcut_range = longcut_range_,
            .size = size2d,
            .resolution = map_info.linear_resolution,
        };
    dm_.init(model, dmp);
  }
};

TEST_F(DistanceMapTestLongMap, OvershootRange)
{
  const Astar::Vec s(4, 1, 0);
  const Astar::Vec e(1, 1, 0);

  const float tolerance = 0.4;

  const int range_overshoot = range_ + local_range_ + longcut_range_ + search_range_;

  const auto validate = [this, s, e, tolerance, range_overshoot]()
  {
    for (int y = 0; y < h_; y++)
    {
      for (int x = 0; x < w_; x++)
      {
        const Astar::Vec p(x, y, 0);
        const float cost = dm_[p];
        const float d = (p - e).norm();
        const float d_from_start = (p - s).norm();
        if (x < s[0] || d_from_start <= range_overshoot)
        {
          ASSERT_NEAR(ec_[0] * d, cost, tolerance) << xyStr(x, y);
        }
        else if (d_from_start > range_overshoot)
        {
          ASSERT_EQ(std::numeric_limits<float>::max(), cost) << xyStr(x, y);
        }
      }
    }
  };

  dm_.create(s, e);
  validate();
  if (::testing::Test::HasFatalFailure())
    debugOutput(dm_, cm_rough_, s, e);

  dm_.update(
      s, e,
      DistanceMap::Rect(
          Vec3(8, 0, 0), Vec3(9, 2, 0)));
  validate();
  if (::testing::Test::HasFatalFailure())
    debugOutput(dm_, cm_rough_, s, e);
}

TEST_F(DistanceMapTestLongMap, BlockedAndRecovered)
{
  const Astar::Vec s(7, 1, 0);
  const Astar::Vec e(1, 1, 0);

  const float tolerance = 0.4;

  const int range_overshoot = range_ + local_range_ + longcut_range_ + search_range_;
  const auto validate = [this, s, e, tolerance, range_overshoot](const int obstacle_x)
  {
    for (int y = 0; y < h_; y++)
    {
      for (int x = 0; x < w_; x++)
      {
        const Astar::Vec p(x, y, 0);
        const float cost = dm_[p];
        const float d = (p - e).norm();
        const float d_from_start = (p - s).norm();
        if ((x < s[0] || d_from_start < range_overshoot) && x < obstacle_x)
        {
          ASSERT_NEAR(ec_[0] * d, cost, tolerance) << xyStr(x, y);
        }
        else if (x > obstacle_x)
        {
          ASSERT_EQ(std::numeric_limits<float>::max(), cost) << xyStr(x, y);
        }
      }
    }
  };

  dm_.create(s, e);

  for (int y = 0; y < h_; y++)
  {
    cm_rough_[Astar::Vec(5, y, 0)] = 100;
  }
  dm_.update(
      s, e,
      DistanceMap::Rect(
          Vec3(5, 0, 0), Vec3(5, 2, 0)));
  validate(5);
  if (::testing::Test::HasFatalFailure())
  {
    debugOutput(dm_, cm_rough_, s, e);
    return;
  }

  cm_rough_.clear(0);

  dm_.update(
      s, e,
      DistanceMap::Rect(
          Vec3(5, 0, 0), Vec3(5, 2, 0)));
  validate(w_);
  if (::testing::Test::HasFatalFailure())
  {
    debugOutput(dm_, cm_rough_, s, e);
    return;
  }
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
