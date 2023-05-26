/*
 * Copyright (c) 2023, the neonavigation authors
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

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <omp.h>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/distance_map_utils.h>
#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/costmap_bbf.h>
#include <planner_cspace/planner_3d/distance_map.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>

namespace planner_cspace
{
namespace planner_3d
{
using Vec3 = CyclicVecInt<3, 2>;
using Astar = GridAstar<3, 2>;

class DistanceMapTest : public ::testing::Test
{
protected:
  const int w_ = 12;
  const int h_ = 8;
  const int angle_ = 10;

  const Astar::Vecf ec_;
  const float tolerance_ = 0.4;

  Astar::Gridmap<char, 0x80> cm_rough_;
  CostmapBBF bbf_costmap_;

  DistanceMap dm_full_;
  DistanceMap dm_fast_;

  DistanceMapTest()
    : ec_(0.5f, 0.5f, 0.2f)
    , dm_full_(cm_rough_, bbf_costmap_)
    , dm_fast_(cm_rough_, bbf_costmap_)
  {
    const int range = 1;
    const int local_range = 1;
    omp_set_num_threads(2);

    costmap_cspace_msgs::MapMetaData3D map_info;
    map_info.width = w_;
    map_info.height = h_;
    map_info.angle = angle_;
    map_info.linear_resolution = 1.0;
    map_info.angular_resolution = M_PI * 2 / angle_;

    CostCoeff cc;
    cc.weight_costmap_ = 1.0f;
    cc.weight_remembered_ = 0.0f;
    cc.angle_resolution_aspect_ = 2.0f / tanf(map_info.angular_resolution);
    dm_full_.setParams(cc, 64 * 2);
    dm_fast_.setParams(cc, 64 * 2);

    const Astar::Vec size3d(w_, h_, angle_);
    const Astar::Vec size2d(w_, h_, 1);
    Astar::Gridmap<char, 0x40> cm;
    Astar::Gridmap<char, 0x80> cm_hyst;
    GridAstarModel3D::Ptr model(
        new GridAstarModel3D(
            map_info,
            ec_,
            local_range,
            dm_full_.gridmap(), cm, cm_hyst, cm_rough_,
            cc, range));
    cm.reset(size3d);
    cm_hyst.reset(size3d);
    cm_rough_.reset(size2d);
    bbf_costmap_.reset(size2d);

    cm.clear(0);
    cm_hyst.clear(0);
    cm_rough_.clear(0);
    bbf_costmap_.clear();

    const DistanceMap::Params dmp =
        {
            .euclid_cost = ec_,
            .range = range,
            .local_range = local_range,
            .longcut_range = 0,
            .size = size2d,
            .resolution = map_info.linear_resolution,
        };
    dm_full_.init(model, dmp);
    dm_fast_.init(model, dmp);
  }

  void setupCostmap()
  {
    /*
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][X][X][X][X][X][X][X][X][X][X][ ]
       [ ][X][X][X][X][X][X][X][X][X][X][ ]
       [ ][X][X][X][X][X][X][X][X][X][X][ ]
       [ ][X][X][X][X][X][X][X][X][X][X][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][G]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
     */
    cm_rough_.clear(0);
    for (int x = 1; x < w_ - 1; x++)
    {
      cm_rough_[Astar::Vec(x, 2, 0)] = 100;
      cm_rough_[Astar::Vec(x, 3, 0)] = 100;
      cm_rough_[Astar::Vec(x, 4, 0)] = 100;
      cm_rough_[Astar::Vec(x, 5, 0)] = 100;
    }
  }

  bool validate(const std::string& msg) const
  {
    for (int y = 0; y < h_; y++)
    {
      for (int x = 0; x < w_; x++)
      {
        const Astar::Vec pos(x, y, 0);
        if (cm_rough_[pos] == 100)
        {
          continue;
        }
        EXPECT_NEAR(dm_full_[pos], dm_fast_[pos], 0.5) << msg + " failed at " + xyStr(x, y);
        if (::testing::Test::HasFailure())
        {
          return false;
        }
      }
    }
    return true;
  }
};

TEST_F(DistanceMapTest, Create)
{
  setupCostmap();

  const Astar::Vec s(9, 1, 0);
  const Astar::Vec e(11, 6, 0);

  dm_full_.create(s, e);
  dm_fast_.create(s, e);
  debugOutput(dm_fast_, cm_rough_, s, e);

  if (!validate("create"))
  {
    fprintf(stderr, "expected:\n");
    debugOutput(dm_full_, cm_rough_, s, e);
    return;
  }

  cm_rough_[Astar::Vec(10, 0, 0)] = 100;
  cm_rough_[Astar::Vec(10, 1, 0)] = 100;

  dm_full_.create(s, e);
  dm_fast_.update(s, e, DistanceMap::Rect(Astar::Vec(9, 1, 0), Astar::Vec(11, 1, 0)));
  debugOutput(dm_fast_, cm_rough_, s, e);

  if (!validate("move1"))
  {
    fprintf(stderr, "expected:\n");
    debugOutput(dm_full_, cm_rough_, s, e);
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
