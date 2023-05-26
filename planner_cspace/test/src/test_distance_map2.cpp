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
  const int w_ = 10;
  const int h_ = 7;
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
    , e_(8, 6, 0)
    , ec_(0.5f, 0.5f, 0.2f)
    , dm_(cm_rough_, bbf_costmap_)
  {
    const int range = 3;
    const int local_range = 3;
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
    dm_.setParams(cc, 64 * 2);

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

    cm.clear(0);
    cm_hyst.clear(0);
    cm_rough_.clear(0);
    bbf_costmap_.clear();

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
       [X][ ][X][X][X][X][X][X][ ][X]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][G][ ]
       [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
     */
    cm_rough_.clear(0);
    for (int x = 0; x < w_; x++)
    {
      cm_rough_[Astar::Vec(x, 3, 0)] = 100;
    }
    cm_rough_[Astar::Vec(1, 3, 0)] = 0;
    cm_rough_[Astar::Vec(8, 3, 0)] = 0;
  }

  void validateDistance(
      const Astar::Vec p,
      const float d,
      const std::string& msg) const
  {
    // TODO(at-wat): implement check
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
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
