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

#ifndef PLANNER_CSPACE_PLANNER_3D_START_POSE_PREDICTOR_H
#define PLANNER_CSPACE_PLANNER_3D_START_POSE_PREDICTOR_H

#include <vector>

#include <nav_msgs/Path.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/grid_metric_converter.h>
#include <trajectory_tracker/path2d.h>

namespace planner_cspace
{
namespace planner_3d
{

class StartPosePredictor
{
public:
  using Astar = GridAstar<3, 2>;

  struct Config
  {
    double prediction_sec_;
    double switch_back_prediction_sec_;
    double dist_stop_;
    double lin_vel_;
    double ang_vel_;
  };
  void setConfig(const Config& config)
  {
    config_ = config;
  }
  bool process(const geometry_msgs::Pose& robot_pose,
               const GridAstar<3, 2>::Gridmap<char, 0x40>& cm,
               const costmap_cspace_msgs::MapMetaData3D& map_info,
               const nav_msgs::Path& previous_path_msg,
               Astar::Vec& result_start_grid);
  void clear();
  const nav_msgs::Path& getPreservedPath() const
  {
    return preserved_path_;
  }
  const double getPreservedPathLength() const
  {
    return preserved_path_length_;
  }

private:
  bool removeAlreadyPassed(const geometry_msgs::Pose& robot_pose);
  double getInitialETA(
      const geometry_msgs::Pose& robot_pose, const trajectory_tracker::Pose2D& initial_path_pose) const;
  bool buildResults(const trajectory_tracker::Path2D::ConstIterator& expected_start_pose_it,
                    const geometry_msgs::Pose& start_metric,
                    const GridAstar<3, 2>::Gridmap<char, 0x40>& cm,
                    Astar::Vec& start_grid);
  bool isGridCenter(const trajectory_tracker::Pose2D& pose) const;
  bool isPathColliding(const trajectory_tracker::Path2D::ConstIterator& begin,
                       const trajectory_tracker::Path2D::ConstIterator& end,
                       const GridAstar<3, 2>::Gridmap<char, 0x40>& cm);
  trajectory_tracker::Path2D::ConstIterator getSwitchBack(const std::vector<double>& etas) const;
  trajectory_tracker::Path2D::ConstIterator getExpectedPose(const std::vector<double>& etas) const;

  Config config_;
  costmap_cspace_msgs::MapMetaData3D map_info_;
  trajectory_tracker::Path2D previous_path_2d_;
  nav_msgs::Path preserved_path_;
  double preserved_path_length_ = 0.0;
};

}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_START_POSE_PREDICTOR_H
