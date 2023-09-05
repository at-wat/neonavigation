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

#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <tf2/utils.h>

#include <planner_cspace/planner_3d/start_pose_predictor.h>

namespace planner_cspace
{
namespace planner_3d
{
bool StartPosePredictor::process(const geometry_msgs::Pose& robot_pose,
                                 const GridAstar<3, 2>::Gridmap<char, 0x40>& cm,
                                 const costmap_cspace_msgs::MapMetaData3D& map_info,
                                 const nav_msgs::Path& previous_path_msg,
                                 StartPosePredictor::Astar::Vec& result_start_grid)
{
  clear();
  preserved_path_.header = previous_path_msg.header;
  previous_path_2d_.fromMsg(previous_path_msg);
  map_info_ = map_info;

  if (previous_path_2d_.empty() || !removeAlreadyPassed(robot_pose))
  {
    return false;
  }
  const double initial_eta = getInitialETA(robot_pose, previous_path_2d_.front());
  const std::vector<double> etas = previous_path_2d_.getEstimatedTimeOfArrivals(
      previous_path_2d_.begin(), previous_path_2d_.end(), config_.lin_vel_, config_.ang_vel_, initial_eta);

  const trajectory_tracker::Path2D::ConstIterator local_goal_it = getSwitchBack(etas);
  if (local_goal_it != previous_path_2d_.end())
  {
    return buildResults(local_goal_it, robot_pose, cm, result_start_grid);
  }
  const trajectory_tracker::Path2D::ConstIterator expected_pose_it = getExpectedPose(etas);
  if (expected_pose_it != previous_path_2d_.end())
  {
    return buildResults(expected_pose_it, robot_pose, cm, result_start_grid);
  }
  ROS_DEBUG("The robot will reach the goal in %f sec.", etas.back());
  return buildResults(previous_path_2d_.end() - 1, robot_pose, cm, result_start_grid);
}

void planner_cspace::planner_3d::StartPosePredictor::clear()
{
  previous_path_2d_.clear();
  preserved_path_.poses.clear();
  preserved_path_length_ = 0.0;
}

bool StartPosePredictor::removeAlreadyPassed(const geometry_msgs::Pose& start_metric)
{
  const Eigen::Vector2d robot_pose_2d(start_metric.position.x, start_metric.position.y);
  double dist_err;
  trajectory_tracker::Path2D::ConstIterator it_nearest;
  std::tie(it_nearest, dist_err) =
      previous_path_2d_.findNearestWithDistance(previous_path_2d_.begin(), previous_path_2d_.end(), robot_pose_2d);
  if (dist_err > config_.dist_stop_)
  {
    ROS_WARN("The robot is too far from path. An empty path is published. dist: %f, thr: %f",
             dist_err, config_.dist_stop_);
    return false;
  }
  if (previous_path_2d_.size() == 1)
  {
    return true;
  }
  size_t nearest_pos = it_nearest - previous_path_2d_.begin();
  if (nearest_pos == 1)
  {
    // findNearestWithDistance() never returns begin() and (nearest_pos == 1) means that the first line strip is the
    // nearest. In this case, no points should be removed when the robot is behind the path.
    const double distance_from_start = std::hypot(start_metric.position.x - previous_path_2d_.begin()->pos_.x(),
                                                  start_metric.position.y - previous_path_2d_.begin()->pos_.y());
    if (std::abs(distance_from_start - dist_err) < 1.0e-6)
    {
      ROS_DEBUG("Nearest pose in previous path: (%f, %f, %f), dist: %f, index: 0, remaining poses num: %lu",
                previous_path_2d_.begin()->pos_.x(), previous_path_2d_.begin()->pos_.y(),
                previous_path_2d_.begin()->yaw_, dist_err, previous_path_2d_.size());
      return true;
    }
  }
  ROS_DEBUG("Nearest pose in previous path: (%f, %f, %f), dist: %f, index: %lu, remaining poses num: %lu",
            it_nearest->pos_.x(), it_nearest->pos_.y(), it_nearest->yaw_, dist_err, nearest_pos,
            previous_path_2d_.size() - nearest_pos);
  previous_path_2d_.erase(previous_path_2d_.begin(), it_nearest);
  return true;
}

double StartPosePredictor::getInitialETA(
    const geometry_msgs::Pose& robot_pose, const trajectory_tracker::Pose2D& initial_path_pose) const
{
  double angle_diff = std::abs(initial_path_pose.yaw_ - tf2::getYaw(robot_pose.orientation));
  angle_diff = (angle_diff > M_PI) ? 2 * M_PI - angle_diff : angle_diff;  // Normalize
  const double dist_diff = std::hypot(initial_path_pose.pos_.x() - robot_pose.position.x,
                                      initial_path_pose.pos_.y() - robot_pose.position.y);
  return std::max(std::abs(angle_diff) / config_.ang_vel_, dist_diff / config_.lin_vel_);
}

bool StartPosePredictor::buildResults(
    const trajectory_tracker::Path2D::ConstIterator& expected_start_pose_it,
    const geometry_msgs::Pose& robot_pose,
    const GridAstar<3, 2>::Gridmap<char, 0x40>& cm,
    StartPosePredictor::Astar::Vec& start_grid)
{
  if (isPathColliding(previous_path_2d_.begin(), expected_start_pose_it, cm))
  {
    ROS_WARN("The robot might collide with an obstacle during the next planning. An empty path is published.");
    clear();
    return false;
  }

  grid_metric_converter::metric2Grid(
      map_info_, start_grid[0], start_grid[1], start_grid[2],
      expected_start_pose_it->pos_.x(), expected_start_pose_it->pos_.y(), expected_start_pose_it->yaw_);
  start_grid.cycleUnsigned(map_info_.angle);
  const size_t previous_path_size = previous_path_2d_.size();
  previous_path_2d_.erase(expected_start_pose_it + 1, previous_path_2d_.end());
  ROS_DEBUG("Robot Pose: (%f, %f, %f), Start Pose: (%f, %f, %f), Start Grid: (%d, %d, %d), path size: %lu -> %lu",
            robot_pose.position.x, robot_pose.position.y, tf2::getYaw(robot_pose.orientation),
            expected_start_pose_it->pos_.x(), expected_start_pose_it->pos_.y(), expected_start_pose_it->yaw_,
            start_grid[0], start_grid[1], start_grid[2], previous_path_size, previous_path_2d_.size());
  if (!previous_path_2d_.empty())
  {
    preserved_path_length_ = previous_path_2d_.length();
    previous_path_2d_.toMsg(preserved_path_);
    // The last pose will be the first pose of the next planning, and it should be removed from preserved_path_.
    preserved_path_.poses.pop_back();
  }
  return true;
}

bool StartPosePredictor::isGridCenter(const trajectory_tracker::Pose2D& pose) const
{
  // Return true when all of the x, y and yaw in the grid coordinate are integers.
  Astar::Vecf grid;
  grid_metric_converter::metric2Grid(map_info_, grid[0], grid[1], grid[2], pose.pos_.x(), pose.pos_.y(), pose.yaw_);
  const float x_diff = std::abs(grid[0] - 0.5 - std::round(grid[0] - 0.5));
  const float y_diff = std::abs(grid[1] - 0.5 - std::round(grid[1] - 0.5));
  const float r_diff = std::abs(grid[2] - std::round(grid[2]));
  return (x_diff < 1.0e-3) && (y_diff < 1.0e-3) && (r_diff < 1.0e-3);
}

bool StartPosePredictor::isPathColliding(const trajectory_tracker::Path2D::ConstIterator& begin,
                                         const trajectory_tracker::Path2D::ConstIterator& end,
                                         const GridAstar<3, 2>::Gridmap<char, 0x40>& cm)
{
  for (auto it = begin; it != end; ++it)
  {
    Astar::Vec grid;
    grid_metric_converter::metric2Grid(map_info_, grid[0], grid[1], grid[2], it->pos_.x(), it->pos_.y(), it->yaw_);
    grid.cycleUnsigned(map_info_.angle);
    if (cm[grid] == 100)
    {
      ROS_DEBUG("Path colliding at (%f, %f, %f)", it->pos_.x(), it->pos_.y(), it->yaw_);
      return true;
    }
  }
  return false;
}

trajectory_tracker::Path2D::ConstIterator StartPosePredictor::getSwitchBack(const std::vector<double>& etas) const
{
  const auto local_goal_its = previous_path_2d_.enumerateLocalGoals(
      previous_path_2d_.begin(), previous_path_2d_.end(), true, false);
  if (local_goal_its.empty())
  {
    return previous_path_2d_.end();
  }

  for (const auto& local_goal : local_goal_its)
  {
    // Note that Path2D::findNearest() never returns the begin iterator and we can safely subtract 1.
    const size_t local_goal_index = local_goal - previous_path_2d_.begin() - 1;
    const double local_goal_eta = etas[local_goal_index];
    ROS_DEBUG("Switch back index: %lu, eta: %f", local_goal_index, local_goal_eta);
    if (isGridCenter(previous_path_2d_[local_goal_index]))
    {
      if ((config_.prediction_sec_ < local_goal_eta) &&
          (local_goal_eta < config_.switch_back_prediction_sec_))
      {
        ROS_INFO("The robot will reach a switch back point in %f sec. The next plan starts from the switch back.",
                 local_goal_eta);
        // Actually returning the next of the switch back point.
        return local_goal - 1;
      }
    }
    else
    {
      ROS_WARN("The switch back point is not placed on a grid center. (%f, %f, %f) -> (%f, %f, %f)",
               previous_path_2d_[local_goal_index].pos_.x(), previous_path_2d_[local_goal_index].pos_.y(),
               previous_path_2d_[local_goal_index].yaw_, previous_path_2d_[local_goal_index + 1].pos_.x(),
               previous_path_2d_[local_goal_index + 1].pos_.y(), previous_path_2d_[local_goal_index + 1].yaw_);
    }
  }
  return previous_path_2d_.end();
}

trajectory_tracker::Path2D::ConstIterator StartPosePredictor::getExpectedPose(const std::vector<double>& etas) const
{
  for (size_t i = 0; i < previous_path_2d_.size() - 1; ++i)
  {
    if (etas[i] > config_.prediction_sec_)
    {
      if (isGridCenter(previous_path_2d_[i]))
      {
        ROS_DEBUG("The robot will reach a grid center in %f sec.", etas[i]);
        return previous_path_2d_.begin() + i;
      }
    }
  }
  return previous_path_2d_.end();
}

}  // namespace planner_3d
}  // namespace planner_cspace
