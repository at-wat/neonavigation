/*
 * Copyright (c) 2018, the neonavigation authors
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

#ifndef TRAJECTORY_TRACKER_PATH2D_H
#define TRAJECTORY_TRACKER_PATH2D_H

#include <vector>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>

#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/average.h>

namespace trajectory_tracker
{
class Pose2D
{
public:
  Eigen::Vector2d pos_;
  float yaw_;
  float velocity_;

  inline Pose2D()
    : pos_(0, 0)
    , yaw_(0)
    , velocity_(0)
  {
  }
  inline Pose2D(const Eigen::Vector2d& p, float y, float velocity)
    : pos_(p)
    , yaw_(y)
    , velocity_(velocity)
  {
  }
  inline explicit Pose2D(const geometry_msgs::Pose& pose, float velocity)
    : pos_(Eigen::Vector2d(pose.position.x, pose.position.y))
    , yaw_(tf2::getYaw(pose.orientation))
    , velocity_(velocity)
  {
  }
};
class Path2D : public std::vector<Pose2D>
{
private:
  using Super = std::vector<Pose2D>;

public:
  using Iterator = std::vector<Pose2D>::iterator;
  using ConstIterator = std::vector<Pose2D>::const_iterator;

  inline float length() const
  {
    double l = 0;
    for (size_t i = 1; i < size(); i++)
      l += ((*this)[i - 1].pos_ - (*this)[i].pos_).norm();
    return l;
  }
  inline ConstIterator findLocalGoal(
      const ConstIterator& begin,
      const ConstIterator& end,
      const bool allow_backward_motion) const
  {
    float sign_vel_prev = 0;
    ConstIterator it_prev = begin;
    for (ConstIterator it = begin + 1; it < end; ++it)
    {
      const Eigen::Vector2d inc = it->pos_ - it_prev->pos_;

      // stop reading forward if the path is switching back
      const float angle = atan2(inc[1], inc[0]);
      const float angle_pose = allow_backward_motion ? it->yaw_ : angle;
      const float sign_vel_req = std::cos(angle) * std::cos(angle_pose) + std::sin(angle) * std::sin(angle_pose);
      if (sign_vel_prev * sign_vel_req < 0)
        return it;
      sign_vel_prev = sign_vel_req;
      it_prev = it;
    }
    return end;
  }
  inline ConstIterator findNearest(
      const ConstIterator& begin,
      const ConstIterator& end,
      const Eigen::Vector2d& target,
      const float max_search_range = 0) const
  {
    float distance_path_search = 0;
    float min_dist = std::numeric_limits<float>::max();
    ConstIterator it_nearest = Super::end();

    ConstIterator it_prev = begin;
    for (ConstIterator it = begin + 1; it < end; ++it)
    {
      const Eigen::Vector2d inc = it->pos_ - it_prev->pos_;
      distance_path_search += inc.norm();
      if (max_search_range > 0 && distance_path_search > max_search_range)
        break;

      const float d =
          trajectory_tracker::lineStripDistance(it_prev->pos_, it->pos_, target);
      if (d <= min_dist)
      {
        min_dist = d;
        it_nearest = it;
      }
      it_prev = it;
    }
    return it_nearest;
  }
  inline double remainedDistance(
      const ConstIterator& begin,
      const ConstIterator& nearest,
      const ConstIterator& end,
      const Eigen::Vector2d& target_on_line) const
  {
    double remain = (nearest->pos_ - target_on_line).norm();
    if (nearest + 1 >= end)
    {
      const ConstIterator last = end - 1;
      const ConstIterator last_pre = end - 2;
      if (last_pre < begin || last < begin)
      {
        // no enough points: orientation control mode
        return 0;
      }
      const Eigen::Vector2d vec_path = last->pos_ - last_pre->pos_;
      const Eigen::Vector2d vec_remain = last->pos_ - target_on_line;
      if (vec_path.dot(vec_remain) >= 0)
      {
        // ongoing
        return remain;
      }
      // overshoot
      return -remain;
    }
    ConstIterator it_prev = nearest;
    for (ConstIterator it = nearest + 1; it < end; ++it)
    {
      remain += (it_prev->pos_ - it->pos_).norm();
      it_prev = it;
    }
    return remain;
  }
  inline float getCurvature(
      const ConstIterator& begin,
      const ConstIterator& end,
      const Eigen::Vector2d& target_on_line,
      const float max_search_range) const
  {
    const float max_search_range_sq = max_search_range * max_search_range;
    trajectory_tracker::Average<float> curv;
    ConstIterator it_prev2 = begin;
    ConstIterator it_prev1 = begin + 1;
    for (ConstIterator it = begin + 2; it < end; ++it)
    {
      curv += trajectory_tracker::curv3p(it_prev2->pos_, it_prev1->pos_, it->pos_);
      if ((it->pos_ - target_on_line).squaredNorm() > max_search_range_sq)
        break;
      it_prev2 = it_prev1;
      it_prev1 = it;
    }
    return curv;
  }
};
}  // namespace trajectory_tracker

#endif  // TRAJECTORY_TRACKER_PATH2D_H
