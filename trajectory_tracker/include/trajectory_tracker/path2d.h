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
  inline void rotate(const float ang)
  {
    const float org_x = pos_.x();
    const float org_y = pos_.y();
    const float cos_v = std::cos(ang);
    const float sin_v = std::sin(ang);

    pos_.x() = cos_v * org_x - sin_v * org_y;
    pos_.y() = sin_v * org_x + cos_v * org_y;
    yaw_ += ang;
    while (yaw_ < 0)
      yaw_ += 2 * M_PI;
    while (yaw_ > 2 * M_PI)
      yaw_ -= 2 * M_PI;
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
      // stop reading forward if the path is switching back or in-place turning
      if (it->pos_ == it_prev->pos_)
        return it;

      const Eigen::Vector2d inc = it->pos_ - it_prev->pos_;

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
      const float max_search_range = 0,
      const float epsilon = 1e-6) const
  {
    if (begin == end)
      return end;

    float distance_path_search = 0;
    ConstIterator it_nearest = begin;
    float min_dist = (begin->pos_ - target).norm() + epsilon;

    ConstIterator it_prev = begin;
    for (ConstIterator it = begin + 1; it < end; ++it)
    {
      const Eigen::Vector2d inc = it->pos_ - it_prev->pos_;
      distance_path_search += inc.norm();
      if (max_search_range > 0 && distance_path_search > max_search_range)
        break;

      const float d =
          trajectory_tracker::lineStripDistanceSigned(it_prev->pos_, it->pos_, target);

      // Use earlier point if the robot is same distance from two line strip
      // to avoid chattering.
      const float d_compare = (d > 0) ? d : (-d - epsilon);
      const float d_abs = std::abs(d);

      // If it is the last point, select it as priority
      // to calculate correct remained distance.
      if (d_compare <= min_dist || (it + 1 == end && d_abs <= min_dist + epsilon))
      {
        min_dist = d_abs;
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
    if (end - begin <= 1)
    {
      return 0;
    }
    else if (end - begin == 2)
    {
      // When only two poses are remained, the logic same as planner_cspace::planner_3d::RotationCache is used.
      ConstIterator it_prev = begin;
      ConstIterator it = begin + 1;
      Pose2D rel(it->pos_ - it_prev->pos_, it->yaw_, 0.0f);
      rel.rotate(-it_prev->yaw_);
      const float sin_v = std::sin(rel.yaw_);
      static const float EPS = 1.0e-6f;
      if (std::abs(sin_v) < EPS)
      {
        return 0;
      }
      const float cos_v = std::cos(rel.yaw_);
      const float r1 = rel.pos_.y() + rel.pos_.x() * cos_v / sin_v;
      const float r2 = std::copysign(
          std::sqrt(std::pow(rel.pos_.x(), 2) + std::pow(rel.pos_.x() * cos_v / sin_v, 2)),
          rel.pos_.x() * sin_v);
      return 1.0f / ((r1 + r2) / 2);
    }
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
