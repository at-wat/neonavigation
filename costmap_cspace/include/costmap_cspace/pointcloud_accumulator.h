/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#ifndef COSTMAP_CSPACE_POINTCLOUD_ACCUMULATOR_H
#define COSTMAP_CSPACE_POINTCLOUD_ACCUMULATOR_H

#include <ros/ros.h>

#include <list>

namespace costmap_cspace
{
template <typename T>
class PointcloudAccumurator
{
public:
  class Points : public T
  {
  public:
    ros::Time stamp_;

    Points(const T& points, const ros::Time& stamp)
      : T(points)
      , stamp_(stamp)
    {
    }
  };

  PointcloudAccumurator()
  {
  }

  explicit PointcloudAccumurator(const ros::Duration& duration)
  {
    reset(duration);
  }

  void reset(const ros::Duration& duration)
  {
    time_to_hold_ = duration;
    clear();
  }

  void clear()
  {
    points_.clear();
  }

  void push(const Points& points)
  {
    for (auto it = points_.begin(); it != points_.end(); ++it)
    {
      if (it->stamp_ + time_to_hold_ < points.stamp_)
      {
        it = points_.erase(it);
        continue;
      }
      break;
    }
    points_.push_back(points);
  }

  typename std::list<Points>::iterator begin()
  {
    return points_.begin();
  }
  typename std::list<Points>::iterator end()
  {
    return points_.end();
  }

  typename std::list<Points>::const_iterator begin() const
  {
    return points_.cbegin();
  }
  typename std::list<Points>::const_iterator end() const
  {
    return points_.cend();
  }

protected:
  ros::Duration time_to_hold_;
  std::list<Points> points_;
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_POINTCLOUD_ACCUMULATOR_H
