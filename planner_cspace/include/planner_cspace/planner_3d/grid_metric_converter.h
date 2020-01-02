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

#ifndef PLANNER_CSPACE_PLANNER_3D_GRID_METRIC_CONVERTER_H
#define PLANNER_CSPACE_PLANNER_3D_GRID_METRIC_CONVERTER_H

#include <cmath>
#include <list>
#include <memory>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <planner_cspace/cyclic_vec.h>

namespace planner_cspace
{
namespace planner_3d
{
namespace grid_metric_converter
{
template <typename T>
void grid2Metric(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    const T x, const T y, const T yaw,
    float& gx, float& gy, float& gyaw)
{
  static_assert(
      std::is_same<float, T>() || std::is_same<int, T>(), "T must be float or int");

  gx = (x + 0.5) * map_info.linear_resolution + map_info.origin.position.x;
  gy = (y + 0.5) * map_info.linear_resolution + map_info.origin.position.y;
  gyaw = yaw * map_info.angular_resolution;
}
inline void metric2Grid(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    int& x, int& y, int& yaw,
    const float gx, const float gy, const float gyaw)
{
  x = static_cast<int>(std::floor((gx - map_info.origin.position.x) / map_info.linear_resolution));
  y = static_cast<int>(std::floor((gy - map_info.origin.position.y) / map_info.linear_resolution));
  yaw = std::lround(gyaw / map_info.angular_resolution);
}
inline void metric2Grid(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    float& x, float& y, float& yaw,
    const float gx, const float gy, const float gyaw)
{
  x = (gx - map_info.origin.position.x) / map_info.linear_resolution;
  y = (gy - map_info.origin.position.y) / map_info.linear_resolution;
  yaw = gyaw / map_info.angular_resolution;
}

template <template <class, class> class STL_CONTAINER = std::list>
void grid2MetricPath(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    const STL_CONTAINER<CyclicVecFloat<3, 2>,
                        std::allocator<CyclicVecFloat<3, 2>>>& path_grid,
    nav_msgs::Path& path)
{
  for (const auto& p : path_grid)
  {
    float x, y, yaw;
    grid2Metric(map_info, p[0], p[1], p[2], x, y, yaw);
    geometry_msgs::PoseStamped ps;
    ps.header = path.header;

    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0;
    ps.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    path.poses.push_back(ps);
  }
}
}  // namespace grid_metric_converter
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_GRID_METRIC_CONVERTER_H
