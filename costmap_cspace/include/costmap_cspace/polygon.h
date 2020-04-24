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

#ifndef COSTMAP_CSPACE_POLYGON_H
#define COSTMAP_CSPACE_POLYGON_H

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <xmlrpcpp/XmlRpcException.h>

namespace costmap_cspace
{
class Vec
{
public:
  float c[2];
  float& operator[](const int& i)
  {
    ROS_ASSERT(i < 2);
    return c[i];
  }
  const float& operator[](const int& i) const
  {
    ROS_ASSERT(i < 2);
    return c[i];
  }
  Vec operator-(const Vec& a) const
  {
    Vec out = *this;
    out[0] -= a[0];
    out[1] -= a[1];
    return out;
  }
  float cross(const Vec& a) const
  {
    return (*this)[0] * a[1] - (*this)[1] * a[0];
  }
  float dot(const Vec& a) const
  {
    return (*this)[0] * a[0] + (*this)[1] * a[1];
  }
  float dist(const Vec& a) const
  {
    return std::hypot((*this)[0] - a[0], (*this)[1] - a[1]);
  }
  float dist_line(const Vec& a, const Vec& b) const
  {
    return (b - a).cross((*this) - a) / b.dist(a);
  }
  float dist_linestrip(const Vec& a, const Vec& b) const
  {
    if ((b - a).dot((*this) - a) <= 0)
      return this->dist(a);
    if ((a - b).dot((*this) - b) <= 0)
      return this->dist(b);
    return std::abs(this->dist_line(a, b));
  }
};
class Polygon
{
public:
  std::vector<Vec> v;

  Polygon()
  {
  }
  explicit Polygon(const XmlRpc::XmlRpcValue footprint_xml_const)
  {
    XmlRpc::XmlRpcValue footprint_xml = footprint_xml_const;
    if (footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
    {
      throw std::runtime_error("Invalid footprint xml.");
    }

    for (int i = 0; i < footprint_xml.size(); i++)
    {
      Vec p;
      try
      {
        p[0] = static_cast<double>(footprint_xml[i][0]);
        p[1] = static_cast<double>(footprint_xml[i][1]);
      }
      catch (XmlRpc::XmlRpcException& e)
      {
        throw std::runtime_error(("Invalid footprint xml." + e.getMessage()).c_str());
      }

      v.push_back(p);
    }
    v.push_back(v.front());
  }
  geometry_msgs::PolygonStamped toMsg() const
  {
    geometry_msgs::PolygonStamped msg;

    msg.polygon.points.clear();
    msg.header.frame_id = "base_link";
    for (const auto& p : v)
    {
      geometry_msgs::Point32 point;
      point.x = p[0];
      point.y = p[1];
      point.z = 0;
      msg.polygon.points.push_back(point);
    }
    msg.polygon.points.push_back(msg.polygon.points[0]);

    return msg;
  }
  float radius() const
  {
    float radius = 0;
    for (const auto& p : v)
    {
      const auto dist = std::hypot(p[0], p[1]);
      if (dist > radius)
        radius = dist;
    }
    return radius;
  }
  void move(const float& x, const float& y, const float& yaw)
  {
    float cos_v = cosf(yaw);
    float sin_v = sinf(yaw);
    for (auto& p : v)
    {
      auto tmp = p;
      p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
      p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
    }
  }
  bool inside(const Vec& a) const
  {
    int cn = 0;
    for (size_t i = 0; i < v.size() - 1; i++)
    {
      auto& v1 = v[i];
      auto& v2 = v[i + 1];
      if ((v1[1] <= a[1] && a[1] < v2[1]) ||
          (v2[1] <= a[1] && a[1] < v1[1]))
      {
        float lx;
        lx = v1[0] + (v2[0] - v1[0]) * (a[1] - v1[1]) / (v2[1] - v1[1]);
        if (a[0] < lx)
          cn++;
      }
    }
    return ((cn & 1) == 1);
  }
  float dist(const Vec& a) const
  {
    float dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < v.size() - 1; i++)
    {
      auto& v1 = v[i];
      auto& v2 = v[i + 1];
      auto d = a.dist_linestrip(v1, v2);
      if (d < dist)
        dist = d;
    }
    return dist;
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_POLYGON_H
