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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <random>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

pcl::PointXYZ operator-(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
  auto c = a;
  c.x -= b.x;
  c.y -= b.y;
  c.z -= b.z;
  return c;
}
pcl::PointXYZ operator+(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
  auto c = a;
  c.x += b.x;
  c.y += b.y;
  c.z += b.z;
  return c;
}
pcl::PointXYZ operator*(const pcl::PointXYZ &a, const float &b)
{
  auto c = a;
  c.x *= b;
  c.y *= b;
  c.z *= b;
  return c;
}

class SafetyLimiter
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_debug_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_disable_;
  tf::TransformListener tfl_;

  geometry_msgs::Twist twist_;
  sensor_msgs::PointCloud2 cloud_;
  double hz_;
  double timeout_;
  double disable_timeout_;
  double vel_[2];
  double acc_[2];
  double tmax_;
  double dt_;
  double tmargin_;
  double t_col_;
  double z_range_[2];
  float footprint_radius_;
  double downsample_grid_;
  std::string frame_id_;

  ros::Time last_disable_cmd_;
  ros::Duration hold_;
  ros::Time hold_off_;

  bool has_cloud_;
  bool has_twist_;

public:
  SafetyLimiter()
    : nh_("~")
  {
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 1, true);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud>("collision", 1, true);
    pub_debug_ = nh_.advertise<sensor_msgs::PointCloud>("debug", 1, true);
    sub_twist_ = nh_.subscribe("cmd_vel_in", 1, &SafetyLimiter::cbTwist, this);
    sub_cloud_ = nh_.subscribe("cloud", 1, &SafetyLimiter::cbCloud, this);
    sub_disable_ = nh_.subscribe("disable", 100, &SafetyLimiter::cbDisable, this);

    nh_.param("freq", hz_, 6.0);
    nh_.param("cloud_timeout", timeout_, 0.8);
    nh_.param("disable_timeout", disable_timeout_, 0.1);
    nh_.param("lin_vel", vel_[0], 0.5);
    nh_.param("lin_acc", acc_[0], 1.0);
    nh_.param("ang_vel", vel_[1], 0.8);
    nh_.param("ang_acc", acc_[1], 1.6);
    nh_.param("z_range_min", z_range_[0], 0.0);
    nh_.param("z_range_max", z_range_[1], 0.5);
    nh_.param("dt", dt_, 0.1);
    nh_.param("t_margin", tmargin_, 0.2);
    nh_.param("downsample_grid", downsample_grid_, 0.05);
    nh_.param("frame_id", frame_id_, std::string("base_link"));
    double hold_d;
    nh_.param("hold", hold_d, 0.0);
    hold_ = ros::Duration(hold_d);

    tmax_ = 0.0;
    for (int i = 0; i < 2; i++)
    {
      auto t = vel_[i] / acc_[i];
      if (tmax_ < t)
        tmax_ = t;
    }
    tmax_ *= 1.5;
    tmax_ += tmargin_;

    XmlRpc::XmlRpcValue footprint_xml;
    if (!nh_.hasParam("footprint"))
    {
      ROS_FATAL("Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified");
    }
    nh_.getParam("footprint", footprint_xml);
    if (footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
    {
      ROS_FATAL("Invalid footprint");
      throw std::runtime_error("Invalid footprint");
    }
    footprint_radius_ = 0;
    for (int i = 0; i < footprint_xml.size(); i++)
    {
      if (!XmlRpc_isNumber(footprint_xml[i][0]) ||
          !XmlRpc_isNumber(footprint_xml[i][1]))
      {
        ROS_FATAL("Invalid footprint value");
        throw std::runtime_error("Invalid footprint value");
      }

      vec v;
      v[0] = static_cast<double>(footprint_xml[i][0]);
      v[1] = static_cast<double>(footprint_xml[i][1]);
      footprint_p.v.push_back(v);

      auto dist = hypotf(v[0], v[2]);
      if (dist > footprint_radius_)
        footprint_radius_ = dist;
    }
    footprint_p.v.push_back(footprint_p.v.front());
    ROS_INFO("footprint radius: %0.3f", footprint_radius_);

    has_cloud_ = false;
    has_twist_ = true;

    last_disable_cmd_ = ros::Time(0);
  }
  void cbTimer(const ros::TimerEvent &event)
  {
    if (!has_twist_)
      return;
    if (!has_cloud_)
      return;

    ros::Time now = ros::Time::now();
    const double t_col_current = predict(twist_, cloud_);

    if (t_col_current != DBL_MAX)
    {
      if (t_col_current < t_col_)
        t_col_ = t_col_current;

      hold_off_ = now + hold_;
    }
  }
  void spin()
  {
    ros::Timer timer = nh_.createTimer(ros::Duration(1.0 / hz_), &SafetyLimiter::cbTimer, this);

    ros::spin();
  }
  double predict(const geometry_msgs::Twist &in,
                 const sensor_msgs::PointCloud2 &cloud_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_, *pc);

    try
    {
      tfl_.waitForTransform(frame_id_, cloud_.header.frame_id, cloud_.header.stamp,
                            ros::Duration(0.1));
      pcl_ros::transformPointCloud(frame_id_, *pc, *pc, tfl_);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("Safety Limit: Transform failed");
      geometry_msgs::Twist cmd_vel;
      return 0.0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> ds;
    ds.setInputCloud(pc);
    ds.setLeafSize(downsample_grid_, downsample_grid_, downsample_grid_);
    ds.filter(*pc_ds);
    *pc = *pc_ds;

    auto filter_z = [this](pcl::PointXYZ &p)
    {
      if (p.z < this->z_range_[0] || this->z_range_[1] < p.z)
        return true;
      p.z = 0.0;
      return false;
    };
    pc->erase(std::remove_if(pc->points.begin(), pc->points.end(), filter_z),
              pc->points.end());

    if (pc->width == 0)
    {
      ROS_WARN("Safety Limit: Empty pointcloud passed.");
      return DBL_MAX;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pc);

    Eigen::Affine3f move;
    Eigen::Affine3f move_inv;
    Eigen::Affine3f motion =
        Eigen::AngleAxisf(-twist_.angular.z * dt_, Eigen::Vector3f::UnitZ()) *
        Eigen::Translation3f(Eigen::Vector3f(-twist_.linear.x * dt_, 0.0, 0.0));
    Eigen::Affine3f motion_inv =
        Eigen::Translation3f(Eigen::Vector3f(twist_.linear.x * dt_, 0.0, 0.0)) *
        Eigen::AngleAxisf(twist_.angular.z * dt_, Eigen::Vector3f::UnitZ());
    move.setIdentity();
    move_inv.setIdentity();
    double t_col_estim = DBL_MAX;
    sensor_msgs::PointCloud col_points;
    sensor_msgs::PointCloud debug_points;
    col_points.header.frame_id = frame_id_;
    col_points.header.stamp = ros::Time::now();
    debug_points.header = col_points.header;

    for (float t = 0; t < tmax_; t += dt_)
    {
      move = move * motion;
      move_inv = move_inv * motion_inv;
      pcl::PointXYZ center;
      center = pcl::transformPoint(center, move_inv);
      geometry_msgs::Point32 p;
      p.x = center.x;
      p.y = center.y;
      p.z = 0.0;
      debug_points.points.push_back(p);

      std::vector<int> indices;
      std::vector<float> dist;
      int num = kdtree.radiusSearch(center, footprint_radius_, indices, dist);
      if (num == 0)
        continue;

      bool col = false;
      for (auto &i : indices)
      {
        auto &p = pc->points[i];
        auto point = pcl::transformPoint(p, move);
        vec v(point.x, point.y);
        if (footprint_p.inside(v))
        {
          geometry_msgs::Point32 pos;
          pos.x = p.x;
          pos.y = p.y;
          pos.z = p.z;
          col_points.points.push_back(pos);
          col = true;
          break;
        }
      }
      if (col)
      {
        t_col_estim = t;
        break;
      }
    }
    pub_debug_.publish(debug_points);
    pub_cloud_.publish(col_points);

    t_col_estim -= tmargin_;
    if (t_col_estim < 0)
      t_col_estim = 0;

    return t_col_estim;
  }

  geometry_msgs::Twist limit(const geometry_msgs::Twist &in)
  {
    auto out = in;
    const float lin_vel_lim = t_col_ * acc_[0];
    const float ang_vel_lim = t_col_ * acc_[1];

    bool col = false;
    if (fabs(out.linear.x) > lin_vel_lim)
    {
      float r = lin_vel_lim / fabs(out.linear.x);
      out.linear.x *= r;
      out.angular.z *= r;
      col = true;
    }
    if (fabs(out.angular.z) > ang_vel_lim)
    {
      const float r = ang_vel_lim / fabs(out.angular.z);
      out.linear.x *= r;
      out.angular.z *= r;
      col = true;
    }
    if (col)
    {
      ROS_WARN("Safety Limit: (%0.2f, %0.2f)->(%0.2f, %0.2f)",
               in.linear.x, in.angular.z,
               out.linear.x, out.angular.z);
    }
    return out;
  }

private:
  bool XmlRpc_isNumber(XmlRpc::XmlRpcValue &value)
  {
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
           value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
  }
  class vec
  {
  public:
    float c[2];
    vec(const float x, const float y)
    {
      c[0] = x;
      c[1] = y;
    }
    vec()
    {
      c[0] = c[1] = 0.0;
    }
    float &operator[](const int &i)
    {
      return c[i];
    }
    const float &operator[](const int &i) const
    {
      return c[i];
    }
    vec operator-(const vec &a) const
    {
      vec out = *this;
      out[0] -= a[0];
      out[1] -= a[1];
      return out;
    }
    float cross(const vec &a) const
    {
      return (*this)[0] * a[1] - (*this)[1] * a[0];
    }
    float dot(const vec &a) const
    {
      return (*this)[0] * a[0] + (*this)[1] * a[1];
    }
    float dist(const vec &a) const
    {
      return hypotf((*this)[0] - a[0], (*this)[1] - a[1]);
    }
    float dist_line(const vec &a, const vec &b) const
    {
      return (b - a).cross((*this) - a) / b.dist(a);
    }
    float dist_linestrip(const vec &a, const vec &b) const
    {
      if ((b - a).dot((*this) - a) <= 0)
        return this->dist(a);
      if ((a - b).dot((*this) - b) <= 0)
        return this->dist(b);
      return fabs(this->dist_line(a, b));
    }
  };
  class polygon
  {
  public:
    std::vector<vec> v;
    void move(const float &x, const float &y, const float &yaw)
    {
      float cos_v = cosf(yaw);
      float sin_v = sinf(yaw);
      for (auto &p : v)
      {
        auto tmp = p;
        p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
        p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
      }
    }
    bool inside(const vec &a)
    {
      int cn = 0;
      for (size_t i = 0; i < v.size() - 1; i++)
      {
        auto &v1 = v[i];
        auto &v2 = v[i + 1];
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
    float dist(const vec &a)
    {
      float dist = FLT_MAX;
      for (size_t i = 0; i < v.size() - 1; i++)
      {
        auto &v1 = v[i];
        auto &v2 = v[i + 1];
        auto d = a.dist_linestrip(v1, v2);
        if (d < dist)
          dist = d;
      }
      return dist;
    }
  };

  polygon footprint_p;

  void cbTwist(const geometry_msgs::Twist::Ptr &msg)
  {
    ros::Time now = ros::Time::now();

    twist_ = *msg;
    has_twist_ = true;

    if (now - last_disable_cmd_ < ros::Duration(disable_timeout_))
    {
      pub_twist_.publish(twist_);
    }
    else if (ros::Time::now() - cloud_.header.stamp > ros::Duration(timeout_))
    {
      if (has_cloud_)
        ROS_WARN("Safety Limit: PointCloud timed-out");
      geometry_msgs::Twist cmd_vel;
      pub_twist_.publish(cmd_vel);
      has_cloud_ = false;
    }
    else if (now <= hold_off_)
    {
      geometry_msgs::Twist cmd_vel = limit(twist_);
      pub_twist_.publish(cmd_vel);
    }
    else
    {
      pub_twist_.publish(twist_);
    }
  }
  void cbCloud(const sensor_msgs::PointCloud2::Ptr &msg)
  {
    cloud_ = *msg;
    has_cloud_ = true;
  }
  void cbDisable(const std_msgs::Bool::Ptr &msg)
  {
    if (msg->data)
    {
      last_disable_cmd_ = ros::Time::now();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "safety_limiter");

  SafetyLimiter limiter;
  limiter.spin();

  return 0;
}
