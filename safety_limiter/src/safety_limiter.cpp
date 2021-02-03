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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <safety_limiter_msgs/SafetyLimiterStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <neonavigation_common/compatibility.h>

#include <safety_limiter/SafetyLimiterConfig.h>

namespace safety_limiter
{
pcl::PointXYZ operator-(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
  auto c = a;
  c.x -= b.x;
  c.y -= b.y;
  c.z -= b.z;
  return c;
}
pcl::PointXYZ operator+(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
  auto c = a;
  c.x += b.x;
  c.y += b.y;
  c.z += b.z;
  return c;
}
pcl::PointXYZ operator*(const pcl::PointXYZ& a, const float& b)
{
  auto c = a;
  c.x *= b;
  c.y *= b;
  c.z *= b;
  return c;
}
bool XmlRpc_isNumber(XmlRpc::XmlRpcValue& value)
{
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
         value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
}

class SafetyLimiterNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_status_;
  ros::Subscriber sub_twist_;
  std::vector<ros::Subscriber> sub_clouds_;
  ros::Subscriber sub_disable_;
  ros::Subscriber sub_watchdog_;
  ros::Timer watchdog_timer_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  boost::recursive_mutex parameter_server_mutex_;
  std::unique_ptr<dynamic_reconfigure::Server<SafetyLimiterConfig>> parameter_server_;

  geometry_msgs::Twist twist_;
  ros::Time last_cloud_stamp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accum_;
  bool cloud_clear_;
  double hz_;
  double timeout_;
  double disable_timeout_;
  double vel_[2];
  double acc_[2];
  double tmax_;
  double dt_;
  double d_margin_;
  double d_escape_;
  double yaw_margin_;
  double yaw_escape_;
  double r_lim_;
  double max_values_[2];
  double z_range_[2];
  float footprint_radius_;
  double downsample_grid_;
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  ros::Time last_disable_cmd_;
  ros::Duration hold_;
  ros::Time hold_off_;
  ros::Duration watchdog_interval_;
  bool allow_empty_cloud_;

  bool watchdog_stop_;
  bool has_cloud_;
  bool has_twist_;
  bool has_collision_at_now_;
  ros::Time stuck_started_since_;

  constexpr static float EPSILON = 1e-6;

  diagnostic_updater::Updater diag_updater_;

public:
  SafetyLimiterNode()
    : nh_()
    , pnh_("~")
    , tfl_(tfbuf_)
    , cloud_accum_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_clear_(false)
    , last_disable_cmd_(0)
    , watchdog_stop_(false)
    , has_cloud_(false)
    , has_twist_(true)
    , has_collision_at_now_(false)
    , stuck_started_since_(ros::Time(0))
  {
    neonavigation_common::compat::checkCompatMode();
    pub_twist_ = neonavigation_common::compat::advertise<geometry_msgs::Twist>(
        nh_, "cmd_vel",
        pnh_, "cmd_vel_out", 1, true);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud>("collision", 1, true);
    pub_status_ = pnh_.advertise<safety_limiter_msgs::SafetyLimiterStatus>("status", 1, true);
    sub_twist_ = neonavigation_common::compat::subscribe(
        nh_, "cmd_vel_in",
        pnh_, "cmd_vel_in", 1, &SafetyLimiterNode::cbTwist, this);
    sub_disable_ = neonavigation_common::compat::subscribe(
        nh_, "disable_safety",
        pnh_, "disable", 1, &SafetyLimiterNode::cbDisable, this);
    sub_watchdog_ = neonavigation_common::compat::subscribe(
        nh_, "watchdog_reset",
        pnh_, "watchdog_reset", 1, &SafetyLimiterNode::cbWatchdogReset, this);

    int num_input_clouds;
    pnh_.param("num_input_clouds", num_input_clouds, 1);
    if (num_input_clouds == 1)
    {
      sub_clouds_.push_back(neonavigation_common::compat::subscribe(
          nh_, "cloud",
          pnh_, "cloud", 1, &SafetyLimiterNode::cbCloud, this));
    }
    else
    {
      for (int i = 0; i < num_input_clouds; ++i)
      {
        sub_clouds_.push_back(nh_.subscribe(
            "cloud" + std::to_string(i), 1, &SafetyLimiterNode::cbCloud, this));
      }
    }

    if (pnh_.hasParam("t_margin"))
      ROS_WARN("safety_limiter: t_margin parameter is obsolated. Use d_margin and yaw_margin instead.");
    pnh_.param("base_frame", base_frame_id_, std::string("base_link"));
    pnh_.param("fixed_frame", fixed_frame_id_, std::string("odom"));
    double watchdog_interval_d;
    pnh_.param("watchdog_interval", watchdog_interval_d, 0.0);
    watchdog_interval_ = ros::Duration(watchdog_interval_d);
    pnh_.param("max_linear_vel", max_values_[0], std::numeric_limits<double>::infinity());
    pnh_.param("max_angular_vel", max_values_[1], std::numeric_limits<double>::infinity());

    parameter_server_.reset(
        new dynamic_reconfigure::Server<SafetyLimiterConfig>(parameter_server_mutex_, pnh_));
    parameter_server_->setCallback(boost::bind(&SafetyLimiterNode::cbParameter, this, _1, _2));

    XmlRpc::XmlRpcValue footprint_xml;
    if (!pnh_.hasParam("footprint"))
    {
      ROS_FATAL("Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified");
    }
    pnh_.getParam("footprint", footprint_xml);
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

      const float dist = std::hypot(v[0], v[1]);
      if (dist > footprint_radius_)
        footprint_radius_ = dist;
    }
    footprint_p.v.push_back(footprint_p.v.front());
    ROS_INFO("footprint radius: %0.3f", footprint_radius_);

    diag_updater_.setHardwareID("none");
    diag_updater_.add("Collision", this, &SafetyLimiterNode::diagnoseCollision);
  }
  void spin()
  {
    ros::Timer predict_timer =
        nh_.createTimer(ros::Duration(1.0 / hz_), &SafetyLimiterNode::cbPredictTimer, this);

    if (watchdog_interval_ != ros::Duration(0.0))
    {
      watchdog_timer_ =
          nh_.createTimer(watchdog_interval_, &SafetyLimiterNode::cbWatchdogTimer, this);
    }

    ros::spin();
  }

protected:
  void cbWatchdogReset(const std_msgs::Empty::ConstPtr& msg)
  {
    watchdog_timer_.setPeriod(watchdog_interval_, true);
    watchdog_stop_ = false;
  }
  void cbWatchdogTimer(const ros::TimerEvent& event)
  {
    ROS_WARN_THROTTLE(1.0, "safety_limiter: Watchdog timed-out");
    watchdog_stop_ = true;
    r_lim_ = 0;
    geometry_msgs::Twist cmd_vel;
    pub_twist_.publish(cmd_vel);

    diag_updater_.force_update();
  }
  void cbPredictTimer(const ros::TimerEvent& event)
  {
    if (!has_twist_)
      return;
    if (!has_cloud_)
      return;

    if (ros::Time::now() - last_cloud_stamp_ > ros::Duration(timeout_))
    {
      ROS_WARN_THROTTLE(1.0, "safety_limiter: PointCloud timed-out");
      geometry_msgs::Twist cmd_vel;
      pub_twist_.publish(cmd_vel);

      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      has_cloud_ = false;
      r_lim_ = 0;

      diag_updater_.force_update();
      return;
    }

    ros::Time now = ros::Time::now();
    const double r_lim_current = predict(twist_);

    if (r_lim_current < r_lim_)
      r_lim_ = r_lim_current;

    if (r_lim_current < 1.0)
      hold_off_ = now + hold_;

    cloud_clear_ = true;

    diag_updater_.force_update();
  }
  void cbParameter(const SafetyLimiterConfig& config, const uint32_t /* level */)
  {
    boost::recursive_mutex::scoped_lock lock(parameter_server_mutex_);
    hz_ = config.freq;
    timeout_ = config.cloud_timeout;
    disable_timeout_ = config.disable_timeout;
    vel_[0] = config.lin_vel;
    acc_[0] = config.lin_acc;
    vel_[1] = config.ang_vel;
    acc_[1] = config.ang_acc;
    max_values_[0] = config.max_linear_vel;
    max_values_[1] = config.max_angular_vel;
    z_range_[0] = config.z_range_min;
    z_range_[1] = config.z_range_max;
    dt_ = config.dt;
    d_margin_ = config.d_margin;
    d_escape_ = config.d_escape;
    yaw_margin_ = config.yaw_margin;
    yaw_escape_ = config.yaw_escape;
    downsample_grid_ = config.downsample_grid;
    hold_ = ros::Duration(std::max(config.hold, 1.0 / hz_));
    allow_empty_cloud_ = config.allow_empty_cloud;

    tmax_ = 0.0;
    for (int i = 0; i < 2; i++)
    {
      auto t = vel_[i] / acc_[i];
      if (tmax_ < t)
        tmax_ = t;
    }
    tmax_ *= 1.5;
    tmax_ += std::max(d_margin_ / vel_[0], yaw_margin_ / vel_[1]);
    r_lim_ = 1.0;
  }
  double predict(const geometry_msgs::Twist& in)
  {
    if (cloud_accum_->size() == 0)
    {
      if (allow_empty_cloud_)
      {
        return 1.0;
      }
      ROS_WARN_THROTTLE(1.0, "safety_limiter: Empty pointcloud passed.");
      return 0.0;
    }

    const bool can_transform = tfbuf_.canTransform(
        base_frame_id_, cloud_accum_->header.frame_id,
        pcl_conversions::fromPCL(cloud_accum_->header.stamp));
    const ros::Time stamp =
        can_transform ? pcl_conversions::fromPCL(cloud_accum_->header.stamp) : ros::Time(0);

    geometry_msgs::TransformStamped fixed_to_base;
    try
    {
      fixed_to_base = tfbuf_.lookupTransform(
          base_frame_id_, cloud_accum_->header.frame_id, stamp);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(1.0, "safety_limiter: Transform failed: %s", e.what());
      return 0.0;
    }

    const Eigen::Affine3f fixed_to_base_eigen =
        Eigen::Translation3f(
            fixed_to_base.transform.translation.x,
            fixed_to_base.transform.translation.y,
            fixed_to_base.transform.translation.z) *
        Eigen::Quaternionf(
            fixed_to_base.transform.rotation.w,
            fixed_to_base.transform.rotation.x,
            fixed_to_base.transform.rotation.y,
            fixed_to_base.transform.rotation.z);
    pcl::transformPointCloud(*cloud_accum_, *cloud_accum_, fixed_to_base_eigen);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> ds;
    ds.setInputCloud(cloud_accum_);
    ds.setLeafSize(downsample_grid_, downsample_grid_, downsample_grid_);
    ds.filter(*pc);

    auto filter_z = [this](pcl::PointXYZ& p)
    {
      if (p.z < this->z_range_[0] || this->z_range_[1] < p.z)
        return true;
      p.z = 0.0;
      return false;
    };
    pc->erase(std::remove_if(pc->points.begin(), pc->points.end(), filter_z),
              pc->points.end());

    if (pc->size() == 0)
    {
      if (allow_empty_cloud_)
      {
        return 1.0;
      }
      ROS_WARN_THROTTLE(1.0, "safety_limiter: Empty pointcloud passed.");
      return 0.0;
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
    sensor_msgs::PointCloud col_points;
    col_points.header.frame_id = base_frame_id_;
    col_points.header.stamp = ros::Time::now();

    float d_col = 0;
    float yaw_col = 0;
    bool has_collision = false;
    float d_escape_remain = 0;
    float yaw_escape_remain = 0;
    has_collision_at_now_ = false;

    for (float t = 0; t < tmax_; t += dt_)
    {
      if (t != 0)
      {
        d_col += twist_.linear.x * dt_;
        d_escape_remain -= std::abs(twist_.linear.x) * dt_;
        yaw_col += twist_.angular.z * dt_;
        yaw_escape_remain -= std::abs(twist_.angular.z) * dt_;
        move = move * motion;
        move_inv = move_inv * motion_inv;
      }

      pcl::PointXYZ center;
      center = pcl::transformPoint(center, move_inv);

      std::vector<int> indices;
      std::vector<float> dist;
      const int num = kdtree.radiusSearch(center, footprint_radius_, indices, dist);
      if (num == 0)
        continue;

      bool colliding = false;
      for (auto& i : indices)
      {
        auto& p = pc->points[i];
        auto point = pcl::transformPoint(p, move);
        vec v(point.x, point.y);
        if (footprint_p.inside(v))
        {
          geometry_msgs::Point32 pos;
          pos.x = p.x;
          pos.y = p.y;
          pos.z = p.z;
          col_points.points.push_back(pos);
          colliding = true;
          break;
        }
      }
      if (colliding)
      {
        if (t == 0)
        {
          // The robot is already in collision.
          // Allow movement under d_escape_ and yaw_escape_
          d_escape_remain = d_escape_;
          yaw_escape_remain = yaw_escape_;
          has_collision_at_now_ = true;
        }
        if (d_escape_remain <= 0 || yaw_escape_remain <= 0)
        {
          if (has_collision_at_now_)
          {
            // It's not possible to escape from collision; stop completely.
            d_col = yaw_col = 0;
          }

          has_collision = true;
          break;
        }
      }
    }
    pub_cloud_.publish(col_points);

    if (has_collision_at_now_)
    {
      if (stuck_started_since_ == ros::Time(0))
        stuck_started_since_ = ros::Time::now();
    }
    else
    {
      if (stuck_started_since_ != ros::Time(0))
        stuck_started_since_ = ros::Time(0);
    }

    if (!has_collision)
      return 1.0;

    // delay compensation:
    //   solve for d_compensated: d_compensated = d - delay * sqrt(2 * acc * d_compensated)
    //     d_compensated = d + acc * delay^2 - sqrt((acc * delay^2)^2 + 2 * d * acc * delay^2)

    const float delay = 1.0 * (1.0 / hz_) + dt_;
    const float acc_dtsq[2] =
        {
            static_cast<float>(acc_[0] * std::pow(delay, 2)),
            static_cast<float>(acc_[1] * std::pow(delay, 2)),
        };

    d_col = std::max<float>(
        0.0,
        std::abs(d_col) - d_margin_ + acc_dtsq[0] -
            std::sqrt(std::pow(acc_dtsq[0], 2) + 2 * acc_dtsq[0] * std::abs(d_col)));
    yaw_col = std::max<float>(
        0.0,
        std::abs(yaw_col) - yaw_margin_ + acc_dtsq[1] -
            std::sqrt(std::pow(acc_dtsq[1], 2) + 2 * acc_dtsq[1] * std::abs(yaw_col)));

    float d_r =
        std::sqrt(std::abs(2 * acc_[0] * d_col)) / std::abs(twist_.linear.x);
    float yaw_r =
        std::sqrt(std::abs(2 * acc_[1] * yaw_col)) / std::abs(twist_.angular.z);
    if (!std::isfinite(d_r))
      d_r = 1.0;
    if (!std::isfinite(yaw_r))
      yaw_r = 1.0;

    return std::min(d_r, yaw_r);
  }

  geometry_msgs::Twist
  limit(const geometry_msgs::Twist& in)
  {
    auto out = in;
    if (r_lim_ < 1.0 - EPSILON)
    {
      out.linear.x *= r_lim_;
      out.angular.z *= r_lim_;
      if (std::abs(in.linear.x - out.linear.x) > EPSILON ||
          std::abs(in.angular.z - out.angular.z) > EPSILON)
      {
        ROS_WARN_THROTTLE(
            1.0, "safety_limiter: (%0.2f, %0.2f)->(%0.2f, %0.2f)",
            in.linear.x, in.angular.z,
            out.linear.x, out.angular.z);
      }
    }
    return out;
  }

  geometry_msgs::Twist
  limitMaxVelocities(const geometry_msgs::Twist& in)
  {
    auto out = in;
    out.linear.x =  (out.linear.x > 0) ?
                    std::min(out.linear.x, max_values_[0]) :
                    std::max(out.linear.x, -max_values_[0]);

    out.angular.z = (out.angular.z > 0) ?
                    std::min(out.angular.z, max_values_[1]) :
                    std::max(out.angular.z, -max_values_[1]);

    return out;
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
    vec operator-(const vec& a) const
    {
      vec out = *this;
      out[0] -= a[0];
      out[1] -= a[1];
      return out;
    }
    float cross(const vec& a) const
    {
      return (*this)[0] * a[1] - (*this)[1] * a[0];
    }
    float dot(const vec& a) const
    {
      return (*this)[0] * a[0] + (*this)[1] * a[1];
    }
    float dist(const vec& a) const
    {
      return std::hypot((*this)[0] - a[0], (*this)[1] - a[1]);
    }
    float dist_line(const vec& a, const vec& b) const
    {
      return (b - a).cross((*this) - a) / b.dist(a);
    }
    float dist_linestrip(const vec& a, const vec& b) const
    {
      if ((b - a).dot((*this) - a) <= 0)
        return this->dist(a);
      if ((a - b).dot((*this) - b) <= 0)
        return this->dist(b);
      return std::abs(this->dist_line(a, b));
    }
  };
  class polygon
  {
  public:
    std::vector<vec> v;
    void move(const float& x, const float& y, const float& yaw)
    {
      const float cos_v = cosf(yaw);
      const float sin_v = sinf(yaw);
      for (auto& p : v)
      {
        const auto tmp = p;
        p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
        p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
      }
    }
    bool inside(const vec& a) const
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
    float dist(const vec& a) const
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

  polygon footprint_p;

  void cbTwist(const geometry_msgs::Twist::ConstPtr& msg)
  {
    ros::Time now = ros::Time::now();

    twist_ = *msg;
    has_twist_ = true;

    if (now - last_disable_cmd_ < ros::Duration(disable_timeout_))
    {
      pub_twist_.publish(limitMaxVelocities(twist_));
    }
    else if (!has_cloud_ || watchdog_stop_)
    {
      geometry_msgs::Twist cmd_vel;
      pub_twist_.publish(cmd_vel);
    }
    else
    {
      geometry_msgs::Twist cmd_vel = limitMaxVelocities(limit(twist_));
      pub_twist_.publish(cmd_vel);

      if (now > hold_off_)
        r_lim_ = 1.0;
    }
  }

  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    const bool can_transform = tfbuf_.canTransform(
        fixed_frame_id_, msg->header.frame_id, msg->header.stamp);
    const ros::Time stamp =
        can_transform ? msg->header.stamp : ros::Time(0);

    sensor_msgs::PointCloud2 cloud_msg_fixed;
    try
    {
      const geometry_msgs::TransformStamped cloud_to_fixed =
          tfbuf_.lookupTransform(fixed_frame_id_, msg->header.frame_id, stamp);
      tf2::doTransform(*msg, cloud_msg_fixed, cloud_to_fixed);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(1.0, "safety_limiter: Transform failed: %s", e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fixed(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_fixed->header.frame_id = fixed_frame_id_;
    pcl::fromROSMsg(cloud_msg_fixed, *cloud_fixed);

    if (cloud_clear_)
    {
      cloud_clear_ = false;
      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    *cloud_accum_ += *cloud_fixed;
    cloud_accum_->header.frame_id = fixed_frame_id_;
    last_cloud_stamp_ = msg->header.stamp;
    has_cloud_ = true;
  }
  void cbDisable(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg->data)
    {
      last_disable_cmd_ = ros::Time::now();
    }
  }

  void diagnoseCollision(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    safety_limiter_msgs::SafetyLimiterStatus status_msg;

    if (!has_cloud_ || watchdog_stop_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Stopped due to data timeout.");
    }
    else if (r_lim_ == 1.0)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
    else if (r_lim_ < EPSILON)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   (has_collision_at_now_) ?
                       "Cannot escape from collision." :
                       "Trying to avoid collision, but cannot move anymore.");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   (has_collision_at_now_) ?
                       "Escaping from collision." :
                       "Reducing velocity to avoid collision.");
    }
    stat.addf("Velocity Limit Ratio", "%.2f", r_lim_);
    stat.add("Pointcloud Availability", has_cloud_ ? "true" : "false");
    stat.add("Watchdog Timeout", watchdog_stop_ ? "true" : "false");

    status_msg.limit_ratio = r_lim_;
    status_msg.is_cloud_available = has_cloud_;
    status_msg.has_watchdog_timed_out = watchdog_stop_;
    status_msg.stuck_started_since = stuck_started_since_;

    pub_status_.publish(status_msg);
  }
};

}  // namespace safety_limiter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_limiter");

  safety_limiter::SafetyLimiterNode limiter;
  limiter.spin();

  return 0;
}
