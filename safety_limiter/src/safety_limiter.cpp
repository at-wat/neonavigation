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
#include <cassert>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <safety_limiter_msgs/msg/safety_limiter_status.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <neonavigation_common/neonavigation_node.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

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
class SafetyLimiterNode : public neonavigation_common::NeonavigationNode
{
public:
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
      assert(i < 2);
      return c[i];
    }
    const float& operator[](const int& i) const
    {
      assert(i < 2);
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
        if ((v1[1] <= a[1] && a[1] < v2[1]) || (v2[1] <= a[1] && a[1] < v1[1]))
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

protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_cloud_;
  rclcpp::Publisher<safety_limiter_msgs::msg::SafetyLimiterStatus>::SharedPtr pub_status_;
  rclcpp::SubscriptionBase::SharedPtr sub_twist_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> sub_clouds_;
  rclcpp::SubscriptionBase::SharedPtr sub_disable_;
  rclcpp::SubscriptionBase::SharedPtr sub_watchdog_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr predict_timer_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  geometry_msgs::msg::Twist twist_;
  rclcpp::Time last_cloud_stamp_;
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

  rclcpp::Time last_disable_cmd_;
  double hold_d_;
  rclcpp::Duration hold_;
  rclcpp::Time hold_off_;
  double watchdog_interval_d_;
  std::string footprint_str_;
  bool allow_empty_cloud_;

  bool watchdog_stop_;
  bool has_cloud_;
  bool has_twist_;
  bool has_collision_at_now_;
  rclcpp::Time stuck_started_since_;

  constexpr static float EPSILON = 1e-6;

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;

  polygon footprint_p;

public:
  explicit SafetyLimiterNode(const std::string& node_name = "safety_limiter")
    : neonavigation_common::NeonavigationNode(node_name, rclcpp::NodeOptions())
    , tfbuf_(get_clock())
    , tfl_(tfbuf_)
    , last_cloud_stamp_(0, 0, get_clock()->get_clock_type())
    , cloud_accum_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_clear_(false)
    , last_disable_cmd_(0, 0, get_clock()->get_clock_type())
    , hold_(0, 0)
    , hold_off_(0, 0, get_clock()->get_clock_type())
    , watchdog_stop_(false)
    , has_cloud_(false)
    , has_twist_(true)
    , has_collision_at_now_(false)
    , stuck_started_since_(0, 0, get_clock()->get_clock_type())
  {
    declare_dynamic_parameter("freq", &hz_, 6.0);
    declare_dynamic_parameter("cloud_timeout", &timeout_, 0.8);
    declare_dynamic_parameter("disable_timeout", &disable_timeout_, 0.1);
    declare_dynamic_parameter("lin_vel", &vel_[0], 0.5);
    declare_dynamic_parameter("lin_acc", &acc_[0], 1.0);
    declare_dynamic_parameter("ang_vel", &vel_[1], 0.8);
    declare_dynamic_parameter("ang_acc", &acc_[1], 1.6);
    declare_dynamic_parameter("max_linear_vel", &max_values_[0], std::numeric_limits<double>::infinity());
    declare_dynamic_parameter("max_angular_vel", &max_values_[1], std::numeric_limits<double>::infinity());
    declare_dynamic_parameter("z_range_min", &z_range_[0], 0.0);
    declare_dynamic_parameter("z_range_max", &z_range_[1], 0.5);
    declare_dynamic_parameter("dt", &dt_, 0.1);
    declare_dynamic_parameter("d_margin", &d_margin_, 0.2);
    declare_dynamic_parameter("d_escape", &d_escape_, 0.05);
    declare_dynamic_parameter("yaw_margin", &yaw_margin_, 0.2);
    declare_dynamic_parameter("yaw_escape", &yaw_escape_, 0.05);
    declare_dynamic_parameter("downsample_grid", &downsample_grid_, 0.05);
    declare_dynamic_parameter("hold", &hold_d_, 0.0);
    declare_dynamic_parameter("allow_empty_cloud", &allow_empty_cloud_, false);
    declare_dynamic_parameter("base_frame", &base_frame_id_, std::string("base_link"));
    declare_dynamic_parameter("fixed_frame", &fixed_frame_id_, std::string("odom"));
    declare_dynamic_parameter("watchdog_interval", &watchdog_interval_d_, 0.0);
    declare_dynamic_parameter("footprint", &footprint_str_, std::string());
    if (footprint_str_.empty())
    {
      RCLCPP_FATAL(get_logger(), "Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified");
    }

    pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1).transient_local());
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud>("collision", rclcpp::QoS(1).transient_local());
    pub_status_ =
        create_publisher<safety_limiter_msgs::msg::SafetyLimiterStatus>("~/status", rclcpp::QoS(1).transient_local());
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_in", 1, std::bind(&SafetyLimiterNode::cbTwist, this, std::placeholders::_1));
    sub_disable_ = create_subscription<std_msgs::msg::Bool>(
        "disable_safety", 1, std::bind(&SafetyLimiterNode::cbDisable, this, std::placeholders::_1));
    sub_watchdog_ = create_subscription<std_msgs::msg::Empty>(
        "watchdog_reset", 1, std::bind(&SafetyLimiterNode::cbWatchdogReset, this, std::placeholders::_1));

    const int num_input_clouds = declare_parameter("num_input_clouds", 1);
    if (num_input_clouds == 1)
    {
      sub_clouds_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
          "cloud", 1, std::bind(&SafetyLimiterNode::cbCloud, this, std::placeholders::_1)));
    }
    else
    {
      for (int i = 0; i < num_input_clouds; ++i)
      {
        sub_clouds_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud" + std::to_string(i), 1, std::bind(&SafetyLimiterNode::cbCloud, this, std::placeholders::_1)));
      }
    }
    diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID("none");
    diag_updater_->add("Collision", this, &SafetyLimiterNode::diagnoseCollision);

    onDynamicParameterUpdated({});
  }

protected:
  void onDynamicParameterUpdated(const std::vector<rclcpp::Parameter>&) final
  {
    std::vector<geometry_msgs::msg::Point> footprint;
    nav2_costmap_2d::makeFootprintFromString(footprint_str_, footprint);
    footprint_p.v.clear();
    footprint_radius_ = 0;
    for (const auto& p : footprint)
    {
      vec v;
      v[0] = p.x;
      v[1] = p.y;
      footprint_p.v.push_back(v);

      const float dist = std::hypot(v[0], v[1]);
      if (dist > footprint_radius_)
        footprint_radius_ = dist;
    }
    footprint_p.v.push_back(footprint_p.v.front());
    hold_ = rclcpp::Duration::from_seconds(std::max(hold_d_, 1.0 / hz_));
    for (int i = 0; i < 2; i++)
    {
      auto t = vel_[i] / acc_[i];
      if (tmax_ < t)
        tmax_ = t;
    }
    tmax_ *= 1.5;
    tmax_ += std::max(d_margin_ / vel_[0], yaw_margin_ / vel_[1]);
    r_lim_ = 1.0;

    watchdog_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(watchdog_interval_d_),
                                           std::bind(&SafetyLimiterNode::cbWatchdogTimer, this));
    predict_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / hz_),
                                          std::bind(&SafetyLimiterNode::cbPredictTimer, this));
  }

  void cbWatchdogReset(const std_msgs::msg::Empty& msg)
  {
    watchdog_timer_->reset();
    watchdog_stop_ = false;
  }
  void cbWatchdogTimer()
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "safety_limiter: Watchdog timed-out");
    watchdog_stop_ = true;
    r_lim_ = 0;
    geometry_msgs::msg::Twist cmd_vel;
    pub_twist_->publish(cmd_vel);

    diag_updater_->force_update();
  }
  void cbPredictTimer()
  {
    if (!has_twist_)
      return;
    if (!has_cloud_)
      return;

    if (now() - last_cloud_stamp_ > rclcpp::Duration::from_seconds(timeout_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "safety_limiter: PointCloud timed-out");
      geometry_msgs::msg::Twist cmd_vel;
      pub_twist_->publish(cmd_vel);

      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      has_cloud_ = false;
      r_lim_ = 0;

      diag_updater_->force_update();
      return;
    }

    auto now = this->now();
    const double r_lim_current = predict(twist_);

    if (r_lim_current < r_lim_)
      r_lim_ = r_lim_current;

    if (r_lim_current < 1.0)
    {
      hold_off_ = now + hold_;
    }
    RCLCPP_DEBUG(get_logger(), "safety_limiter: r_lim = %0.3f, hold_off = %0.3f", r_lim_, hold_off_.seconds());
    cloud_clear_ = true;
    diag_updater_->force_update();
  }
  double predict(const geometry_msgs::msg::Twist& in)
  {
    if (cloud_accum_->size() == 0)
    {
      if (allow_empty_cloud_)
      {
        return 1.0;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "safety_limiter: Empty pointcloud passed.");
      return 0.0;
    }

    const rclcpp::Time pcl_stamp = pcl_conversions::fromPCL(cloud_accum_->header.stamp);
    const bool can_transform = tfbuf_.canTransform(base_frame_id_, cloud_accum_->header.frame_id, pcl_stamp);
    const rclcpp::Time stamp = can_transform ? pcl_stamp : rclcpp::Time(0);
    geometry_msgs::msg::TransformStamped fixed_to_base;
    try
    {
      fixed_to_base = tfbuf_.lookupTransform(base_frame_id_, cloud_accum_->header.frame_id, stamp);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "safety_limiter: Transform failed: %s", e.what());
      return 0.0;
    }

    const Eigen::Affine3f fixed_to_base_eigen =
        Eigen::Translation3f(fixed_to_base.transform.translation.x, fixed_to_base.transform.translation.y,
                             fixed_to_base.transform.translation.z) *
        Eigen::Quaternionf(fixed_to_base.transform.rotation.w, fixed_to_base.transform.rotation.x,
                           fixed_to_base.transform.rotation.y, fixed_to_base.transform.rotation.z);
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
    pc->erase(std::remove_if(pc->points.begin(), pc->points.end(), filter_z), pc->points.end());

    if (pc->size() == 0)
    {
      if (allow_empty_cloud_)
      {
        return 1.0;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "safety_limiter: Empty pointcloud passed.");
      return 0.0;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pc);

    Eigen::Affine3f move;
    Eigen::Affine3f move_inv;
    Eigen::Affine3f motion = Eigen::AngleAxisf(-twist_.angular.z * dt_, Eigen::Vector3f::UnitZ()) *
                             Eigen::Translation3f(Eigen::Vector3f(-twist_.linear.x * dt_, -twist_.linear.y * dt_, 0.0));
    Eigen::Affine3f motion_inv =
        Eigen::Translation3f(Eigen::Vector3f(twist_.linear.x * dt_, twist_.linear.y * dt_, 0.0)) *
        Eigen::AngleAxisf(twist_.angular.z * dt_, Eigen::Vector3f::UnitZ());
    move.setIdentity();
    move_inv.setIdentity();
    sensor_msgs::msg::PointCloud col_points;
    col_points.header.frame_id = base_frame_id_;
    col_points.header.stamp = now();

    float d_col = 0;
    float yaw_col = 0;
    bool has_collision = false;
    float d_escape_remain = 0;
    float yaw_escape_remain = 0;
    has_collision_at_now_ = false;
    const double linear_vel = std::hypot(twist_.linear.x, twist_.linear.y);

    for (float t = 0; t < tmax_; t += dt_)
    {
      if (t != 0)
      {
        d_col += linear_vel * dt_;
        d_escape_remain -= linear_vel * dt_;
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
          geometry_msgs::msg::Point32 pos;
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
        d_col -= linear_vel * dt_;
        yaw_col -= twist_.angular.z * dt_;
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
    pub_cloud_->publish(col_points);

    if (has_collision_at_now_)
    {
      if (stuck_started_since_.nanoseconds() == 0L)
        stuck_started_since_ = now();
    }
    else
    {
      if (stuck_started_since_.nanoseconds() != 0L)
        stuck_started_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    if (!has_collision)
      return 1.0;

    // delay compensation:
    //   solve for d_compensated: d_compensated = d - delay * sqrt(2 * acc * d_compensated)
    //     d_compensated = d + acc * delay^2 - sqrt((acc * delay^2)^2 + 2 * d * acc * delay^2)

    const float delay = 1.0 * (1.0 / hz_) + dt_;
    const float acc_dtsq[2] = {
        static_cast<float>(acc_[0] * std::pow(delay, 2)),
        static_cast<float>(acc_[1] * std::pow(delay, 2)),
    };

    d_col = std::max<float>(0.0, std::abs(d_col) - d_margin_ + acc_dtsq[0] -
                                     std::sqrt(std::pow(acc_dtsq[0], 2) + 2 * acc_dtsq[0] * std::abs(d_col)));
    yaw_col = std::max<float>(0.0, std::abs(yaw_col) - yaw_margin_ + acc_dtsq[1] -
                                       std::sqrt(std::pow(acc_dtsq[1], 2) + 2 * acc_dtsq[1] * std::abs(yaw_col)));

    float d_r = std::sqrt(std::abs(2 * acc_[0] * d_col)) / linear_vel;
    float yaw_r = std::sqrt(std::abs(2 * acc_[1] * yaw_col)) / std::abs(twist_.angular.z);
    if (!std::isfinite(d_r))
      d_r = 1.0;
    if (!std::isfinite(yaw_r))
      yaw_r = 1.0;

    return std::min(d_r, yaw_r);
  }

  geometry_msgs::msg::Twist limit(const geometry_msgs::msg::Twist& in)
  {
    auto out = in;
    if (r_lim_ < 1.0 - EPSILON)
    {
      out.linear.x *= r_lim_;
      out.linear.y *= r_lim_;
      out.angular.z *= r_lim_;

      if (std::abs(in.linear.x - out.linear.x) > EPSILON || std::abs(in.linear.y - out.linear.y) > EPSILON ||
          std::abs(in.angular.z - out.angular.z) > EPSILON)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "safety_limiter: (%0.2f, %0.2f, %0.2f)->(%0.2f, %0.2f, %0.2f)", in.linear.x, in.linear.y,
                             in.angular.z, out.linear.x, out.linear.y, out.angular.z);
      }
    }
    return out;
  }

  geometry_msgs::msg::Twist limitMaxVelocities(const geometry_msgs::msg::Twist& in)
  {
    auto out = in;
    if (max_values_[0] <= 0.0)
    {
      out.linear.x = 0;
      out.linear.y = 0;
    }
    else
    {
      const double out_linear_vel = std::hypot(out.linear.x, out.linear.y);
      if (out_linear_vel > max_values_[0])
      {
        const double vel_ratio = max_values_[0] / out_linear_vel;
        out.linear.x *= vel_ratio;
        out.linear.y *= vel_ratio;
      }
    }
    out.angular.z =
        (out.angular.z > 0) ? std::min(out.angular.z, max_values_[1]) : std::max(out.angular.z, -max_values_[1]);

    return out;
  }

  void cbTwist(const geometry_msgs::msg::Twist& msg)
  {
    const rclcpp::Time current_stamp = now();

    twist_ = msg;
    has_twist_ = true;

    if (current_stamp - last_disable_cmd_ < rclcpp::Duration::from_seconds(disable_timeout_))
    {
      pub_twist_->publish(limitMaxVelocities(twist_));
    }
    else if (!has_cloud_ || watchdog_stop_)
    {
      geometry_msgs::msg::Twist cmd_vel;
      pub_twist_->publish(cmd_vel);
    }
    else
    {
      geometry_msgs::msg::Twist cmd_vel = limitMaxVelocities(limit(twist_));
      pub_twist_->publish(cmd_vel);
      if (current_stamp > hold_off_)
      {
        r_lim_ = 1.0;
      }
    }
  }

  void cbCloud(const sensor_msgs::msg::PointCloud2& msg)
  {
    const bool can_transform = tfbuf_.canTransform(fixed_frame_id_, msg.header.frame_id, msg.header.stamp);
    const rclcpp::Time stamp = can_transform ? rclcpp::Time(msg.header.stamp) : rclcpp::Time(0);
    sensor_msgs::msg::PointCloud2 cloud_msg_fixed;
    try
    {
      const geometry_msgs::msg::TransformStamped cloud_to_fixed =
          tfbuf_.lookupTransform(fixed_frame_id_, msg.header.frame_id, stamp);
      tf2::doTransform(msg, cloud_msg_fixed, cloud_to_fixed);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "safety_limiter: Transform failed: %s", e.what());
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
    last_cloud_stamp_ = msg.header.stamp;
    has_cloud_ = true;
  }
  void cbDisable(const std_msgs::msg::Bool& msg)
  {
    if (msg.data)
    {
      last_disable_cmd_ = now();
    }
  }

  void diagnoseCollision(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    safety_limiter_msgs::msg::SafetyLimiterStatus status_msg;

    if (!has_cloud_ || watchdog_stop_)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Stopped due to data timeout.");
    }
    else if (r_lim_ == 1.0)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    }
    else if (r_lim_ < EPSILON)
    {
      const auto msg = has_collision_at_now_ ? "Cannot escape from collision." :
                                               "Trying to avoid collision, but cannot move anymore..";
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg);
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   (has_collision_at_now_) ? "Escaping from collision." : "Reducing velocity to avoid collision.");
    }
    stat.addf("Velocity Limit Ratio", "%.2f", r_lim_);
    stat.add("Pointcloud Availability", has_cloud_ ? "true" : "false");
    stat.add("Watchdog Timeout", watchdog_stop_ ? "true" : "false");

    status_msg.limit_ratio = r_lim_;
    status_msg.is_cloud_available = has_cloud_;
    status_msg.has_watchdog_timed_out = watchdog_stop_;
    status_msg.stuck_started_since = stuck_started_since_;

    pub_status_->publish(status_msg);
  }
};

}  // namespace safety_limiter

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_limiter::SafetyLimiterNode>("trajectory_tracker"));
  rclcpp::shutdown();
  return 0;
}
