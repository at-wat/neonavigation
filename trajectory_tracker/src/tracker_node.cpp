/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
   Original idea of the implemented control scheme was published on:
   S. Iida, S. Yuta, "Vehicle command system and trajectory control for
   autonomous mobile robots," in Proceedings of the 1991 IEEE/RSJ
   International Workshop on Intelligent Robots and Systems (IROS),
   1991, pp. 212-217.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>
#include <trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp>
#include <neonavigation_common/neonavigation_node.hpp>

#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/path2d.h>
#include <trajectory_tracker/basic_control.h>

#include <trajectory_tracker/tracker_node.hpp>

namespace trajectory_tracker
{

TrackerNode::TrackerNode(const std::string& name, const rclcpp::NodeOptions& options)
  : neonavigation_common::NeonavigationNode(name, options)
  , tfbuf_(get_clock())
  , tfl_(tfbuf_)
  , is_path_updated_(false)
{
}

void TrackerNode::initialize()
{
  declare_dynamic_parameter("look_forward", &look_forward_, 0.5);
  declare_dynamic_parameter("curv_forward", &curv_forward_, 0.5);
  declare_dynamic_parameter("k_dist", &k_[0], 1.0);
  declare_dynamic_parameter("k_ang", &k_[1], 1.0);
  declare_dynamic_parameter("k_avel", &k_[2], 1.0);
  declare_dynamic_parameter("gain_at_vel", &gain_at_vel_, 0.0);
  declare_dynamic_parameter("dist_lim", &d_lim_, 0.5);
  declare_dynamic_parameter("dist_stop", &d_stop_, 2.0);
  declare_dynamic_parameter("rotate_ang", &rotate_ang_, 0.78539816339);
  declare_dynamic_parameter("max_vel", &vel_[0], 0.5);
  declare_dynamic_parameter("max_angvel", &vel_[1], 1.0);
  declare_dynamic_parameter("max_acc", &acc_[0], 1.0);
  declare_dynamic_parameter("max_angacc", &acc_[1], 2.0);
  declare_dynamic_parameter("acc_toc_factor", &acc_toc_factor_, 0.9);
  declare_dynamic_parameter("angacc_toc_factor", &angacc_toc_factor_, 0.9);
  declare_dynamic_parameter("path_step", &path_step_, 1l);
  declare_dynamic_parameter("goal_tolerance_dist", &goal_tolerance_dist_, 0.2);
  declare_dynamic_parameter("goal_tolerance_ang", &goal_tolerance_ang_, 0.1);
  declare_dynamic_parameter("stop_tolerance_dist", &stop_tolerance_dist_, 0.1);
  declare_dynamic_parameter("stop_tolerance_ang", &stop_tolerance_ang_, 0.05);
  declare_dynamic_parameter("no_position_control_dist", &no_pos_cntl_dist_, 0.0);
  declare_dynamic_parameter("min_tracking_path", &min_track_path_, 0.0);
  declare_dynamic_parameter("allow_backward", &allow_backward_, true);
  declare_dynamic_parameter("limit_vel_by_avel", &limit_vel_by_avel_, false);
  declare_dynamic_parameter("check_old_path", &check_old_path_, false);
  declare_dynamic_parameter("epsilon", &epsilon_, 0.001);
  declare_dynamic_parameter("use_time_optimal_control", &use_time_optimal_control_, true);
  declare_dynamic_parameter("time_optimal_control_future_gain", &time_optimal_control_future_gain_, 1.5);
  declare_dynamic_parameter("k_ang_rotation", &k_ang_rotation_, 1.0);
  declare_dynamic_parameter("k_avel_rotation", &k_avel_rotation_, 1.0);
  declare_dynamic_parameter("goal_tolerance_lin_vel", &goal_tolerance_lin_vel_, 0.0);
  declare_dynamic_parameter("goal_tolerance_ang_vel", &goal_tolerance_ang_vel_, 0.0);
  declare_dynamic_parameter("frame_robot", &frame_robot_, std::string("base_link"));
  declare_dynamic_parameter("frame_odom", &frame_odom_, std::string("odom"));
  declare_dynamic_parameter("hz", &hz_, 50.0);
  declare_dynamic_parameter("predict_odom", &predict_odom_, true);
  declare_dynamic_parameter("max_dt", &max_dt_, 0.1);
  declare_dynamic_parameter("odom_timeout_sec", &odom_timeout_sec_, 0.1);
  onDynamicParameterUpdated({});

  if (declare_parameter("use_action_server", false))
  {
    RCLCPP_WARN(get_logger(), "Action server is enabled.");
    action_server_ =
        std::make_unique<ActionServer>(shared_from_this(), "follow_path", std::bind(&TrackerNode::computeControl, this),
                                       nullptr, std::chrono::milliseconds(500), false);
    RCLCPP_WARN(get_logger(), "Action server done.");

    action_server_->activate();
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TrackerNode::cbOdometry, this, std::placeholders::_1));
  }
  else
  {
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        "path", 2, std::bind(&TrackerNode::cbPath<nav_msgs::msg::Path>, this, std::placeholders::_1));
    sub_path_velocity_ = create_subscription<trajectory_tracker_msgs::msg::PathWithVelocity>(
        "path_velocity", 2,
        std::bind(&TrackerNode::cbPath<trajectory_tracker_msgs::msg::PathWithVelocity>, this, std::placeholders::_1));
    if (declare_parameter("use_odom", false))
    {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&TrackerNode::cbOdometry, this, std::placeholders::_1));
    }
    else
    {
      timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / hz_),
                                    std::bind(&TrackerNode::cbTimer, this));
    }
  }
  sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      "speed", 20, std::bind(&TrackerNode::cbSpeed, this, std::placeholders::_1));

  pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pub_status_ = create_publisher<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>("~/status", 10);
  pub_tracking_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/tracking", 10);
}

TrackerNode::~TrackerNode()
{
  if (action_server_)
    action_server_->deactivate();
  publishZeroVelocity();
}

void TrackerNode::onDynamicParameterUpdated(const std::vector<rclcpp::Parameter>&)
{
  acc_toc_[0] = acc_[0] * acc_toc_factor_;
  acc_toc_[1] = acc_[1] * angacc_toc_factor_;
}

void TrackerNode::cbSpeed(const std_msgs::msg::Float32& msg)
{
  vel_[0] = msg.data;
}

template <typename MSG_TYPE>
void TrackerNode::cbPath(const MSG_TYPE& msg)
{
  std::lock_guard<std::recursive_mutex> lock(action_server_mutex_);
  path_header_ = msg.header;
  is_path_updated_ = true;
  path_step_done_ = 0;
  path_.fromMsg(msg, epsilon_);
  for (const auto& path_pose : path_)
  {
    if (std::isfinite(path_pose.velocity_) && path_pose.velocity_ < -0.0)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "path_velocity.velocity.x must be positive");
      path_.clear();
      return;
    }
  }
}

void TrackerNode::cbOdometry(const nav_msgs::msg::Odometry& odom)
{
  if (odom.header.frame_id != frame_odom_)
  {
    RCLCPP_WARN(get_logger(), "frame_odom is invalid. Update from \"%s\" to \"%s\"", frame_odom_.c_str(),
                odom.header.frame_id.c_str());
    frame_odom_ = odom.header.frame_id;
  }
  if (odom.child_frame_id != frame_robot_)
  {
    RCLCPP_WARN(get_logger(), "frame_robot is invalid. Update from \"%s\" to \"%s\"", frame_robot_.c_str(),
                odom.child_frame_id.c_str());
    frame_robot_ = odom.child_frame_id;
  }
  if (odom_timeout_sec_ != 0.0)
  {
    if (odom_timeout_timer_)
    {
      odom_timeout_timer_->cancel();
    }
    odom_timeout_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(odom_timeout_sec_),
                                               std::bind(&TrackerNode::cbOdomTimeout, this));
  }

  std::lock_guard<std::recursive_mutex> lock(action_server_mutex_);
  if ((prev_odom_stamp_.nanoseconds() != 0L) && (!action_server_ || (action_server_->is_running())))
  {
    const rclcpp::Time odom_stamp(odom.header.stamp);
    const double dt = std::min(max_dt_, (odom_stamp - prev_odom_stamp_).seconds());
    nav_msgs::msg::Odometry odom_compensated = odom;
    Eigen::Vector3d prediction_offset(0, 0, 0);
    if (predict_odom_)
    {
      const double predict_dt = std::max(0.0, std::min(max_dt_, (now() - odom_stamp).seconds()));
      tf2::Transform trans;
      const tf2::Quaternion rotation(tf2::Vector3(0, 0, 1), odom.twist.twist.angular.z * predict_dt);
      const tf2::Vector3 translation(odom.twist.twist.linear.x * predict_dt, 0, 0);

      prediction_offset[0] = odom.twist.twist.linear.x * predict_dt;
      prediction_offset[2] = odom.twist.twist.angular.z * predict_dt;

      tf2::fromMsg(odom.pose.pose, trans);
      trans.setOrigin(trans.getOrigin() + tf2::Transform(trans.getRotation()) * translation);
      trans.setRotation(trans.getRotation() * rotation);
      tf2::toMsg(trans, odom_compensated.pose.pose);
    }

    tf2::Transform odom_to_robot;
    tf2::fromMsg(odom_compensated.pose.pose, odom_to_robot);
    const tf2::Stamped<tf2::Transform> robot_to_odom(odom_to_robot.inverse(), tf2_ros::fromMsg(odom_stamp),
                                                     odom.header.frame_id);

    control(robot_to_odom, prediction_offset, odom.twist.twist.linear.x, odom.twist.twist.angular.z, dt);
  }
  prev_odom_stamp_ = odom.header.stamp;
}

void TrackerNode::cbTimer()
{
  try
  {
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(tfbuf_.lookupTransform(frame_robot_, frame_odom_, rclcpp::Time(0)), transform);
    control(transform, Eigen::Vector3d(0, 0, 0), 0, 0, 1.0 / hz_);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF exception: %s", e.what());
    trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
    status.header.stamp = now();
    status.distance_remains = 0.0;
    status.angle_remains = 0.0;
    status.path_header = path_header_;
    status.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
    pub_status_->publish(status);
    return;
  }
}

void TrackerNode::cbOdomTimeout()
{
  std::lock_guard<std::recursive_mutex> lock(action_server_mutex_);
  RCLCPP_WARN_STREAM(get_logger(), "Odometry timeout. Last odometry stamp: " << prev_odom_stamp_.seconds());
  v_lim_.clear();
  w_lim_.clear();
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  pub_vel_->publish(cmd_vel);

  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
  status.header.stamp = now();
  status.distance_remains = 0.0;
  status.angle_remains = 0.0;
  status.path_header = path_header_;
  status.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
  pub_status_->publish(status);

  // One shot timer
  odom_timeout_timer_->cancel();
  odom_timeout_timer_.reset();
}

void TrackerNode::control(const tf2::Stamped<tf2::Transform>& robot_to_odom, const Eigen::Vector3d& prediction_offset,
                          const double odom_linear_vel, const double odom_angular_vel, const double dt)
{
  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
  status.header.stamp = now();
  status.path_header = path_header_;
  if (is_path_updated_)
  {
    // Call getTrackingResult to update path_step_done_.
    const TrackingResult initial_tracking_result =
        getTrackingResult(robot_to_odom, prediction_offset, odom_linear_vel, odom_angular_vel);
    path_step_done_ = initial_tracking_result.path_step_done;
    is_path_updated_ = false;
  }
  const TrackingResult tracking_result =
      getTrackingResult(robot_to_odom, prediction_offset, odom_linear_vel, odom_angular_vel);
  switch (tracking_result.status)
  {
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH:
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH:
    {
      v_lim_.clear();
      w_lim_.clear();
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      pub_vel_->publish(cmd_vel);
      break;
    }
    default:
    {
      if (tracking_result.turning_in_place)
      {
        v_lim_.set(0.0, tracking_result.target_linear_vel, acc_[0], dt);

        if (use_time_optimal_control_)
        {
          const double expected_angle_remains =
              tracking_result.angle_remains + w_lim_.get() * dt * time_optimal_control_future_gain_;
          w_lim_.set(trajectory_tracker::timeOptimalControl(expected_angle_remains, acc_toc_[1]), vel_[1], acc_[1], dt);
        }
        else
        {
          const double wvel_increment =
              (-tracking_result.angle_remains * k_ang_rotation_ - w_lim_.get() * k_avel_rotation_) * dt;
          w_lim_.increment(wvel_increment, vel_[1], acc_[1], dt);
        }
        RCLCPP_DEBUG(get_logger(), "angular residual %0.3f, angular vel %0.3f", tracking_result.angle_remains,
                     w_lim_.get());
      }
      else
      {
        v_lim_.set(trajectory_tracker::timeOptimalControl(tracking_result.signed_local_distance, acc_toc_[0]),
                   tracking_result.target_linear_vel, acc_[0], dt);

        float wref = std::abs(v_lim_.get()) * tracking_result.tracking_point_curv;

        if (limit_vel_by_avel_ && std::abs(wref) > vel_[1])
        {
          v_lim_.set(std::copysign(1.0, v_lim_.get()) * std::abs(vel_[1] / tracking_result.tracking_point_curv),
                     tracking_result.target_linear_vel, acc_[0], dt);
          wref = std::copysign(1.0, wref) * vel_[1];
        }

        const double k_ang =
            (gain_at_vel_ == 0.0) ? (k_[1]) : (k_[1] * tracking_result.target_linear_vel / gain_at_vel_);
        const double dist_diff = tracking_result.distance_from_target;
        const double angle_diff = tracking_result.angle_remains;
        const double wvel_diff = w_lim_.get() - wref;
        w_lim_.increment(dt * (-dist_diff * k_[0] - angle_diff * k_ang - wvel_diff * k_[2]), vel_[1], acc_[1], dt);

        RCLCPP_DEBUG(get_logger(),
                     "distance residual %0.3f, angular residual %0.3f, ang vel residual %0.3f"
                     ", v_lim %0.3f, w_lim %0.3f signed_local_distance %0.3f, k_ang %0.3f",
                     dist_diff, angle_diff, wvel_diff, v_lim_.get(), w_lim_.get(),
                     tracking_result.signed_local_distance, k_ang);
      }
      if (std::abs(tracking_result.distance_remains) < stop_tolerance_dist_ &&
          std::abs(tracking_result.angle_remains) < stop_tolerance_ang_ &&
          std::abs(tracking_result.distance_remains_raw) < stop_tolerance_dist_ &&
          std::abs(tracking_result.angle_remains_raw) < stop_tolerance_ang_)
      {
        v_lim_.clear();
        w_lim_.clear();
      }
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = v_lim_.get();
      cmd_vel.angular.z = w_lim_.get();
      pub_vel_->publish(cmd_vel);
      path_step_done_ = tracking_result.path_step_done;
      break;
    }
  }
  status.status = tracking_result.status;
  status.distance_remains = tracking_result.distance_remains;
  status.angle_remains = tracking_result.angle_remains;
  pub_status_->publish(status);
  latest_status_ = status;
}

TrackerNode::TrackingResult TrackerNode::getTrackingResult(const tf2::Stamped<tf2::Transform>& robot_to_odom,
                                                           const Eigen::Vector3d& prediction_offset,
                                                           const double odom_linear_vel,
                                                           const double odom_angular_vel) const
{
  if (path_header_.frame_id.size() == 0 || path_.size() == 0)
  {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }
  // Transform
  trajectory_tracker::Path2D lpath;
  double transform_delay = 0;
  tf2::Stamped<tf2::Transform> transform = robot_to_odom;
  try
  {
    tf2::Stamped<tf2::Transform> odom_to_path;
    tf2::fromMsg(tfbuf_.lookupTransform(frame_odom_, path_header_.frame_id, tf2::TimePointZero), odom_to_path);

    transform *= odom_to_path;
    transform_delay = tf2::durationToSec(tf2_ros::fromRclcpp(now()) - transform.stamp_);
    if (std::abs(transform_delay) > 0.1 && check_old_path_)
    {
      const double seconds1 = now().seconds();
      const double seconds2 = tf2::timeToSec(transform.stamp_);
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "Timestamp of the transform is too old %f %f", seconds1,
                            seconds2);
    }
    const float trans_yaw = tf2::getYaw(transform.getRotation());
    const Eigen::Transform<double, 2, Eigen::TransformTraits::AffineCompact> trans =
        Eigen::Translation2d(Eigen::Vector2d(transform.getOrigin().x(), transform.getOrigin().y())) *
        Eigen::Rotation2Dd(trans_yaw);

    for (size_t i = 0; i < path_.size(); i += path_step_)
      lpath.push_back(trajectory_tracker::Pose2D(trans * path_[i].pos_, trans_yaw + path_[i].yaw_, path_[i].velocity_));
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(get_logger(), "TF exception: %s", e.what());
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const Eigen::Vector2d origin_raw = prediction_offset.head<2>();
  const float yaw_raw = prediction_offset[2];

  const float yaw_predicted = w_lim_.get() * look_forward_ / 2;
  const Eigen::Vector2d origin =
      Eigen::Vector2d(std::cos(yaw_predicted), std::sin(yaw_predicted)) * v_lim_.get() * look_forward_;

  const double path_length = lpath.length();

  // Find nearest line strip
  const trajectory_tracker::Path2D::ConstIterator it_local_goal =
      lpath.findLocalGoal(lpath.cbegin() + path_step_done_, lpath.cend(), allow_backward_);

  const float max_search_range = (path_step_done_ > 0) ? 1.0 : 0.0;
  const trajectory_tracker::Path2D::ConstIterator it_nearest =
      lpath.findNearest(lpath.cbegin() + path_step_done_, it_local_goal, origin, max_search_range, epsilon_);

  if (it_nearest == lpath.end())
  {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const int64_t i_nearest = std::distance(lpath.cbegin(), it_nearest);
  const int64_t i_nearest_prev = std::max(0l, i_nearest - 1);
  const int64_t i_local_goal = std::distance(lpath.cbegin(), it_local_goal);

  const Eigen::Vector2d pos_on_line =
      trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);
  const Eigen::Vector2d pos_on_line_raw =
      trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin_raw);

  const float linear_vel = std::isnan(lpath[i_nearest].velocity_) ? vel_[0] : lpath[i_nearest].velocity_;

  // Remained distance to the local goal
  float remain_local = lpath.remainedDistance(lpath.cbegin(), it_nearest, it_local_goal, pos_on_line);
  // Remained distance to the final goal
  float distance_remains = lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line);
  float distance_remains_raw = lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line_raw);
  if (path_length < no_pos_cntl_dist_)
    distance_remains = distance_remains_raw = remain_local = 0;

  // Signed distance error
  const float dist_err = trajectory_tracker::lineDistance(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);

  // Angular error
  const Eigen::Vector2d vec = lpath[i_nearest].pos_ - lpath[i_nearest_prev].pos_;
  float angle_remains = -atan2(vec[1], vec[0]);
  const float angle_pose = allow_backward_ ? lpath[i_nearest].yaw_ : -angle_remains;
  float sign_vel = 1.0;
  if (std::cos(-angle_remains) * std::cos(angle_pose) + std::sin(-angle_remains) * std::sin(angle_pose) < 0)
  {
    sign_vel = -1.0;
    angle_remains = angle_remains + M_PI;
  }
  angle_remains = trajectory_tracker::angleNormalized(angle_remains);

  // Curvature
  const float curv = lpath.getCurvature(it_nearest, it_local_goal, pos_on_line, curv_forward_);

  RCLCPP_DEBUG(get_logger(), "nearest: %ld, local goal: %ld, done: %ld, goal: %lu, remain: %0.3f, remain_local: %0.3f",
               i_nearest, i_local_goal, path_step_done_, lpath.size(), distance_remains, remain_local);

  bool arrive_local_goal(false);
  bool in_place_turning = (vec[1] == 0.0 && vec[0] == 0.0);

  TrackingResult result(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING);

  // Stop and rotate
  const bool large_angle_error = std::abs(rotate_ang_) < M_PI && std::cos(rotate_ang_) > std::cos(angle_remains);
  if (large_angle_error || std::abs(remain_local) < stop_tolerance_dist_ || path_length < min_track_path_ ||
      in_place_turning)
  {
    if (large_angle_error)
    {
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_DEBUG_THROTTLE(get_logger(), clock, 1000, "Stop and rotate due to large angular error: %0.3f",
                            angle_remains);
    }

    if (path_length < min_track_path_ || std::abs(remain_local) < stop_tolerance_dist_ || in_place_turning)
    {
      angle_remains = trajectory_tracker::angleNormalized(-(it_local_goal - 1)->yaw_);
      if (it_local_goal != lpath.end())
        arrive_local_goal = true;
    }
    if (path_length < stop_tolerance_dist_ || in_place_turning)
      distance_remains = distance_remains_raw = 0.0;

    result.turning_in_place = true;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
  }
  else
  {
    // Too far from given path
    float dist_from_path = dist_err;
    if (i_nearest == 0)
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    else if (i_nearest + 1 >= static_cast<int>(path_.size()))
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    if (std::abs(dist_from_path) > d_stop_)
    {
      result.distance_remains = distance_remains;
      result.distance_remains_raw = distance_remains_raw;
      result.angle_remains = angle_remains;
      result.angle_remains_raw = angle_remains + yaw_raw;
      result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH;
      return result;
    }

    // Path following control
    result.turning_in_place = false;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
    result.angle_remains_raw = angle_remains + yaw_raw;
    result.distance_from_target = trajectory_tracker::clip(dist_err, d_lim_);
    result.signed_local_distance = -remain_local * sign_vel;
    result.tracking_point_curv = curv;
    result.tracking_point_x = pos_on_line[0];
    result.tracking_point_y = pos_on_line[1];
  }

  if (std::abs(result.distance_remains) < goal_tolerance_dist_ &&
      std::abs(result.angle_remains) < goal_tolerance_ang_ &&
      std::abs(result.distance_remains_raw) < goal_tolerance_dist_ &&
      std::abs(result.angle_remains_raw) < goal_tolerance_ang_ &&
      (goal_tolerance_lin_vel_ == 0.0 || std::abs(odom_linear_vel) < goal_tolerance_lin_vel_) &&
      (goal_tolerance_ang_vel_ == 0.0 || std::abs(odom_angular_vel) < goal_tolerance_ang_vel_) &&
      it_local_goal == lpath.end())
  {
    result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL;
  }

  if (arrive_local_goal)
    result.path_step_done = i_local_goal;
  else
    result.path_step_done = std::max(path_step_done_, i_nearest - 1);

  return result;
}

void TrackerNode::publishZeroVelocity()
{
  std::lock_guard<std::recursive_mutex> lock(action_server_mutex_);
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub_vel_->publish(cmd_vel);
}

void TrackerNode::resetLatestStatus()
{
  std::lock_guard<std::recursive_mutex> lock(action_server_mutex_);
  latest_status_.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
  latest_status_.distance_remains = 0.0;
  latest_status_.angle_remains = 0.0;
  latest_status_.path_header = std_msgs::msg::Header();
}

static std::string to_string(const trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status)
{
  switch (status.status)
  {
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH:
      return "NO_PATH";
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH:
      return "FAR_FROM_PATH";
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING:
      return "FOLLOWING";
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL:
      return "GOAL";
    default:
      return "UNKNOWN";
  }
}

void TrackerNode::computeControl()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try
  {
    resetLatestStatus();
    cbPath(action_server_->get_current_goal()->path);

    rclcpp::WallRate loop_rate(hz_);
    while (rclcpp::ok())
    {
      if (action_server_ == nullptr || !action_server_->is_server_active())
      {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested())
      {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      if (action_server_->is_preempt_requested())
      {
        RCLCPP_INFO(get_logger(), "Passing new path to controller.");
        auto goal = action_server_->accept_pending_goal();
        cbPath(goal->path);
      }

      {
        std::lock_guard<std::recursive_mutex> lock(action_server_mutex_);
        if (latest_status_.status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
        {
          RCLCPP_INFO(get_logger(), "Reached the goal!");
          break;
        }
        else
        {
          RCLCPP_DEBUG(get_logger(), "Status: %s, remaining distance: %0.3f, remaining angle: %0.3f",
                       to_string(latest_status_).c_str(), latest_status_.distance_remains,
                       latest_status_.angle_remains);
        }
      }
      loop_rate.sleep();
    }
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();
  action_server_->succeeded_current();
}

}  // namespace trajectory_tracker
