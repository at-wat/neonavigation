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
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <neonavigation_common/compatibility.h>
#include <trajectory_tracker_msgs/TrajectoryTrackerStatus.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>

#include <trajectory_tracker/basic_control.h>
#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/path2d.h>

class TrackerNode
{
public:
  TrackerNode();
  ~TrackerNode();
  void spin();

private:
  std::string topic_path_;
  std::string topic_cmd_vel_;
  std::string frame_robot_;
  std::string frame_odom_;
  double hz_;
  double look_forward_;
  double curv_forward_;
  double k_[3];
  double gain_at_vel_;
  double d_lim_;
  double d_stop_;
  double vel_[2];
  double acc_[2];
  double acc_toc_[2];
  trajectory_tracker::VelAccLimitter v_lim_;
  trajectory_tracker::VelAccLimitter w_lim_;
  double rotate_ang_;
  double goal_tolerance_dist_;
  double goal_tolerance_ang_;
  double stop_tolerance_dist_;
  double stop_tolerance_ang_;
  double no_pos_cntl_dist_;
  double min_track_path_;
  int path_step_;
  int path_step_done_;
  bool allow_backward_;
  bool limit_vel_by_avel_;
  bool check_old_path_;
  double epsilon_;
  bool in_place_turn_;

  ros::Subscriber sub_path_;
  ros::Subscriber sub_path_velocity_;
  ros::Subscriber sub_vel_;
  ros::Publisher pub_vel_;
  ros::Publisher pub_status_;
  ros::Publisher pub_tracking_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  trajectory_tracker::Path2D path_;
  std_msgs::Header path_header_;

  template <typename MSG_TYPE>
  void cbPath(const typename MSG_TYPE::ConstPtr&);
  void cbSpeed(const std_msgs::Float32::ConstPtr&);
  void cbTimer(const ros::TimerEvent&);
  void control();
};

TrackerNode::TrackerNode()
  : nh_()
  , pnh_("~")
  , tfl_(tfbuf_)
{
  neonavigation_common::compat::checkCompatMode();
  pnh_.param("frame_robot", frame_robot_, std::string("base_link"));
  pnh_.param("frame_odom", frame_odom_, std::string("odom"));
  neonavigation_common::compat::deprecatedParam(pnh_, "path", topic_path_, std::string("path"));
  neonavigation_common::compat::deprecatedParam(pnh_, "cmd_vel", topic_cmd_vel_, std::string("cmd_vel"));
  pnh_.param("hz", hz_, 50.0);
  pnh_.param("look_forward", look_forward_, 0.5);
  pnh_.param("curv_forward", curv_forward_, 0.5);
  pnh_.param("k_dist", k_[0], 1.0);
  pnh_.param("k_ang", k_[1], 1.0);
  pnh_.param("k_avel", k_[2], 1.0);
  pnh_.param("gain_at_vel", gain_at_vel_, 0.0);
  pnh_.param("dist_lim", d_lim_, 0.5);
  pnh_.param("dist_stop", d_stop_, 2.0);
  pnh_.param("rotate_ang", rotate_ang_, M_PI / 4);
  pnh_.param("max_vel", vel_[0], 0.5);
  pnh_.param("max_angvel", vel_[1], 1.0);
  pnh_.param("max_acc", acc_[0], 1.0);
  pnh_.param("max_angacc", acc_[1], 2.0);
  double acc_toc_factor[2];
  pnh_.param("acc_toc_factor", acc_toc_factor[0], 0.9);
  pnh_.param("angacc_toc_factor", acc_toc_factor[1], 0.9);
  acc_toc_[0] = acc_[0] * acc_toc_factor[0];
  acc_toc_[1] = acc_[1] * acc_toc_factor[1];
  pnh_.param("path_step", path_step_, 1);
  pnh_.param("goal_tolerance_dist", goal_tolerance_dist_, 0.2);
  pnh_.param("goal_tolerance_ang", goal_tolerance_ang_, 0.1);
  pnh_.param("stop_tolerance_dist", stop_tolerance_dist_, 0.1);
  pnh_.param("stop_tolerance_ang", stop_tolerance_ang_, 0.05);
  pnh_.param("no_position_control_dist", no_pos_cntl_dist_, 0.0);
  pnh_.param("min_tracking_path", min_track_path_, no_pos_cntl_dist_);
  pnh_.param("allow_backward", allow_backward_, true);
  pnh_.param("limit_vel_by_avel", limit_vel_by_avel_, false);
  pnh_.param("check_old_path", check_old_path_, false);
  pnh_.param("epsilon", epsilon_, 0.001);

  sub_path_ = neonavigation_common::compat::subscribe<nav_msgs::Path>(
      nh_, "path",
      pnh_, topic_path_, 2,
      boost::bind(&TrackerNode::cbPath<nav_msgs::Path>, this, _1));
  sub_path_velocity_ = nh_.subscribe<trajectory_tracker_msgs::PathWithVelocity>(
      "path_velocity", 2,
      boost::bind(&TrackerNode::cbPath<trajectory_tracker_msgs::PathWithVelocity>, this, _1));
  sub_vel_ = neonavigation_common::compat::subscribe(
      nh_, "speed",
      pnh_, "speed", 20, &TrackerNode::cbSpeed, this);
  pub_vel_ = neonavigation_common::compat::advertise<geometry_msgs::Twist>(
      nh_, "cmd_vel",
      pnh_, topic_cmd_vel_, 10);
  pub_status_ = pnh_.advertise<trajectory_tracker_msgs::TrajectoryTrackerStatus>("status", 10);
  pub_tracking_ = pnh_.advertise<geometry_msgs::PoseStamped>("tracking", 10);
}
TrackerNode::~TrackerNode()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub_vel_.publish(cmd_vel);
}

void TrackerNode::cbSpeed(const std_msgs::Float32::ConstPtr& msg)
{
  vel_[0] = msg->data;
}

namespace
{
float getVelocity(const geometry_msgs::PoseStamped& msg)
{
  return std::numeric_limits<float>::quiet_NaN();
}
float getVelocity(const trajectory_tracker_msgs::PoseStampedWithVelocity& msg)
{
  return msg.linear_velocity.x;
}
}  // namespace

template <typename MSG_TYPE>
void TrackerNode::cbPath(const typename MSG_TYPE::ConstPtr& msg)
{
  path_header_ = msg->header;
  path_.clear();
  path_step_done_ = 0;
  in_place_turn_ = false;
  if (msg->poses.size() == 0)
    return;

  auto j = msg->poses.begin();
  path_.push_back(trajectory_tracker::Pose2D(j->pose, getVelocity(*j)));
  for (auto j = msg->poses.begin(); j != msg->poses.end(); ++j)
  {
    const float velocity = getVelocity(*j);
    if (std::isfinite(velocity) && velocity < -0.0)
    {
      ROS_ERROR_THROTTLE(1.0, "path_velocity.velocity.x must be positive");
      path_.clear();
      return;
    }
    const trajectory_tracker::Pose2D next(j->pose, velocity);

    if ((path_.back().pos_ - next.pos_).squaredNorm() >= std::pow(epsilon_, 2))
      path_.push_back(next);
  }

  if (path_.size() == 1)
  {
    path_.back() = trajectory_tracker::Pose2D(
        msg->poses.back().pose, std::numeric_limits<float>::quiet_NaN());
    in_place_turn_ = true;
    while (path_.size() < 3)
    {
      // to make line direction computable, line should have a few points
      const float yaw = path_.back().yaw_;
      const trajectory_tracker::Pose2D next(
          path_.back().pos_ + Eigen::Vector2d(std::cos(yaw), std::sin(yaw)) * epsilon_,
          yaw, path_.back().velocity_);
      path_.push_back(next);
    }
  }
}
void TrackerNode::cbTimer(const ros::TimerEvent& event)
{
  control();
}

void TrackerNode::spin()
{
  ros::Timer timer = nh_.createTimer(ros::Duration(1.0 / hz_), &TrackerNode::cbTimer, this);

  ros::spin();
}

void TrackerNode::control()
{
  trajectory_tracker_msgs::TrajectoryTrackerStatus status;
  status.header.stamp = ros::Time::now();
  status.distance_remains = 0.0;
  status.angle_remains = 0.0;

  if (path_header_.frame_id.size() == 0 || path_.size() == 0)
  {
    v_lim_.clear();
    w_lim_.clear();
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_vel_.publish(cmd_vel);
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }
  // Transform
  trajectory_tracker::Path2D lpath;
  tf2::Stamped<tf2::Transform> transform;
  double transform_delay = 0;
  try
  {
    tf2::Stamped<tf2::Transform> trans_odom;
    tf2::fromMsg(
        tfbuf_.lookupTransform(frame_robot_, frame_odom_, ros::Time(0)), transform);
    tf2::fromMsg(
        tfbuf_.lookupTransform(frame_odom_, path_header_.frame_id, ros::Time(0)), trans_odom);
    transform *= trans_odom;
    transform_delay = (ros::Time::now() - transform.stamp_).toSec();
    if (std::abs(transform_delay) > 0.1 && check_old_path_)
    {
      ROS_ERROR_THROTTLE(
          1.0, "Timestamp of the transform is too old %f %f",
          ros::Time::now().toSec(), transform.stamp_.toSec());
    }

    const float trans_yaw = tf2::getYaw(transform.getRotation());
    const Eigen::Transform<double, 2, Eigen::TransformTraits::AffineCompact> trans =
        Eigen::Translation2d(
            Eigen::Vector2d(transform.getOrigin().x(), transform.getOrigin().y())) *
        Eigen::Rotation2Dd(trans_yaw);

    for (size_t i = 0; i < path_.size(); i += path_step_)
      lpath.push_back(
          trajectory_tracker::Pose2D(
              trans * path_[i].pos_, trans_yaw + path_[i].yaw_, path_[i].velocity_));
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN("TF exception: %s", e.what());
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }

  const float predicted_yaw = w_lim_.get() * look_forward_ / 2;
  const Eigen::Vector2d origin =
      Eigen::Vector2d(std::cos(predicted_yaw), std::sin(predicted_yaw)) * v_lim_.get() * look_forward_;

  const double path_length = lpath.length();

  // Find nearest line strip
  const trajectory_tracker::Path2D::ConstIterator it_local_goal =
      lpath.findLocalGoal(lpath.begin() + path_step_done_, lpath.end(), allow_backward_);

  const float max_search_range = (path_step_done_ > 0) ? 1.0 : 0.0;
  const trajectory_tracker::Path2D::ConstIterator it_nearest =
      lpath.findNearest(lpath.begin() + path_step_done_, it_local_goal, origin, max_search_range);

  if (it_nearest == lpath.end())
  {
    v_lim_.clear();
    w_lim_.clear();
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_vel_.publish(cmd_vel);
    // ROS_WARN("failed to find nearest node");
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }

  const int i_nearest = std::distance(
      static_cast<trajectory_tracker::Path2D::ConstIterator>(lpath.begin()), it_nearest);
  const int i_local_goal = std::distance(
      static_cast<trajectory_tracker::Path2D::ConstIterator>(lpath.begin()), it_local_goal);

  const Eigen::Vector2d pos_on_line =
      trajectory_tracker::projection2d(lpath[i_nearest - 1].pos_, lpath[i_nearest].pos_, origin);

  const float linear_vel =
      std::isnan(lpath[i_nearest].velocity_) ? vel_[0] : lpath[i_nearest].velocity_;

  // Remained distance to the local goal
  float remain_local = lpath.remainedDistance(lpath.begin(), it_nearest, it_local_goal, pos_on_line);
  // Remained distance to the final goal
  float remain = lpath.remainedDistance(lpath.begin(), it_nearest, lpath.end(), pos_on_line);
  if (path_length < no_pos_cntl_dist_)
    remain = remain_local = 0;

  // Signed distance error
  const float dist_err = trajectory_tracker::lineDistance(
      lpath[i_nearest - 1].pos_, lpath[i_nearest].pos_, origin);

  // Angular error
  const Eigen::Vector2d vec = lpath[i_nearest].pos_ - lpath[i_nearest - 1].pos_;
  float angle = -atan2(vec[1], vec[0]);
  const float angle_pose = allow_backward_ ? lpath[i_nearest].yaw_ : -angle;
  float sign_vel = 1.0;
  if (std::cos(-angle) * std::cos(angle_pose) + std::sin(-angle) * std::sin(angle_pose) < 0)
  {
    sign_vel = -1.0;
    angle = angle + M_PI;
  }
  angle = trajectory_tracker::angleNormalized(angle);

  // Curvature
  const float curv = lpath.getCurvature(it_nearest, it_local_goal, pos_on_line, curv_forward_);

  status.distance_remains = remain;
  status.angle_remains = angle;

  ROS_DEBUG(
      "trajectory_tracker: nearest: %d, local goal: %d, done: %d, goal: %lu, remain: %0.3f, remain_local: %0.3f",
      i_nearest, i_local_goal, path_step_done_, lpath.size(), remain, remain_local);

  bool arrive_local_goal(false);

  const float dt = 1.0 / hz_;
  // Stop and rotate
  if ((std::abs(rotate_ang_) < M_PI && std::cos(rotate_ang_) > std::cos(angle)) ||
      std::abs(remain_local) < stop_tolerance_dist_ ||
      path_length < min_track_path_ ||
      in_place_turn_)
  {
    if (path_length < min_track_path_ || std::abs(remain_local) < stop_tolerance_dist_)
    {
      angle = trajectory_tracker::angleNormalized(-(it_local_goal - 1)->yaw_);
      status.angle_remains = angle;
      if (it_local_goal != lpath.end())
        arrive_local_goal = true;
    }
    v_lim_.set(
        0.0,
        linear_vel, acc_[0], dt);
    w_lim_.set(
        trajectory_tracker::timeOptimalControl(angle + w_lim_.get() * dt * 1.5, acc_toc_[1]),
        vel_[1], acc_[1], dt);

    ROS_DEBUG(
        "trajectory_tracker: angular residual %0.3f, angular vel %0.3f, tf delay %0.3f",
        angle, w_lim_.get(), transform_delay);

    if (path_length < stop_tolerance_dist_ || in_place_turn_)
      status.distance_remains = remain = 0.0;
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
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      pub_vel_.publish(cmd_vel);
      // ROS_WARN("Far from given path");
      status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::FAR_FROM_PATH;
      pub_status_.publish(status);
      return;
    }

    // Path following control
    const float dist_err_clip = trajectory_tracker::clip(dist_err, d_lim_);

    v_lim_.set(
        trajectory_tracker::timeOptimalControl(-remain_local * sign_vel, acc_toc_[0]),
        linear_vel, acc_[0], dt);

    float wref = std::abs(v_lim_.get()) * curv;

    if (limit_vel_by_avel_ && std::abs(wref) > vel_[1])
    {
      v_lim_.set(
          std::copysign(1.0, v_lim_.get()) * std::abs(vel_[1] / curv),
          linear_vel, acc_[0], dt);
      wref = std::copysign(1.0, wref) * vel_[1];
    }

    const double k_ang = (gain_at_vel_ == 0.0) ? (k_[1]) : (k_[1] * linear_vel / gain_at_vel_);
    w_lim_.increment(
        dt * (-dist_err_clip * k_[0] - angle * k_ang - (w_lim_.get() - wref) * k_[2]),
        vel_[1], acc_[1], dt);

    ROS_DEBUG(
        "trajectory_tracker: distance residual %0.3f, angular residual %0.3f, ang vel residual %0.3f"
        ", v_lim: %0.3f, sign_vel: %0.0f, angle: %0.3f, yaw: %0.3f",
        dist_err_clip, angle, w_lim_.get() - wref, v_lim_.get(), sign_vel, angle, lpath[i_nearest].yaw_);
  }

  geometry_msgs::Twist cmd_vel;
  if (std::abs(status.distance_remains) < stop_tolerance_dist_ &&
      std::abs(status.angle_remains) < stop_tolerance_ang_)
  {
    v_lim_.clear();
    w_lim_.clear();
  }

  cmd_vel.linear.x = v_lim_.get();
  cmd_vel.angular.z = w_lim_.get();
  pub_vel_.publish(cmd_vel);
  status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::FOLLOWING;
  if (std::abs(status.distance_remains) < goal_tolerance_dist_ &&
      std::abs(status.angle_remains) < goal_tolerance_ang_)
  {
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL;
  }
  pub_status_.publish(status);
  geometry_msgs::PoseStamped tracking;
  tracking.header = status.header;
  tracking.header.frame_id = frame_robot_;
  tracking.pose.position.x = pos_on_line[0];
  tracking.pose.position.y = pos_on_line[1];
  tracking.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -angle));
  pub_tracking_.publish(tracking);

  path_step_done_ = i_nearest - 1;  // i_nearest is the next of the nearest node
  if (path_step_done_ < 0)
    path_step_done_ = 0;
  if (arrive_local_goal)
    path_step_done_ = i_local_goal + 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_tracker");

  TrackerNode track;
  track.spin();

  return 0;
}
