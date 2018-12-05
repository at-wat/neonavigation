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

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <neonavigation_common/compatibility.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/TrajectoryTrackerStatus.h>

#include <trajectory_tracker/average.h>
#include <trajectory_tracker/basic_control.h>
#include <trajectory_tracker/eigen_line.h>

namespace
{
struct PoseWithVelocity
{
  PoseWithVelocity(const Eigen::Vector2d& p, float y, float v)
    : pos(p)
    , yaw(y)
    , velocity(v)
  {
  }
  PoseWithVelocity(const geometry_msgs::Pose& pose, float v)
    : pos(Eigen::Vector2d(pose.position.x, pose.position.y))
    , yaw(tf2::getYaw(pose.orientation))
    , velocity(v)
  {
  }
  Eigen::Vector2d pos;
  float yaw;
  float velocity;
};
using PoseWithVelocityArray = std::vector<PoseWithVelocity>;
}  // namespace

class TrackerNode
{
public:
  TrackerNode();
  ~TrackerNode();
  void spin();

private:
  std::string topic_path_;
  std::string topic_cmd_vel_;
  std::string topic_odom_;
  std::string frame_robot_;
  std::string frame_odom_;
  double hz_;
  double look_forward_;
  double curv_forward_;
  double k_[3];
  double d_lim_;
  double d_stop_;
  double vel_[2];
  double acc_[2];
  trajectory_tracker::VelAccLimitter v_ref_;
  trajectory_tracker::VelAccLimitter w_ref_;
  double dec_;
  double rotate_ang_;
  double ang_factor_;
  double sw_dist_;
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

  ros::Subscriber sub_path_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_vel_;
  ros::Publisher pub_vel_;
  ros::Publisher pub_status_;
  ros::Publisher pub_tracking_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  PoseWithVelocityArray path_;
  std_msgs::Header path_header_;
  nav_msgs::Odometry odom_;

  void cbPath(const nav_msgs::Path::ConstPtr&);
  void cbOdom(const nav_msgs::Odometry::ConstPtr&);
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
  neonavigation_common::compat::deprecatedParam(pnh_, "odom", topic_odom_, std::string("odom"));
  neonavigation_common::compat::deprecatedParam(pnh_, "cmd_vel", topic_cmd_vel_, std::string("cmd_vel"));
  pnh_.param("hz", hz_, 50.0);
  pnh_.param("look_forward", look_forward_, 0.5);
  pnh_.param("curv_forward", curv_forward_, 0.5);
  pnh_.param("k_dist", k_[0], 1.0);
  pnh_.param("k_ang", k_[1], 1.0);
  pnh_.param("k_avel", k_[2], 1.0);
  pnh_.param("k_dcel", dec_, 0.2);
  pnh_.param("dist_lim", d_lim_, 0.5);
  pnh_.param("dist_stop", d_stop_, 2.0);
  pnh_.param("rotate_ang", rotate_ang_, M_PI / 4);
  pnh_.param("max_vel", vel_[0], 0.5);
  pnh_.param("max_angvel", vel_[1], 1.0);
  pnh_.param("max_acc", acc_[0], 1.0);
  pnh_.param("max_angacc", acc_[1], 2.0);
  pnh_.param("path_step", path_step_, 1);
  pnh_.param("distance_angle_factor", ang_factor_, 0.0);
  pnh_.param("switchback_dist", sw_dist_, 0.3);
  pnh_.param("goal_tolerance_dist", goal_tolerance_dist_, 0.2);
  pnh_.param("goal_tolerance_ang", goal_tolerance_ang_, 0.1);
  pnh_.param("stop_tolerance_dist", stop_tolerance_dist_, 0.1);
  pnh_.param("stop_tolerance_ang", stop_tolerance_ang_, 0.05);
  pnh_.param("no_position_control_dist", no_pos_cntl_dist_, 0.0);
  pnh_.param("min_tracking_path", min_track_path_, no_pos_cntl_dist_);
  pnh_.param("allow_backward", allow_backward_, true);
  pnh_.param("limit_vel_by_avel", limit_vel_by_avel_, false);
  pnh_.param("check_old_path", check_old_path_, false);

  sub_path_ = neonavigation_common::compat::subscribe(
      nh_, "path",
      pnh_, topic_path_, 2, &TrackerNode::cbPath, this);
  sub_odom_ = neonavigation_common::compat::subscribe(
      nh_, "odom",
      pnh_, topic_odom_, 20, &TrackerNode::cbOdom, this);
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
void TrackerNode::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_ = *msg;
}
void TrackerNode::cbPath(const nav_msgs::Path::ConstPtr& msg)
{
  path_header_ = msg->header;
  path_.clear();
  path_step_done_ = 0;
  if (msg->poses.size() == 0)
    return;

  auto j = msg->poses.begin();
  path_.push_back(PoseWithVelocity(j->pose, 0));
  for (auto j = msg->poses.begin(); j != msg->poses.end(); ++j)
  {
    const PoseWithVelocity next(j->pose, 0);
    if ((path_.back().pos - next.pos).squaredNorm() >= std::pow(0.01, 2))
    {
      path_.push_back(next);
    }
  }

  while (path_.size() < 3 && path_.size() > 0)
  {
    // to make line direction computable, line should have a few points
    const float yaw = path_.back().yaw;
    const PoseWithVelocity next(
        path_.back().pos + Eigen::Vector2d(std::cos(yaw), std::sin(yaw)) * 0.001, yaw, 0);
    path_.push_back(next);
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

  if (path_header_.frame_id.size() == 0)
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_vel_.publish(cmd_vel);
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }
  // Transform
  PoseWithVelocityArray lpath;
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
    {
      lpath.push_back(
          PoseWithVelocity(trans * path_[i].pos, trans_yaw + path_[i].yaw, vel_[0]));
    }
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN("TF exception: %s", e.what());
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }

  float min_dist_ = 10000.0;
  int iclose = -1;
  const Eigen::Vector2d origin =
      Eigen::Vector2d(std::cos(w_ref_.get()), std::sin(w_ref_.get())) *
      (look_forward_ / 2.0) * v_ref_.get() * look_forward_;

  // Find nearest line strip
  float distance_path_ = 0;
  for (size_t i = 1; i < lpath.size(); i++)
    distance_path_ += (lpath[i - 1].pos - lpath[i].pos).norm();

  float distance_path_search_ = 0;
  float sign_vel_prev_ = 0;
  for (size_t i = path_step_done_; i < lpath.size(); i++)
  {
    if (i < 1)
      continue;
    distance_path_search_ += (lpath[i - 1].pos - lpath[i].pos).norm();
    if ((origin - lpath[i].pos).squaredNorm() < std::pow(0.05f, 2) && i < lpath.size() - 1)
      continue;
    const float d = trajectory_tracker::lineStripDistance(
        lpath[i - 1].pos, lpath[i].pos, origin);
    if (std::abs(d) <= std::abs(min_dist_))
    {
      min_dist_ = d;
      iclose = i;
    }
    if (path_step_done_ > 0 && distance_path_search_ > 1.0)
      break;

    const Eigen::Vector2d vec = lpath[i].pos - lpath[i - 1].pos;
    const float angle = atan2(vec[1], vec[0]);
    const float angle_pose = allow_backward_ ? lpath[i].yaw : angle;
    const float sign_vel_req = std::cos(angle) * std::cos(angle_pose) + std::sin(angle) * std::sin(angle_pose);
    if (sign_vel_prev_ * sign_vel_req < 0)
    {
      // Stop read forward if the path_ switched back
      break;
    }
    sign_vel_prev_ = sign_vel_req;
  }
  if (iclose < 0)
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_vel_.publish(cmd_vel);
    // ROS_WARN("failed to find nearest node");
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }
  // Signed distance error
  float dist = trajectory_tracker::lineDistance(lpath[iclose - 1].pos, lpath[iclose].pos, origin);
  float dist_err_compensated = dist;
  if (iclose == 0)
    dist_err_compensated = -(lpath[iclose].pos - origin).norm();
  else if (iclose + 1 >= static_cast<int>(path_.size()))
    dist_err_compensated = -(lpath[iclose].pos - origin).norm();

  // Angular error
  const Eigen::Vector2d vec = lpath[iclose].pos - lpath[iclose - 1].pos;
  float angle = -atan2(vec[1], vec[0]);
  const float angle_pose = allow_backward_ ? lpath[iclose].yaw : (-angle);
  float sign_vel_ = 1.0;
  if (std::cos(-angle) * std::cos(angle_pose) + std::sin(-angle) * std::sin(angle_pose) < 0)
  {
    sign_vel_ = -1.0;
    angle = angle + M_PI;
    if (angle > M_PI)
      angle -= 2.0 * M_PI;
  }
  // Curvature
  trajectory_tracker::Average<float> curv;
  const Eigen::Vector2d pos_line =
      trajectory_tracker::projection2d(lpath[iclose - 1].pos, lpath[iclose].pos, origin);
  int local_goal = lpath.size() - 1;
  float remain_local_ = (pos_line - lpath[iclose].pos).norm();
  for (int i = iclose - 1; i < static_cast<int>(lpath.size()) - 1; i++)
  {
    if (i > 2)
    {
      const Eigen::Vector2d vec = lpath[i].pos - lpath[i - 1].pos;
      const float angle = atan2(vec[1], vec[0]);
      const float angle_pose = allow_backward_ ? lpath[i + 1].yaw : angle;
      const float sign_vel_req = std::cos(angle) * std::cos(angle_pose) + std::sin(angle) * std::sin(angle_pose);
      if (sign_vel_ * sign_vel_req < 0)
      {
        // Stop read forward if the path_ switched back
        local_goal = i;
        break;
      }
      if (i > iclose)
        remain_local_ += (lpath[i - 1].pos - lpath[i].pos).norm();
    }
  }
  for (int i = iclose - 1; i < local_goal; i++)
  {
    if (i > 2)
      curv += trajectory_tracker::curv3p(lpath[i - 2].pos, lpath[i - 1].pos, lpath[i].pos);

    if ((lpath[i].pos - lpath[local_goal].pos).squaredNorm() < std::pow(0.05f, 2))
      break;
    if ((lpath[i].pos - pos_line).norm() > curv_forward_)
      break;
  }
  float remain = (origin - lpath.back().pos).norm();
  if (min_dist_ < 0 && iclose == local_goal)
  {
    remain = -remain;
    remain_local_ = -remain_local_;
  }
  if (distance_path_ < no_pos_cntl_dist_)
    remain = remain_local_ = 0;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  while (angle > M_PI)
    angle -= 2.0 * M_PI;

  status.distance_remains = remain;
  status.angle_remains = angle;

  const float dt = 1.0 / hz_;
  // Stop and rotate
  if ((std::abs(rotate_ang_) < M_PI && std::cos(rotate_ang_) > std::cos(angle)) ||
      std::abs(remain_local_) < stop_tolerance_dist_ ||
      distance_path_ < min_track_path_)
  {
    if (distance_path_ < min_track_path_ || std::abs(remain_local_) < stop_tolerance_dist_)
    {
      angle = -lpath.back().yaw;
      status.angle_remains = angle;
    }
    v_ref_.set(
        0.0,
        vel_[0], acc_[0], dt);
    w_ref_.set(
        trajectory_tracker::timeoptimalControl(angle + w_ref_.get() * dt * 1.5, acc_[1], dt),
        vel_[1], acc_[1], dt);

    ROS_DEBUG("trajectory_tracker: angular residual %0.3f, angular vel %0.3f, tf delay %0.3f",
              angle, w_ref_, transform_delay);

    if (distance_path_ < stop_tolerance_dist_)
      status.distance_remains = remain = remain_local_ = 0.0;
  }
  else
  {
    // Control
    if (dist < -d_lim_)
      dist = -d_lim_;
    else if (dist > d_lim_)
      dist = d_lim_;

    v_ref_.set(
        trajectory_tracker::timeoptimalControl(-remain_local_ * sign_vel_, acc_[0], dt),
        vel_[0], acc_[0], dt);

    const float wref = std::abs(v_ref_.get()) * curv;

    if (limit_vel_by_avel_ && std::abs(wref) > vel_[1])
      v_ref_.set(
          std::copysign(1.0, v_ref_.get()) * std::abs(vel_[1] / curv),
          vel_[0], acc_[0], dt);

    w_ref_.increment(
        dt * (-dist * k_[0] - angle * k_[1] - (w_ref_.get() - wref) * k_[2]),
        vel_[1], acc_[1], dt);

    // Too far from given path
    if (std::abs(dist_err_compensated) > d_stop_)
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
  }

  geometry_msgs::Twist cmd_vel;
  if (std::abs(status.distance_remains) < stop_tolerance_dist_ &&
      std::abs(status.angle_remains) < stop_tolerance_ang_)
  {
    v_ref_.clear();
    w_ref_.clear();
  }

  cmd_vel.linear.x = v_ref_.get();
  cmd_vel.angular.z = w_ref_.get();
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
  tracking.pose.position.x = pos_line[0];
  tracking.pose.position.y = pos_line[1];
  tracking.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -angle));
  pub_tracking_.publish(tracking);

  path_step_done_ = iclose;
  if (path_step_done_ < 0)
    path_step_done_ = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_tracker");

  TrackerNode track;
  track.spin();

  return 0;
}
