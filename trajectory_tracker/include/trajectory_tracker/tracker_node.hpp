/*
 * Copyright (c) 2024, the neonavigation authors
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

#ifndef TRAJECTORY_TRACKER__TRACKER_NODE_H_
#define TRAJECTORY_TRACKER__TRACKER_NODE_H_

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

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace trajectory_tracker
{
class TrackerNode : public neonavigation_common::NeonavigationNode
{
public:
  TrackerNode(const std::string& name, const rclcpp::NodeOptions& options);
  void initialize();
  ~TrackerNode();

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
  double acc_toc_factor_;
  double angacc_toc_factor_;
  trajectory_tracker::VelAccLimitter v_lim_;
  trajectory_tracker::VelAccLimitter w_lim_;
  double rotate_ang_;
  double goal_tolerance_dist_;
  double goal_tolerance_ang_;
  double stop_tolerance_dist_;
  double stop_tolerance_ang_;
  double no_pos_cntl_dist_;
  double min_track_path_;
  int64_t path_step_;
  int64_t path_step_done_;
  bool allow_backward_;
  bool limit_vel_by_avel_;
  bool check_old_path_;
  double epsilon_;
  double max_dt_;
  bool use_time_optimal_control_;
  double time_optimal_control_future_gain_;
  double k_ang_rotation_;
  double k_avel_rotation_;
  double goal_tolerance_lin_vel_;
  double goal_tolerance_ang_vel_;

  rclcpp::SubscriptionBase::SharedPtr sub_path_;
  rclcpp::SubscriptionBase::SharedPtr sub_path_velocity_;
  rclcpp::SubscriptionBase::SharedPtr sub_speed_;
  rclcpp::SubscriptionBase::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tracking_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr odom_timeout_timer_;
  double odom_timeout_sec_;

  trajectory_tracker::Path2D path_;
  std_msgs::msg::Header path_header_;
  bool is_path_updated_;

  bool predict_odom_;
  rclcpp::Time prev_odom_stamp_;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;
  std::unique_ptr<ActionServer> action_server_;
  std::recursive_mutex action_server_mutex_;
  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus latest_status_;

  struct TrackingResult
  {
    explicit TrackingResult(const int s)
      : status(s)
      , distance_remains(0.0)
      , angle_remains(0.0)
      , distance_remains_raw(0.0)
      , angle_remains_raw(0.0)
      , turning_in_place(false)
      , signed_local_distance(0.0)
      , distance_from_target(0.0)
      , target_linear_vel(0.0)
      , tracking_point_x(0.0)
      , tracking_point_y(0.0)
      , tracking_point_curv(0.0)
      , path_step_done(0)
    {
    }

    int status;  // same as trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::status
    double distance_remains;
    double angle_remains;
    double distance_remains_raw;  // remained distance without prediction
    double angle_remains_raw;
    bool turning_in_place;
    double signed_local_distance;
    double distance_from_target;
    double target_linear_vel;
    double tracking_point_x;
    double tracking_point_y;
    double tracking_point_curv;
    int path_step_done;
  };

  void onDynamicParameterUpdated(const std::vector<rclcpp::Parameter>&) final;

  template <typename MSG_TYPE>
  void cbPath(const MSG_TYPE&);
  void cbSpeed(const std_msgs::msg::Float32&);
  void cbOdometry(const nav_msgs::msg::Odometry&);

  void cbTimer();
  void cbOdomTimeout();
  void control(const tf2::Stamped<tf2::Transform>&, const Eigen::Vector3d&, const double, const double, const double);
  TrackingResult getTrackingResult(const tf2::Stamped<tf2::Transform>&, const Eigen::Vector3d&, const double,
                                   const double) const;
  void computeControl();
  void publishZeroVelocity();
  void resetLatestStatus();
};

}  // namespace trajectory_tracker
#endif  // TRAJECTORY_TRACKER__TRACKER_NODE_H_
