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

#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <trajectory_tracker/TrajectoryTrackerStatus.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

class tracker
{
public:
	tracker();
	~tracker();
	void spin();
private:
	std::string topicPath;
	std::string topicCmdVel;
	std::string topicOdom;
	std::string frameRobot;
	double hz;
	double lookForward;
	double curvForward;
	double k[3];
	double d_lim;
	double d_stop;
	double vel[2];
	double acc[2];
	double w;
	double v;
	double dec;
	double rotate_ang;
	double angFactor;
	double swDist;
	double goalToleranceDist;
	double goalToleranceAng;
	double stopToleranceDist;
	double stopToleranceAng;
	double noPosCntlDist;
	double minTrackPath;
	int pathStep;
	int pathStepDone;
	bool outOfLineStrip;
    bool allowBackward;
	bool limitVelByAvel;
	bool checkOldPath;

	int error_cnt;

	ros::Subscriber subPath;
	ros::Subscriber subOdom;
	ros::Subscriber subVel;
	ros::Publisher pubVel;
	ros::Publisher pubStatus;
	ros::Publisher pubTracking;
	ros::NodeHandle nh;
	tf::TransformListener tf;

	nav_msgs::Path path;
	nav_msgs::Odometry odom;

	void cbPath(const nav_msgs::Path::ConstPtr& msg);
	void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
	void cbSpeed(const std_msgs::Float32::ConstPtr& msg);
	void control();
};

template<typename T>
class average
{
public:
	average():
		sum()
	{
		num = 0;
	};
	void operator +=(const T &val)
	{
		sum += val;
		num ++;
	};
	operator T()
	{
		if(num == 0) return 0;
		return sum / num;
	};
private:
	T sum;
	int num;
};

tracker::tracker() :
	w(0.0),
	v(0.0),
	nh("~")
{
	nh.param("frame_robot", frameRobot, std::string("base_link"));
	nh.param("path", topicPath, std::string("path"));
	nh.param("odom", topicOdom, std::string("odom"));
	nh.param("cmd_vel", topicCmdVel, std::string("cmd_vel"));
	nh.param("hz", hz, 50.0);
	nh.param("look_forward", lookForward, 0.5);
	nh.param("curv_forward", curvForward, 0.5);
	nh.param("k_dist", k[0], 1.0);
	nh.param("k_ang", k[1], 1.0);
	nh.param("k_avel", k[2], 1.0);
	nh.param("k_dcel", dec, 0.2);
	nh.param("dist_lim", d_lim, 0.5);
	nh.param("dist_stop", d_stop, 2.0);
	nh.param("rotate_ang", rotate_ang, M_PI / 4);
	nh.param("max_vel", vel[0], 0.5);
	nh.param("max_angvel", vel[1], 1.0);
	nh.param("max_acc", acc[0], 1.0);
	nh.param("max_angacc", acc[1], 2.0);
	nh.param("path_step", pathStep, 1);
	nh.param("distance_angle_factor", angFactor, 0.0);
	nh.param("switchback_dist", swDist, 0.3);
	nh.param("goal_tolerance_dist", goalToleranceDist, 0.2);
	nh.param("goal_tolerance_ang", goalToleranceAng, 0.1);
	nh.param("stop_tolerance_dist", stopToleranceDist, 0.1);
	nh.param("stop_tolerance_ang", stopToleranceAng, 0.05);
	nh.param("no_position_control_dist", noPosCntlDist, 0.0);
	nh.param("min_tracking_path", minTrackPath, noPosCntlDist);
	nh.param("allow_backward", allowBackward, true);
	nh.param("limit_vel_by_avel", limitVelByAvel, false);
	nh.param("check_old_path", checkOldPath, false);

	subPath = nh.subscribe(topicPath, 2, &tracker::cbPath, this);
	subOdom = nh.subscribe(topicOdom, 20, &tracker::cbOdom, this);
	subVel = nh.subscribe("speed", 20, &tracker::cbSpeed, this);
	pubVel = nh.advertise<geometry_msgs::Twist>(topicCmdVel, 10);
	pubStatus = nh.advertise<trajectory_tracker::TrajectoryTrackerStatus>("status", 10);
	pubTracking = nh.advertise<geometry_msgs::PoseStamped>("tracking", 10);
}
tracker::~tracker()
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	pubVel.publish(cmd_vel);
}

float dist2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	return sqrtf(powf(a.x-b.x,2) + powf(a.y-b.y,2));
}
float len2d(geometry_msgs::Point &a)
{
	return sqrtf(powf(a.x,2) + powf(a.y,2));
}
float len2d(geometry_msgs::Point a)
{
	return sqrtf(powf(a.x,2) + powf(a.y,2));
}
float curv3p(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	float ret;
	ret = 2 * (a.x*b.y + b.x*c.y + c.x*a.y - a.x*c.y - b.x*a.y - c.x*b.y);
	ret /= sqrtf( (powf(b.x-a.x, 2) + powf(b.y-a.y, 2)) * (powf(b.x-c.x, 2) + powf(b.y-c.y, 2)) * (powf(c.x-a.x, 2) + powf(c.y-a.y, 2)) );

	return ret;
}
float cross2d(geometry_msgs::Point &a, geometry_msgs::Point &b) 
{
	return a.x*b.y - a.y*b.x;
}
float cross2d(geometry_msgs::Point a, geometry_msgs::Point b) 
{
	return a.x*b.y - a.y*b.x;
}
float dot2d(geometry_msgs::Point &a, geometry_msgs::Point &b) 
{
	return a.x*b.x + a.y*b.y;
}
float dot2d(geometry_msgs::Point a, geometry_msgs::Point b) 
{
	return a.x*b.x + a.y*b.y;
}
geometry_msgs::Point point2d(float x, float y)
{
	geometry_msgs::Point ret;
	ret.x = x;
	ret.y = y;
	return ret;
}
geometry_msgs::Point sub2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	geometry_msgs::Point ret;
	ret.x = a.x - b.x;
	ret.y = a.y - b.y;
	return ret;
}
float sign(float a)
{
	if(a < 0) return -1;
	return 1;
}
float dist2d_line(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	return (cross2d(sub2d(b, a), sub2d(c, a)) / dist2d(b, a));
}
float dist2d_linestrip(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	if(dot2d(sub2d(b, a), sub2d(c, a) ) <= 0) return dist2d(c, a);
	if(dot2d(sub2d(a, b), sub2d(c, b) ) <= 0) return -dist2d(c, b) - 0.005;
	return fabs( dist2d_line(a, b, c) );
}
geometry_msgs::Point projection2d(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	float r = dot2d(sub2d(b, a), sub2d(c, a)) / pow(len2d(sub2d(b, a)), 2);
	geometry_msgs::Point ret;
	ret.x = b.x*r + a.x*(1-r);
	ret.y = b.y*r + a.y*(1-r);
	return ret;
}

void tracker::cbSpeed(const std_msgs::Float32::ConstPtr& msg)
{
	vel[0] = msg->data;
}

void tracker::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;
}

void tracker::cbPath(const nav_msgs::Path::ConstPtr& msg)
{
	path = *msg;
	auto i = path.poses.begin();
	for(auto j = path.poses.begin(); j != path.poses.end();)
	{
		if(i != j && dist2d((*i).pose.position, (*j).pose.position) < 0.01)
		{
			j = path.poses.erase(j);
			continue;
		}
		i = j;
		j ++;
	}
	pathStepDone = 0;
	
	while(path.poses.size() < 3 && path.poses.size() > 0)
	{
		float yaw = tf::getYaw(path.poses.back().pose.orientation);
		auto next = path.poses.back();
		next.pose.position.x += 0.001 * cos(yaw);
		next.pose.position.y += 0.001 * sin(yaw);
		path.poses.push_back(next);
	}
}

void tracker::spin()
{
	ros::Rate loop_rate(hz);

	while(ros::ok())
	{
		control();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void tracker::control()
{
	trajectory_tracker::TrajectoryTrackerStatus status;
	status.header.stamp = ros::Time::now();
	status.header.seq = path.header.seq;
	status.distance_remains = 0.0;
	status.angle_remains = 0.0;

	if(path.header.frame_id.size() == 0)
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
		pubVel.publish(cmd_vel);
		status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
		pubStatus.publish(status);
		return;
	}
	// Transform
	nav_msgs::Path lpath;
	lpath.header = path.header;
	tf::StampedTransform transform;
	try
	{
		ros::Time now = ros::Time(0);
		tf.waitForTransform(frameRobot, path.header.frame_id, now, ros::Duration(0.05));
		tf.lookupTransform(frameRobot, path.header.frame_id, now, transform);
		if(fabs((ros::Time::now() - transform.stamp_).toSec()) > 0.1)
		{
			if(error_cnt % 16 == 0 && checkOldPath)
				ROS_ERROR("Timestamp of the transform is too old %f %f", ros::Time::now().toSec(), transform.stamp_.toSec());
			error_cnt ++;
		}
		else
		{
			error_cnt = 0;
		}
		
		for(int i = 0; i < (int)path.poses.size(); i += pathStep)
		{
			geometry_msgs::PoseStamped pose;
			tf.transformPose(frameRobot, now, path.poses[i], path.header.frame_id, pose);
			lpath.poses.push_back(pose);
		}
	}
	catch (tf::TransformException &e)
	{
		ROS_WARN("TF exception: %s", e.what());
		status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
		pubStatus.publish(status);
		return;
	}

	float minDist = 10000.0;
	int iclose = -1;
	geometry_msgs::Point origin;
	origin.x = cos(w * lookForward / 2.0) * v * lookForward;
	origin.y = sin(w * lookForward / 2.0) * v * lookForward;
	// Find nearest line strip
	outOfLineStrip = false;
	float distancePath = 0;
	for(int i = 1; i < (int)lpath.poses.size(); i ++)
	{
		distancePath += dist2d(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
	}
	float distancePathSearch = 0;
	float signVel_prev = 0;
	for(int i = pathStepDone; i < (int)lpath.poses.size(); i ++)
	{
		if(i < 1) continue;
		distancePathSearch += dist2d(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
		if(dist2d(origin, lpath.poses[i].pose.position) < 0.05 &&
				i < (int)lpath.poses.size() - 1) continue;
		float d = dist2d_linestrip(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position, origin);
		if(fabs(d) <= fabs(minDist))
		{
			minDist = d;
			iclose = i;
		}
		if(pathStepDone > 0 && distancePathSearch > 1.0) break;

		geometry_msgs::Point vec = sub2d(lpath.poses[i].pose.position, 
				lpath.poses[i-1].pose.position);
		float angle = atan2(vec.y, vec.x);
		float anglePose;
		if(allowBackward) anglePose = tf::getYaw(lpath.poses[i+1].pose.orientation);
		else anglePose = angle;
		float signVel_req = cos(angle) * cos(anglePose) + sin(angle) * sin(anglePose);
		if(signVel_prev * signVel_req < 0)
		{
			// Stop read forward if the path switched back
			break;
		}
		signVel_prev = signVel_req;
	}
	if(iclose < 0)
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
		pubVel.publish(cmd_vel);
		//ROS_WARN("failed to find nearest node");
		status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
		pubStatus.publish(status);
		return;
	}
	// Signed distance error
	float dist = dist2d_line(lpath.poses[iclose-1].pose.position, lpath.poses[iclose].pose.position, origin);
	float _dist = dist;
	if(iclose == 0)
	{
		_dist = -dist2d(lpath.poses[iclose].pose.position, origin);
	}
	if(iclose + 1 >= (int)path.poses.size())
	{
		_dist = -dist2d(lpath.poses[iclose].pose.position, origin);
	}
	
	// Angular error
	geometry_msgs::Point vec = sub2d(lpath.poses[iclose].pose.position, lpath.poses[iclose - 1].pose.position);
	float angle = -atan2(vec.y, vec.x);
	float anglePose;
    if(allowBackward) anglePose = tf::getYaw(lpath.poses[iclose].pose.orientation);
    else anglePose = -angle;
	float signVel = 1.0;
	if(cos(-angle) * cos(anglePose) + sin(-angle) * sin(anglePose) < 0)
	{
		signVel = -1.0;
		angle = angle + M_PI;
		if(angle > M_PI) angle -= 2.0 * M_PI;
	}
	// Curvature
	average<float> curv;
	geometry_msgs::Point posLine = projection2d(lpath.poses[iclose-1].pose.position, lpath.poses[iclose].pose.position, origin);
	int local_goal = lpath.poses.size() - 1;
	float remainLocal = 0;
	remainLocal = dist2d(posLine, lpath.poses[iclose].pose.position);
	for(int i = iclose - 1; i < (int)lpath.poses.size() - 1; i ++)
	{
		if(i > 2)
		{
			geometry_msgs::Point vec = sub2d(lpath.poses[i].pose.position, 
					lpath.poses[i-1].pose.position);
			float angle = atan2(vec.y, vec.x);
            float anglePose;
            if(allowBackward) anglePose = tf::getYaw(lpath.poses[i+1].pose.orientation);
            else anglePose = angle;
			float signVel_req = cos(angle) * cos(anglePose) + sin(angle) * sin(anglePose);
			if(signVel * signVel_req < 0)
			{
				// Stop read forward if the path switched back
				local_goal = i;
				break;
			}
			if(i > iclose)
				remainLocal += dist2d(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
		}
	}
	for(int i = iclose - 1; i < local_goal; i ++)
	{
		if(i > 2)
		{
			curv += curv3p(lpath.poses[i-2].pose.position, lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
		}
		if(dist2d(lpath.poses[i].pose.position, 
					lpath.poses[local_goal].pose.position) < 0.05) break;
		if(dist2d(lpath.poses[i].pose.position, posLine) > curvForward) break;
	}
	float remain;
	remain = dist2d(origin, lpath.poses.back().pose.position);
	if(minDist < 0 && iclose == local_goal) outOfLineStrip = true;
	if(outOfLineStrip)
	{
		remain = -remain;
		remainLocal = -remainLocal;
	}
	if(distancePath < noPosCntlDist) remain = remainLocal = 0;
	//fprintf(stderr,"%d %d   %0.3f  %+0.3f %+0.3f  %f  %f  sv %f\n",outOfLineStrip, iclose, distancePath, remain,remainLocal,minDist, angle, signVel);
	//printf("d=%.2f, th=%.2f, curv=%.2f\n", dist, angle, (float)curv);
    while(angle < -M_PI) angle += 2.0*M_PI;
    while(angle > M_PI) angle -= 2.0*M_PI;

	status.distance_remains = remain;
	status.angle_remains = angle;

	float dt = 1/hz;
	float _v = v;
	float _w = w;
	// Stop and rotate
	if((fabs(rotate_ang) < M_PI && cos(rotate_ang) > cos(angle)) ||
		fabs(remainLocal) < stopToleranceDist ||
		distancePath < minTrackPath)
	{
		w = -sign(angle) * sqrtf(fabs(2 * angle * acc[1] * 0.9));
		v = 0;
		if(v > vel[0]) v = vel[0];
		else if(v < -vel[0]) v = -vel[0];
		if(v > _v + dt*acc[0]) v = _v + dt*acc[0];
		else if(v < _v - dt*acc[0]) v = _v - dt*acc[0];
		if(w > vel[1]) w = vel[1];
		else if(w < -vel[1]) w = -vel[1];
		if(w > _w + dt*acc[1]) w = _w + dt*acc[1];
		else if(w < _w - dt*acc[1]) w = _w - dt*acc[1];

		if(distancePath < stopToleranceDist)
		{
			status.distance_remains = remain = remainLocal = 0.0;
		}
	}
	else
	{
		// Control
		if(dist < -d_lim) dist = -d_lim;
		else if(dist > d_lim) dist = d_lim;

		v = sign(remainLocal) * signVel * sqrtf(fabs(2 * remainLocal * acc[0] * 0.95));

		if(v > vel[0]) v = vel[0];
		else if(v < -vel[0]) v = -vel[0];
		if(v > _v + dt*acc[0]) v = _v + dt*acc[0];
		else if(v < _v - dt*acc[0]) v = _v - dt*acc[0];

		float wref = fabs(v) * curv;

		if(limitVelByAvel)
		{
			if(fabs(wref) > vel[1])
			{
				v = sign(v) * fabs(vel[1] / curv);
				if(v > vel[0]) v = vel[0];
				else if(v < -vel[0]) v = -vel[0];
				if(v > _v + dt*acc[0]) v = _v + dt*acc[0];
				else if(v < _v - dt*acc[0]) v = _v - dt*acc[0];
			}
		}

		w += dt * (-dist*k[0] -angle*k[1] -(w - wref)*k[2]);

		if(w > vel[1]) w = vel[1];
		else if(w < -vel[1]) w = -vel[1];
		if(w > _w + dt*acc[1]) w = _w + dt*acc[1];
		else if(w < _w - dt*acc[1]) w = _w - dt*acc[1];

		if(!std::isfinite(v)) v = 0;
		if(!std::isfinite(w)) w = 0;

		// Too far from given path
		if(fabs(_dist) > d_stop)
		{
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			pubVel.publish(cmd_vel);
			//ROS_WARN("Far from given path");
			status.status = trajectory_tracker::TrajectoryTrackerStatus::FAR_FROM_PATH;
			pubStatus.publish(status);
			return;
		}
	}

	geometry_msgs::Twist cmd_vel;
	if(fabs(status.distance_remains) < stopToleranceDist &&
			fabs(status.angle_remains) < stopToleranceAng)
	{
		v = 0;
		w = 0;
	}

	cmd_vel.linear.x = v;
	cmd_vel.angular.z = w;
	pubVel.publish(cmd_vel);
	status.status = trajectory_tracker::TrajectoryTrackerStatus::FOLLOWING;
	if(fabs(status.distance_remains) < goalToleranceDist &&
			fabs(status.angle_remains) < goalToleranceAng)
	{
		status.status = trajectory_tracker::TrajectoryTrackerStatus::GOAL;
	}
	pubStatus.publish(status);
	geometry_msgs::PoseStamped tracking;
	tracking.header = status.header;
	tracking.header.frame_id = frameRobot;
	tracking.pose.position = posLine;
	tracking.pose.orientation = tf::createQuaternionMsgFromYaw(-angle);
	pubTracking.publish(tracking);

	pathStepDone = iclose;
	if(pathStepDone < 0) pathStepDone = 0;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_tracker");

	tracker track;
	track.spin();

	return 0;
}
