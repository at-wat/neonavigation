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
 */

#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class recorder
{
public:
	recorder();
	~recorder();
	void spin();
private:
	std::string topicPath;
	std::string frameRobot;
	std::string frameGlobal;
	double dist_interval;
	double ang_interval;
	bool store_time;

	ros::NodeHandle nh;
	ros::Publisher pubPath;
	tf::TransformListener tf;

	nav_msgs::Path path;
};

recorder::recorder():
	nh("~")
{
	nh.param("frame_robot", frameRobot, std::string("base_link"));
	nh.param("frame_global", frameGlobal, std::string("map"));
	nh.param("path", topicPath, std::string("recpath"));
	nh.param("dist_interval", dist_interval, 0.3);
	nh.param("ang_interval", ang_interval, 1.0);
	nh.param("store_time", store_time, false);

	pubPath = nh.advertise<nav_msgs::Path>(topicPath, 10, true);
}
recorder::~recorder()
{
}

float dist2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	return sqrtf(powf(a.x-b.x,2) + powf(a.y-b.y,2));
}

void recorder::spin()
{
	ros::Rate loop_rate(50);
	path.header.frame_id = frameGlobal;
	path.header.seq = 0;

	while(ros::ok())
	{
		ros::Time now = ros::Time(0);
		if(store_time) now = ros::Time::now();
		tf::StampedTransform transform;
		try
		{
			tf.waitForTransform(frameGlobal, frameRobot, now, ros::Duration(0.2));
			tf.lookupTransform(frameGlobal, frameRobot, now, transform);
		}
		catch (tf::TransformException &e)
		{
			ROS_WARN("TF exception: %s", e.what());
			continue;
		}
		geometry_msgs::PoseStamped pose;
		tf::Quaternion q;
		transform.getBasis().getRotation(q);
		tf::quaternionTFToMsg(q, pose.pose.orientation);
		tf::Vector3 origin = transform.getOrigin();
		pose.pose.position.x = origin.x();
		pose.pose.position.y = origin.y();
		pose.pose.position.z = origin.z();
		pose.header.frame_id = frameGlobal;
		pose.header.stamp = now;
		pose.header.seq = path.poses.size();

		path.header.seq ++;
		path.header.stamp = now;
		
		if(path.poses.size() == 0)
		{
			path.poses.push_back(pose);
			pubPath.publish(path);
		}
		else if(dist2d(path.poses.back().pose.position, pose.pose.position) > dist_interval)
		{
			path.poses.push_back(pose);
			pubPath.publish(path);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_recorder");

	recorder rec;
	rec.spin();

	return 0;
}
