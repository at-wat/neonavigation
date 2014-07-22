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
