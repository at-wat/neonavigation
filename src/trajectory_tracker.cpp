#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

class tracker
{
public:
	tracker();
	void spin();
private:
	std::string topicPath;
	std::string topicCmdVel;
	std::string frameRobot;
	double hz;

	ros::NodeHandle nh;
	ros::Subscriber subPath;
	ros::Publisher pubVel;
	tf::TransformListener tf;

	nav_msgs::Path path;

	void cbPath(const nav_msgs::Path::ConstPtr& msg);
	void control();
};

tracker::tracker():
	nh("~")
{
	nh.param("frame_robot", frameRobot, std::string("base_link"));
	nh.param("path", topicPath, std::string("path"));
	nh.param("cmd_vel", topicCmdVel, std::string("cmd_vel"));
	nh.param("hz", hz, 50.0);

	subPath = nh.subscribe(topicPath, 200, &tracker::cbPath, this);
	pubVel = nh.advertise<geometry_msgs::Twist>(topicCmdVel, 10);
}

float dist2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	return sqrtf(powf(a.x-b.x,2) + powf(a.y-b.y,2));
}

float len2d(geometry_msgs::Point &a)
{
	return sqrtf(powf(a.x,2) + powf(a.y,2));
}

void tracker::cbPath(const nav_msgs::Path::ConstPtr& msg)
{
	path = *msg;
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
	if(path.header.frame_id.size() == 0 ||
			path.poses.size() == 0) return;
	nav_msgs::Path lpath = path;
	try
	{
		ros::Time now = ros::Time(0);
		tf.waitForTransform(frameRobot, path.header.frame_id, now, ros::Duration(0.2));

		for(int i = 0; i < path.poses.size(); i ++)
		{
			tf.transformPose(frameRobot, now, path.poses[i], path.header.frame_id, lpath.poses[i]);
		}
	}
	catch (tf::TransformException &e)
	{
		ROS_WARN("TF exception: %s", e.what());
		return;
	}
	float minDist = FLT_MAX;
	int iclose = 0;
	for(int i = 0; i < path.poses.size(); i ++)
	{
		float d = len2d(lpath.poses[i].pose.position);
		if(d < minDist)
		{
			minDist = d;
			iclose = i;
		}
	}
	for(int i = iclose; i < path.poses.size(); i ++)
	{
		if(dist2d(lpath.poses[i].pose.position, lpath.poses[iclose].pose.position) > 1) break;
	}

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_tracker");

	tracker track;
	track.spin();

	return 0;
}
