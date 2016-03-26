#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>


std::string to;
std::shared_ptr<tf::TransformListener> tfl;

ros::Publisher pubPose;

void cbPose(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
	try
	{
		geometry_msgs::PoseStamped in;
		geometry_msgs::PoseStamped out;
		geometry_msgs::PoseWithCovarianceStamped out_msg;
		in.header = msg->header;
		in.pose = msg->pose.pose;
		tfl->waitForTransform(to, msg->header.frame_id, msg->header.stamp, ros::Duration(0.5));
		tfl->transformPose(to, in, out);
		out_msg = *msg;
		out_msg.header = out.header;
		out_msg.pose.pose = out.pose;
		pubPose.publish(out_msg);
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN("pose_transform: %s", e.what());
	}
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "pose_transform");
	ros::NodeHandle nh("~");

	auto subPose = nh.subscribe("pose_in", 1, cbPose);
	pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 1, false);
	nh.param("to_frame", to, std::string("map"));

	tfl.reset(new tf::TransformListener);
	ros::spin();

	return 0;
}

