#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "tf_projection.hpp"

class tf_projection_node : public tf_projection
{
private:
	ros::NodeHandle nh;
	tf::TransformBroadcaster tf_broadcaster;
	tf::TransformListener tf_listener;

	double rate;

public:
	tf_projection_node() :
		nh("~")
	{
		nh.param("base_link_frame", frames["base"], std::string("base_link"));
		nh.param("projection_frame", frames["projection"], std::string("map"));
		nh.param("target_frame", frames["target"], std::string("map"));
		nh.param("frame", frames["frame"], std::string("base_link_projected"));

		nh.param("hz", rate, 10.0);
	}
	void process()
	{
		try
		{
			tf::StampedTransform trans;
			tf_listener.waitForTransform(frames["projection"], frames["base"],
					ros::Time(0), ros::Duration(0.1));
			tf_listener.lookupTransform(frames["projection"], frames["base"],
					ros::Time(0), trans);
			
			tf::StampedTransform trans_target;
			tf_listener.waitForTransform(frames["target"], frames["projection"],
					trans.stamp_, ros::Duration(0.1));
			tf_listener.lookupTransform(frames["target"], frames["projection"],
					trans.stamp_, trans_target);

			const auto result = project(trans, trans_target);

			geometry_msgs::TransformStamped trans_out;
			tf::transformStampedTFToMsg(result, trans_out);

			tf_broadcaster.sendTransform(trans_out);
		}
		catch(tf::TransformException &e)
		{
			ROS_WARN_ONCE("%s", e.what());
		}
	}
	void spin()
	{
		ros::Rate r(rate);
		while(ros::ok())
		{
			ros::spinOnce();
			process();
			r.sleep();
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "tf_projection");
	
	tf_projection_node proj;
	proj.spin();

	return 0;
}

