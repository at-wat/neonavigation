#include <ros/ros.h>
#include <math.h>
#include <fstream>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class saver
{
public:
	saver();
	~saver();
	void save();
private:
	ros::NodeHandle nh;
	ros::Subscriber subPath;
	tf::TransformListener tf;

	std::string topicPath;
	std::string filename;
	bool saved;
	void cbPath(const nav_msgs::Path::ConstPtr& msg);
};

saver::saver():
	nh("~"), saved(false)
{
	nh.param("path", topicPath, std::string("recpath"));
	nh.param("file", filename, std::string("a.path"));

	subPath = nh.subscribe(topicPath, 10, &saver::cbPath, this);
}
saver::~saver()
{
}

void saver::cbPath(const nav_msgs::Path::ConstPtr& msg)
{
	if(saved) return;
	std::ofstream ofs(filename.c_str());
	
	if(!ofs)
	{
		ROS_ERROR("Failed to open %s", filename.c_str());
		return;
	}

	uint32_t serial_size = ros::serialization::serializationLength(*msg);
	ROS_INFO("Size: %d\n", (int)serial_size);
	boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

	ros::serialization::OStream stream(buffer.get(), serial_size);
	ros::serialization::serialize(stream, *msg);

	ofs.write((char*)buffer.get(), serial_size);

	saved = true;
}

void saver::save()
{
	ros::Rate loop_rate(5);
	ROS_INFO("Waiting for the path");

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		if(saved) break;
	}
	ROS_INFO("Path saved");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_saver");

	saver rec;
	rec.save();

	return 0;
}
