#include <ros/ros.h>
#include <math.h>
#include <fstream>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <trajectory_tracker/ChangePath.h>

class server
{
public:
	server();
	~server();
	void spin();
	void loadPath();
	bool change(trajectory_tracker::ChangePath::Request &req,
			trajectory_tracker::ChangePath::Response &res);
private:
	ros::NodeHandle nh;
	ros::Publisher pubPath;
	tf::TransformListener tf;
	ros::ServiceServer srvChangePath;

	nav_msgs::Path path;
	std::string topicPath;
	std::string filename;
	double hz;
	bool load;
};

server::server():
	nh("~"), load(false)
{
	nh.param("path", topicPath, std::string("path"));
	nh.param("file", filename, std::string("a.path"));
	nh.param("hz", hz, double(5));

	pubPath = nh.advertise<nav_msgs::Path>(topicPath, 2, true);
	srvChangePath = nh.advertiseService("ChangePath", &server::change, this);
}
server::~server()
{
}

bool server::change(trajectory_tracker::ChangePath::Request &req,
		trajectory_tracker::ChangePath::Response &res)
{
	filename = req.filename;
	load = false;
	return true;
}

void server::spin()
{
	ros::Rate loop_rate(hz);

	while(ros::ok())
	{
		if(!load)
		{
			if(filename.size() <= 0)
			{
				path.poses.clear();
				path.header.frame_id = "map";
			}
			else
			{
				std::ifstream ifs(filename.c_str());
				if(ifs.good())
				{
					ifs.seekg(0, ifs.end);
					int serial_size = ifs.tellg();
					ifs.seekg(0, ifs.beg);
					boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
					ifs.read((char*)buffer.get(), serial_size);

					ros::serialization::IStream stream(buffer.get(), serial_size);
					ros::serialization::deserialize(stream, path);
				}
				else
				{
					path.poses.clear();
					path.header.frame_id = "map";
				}
			}
			load = true;
		}
		path.header.stamp = ros::Time(0);
		pubPath.publish(path);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_server");

	server serv;
	serv.spin();

	return 0;
}

