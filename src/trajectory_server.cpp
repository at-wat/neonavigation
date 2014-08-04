#include <ros/ros.h>
#include <math.h>
#include <fstream>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <trajectory_tracker/ChangePath.h>
#include <trajectory_tracker/TrajectoryServerStatus.h>

class server
{
public:
	server();
	~server();
	void spin();
private:
	ros::NodeHandle nh;
	ros::Publisher pubPath;
	ros::Publisher pubStatus;
	tf::TransformListener tf;
	ros::ServiceServer srvChangePath;

	nav_msgs::Path path;
	std::string topicPath;
	trajectory_tracker::ChangePath::Request reqPath;
	double hz;
	boost::shared_array<uint8_t> buffer;
	int serial_size;
	
	bool loadFile();
	void loadPath();
	bool change(trajectory_tracker::ChangePath::Request &req,
			trajectory_tracker::ChangePath::Response &res);
};

server::server():
	nh("~"),
	buffer(new uint8_t[1024])
{
	nh.param("path", topicPath, std::string("path"));
	nh.param("file", reqPath.filename, std::string("a.path"));
	nh.param("hz", hz, double(5));

	pubPath = nh.advertise<nav_msgs::Path>(topicPath, 2, true);
	pubStatus = nh.advertise<trajectory_tracker::TrajectoryServerStatus>("status", 2);
	srvChangePath = nh.advertiseService("ChangePath", &server::change, this);
}
server::~server()
{
}

bool server::loadFile()
{
	std::ifstream ifs(reqPath.filename.c_str());
	if(ifs.good())
	{
		ifs.seekg(0, ifs.end);
		serial_size = ifs.tellg();
		ifs.seekg(0, ifs.beg);
		buffer.reset(new uint8_t[serial_size]);
		ifs.read((char*)buffer.get(), serial_size);
		return true;
	}
	return false;
}

bool server::change(trajectory_tracker::ChangePath::Request &req,
		trajectory_tracker::ChangePath::Response &res)
{
	reqPath = req;
	res.success = false;

	if(loadFile())
	{
		res.success = true;
		ros::serialization::IStream stream(buffer.get(), serial_size);
		ros::serialization::deserialize(stream, path);
		path.header.stamp = ros::Time::now();
		pubPath.publish(path);
	}
	else
	{
		serial_size = 0;
		reqPath.filename = "";
		path.poses.clear();
		path.header.frame_id = "map";
	}
	return true;
}

void server::spin()
{
	ros::Rate loop_rate(hz);
	trajectory_tracker::TrajectoryServerStatus status;

	while(ros::ok())
	{
		status.header = path.header;
		status.filename = reqPath.filename;
		status.id = reqPath.id;
		pubStatus.publish(status);
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

