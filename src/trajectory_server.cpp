#include <ros/ros.h>
#include <math.h>
#include <fstream>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <trajectory_tracker/ChangePath.h>
#include <trajectory_tracker/TrajectoryServerStatus.h>
#include <interactive_markers/interactive_marker_server.h>

#include "filter.h"

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
	interactive_markers::InteractiveMarkerServer srvIMFb;

	nav_msgs::Path path;
	std::string topicPath;
	trajectory_tracker::ChangePath::Request reqPath;
	double hz;
	boost::shared_array<uint8_t> buffer;
	int serial_size;
    double filter_step;
    filter *lpf[2];
	
	bool loadFile();
	void loadPath();
	bool change(trajectory_tracker::ChangePath::Request &req,
			trajectory_tracker::ChangePath::Response &res);
	void processFeedback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void updateIM();
	enum
	{
		MENU_DELETE = 1,
		MENU_ADD = 2
	};
	int updateNum;
	int maxMarkers;
};

server::server():
	nh("~"),
	srvIMFb("trajectory_server"),
	buffer(new uint8_t[1024])
{
	nh.param("path", topicPath, std::string("path"));
	nh.param("file", reqPath.filename, std::string("a.path"));
	nh.param("hz", hz, double(5));
	nh.param("filter_step", filter_step, 0.0);

	pubPath = nh.advertise<nav_msgs::Path>(topicPath, 2, true);
	pubStatus = nh.advertise<trajectory_tracker::TrajectoryServerStatus>("status", 2);
	srvChangePath = nh.advertiseService("ChangePath", &server::change, this);
	updateNum = 0;
	maxMarkers = 0;
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

void server::processFeedback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	int id = std::atoi(feedback->marker_name.c_str());
	switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		path.poses[id].pose = feedback->pose;
		break;
	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
		path.header.stamp = ros::Time::now();
		pubPath.publish(path);
		break;
	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		switch(feedback->menu_entry_id)
		{
		case MENU_DELETE:
			path.poses.erase(path.poses.begin() + id);
			break;
		case MENU_ADD:
			path.poses.insert(path.poses.begin() + id, path.poses[id]);
			break;
		}
		pubPath.publish(path);
		updateIM();
		break;
	}
}

void server::updateIM()
{
	visualization_msgs::InteractiveMarkerUpdate viz;
	viz.type = viz.KEEP_ALIVE;
	viz.seq_num = updateNum ++;
	viz.server_id = "Path";
	srvIMFb.clear();
	int i = 0;
	for(auto &p: path.poses)
	{
		visualization_msgs::InteractiveMarker mark;
		visualization_msgs::Marker marker;
		visualization_msgs::InteractiveMarkerControl ctl;
		visualization_msgs::MenuEntry menu;
		mark.header = path.header;
		mark.pose = p.pose;
		mark.scale = 1.0;
		std::stringstream ss;
		ss << i ++;
		mark.name = ss.str();
		marker.ns = "Path";
		marker.id = i;
		marker.scale.x = 0.2;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color.r = 1;
		marker.color.g = 1;
		marker.color.b = 1;
		marker.color.a = 1;
		marker.type = marker.ARROW;

		ss << " ctl";
		ctl.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI/2.0, 0.0);
		ctl.interaction_mode = ctl.MOVE_ROTATE;
		ctl.orientation_mode = ctl.INHERIT;
		ctl.always_visible = true;
		ctl.markers.push_back(marker);
		ctl.name = ss.str();
		mark.controls.push_back(ctl);

		ss << " menu";
		ctl.interaction_mode = ctl.MENU;
		ctl.name = ss.str();
		marker.type = marker.SPHERE;
		marker.color.g = 0;
		marker.color.b = 0;
		marker.scale.x = 0.08;
		marker.scale.y = 0.08;
		marker.scale.z = 0.08;
		ctl.markers[0] = marker;
		mark.controls.push_back(ctl);

		menu.id = MENU_DELETE;
		menu.parent_id = 0;
		menu.title = "Delete";
		menu.command_type = menu.FEEDBACK;
		mark.menu_entries.push_back(menu);
		menu.id = MENU_ADD;
		menu.parent_id = 0;
		menu.title = "Add";

		mark.menu_entries.push_back(menu);
		srvIMFb.insert(mark, boost::bind(&server::processFeedback, this, _1));
		viz.markers.push_back(mark);
	}
	srvIMFb.applyChanges();
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
        if(filter_step > 0)
        {
            std::cout << filter_step << std::endl;
            lpf[0] = new filter(FILTER_LPF, filter_step, path.poses[0].pose.position.x);
            lpf[1] = new filter(FILTER_LPF, filter_step, path.poses[0].pose.position.y);
		    
            for(int i = 0; i < (int)path.poses.size(); i ++)
            {
                path.poses[i].pose.position.x = lpf[0]->in(path.poses[i].pose.position.x);
                path.poses[i].pose.position.y = lpf[1]->in(path.poses[i].pose.position.y);
            }

            delete lpf[0];
            delete lpf[1];
        }

		pubPath.publish(path);
		updateIM();
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

