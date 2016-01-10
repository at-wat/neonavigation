#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap/CSpace3D.h>

namespace ros
{
	class NodeHandle_f: public NodeHandle
	{
	public:
		void param_cast(const std::string& param_name, 
				float& param_val, const float& default_val) const
		{
			double _default_val_d = (double)default_val;
			double _param_val_d;

			param<double>(param_name, _param_val_d, _default_val_d);
			param_val = (float)_param_val_d;
		}
		NodeHandle_f(const std::string& ns = std::string(), 
				const M_string& remappings = M_string()) : NodeHandle(ns, remappings)
		{
		}

	};
}

class costmap_3d
{
private:
	ros::NodeHandle_f nh;
	ros::Subscriber sub_map;
	ros::Publisher pub_costmap;
	ros::Publisher pub_footprint;
	geometry_msgs::PolygonStamped footprint;

	int ang_resolution;
	float linear_expand;
	float angular_expand;
	float linear_spread;
	float angular_spread;

	bool XmlRpc_isNumber(XmlRpc::XmlRpcValue &value)
	{
		return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
			value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
	}

	void cb_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
	{
		costmap::CSpace3D map;

		map.header = msg->header;
		map.info.width = msg->info.width;
		map.info.height = msg->info.height;
		map.info.angle = ang_resolution;
		map.info.linear_resolution = msg->info.resolution;
		map.info.angular_resolution = 2.0 * M_PI / ang_resolution;
		map.info.origin = msg->info.origin;
		map.data.resize(msg->data.size() * ang_resolution);

		for(unsigned int i = 0; i < msg->data.size(); i ++)
		{
			for(int j = 0; j < ang_resolution; j ++)
			{
				map.data[i + j * msg->data.size()] = msg->data[i];
			}
		}

		pub_costmap.publish(map);
	}

public:
	void spin()
	{
		ros::Rate wait(1);
		while(ros::ok())
		{
			wait.sleep();
			ros::spinOnce();
			footprint.header.stamp = ros::Time::now();
			pub_footprint.publish(footprint);
		}
	}
	costmap_3d():
		nh("~")
	{
		nh.param("ang_resolution", ang_resolution, 16);
		nh.param("linear_expand", linear_expand, 0.2f);
		nh.param("angular_expand", angular_expand, 0.4f);
		nh.param("linear_spread", linear_spread, 0.5f);
		nh.param("angular_spread", angular_spread, 1.0f);

		XmlRpc::XmlRpcValue footprint_xml;
		if(!nh.hasParam("footprint"))
		{
			ROS_FATAL("Footprint doesn't specified");
			throw std::runtime_error("Footprint doesn't specified");
		}
		nh.getParam("footprint", footprint_xml);
		if(footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
		{
			ROS_FATAL("Invalid footprint");
			throw std::runtime_error("Invalid footprint");
		}
		footprint.polygon.points.clear();
		footprint.header.frame_id = "base_link";
		for(int i = 0; i < (int)footprint_xml.size(); i ++)
		{
			if(!XmlRpc_isNumber(footprint_xml[i][0]) || 
					!XmlRpc_isNumber(footprint_xml[i][1]))
			{
				ROS_FATAL("Invalid footprint value");
				throw std::runtime_error("Invalid footprint value");
			}

			geometry_msgs::Point32 point;
			point.x = (double)footprint_xml[i][0];
			point.y = (double)footprint_xml[i][1];
			point.z = 0;
			footprint.polygon.points.push_back(point);
		}
		footprint.polygon.points.push_back(footprint.polygon.points[0]);

		sub_map = nh.subscribe("/map", 1, &costmap_3d::cb_map, this);
		pub_costmap = nh.advertise<costmap::CSpace3D>("costmap", 1, true);
		pub_footprint = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 2, true);
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "costmap_3d");
	
	costmap_3d cm;
	cm.spin();

	return 0;
}


