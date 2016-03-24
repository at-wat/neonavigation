#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <random>
#include <string>
#include <iostream>
#include <sstream>

pcl::PointXYZ operator-(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
	auto c = a;
	c.x -= b.x;
	c.y -= b.y;
	c.z -= b.z;
	return c;
}
pcl::PointXYZ operator+(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
	auto c = a;
	c.x += b.x;
	c.y += b.y;
	c.z += b.z;
	return c;
}
pcl::PointXYZ operator*(const pcl::PointXYZ &a, const float &b)
{
	auto c = a;
	c.x *= b;
	c.y *= b;
	c.z *= b;
	return c;
}

class safety_limiter
{
public:
	safety_limiter():
		nh("~")
	{
		pub_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 1, true);
		pub_cloud = nh.advertise<sensor_msgs::PointCloud>("collision", 1, true);
		pub_debug = nh.advertise<sensor_msgs::PointCloud>("debug", 1, true);
		sub_twist = nh.subscribe("cmd_vel_in", 1, &safety_limiter::cb_twist, this);
		sub_cloud = nh.subscribe("cloud", 1, &safety_limiter::cb_cloud, this);
		sub_disable = nh.subscribe("disable", 1, &safety_limiter::cb_disable, this);

		nh.param("freq", hz, 6.0);
		nh.param("cloud_timeout", timeout, 0.8);
		nh.param("disable_timeout", disable_timeout, 0.1);
		nh.param("lin_vel", vel[0], 0.5);
		nh.param("lin_acc", acc[0], 1.0);
		nh.param("ang_vel", vel[1], 0.8);
		nh.param("ang_acc", acc[1], 1.6);
		nh.param("z_range_min", z_range[0], 0.0);
		nh.param("z_range_max", z_range[1], 0.5);
		nh.param("dt", dt, 0.1);
		nh.param("t_margin", tmargin, 0.2);
		nh.param("downsample_grid", downsample_grid, 0.05);
		nh.param("frame_id", frame_id, std::string("base_link"));
		tmax = 0.0;
		for(int i = 0; i < 2; i ++)
		{
			auto t = vel[i] / acc[i];
			if(tmax < t) tmax = t;
		}
		tmax *= 1.5;
		tmax += tmargin;

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
		footprint_radius = 0;
		for(int i = 0; i < (int)footprint_xml.size(); i ++)
		{
			if(!XmlRpc_isNumber(footprint_xml[i][0]) || 
					!XmlRpc_isNumber(footprint_xml[i][1]))
			{
				ROS_FATAL("Invalid footprint value");
				throw std::runtime_error("Invalid footprint value");
			}

			vec v;
			v[0] = (double)footprint_xml[i][0];
			v[1] = (double)footprint_xml[i][1];
			footprint_p.v.push_back(v);

			auto dist = hypotf(v[0], v[2]);
			if(dist > footprint_radius) footprint_radius = dist;
		}
		footprint_p.v.push_back(footprint_p.v.front());
		ROS_INFO("footprint radius: %0.3f", footprint_radius);

		has_cloud = false;
		has_twist = true;

		last_disable_cmd = ros::Time(0);
	}
	void spin()
	{
		ros::Rate rate(hz);
		while(ros::ok())
		{
			rate.sleep();
			ros::spinOnce();

			if(!has_twist) continue;
			
			if(ros::Time::now() - last_disable_cmd < ros::Duration(disable_timeout))
			{
				pub_twist.publish(twist);
				continue;
			}

			if(ros::Time::now() - cloud.header.stamp > ros::Duration(timeout))
			{
				if(has_cloud) ROS_WARN("Safety Limit: PointCloud timed-out");
				geometry_msgs::Twist cmd_vel;
				pub_twist.publish(cmd_vel);
				has_cloud = false;
				continue;
			}
			if(!has_cloud) continue;
			geometry_msgs::Twist cmd_vel;
			cmd_vel = limit(twist, cloud);
			pub_twist.publish(cmd_vel);
		}
	}
	geometry_msgs::Twist limit(const geometry_msgs::Twist &in, 
			const sensor_msgs::PointCloud2 &cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(cloud, *pc);
		
		try
		{
			tfl.waitForTransform(frame_id, cloud.header.frame_id, cloud.header.stamp,
					ros::Duration(0.1));
			pcl_ros::transformPointCloud(frame_id, *pc, *pc, tfl);
		}
		catch(tf::TransformException &e)
		{
			ROS_WARN("Safety Limit: Transform failed");
			geometry_msgs::Twist cmd_vel;
			return cmd_vel;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ds(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> ds;
		ds.setInputCloud(pc);
		ds.setLeafSize(downsample_grid, downsample_grid, downsample_grid);
		ds.filter(*pc_ds);
		*pc = *pc_ds;

		pc->erase(std::remove_if(pc->points.begin(), pc->points.end(),
					[&](pcl::PointXYZ &p)
					{
						if(p.z < z_range[0] || z_range[1] < p.z) return true;
						p.z = 0.0;
						return false;
					}), pc->points.end());

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(pc);
		
		Eigen::Affine3f move;
		Eigen::Affine3f move_inv;
		Eigen::Affine3f motion = 
			Eigen::AngleAxisf(-twist.angular.z * dt, Eigen::Vector3f::UnitZ())
			* Eigen::Translation3f(Eigen::Vector3f(-twist.linear.x * dt, 0.0, 0.0));
		Eigen::Affine3f motion_inv = 
			Eigen::Translation3f(Eigen::Vector3f(twist.linear.x * dt, 0.0, 0.0))
			* Eigen::AngleAxisf(twist.angular.z * dt, Eigen::Vector3f::UnitZ());
		move.setIdentity();
		move_inv.setIdentity();
		float t_col = tmax;
		sensor_msgs::PointCloud col_points;
		sensor_msgs::PointCloud debug_points;
		col_points.header.frame_id = frame_id;
		col_points.header.stamp = ros::Time::now();
		debug_points.header = col_points.header;
		for(float t = 0; t < tmax; t += dt)
		{
			move = move * motion;
			move_inv = move_inv * motion_inv;
			pcl::PointXYZ center;
			center = pcl::transformPoint(center, move_inv);
			geometry_msgs::Point32 p;
			p.x = center.x;
			p.y = center.y;
			p.z = 0.0;
			debug_points.points.push_back(p);

			std::vector<int> indices;
			std::vector<float> dist;
			int num = kdtree.radiusSearch(center, footprint_radius, indices, dist);
			if(num == 0) continue;

			bool col = false;
			for(auto &i: indices)
			{
				auto &p = pc->points[i];
				auto point = pcl::transformPoint(p, move);
				vec v(point.x, point.y);
				if(footprint_p.inside(v))
				{
					geometry_msgs::Point32 pos;
					pos.x = p.x;
					pos.y = p.y;
					pos.z = p.z;
					col_points.points.push_back(pos);
					col = true;
					break;
				}
			}
			if(col)
			{
				t_col = t;
				break;
			}
		}
		pub_debug.publish(debug_points);

		t_col -= tmargin;
		if(t_col < 0) t_col = 0;
		auto out = in;
		float lin_vel_lim = t_col * acc[0];
		float ang_vel_lim = t_col * acc[1];
		
		bool col = false;
		if(fabs(out.linear.x) > lin_vel_lim)
		{
			float r = lin_vel_lim / fabs(out.linear.x);
			out.linear.x *= r;
			out.angular.z *= r;
			col = true;
		}
		if(fabs(out.angular.z) > ang_vel_lim)
		{
			float r = ang_vel_lim / fabs(out.angular.z);
			out.linear.x *= r;
			out.angular.z *= r;
			col = true;
		}
		pub_cloud.publish(col_points);
		if(col)
		{
			ROS_WARN("Safety Limit: (%0.2f, %0.2f)->(%0.2f, %0.2f)",
					in.linear.x, in.angular.z,
					out.linear.x, out.angular.z);
		}
		return out;
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub_twist;
	ros::Publisher pub_cloud;
	ros::Publisher pub_debug;
	ros::Subscriber sub_twist;
	ros::Subscriber sub_cloud;
	ros::Subscriber sub_disable;
	tf::TransformListener tfl;

	geometry_msgs::Twist twist;
	sensor_msgs::PointCloud2 cloud;
	double hz;
	double timeout;
	double disable_timeout;
	double vel[2];
	double acc[2];
	double tmax;
	double dt;
	double tmargin;
	double z_range[2];
	float footprint_radius;
	double downsample_grid;
	std::string frame_id;

	ros::Time last_disable_cmd;

	bool has_cloud;
	bool has_twist;

	bool XmlRpc_isNumber(XmlRpc::XmlRpcValue &value)
	{
		return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
			value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
	}
	class vec
	{
	public:
		float c[2];
		vec(const float x, const float y)
		{
			c[0] = x;
			c[1] = y;
		}
		vec()
		{
			c[0] = c[1] = 0.0;
		}
		float &operator[](const int &i)
		{
			return c[i];
		}
		const float &operator[](const int &i) const
		{
			return c[i];
		}
		vec operator-(const vec &a) const
		{
			vec out = *this;
			out[0] -= a[0];
			out[1] -= a[1];
			return out;
		}
		float cross(const vec &a) const
		{
			return (*this)[0] * a[1] - (*this)[1] * a[0];
		}
		float dot(const vec &a) const
		{
			return (*this)[0] * a[0] + (*this)[1] * a[1];
		}
		float dist(const vec &a) const
		{
			return hypotf((*this)[0] - a[0], (*this)[1] - a[1]);
		}
		float dist_line(const vec &a, const vec &b) const
		{
			return (b - a).cross((*this) - a) / b.dist(a);
		}
		float dist_linestrip(const vec &a, const vec &b) const
		{
			if((b - a).dot((*this) - a) <= 0) return this->dist(a);
			if((a - b).dot((*this) - b) <= 0) return this->dist(b);
			return fabs(this->dist_line(a, b));
		}
	};
	class polygon
	{
	public:
		std::vector<vec> v;
		void move(const float &x, const float &y, const float &yaw)
		{
			float cos_v = cosf(yaw);
			float sin_v = sinf(yaw);
			for(auto &p: v)
			{
				auto tmp = p;
				p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
				p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
			}
		}
		bool inside(const vec &a)
		{
			int cn = 0;
			for(int i = 0; i < (int)v.size() - 1; i ++)
			{
				auto &v1 = v[i];
				auto &v2 = v[i + 1];
				if((v1[1] <= a[1] && a[1] < v2[1]) ||
						(v2[1] <= a[1] && a[1] < v1[1]))
				{
					float lx;
					lx = v1[0] + (v2[0] - v1[0]) * (a[1] - v1[1]) / (v2[1] - v1[1]);
					if(a[0] < lx) cn ++;
				}
			}
			return ((cn & 1) == 1);
		}
		float dist(const vec &a)
		{
			float dist = FLT_MAX;
			for(int i = 0; i < (int)v.size() - 1; i ++)
			{
				auto &v1 = v[i];
				auto &v2 = v[i + 1];
				auto d = a.dist_linestrip(v1, v2);
				if(d < dist) dist = d;
			}
			return dist;
		}
	};

	polygon footprint_p;

	void cb_twist(const geometry_msgs::Twist::Ptr &msg)
	{
		twist = *msg;
		has_twist = true;
	}
	void cb_cloud(const sensor_msgs::PointCloud2::Ptr &msg)
	{
		cloud = *msg;
		has_cloud = true;
	}
	void cb_disable(const std_msgs::Bool::Ptr &msg)
	{
		if(msg->data)
		{
			last_disable_cmd = ros::Time::now();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "safety_limiter");

	safety_limiter limiter;
	limiter.spin();

	return 0;
}
 
