#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>


static geometry_msgs::Vector3 operator*(const double &a, const geometry_msgs::Vector3 &vin)
{
	geometry_msgs::Vector3 vout;
	vout.x = vin.x * a;
	vout.y = vin.y * a;
	vout.z = vin.z * a;
	return vout;
}
static geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3 &a, 
		const geometry_msgs::Vector3 &b)
{
	geometry_msgs::Vector3 vout;
	vout.x = a.x + b.x;
	vout.y = a.y + b.y;
	vout.z = a.z + b.z;
	return vout;
}
static geometry_msgs::Point operator+(const geometry_msgs::Point &a, 
		const geometry_msgs::Vector3 &b)
{
	geometry_msgs::Point vout;
	vout.x = a.x + b.x;
	vout.y = a.y + b.y;
	vout.z = a.z + b.z;
	return vout;
}
geometry_msgs::Vector3 to_vector3(const geometry_msgs::Point &a)
{
	geometry_msgs::Vector3 vout;
	vout.x = a.x;
	vout.y = a.y;
	vout.z = a.z;
	return vout;
}
geometry_msgs::Vector3 cross(const geometry_msgs::Quaternion &q,
		const geometry_msgs::Vector3 &vin)
{
	geometry_msgs::Vector3 vout;
	vout.x = q.y * vin.z - q.z * vin.y;
	vout.y = q.z * vin.x - q.x * vin.z;
	vout.z = q.x * vin.y - q.y * vin.x;
	return vout;
}

class track_odometry
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_imu;
	ros::Publisher pub_odom;
	tf::TransformBroadcaster tf_broadcaster;
	nav_msgs::Odometry odom_prev;

	std::string base_link_id;

	sensor_msgs::Imu imu;
	double gyro_zero[3];

	void cb_imu(const sensor_msgs::Imu::Ptr &msg)
	{
		imu = *msg;
	}
	void cb_odom(const nav_msgs::Odometry::Ptr &msg)
	{
		nav_msgs::Odometry odom = *msg;

		odom.twist.twist.angular = imu.angular_velocity;
		odom.pose.pose.orientation = imu.orientation;
		if(fabs(msg->twist.twist.angular.z) > 0.01)
		{
			odom.twist.twist.linear.x =
				msg->twist.twist.linear.x * imu.angular_velocity.z / msg->twist.twist.angular.z;
		}

		geometry_msgs::Vector3 v, t;
		t = 2.0 * cross(odom.pose.pose.orientation, odom.twist.twist.linear);
		v = odom.twist.twist.linear 
			+ odom.pose.pose.orientation.w * t 
			+ cross(odom.pose.pose.orientation, t);

		double dt = (odom.header.stamp - odom_prev.header.stamp).toSec();
		odom.pose.pose.position = odom_prev.pose.pose.position + dt * v;

		pub_odom.publish(odom);
		odom_prev = odom;


		geometry_msgs::TransformStamped odom_trans;

		odom_trans.header = odom.header;
		if(base_link_id.size() > 0)
		{
			odom_trans.child_frame_id = base_link_id;
		}
		else
		{
			odom_trans.child_frame_id = odom.child_frame_id;
		}
		odom_trans.transform.translation = to_vector3(odom.pose.pose.position);
		odom_trans.transform.rotation = odom.pose.pose.orientation;
		tf_broadcaster.sendTransform(odom_trans);
	}
public:
	track_odometry():
		nh("~")
	{
		sub_odom = nh.subscribe("/odom_raw", 1, &track_odometry::cb_odom, this);
		sub_imu = nh.subscribe("/imu", 1, &track_odometry::cb_imu, this);
		pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 2);

		nh.param("base_link_id", base_link_id, std::string(""));
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "track_odometry");
	
	track_odometry odom;

	ros::spin();

	return 0;
}


