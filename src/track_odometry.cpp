#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


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
	ros::Subscriber sub_reset_z;
	ros::Publisher pub_odom;
	tf::TransformBroadcaster tf_broadcaster;
	tf::TransformListener tf_listener;
	nav_msgs::Odometry odom_prev;

	std::string base_link_id;
	std::string base_link_projected_id;
	std::string base_link_id_overwrite;

	sensor_msgs::Imu imu;
	double gyro_zero[3];
	double z_filter;
	double tf_tolerance;

	bool has_imu;
	bool has_odom;

	void cb_reset_z(const std_msgs::Float32::Ptr &msg)
	{
		odom_prev.pose.pose.position.z = msg->data;
	}
	void cb_imu(const sensor_msgs::Imu::Ptr &msg)
	{
		if(base_link_id.size() == 0) return;

		imu.header = msg->header;
		try
		{
			tf_listener.waitForTransform(base_link_id, msg->header.frame_id, 
					msg->header.stamp, ros::Duration(0.1));
			
			geometry_msgs::Vector3Stamped vin, vout;
			vin.header = imu.header;
			vin.vector = msg->linear_acceleration;
			tf_listener.transformVector(base_link_id, vin, vout);
			imu.linear_acceleration = vout.vector;
			
			vin.header = imu.header;
			vin.vector = msg->angular_velocity;
			tf_listener.transformVector(base_link_id, vin, vout);
			imu.angular_velocity = vout.vector;

			tf::StampedTransform trans;
			tf_listener.lookupTransform(base_link_id, msg->header.frame_id, msg->header.stamp, trans);

			tf::Stamped<tf::Quaternion> qin, qout;
			geometry_msgs::QuaternionStamped qmin, qmout;
			qmin.header = imu.header;
			qmin.quaternion = msg->orientation;
			tf::quaternionStampedMsgToTF(qmin, qin);

			auto axis = qin.getAxis();
			auto angle = qin.getAngle();
			tf::Stamped<tf::Vector3> axis2;
			tf::Stamped<tf::Vector3> axis1;
			axis1.setData(axis);
			axis1.stamp_ = qin.stamp_;
			axis1.frame_id_ = qin.frame_id_;
			tf_listener.transformVector(base_link_id, axis1, axis2);

			qout.setData(tf::Quaternion(axis2, angle));
			qout.stamp_ = qin.stamp_;
			qout.frame_id_ = base_link_id;

			tf::quaternionStampedTFToMsg(qout, qmout);
			imu.orientation = qmout.quaternion;
			//ROS_INFO("%0.3f %s -> %0.3f %s", 
			//		tf::getYaw(qmin.quaternion), qmin.header.frame_id.c_str(),
			//		tf::getYaw(qmout.quaternion), qmout.header.frame_id.c_str());

			has_imu = true;
		}
		catch(tf::TransformException &e)
		{
			if(has_imu)
				ROS_ERROR("%s", e.what());
			has_imu = false;
			return;
		}
	}
	void cb_odom(const nav_msgs::Odometry::Ptr &msg)
	{
		nav_msgs::Odometry odom = *msg;
		if(has_odom)
		{
			if(base_link_id_overwrite.size() > 0)
			{
				base_link_id = base_link_id_overwrite;
			}
			else
			{
				base_link_id = odom.child_frame_id;
			}

			if(!has_imu) return;

			odom.header.stamp += ros::Duration(tf_tolerance);
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
			odom.pose.pose.position.z *= z_filter;

			pub_odom.publish(odom);

			geometry_msgs::TransformStamped odom_trans;

			odom_trans.header = odom.header;
			odom_trans.child_frame_id = base_link_id;
			odom_trans.transform.translation = to_vector3(odom.pose.pose.position);
			odom_trans.transform.rotation = odom.pose.pose.orientation;
			tf_broadcaster.sendTransform(odom_trans);

			odom_trans.child_frame_id = base_link_projected_id;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(
					tf::getYaw(odom_trans.transform.rotation));
			tf_broadcaster.sendTransform(odom_trans);
		}
		odom_prev = odom;
		has_odom = true;
	}
public:
	track_odometry():
		nh("~")
	{
		sub_odom = nh.subscribe("/odom_raw", 1, &track_odometry::cb_odom, this);
		sub_imu = nh.subscribe("/imu", 1, &track_odometry::cb_imu, this);
		sub_reset_z = nh.subscribe("reset_z", 1, &track_odometry::cb_reset_z, this);
		pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 2);

		nh.param("base_link_id", base_link_id_overwrite, std::string(""));
		nh.param("base_link_projected_id", base_link_projected_id, std::string("base_link_projected"));
		nh.param("z_filter", z_filter, 0.99);
		nh.param("tf_tolerance", tf_tolerance, 0.01);

		has_imu = false;
		has_odom = false;
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "track_odometry");
	
	track_odometry odom;

	ros::spin();

	return 0;
}


