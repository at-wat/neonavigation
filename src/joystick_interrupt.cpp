#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>


class joystick_interrupt
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_twist;
	ros::Subscriber sub_joy;
	ros::Publisher pub_twist;
	ros::Publisher pub_int;
	double linear_vel;
	double angular_vel;
	double timeout;
	int linear_axis;
	int angular_axis;
	int linear_axis2;
	int angular_axis2;
	int interrupt_button;
	ros::Time last_joy_msg;

	void cb_joy(const sensor_msgs::Joy::Ptr msg)
	{
		if(msg->buttons[interrupt_button])
		{
			last_joy_msg = ros::Time::now();

			float lin, ang;
			lin = msg->axes[linear_axis];
			ang = msg->axes[angular_axis];
			if(linear_axis2 >= 0)
			{
				if(fabs(msg->axes[linear_axis2]) > fabs(lin))
				{
					lin = msg->axes[linear_axis2];
				}
			}
			if(angular_axis2 >= 0)
			{
				if(fabs(msg->axes[angular_axis2]) > fabs(lin))
				{
					ang = msg->axes[angular_axis2];
				}
			}

			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = lin * linear_vel;
			cmd_vel.linear.y = cmd_vel.linear.z = 0.0;
			cmd_vel.angular.z = ang * angular_vel;
			cmd_vel.angular.x = cmd_vel.angular.y = 0.0;
			pub_twist.publish(cmd_vel);
		}
		else
		{
			last_joy_msg = ros::Time(0);
		}
	};
	void cb_twist(const geometry_msgs::Twist::Ptr msg)
	{
		std_msgs::Bool status;
		if(ros::Time::now() - last_joy_msg > ros::Duration(timeout))
		{
			pub_twist.publish(*msg);
			status.data = true;
		}
		else
		{
			status.data = false;
		}
		pub_int.publish(status);
	};

public:
	joystick_interrupt():
		nh("~")
	{
		sub_joy = nh.subscribe("/joy", 1, &joystick_interrupt::cb_joy, this);
		sub_twist = nh.subscribe("cmd_vel_input", 1, &joystick_interrupt::cb_twist, this);
		pub_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);
		pub_int = nh.advertise<std_msgs::Bool>("interrupt_status", 2);

		nh.param("linear_vel", linear_vel, 0.5);
		nh.param("angular_vel", angular_vel, 0.8);
		nh.param("linear_axis", linear_axis, 1);
		nh.param("angular_axis", angular_axis, 0);
		nh.param("linear_axis2", linear_axis2, -1);
		nh.param("angular_axis2", angular_axis2, -1);
		nh.param("interrupt_button", interrupt_button, 6);
		nh.param("timeout", timeout, 0.5);
		last_joy_msg = ros::Time(0);
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "joystick_interrupt");
	
	joystick_interrupt jy;
	ros::spin();

	return 0;
}


