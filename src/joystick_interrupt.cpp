#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class joystick_interrupt
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_twist;
	ros::Subscriber sub_joy;
	ros::Publisher pub_twist;
	double linear_vel;
	double angular_vel;
	double timeout;
	int linear_axis;
	int angular_axis;
	int interrupt_button;
	ros::Time last_joy_msg;

	void cb_joy(const sensor_msgs::Joy::Ptr msg)
	{
		if(msg->buttons[interrupt_button])
		{
			last_joy_msg = ros::Time::now();

			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = msg->axes[linear_axis] * linear_vel;
			cmd_vel.linear.y = cmd_vel.linear.z = 0.0;
			cmd_vel.angular.z = msg->axes[angular_axis] * angular_vel;
			cmd_vel.angular.x = cmd_vel.angular.y = 0.0;
			pub_twist.publish(cmd_vel);
		}
	};
	void cb_twist(const geometry_msgs::Twist::Ptr msg)
	{
		if(ros::Time::now() - last_joy_msg > ros::Duration(timeout))
		{
			pub_twist.publish(*msg);
		}
	};

public:
	joystick_interrupt():
		nh("~")
	{
		sub_joy = nh.subscribe("/joy", 1, &joystick_interrupt::cb_joy, this);
		sub_twist = nh.subscribe("cmd_vel_input", 1, &joystick_interrupt::cb_twist, this);
		pub_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);

		nh.param("linear_vel", linear_vel, 0.5);
		nh.param("angular_vel", angular_vel, 0.8);
		nh.param("linear_axis", linear_axis, 1);
		nh.param("angular_axis", angular_axis, 0);
		nh.param("interrupt_button", interrupt_button, 6);
		nh.param("timeout", timeout, 0.5);
		last_joy_msg = ros::Time::now();
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "joystick_interrupt");
	
	joystick_interrupt jy;
	ros::spin();

	return 0;
}


