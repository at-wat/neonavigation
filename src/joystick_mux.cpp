#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <topic_tools/shape_shifter.h>

class joystick_mux
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_topics[2];
	ros::Subscriber sub_joy;
	ros::Publisher pub_topic;
	double timeout;
	int interrupt_button;
	ros::Time last_joy_msg;
	bool advertised;
	int selected;
	topic_tools::ShapeShifter buffer[2];
	bool received_initial;

	void cb_joy(const sensor_msgs::Joy::Ptr msg)
	{
		last_joy_msg = ros::Time::now();
		if(msg->buttons[interrupt_button])
		{
			selected = 1;
		}
		else
		{
			selected = 0;
		}
	};
	void cb_topic(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg, int id)
	{
		buffer[id] = *msg;
		if(id == 0) received_initial = true;

		if(selected == id)
		{
			if(!advertised)
			{
				advertised = true;
				pub_topic = msg->advertise(nh, "output", 1, false);
			}
			pub_topic.publish(*msg);
		}
	};

public:
	joystick_mux():
		nh("~")
	{
		sub_joy = nh.subscribe("/joy", 1, &joystick_mux::cb_joy, this);
		sub_topics[0] = nh.subscribe<topic_tools::ShapeShifter>("input0", 1, 
				boost::bind(&joystick_mux::cb_topic, this, _1, 0));
		sub_topics[1] = nh.subscribe<topic_tools::ShapeShifter>("input1", 1, 
				boost::bind(&joystick_mux::cb_topic, this, _1, 1));

		nh.param("interrupt_button", interrupt_button, 5);
		nh.param("timeout", timeout, 0.5);
		last_joy_msg = ros::Time::now();

		advertised = false;
		received_initial = false;
		selected = 0;
	}
	void spin()
	{
		ros::Rate wait(10);
		while(ros::ok())
		{
			wait.sleep();
			ros::spinOnce();
			if(ros::Time::now() - last_joy_msg > ros::Duration(timeout))
			{
				selected = 0;
				if(advertised && received_initial)
					pub_topic.publish(buffer[0]);
			}
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "joystick_mux");
	
	joystick_mux jy;
	jy.spin();

	return 0;
}


