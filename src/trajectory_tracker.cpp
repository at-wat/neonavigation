#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

class tracker
{
public:
	tracker();
	~tracker();
	void spin();
private:
	std::string topicPath;
	std::string topicCmdVel;
	std::string frameRobot;
	double hz;
	double lookForward;
	double curvForward;
	double k[3];
	double d_lim;
	double vel[2];
	double acc[2];
	double w;
	double v;
	double dec;
	int pathStep;

	ros::NodeHandle nh;
	ros::Subscriber subPath;
	ros::Publisher pubVel;
	tf::TransformListener tf;

	nav_msgs::Path path;

	void cbPath(const nav_msgs::Path::ConstPtr& msg);
	void control();
};

template<typename T>
class average
{
public:
	average()
	{
		num = 0;
	};
	void operator +=(const T &val)
	{
		sum += val;
		num ++;
	};
	operator T()
	{
		if(num == 0) return 0;
		return sum / num;
	};
private:
	T sum;
	int num;
};

tracker::tracker():
	nh("~")
{
	nh.param("frame_robot", frameRobot, std::string("base_link"));
	nh.param("path", topicPath, std::string("path"));
	nh.param("cmd_vel", topicCmdVel, std::string("cmd_vel"));
	nh.param("hz", hz, 50.0);
	nh.param("look_forward", lookForward, 1.0);
	nh.param("curv_forward", curvForward, 0.5);
	nh.param("k_dist", k[0], 1.0);
	nh.param("k_ang", k[1], 1.0);
	nh.param("k_avel", k[2], 1.0);
	nh.param("k_dcel", dec, 0.2);
	nh.param("dist_lim", d_lim, 0.5);
	nh.param("max_vel", vel[0], 0.5);
	nh.param("max_angvel", vel[1], 1.0);
	nh.param("max_acc", acc[0], 1.0);
	nh.param("max_angacc", acc[1], 2.0);
	nh.param("path_step", pathStep, 2);

	subPath = nh.subscribe(topicPath, 200, &tracker::cbPath, this);
	pubVel = nh.advertise<geometry_msgs::Twist>(topicCmdVel, 10);
}
tracker::~tracker()
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	pubVel.publish(cmd_vel);
}

float dist2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	return sqrtf(powf(a.x-b.x,2) + powf(a.y-b.y,2));
}
float len2d(geometry_msgs::Point &a)
{
	return sqrtf(powf(a.x,2) + powf(a.y,2));
}
float len2d(geometry_msgs::Point a)
{
	return sqrtf(powf(a.x,2) + powf(a.y,2));
}
float curv3p(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	float ret;
	ret = 2 * (a.x*b.y + b.x*c.y + c.x*a.y - a.x*c.y - b.x*a.y - c.x*b.y);
	ret /= sqrtf( (powf(b.x-a.x, 2) + powf(b.y-a.y, 2)) * (powf(b.x-c.x, 2) + powf(b.y-c.y, 2)) * (powf(c.x-a.x, 2) + powf(c.y-a.y, 2)) );

	return ret;
}
float cross2d(geometry_msgs::Point &a, geometry_msgs::Point &b) 
{
	return a.x*b.y - a.y*b.x;
}
float cross2d(geometry_msgs::Point a, geometry_msgs::Point b) 
{
	return a.x*b.y - a.y*b.x;
}
float dot2d(geometry_msgs::Point &a, geometry_msgs::Point &b) 
{
	return a.x*b.x + a.y*b.y;
}
float dot2d(geometry_msgs::Point a, geometry_msgs::Point b) 
{
	return a.x*b.x + a.y*b.y;
}
geometry_msgs::Point point2d(float x, float y)
{
	geometry_msgs::Point ret;
	ret.x = x;
	ret.y = y;
	return ret;
}
geometry_msgs::Point sub2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	geometry_msgs::Point ret;
	ret.x = a.x - b.x;
	ret.y = a.y - b.y;
	return ret;
}
float sign(float a)
{
	if(a < 0) return -1;
	return 1;
}
float dist2d_line(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	return (cross2d(sub2d(b, a), sub2d(c, a)) / dist2d(b, a));
}
float dist2d_linestrip(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	if(dot2d(sub2d(b, a), sub2d(c, a) ) <= 0) return dist2d(c, a);
	if(dot2d(sub2d(a, b), sub2d(c, b) ) <= 0) return dist2d(c, b);
	return fabs( dist2d_line(a, b, c) );
}
geometry_msgs::Point projection2d(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	float r = dot2d(sub2d(b, a), sub2d(c, a)) / pow(len2d(sub2d(b, a)), 2);
	geometry_msgs::Point ret;
	ret.x = b.x*r + a.x*(1-r);
	ret.y = b.y*r + a.y*(1-r);
	return ret;
}


void tracker::cbPath(const nav_msgs::Path::ConstPtr& msg)
{
	path = *msg;
}

void tracker::spin()
{
	ros::Rate loop_rate(hz);

	while(ros::ok())
	{
		control();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void tracker::control()
{
	if(path.header.frame_id.size() == 0 ||
			path.poses.size() < 3)
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
		pubVel.publish(cmd_vel);
		return;
	}
	nav_msgs::Path lpath;
	lpath.header = path.header;
	try
	{
		ros::Time now = ros::Time(0);
		tf.waitForTransform(frameRobot, path.header.frame_id, now, ros::Duration(0.2));

		for(int i = 0; i < path.poses.size(); i += pathStep)
		{
			geometry_msgs::PoseStamped pose;
			tf.transformPose(frameRobot, now, path.poses[i], path.header.frame_id, pose);
			lpath.poses.push_back(pose);
		}
	}
	catch (tf::TransformException &e)
	{
		ROS_WARN("TF exception: %s", e.what());
		return;
	}
	float minDist = FLT_MAX;
	int iclose = 0;
	geometry_msgs::Point origin;
	for(int i = 1; i < lpath.poses.size(); i ++)
	{
		float d = dist2d_linestrip(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position, origin);
		if(d < minDist)
		{
			minDist = d;
			iclose = i;
		}
	}
	if(iclose < 1) return;
	// Signed distance error
	float dist = dist2d_line(lpath.poses[iclose-1].pose.position, lpath.poses[iclose].pose.position, origin);
	
	// Angular error
	geometry_msgs::Point vec = sub2d(lpath.poses[iclose].pose.position, lpath.poses[iclose-1].pose.position);
	float angle = -atan2(vec.y, vec.x);
	
	// Curvature
	average<float> curv;
	geometry_msgs::Point posLine = projection2d(lpath.poses[iclose-1].pose.position, lpath.poses[iclose].pose.position, origin);
	float remain = dist2d(lpath.poses.back().pose.position, posLine);
	for(int i = iclose + 1; i < lpath.poses.size() - 1; i ++)
	{
		if(dist2d(lpath.poses[i].pose.position, posLine) > curvForward) break;
		if(i > 2)
			curv += curv3p(lpath.poses[i-2].pose.position, lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
		else
			curv += 0.0;
	}
	if(iclose + 1 >= path.poses.size())
	{
		remain = -dist2d(lpath.poses[iclose].pose.position, posLine);
	}
	printf("d=%.2f, th=%.2f, curv=%.2f\n", dist, angle, (float)curv);

	if(dist < -d_lim) dist = -d_lim;
	else if(dist > d_lim) dist = d_lim;

	float dt = 1/hz;
	float _v = v;
	
	v = sign(remain) * sqrtf(2 * fabs(remain) * acc[0] * 0.95);
	if(fabs(remain) < 0.3) v = 0;
	
	if(v > vel[0]) v = vel[0];
	else if(v < -vel[0]) v = -vel[0];
	if(v > _v + dt*acc[0]) v = _v + dt*acc[0];
	else if(v < _v - dt*acc[0]) v = _v - dt*acc[0];

	float wref = v * curv;
	float _w = w;
	
	w += dt * (-dist*k[0] -angle*k[1] -(w - wref)*k[2]);

	if(w > vel[1]) w = vel[1];
	else if(w < -vel[1]) w = -vel[1];
	if(w > _w + dt*acc[1]) w = _w + dt*acc[1];
	else if(w < _w - dt*acc[1]) w = _w - dt*acc[1];

	geometry_msgs::Twist cmd_vel;
	if(!std::isfinite(v)) v = 0;
	if(!std::isfinite(w)) w = 0;

	cmd_vel.linear.x = v;
	cmd_vel.angular.z = w;
	pubVel.publish(cmd_vel);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_tracker");

	tracker track;
	track.spin();

	return 0;
}
