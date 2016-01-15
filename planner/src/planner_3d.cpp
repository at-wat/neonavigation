#include <ros/ros.h>
#include <costmap/CSpace3D.h>
#include <costmap/CSpace3DUpdate.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>

#include "grid_astar.hpp"


float signf(float a)
{
	if(a < 0) return -1;
	else if(a > 0) return 1;
	return 0;
}

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
		void param_cast(const std::string& param_name, 
				size_t& param_val, const size_t& default_val) const
		{
			int _default_val_d = (int)default_val;
			int _param_val_d;

			param<int>(param_name, _param_val_d, _default_val_d);
			param_val = (int)_param_val_d;
		}
		NodeHandle_f(const std::string& ns = std::string(), 
				const M_string& remappings = M_string()) : NodeHandle(ns, remappings)
		{
		}
	};
}

typedef grid_astar astar;
//typedef grid_astar<3, 2> astar;

class planner_3d
{
private:
	ros::NodeHandle_f nh;
	ros::Subscriber sub_map;
	ros::Subscriber sub_map_update;
	ros::Subscriber sub_goal;
	ros::Publisher pub_path;
	ros::Publisher pub_debug;

	tf::TransformListener tfl;

	astar as;
	astar::gridmap<char, 0x40> cm;
	astar::gridmap<char, 0x80> cm_rough;
	astar::gridmap<char, 0x80> cm_hyst;
	astar::gridmap<float> cost_estim_cache;
	
	astar::vecf euclid_cost_coef;
	float euclid_cost(const astar::vec &v, const astar::vecf coef)
	{
		auto vc = v;
		float cost = 0;
		for(int i = 0; i < noncyclic; i ++)
		{
			cost += powf(coef[i] * vc[i], 2.0);
		}
		cost = sqrtf(cost);
		for(int i = noncyclic; i < dim; i ++)
		{
			vc.cycle(vc[i], cm.size[i]);
			cost += fabs(coef[i] * vc[i]);
		}
		return cost;
	}
	float euclid_cost(const astar::vec &v)
	{
		return euclid_cost(v, euclid_cost_coef);
	}

	class rotation_cache
	{
	public:
		std::unique_ptr<astar::vecf[]> c;
		astar::vec size;
		int ser_size;
		void reset(const astar::vec &size)
		{
			size_t ser_size = 1;
			for(int i = 0; i < 3; i ++)
			{
				ser_size *= size[i];
			}
			this->size = size;
			this->ser_size = ser_size;

			c.reset(new astar::vecf[ser_size]);
		}
		rotation_cache(const astar::vec &size)
		{
			reset(size);
		}
		rotation_cache()
		{
		}
		astar::vecf &operator [](const astar::vec &pos)
		{
			size_t addr = pos[2];
			for(int i = 1; i >= 0; i --)
			{
				addr *= size[i];
				addr += pos[i];
			}
			return c[addr];
		}
	};
	std::vector<rotation_cache> rotgm;
	rotation_cache *rot_cache;

	costmap::MapMetaData3D map_info;
	std_msgs::Header map_header;
	float max_vel;
	float max_ang_vel;
	float freq;
	float freq_min;
	float search_range;
	int range;
	int local_range;
	double local_range_f;
	int unknown_cost;
	bool has_map;
	bool has_goal;
	std::vector<astar::vec> search_list;
	std::vector<astar::vec> search_list_rough;

	// Cost weights
	class cost_coeff_
	{
	public:
		float weight_decel;
		float weight_backward;
		float weight_ang_vel;
		float weight_costmap;
		float weight_hysteresis;
		float in_place_turn;
		float hysteresis_max_dist;
	} cc;

	geometry_msgs::PoseStamped start;
	geometry_msgs::PoseStamped goal;
	astar::vecf ec;
	astar::vecf ec_rough;
	astar::vecf resolution;

	void cb_goal(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		goal = *msg;

		has_goal = true;
		update_goal();
	}
	void fill_costmap(astar::reservable_priority_queue<astar::pq> &open,
			astar::gridmap<float> &g,
			const astar::vec &s, const astar::vec &e)
	{
		auto s_rough = s;
		s_rough[2] = 0;
		while(true)
		{
			if(open.size() < 1) break;
			const auto center = open.top();
			const auto p = center.v;
			const auto c = center.p_raw;
			open.pop();
			if(c > g[p]) continue;
			if(c - ec_rough[0] * (range + local_range) > g[s_rough]) continue;

			astar::vec d;
			d[2] = 0;

			const int range_rough = 4;
			for(d[0] = -range_rough; d[0] <= range_rough; d[0] ++)
			{
				for(d[1] = -range_rough; d[1] <= range_rough; d[1] ++)
				{
					if(d[0] == 0 && d[1] == 0) continue;
					if(d.sqlen() > range_rough * range_rough) continue;

					const auto next = p + d;
					if((unsigned int)next[0] >= (unsigned int)map_info.width ||
							(unsigned int)next[1] >= (unsigned int)map_info.height)
						continue;
					auto &gnext = g[next];
					if(gnext < 0) continue;

					float cost = 0;

					{
						float v[3], dp[3], sum = 0;
						float distf = d.len();
						const int dist = distf;
						distf /= dist;
						v[0] = p[0];
						v[1] = p[1];
						v[2] = 0;
						dp[0] = (float)d[0] / dist;
						dp[1] = (float)d[1] / dist;
						astar::vec pos(v);
						char c = 0;
						for(int i = 0; i < dist; i ++)
						{
							pos[0] = lroundf(v[0]);
							pos[1] = lroundf(v[1]);
							c = cm_rough[pos];
							if(c > 99) break;
							sum += c;
							v[0] += dp[0];
							v[1] += dp[1];
						}
						if(c > 99) continue;
						cost += sum * map_info.linear_resolution
							* distf * cc.weight_costmap / 100.0;
					}
					cost += euclid_cost(d, ec_rough);

					const auto gp = c + cost;
					if(gnext > gp)
					{
						gnext = gp;
						open.push(astar::pq(gp, gp, next));
					}
				}
			}
		}
	}
	void update_goal()
	{	
		if(!has_map) return;

		astar::vec s, e;
		metric2grid(s[0], s[1], s[2],
				start.pose.position.x, start.pose.position.y, 
				tf::getYaw(start.pose.orientation));
		metric2grid(e[0], e[1], e[2],
				goal.pose.position.x, goal.pose.position.y, 
				tf::getYaw(goal.pose.orientation));
		ROS_INFO("New goal received (%d, %d, %d)",
				e[0], e[1], e[2]);
		e[2] = 0;

		const auto ts = std::chrono::high_resolution_clock::now();
		astar::reservable_priority_queue<astar::pq> open;
		auto &g = cost_estim_cache;

		g.clear(FLT_MAX);

		g[e] = -ec_rough[0] * 0.5; // Decrement to reduce calculation error
		open.push(astar::pq(g[e], g[e], e));
		fill_costmap(open, g, s, e);
		const auto tnow = std::chrono::high_resolution_clock::now();
		ROS_INFO("Cost estimation cache generated (%0.3f sec.)",
				std::chrono::duration<float>(tnow - ts).count());
		g[e] = 0;
		
		cm_hyst.clear(0);

		publish_costmap();
	}
	void publish_costmap()
	{
		sensor_msgs::PointCloud debug;
		debug.header = map_header;
		debug.header.stamp = ros::Time::now();
		{
			astar::vec p;
			for(p[1] = 0; p[1] < cost_estim_cache.size[1]; p[1] ++)
			{
				for(p[0] = 0; p[0] < cost_estim_cache.size[0]; p[0] ++)
				{
					p[2] = 0;
					if(cost_estim_cache[p] < 0)
						cost_estim_cache[p] = FLT_MAX;
					if(cost_estim_cache[p] == FLT_MAX) continue;
					float x, y, yaw;
					grid2metric(p[0], p[1], p[2], x, y, yaw);
					geometry_msgs::Point32 point;
					point.x = x;
					point.y = y;
					//point.z = cm_hyst[p] * 0.01;
					point.z = cost_estim_cache[p] / 500;
					debug.points.push_back(point);
				}
			}
		}
		pub_debug.publish(debug);
	}
	void cb_map_update(const costmap::CSpace3DUpdate::ConstPtr &msg)
	{
		ROS_INFO("Map updated");
	
		astar::vec s, e;
		metric2grid(s[0], s[1], s[2],
				start.pose.position.x, start.pose.position.y, 
				tf::getYaw(start.pose.orientation));
		metric2grid(e[0], e[1], e[2],
				goal.pose.position.x, goal.pose.position.y, 
				tf::getYaw(goal.pose.orientation));
		e[2] = 0;

		{
			astar::vec p;
			astar::vec gp;
			gp[0] = msg->x;
			gp[1] = msg->y;
			gp[2] = msg->yaw;
			astar::vec gp_rough = gp;
			gp_rough[2] = 0;
			for(p[0] = 0; p[0] < (int)msg->width; p[0] ++)
			{
				for(p[1] = 0; p[1] < (int)msg->height; p[1] ++)
				{
					int cost_min = 100;
					for(p[2] = 0; p[2] < (int)msg->angle; p[2] ++)
					{
						const size_t addr = ((p[2] * msg->height) + p[1])
							* msg->width + p[0];
						char c = msg->data[addr];
						if(c < 0) c = unknown_cost;
						cm[gp + p] = c;
						if(c < cost_min) cost_min = c;
					}
					p[2] = 0;
					cm_rough[gp_rough + p] = cost_min;
				}
			}
		}

		const auto ts = std::chrono::high_resolution_clock::now();
		astar::reservable_priority_queue<astar::pq> open;
		auto &g = cost_estim_cache;

		astar::vec p;
		p[2] = 0;
		for(p[1] = (int)msg->y; p[1] < (int)(msg->y + msg->height); p[1] ++)
		{
			for(p[0] = (int)msg->x; p[0] < (int)(msg->x + msg->width); p[0] ++)
			{
				g[p] = FLT_MAX;
			}
		}
		for(p[1] = (int)msg->y; p[1] < (int)(msg->y + msg->height); p[1] ++)
		{
			if((int)msg->x - 1 >= 0)
			{
				p[0] = msg->x - 1;
				auto &gp = g[p];
				if(gp < FLT_MAX)
					open.push(astar::pq(gp, gp, p));
			}
			if(msg->x + msg->width < map_info.width)
			{
				p[0] = msg->x + msg->width;
				auto &gp = g[p];
				if(gp < FLT_MAX)
					open.push(astar::pq(gp, gp, p));
			}
		}
		for(p[0] = (int)msg->x; p[0] < (int)(msg->x + msg->width); p[0] ++)
		{
			if((int)msg->y - 1 >= 0)
			{
				p[1] = msg->y - 1;
				auto &gp = g[p];
				if(gp < FLT_MAX)
					open.push(astar::pq(gp, gp, p));
			}
			if(msg->y + msg->height < map_info.height)
			{
				p[1] = msg->y + msg->height;
				auto &gp = g[p];
				if(gp < FLT_MAX)
					open.push(astar::pq(gp, gp, p));
			}
		}
		fill_costmap(open, g, s, e);
		const auto tnow = std::chrono::high_resolution_clock::now();
		ROS_INFO("Cost estimation cache updated (%0.3f sec.)",
				std::chrono::duration<float>(tnow - ts).count());
		publish_costmap();
	}
	void cb_map(const costmap::CSpace3D::ConstPtr &msg)
	{
		ROS_INFO("Map received");
		ROS_INFO(" linear_resolution %0.2f x (%dx%d) px", msg->info.linear_resolution,
				msg->info.width, msg->info.height);
		ROS_INFO(" angular_resolution %0.2f x %d px", msg->info.angular_resolution,
				msg->info.angle);
		ROS_INFO(" origin %0.3f m, %0.3f m, %0.3f rad", 
				msg->info.origin.position.x,
				msg->info.origin.position.y,
				tf::getYaw(msg->info.origin.orientation));

		float ec_val[3] = {1.0f / max_vel, 
			1.0f / max_vel, 
			1.0f * cc.weight_ang_vel / max_ang_vel};

		ec = astar::vecf(ec_val);
		ec_val[2] = 0;
		ec_rough = astar::vecf(ec_val);

		if(map_info.linear_resolution != msg->info.linear_resolution ||
				map_info.angular_resolution != msg->info.angular_resolution)
		{
			astar::vec d;
			range = (int)(search_range / msg->info.linear_resolution);

			search_list.clear();
			for(d[0] = -range; d[0] <= range; d[0] ++)
			{
				for(d[1] = -range; d[1] <= range; d[1] ++)
				{
					if(d.sqlen() > range * range) continue;
					for(d[2] = 0; d[2] < (int)msg->info.angle; d[2] ++)
					{
						search_list.push_back(d);
					}
				}
			}
			search_list_rough.clear();
			for(d[0] = -range; d[0] <= range; d[0] ++)
			{
				for(d[1] = -range; d[1] <= range; d[1] ++)
				{
					if(d.sqlen() > range * range) continue;
					d[2] = 0;
					search_list_rough.push_back(d);
				}
			}
			ROS_INFO("Search list updated (range: ang %d, lin %d) %d", 
					msg->info.angle, range, (int)search_list.size());

			rotgm.resize(msg->info.angle);
			for(int i = 0; i < (int)msg->info.angle; i ++)
			{
				const int size[3] = {range * 2 + 1, range * 2 + 1, (int)msg->info.angle};
				auto &r = rotgm[i];
				r.reset(astar::vec(size));

				astar::vec d;

				for(d[0] = 0; d[0] <= range * 2; d[0] ++)
				{
					for(d[1] = 0; d[1] <= range * 2; d[1] ++)
					{
						for(d[2] = 0; d[2] < (int)msg->info.angle; d[2] ++)
						{
							const float val[3] = {
								(d[0] - range) * msg->info.linear_resolution, 
								(d[1] - range) * msg->info.linear_resolution, 
								d[2] * msg->info.angular_resolution};
							auto v = astar::vecf(val);
							rotate(v, -i * msg->info.angular_resolution);
							r[d] = v;
						}
					}
				}
			}
			ROS_INFO("Rotation cache generated");
		}
		map_info = msg->info;
		map_header = msg->header;

		resolution[0] = 1.0 / map_info.linear_resolution;
		resolution[1] = 1.0 / map_info.linear_resolution;
		resolution[2] = 1.0 / map_info.angular_resolution;

		local_range = lroundf(local_range_f / map_info.linear_resolution);

		int size[3] = {(int)map_info.width, (int)map_info.height, (int)map_info.angle};
		as.reset(astar::vec(size));
		cm.reset(astar::vec(size));
		size[2] = 1;
		cost_estim_cache.reset(astar::vec(size));
		cm_rough.reset(astar::vec(size));
		cm_hyst.reset(astar::vec(size));

		astar::vec p;
		for(p[0] = 0; p[0] < (int)map_info.width; p[0] ++)
		{
			for(p[1] = 0; p[1] < (int)map_info.height; p[1] ++)
			{
				int cost_min = 100;
				for(p[2] = 0; p[2] < (int)map_info.angle; p[2] ++)
				{
					const size_t addr = ((p[2] * size[1]) + p[1]) * size[0] + p[0];
					char c = msg->data[addr];
					if(c < 0) c = unknown_cost;
					cm[p] = c;
					if(c < cost_min) cost_min = c;
				}
				p[2] = 0;
				cm_rough[p] = cost_min;
			}
		}
		ROS_INFO("Map copied");
		cm_hyst.clear(0);

		has_map = true;
		update_goal();
	}

public:
	planner_3d():
		nh("~")
	{
		sub_map = nh.subscribe("costmap", 1, &planner_3d::cb_map, this);
		sub_map_update = nh.subscribe("costmap_update", 1, &planner_3d::cb_map_update, this);
		sub_goal = nh.subscribe("goal", 1, &planner_3d::cb_goal, this);
		pub_path = nh.advertise<nav_msgs::Path>("path", 1, true);
		pub_debug = nh.advertise<sensor_msgs::PointCloud>("debug", 1, true);

		nh.param_cast("freq", freq, 4.0f);
		nh.param_cast("freq_min", freq_min, 2.0f);
		nh.param_cast("search_range", search_range, 0.4f);

		nh.param_cast("max_vel", max_vel, 0.3f);
		nh.param_cast("max_ang_vel", max_ang_vel, 0.6f);

		nh.param_cast("weight_decel", cc.weight_decel, 50.0f);
		nh.param_cast("weight_backward", cc.weight_backward, 0.9f);
		nh.param_cast("weight_ang_vel", cc.weight_ang_vel, 1.0f);
		nh.param_cast("weight_costmap", cc.weight_costmap, 50.0f);
		nh.param_cast("cost_in_place_turn", cc.in_place_turn, 6.0f);
		nh.param_cast("hysteresis_max_dist", cc.hysteresis_max_dist, 0.3f);
		nh.param_cast("weight_hysteresis", cc.weight_hysteresis, 5.0f);
		
		nh.param("unknown_cost", unknown_cost, 100);
		
		nh.param("local_range", local_range_f, 8.0);

		int queue_size_limit;
		nh.param("queue_size_limit", queue_size_limit, 0);
		as.set_queue_size_limit(queue_size_limit);

		has_map = false;
		has_goal = false;
	}
	void spin()
	{
		ros::Rate wait(freq);
		ROS_INFO("Initialized");

		while(ros::ok())
		{
			wait.sleep();
			ros::spinOnce();
			start.header.frame_id = "base_link";
			start.header.stamp = ros::Time::now();
			start.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			start.pose.position.x = 0;
			start.pose.position.y = 0;
			start.pose.position.z = 0;
			try
			{
				tfl.waitForTransform("base_link", map_header.frame_id, 
						map_header.stamp, ros::Duration(0.1));
				tfl.transformPose(map_header.frame_id, start, start);
			}
			catch(tf::TransformException &e)
			{
				ROS_INFO("Transform failed");
				continue;
			}

			if(has_map && has_goal)
			{
				nav_msgs::Path path;
				make_plan(start.pose, goal.pose, path, true);
				pub_path.publish(path);
			}
		}
	}

private:
	void grid2metric(
			const int x, const int y, const int yaw,
			float &gx, float &gy, float &gyaw)
	{
		gx = x * map_info.linear_resolution + map_info.origin.position.x;
		gy = y * map_info.linear_resolution + map_info.origin.position.y;
		gyaw = yaw * map_info.angular_resolution;
	}
	void grid2metric(const std::list<astar::vec> &path_grid, 
			nav_msgs::Path &path, const astar::vec &v_start)
	{
		path.header = map_header;
		path.header.stamp = ros::Time::now();

		//static int cnt = 0;
		//cnt ++;
		float x_ = 0, y_ = 0, yaw_ = 0;
		astar::vec p_;
		bool init = false;
		for(auto &p: path_grid)
		{
			float x, y, yaw;
			grid2metric(p[0], p[1], p[2], x, y, yaw);
			geometry_msgs::PoseStamped ps;
			ps.header = path.header;
		/*	if(cnt % 2 == 0)
			{
				ps.pose.position.x = x;
				ps.pose.position.y = y;
				ps.pose.position.z = 0;
				ps.pose.orientation =
					tf::createQuaternionMsgFromYaw(yaw);
				path.poses.push_back(ps);
				continue;
			}*/

			//printf("%d %d %d  %f\n", p[0], p[1], p[2], as.g[p]);
			if(init)
			{
				auto ds = v_start - p;
				auto d = p - p_;
				float diff_val[3] = {
					d[0] * map_info.linear_resolution, 
					d[1] * map_info.linear_resolution, 
					p[2] * map_info.angular_resolution};
				astar::vecf motion_(diff_val);
				rotate(motion_, -p_[2] * map_info.angular_resolution);

				float inter = 0.1 / d.len();

				const astar::vecf motion = motion_;
				float cos_v = cosf(motion[2]);
				float sin_v = sinf(motion[2]);
				if(d[0] == 0 && d[1] == 0)
				{
					ps.pose.position.x = x;
					ps.pose.position.y = y;
					ps.pose.position.z = 0;
					ps.pose.orientation =
						tf::createQuaternionMsgFromYaw(yaw);
					path.poses.push_back(ps);
				}
				else if(fabs(sin_v) < 0.001 ||
						ds.sqlen() > local_range * local_range)
				{
					for(float i = 0; i < 1.0; i += inter)
					{
						float x2, y2, yaw2;
						x2 = x_ * (1 - i) + x * i;
						y2 = y_ * (1 - i) + y * i;
						yaw2 = yaw_ * (1 - i) + yaw * i;
						ps.pose.position.x = x2;
						ps.pose.position.y = y2;
						ps.pose.position.z = 0;
						ps.pose.orientation =
							tf::createQuaternionMsgFromYaw(yaw2);
						path.poses.push_back(ps);
					}
				}
				else
				{
					float r1 = motion[1] + motion[0] * cos_v / sin_v;
					float r2 = sqrtf(powf(motion[0], 2.0) + powf(motion[0] * cos_v / sin_v, 2.0));
					if(motion[0] * sin_v < 0) r2 = -r2;

					float cx, cy, cx_, cy_, dyaw;
					dyaw = yaw - yaw_;
					if(dyaw < -M_PI) dyaw += 2 * M_PI;
					else if(dyaw > M_PI) dyaw -= 2 * M_PI;

					cx = x + r2 * cosf(yaw + M_PI/2);
					cy = y + r2 * sinf(yaw + M_PI/2);
					cx_ = x_ + r1 * cosf(yaw_ + M_PI/2);
					cy_ = y_ + r1 * sinf(yaw_ + M_PI/2);
						
					/*ps.pose.position.x = cx;
					ps.pose.position.y = cy;
					ps.pose.position.z = 0;
					ps.pose.orientation =
						tf::createQuaternionMsgFromYaw(0);
					path.poses.push_back(ps);*/

					for(float i = 0; i < 1.0; i += inter)
					{
						float r = r1 * (1.0 - i) + r2 * i;
						float cx2 = cx_ * (1.0 - i) + cx * i;
						float cy2 = cy_ * (1.0 - i) + cy * i;
						float cyaw = yaw_ + i * dyaw;

						float x2, y2, yaw2;
						x2 = cx2 - r * cosf(cyaw + M_PI/2);
						y2 = cy2 - r * sinf(cyaw + M_PI/2);
						yaw2 = cyaw;
						ps.pose.position.x = x2;
						ps.pose.position.y = y2;
						ps.pose.position.z = 0;
						ps.pose.orientation =
							tf::createQuaternionMsgFromYaw(yaw2);
						path.poses.push_back(ps);
					}
					/*ps.pose.position.x = cx_;
					ps.pose.position.y = cy_;
					ps.pose.position.z = 0;
					ps.pose.orientation =
						tf::createQuaternionMsgFromYaw(0);
					path.poses.push_back(ps);*/
					ps.pose.position.x = x;
					ps.pose.position.y = y;
					ps.pose.position.z = 0;
					ps.pose.orientation =
						tf::createQuaternionMsgFromYaw(yaw);
					path.poses.push_back(ps);
				}
			}

			x_ = x;
			y_ = y;
			yaw_ = yaw;
			p_ = p;
			init = true;
		}
	}
	void metric2grid(
			int &x, int &y, int &yaw,
			const float gx, const float gy, const float gyaw)
	{
		x = lroundf((gx - map_info.origin.position.x) / map_info.linear_resolution);
		y = lroundf((gy - map_info.origin.position.y) / map_info.linear_resolution);
		yaw = lroundf(gyaw / map_info.angular_resolution);
	}
	bool make_plan(const geometry_msgs::Pose &gs, const geometry_msgs::Pose &ge, 
			nav_msgs::Path &path, bool hyst)
	{
		astar::vec s, e;
		metric2grid(s[0], s[1], s[2],
				gs.position.x, gs.position.y, tf::getYaw(gs.orientation));
		metric2grid(e[0], e[1], e[2],
				ge.position.x, ge.position.y, tf::getYaw(ge.orientation));

		auto s_rough = s;
		s_rough[2] = 0;

		if(cost_estim_cache[s_rough] == FLT_MAX)
		{
			ROS_WARN("Goal unreachable");
			return false;
		}
		auto range_limit = cost_estim_cache[s_rough]
			- (local_range + range) * ec[0];

		//ROS_INFO("Planning from (%d, %d, %d) to (%d, %d, %d)",
		//		s[0], s[1], s[2], e[0], e[1], e[2]);
		std::list<astar::vec> path_grid;
		//const auto ts = std::chrono::high_resolution_clock::now();
		if(!as.search(s, e, path_grid, 
				std::bind(&planner_3d::cb_cost, 
					this, std::placeholders::_1, std::placeholders::_2, 
					std::placeholders::_3, std::placeholders::_4, hyst), 
				std::bind(&planner_3d::cb_cost_estim, 
					this, std::placeholders::_1, std::placeholders::_2), 
				std::bind(&planner_3d::cb_search, 
					this, std::placeholders::_1,
					std::placeholders::_2, std::placeholders::_3), 
				std::bind(&planner_3d::cb_progress, 
					this, std::placeholders::_1),
				range_limit,
				1.0f / freq_min))
		{
			ROS_WARN("Path plan failed (goal unreachable)");
			return false;
		}
		//const auto tnow = std::chrono::high_resolution_clock::now();
		//ROS_INFO("Path found (%0.3f sec.)",
		//		std::chrono::duration<float>(tnow - ts).count());

		grid2metric(path_grid, path, s);

		if(hyst)
		{
			std::unordered_map<astar::vec, bool, astar::vec> path_points;
			float max_dist = cc.hysteresis_max_dist / map_info.linear_resolution;
			int path_range = range + max_dist + 1;
			for(auto &p: path_grid)
			{
				astar::vec d;
				for(d[0] = -path_range; d[0] <= path_range; d[0] ++)
				{
					for(d[1] = -path_range; d[1] <= path_range; d[1] ++)
					{
						auto point = p + d;
						point[2] = 0;
						if((unsigned int)point[0] >= (unsigned int)map_info.width ||
								(unsigned int)point[1] >= (unsigned int)map_info.height)
							continue;
						path_points[point] = true;
					}
				}
			}

			cm_hyst.clear(100);
			//const auto ts = std::chrono::high_resolution_clock::now();
			for(auto &ps: path_points)
			{
				auto &p = ps.first;
				float d_min = FLT_MAX;
				auto it_prev = path_grid.begin();
				for(auto it = path_grid.begin(); it != path_grid.end(); it ++)
				{
					if(it != it_prev)
					{
						auto d = p.dist_linestrip(*it_prev, *it);
						if(d < d_min) d_min = d;
					}
					it_prev = it;
				}
				if(d_min < 0) d_min = 0;
				if(d_min > max_dist)
					d_min = max_dist;
				cm_hyst[p] = d_min * 100.0 / max_dist;
			}
			//const auto tnow = std::chrono::high_resolution_clock::now();
			//ROS_INFO("Hysteresis map generated (%0.3f sec.)",
			//		std::chrono::duration<float>(tnow - ts).count());
			publish_costmap();
		}

		return true;
	}
	bool rough;
	std::vector<astar::vec> &cb_search(
			const astar::vec& p,
			const astar::vec& s, const astar::vec& e)
	{
		const auto ds = s - p;
		rot_cache = &rotgm[p[2]];
		
		if(ds.sqlen() < local_range * local_range)
		{
			rough = false;
			euclid_cost_coef = ec;
			return search_list;
		}
		rough = true;
		euclid_cost_coef = ec_rough;
		return search_list_rough;
	}
	bool cb_progress(const std::list<astar::vec>& path_grid)
	{
		nav_msgs::Path path;
		path.header = map_header;
		path.header.stamp = ros::Time::now();
		//grid2metric(path_grid, path);
		pub_path.publish(path);
		ROS_INFO("Search timed out");
		return true;
	}
	void rotate(astar::vecf &v, const float &ang)
	{
		const astar::vecf tmp = v;
		const float cos_v = cosf(ang);
		const float sin_v = sinf(ang);

		v[0] = cos_v * tmp[0] - sin_v * tmp[1];
		v[1] = sin_v * tmp[0] + cos_v * tmp[1];
		v[2] = v[2] + ang;
		if(v[2] > M_PI) v[2] -= 2 * M_PI;
		else if(v[2] < -M_PI) v[2] += 2 * M_PI;
	}
	float cb_cost_estim(const astar::vec &s, const astar::vec &e)
	{
		auto s2 = s;
		s2[2] = 0;
		auto cost = cost_estim_cache[s2];
		if(!rough)
		{
			cost += ec_rough[2] * fabs(s[2]);
		}
		return cost;
	}
	float cb_cost(const astar::vec &s, astar::vec &e,
			const astar::vec &v_goal,
			const astar::vec &v_start,
			const bool hyst)
	{
		const auto d = e - s;
		float cost = euclid_cost(d);

		if(rough)
		{
			// Snap
			if((e - v_goal).sqlen() < range * range) e = v_goal;

			// Go-straight
			float v[3], dp[3];
			int sum = 0, sum_hyst = 0;
			float distf = d.len();
			const int dist = distf;
			distf /= dist;
			v[0] = s[0];
			v[1] = s[1];
			v[2] = 0;
			dp[0] = (float)d[0] / dist;
			dp[1] = (float)d[1] / dist;
			astar::vec pos(v);
			for(int i = 0; i < dist; i ++)
			{
				pos[0] = lroundf(v[0]);
				pos[1] = lroundf(v[1]);
				const auto c = cm_rough[pos];
				if(c > 99) return -1;
				sum += c;
				if(hyst)
				{
					sum_hyst += cm_hyst[pos];
				}
				v[0] += dp[0];
				v[1] += dp[1];
			}
			if(e[0] == v_goal[0] && e[1] == v_goal[1])
			{
				e[2] = v_goal[2];
			}
			else
			{
				e[2] = lroundf(atan2f(d[1], d[0]) / map_info.angular_resolution);
				if(e[2] < 0) e[2] += map_info.angle;
			}
			cost += sum * map_info.linear_resolution * distf * cc.weight_costmap / 100.0;
			cost += sum_hyst * map_info.linear_resolution * distf * cc.weight_hysteresis / 100.0;
			return cost;
		}
		if(d[0] == 0 && d[1] == 0)
		{
			// In-place turn
			return cc.in_place_turn;
		}

		/*float diff_val[3] = {
			d[0] * map_info.linear_resolution, 
			d[1] * map_info.linear_resolution, 
			e[2] * map_info.angular_resolution};
		astar::vecf motion(diff_val);
		rotate(motion, -s[2] * map_info.angular_resolution);*/
		astar::vec d2;
		d2[0] = d[0] + range;
		d2[1] = d[1] + range;
		d2[2] = e[2];
		const astar::vecf motion = (*rot_cache)[d2];
		
		const astar::vecf motion_grid = motion * resolution;
		//motion_grid[0] /= map_info.linear_resolution;
		//motion_grid[1] /= map_info.linear_resolution;
		//motion_grid[2] /= map_info.angular_resolution;

		if(lroundf(motion_grid[0]) == 0 && lroundf(motion_grid[1]) != 0)
		{
			// Not non-holonomic
			return -1;
		}
		if(lroundf(motion_grid[2]) == 0 && lroundf(motion_grid[1]) != 0)
		{
			// Drifted
			return -1;
		}

		if(fabs(motion[2]) >= 2.0 * M_PI / 4.0)
		{
			// Over 90 degree turn
			// must be separated into two curves
			return -1;
		}

		const float dist = motion.len();

		const float cos_v = cosf(motion[2]);
		const float sin_v = sinf(motion[2]);

		bool forward(true);
		if(motion[0] < 0) forward = false;

		if(!forward)
		{
			cost *= 1.0 + cc.weight_backward;
		}

		if(lroundf(motion_grid[2]) == 0)
		{
			float distf = d.len();

			astar::vecf vg(s);
			const float yaw = s[2] * map_info.angular_resolution;
			vg[0] += cosf(yaw) * distf;
			vg[1] += sinf(yaw) * distf;
			if((vg - astar::vecf(e)).len() >= sinf(map_info.angular_resolution)) return -1;

			// Go-straight
			float v[3], dp[3];
			int sum = 0, sum_hyst = 0;
			const int dist = distf;
			distf /= dist;
			v[0] = s[0];
			v[1] = s[1];
			v[2] = s[2];
			dp[0] = (float)d[0] / dist;
			dp[1] = (float)d[1] / dist;
			astar::vec pos(v);
			for(int i = 0; i < dist; i ++)
			{
				pos[0] = lroundf(v[0]);
				pos[1] = lroundf(v[1]);
				const auto c = cm[pos];
				if(c > 99) return -1;
				if(hyst)
				{
					auto pos_rough = pos;
					pos_rough[2] = 0;
					sum_hyst += cm_hyst[pos_rough];
				}
				sum += c;
				v[0] += dp[0];
				v[1] += dp[1];
			}
			cost += sum * map_info.linear_resolution * distf * cc.weight_costmap / 100.0;
			cost += sum_hyst * map_info.linear_resolution * distf * cc.weight_hysteresis / 100.0;
		}
		else
		{
			// Curve
			if(motion[0] * motion[1] * motion[2] < 0)
				return -1;
			if(d.sqlen() < 3 * 3) return -1;
			if(fabs(motion[1]) <= map_info.linear_resolution * 0.5) return -1;


			const float r1 = motion[1] + motion[0] * cos_v / sin_v;
			float r2 = sqrtf(powf(motion[0], 2.0) + powf(motion[0] * cos_v / sin_v, 2.0));
			if(motion[0] * sin_v < 0) r2 = -r2;

			// curveture at the start pose and the end pose must be same
			if(fabs(r1 - r2) >= map_info.linear_resolution)
			{
				// Drifted
				return -1;
			}

			const float curv_radius = (r1 + r2) / 2;

			float vel = max_vel;
			float ang_vel = cos_v * vel / (cos_v * motion[0] + sin_v * motion[1]);
			if(fabs(ang_vel) > max_ang_vel)
			{
				ang_vel = signf(ang_vel) * max_ang_vel;
				vel = fabs(curv_radius) * max_ang_vel;

				// Curve deceleration penalty
				cost += dist * fabs(vel / max_vel) * cc.weight_decel;
			}

			{
				float v[3], dp[3];
				int sum = 0, sum_hyst = 0;
				float distf = d.len();
				const int dist = distf;
				distf /= dist;
				v[0] = s[0];
				v[1] = s[1];
				v[2] = s[2];
				dp[0] = (float)d[0] / dist;
				dp[1] = (float)d[1] / dist;
				dp[2] = (float)d[2];
				if(dp[2] < -map_info.angle / 2) dp[2] += map_info.angle;
				else if(dp[2] >= map_info.angle / 2) dp[2] -= map_info.angle;
				dp[2] /= dist;
				astar::vec pos(v);
				for(int i = 0; i < dist; i ++)
				{
					pos[0] = lroundf(v[0]);
					pos[1] = lroundf(v[1]);
					pos[2] = lroundf(v[2]);
					if(pos[2] < 0) pos[2] += map_info.angle;
					else if(pos[2] >= (int)map_info.angle) pos[2] -= map_info.angle;
					const auto c = cm[pos];
					if(c > 99) return -1;
					sum += c;
					if(hyst)
					{
						auto pos_rough = pos;
						pos_rough[2] = 0;
						sum_hyst += cm_hyst[pos_rough];
					}
					v[0] += dp[0];
					v[1] += dp[1];
					v[2] += dp[2];
				}
				cost += sum * map_info.linear_resolution * distf * cc.weight_costmap / 100.0;
				cost += sum_hyst * map_info.linear_resolution * distf * cc.weight_hysteresis / 100.0;
			}
		}

		return cost;
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "planner_3d");
	
	planner_3d jy;
	jy.spin();

	return 0;
}


