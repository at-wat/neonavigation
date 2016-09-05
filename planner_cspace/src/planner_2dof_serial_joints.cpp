#include <ros/ros.h>
#include <planner_cspace/PlannerStatus.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

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

typedef grid_astar<2, 0> astar;

class planner_2dof_serial_joints
{
private:
	ros::NodeHandle_f nh;
	ros::Publisher pub_status;

	tf::TransformListener tfl;

	astar as;
	astar::gridmap<char, 0x40> cm;

	astar::vecf euclid_cost_coef;
	
	float euclid_cost(const astar::vec &v, const astar::vecf coef)
	{
		auto vc = v;
		float cost = 0;
		for(int i = 0; i < as.get_dim(); i ++)
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

	float freq;
	float freq_min;
	bool has_goal;
	bool has_start;
	std::vector<astar::vec> search_list;
	int resolution;
	float weight_cost;

	class link_body
	{
	public:
		class vec3dof
		{
		public:
			float x;
			float y;
			float th;

			float dist(const vec3dof &b)
			{
				return hypotf(b.x - x, b.y - y);
			}
		};
	public:
		float radius[2];
		float length;
		vec3dof origin;
		vec3dof end(const float th) const
		{
			vec3dof e = origin;
			e.x += cosf(e.th + th) * length;
			e.y += sinf(e.th + th) * length;
			e.th += th;
			return e;
		}
		bool isCollide(const link_body b, const float th0, const float th1)
		{
			auto end0 = end(th0);
			auto end1 = b.end(th1);
			auto &end0r = radius[1];
			auto &end1r = b.radius[1];
			auto &origin0 = origin;
			auto &origin1 = b.origin;
			auto &origin0r = radius[0];
			auto &origin1r = b.radius[0];

			if(end0.dist(end1) < end0r + end1r) return true;
			if(end0.dist(origin1) < end0r + origin1r) return true;
			if(end1.dist(origin0) < end1r + origin0r) return true;

			// add side collision

			return false;
		}
	};
    
	planner_cspace::PlannerStatus status;

public:
	planner_2dof_serial_joints():
		nh("~")
	{
		pub_status = nh.advertise<planner_cspace::PlannerStatus>("status", 1, true);

		nh.param_cast("freq", freq, 4.0f);
		nh.param_cast("freq_min", freq_min, 2.0f);
		nh.param("resolution", resolution, 128);

		int queue_size_limit;
		nh.param("queue_size_limit", queue_size_limit, 0);
		as.set_queue_size_limit(queue_size_limit);

		status.status = planner_cspace::PlannerStatus::DONE;

		has_goal = false;
		has_start = false;

		int size[2] = {resolution, resolution};
		cm.reset(astar::vec(size));
		as.reset(astar::vec(size));
		cm.clear(0);

		link_body links[2];
		nh.param_cast("link0_joint_radius", links[0].radius[0], 0.05f);
		nh.param_cast("link0_end_radius", links[0].radius[1], 0.05f);
		nh.param_cast("link0_length", links[0].length, 0.15f);
		nh.param_cast("link0_x", links[0].origin.x, 0.2f);
		nh.param_cast("link0_y", links[0].origin.y, 0.0f);
		nh.param_cast("link0_th", links[0].origin.th, 0.0f);
		nh.param_cast("link1_joint_radius", links[1].radius[0], 0.05f);
		nh.param_cast("link1_end_radius", links[1].radius[1], 0.05f);
		nh.param_cast("link1_length", links[1].length, 0.25f);
		nh.param_cast("link1_x", links[1].origin.x, -0.2f);
		nh.param_cast("link1_y", links[1].origin.y, 0.0f);
		nh.param_cast("link1_th", links[1].origin.th, 0.0f);

		nh.param_cast("link0_coef", euclid_cost_coef[0], 1.0f);
		nh.param_cast("link1_coef", euclid_cost_coef[1], 1.5f);

		nh.param_cast("weight_cost", weight_cost, 10000.0);

		ROS_INFO("Resolution: %d", resolution);
		astar::vec p;
		for(p[0] = 0; p[0] < resolution; p[0] ++)
		{
			for(p[1] = 0; p[1] < resolution; p[1] ++)
			{
				astar::vecf pf;
				grid2metric(p, pf);
				
				if(links[0].isCollide(links[1], pf[0], pf[1]))
					cm[p] = 100;
				else if(pf[0] > M_PI || pf[1] > M_PI)
					cm[p] = 50;
				else
					cm[p] = 0;
				//if(p[0] == 62 && p[1] == 11)
				//	printf("\033[31mX\033[0m");
				//else
				//	printf("%d", cm[p] / 100);
			}
			printf("\n");
		}
		printf("\n");

		int range;
		nh.param("range", range, 8);
		for(p[0] = -range; p[0] <= range; p[0] ++)
		{
			for(p[1] = -range; p[1] <= range; p[1] ++)
			{
				search_list.push_back(p);
			}
		}
		has_start = has_goal = true;
	}
	void spin()
	{
		ros::Rate wait(freq);
		ROS_DEBUG("Initialized");

		while(ros::ok())
		{
			wait.sleep();
			ros::spinOnce();

			if(has_goal && has_start)
			{
				if(status.status == planner_cspace::PlannerStatus::FINISHING)
				{
					status.status = planner_cspace::PlannerStatus::DONE;
					has_goal = false;
					ROS_INFO("Path plan finished");
				}
				else
				{
					status.error = planner_cspace::PlannerStatus::GOING_WELL;

					float st[2] = {0, 0};
					float en[2] = {3.05, 0.52};
					astar::vecf start(st);
					astar::vecf end(en);

					ROS_INFO("Start searching");
					make_plan(start, end);
					break;
				}
			}
			else if(!has_goal)
			{
			}
			pub_status.publish(status);
		}
	}

private:
	void grid2metric(
			const int t0, const int t1,
			float &gt0, float &gt1)
	{
		gt0 = t0 * 2.0 * M_PI / (float)resolution;
		gt1 = t1 * 2.0 * M_PI / (float)resolution;
	}
	void metric2grid(
			int &t0, int &t1,
			const float gt0, const float gt1)
	{
		t0 = lroundf(gt0 * resolution / (2.0 * M_PI));
		t1 = lroundf(gt1 * resolution / (2.0 * M_PI));
	}
	void grid2metric(
			const astar::vec t,
			astar::vecf &gt)
	{
		grid2metric(t[0], t[1], gt[0], gt[1]);
	}
	void metric2grid(
			astar::vec &t,
			const astar::vecf gt)
	{
		metric2grid(t[0], t[1], gt[0], gt[1]);
	}
	bool make_plan(const astar::vecf sg, const astar::vecf eg)
	{
		astar::vec s, e;
		metric2grid(s, sg);
		metric2grid(e, eg);
		ROS_INFO("Planning from (%d, %d) to (%d, %d)",
				s[0], s[1], e[0], e[1]);
		std::list<astar::vec> path_grid;
		//const auto ts = std::chrono::high_resolution_clock::now();
		if(!as.search(s, e, path_grid, 
				std::bind(&planner_2dof_serial_joints::cb_cost, 
					this, std::placeholders::_1, std::placeholders::_2, 
					std::placeholders::_3, std::placeholders::_4), 
				std::bind(&planner_2dof_serial_joints::cb_cost_estim, 
					this, std::placeholders::_1, std::placeholders::_2), 
				std::bind(&planner_2dof_serial_joints::cb_search, 
					this, std::placeholders::_1,
					std::placeholders::_2, std::placeholders::_3), 
				std::bind(&planner_2dof_serial_joints::cb_progress, 
					this, std::placeholders::_1),
				0,
				1.0f / freq_min,
				true))
		{
			ROS_WARN("Path plan failed (goal unreachable)");
			status.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
		}
		//const auto tnow = std::chrono::high_resolution_clock::now();
		//ROS_INFO("Path found (%0.3f sec.)",
		//		std::chrono::duration<float>(tnow - ts).count());

		astar::vec p;
		for(p[0] = 0; p[0] < resolution; p[0] ++)
		{
			for(p[1] = 0; p[1] < resolution; p[1] ++)
			{
				bool found = false;
				for(auto &g: path_grid)
				{
					if(g == p) found = true;
				}
				if(p == s)
					printf("\033[31ms\033[0m");
				else if(p == e)
					printf("\033[31me\033[0m");
				else if(found)
					printf("\033[34m*\033[0m");
				else
					printf("%d", cm[p] / 11);
			}
			printf("\n");
		}
		printf("\n");


		return true;
	}
	std::vector<astar::vec> &cb_search(
			const astar::vec& p,
			const astar::vec& s, const astar::vec& e)
	{
		return search_list;
	}
	bool cb_progress(const std::list<astar::vec>& path_grid)
	{
		return true;
	}
	float cb_cost_estim(const astar::vec &s, const astar::vec &e)
	{
		const auto d = e - s;
		auto cost = euclid_cost(d);
		return cost;
	}
	float cb_cost(const astar::vec &s, astar::vec &e,
			const astar::vec &v_goal,
			const astar::vec &v_start)
	{
		auto d = e - s;
		d.cycle(d[0], resolution);
		d.cycle(d[1], resolution);

		float cost = euclid_cost(d);

		float distf = hypotf((float)d[0], (float)d[1]);
		float v[2], dp[2];
		int sum = 0;
		const int dist = distf;
		distf /= dist;
		v[0] = s[0];
		v[1] = s[1];
		dp[0] = (float)d[0] / dist;
		dp[1] = (float)d[1] / dist;
		astar::vec pos;
		for(int i = 0; i < dist; i ++)
		{
			pos[0] = lroundf(v[0]);
			pos[1] = lroundf(v[1]);
			pos.cycle_unsigned(pos[0], resolution);
			pos.cycle_unsigned(pos[1], resolution);
			const auto c = cm[pos];
			if(c > 99) return -1;
			sum += c;
			v[0] += dp[0];
			v[1] += dp[1];
		}
		cost += sum * weight_cost / 100.0;
		return cost;
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "planner_2dof_serial_joints");
	
	planner_2dof_serial_joints jy;
	jy.spin();

	return 0;
}


