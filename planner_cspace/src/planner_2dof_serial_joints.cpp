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
	int range;
	int local_range;
	double local_range_f;
	bool has_goal;
	bool has_start;
	std::vector<astar::vec> search_list;
    
	planner_cspace::PlannerStatus status;

public:
	planner_2dof_serial_joints():
		nh("~")
	{
		pub_status = nh.advertise<planner_cspace::PlannerStatus>("status", 1, true);

		nh.param_cast("freq", freq, 4.0f);
		nh.param_cast("freq_min", freq_min, 2.0f);

		int queue_size_limit;
		nh.param("queue_size_limit", queue_size_limit, 0);
		as.set_queue_size_limit(queue_size_limit);

		status.status = planner_cspace::PlannerStatus::DONE;

		has_goal = false;
		has_start = false;
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

					make_plan();
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
	}
	void metric2grid(
			int &t0, int &t1,
			const float gt0, const float gt1)
	{
	}
	bool make_plan()
	{
		astar::vec s, e;
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
				INT_MAX,
				1.0f / freq_min,
				true))
		{
			ROS_WARN("Path plan failed (goal unreachable)");
			status.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
		}
		//const auto tnow = std::chrono::high_resolution_clock::now();
		//ROS_INFO("Path found (%0.3f sec.)",
		//		std::chrono::duration<float>(tnow - ts).count());

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
		ROS_WARN("Search timed out");
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
		const auto d = e - s;
		float cost = euclid_cost(d);
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


