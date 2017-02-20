#ifndef __TF_PROJECTION_HPP__
#define __TF_PROJECTION_HPP__

#include <tf/transform_datatypes.h>

class tf_projection
{
protected:
	std::map<std::string, std::string> frames;

public:
	tf::StampedTransform project(
			const tf::StampedTransform &trans, const tf::StampedTransform &trans_target)
	{
		auto origin = trans.getOrigin();
		origin.setZ(0.0);
		tf::StampedTransform projected = trans;
		projected.setOrigin(origin);

		tf::StampedTransform output = trans_target;
		output *= projected;
		output.child_frame_id_ = frames["frame"];
		output.frame_id_ = frames["target"];

		return output;
	}
};

#endif

