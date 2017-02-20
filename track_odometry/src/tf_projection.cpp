#include "tf_projection.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "tf_projection");
	
	tf_projection_node proj;
	proj.spin();

	return 0;
}

