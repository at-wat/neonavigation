#include "tf_projection.hpp"
#include "gtest/gtest.h"

void test_transform(
		const tf::StampedTransform proj2base,
		const tf::StampedTransform targ2proj,
		const tf::StampedTransform truth)
{
	tf_projection tfp;

	tf::StampedTransform result = tfp.project(proj2base, targ2proj);

	const float error_x = fabs(result.getOrigin().x() - truth.getOrigin().x());
	const float error_y = fabs(result.getOrigin().y() - truth.getOrigin().y());
	const float error_z = fabs(result.getOrigin().z() - truth.getOrigin().z());
	const tf::Quaternion error_q = result.getRotation() * truth.getRotation().inverse();
	ASSERT_LT(error_x, 0.001);
	ASSERT_LT(error_y, 0.001);
	ASSERT_LT(error_z, 0.001);
	ASSERT_LT(fabs(error_q.getAngle()), 0.001);
}

TEST(tf_projection_test, projection_test)
{
	test_transform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), M_PI/2.0),
					tf::Vector3(1.0, 3.0, 10.0)),
				ros::Time(0),
				"map", "base_link"),
			// projected: t(1.0, 3.0, 0.0), r((0.0, 0.0, 1.0), M_PI/2.0)
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), M_PI/2.0),
					tf::Vector3(1.0, 0.0, 0.5)),
				ros::Time(0),
				"odom", "map"),
			// rotate 90deg: (-3.0, 1.0, 0.0), r((0.0, 0.0, 1.0), M_PI)
			// offset x+1.0, z+0.5: t(-2.0, 1.0, 0.5), r((0.0, 0.0, 1.0), M_PI)
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), M_PI),
					tf::Vector3(-2.0, 1.0, 0.5)),
				ros::Time(0),
				"odom", "base_link"));
	
	test_transform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0.0),
					tf::Vector3(1.0, 1.0, 100.0)),
				ros::Time(0),
				"map", "base_link"),
			// projected: t(1.0, 1.0, 0.0), r((0.0, 0.0, 1.0), 0.0)
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), M_PI/6.0),
					tf::Vector3(0.0, -sqrtf(3.0) / 2.0, 3.0 / 2.0)),
				ros::Time(0),
				"odom", "map"),
			// rotate 30deg and offset to make it on z axis
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), M_PI/6.0),
					tf::Vector3(1.0, 0.0, 2.0)),
				ros::Time(0),
				"odom", "base_link"));
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "tf_projection_test");

	return RUN_ALL_TESTS();
}

